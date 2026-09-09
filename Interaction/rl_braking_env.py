"""Offline-only binary optimal stopping on the independent delayed tilt plant.

Action 0 retains a -20 degree simulated braking command for one control tick;
action 1 sends level, latches, integrates the entire observation tail, and ends
the episode. There is no position controller, target or hardware interface.
Randomized states/parameters are synthetic sensitivity cases, not measured
trajectories or identified uncertainty bounds. Success is not flight approval.

The sole reward is the negative terminal loss from ``score_episode``. The level
action is an explicit terminal macrostep: its complete 1.5 s tail is integrated
without further policy decisions. Training discounts therefore apply per brake
decision, not per tail integration tick; use gamma=1 for the exact stated
undiscounted objective. Neither reversal nor apparent success shortens the tail.
"""
from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass, replace
import math

import gymnasium as gym
import numpy as np

from Interaction.braking_validation_simulator import DelayedTiltPlant
from Interaction.offline_braking_selector import BrakingSnapshot, FrozenTiltModel
from Interaction.validate_predictive_brake_release import summary


@dataclass(frozen=True)
class BrakingScenario:
    name: str
    model: FrozenTiltModel
    snapshot: BrakingSnapshot
    command_history: tuple
    first_decision_s: float = 0.0
    measurement_delay_s: float = 0.0
    transport_delay_s: float = 0.0
    velocity_bias_m_s: float = 0.0
    tilt_bias_deg: float = 0.0
    rate_bias_deg_s: float = 0.0

    def validate(self):
        """Require a finite time-zero state and only already-sent history."""
        try:
            values = (self.first_decision_s, self.measurement_delay_s,
                      self.transport_delay_s, self.velocity_bias_m_s,
                      self.tilt_bias_deg, self.rate_bias_deg_s)
            if not all(math.isfinite(float(x)) for x in values):
                raise ValueError('scenario fields must be finite scalars')
            if (not isinstance(self.name, str) or not self.name
                    or not isinstance(self.model, FrozenTiltModel)
                    or not isinstance(self.snapshot, BrakingSnapshot)
                    or self.snapshot.time_s != 0.0
                    or min(self.first_decision_s, self.measurement_delay_s,
                           self.transport_delay_s) < 0):
                raise ValueError('scenario must have a name, time-zero state and nonnegative delays')
            DelayedTiltPlant(self.model, self.snapshot, self.command_history,
                            transport_delay_s=self.transport_delay_s)
            if abs(self.snapshot.tilt_rad + math.radians(self.tilt_bias_deg)) >= math.radians(80):
                raise ValueError('biased initial measured tilt outside small-angle domain')
        except (TypeError, AttributeError, OverflowError) as exc:
            raise ValueError('invalid braking scenario') from exc


@dataclass(frozen=True)
class RLBrakeConfig:
    control_period_s: float = 0.01
    max_brake_s: float = 0.5
    tail_observation_s: float = 1.5
    history_length: int = 12
    target_tail_speed_m_s: float = 0.025
    reverse_weight: float = 4.0
    rollback_weight: float = 2.0
    duration_weight: float = 0.03

    def validate(self):
        try:
            values = list(asdict(self).values())
            if not all(math.isfinite(float(x)) for x in values):
                raise ValueError('RL configuration fields must be finite scalars')
            if (not 0 < self.control_period_s <= .1
                    or self.max_brake_s <= 0 or self.tail_observation_s < 1.2
                    or isinstance(self.history_length, bool)
                    or not isinstance(self.history_length, (int, np.integer))
                    or not 1 <= self.history_length <= 1000
                    or min(self.target_tail_speed_m_s, self.reverse_weight,
                           self.rollback_weight, self.duration_weight) < 0):
                raise ValueError('RL configuration outside domain')
            ticks = self.max_brake_s / self.control_period_s
            if not math.isclose(ticks, round(ticks), rel_tol=0., abs_tol=1e-9):
                raise ValueError('maximum brake duration must be a whole number of control ticks')
        except (TypeError, AttributeError, OverflowError) as exc:
            raise ValueError('invalid RL configuration') from exc


def sample_scenario(rng, nominal_model, randomize=True, name='synthetic_random_initial_state'):
    """Sample an explicitly synthetic initial state, not a measured trajectory.

    Initial speed/tilt/rate are independently sampled in [0.2, 1.0] m/s,
    [0, 8] degrees, and [-80, 5] degrees/s. With randomization, each gain is
    multiplied by U[.8,1.2], wn and zeta by U[.7,1.3], model delay is
    U[.015,.055] s, and projected bias adds U[-.002,.002] rad. Measurement and
    transport delay are U[0,.020] s; fixed episode sensor biases are +/- .015
    m/s, .5 degree, and 5 degrees/s. These bounds are predeclared, not fitted.
    ``randomize=False`` retains synthetic initial-state sampling but uses the
    nominal plant and unbiased, undelayed sensors.
    """
    if not isinstance(randomize, (bool, np.bool_)):
        raise ValueError('randomize must be a boolean')
    if not isinstance(nominal_model, FrozenTiltModel):
        raise ValueError('nominal_model must be a FrozenTiltModel')
    DelayedTiltPlant(nominal_model, BrakingSnapshot(0., 0., 0., 0., 0.))
    state = BrakingSnapshot(0., 0., float(rng.uniform(.2, 1.0)),
                            float(np.radians(rng.uniform(0., 8.))),
                            float(np.radians(rng.uniform(-80., 5.))))
    changes = {}
    model = nominal_model
    if randomize:
        model = replace(nominal_model,
            motion_gain=nominal_model.motion_gain * float(rng.uniform(.8, 1.2)),
            command_gain=nominal_model.command_gain * float(rng.uniform(.8, 1.2)),
            wn_rad_s=nominal_model.wn_rad_s * float(rng.uniform(.7, 1.3)),
            zeta=nominal_model.zeta * float(rng.uniform(.7, 1.3)),
            delay_s=float(rng.uniform(.015, .055)),
            projected_bias_rad=nominal_model.projected_bias_rad + float(rng.uniform(-.002, .002)))
        changes = dict(measurement_delay_s=float(rng.uniform(0., .02)),
                       transport_delay_s=float(rng.uniform(0., .02)),
                       velocity_bias_m_s=float(rng.uniform(-.015, .015)),
                       tilt_bias_deg=float(rng.uniform(-.5, .5)),
                       rate_bias_deg_s=float(rng.uniform(-5., 5.)))
    scenario = BrakingScenario(name, model, state, ((-1., 0.),), **changes)
    scenario.validate()
    return scenario


def score_episode(metrics, config):
    """Finite nonnegative loss; all extrema use the full integration trace.

    |tail mean speed - .025| + 4 max(0, -minimum speed)
        + 2 maximum rollback + .03 brake duration, at default weights.
    The tail mean is the final 100 ms mean from the shared summary function.
    """
    config.validate()
    try:
        tail = float(metrics['terminal_mean_velocity_m_s'])
        minimum = float(metrics['min_velocity_m_s'])
        rollback = float(metrics['max_rollback_m'])
        duration = float(metrics['release_after_brake_s'])
        if (not all(math.isfinite(x) for x in (tail, minimum, rollback, duration))
                or rollback < 0 or duration < 0):
            raise ValueError('episode metrics must be finite with nonnegative rollback/duration')
        loss = (abs(tail - config.target_tail_speed_m_s)
                + config.reverse_weight * max(0., -minimum)
                + config.rollback_weight * rollback + config.duration_weight * duration)
        if not math.isfinite(loss):
            raise ValueError('episode loss overflowed')
        return float(loss)
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError('invalid episode scoring metrics') from exc


class RLBrakingEnv(gym.Env):
    """Partially observed, latched binary stopping environment; never flight.

    Observation layout (114 float32 values with the default 12-frame history):
    current [velocity, tilt, rate, position, measurement age], oldest-to-newest
    history_length [measured frame, validity] rows, history_length SENT
    [command age, angle, validity] rows, then elapsed braking time. Frames
    include the current measurement. Validity is 1 for available data, else 0.
    Fixed divisors are [1 m/s, 20 deg, 200 deg/s, 1 m, .03 s], [.5 s, 20 deg],
    and .5 s. No true model, sensor bias, hidden delay, future state, predicted
    terminal state, target position, or unsent command is included.

    Unavailable early history is zero-padded with explicit validity 0, never
    presented as real prehistory. Per-frame age is its age at
    that original decision. ``info['snapshot']`` preserves measurement time and
    biases and is the same observation supplied to non-RL baselines. Full true
    plant/scenario properties are deliberately available for offline evaluation
    only, never copied into policy observations or normal info dictionaries.
    """

    metadata = {'render_modes': []}
    offline_only = True
    flight_command_generated = False
    position_target_used = False
    brake_tilt_rad = -math.radians(20.)

    def __init__(self, nominal_model, config=None, scenarios=None, randomize=True):
        super().__init__()
        self.config = config if config is not None else RLBrakeConfig()
        if not isinstance(self.config, RLBrakeConfig):
            raise ValueError('config must be RLBrakeConfig')
        self.config.validate()
        if not isinstance(nominal_model, FrozenTiltModel):
            raise ValueError('nominal_model must be FrozenTiltModel')
        DelayedTiltPlant(nominal_model, BrakingSnapshot(0., 0., 0., 0., 0.))
        if not isinstance(randomize, (bool, np.bool_)):
            raise ValueError('randomize must be a boolean')
        self.nominal_model = nominal_model
        self.randomize = bool(randomize)
        self.scenarios = None if scenarios is None else tuple(scenarios)
        if self.scenarios is not None:
            if not self.scenarios:
                raise ValueError('scenarios must be nonempty when supplied')
            for scenario in self.scenarios:
                self._validate_scenario(scenario)
        self.action_space = gym.spaces.Discrete(2)
        self.observation_space = gym.spaces.Box(
            -np.inf, np.inf, shape=(6 + 9 * self.config.history_length,), dtype=np.float32)
        self._frames = deque(maxlen=self.config.history_length)
        self._plant = self._scenario = None
        self._first_decision_s = None
        self._done = True
        self._tick = 0
        self._max_ticks = int(round(self.config.max_brake_s / self.config.control_period_s))

    @staticmethod
    def _validate_scenario(scenario):
        if not isinstance(scenario, BrakingScenario):
            raise ValueError('scenario must be BrakingScenario')
        scenario.validate()

    @property
    def scenario(self):
        return self._scenario

    @property
    def plant(self):
        return self._plant

    @property
    def first_decision_s(self):
        return self._first_decision_s

    def _measured(self):
        scenario = self._scenario
        now = self._plant.current_snapshot.time_s
        measured = self._plant.snapshot_at_or_before(now - scenario.measurement_delay_s)
        if measured is None:
            raise ValueError('delayed measurement unavailable after explicit warmup')
        return replace(measured,
            velocity_m_s=measured.velocity_m_s + scenario.velocity_bias_m_s,
            tilt_rad=measured.tilt_rad + math.radians(scenario.tilt_bias_deg),
            tilt_rate_rad_s=measured.tilt_rate_rad_s + math.radians(scenario.rate_bias_deg_s))

    def _observation_and_info(self):
        measured = self._measured()
        now = float(self._plant.current_snapshot.time_s)
        frame = np.asarray([measured.velocity_m_s,
                            measured.tilt_rad / math.radians(20.),
                            measured.tilt_rate_rad_s / math.radians(200.),
                            measured.position_m, (now - measured.time_s) / .03], dtype=float)
        self._frames.append(frame)
        frames = np.zeros((self.config.history_length, 6), dtype=float)
        frames[-len(self._frames):, :5] = np.asarray(self._frames)
        frames[-len(self._frames):, 5] = 1.
        commands = list(self._plant.command_history[-self.config.history_length:])
        command_features = np.zeros((self.config.history_length, 3), dtype=float)
        if commands:
            command_features[-len(commands):] = [
                ((now-t)/.5, a/math.radians(20.), 1.) for t, a in commands]
        values = np.concatenate((frame, np.asarray(frames).ravel(), command_features.ravel(),
                                 [(now - self._first_decision_s) / .5]))
        if not np.isfinite(values).all() or np.any(np.abs(values) > np.finfo(np.float32).max):
            raise ValueError('nonfinite or overflowing observation')
        observation = values.astype(np.float32)
        info = dict(snapshot=measured, command_history=self._plant.command_history,
                    decision_time_s=now, elapsed_brake_s=now - self._first_decision_s,
                    offline_only=True, flight_command_generated=False,
                    position_target_used=False)
        return observation, info

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if options is not None and (not isinstance(options, dict) or set(options) - {'scenario'}):
            raise ValueError('reset options may contain only scenario')
        scenario = None if options is None else options.get('scenario')
        if scenario is None:
            if self.scenarios is not None:
                scenario = self.scenarios[int(self.np_random.integers(len(self.scenarios)))]
            else:
                scenario = sample_scenario(self.np_random, self.nominal_model, self.randomize)
        self._validate_scenario(scenario)
        self._scenario = scenario
        self._plant = DelayedTiltPlant(scenario.model, scenario.snapshot,
                                      scenario.command_history,
                                      transport_delay_s=scenario.transport_delay_s)
        self._first_decision_s = float(max(scenario.first_decision_s, scenario.measurement_delay_s))
        # No pre-seed measurements or brake commands are fabricated in warmup.
        self._plant.advance_to(self._first_decision_s)
        self._tick, self._done = 0, False
        self._frames.clear()
        return self._observation_and_info()

    def step(self, action):
        if self._done:
            raise RuntimeError('episode is latched/finished; call reset before step')
        if not self.action_space.contains(action):
            raise ValueError('action must be 0 (keep braking) or 1 (level and latch)')
        release = int(action) == 1 or self._tick >= self._max_ticks
        now = float(self._plant.current_snapshot.time_s)
        command = 0. if release else self.brake_tilt_rad
        # Like the existing comparator, only transmit when the selected command
        # changes; this preserves identical delayed switch boundaries.
        if (self._tick == 0 or not self._plant.command_history
                or self._plant.command_history[-1][1] != command):
            self._plant.send_command(now, command)
        if not release:
            self._tick += 1
            self._plant.advance_to(self._first_decision_s + self._tick * self.config.control_period_s)
            observation, info = self._observation_and_info()
            return observation, 0.0, False, False, info

        reason = 'maximum_brake_duration' if self._tick >= self._max_ticks else 'policy_level'
        end_time = now + self.config.tail_observation_s
        tail_ticks = self.config.tail_observation_s / self.config.control_period_s
        if math.isclose(tail_ticks, round(tail_ticks), rel_tol=0., abs_tol=1e-9):
            # Avoid adding a roundoff-only duplicate terminal sample, which
            # otherwise changes the shared unweighted 100 ms tail mean.
            end_time = self._first_decision_s + (self._tick + round(tail_ticks)) * self.config.control_period_s
        # Keep the same control-grid integration boundaries as the existing
        # baseline simulator, but make no more decisions during this macrostep.
        tail_tick = self._tick + 1
        while self._plant.current_snapshot.time_s < end_time:
            endpoint = min(end_time, self._first_decision_s + tail_tick * self.config.control_period_s)
            self._plant.advance_to(endpoint)
            tail_tick += 1
        self._done = True
        metrics = summary([asdict(state) for state in self._plant.snapshot_history],
                          now, self._first_decision_s, 'rl_binary_stopping', reason)
        loss = score_episode(metrics, self.config)
        observation, info = self._observation_and_info()
        info.update(metrics=metrics, loss=loss, episode_reward=-loss,
                    release_reason=reason, release_time_s=now,
                    terminal_macrostep_duration_s=self.config.tail_observation_s)
        return observation, -loss, True, False, info
