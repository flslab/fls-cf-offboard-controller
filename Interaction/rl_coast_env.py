"""Offline v2: fixed-target posture braking followed by assumed position control.

The only learned choice is when to latch directly from -20 degree braking into
position takeover. There is NO intermediate level command. Activation latency
and unavailable delayed feedback retain the previous SENT command. An explicitly
uncalibrated PD surrogate then generates tilt commands through the SAME delayed
RK4 plant, retaining attitude, angular rate and pending commands. No state is
reset and the target is never moved. Every policy runs to the common horizon,
3 seconds after its first decision, even after reversal or apparent settling.

All position-controller parameters and their randomization ranges are heuristic
assumptions, not identified hardware dynamics. The independent target gap is a
virtual-coast input, not a measured or calibrated stopping endpoint. Simulated
joint settling is neither physical validation nor permission to fly.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import math

import gymnasium as gym
import numpy as np
from scipy.integrate import trapezoid

from Interaction.braking_validation_simulator import DelayedTiltPlant
from Interaction.offline_braking_selector import BrakingSnapshot, FrozenTiltModel
from Interaction.rl_braking_env import BrakingScenario, RLBrakeConfig, RLBrakingEnv


TASK_VERSION = 'coast_v2'
OBSERVATION_VERSION = 'coast_v2_v1prefix114_target_time3'
ACTION_VERSION = 'brake20_or_latched_position_v2'


@dataclass(frozen=True)
class CoastBrakeConfig:
    control_period_s: float = .01
    max_brake_s: float = .5
    history_length: int = 12
    horizon_s: float = 3.
    terminal_window_s: float = .3
    position_control_period_s: float = .02
    stable_dwell_s: float = .3
    position_tolerance_m: float = .03
    velocity_tolerance_m_s: float = .04
    tilt_tolerance_deg: float = 3.
    rate_tolerance_deg_s: float = 5.

    def validate(self):
        try:
            if not all(math.isfinite(float(x)) for x in asdict(self).values()):
                raise ValueError('coast configuration must contain finite scalars')
            if (self.control_period_s != .01
                    or isinstance(self.history_length, bool) or self.history_length != 12
                    or not isinstance(self.history_length, (int, np.integer))
                    or not 0 < self.max_brake_s <= .5
                    or not 0 < self.position_control_period_s <= .1
                    or not 0 < self.stable_dwell_s <= self.terminal_window_s
                    or self.horizon_s < self.max_brake_s + .08 + self.terminal_window_s
                    or min(self.position_tolerance_m, self.velocity_tolerance_m_s,
                           self.tilt_tolerance_deg, self.rate_tolerance_deg_s) <= 0):
                raise ValueError('coast configuration outside domain or incompatible with v1 prefix')
            for duration in (self.max_brake_s, self.horizon_s):
                ticks = duration / self.control_period_s
                if not math.isclose(ticks, round(ticks), rel_tol=0., abs_tol=1e-9):
                    raise ValueError('brake duration and horizon must be whole control ticks')
        except (TypeError, AttributeError, OverflowError) as exc:
            raise ValueError('invalid coast configuration') from exc


@dataclass(frozen=True)
class CoastScenario:
    braking: BrakingScenario
    target_distance_m: float
    direction_sign: int = 1
    position_kp_s2: float = 4.
    position_kd_s: float = 3.
    position_accel_limit_m_s2: float = 2.
    position_extra_measurement_delay_s: float = 0.
    takeover_activation_delay_s: float = 0.
    prelude: dict | None = None

    @property
    def name(self):
        return self.braking.name

    def validate(self):
        if not isinstance(self.braking, BrakingScenario):
            raise ValueError('braking must be a BrakingScenario')
        self.braking.validate()
        try:
            values = (self.target_distance_m, self.direction_sign, self.position_kp_s2,
                      self.position_kd_s, self.position_accel_limit_m_s2,
                      self.position_extra_measurement_delay_s, self.takeover_activation_delay_s)
            if not all(math.isfinite(float(x)) for x in values):
                raise ValueError('coast scenario fields must be finite scalars')
            if (self.target_distance_m <= 0 or isinstance(self.direction_sign, bool)
                    or self.direction_sign not in (-1, 1)
                    or not isinstance(self.direction_sign, (int, np.integer))
                    or min(self.position_kp_s2, self.position_kd_s, self.position_accel_limit_m_s2) <= 0
                    or not 0 <= self.position_extra_measurement_delay_s <= .04
                    or not 0 <= self.takeover_activation_delay_s <= .08
                    or self.prelude is not None and not isinstance(self.prelude, dict)):
                raise ValueError('coast scenario outside domain')
            if not math.isfinite(self.braking.snapshot.position_m + self.target_distance_m):
                raise ValueError('fixed target overflowed')
        except (TypeError, AttributeError, OverflowError) as exc:
            raise ValueError('invalid coast scenario') from exc


def sample_coast_scenario(rng, nominal_model, randomize=True,
                          name='synthetic_coast_prelude'):
    """Integrate a real synthetic prelude, then retain only its causal endpoint.

    Both world +/-Y directions use the positive travel-axis convention; only the
    projected nominal world bias changes sign. Commands are +20 degrees for
    U[.08,.25] s, then level for U[.04,.20] s. Initial v/theta/rate at coast entry
    are the resulting RK4 state, never independent arbitrary samples. Past SENT
    commands are rebased to negative times. The env does not fabricate measured
    history from these commands: unavailable pre-initial frames remain invalid.

    Tilt-plant perturbations match the declared v1 ranges: independent motion
    and command gains [.8,1.2], wn/zeta [.7,1.3], delay [.015,.055] s, projected
    bias +/- .002 rad, transport/measurement delay [0,.02] s, fixed sensor biases
    +/- .015 m/s, .5 deg and 5 deg/s. Position assumptions are independently
    sampled: kp [2,6] s^-2, kd [2,4.5] s^-1, amax [1,3] m/s^2, extra measurement
    delay [0,.04] s and activation delay [0,.08] s. Target gap U[.04,.4] m is an
    external synthetic goal, independent of the plant and any future trajectory.
    randomize=False uses nominal dynamics/sensors/position parameters, but still
    samples direction, prelude durations and target gap.
    """
    if not isinstance(nominal_model, FrozenTiltModel):
        raise ValueError('nominal_model must be a FrozenTiltModel')
    if not isinstance(randomize, (bool, np.bool_)):
        raise ValueError('randomize must be boolean')
    DelayedTiltPlant(nominal_model, BrakingSnapshot(0., 0., 0., 0., 0.))
    direction = int(rng.choice((-1, 1)))
    accel_s, level_s = float(rng.uniform(.08, .25)), float(rng.uniform(.04, .20))
    target_gap = float(rng.uniform(.04, .4))
    model = replace(nominal_model, projected_bias_rad=direction * nominal_model.projected_bias_rad)
    braking_changes, position_changes = {}, {}
    if randomize:
        model = replace(model,
            motion_gain=model.motion_gain * float(rng.uniform(.8, 1.2)),
            command_gain=model.command_gain * float(rng.uniform(.8, 1.2)),
            wn_rad_s=model.wn_rad_s * float(rng.uniform(.7, 1.3)),
            zeta=model.zeta * float(rng.uniform(.7, 1.3)),
            delay_s=float(rng.uniform(.015, .055)),
            projected_bias_rad=model.projected_bias_rad + float(rng.uniform(-.002, .002)))
        braking_changes = dict(
            transport_delay_s=float(rng.uniform(0., .02)),
            measurement_delay_s=float(rng.uniform(0., .02)),
            velocity_bias_m_s=float(rng.uniform(-.015, .015)),
            tilt_bias_deg=float(rng.uniform(-.5, .5)),
            rate_bias_deg_s=float(rng.uniform(-5., 5.)))
        position_changes = dict(
            position_kp_s2=float(rng.uniform(2., 6.)), position_kd_s=float(rng.uniform(2., 4.5)),
            position_accel_limit_m_s2=float(rng.uniform(1., 3.)),
            position_extra_measurement_delay_s=float(rng.uniform(0., .04)),
            takeover_activation_delay_s=float(rng.uniform(0., .08)))
    start = BrakingSnapshot(0., 0., 0., 0., 0.)
    plant = DelayedTiltPlant(model, start, ((-1., 0.),),
                            transport_delay_s=braking_changes.get('transport_delay_s', 0.))
    plant.send_command(0., math.radians(20.))
    plant.advance_to(accel_s)
    plant.send_command(accel_s, 0.)
    final = plant.advance_to(accel_s + level_s)
    snapshot = replace(final, time_s=0., position_m=0.)
    past = tuple((t - final.time_s, a) for t, a in plant.command_history)
    braking = BrakingScenario(name, model, snapshot, past, **braking_changes)
    result = CoastScenario(braking, target_gap, direction,
        prelude=dict(kind='synthetic_physically_integrated_prelude',
                     acceleration_duration_s=accel_s, level_duration_s=level_s,
                     command_tilt_deg=20., initial_snapshot=asdict(start),
                     endpoint_before_rebase=asdict(final),
                     world_travel_direction_y=direction,
                     measured_trajectory=False, target_source='independent_virtual_coast_gap',
                     position_model_calibrated=False), **position_changes)
    result.validate()
    return result


def score_coast_episode(metrics, config):
    """Fixed, terminal-only nonnegative cost; gamma=1 preserves this objective.

    Emax/.03 + Vmax/.04 + 2*overshoot/.03 + rollback/.03
        + .2*integral(|position error| dt)/(horizon*.1).
    Emax and Vmax cover the complete final .3 s window by default. All extrema
    and the integral use every RK4 boundary since the first decision. These
    weights/scales are fixed design choices, not an uncertainty calibration.
    """
    if not isinstance(config, CoastBrakeConfig):
        raise ValueError('config must be CoastBrakeConfig')
    config.validate()
    keys = ('terminal_max_abs_position_error_m', 'terminal_max_abs_velocity_m_s',
            'max_target_overshoot_m', 'max_rollback_m', 'integrated_abs_position_error_m_s')
    try:
        e, v, overshoot, rollback, integral = (float(metrics[key]) for key in keys)
        if not all(math.isfinite(x) and x >= 0 for x in (e, v, overshoot, rollback, integral)):
            raise ValueError('coast scoring metrics must be finite and nonnegative')
        loss = e/.03 + v/.04 + 2*overshoot/.03 + rollback/.03 + .2*integral/(config.horizon_s*.1)
        if not math.isfinite(loss):
            raise ValueError('coast loss overflowed')
        return float(loss)
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError('invalid coast scoring metrics') from exc


class CoastBrakingEnv(gym.Env):
    """Latched target-capture experiment, isolated from flight and frozen v1.

    The first 114 observations are obtained directly from the unmodified v1
    environment. Append [measured remaining target distance/.1 m, fixed target
    distance/.1 m, remaining common horizon/horizon]. Real prelude commands are
    available, but unavailable pre-initial measured frames keep validity 0.

    action 0: send/retain -20 deg for one 10 ms tick. action 1: latch POSITION;
    the common-horizon remainder is a terminal macrostep. No level-settle stage,
    arbitrary cancellation pulse, true-model compensation or target rewriting.
    Intermediate reward is exactly zero, terminal reward exactly negative loss.
    For comparisons, pass only observations/normal info to policies. scenario
    and plant properties exist for offline audits, not policy inputs.
    """

    metadata = {'render_modes': []}
    offline_only = True
    flight_command_generated = False
    position_target_used = True
    position_model_calibrated = False

    def __init__(self, nominal_model, config=None, scenarios=None, randomize=True):
        super().__init__()
        self.config = CoastBrakeConfig() if config is None else config
        if not isinstance(self.config, CoastBrakeConfig):
            raise ValueError('config must be CoastBrakeConfig')
        self.config.validate()
        if not isinstance(randomize, (bool, np.bool_)):
            raise ValueError('randomize must be boolean')
        self.nominal_model, self.randomize = nominal_model, bool(randomize)
        self._legacy = RLBrakingEnv(nominal_model, RLBrakeConfig(
            control_period_s=self.config.control_period_s, max_brake_s=self.config.max_brake_s,
            history_length=self.config.history_length), randomize=False)
        try:
            self.scenarios = None if scenarios is None else tuple(scenarios)
        except TypeError as exc:
            raise ValueError('scenarios must be an iterable of CoastScenario') from exc
        if self.scenarios is not None:
            if not self.scenarios:
                raise ValueError('scenarios must not be empty')
            for scenario in self.scenarios:
                self._validate_scenario(scenario)
        self.action_space = gym.spaces.Discrete(2)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(117,), dtype=np.float32)
        self._scenario = None
        self._target_position_m = None
        self._last_observation = None
        self._done = True
        self._tick = 0
        self._position_decisions = []

    @staticmethod
    def _validate_scenario(scenario):
        if not isinstance(scenario, CoastScenario):
            raise ValueError('scenario must be CoastScenario')
        scenario.validate()

    @property
    def scenario(self):
        return self._scenario

    @property
    def plant(self):
        return self._legacy.plant

    @property
    def first_decision_s(self):
        return self._legacy.first_decision_s

    @property
    def target_position_m(self):
        return self._target_position_m

    @property
    def legacy_observation(self):
        return None if self._last_observation is None else self._last_observation[:114].copy()

    @property
    def position_decisions(self):
        """Past controller evidence only; the last command is not a future plan."""
        return tuple(dict(row) for row in self._position_decisions)

    def _extend_observation(self, legacy_observation, info):
        now = info['decision_time_s']
        remaining = max(0., self.first_decision_s + self.config.horizon_s - now)
        extra = [(self._target_position_m - info['snapshot'].position_m)/.1,
                 self._scenario.target_distance_m/.1, remaining/self.config.horizon_s]
        if not np.isfinite(extra).all() or np.max(np.abs(extra)) > np.finfo(np.float32).max:
            raise ValueError('target observation is nonfinite or overflowing')
        observation = np.r_[legacy_observation, np.asarray(extra, dtype=np.float32)].astype(np.float32)
        self._last_observation = observation.copy()
        info.update(target_position_m=self._target_position_m,
                    target_distance_m=self._scenario.target_distance_m,
                    position_target_used=True, position_model_calibrated=False,
                    position_takeover_latched=self._done,
                    task_version=TASK_VERSION)
        return observation, info

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if options is not None and (not isinstance(options, dict) or set(options)-{'scenario'}):
            raise ValueError('reset options may contain only scenario')
        scenario = None if options is None else options.get('scenario')
        if scenario is None:
            scenario = (self.scenarios[int(self.np_random.integers(len(self.scenarios)))]
                        if self.scenarios is not None else
                        sample_coast_scenario(self.np_random, self.nominal_model, self.randomize))
        self._validate_scenario(scenario)
        self._scenario = scenario
        self._target_position_m = float(scenario.braking.snapshot.position_m + scenario.target_distance_m)
        self._tick, self._done = 0, False
        self._position_decisions = []
        # v1's common policy-sensor warmup is independent of hidden position
        # parameters. Missing extra-delayed position feedback waits after latch.
        legacy, info = self._legacy.reset(seed=seed, options={'scenario': scenario.braking})
        return self._extend_observation(legacy, info)

    def _position_measurement(self, now):
        case = self._scenario
        delay = case.braking.measurement_delay_s + case.position_extra_measurement_delay_s
        measured = self.plant.snapshot_at_or_before(now - delay)
        if measured is None:
            return None
        return replace(measured,
            velocity_m_s=measured.velocity_m_s + case.braking.velocity_bias_m_s,
            tilt_rad=measured.tilt_rad + math.radians(case.braking.tilt_bias_deg),
            tilt_rate_rad_s=measured.tilt_rate_rad_s + math.radians(case.braking.rate_bias_deg_s))

    def _position_command(self, now):
        measured = self._position_measurement(now)
        if measured is None:
            return False
        case = self._scenario
        acceleration = float(np.clip(
            case.position_kp_s2 * (self._target_position_m-measured.position_m)
            - case.position_kd_s * measured.velocity_m_s,
            -case.position_accel_limit_m_s2, case.position_accel_limit_m_s2))
        # No division by hidden motion/command gains and no hidden bias removal.
        command = float(np.clip(math.atan(acceleration/9.81), -math.radians(20), math.radians(20)))
        self.plant.send_command(now, command)
        self._position_decisions.append(dict(decision_time_s=float(now),
            snapshot=measured, command_tilt_rad=command, target_position_m=self._target_position_m))
        return True

    def _advance_position_to(self, endpoint):
        # Include the exact left edge of the terminal window, even when random
        # activation shifts the 20 ms position grid relative to that boundary.
        terminal_start = self.first_decision_s + self.config.horizon_s - self.config.terminal_window_s
        if self.plant.current_snapshot.time_s < terminal_start < endpoint:
            self.plant.advance_to(terminal_start)
        self.plant.advance_to(endpoint)

    def _metrics(self, handoff_time, activation_time):
        states = [s for s in self.plant.snapshot_history if s.time_s >= self.first_decision_s-1e-12]
        t = np.asarray([s.time_s for s in states])
        p = np.asarray([s.position_m for s in states])
        v = np.asarray([s.velocity_m_s for s in states])
        angle = np.asarray([s.tilt_rad for s in states])
        rate = np.asarray([s.tilt_rate_rad_s for s in states])
        error = p-self._target_position_m
        tail = t >= t[-1]-self.config.terminal_window_s-1e-12
        # "Position" starts at the first actual PD command SENT, not the latch
        # request. Brake/old commands can persist during activation/feedback
        # waiting. The plant's further command delay is intentionally retained;
        # this is a controller-command boundary, not instantaneous actuation.
        before = np.ones(len(t), dtype=bool) if activation_time is None else t <= activation_time+1e-12
        after = np.zeros(len(t), dtype=bool) if activation_time is None else t >= activation_time-1e-12
        before_request, after_request = t <= handoff_time+1e-12, t >= handoff_time-1e-12
        if not all(np.isfinite(a).all() for a in (t, p, v, angle, rate)):
            raise ValueError('coast trajectory is not finite')
        settled = ((np.abs(error) <= self.config.position_tolerance_m)
                   & (np.abs(v) <= self.config.velocity_tolerance_m_s)
                   & (np.abs(angle) <= math.radians(self.config.tilt_tolerance_deg))
                   & (np.abs(rate) <= math.radians(self.config.rate_tolerance_deg_s)))
        dwell_start, first_dwell = None, None
        for timestamp, valid in zip(t, settled):
            if not valid:
                dwell_start = None
            elif dwell_start is None:
                dwell_start = float(timestamp)
            if dwell_start is not None and timestamp-dwell_start >= self.config.stable_dwell_s-1e-12:
                first_dwell = float(timestamp-self.first_decision_s)
                break
        tail_duration = float(t[tail][-1]-t[tail][0])
        if not math.isclose(tail_duration, self.config.terminal_window_s, rel_tol=0., abs_tol=1e-10):
            raise ValueError('terminal observation must cover the exact complete final window')
        return dict(
            terminal_max_abs_position_error_m=float(np.max(np.abs(error[tail]))),
            terminal_max_abs_velocity_m_s=float(np.max(np.abs(v[tail]))),
            terminal_mean_abs_position_error_m=float(trapezoid(np.abs(error[tail]), t[tail])/tail_duration),
            terminal_mean_abs_velocity_m_s=float(trapezoid(np.abs(v[tail]), t[tail])/tail_duration),
            terminal_mean_velocity_m_s=float(trapezoid(v[tail], t[tail])/tail_duration),
            terminal_max_abs_tilt_deg=float(np.degrees(np.max(np.abs(angle[tail])))),
            terminal_max_abs_rate_deg_s=float(np.degrees(np.max(np.abs(rate[tail])))),
            max_target_overshoot_m=float(max(0., np.max(error))),
            max_rollback_m=float(np.max(np.maximum.accumulate(p)-p)),
            min_velocity_m_s=float(np.min(v)),
            min_velocity_before_position_m_s=float(np.min(v[before])),
            min_velocity_after_position_m_s=None if not np.any(after) else float(np.min(v[after])),
            reversal_before_position=bool(np.min(v[before]) < -1e-6),
            reversal_after_position=bool(np.any(after) and np.min(v[after]) < -1e-6),
            reversal_before_handoff_request=bool(np.min(v[before_request]) < -1e-6),
            reversal_after_handoff_request=bool(np.min(v[after_request]) < -1e-6),
            position_reversal_boundary='first_position_command_sent_not_effective_actuation',
            no_reverse_in_simulation=bool(np.min(v) >= -1e-6),
            joint_settled_in_simulation=bool(np.all(settled[tail])),
            first_stable_dwell_after_s=first_dwell,
            integrated_abs_position_error_m_s=float(trapezoid(np.abs(error), t)),
            handoff_after_s=float(handoff_time-self.first_decision_s),
            position_activation_after_s=(None if activation_time is None else
                                         float(activation_time-self.first_decision_s)),
            position_activation_wait_s=(None if activation_time is None else float(activation_time-handoff_time)),
            final_time_s=float(t[-1]), horizon_s=float(t[-1]-self.first_decision_s),
            target_position_m=self._target_position_m,
            position_model_calibrated=False, target_endpoint_calibrated=False,
            position_target_validated=False, flight_validated=False)

    def step(self, action):
        if self._done:
            raise RuntimeError('position takeover is latched/episode finished; reset before stepping')
        if not self.action_space.contains(action):
            raise ValueError('action must be 0 (brake) or 1 (latch position takeover)')
        forced = self._tick >= round(self.config.max_brake_s/self.config.control_period_s)
        if int(action) == 0 and not forced:
            legacy, reward, terminated, truncated, info = self._legacy.step(0)
            if terminated or truncated or reward != 0:
                raise RuntimeError('legacy prefix env unexpectedly ended before v2 takeover')
            self._tick += 1
            observation, info = self._extend_observation(legacy, info)
            return observation, 0., False, False, info

        handoff_time = float(self.plant.current_snapshot.time_s)
        end = self.first_decision_s + self.config.horizon_s
        eligible = handoff_time + self._scenario.takeover_activation_delay_s
        activation_time = None
        # No command is inserted at handoff. Keep the last SENT command while
        # activation or an unavailable extra-delayed measurement is pending.
        if eligible > handoff_time:
            self._advance_position_to(min(eligible, end))
        position_tick = 0
        while self.plant.current_snapshot.time_s < end:
            now = float(self.plant.current_snapshot.time_s)
            if self._position_command(now) and activation_time is None:
                activation_time = now
            position_tick += 1
            self._advance_position_to(min(end, eligible + position_tick*self.config.position_control_period_s))
        self._done, self._legacy._done = True, True
        reason = 'maximum_brake_duration' if forced else 'policy_position_takeover'
        metrics = self._metrics(handoff_time, activation_time)
        loss = score_coast_episode(metrics, self.config)
        legacy, info = self._legacy._observation_and_info()
        observation, info = self._extend_observation(legacy, info)
        info.update(metrics=metrics, loss=loss, episode_reward=-loss,
                    handoff_reason=reason, release_reason=reason,
                    handoff_time_s=handoff_time, position_activation_time_s=activation_time,
                    terminal_macrostep_duration_s=float(end-handoff_time))
        return observation, -loss, True, False, info
