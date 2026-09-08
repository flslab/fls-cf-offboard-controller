"""Offline minimum-time velocity MPC using a learned planar flight model.

This module has no Crazyflie/cflib imports and never sends a command.  It is a
reference-inspired first step toward Learning MPC: a frozen, identified
second-order attitude model predicts candidate maximum-acceleration/level
trajectories, while a small causal residual learner corrects repeatable
translation-model error between episodes.

The controller's task is deliberately narrow: move the velocity projected on a
fixed world-XY direction from its episode-start value to ``target_velocity_m_s``
as quickly as possible, then arrive level.  Target-speed overshoot, terminal
tilt/rate, command slew, state freshness and the learned-model domain are hard
candidate gates, not reward terms.  Guarantees are therefore model-conditional;
this module is not flight authorization.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math

import numpy as np

from Interaction.model_based_braking import _second_order_transition
from Interaction.offline_braking_selector import FrozenTiltModel


@dataclass(frozen=True)
class VelocityMPCConfig:
    control_period_s: float = 0.01
    prediction_step_s: float = 0.01
    prediction_horizon_s: float = 1.20
    max_acceleration_tilt_deg: float = 20.0
    max_command_slew_deg_s: float = 180.0
    max_tilt_deg: float = 29.0
    max_tilt_rate_deg_s: float = 300.0
    terminal_velocity_tolerance_m_s: float = 0.05
    terminal_tilt_tolerance_deg: float = 3.0
    terminal_tilt_rate_tolerance_deg_s: float = 20.0
    terminal_dwell_s: float = 0.08
    overshoot_tolerance_m_s: float = 0.02
    velocity_uncertainty_margin_m_s: float = 0.0
    pulse_grid_step_s: float = 0.01
    max_state_age_s: float = 0.10
    max_state_group_skew_s: float = 0.03
    max_cross_axis_speed_m_s: float = 0.15
    max_abs_velocity_m_s: float = 2.0
    residual_learning_rate: float = 0.20
    residual_acceleration_limit_m_s2: float = 1.5
    residual_min_samples: int = 4

    def validate(self) -> None:
        values = asdict(self)
        integers = {"residual_min_samples"}
        for key, value in values.items():
            if key in integers:
                if isinstance(value, bool) or int(value) != value:
                    raise ValueError(key + " must be an integer")
                continue
            if isinstance(value, bool) or not math.isfinite(float(value)):
                raise ValueError(key + " must be finite")
        if not 0.005 <= self.control_period_s <= 0.05:
            raise ValueError("control_period_s must be in [0.005, 0.05]")
        if not 0.005 <= self.prediction_step_s <= 0.02:
            raise ValueError("prediction_step_s must be in [0.005, 0.02]")
        if self.prediction_step_s > self.control_period_s:
            raise ValueError("prediction_step_s cannot exceed control_period_s")
        if not 0.30 <= self.prediction_horizon_s <= 2.0:
            raise ValueError("prediction_horizon_s must be in [0.30, 2.0]")
        if not 0 < self.max_acceleration_tilt_deg < 30:
            raise ValueError("max_acceleration_tilt_deg must be in (0, 30)")
        if not self.max_acceleration_tilt_deg <= self.max_tilt_deg < 30:
            raise ValueError("max_tilt_deg must cover the command and stay below 30")
        if not 10 <= self.max_command_slew_deg_s <= 720:
            raise ValueError("max_command_slew_deg_s must be in [10, 720]")
        if not 0 < self.max_tilt_rate_deg_s <= 1000:
            raise ValueError("max_tilt_rate_deg_s must be in (0, 1000]")
        if not 0 < self.terminal_velocity_tolerance_m_s <= 0.10:
            raise ValueError("terminal velocity tolerance must be in (0, 0.10]")
        if not 0 < self.terminal_tilt_tolerance_deg <= 5:
            raise ValueError("terminal tilt tolerance must be in (0, 5]")
        if not 0 < self.terminal_tilt_rate_tolerance_deg_s <= 60:
            raise ValueError("terminal tilt-rate tolerance must be in (0, 60]")
        if not self.control_period_s <= self.terminal_dwell_s <= 0.30:
            raise ValueError("terminal_dwell_s is outside the supported range")
        if not 0 <= self.overshoot_tolerance_m_s <= 0.05:
            raise ValueError("overshoot tolerance must be in [0, 0.05]")
        if not 0 <= self.velocity_uncertainty_margin_m_s <= 0.10:
            raise ValueError("velocity uncertainty margin must be in [0, 0.10]")
        if self.velocity_uncertainty_margin_m_s > self.overshoot_tolerance_m_s:
            raise ValueError("uncertainty margin exceeds the overshoot allowance")
        if not self.prediction_step_s <= self.pulse_grid_step_s <= 0.05:
            raise ValueError("pulse_grid_step_s is outside the supported range")
        if not 0 <= self.max_state_age_s <= 0.10:
            raise ValueError("max_state_age_s must be in [0, 0.10]")
        if not 0 <= self.max_state_group_skew_s <= 0.03:
            raise ValueError("max_state_group_skew_s must be in [0, 0.03]")
        if not 0 < self.max_cross_axis_speed_m_s <= 0.5:
            raise ValueError("max_cross_axis_speed_m_s must be in (0, 0.5]")
        if not 0 < self.max_abs_velocity_m_s <= 5:
            raise ValueError("max_abs_velocity_m_s must be in (0, 5]")
        if not 0 < self.residual_learning_rate <= 1:
            raise ValueError("residual_learning_rate must be in (0, 1]")
        if not 0 < self.residual_acceleration_limit_m_s2 <= 5:
            raise ValueError("residual acceleration limit must be in (0, 5]")
        if not 2 <= self.residual_min_samples <= 100:
            raise ValueError("residual_min_samples must be in [2, 100]")


@dataclass(frozen=True)
class VelocityMPCState:
    time_s: float
    velocity_xy: tuple[float, float]
    orientation_rpy_rad: tuple[float, float, float]
    angular_velocity_rad_s: tuple[float, float, float]
    state_group_skew_s: float = 0.0


class CausalAccelerationResidualLearner:
    """Bounded EWMA of measured minus modelled projected acceleration."""

    def __init__(self, *, learning_rate=0.20, max_abs_acceleration_m_s2=1.5,
                 min_samples=4):
        self.learning_rate = float(learning_rate)
        self.max_abs_acceleration_m_s2 = float(max_abs_acceleration_m_s2)
        self.min_samples = int(min_samples)
        self.sample_count = 0
        self.estimate_m_s2 = 0.0
        self.rejected_count = 0

    @property
    def ready(self):
        return self.sample_count >= self.min_samples

    def update(self, *, dt_s, velocity_before_m_s, velocity_after_m_s,
               mean_projected_tilt_rad, motion_gain):
        values = np.asarray([
            dt_s, velocity_before_m_s, velocity_after_m_s,
            mean_projected_tilt_rad, motion_gain,
        ], dtype=float)
        if (not np.isfinite(values).all() or dt_s <= 0
                or dt_s > 0.10 or motion_gain <= 0
                or abs(mean_projected_tilt_rad) >= math.radians(60)):
            self.rejected_count += 1
            return self.snapshot("invalid_sample")
        measured = (velocity_after_m_s-velocity_before_m_s)/dt_s
        modelled = motion_gain*9.81*math.tan(mean_projected_tilt_rad)
        residual = measured-modelled
        if (not math.isfinite(residual)
                or abs(residual) > self.max_abs_acceleration_m_s2):
            self.rejected_count += 1
            return self.snapshot("outlier_rejected")
        if self.sample_count == 0:
            self.estimate_m_s2 = residual
        else:
            alpha = self.learning_rate
            self.estimate_m_s2 = (
                (1-alpha)*self.estimate_m_s2 + alpha*residual
            )
        self.sample_count += 1
        return self.snapshot("ready" if self.ready else "warming_up")

    def snapshot(self, status=None):
        return {
            "ready": self.ready,
            "status": status or ("ready" if self.ready else "warming_up"),
            "sample_count": self.sample_count,
            "rejected_count": self.rejected_count,
            "acceleration_m_s2": (
                float(self.estimate_m_s2) if self.ready else 0.0
            ),
        }


class LearningVelocityMPC:
    """Receding-horizon, minimum-time controller with hard terminal gates.

    ``projected_command_tilt_rad > 0`` requests acceleration along
    ``direction_xy``.  A caller may record only an actually sent command with
    :meth:`record_sent_command`; without that history, decisions fail closed.
    """

    def __init__(self, model: FrozenTiltModel, *, direction_xy,
                 target_velocity_m_s, config: VelocityMPCConfig | None = None):
        self.model = model
        self.config = config or VelocityMPCConfig()
        self.config.validate()
        direction = np.asarray(direction_xy, dtype=float)
        if (direction.shape != (2,) or not np.isfinite(direction).all()
                or np.linalg.norm(direction) <= 1e-9):
            raise ValueError("direction_xy must be a finite nonzero XY vector")
        self.direction_xy = direction/np.linalg.norm(direction)
        self.target_velocity_m_s = float(target_velocity_m_s)
        model_values = np.asarray(list(asdict(model).values()), dtype=object)
        numeric_model = np.asarray([
            model.delay_s, model.wn_rad_s, model.zeta, model.command_gain,
            model.motion_gain, model.projected_bias_rad,
            self.target_velocity_m_s,
        ], dtype=float)
        if (model_values.shape != (6,) or not np.isfinite(numeric_model).all()
                or model.delay_s < 0 or model.wn_rad_s <= 0 or model.zeta <= 0
                or model.command_gain <= 0 or model.motion_gain <= 0
                or abs(model.projected_bias_rad) >= math.radians(30)
                or abs(self.target_velocity_m_s) > self.config.max_abs_velocity_m_s):
            raise ValueError("model or target velocity is outside the supported domain")
        self._last_sent_time_s = None
        self._last_sent_tilt_rad = None
        self._sent_history = []
        self._last_decision_time_s = None
        self._episode_initial_velocity_m_s = None
        self._episode_sign = None
        self.residual_learner = CausalAccelerationResidualLearner(
            learning_rate=self.config.residual_learning_rate,
            max_abs_acceleration_m_s2=(
                self.config.residual_acceleration_limit_m_s2
            ),
            min_samples=self.config.residual_min_samples,
        )

    def record_sent_command(self, time_s, projected_tilt_rad):
        stamp = float(time_s)
        tilt = float(projected_tilt_rad)
        if (not math.isfinite(stamp) or not math.isfinite(tilt)
                or abs(tilt) > math.radians(self.config.max_acceleration_tilt_deg)+1e-12):
            raise ValueError("sent command is invalid or outside the MPC limit")
        if self._last_sent_time_s is not None and stamp < self._last_sent_time_s:
            raise ValueError("sent command time moved backwards")
        self._last_sent_time_s = stamp
        self._last_sent_tilt_rad = tilt
        self._sent_history.append((stamp, tilt))
        self._sent_history = self._sent_history[-256:]

    def observe_transition(self, before: VelocityMPCState,
                           after: VelocityMPCState):
        before_values = self._state_components(before)
        after_values = self._state_components(after)
        return self.residual_learner.update(
            dt_s=after.time_s-before.time_s,
            velocity_before_m_s=before_values[0],
            velocity_after_m_s=after_values[0],
            mean_projected_tilt_rad=0.5*(before_values[1]+after_values[1]),
            motion_gain=self.model.motion_gain,
        )

    def _state_components(self, state):
        if not isinstance(state, VelocityMPCState):
            raise ValueError("state must be VelocityMPCState")
        velocity = np.asarray(state.velocity_xy, dtype=float)
        rpy = np.asarray(state.orientation_rpy_rad, dtype=float)
        rates = np.asarray(state.angular_velocity_rad_s, dtype=float)
        values = np.r_[state.time_s, velocity, rpy, rates,
                       state.state_group_skew_s]
        if (velocity.shape != (2,) or rpy.shape != (3,) or rates.shape != (3,)
                or not np.isfinite(values).all()):
            raise ValueError("state contains malformed or nonfinite values")
        if state.state_group_skew_s < 0:
            raise ValueError("state group skew cannot be negative")
        yaw = float(rpy[2])
        cosine, sine = math.cos(yaw), math.sin(yaw)
        # R(-yaw) maps the fixed world direction into body XY.
        body_direction = np.array([
            cosine*self.direction_xy[0]+sine*self.direction_xy[1],
            -sine*self.direction_xy[0]+cosine*self.direction_xy[1],
        ])
        projected_tilt = float(
            -body_direction[0]*rpy[1]-body_direction[1]*rpy[0]
        )
        projected_rate = float(
            -body_direction[0]*rates[1]-body_direction[1]*rates[0]
        )
        projected_velocity = float(velocity @ self.direction_xy)
        cross_velocity = float(velocity @ np.array([
            -self.direction_xy[1], self.direction_xy[0]
        ]))
        return projected_velocity, projected_tilt, projected_rate, yaw, cross_velocity

    def _slew(self, current, target, dt):
        limit = math.radians(self.config.max_command_slew_deg_s)*dt
        return float(current+np.clip(target-current, -limit, limit))

    def _delay_queue(self, now_s, delay_steps, dt):
        if not delay_steps:
            return []
        required_time = now_s-self.model.delay_s
        if not self._sent_history or self._sent_history[0][0] > required_time+1e-12:
            raise ValueError("sent command history does not cover model delay")
        queue = []
        index = 0
        active = self._sent_history[0][1]
        for offset in range(delay_steps):
            effective_time = now_s+offset*dt-self.model.delay_s
            while (index+1 < len(self._sent_history)
                   and self._sent_history[index+1][0] <= effective_time+1e-12):
                index += 1
                active = self._sent_history[index][1]
            queue.append(float(active))
        return queue

    def _forecast(self, *, now_s, velocity, tilt, tilt_rate, current_command,
                  episode_sign, pulse_s):
        c, m = self.config, self.model
        dt = c.prediction_step_s
        steps = int(math.ceil(c.prediction_horizon_s/dt))
        delay_steps = int(math.ceil(m.delay_s/dt-1e-12))
        queue = self._delay_queue(now_s, delay_steps, dt)
        command = current_command
        transition = _second_order_transition(m.wn_rad_s, m.zeta, dt)
        terminal_needed = max(1, int(math.ceil(c.terminal_dwell_s/dt)))
        residual = self.residual_learner.snapshot()["acceleration_m_s2"]
        hard_limit = (
            c.overshoot_tolerance_m_s-c.velocity_uncertainty_margin_m_s
        )
        max_signed_overshoot = episode_sign*(velocity-self.target_velocity_m_s)
        terminal_run = 0
        arrival_time = None
        trace = []
        max_abs_tilt = abs(tilt)
        max_abs_rate = abs(tilt_rate)
        for index in range(steps):
            time_s = index*dt
            desired = (
                episode_sign*math.radians(c.max_acceleration_tilt_deg)
                if time_s < pulse_s-1e-12 else 0.0
            )
            command = self._slew(command, desired, dt)
            queue.append(command)
            effective = queue.pop(0) if delay_steps else command
            equilibrium = m.command_gain*effective+m.projected_bias_rad
            centered = transition @ np.array([tilt-equilibrium, tilt_rate])
            next_tilt = float(centered[0]+equilibrium)
            next_rate = float(centered[1])
            acceleration_before = (
                m.motion_gain*9.81*math.tan(tilt)+residual
            )
            acceleration_after = (
                m.motion_gain*9.81*math.tan(next_tilt)+residual
            )
            velocity += 0.5*(acceleration_before+acceleration_after)*dt
            tilt, tilt_rate = next_tilt, next_rate
            max_abs_tilt = max(max_abs_tilt, abs(tilt))
            max_abs_rate = max(max_abs_rate, abs(tilt_rate))
            signed_overshoot = episode_sign*(
                velocity-self.target_velocity_m_s
            )
            max_signed_overshoot = max(max_signed_overshoot, signed_overshoot)
            terminal_now = (
                abs(velocity-self.target_velocity_m_s)
                <= c.terminal_velocity_tolerance_m_s
                and abs(tilt) <= math.radians(c.terminal_tilt_tolerance_deg)
                and abs(tilt_rate)
                <= math.radians(c.terminal_tilt_rate_tolerance_deg_s)
                and abs(command) <= math.radians(c.terminal_tilt_tolerance_deg)
                and signed_overshoot <= hard_limit+1e-12
            )
            terminal_run = terminal_run+1 if terminal_now else 0
            if terminal_run >= terminal_needed and arrival_time is None:
                arrival_time = float((index+1-terminal_needed+1)*dt)
            trace.append((time_s+dt, velocity, tilt, tilt_rate, command))
        hard_ok = (
            max_signed_overshoot <= hard_limit+1e-12
            and max_abs_tilt <= math.radians(c.max_tilt_deg)+1e-12
            and max_abs_rate <= math.radians(c.max_tilt_rate_deg_s)+1e-12
        )
        return {
            "pulse_s": float(pulse_s),
            "arrival_time_s": arrival_time,
            "terminal_reached": arrival_time is not None,
            "hard_constraints_satisfied": bool(hard_ok),
            "terminal_velocity_m_s": float(velocity),
            "terminal_velocity_error_m_s": float(
                velocity-self.target_velocity_m_s
            ),
            "terminal_tilt_deg": float(math.degrees(tilt)),
            "terminal_tilt_rate_deg_s": float(math.degrees(tilt_rate)),
            "max_signed_overshoot_m_s": float(max_signed_overshoot),
            "max_abs_tilt_deg": float(math.degrees(max_abs_tilt)),
            "max_abs_tilt_rate_deg_s": float(math.degrees(max_abs_rate)),
            "first_command_tilt_rad": float(trace[0][4]),
            "trace": trace,
        }

    def _forecast_candidates(self, *, now_s, velocity, tilt, tilt_rate,
                             current_command, episode_sign, pulse_grid):
        """Vectorized candidate forecast for the online selection path."""
        c, m = self.config, self.model
        pulses = np.asarray(pulse_grid, dtype=float)
        count = len(pulses)
        dt = c.prediction_step_s
        steps = int(math.ceil(c.prediction_horizon_s/dt))
        delay_steps = int(math.ceil(m.delay_s/dt-1e-12))
        initial_queue = self._delay_queue(now_s, delay_steps, dt)
        queue = np.tile(np.asarray(initial_queue), (count, 1))
        commands = np.full(count, current_command, dtype=float)
        angles = np.full(count, tilt, dtype=float)
        rates = np.full(count, tilt_rate, dtype=float)
        velocities = np.full(count, velocity, dtype=float)
        transition = _second_order_transition(m.wn_rad_s, m.zeta, dt)
        terminal_needed = max(1, int(math.ceil(c.terminal_dwell_s/dt)))
        terminal_run = np.zeros(count, dtype=int)
        arrival = np.full(count, np.nan)
        max_abs_tilt = np.abs(angles)
        max_abs_rate = np.abs(rates)
        max_signed_overshoot = np.full(
            count, episode_sign*(velocity-self.target_velocity_m_s)
        )
        first_commands = None
        residual = self.residual_learner.snapshot()["acceleration_m_s2"]
        command_step = math.radians(c.max_command_slew_deg_s)*dt
        max_command = episode_sign*math.radians(c.max_acceleration_tilt_deg)
        hard_limit = (
            c.overshoot_tolerance_m_s-c.velocity_uncertainty_margin_m_s
        )
        for index in range(steps):
            desired = np.where(index*dt < pulses-1e-12, max_command, 0.0)
            commands += np.clip(desired-commands, -command_step, command_step)
            if first_commands is None:
                first_commands = commands.copy()
            if delay_steps:
                effective = queue[:, 0].copy()
                if delay_steps > 1:
                    queue[:, :-1] = queue[:, 1:]
                queue[:, -1] = commands
            else:
                effective = commands
            equilibrium = m.command_gain*effective+m.projected_bias_rad
            centered_angle = angles-equilibrium
            next_angles = (
                transition[0, 0]*centered_angle
                + transition[0, 1]*rates + equilibrium
            )
            next_rates = (
                transition[1, 0]*centered_angle
                + transition[1, 1]*rates
            )
            acceleration_before = m.motion_gain*9.81*np.tan(angles)+residual
            acceleration_after = m.motion_gain*9.81*np.tan(next_angles)+residual
            velocities += 0.5*(acceleration_before+acceleration_after)*dt
            angles, rates = next_angles, next_rates
            max_abs_tilt = np.maximum(max_abs_tilt, np.abs(angles))
            max_abs_rate = np.maximum(max_abs_rate, np.abs(rates))
            signed_overshoot = episode_sign*(
                velocities-self.target_velocity_m_s
            )
            max_signed_overshoot = np.maximum(
                max_signed_overshoot, signed_overshoot
            )
            terminal_now = (
                (np.abs(velocities-self.target_velocity_m_s)
                 <= c.terminal_velocity_tolerance_m_s)
                & (np.abs(angles)
                   <= math.radians(c.terminal_tilt_tolerance_deg))
                & (np.abs(rates)
                   <= math.radians(c.terminal_tilt_rate_tolerance_deg_s))
                & (np.abs(commands)
                   <= math.radians(c.terminal_tilt_tolerance_deg))
                & (signed_overshoot <= hard_limit+1e-12)
            )
            terminal_run = np.where(terminal_now, terminal_run+1, 0)
            newly_arrived = np.isnan(arrival) & (terminal_run >= terminal_needed)
            arrival[newly_arrived] = (
                index-terminal_needed+2
            )*dt
        hard_ok = (
            (max_signed_overshoot <= hard_limit+1e-12)
            & (max_abs_tilt <= math.radians(c.max_tilt_deg)+1e-12)
            & (max_abs_rate <= math.radians(c.max_tilt_rate_deg_s)+1e-12)
        )
        return [{
            "pulse_s": float(pulses[index]),
            "arrival_time_s": (
                None if np.isnan(arrival[index]) else float(arrival[index])
            ),
            "terminal_reached": bool(not np.isnan(arrival[index])),
            "hard_constraints_satisfied": bool(hard_ok[index]),
            "terminal_velocity_m_s": float(velocities[index]),
            "terminal_velocity_error_m_s": float(
                velocities[index]-self.target_velocity_m_s
            ),
            "terminal_tilt_deg": float(math.degrees(angles[index])),
            "terminal_tilt_rate_deg_s": float(math.degrees(rates[index])),
            "max_signed_overshoot_m_s": float(max_signed_overshoot[index]),
            "max_abs_tilt_deg": float(math.degrees(max_abs_tilt[index])),
            "max_abs_tilt_rate_deg_s": float(
                math.degrees(max_abs_rate[index])
            ),
            "first_command_tilt_rad": float(first_commands[index]),
        } for index in range(count)]

    def _attitude_command(self, projected_tilt_rad, yaw_rad):
        cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
        body_direction = np.array([
            cosine*self.direction_xy[0]+sine*self.direction_xy[1],
            -sine*self.direction_xy[0]+cosine*self.direction_xy[1],
        ])
        tilt_deg = math.degrees(projected_tilt_rad)
        return (
            float(-tilt_deg*body_direction[1]),
            float(-tilt_deg*body_direction[0]),
        )

    def _fallback(self, reason, *, now=None, **details):
        result = {
            "offline_only": True,
            "flight_command_generated": False,
            "physical_flight_guaranteed": False,
            "hard_constraints_checked_in_learned_model": True,
            "action": "fallback_level",
            "reason": reason,
            "decision_time_s": now,
            "projected_command_tilt_rad": None,
            "roll_deg": None,
            "pitch_deg": None,
            "hard_terminal_constraints_satisfied": False,
            "residual_learning": self.residual_learner.snapshot(),
        }
        result.update(details)
        return result

    def decide(self, now_s, state: VelocityMPCState):
        try:
            now = float(now_s)
            if not math.isfinite(now):
                raise ValueError("decision time is nonfinite")
            velocity, tilt, tilt_rate, yaw, cross_velocity = (
                self._state_components(state)
            )
            age = now-state.time_s
            if age < 0 or age > self.config.max_state_age_s+1e-12:
                raise ValueError("state is future-dated or stale")
            if state.state_group_skew_s > self.config.max_state_group_skew_s:
                raise ValueError("state groups are not synchronized")
            if abs(cross_velocity) > self.config.max_cross_axis_speed_m_s:
                raise ValueError("cross-axis velocity is outside the model scope")
            if abs(velocity) > self.config.max_abs_velocity_m_s:
                raise ValueError("projected velocity is outside the safety envelope")
            if self._last_decision_time_s is not None and now < self._last_decision_time_s:
                raise ValueError("decision time moved backwards")
            if self._last_sent_tilt_rad is None:
                return self._fallback(
                    "missing_actual_sent_command_history", now=now,
                    measured_velocity_m_s=velocity,
                )
            if self._last_sent_time_s > now:
                raise ValueError("sent command history contains a future command")
            self._delay_queue(
                now,
                int(math.ceil(
                    self.model.delay_s/self.config.prediction_step_s-1e-12
                )),
                self.config.prediction_step_s,
            )
        except (TypeError, ValueError, AttributeError, OverflowError) as exc:
            return self._fallback("invalid_state_or_time", detail=str(exc))

        self._last_decision_time_s = now
        if self._episode_initial_velocity_m_s is None:
            self._episode_initial_velocity_m_s = velocity
            error = self.target_velocity_m_s-velocity
            self._episode_sign = 0.0 if abs(error) <= 1e-12 else math.copysign(1.0, error)
        episode_sign = self._episode_sign
        if episode_sign == 0:
            roll, pitch = self._attitude_command(0.0, yaw)
            terminal = (
                abs(velocity-self.target_velocity_m_s)
                <= self.config.terminal_velocity_tolerance_m_s
                and abs(tilt) <= math.radians(self.config.terminal_tilt_tolerance_deg)
                and abs(tilt_rate)
                <= math.radians(self.config.terminal_tilt_rate_tolerance_deg_s)
            )
            return {
                "offline_only": True, "flight_command_generated": False,
                "physical_flight_guaranteed": False,
                "hard_constraints_checked_in_learned_model": True,
                "action": "hold_level", "reason": "target_velocity_already_reached",
                "decision_time_s": now, "projected_command_tilt_rad": 0.0,
                "roll_deg": roll, "pitch_deg": pitch,
                "hard_terminal_constraints_satisfied": bool(terminal),
                "residual_learning": self.residual_learner.snapshot(),
            }

        grid = np.arange(
            0.0,
            self.config.prediction_horizon_s-self.config.terminal_dwell_s
                + 0.5*self.config.pulse_grid_step_s,
            self.config.pulse_grid_step_s,
        )
        candidates = self._forecast_candidates(
            now_s=now, velocity=velocity, tilt=tilt, tilt_rate=tilt_rate,
            current_command=self._last_sent_tilt_rad,
            episode_sign=episode_sign, pulse_grid=grid,
        )
        hard = [row for row in candidates if row["hard_constraints_satisfied"]]
        terminal = [row for row in hard if row["terminal_reached"]]
        if terminal:
            selected = min(terminal, key=lambda row: (
                row["arrival_time_s"],
                abs(row["terminal_velocity_error_m_s"]),
                row["pulse_s"],
            ))
            reason = "minimum_time_terminal_candidate"
        elif hard:
            # If the target equilibrium lies beyond this finite horizon, make
            # maximum safe progress and replan next cycle. This is the LMPC
            # receding-horizon step, not a claim that the terminal set was hit.
            selected = min(hard, key=lambda row: (
                abs(row["terminal_velocity_error_m_s"]),
                abs(row["terminal_tilt_deg"]),
                row["pulse_s"],
            ))
            reason = "maximum_safe_progress_terminal_beyond_horizon"
        else:
            return self._fallback(
                "no_hard_feasible_candidate_level_required", now=now,
                measured_velocity_m_s=velocity,
                target_velocity_m_s=self.target_velocity_m_s,
                candidate_count=len(candidates),
            )
        # Retain one full trajectory for diagnostics without constructing a
        # trace for every candidate in the real-time selection path.
        selected_with_trace = self._forecast(
            now_s=now, velocity=velocity, tilt=tilt, tilt_rate=tilt_rate,
            current_command=self._last_sent_tilt_rad,
            episode_sign=episode_sign, pulse_s=selected["pulse_s"],
        )
        selected["trace"] = selected_with_trace["trace"]
        command = selected["first_command_tilt_rad"]
        roll, pitch = self._attitude_command(command, yaw)
        return {
            "offline_only": True,
            "flight_command_generated": False,
            "physical_flight_guaranteed": False,
            "hard_constraints_checked_in_learned_model": True,
            "action": "level" if abs(command) <= 1e-12 else "accelerate",
            "reason": reason,
            "decision_time_s": now,
            "measurement_time_s": float(state.time_s),
            "state_age_s": float(now-state.time_s),
            "measured_velocity_m_s": velocity,
            "target_velocity_m_s": self.target_velocity_m_s,
            "episode_acceleration_sign": episode_sign,
            "projected_command_tilt_rad": command,
            "roll_deg": roll,
            "pitch_deg": pitch,
            "selected_pulse_s": selected["pulse_s"],
            "predicted_arrival_time_s": selected["arrival_time_s"],
            "predicted_terminal_velocity_m_s": selected["terminal_velocity_m_s"],
            "predicted_terminal_velocity_error_m_s": selected[
                "terminal_velocity_error_m_s"
            ],
            "predicted_terminal_tilt_deg": selected["terminal_tilt_deg"],
            "predicted_terminal_tilt_rate_deg_s": selected[
                "terminal_tilt_rate_deg_s"
            ],
            "predicted_max_signed_overshoot_m_s": selected[
                "max_signed_overshoot_m_s"
            ],
            "hard_terminal_constraints_satisfied": selected["terminal_reached"],
            "hard_path_constraints_satisfied": selected[
                "hard_constraints_satisfied"
            ],
            "candidate_count": len(candidates),
            "hard_feasible_candidate_count": len(hard),
            "terminal_feasible_candidate_count": len(terminal),
            "residual_learning": self.residual_learner.snapshot(),
            "selected_trace": selected["trace"],
        }
