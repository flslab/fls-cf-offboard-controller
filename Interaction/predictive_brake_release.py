"""Offline-only prediction of when to end a reverse-tilt braking pulse.

This is an exploratory controller *simulation*, not a flight command generator.
All quantities are projected onto the original forward travel direction. The
only simulated actions are a negative braking tilt and level; there is no
forward cancellation pulse, position target, or position-control handoff.

Each decision compares leveling now with braking for one more control tick and
then leveling. Both forecasts include the measured angle/rate, known SENT
command queue, command delay and the complete >= 0.8 s tail. The 0.08 m/s margin
is an explicit exploratory allowance, NOT an identified confidence interval or
a guarantee that a real aircraft will stop without reversing.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from types import SimpleNamespace

import numpy as np
from scipy.integrate import cumulative_trapezoid

from Interaction.braking_split_diagnostic import TiltTrial, tilt_prediction
from Interaction.offline_braking_selector import BrakingSnapshot, FrozenTiltModel


@dataclass(frozen=True)
class BrakeReleaseConfig:
    control_period_s: float = 0.01
    prediction_horizon_s: float = 0.8
    release_speed_margin_m_s: float = 0.08
    brake_tilt_deg: float = 20.0
    max_brake_duration_s: float = 0.5
    integration_step_s: float = 0.002
    max_state_age_s: float = 0.03

    def validate(self) -> None:
        """Raise ``ValueError`` for invalid finite domains before simulation.

        Public validation lets callers reject non-advancing time steps before
        entering their loop. The policy also calls it, retaining its normal
        invalid-input simulated-level latch instead of raising to its caller.
        Model-dependent delay/horizon constraints remain policy validation.
        """
        try:
            numeric = np.asarray(list(asdict(self).values()), dtype=float)
            if numeric.ndim != 1 or not np.isfinite(numeric).all():
                raise ValueError('configuration fields must be finite scalars')
            if (not 0 < self.control_period_s <= 0.1
                    or self.prediction_horizon_s < 0.8
                    or not 0 < self.integration_step_s <= min(0.01, self.control_period_s)
                    or self.release_speed_margin_m_s < 0
                    or not 0 < self.brake_tilt_deg <= 20
                    or self.max_brake_duration_s <= 0 or self.max_state_age_s < 0):
                raise ValueError('brake-release configuration outside domain')
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f'invalid brake-release configuration: {exc}') from exc


class PredictiveBrakeRelease:
    """Latched early-level decisions for an isolated offline braking episode.

    Construct a new object for each episode. ``command_history`` contains ordered
    ``(sent_time_s, signed_tilt_rad)`` pairs, including the last effective command
    and every pending command. Empty history explicitly assumes prior level.
    Command history must be complete for meaningful forecasts; this class cannot
    verify completeness. It never inserts its own decisions into that history.

    ``snapshot.time_s`` is measurement time, while ``decision_time_s`` is the
    current decision time on the same clock. Known commands sent since the
    measurement are allowed. No later measurements or future SENT commands are
    accepted. Old state is propagated with the frozen model, not relabeled as a
    fresh measurement. ``state_valid`` is an additional caller-provided gate.

    Invalid inputs latch simulated level. They do not authorize a real fallback
    command or replace hardware/localization safety handling. Once released, the
    original reason is retained and this episode can never resume braking.
    """

    def __init__(self, model: FrozenTiltModel, config: BrakeReleaseConfig | None = None):
        self.model = model
        self.config = config if config is not None else BrakeReleaseConfig()
        self._started_s = None
        self._last_decision_s = None
        self._released = False
        self._release_reason = None
        self._release_time_s = None
        self._release_detail = None

    def _base_result(self):
        try:
            margin = float(self.config.release_speed_margin_m_s)
            margin = margin if np.isfinite(margin) else None
        except (AttributeError, TypeError, ValueError, OverflowError):
            margin = None
        return dict(
            offline_only=True, flight_command_generated=False,
            physical_non_reversal_guaranteed=False,
            position_target_used=False, command_tilt_rad=0.0,
            released=self._released, reason=None, release_reason=self._release_reason,
            release_time_s=self._release_time_s, decision_time_s=None,
            measurement_time_s=None, state_age_s=None, elapsed_brake_s=None,
            nominal_uncertainty_margin=margin,
            nominal_uncertainty_margin_m_s=margin,
            margin_is_calibrated_confidence_bound=False,
            level_now=None, continue_then_level=None,
            level_now_min_velocity_m_s=None, level_now_final_velocity_m_s=None,
            continue_then_level_min_velocity_m_s=None,
            continue_then_level_final_velocity_m_s=None,
            predicted_decision_velocity_m_s=None,
            forecast_status='not_computed', detail=None,
        )

    def _release(self, result, reason, detail=None):
        self._released = True
        self._release_reason = reason
        self._release_time_s = result['decision_time_s']
        self._release_detail = detail
        result.update(command_tilt_rad=0.0, released=True, reason=reason,
                      release_reason=reason, release_time_s=self._release_time_s,
                      detail=detail)
        return result

    def _validate(self, snapshot, history, decision_time):
        self.config.validate()
        numeric = np.asarray([*asdict(snapshot).values(), *asdict(self.model).values(),
                              decision_time], dtype=float)
        if (not np.isfinite(numeric).all() or history.ndim != 2
                or history.shape[1] != 2 or not np.isfinite(history).all()):
            raise ValueError('nonfinite or malformed state, model, config or history')
        c, m = self.config, self.model
        if (m.delay_s < 0 or m.wn_rad_s <= 0 or m.zeta <= 0
                or m.command_gain <= 0 or m.motion_gain <= 0
                or abs(m.projected_bias_rad) >= np.radians(80)
                or abs(snapshot.tilt_rad) >= np.radians(80)
                or c.control_period_s + m.delay_s >= c.prediction_horizon_s
                or np.any(np.diff(history[:, 0]) < 0)
                or np.any(history[:, 0] > decision_time)
                or np.any(np.abs(history[:, 1]) >= np.radians(80))):
            raise ValueError('state, model, config or SENT history outside domain')
        if self._last_decision_s is not None and decision_time < self._last_decision_s:
            raise ValueError('decision time moved backwards')

    def _forecast(self, snapshot, history, decision_time, pulse_s):
        c, m = self.config, self.model
        age = decision_time - snapshot.time_s
        brake = -np.radians(c.brake_tilt_deg)
        action_times = [age] if pulse_s == 0 else [age, age + pulse_s]
        action_angles = [0.0] if pulse_s == 0 else [brake, 0.0]
        command_times = np.r_[history[:, 0] - snapshot.time_s, action_times]
        command_angles = np.r_[history[:, 1], action_angles]
        end = age + c.prediction_horizon_s
        events = command_times + m.delay_s
        # Integrate from the actual measurement time. Include the decision time
        # and exact pending-command events even between regular grid samples.
        times = np.unique(np.r_[np.arange(0, end, c.integration_step_s), age,
                                events[(events >= 0) & (events <= end)], end])
        trial = SimpleNamespace(times=times, command_times=command_times,
                                commands=9.81 * np.tan(command_angles),
                                direction=np.array([0.0, 1.0]))
        item = TiltTrial(trial, np.array([snapshot.tilt_rad]),
                         np.array([snapshot.tilt_rate_rad_s]))
        fit = dict(model='second_order', delay_s=m.delay_s, wn_rad_s=m.wn_rad_s,
                   zeta=m.zeta, gain=m.command_gain,
                   bias_world_y_rad=m.projected_bias_rad)
        with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
            angle = tilt_prediction(item, fit)
            acceleration = m.motion_gain * 9.81 * np.tan(angle)
            velocity = snapshot.velocity_m_s + cumulative_trapezoid(
                acceleration, times, initial=0)
        if (not np.isfinite(angle).all() or not np.isfinite(velocity).all()
                or np.any(np.abs(angle) >= np.radians(80))):
            raise ValueError('nonfinite or out-of-domain predicted state')
        future = times >= age
        times, angle, velocity = times[future] - age, angle[future], velocity[future]
        return dict(pulse_duration_s=float(pulse_s),
                    min_velocity_m_s=float(np.min(velocity)),
                    terminal_velocity_m_s=float(velocity[-1]),
                    predicted_reverse=bool(np.min(velocity) < 0),
                    predicted_decision_velocity_m_s=float(velocity[0]),
                    predicted_decision_tilt_rad=float(angle[0]),
                    time_s=times.tolist(), tilt_rad=angle.tolist(),
                    velocity_m_s=velocity.tolist())

    def update(self, snapshot: BrakingSnapshot, command_history=(), *,
               state_valid=True, decision_time_s=None):
        """Return a simulated tilt choice and explicit full-tail diagnostics.

        The minimum includes the decision instant and every future forecast
        sample. A low measured entry speed can therefore correctly cause an
        immediate release, rather than being hidden by terminal-only scoring.
        """
        result = self._base_result()
        if self._released:
            result.update(reason=self._release_reason, detail=self._release_detail,
                          forecast_status='skipped_latched_release')
            return result
        try:
            if not isinstance(state_valid, (bool, np.bool_)):
                raise ValueError('state_valid must be a boolean')
            now = float(snapshot.time_s if decision_time_s is None else decision_time_s)
            result['decision_time_s'] = now if np.isfinite(now) else None
            history = np.asarray(list(command_history), dtype=float)
            if history.size == 0 and history.ndim == 1:
                history = np.empty((0, 2), dtype=float)
            self._validate(snapshot, history, now)
            result.update(measurement_time_s=float(snapshot.time_s),
                          state_age_s=float(now - snapshot.time_s),
                          sent_command_history=history.tolist(),
                          prior_level_assumed=not len(history))
        except (TypeError, ValueError, AttributeError, OverflowError) as exc:
            return self._release(result, 'invalid_input', str(exc))
        if not state_valid:
            return self._release(result, 'invalid_state', 'caller state-validity gate is false')
        if result['state_age_s'] < 0:
            return self._release(result, 'future_measurement', 'measurement is newer than decision time')
        if result['state_age_s'] > self.config.max_state_age_s + 1e-12:
            return self._release(result, 'stale_state', 'measurement exceeds configured maximum age')
        if self._started_s is None:
            self._started_s = now
        self._last_decision_s = now
        result['elapsed_brake_s'] = float(now - self._started_s)
        if snapshot.velocity_m_s < 0:
            return self._release(result, 'measured_reverse')
        if result['elapsed_brake_s'] >= self.config.max_brake_duration_s - 1e-12:
            return self._release(result, 'maximum_brake_duration')
        try:
            level = self._forecast(snapshot, history, now, 0.0)
            more = self._forecast(snapshot, history, now, self.config.control_period_s)
        except (ValueError, FloatingPointError, OverflowError) as exc:
            return self._release(result, 'invalid_prediction', str(exc))
        result.update(
            forecast_status='complete', level_now=level, continue_then_level=more,
            level_now_min_velocity_m_s=level['min_velocity_m_s'],
            level_now_final_velocity_m_s=level['terminal_velocity_m_s'],
            continue_then_level_min_velocity_m_s=more['min_velocity_m_s'],
            continue_then_level_final_velocity_m_s=more['terminal_velocity_m_s'],
            predicted_decision_velocity_m_s=level['predicted_decision_velocity_m_s'],
        )
        if level['min_velocity_m_s'] < 0:
            return self._release(result, 'level_now_already_predicts_reverse')
        if more['min_velocity_m_s'] < self.config.release_speed_margin_m_s:
            return self._release(result, 'next_tick_tail_below_margin')
        result.update(command_tilt_rad=float(-np.radians(self.config.brake_tilt_deg)),
                      released=False, reason='continue_braking_one_tick')
        return result
