"""Pure offline snapshots of finite braking-pulse candidates; never flight commands.

The caller supplies a frozen model, current measured state, past SENT commands,
and an explicit remaining target distance. Angles/velocities are projected onto
the original forward travel axis (negative command angle means braking). No
future measured state or retrospectively observed stop position is used.

A selected candidate is only a model diagnostic, NOT permission to fly it. The
full >= 0.8 s tail is checked even when a caller compares 50--100 ms refreshes.
Repeating this function on logged states after choosing different commands is
shadow snapshot analysis, not a counterfactual closed-loop trajectory. Model
error, localization timing, cross-axis motion and position control are not
identified here. Predicted non-reversal is not a physical stopping guarantee.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from types import SimpleNamespace

import numpy as np
from scipy.integrate import cumulative_trapezoid

from Interaction.braking_split_diagnostic import TiltTrial, tilt_prediction


@dataclass(frozen=True)
class FrozenTiltModel:
    delay_s: float
    wn_rad_s: float
    zeta: float
    command_gain: float
    motion_gain: float
    # Already projected on the chosen travel axis, NOT always world +Y.
    projected_bias_rad: float = 0.0


@dataclass(frozen=True)
class BrakingSnapshot:
    time_s: float
    position_m: float
    velocity_m_s: float
    tilt_rad: float
    tilt_rate_rad_s: float


def evaluate_braking_candidates(
    snapshot: BrakingSnapshot,
    model: FrozenTiltModel,
    command_history=(),
    *,
    target_remaining_m: float,
    pulse_durations_s=(0.0, 0.02, 0.05, 0.10),
    brake_tilt_deg=20.0,
    forecast_s=0.8,
    integration_step_s=0.002,
    reverse_limit_m_s=0.0,
    target_tolerance_m=0.02,
):
    """Return all candidate traces and a selected diagnostic or ``None``.

    ``command_history`` is ordered ``(sent_time_s, signed_tilt_rad)`` pairs.
    Include the last effective command plus every pending command. Empty history
    explicitly assumes level before the snapshot, while retaining measured tilt
    and angular rate. Future-sent commands and unordered history are rejected.
    Invalid inputs return no candidate; they never generate a fallback command.

    Feasibility checks all predicted samples, including the delayed response
    after leveling. Candidate ranking first minimizes absolute terminal speed,
    then remaining distance, then pulse duration. An accepted candidate need not
    predict a settled stop; ``settled_stop_predicted`` makes this explicit.
    """
    result = dict(offline_only=True, flight_command_generated=False,
                  selected=None, candidates=[], reason=None,
                  target_remaining_m=None, model=None, snapshot=None)
    try:
        numeric = np.asarray([
            *asdict(snapshot).values(), *asdict(model).values(),
            target_remaining_m, brake_tilt_deg, forecast_s,
            integration_step_s, reverse_limit_m_s, target_tolerance_m,
        ], dtype=float)
        durations = np.asarray(pulse_durations_s, dtype=float)
        history = np.asarray(list(command_history), dtype=float)
        if history.size == 0:
            history = np.empty((0, 2), dtype=float)
        valid = (
            np.isfinite(numeric).all()
            and durations.ndim == 1 and len(durations) > 0
            and np.isfinite(durations).all()
            and history.ndim == 2 and history.shape[1] == 2
            and np.isfinite(history).all()
        )
        if not valid:
            raise ValueError('nonfinite or malformed inputs')
        if (snapshot.velocity_m_s < 0 or target_remaining_m < 0
                or abs(snapshot.tilt_rad) >= np.radians(80)
                or model.delay_s < 0 or model.wn_rad_s <= 0
                or model.zeta <= 0 or model.command_gain <= 0
                or model.motion_gain <= 0
                or abs(model.projected_bias_rad) >= np.radians(80)
                or brake_tilt_deg <= 0 or forecast_s < 0.8
                or not 0 < integration_step_s <= 0.01
                or reverse_limit_m_s > 0 or target_tolerance_m < 0
                or np.any(durations < 0) or not np.any(durations == 0)
                or np.any(durations + model.delay_s >= forecast_s)
                or np.any(history[:, 0] > snapshot.time_s)
                or np.any(np.diff(history[:, 0]) < 0)
                or np.any(np.abs(history[:, 1]) >= np.radians(80))):
            raise ValueError('state, model, target, history or limits outside domain')
    except (TypeError, ValueError, AttributeError, OverflowError) as exc:
        result.update(reason='invalid_input', detail=str(exc))
        return result

    result.update(target_remaining_m=float(target_remaining_m),
                  model=asdict(model), snapshot=asdict(snapshot),
                  sent_command_history=history.tolist(),
                  limits=dict(reverse_limit_m_s=float(reverse_limit_m_s),
                              target_tolerance_m=float(target_tolerance_m),
                              forecast_s=float(forecast_s),
                              integration_step_s=float(integration_step_s)))
    brake_rad = -np.radians(min(float(brake_tilt_deg), 20.0))
    fit = dict(model='second_order', delay_s=model.delay_s,
               wn_rad_s=model.wn_rad_s, zeta=model.zeta,
               gain=model.command_gain,
               bias_world_y_rad=model.projected_bias_rad)
    for duration in sorted(set(float(x) for x in durations)):
        past_times = history[:, 0] - snapshot.time_s
        command_times = np.r_[past_times, [0.0] if duration == 0 else [0.0, duration]]
        command_angles = np.r_[history[:, 1], [0.0] if duration == 0 else [brake_rad, 0.0]]
        # Preserve exact delayed pulse boundaries as well as a fine integration
        # grid. A short pulse must not disappear between integration samples.
        events = command_times + model.delay_s
        times = np.unique(np.r_[np.arange(0, forecast_s, integration_step_s),
                                events[(events >= 0) & (events <= forecast_s)],
                                forecast_s])
        trial = SimpleNamespace(times=times, command_times=command_times,
                                commands=9.81*np.tan(command_angles),
                                direction=np.array([0.0, 1.0]))
        item = TiltTrial(trial, np.array([snapshot.tilt_rad]),
                         np.array([snapshot.tilt_rate_rad_s]))
        with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
            angle = tilt_prediction(item, fit)
            acceleration = model.motion_gain * 9.81 * np.tan(angle)
            velocity = snapshot.velocity_m_s + cumulative_trapezoid(
                acceleration, times, initial=0)
            displacement = cumulative_trapezoid(velocity, times, initial=0)
        if (not np.isfinite([angle, velocity, displacement]).all()
                or np.any(np.abs(angle) >= np.radians(80))):
            result['candidates'].append(dict(pulse_duration_s=duration,
                feasible=False, rejection_reasons=['invalid_prediction']))
            continue
        reverse = bool(np.min(velocity) < reverse_limit_m_s)
        overshoot = float(max(0.0, np.max(displacement)-target_remaining_m))
        reasons = []
        if reverse:
            reasons.append('predicted_reverse')
        if overshoot > target_tolerance_m:
            reasons.append('predicted_target_overshoot')
        tail = times >= forecast_s-0.10
        terminal_rate = float((angle[-1]-angle[-2])/(times[-1]-times[-2]))
        candidate = dict(
            pulse_duration_s=duration,
            hypothetical_pulse_tilt_deg=(0.0 if duration == 0 else float(np.degrees(brake_rad))),
            feasible=not reasons, rejection_reasons=reasons,
            predicted_reverse=reverse,
            min_velocity_m_s=float(np.min(velocity)),
            terminal_velocity_m_s=float(velocity[-1]),
            terminal_mean_velocity_m_s=float(np.mean(velocity[tail])),
            end_displacement_m=float(displacement[-1]),
            end_position_m=float(snapshot.position_m+displacement[-1]),
            max_forward_displacement_m=float(np.max(displacement)),
            overshoot_m=overshoot,
            rollback_m=float(np.max(displacement)-displacement[-1]),
            settled_stop_predicted=bool(
                np.max(np.abs(velocity[tail])) <= 0.04
                and np.max(np.abs(angle[tail])) <= np.radians(3)
                and abs(terminal_rate) <= np.radians(5)),
            time_s=times.tolist(), tilt_rad=angle.tolist(),
            velocity_m_s=velocity.tolist(), displacement_m=displacement.tolist(),
        )
        result['candidates'].append(candidate)

    level = result['candidates'][0]  # Zero duration required and sorted first.
    if 'invalid_prediction' in level['rejection_reasons']:
        result['reason'] = 'invalid_prediction'
    elif level['predicted_reverse']:
        result['reason'] = 'level_now_already_predicts_reverse'
    else:
        feasible = [row for row in result['candidates'] if row['feasible']]
        if feasible:
            result['selected'] = min(feasible, key=lambda row: (
                abs(row['terminal_velocity_m_s']),
                abs(target_remaining_m-row['end_displacement_m']),
                row['pulse_duration_s']))
            result['reason'] = 'model_candidate_selected_not_flight_validated'
        else:
            result['reason'] = 'no_feasible_candidate'
    return result
