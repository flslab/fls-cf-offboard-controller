"""Bounded, non-actuating system identification for calibration telemetry.

The host receive clock yields an *effective* delay, not a physical actuator
latency. Only opposed world-Y trials are supported. A caller must submit a
segment after its actual recovery command: seeing the last phase in a sample
stream cannot by itself establish that the planned observation tail completed.

Fitting may run during calibration, but no function here changes a controller,
approves a model for flight, or learns a brake-release policy. Predictions use
one measured initial state and an explicitly supplied command history/schedule.
"""
from __future__ import annotations

from collections import OrderedDict
import hashlib
import json
import math
import warnings as python_warnings

import numpy as np
from scipy.integrate import cumulative_trapezoid

from Interaction.braking_replay import Trial, attitude_acceleration, first_crossing
from Interaction.braking_split_diagnostic import (
    TiltTrial, brake_anchor, fit_tilt, motion_gain, tilt_prediction,
)


PHASES = ("level_before_acceleration", "accelerate", "level_before_brake",
          "brake", "level_after_brake")
PARAMETER_BOUNDS = {
    "delay_s": (0., .15), "wn_rad_s": (5., 100.), "zeta": (.2, 2.),
    "gain": (.7, 1.3), "bias_world_y_rad": (-.035, .035),
}
PARAMETER_UNITS = {
    "delay_s": "s", "wn_rad_s": "rad/s", "zeta": "dimensionless",
    "gain": "dimensionless", "bias_world_y_rad": "rad",
    "motion_gain": "dimensionless",
}
HORIZONS_S = (.05, .10, .20, .40)
_CLOCK_TOL_S = 1e-6


def _number(value, name):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} must be a finite number")
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return value


def _vector(value, count, name):
    if not isinstance(value, (list, tuple, np.ndarray)) or len(value) != count:
        raise ValueError(f"{name} must contain {count} finite values")
    return np.array([_number(x, name) for x in value], dtype=float)


def _json_safe(value):
    """Serializable reports never contain NaN or Infinity, even in diagnostics."""
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [_json_safe(v) for v in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    return value


def build_tilt_trials(samples, max_sample_gap_s=.06):
    """Validate complete raw trials without sorting, deduping, or filling gaps.

    ``command_started_at`` is the actual first sent command for that phase and
    stays constant within the phase. It must share the host clock of timestamp.
    Recovery samples, if present, are ignored after the five observed phases;
    the caller remains responsible for certifying complete planned trial tails.
    Time arrays are relative to each segment's first sample; negative command
    onsets are retained so pending commands survive a new prediction anchor.
    """
    max_gap = _number(max_sample_gap_s, "max_sample_gap_s")
    if max_gap <= 0:
        raise ValueError("max_sample_gap_s must be positive")
    grouped = OrderedDict()
    last_sample_time = None
    for row in samples:
        if not isinstance(row, dict):
            raise ValueError("samples must contain dictionaries")
        try:
            segment = _number(row["segment_id"], "segment_id")
            sample_time = _number(row["timestamp"], "timestamp")
        except KeyError as exc:
            raise ValueError(f"missing {exc.args[0]}") from exc
        if segment < 0 or int(segment) != segment:
            raise ValueError("segment_id must be a nonnegative integer")
        if last_sample_time is not None and sample_time <= last_sample_time:
            raise ValueError("timestamps must be strictly increasing in the calibration sample stream")
        last_sample_time = sample_time
        grouped.setdefault(int(segment), []).append(row)
    if not grouped:
        raise ValueError("no calibration samples")

    items = []
    for segment, rows in grouped.items():
        times, positions, velocities, angles, rates = [], [], [], [], []
        phase_times, commands = OrderedDict(), []
        counts = {name: 0 for name in PHASES}
        direction = None
        previous_t = None
        recovery_seen = False
        phase = None
        current_command = None
        for row in rows:
            try:
                timestamp = _number(row["timestamp"], "timestamp")
                row_phase = row["phase"]
                if previous_t is not None and timestamp <= previous_t:
                    raise ValueError("timestamps must be strictly increasing within a segment")
                if row_phase == "recovery":
                    if list(phase_times) != list(PHASES):
                        raise ValueError("recovery before all required phases")
                    recovery_seen = True
                    previous_t = timestamp
                    continue
                if recovery_seen or row_phase not in PHASES:
                    raise ValueError("unexpected phase or samples following recovery")
                started = _number(row["command_started_at"], "command_started_at")
                row_direction = _vector(row["direction_xy"], 2, "direction_xy")
                if (abs(row_direction[0]) > 1e-6 or
                        abs(abs(row_direction[1])-1.) > 1e-6):
                    raise ValueError("only unit positive/negative world-Y directions are supported")
                if direction is None:
                    direction = np.array([0., 1. if row_direction[1] > 0 else -1.])
                elif not np.allclose(row_direction, direction, atol=1e-6, rtol=0):
                    raise ValueError("direction changed within a segment")
                command = _vector(row["command_acceleration_xy"], 2, "command_acceleration_xy")
                if abs(command[0]) > 1e-6:
                    raise ValueError("command must be planar world-Y only")
                projected_command = float(command @ direction)
                if row_phase == "accelerate" and projected_command <= 0:
                    raise ValueError("accelerate command must be positive along trial direction")
                if row_phase == "brake" and projected_command >= 0:
                    raise ValueError("brake command must oppose trial direction")
                if row_phase.startswith("level_") and np.max(np.abs(command)) > 1e-6:
                    raise ValueError("level phase must command zero planar acceleration")
                rpy = _vector(row["actual_attitude_rpy_rad"], 3, "actual_attitude_rpy_rad")
                angular = _vector(row["angular_velocity_rad_s"], 3, "angular_velocity_rad_s")
                velocity = _vector(row["velocity_xy"], 2, "velocity_xy")
                position = _vector(row["position_xy"], 2, "position_xy")
                if _number(row["battery_voltage_V"], "battery_voltage_V") <= 0:
                    raise ValueError("battery_voltage_V must be positive")
                if "state_group_skew_s" in row:
                    skew = _number(row["state_group_skew_s"], "state_group_skew_s")
                    if skew < 0 or skew > .03:
                        raise ValueError("state group skew exceeds 30 ms")
            except KeyError as exc:
                raise ValueError(f"segment {segment}: missing required field {exc.args[0]}") from exc

            if started > timestamp + _CLOCK_TOL_S:
                raise ValueError("command_started_at is in the future or uses a different clock")
            if row_phase != phase:
                if len(phase_times) >= len(PHASES) or row_phase != PHASES[len(phase_times)]:
                    raise ValueError(f"segment {segment}: missing, repeated, or out-of-order phases")
                if phase_times and started <= next(reversed(phase_times.values())):
                    raise ValueError("command phase start times must be strictly increasing")
                if previous_t is not None and started < previous_t-_CLOCK_TOL_S:
                    raise ValueError("phase start precedes a sample attributed to the previous command")
                if timestamp-started > max_gap+_CLOCK_TOL_S:
                    raise ValueError("first sample is too far after actual phase command start")
                phase_times[row_phase] = started
                commands.append(projected_command)
                current_command = command
                phase = row_phase
            elif (abs(started-phase_times[phase]) > _CLOCK_TOL_S or
                  not np.allclose(command, current_command, atol=1e-9, rtol=0)):
                raise ValueError("command start or value changed within a phase")
            if previous_t is not None and timestamp-previous_t > max_gap+_CLOCK_TOL_S:
                raise ValueError("telemetry sample gap exceeds limit; interpolation is not allowed")

            projected_acceleration = attitude_acceleration(rpy, direction)
            if not math.isfinite(projected_acceleration):
                raise ValueError("nonfinite attitude acceleration")
            # Same documented small-planar-angle RATE_EST projection used by
            # braking_split_diagnostic, not an exact Euler-angle derivative.
            c, s = np.cos(rpy[2]), np.sin(rpy[2])
            dx, dy = direction
            rate = -(angular[1]*(c*dx+s*dy) + angular[0]*(-s*dx+c*dy))
            if not math.isfinite(rate):
                raise ValueError("nonfinite projected angular rate")
            times.append(timestamp)
            positions.append(float(position @ direction))
            velocities.append(float(velocity @ direction))
            angles.append(float(np.arctan(projected_acceleration/9.81)))
            rates.append(float(rate))
            counts[phase] += 1
            previous_t = timestamp

        if list(phase_times) != list(PHASES) or any(count < 2 for count in counts.values()):
            raise ValueError(f"segment {segment}: all five phases need at least two observed samples")
        if len(times) < 20:
            raise ValueError(f"segment {segment}: at least 20 observations are required")
        origin = times[0]
        relative_times = np.array(times)-origin
        command_times = np.array(list(phase_times.values()))-origin
        trial = Trial(
            segment=segment,
            duration=float(phase_times["level_before_brake"]-phase_times["accelerate"]),
            direction=direction, times=relative_times, positions=np.array(positions),
            velocities=np.array(velocities), attitude_acceleration=9.81*np.tan(angles),
            command_times=command_times, commands=np.array(commands),
            phase_times={name: timestamp-origin for name, timestamp in phase_times.items()},
            max_gap=float(np.max(np.diff(relative_times))),
        )
        items.append(TiltTrial(trial=trial, angle=np.array(angles), rate=np.array(rates)))
    return items


def _identifiability(items, fit):
    """Local sensitivity in unit-bound parameter coordinates, not a CI."""
    def residual(candidate):
        result = []
        for item in items:
            dt = np.diff(item.trial.times)
            weight = np.r_[dt[0]/2, (dt[:-1]+dt[1:])/2, dt[-1]/2]
            weight /= weight.sum()*len(items)
            result.extend((tilt_prediction(item, candidate)-item.angle)*np.sqrt(weight))
        return np.array(result)

    columns = []
    near_bounds = []
    for key, (lower, upper) in PARAMETER_BOUNDS.items():
        width = upper-lower
        step = width*1e-5
        lo, hi = max(lower, fit[key]-step), min(upper, fit[key]+step)
        columns.append((residual(dict(fit, **{key: hi}))-
                        residual(dict(fit, **{key: lo})))*width/(hi-lo))
        if min(fit[key]-lower, upper-fit[key]) <= width*1e-4:
            near_bounds.append(key)
    singular = np.linalg.svd(np.column_stack(columns), compute_uv=False)
    rank = int(np.sum(singular > max(singular[0]*1e-6, 1e-7)))
    condition = None if singular[-1] <= 1e-12 else float(singular[0]/singular[-1])
    return {
        "method": "local finite-difference sensitivity scaled by declared parameter bounds",
        "parameter_order": list(PARAMETER_BOUNDS),
        "singular_values": singular.tolist(), "rank": rank,
        "condition_number": condition,
        "identifiable": bool(rank == 5 and condition is not None and condition < 1e6
                             and singular[-1] >= 1e-5),
        "bound_active_parameters": near_bounds,
        "uncertainty_interval_available": False,
    }


def fit_predictive_model(samples, *, max_sample_gap_s=.06):
    """Fit one immutable candidate using only the supplied training segments."""
    samples = list(samples)
    items = build_tilt_trials(samples, max_sample_gap_s=max_sample_gap_s)
    if len(items) < 2 or set(int(t.trial.direction[1]) for t in items) != {-1, 1}:
        raise ValueError("training requires at least one complete opposed world-Y trial pair")
    with python_warnings.catch_warnings(record=True) as caught_warnings:
        python_warnings.simplefilter("always", RuntimeWarning)
        attitude_fit = fit_tilt(items, "second_order")
    numerical_warnings = list(dict.fromkeys(str(warning.message) for warning in caught_warnings
                                            if issubclass(warning.category, RuntimeWarning)))
    # _json_safe is for absent report diagnostics, never for repairing a failed
    # fit. In particular BLAS residual products in the reused optimizer may
    # warn or overflow even when every residual itself is finite.
    for key, (lower, upper) in PARAMETER_BOUNDS.items():
        value = _number(attitude_fit.get(key), "fitted " + key)
        if not lower <= value <= upper:
            raise ValueError("fitted parameter is outside its declared bounds: " + key)
    reported_rmse = _number(attitude_fit.get("train_rmse_deg"), "train_rmse_deg")
    costs = attitude_fit.get("seed_costs")
    if not isinstance(costs, list) or not costs:
        raise ValueError("fit is missing finite seed costs")
    costs = [_number(value, "fit seed cost") for value in costs]
    if reported_rmse < 0 or min(costs) < 0:
        raise ValueError("fit costs and RMSE must be nonnegative")
    # Independently check the selected model's objective using elementwise
    # squares/sums instead of another BLAS dot product.
    checked_cost = 0.
    for item in items:
        dt = np.diff(item.trial.times)
        weight = np.r_[dt[0]/2, (dt[:-1]+dt[1:])/2, dt[-1]/2]
        weight /= weight.sum()*len(items)
        residual = tilt_prediction(item, attitude_fit)-item.angle
        checked_cost += float(np.sum(np.square(residual)*weight))
    checked_cost = _number(checked_cost, "independently checked training objective")
    if (not np.isclose(np.radians(reported_rmse)**2, checked_cost, rtol=1e-5, atol=1e-12)
            or not np.isclose(min(costs), checked_cost, rtol=1e-5, atol=1e-12)):
        raise ValueError("reported fit objective disagrees with independently recomputed residuals")
    gain = motion_gain(items)
    diagnostic = _identifiability(items, attitude_fit)
    warnings = []
    if diagnostic["bound_active_parameters"]:
        warnings.append("attitude parameters reached declared fit bounds")
    if not diagnostic["identifiable"]:
        warnings.append("local parameter sensitivity is weak or rank deficient")
    if numerical_warnings:
        warnings.append("optimizer emitted numeric warnings; finite metadata and residual objective were independently checked")
    warnings.extend([
        "host-receive effective delay includes telemetry and scheduling latency",
        "world-Y small-planar-angle identification only; cross-axis dynamics are not fitted",
        "last-phase samples do not certify trial completion; caller must observe actual recovery send",
        "training fit is a candidate, not independent validation or flight approval",
    ])
    training_rows = [row for row in samples if row.get("phase") in PHASES]
    # Hash only the consumed fields. Unrelated metadata need not be JSON-safe.
    fields = ("segment_id", "timestamp", "command_started_at", "phase", "direction_xy",
              "command_acceleration_xy", "actual_attitude_rpy_rad", "angular_velocity_rad_s",
              "velocity_xy", "position_xy", "battery_voltage_V", "state_group_skew_s")
    canonical = [{key: _json_safe(row[key]) for key in fields if key in row} for row in training_rows]
    digest = hashlib.sha256(json.dumps(canonical, sort_keys=True, separators=(",", ":"),
                                       allow_nan=False).encode()).hexdigest()
    ranges = []
    for item in items:
        trial = item.trial
        rows = [r for r in training_rows if int(r["segment_id"]) == trial.segment]
        ranges.append({
            "segment_id": trial.segment, "direction_y": int(trial.direction[1]),
            "sample_count": len(trial.times), "host_time_origin_s": float(rows[0]["timestamp"]),
            "observed_duration_s": float(trial.times[-1]), "max_sample_gap_s": trial.max_gap,
            "phase_sample_counts": {phase: sum(r["phase"] == phase for r in rows) for phase in PHASES},
            "phase_command_times_relative_s": trial.phase_times,
            "observed_phase_last_sample_s": {
                phase: max(float(r["timestamp"]) for r in rows if r["phase"] == phase)-float(rows[0]["timestamp"])
                for phase in PHASES},
            "command_acceleration_m_s2": [float(min(trial.commands)), float(max(trial.commands))],
            "theta_rad": [float(min(item.angle)), float(max(item.angle))],
            "velocity_m_s": [float(min(trial.velocities)), float(max(trial.velocities))],
            "position_m": [float(min(trial.positions)), float(max(trial.positions))],
            "battery_voltage_V": [min(float(r["battery_voltage_V"]) for r in rows),
                                  max(float(r["battery_voltage_V"]) for r in rows)],
        })
    result = dict(
        schema_version=1, kind="delayed_second_order_planar_prediction",
        attitude_fit=attitude_fit, motion_gain=gain,
        train_segment_ids=[t.trial.segment for t in items],
        clock_scope="host_receive_effective_delay", prediction_scope="attitude_command_only",
        runtime_enabled=False, deployment_approved=False,
        candidate_status=("requires_identifiability_review" if
                          (not diagnostic["identifiable"] or diagnostic["bound_active_parameters"])
                          else "requires_held_out_validation"),
        independent_validation_complete=False, identifiability=diagnostic,
        numerical_diagnostics=dict(finite_fit_metadata=True,
                                   residual_objective_independently_checked=True,
                                   checked_training_objective_rad2=checked_cost,
                                   optimizer_runtime_warnings=numerical_warnings),
        parameter_bounds={**PARAMETER_BOUNDS, "motion_gain": (.2, 2.5)},
        parameter_units=PARAMETER_UNITS, data_ranges=ranges,
        source_provenance=dict(sample_sha256=digest, sample_count=len(training_rows),
                               source="raw_calibration_samples", command_clock="caller_supplied_phase_command_host_time",
                               command_clock_caveat="Live samples require actual sends; historical reconstructed sends must be labeled by the caller.",
                               completion_certified_by="caller_after_actual_recovery_send"),
        warnings=warnings,
    )
    return _json_safe(result)


def _validate_model(model):
    if (not isinstance(model, dict) or model.get("schema_version") != 1 or
            model.get("kind") != "delayed_second_order_planar_prediction"):
        raise ValueError("unsupported predictive model schema")
    fit = model.get("attitude_fit")
    if not isinstance(fit, dict) or fit.get("model") != "second_order":
        raise ValueError("a delayed stable second-order attitude fit is required")
    fit = dict(fit)
    for key, (lower, upper) in PARAMETER_BOUNDS.items():
        try:
            fit[key] = _number(fit[key], key)
        except KeyError as exc:
            raise ValueError(f"missing model parameter {key}") from exc
        if not lower <= fit[key] <= upper:
            raise ValueError(f"model parameter {key} is outside declared bounds")
    gain = _number(model.get("motion_gain"), "motion_gain")
    if not .2 < gain < 2.5:
        raise ValueError("motion_gain is outside declared bounds")
    return fit, gain


def predict_trajectory(model, initial_state, command_history, sample_times):
    """Predict from a single state; no future measurement argument exists.

    State fields: time_s, position_m, velocity_m_s, theta_rad, omega_rad_s,
    direction_y. Position/velocity/tilt/rate and command tilt are projected onto
    the trial direction, not unsigned speeds or world-Y values. Command entries
    contain time_s (actual sent history or declared future schedule) and tilt_rad.
    Before the first supplied command, level (zero tilt) is the assumed command;
    therefore callers must retain the last effective command and pending history.
    Future commands, when provided, make the result a conditional trajectory.
    Returned arrays ``x``, ``v``, ``theta`` use m, m/s, and rad respectively.
    """
    fit, gain = _validate_model(model)
    required = ("time_s", "position_m", "velocity_m_s", "theta_rad", "omega_rad_s", "direction_y")
    try:
        state = {key: _number(initial_state[key], key) for key in required}
    except (KeyError, TypeError) as exc:
        raise ValueError("initial_state is missing a required finite state field") from exc
    direction = state["direction_y"]
    if abs(abs(direction)-1.) > 1e-6:
        raise ValueError("direction_y must be +1 or -1")
    origin = state["time_s"]
    times = np.array([_number(x, "sample_times") for x in sample_times])
    if len(times) == 0 or np.any(np.diff(times) <= 0) or times[0] < origin:
        raise ValueError("sample_times must be nonempty, strictly increasing, and not before the anchor")
    if abs(state["theta_rad"]) >= np.pi/2:
        raise ValueError("initial theta must stay inside the planar tangent domain")
    onsets, tilts = [], []
    for command in command_history:
        try:
            timestamp = _number(command["time_s"], "command time_s")
            tilt = _number(command["tilt_rad"], "command tilt_rad")
        except (KeyError, TypeError) as exc:
            raise ValueError("command history requires time_s and tilt_rad") from exc
        if onsets and timestamp <= onsets[-1]:
            raise ValueError("command history times must be strictly increasing")
        if abs(tilt) >= np.pi/2:
            raise ValueError("command tilt must stay inside the planar tangent domain")
        onsets.append(timestamp)
        tilts.append(tilt)
    relative_times = times-origin
    command_times = np.array(onsets)-origin
    end = float(relative_times[-1])
    # Fixed internal integration spacing avoids forecasts changing substantially
    # just because a consumer requests only the 400 ms endpoint. Exact analytic
    # tilt is reused; velocity/position use bounded-step trapezoidal integration.
    if end > 60.:
        raise ValueError("prediction horizon exceeds the 60 s bounded numerical scope")
    grid = np.linspace(0., end, max(2, int(np.ceil(end/.002))+1)) if end else np.array([0.])
    delayed = command_times+fit["delay_s"]
    grid = np.unique(np.r_[grid, relative_times, delayed[(delayed > 0) & (delayed < end)]])
    trial = Trial(0, 0., np.array([0., direction]), grid,
                  np.zeros(len(grid)), np.zeros(len(grid)), np.zeros(len(grid)),
                  command_times, 9.81*np.tan(tilts), {}, 0.)
    angle = np.zeros(len(grid)); angle[0] = state["theta_rad"]
    rate = np.zeros(len(grid)); rate[0] = state["omega_rad_s"]
    theta = tilt_prediction(TiltTrial(trial, angle, rate), fit)
    if not np.isfinite(theta).all() or np.any(np.abs(theta) >= np.pi/2):
        raise ValueError("predicted tilt leaves the planar tangent domain")
    velocity = state["velocity_m_s"]+gain*cumulative_trapezoid(9.81*np.tan(theta), grid, initial=0)
    position = state["position_m"]+cumulative_trapezoid(velocity, grid, initial=0)
    if not np.isfinite(np.r_[position, velocity]).all():
        raise ValueError("prediction produced nonfinite motion")
    indices = np.searchsorted(grid, relative_times)
    return dict(time_s=times.tolist(), x=position[indices].tolist(), v=velocity[indices].tolist(),
                theta=theta[indices].tolist(),
                prediction_scope="attitude_command_only",
                conditional_on_declared_future_commands=bool(any(t > origin for t in onsets)),
                clock_scope="host_receive_effective_delay")


def _predict_item(model, item, anchor):
    trial = item.trial
    return predict_trajectory(
        model,
        dict(time_s=trial.times[anchor], position_m=trial.positions[anchor],
             velocity_m_s=trial.velocities[anchor], theta_rad=item.angle[anchor],
             omega_rad_s=item.rate[anchor], direction_y=trial.direction[1]),
        [dict(time_s=t, tilt_rad=np.arctan(command/9.81))
         for t, command in zip(trial.command_times, trial.commands)],
        trial.times[anchor:],
    )


def evaluate_predictive_model(model, samples, *, max_sample_gap_s=.06):
    """Whole held-out trials, with conditional executed-schedule evaluation.

    No parameters are updated. Full brake-tail and rolling forecasts use only
    the measured state at their anchor; measurements after it are scoring
    targets only. An executed future command schedule is not an actual online
    forecast of a future policy's actions.
    """
    _validate_model(model)
    items = build_tilt_trials(samples, max_sample_gap_s=max_sample_gap_s)
    train_ids = model.get("train_segment_ids")
    if not isinstance(train_ids, list) or not train_ids:
        raise ValueError("model is missing training segment provenance")
    held_ids = [item.trial.segment for item in items]
    if set(held_ids) & set(train_ids):
        raise ValueError("training and validation segment IDs overlap")
    per_trial, rolling = [], []
    for item in items:
        trial = item.trial
        anchor = brake_anchor(item)
        prediction = _predict_item(model, item, anchor)
        t = trial.times[anchor:]-trial.times[anchor]
        pv, pp, theta = (np.array(prediction[key]) for key in ("v", "x", "theta"))
        velocity, position = trial.velocities[anchor:], trial.positions[anchor:]
        tail = t >= t[-1]-.10
        actual_cross = first_crossing(t, velocity, position-position[0])
        predicted_cross = first_crossing(t, pv, pp-pp[0])
        history = _predict_item(model, item, 0)
        per_trial.append(dict(
            segment=trial.segment, segment_id=trial.segment, direction_y=int(trial.direction[1]),
            prediction_anchor_relative_s=float(trial.times[anchor]), duration_s=float(t[-1]),
            theta_rmse_deg=float(np.sqrt(np.mean(np.degrees(theta-item.angle[anchor:])**2))),
            full_trial_theta_rmse_deg=float(np.sqrt(np.mean(np.degrees(np.array(history["theta"])-item.angle)**2))),
            velocity_rmse_m_s=float(np.sqrt(np.mean((pv-velocity)**2))),
            actual_terminal_m_s=float(np.mean(velocity[tail])), predicted_terminal_m_s=float(np.mean(pv[tail])),
            terminal_error_m_s=float(np.mean(pv[tail]-velocity[tail])),
            end_position_error_m=float(pp[-1]-position[-1]),
            actual_reverse=bool(np.min(velocity) < -.03), predicted_reverse=bool(np.min(pv) < -.03),
            actual_peak_reverse_m_s=float(max(0., -np.min(velocity))),
            predicted_peak_reverse_m_s=float(max(0., -np.min(pv))),
            actual_crossing=actual_cross, predicted_crossing=predicted_cross,
            crossing_error_s=(None if actual_cross is None or predicted_cross is None else
                              predicted_cross["time_s"]-actual_cross["time_s"]),
        ))
        for rolling_anchor in range(anchor, len(trial.times)-1, 5):
            horizon_prediction = _predict_item(model, item, rolling_anchor)
            elapsed = trial.times[rolling_anchor:]-trial.times[rolling_anchor]
            for horizon in HORIZONS_S:
                if elapsed[-1]+1e-12 < horizon:
                    continue
                interpolate = lambda values: float(np.interp(horizon, elapsed, values))
                rolling.append(dict(
                    segment=trial.segment, segment_id=trial.segment,
                    anchor_s=float(trial.times[rolling_anchor]), horizon_s=horizon,
                    theta_error_deg=float(np.degrees(interpolate(horizon_prediction["theta"])-
                                                     interpolate(item.angle[rolling_anchor:]))),
                    velocity_error_m_s=(interpolate(horizon_prediction["v"])-
                                        interpolate(trial.velocities[rolling_anchor:])),
                    position_error_m=(interpolate(horizon_prediction["x"])-
                                      interpolate(trial.positions[rolling_anchor:])),
                ))
    aggregates = {"trial_count": len(per_trial), "weighting": "equal trial means"}
    for key in ("theta_rmse_deg", "velocity_rmse_m_s", "terminal_error_m_s", "end_position_error_m"):
        values = np.array([row[key] for row in per_trial])
        aggregates[key] = float(np.mean(values))
        aggregates["mean_"+key] = float(np.mean(values))
        aggregates["max_abs_"+key] = float(np.max(np.abs(values)))
    aggregates.update(
        actual_reverse_trial_count=sum(row["actual_reverse"] for row in per_trial),
        predicted_reverse_trial_count=sum(row["predicted_reverse"] for row in per_trial),
        reverse_classification_mismatch_count=sum(row["actual_reverse"] != row["predicted_reverse"] for row in per_trial),
        zero_crossing_presence_mismatch_count=sum((row["actual_crossing"] is None) !=
                                                 (row["predicted_crossing"] is None) for row in per_trial),
    )
    crossing_errors = [abs(row["crossing_error_s"]) for row in per_trial if row["crossing_error_s"] is not None]
    aggregates["max_abs_crossing_error_s"] = max(crossing_errors) if crossing_errors else None
    horizon_aggregates = []
    for horizon in HORIZONS_S:
        rows = [row for row in rolling if row["horizon_s"] == horizon]
        summary = dict(horizon_s=horizon, forecast_count=len(rows))
        for input_key, output_key in (("theta_error_deg", "theta_rmse_deg"),
                                      ("velocity_error_m_s", "velocity_rmse_m_s"),
                                      ("position_error_m", "position_rmse_m")):
            summary[output_key] = (float(np.sqrt(np.mean([row[input_key]**2 for row in rows])))
                                   if rows else None)
        horizon_aggregates.append(summary)
    return _json_safe(dict(
        schema_version=1, validation_segment_ids=held_ids,
        train_segment_ids=list(train_ids), per_trial=per_trial, aggregates=aggregates,
        rolling_horizons=rolling, rolling_horizon_aggregates=horizon_aggregates,
        evaluation_scope="conditional_on_executed_command_schedule",
        prediction_scope="attitude_command_only", clock_scope="host_receive_effective_delay",
        future_measurements_used_for_prediction=False,
        validation_contains_opposed_pair=set(int(t.trial.direction[1]) for t in items) == {-1, 1},
        model_parameters_updated=False, runtime_enabled=False, deployment_approved=False,
        warning="Executed future commands are explicit evaluation inputs, not an online forecast or flight approval.",
    ))
