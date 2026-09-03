"""Persisted per-axis actuator/velocity alignment calibration."""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile

import numpy as np

from Interaction.position_capture_calibration import position_capture_fit_is_current


DEFAULT_CALIBRATION_PATH = Path(__file__).with_name("wrench_calibration.json")
SCHEMA_VERSION = 1
PLANAR_BRAKING_FIT_SCHEMA_VERSION = 2
DEFAULT_COAST_MAX_ACCELERATION_M_S2 = 5.0
MAX_PLANAR_RESPONSE_DELAY_S = 0.50
MAX_PLANAR_RESPONSE_TIME_CONSTANT_S = 0.50
MIN_PLANAR_ACCELERATION_SCALE = 0.20
MAX_PLANAR_ACCELERATION_SCALE = 2.50
MAX_PLANAR_DIRECTION_GAIN_RATIO = 1.50
MAX_PLANAR_CALIBRATION_TILT_DEG = 30.0
MAX_PLANAR_ACCELERATION_EXTRAPOLATION_RATIO = 1.50
PLANAR_BRAKING_REQUIRED_FIELDS = frozenset({
    "command_delay_s",
    "command_time_constant_s",
    "horizontal_acceleration_scale",
    "fitted_step_acceleration_m_s2",
    "validated_max_acceleration_m_s2",
    "maximum_acceleration_extrapolation_ratio",
    "direction_quality",
    "protocol",
    "maneuver_count",
})


def planar_braking_fit_is_current(braking) -> bool:
    """Return whether a saved braking fit passed the current safety contract."""
    if not (
        isinstance(braking, dict)
        and braking.get("fit_schema_version")
        == PLANAR_BRAKING_FIT_SCHEMA_VERSION
        and braking.get("usable") is True
        and PLANAR_BRAKING_REQUIRED_FIELDS.issubset(braking)
        and isinstance(braking.get("direction_quality"), dict)
        and isinstance(braking.get("protocol"), dict)
    ):
        return False
    try:
        delay_s = float(braking["command_delay_s"])
        time_constant_s = float(braking["command_time_constant_s"])
        acceleration_scale = float(braking["horizontal_acceleration_scale"])
        fitted_step_acceleration = float(
            braking["fitted_step_acceleration_m_s2"]
        )
        acceleration_limit = float(
            braking["validated_max_acceleration_m_s2"]
        )
        extrapolation_ratio = float(
            braking["maximum_acceleration_extrapolation_ratio"]
        )
        gain_ratio = float(braking["direction_quality"]["gain_ratio"])
        maximum_gain_ratio = float(
            braking["direction_quality"]["maximum_direction_gain_ratio"]
        )
        calibration_tilt_deg = float(braking["protocol"]["tilt_deg"])
        tilt_levels_deg = np.asarray(
            braking["protocol"].get(
                "tilt_levels_deg", [calibration_tilt_deg]
            ),
            dtype=float,
        )
        repetitions = int(braking["protocol"]["repetitions"])
        repetitions_per_tilt = int(
            braking["protocol"].get(
                "repetitions_per_tilt",
                repetitions if len(tilt_levels_deg) == 1 else 0,
            )
        )
        calibrated_axis = _normalized_planar_calibration_axis(
            braking["protocol"].get("directions_xy"),
            "saved planar braking calibration",
        )
    except (KeyError, TypeError, ValueError):
        return False
    return bool(
        np.all(np.isfinite([
            delay_s,
            time_constant_s,
            acceleration_scale,
            fitted_step_acceleration,
            acceleration_limit,
            extrapolation_ratio,
            gain_ratio,
            maximum_gain_ratio,
            calibration_tilt_deg,
        ]))
        and 0.0 <= delay_s <= MAX_PLANAR_RESPONSE_DELAY_S
        and 0.0 <= time_constant_s <= MAX_PLANAR_RESPONSE_TIME_CONSTANT_S
        and MIN_PLANAR_ACCELERATION_SCALE
        <= acceleration_scale
        <= MAX_PLANAR_ACCELERATION_SCALE
        and 0.0 < fitted_step_acceleration
        <= DEFAULT_COAST_MAX_ACCELERATION_M_S2
        and 0.0 < acceleration_limit
        <= DEFAULT_COAST_MAX_ACCELERATION_M_S2
        and 1.0 <= extrapolation_ratio
        <= MAX_PLANAR_ACCELERATION_EXTRAPOLATION_RATIO
        and np.isclose(
            acceleration_limit,
            min(
                DEFAULT_COAST_MAX_ACCELERATION_M_S2,
                fitted_step_acceleration * extrapolation_ratio,
            ),
            rtol=0.02,
            atol=1e-3,
        )
        and np.isclose(
            fitted_step_acceleration,
            acceleration_scale * 9.81
            * np.tan(np.radians(calibration_tilt_deg)),
            rtol=0.05,
            atol=0.02,
        )
        and 1.0 <= gain_ratio <= maximum_gain_ratio
        and 1.0 <= maximum_gain_ratio <= MAX_PLANAR_DIRECTION_GAIN_RATIO
        and 0.0 < calibration_tilt_deg
        < MAX_PLANAR_CALIBRATION_TILT_DEG
        and tilt_levels_deg.ndim == 1
        and len(tilt_levels_deg) > 0
        and np.all(np.isfinite(tilt_levels_deg))
        and np.all(tilt_levels_deg > 0.0)
        and np.all(tilt_levels_deg < MAX_PLANAR_CALIBRATION_TILT_DEG)
        and np.all(np.diff(tilt_levels_deg) > 0.0)
        and np.isclose(
            calibration_tilt_deg,
            float(np.max(tilt_levels_deg)),
            rtol=0.0,
            atol=1e-6,
        )
        and repetitions_per_tilt > 0
        and repetitions == len(tilt_levels_deg) * repetitions_per_tilt
        and _planar_direction_quality_is_current(
            braking, calibrated_axis
        )
    )


def _normalized_planar_calibration_axis(directions, label):
    """Return the single calibrated line shared by an opposed direction set."""
    directions = np.asarray(directions, dtype=float)
    if (
        directions.ndim != 2
        or directions.shape[1:] != (2,)
        or len(directions) == 0
        or not np.all(np.isfinite(directions))
    ):
        raise ValueError(f"{label} directions_xy must contain finite XY pairs")
    norms = np.linalg.norm(directions, axis=1)
    if np.any(norms <= 1e-9):
        raise ValueError(f"{label} directions_xy cannot contain zero")
    normalized = directions / norms[:, None]
    axis = normalized[0]
    projections = normalized @ axis
    if (
        np.any(np.abs(projections) < 0.98)
        or not np.any(projections > 0.98)
        or not np.any(projections < -0.98)
    ):
        raise ValueError(
            f"{label} directions_xy must share one opposed planar axis"
        )
    return axis


def _planar_direction_quality_is_current(braking, calibrated_axis):
    """Recheck the signed trial evidence persisted by the v2 fitter."""
    try:
        quality = braking["direction_quality"]
        directions = quality["directions"]
        minimum_trials = int(quality["minimum_trials_per_direction"])
        minimum_windows = int(quality["minimum_windows_per_trial"])
        minimum_train_r_squared = float(
            quality["minimum_direction_r_squared"]
        )
        minimum_validation_r_squared = float(
            quality["minimum_direction_validation_r_squared"]
        )
        maximum_nrmse = float(quality["maximum_direction_nrmse"])
        maximum_gain_ratio = float(
            quality["maximum_direction_gain_ratio"]
        )
        maximum_repeat_deviation = float(
            quality["maximum_repeat_gain_deviation"]
        )
        reported_gain_ratio = float(quality["gain_ratio"])
        reported_axis = np.asarray(quality["axis_xy"], dtype=float)
        maneuver_count = int(braking["maneuver_count"])
        repetitions = int(braking["protocol"]["repetitions"])
        protocol_direction_count = len(braking["protocol"]["directions_xy"])
        protocol_tilt_levels = np.asarray(
            braking["protocol"].get(
                "tilt_levels_deg", [braking["protocol"]["tilt_deg"]]
            ),
            dtype=float,
        )
        repetitions_per_tilt = int(
            braking["protocol"].get(
                "repetitions_per_tilt",
                repetitions if len(protocol_tilt_levels) == 1 else 0,
            )
        )
    except (KeyError, TypeError, ValueError):
        return False
    threshold_values = np.asarray([
        minimum_train_r_squared,
        minimum_validation_r_squared,
        maximum_nrmse,
        maximum_gain_ratio,
        maximum_repeat_deviation,
        reported_gain_ratio,
    ], dtype=float)
    if (
        not isinstance(directions, dict)
        or reported_axis.shape != (2,)
        or not np.all(np.isfinite(reported_axis))
        or not np.all(np.isfinite(threshold_values))
        or np.linalg.norm(reported_axis) <= 1e-9
        or abs(float(
            (reported_axis / np.linalg.norm(reported_axis))
            @ calibrated_axis
        )) < 0.98
        or minimum_trials < 2
        or minimum_windows < 5
        or not 0.0 <= minimum_train_r_squared <= 1.0
        or not 0.0 <= minimum_validation_r_squared <= 1.0
        or not 0.0 < maximum_nrmse <= 0.50
        or not 1.0 <= maximum_gain_ratio <= MAX_PLANAR_DIRECTION_GAIN_RATIO
        or not 0.0 <= maximum_repeat_deviation <= 0.50
        or not 1.0 <= reported_gain_ratio <= maximum_gain_ratio
        or repetitions < minimum_trials
        or maneuver_count != repetitions * protocol_direction_count
    ):
        return False

    signed_gains = []
    all_trial_ids = []
    observed_level_gains = {}
    for label, expected_sign in (("positive", 1.0), ("negative", -1.0)):
        try:
            evidence = directions[label]
            direction = np.asarray(evidence["direction_xy"], dtype=float)
            trial_ids = [int(value) for value in evidence["trial_ids"]]
            trial_count = int(evidence["trial_count"])
            trial_gains = np.asarray(evidence["trial_gains"], dtype=float)
            direction_gain = float(evidence["gain"])
            train_window_count = int(evidence["train_window_count"])
            train_r_squared = float(evidence["train_r_squared"])
            train_nrmse = float(evidence["train_nrmse"])
            validation_window_count = int(
                evidence["validation_window_count"]
            )
            validation_r_squared = float(
                evidence["validation_r_squared"]
            )
            repeat_deviation = float(
                evidence["maximum_repeat_gain_deviation"]
            )
        except (KeyError, TypeError, ValueError):
            return False
        numeric = np.r_[
            direction,
            trial_gains,
            direction_gain,
            train_r_squared,
            train_nrmse,
            validation_r_squared,
            repeat_deviation,
        ]
        calculated_repeat_deviation = float(np.max(np.abs(
            trial_gains / max(abs(direction_gain), 1e-12) - 1.0
        ))) if len(trial_gains) else float("inf")
        if (
            direction.shape != (2,)
            or not np.all(np.isfinite(numeric))
            or np.linalg.norm(direction) <= 1e-9
            or expected_sign * float(
                (direction / np.linalg.norm(direction)) @ calibrated_axis
            ) < 0.98
            or trial_count < minimum_trials
            or len(trial_ids) != trial_count
            or len(set(trial_ids)) != trial_count
            or len(trial_gains) != trial_count
            or train_window_count < minimum_windows * trial_count
            or validation_window_count < minimum_windows * trial_count
            or not MIN_PLANAR_ACCELERATION_SCALE
            <= direction_gain
            <= MAX_PLANAR_ACCELERATION_SCALE
            or np.any(
                (trial_gains < MIN_PLANAR_ACCELERATION_SCALE)
                | (trial_gains > MAX_PLANAR_ACCELERATION_SCALE)
            )
            or train_r_squared < minimum_train_r_squared
            or train_nrmse > maximum_nrmse
            or validation_r_squared < minimum_validation_r_squared
            or not np.isclose(
                repeat_deviation,
                calculated_repeat_deviation,
                rtol=0.02,
                atol=1e-3,
            )
            or calculated_repeat_deviation > maximum_repeat_deviation
        ):
            return False
        signed_gains.append(direction_gain)
        all_trial_ids.extend(trial_ids)
        if len(protocol_tilt_levels) > 1:
            try:
                trial_tilt_levels = np.asarray(
                    evidence["trial_tilt_levels_deg"], dtype=float
                )
            except (KeyError, TypeError, ValueError):
                return False
            expected_tilt_levels = np.tile(
                protocol_tilt_levels, repetitions_per_tilt
            )
            if (
                trial_tilt_levels.shape != (trial_count,)
                or not np.all(np.isfinite(trial_tilt_levels))
                or expected_tilt_levels.shape != (trial_count,)
                or not np.allclose(
                    np.sort(trial_tilt_levels),
                    np.sort(expected_tilt_levels),
                    rtol=0.0,
                    atol=0.05,
                )
            ):
                return False
            observed_level_gains[label] = (
                trial_tilt_levels.copy(), trial_gains.copy()
            )
    if len(set(all_trial_ids)) != maneuver_count:
        return False
    if len(protocol_tilt_levels) > 1:
        try:
            reported_pairs = quality["per_tilt_gain_ratios"]
            reported_maximum_pair_ratio = float(
                quality["maximum_per_tilt_gain_ratio"]
            )
        except (KeyError, TypeError, ValueError):
            return False
        if (
            not isinstance(reported_pairs, list)
            or len(reported_pairs) != len(protocol_tilt_levels)
            or not np.isfinite(reported_maximum_pair_ratio)
        ):
            return False
        calculated_pair_ratios = []
        for expected_tilt, reported in zip(
                protocol_tilt_levels, reported_pairs):
            try:
                reported_tilt = float(reported["tilt_deg"])
                reported_positive_gain = float(reported["positive_gain"])
                reported_negative_gain = float(reported["negative_gain"])
                reported_pair_ratio = float(reported["gain_ratio"])
            except (KeyError, TypeError, ValueError):
                return False
            positive_tilts, positive_gains = observed_level_gains["positive"]
            negative_tilts, negative_gains = observed_level_gains["negative"]
            positive_mask = np.isclose(
                positive_tilts, expected_tilt, rtol=0.0, atol=0.05
            )
            negative_mask = np.isclose(
                negative_tilts, expected_tilt, rtol=0.0, atol=0.05
            )
            if not np.any(positive_mask) or not np.any(negative_mask):
                return False
            calculated_positive_gain = float(np.median(
                positive_gains[positive_mask]
            ))
            calculated_negative_gain = float(np.median(
                negative_gains[negative_mask]
            ))
            calculated_pair_ratio = max(
                calculated_positive_gain, calculated_negative_gain
            ) / min(calculated_positive_gain, calculated_negative_gain)
            values = np.asarray([
                reported_tilt,
                reported_positive_gain,
                reported_negative_gain,
                reported_pair_ratio,
                calculated_pair_ratio,
            ])
            if (
                not np.all(np.isfinite(values))
                or not np.isclose(
                    reported_tilt, expected_tilt, rtol=0.0, atol=0.05
                )
                or not np.isclose(
                    reported_positive_gain,
                    calculated_positive_gain,
                    rtol=0.02,
                    atol=1e-3,
                )
                or not np.isclose(
                    reported_negative_gain,
                    calculated_negative_gain,
                    rtol=0.02,
                    atol=1e-3,
                )
                or not np.isclose(
                    reported_pair_ratio,
                    calculated_pair_ratio,
                    rtol=0.02,
                    atol=1e-3,
                )
                or calculated_pair_ratio > maximum_gain_ratio
            ):
                return False
            calculated_pair_ratios.append(calculated_pair_ratio)
        if not np.isclose(
            reported_maximum_pair_ratio,
            max(calculated_pair_ratios),
            rtol=0.02,
            atol=1e-3,
        ):
            return False
    calculated_gain_ratio = max(signed_gains) / min(signed_gains)
    return bool(
        np.isclose(
            calculated_gain_ratio,
            reported_gain_ratio,
            rtol=0.02,
            atol=1e-3,
        )
        and np.isclose(
            float(np.mean(signed_gains)),
            float(braking["horizontal_acceleration_scale"]),
            rtol=0.10,
            atol=0.02,
        )
    )


def _planar_fit_meets_configured_quality(braking, configured) -> bool:
    """Re-evaluate saved measurements against the current mission gates."""
    configured = dict(configured or {})
    if not configured:
        return True
    try:
        quality = braking["direction_quality"]
        directions = quality["directions"]

        def optional_float(key):
            if key not in configured:
                return None
            value = float(configured[key])
            if not np.isfinite(value):
                raise ValueError(key)
            return value

        def optional_int(key):
            if key not in configured:
                return None
            numeric = float(configured[key])
            value = int(numeric)
            if not np.isfinite(numeric) or numeric != value or value <= 0:
                raise ValueError(key)
            return value

        max_delay = optional_float("max_fit_delay_s")
        max_tau = optional_float("max_fit_time_constant_s")
        minimum_fit_r_squared = optional_float("minimum_fit_r_squared")
        minimum_validation_r_squared = optional_float(
            "minimum_validation_r_squared"
        )
        minimum_scale = optional_float("minimum_acceleration_scale")
        maximum_scale = optional_float("maximum_acceleration_scale")
        minimum_trials = optional_int("minimum_trials_per_direction")
        minimum_windows = optional_int("minimum_windows_per_trial")
        minimum_direction_r_squared = optional_float(
            "minimum_direction_r_squared"
        )
        minimum_direction_validation_r_squared = optional_float(
            "minimum_direction_validation_r_squared"
        )
        maximum_direction_nrmse = optional_float(
            "maximum_direction_nrmse"
        )
        maximum_direction_gain_ratio = optional_float(
            "maximum_direction_gain_ratio"
        )
        maximum_trial_gain_deviation = optional_float(
            "maximum_repeat_gain_deviation"
        )
        extrapolation_ratio = optional_float(
            "maximum_acceleration_extrapolation_ratio"
        )
        delay = float(braking["command_delay_s"])
        tau = float(braking["command_time_constant_s"])
        scale = float(braking["horizontal_acceleration_scale"])
        fit_r_squared = (
            None
            if minimum_fit_r_squared is None
            else float(braking["r_squared"])
        )
        validation_r_squared = (
            None
            if minimum_validation_r_squared is None
            else float(braking["acceleration_validation_r_squared"])
        )
    except (KeyError, TypeError, ValueError):
        return False

    measured_values = [delay, tau, scale]
    if fit_r_squared is not None:
        measured_values.append(fit_r_squared)
    if validation_r_squared is not None:
        measured_values.append(validation_r_squared)
    if not np.all(np.isfinite(measured_values)):
        return False

    if (
        (max_delay is not None and (max_delay < 0.0 or delay > max_delay))
        or (max_tau is not None and (max_tau < 0.0 or tau > max_tau))
        or (
            minimum_fit_r_squared is not None
            and fit_r_squared < minimum_fit_r_squared
        )
        or (
            minimum_validation_r_squared is not None
            and validation_r_squared < minimum_validation_r_squared
        )
        or (minimum_scale is not None and scale < minimum_scale)
        or (maximum_scale is not None and scale > maximum_scale)
        or (
            minimum_scale is not None and maximum_scale is not None
            and maximum_scale <= minimum_scale
        )
        or (
            maximum_direction_gain_ratio is not None
            and (
                maximum_direction_gain_ratio < 1.0
                or float(quality["gain_ratio"])
                > maximum_direction_gain_ratio
                or float(quality.get(
                    "maximum_per_tilt_gain_ratio", quality["gain_ratio"]
                )) > maximum_direction_gain_ratio
            )
        )
        or (
            extrapolation_ratio is not None
            and not 1.0 <= extrapolation_ratio
            <= MAX_PLANAR_ACCELERATION_EXTRAPOLATION_RATIO
        )
    ):
        return False

    for label in ("positive", "negative"):
        try:
            evidence = directions[label]
            trial_count = int(evidence["trial_count"])
            trial_gains = np.asarray(evidence["trial_gains"], dtype=float)
            direction_gain = float(evidence["gain"])
            train_windows = int(evidence["train_window_count"])
            validation_windows = int(evidence["validation_window_count"])
            train_r_squared = float(evidence["train_r_squared"])
            validation_direction_r_squared = float(
                evidence["validation_r_squared"]
            )
            train_nrmse = float(evidence["train_nrmse"])
        except (KeyError, TypeError, ValueError):
            return False
        evidence_values = np.r_[
            trial_gains,
            direction_gain,
            train_r_squared,
            validation_direction_r_squared,
            train_nrmse,
        ]
        if not np.all(np.isfinite(evidence_values)):
            return False
        trial_deviation = float(np.max(np.abs(
            trial_gains / max(abs(direction_gain), 1e-12) - 1.0
        ))) if len(trial_gains) else float("inf")
        if (
            (minimum_trials is not None and trial_count < minimum_trials)
            or (
                minimum_windows is not None
                and (
                    train_windows < minimum_windows * trial_count
                    or validation_windows < minimum_windows * trial_count
                )
            )
            or (
                minimum_direction_r_squared is not None
                and train_r_squared < minimum_direction_r_squared
            )
            or (
                minimum_direction_validation_r_squared is not None
                and validation_direction_r_squared
                < minimum_direction_validation_r_squared
            )
            or (
                maximum_direction_nrmse is not None
                and (
                    maximum_direction_nrmse <= 0.0
                    or train_nrmse > maximum_direction_nrmse
                )
            )
            or (
                minimum_scale is not None
                and (
                    direction_gain < minimum_scale
                    or np.any(trial_gains < minimum_scale)
                )
            )
            or (
                maximum_scale is not None
                and (
                    direction_gain > maximum_scale
                    or np.any(trial_gains > maximum_scale)
                )
            )
            or (
                maximum_trial_gain_deviation is not None
                and (
                    maximum_trial_gain_deviation < 0.0
                    or trial_deviation > maximum_trial_gain_deviation
                )
            )
        ):
            return False
    return True


def _low_pass(values: np.ndarray, times: np.ndarray, tau_s: float) -> np.ndarray:
    if tau_s <= 0.0:
        return values.copy()
    result = np.empty_like(values)
    result[0] = values[0]
    for index in range(1, len(values)):
        dt = max(float(times[index] - times[index - 1]), 0.0)
        alpha = 1.0 - np.exp(-dt / tau_s)
        result[index] = result[index - 1] + alpha * (
            values[index] - result[index - 1]
        )
    return result


def _window_regression(
        times: np.ndarray,
        model_acceleration: np.ndarray,
        velocity: np.ndarray,
        window_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    dt = np.diff(times)
    impulse = np.zeros(len(times))
    impulse[1:] = np.cumsum(model_acceleration[:-1] * dt)
    starts = np.searchsorted(times, times - window_s, side="left")
    indices = np.arange(len(times))
    durations = times - times[starts]
    valid = (starts < indices) & (durations >= 0.8 * window_s)
    return (
        impulse[valid] - impulse[starts[valid]],
        velocity[valid] - velocity[starts[valid]],
        durations[valid],
    )


def identify_axis_alignment(
        timestamps,
        expected_acceleration,
        velocity,
        window_s=0.08,
        max_delay_s=0.15,
        max_time_constant_s=0.20,
        delay_step_s=0.005,
        time_constant_step_s=0.01,
) -> dict:
    """Grid-search delay/tau and fit acceleration gain from momentum changes."""
    times = np.asarray(timestamps, dtype=float)
    model = np.asarray(expected_acceleration, dtype=float)
    measured_velocity = np.asarray(velocity, dtype=float)
    if not (times.ndim == model.ndim == measured_velocity.ndim == 1):
        raise ValueError("axis calibration inputs must be one-dimensional")
    if not (len(times) == len(model) == len(measured_velocity)) or len(times) < 50:
        raise ValueError("axis calibration requires at least 50 synchronized samples")
    finite = np.isfinite(times) & np.isfinite(model) & np.isfinite(measured_velocity)
    times, model, measured_velocity = times[finite], model[finite], measured_velocity[finite]
    order = np.argsort(times)
    times, model, measured_velocity = times[order], model[order], measured_velocity[order]
    unique = np.concatenate(([True], np.diff(times) > 0.0))
    times, model, measured_velocity = times[unique], model[unique], measured_velocity[unique]
    if len(times) < 50 or np.ptp(times) < 2.0 or np.std(model) < 0.03:
        raise ValueError("insufficient contact-free excitation for axis calibration")

    best = None
    delays = np.arange(0.0, max_delay_s + 0.5 * delay_step_s, delay_step_s)
    taus = np.arange(
        0.0,
        max_time_constant_s + 0.5 * time_constant_step_s,
        time_constant_step_s,
    )
    for tau_s in taus:
        filtered = _low_pass(model, times, float(tau_s))
        for delay_s in delays:
            target_times = times - float(delay_s)
            aligned = np.interp(
                target_times,
                times,
                filtered,
                left=float(filtered[0]),
                right=float(filtered[-1]),
            )
            x, y, durations = _window_regression(
                times, aligned, measured_velocity, float(window_s)
            )
            valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(durations)
            x, y, durations = x[valid], y[valid], durations[valid]
            if len(x) < 30 or np.std(x) < 1e-4:
                continue
            design = np.column_stack((x, durations))
            gain, bias = np.linalg.lstsq(design, y, rcond=None)[0]
            gain = float(np.clip(gain, 0.25, 4.0))
            bias = float(np.mean((y - gain * x) / durations))
            predicted = gain * x + bias * durations
            residual = y - predicted
            rmse = float(np.sqrt(np.mean(np.square(residual))))
            variance = float(np.sum(np.square(y - np.mean(y))))
            r_squared = 1.0 - float(np.sum(np.square(residual))) / max(variance, 1e-12)
            score = rmse + 1e-4 * (delay_s + tau_s)
            candidate = (score, delay_s, tau_s, gain, bias, rmse, r_squared, len(x))
            if best is None or candidate[0] < best[0]:
                best = candidate
    if best is None:
        raise ValueError("could not fit axis calibration")
    _, delay_s, tau_s, gain, bias, rmse, r_squared, count = best
    return {
        "model_delay_s": round(float(delay_s), 6),
        "model_time_constant_s": round(float(tau_s), 6),
        "model_acceleration_scale": round(float(gain), 6),
        "acceleration_bias_m_s2": round(float(bias), 6),
        "velocity_delta_rmse_m_s": round(float(rmse), 6),
        "r_squared": round(float(r_squared), 6),
        "window_count": int(count),
    }


def identify_xyz_alignment(samples, window_s=0.08) -> dict:
    times = np.asarray([sample[0] for sample in samples], dtype=float)
    expected = np.asarray([sample[1] for sample in samples], dtype=float)
    velocity = np.asarray([sample[2] for sample in samples], dtype=float)
    if expected.ndim != 2 or expected.shape[1] != 3 or velocity.shape != expected.shape:
        raise ValueError("calibration samples must contain XYZ acceleration and velocity")
    fits = [
        identify_axis_alignment(times, expected[:, axis], velocity[:, axis], window_s)
        for axis in range(3)
    ]
    return {
        "model_delay_s": [fit["model_delay_s"] for fit in fits],
        "model_time_constant_s": [fit["model_time_constant_s"] for fit in fits],
        "model_acceleration_scale": [
            fit["model_acceleration_scale"] for fit in fits
        ],
        "axes": dict(zip(("x", "y", "z"), fits)),
        "sample_count": int(len(times)),
        "duration_s": round(float(times[-1] - times[0]), 6),
    }


def identify_planar_braking_response(
        samples,
        window_s=0.08,
        max_delay_s=0.30,
        max_time_constant_s=0.30,
        delay_step_s=0.005,
        time_constant_step_s=0.01,
        expected_maneuver_count=None,
        minimum_r_squared=0.35,
        minimum_validation_r_squared=0.0,
        minimum_acceleration_scale=0.20,
        maximum_acceleration_scale=2.50,
        minimum_trials_per_direction=2,
        minimum_windows_per_trial=12,
        minimum_direction_r_squared=0.70,
        minimum_direction_validation_r_squared=0.50,
        maximum_direction_nrmse=0.20,
        maximum_direction_gain_ratio=1.25,
        maximum_repeat_gain_deviation=0.20,
        maximum_acceleration_extrapolation_ratio=1.25,
) -> dict:
    """Fit attitude-command delay, lag, and horizontal acceleration scale.

    Each sample is a mapping containing ``segment_id``, ``timestamp``,
    ``phase``, ``direction_xy``, ``command_acceleration_xy``, and
    ``velocity_xy``.  Trials are projected onto their own commanded direction
    before fitting, which permits paired positive/negative maneuvers without
    treating the position-controller recovery interval as an attitude input.
    Only the brake and post-brake-level windows determine the fit; acceleration
    windows are retained as an independent diagnostic.
    """
    if len(samples) < 80:
        raise ValueError("braking response calibration requires at least 80 samples")
    expected_maneuver_count = (
        None
        if expected_maneuver_count is None
        else int(expected_maneuver_count)
    )
    if expected_maneuver_count is not None and expected_maneuver_count <= 0:
        raise ValueError("expected maneuver count must be positive")
    minimum_trials_per_direction = int(minimum_trials_per_direction)
    minimum_windows_per_trial = int(minimum_windows_per_trial)
    quality_parameters = np.asarray([
        minimum_r_squared,
        minimum_validation_r_squared,
        minimum_acceleration_scale,
        maximum_acceleration_scale,
        minimum_direction_r_squared,
        minimum_direction_validation_r_squared,
        maximum_direction_nrmse,
        maximum_direction_gain_ratio,
        maximum_repeat_gain_deviation,
        maximum_acceleration_extrapolation_ratio,
    ], dtype=float)
    if (
        not np.all(np.isfinite(quality_parameters))
        or minimum_trials_per_direction <= 0
        or minimum_windows_per_trial <= 0
        or minimum_acceleration_scale <= 0.0
        or maximum_acceleration_scale <= minimum_acceleration_scale
        or maximum_direction_nrmse <= 0.0
        or maximum_direction_gain_ratio < 1.0
        or maximum_repeat_gain_deviation < 0.0
        or maximum_acceleration_extrapolation_ratio < 1.0
    ):
        raise ValueError("planar braking quality-gate parameters are invalid")
    grouped = {}
    for sample in samples:
        try:
            segment_id = int(sample["segment_id"])
            timestamp = float(sample["timestamp"])
            command_started_at = float(
                sample.get("command_started_at", timestamp)
            )
            phase = str(sample["phase"])
            direction = np.asarray(sample["direction_xy"], dtype=float)
            command = np.asarray(
                sample["command_acceleration_xy"], dtype=float
            )
            velocity = np.asarray(sample["velocity_xy"], dtype=float)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("invalid braking response calibration sample") from exc
        if (
            direction.shape != (2,)
            or command.shape != (2,)
            or velocity.shape != (2,)
            or not np.all(np.isfinite(np.r_[
                timestamp, command_started_at, direction, command, velocity,
            ]))
        ):
            raise ValueError("braking response samples must be finite planar data")
        direction_norm = float(np.linalg.norm(direction))
        if direction_norm <= 1e-9:
            raise ValueError("braking response sample direction cannot be zero")
        direction = direction / direction_norm
        grouped.setdefault(segment_id, []).append((
            timestamp,
            command_started_at,
            float(command @ direction),
            float(velocity @ direction),
            phase,
            direction.copy(),
        ))
    segments = []
    segment_directions = []
    for segment_id, rows in sorted(grouped.items()):
        rows.sort(key=lambda row: row[0])
        times = np.asarray([row[0] for row in rows], dtype=float)
        command_starts = np.asarray([row[1] for row in rows], dtype=float)
        command = np.asarray([row[2] for row in rows], dtype=float)
        velocity = np.asarray([row[3] for row in rows], dtype=float)
        phases = np.asarray([row[4] for row in rows], dtype=object)
        direction = np.asarray(rows[0][5], dtype=float)
        unique = np.concatenate(([True], np.diff(times) > 0.0))
        times, command_starts, command, velocity, phases = (
            times[unique], command_starts[unique], command[unique],
            velocity[unique], phases[unique]
        )
        if len(times) < 20 or np.ptp(times) < 0.5 or np.std(command) < 0.05:
            continue
        event_indices = np.concatenate((
            [True],
            (np.diff(command_starts) != 0.0)
            | (np.diff(command) != 0.0),
        ))
        event_times = command_starts[event_indices]
        event_commands = command[event_indices]
        order = np.argsort(event_times, kind="stable")
        event_times = event_times[order]
        event_commands = event_commands[order]
        event_unique = np.concatenate((
            [True], np.diff(event_times) > 1e-9
        ))
        event_times = event_times[event_unique]
        event_commands = event_commands[event_unique]
        command_grid_times = np.unique(np.concatenate((times, event_times)))
        event_lookup = np.searchsorted(
            event_times, command_grid_times, side="right"
        ) - 1
        event_lookup = np.clip(event_lookup, 0, len(event_commands) - 1)
        command_grid = event_commands[event_lookup]
        segments.append((
            segment_id,
            times,
            command_grid_times,
            command_grid,
            velocity,
            phases,
        ))
        segment_directions.append(direction)
    if not segments:
        raise ValueError("insufficient complete planar braking maneuvers")
    if (
        expected_maneuver_count is not None
        and len(segments) != expected_maneuver_count
    ):
        raise ValueError(
            "planar braking calibration is incomplete: "
            f"expected {expected_maneuver_count} maneuvers, got {len(segments)}"
        )
    direction_matrix = np.asarray(segment_directions, dtype=float)
    calibration_axis = direction_matrix[0]
    direction_projection = direction_matrix @ calibration_axis
    if (
        len(segment_directions) < 2
        or np.any(np.abs(direction_projection) < 0.8)
        or not np.any(direction_projection > 0.8)
        or not np.any(direction_projection < -0.8)
    ):
        raise ValueError(
            "planar braking calibration requires one pair of opposed directions"
        )
    direction_labels = np.where(
        direction_projection >= 0.0, "positive", "negative"
    )
    for direction_label in ("positive", "negative"):
        trial_count = int(np.sum(direction_labels == direction_label))
        if trial_count < minimum_trials_per_direction:
            raise ValueError(
                "planar braking calibration requires at least "
                f"{minimum_trials_per_direction} trials in each direction; "
                f"{direction_label} has {trial_count}"
            )

    def regression_windows(times, aligned_command, velocity, phases, accepted):
        dt = np.diff(times)
        impulse = np.zeros(len(times))
        impulse[1:] = np.cumsum(aligned_command[:-1] * dt)
        starts = np.searchsorted(times, times - float(window_s), side="left")
        indices = np.arange(len(times))
        durations = times - times[starts]
        valid = (
            (starts < indices)
            & (durations >= 0.8 * float(window_s))
            & np.isin(phases, tuple(accepted))
        )
        return (
            impulse[valid] - impulse[starts[valid]],
            velocity[valid] - velocity[starts[valid]],
            durations[valid],
        )

    best = None
    delays = np.arange(
        0.0, float(max_delay_s) + 0.5 * float(delay_step_s),
        float(delay_step_s),
    )
    taus = np.arange(
        0.0,
        float(max_time_constant_s) + 0.5 * float(time_constant_step_s),
        float(time_constant_step_s),
    )
    for tau_s in taus:
        filtered_by_segment = [
            _low_pass(command_grid, command_grid_times, float(tau_s))
            for (
                _, _, command_grid_times, command_grid, _, _
            ) in segments
        ]
        for delay_s in delays:
            pooled_x = []
            pooled_y = []
            pooled_duration = []
            for (
                    (
                        (_, times, command_grid_times, _, velocity, phases),
                        filtered,
                    )
            ) in zip(segments, filtered_by_segment):
                aligned = np.interp(
                    times - float(delay_s),
                    command_grid_times,
                    filtered,
                    left=float(filtered[0]),
                    right=float(filtered[-1]),
                )
                x, y, durations = regression_windows(
                    times,
                    aligned,
                    velocity,
                    phases,
                    {"brake", "level_after_brake"},
                )
                pooled_x.extend(x)
                pooled_y.extend(y)
                pooled_duration.extend(durations)
            x = np.asarray(pooled_x, dtype=float)
            y = np.asarray(pooled_y, dtype=float)
            durations = np.asarray(pooled_duration, dtype=float)
            valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(durations)
            x, y, durations = x[valid], y[valid], durations[valid]
            if len(x) < 40 or np.std(x) < 1e-4:
                continue
            design = np.column_stack((x, durations))
            gain, bias = np.linalg.lstsq(design, y, rcond=None)[0]
            gain = float(gain)
            bias = float(bias)
            if not (
                float(minimum_acceleration_scale)
                <= gain
                <= float(maximum_acceleration_scale)
            ):
                continue
            predicted = gain * x + bias * durations
            residual = y - predicted
            rmse = float(np.sqrt(np.mean(np.square(residual))))
            variance = float(np.sum(np.square(y - np.mean(y))))
            r_squared = 1.0 - float(np.sum(np.square(residual))) / max(
                variance, 1e-12
            )
            score = rmse + 1e-5 * (float(delay_s) + float(tau_s))
            candidate = (
                score, float(delay_s), float(tau_s), gain, bias,
                rmse, r_squared, len(x),
            )
            if best is None or candidate[0] < best[0]:
                best = candidate
    if best is None:
        raise ValueError("could not fit planar braking response")
    _, delay_s, tau_s, gain, bias, rmse, r_squared, count = best

    # Validate the selected braking model on the independent acceleration
    # phases and on each physical direction separately.  A pooled scalar can
    # otherwise look acceptable while one sign is under-braked and the other is
    # over-braked, which is unsafe for zero-crossing prediction.
    validation_y = []
    validation_predicted = []
    commanded_decelerations = []
    direction_windows = {
        "positive": {
            "train_x": [], "train_y": [], "train_duration": [],
            "validation_x": [], "validation_y": [],
            "validation_duration": [], "trial_gains": [], "trial_ids": [],
            "trial_tilt_levels_deg": [],
        },
        "negative": {
            "train_x": [], "train_y": [], "train_duration": [],
            "validation_x": [], "validation_y": [],
            "validation_duration": [], "trial_gains": [], "trial_ids": [],
            "trial_tilt_levels_deg": [],
        },
    }
    for segment_index, (
        segment_id, times, command_grid_times, command_grid, velocity, phases
    ) in enumerate(segments):
        filtered = _low_pass(command_grid, command_grid_times, tau_s)
        aligned = np.interp(
            times - delay_s,
            command_grid_times,
            filtered,
            left=float(filtered[0]),
            right=float(filtered[-1]),
        )
        train_x, train_y, train_durations = regression_windows(
            times, aligned, velocity, phases,
            {"brake", "level_after_brake"},
        )
        if len(train_x) < minimum_windows_per_trial:
            raise ValueError(
                "planar braking calibration has too few fit windows in "
                f"trial {segment_id}: {len(train_x)} < "
                f"{minimum_windows_per_trial}"
            )
        gain_denominator = float(np.sum(np.square(train_x)))
        if gain_denominator <= 1e-12:
            raise ValueError(
                f"planar braking trial {segment_id} has no usable command impulse"
            )
        trial_gain = float(np.sum(
            train_x * (train_y - bias * train_durations)
        ) / gain_denominator)
        trial_direction_label = str(direction_labels[segment_index])
        # The training window is the opposite brake command, while the
        # independent validation window is the acceleration command. Group by
        # the physical world-command sign, not by the trial's initial direction.
        train_direction_label = (
            "negative"
            if trial_direction_label == "positive" else "positive"
        )
        train_windows = direction_windows[train_direction_label]
        train_windows["train_x"].extend(train_x)
        train_windows["train_y"].extend(train_y)
        train_windows["train_duration"].extend(train_durations)
        train_windows["trial_gains"].append(trial_gain)
        train_windows["trial_ids"].append(int(segment_id))
        step_command_acceleration = float(np.max(np.abs(command_grid)))
        train_windows["trial_tilt_levels_deg"].append(float(np.degrees(
            np.arctan2(step_command_acceleration, 9.81)
        )))

        validation_x, segment_validation_y, validation_durations = (
            regression_windows(
            times, aligned, velocity, phases, {"accelerate"}
            )
        )
        validation_windows = direction_windows[trial_direction_label]
        validation_windows["validation_x"].extend(validation_x)
        validation_windows["validation_y"].extend(segment_validation_y)
        validation_windows["validation_duration"].extend(
            validation_durations
        )
        validation_y.extend(segment_validation_y)
        validation_predicted.extend(
            gain * validation_x + bias * validation_durations
        )
        brake_commands = np.abs(command_grid[
            np.abs(command_grid) > 1e-6
        ])
        if len(brake_commands):
            commanded_decelerations.append(
                gain * float(np.median(brake_commands))
            )
    validation_y = np.asarray(validation_y, dtype=float)
    validation_predicted = np.asarray(validation_predicted, dtype=float)
    if len(validation_y) > 1:
        validation_residual = validation_y - validation_predicted
        validation_variance = float(np.sum(np.square(
            validation_y - np.mean(validation_y)
        )))
        validation_r_squared = 1.0 - float(np.sum(np.square(
            validation_residual
        ))) / max(validation_variance, 1e-12)
        validation_rmse = float(np.sqrt(np.mean(np.square(
            validation_residual
        ))))
    else:
        validation_r_squared = None
        validation_rmse = None

    direction_quality = {
        "axis_xy": [round(float(value), 6) for value in calibration_axis],
        "directions": {},
        "minimum_trials_per_direction": minimum_trials_per_direction,
        "minimum_windows_per_trial": minimum_windows_per_trial,
        "minimum_direction_r_squared": float(minimum_direction_r_squared),
        "minimum_direction_validation_r_squared": float(
            minimum_direction_validation_r_squared
        ),
        "maximum_direction_nrmse": float(maximum_direction_nrmse),
        "maximum_direction_gain_ratio": float(maximum_direction_gain_ratio),
        "maximum_repeat_gain_deviation": float(
            maximum_repeat_gain_deviation
        ),
    }
    direction_gains = []
    quality_failures = []
    for direction_label in ("positive", "negative"):
        windows = direction_windows[direction_label]
        train_x = np.asarray(windows["train_x"], dtype=float)
        train_y = np.asarray(windows["train_y"], dtype=float)
        train_durations = np.asarray(
            windows["train_duration"], dtype=float
        )
        validation_x = np.asarray(windows["validation_x"], dtype=float)
        segment_validation_y = np.asarray(
            windows["validation_y"], dtype=float
        )
        validation_durations = np.asarray(
            windows["validation_duration"], dtype=float
        )
        direction_gain_denominator = float(np.sum(np.square(train_x)))
        direction_gain = float(np.sum(
            train_x * (train_y - bias * train_durations)
        ) / max(direction_gain_denominator, 1e-12))
        direction_gains.append(direction_gain)
        train_prediction = gain * train_x + bias * train_durations
        train_residual = train_y - train_prediction
        train_rmse = float(np.sqrt(np.mean(np.square(train_residual))))
        train_variance = float(np.sum(np.square(
            train_y - np.mean(train_y)
        )))
        train_r_squared = 1.0 - float(np.sum(np.square(
            train_residual
        ))) / max(train_variance, 1e-12)
        train_nrmse = train_rmse / max(
            float(np.sqrt(np.mean(np.square(train_y)))), 0.02
        )
        validation_prediction = (
            gain * validation_x + bias * validation_durations
        )
        if len(segment_validation_y) > 1:
            direction_validation_residual = (
                segment_validation_y - validation_prediction
            )
            direction_validation_variance = float(np.sum(np.square(
                segment_validation_y - np.mean(segment_validation_y)
            )))
            direction_validation_r_squared = 1.0 - float(np.sum(np.square(
                direction_validation_residual
            ))) / max(direction_validation_variance, 1e-12)
            direction_validation_rmse = float(np.sqrt(np.mean(np.square(
                direction_validation_residual
            ))))
        else:
            direction_validation_r_squared = None
            direction_validation_rmse = None
        trial_gains = np.asarray(windows["trial_gains"], dtype=float)
        repeat_gain_deviation = float(np.max(np.abs(
            trial_gains / max(abs(direction_gain), 1e-12) - 1.0
        )))
        physical_direction = (
            calibration_axis
            if direction_label == "positive" else -calibration_axis
        )
        direction_quality["directions"][direction_label] = {
            "direction_xy": [
                round(float(value), 6) for value in physical_direction
            ],
            "trial_ids": windows["trial_ids"],
            "trial_count": len(windows["trial_ids"]),
            "trial_gains": [round(float(value), 6) for value in trial_gains],
            "trial_tilt_levels_deg": [
                round(float(value), 6)
                for value in windows["trial_tilt_levels_deg"]
            ],
            "gain": round(direction_gain, 6),
            "train_window_count": int(len(train_x)),
            "train_r_squared": round(train_r_squared, 6),
            "train_nrmse": round(train_nrmse, 6),
            "validation_window_count": int(len(segment_validation_y)),
            "validation_r_squared": (
                None
                if direction_validation_r_squared is None
                else round(direction_validation_r_squared, 6)
            ),
            "validation_rmse_m_s": (
                None
                if direction_validation_rmse is None
                else round(direction_validation_rmse, 6)
            ),
            "maximum_repeat_gain_deviation": round(
                repeat_gain_deviation, 6
            ),
        }
        if not (
            minimum_acceleration_scale
            <= direction_gain
            <= maximum_acceleration_scale
        ):
            quality_failures.append(
                f"{direction_label} gain {direction_gain:.3f} is out of range"
            )
        if np.any(
            (trial_gains < minimum_acceleration_scale)
            | (trial_gains > maximum_acceleration_scale)
        ):
            quality_failures.append(
                f"{direction_label} has an out-of-range trial gain"
            )
        if train_r_squared < minimum_direction_r_squared:
            quality_failures.append(
                f"{direction_label} fit R^2 {train_r_squared:.3f} is too low"
            )
        if train_nrmse > maximum_direction_nrmse:
            quality_failures.append(
                f"{direction_label} NRMSE {train_nrmse:.3f} is too high"
            )
        if (
            direction_validation_r_squared is None
            or direction_validation_r_squared
            < minimum_direction_validation_r_squared
        ):
            quality_failures.append(
                f"{direction_label} validation R^2 is too low"
            )
        if repeat_gain_deviation > maximum_repeat_gain_deviation:
            quality_failures.append(
                f"{direction_label} repeat gain deviation "
                f"{repeat_gain_deviation:.3f} is too high"
            )

    direction_gain_ratio = max(direction_gains) / max(
        min(direction_gains), 1e-12
    )
    direction_quality["gain_ratio"] = round(direction_gain_ratio, 6)
    if direction_gain_ratio > maximum_direction_gain_ratio:
        quality_failures.append(
            f"opposed-direction gain ratio {direction_gain_ratio:.3f} is too high"
        )
    # A pooled +Y/-Y ratio can hide crossed amplitude asymmetry (for example,
    # weak +Y only at 8 deg and weak -Y only at 20 deg).  Pair the fitted trial
    # gains by commanded tilt and require symmetry at every tested level.
    paired_tilt_gains = []
    positive_evidence = direction_quality["directions"]["positive"]
    negative_evidence = direction_quality["directions"]["negative"]
    positive_tilts = np.asarray(
        positive_evidence["trial_tilt_levels_deg"], dtype=float
    )
    negative_tilts = np.asarray(
        negative_evidence["trial_tilt_levels_deg"], dtype=float
    )
    positive_trial_gains = np.asarray(
        positive_evidence["trial_gains"], dtype=float
    )
    negative_trial_gains = np.asarray(
        negative_evidence["trial_gains"], dtype=float
    )
    all_tilt_levels = np.unique(np.round(np.r_[
        positive_tilts, negative_tilts,
    ], decimals=4))
    for tilt_deg in all_tilt_levels:
        positive_mask = np.isclose(
            positive_tilts, tilt_deg, rtol=0.0, atol=0.05
        )
        negative_mask = np.isclose(
            negative_tilts, tilt_deg, rtol=0.0, atol=0.05
        )
        if not np.any(positive_mask) or not np.any(negative_mask):
            quality_failures.append(
                f"tilt {tilt_deg:.2f} deg is missing one command direction"
            )
            continue
        positive_gain = float(np.median(
            positive_trial_gains[positive_mask]
        ))
        negative_gain = float(np.median(
            negative_trial_gains[negative_mask]
        ))
        paired_ratio = max(positive_gain, negative_gain) / max(
            min(positive_gain, negative_gain), 1e-12
        )
        paired_tilt_gains.append({
            "tilt_deg": round(float(tilt_deg), 6),
            "positive_gain": round(positive_gain, 6),
            "negative_gain": round(negative_gain, 6),
            "gain_ratio": round(paired_ratio, 6),
        })
        if paired_ratio > maximum_direction_gain_ratio:
            quality_failures.append(
                f"opposed-direction gain ratio at {tilt_deg:.2f} deg "
                f"is {paired_ratio:.3f}"
            )
    direction_quality["per_tilt_gain_ratios"] = paired_tilt_gains
    direction_quality["maximum_per_tilt_gain_ratio"] = round(
        max(
            (entry["gain_ratio"] for entry in paired_tilt_gains),
            default=float("inf"),
        ),
        6,
    )
    if r_squared < float(minimum_r_squared):
        quality_failures.append(f"pooled fit R^2 {r_squared:.3f} is too low")
    if (
        validation_r_squared is None
        or validation_r_squared < float(minimum_validation_r_squared)
    ):
        quality_failures.append("pooled validation R^2 is too low")
    usable = not quality_failures
    # The runtime envelope is anchored to the largest step actually exercised,
    # not the median step.  This is equivalent for the former single-amplitude
    # protocol and lets a quality-gated multi-amplitude sweep validate stronger
    # braking without extrapolating from its middle level.
    fitted_deceleration = (
        float(np.max(commanded_decelerations))
        if commanded_decelerations else 0.0
    )
    validated_max_acceleration = (
        min(
            DEFAULT_COAST_MAX_ACCELERATION_M_S2,
            fitted_deceleration
            * float(maximum_acceleration_extrapolation_ratio),
        )
    )
    result = {
        "fit_schema_version": PLANAR_BRAKING_FIT_SCHEMA_VERSION,
        "usable": usable,
        "command_delay_s": round(delay_s, 6),
        "command_time_constant_s": round(tau_s, 6),
        "horizontal_acceleration_scale": round(gain, 6),
        "acceleration_bias_m_s2": round(bias, 6),
        "effective_response_time_s": round(delay_s + tau_s, 6),
        "fitted_step_acceleration_m_s2": round(fitted_deceleration, 6),
        "validated_max_acceleration_m_s2": round(
            validated_max_acceleration, 6
        ),
        "maximum_acceleration_extrapolation_ratio": float(
            maximum_acceleration_extrapolation_ratio
        ),
        "velocity_delta_rmse_m_s": round(rmse, 6),
        "r_squared": round(r_squared, 6),
        "acceleration_validation_rmse_m_s": (
            None if validation_rmse is None else round(validation_rmse, 6)
        ),
        "acceleration_validation_r_squared": (
            None
            if validation_r_squared is None
            else round(validation_r_squared, 6)
        ),
        "window_count": int(count),
        "sample_count": int(sum(len(segment[1]) for segment in segments)),
        "maneuver_count": int(len(segments)),
        "minimum_required_r_squared": float(minimum_r_squared),
        "minimum_required_validation_r_squared": float(
            minimum_validation_r_squared
        ),
        "direction_quality": direction_quality,
    }
    if not usable:
        raise ValueError(
            "planar braking response fit failed quality gates: "
            + "; ".join(quality_failures)
        )
    return result


def load_drone_calibration(drone_id, path=DEFAULT_CALIBRATION_PATH):
    path = Path(path)
    if not path.exists():
        return None
    with path.open() as stream:
        document = json.load(stream)
    if document.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"unsupported wrench calibration schema in {path}")
    return document.get("drones", {}).get(str(drone_id))


def apply_drone_calibration(
        config,
        drone_id,
        path=DEFAULT_CALIBRATION_PATH,
        runtime_interaction_axis=None,
        runtime_interaction_direction_xy=None,
):
    resolved = deepcopy(config)
    calibration = load_drone_calibration(drone_id, path)
    if calibration is None:
        return resolved, None
    fitted = calibration["impulse_estimator"]
    impulse = resolved.setdefault("impulse_estimator", {})
    for key in (
        "model_delay_s", "model_time_constant_s", "model_acceleration_scale"
    ):
        impulse[key] = deepcopy(fitted[key])
    braking = calibration.get("planar_braking_fit")
    if planar_braking_fit_is_current(braking):
        configured_planar_calibration = resolved.get(
            "planar_braking_calibration", {}
        )
        if not _planar_fit_meets_configured_quality(
                braking, configured_planar_calibration):
            raise ValueError(
                "saved planar braking fit does not satisfy current mission "
                "quality gates; rerun --calibrate"
            )
        configured_directions = np.asarray(
            configured_planar_calibration.get(
                "directions_xy", [[0.0, 1.0], [0.0, -1.0]]
            ),
            dtype=float,
        )
        try:
            calibrated_axis = _normalized_planar_calibration_axis(
                braking.get("protocol", {}).get("directions_xy"),
                "saved planar braking calibration",
            )
            configured_axis = _normalized_planar_calibration_axis(
                configured_directions,
                "mission planar braking calibration",
            )
        except ValueError as exc:
            raise ValueError(f"{exc}; rerun --calibrate") from exc
        if abs(float(calibrated_axis @ configured_axis)) < 0.98:
            raise ValueError(
                "saved planar braking direction does not match this mission; "
                "rerun --calibrate"
            )
        if "tilt_levels_deg" in configured_planar_calibration:
            configured_tilt_levels = np.asarray(
                configured_planar_calibration["tilt_levels_deg"],
                dtype=float,
            )
            saved_tilt_levels = np.asarray(
                braking["protocol"].get(
                    "tilt_levels_deg", [braking["protocol"]["tilt_deg"]]
                ),
                dtype=float,
            )
            configured_repetitions_per_tilt = int(
                configured_planar_calibration.get(
                    "repetitions_per_tilt", 1
                )
            )
            saved_repetitions_per_tilt = int(
                braking["protocol"].get(
                    "repetitions_per_tilt",
                    braking["protocol"]["repetitions"]
                    if len(saved_tilt_levels) == 1 else 0,
                )
            )
            if (
                configured_tilt_levels.shape != saved_tilt_levels.shape
                or not np.allclose(
                    configured_tilt_levels,
                    saved_tilt_levels,
                    rtol=0.0,
                    atol=0.05,
                )
                or configured_repetitions_per_tilt
                != saved_repetitions_per_tilt
            ):
                raise ValueError(
                    "saved planar braking tilt sweep does not match this "
                    "mission; rerun --calibrate"
                )
        runtime_axis = (
            None
            if runtime_interaction_axis is None
            else str(runtime_interaction_axis).strip().lower()
        )
        if runtime_axis is not None and runtime_axis not in ("x", "y"):
            raise ValueError(
                "planar braking calibration supports only x/y interaction axes"
            )
        runtime_axis_vector = {
            "x": np.array([1.0, 0.0]),
            "y": np.array([0.0, 1.0]),
        }.get(runtime_axis)
        if runtime_interaction_direction_xy is not None:
            runtime_direction = np.asarray(
                runtime_interaction_direction_xy, dtype=float
            )
            if (
                runtime_direction.shape != (2,)
                or not np.all(np.isfinite(runtime_direction))
                or np.linalg.norm(runtime_direction) <= 1e-9
            ):
                raise ValueError(
                    "runtime interaction direction must be finite, nonzero XY"
                )
            runtime_direction = runtime_direction / np.linalg.norm(
                runtime_direction
            )
            if (
                runtime_axis_vector is not None
                and abs(float(runtime_axis_vector @ runtime_direction)) < 0.98
            ):
                raise ValueError(
                    "runtime interaction axis and direction disagree"
                )
            runtime_axis_vector = runtime_direction
        if (
            runtime_axis_vector is not None
            and abs(float(calibrated_axis @ runtime_axis_vector)) < 0.98
        ):
            raise ValueError(
                "saved planar braking direction does not cover runtime "
                f"{runtime_axis!r}; rerun --calibrate"
            )
        handoff = resolved.setdefault("control_handoff", {})
        handoff["coast_attitude_response_delay_s"] = float(
            braking["command_delay_s"]
        )
        handoff["coast_attitude_time_constant_s"] = float(
            braking["command_time_constant_s"]
        )
        handoff["coast_attitude_acceleration_scale"] = float(
            braking["horizontal_acceleration_scale"]
        )
        handoff["coast_calibrated_direction_xy"] = [
            float(value) for value in calibrated_axis
        ]
        validated_max_acceleration = braking.get(
            "validated_max_acceleration_m_s2"
        )
        if validated_max_acceleration is not None:
            configured_max_acceleration = float(handoff.get(
                "coast_max_acceleration_m_s2",
                DEFAULT_COAST_MAX_ACCELERATION_M_S2,
            ))
            configured_extrapolation_ratio = float(
                configured_planar_calibration.get(
                    "maximum_acceleration_extrapolation_ratio",
                    braking["maximum_acceleration_extrapolation_ratio"],
                )
            )
            configured_fit_limit = round(
                min(
                    DEFAULT_COAST_MAX_ACCELERATION_M_S2,
                    float(braking["fitted_step_acceleration_m_s2"])
                    * configured_extrapolation_ratio,
                ),
                6,
            )
            handoff["coast_max_acceleration_m_s2"] = min(
                configured_max_acceleration,
                float(validated_max_acceleration),
                configured_fit_limit,
            )
    return resolved, calibration


def save_drone_calibration(
        drone_id,
        fit,
        motor_model,
        path=DEFAULT_CALIBRATION_PATH,
        planar_braking_fit=None,
        position_capture_fit=None,
):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    document = {"schema_version": SCHEMA_VERSION, "drones": {}}
    if path.exists():
        with path.open() as stream:
            existing = json.load(stream)
        if (
            not isinstance(existing, dict)
            or existing.get("schema_version") != SCHEMA_VERSION
            or not isinstance(existing.get("drones"), dict)
        ):
            raise ValueError(
                f"unsupported wrench calibration schema in {path}"
            )
        document = existing
    previous_entry = document.get("drones", {}).get(str(drone_id), {})
    if not isinstance(previous_entry, dict):
        raise ValueError(f"invalid drone calibration entry in {path}")
    entry = deepcopy(previous_entry)
    entry.update({
        "updated_at": datetime.now(timezone.utc).isoformat(),
        "impulse_estimator": {
            "model_delay_s": fit["model_delay_s"],
            "model_time_constant_s": fit["model_time_constant_s"],
            "model_acceleration_scale": fit["model_acceleration_scale"],
        },
        "motor_model": deepcopy(motor_model),
        "fit": deepcopy(fit),
    })
    if planar_braking_fit is not None:
        if not planar_braking_fit_is_current(planar_braking_fit):
            raise ValueError(
                "refusing to save a planar braking fit that does not satisfy "
                "the current safety contract"
            )
        previous_handoff = previous_entry.get("control_handoff", {})
        if not isinstance(previous_handoff, dict):
            previous_handoff = {}
        entry["control_handoff"] = deepcopy(previous_handoff)
        entry["control_handoff"].update({
            "coast_attitude_response_delay_s": float(
                planar_braking_fit["command_delay_s"]
            ),
            "coast_attitude_time_constant_s": float(
                planar_braking_fit["command_time_constant_s"]
            ),
            "coast_attitude_acceleration_scale": float(
                planar_braking_fit["horizontal_acceleration_scale"]
            ),
        })
        entry["control_handoff"]["coast_calibrated_direction_xy"] = [
            float(value)
            for value in _normalized_planar_calibration_axis(
                planar_braking_fit.get("protocol", {}).get(
                    "directions_xy"
                ),
                "saved planar braking calibration",
            )
        ]
        entry["control_handoff"]["coast_max_acceleration_m_s2"] = float(
            planar_braking_fit["validated_max_acceleration_m_s2"]
        )
        entry["planar_braking_fit"] = deepcopy(planar_braking_fit)
    elif "planar_braking_fit" in previous_entry:
        # An impulse-only caller must not silently erase a braking calibration
        # produced by a newer flight.
        entry["control_handoff"] = deepcopy(
            previous_entry.get("control_handoff", {})
        )
        entry["planar_braking_fit"] = deepcopy(
            previous_entry["planar_braking_fit"]
        )
    if position_capture_fit is not None:
        if not position_capture_fit_is_current(position_capture_fit):
            raise ValueError(
                "refusing to save unusable position capture calibration; "
                "the previous calibration has not been replaced"
            )
        # This is independent position-command evidence, not another estimate
        # of the open-loop attitude gain. Do not map it to aMax or silently
        # activate an unvalidated continuous runtime capture envelope.
        entry["position_capture_fit"] = deepcopy(position_capture_fit)
    # entry starts as a copy, so older/partial callers retain this evidence.
    document.setdefault("drones", {})[str(drone_id)] = entry
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w") as stream:
            json.dump(document, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary_name, path)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
    return path, entry
