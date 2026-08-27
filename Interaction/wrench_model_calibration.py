"""Persisted per-axis actuator/velocity alignment calibration."""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile

import numpy as np


DEFAULT_CALIBRATION_PATH = Path(__file__).with_name("wrench_calibration.json")
SCHEMA_VERSION = 1


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


def load_drone_calibration(drone_id, path=DEFAULT_CALIBRATION_PATH):
    path = Path(path)
    if not path.exists():
        return None
    with path.open() as stream:
        document = json.load(stream)
    if document.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"unsupported wrench calibration schema in {path}")
    return document.get("drones", {}).get(str(drone_id))


def apply_drone_calibration(config, drone_id, path=DEFAULT_CALIBRATION_PATH):
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
    return resolved, calibration


def save_drone_calibration(
        drone_id,
        fit,
        motor_model,
        path=DEFAULT_CALIBRATION_PATH,
):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    document = {"schema_version": SCHEMA_VERSION, "drones": {}}
    if path.exists():
        with path.open() as stream:
            existing = json.load(stream)
        if existing.get("schema_version") == SCHEMA_VERSION:
            document = existing
    entry = {
        "updated_at": datetime.now(timezone.utc).isoformat(),
        "impulse_estimator": {
            "model_delay_s": fit["model_delay_s"],
            "model_time_constant_s": fit["model_time_constant_s"],
            "model_acceleration_scale": fit["model_acceleration_scale"],
        },
        "motor_model": deepcopy(motor_model),
        "fit": deepcopy(fit),
    }
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
