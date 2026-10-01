"""Fit the inner roll/pitch response from the ordinary XYZ calibration log.

The calibration motion remains one continuous position-reference excitation.
Firmware ``controller.roll/pitch`` are the inputs. The measurement source is
explicit: ordinary firmware roll/pitch by default, or the isolated 15-state
quaternion when requested and logged. No targeted braking pulse is required.
"""

from __future__ import annotations

import math

import numpy as np

from Interaction.model_based_braking import _second_order_transition


ATTITUDE_RESPONSE_SCHEMA_VERSION = 1
ATTITUDE_ACCELERATION_SCHEMA_VERSION = 1


def _sample_clock(rows):
    """Return a monotonic sample clock, preferring the firmware timestamp."""
    if all(row.get("cf_timestamp_ms") is not None for row in rows):
        raw = np.asarray(
            [float(row["cf_timestamp_ms"]) for row in rows], dtype=float
        )
        # CRTP log timestamps wrap at 24 bits, not uint32.
        unwrapped = raw.copy()
        offset = 0.0
        for index in range(1, len(unwrapped)):
            if raw[index] + offset < unwrapped[index-1] - 2**23:
                offset += 2**24
            unwrapped[index] = raw[index] + offset
        return 0.001 * unwrapped, "firmware_timestamp_ms"
    return (
        np.asarray([row["time"] for row in rows], dtype=float),
        "host_receive_time_fallback",
    )


def _quaternion_wxyz_to_roll_pitch_deg(quaternion):
    w, x, y, z = (float(value) for value in quaternion)
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if not math.isfinite(norm) or norm <= 1e-9:
        raise ValueError("15-state quaternion is invalid")
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    roll = math.atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0*(w*y - z*x))))
    return np.degrees([roll, pitch])


def _quaternion_wxyz_to_rpy_rad(quaternion):
    w, x, y, z = (float(value) for value in quaternion)
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if not math.isfinite(norm) or norm <= 1e-9:
        raise ValueError("15-state quaternion is invalid")
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    return np.asarray([
        math.atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y)),
        math.asin(max(-1.0, min(1.0, 2.0*(w*y - z*x)))),
        math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z)),
    ])


def identify_attitude_acceleration_axis(
        timestamps_s, velocity_m_s, nominal_acceleration_m_s2, *,
        sample_period_s=0.01):
    """Fit measured acceleration = gain * gravity/tilt prediction + bias."""
    from scipy.signal import savgol_filter

    times = np.asarray(timestamps_s, dtype=float)
    velocity = np.asarray(velocity_m_s, dtype=float)
    nominal = np.asarray(nominal_acceleration_m_s2, dtype=float)
    if not (
        times.ndim == velocity.ndim == nominal.ndim == 1
        and len(times) == len(velocity) == len(nominal)
        and len(times) >= 100
        and np.all(np.isfinite(times))
        and np.all(np.isfinite(velocity))
        and np.all(np.isfinite(nominal))
        and np.all(np.diff(times) > 0.0)
    ):
        raise ValueError("attitude acceleration fit needs ordered finite data")
    period = float(sample_period_s)
    if not math.isfinite(period) or period <= 0.0:
        raise ValueError("attitude acceleration sample period must be positive")
    window = min(21, len(velocity) if len(velocity) % 2 else len(velocity)-1)
    if window < 7:
        raise ValueError("attitude acceleration fit window is too short")
    measured = savgol_filter(
        velocity, window, 3, deriv=1, delta=period, mode="interp"
    )
    selected = np.abs(nominal) >= 0.10
    if np.count_nonzero(selected) < 100:
        raise ValueError("attitude acceleration excitation is too small")
    design = np.column_stack([
        nominal[selected], np.ones(np.count_nonzero(selected))
    ])
    gain, bias = np.linalg.lstsq(design, measured[selected], rcond=None)[0]
    prediction = gain*nominal[selected]+bias
    residual = measured[selected]-prediction
    variance = float(np.sum(np.square(
        measured[selected]-np.mean(measured[selected])
    )))
    rmse = float(np.sqrt(np.mean(np.square(residual))))
    r_squared = 1.0-float(np.sum(np.square(residual)))/max(variance, 1e-12)
    usable = bool(
        0.70 <= gain <= 1.30 and r_squared >= 0.90 and rmse <= 0.60
    )
    return {
        "gain": round(float(gain), 6),
        "bias_m_s2": round(float(bias), 6),
        "rmse_m_s2": round(rmse, 6),
        "r_squared": round(r_squared, 6),
        "sample_count": int(np.count_nonzero(selected)),
        "usable": usable,
    }


def identify_second_order_axis(
        timestamps_s, command_deg, measured_deg, *, sample_period_s=0.01):
    """Identify delayed unit-input second-order dynamics on a uniform grid."""
    from scipy.optimize import least_squares

    times = np.asarray(timestamps_s, dtype=float)
    command = np.asarray(command_deg, dtype=float)
    measured = np.asarray(measured_deg, dtype=float)
    if not (
        times.ndim == command.ndim == measured.ndim == 1
        and len(times) == len(command) == len(measured)
        and len(times) >= 100
        and np.all(np.isfinite(times))
        and np.all(np.isfinite(command))
        and np.all(np.isfinite(measured))
        and np.all(np.diff(times) > 0.0)
        and np.ptp(times) >= 4.0
        and np.std(command) >= 1.0
    ):
        raise ValueError("attitude response needs finite ordered excitation data")
    period = float(sample_period_s)
    if not math.isfinite(period) or period <= 0.0:
        raise ValueError("attitude response sample period must be positive")
    # Ignore startup/terminal transients that are outside the chirp itself.
    grid = np.arange(times[0] + 0.5, times[-1] - 0.5, period)
    if len(grid) < 100:
        raise ValueError("attitude response excitation window is too short")
    u = np.interp(grid, times, command)
    y = np.interp(grid, times, measured)

    def simulate(parameters):
        delay_s, wn_rad_s, zeta, gain, bias_deg = parameters
        delayed_u = np.interp(
            grid - delay_s, grid, u, left=float(u[0]), right=float(u[-1])
        )
        transition = _second_order_transition(wn_rad_s, zeta, period)
        angle = float(y[0])
        rate = float(np.gradient(y[:min(len(y), 9)], period)[0])
        prediction = np.empty_like(y)
        for index, input_deg in enumerate(delayed_u):
            prediction[index] = angle
            target = gain*input_deg + bias_deg
            error, rate = transition @ np.array([angle-target, rate])
            angle = target + error
        return prediction

    result = least_squares(
        lambda parameters: simulate(parameters) - y,
        x0=np.array([0.04, 12.0, 0.6, 1.0, 0.0]),
        bounds=(
            np.array([0.0, 5.0, 0.20, 0.70, -3.0]),
            np.array([0.15, 60.0, 2.00, 1.30, 3.0]),
        ),
        max_nfev=400,
    )
    prediction = simulate(result.x)
    residual = prediction-y
    variance = float(np.sum(np.square(y-np.mean(y))))
    rmse_deg = float(np.sqrt(np.mean(np.square(residual))))
    r_squared = 1.0-float(np.sum(np.square(residual)))/max(variance, 1e-12)
    normalized_rmse = rmse_deg/max(float(np.std(y)), 1e-12)
    delay_s, wn_rad_s, zeta, gain, bias_deg = map(float, result.x)
    usable = bool(
        result.success
        and r_squared >= 0.95
        and normalized_rmse <= 0.15
        and rmse_deg <= 3.0
        and 0.70 < gain < 1.30
        and 5.0 < wn_rad_s < 60.0
        and 0.20 < zeta < 2.0
        and 0.0 <= delay_s < 0.15
    )
    return {
        "model": "delayed_second_order",
        "delay_s": round(delay_s, 6),
        "wn_rad_s": round(wn_rad_s, 6),
        "zeta": round(zeta, 6),
        "gain": round(gain, 6),
        "bias_deg": round(bias_deg, 6),
        "rmse_deg": round(rmse_deg, 6),
        "normalized_rmse": round(normalized_rmse, 6),
        "r_squared": round(r_squared, 6),
        "sample_count": int(len(grid)),
        "usable": usable,
    }


def identify_attitude_response_from_log_records(records, *, attitude_source="ordinary"):
    """Fit roll/pitch using only the plain calibration excitation interval."""
    if not isinstance(records, list):
        raise ValueError("calibration log must contain a list of records")
    events = {
        record.get("name"): record.get("data", {}).get("time")
        for record in records if record.get("type") == "events"
    }
    try:
        start = float(events["Wrench Calibration Excitation Started"])
        end = float(events["Wrench Calibration Excitation Complete"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("plain XYZ calibration excitation is incomplete") from error
    if attitude_source not in ("ordinary", "post_release15"):
        raise ValueError("unknown attitude response measurement source")
    desired_group = ("ATT_DES" if any(r.get("group") == "ATT_DES" for r in records)
                     else "ATT_RATE_CTL")
    actual_group = "P_REL_ATT" if attitude_source == "post_release15" else "VEL_ORI"
    desired_rows = [
        record["data"] for record in records
        if record.get("type") == "state"
        and record.get("group") == desired_group
        and start <= float(record.get("data", {}).get("time", -math.inf)) <= end
    ]
    attitude_rows = [
        record["data"] for record in records
        if record.get("type") == "state"
        and record.get("group") == actual_group
        and start <= float(record.get("data", {}).get("time", -math.inf)) <= end
    ]
    if len(desired_rows) < 100 or len(attitude_rows) < 100:
        raise ValueError("plain calibration is missing command/" + actual_group + " attitude")

    desired_time, desired_clock = _sample_clock(desired_rows)
    actual_time, actual_clock = _sample_clock(attitude_rows)
    desired = np.asarray([
        [row["controller.roll"], row["controller.pitch"]]
        for row in desired_rows
    ], dtype=float)
    actual = np.asarray([
        _quaternion_wxyz_to_roll_pitch_deg([
            row["kalmanPRel.q0"], row["kalmanPRel.q1"],
            row["kalmanPRel.q2"], row["kalmanPRel.q3"],
        ])
        for row in attitude_rows
    ], dtype=float) if attitude_source == "post_release15" else np.asarray([
        [row["stateEstimate.roll"], row["stateEstimate.pitch"]]
        for row in attitude_rows
    ], dtype=float)
    desired_unique = np.r_[True, np.diff(desired_time) > 0.0]
    actual_unique = np.r_[True, np.diff(actual_time) > 0.0]
    desired_time = desired_time[desired_unique]
    desired = desired[desired_unique]
    actual_time = actual_time[actual_unique]
    actual = actual[actual_unique]
    # Crazyflie setpoint pitch and estimator Euler pitch use opposite signs.
    estimator_angle_sign = np.array([1.0, -1.0] if attitude_source == "post_release15" else [1.0, 1.0])
    axes = {}
    for index, name in enumerate(("roll", "pitch")):
        # Do not invent response outside overlapping telemetry or across stalls.
        selected = (desired_time >= actual_time[0]) & (desired_time <= actual_time[-1])
        fit_time = desired_time[selected]
        if (len(fit_time) < 100 or np.max(np.diff(fit_time)) > .05
                or np.max(np.diff(actual_time)) > .05):
            raise ValueError("attitude response has insufficient overlap or >50 ms telemetry gap")
        measured = np.interp(fit_time, actual_time, actual[:, index])
        axes[name] = identify_second_order_axis(
            fit_time-fit_time[0],
            desired[selected, index],
            estimator_angle_sign[index]*measured,
        )
    usable = (all(axis["usable"] for axis in axes.values())
              and desired_clock == actual_clock == "firmware_timestamp_ms")
    return {
        "fit_schema_version": ATTITUDE_RESPONSE_SCHEMA_VERSION,
        "source": "plain_xyz_calibration_controller_setpoint_to_" + attitude_source,
        "attitude_source": attitude_source,
        "clock_basis": (
            "firmware_timestamp_ms"
            if desired_clock == actual_clock == "firmware_timestamp_ms"
            else "host_receive_time_fallback"
        ),
        "estimator_angle_sign": dict(zip(("roll", "pitch"), estimator_angle_sign.tolist())),
        "axes": axes,
        "usable": usable,
        "targeted_trials_used": False,
    }


def identify_attitude_acceleration_from_log_records(records):
    """Fit planar gravity/tilt gain from the same plain XYZ calibration."""
    if not isinstance(records, list):
        raise ValueError("calibration log must contain a list of records")
    events = {
        record.get("name"): record.get("data", {}).get("time")
        for record in records if record.get("type") == "events"
    }
    try:
        start = float(events["Wrench Calibration Excitation Started"])
        end = float(events["Wrench Calibration Excitation Complete"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("plain XYZ calibration excitation is incomplete") from error
    state_rows = [
        record["data"] for record in records
        if record.get("type") == "state"
        and record.get("group") == "P_REL_ATT"
        and start <= float(record.get("data", {}).get("time", -math.inf)) <= end
    ]
    if len(state_rows) < 100:
        raise ValueError("plain calibration lacks 15-state velocity/attitude")
    period = 0.01
    state_time, clock_basis = _sample_clock(state_rows)
    unique = np.r_[True, np.diff(state_time) > 0.0]
    state_time = state_time[unique]
    state_rows = [row for row, keep in zip(state_rows, unique) if keep]
    grid = np.arange(state_time[0]+0.5, state_time[-1]-0.5, period)
    velocity = np.column_stack([
        np.interp(grid, state_time, [row[key] for row in state_rows])
        for key in ("kalmanPRel.vx", "kalmanPRel.vy")
    ])
    rpy_rows = np.asarray([
        _quaternion_wxyz_to_rpy_rad([
            row["kalmanPRel.q0"], row["kalmanPRel.q1"],
            row["kalmanPRel.q2"], row["kalmanPRel.q3"],
        ]) for row in state_rows
    ])
    roll = np.interp(grid, state_time, rpy_rows[:, 0])
    # Firmware quaternion pitch is native; the offboard acceleration helper
    # uses the legacy Crazyflie pitch sign.
    pitch = -np.interp(grid, state_time, rpy_rows[:, 1])
    yaw = np.interp(grid, state_time, np.unwrap(rpy_rows[:, 2]))
    tilt = np.hypot(roll, pitch)
    acceleration_norm = 9.81*np.tan(tilt)
    ratio = np.divide(
        acceleration_norm, tilt, out=np.zeros_like(tilt), where=tilt > 1e-9
    )
    body_x = -ratio*pitch
    body_y = -ratio*roll
    nominal = np.column_stack([
        body_x*np.cos(yaw)-body_y*np.sin(yaw),
        body_x*np.sin(yaw)+body_y*np.cos(yaw),
    ])
    axes = {
        name: identify_attitude_acceleration_axis(
            grid-grid[0], velocity[:, index], nominal[:, index],
            sample_period_s=period,
        )
        for index, name in enumerate(("x", "y"))
    }
    return {
        "fit_schema_version": ATTITUDE_ACCELERATION_SCHEMA_VERSION,
        "source": "plain_xyz_calibration_firmware_15_state_tilt_and_velocity",
        "clock_basis": clock_basis,
        "model": "world_acceleration=gain_times_gravity_tangent_tilt_plus_bias",
        "axes": axes,
        "usable": all(axis["usable"] for axis in axes.values()),
        "targeted_trials_used": False,
    }
