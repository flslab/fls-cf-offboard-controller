"""Bounded position-command capture trials, separate from attitude identification.

Results describe the *observed trials*, not a certified continuous stopping
envelope. They deliberately do not infer position-loop acceleration from the
attitude-response fit or enable an untested runtime handoff policy.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass

import numpy as np

from Interaction.braking_response_calibration import PlanarBrakingCalibration


POSITION_CAPTURE_SCHEMA_VERSION = 1
POSITION_CAPTURE_KIND = "empirical_position_capture_trials"


def _vector(value, length, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {length} finite values")
    return result.copy()


def _readonly(value):
    value = np.asarray(value, dtype=float).copy()
    value.flags.writeable = False
    return value


@dataclass(frozen=True)
class PositionCaptureCommand:
    active: bool
    attitude_control: bool
    phase: str
    segment_id: int | None
    direction_xy: np.ndarray
    roll_deg: float
    pitch_deg: float
    position_target: np.ndarray | None


class PositionCaptureCalibration:
    """Schedule settled start -> accelerate -> level -> fixed target -> recover.

    The caller owns nominal-position holding, flight-boundary checks, command
    transmission and telemetry watchdogs. ``command`` validates measured states
    before starting excitation and latches each target exactly once. Do not use
    it to peek at a future phase: ``attitude_phase_due`` is the pure watchdog API.
    """

    def __init__(self, config, start_after_s=0.0):
        config = dict(config or {})
        self.enabled = bool(config.get("enabled", False))
        self.start_after_s = float(start_after_s)
        defaults = {
            "start_delay_s": 1.0,
            "settle_s": 0.30,
            "tilt_deg": 8.0,
            "level_before_capture_s": 0.20,
            "capture_s": 4.0,
            "recovery_s": 2.0,
            "max_xy_speed_m_s": 0.70,
            "max_displacement_m": 1.0,
            "trial_start_max_xy_speed_m_s": 0.05,
            "trial_start_max_tilt_deg": 4.0,
            "trial_start_max_position_error_m": 0.08,
            "trial_start_dwell_s": 0.30,
            "trial_start_timeout_s": 5.0,
            "trial_start_max_sample_gap_s": 0.10,
            "minimum_entry_speed_m_s": 0.03,
            "position_tolerance_m": 0.03,
            "settle_speed_m_s": 0.05,
            "settle_dwell_s": 0.30,
            "max_overshoot_m": 0.03,
            "max_reverse_speed_m_s": 0.05,
            "max_sample_gap_s": 0.10,
        }
        for name, default in defaults.items():
            value = float(config.get(name, default))
            if not np.isfinite(value) or value <= 0.0:
                if name != "start_delay_s" or value != 0.0:
                    raise ValueError(f"position capture {name} must be positive and finite")
            setattr(self, name, value)
        if not np.isfinite(self.start_after_s) or self.start_after_s < 0:
            raise ValueError("position capture start_after_s must be finite and nonnegative")
        if self.tilt_deg >= 30 or self.trial_start_max_tilt_deg >= 30:
            raise ValueError("position capture tilt limits must be below 30 degrees")
        if self.capture_s <= self.settle_dwell_s + 2 * self.max_sample_gap_s:
            raise ValueError("position capture duration must cover settling and endpoint samples")
        if self.minimum_entry_speed_m_s >= self.max_xy_speed_m_s:
            raise ValueError("position capture minimum entry speed exceeds safety limit")
        if self.trial_start_dwell_s >= self.trial_start_timeout_s:
            raise ValueError("position capture start dwell must be shorter than timeout")
        self.accelerate_durations_s = np.asarray(
            config.get("accelerate_durations_s", [0.10, 0.20, 0.30]), dtype=float
        )
        self.target_distances_m = np.asarray(
            config.get("target_distances_m", [0.08, 0.16, 0.28]), dtype=float
        )
        if (
            self.accelerate_durations_s.ndim != 1
            or not len(self.accelerate_durations_s)
            or self.target_distances_m.shape != self.accelerate_durations_s.shape
            or not np.all(np.isfinite(self.accelerate_durations_s))
            or not np.all(np.isfinite(self.target_distances_m))
            or np.any(self.accelerate_durations_s <= 0)
            or np.any(self.target_distances_m <= 0)
            or np.any(self.target_distances_m >= self.max_displacement_m)
            or np.any(np.diff(self.accelerate_durations_s) <= 0)
            or np.any(np.diff(self.target_distances_m) <= 0)
        ):
            raise ValueError("position capture acceleration/gap levels must be matched, positive and increasing")
        repetitions = config.get("repetitions", 1)
        if isinstance(repetitions, bool) or int(repetitions) != repetitions or int(repetitions) <= 0:
            raise ValueError("position capture repetitions must be a positive integer")
        self.repetitions = int(repetitions)
        directions = np.asarray(config.get("directions_xy", [
            [1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0],
        ]), dtype=float)
        if (
            directions.ndim != 2 or directions.shape[1:] != (2,)
            or not len(directions) or not np.all(np.isfinite(directions))
            or np.any(np.linalg.norm(directions, axis=1) <= 1e-9)
        ):
            raise ValueError("position capture directions must be finite nonzero XY pairs")
        self.directions = directions / np.linalg.norm(directions, axis=1)[:, None]
        for direction in self.directions:
            if not np.any(self.directions @ direction < -0.999):
                raise ValueError("position capture requires both signs of every tested direction")
        if len(np.unique(np.round(self.directions, 6), axis=0)) != len(self.directions):
            raise ValueError("position capture directions must be unique")
        self.maneuver_start_s = self.start_after_s + self.start_delay_s
        self.trials = []
        start = self.maneuver_start_s
        for repetition in range(self.repetitions):
            for level, (accel_s, distance) in enumerate(zip(
                self.accelerate_durations_s, self.target_distances_m
            )):
                for direction in self.directions:
                    trial = {
                        "segment_id": len(self.trials), "repetition": repetition,
                        "level_index": level, "direction_xy": direction.tolist(),
                        "accelerate_s": float(accel_s), "target_distance_m": float(distance),
                        "start_s": float(start),
                    }
                    trial["capture_start_s"] = float(start + self.settle_s + accel_s + self.level_before_capture_s)
                    trial["capture_end_s"] = trial["capture_start_s"] + self.capture_s
                    trial["end_s"] = trial["capture_end_s"] + self.recovery_s
                    self.trials.append(trial)
                    start = trial["end_s"]
        self.end_s = float(start)
        self.trial_directions = np.asarray([trial["direction_xy"] for trial in self.trials])
        self._started = set()
        self._entries = {}
        self._last_elapsed_s = None

    @property
    def duration_s(self):
        return self.end_s if self.enabled else 0.0

    def _scheduled(self, elapsed_s):
        elapsed_s = float(elapsed_s)
        if not np.isfinite(elapsed_s):
            raise ValueError("position capture time must be finite")
        if not self.enabled or elapsed_s < self.maneuver_start_s:
            return None, "waiting"
        for trial in self.trials:
            if elapsed_s < trial["end_s"]:
                if elapsed_s < trial["start_s"] + self.settle_s:
                    phase = "settle"
                elif elapsed_s < trial["start_s"] + self.settle_s + trial["accelerate_s"]:
                    phase = "accelerate"
                elif elapsed_s < trial["capture_start_s"]:
                    phase = "level_before_capture"
                elif elapsed_s < trial["capture_end_s"]:
                    phase = "capture"
                else:
                    phase = "recovery"
                return trial, phase
        return None, "complete"

    def attitude_phase_due(self, elapsed_s):
        return self._scheduled(elapsed_s)[1] in {"accelerate", "level_before_capture"}

    def command(self, elapsed_s, yaw_deg, position, velocity, orientation_rpy):
        position = _vector(position, 3, "position capture position")
        velocity = _vector(velocity, 3, "position capture velocity")
        orientation = _vector(orientation_rpy, 3, "position capture orientation")
        if not np.isfinite(float(yaw_deg)):
            raise ValueError("position capture yaw must be finite")
        trial, phase = self._scheduled(elapsed_s)
        if self._last_elapsed_s is not None and float(elapsed_s) < self._last_elapsed_s - 1e-9:
            raise ValueError("position capture time cannot move backwards")
        self._last_elapsed_s = float(elapsed_s)
        if trial is None:
            return PositionCaptureCommand(False, False, phase, None, _readonly([0, 0]), 0.0, 0.0, None)
        direction = np.asarray(trial["direction_xy"])
        segment = trial["segment_id"]
        speed = float(np.linalg.norm(velocity[:2]))
        # Retired experiment: speed is evaluated in the diagnostic report,
        # not used as an in-flight abort. Settled trial starts remain required.
        if phase == "accelerate" and segment not in self._started:
            if speed > self.trial_start_max_xy_speed_m_s:
                raise ValueError("position capture trial must start at settled XY speed")
            if np.linalg.norm(np.degrees(orientation[:2])) > self.trial_start_max_tilt_deg:
                raise ValueError("position capture trial must start at settled tilt")
            self._started.add(segment)
        target = None
        if phase == "capture":
            if segment not in self._started:
                raise ValueError("position capture trial skipped its acceleration phase")
            if segment not in self._entries:
                projected_speed = float(velocity[:2] @ direction)
                if projected_speed < self.minimum_entry_speed_m_s:
                    raise ValueError("position capture entry lacks positive directional excitation")
                target = position.copy()
                target[:2] += trial["target_distance_m"] * direction
                self._entries[segment] = {
                    "entry_elapsed_s": float(elapsed_s),
                    "entry_position": position.tolist(), "entry_velocity": velocity.tolist(),
                    "entry_orientation_rpy": orientation.tolist(), "position_target": target.tolist(),
                }
            target = _readonly(self._entries[segment]["position_target"])
        roll = pitch = 0.0
        if phase == "accelerate":
            acceleration = 9.81 * np.tan(np.radians(self.tilt_deg)) * direction
            roll, pitch = PlanarBrakingCalibration._world_acceleration_to_attitude(acceleration, yaw_deg)
        return PositionCaptureCommand(
            True, phase in {"accelerate", "level_before_capture"}, phase,
            segment, _readonly(direction), float(roll), float(pitch), target,
        )

    def protocol(self):
        fields = (
            "start_delay_s", "settle_s", "tilt_deg", "level_before_capture_s",
            "capture_s", "recovery_s", "max_xy_speed_m_s", "max_displacement_m",
            "trial_start_max_xy_speed_m_s", "trial_start_max_tilt_deg",
            "trial_start_max_position_error_m",
            "trial_start_dwell_s", "trial_start_timeout_s",
            "trial_start_max_sample_gap_s",
            "minimum_entry_speed_m_s", "position_tolerance_m",
            "settle_speed_m_s", "settle_dwell_s", "max_overshoot_m",
            "max_reverse_speed_m_s",
            "max_sample_gap_s", "repetitions",
        )
        result = {name: getattr(self, name) for name in fields}
        result.update({
            "directions_xy": self.directions.tolist(),
            "accelerate_durations_s": self.accelerate_durations_s.tolist(),
            "target_distances_m": self.target_distances_m.tolist(),
            "trial_count": len(self.trials),
            "trials": [dict(deepcopy(trial), **deepcopy(self._entries.get(trial["segment_id"], {}))) for trial in self.trials],
        })
        return result

    def summarize(self, samples):
        return summarize_position_capture(samples, self.protocol())


def summarize_position_capture(samples, protocol):
    """Return pass/fail evidence without fitting an untested continuous envelope.

    Samples use absolute ``timestamp`` and first actual position-command send
    ``command_started_at`` plus ``segment_id``, ``phase='capture'`` and XYZ
    ``position``, ``velocity``, ``orientation_rpy``, ``position_target`` vectors.
    """
    protocol = deepcopy(protocol)
    failures = []
    records = []
    grouped = {trial["segment_id"]: [] for trial in protocol["trials"]}
    for sample in samples:
        if not isinstance(sample, dict):
            failures.append("capture sample must be a mapping")
            continue
        if sample.get("phase") != "capture":
            continue
        segment = sample.get("segment_id")
        if not isinstance(segment, int) or isinstance(segment, bool) or segment not in grouped:
            failures.append("sample has unknown segment_id")
        else:
            grouped[segment].append(sample)
    max_gap = float(protocol["max_sample_gap_s"])
    for trial in protocol["trials"]:
        segment = trial["segment_id"]
        record = {"segment_id": segment, "direction_xy": trial["direction_xy"], "passed": False, "quality_failures": []}
        issues = record["quality_failures"]
        rows = grouped[segment]
        record["sample_count"] = len(rows)
        if not rows or "entry_position" not in trial:
            issues.append("missing capture samples or recorded entry state")
            records.append(record)
            continue
        try:
            times = np.asarray([float(row["timestamp"]) for row in rows])
            starts = np.asarray([float(row["command_started_at"]) for row in rows])
            positions = np.array([_vector(row["position"], 3, "sample position") for row in rows])
            velocities = np.array([_vector(row["velocity"], 3, "sample velocity") for row in rows])
            targets = np.array([_vector(row["position_target"], 3, "sample target") for row in rows])
            for row in rows:
                _vector(row["orientation_rpy"], 3, "sample orientation")
            if not np.all(np.isfinite(np.r_[times, starts])):
                raise ValueError("non-finite sample timestamps")
            entry = _vector(trial["entry_position"], 3, "trial entry")
            entry_velocity = _vector(trial["entry_velocity"], 3, "trial velocity")
            entry_orientation = _vector(trial["entry_orientation_rpy"], 3, "trial orientation")
            target = _vector(trial["position_target"], 3, "trial target")
        except (ValueError, KeyError, TypeError) as exc:
            issues.append(f"malformed capture samples: {exc}")
            records.append(record)
            continue
        gaps = np.diff(times)
        if len(times) < 2 or np.any(gaps <= 0):
            issues.append("sample timestamps must be strictly increasing")
        observed_gap = float(np.max(gaps)) if len(gaps) else 0.0
        if observed_gap > max_gap + 1e-9:
            issues.append("capture sample gap exceeds maximum")
        if np.max(np.abs(starts - starts[0])) > 1e-6:
            issues.append("capture command start timestamp changed")
        if times[0] < starts[0] - 1e-6 or times[0] - starts[0] > max_gap + 1e-9:
            issues.append("capture entry sample coverage is incomplete")
        duration = float(times[-1] - starts[0])
        if duration < float(protocol["capture_s"]) - max_gap - 1e-9:
            issues.append("capture observation ended too early")
        if np.max(np.abs(targets - target)) > 1e-6:
            issues.append("capture target changed during trial")
        direction = _vector(trial["direction_xy"], 2, "trial direction")
        forward_speed = float(entry_velocity[:2] @ direction)
        lateral_speed = float(np.linalg.norm(entry_velocity[:2] - forward_speed * direction))
        if forward_speed < float(protocol["minimum_entry_speed_m_s"]):
            issues.append("entry lacks positive directional excitation")
        if np.max(np.linalg.norm(velocities[:, :2], axis=1)) > float(protocol["max_xy_speed_m_s"]) + 1e-9:
            issues.append("capture speed exceeded safety limit")
        actual_gap = float((target[:2] - entry[:2]) @ direction)
        if abs(actual_gap - float(trial["target_distance_m"])) > 1e-6:
            issues.append("target gap differs from planned trial")
        overshoot = float(max(0.0, np.max((positions[:, :2] - target[:2]) @ direction)))
        if overshoot > float(protocol["max_overshoot_m"]) + 1e-9:
            issues.append("position capture overshoot exceeds tolerance")
        reverse_speed = float(max(0.0, -np.min(velocities[:, :2] @ direction)))
        if reverse_speed > float(protocol["max_reverse_speed_m_s"]) + 1e-9:
            issues.append("position capture reverse speed exceeds tolerance")
        errors = np.linalg.norm(positions[:, :2] - target[:2], axis=1)
        speeds = np.linalg.norm(velocities[:, :2], axis=1)
        settled = (errors <= float(protocol["position_tolerance_m"])) & (speeds <= float(protocol["settle_speed_m_s"]))
        final_start = len(times)
        while final_start > 0 and settled[final_start - 1]:
            final_start -= 1
        dwell = float(times[-1] - times[final_start]) if final_start < len(times) else 0.0
        if dwell + 1e-9 < float(protocol["settle_dwell_s"]):
            issues.append("terminal position and speed did not remain settled")
        record.update({
            "entry_position": entry.tolist(), "entry_velocity": entry_velocity.tolist(),
            "entry_orientation_rpy": entry_orientation.tolist(), "position_target": target.tolist(),
            "entry_speed_m_s": forward_speed, "entry_lateral_speed_m_s": lateral_speed,
            "entry_tilt_deg": float(np.linalg.norm(np.degrees(entry_orientation[:2]))),
            "target_distance_m": actual_gap, "peak_overshoot_m": overshoot,
            "peak_reverse_speed_m_s": reverse_speed,
            "terminal_error_m": float(errors[-1]), "terminal_speed_m_s": float(speeds[-1]),
            "terminal_settled_dwell_s": dwell,
            "time_to_settle_s": float(times[final_start] - starts[0]) if final_start < len(times) else None,
            "observation_duration_s": duration, "maximum_sample_gap_s": observed_gap,
            "passed": not issues,
        })
        records.append(record)
    for record in records:
        failures.extend(f"trial {record['segment_id']}: {issue}" for issue in record["quality_failures"])
    return {
        "fit_schema_version": POSITION_CAPTURE_SCHEMA_VERSION, "kind": POSITION_CAPTURE_KIND,
        "usable": bool(records) and not failures and all(record["passed"] for record in records),
        "continuous_capture_envelope_certified": False,
        "protocol": protocol, "trial_count": len(records), "trials": records,
        "tested_directions_xy": deepcopy(protocol["directions_xy"]), "quality_failures": failures,
    }


def position_capture_fit_is_current(fit):
    """Validate complete empirical evidence; this is not a runtime safety gate."""
    try:
        if (
            not isinstance(fit, dict) or fit.get("fit_schema_version") != POSITION_CAPTURE_SCHEMA_VERSION
            or fit.get("kind") != POSITION_CAPTURE_KIND or fit.get("usable") is not True
            or fit.get("continuous_capture_envelope_certified") is not False
            or fit.get("quality_failures") != []
        ):
            return False
        protocol = fit["protocol"]
        # Revalidate all configured numerical limits and signed direction coverage.
        checked = PositionCaptureCalibration(protocol)
        records = fit["trials"]
        if len(records) != len(checked.trials) or fit["trial_count"] != len(records):
            return False
        if not np.allclose(fit["tested_directions_xy"], checked.directions, atol=1e-8):
            return False
        if len(protocol["trials"]) != len(records) or protocol["trial_count"] != len(records):
            return False
        def finite_tree(value):
            if isinstance(value, dict):
                return all(finite_tree(item) for item in value.values())
            if isinstance(value, (list, tuple)):
                return all(finite_tree(item) for item in value)
            if isinstance(value, (float, int)) and not isinstance(value, bool):
                return bool(np.isfinite(value))
            return True
        if not finite_tree(fit):
            return False
        for index, record in enumerate(records):
            if record.get("segment_id") != index or record.get("passed") is not True or record.get("quality_failures") != []:
                return False
            if record["sample_count"] < 2:
                return False
            target = _vector(record["position_target"], 3, "saved capture target")
            entry = _vector(record["entry_position"], 3, "saved capture entry")
            entry_velocity = _vector(record["entry_velocity"], 3, "saved capture velocity")
            orientation = _vector(record["entry_orientation_rpy"], 3, "saved capture orientation")
            if not np.allclose(record["direction_xy"], checked.trials[index]["direction_xy"], atol=1e-8):
                return False
            direction = np.asarray(record["direction_xy"], dtype=float)
            projected_speed = float(entry_velocity[:2] @ direction)
            lateral_speed = float(np.linalg.norm(entry_velocity[:2] - projected_speed * direction))
            expected_target = entry.copy()
            expected_target[:2] += checked.trials[index]["target_distance_m"] * direction
            if (
                not np.allclose(target, expected_target, atol=1e-6, rtol=0)
                or abs(record["entry_speed_m_s"] - projected_speed) > 1e-6
                or abs(record["entry_lateral_speed_m_s"] - lateral_speed) > 1e-6
                or abs(record["entry_tilt_deg"] - np.linalg.norm(np.degrees(orientation[:2]))) > 1e-6
                or np.linalg.norm(entry_velocity[:2]) > checked.max_xy_speed_m_s + 1e-9
                or not 0 <= record["peak_reverse_speed_m_s"] <= checked.max_reverse_speed_m_s + 1e-9
            ):
                return False
            bounded = (
                0 <= record["peak_overshoot_m"] <= checked.max_overshoot_m + 1e-9,
                0 <= record["terminal_error_m"] <= checked.position_tolerance_m,
                0 <= record["terminal_speed_m_s"] <= checked.settle_speed_m_s,
                record["terminal_settled_dwell_s"] + 1e-9 >= checked.settle_dwell_s,
                checked.minimum_entry_speed_m_s <= record["entry_speed_m_s"] <= checked.max_xy_speed_m_s,
                0 < record["maximum_sample_gap_s"] <= checked.max_sample_gap_s + 1e-9,
                record["observation_duration_s"] >= checked.capture_s - checked.max_sample_gap_s - 1e-9,
                0 <= record["time_to_settle_s"] <= record["observation_duration_s"],
                abs(record["target_distance_m"] - checked.trials[index]["target_distance_m"]) <= 1e-6,
            )
            if not all(bounded):
                return False
        return True
    except (KeyError, ValueError, TypeError, OverflowError):
        return False
