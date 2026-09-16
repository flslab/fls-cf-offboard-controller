"""Evaluate post-release attitude fusion from a CrazySim JSONL trace.

The evaluator is deliberately offline and command-free.  It initializes the
post-release filter from a stable IMU alignment window and the release odometry
snapshot, then propagates :class:`PostReleaseInertialEkf` with measured gyro and
specific force.  Every later odometry observation is reduced to position before
it reaches the filter.  Odometry orientation is retained only for scoring.

Expected JSONL records are, in chronological order::

    {"type":"meta","schema":"contact_attitude_crazysim_trace_v2",
     "position_only":true,"alignment_yaw_deg":0.0,
     "sim_time_basis":"gazebo_header_and_scheduler_sim_time_v1",
     "external_pose_mode":"position_only","host_arrival_time_used":false,
     "odom_position_role":"position_only_estimator_input",
     "odom_velocity_role":"plant_diagnostics_only_not_estimator_input",
     "odom_quaternion_role":"evaluation_truth_only",
     "command_history_is_estimator_input":false,
     "release_velocity_seed_policy":"causal_position_finite_difference",
     "contact_start_sim_time_ns":300000000,
     "release_sim_time_ns":400000000}
    {"type":"imu","sim_time_ns":1000000,
     "angular_velocity_rad_s":[0,0,0],
     "linear_acceleration_m_s2":[0,0,9.80665]}
    {"type":"odom","sim_time_ns":1000000,"position_m":[0,0,1],
     "velocity_m_s":[0,0,0],"quaternion_xyzw":[0,0,0,1]}
    {"type":"event","name":"contact","sim_time_ns":300000000}
    {"type":"event","name":"release","sim_time_ns":400000000}

Simulator nanoseconds must lie on integer millisecond epochs because the
Crazyflie timestamp transported by the shadow protocol is a wrapping 24-bit
millisecond counter.  The report includes a truth-leakage canary: estimator
trajectories are re-run after post-release truth quaternions are first mutated
and then removed, and all three estimator-output hashes must be identical.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
from numbers import Integral, Real
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import numpy as np

from Interaction.contact_attitude_observer import (
    CF_TIMESTAMP_MODULUS_MS,
    integrate_body_rate,
    quaternion_from_accel_and_legacy_yaw,
)
from Interaction.post_release_inertial_ekf import (
    GRAVITY_M_S2,
    PostReleaseEkfConfig,
    PostReleaseInertialEkf,
)


TRACE_SCHEMA = "contact_attitude_crazysim_trace_v2"
REPORT_SCHEMA = "contact_attitude_crazysim_evaluation_v2"
SIM_TIME_NS_PER_CF_MS = 1_000_000
EXPECTED_CLOCK_BASIS = "gazebo_header_and_scheduler_sim_time_v1"


class TraceValidationError(ValueError):
    """Raised when a trace cannot establish an unambiguous causal timeline."""


@dataclass(frozen=True)
class EvaluationConfig:
    min_alignment_coverage_ms: float = 300.0
    min_alignment_samples: int = 100
    max_alignment_accel_norm_error_g: float = 0.12
    max_alignment_mean_gyro_deg_s: float = 2.0
    max_alignment_gyro_std_deg_s: float = 2.0
    max_imu_gap_ms: float = 5.0
    max_position_gap_ms: float = 50.0
    max_release_seed_skew_ms: float = 5.0
    release_velocity_seed_window_ms: float = 10.0
    min_release_velocity_seed_dt_ms: float = 5.0
    max_release_velocity_seed_dt_ms: float = 20.0
    min_post_release_coverage_ms: float = 800.0
    min_position_coverage_ms: float = 750.0
    min_position_updates: int = 40
    min_truth_samples: int = 40
    min_rotation_excitation_deg: float = 5.0
    min_truth_attitude_excursion_deg: float = 3.0
    min_position_displacement_m: float = 0.03
    min_specific_force_rms_g: float = 0.5
    max_position_rejection_fraction: float = 0.10
    max_release_seed_attitude_error_deg: float = 1.0
    max_fused_rmse_deg: float = 1.0
    max_nominal_fused_p95_deg: float = 1.0
    max_stressed_fused_p95_deg: float = 1.2
    max_fused_final_deg: float = 1.2
    min_rmse_improvement_deg: float = 0.20
    max_fused_to_gyro_rmse_ratio: float = 0.85

    def __post_init__(self):
        numeric = asdict(self)
        if any(
            isinstance(value, bool) or not math.isfinite(float(value))
            or float(value) <= 0.0
            for value in numeric.values()
        ):
            raise ValueError("all evaluation thresholds must be positive")
        if self.max_position_rejection_fraction > 1.0:
            raise ValueError("position rejection fraction cannot exceed one")
        if self.max_fused_to_gyro_rmse_ratio > 1.0:
            raise ValueError("A/B error ratio cannot exceed one")
        if not (
            self.min_release_velocity_seed_dt_ms
            <= self.release_velocity_seed_window_ms
            <= self.max_release_velocity_seed_dt_ms
        ):
            raise ValueError(
                "release velocity seed window must lie inside its dt bounds"
            )


@dataclass(frozen=True)
class ImuSample:
    sim_time_ns: int
    gyro_rad_s: tuple[float, float, float]
    accel_m_s2: tuple[float, float, float]

    @property
    def sim_time_ms(self) -> int:
        return self.sim_time_ns // SIM_TIME_NS_PER_CF_MS


@dataclass(frozen=True)
class OdomSample:
    sim_time_ns: int
    position_m: tuple[float, float, float]
    velocity_m_s: tuple[float, float, float]
    truth_quaternion_wxyz: tuple[float, float, float, float] | None

    @property
    def sim_time_ms(self) -> int:
        return self.sim_time_ns // SIM_TIME_NS_PER_CF_MS


@dataclass(frozen=True)
class ParsedTrace:
    metadata: dict
    imu: tuple[ImuSample, ...]
    odom: tuple[OdomSample, ...]
    release_time_ns: int

    @property
    def release_time_ms(self) -> int:
        return self.release_time_ns // SIM_TIME_NS_PER_CF_MS


def _finite_scalar(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TraceValidationError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise TraceValidationError(f"{name} must be a finite number")
    return result


def _finite_vector(value, size: int, name: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise TraceValidationError(f"{name} must contain {size} finite numbers")
    result = tuple(_finite_scalar(item, name) for item in value)
    return result


def _sim_time_ns(record: Mapping, record_index: int) -> int:
    value = record.get("sim_time_ns")
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise TraceValidationError(
            f"record {record_index} sim_time_ns must be an integer"
        )
    result = int(value)
    if result < 0:
        raise TraceValidationError("sim_time_ns cannot be negative")
    if result % SIM_TIME_NS_PER_CF_MS:
        raise TraceValidationError(
            "sim_time_ns must map exactly to a Crazyflie millisecond epoch"
        )
    return result


def _normalized_truth_quaternion_xyzw(value) -> tuple[float, float, float, float]:
    xyzw = np.asarray(_finite_vector(value, 4, "quaternion_xyzw"), dtype=float)
    norm = float(np.linalg.norm(xyzw))
    if norm <= 1e-12:
        raise TraceValidationError("quaternion_xyzw must have non-zero norm")
    xyzw /= norm
    wxyz = np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]], dtype=float)
    if wxyz[0] < 0.0:
        wxyz *= -1.0
    return tuple(float(item) for item in wxyz)


def parse_records(records: Sequence[Mapping]) -> ParsedTrace:
    """Validate and parse records without deriving any estimator state."""
    if not isinstance(records, Sequence) or isinstance(records, (str, bytes)):
        raise TraceValidationError("trace must be a sequence of JSON objects")
    if not records:
        raise TraceValidationError("trace is empty")
    first = records[0]
    if not isinstance(first, Mapping) or first.get("type") != "meta":
        raise TraceValidationError("the first JSONL record must be metadata")
    if first.get("schema") != TRACE_SCHEMA:
        raise TraceValidationError(f"metadata schema must be {TRACE_SCHEMA}")
    if first.get("position_only") is not True:
        raise TraceValidationError("metadata position_only must be true")
    alignment_yaw_deg = _finite_scalar(
        first.get("alignment_yaw_deg"), "alignment_yaw_deg"
    )
    clock_basis = first.get("sim_time_basis")
    if clock_basis != EXPECTED_CLOCK_BASIS:
        raise TraceValidationError(
            f"sim_time_basis must be {EXPECTED_CLOCK_BASIS}"
        )
    provenance_contract = {
        "external_pose_mode": "position_only",
        "host_arrival_time_used": False,
        "odom_position_role": "position_only_estimator_input",
        "odom_velocity_role": (
            "plant_diagnostics_only_not_estimator_input"
        ),
        "odom_quaternion_role": "evaluation_truth_only",
        "command_history_is_estimator_input": False,
        "release_velocity_seed_policy": (
            "causal_position_finite_difference"
        ),
    }
    for field, expected in provenance_contract.items():
        if first.get(field) != expected:
            raise TraceValidationError(
                f"metadata {field} must be {expected!r}"
            )
    contact_meta_ns = _sim_time_ns(
        {"sim_time_ns": first.get("contact_start_sim_time_ns")}, 0
    )
    release_meta_ns = _sim_time_ns(
        {"sim_time_ns": first.get("release_sim_time_ns")}, 0
    )
    if contact_meta_ns >= release_meta_ns:
        raise TraceValidationError("metadata contact time must precede release")

    metadata = dict(first)
    metadata["alignment_yaw_deg"] = alignment_yaw_deg
    metadata["sim_time_basis"] = clock_basis
    imu = []
    odom = []
    event_times = {"contact": [], "release": []}
    previous_global_time = None
    previous_by_type = {"imu": None, "odom": None}
    for index, record in enumerate(records[1:], start=1):
        if not isinstance(record, Mapping):
            raise TraceValidationError(f"record {index} must be a JSON object")
        record_type = record.get("type")
        if record_type == "meta":
            raise TraceValidationError("metadata may appear only once and first")
        if record_type not in {"imu", "odom", "event"}:
            raise TraceValidationError(f"record {index} has unsupported type")
        timestamp = _sim_time_ns(record, index)
        if previous_global_time is not None and timestamp < previous_global_time:
            raise TraceValidationError("records must use a nondecreasing sim clock")
        previous_global_time = timestamp
        if record_type in previous_by_type:
            previous = previous_by_type[record_type]
            if previous is not None and timestamp <= previous:
                raise TraceValidationError(
                    f"{record_type} sim_time_ns must be strictly increasing"
                )
            previous_by_type[record_type] = timestamp

        if record_type == "imu":
            imu.append(ImuSample(
                timestamp,
                _finite_vector(
                    record.get("angular_velocity_rad_s"), 3,
                    "angular_velocity_rad_s",
                ),
                _finite_vector(
                    record.get("linear_acceleration_m_s2"), 3,
                    "linear_acceleration_m_s2",
                ),
            ))
        elif record_type == "odom":
            odom.append(OdomSample(
                timestamp,
                _finite_vector(record.get("position_m"), 3, "position_m"),
                _finite_vector(record.get("velocity_m_s"), 3, "velocity_m_s"),
                _normalized_truth_quaternion_xyzw(
                    record.get("quaternion_xyzw")
                ),
            ))
        else:
            name = record.get("name")
            if name not in event_times:
                raise TraceValidationError(
                    "the only supported events are contact and release"
                )
            event_times[name].append(timestamp)

    if not imu:
        raise TraceValidationError("trace has no IMU samples")
    if not odom:
        raise TraceValidationError("trace has no odometry samples")
    if any(len(event_times[name]) != 1 for name in event_times):
        raise TraceValidationError(
            "trace must contain exactly one contact and one release event"
        )
    contact = event_times["contact"][0]
    release = event_times["release"][0]
    if contact >= release:
        raise TraceValidationError("contact event must precede release event")
    if contact != contact_meta_ns:
        raise TraceValidationError(
            "contact event does not match contact_start_sim_time_ns"
        )
    if release != release_meta_ns:
        raise TraceValidationError(
            "release event does not match release_sim_time_ns"
        )
    if not imu[0].sim_time_ns < release < imu[-1].sim_time_ns:
        raise TraceValidationError("release must lie inside the IMU timeline")
    if not odom[0].sim_time_ns <= release < odom[-1].sim_time_ns:
        raise TraceValidationError("release must lie inside the odometry timeline")
    release_ms = release // SIM_TIME_NS_PER_CF_MS
    if release_ms not in {sample.sim_time_ms for sample in imu}:
        raise TraceValidationError("release must coincide with an IMU epoch")
    return ParsedTrace(metadata, tuple(imu), tuple(odom), release)


def read_jsonl(path: Path) -> list[dict]:
    records = []
    with path.open() as source:
        for line_number, line in enumerate(source, start=1):
            if not line.strip():
                raise TraceValidationError(
                    f"blank JSONL record at line {line_number}"
                )
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise TraceValidationError(
                    f"invalid JSON at line {line_number}: {error.msg}"
                ) from error
            records.append(record)
    return records


def _maximum_gap_ms(
        samples: Sequence[ImuSample | OdomSample],
) -> float | None:
    if len(samples) < 2:
        return None
    return max(
        (right.sim_time_ns - left.sim_time_ns) / SIM_TIME_NS_PER_CF_MS
        for left, right in zip(samples, samples[1:])
    )


def _quaternion_error_deg(left, right) -> float:
    dot = abs(float(np.dot(left, right)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def _percentile(values: Sequence[float], percentile: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=float), percentile))


def _attitude_metrics(errors: Sequence[float]) -> dict:
    if not errors:
        return {"sample_count": 0, "rmse_deg": None, "p95_deg": None,
                "final_deg": None, "max_deg": None}
    array = np.asarray(errors, dtype=float)
    return {
        "sample_count": len(errors),
        "rmse_deg": float(math.sqrt(float(np.mean(array * array)))),
        "p95_deg": _percentile(errors, 95.0),
        "final_deg": float(errors[-1]),
        "max_deg": float(np.max(array)),
    }


def _alignment_seed(trace: ParsedTrace, config: EvaluationConfig):
    pre_release = [
        sample for sample in trace.imu
        if sample.sim_time_ns <= trace.release_time_ns
    ]
    first_ms = pre_release[0].sim_time_ms
    target_end_ms = first_ms + int(math.ceil(config.min_alignment_coverage_ms))
    alignment = [sample for sample in pre_release
                 if sample.sim_time_ms <= target_end_ms]
    if not alignment:
        raise TraceValidationError("no pre-release alignment samples")
    alignment_end = alignment[-1]
    accelerations = np.asarray([sample.accel_m_s2 for sample in alignment])
    gyros = np.asarray([sample.gyro_rad_s for sample in alignment])
    accel_norm_error_g = float(
        np.max(np.abs(np.linalg.norm(accelerations, axis=1) / GRAVITY_M_S2 - 1.0))
    )
    gyro_mean = np.mean(gyros, axis=0)
    gyro_mean_norm_deg_s = float(np.linalg.norm(np.degrees(gyro_mean)))
    gyro_std_norm_deg_s = float(np.linalg.norm(np.degrees(
        np.std(gyros, axis=0)
    )))
    quaternion = quaternion_from_accel_and_legacy_yaw(
        np.mean(accelerations, axis=0) / GRAVITY_M_S2,
        math.radians(trace.metadata["alignment_yaw_deg"]),
    )
    previous_gyro = np.asarray(alignment_end.gyro_rad_s, dtype=float)
    previous_ms = alignment_end.sim_time_ms
    for sample in pre_release[len(alignment):]:
        gyro = np.asarray(sample.gyro_rad_s, dtype=float)
        dt = (sample.sim_time_ms - previous_ms) / 1000.0
        quaternion = integrate_body_rate(
            quaternion, 0.5 * (previous_gyro + gyro) - gyro_mean, dt
        )
        previous_gyro = gyro
        previous_ms = sample.sim_time_ms
    coverage_ms = alignment[-1].sim_time_ms - alignment[0].sim_time_ms
    return {
        "quaternion_wxyz": tuple(float(item) for item in quaternion),
        "gyro_bias_rad_s": tuple(float(item) for item in gyro_mean),
        "last_gyro_rad_s": tuple(float(item) for item in previous_gyro),
        "release_imu_time_ms": previous_ms,
        "sample_count": len(alignment),
        "coverage_ms": float(coverage_ms),
        "max_accel_norm_error_g": accel_norm_error_g,
        "mean_gyro_norm_deg_s": gyro_mean_norm_deg_s,
        "gyro_std_norm_deg_s": gyro_std_norm_deg_s,
    }


def _canonical_hash_update(digest, value) -> None:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    digest.update(encoded)
    digest.update(b"\n")


def _release_position_seed(
        trace: ParsedTrace, config: EvaluationConfig,
) -> dict:
    """Construct a release p/v seed using causal world-frame positions only."""
    causal = [
        sample for sample in trace.odom
        if sample.sim_time_ns <= trace.release_time_ns
    ]
    if len(causal) < 2:
        raise TraceValidationError(
            "release velocity seed requires at least two causal positions"
        )
    latest = causal[-1]
    candidates = []
    for sample in causal[:-1]:
        dt_ms = latest.sim_time_ms - sample.sim_time_ms
        if (
            config.min_release_velocity_seed_dt_ms <= dt_ms
            <= config.max_release_velocity_seed_dt_ms
        ):
            candidates.append((
                abs(dt_ms - config.release_velocity_seed_window_ms),
                -sample.sim_time_ms,
                sample,
                float(dt_ms),
            ))
    if not candidates:
        raise TraceValidationError(
            "no causal position pair satisfies release velocity seed dt bounds"
        )
    _, _, previous, seed_dt_ms = min(candidates, key=lambda item: item[:2])
    velocity = (
        np.asarray(latest.position_m) - np.asarray(previous.position_m)
    ) / (seed_dt_ms / 1000.0)
    skew_ms = (
        trace.release_time_ns - latest.sim_time_ns
    ) / SIM_TIME_NS_PER_CF_MS
    position_at_release = (
        np.asarray(latest.position_m) + velocity * (skew_ms / 1000.0)
    )
    return {
        "position_m": tuple(float(item) for item in position_at_release),
        "velocity_m_s": tuple(float(item) for item in velocity),
        "latest_position_m": latest.position_m,
        "latest_position_sim_time_ms": latest.sim_time_ms,
        "previous_position_sim_time_ms": previous.sim_time_ms,
        "finite_difference_dt_ms": seed_dt_ms,
        "position_extrapolation_dt_ms": float(skew_ms),
        "policy": "causal_position_finite_difference",
        "coordinate_frame": "world",
        "odom_velocity_field_used": False,
    }


def _run_estimator(
        trace: ParsedTrace, alignment: Mapping,
        gyro_bias_stress_deg_s: Sequence[float],
        config: EvaluationConfig | None = None,
) -> dict:
    config = config or EvaluationConfig()
    release_ms = trace.release_time_ms
    release_seed = _release_position_seed(trace, config)
    ekf = PostReleaseInertialEkf(
        position_m=release_seed["position_m"],
        velocity_m_s=release_seed["velocity_m_s"],
        quaternion_wxyz=alignment["quaternion_wxyz"],
        gyro_bias_rad_s=alignment["gyro_bias_rad_s"],
        cf_timestamp_ms=release_ms % CF_TIMESTAMP_MODULUS_MS,
        unwrapped_timestamp_ms=release_ms,
        initial_gyro_deg_s=np.degrees(alignment["last_gyro_rad_s"]),
        config=PostReleaseEkfConfig(max_imu_gap_ms=config.max_imu_gap_ms),
    )
    baseline_quaternion = np.asarray(alignment["quaternion_wxyz"], dtype=float)
    baseline_previous_gyro = np.asarray(
        alignment["last_gyro_rad_s"], dtype=float
    )
    previous_raw_gyro = baseline_previous_gyro.copy()
    alignment_bias = np.asarray(alignment["gyro_bias_rad_s"], dtype=float)
    stress_rad_s = np.radians(np.asarray(gyro_bias_stress_deg_s, dtype=float))
    odom_by_ms = {
        sample.sim_time_ms: sample for sample in trace.odom
        if sample.sim_time_ms > release_ms
    }
    digest = hashlib.sha256()
    fused_errors = []
    baseline_errors = []
    score_times_ms = []
    final_estimate = ekf.snapshot()
    post_imu = [sample for sample in trace.imu
                if sample.sim_time_ms > release_ms]
    previous_ms = release_ms
    integrated_rotation_deg = 0.0
    force_norms_g = []
    for sample in post_imu:
        raw_gyro = np.asarray(sample.gyro_rad_s)
        measured_gyro = raw_gyro + stress_rad_s
        dt = (sample.sim_time_ms - previous_ms) / 1000.0
        mean_corrected_gyro = (
            0.5 * (baseline_previous_gyro + measured_gyro) - alignment_bias
        )
        raw_mean_corrected_gyro = (
            0.5 * (previous_raw_gyro + raw_gyro) - alignment_bias
        )
        integrated_rotation_deg += math.degrees(
            float(np.linalg.norm(raw_mean_corrected_gyro)) * dt
        )
        final_estimate = ekf.propagate(
            sample.sim_time_ms % CF_TIMESTAMP_MODULUS_MS,
            np.degrees(measured_gyro),
            np.asarray(sample.accel_m_s2) / GRAVITY_M_S2,
        )
        if dt > 0.0:
            baseline_quaternion = integrate_body_rate(
                baseline_quaternion, mean_corrected_gyro, dt
            )
        baseline_previous_gyro = measured_gyro
        previous_raw_gyro = raw_gyro
        previous_ms = sample.sim_time_ms
        force_norms_g.append(
            float(np.linalg.norm(sample.accel_m_s2) / GRAVITY_M_S2)
        )
        _canonical_hash_update(digest, {
            "type": "imu", "sim_time_ms": sample.sim_time_ms,
            "fused": asdict(final_estimate),
            "gyro_only_quaternion_wxyz": [
                float(item) for item in baseline_quaternion
            ],
        })
        odom = odom_by_ms.get(sample.sim_time_ms)
        if odom is None:
            continue
        # Position is the only odometry field passed into the estimator.
        final_estimate = ekf.update_extpos(odom.position_m)
        _canonical_hash_update(digest, {
            "type": "position_update", "sim_time_ms": sample.sim_time_ms,
            "fused": asdict(final_estimate),
            "gyro_only_quaternion_wxyz": [
                float(item) for item in baseline_quaternion
            ],
        })
        if odom.truth_quaternion_wxyz is not None:
            fused_errors.append(_quaternion_error_deg(
                final_estimate.quaternion_wxyz,
                odom.truth_quaternion_wxyz,
            ))
            baseline_errors.append(_quaternion_error_deg(
                baseline_quaternion, odom.truth_quaternion_wxyz,
            ))
            score_times_ms.append(sample.sim_time_ms)
    return {
        "estimator_output_sha256": digest.hexdigest(),
        "estimator_valid": bool(final_estimate.valid),
        "estimator_reason": final_estimate.reason,
        "position_update_count": int(final_estimate.position_update_count),
        "rejected_position_count": int(final_estimate.rejected_position_count),
        "fused_attitude": _attitude_metrics(fused_errors),
        "gyro_only_attitude": _attitude_metrics(baseline_errors),
        "truth_score_coverage_ms": (
            float(score_times_ms[-1] - score_times_ms[0])
            if len(score_times_ms) >= 2 else 0.0
        ),
        "integrated_raw_rotation_deg": float(integrated_rotation_deg),
        "specific_force_rms_g": (
            float(math.sqrt(float(np.mean(np.square(force_norms_g)))))
            if force_norms_g else 0.0
        ),
        "final_fused_state": asdict(final_estimate),
    }


def _gate(value, *, minimum=None, maximum=None) -> dict:
    if value is None or isinstance(value, bool):
        passed = False
    elif minimum is not None:
        passed = float(value) >= float(minimum)
    else:
        passed = float(value) <= float(maximum)
    result = {"passed": bool(passed), "value": value}
    if minimum is not None:
        result["minimum"] = minimum
    if maximum is not None:
        result["maximum"] = maximum
    return result


def evaluate_records(
        records: Sequence[Mapping], *,
        gyro_bias_stress_deg_s: Sequence[float] = (0.0, 0.0, 0.0),
        config: EvaluationConfig | None = None,
) -> dict:
    """Return a fail-closed JSON-serializable evaluation report."""
    config = config or EvaluationConfig()
    stress = _finite_vector(
        gyro_bias_stress_deg_s, 3, "gyro_bias_stress_deg_s"
    )
    trace = parse_records(records)
    alignment = _alignment_seed(trace, config)
    release_position_seed = _release_position_seed(trace, config)
    nominal = _run_estimator(trace, alignment, stress, config)

    deterministic_mutation = (math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0)
    mutated_trace = replace(trace, odom=tuple(
        replace(
            sample,
            velocity_m_s=(123.0, -456.0, 789.0),
            truth_quaternion_wxyz=deterministic_mutation,
        )
        for sample in trace.odom
    ))
    removed_trace = replace(trace, odom=tuple(
        replace(
            sample,
            velocity_m_s=(-987.0, 654.0, -321.0),
            truth_quaternion_wxyz=None,
        )
        for sample in trace.odom
    ))
    mutated_hash = _run_estimator(
        mutated_trace, alignment, stress, config
    )["estimator_output_sha256"]
    removed_hash = _run_estimator(
        removed_trace, alignment, stress, config
    )["estimator_output_sha256"]
    nominal_hash = nominal["estimator_output_sha256"]
    leakage_passed = nominal_hash == mutated_hash == removed_hash

    post_imu = [sample for sample in trace.imu
                if sample.sim_time_ns > trace.release_time_ns]
    post_odom = [sample for sample in trace.odom
                 if sample.sim_time_ns > trace.release_time_ns]
    release_seed = next(
        sample for sample in reversed(trace.odom)
        if sample.sim_time_ns <= trace.release_time_ns
    )
    release_seed_skew_ms = release_position_seed[
        "position_extrapolation_dt_ms"
    ]
    release_seed_attitude_error_deg = _quaternion_error_deg(
        alignment["quaternion_wxyz"],
        release_seed.truth_quaternion_wxyz,
    )
    first_position_delay_ms = (
        (post_odom[0].sim_time_ns - trace.release_time_ns)
        / SIM_TIME_NS_PER_CF_MS if post_odom else None
    )
    imu_gap_ms = _maximum_gap_ms(post_imu)
    odom_gap_ms = _maximum_gap_ms(post_odom)
    post_coverage_ms = (
        post_imu[-1].sim_time_ms - trace.release_time_ms if post_imu else 0.0
    )
    position_coverage_ms = (
        post_odom[-1].sim_time_ms - post_odom[0].sim_time_ms
        if len(post_odom) >= 2 else 0.0
    )
    position_displacement_m = (
        float(np.linalg.norm(
            np.asarray(post_odom[-1].position_m)
            - np.asarray(post_odom[0].position_m)
        )) if len(post_odom) >= 2 else 0.0
    )
    truth_attitude_excursion_deg = max(
        _quaternion_error_deg(
            sample.truth_quaternion_wxyz,
            release_seed.truth_quaternion_wxyz,
        )
        for sample in post_odom
    )
    imu_epochs = {sample.sim_time_ms for sample in post_imu}
    unmatched_position_epochs = sum(
        sample.sim_time_ms not in imu_epochs for sample in post_odom
    )
    attempted_updates = (
        nominal["position_update_count"] + nominal["rejected_position_count"]
    )
    rejection_fraction = (
        nominal["rejected_position_count"] / attempted_updates
        if attempted_updates else 1.0
    )
    fused = nominal["fused_attitude"]
    gyro_only = nominal["gyro_only_attitude"]
    if fused["rmse_deg"] is None or gyro_only["rmse_deg"] is None:
        rmse_improvement = None
        rmse_ratio = None
    else:
        rmse_improvement = gyro_only["rmse_deg"] - fused["rmse_deg"]
        rmse_ratio = (
            fused["rmse_deg"] / gyro_only["rmse_deg"]
            if gyro_only["rmse_deg"] > 1e-12 else None
        )
    stress_applied = float(np.linalg.norm(stress)) > 1e-12
    fused_p95_limit_deg = (
        config.max_stressed_fused_p95_deg if stress_applied
        else config.max_nominal_fused_p95_deg
    )
    if stress_applied:
        ab_improvement_gate = _gate(
            rmse_improvement, minimum=config.min_rmse_improvement_deg,
        )
        ab_ratio_gate = _gate(
            rmse_ratio, maximum=config.max_fused_to_gyro_rmse_ratio,
        )
        ab_improvement_gate["applicable"] = True
        ab_ratio_gate["applicable"] = True
    else:
        ab_improvement_gate = {
            "passed": True, "applicable": False,
            "reason": "no_deterministic_gyro_bias_stress_requested",
            "value": rmse_improvement,
        }
        ab_ratio_gate = {
            "passed": True, "applicable": False,
            "reason": "no_deterministic_gyro_bias_stress_requested",
            "value": rmse_ratio,
        }

    gates = {
        "metadata_schema": {"passed": True, "value": TRACE_SCHEMA},
        "position_only_contract": {"passed": True, "value": True},
        "common_sim_clock": {
            "passed": unmatched_position_epochs == 0,
            "unmatched_position_epochs": unmatched_position_epochs,
            "basis": EXPECTED_CLOCK_BASIS,
        },
        "alignment_coverage_ms": _gate(
            alignment["coverage_ms"],
            minimum=config.min_alignment_coverage_ms,
        ),
        "alignment_sample_count": _gate(
            alignment["sample_count"], minimum=config.min_alignment_samples,
        ),
        "alignment_accel_norm_error_g": _gate(
            alignment["max_accel_norm_error_g"],
            maximum=config.max_alignment_accel_norm_error_g,
        ),
        "alignment_mean_gyro_deg_s": _gate(
            alignment["mean_gyro_norm_deg_s"],
            maximum=config.max_alignment_mean_gyro_deg_s,
        ),
        "alignment_gyro_std_deg_s": _gate(
            alignment["gyro_std_norm_deg_s"],
            maximum=config.max_alignment_gyro_std_deg_s,
        ),
        "maximum_imu_gap_ms": _gate(
            imu_gap_ms, maximum=config.max_imu_gap_ms,
        ),
        "maximum_position_gap_ms": _gate(
            odom_gap_ms, maximum=config.max_position_gap_ms,
        ),
        "release_seed_skew_ms": _gate(
            release_seed_skew_ms,
            maximum=config.max_release_seed_skew_ms,
        ),
        "release_seed_attitude_error_deg": _gate(
            release_seed_attitude_error_deg,
            maximum=config.max_release_seed_attitude_error_deg,
        ),
        "first_post_release_position_delay_ms": _gate(
            first_position_delay_ms,
            maximum=config.max_position_gap_ms,
        ),
        "post_release_coverage_ms": _gate(
            post_coverage_ms, minimum=config.min_post_release_coverage_ms,
        ),
        "position_coverage_ms": _gate(
            position_coverage_ms, minimum=config.min_position_coverage_ms,
        ),
        "truth_score_sample_count": _gate(
            fused["sample_count"], minimum=config.min_truth_samples,
        ),
        "position_update_count": _gate(
            nominal["position_update_count"],
            minimum=config.min_position_updates,
        ),
        "position_rejection_fraction": _gate(
            rejection_fraction,
            maximum=config.max_position_rejection_fraction,
        ),
        "rotation_excitation_deg": _gate(
            nominal["integrated_raw_rotation_deg"],
            minimum=config.min_rotation_excitation_deg,
        ),
        "truth_attitude_excursion_deg": _gate(
            truth_attitude_excursion_deg,
            minimum=config.min_truth_attitude_excursion_deg,
        ),
        "position_displacement_m": _gate(
            position_displacement_m,
            minimum=config.min_position_displacement_m,
        ),
        "specific_force_rms_g": _gate(
            nominal["specific_force_rms_g"],
            minimum=config.min_specific_force_rms_g,
        ),
        "estimator_valid": {
            "passed": nominal["estimator_valid"],
            "reason": nominal["estimator_reason"],
        },
        "fused_rmse_deg": _gate(
            fused["rmse_deg"], maximum=config.max_fused_rmse_deg,
        ),
        "fused_p95_deg": _gate(
            fused["p95_deg"], maximum=fused_p95_limit_deg,
        ),
        "fused_final_deg": _gate(
            fused["final_deg"], maximum=config.max_fused_final_deg,
        ),
        "ab_rmse_improvement_deg": ab_improvement_gate,
        "ab_fused_to_gyro_rmse_ratio": ab_ratio_gate,
        "truth_leakage_canary": {"passed": leakage_passed},
    }
    failed_gates = sorted(name for name, gate in gates.items()
                          if not gate["passed"])
    release_raw_ms = trace.release_time_ms % CF_TIMESTAMP_MODULUS_MS
    return {
        "schema": REPORT_SCHEMA,
        "gate_verdict": "PASS" if not failed_gates else "FAIL",
        "passed": not failed_gates,
        "offline_only": True,
        "flight_commands_generated": False,
        "estimator": "PostReleaseInertialEkf",
        "process_input": "measured_body_specific_force_rotated_by_estimated_attitude",
        "command_history_used": False,
        "post_release_odometry_update_fields": ["position_m"],
        "release_state_seed": release_position_seed,
        "odom_position_use": (
            "release_position_and_world_frame_velocity_finite_difference;"
            "post_release_position_updates"
        ),
        "odom_velocity_use": "ignored_plant_diagnostics_only",
        "odom_quaternion_use": "scoring_only",
        "truth_fed_to_estimator": "position_only",
        "position_truth_fed_to_estimator": True,
        "velocity_truth_fed_to_estimator": False,
        "orientation_truth_fed_to_estimator": False,
        "metadata": trace.metadata,
        "clock_mapping": {
            "source_basis": EXPECTED_CLOCK_BASIS,
            "source_unit": "ns",
            "requires_integral_millisecond_epochs": True,
            "cf_timestamp_modulus_ms": CF_TIMESTAMP_MODULUS_MS,
            "formula": "(sim_time_ns / 1000000) mod 2^24",
            "release_sim_time_ms": trace.release_time_ms,
            "release_cf_timestamp_ms": release_raw_ms,
        },
        "gyro_bias_stress_deg_s": list(stress),
        "ab_improvement_gate_applicable": stress_applied,
        "alignment": alignment,
        "metrics": {
            **nominal,
            "maximum_imu_gap_ms": imu_gap_ms,
            "maximum_position_gap_ms": odom_gap_ms,
            "release_seed_skew_ms": release_seed_skew_ms,
            "release_seed_sim_time_ms": release_seed.sim_time_ms,
            "release_seed_attitude_error_deg": (
                release_seed_attitude_error_deg
            ),
            "first_post_release_position_delay_ms": first_position_delay_ms,
            "post_release_coverage_ms": post_coverage_ms,
            "position_coverage_ms": position_coverage_ms,
            "position_displacement_m": position_displacement_m,
            "truth_attitude_excursion_deg": truth_attitude_excursion_deg,
            "unmatched_position_epochs": unmatched_position_epochs,
            "position_rejection_fraction": rejection_fraction,
            "ab_rmse_improvement_deg": rmse_improvement,
            "ab_fused_to_gyro_rmse_ratio": rmse_ratio,
        },
        "truth_leakage_canary": {
            "passed": leakage_passed,
            "method": (
                "all odometry velocity and orientation fields mutated, then "
                "all orientations removed; estimator-output trajectories "
                "hashed independently of scores"
            ),
            "nominal_estimator_output_sha256": nominal_hash,
            "mutated_truth_estimator_output_sha256": mutated_hash,
            "removed_truth_estimator_output_sha256": removed_hash,
        },
        "thresholds": asdict(config),
        "gates": gates,
        "failed_gates": failed_gates,
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path, help="CrazySim JSONL trace")
    parser.add_argument(
        "--gyro-bias-stress-deg-s", nargs=3, type=float,
        metavar=("X", "Y", "Z"), default=(0.0, 0.0, 0.0),
        help="deterministic post-release gyro bias added to both A/B estimators",
    )
    parser.add_argument(
        "--output", type=Path,
        help="optional new JSON report path; existing paths are refused",
    )
    args = parser.parse_args(argv)
    if args.output is not None and args.output.exists():
        parser.error("output already exists; choose a new report path")
    try:
        records = read_jsonl(args.trace)
        report = evaluate_records(
            records, gyro_bias_stress_deg_s=args.gyro_bias_stress_deg_s
        )
        report["source"] = {
            "path": str(args.trace.resolve()), "sha256": _sha256(args.trace),
        }
        serialized = json.dumps(
            report, indent=2, sort_keys=True, allow_nan=False
        ) + "\n"
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x") as destination:
                destination.write(serialized)
        print(serialized, end="")
    except (OSError, TraceValidationError, ValueError) as error:
        parser.error(str(error))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
