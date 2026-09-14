"""Replay the contact-attitude shadow estimator from a LiveLogger JSON file.

The replay consumes raw Crazyflie IMU/state packets in their recorded order.  A
full-pose quaternion, when present, is used only after propagation to score the
estimate; it is never passed to the observer or the post-release EKF.

This tool is diagnostic-only.  It has no commander, setpoint, calibration-save,
deployment, or firmware-write path.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

from Interaction.contact_attitude_observer import (
    CF_TIMESTAMP_MODULUS_MS,
    ContactAttitudeConfig,
    quaternion_multiply,
)
from Interaction.contact_attitude_shadow import (
    ContactAttitudeShadow,
    ContactAttitudeShadowConfig,
)
from Interaction.log_manager import CfLogPacket, MocapFramePacket
from Interaction.post_release_inertial_ekf import (
    PostReleaseEkfConfig,
    rotation_matrix,
)


CONTACT_START_EVENTS = (
    "Contact Attitude Shadow Contact Candidate Started",
    "Contact Attitude Shadow Contact Started",
    "Translation Contact Start",
)
CONTACT_CANCEL_EVENTS = (
    "Contact Attitude Shadow Contact Candidate Cancelled",
)
CONTACT_CONFIRM_EVENTS = (
    "Contact Attitude Shadow Contact Confirmed",
)
CANDIDATE_START_EVENTS = (
    "Contact Attitude Shadow Candidate Started",
    "Potentiometer Release Candidate Started",
)
CANDIDATE_CANCEL_EVENTS = (
    "Contact Attitude Shadow Candidate Cancelled",
    "Potentiometer Release Candidate Cancelled",
)
RELEASE_EVENTS = (
    "Contact Attitude Shadow Released",
    "Potentiometer Release Coasting Started",
)
RELEASE_REQUEST_EVENTS = (
    "Contact Attitude Shadow Release Deferred",
)
TRUTH_TYPES = {
    "contact_attitude_truth", "ground_truth", "mocap_full_pose", "sim_truth",
}
TRUTH_GROUPS = {
    "CONTACT_ATTITUDE_TRUTH", "GROUND_TRUTH", "MOCAP_FULL_POSE", "ODOM_TRUTH",
    "SIM_TRUTH", "SIMULATOR_TRUTH",
}
LOGGED_SHADOW_COUNTERS = (
    "dropped_packets",
    "skipped_position_packets",
    "contained_failure_count",
    "drain_budget_exceeded_count",
    "release_budget_exceeded_count",
)


@dataclass(frozen=True)
class ReplayGateConfig:
    """Accuracy gates and clock joins for one offline replay."""

    release_tilt_error_max_deg: float = 1.0
    post_release_window_ms: float = 400.0
    post_release_p95_tilt_error_max_deg: float = 1.5
    drift_horizon_ms: float = 800.0
    relative_tilt_drift_max_deg: float = 1.0
    truth_join_tolerance_ms: float = 15.0
    drift_horizon_tolerance_ms: float = 25.0
    release_state_join_tolerance_ms: float = 15.0
    min_post_release_truth_samples: int = 20
    min_strict_position_updates: int = 10
    min_strict_position_update_epochs: int = 10
    min_strict_position_coverage_ms: float = 200.0
    max_strict_position_gap_ms: float = 50.0

    def __post_init__(self):
        positive = (
            self.release_tilt_error_max_deg,
            self.post_release_window_ms,
            self.post_release_p95_tilt_error_max_deg,
            self.drift_horizon_ms,
            self.relative_tilt_drift_max_deg,
            self.truth_join_tolerance_ms,
            self.drift_horizon_tolerance_ms,
            self.release_state_join_tolerance_ms,
            self.min_strict_position_coverage_ms,
            self.max_strict_position_gap_ms,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("all replay gate limits and time windows must be positive")
        if self.min_post_release_truth_samples < 2:
            raise ValueError("min_post_release_truth_samples must be at least two")
        if self.min_strict_position_updates < 1:
            raise ValueError("min_strict_position_updates must be positive")
        if self.min_strict_position_update_epochs < 2:
            raise ValueError(
                "min_strict_position_update_epochs must be at least two"
            )


@dataclass(frozen=True)
class _TimedQuaternion:
    record_index: int
    host_time_s: float | None
    cf_timestamp_ms: int | None
    quaternion_wxyz: tuple[float, float, float, float]
    source: str
    valid: bool = True
    reason: str | None = None


def _finite_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _raw_cf_timestamp(data: Mapping, record: Mapping | None = None):
    for source in (data, record or {}):
        for key in ("cf_timestamp_ms", "cfTimestampMs"):
            if key in source:
                value = _finite_float(source[key])
                if value is None or not value.is_integer():
                    return None
                result = int(value)
                return result if 0 <= result < CF_TIMESTAMP_MODULUS_MS else None
    return None


def _host_time(data: Mapping, record: Mapping | None = None):
    for source in (data, record or {}):
        for key in ("host_receive_time_s", "time", "timestamp_s"):
            if key in source:
                value = _finite_float(source[key])
                if value is not None:
                    return value
    return None


def _host_monotonic_time(data: Mapping, record: Mapping | None = None):
    for source in (data, record or {}):
        value = _finite_float(source.get("host_receive_monotonic_s"))
        if value is not None:
            return value
    timing = data.get("mocap_timing")
    if isinstance(timing, Mapping):
        value = _finite_float(timing.get("wait_return_monotonic_s"))
        if value is not None:
            return value
    return None


def _packet_sequence(data: Mapping, record: Mapping):
    for source in (data, record):
        for key in ("sequence", "cf_packet_sequence", "packet_sequence"):
            value = source.get(key)
            if isinstance(value, bool):
                continue
            try:
                result = int(value)
            except (TypeError, ValueError):
                continue
            if result >= 0:
                return result
    return None


def _vector(data: Mapping, keys: Sequence[str]):
    values = [_finite_float(data.get(key)) for key in keys]
    if any(value is None for value in values):
        return None
    return tuple(values)


def _vector_aliases(data: Mapping, *key_sets: Sequence[str]):
    for keys in key_sets:
        values = _vector(data, keys)
        if values is not None:
            return values
    return None


def _first_finite(data: Mapping, *keys: str):
    for key in keys:
        value = _finite_float(data.get(key))
        if value is not None:
            return value
    return None


def _optional_cf_timestamp(value):
    if isinstance(value, bool):
        return None
    numeric = _finite_float(value)
    if numeric is None or not numeric.is_integer():
        return None
    timestamp = int(numeric)
    return timestamp if 0 <= timestamp < CF_TIMESTAMP_MODULUS_MS else None


def _replay_cf_packet(
        *, sequence, group, cf_timestamp_ms, host_receive_time_s, data,
        host_receive_monotonic_s):
    """Rebuild a runtime packet without inventing source provenance."""
    basis = data.get('source_cf_timestamp_basis')
    if not isinstance(basis, str):
        basis = None
    return CfLogPacket(
        sequence=sequence,
        group=group,
        cf_timestamp_ms=cf_timestamp_ms,
        host_receive_time_s=host_receive_time_s,
        data=data,
        host_receive_monotonic_s=host_receive_monotonic_s,
        transport_cf_timestamp_ms=_optional_cf_timestamp(
            data.get('transport_cf_timestamp_ms')
        ),
        source_cf_timestamp_basis=basis,
        source_snapshot_atomic=(
            data.get('source_snapshot_atomic') is True
        ),
    )


def _normalize_quaternion(value):
    try:
        quaternion = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        return None
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        return None
    quaternion = quaternion / norm
    if quaternion[0] < 0.0:
        quaternion = -quaternion
    return tuple(float(component) for component in quaternion)


def _truth_quaternion(data: Mapping, *, frames_record: bool):
    """Return an explicitly labeled full-pose quaternion in WXYZ order."""
    for key in ("quaternion_wxyz", "quat_wxyz", "orientation_wxyz"):
        if key in data:
            return _normalize_quaternion(data[key]), f"{key}:wxyz"
    for key in ("quaternion_xyzw", "quat_xyzw", "orientation_xyzw"):
        if key in data:
            value = data[key]
            try:
                x, y, z, w = value
            except (TypeError, ValueError):
                return None, f"{key}:invalid"
            return _normalize_quaternion([w, x, y, z]), f"{key}:xyzw"
    orientation = data.get("orientation")
    if isinstance(orientation, Mapping):
        values = _vector(orientation, ("w", "x", "y", "z"))
        if values is not None:
            return _normalize_quaternion(values), "orientation_mapping:wxyz"
    dotted = _vector(data, (
        "orientation.w", "orientation.x", "orientation.y", "orientation.z",
    ))
    if dotted is not None:
        return _normalize_quaternion(dotted), "orientation_fields:wxyz"
    # Vicon/CrazySim frame quaternions use the cflib extpose XYZW convention.
    if frames_record or "quat" in data:
        value = data.get("quat")
        try:
            x, y, z, w = value
        except (TypeError, ValueError):
            return None, "quat:invalid"
        return _normalize_quaternion([w, x, y, z]), "quat:xyzw"
    return None, "quaternion_missing"


def _truth_sample(record, index):
    if not isinstance(record, Mapping):
        return None
    data = record.get("data")
    if not isinstance(data, Mapping):
        return None
    record_type = record.get("type")
    group = record.get("group")
    frames_record = record_type == "frames"
    explicitly_truth = (
        frames_record
        or record_type in TRUTH_TYPES
        or (record_type == "state" and group in TRUTH_GROUPS)
    )
    if not explicitly_truth:
        return None
    quaternion, convention = _truth_quaternion(data, frames_record=frames_record)
    if quaternion is None:
        return None
    host_time = _host_time(data, record)
    cf_timestamp = _raw_cf_timestamp(data, record)
    if host_time is None and cf_timestamp is None:
        return None
    return _TimedQuaternion(
        record_index=index,
        host_time_s=host_time,
        cf_timestamp_ms=cf_timestamp,
        quaternion_wxyz=quaternion,
        source=f"{record_type or 'unknown'}:{group or '-'}:{convention}",
    )


def quaternion_distance_deg(left, right):
    left_q = _normalize_quaternion(left)
    right_q = _normalize_quaternion(right)
    if left_q is None or right_q is None:
        raise ValueError("quaternion distance requires two finite quaternions")
    cosine = min(1.0, abs(float(np.dot(left_q, right_q))))
    return math.degrees(2.0 * math.acos(cosine))


def tilt_error_deg(left, right):
    """Yaw-invariant roll/pitch error: angle between the two body-Z axes."""
    left_z = rotation_matrix(left)[:, 2]
    right_z = rotation_matrix(right)[:, 2]
    cosine = float(np.clip(left_z @ right_z, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _quaternion_conjugate(quaternion):
    w, x, y, z = _normalize_quaternion(quaternion)
    return (w, -x, -y, -z)


def _relative_tilt_drift_deg(estimate_release, estimate_later,
                             truth_release, truth_later):
    """Compare body-Z attitude change after separately anchoring release."""
    estimate_relative = quaternion_multiply(
        _quaternion_conjugate(estimate_release), estimate_later
    )
    truth_relative = quaternion_multiply(
        _quaternion_conjugate(truth_release), truth_later
    )
    return tilt_error_deg(estimate_relative, truth_relative)


def _signed_timestamp_delta_ms(current, previous):
    delta = (int(current) - int(previous)) % CF_TIMESTAMP_MODULUS_MS
    if delta >= CF_TIMESTAMP_MODULUS_MS // 2:
        delta -= CF_TIMESTAMP_MODULUS_MS
    return delta


def _elapsed_ms(sample, release):
    if sample.cf_timestamp_ms is not None and release.cf_timestamp_ms is not None:
        return float(_signed_timestamp_delta_ms(
            sample.cf_timestamp_ms, release.cf_timestamp_ms
        ))
    if sample.host_time_s is not None and release.host_time_s is not None:
        return 1000.0 * (sample.host_time_s - release.host_time_s)
    return None


def _clock_distance_ms(left, right):
    if left.cf_timestamp_ms is not None and right.cf_timestamp_ms is not None:
        return abs(float(_signed_timestamp_delta_ms(
            left.cf_timestamp_ms, right.cf_timestamp_ms
        )))
    if left.host_time_s is not None and right.host_time_s is not None:
        return abs(1000.0 * (left.host_time_s - right.host_time_s))
    return None


def _nearest_by_clock(samples, target, tolerance_ms):
    candidates = []
    for sample in samples:
        distance = _clock_distance_ms(sample, target)
        if distance is not None:
            candidates.append((distance, sample.record_index, sample))
    if not candidates:
        return None, None
    distance, _index, sample = min(candidates, key=lambda item: item[:2])
    return (sample, distance) if distance <= tolerance_ms else (None, distance)


def _nearest_by_cf_clock(samples, target, tolerance_ms):
    """Join only samples sharing the logged Crazyflie/simulation clock."""
    if target.cf_timestamp_ms is None:
        return None, None
    candidates = []
    for sample in samples:
        if sample.cf_timestamp_ms is None:
            continue
        distance = abs(float(_signed_timestamp_delta_ms(
            sample.cf_timestamp_ms, target.cf_timestamp_ms
        )))
        candidates.append((distance, sample.record_index, sample))
    if not candidates:
        return None, None
    distance, _index, sample = min(candidates, key=lambda item: item[:2])
    return (sample, distance) if distance <= tolerance_ms else (None, distance)


def _elapsed_cf_ms(sample, release):
    if sample.cf_timestamp_ms is None or release.cf_timestamp_ms is None:
        return None
    return float(_signed_timestamp_delta_ms(
        sample.cf_timestamp_ms, release.cf_timestamp_ms
    ))


def _select_event_names(records):
    names = {
        record.get("name") for record in records
        if isinstance(record, Mapping) and record.get("type") == "events"
    }

    def preferred(options):
        return next((name for name in options if name in names), None)

    return {
        "contact_start": preferred(CONTACT_START_EVENTS),
        "contact_cancel": preferred(CONTACT_CANCEL_EVENTS),
        "contact_confirm": preferred(CONTACT_CONFIRM_EVENTS),
        "candidate_start": preferred(CANDIDATE_START_EVENTS),
        "candidate_cancel": preferred(CANDIDATE_CANCEL_EVENTS),
        "release": preferred(RELEASE_EVENTS),
    }


def _timestamp_stream_metrics(timestamps, host_times, expected_period_ms):
    duplicates = 0
    backward = 0
    gaps_over_expected = 0
    timestamp_gap_slots = 0
    maximum_gap = None
    clock_increment_errors = []
    for index in range(1, len(timestamps)):
        previous = timestamps[index - 1]
        current = timestamps[index]
        if previous is None or current is None:
            continue
        delta = _signed_timestamp_delta_ms(current, previous)
        if delta == 0:
            duplicates += 1
            continue
        if delta < 0:
            backward += 1
            continue
        maximum_gap = delta if maximum_gap is None else max(maximum_gap, delta)
        if delta > expected_period_ms:
            gaps_over_expected += 1
            timestamp_gap_slots += max(
                0, int(round(delta / expected_period_ms)) - 1
            )
        previous_host = host_times[index - 1]
        current_host = host_times[index]
        if previous_host is not None and current_host is not None:
            host_delta_ms = 1000.0 * (current_host - previous_host)
            if host_delta_ms >= 0.0:
                clock_increment_errors.append(host_delta_ms - delta)
    valid_host_gaps = [
        1000.0 * (right - left)
        for left, right in zip(host_times, host_times[1:])
        if left is not None and right is not None and right >= left
    ]
    # Millisecond timestamps can quantize two genuine high-rate callbacks onto
    # one tick and then show a two-tick step. Offset those duplicate slots before
    # calling the remainder an inferred deficit. Only sequence gaps can confirm
    # packet drops.
    inferred_missing = max(0, timestamp_gap_slots - duplicates)
    return {
        "sample_count": len(timestamps),
        "timestamp_missing_count": sum(value is None for value in timestamps),
        "duplicate_timestamp_count": duplicates,
        "backward_timestamp_count": backward,
        "gap_over_expected_count": gaps_over_expected,
        "timestamp_gap_slot_count": timestamp_gap_slots,
        "inferred_missing_sample_count": inferred_missing,
        "inferred_missing_definition": (
            "timestamp_gap_slots_minus_duplicate_timestamp_slots; "
            "not a confirmed packet-drop count"
        ),
        "max_cf_gap_ms": None if maximum_gap is None else float(maximum_gap),
        "max_host_callback_gap_ms": (
            None if not valid_host_gaps else float(max(valid_host_gaps))
        ),
        "p95_abs_host_minus_cf_increment_ms": (
            None if not clock_increment_errors else float(np.percentile(
                np.abs(clock_increment_errors), 95
            ))
        ),
    }


def _gate(value, limit, unsupported_reason=None):
    if value is None:
        return {
            "status": "UNSUPPORTED", "value": None, "limit": float(limit),
            "passed": None, "reason": unsupported_reason,
        }
    passed = bool(value <= limit)
    return {
        "status": "PASS" if passed else "FAIL",
        "value": float(value), "limit": float(limit), "passed": passed,
        "reason": None,
    }


def _minimum_evidence_gate(value, minimum, unsupported_reason):
    """Require enough evidence without turning absent evidence into failure."""
    if value is None or value < minimum:
        return {
            "status": "UNSUPPORTED", "value": value,
            "limit": minimum, "passed": None,
            "reason": unsupported_reason,
        }
    return {
        "status": "PASS", "value": value, "limit": minimum,
        "passed": True, "reason": None,
    }


def _fresh_state(state, release_marker, tolerance_ms):
    if state is None:
        return False, None
    distance = _clock_distance_ms(state["clock"], release_marker)
    return bool(distance is not None and distance <= tolerance_ms), distance


def _begin_contact(shadow):
    """Use the corrected contact-start API, with compatibility for this branch."""
    for method_name in ("begin_contact", "start_contact"):
        method = getattr(shadow, method_name, None)
        if method is not None:
            return method()
    # Before the lifecycle rename, this method performed the same transition.
    return shadow.begin_contact_candidate()


def _episode_metrics(episode, truth_samples, config):
    release = episode["release_estimate"]
    estimates = episode["estimates"]
    baseline_counters = episode["position_counter_baseline"]
    final_counters = episode["final_shadow_position_counters"]
    final_filter = episode.get("final_filter_snapshot") or {}

    def counter_delta(final_value, initial_value):
        final_value = _finite_float(final_value)
        initial_value = _finite_float(initial_value)
        if (
                final_value is None or initial_value is None
                or not final_value.is_integer()
                or not initial_value.is_integer()):
            return None
        return int(final_value - initial_value)

    strict_position_updates = counter_delta(
        final_counters.get("strict"), baseline_counters.get("strict")
    )
    approximate_position_updates = counter_delta(
        final_counters.get("approximate"),
        baseline_counters.get("approximate"),
    )
    accepted_position_updates = counter_delta(
        final_filter.get("position_update_count"),
        baseline_counters.get("accepted"),
    )
    rejected_position_updates = counter_delta(
        final_filter.get("rejected_position_count"),
        baseline_counters.get("rejected"),
    )
    position_counter_consistent = (
        strict_position_updates is not None
        and approximate_position_updates is not None
        and accepted_position_updates is not None
        and rejected_position_updates is not None
        and min(
            strict_position_updates,
            approximate_position_updates,
            accepted_position_updates,
            rejected_position_updates,
        ) >= 0
        and accepted_position_updates
        == strict_position_updates + approximate_position_updates
        and episode.get("position_counter_monotonic") is True
    )
    strict_update_epochs = []
    for item in episode["strict_position_update_trace"]:
        timestamp = item.get("cf_timestamp_ms")
        if timestamp is None:
            continue
        if timestamp not in strict_update_epochs:
            strict_update_epochs.append(timestamp)
    strict_update_elapsed_ms = [
        float(_signed_timestamp_delta_ms(timestamp, release.cf_timestamp_ms))
        for timestamp in strict_update_epochs
        if release.cf_timestamp_ms is not None
    ]
    strict_epoch_ordered = all(
        current > previous
        for previous, current in zip(
            strict_update_elapsed_ms, strict_update_elapsed_ms[1:]
        )
    )
    if not strict_epoch_ordered:
        position_counter_consistent = False
    strict_position_coverage_ms = (
        None if len(strict_update_elapsed_ms) < 2 else
        strict_update_elapsed_ms[-1] - strict_update_elapsed_ms[0]
    )
    strict_position_gaps_ms = [
        current - previous
        for previous, current in zip(
            strict_update_elapsed_ms, strict_update_elapsed_ms[1:]
        )
    ]
    strict_position_max_gap_ms = (
        None if not strict_position_gaps_ms else
        max(strict_position_gaps_ms)
    )
    release_truth, release_truth_distance = _nearest_by_cf_clock(
        truth_samples, release, config.truth_join_tolerance_ms
    )
    release_tilt_error = None
    release_full_error = None
    if release_truth is not None:
        release_tilt_error = tilt_error_deg(
            release.quaternion_wxyz, release_truth.quaternion_wxyz
        )
        release_full_error = quaternion_distance_deg(
            release.quaternion_wxyz, release_truth.quaternion_wxyz
        )

    post_pairs = []
    for truth in truth_samples:
        elapsed = _elapsed_cf_ms(truth, release)
        if elapsed is None or elapsed < 0.0 or elapsed > config.post_release_window_ms:
            continue
        estimate, distance = _nearest_by_cf_clock(
            estimates, truth, config.truth_join_tolerance_ms
        )
        if estimate is None:
            continue
        post_pairs.append({
            "elapsed_ms": float(elapsed),
            "join_distance_ms": float(distance),
            "tilt_error_deg": tilt_error_deg(
                estimate.quaternion_wxyz, truth.quaternion_wxyz
            ),
            "full_attitude_error_deg": quaternion_distance_deg(
                estimate.quaternion_wxyz, truth.quaternion_wxyz
            ),
        })
    post_p95 = None
    post_reason = None
    if len(post_pairs) >= config.min_post_release_truth_samples:
        post_p95 = float(np.percentile(
            [pair["tilt_error_deg"] for pair in post_pairs], 95
        ))
    else:
        post_reason = (
            "insufficient_joined_common_clock_orientation_truth_samples:"
            f"{len(post_pairs)}<{config.min_post_release_truth_samples}"
        )

    horizon_truth = None
    horizon_elapsed = None
    horizon_distance = None
    horizon_candidates = []
    for truth in truth_samples:
        elapsed = _elapsed_cf_ms(truth, release)
        if elapsed is not None:
            horizon_candidates.append((
                abs(elapsed - config.drift_horizon_ms), truth.record_index,
                elapsed, truth,
            ))
    if horizon_candidates:
        horizon_distance, _index, horizon_elapsed, candidate = min(
            horizon_candidates, key=lambda item: item[:2]
        )
        if horizon_distance <= config.drift_horizon_tolerance_ms:
            horizon_truth = candidate
    horizon_estimate = None
    horizon_join_distance = None
    if horizon_truth is not None:
        horizon_estimate, horizon_join_distance = _nearest_by_cf_clock(
            estimates, horizon_truth, config.truth_join_tolerance_ms
        )
    drift = None
    full_drift = None
    drift_reason = None
    if release_truth is None:
        drift_reason = "release_common_clock_orientation_truth_not_joined"
    elif horizon_truth is None:
        drift_reason = "orientation_truth_missing_near_drift_horizon"
    elif horizon_estimate is None:
        drift_reason = "estimate_missing_near_drift_horizon"
    else:
        drift = _relative_tilt_drift_deg(
            release.quaternion_wxyz, horizon_estimate.quaternion_wxyz,
            release_truth.quaternion_wxyz, horizon_truth.quaternion_wxyz,
        )
        estimate_relative = quaternion_multiply(
            _quaternion_conjugate(release.quaternion_wxyz),
            horizon_estimate.quaternion_wxyz,
        )
        truth_relative = quaternion_multiply(
            _quaternion_conjugate(release_truth.quaternion_wxyz),
            horizon_truth.quaternion_wxyz,
        )
        full_drift = quaternion_distance_deg(estimate_relative, truth_relative)

    release_reason = (
        None if release_truth is not None else
        "common_clock_orientation_truth_not_joined_at_release"
    )
    gates = {
        "release_tilt_error": _gate(
            release_tilt_error, config.release_tilt_error_max_deg, release_reason
        ),
        "post_release_400ms_p95_tilt_error": _gate(
            post_p95, config.post_release_p95_tilt_error_max_deg, post_reason
        ),
        "relative_tilt_drift_at_800ms": _gate(
            drift, config.relative_tilt_drift_max_deg, drift_reason
        ),
        "strict_position_update_count": _minimum_evidence_gate(
            strict_position_updates,
            config.min_strict_position_updates,
            "insufficient_strict_position_updates",
        ),
        "strict_position_update_epoch_count": _minimum_evidence_gate(
            len(strict_update_epochs),
            config.min_strict_position_update_epochs,
            "insufficient_unique_strict_position_update_epochs",
        ),
        "strict_position_update_coverage_ms": _minimum_evidence_gate(
            strict_position_coverage_ms,
            config.min_strict_position_coverage_ms,
            "insufficient_strict_position_update_time_coverage",
        ),
    }
    if strict_position_max_gap_ms is None:
        gates["strict_position_update_max_gap_ms"] = {
            "status": "UNSUPPORTED", "value": None,
            "limit": config.max_strict_position_gap_ms,
            "passed": None, "reason": "strict_position_update_gap_missing",
        }
    elif strict_position_max_gap_ms > config.max_strict_position_gap_ms:
        gates["strict_position_update_max_gap_ms"] = {
            "status": "UNSUPPORTED", "value": strict_position_max_gap_ms,
            "limit": config.max_strict_position_gap_ms,
            "passed": None, "reason": "strict_position_update_gap_exceeded",
        }
    else:
        gates["strict_position_update_max_gap_ms"] = {
            "status": "PASS", "value": strict_position_max_gap_ms,
            "limit": config.max_strict_position_gap_ms,
            "passed": True, "reason": None,
        }
    gates["position_update_counter_consistency"] = {
        "status": "PASS" if position_counter_consistent else "FAIL",
        "value": position_counter_consistent,
        "limit": True,
        "passed": position_counter_consistent,
        "reason": None if position_counter_consistent else (
            "position_update_counters_invalid_or_nonmonotonic"
        ),
    }
    no_rejections = rejected_position_updates == 0
    gates["rejected_position_updates"] = {
        "status": "PASS" if no_rejections else "FAIL",
        "value": rejected_position_updates,
        "limit": 0,
        "passed": no_rejections,
        "reason": None if no_rejections else "position_update_rejected",
    }
    no_approximate = approximate_position_updates == 0
    gates["approximate_position_updates"] = {
        "status": "PASS" if no_approximate else "UNSUPPORTED",
        "value": approximate_position_updates,
        "limit": 0,
        "passed": True if no_approximate else None,
        "reason": None if no_approximate else (
            "position_updates_lack_common_device_clock"
        ),
    }
    filter_valid = bool(episode["release_valid"])
    invalid_within_horizon = next((
        sample for sample in estimates
        if not sample.valid
        and (_elapsed_ms(sample, release) is not None)
        and 0.0 <= _elapsed_ms(sample, release) <= config.drift_horizon_ms
    ), None)
    if invalid_within_horizon is not None:
        filter_valid = False
    gates["filter_valid_through_800ms"] = {
        "status": "PASS" if filter_valid else "FAIL",
        "value": filter_valid,
        "limit": True,
        "passed": filter_valid,
        "reason": (
            None if filter_valid else
            episode.get("release_invalid_reason")
            or invalid_within_horizon.reason
            or "filter_invalid"
        ),
    }
    statuses = [gate["status"] for gate in gates.values()]
    verdict = (
        "FAIL" if "FAIL" in statuses else
        "UNSUPPORTED" if "UNSUPPORTED" in statuses else "PASS"
    )
    return {
        "episode_index": episode["episode_index"],
        "release_record_index": release.record_index,
        "release_host_time_s": release.host_time_s,
        "release_cf_timestamp_ms": release.cf_timestamp_ms,
        "release_state_sources": episode["release_state_sources"],
        "release_state_join_distance_ms": episode["release_state_join_distance_ms"],
        "release_valid": episode["release_valid"],
        "release_invalid_reason": episode.get("release_invalid_reason"),
        "release_tilt_error_deg": release_tilt_error,
        "release_full_attitude_error_deg": release_full_error,
        "release_truth_join_distance_ms": release_truth_distance,
        "release_truth_source": None if release_truth is None else release_truth.source,
        "post_release_400ms_joined_truth_count": len(post_pairs),
        "post_release_400ms_p95_tilt_error_deg": post_p95,
        "drift_truth_elapsed_ms": horizon_elapsed,
        "drift_truth_target_distance_ms": horizon_distance,
        "drift_estimate_join_distance_ms": horizon_join_distance,
        "relative_tilt_drift_at_800ms_deg": drift,
        "relative_full_attitude_drift_at_800ms_deg": full_drift,
        "estimate_sample_count": len(estimates),
        "strict_position_update_count": strict_position_updates,
        "approximate_position_update_count": approximate_position_updates,
        "accepted_position_update_count": accepted_position_updates,
        "rejected_position_update_count": rejected_position_updates,
        "unique_strict_position_update_epoch_count": len(
            strict_update_epochs
        ),
        "strict_position_update_coverage_ms": strict_position_coverage_ms,
        "strict_position_update_max_gap_ms": strict_position_max_gap_ms,
        "final_filter_snapshot": episode["final_filter_snapshot"],
        "gates": gates,
        "gate_verdict": verdict,
    }


def analyze_records(
        records, config: ReplayGateConfig | None = None,
        observer_config: ContactAttitudeConfig | None = None,
        shadow_config: ContactAttitudeShadowConfig | None = None,
        ekf_config: PostReleaseEkfConfig | None = None,
):
    """Replay records and return a tri-state PASS/FAIL/UNSUPPORTED report."""
    if not isinstance(records, list):
        raise ValueError("LiveLogger input must be a JSON array")
    config = config or ReplayGateConfig()
    event_names = _select_event_names(records)
    truth_samples = [
        sample for index, record in enumerate(records)
        if (sample := _truth_sample(record, index)) is not None
    ]
    common_clock_truth_samples = [
        sample for sample in truth_samples
        if sample.cf_timestamp_ms is not None
    ]
    explicit_packet_monotonic_clock = any(
        _host_monotonic_time(record.get("data", {}), record) is not None
        for record in records
        if isinstance(record, Mapping)
        and record.get("type") in ("state", "frames")
        and isinstance(record.get("data"), Mapping)
    )
    replay_device_clock = {'raw_ms': None, 'unwrapped_ms': None}

    def replay_monotonic_time(data, record):
        value = _host_monotonic_time(data, record)
        if value is not None:
            return value
        if explicit_packet_monotonic_clock:
            return None
        # Legacy all-wall-clock logs remain replayable, but one replay never
        # mixes a wall timestamp with explicit monotonic packet timestamps.
        wall_time = _host_time(data, record)
        if wall_time is not None:
            return wall_time
        raw_ms = _raw_cf_timestamp(data, record)
        if raw_ms is None:
            return None
        previous_raw = replay_device_clock['raw_ms']
        if previous_raw is None:
            unwrapped_ms = float(raw_ms)
        else:
            delta_ms = _signed_timestamp_delta_ms(raw_ms, previous_raw)
            # This is only a causal scheduler for device-clock-only replay. It
            # is never reported as a measured host/capture time. Independent
            # log streams may arrive a few samples out of order, so do not let
            # that synthetic scheduler run backwards.
            unwrapped_ms = (
                replay_device_clock['unwrapped_ms'] + max(0.0, float(delta_ms))
            )
        replay_device_clock['raw_ms'] = int(raw_ms)
        replay_device_clock['unwrapped_ms'] = unwrapped_ms
        return unwrapped_ms / 1000.0

    replay_clock = [0.0]
    shadow = ContactAttitudeShadow(
        config=shadow_config or ContactAttitudeShadowConfig(queue_capacity=8192),
        observer_config=observer_config,
        ekf_config=ekf_config,
        clock=lambda: replay_clock[0],
    )

    counts = {
        "GYRO_1KHZ": 0, "ACC_ALIGN": 0, "POS_ACC": 0, "VEL_ORI": 0,
        "CONTACT_STATE_SEED": 0,
        "MOCAP_POSITION": 0,
        "contact_start": 0, "contact_cancel": 0, "contact_confirm": 0,
        "candidate_start": 0,
        "candidate_cancel": 0, "release_request": 0, "release": 0,
    }
    malformed_record_count = 0
    missing_packet_field_count = 0
    lifecycle_results = []
    latest_position = None
    latest_velocity = None
    latest_state_seed = None
    gyro_timestamps, gyro_host_times = [], []
    accel_timestamps, accel_host_times = [], []
    position_timestamps, velocity_timestamps, state_seed_timestamps = [], [], []
    all_packet_sequences = []
    logged_shadow_counter_maxima = {
        field: 0 for field in LOGGED_SHADOW_COUNTERS
    }
    logged_shadow_counter_invalid = set()
    logged_shadow_fatal_reasons = set()
    episodes = []
    active_episode = None

    for index, record in enumerate(records):
        if not isinstance(record, Mapping):
            malformed_record_count += 1
            continue
        data = record.get("data")
        if not isinstance(data, Mapping):
            data = {}
        record_type = record.get("type")
        if record_type == "contact_attitude_shadow":
            fatal_reason = data.get("fatal_reason")
            if fatal_reason is not None:
                logged_shadow_fatal_reasons.add(str(fatal_reason))
            for field in LOGGED_SHADOW_COUNTERS:
                value = data.get(field)
                if value is None:
                    continue
                try:
                    numeric = int(value)
                    if (
                            isinstance(value, bool)
                            or numeric < 0
                            or float(value) != numeric):
                        raise ValueError
                    logged_shadow_counter_maxima[field] = max(
                        logged_shadow_counter_maxima[field], numeric
                    )
                except (TypeError, ValueError):
                    logged_shadow_counter_invalid.add(field)

        if record_type == "frames":
            try:
                raw_position = data.get("tvec")
                position = tuple(float(value) for value in raw_position)
            except (TypeError, ValueError):
                position = None
            if (
                position is None
                or len(position) != 3
                or not all(math.isfinite(value) for value in position)
            ):
                missing_packet_field_count += 1
                continue
            host_time = _host_time(data, record)
            host_monotonic_time = replay_monotonic_time(data, record)
            cf_timestamp = _raw_cf_timestamp(data, record)
            if host_monotonic_time is None:
                missing_packet_field_count += 1
                continue
            replay_clock[0] = host_monotonic_time
            packet_wall_time = (
                host_monotonic_time if host_time is None else host_time
            )
            counts["MOCAP_POSITION"] += 1
            clock = _TimedQuaternion(
                index, host_time, cf_timestamp,
                (1.0, 0.0, 0.0, 0.0), "frames:tvec",
            )
            latest_position = {"value": position, "clock": clock}
            frame_sequence = data.get("mocap_frame_sequence", index)
            try:
                frame_sequence = int(frame_sequence)
            except (TypeError, ValueError):
                frame_sequence = index
            shadow.enqueue_mocap_frame(MocapFramePacket(
                sequence=frame_sequence,
                group="frames",
                host_receive_time_s=packet_wall_time,
                data=data,
                cf_timestamp_ms=cf_timestamp,
                host_receive_monotonic_s=host_monotonic_time,
            ))
            continue

        if record_type == "state":
            group = record.get("group")
            sequence = _packet_sequence(data, record)
            if sequence is not None:
                all_packet_sequences.append(sequence)
            if group not in counts:
                continue
            counts[group] += 1
            cf_timestamp = _raw_cf_timestamp(data, record)
            host_time = _host_time(data, record)
            host_monotonic_time = replay_monotonic_time(data, record)
            if host_monotonic_time is None:
                missing_packet_field_count += 1
                continue
            replay_clock[0] = host_monotonic_time
            packet_wall_time = (
                host_monotonic_time if host_time is None else host_time
            )
            clock = _TimedQuaternion(
                index, host_time, cf_timestamp, (1.0, 0.0, 0.0, 0.0),
                f"state:{group}",
            )
            if group == "GYRO_1KHZ":
                gyro_timestamps.append(cf_timestamp)
                gyro_host_times.append(host_monotonic_time)
                gyro = _vector_aliases(
                    data,
                    ('contactImu.gx', 'contactImu.gy', 'contactImu.gz'),
                    ("contactGyro.x", "contactGyro.y", "contactGyro.z"),
                    ("gyro.x", "gyro.y", "gyro.z"),
                )
                if cf_timestamp is None or gyro is None:
                    missing_packet_field_count += 1
                    continue
                packed_acceleration = _vector(
                    data,
                    ('contactImu.ax', 'contactImu.ay', 'contactImu.az'),
                )
                packed_position = _vector(
                    data,
                    ('contactImu.px', 'contactImu.py', 'contactImu.pz'),
                )
                packed_velocity = _vector(
                    data,
                    ('contactImu.vx', 'contactImu.vy', 'contactImu.vz'),
                )
                packed_fields_present = any(
                    str(key).startswith('contactImu.') for key in data
                )
                if packed_fields_present:
                    if (
                        packed_acceleration is None
                        or packed_position is None
                        or packed_velocity is None
                    ):
                        missing_packet_field_count += 1
                        continue
                    # One physical CRTP record supplies all three logical
                    # streams from one producer epoch.
                    counts['ACC_ALIGN'] += 1
                    counts['CONTACT_STATE_SEED'] += 1
                    accel_timestamps.append(cf_timestamp)
                    accel_host_times.append(host_monotonic_time)
                    state_seed_timestamps.append(cf_timestamp)
                    latest_state_seed = {
                        'position': packed_position,
                        'velocity': packed_velocity,
                        'clock': clock,
                    }
                shadow.enqueue_packet(_replay_cf_packet(
                    sequence=index if sequence is None else sequence,
                    group=group,
                    cf_timestamp_ms=cf_timestamp,
                    host_receive_time_s=packet_wall_time,
                    data=data,
                    host_receive_monotonic_s=host_monotonic_time,
                ))
                try:
                    snapshot = shadow.drain()
                except (TypeError, ValueError) as error:
                    lifecycle_results.append({
                        "record_index": index, "action": "gyro",
                        "valid": False, "reason": str(error),
                    })
                    continue
                if active_episode is not None:
                    current_shadow_counters = {
                        "strict": snapshot.get(
                            "strict_position_time_update_count"
                        ),
                        "approximate": snapshot.get(
                            "approximate_position_time_update_count"
                        ),
                    }
                    previous_shadow_counters = active_episode[
                        "final_shadow_position_counters"
                    ]
                    for field in ("strict", "approximate"):
                        current = current_shadow_counters[field]
                        previous = previous_shadow_counters[field]
                        if (
                                not isinstance(current, int)
                                or isinstance(current, bool)
                                or current < previous):
                            active_episode["position_counter_monotonic"] = False
                    if (
                            isinstance(current_shadow_counters["strict"], int)
                            and current_shadow_counters["strict"]
                            > previous_shadow_counters["strict"]):
                        active_episode["strict_position_update_trace"].append({
                            "record_index": index,
                            "strict_count": current_shadow_counters["strict"],
                            "cf_timestamp_ms": snapshot.get(
                                "last_strict_position_update_cf_timestamp_ms"
                            ),
                        })
                    active_episode["final_shadow_position_counters"] = (
                        current_shadow_counters
                    )
                    estimate_data = snapshot.get("post_release_ekf")
                    if estimate_data is not None:
                        quaternion = _normalize_quaternion(
                            estimate_data.get("quaternion_wxyz")
                        )
                        if quaternion is not None:
                            sample = _TimedQuaternion(
                                index, host_time, cf_timestamp, quaternion,
                                "post_release_inertial_ekf",
                                bool(estimate_data.get("valid")),
                                estimate_data.get("reason"),
                            )
                            elapsed = _elapsed_ms(
                                sample, active_episode["release_estimate"]
                            )
                            retained_horizon_ms = (
                                config.drift_horizon_ms
                                + config.drift_horizon_tolerance_ms
                                + config.truth_join_tolerance_ms
                            )
                            if elapsed is None or elapsed <= retained_horizon_ms:
                                active_episode["estimates"].append(sample)
                            active_episode["final_filter_snapshot"] = estimate_data
                continue
            if group == "ACC_ALIGN":
                accel_timestamps.append(cf_timestamp)
                accel_host_times.append(host_monotonic_time)
                acceleration = _vector_aliases(
                    data,
                    (
                        "contactAccel.x", "contactAccel.y",
                        "contactAccel.z",
                    ),
                    ("acc.x", "acc.y", "acc.z"),
                )
                yaw = _first_finite(
                    data, "contactAccel.yaw", "stateEstimate.yaw"
                )
                if cf_timestamp is None or acceleration is None or yaw is None:
                    missing_packet_field_count += 1
                    continue
                shadow.enqueue_packet(_replay_cf_packet(
                    sequence=index if sequence is None else sequence,
                    group=group,
                    cf_timestamp_ms=cf_timestamp,
                    host_receive_time_s=packet_wall_time,
                    data=data,
                    host_receive_monotonic_s=host_monotonic_time,
                ))
                continue
            if group == "POS_ACC":
                position_timestamps.append(cf_timestamp)
                position = _vector(data, (
                    "stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
                ))
                if position is None:
                    missing_packet_field_count += 1
                else:
                    if cf_timestamp is None:
                        missing_packet_field_count += 1
                    else:
                        # Preserve the runtime adapter's timestamp-causal
                        # position queue; never fuse by host arrival time here.
                        shadow.enqueue_packet(_replay_cf_packet(
                            sequence=index if sequence is None else sequence,
                            group=group,
                            cf_timestamp_ms=cf_timestamp,
                            host_receive_time_s=packet_wall_time,
                            data=data,
                            host_receive_monotonic_s=host_monotonic_time,
                        ))
                continue
            if group == "VEL_ORI":
                velocity_timestamps.append(cf_timestamp)
                velocity = _vector(data, (
                    "stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz",
                ))
                if velocity is None:
                    missing_packet_field_count += 1
                else:
                    latest_velocity = {"value": velocity, "clock": clock}
                    if cf_timestamp is None:
                        missing_packet_field_count += 1
                    else:
                        shadow.enqueue_packet(_replay_cf_packet(
                            sequence=index if sequence is None else sequence,
                            group=group,
                            cf_timestamp_ms=cf_timestamp,
                            host_receive_time_s=packet_wall_time,
                            data=data,
                            host_receive_monotonic_s=host_monotonic_time,
                        ))
                continue
            if group == "CONTACT_STATE_SEED":
                state_seed_timestamps.append(cf_timestamp)
                position = _vector_aliases(
                    data,
                    ("contactSeed.x", "contactSeed.y", "contactSeed.z"),
                    (
                        "stateEstimate.x", "stateEstimate.y",
                        "stateEstimate.z",
                    ),
                )
                velocity = _vector_aliases(
                    data,
                    (
                        "contactSeed.vx", "contactSeed.vy",
                        "contactSeed.vz",
                    ),
                    (
                        "stateEstimate.vx", "stateEstimate.vy",
                        "stateEstimate.vz",
                    ),
                )
                if cf_timestamp is None or position is None or velocity is None:
                    missing_packet_field_count += 1
                else:
                    latest_state_seed = {
                        "position": position,
                        "velocity": velocity,
                        "clock": clock,
                    }
                    shadow.enqueue_packet(_replay_cf_packet(
                        sequence=index if sequence is None else sequence,
                        group=group,
                        cf_timestamp_ms=cf_timestamp,
                        host_receive_time_s=packet_wall_time,
                        data=data,
                        host_receive_monotonic_s=host_monotonic_time,
                    ))
                continue

        if record_type != "events":
            continue
        name = record.get("name")
        event_time = _host_time(data, record)
        event_monotonic_time = replay_monotonic_time(data, record)
        if event_monotonic_time is not None:
            replay_clock[0] = event_monotonic_time
        event_cf_timestamp = _raw_cf_timestamp(data, record)
        event_marker = _TimedQuaternion(
            index, event_time, event_cf_timestamp,
            (1.0, 0.0, 0.0, 0.0), "event",
        )
        if (event_names["contact_start"] is not None
                and name == event_names["contact_start"]):
            counts["contact_start"] += 1
            try:
                result = _begin_contact(shadow)
            except (TypeError, ValueError) as error:
                result = {"valid": False, "invalid_reason": str(error)}
            lifecycle_results.append({
                "record_index": index, "action": "contact_start",
                "valid": bool(result.get("valid")),
                "reason": result.get("invalid_reason"),
            })
            continue
        if (event_names["contact_cancel"] is not None
                and name == event_names["contact_cancel"]):
            counts["contact_cancel"] += 1
            try:
                result = shadow.cancel_contact_candidate()
            except (TypeError, ValueError) as error:
                result = {"valid": False, "invalid_reason": str(error)}
            lifecycle_results.append({
                "record_index": index, "action": "contact_cancel",
                # Returning to ALIGNING is intentionally not a valid estimate
                # yet, but it is a successful rollback of a false onset edge.
                "valid": result.get("invalid_reason") is None,
                "reason": result.get("invalid_reason"),
            })
            continue
        if (event_names["contact_confirm"] is not None
                and name == event_names["contact_confirm"]):
            counts["contact_confirm"] += 1
            method = getattr(shadow, "confirm_contact", None)
            result = shadow.snapshot() if method is None else method()
            lifecycle_results.append({
                "record_index": index, "action": "contact_confirm",
                "valid": bool(result.get("valid")),
                "reason": result.get("invalid_reason"),
            })
            continue
        if (event_names["candidate_start"] is not None
                and name == event_names["candidate_start"]):
            counts["candidate_start"] += 1
            lifecycle_results.append({
                "record_index": index, "action": "candidate_start",
                "valid": True, "reason": None,
            })
            continue
        if (event_names["candidate_cancel"] is not None
                and name == event_names["candidate_cancel"]):
            # A cancelled release candidate is still physical contact.  The
            # contact-period gyro observer must continue without re-alignment.
            counts["candidate_cancel"] += 1
            lifecycle_results.append({
                "record_index": index, "action": "candidate_cancel",
                "valid": True, "reason": "contact_observer_continues",
            })
            continue
        if name in RELEASE_REQUEST_EVENTS:
            counts["release_request"] += 1
            request = data.get("release_request")
            if not isinstance(request, Mapping):
                request = {}
            host_position = request.get("host_loop_position", (0.0, 0.0, 0.0))
            host_velocity = request.get("host_loop_velocity", (0.0, 0.0, 0.0))
            try:
                result = shadow.release(
                    host_position,
                    host_velocity,
                    interaction_direction=request.get("interaction_direction"),
                    interaction_direction_source=request.get(
                        "interaction_direction_source"
                    ),
                    active_setpoint=request.get("active_setpoint"),
                    effective_command_at_state=request.get(
                        "effective_command_at_state"
                    ),
                    pending_transport_commands=request.get(
                        "pending_transport_commands"
                    ),
                    inner_loop_tail=request.get("inner_loop_tail"),
                    release_event_monotonic_s=request.get(
                        "release_event_monotonic_s"
                    ),
                )
            except (TypeError, ValueError) as error:
                result = {"valid": False, "invalid_reason": str(error)}
            anchored = result.get("pending_release_cf_timestamp_ms") is not None
            lifecycle_results.append({
                "record_index": index,
                "action": "release_request",
                "valid": anchored,
                "reason": (
                    "release_epoch_frozen_pending_state"
                    if anchored else result.get("invalid_reason")
                ),
            })
            continue
        if (event_names["release"] is None
                or name != event_names["release"]):
            continue
        counts["release"] += 1
        # If the event has no clock of its own, anchor it to the latest raw gyro.
        observer_before_release = shadow.snapshot().get("observer") or {}
        if event_marker.cf_timestamp_ms is None:
            event_marker = _TimedQuaternion(
                index, event_marker.host_time_s,
                observer_before_release.get("cf_timestamp_ms"),
                event_marker.quaternion_wxyz, event_marker.source,
            )
        state_seed_fresh, state_seed_distance = _fresh_state(
            latest_state_seed, event_marker,
            config.release_state_join_tolerance_ms,
        )
        if not state_seed_fresh:
            lifecycle_results.append({
                "record_index": index, "action": "release",
                "valid": False,
                "reason": "fresh_CONTACT_STATE_SEED_required_at_release",
            })
            continue
        release_metadata = data.get("release_snapshot")
        if not isinstance(release_metadata, Mapping):
            release_metadata = {}
        try:
            result = shadow.release(
                latest_state_seed["position"],
                latest_state_seed["velocity"],
                interaction_direction=release_metadata.get("interaction_direction"),
                interaction_direction_source=release_metadata.get(
                    "interaction_direction_source"
                ),
                active_setpoint=release_metadata.get("active_setpoint"),
                pending_transport_commands=release_metadata.get(
                    "pending_transport_commands"
                ),
                inner_loop_tail=release_metadata.get("inner_loop_tail"),
                release_event_monotonic_s=release_metadata.get(
                    "release_event_monotonic_s"
                ),
            )
        except (TypeError, ValueError) as error:
            result = {"valid": False, "invalid_reason": str(error)}
        observer = result.get("observer") or {}
        release_snapshot = result.get("release_snapshot")
        if not isinstance(release_snapshot, Mapping):
            release_snapshot = {}
        quaternion = _normalize_quaternion(
            release_snapshot.get("gyro_quaternion_wxyz")
            or observer.get("quaternion_wxyz")
        )
        if quaternion is None:
            lifecycle_results.append({
                "record_index": index, "action": "release",
                "valid": False,
                "reason": result.get("invalid_reason") or "release_quaternion_missing",
            })
            continue
        release_estimate = _TimedQuaternion(
            record_index=index,
            host_time_s=event_marker.host_time_s,
            cf_timestamp_ms=(
                release_snapshot.get("cf_timestamp_ms")
                if release_snapshot.get("cf_timestamp_ms") is not None
                else observer.get("cf_timestamp_ms")
            ),
            quaternion_wxyz=quaternion,
            source="contact_gyro_observer",
            valid=bool(result.get("valid")),
            reason=result.get("invalid_reason") or observer.get("reason"),
        )
        active_episode = {
            "episode_index": len(episodes),
            "release_estimate": release_estimate,
            "estimates": [release_estimate],
            "release_valid": bool(result.get("valid")),
            "release_invalid_reason": result.get("invalid_reason"),
            "release_state_sources": {
                "position": release_snapshot.get("position_source"),
                "velocity": release_snapshot.get("velocity_source"),
            },
            "release_state_join_distance_ms": {
                "position": release_snapshot.get("position_seed_skew_ms"),
                "velocity": release_snapshot.get(
                    "velocity_seed_skew_ms", state_seed_distance
                ),
            },
            "final_filter_snapshot": result.get("post_release_ekf"),
            "position_counter_baseline": {
                "strict": result.get(
                    "strict_position_time_update_count", 0
                ),
                "approximate": result.get(
                    "approximate_position_time_update_count", 0
                ),
                "accepted": (result.get("post_release_ekf") or {}).get(
                    "position_update_count", 0
                ),
                "rejected": (result.get("post_release_ekf") or {}).get(
                    "rejected_position_count", 0
                ),
            },
            "final_shadow_position_counters": {
                "strict": result.get(
                    "strict_position_time_update_count", 0
                ),
                "approximate": result.get(
                    "approximate_position_time_update_count", 0
                ),
            },
            "position_counter_monotonic": True,
            "strict_position_update_trace": [],
        }
        episodes.append(active_episode)
        lifecycle_results.append({
            "record_index": index, "action": "release",
            "valid": bool(result.get("valid")),
            "reason": result.get("invalid_reason"),
        })

    sequence_gaps = 0
    sequence_missing = 0
    sequence_non_monotonic = 0
    for previous, current in zip(all_packet_sequences, all_packet_sequences[1:]):
        if current <= previous:
            sequence_non_monotonic += 1
        elif current > previous + 1:
            sequence_gaps += 1
            sequence_missing += current - previous - 1
    sequence_status = (
        "SUPPORTED" if len(all_packet_sequences) >= 2 else "UNSUPPORTED"
    )
    final_shadow_snapshot = shadow.snapshot()
    data_quality = {
        "record_count": len(records),
        "malformed_record_count": malformed_record_count,
        "missing_required_packet_field_count": missing_packet_field_count,
        "logged_shadow_dropped_packets": logged_shadow_counter_maxima[
            "dropped_packets"
        ],
        "logged_shadow_skipped_position_packets": (
            logged_shadow_counter_maxima["skipped_position_packets"]
        ),
        "logged_shadow_contained_failure_count": (
            logged_shadow_counter_maxima["contained_failure_count"]
        ),
        "logged_shadow_drain_budget_exceeded_count": (
            logged_shadow_counter_maxima["drain_budget_exceeded_count"]
        ),
        "logged_shadow_release_budget_exceeded_count": (
            logged_shadow_counter_maxima["release_budget_exceeded_count"]
        ),
        "logged_shadow_counter_invalid_fields": sorted(
            logged_shadow_counter_invalid
        ),
        "logged_shadow_fatal_reasons": sorted(logged_shadow_fatal_reasons),
        "replay_shadow_dropped_packets": final_shadow_snapshot.get(
            "dropped_packets", 0
        ),
        "replay_skipped_position_packets": final_shadow_snapshot.get(
            "skipped_position_packets"
        ),
        "replay_last_position_measurement_skew_ms": final_shadow_snapshot.get(
            "last_position_measurement_skew_ms"
        ),
        "replay_contained_failure_count": final_shadow_snapshot.get(
            "contained_failure_count", 0
        ),
        "replay_drain_budget_exceeded_count": final_shadow_snapshot.get(
            "drain_budget_exceeded_count", 0
        ),
        "replay_release_budget_exceeded_count": final_shadow_snapshot.get(
            "release_budget_exceeded_count", 0
        ),
        "replay_fatal_reason": final_shadow_snapshot.get("fatal_reason"),
        "packet_sequence": {
            "status": sequence_status,
            "sample_count": len(all_packet_sequences),
            "gap_count": None if sequence_status == "UNSUPPORTED" else sequence_gaps,
            "missing_packet_count": (
                None if sequence_status == "UNSUPPORTED" else sequence_missing
            ),
            "non_monotonic_count": (
                None if sequence_status == "UNSUPPORTED" else sequence_non_monotonic
            ),
            "reason": (
                None if sequence_status == "SUPPORTED" else
                "LiveLogger records do not contain packet sequence numbers"
            ),
        },
        "gyro_stream": _timestamp_stream_metrics(
            gyro_timestamps, gyro_host_times, 1.0
        ),
        "accelerometer_stream": _timestamp_stream_metrics(
            accel_timestamps, accel_host_times, 10.0
        ),
        "truth_sample_count": len(truth_samples),
        "common_cf_clock_truth_sample_count": len(
            common_clock_truth_samples
        ),
        "orientation_truth_common_clock_required_for_gates": True,
        "truth_sources": sorted({sample.source for sample in truth_samples}),
    }
    episode_reports = [
        _episode_metrics(episode, common_clock_truth_samples, config)
        for episode in episodes
    ]
    for report, episode in zip(episode_reports, episodes):
        # Capture the final EKF state after all packets, not merely the last
        # truth-paired packet.
        report["final_filter_snapshot"] = episode["final_filter_snapshot"]

    def aggregate_max(field):
        values = [
            episode[field] for episode in episode_reports
            if episode.get(field) is not None
        ]
        return None if not values else float(max(values))

    aggregate_metrics = {
        "release_tilt_error_max_deg": aggregate_max(
            "release_tilt_error_deg"
        ),
        "post_release_400ms_p95_tilt_error_max_deg": aggregate_max(
            "post_release_400ms_p95_tilt_error_deg"
        ),
        "relative_tilt_drift_at_800ms_max_deg": aggregate_max(
            "relative_tilt_drift_at_800ms_deg"
        ),
        "max_gyro_cf_gap_ms": data_quality["gyro_stream"]["max_cf_gap_ms"],
        "inferred_gyro_missing_sample_count": data_quality["gyro_stream"][
            "inferred_missing_sample_count"
        ],
        "confirmed_packet_drop_count": (
            data_quality["packet_sequence"]["missing_packet_count"]
        ),
        "logged_shadow_dropped_packets": logged_shadow_counter_maxima[
            "dropped_packets"
        ],
        "replay_shadow_dropped_packets": data_quality[
            "replay_shadow_dropped_packets"
        ],
    }

    unsupported_reasons = []
    if counts["GYRO_1KHZ"] == 0:
        unsupported_reasons.append("GYRO_1KHZ_stream_missing")
    if counts["ACC_ALIGN"] == 0:
        unsupported_reasons.append("ACC_ALIGN_stream_missing")
    if counts["POS_ACC"] == 0:
        unsupported_reasons.append("POS_ACC_stream_missing")
    if counts["CONTACT_STATE_SEED"] == 0:
        unsupported_reasons.append("CONTACT_STATE_SEED_stream_missing")
    if counts["MOCAP_POSITION"] == 0:
        unsupported_reasons.append("mocap_position_stream_missing")
    if counts["VEL_ORI"] == 0:
        unsupported_reasons.append("VEL_ORI_stream_missing")
    if any(timestamp is None for timestamp in (
            gyro_timestamps + accel_timestamps
            + position_timestamps + velocity_timestamps
            + state_seed_timestamps)):
        unsupported_reasons.append("raw_Crazyflie_timestamp_missing")
    if counts["contact_start"] == 0:
        unsupported_reasons.append("contact_start_lifecycle_event_missing")
    if counts["release"] == 0:
        unsupported_reasons.append("confirmed_release_lifecycle_event_missing")
    if sequence_status == "UNSUPPORTED":
        unsupported_reasons.append("packet_sequence_metadata_missing")
    if not episodes:
        unsupported_reasons.append("no_release_episode_could_be_replayed")
    if not truth_samples:
        unsupported_reasons.append("withheld_full_pose_orientation_missing")
    elif not common_clock_truth_samples:
        unsupported_reasons.append("orientation_truth_common_cf_clock_missing")

    episode_statuses = [episode["gate_verdict"] for episode in episode_reports]
    known_failure = (
        "FAIL" in episode_statuses
        or any(logged_shadow_counter_maxima.values())
        or bool(logged_shadow_counter_invalid)
        or bool(logged_shadow_fatal_reasons)
        or data_quality["replay_shadow_dropped_packets"] > 0
        or (data_quality["replay_skipped_position_packets"] or 0) > 0
        or data_quality["replay_contained_failure_count"] > 0
        or data_quality["replay_drain_budget_exceeded_count"] > 0
        or data_quality["replay_release_budget_exceeded_count"] > 0
        or data_quality["replay_fatal_reason"] is not None
        or sequence_gaps > 0
        or sequence_non_monotonic > 0
        or data_quality["gyro_stream"]["backward_timestamp_count"] > 0
        or data_quality["accelerometer_stream"][
            "backward_timestamp_count"
        ] > 0
        or malformed_record_count > 0
        or missing_packet_field_count > 0
        or any(not result["valid"] for result in lifecycle_results)
    )
    schema_blockers = {
        "GYRO_1KHZ_stream_missing",
            "ACC_ALIGN_stream_missing",
            "POS_ACC_stream_missing",
            "CONTACT_STATE_SEED_stream_missing",
        "mocap_position_stream_missing",
        "VEL_ORI_stream_missing",
        "raw_Crazyflie_timestamp_missing",
        "contact_start_lifecycle_event_missing",
        "confirmed_release_lifecycle_event_missing",
    }
    # A legacy log that never recorded the required raw fields cannot falsify
    # the estimator; classify it as unsupported instead of turning missing
    # evidence into a numerical failure.
    if schema_blockers.intersection(unsupported_reasons):
        verdict = "UNSUPPORTED"
    elif known_failure:
        verdict = "FAIL"
    elif unsupported_reasons or "UNSUPPORTED" in episode_statuses:
        verdict = "UNSUPPORTED"
    elif episode_reports and all(status == "PASS" for status in episode_statuses):
        verdict = "PASS"
    else:
        verdict = "UNSUPPORTED"

    return {
        "schema_version": 1,
        "offline_only": True,
        "command_authority": False,
        "flight_commands_generated": False,
        "truth_usage": (
            "orientation_evaluation_only_never_fed_to_shadow_filter;"
            "shared_mocap_position_sensor_correlation_remains"
        ),
        "truth_fed_to_filter": False,
        "velocity_policy": (
            "onboard_EKF_velocity_release_seed_then_measured_accelerometer_"
            "propagation_plus_position_only_extpos_updates"
        ),
        "command_history_used_for_state_reconstruction": False,
        "lifecycle_event_names": event_names,
        "lifecycle_counts": counts,
        "lifecycle_results": lifecycle_results,
        "data_quality": data_quality,
        "aggregate_metrics": aggregate_metrics,
        "gate_thresholds": {
            "release_tilt_error_max_deg": config.release_tilt_error_max_deg,
            "post_release_window_ms": config.post_release_window_ms,
            "post_release_p95_tilt_error_max_deg": (
                config.post_release_p95_tilt_error_max_deg
            ),
            "drift_horizon_ms": config.drift_horizon_ms,
            "relative_tilt_drift_max_deg": config.relative_tilt_drift_max_deg,
            "truth_join_tolerance_ms": config.truth_join_tolerance_ms,
            "drift_horizon_tolerance_ms": config.drift_horizon_tolerance_ms,
            "min_post_release_truth_samples": (
                config.min_post_release_truth_samples
            ),
            "min_strict_position_updates": (
                config.min_strict_position_updates
            ),
            "min_strict_position_update_epochs": (
                config.min_strict_position_update_epochs
            ),
            "min_strict_position_coverage_ms": (
                config.min_strict_position_coverage_ms
            ),
            "max_strict_position_gap_ms": (
                config.max_strict_position_gap_ms
            ),
        },
        "episodes": episode_reports,
        "unsupported_reasons": sorted(set(unsupported_reasons)),
        "gate_verdict": verdict,
        "passed": True if verdict == "PASS" else False if verdict == "FAIL" else None,
    }


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="LiveLogger JSON array")
    parser.add_argument(
        "--output", type=Path,
        help="optional new JSON report path; existing files are refused",
    )
    args = parser.parse_args(argv)
    if args.output is not None and args.output.exists():
        parser.error("output already exists; choose a new report path")
    try:
        with args.log.open() as source:
            records = json.load(source)
        report = analyze_records(records)
    except (OSError, json.JSONDecodeError, ValueError) as error:
        parser.error(str(error))
    report["source"] = {
        "path": str(args.log.resolve()), "sha256": _sha256(args.log),
    }
    serialized = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output is None:
        print(serialized, end="")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized)
        print(f"{report['gate_verdict']}: {args.output}")
    return 0 if report["gate_verdict"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
