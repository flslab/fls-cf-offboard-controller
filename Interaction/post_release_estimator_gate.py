"""Fail-closed state-eligibility gate for the post-release inertial EKF.

This module has no commander dependency and cannot grant command authority.
An eligible result only says that one immutable estimator snapshot has the
required provenance, causal timing, observability evidence, and bounded
uncertainty. A caller must independently pass all command-path safety gates.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import numbers
from typing import Mapping, Optional

import numpy as np

from Interaction.contact_attitude_experiment import (
    AUTHORITY_RELEASE_CLOCK_MAPPING_BASES,
    CONTACT_ATTITUDE_PROTOCOL_VERSION,
    INERTIAL_POSITION,
    RELEASE_EVENT_TIME_SOURCE,
)
from Interaction.contact_attitude_observer import CF_TIMESTAMP_MODULUS_MS
from Interaction.contact_attitude_shadow import (
    DEFAULT_IMU_QUALITY_PROVENANCE_ID,
    POSITION_TIMESTAMP_UNCERTAINTY_ACCOUNTING,
    POST_RELEASE_ANGULAR_RATE_SOURCE,
    POST_RELEASE_ORIENTATION_CONVENTION,
    POST_RELEASE_STATE_FRAME,
)
from Interaction.log_manager import (
    CONTACT_SOURCE_AUTHORITY_MAX_TRANSPORT_SKEW_MS,
    CONTACT_SOURCE_TIMESTAMP_BASIS,
    TRUSTED_MOCAP_CF_TIMESTAMP_BASES,
)


@dataclass(frozen=True)
class PostReleaseEstimatorGateConfig:
    enabled: bool = False
    max_ekf_age_ms: float = 5.0
    max_position_age_ms: float = 30.0
    max_current_host_receive_age_s: float = 0.030
    # Host callback ordering is diagnostic only.  Physical release matching is
    # exact on the mapped CF source epoch below; this bounds only the observed
    # callback-arrival separation between the independent transports.
    max_release_epoch_skew_ms: float = 20.0
    min_release_confirmation_dwell_ms: float = 20.0
    max_release_confirmation_dwell_ms: float = 500.0
    max_release_confirmation_clock_disagreement_ms: float = 10.0
    min_position_updates: int = 3
    min_position_update_span_ms: float = 30.0
    min_post_release_device_time_coverage_ms: float = 40.0
    min_inertial_propagation_samples: int = 20
    max_position_rejection_fraction: float = 0.25
    max_imu_gap_ms: float = 5.0
    max_position_std_m: float = 0.025
    max_velocity_std_m_s: float = 0.20
    max_attitude_std_deg: float = 5.0
    max_gyro_bias_std_deg_s: float = 1.0
    max_accel_bias_std_m_s2: float = 0.10
    min_tilt_variance_reduction_rad2: float = 1e-10
    # No safe motion-dependent R inflation is implemented yet.  Consequently
    # the only authority-grade position clock uncertainty is exact zero.
    max_position_timestamp_uncertainty_ms: float = 0.0

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError("enabled must be boolean")
        finite_positive = (
            self.max_ekf_age_ms,
            self.max_position_age_ms,
            self.max_current_host_receive_age_s,
            self.max_release_epoch_skew_ms,
            self.min_release_confirmation_dwell_ms,
            self.max_release_confirmation_dwell_ms,
            self.max_release_confirmation_clock_disagreement_ms,
            self.min_position_update_span_ms,
            self.min_post_release_device_time_coverage_ms,
            self.max_imu_gap_ms,
            self.max_position_std_m,
            self.max_velocity_std_m_s,
            self.max_attitude_std_deg,
            self.max_gyro_bias_std_deg_s,
            self.max_accel_bias_std_m_s2,
            self.min_tilt_variance_reduction_rad2,
        )
        if any(
            isinstance(value, bool)
            or not isinstance(value, numbers.Real)
            or not math.isfinite(float(value))
            or float(value) <= 0.0
            for value in finite_positive
        ):
            raise ValueError(
                "post-release estimator limits must be finite and positive"
            )
        if (
            self.max_release_confirmation_dwell_ms
            < self.min_release_confirmation_dwell_ms
        ):
            raise ValueError(
                "maximum release confirmation dwell must not be below minimum"
            )
        for name, value in (
            ("min_position_updates", self.min_position_updates),
            ("min_inertial_propagation_samples",
             self.min_inertial_propagation_samples),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, numbers.Integral)
                or int(value) < 1
            ):
                raise ValueError(f"{name} must be a positive integer")
        rejection_fraction = self.max_position_rejection_fraction
        if (
            isinstance(rejection_fraction, bool)
            or not isinstance(rejection_fraction, numbers.Real)
            or not math.isfinite(float(rejection_fraction))
            or not 0.0 <= float(rejection_fraction) <= 1.0
        ):
            raise ValueError(
                "max_position_rejection_fraction must be in [0, 1]"
            )
        position_time_uncertainty = self.max_position_timestamp_uncertainty_ms
        if (
            isinstance(position_time_uncertainty, bool)
            or not isinstance(position_time_uncertainty, numbers.Real)
            or not math.isfinite(float(position_time_uncertainty))
            or float(position_time_uncertainty) < 0.0
        ):
            raise ValueError(
                'max_position_timestamp_uncertainty_ms must be finite and '
                'nonnegative'
            )


@dataclass(frozen=True)
class PostReleaseEstimatorGateDecision:
    estimator_control_eligible: bool
    reason: str
    detail: Optional[str]
    gate_grants_command_authority: bool
    ekf_age_ms: Optional[float]
    position_age_ms: Optional[float]
    current_host_receive_age_s: Optional[float]
    position_std_m: Optional[tuple[float, float, float]]
    velocity_std_m_s: Optional[tuple[float, float, float]]
    attitude_std_deg: Optional[tuple[float, float, float]]
    attitude_variance_reduction_rad2: Optional[
        tuple[float, float, float]
    ]
    gyro_bias_std_deg_s: Optional[tuple[float, float, float]]
    accel_bias_std_m_s2: Optional[tuple[float, float, float]]
    post_release_device_time_coverage_ms: Optional[float]
    position_update_span_ms: Optional[float]
    inertial_propagation_samples: int
    accepted_position_updates: int
    rejected_position_updates: int


def _decision(
        eligible, reason, *, detail=None, ekf_age_ms=None,
        position_age_ms=None, current_host_receive_age_s=None,
        position_std_m=None, velocity_std_m_s=None, attitude_std_deg=None,
        attitude_variance_reduction_rad2=None,
        gyro_bias_std_deg_s=None, accel_bias_std_m_s2=None,
        post_release_device_time_coverage_ms=None,
        position_update_span_ms=None, inertial_propagation_samples=0,
        accepted=0, rejected=0):
    return PostReleaseEstimatorGateDecision(
        estimator_control_eligible=bool(eligible),
        reason=str(reason),
        detail=None if detail is None else str(detail),
        gate_grants_command_authority=False,
        ekf_age_ms=ekf_age_ms,
        position_age_ms=position_age_ms,
        current_host_receive_age_s=current_host_receive_age_s,
        position_std_m=position_std_m,
        velocity_std_m_s=velocity_std_m_s,
        attitude_std_deg=attitude_std_deg,
        attitude_variance_reduction_rad2=(
            attitude_variance_reduction_rad2
        ),
        gyro_bias_std_deg_s=gyro_bias_std_deg_s,
        accel_bias_std_m_s2=accel_bias_std_m_s2,
        post_release_device_time_coverage_ms=(
            post_release_device_time_coverage_ms
        ),
        position_update_span_ms=position_update_span_ms,
        inertial_propagation_samples=int(inertial_propagation_samples),
        accepted_position_updates=int(accepted),
        rejected_position_updates=int(rejected),
    )


def _finite_number(value):
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _nonempty_string(value):
    if not isinstance(value, str):
        return None
    result = value.strip()
    return result if result else None


def _nonnegative_integer(value):
    if isinstance(value, bool) or not isinstance(value, numbers.Integral):
        return None
    result = int(value)
    return result if result >= 0 else None


def _validated_epoch(raw_value, unwrapped_value):
    raw = _nonnegative_integer(raw_value)
    unwrapped = _nonnegative_integer(unwrapped_value)
    if raw is None or unwrapped is None:
        return None, "timestamp_invalid_type_or_sign"
    if raw >= CF_TIMESTAMP_MODULUS_MS:
        return None, "timestamp_out_of_range"
    if unwrapped % CF_TIMESTAMP_MODULUS_MS != raw:
        return None, "timestamp_raw_unwrapped_mismatch"
    return (raw, unwrapped), None


def _source_transport_skew_ms(transport_value, source_value):
    transport = _nonnegative_integer(transport_value)
    source = _nonnegative_integer(source_value)
    if (
        transport is None or source is None
        or transport >= CF_TIMESTAMP_MODULUS_MS
        or source >= CF_TIMESTAMP_MODULUS_MS
    ):
        return None
    delta = (transport - source) % CF_TIMESTAMP_MODULUS_MS
    if delta >= CF_TIMESTAMP_MODULUS_MS // 2:
        delta -= CF_TIMESTAMP_MODULUS_MS
    return int(delta)


def _unwrapped_uint32_elapsed_ms(later_value, earlier_value):
    """Return the causal unwrapped Arduino-millis delta across at most one wrap."""
    later = _nonnegative_integer(later_value)
    earlier = _nonnegative_integer(earlier_value)
    modulus = 1 << 32
    if (
        later is None or earlier is None
        or later >= modulus or earlier >= modulus
    ):
        return None
    delta = (later - earlier) % modulus
    if delta >= modulus // 2:
        return None
    return float(delta)


def evaluate_post_release_estimator(
        snapshot, *, now_cf_timestamp_ms, now_timestamp_basis,
        now_unwrapped_cf_timestamp_ms=None,
        now_host_receive_age_s=None,
        config: PostReleaseEstimatorGateConfig | None = None,
) -> PostReleaseEstimatorGateDecision:
    """Return a fail-closed, explicitly non-authorizing state verdict.

    ``now_*`` must describe ``post_release_control_epoch`` from the same
    immutable snapshot. A VEL_ORI transport tick is not interchangeable with
    this producer-latched contact-IMU source epoch.
    """
    config = config or PostReleaseEstimatorGateConfig()
    if not config.enabled:
        return _decision(False, "disabled")
    if not isinstance(snapshot, Mapping):
        return _decision(False, "snapshot_missing")
    if (
        snapshot.get("shadow_only") is not True
        or snapshot.get("command_authority") is not False
    ):
        return _decision(False, "shadow_ownership_contract_missing")
    if (
        snapshot.get("protocol_version") != CONTACT_ATTITUDE_PROTOCOL_VERSION
        or snapshot.get("mode") != INERTIAL_POSITION
    ):
        return _decision(False, "snapshot_protocol_or_mode_untrusted")
    if snapshot.get("valid") is not True:
        return _decision(False, "shadow_estimator_invalid")
    alignment_metrics = snapshot.get("alignment_gate_metrics")
    if (
        not isinstance(alignment_metrics, Mapping)
        or alignment_metrics.get("passed") is not True
        or snapshot.get("alignment_yaw_source")
        not in ("onboard_ekf_yaw", "configured_nominal_yaw")
    ):
        return _decision(False, "yaw_initialization_provenance_untrusted")
    certified_yaw_deg = _finite_number(
        snapshot.get('absolute_yaw_reference_yaw_deg')
    )
    alignment_yaw_deg = _finite_number(
        snapshot.get('alignment_nominal_yaw_deg')
    )
    if (
        snapshot.get('absolute_yaw_reference_certified') is not True
        or _nonempty_string(
            snapshot.get('absolute_yaw_reference_certificate_id')
        ) is None
        or certified_yaw_deg is None
    ):
        return _decision(False, 'absolute_yaw_reference_uncertified')
    yaw_difference_deg = (
        (alignment_yaw_deg - certified_yaw_deg + 180.0) % 360.0 - 180.0
        if alignment_yaw_deg is not None else None
    )
    if (
        snapshot.get('alignment_yaw_source') != 'configured_nominal_yaw'
        or alignment_yaw_deg is None
        or not math.isclose(yaw_difference_deg, 0.0, abs_tol=1e-9)
    ):
        return _decision(False, 'absolute_yaw_reference_not_bound_to_alignment')

    current_epoch = snapshot.get("post_release_control_epoch")
    if not isinstance(current_epoch, Mapping):
        return _decision(False, "current_control_epoch_missing")
    if (
        now_timestamp_basis != CONTACT_SOURCE_TIMESTAMP_BASIS
        or current_epoch.get("timestamp_basis")
        != CONTACT_SOURCE_TIMESTAMP_BASIS
        or current_epoch.get("source_snapshot_atomic") is not True
    ):
        return _decision(False, "current_state_timestamp_untrusted")
    now_epoch, error = _validated_epoch(
        now_cf_timestamp_ms, now_unwrapped_cf_timestamp_ms
    )
    if error:
        return _decision(False, f"current_{error}")
    snapshot_epoch, error = _validated_epoch(
        current_epoch.get("cf_timestamp_ms"),
        current_epoch.get("unwrapped_timestamp_ms"),
    )
    if error:
        return _decision(False, f"snapshot_current_{error}")
    if now_epoch != snapshot_epoch:
        return _decision(False, "current_control_epoch_mismatch")
    current_transport_skew_ms = _source_transport_skew_ms(
        current_epoch.get('transport_cf_timestamp_ms'),
        snapshot_epoch[0],
    )
    reported_current_transport_skew_ms = _finite_number(
        current_epoch.get('transport_minus_source_timestamp_ms')
    )
    if (
        current_transport_skew_ms is None
        or reported_current_transport_skew_ms is None
        or not math.isclose(
            reported_current_transport_skew_ms,
            current_transport_skew_ms,
            abs_tol=1e-9,
        )
        or current_transport_skew_ms < 0
        or current_transport_skew_ms
        > CONTACT_SOURCE_AUTHORITY_MAX_TRANSPORT_SKEW_MS
    ):
        return _decision(False, 'current_source_transport_skew_untrusted')
    supplied_host_age = _finite_number(now_host_receive_age_s)
    snapshot_host_age = _finite_number(
        current_epoch.get("host_receive_age_s")
    )
    if (
        supplied_host_age is None or snapshot_host_age is None
        or supplied_host_age < 0.0 or snapshot_host_age < 0.0
    ):
        return _decision(False, "current_host_receive_age_invalid")
    effective_host_age = max(supplied_host_age, snapshot_host_age)
    if effective_host_age > config.max_current_host_receive_age_s:
        return _decision(
            False, "current_control_epoch_host_stale",
            current_host_receive_age_s=effective_host_age,
        )
    try:
        current_rate = np.asarray(
            current_epoch["legacy_body_rate_rad_s"], dtype=float
        )
    except (KeyError, TypeError, ValueError, OverflowError):
        return _decision(False, "current_control_epoch_state_invalid")
    if (
        current_rate.shape != (3,)
        or not np.all(np.isfinite(current_rate))
        or current_epoch.get("angular_rate_source")
        != POST_RELEASE_ANGULAR_RATE_SOURCE
        or current_epoch.get("state_frame") != POST_RELEASE_STATE_FRAME
        or current_epoch.get("orientation_convention")
        != POST_RELEASE_ORIENTATION_CONVENTION
    ):
        return _decision(False, "current_control_epoch_state_invalid")

    contracts = (
        snapshot.get("post_release_process_input")
        == "measured_body_specific_force",
        snapshot.get("post_release_velocity_process_command_independent")
        is True,
        snapshot.get("post_release_acceleration_attitude_coupled") is True,
        snapshot.get("post_release_observation") == "position_only",
        snapshot.get("command_history_used_for_state_reconstruction") is False,
    )
    contract_reasons = (
        "process_input_contract_missing",
        "velocity_process_depends_on_command",
        "acceleration_attitude_coupling_missing",
        "position_only_contract_missing",
        "command_history_contract_violated",
    )
    for satisfied, reason in zip(contracts, contract_reasons):
        if not satisfied:
            return _decision(False, reason)
    observer = snapshot.get("observer")
    if (
        not isinstance(observer, Mapping)
        or observer.get("phase") != "released"
        or snapshot.get("release_candidate_active") is not False
        or snapshot.get("pending_release_cf_timestamp_ms") is not None
    ):
        return _decision(False, "release_not_committed")

    release = snapshot.get("release_snapshot")
    if not isinstance(release, Mapping):
        return _decision(False, "release_snapshot_missing")
    if (
        release.get('release_preview_prepared_before_confirmation') is not True
        or release.get('release_mapping_frozen_from_preview') is not True
    ):
        return _decision(False, 'release_preview_provenance_untrusted')
    if (
        release.get("position_seed_scientifically_time_aligned") is not True
        or release.get("position_seed_timing_basis")
        != "firmware_latched_stabilizer_source_timestamp_exact"
        or release.get("state_seed_source_snapshot_atomic") is not True
        or release.get("release_gyro_source_snapshot_atomic") is not True
        or release.get("state_seed_source_timestamp_basis")
        != CONTACT_SOURCE_TIMESTAMP_BASIS
        or release.get("release_gyro_source_timestamp_basis")
        != CONTACT_SOURCE_TIMESTAMP_BASIS
        or release.get("release_event_time_source")
        != RELEASE_EVENT_TIME_SOURCE
        or release.get("release_clock_mapping_basis")
        not in AUTHORITY_RELEASE_CLOCK_MAPPING_BASES
    ):
        return _decision(False, "release_provenance_untrusted")
    event_time = _finite_number(release.get("release_event_monotonic_s"))
    confirmation_time = _finite_number(
        release.get("release_confirmation_monotonic_s")
    )
    preview_prepared_time = _finite_number(
        release.get('release_preview_prepared_monotonic_s')
    )
    event_sample = _nonnegative_integer(
        release.get("release_event_arduino_time_ms")
    )
    confirmation_sample = release.get(
        "release_confirmation_arduino_time_ms"
    )
    if (
        event_time is None or confirmation_time is None
        or preview_prepared_time is None
        or event_sample is None
    ):
        return _decision(False, "release_identity_missing_or_invalid")
    confirmation_dwell_ms = 1000.0 * (confirmation_time - event_time)
    # The confirmation sample must itself prove the dwell. This is an
    # unwrapped device-clock delta, not a raw ``confirmation_id > event_id``
    # comparison, and is independently cross-checked against host monotonic
    # time below.
    device_confirmation_dwell_ms = _unwrapped_uint32_elapsed_ms(
        confirmation_sample, event_sample
    )
    if (
        device_confirmation_dwell_ms is None
        or device_confirmation_dwell_ms
        < config.min_release_confirmation_dwell_ms
        or device_confirmation_dwell_ms
        > config.max_release_confirmation_dwell_ms
        or confirmation_dwell_ms < config.min_release_confirmation_dwell_ms
        or confirmation_dwell_ms > config.max_release_confirmation_dwell_ms
        or abs(device_confirmation_dwell_ms - confirmation_dwell_ms)
        > config.max_release_confirmation_clock_disagreement_ms
    ):
        return _decision(False, "release_confirmation_order_invalid")
    if not event_time <= preview_prepared_time < confirmation_time:
        return _decision(False, 'release_preview_not_prepared_before_confirmation')
    mapping_calibration_id = _nonempty_string(
        release.get('release_clock_mapping_calibration_id')
    )
    mapping_uncertainty_ms = _finite_number(
        release.get('release_clock_mapping_uncertainty_ms')
    )
    if (
        mapping_calibration_id is None
        or mapping_uncertainty_ms is None
        or mapping_uncertainty_ms != 0.0
    ):
        return _decision(False, 'release_clock_mapping_evidence_invalid')
    mapped_release_epoch, error = _validated_epoch(
        release.get('release_event_cf_timestamp_ms'),
        release.get('release_event_unwrapped_cf_timestamp_ms'),
    )
    if error:
        return _decision(False, f'release_event_cf_{error}')
    release_epoch, error = _validated_epoch(
        release.get("release_gyro_cf_timestamp_ms"),
        release.get("release_gyro_unwrapped_timestamp_ms"),
    )
    if error:
        return _decision(False, f"release_gyro_{error}")
    # Recompute the physical-release-to-gyro skew exclusively on the mapped CF
    # source clock.  Until interval-aware contact/free-flight propagation is
    # implemented, control eligibility requires one exact sampled epoch: mapped
    # release == release gyro == atomic p/v seed with zero mapping uncertainty.
    release_skew_ms = float(
        release_epoch[1] - mapped_release_epoch[1]
    )
    reported_cf_skew_ms = _finite_number(
        release.get('release_event_to_gyro_skew_cf_ms')
    )
    if (
        reported_cf_skew_ms is None
        or not math.isclose(
            reported_cf_skew_ms, release_skew_ms, abs_tol=1e-9
        )
    ):
        return _decision(False, 'release_clock_mapping_inconsistent')
    if (
        release_skew_ms != 0.0
    ):
        return _decision(False, "release_epoch_not_exactly_observed")
    # Retain the host-domain number as a required diagnostic, but never use it
    # to replace the independently checked CF-domain calculation above.
    release_skew_s = _finite_number(
        release.get("release_event_to_gyro_skew_s")
    )
    if (
        release_skew_s is None
        or abs(1000.0 * release_skew_s)
        > config.max_release_epoch_skew_ms
    ):
        return _decision(False, "release_epoch_host_skew_invalid")
    position_seed_skew = _finite_number(release.get("position_seed_skew_ms"))
    velocity_seed_skew = _finite_number(release.get("velocity_seed_skew_ms"))
    if (
        position_seed_skew is None or velocity_seed_skew is None
        or not math.isclose(position_seed_skew, 0.0, abs_tol=1e-9)
        or not math.isclose(velocity_seed_skew, 0.0, abs_tol=1e-9)
    ):
        return _decision(False, "release_seed_not_release_epoch")
    seed_epoch, error = _validated_epoch(
        release.get("state_seed_cf_timestamp_ms"),
        release.get("state_seed_unwrapped_timestamp_ms"),
    )
    if error:
        return _decision(False, f"release_seed_{error}")
    seed_sequence = _nonnegative_integer(
        release.get('state_seed_packet_sequence')
    )
    gyro_sequence = _nonnegative_integer(
        release.get('release_gyro_packet_sequence')
    )
    for label, source_epoch in (
        ('state_seed', seed_epoch),
        ('release_gyro', release_epoch),
    ):
        transport_skew_ms = _source_transport_skew_ms(
            release.get(f'{label}_transport_cf_timestamp_ms'),
            source_epoch[0],
        )
        reported_transport_skew_ms = _finite_number(
            release.get(
                f'{label}_transport_minus_source_timestamp_ms'
            )
        )
        if (
            transport_skew_ms is None
            or reported_transport_skew_ms is None
            or not math.isclose(
                reported_transport_skew_ms,
                transport_skew_ms,
                abs_tol=1e-9,
            )
            or transport_skew_ms < 0
            or transport_skew_ms
            > CONTACT_SOURCE_AUTHORITY_MAX_TRANSPORT_SKEW_MS
        ):
            return _decision(False, f'{label}_source_transport_skew_untrusted')
    if (
        seed_epoch != release_epoch
        or seed_sequence is None or gyro_sequence is None
        or seed_sequence != gyro_sequence
        or release.get('state_seed_same_atomic_packed_epoch') is not True
    ):
        return _decision(False, "release_seed_not_same_atomic_packed_epoch")
    if (
        release.get("cf_timestamp_ms") != release_epoch[0]
        or release.get("unwrapped_timestamp_ms") != release_epoch[1]
    ):
        return _decision(False, "release_gyro_epoch_inconsistent")

    if snapshot.get("position_timing_scientifically_valid") is not True:
        return _decision(False, "position_timing_not_strict")
    if snapshot.get("last_position_timing_basis") not in (
        TRUSTED_MOCAP_CF_TIMESTAMP_BASES
    ):
        return _decision(False, "position_timestamp_basis_untrusted")
    approximate = _nonnegative_integer(
        snapshot.get("approximate_position_time_update_count")
    )
    strict = _nonnegative_integer(
        snapshot.get("strict_position_time_update_count")
    )
    if approximate is None or strict is None:
        return _decision(False, "position_update_count_contract_invalid")
    if approximate != 0:
        return _decision(False, "approximate_position_time_used")

    ekf = snapshot.get("post_release_ekf")
    if not isinstance(ekf, Mapping) or ekf.get("valid") is not True:
        return _decision(False, "post_release_ekf_invalid")
    accepted = _nonnegative_integer(ekf.get("position_update_count"))
    rejected = _nonnegative_integer(ekf.get("rejected_position_count"))
    if accepted is None or rejected is None:
        return _decision(False, "position_update_counts_invalid")
    if strict != accepted:
        return _decision(
            False, "strict_position_update_count_mismatch",
            accepted=accepted, rejected=rejected,
        )
    if snapshot.get('position_timestamp_uncertainty_accounting') != (
            POSITION_TIMESTAMP_UNCERTAINTY_ACCOUNTING):
        return _decision(
            False, 'position_timestamp_uncertainty_contract_missing',
            accepted=accepted, rejected=rejected,
        )
    uncertainty_values = snapshot.get(
        'strict_position_timestamp_uncertainties_ms'
    )
    if not isinstance(uncertainty_values, (list, tuple)):
        return _decision(
            False, 'position_timestamp_uncertainty_evidence_invalid',
            accepted=accepted, rejected=rejected,
        )
    position_timestamp_uncertainties = [
        _finite_number(value) for value in uncertainty_values
    ]
    reported_max_position_time_uncertainty = _finite_number(
        snapshot.get('max_strict_position_timestamp_uncertainty_ms')
    )
    if (
        len(position_timestamp_uncertainties) != strict
        or any(
            value is None or value < 0.0
            for value in position_timestamp_uncertainties
        )
        or reported_max_position_time_uncertainty is None
    ):
        return _decision(
            False, 'position_timestamp_uncertainty_evidence_invalid',
            accepted=accepted, rejected=rejected,
        )
    actual_max_position_time_uncertainty = max(
        position_timestamp_uncertainties, default=0.0
    )
    if not math.isclose(
        reported_max_position_time_uncertainty,
        actual_max_position_time_uncertainty,
        abs_tol=1e-12,
    ):
        return _decision(
            False, 'position_timestamp_uncertainty_evidence_inconsistent',
            accepted=accepted, rejected=rejected,
        )
    if actual_max_position_time_uncertainty > (
            config.max_position_timestamp_uncertainty_ms):
        return _decision(
            False, 'position_timestamp_uncertainty_exceeded',
            accepted=accepted, rejected=rejected,
        )
    if actual_max_position_time_uncertainty != 0.0:
        return _decision(
            False, 'position_timestamp_uncertainty_unaccounted',
            accepted=accepted, rejected=rejected,
        )
    if accepted < config.min_position_updates:
        return _decision(
            False, "insufficient_position_updates",
            accepted=accepted, rejected=rejected,
        )
    total = accepted + rejected
    if total <= 0 or rejected / total > config.max_position_rejection_fraction:
        return _decision(
            False, "position_rejection_fraction_exceeded",
            accepted=accepted, rejected=rejected,
        )

    ekf_epoch, error = _validated_epoch(
        ekf.get("cf_timestamp_ms"), ekf.get("unwrapped_timestamp_ms")
    )
    if error:
        return _decision(False, f"ekf_{error}", accepted=accepted,
                         rejected=rejected)
    if ekf_epoch != snapshot_epoch:
        return _decision(False, "estimator_control_epoch_mismatch",
                         accepted=accepted, rejected=rejected)
    first_position, error = _validated_epoch(
        snapshot.get("first_strict_position_update_cf_timestamp_ms"),
        snapshot.get("first_strict_position_update_unwrapped_timestamp_ms"),
    )
    if error:
        return _decision(False, f"first_position_{error}", accepted=accepted,
                         rejected=rejected)
    last_position, error = _validated_epoch(
        snapshot.get("last_strict_position_update_cf_timestamp_ms"),
        snapshot.get("last_strict_position_update_unwrapped_timestamp_ms"),
    )
    if error:
        return _decision(False, f"last_position_{error}", accepted=accepted,
                         rejected=rejected)
    if not (
        release_epoch[1] <= first_position[1] <= last_position[1]
        <= ekf_epoch[1] <= now_epoch[1]
    ):
        return _decision(False, "estimator_epoch_order_noncausal",
                         accepted=accepted, rejected=rejected)
    ekf_age_ms = float(now_epoch[1] - ekf_epoch[1])
    position_age_ms = float(now_epoch[1] - last_position[1])
    if ekf_age_ms > config.max_ekf_age_ms:
        return _decision(False, "ekf_state_stale", ekf_age_ms=ekf_age_ms,
                         position_age_ms=position_age_ms,
                         current_host_receive_age_s=effective_host_age,
                         accepted=accepted, rejected=rejected)
    if position_age_ms > config.max_position_age_ms:
        return _decision(False, "position_measurement_stale",
                         ekf_age_ms=ekf_age_ms,
                         position_age_ms=position_age_ms,
                         current_host_receive_age_s=effective_host_age,
                         accepted=accepted, rejected=rejected)

    coverage_ms = float(ekf_epoch[1] - release_epoch[1])
    reported_coverage = _finite_number(
        snapshot.get("post_release_device_time_coverage_ms")
    )
    propagation_samples = _nonnegative_integer(
        snapshot.get("post_release_inertial_propagation_count")
    )
    strict_atomic_imu_samples = _nonnegative_integer(
        snapshot.get("post_release_strict_atomic_imu_count")
    )
    nonatomic_imu_samples = _nonnegative_integer(
        snapshot.get("post_release_nonatomic_imu_count")
    )
    if (
        reported_coverage is None or propagation_samples is None
        or not math.isclose(reported_coverage, coverage_ms, abs_tol=1e-9)
        or propagation_samples > coverage_ms
    ):
        return _decision(False, "post_release_inertial_evidence_inconsistent",
                         accepted=accepted, rejected=rejected)
    imu_quality_provenance = _nonempty_string(
        snapshot.get('post_release_imu_quality_provenance_id')
    )
    imu_quality_limits = snapshot.get('post_release_imu_quality_limits')
    if (
        snapshot.get('post_release_imu_quality_calibrated') is not True
        or imu_quality_provenance is None
        or imu_quality_provenance == DEFAULT_IMU_QUALITY_PROVENANCE_ID
        or not isinstance(imu_quality_limits, Mapping)
        or any(
            (value := _finite_number(imu_quality_limits.get(name))) is None
            or value <= 0.0
            for name in (
                'max_abs_gyro_deg_s', 'max_abs_accel_g',
                'max_gyro_step_deg_s', 'max_accel_step_g',
            )
        )
    ):
        return _decision(
            False, 'post_release_imu_quality_calibration_untrusted',
            post_release_device_time_coverage_ms=coverage_ms,
            inertial_propagation_samples=propagation_samples,
            accepted=accepted, rejected=rejected,
        )
    imu_quality_accepted = _nonnegative_integer(
        snapshot.get('post_release_imu_quality_accepted_count')
    )
    imu_quality_rejected = _nonnegative_integer(
        snapshot.get('post_release_imu_quality_rejected_count')
    )
    imu_quality_tainted = _nonnegative_integer(
        snapshot.get('post_release_imu_quality_tainted_count')
    )
    replayed_imu_count = _nonnegative_integer(
        release.get('release_replayed_imu_count')
    )
    replay_quality_accepted = _nonnegative_integer(
        release.get('release_replay_imu_quality_accepted_count')
    )
    replay_quality_rejected = _nonnegative_integer(
        release.get('release_replay_imu_quality_rejected_count')
    )
    replay_quality_tainted = _nonnegative_integer(
        release.get('release_replay_imu_quality_tainted_count')
    )
    if (
        imu_quality_accepted is None
        or imu_quality_rejected is None
        or imu_quality_tainted is None
        or imu_quality_accepted != propagation_samples
        or imu_quality_rejected != 0
        or imu_quality_tainted != 0
        or replayed_imu_count is None
        or replay_quality_accepted is None
        or replay_quality_rejected is None
        or replay_quality_tainted is None
        or replay_quality_accepted != replayed_imu_count
        or replay_quality_rejected != 0
        or replay_quality_tainted != 0
        or release.get('release_seed_imu_quality_accepted') is not True
    ):
        return _decision(
            False, 'post_release_imu_quality_evidence_untrusted',
            post_release_device_time_coverage_ms=coverage_ms,
            inertial_propagation_samples=propagation_samples,
            accepted=accepted, rejected=rejected,
        )
    if (
        strict_atomic_imu_samples is None
        or nonatomic_imu_samples is None
        or current_epoch.get("strict_atomic_imu") is not True
        or snapshot.get("post_release_imu_atomicity_basis")
        != "packed_contactImu_same_producer_epoch_v1"
        or strict_atomic_imu_samples + nonatomic_imu_samples
        != propagation_samples
        or strict_atomic_imu_samples != propagation_samples
        or nonatomic_imu_samples != 0
    ):
        return _decision(
            False, "post_release_imu_atomicity_untrusted",
            post_release_device_time_coverage_ms=coverage_ms,
            inertial_propagation_samples=propagation_samples,
            accepted=accepted, rejected=rejected,
        )
    if coverage_ms < config.min_post_release_device_time_coverage_ms:
        return _decision(
            False, "insufficient_post_release_device_time_coverage",
            post_release_device_time_coverage_ms=coverage_ms,
            inertial_propagation_samples=propagation_samples,
            accepted=accepted, rejected=rejected,
        )
    if propagation_samples < config.min_inertial_propagation_samples:
        return _decision(
            False, "insufficient_inertial_propagation",
            post_release_device_time_coverage_ms=coverage_ms,
            inertial_propagation_samples=propagation_samples,
            accepted=accepted, rejected=rejected,
        )
    position_span_ms = float(last_position[1] - first_position[1])
    reported_span = _finite_number(
        snapshot.get("strict_position_update_span_ms")
    )
    if (
        reported_span is None
        or not math.isclose(reported_span, position_span_ms, abs_tol=1e-9)
    ):
        return _decision(False, "position_update_span_inconsistent",
                         accepted=accepted, rejected=rejected)
    if position_span_ms < config.min_position_update_span_ms:
        return _decision(
            False, "insufficient_position_update_span",
            post_release_device_time_coverage_ms=coverage_ms,
            position_update_span_ms=position_span_ms,
            inertial_propagation_samples=propagation_samples,
            accepted=accepted, rejected=rejected,
        )

    try:
        diagonal = np.asarray(ekf["covariance_diagonal"], dtype=float)
        covariance = np.asarray(ekf["covariance_matrix"], dtype=float)
        initial_diagonal = np.asarray(
            release["initial_covariance_diagonal"], dtype=float
        )
        quaternion = np.asarray(ekf["quaternion_wxyz"], dtype=float)
        max_imu_gap_ms = float(ekf["max_imu_gap_ms"])
        reported_symmetry = float(ekf["covariance_symmetry_error"])
        reported_min_eigenvalue = float(
            ekf["covariance_min_eigenvalue"]
        )
    except (KeyError, TypeError, ValueError, OverflowError):
        return _decision(False, "estimate_numeric_contract_invalid")
    if (
        diagonal.shape != (15,) or covariance.shape != (15, 15)
        or initial_diagonal.shape != (15,) or quaternion.shape != (4,)
        or not np.all(np.isfinite(diagonal))
        or not np.all(np.isfinite(covariance))
        or not np.all(np.isfinite(initial_diagonal))
        or np.any(diagonal < 0.0) or np.any(initial_diagonal < 0.0)
        or not np.all(np.isfinite(quaternion))
        or not math.isfinite(max_imu_gap_ms) or max_imu_gap_ms < 0.0
        or not math.isfinite(reported_symmetry) or reported_symmetry < 0.0
        or not math.isfinite(reported_min_eigenvalue)
        or ekf.get("state_covariance_same_epoch") is not True
        or not math.isclose(float(np.linalg.norm(quaternion)), 1.0,
                            abs_tol=1e-3)
    ):
        return _decision(False, "estimate_numeric_contract_invalid")
    symmetry = float(np.max(np.abs(covariance - covariance.T)))
    symmetric_covariance = 0.5 * (covariance + covariance.T)
    try:
        covariance_eigenvalues = np.asarray(
            np.linalg.eigvalsh(symmetric_covariance), dtype=float
        )
    except (np.linalg.LinAlgError, TypeError, ValueError, OverflowError):
        return _decision(False, "covariance_eigendecomposition_failed")
    if (
        covariance_eigenvalues.shape != (15,)
        or not np.all(np.isfinite(covariance_eigenvalues))
    ):
        return _decision(False, "covariance_eigendecomposition_invalid")
    min_eigenvalue = float(np.min(covariance_eigenvalues))
    if (
        symmetry > 1e-10 or min_eigenvalue < -1e-10
        or not np.allclose(np.diag(covariance), diagonal,
                           rtol=1e-7, atol=1e-12)
        or not math.isclose(reported_symmetry, symmetry, abs_tol=1e-12)
        or not math.isclose(reported_min_eigenvalue, min_eigenvalue,
                            rel_tol=1e-7, abs_tol=1e-12)
    ):
        return _decision(False, "covariance_contract_invalid")

    position_std = tuple(float(v) for v in np.sqrt(diagonal[0:3]))
    velocity_std = tuple(float(v) for v in np.sqrt(diagonal[3:6]))
    attitude_std = tuple(
        float(v) for v in np.degrees(np.sqrt(diagonal[6:9]))
    )
    attitude_information = tuple(
        float(v) for v in initial_diagonal[6:9] - diagonal[6:9]
    )
    gyro_bias_std = tuple(
        float(v) for v in np.degrees(np.sqrt(diagonal[9:12]))
    )
    accel_bias_std = tuple(float(v) for v in np.sqrt(diagonal[12:15]))
    common = dict(
        ekf_age_ms=ekf_age_ms,
        position_age_ms=position_age_ms,
        current_host_receive_age_s=effective_host_age,
        position_std_m=position_std,
        velocity_std_m_s=velocity_std,
        attitude_std_deg=attitude_std,
        attitude_variance_reduction_rad2=attitude_information,
        gyro_bias_std_deg_s=gyro_bias_std,
        accel_bias_std_m_s2=accel_bias_std,
        post_release_device_time_coverage_ms=coverage_ms,
        position_update_span_ms=position_span_ms,
        inertial_propagation_samples=propagation_samples,
        accepted=accepted,
        rejected=rejected,
    )
    if max_imu_gap_ms > config.max_imu_gap_ms:
        return _decision(False, "imu_gap_exceeded", **common)
    if max(position_std) > config.max_position_std_m:
        return _decision(False, "position_uncertainty_exceeded", **common)
    if max(velocity_std) > config.max_velocity_std_m_s:
        return _decision(False, "velocity_uncertainty_exceeded", **common)
    if max(attitude_std) > config.max_attitude_std_deg:
        return _decision(False, "attitude_uncertainty_exceeded", **common)
    # Gravity supplies direct roll/pitch information through measured specific
    # force. It does not directly observe yaw, so yaw is bounded by its absolute
    # covariance above but is not required to shrink during the short release
    # confirmation window.
    if min(attitude_information[:2]) < (
            config.min_tilt_variance_reduction_rad2):
        return _decision(False, "tilt_information_insufficient", **common)
    if max(gyro_bias_std) > config.max_gyro_bias_std_deg_s:
        return _decision(False, "gyro_bias_uncertainty_exceeded", **common)
    if max(accel_bias_std) > config.max_accel_bias_std_m_s2:
        return _decision(False, "accel_bias_uncertainty_exceeded", **common)
    return _decision(True, "eligible_state_only", **common)
