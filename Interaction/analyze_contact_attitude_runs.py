"""Apply a strict scientific gate to the three contact-attitude flight logs.

Usage:
    python -m Interaction.analyze_contact_attitude_runs RUN1.json RUN2.json \
        RUN3.json --output attitude_comparison.json

The report is offline-only. It reads diagnostic shadow rows and never emits a
setpoint or changes calibration. ``READY_FOR_COMPARISON`` means only that the
recorded data may be compared; it is not flight or command-authority approval.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping

import numpy as np

from Interaction.contact_attitude_observer import CF_TIMESTAMP_MODULUS_MS
from Interaction.contact_attitude_experiment import experiment_run_config


DEFAULT_MIN_COMPARISON_SAMPLES = 30
MIN_UNIQUE_STRICT_POSITION_UPDATES = 10
MIN_COMPARISON_TIME_COVERAGE_S = 0.20
MAX_COMPARISON_SAMPLE_GAP_S = 0.05
STRICT_COMPARISON_TIME_BASES = frozenset({
    'cf_device_timestamp_exact',
    'vicon_capture_to_cf_clock_calibrated',
})
STRICT_POSITION_TIME_BASES = frozenset({
    'cf_device_timestamp_exact',
})
INTEGRITY_COUNTERS = (
    'dropped_packets',
    'skipped_position_packets',
    'contained_failure_count',
    'drain_budget_exceeded_count',
    'release_budget_exceeded_count',
)


def _finite(value):
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _nonnegative_integer(value):
    numeric = _finite(value)
    if numeric is None or numeric < 0.0 or not numeric.is_integer():
        return None
    return int(numeric)


def _pair(value):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return None
    result = [_finite(item) for item in value]
    return None if any(item is None for item in result) else result


def _unit_quaternion(value):
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        return None
    result = [_finite(item) for item in value]
    if any(item is None for item in result):
        return None
    norm = float(np.linalg.norm(result))
    return result if abs(norm - 1.0) <= 1e-3 else None


def _finite_vector(value, length):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        return None
    result = [_finite(item) for item in value]
    return None if any(item is None for item in result) else result


def _fingerprint_vector(value, length):
    result = _finite_vector(value, length)
    if result is None:
        return None
    return tuple(round(item, 12) for item in result)


def _fingerprint_quaternion(value):
    result = _unit_quaternion(value)
    if result is None:
        return None
    quaternion = np.asarray(result, dtype=float)
    quaternion /= float(np.linalg.norm(quaternion))
    for component in quaternion:
        if abs(float(component)) > 1e-12:
            if component < 0.0:
                quaternion = -quaternion
            break
    return tuple(round(float(item), 12) for item in quaternion)


def _error_metrics(pairs):
    if not pairs:
        return None
    values = np.asarray(pairs, dtype=float)
    absolute = np.abs(values)
    norm = np.linalg.norm(values, axis=1)
    return {
        'sample_count': int(len(values)),
        'roll': {
            'mean_error_deg': float(np.mean(values[:, 0])),
            'mae_deg': float(np.mean(absolute[:, 0])),
            'rmse_deg': float(np.sqrt(np.mean(values[:, 0] ** 2))),
            'p95_abs_deg': float(np.percentile(absolute[:, 0], 95)),
            'max_abs_deg': float(np.max(absolute[:, 0])),
        },
        'pitch': {
            'mean_error_deg': float(np.mean(values[:, 1])),
            'mae_deg': float(np.mean(absolute[:, 1])),
            'rmse_deg': float(np.sqrt(np.mean(values[:, 1] ** 2))),
            'p95_abs_deg': float(np.percentile(absolute[:, 1], 95)),
            'max_abs_deg': float(np.max(absolute[:, 1])),
        },
        'roll_pitch_norm': {
            'mean_deg': float(np.mean(norm)),
            'p95_deg': float(np.percentile(norm, 95)),
            'max_deg': float(np.max(norm)),
        },
    }


def _validated_limits(max_join_skew_s, min_comparison_samples):
    max_join_skew_s = _finite(max_join_skew_s)
    if max_join_skew_s is None or max_join_skew_s <= 0.0:
        raise ValueError('max_join_skew_s must be finite and positive')
    if isinstance(min_comparison_samples, bool):
        raise ValueError('min_comparison_samples must be a positive integer')
    try:
        converted_samples = int(min_comparison_samples)
    except (TypeError, ValueError):
        raise ValueError(
            'min_comparison_samples must be a positive integer'
        ) from None
    if converted_samples <= 0 or converted_samples != min_comparison_samples:
        raise ValueError('min_comparison_samples must be a positive integer')
    return max_join_skew_s, converted_samples


def _status(failures, unsupported_reasons):
    if failures:
        return 'FAIL'
    if unsupported_reasons:
        return 'UNSUPPORTED'
    return 'READY_FOR_COMPARISON'


def _comparison_skew(comparison):
    """Read only device/capture-clock skew, never host availability skew."""
    for key in (
            'comparison_time_skew_s',
            'vicon_to_onboard_cf_time_s',
            'vicon_to_onboard_device_time_s'):
        if key in comparison:
            return _finite(comparison.get(key))
    return None


def _host_availability_skew(comparison):
    """Retain legacy host timing as diagnostic-only metadata."""
    for key in (
            'vicon_to_onboard_host_availability_skew_s',
            'vicon_to_onboard_host_time_s'):
        if key in comparison:
            return _finite(comparison.get(key))
    return None


def _signed_cf_delta_ms(current, previous):
    delta = (int(current) - int(previous)) % CF_TIMESTAMP_MODULUS_MS
    if delta >= CF_TIMESTAMP_MODULUS_MS // 2:
        delta -= CF_TIMESTAMP_MODULUS_MS
    return delta


def analyze_run(
        records, expected_run, max_join_skew_s=0.03,
        min_comparison_samples=DEFAULT_MIN_COMPARISON_SAMPLES):
    max_join_skew_s, min_comparison_samples = _validated_limits(
        max_join_skew_s, min_comparison_samples
    )
    if not isinstance(records, list):
        raise ValueError('flight log must be a JSON array')
    protocol = experiment_run_config(expected_run)
    rows = [
        record['data'] for record in records
        if isinstance(record, Mapping)
        and record.get('type') == 'contact_attitude_shadow'
        and isinstance(record.get('data'), Mapping)
    ]
    failures = []
    unsupported = []
    if not rows:
        unsupported.append('contact_attitude_shadow_rows_missing')

    observed_runs = sorted({
        row.get('experiment_run') for row in rows
        if row.get('experiment_run') is not None
    })
    if rows and observed_runs != [expected_run]:
        failures.append('experiment_run_metadata_mismatch')
    observed_modes = sorted({
        row.get('mode') for row in rows if row.get('mode') is not None
    })
    if rows and observed_modes != [protocol['shadow_mode']]:
        failures.append('shadow_mode_metadata_mismatch')
    if any(row.get('shadow_only') is not True for row in rows):
        failures.append('shadow_only_contract_missing')
    if any(row.get('command_authority') is not False for row in rows):
        failures.append('command_authority_must_be_false')
    command_history_labels = [
        row.get('command_history_used_for_state_reconstruction')
        for row in rows
    ]
    if any(value is None for value in command_history_labels):
        unsupported.append(
            'command_history_state_reconstruction_label_missing'
        )
    if any(value not in (False, None) for value in command_history_labels):
        failures.append(
            'command_history_must_not_reconstruct_state'
        )

    orientation_use_labels = [
        row.get('vicon_orientation_used_by_shadow_ekf') for row in rows
    ]
    if any(value is True for value in orientation_use_labels):
        failures.append('vicon_orientation_entered_shadow_estimator')
    if any(value is None for value in orientation_use_labels):
        unsupported.append('vicon_orientation_use_label_missing')
    if any(value not in (False, None, True) for value in orientation_use_labels):
        failures.append('vicon_orientation_use_label_invalid')

    missing_integrity_fields = set()
    for row in rows:
        if 'fatal_reason' not in row:
            missing_integrity_fields.add('fatal_reason')
        elif row.get('fatal_reason') is not None:
            failures.append('shadow_fatal_reason_present')
        for field in INTEGRITY_COUNTERS:
            if field not in row:
                missing_integrity_fields.add(field)
                continue
            count = _nonnegative_integer(row.get(field))
            if count is None:
                failures.append(f'{field}_invalid')
            elif count > 0:
                failures.append(f'{field}_nonzero')
    if missing_integrity_fields:
        unsupported.append(
            'integrity_fields_missing:'
            + ','.join(sorted(missing_integrity_fields))
        )

    mirror_errors = []
    vicon_onboard_errors = []
    vicon_shadow_errors = []
    mirror_exact = []
    mirror_state_sequences = set()
    mirror_state_times = []
    mirror_ordered_samples = []
    strict_join_skews = []
    strict_frame_sequences = set()
    strict_shadow_epochs_ms = set()
    strict_position_update_counts = set()
    strict_position_update_epochs = set()
    strict_sample_keys = set()
    strict_ordered_samples = []
    host_availability_skews = []
    strict_basis_counts = {}
    observed_basis_counts = {}
    comparison_candidate_count = 0
    valid_post_release_count = 0
    position_routes = []
    orientation_routes = []
    shadow_withheld_labels = []
    onboard_withheld_labels = []
    shared_sensor_labels = []
    phase_counts = {}
    valid_count = 0
    saw_alignment_missing = False
    saw_unaligned_comparison = False
    saw_scientific_flag_missing = False
    saw_nonstrict_scientific_flag = False
    saw_capture_clock_label_missing = False
    saw_capture_clock_unavailable = False
    saw_position_timing_missing = False
    saw_nonstrict_position_timing = False
    saw_strict_skew_missing = False
    saw_sample_identity_missing = False
    saw_position_counter_missing = False
    saw_approximate_position_update = False
    saw_zero_strict_position_update = False
    saw_strict_position_epoch_missing = False
    saw_unproven_position_counter_increment = False
    release_snapshots = []
    saw_release_snapshot_missing = False

    for row in rows:
        valid = row.get('valid') is True
        if valid:
            valid_count += 1
        estimate = row.get('shadow_estimate')
        phase = 'unknown'
        if isinstance(estimate, Mapping):
            phase = estimate.get('phase', 'unknown')
            phase_counts[phase] = phase_counts.get(phase, 0) + 1
        eligible_phase = 'released' if expected_run == 1 else 'post_release'
        if phase == eligible_phase and not valid:
            failures.append('invalid_comparison_phase_row_present')
        eligible = valid and phase == eligible_phase
        if eligible:
            valid_post_release_count += 1
            release_snapshot = row.get('release_snapshot')
            if isinstance(release_snapshot, Mapping):
                release_snapshots.append(release_snapshot)
            else:
                saw_release_snapshot_missing = True

        if eligible and expected_run != 1:
            strict_count = _nonnegative_integer(
                row.get('strict_position_time_update_count')
            )
            approximate_count = _nonnegative_integer(
                row.get('approximate_position_time_update_count')
            )
            post_release_state = row.get('post_release_ekf')
            accepted_count = (
                _nonnegative_integer(
                    post_release_state.get('position_update_count')
                )
                if isinstance(post_release_state, Mapping) else None
            )
            rejected_count = (
                _nonnegative_integer(
                    post_release_state.get('rejected_position_count')
                )
                if isinstance(post_release_state, Mapping) else None
            )
            if any(value is None for value in (
                    strict_count, approximate_count,
                    accepted_count, rejected_count)):
                saw_position_counter_missing = True
            else:
                if rejected_count > 0:
                    failures.append('post_release_position_update_rejected')
                if approximate_count > 0:
                    saw_approximate_position_update = True
                if accepted_count != strict_count + approximate_count:
                    failures.append('position_update_counter_mismatch')

        comparison = row.get('comparison')
        if not isinstance(comparison, Mapping):
            continue
        host_skew = _host_availability_skew(comparison)
        if host_skew is not None:
            host_availability_skews.append(host_skew)

        if not eligible:
            continue
        vicon = row.get('vicon')
        if isinstance(vicon, Mapping):
            position_route = vicon.get('position_forwarded_to_onboard_ekf')
            if position_route is not None:
                position_routes.append(position_route)
            orientation_route = vicon.get('orientation_forwarded_to_onboard_ekf')
            if orientation_route is not None:
                orientation_routes.append(orientation_route)

        pair_mirror = _pair(comparison.get(
            'onboard_minus_shadow_roll_pitch_deg'
        ))
        exact = comparison.get('onboard_shadow_mirror_exact')
        if expected_run == 1:
            if exact is not None:
                mirror_exact.append(exact is True)
                if exact is not True:
                    failures.append('onboard_shadow_mirror_not_exact')
            if pair_mirror is not None and exact is True:
                onboard = row.get('onboard_ekf')
                sequence = (
                    _nonnegative_integer(onboard.get('sequence'))
                    if isinstance(onboard, Mapping) else None
                )
                state_time = (
                    _finite(onboard.get('state_time_s'))
                    if isinstance(onboard, Mapping) else None
                )
                if sequence is None or state_time is None:
                    saw_sample_identity_missing = True
                elif sequence not in mirror_state_sequences:
                    mirror_state_sequences.add(sequence)
                    mirror_state_times.append(state_time)
                    mirror_ordered_samples.append((sequence, state_time))
                    mirror_errors.append(pair_mirror)
                    if any(abs(value) > 1e-12 for value in pair_mirror):
                        failures.append('onboard_shadow_mirror_error_nonzero')
            if (
                _pair(comparison.get('vicon_minus_onboard_roll_pitch_deg'))
                is not None
                or _pair(comparison.get('vicon_minus_shadow_roll_pitch_deg'))
                is not None
            ):
                failures.append('run1_unexpected_vicon_orientation_comparison')
            continue

        pair_onboard = _pair(comparison.get(
            'vicon_minus_onboard_roll_pitch_deg'
        ))
        pair_shadow = _pair(comparison.get(
            'vicon_minus_shadow_roll_pitch_deg'
        ))
        if pair_onboard is None or pair_shadow is None:
            continue
        comparison_candidate_count += 1

        shadow_withheld = comparison.get(
            'vicon_orientation_withheld_from_shadow_estimator'
        )
        onboard_withheld = comparison.get(
            'vicon_orientation_withheld_from_onboard_estimator'
        )
        shared_sensor = comparison.get(
            'shared_mocap_position_sensor_correlation_remains'
        )
        shadow_withheld_labels.append(shadow_withheld)
        onboard_withheld_labels.append(onboard_withheld)
        shared_sensor_labels.append(shared_sensor)

        basis = comparison.get('comparison_time_basis')
        if basis is not None:
            observed_basis_counts[str(basis)] = (
                observed_basis_counts.get(str(basis), 0) + 1
            )
        aligned = comparison.get('comparison_time_aligned')
        if aligned is None:
            saw_alignment_missing = True
        elif aligned is not True:
            saw_unaligned_comparison = True
            continue
        capture_available = comparison.get(
            'vicon_capture_timestamp_available'
        )
        common_cf_clock = comparison.get(
            'vicon_capture_mapped_to_cf_clock'
        )
        if capture_available is None or common_cf_clock is None:
            saw_capture_clock_label_missing = True
            continue
        if capture_available is not True or common_cf_clock is not True:
            saw_capture_clock_unavailable = True
            continue
        scientific = comparison.get('comparison_scientifically_valid')
        if scientific is None:
            saw_scientific_flag_missing = True
            continue
        if scientific is not True:
            saw_nonstrict_scientific_flag = True
            continue
        if basis not in STRICT_COMPARISON_TIME_BASES:
            continue
        skew = _comparison_skew(comparison)
        if skew is None:
            saw_strict_skew_missing = True
            continue
        if abs(skew) > max_join_skew_s:
            saw_unaligned_comparison = True
            continue

        position_timing_valid = row.get(
            'position_timing_scientifically_valid'
        )
        position_basis = row.get('last_position_timing_basis')
        if position_timing_valid is None or position_basis is None:
            saw_position_timing_missing = True
            continue
        if (
            position_timing_valid is not True
            or position_basis not in STRICT_POSITION_TIME_BASES
        ):
            saw_nonstrict_position_timing = True
            continue

        expected_route = protocol['vicon_orientation_forwarded']
        route_ok = (
            isinstance(vicon, Mapping)
            and vicon.get('position_forwarded_to_onboard_ekf') is True
            and vicon.get('orientation_forwarded_to_onboard_ekf')
            is expected_route
        )
        disclosure_ok = (
            shadow_withheld is True
            and onboard_withheld is (not expected_route)
            and shared_sensor is True
        )
        if not route_ok or not disclosure_ok:
            continue

        frame_sequence = (
            _nonnegative_integer(vicon.get('frame_sequence'))
            if isinstance(vicon, Mapping) else None
        )
        shadow_epoch = (
            _nonnegative_integer(estimate.get('unwrapped_timestamp_ms'))
            if isinstance(estimate, Mapping) else None
        )
        strict_position_count = _nonnegative_integer(
            row.get('strict_position_time_update_count')
        )
        strict_position_epoch = _nonnegative_integer(
            row.get('last_strict_position_update_cf_timestamp_ms')
        )
        approximate_position_count = _nonnegative_integer(
            row.get('approximate_position_time_update_count')
        )
        post_release = row.get('post_release_ekf')
        ekf_position_count = (
            _nonnegative_integer(post_release.get('position_update_count'))
            if isinstance(post_release, Mapping) else None
        )
        rejected_position_count = (
            _nonnegative_integer(post_release.get('rejected_position_count'))
            if isinstance(post_release, Mapping) else None
        )
        if frame_sequence is None or shadow_epoch is None:
            saw_sample_identity_missing = True
            continue
        if any(value is None for value in (
                strict_position_count, approximate_position_count,
                ekf_position_count, rejected_position_count)):
            saw_position_counter_missing = True
            continue
        if strict_position_epoch is None:
            saw_strict_position_epoch_missing = True
            continue
        if rejected_position_count > 0:
            failures.append('post_release_position_update_rejected')
            continue
        if approximate_position_count > 0:
            saw_approximate_position_update = True
            continue
        if strict_position_count <= 0:
            saw_zero_strict_position_update = True
            continue
        if ekf_position_count != (
                strict_position_count + approximate_position_count):
            failures.append('strict_position_update_counter_mismatch')
            continue

        sample_key = (frame_sequence, shadow_epoch)
        if sample_key in strict_sample_keys:
            continue
        strict_sample_keys.add(sample_key)
        strict_frame_sequences.add(frame_sequence)
        strict_shadow_epochs_ms.add(shadow_epoch)
        strict_position_update_counts.add(strict_position_count)
        strict_position_update_epochs.add(strict_position_epoch)
        strict_ordered_samples.append((
            frame_sequence,
            shadow_epoch,
            strict_position_count,
            strict_position_epoch,
        ))

        strict_join_skews.append(skew)
        strict_basis_counts[str(basis)] = (
            strict_basis_counts.get(str(basis), 0) + 1
        )
        if pair_mirror is not None:
            mirror_errors.append(pair_mirror)
        vicon_onboard_errors.append(pair_onboard)
        vicon_shadow_errors.append(pair_shadow)

    if valid_post_release_count < min_comparison_samples:
        unsupported.append('minimum_valid_post_release_samples_not_met')

    if saw_release_snapshot_missing or not release_snapshots:
        unsupported.append('release_snapshot_missing')
    else:
        event_sources = {
            snapshot.get('release_event_time_source')
            for snapshot in release_snapshots
        }
        if None in event_sources:
            unsupported.append('release_event_time_source_missing')
        elif event_sources != {'force_sensor_candidate_onset_monotonic'}:
            failures.append('release_event_time_source_invalid')
        release_event_skews = [
            _finite(snapshot.get(
                'release_event_to_state_skew_s'
                if expected_run == 1 else
                'release_event_to_gyro_skew_s'
            ))
            for snapshot in release_snapshots
        ]
        if any(value is None for value in release_event_skews):
            unsupported.append('release_event_epoch_skew_missing')
        elif any(
                abs(value) > max_join_skew_s
                for value in release_event_skews):
            failures.append('release_event_epoch_skew_exceeded')

        if expected_run == 1:
            sources = {
                (snapshot.get('position_source'),
                 snapshot.get('velocity_source'))
                for snapshot in release_snapshots
            }
            if sources != {('onboard_ekf_mirror', 'onboard_ekf_mirror')}:
                failures.append('mirror_release_state_source_invalid')
            required_mirror_fields = (
                'position_m', 'onboard_ekf_velocity_m_s', 'state_time_s',
                'state_sequence', 'release_event_monotonic_s',
            )
            if any(
                    any(field not in snapshot for field in required_mirror_fields)
                    for snapshot in release_snapshots):
                unsupported.append('mirror_release_epoch_fields_missing')
            else:
                mirror_release_fingerprints = set()
                for snapshot in release_snapshots:
                    position = _fingerprint_vector(
                        snapshot.get('position_m'), 3
                    )
                    velocity = _fingerprint_vector(
                        snapshot.get('onboard_ekf_velocity_m_s'), 3
                    )
                    state_time = _finite(snapshot.get('state_time_s'))
                    state_sequence = _nonnegative_integer(
                        snapshot.get('state_sequence')
                    )
                    event_time = _finite(
                        snapshot.get('release_event_monotonic_s')
                    )
                    if any(value is None for value in (
                            position, velocity, state_time, state_sequence,
                            event_time)):
                        failures.append('mirror_release_epoch_value_invalid')
                        continue
                    mirror_release_fingerprints.add((
                        position, velocity, round(state_time, 12),
                        state_sequence, round(event_time, 12),
                    ))
                if len(mirror_release_fingerprints) > 1:
                    failures.append('release_snapshot_changed_within_episode')
        else:
            required_release_fields = (
                'position_source', 'velocity_source',
                'position_seed_scientifically_time_aligned',
                'position_seed_timing_basis', 'position_seed_skew_ms',
                'velocity_seed_skew_ms', 'cf_timestamp_ms',
                'position_seed_cf_timestamp_ms', 'gyro_quaternion_wxyz',
                'initial_position_seed_m', 'onboard_ekf_velocity_m_s',
                'onboard_ekf_position_at_velocity_epoch_m',
                'position_m', 'release_velocity_m_s',
                'external_position_seed_m',
                'post_release_position_observation_source',
                'state_seed_packet_sequence', 'state_seed_cf_timestamp_ms',
                'position_seed_packet_sequence',
                'release_gyro_packet_sequence', 'release_event_monotonic_s',
            )
            if any(
                    any(field not in snapshot for field in required_release_fields)
                    for snapshot in release_snapshots):
                unsupported.append('release_common_epoch_fields_missing')
            else:
                release_fingerprints = set()
                for snapshot in release_snapshots:
                    position_skew = _finite(
                        snapshot.get('position_seed_skew_ms')
                    )
                    velocity_skew = _finite(
                        snapshot.get('velocity_seed_skew_ms')
                    )
                    release_epoch = _nonnegative_integer(
                        snapshot.get('cf_timestamp_ms')
                    )
                    position_epoch = _nonnegative_integer(
                        snapshot.get('position_seed_cf_timestamp_ms')
                    )
                    state_epoch = _nonnegative_integer(
                        snapshot.get('state_seed_cf_timestamp_ms')
                    )
                    quaternion = _fingerprint_quaternion(
                        snapshot.get('gyro_quaternion_wxyz')
                    )
                    initial_position = _fingerprint_vector(
                        snapshot.get('initial_position_seed_m'), 3
                    )
                    external_position = _fingerprint_vector(
                        snapshot.get('external_position_seed_m'), 3
                    )
                    initial_velocity = _fingerprint_vector(
                        snapshot.get('onboard_ekf_velocity_m_s'), 3
                    )
                    atomic_onboard_position = _fingerprint_vector(
                        snapshot.get(
                            'onboard_ekf_position_at_velocity_epoch_m'
                        ),
                        3,
                    )
                    release_position = _fingerprint_vector(
                        snapshot.get('position_m'), 3
                    )
                    release_velocity = _fingerprint_vector(
                        snapshot.get('release_velocity_m_s'), 3
                    )
                    state_sequence = _nonnegative_integer(
                        snapshot.get('state_seed_packet_sequence')
                    )
                    position_sequence = _nonnegative_integer(
                        snapshot.get('position_seed_packet_sequence')
                    )
                    gyro_sequence = _nonnegative_integer(
                        snapshot.get('release_gyro_packet_sequence')
                    )
                    event_time = _finite(
                        snapshot.get('release_event_monotonic_s')
                    )
                    if (
                        snapshot.get('position_source')
                        != 'raw_vicon_tvec_forwarded_to_onboard_ekf'
                        or snapshot.get('velocity_source')
                        != 'onboard_ekf_velocity_common_cf_epoch'
                        or snapshot.get(
                            'post_release_position_observation_source'
                        ) != 'raw_vicon_tvec_position_only'
                    ):
                        failures.append('release_state_source_or_timing_invalid')
                        continue
                    position_timing_strict = snapshot.get(
                        'position_seed_scientifically_time_aligned'
                    )
                    position_timing_basis = snapshot.get(
                        'position_seed_timing_basis'
                    )
                    if position_timing_strict is True:
                        if position_timing_basis != 'cf_device_timestamp_exact':
                            failures.append(
                                'release_state_source_or_timing_invalid'
                            )
                            continue
                    elif position_timing_strict is False:
                        if (
                            position_timing_basis
                            != 'host_after_wait_availability_approximation'
                            or position_epoch is not None
                        ):
                            failures.append(
                                'release_state_source_or_timing_invalid'
                            )
                            continue
                        unsupported.append(
                            'release_position_seed_not_on_common_cf_clock'
                        )
                    else:
                        failures.append('release_state_source_or_timing_invalid')
                        continue
                    if (
                        position_skew is None or velocity_skew is None
                        or release_epoch is None or state_epoch is None
                        or release_epoch >= CF_TIMESTAMP_MODULUS_MS
                        or state_epoch >= CF_TIMESTAMP_MODULUS_MS
                        or (
                            position_epoch is not None
                            and position_epoch >= CF_TIMESTAMP_MODULUS_MS
                        )
                        or quaternion is None
                        or initial_position is None
                        or external_position is None
                        or initial_velocity is None
                        or atomic_onboard_position is None
                        or release_position is None
                        or release_velocity is None
                        or state_sequence is None
                        or position_sequence is None
                        or gyro_sequence is None
                        or event_time is None
                    ):
                        failures.append('release_common_epoch_value_invalid')
                        continue
                    if initial_position != external_position:
                        failures.append('release_extpos_position_seed_mismatch')
                        continue
                    expected_state_skew = _signed_cf_delta_ms(
                        state_epoch, release_epoch
                    )
                    if (
                        abs(position_skew) > 1000.0 * max_join_skew_s
                        or abs(velocity_skew) > 1000.0 * max_join_skew_s
                        or abs(velocity_skew - expected_state_skew) > 1e-6
                    ):
                        failures.append('release_common_epoch_mismatch')
                        continue
                    if position_timing_strict is True:
                        if (
                            position_epoch is None
                            or abs(
                                position_skew - _signed_cf_delta_ms(
                                    position_epoch, release_epoch
                                )
                            ) > 1e-6
                        ):
                            failures.append('release_common_epoch_mismatch')
                            continue
                    release_fingerprints.add((
                        release_epoch, state_epoch, position_epoch,
                        position_skew, position_timing_strict,
                        position_timing_basis,
                        snapshot.get('position_source'),
                        snapshot.get('velocity_source'),
                        quaternion, initial_position, external_position,
                        initial_velocity,
                        atomic_onboard_position,
                        release_position, release_velocity,
                        state_sequence, position_sequence, gyro_sequence,
                        round(event_time, 12),
                    ))
                if len(release_fingerprints) > 1:
                    failures.append('release_snapshot_changed_within_episode')

    if position_routes and any(route is not True for route in position_routes):
        failures.append('vicon_position_route_mismatch')
    if not position_routes:
        unsupported.append('vicon_position_route_metadata_missing')

    if expected_run == 1:
        expected_route = protocol['vicon_orientation_forwarded']
        if orientation_routes and any(
                route is not expected_route for route in orientation_routes):
            failures.append('vicon_orientation_route_mismatch')
        if not orientation_routes:
            unsupported.append('vicon_orientation_route_metadata_missing')
        if any(
                current[0] < previous[0]
                or current[1] < previous[1]
                for previous, current in zip(
                    mirror_ordered_samples, mirror_ordered_samples[1:])):
            failures.append('mirror_samples_not_strictly_monotonic')
        if any(
                current[0] == previous[0]
                or current[1] == previous[1]
                for previous, current in zip(
                    mirror_ordered_samples, mirror_ordered_samples[1:])):
            unsupported.append('mirror_samples_not_strictly_increasing')
        mirror_gaps_s = [
            current[1] - previous[1]
            for previous, current in zip(
                mirror_ordered_samples, mirror_ordered_samples[1:])
        ]
        if any(gap > MAX_COMPARISON_SAMPLE_GAP_S for gap in mirror_gaps_s):
            unsupported.append('mirror_sample_gap_exceeded')
        if mirror_exact and not all(mirror_exact):
            failures.append('onboard_shadow_mirror_not_exact')
        if len(mirror_errors) < min_comparison_samples:
            unsupported.append('minimum_exact_mirror_samples_not_met')
        if len(mirror_state_sequences) < min_comparison_samples:
            unsupported.append('minimum_unique_mirror_epochs_not_met')
        mirror_coverage_s = (
            None if not mirror_state_times else
            max(mirror_state_times) - min(mirror_state_times)
        )
        if (
            mirror_coverage_s is None
            or mirror_coverage_s < MIN_COMPARISON_TIME_COVERAGE_S
        ):
            unsupported.append('minimum_mirror_time_coverage_not_met')
        if saw_sample_identity_missing:
            unsupported.append('mirror_sample_identity_or_time_missing')
    else:
        strict_position_epoch_deltas_ms = [
            _signed_cf_delta_ms(current[3], previous[3])
            for previous, current in zip(
                strict_ordered_samples, strict_ordered_samples[1:]
            )
        ]
        if any(
                current[0] < previous[0]
                or current[1] < previous[1]
                or current[2] < previous[2]
                or _signed_cf_delta_ms(current[3], previous[3]) < 0
                for previous, current in zip(
                    strict_ordered_samples, strict_ordered_samples[1:])):
            failures.append('strict_samples_not_strictly_monotonic')
        if any(
                current[0] == previous[0]
                or current[1] == previous[1]
                or current[2] == previous[2]
                or _signed_cf_delta_ms(current[3], previous[3]) == 0
                for previous, current in zip(
                    strict_ordered_samples, strict_ordered_samples[1:])):
            unsupported.append('strict_samples_not_strictly_increasing')
        if any(
                current[2] != previous[2] + 1
                for previous, current in zip(
                    strict_ordered_samples, strict_ordered_samples[1:])):
            saw_unproven_position_counter_increment = True
        strict_sample_gaps_s = [
            (current[1] - previous[1]) / 1000.0
            for previous, current in zip(
                strict_ordered_samples, strict_ordered_samples[1:])
        ]
        if any(
                gap > MAX_COMPARISON_SAMPLE_GAP_S
                for gap in strict_sample_gaps_s):
            unsupported.append('strict_sample_gap_exceeded')
        expected_route = protocol['vicon_orientation_forwarded']
        if orientation_routes and any(
                route is not expected_route for route in orientation_routes):
            failures.append('vicon_orientation_route_mismatch')
        if not orientation_routes:
            unsupported.append('vicon_orientation_route_metadata_missing')
        if any(
                value is not None and value is not True
                for value in shadow_withheld_labels):
            failures.append('vicon_orientation_not_withheld_from_shadow')
        if not shadow_withheld_labels or any(
                value is None for value in shadow_withheld_labels):
            unsupported.append('shadow_orientation_withholding_label_missing')
        expected_onboard_withheld = not expected_route
        if any(
                value is not None and value is not expected_onboard_withheld
                for value in onboard_withheld_labels):
            failures.append('onboard_orientation_route_disclosure_mismatch')
        if not onboard_withheld_labels or any(
                value is None for value in onboard_withheld_labels):
            unsupported.append('onboard_orientation_withholding_label_missing')
        if any(
                value is not None and value is not True
                for value in shared_sensor_labels):
            failures.append('shared_mocap_position_correlation_denied')
        if not shared_sensor_labels or any(
                value is None for value in shared_sensor_labels):
            unsupported.append('shared_mocap_position_correlation_label_missing')
        if saw_alignment_missing:
            unsupported.append('comparison_time_alignment_label_missing')
        if saw_unaligned_comparison:
            failures.append('comparison_time_not_aligned')
        if saw_scientific_flag_missing:
            unsupported.append('comparison_scientific_validity_label_missing')
        if saw_nonstrict_scientific_flag:
            unsupported.append('comparison_not_scientifically_time_aligned')
        if saw_capture_clock_label_missing:
            unsupported.append('capture_common_clock_labels_missing')
        if saw_capture_clock_unavailable:
            unsupported.append('capture_common_cf_clock_unavailable')
        if saw_position_timing_missing:
            unsupported.append('strict_position_timing_metadata_missing')
        if saw_nonstrict_position_timing:
            unsupported.append('position_updates_not_on_strict_device_clock')
        if saw_strict_skew_missing:
            unsupported.append('strict_comparison_skew_missing')
        if saw_sample_identity_missing:
            unsupported.append('strict_sample_identity_missing')
        if saw_position_counter_missing:
            unsupported.append('position_update_counters_missing_or_invalid')
        if saw_strict_position_epoch_missing:
            unsupported.append('strict_position_update_epoch_missing')
        if saw_unproven_position_counter_increment:
            unsupported.append('strict_position_counter_increment_unproven')
        if saw_approximate_position_update:
            unsupported.append('approximate_position_updates_present')
        if saw_zero_strict_position_update:
            unsupported.append('strict_position_update_missing')
        if len(strict_join_skews) < min_comparison_samples:
            unsupported.append('strict_common_clock_sample_minimum_not_met')
        if len(strict_frame_sequences) < min_comparison_samples:
            unsupported.append('minimum_unique_vicon_frames_not_met')
        if len(strict_shadow_epochs_ms) < min_comparison_samples:
            unsupported.append('minimum_unique_shadow_epochs_not_met')
        if (
            len(strict_position_update_counts)
            < MIN_UNIQUE_STRICT_POSITION_UPDATES
        ):
            unsupported.append('minimum_strict_position_updates_not_met')
        if (
            len(strict_position_update_epochs)
            < MIN_UNIQUE_STRICT_POSITION_UPDATES
        ):
            unsupported.append(
                'minimum_unique_strict_position_epochs_not_met'
            )
        strict_position_epoch_coverage_s = (
            None if len(strict_position_update_epochs) < 2 else
            sum(strict_position_epoch_deltas_ms) / 1000.0
        )
        if (
            strict_position_epoch_coverage_s is None
            or strict_position_epoch_coverage_s
            < MIN_COMPARISON_TIME_COVERAGE_S
        ):
            unsupported.append(
                'minimum_strict_position_epoch_coverage_not_met'
            )
        if any(
                gap / 1000.0 > MAX_COMPARISON_SAMPLE_GAP_S
                for gap in strict_position_epoch_deltas_ms):
            unsupported.append('strict_position_epoch_gap_exceeded')
        strict_coverage_s = (
            None if not strict_shadow_epochs_ms else
            (max(strict_shadow_epochs_ms) - min(strict_shadow_epochs_ms))
            / 1000.0
        )
        if (
            strict_coverage_s is None
            or strict_coverage_s < MIN_COMPARISON_TIME_COVERAGE_S
        ):
            unsupported.append('minimum_strict_time_coverage_not_met')
        if len(vicon_onboard_errors) < min_comparison_samples:
            unsupported.append('minimum_three_way_comparison_samples_not_met')

    strict_abs = np.abs(np.asarray(strict_join_skews, dtype=float))
    host_abs = np.abs(np.asarray(host_availability_skews, dtype=float))
    timing = {
        'strict_sample_count': len(strict_join_skews),
        'comparison_candidate_count': comparison_candidate_count,
        'minimum_required_samples': min_comparison_samples,
        'minimum_required_strict_position_updates': (
            MIN_UNIQUE_STRICT_POSITION_UPDATES
        ),
        'minimum_required_time_coverage_s': MIN_COMPARISON_TIME_COVERAGE_S,
        'maximum_allowed_sample_gap_s': MAX_COMPARISON_SAMPLE_GAP_S,
        'unique_mirror_epoch_count': len(mirror_state_sequences),
        'unique_vicon_frame_count': len(strict_frame_sequences),
        'unique_shadow_epoch_count': len(strict_shadow_epochs_ms),
        'unique_strict_position_update_count': (
            len(strict_position_update_counts)
        ),
        'unique_strict_position_update_epoch_count': (
            len(strict_position_update_epochs)
        ),
        'strict_position_update_epoch_coverage_s': (
            None if expected_run == 1 else
            strict_position_epoch_coverage_s
        ),
        'strict_time_coverage_s': (
            None if not strict_shadow_epochs_ms else
            (max(strict_shadow_epochs_ms) - min(strict_shadow_epochs_ms))
            / 1000.0
        ),
        'mirror_time_coverage_s': (
            None if not mirror_state_times else
            max(mirror_state_times) - min(mirror_state_times)
        ),
        'strict_max_sample_gap_s': (
            None if expected_run == 1 or not strict_sample_gaps_s else
            max(strict_sample_gaps_s)
        ),
        'mirror_max_sample_gap_s': (
            None if expected_run != 1 or not mirror_gaps_s else
            max(mirror_gaps_s)
        ),
        'strict_time_basis_counts': strict_basis_counts,
        'observed_time_basis_counts': observed_basis_counts,
        'max_abs_s': (
            None if not strict_join_skews else float(np.max(strict_abs))
        ),
        'p95_abs_s': (
            None if not strict_join_skews
            else float(np.percentile(strict_abs, 95))
        ),
        'within_limit_fraction': (
            None if not strict_join_skews else float(np.mean(
                strict_abs <= max_join_skew_s
            ))
        ),
        'limit_s': max_join_skew_s,
        'host_availability_diagnostic_only': {
            'sample_count': len(host_availability_skews),
            'max_abs_s': (
                None if not host_availability_skews
                else float(np.max(host_abs))
            ),
            'never_used_for_scientific_gate': True,
        },
    }
    failures = sorted(set(failures))
    unsupported = sorted(set(unsupported))
    status = _status(failures, unsupported)
    return {
        'run': expected_run,
        'expected_protocol': protocol,
        'status': status,
        'scientific_gate_passed': status == 'READY_FOR_COMPARISON',
        'failures': failures,
        'unsupported_reasons': unsupported,
        'errors': sorted(set(failures + unsupported)),
        'shadow_row_count': len(rows),
        'valid_shadow_row_count': valid_count,
        'valid_post_release_row_count': valid_post_release_count,
        'valid_shadow_fraction': (
            None if not rows else valid_count / len(rows)
        ),
        'phase_counts': phase_counts,
        'onboard_minus_shadow': _error_metrics(mirror_errors),
        'vicon_minus_onboard': _error_metrics(vicon_onboard_errors),
        'vicon_minus_shadow': _error_metrics(vicon_shadow_errors),
        'onboard_shadow_mirror_exact_fraction': (
            None if not mirror_exact else sum(mirror_exact) / len(mirror_exact)
        ),
        'vicon_onboard_join_timing': timing,
        'interpretation': (
            'Plumbing parity only: shadow is a synchronized onboard-telemetry '
            'copy.'
            if expected_run == 1 else
            'Vicon orientation is withheld from both estimator updates and is '
            'used only as an evaluation channel; Vicon XYZ is shared by both '
            'estimators, so shared-sensor correlation remains.'
            if expected_run == 2 else
            'Vicon orientation is withheld from the shadow estimator update '
            'but forwarded to the onboard EKF; Vicon XYZ is also shared, so '
            'this run cannot independently validate the onboard estimate.'
        ),
    }


def analyze_three_runs(
        run1, run2, run3, max_join_skew_s=0.03,
        min_comparison_samples=DEFAULT_MIN_COMPARISON_SAMPLES):
    max_join_skew_s, min_comparison_samples = _validated_limits(
        max_join_skew_s, min_comparison_samples
    )
    reports = [
        analyze_run(run1, 1, max_join_skew_s, min_comparison_samples),
        analyze_run(run2, 2, max_join_skew_s, min_comparison_samples),
        analyze_run(run3, 3, max_join_skew_s, min_comparison_samples),
    ]
    statuses = {report['status'] for report in reports}
    status = (
        'FAIL' if 'FAIL' in statuses else
        'UNSUPPORTED' if 'UNSUPPORTED' in statuses else
        'READY_FOR_COMPARISON'
    )
    return {
        'schema_version': 2,
        'offline_only': True,
        'command_authority': False,
        'status': status,
        'scientific_gate_passed': status == 'READY_FOR_COMPARISON',
        'minimum_comparison_samples_per_run': min_comparison_samples,
        'runs': reports,
    }


def _sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run1', type=Path)
    parser.add_argument('run2', type=Path)
    parser.add_argument('run3', type=Path)
    parser.add_argument('--max-join-skew-s', type=float, default=0.03)
    parser.add_argument(
        '--min-comparison-samples', type=int,
        default=DEFAULT_MIN_COMPARISON_SAMPLES,
    )
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    try:
        _validated_limits(
            args.max_join_skew_s, args.min_comparison_samples
        )
    except ValueError as error:
        parser.error(str(error))
    if args.output is not None and args.output.exists():
        parser.error('output already exists; choose a new path')
    paths = (args.run1, args.run2, args.run3)
    try:
        records = []
        for path in paths:
            with path.open() as source:
                records.append(json.load(source))
        report = analyze_three_runs(
            *records,
            max_join_skew_s=args.max_join_skew_s,
            min_comparison_samples=args.min_comparison_samples,
        )
    except (OSError, json.JSONDecodeError, ValueError) as error:
        parser.error(str(error))
    report['sources'] = [
        {'run': run, 'path': str(path.resolve()), 'sha256': _sha256(path)}
        for run, path in enumerate(paths, start=1)
    ]
    serialized = json.dumps(
        report, indent=2, sort_keys=True, allow_nan=False
    ) + '\n'
    if args.output is None:
        print(serialized, end='')
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized)
        print(f"{report['status']}: {args.output}")
    return 0 if report['status'] == 'READY_FOR_COMPARISON' else 2


if __name__ == '__main__':
    raise SystemExit(main())
