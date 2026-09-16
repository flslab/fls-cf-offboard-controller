import copy
from dataclasses import replace
import math
import unittest
from unittest import mock

import numpy as np

from Interaction.contact_attitude_experiment import (
    ARDUINO_TO_CF_RELEASE_CLOCK_MAPPING_BASIS,
    AUTHORITY_RELEASE_CLOCK_MAPPING_BASES,
    CONTACT_ATTITUDE_PROTOCOL_VERSION,
    CRAZYSIM_RELEASE_CLOCK_MAPPING_BASIS,
    DIAGNOSTIC_RELEASE_CLOCK_MAPPING_BASES,
    FIRMWARE_SHARED_CLOCK_RELEASE_LATCH_BASIS,
    INERTIAL_POSITION,
    RELEASE_EVENT_TIME_SOURCE,
)
from Interaction.contact_attitude_observer import CF_TIMESTAMP_MODULUS_MS
from Interaction.contact_attitude_shadow import (
    POSITION_TIMESTAMP_UNCERTAINTY_ACCOUNTING,
    POST_RELEASE_ANGULAR_RATE_SOURCE,
    POST_RELEASE_ORIENTATION_CONVENTION,
    POST_RELEASE_STATE_FRAME,
)
from Interaction.log_manager import (
    CONTACT_SOURCE_TIMESTAMP_BASIS,
    CRAZYSIM_CF_TIMESTAMP_BASIS,
)
from Interaction.post_release_estimator_gate import (
    PostReleaseEstimatorGateConfig,
    evaluate_post_release_estimator,
)


def valid_snapshot():
    covariance = [0.01 ** 2] * 3
    covariance += [0.08 ** 2] * 3
    covariance += [0.03 ** 2] * 3
    covariance += [0.01 ** 2] * 6
    initial_covariance = list(covariance)
    initial_covariance[6:9] = [0.04 ** 2] * 3
    covariance_matrix = np.diag(covariance)
    return {
        "shadow_only": True,
        "command_authority": False,
        "mode": INERTIAL_POSITION,
        "protocol_version": CONTACT_ATTITUDE_PROTOCOL_VERSION,
        "valid": True,
        "alignment_yaw_source": "configured_nominal_yaw",
        "alignment_nominal_yaw_deg": 0.0,
        "alignment_gate_metrics": {"passed": True},
        "absolute_yaw_reference_certified": True,
        "absolute_yaw_reference_certificate_id": "crazysim_yaw_truth_v1",
        "absolute_yaw_reference_yaw_deg": 0.0,
        "post_release_process_input": "measured_body_specific_force",
        "post_release_velocity_process_command_independent": True,
        "post_release_acceleration_attitude_coupled": True,
        "post_release_observation": "position_only",
        "command_history_used_for_state_reconstruction": False,
        "position_timing_scientifically_valid": True,
        "last_position_timing_basis": CRAZYSIM_CF_TIMESTAMP_BASIS,
        "approximate_position_time_update_count": 0,
        "strict_position_time_update_count": 10,
        "strict_position_timestamp_uncertainties_ms": [0.0] * 10,
        "max_strict_position_timestamp_uncertainty_ms": 0.0,
        "position_timestamp_uncertainty_accounting": (
            POSITION_TIMESTAMP_UNCERTAINTY_ACCOUNTING
        ),
        "first_strict_position_update_cf_timestamp_ms": 930,
        "first_strict_position_update_unwrapped_timestamp_ms": 930,
        "last_strict_position_update_cf_timestamp_ms": 980,
        "last_strict_position_update_unwrapped_timestamp_ms": 980,
        "strict_position_update_span_ms": 50.0,
        "post_release_device_time_coverage_ms": 100.0,
        "post_release_inertial_propagation_count": 80,
        "post_release_strict_atomic_imu_count": 80,
        "post_release_nonatomic_imu_count": 0,
        "post_release_imu_atomicity_basis": (
            "packed_contactImu_same_producer_epoch_v1"
        ),
        "post_release_imu_quality_calibrated": True,
        "post_release_imu_quality_provenance_id": (
            "crazysim_exact_imu_quality_v1"
        ),
        "post_release_imu_quality_limits": {
            "max_abs_gyro_deg_s": 1990.0,
            "max_abs_accel_g": 23.5,
            "max_gyro_step_deg_s": 1000.0,
            "max_accel_step_g": 8.0,
        },
        "post_release_imu_quality_accepted_count": 80,
        "post_release_imu_quality_rejected_count": 0,
        "post_release_imu_quality_tainted_count": 0,
        "observer": {"phase": "released"},
        "release_candidate_active": False,
        "pending_release_cf_timestamp_ms": None,
        "release_snapshot": {
            "release_preview_prepared_before_confirmation": True,
            "release_mapping_frozen_from_preview": True,
            "release_preview_prepared_monotonic_s": 10.01,
            "position_seed_scientifically_time_aligned": True,
            "position_seed_timing_basis": (
                "firmware_latched_stabilizer_source_timestamp_exact"
            ),
            "position_seed_skew_ms": 0.0,
            "velocity_seed_skew_ms": 0.0,
            "state_seed_source_snapshot_atomic": True,
            "release_gyro_source_snapshot_atomic": True,
            "state_seed_source_timestamp_basis": CONTACT_SOURCE_TIMESTAMP_BASIS,
            "release_gyro_source_timestamp_basis": CONTACT_SOURCE_TIMESTAMP_BASIS,
            "state_seed_cf_timestamp_ms": 900,
            "state_seed_unwrapped_timestamp_ms": 900,
            "state_seed_transport_cf_timestamp_ms": 902,
            "state_seed_transport_minus_source_timestamp_ms": 2.0,
            "state_seed_packet_sequence": 42,
            "release_gyro_cf_timestamp_ms": 900,
            "release_gyro_unwrapped_timestamp_ms": 900,
            "release_gyro_transport_cf_timestamp_ms": 902,
            "release_gyro_transport_minus_source_timestamp_ms": 2.0,
            "release_gyro_packet_sequence": 42,
            "state_seed_same_atomic_packed_epoch": True,
            "cf_timestamp_ms": 900,
            "unwrapped_timestamp_ms": 900,
            "release_event_monotonic_s": 10.0,
            "release_confirmation_monotonic_s": 10.05,
            "release_event_arduino_time_ms": 100,
            "release_confirmation_arduino_time_ms": 150,
            "release_event_time_source": RELEASE_EVENT_TIME_SOURCE,
            "release_clock_mapping_basis": (
                FIRMWARE_SHARED_CLOCK_RELEASE_LATCH_BASIS
            ),
            "release_event_cf_timestamp_ms": 900,
            "release_event_unwrapped_cf_timestamp_ms": 900,
            "release_clock_mapping_uncertainty_ms": 0.0,
            "release_clock_mapping_calibration_id": (
                "firmware_shared_clock_release_latch_calibration_v1"
            ),
            "release_event_to_gyro_skew_cf_ms": 0.0,
            "release_event_to_gyro_skew_s": -0.004,
            "release_replayed_imu_count": 80,
            "release_replay_imu_quality_accepted_count": 80,
            "release_replay_imu_quality_rejected_count": 0,
            "release_replay_imu_quality_tainted_count": 0,
            "release_seed_imu_quality_accepted": True,
            "initial_covariance_diagonal": initial_covariance,
        },
        "post_release_control_epoch": {
            "cf_timestamp_ms": 1000,
            "unwrapped_timestamp_ms": 1000,
            "timestamp_basis": CONTACT_SOURCE_TIMESTAMP_BASIS,
            "transport_cf_timestamp_ms": 1002,
            "transport_minus_source_timestamp_ms": 2.0,
            "source_snapshot_atomic": True,
            "strict_atomic_imu": True,
            "host_receive_age_s": 0.005,
            "legacy_body_rate_rad_s": [0.01, -0.02, 0.03],
            "angular_rate_source": POST_RELEASE_ANGULAR_RATE_SOURCE,
            "state_frame": POST_RELEASE_STATE_FRAME,
            "orientation_convention": POST_RELEASE_ORIENTATION_CONVENTION,
        },
        "post_release_ekf": {
            "valid": True,
            "cf_timestamp_ms": 1000,
            "unwrapped_timestamp_ms": 1000,
            "position_update_count": 10,
            "rejected_position_count": 1,
            "covariance_diagonal": covariance,
            "covariance_matrix": covariance_matrix.tolist(),
            "covariance_symmetry_error": 0.0,
            "covariance_min_eigenvalue": float(np.min(covariance)),
            "state_covariance_same_epoch": True,
            "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            "max_imu_gap_ms": 1.0,
        },
    }


class PostReleaseEstimatorGateTests(unittest.TestCase):
    def setUp(self):
        self.config = PostReleaseEstimatorGateConfig(enabled=True)

    def evaluate(self, snapshot, **kwargs):
        return evaluate_post_release_estimator(
            snapshot,
            now_cf_timestamp_ms=kwargs.pop("now_cf_timestamp_ms", 1000),
            now_unwrapped_cf_timestamp_ms=kwargs.pop(
                "now_unwrapped_cf_timestamp_ms", 1000
            ),
            now_timestamp_basis=kwargs.pop(
                "now_timestamp_basis", CONTACT_SOURCE_TIMESTAMP_BASIS
            ),
            now_host_receive_age_s=kwargs.pop(
                "now_host_receive_age_s", 0.005
            ),
            config=kwargs.pop("config", self.config),
            **kwargs,
        )

    @staticmethod
    def set_covariance_diagonal(snapshot, index, value):
        ekf = snapshot["post_release_ekf"]
        ekf["covariance_diagonal"][index] = value
        ekf["covariance_matrix"][index][index] = value
        matrix = np.asarray(ekf["covariance_matrix"], dtype=float)
        if np.all(np.isfinite(matrix)):
            ekf["covariance_min_eigenvalue"] = float(np.min(
                np.linalg.eigvalsh(0.5 * (matrix + matrix.T))
            ))

    def test_default_is_disabled_and_never_authorizes(self):
        decision = evaluate_post_release_estimator(
            valid_snapshot(),
            now_cf_timestamp_ms=1000,
            now_unwrapped_cf_timestamp_ms=1000,
            now_timestamp_basis=CONTACT_SOURCE_TIMESTAMP_BASIS,
            now_host_receive_age_s=0.005,
        )
        self.assertFalse(decision.estimator_control_eligible)
        self.assertFalse(decision.gate_grants_command_authority)
        self.assertEqual(decision.reason, "disabled")

    def test_strict_fresh_bounded_estimate_is_eligible_but_not_authorized(self):
        decision = self.evaluate(valid_snapshot())
        self.assertTrue(decision.estimator_control_eligible)
        self.assertFalse(decision.gate_grants_command_authority)
        self.assertEqual(decision.reason, "eligible_state_only")
        self.assertEqual(decision.ekf_age_ms, 0.0)
        self.assertEqual(decision.position_age_ms, 20.0)

    def test_velocity_process_must_be_command_independent(self):
        snapshot = valid_snapshot()
        snapshot["post_release_velocity_process_command_independent"] = False
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "velocity_process_depends_on_command",
        )

    def test_acceleration_must_be_attitude_coupled(self):
        snapshot = valid_snapshot()
        snapshot["post_release_acceleration_attitude_coupled"] = False
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "acceleration_attitude_coupling_missing",
        )

    def test_shadow_ownership_and_committed_release_are_required(self):
        snapshot = valid_snapshot()
        snapshot["command_authority"] = True
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "shadow_ownership_contract_missing",
        )
        snapshot = valid_snapshot()
        snapshot["release_candidate_active"] = True
        self.assertEqual(self.evaluate(snapshot).reason, "release_not_committed")

    def test_command_history_or_non_position_observation_fails_closed(self):
        snapshot = valid_snapshot()
        snapshot["command_history_used_for_state_reconstruction"] = True
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "command_history_contract_violated",
        )
        snapshot = valid_snapshot()
        snapshot["post_release_observation"] = "pose"
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "position_only_contract_missing",
        )

    def test_release_and_current_clock_provenance_are_required(self):
        snapshot = valid_snapshot()
        snapshot["release_snapshot"]["state_seed_source_snapshot_atomic"] = False
        self.assertEqual(self.evaluate(snapshot).reason, "release_provenance_untrusted")
        self.assertEqual(
            self.evaluate(
                valid_snapshot(), now_timestamp_basis="host_monotonic"
            ).reason,
            "current_state_timestamp_untrusted",
        )

    def test_diagnostic_release_clocks_never_pass_authority_allowlist(self):
        for basis in (
            CRAZYSIM_RELEASE_CLOCK_MAPPING_BASIS,
            ARDUINO_TO_CF_RELEASE_CLOCK_MAPPING_BASIS,
        ):
            with self.subTest(basis=basis):
                self.assertIn(
                    basis, DIAGNOSTIC_RELEASE_CLOCK_MAPPING_BASES
                )
                self.assertNotIn(
                    basis, AUTHORITY_RELEASE_CLOCK_MAPPING_BASES
                )
                snapshot = valid_snapshot()
                snapshot["release_snapshot"][
                    "release_clock_mapping_basis"
                ] = basis
                decision = self.evaluate(snapshot)
                self.assertFalse(decision.estimator_control_eligible)
                self.assertEqual(
                    decision.reason, "release_provenance_untrusted"
                )

        self.assertEqual(
            AUTHORITY_RELEASE_CLOCK_MAPPING_BASES,
            frozenset({FIRMWARE_SHARED_CLOCK_RELEASE_LATCH_BASIS}),
        )

    def test_source_transport_skew_is_recomputed_at_release_and_current(self):
        snapshot = valid_snapshot()
        snapshot["post_release_control_epoch"][
            "transport_cf_timestamp_ms"
        ] = 1200
        snapshot["post_release_control_epoch"][
            "transport_minus_source_timestamp_ms"
        ] = 200.0
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "current_source_transport_skew_untrusted",
        )

        snapshot = valid_snapshot()
        snapshot["release_snapshot"][
            "state_seed_transport_minus_source_timestamp_ms"
        ] = 3.0
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "state_seed_source_transport_skew_untrusted",
        )

        snapshot = valid_snapshot()
        snapshot["release_snapshot"][
            "release_gyro_transport_cf_timestamp_ms"
        ] = 1101
        snapshot["release_snapshot"][
            "release_gyro_transport_minus_source_timestamp_ms"
        ] = 201.0
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "release_gyro_source_transport_skew_untrusted",
        )

        for transport, reported in ((999, -1.0), (1006, 6.0)):
            with self.subTest(current_transport=transport):
                snapshot = valid_snapshot()
                snapshot["post_release_control_epoch"].update({
                    "transport_cf_timestamp_ms": transport,
                    "transport_minus_source_timestamp_ms": reported,
                })
                self.assertEqual(
                    self.evaluate(snapshot).reason,
                    "current_source_transport_skew_untrusted",
                )

        snapshot = valid_snapshot()
        snapshot["release_snapshot"].update({
            "state_seed_transport_cf_timestamp_ms": 899,
            "state_seed_transport_minus_source_timestamp_ms": -1.0,
        })
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "state_seed_source_transport_skew_untrusted",
        )

    def test_yaw_initialization_provenance_is_required(self):
        snapshot = valid_snapshot()
        snapshot["alignment_gate_metrics"]["passed"] = False
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "yaw_initialization_provenance_untrusted",
        )

        snapshot = valid_snapshot()
        snapshot["absolute_yaw_reference_certified"] = False
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "absolute_yaw_reference_uncertified",
        )
        snapshot = valid_snapshot()
        snapshot["absolute_yaw_reference_certificate_id"] = ""
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "absolute_yaw_reference_uncertified",
        )
        snapshot = valid_snapshot()
        snapshot["alignment_yaw_source"] = "unknown"
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "yaw_initialization_provenance_untrusted",
        )
        snapshot = valid_snapshot()
        snapshot["alignment_yaw_source"] = "onboard_ekf_yaw"
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "absolute_yaw_reference_not_bound_to_alignment",
        )
        snapshot = valid_snapshot()
        snapshot["alignment_nominal_yaw_deg"] = 1.0
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "absolute_yaw_reference_not_bound_to_alignment",
        )
        snapshot = valid_snapshot()
        snapshot["absolute_yaw_reference_yaw_deg"] = float("nan")
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "absolute_yaw_reference_uncertified",
        )

    def test_release_must_have_a_frozen_preview_before_confirmation(self):
        snapshot = valid_snapshot()
        snapshot["release_snapshot"][
            "release_preview_prepared_before_confirmation"
        ] = False
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "release_preview_provenance_untrusted",
        )

        snapshot = valid_snapshot()
        snapshot["release_snapshot"][
            "release_mapping_frozen_from_preview"
        ] = False
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "release_preview_provenance_untrusted",
        )

        snapshot = valid_snapshot()
        snapshot["release_snapshot"][
            "release_preview_prepared_monotonic_s"
        ] = snapshot["release_snapshot"]["release_confirmation_monotonic_s"]
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "release_preview_not_prepared_before_confirmation",
        )

    def test_release_v3_identity_and_causal_gyro_are_required(self):
        snapshot = valid_snapshot()
        del snapshot["release_snapshot"]["release_event_time_source"]
        self.assertEqual(
            self.evaluate(snapshot).reason, "release_provenance_untrusted"
        )

        snapshot = valid_snapshot()
        del snapshot["release_snapshot"][
            "release_confirmation_arduino_time_ms"
        ]
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "release_confirmation_order_invalid",
        )

        snapshot = valid_snapshot()
        snapshot["release_snapshot"][
            "release_event_unwrapped_cf_timestamp_ms"
        ] = 899
        snapshot["release_snapshot"]["release_event_cf_timestamp_ms"] = 899
        snapshot["release_snapshot"][
            "release_event_to_gyro_skew_cf_ms"
        ] = 1.0
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "release_epoch_not_exactly_observed",
        )

    def test_release_mapping_requires_numeric_consistent_calibration(self):
        mutations = (
            (
                "release_clock_mapping_evidence_invalid",
                lambda release: release.pop(
                    "release_clock_mapping_uncertainty_ms"
                ),
            ),
            (
                "release_clock_mapping_evidence_invalid",
                lambda release: release.update(
                    release_clock_mapping_uncertainty_ms=-0.1
                ),
            ),
            (
                "release_clock_mapping_evidence_invalid",
                lambda release: release.update(
                    release_clock_mapping_calibration_id=""
                ),
            ),
            (
                "release_event_cf_timestamp_raw_unwrapped_mismatch",
                lambda release: release.update(
                    release_event_cf_timestamp_ms=903
                ),
            ),
            (
                "release_clock_mapping_inconsistent",
                lambda release: release.update(
                    release_event_to_gyro_skew_cf_ms=-3.0
                ),
            ),
            (
                "release_clock_mapping_evidence_invalid",
                lambda release: release.update(
                    release_clock_mapping_uncertainty_ms=17.0
                ),
            ),
        )
        for reason, mutate in mutations:
            with self.subTest(reason=reason):
                snapshot = valid_snapshot()
                mutate(snapshot["release_snapshot"])
                self.assertEqual(self.evaluate(snapshot).reason, reason)

    def test_confirmation_sample_itself_proves_unwrapped_dwell(self):
        snapshot = valid_snapshot()
        snapshot["release_snapshot"]["release_event_arduino_time_ms"] = (
            (1 << 32) - 25
        )
        snapshot["release_snapshot"][
            "release_confirmation_arduino_time_ms"
        ] = 25
        self.assertTrue(self.evaluate(snapshot).estimator_control_eligible)

        for device_dwell_ms, host_dwell_s in (
            (10, 0.050),
            (50, 0.010),
            (70, 0.050),
        ):
            with self.subTest(
                device_dwell_ms=device_dwell_ms,
                host_dwell_s=host_dwell_s,
            ):
                snapshot = valid_snapshot()
                release = snapshot["release_snapshot"]
                release["release_confirmation_arduino_time_ms"] = (
                    release["release_event_arduino_time_ms"]
                    + device_dwell_ms
                )
                release["release_confirmation_monotonic_s"] = (
                    release["release_event_monotonic_s"] + host_dwell_s
                )
                self.assertEqual(
                    self.evaluate(snapshot).reason,
                    "release_confirmation_order_invalid",
                )

    def test_reported_seed_skew_cannot_hide_old_unwrapped_seed(self):
        snapshot = valid_snapshot()
        release = snapshot["release_snapshot"]
        release["state_seed_cf_timestamp_ms"] = 800
        release["state_seed_unwrapped_timestamp_ms"] = 800
        release["state_seed_transport_cf_timestamp_ms"] = 802
        release["state_seed_transport_minus_source_timestamp_ms"] = 2.0
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "release_seed_not_same_atomic_packed_epoch",
        )

    def test_seed_requires_same_atomic_packet_and_epoch(self):
        snapshot = valid_snapshot()
        snapshot["release_snapshot"]["state_seed_packet_sequence"] = 41
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "release_seed_not_same_atomic_packed_epoch",
        )
        snapshot = valid_snapshot()
        snapshot["release_snapshot"][
            "state_seed_same_atomic_packed_epoch"
        ] = False
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "release_seed_not_same_atomic_packed_epoch",
        )

    def test_strict_position_timing_and_minimum_updates_are_required(self):
        snapshot = valid_snapshot()
        snapshot["position_timing_scientifically_valid"] = False
        self.assertEqual(self.evaluate(snapshot).reason, "position_timing_not_strict")
        snapshot = valid_snapshot()
        snapshot["last_position_timing_basis"] = "host_after_wait"
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "position_timestamp_basis_untrusted",
        )
        snapshot = valid_snapshot()
        snapshot["post_release_ekf"]["position_update_count"] = 2
        snapshot["strict_position_time_update_count"] = 2
        snapshot["strict_position_timestamp_uncertainties_ms"] = [0.0] * 2
        self.assertEqual(self.evaluate(snapshot).reason, "insufficient_position_updates")

    def test_position_timestamp_uncertainty_is_recorded_and_zero_only(self):
        snapshot = valid_snapshot()
        snapshot["strict_position_timestamp_uncertainties_ms"][4] = 0.1
        snapshot["max_strict_position_timestamp_uncertainty_ms"] = 0.1
        config = replace(
            self.config, max_position_timestamp_uncertainty_ms=1.0
        )
        self.assertEqual(
            self.evaluate(snapshot, config=config).reason,
            "position_timestamp_uncertainty_unaccounted",
        )

        snapshot = valid_snapshot()
        snapshot["max_strict_position_timestamp_uncertainty_ms"] = 0.1
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "position_timestamp_uncertainty_evidence_inconsistent",
        )

        snapshot = valid_snapshot()
        snapshot["position_timestamp_uncertainty_accounting"] = "claimed"
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "position_timestamp_uncertainty_contract_missing",
        )

    def test_raw_unwrapped_epochs_reject_aliases_and_accept_wraparound(self):
        snapshot = valid_snapshot()
        snapshot["post_release_ekf"]["unwrapped_timestamp_ms"] = (
            CF_TIMESTAMP_MODULUS_MS + 999
        )
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "ekf_timestamp_raw_unwrapped_mismatch",
        )

        snapshot = valid_snapshot()
        offset = CF_TIMESTAMP_MODULUS_MS
        for mapping, raw_key, unwrapped_key in (
            (snapshot["release_snapshot"], "release_event_cf_timestamp_ms",
             "release_event_unwrapped_cf_timestamp_ms"),
            (snapshot["release_snapshot"], "state_seed_cf_timestamp_ms",
             "state_seed_unwrapped_timestamp_ms"),
            (snapshot["release_snapshot"], "release_gyro_cf_timestamp_ms",
             "release_gyro_unwrapped_timestamp_ms"),
            (snapshot, "first_strict_position_update_cf_timestamp_ms",
             "first_strict_position_update_unwrapped_timestamp_ms"),
            (snapshot, "last_strict_position_update_cf_timestamp_ms",
             "last_strict_position_update_unwrapped_timestamp_ms"),
            (snapshot["post_release_ekf"], "cf_timestamp_ms",
             "unwrapped_timestamp_ms"),
            (snapshot["post_release_control_epoch"], "cf_timestamp_ms",
             "unwrapped_timestamp_ms"),
        ):
            mapping[unwrapped_key] = mapping[raw_key] + offset
        snapshot["release_snapshot"]["unwrapped_timestamp_ms"] = 900 + offset
        decision = self.evaluate(
            snapshot,
            now_cf_timestamp_ms=1000,
            now_unwrapped_cf_timestamp_ms=1000 + offset,
        )
        self.assertTrue(decision.estimator_control_eligible)
        self.assertEqual(decision.ekf_age_ms, 0.0)

        stale = valid_snapshot()
        stale["post_release_control_epoch"]["unwrapped_timestamp_ms"] = (
            CF_TIMESTAMP_MODULUS_MS + 1000
        )
        decision = self.evaluate(
            stale,
            now_unwrapped_cf_timestamp_ms=CF_TIMESTAMP_MODULUS_MS + 1000,
        )
        self.assertFalse(decision.estimator_control_eligible)
        self.assertEqual(decision.reason, "estimator_control_epoch_mismatch")
        self.assertEqual(
            self.evaluate(valid_snapshot(), now_cf_timestamp_ms=True).reason,
            "current_timestamp_invalid_type_or_sign",
        )
        self.assertEqual(
            self.evaluate(valid_snapshot(), now_cf_timestamp_ms=1000.5).reason,
            "current_timestamp_invalid_type_or_sign",
        )
        self.assertEqual(
            self.evaluate(
                valid_snapshot(), now_unwrapped_cf_timestamp_ms=None
            ).reason,
            "current_timestamp_invalid_type_or_sign",
        )

    def test_release_mapping_and_atomic_seed_are_causal_across_raw_wrap(self):
        snapshot = valid_snapshot()
        modulus = CF_TIMESTAMP_MODULUS_MS
        release = snapshot["release_snapshot"]
        release.update({
            "release_event_cf_timestamp_ms": modulus - 2,
            "release_event_unwrapped_cf_timestamp_ms": modulus - 2,
            "release_gyro_cf_timestamp_ms": modulus - 2,
            "release_gyro_unwrapped_timestamp_ms": modulus - 2,
            "state_seed_cf_timestamp_ms": modulus - 2,
            "state_seed_unwrapped_timestamp_ms": modulus - 2,
            "state_seed_transport_cf_timestamp_ms": 0,
            "state_seed_transport_minus_source_timestamp_ms": 2.0,
            "release_gyro_transport_cf_timestamp_ms": 0,
            "release_gyro_transport_minus_source_timestamp_ms": 2.0,
            "cf_timestamp_ms": modulus - 2,
            "unwrapped_timestamp_ms": modulus - 2,
            "release_event_to_gyro_skew_cf_ms": 0.0,
            "release_event_to_gyro_skew_s": -0.003,
        })
        snapshot.update({
            "first_strict_position_update_cf_timestamp_ms": 50,
            "first_strict_position_update_unwrapped_timestamp_ms": (
                modulus + 50
            ),
            "last_strict_position_update_cf_timestamp_ms": 80,
            "last_strict_position_update_unwrapped_timestamp_ms": (
                modulus + 80
            ),
            "strict_position_update_span_ms": 30.0,
            "post_release_device_time_coverage_ms": 102.0,
        })
        snapshot["post_release_ekf"].update({
            "cf_timestamp_ms": 100,
            "unwrapped_timestamp_ms": modulus + 100,
        })
        snapshot["post_release_control_epoch"].update({
            "cf_timestamp_ms": 100,
            "unwrapped_timestamp_ms": modulus + 100,
            "transport_cf_timestamp_ms": 102,
            "transport_minus_source_timestamp_ms": 2.0,
        })
        decision = self.evaluate(
            snapshot,
            now_cf_timestamp_ms=100,
            now_unwrapped_cf_timestamp_ms=modulus + 100,
        )
        self.assertTrue(decision.estimator_control_eligible, decision)

    def test_host_receive_age_and_same_epoch_rate_fail_closed(self):
        self.assertEqual(
            self.evaluate(
                valid_snapshot(), now_host_receive_age_s=-0.001
            ).reason,
            "current_host_receive_age_invalid",
        )
        snapshot = valid_snapshot()
        snapshot["post_release_control_epoch"]["host_receive_age_s"] = 0.031
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "current_control_epoch_host_stale",
        )
        snapshot = valid_snapshot()
        snapshot["post_release_control_epoch"][
            "legacy_body_rate_rad_s"
        ][0] = float("nan")
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "current_control_epoch_state_invalid",
        )

    def test_position_must_be_causal_to_release_ekf_and_current_epoch(self):
        snapshot = valid_snapshot()
        snapshot["last_strict_position_update_cf_timestamp_ms"] = 1001
        snapshot["last_strict_position_update_unwrapped_timestamp_ms"] = 1001
        snapshot["strict_position_update_span_ms"] = 71.0
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "estimator_epoch_order_noncausal",
        )

    def test_observability_requires_coverage_propagation_and_position_span(self):
        cases = (
            (
                "insufficient_post_release_device_time_coverage",
                replace(
                    self.config,
                    min_post_release_device_time_coverage_ms=101.0,
                ),
            ),
            (
                "insufficient_inertial_propagation",
                replace(
                    self.config,
                    min_inertial_propagation_samples=81,
                ),
            ),
            (
                "insufficient_position_update_span",
                replace(self.config, min_position_update_span_ms=51.0),
            ),
        )
        for reason, config in cases:
            with self.subTest(reason=reason):
                self.assertEqual(
                    self.evaluate(valid_snapshot(), config=config).reason,
                    reason,
                )

        snapshot = valid_snapshot()
        current = snapshot["post_release_ekf"]["covariance_diagonal"]
        initial = snapshot["release_snapshot"][
            "initial_covariance_diagonal"
        ]
        initial[6:8] = current[6:8]
        self.assertEqual(
            self.evaluate(snapshot).reason, "tilt_information_insufficient"
        )

    def test_every_post_release_imu_propagation_must_be_atomic(self):
        snapshot = valid_snapshot()
        snapshot["post_release_strict_atomic_imu_count"] = 79
        snapshot["post_release_nonatomic_imu_count"] = 1
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "post_release_imu_atomicity_untrusted",
        )

    def test_imu_quality_requires_calibration_and_clean_exact_counts(self):
        snapshot = valid_snapshot()
        snapshot["post_release_imu_quality_calibrated"] = False
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "post_release_imu_quality_calibration_untrusted",
        )

        for mutate in (
            lambda snapshot: snapshot.update(
                post_release_imu_quality_accepted_count=79
            ),
            lambda snapshot: snapshot.update(
                post_release_imu_quality_rejected_count=1
            ),
            lambda snapshot: snapshot.update(
                post_release_imu_quality_tainted_count=1
            ),
            lambda snapshot: snapshot["release_snapshot"].update(
                release_replay_imu_quality_accepted_count=79
            ),
            lambda snapshot: snapshot["release_snapshot"].update(
                release_replay_imu_quality_rejected_count=1
            ),
        ):
            snapshot = valid_snapshot()
            mutate(snapshot)
            self.assertEqual(
                self.evaluate(snapshot).reason,
                "post_release_imu_quality_evidence_untrusted",
            )

        snapshot = valid_snapshot()
        snapshot["post_release_control_epoch"]["strict_atomic_imu"] = False
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "post_release_imu_atomicity_untrusted",
        )

    def test_yaw_uses_absolute_sigma_not_unobservable_short_window_gain(self):
        snapshot = valid_snapshot()
        snapshot["release_snapshot"]["initial_covariance_diagonal"][8] = (
            0.02 ** 2
        )
        decision = self.evaluate(snapshot)
        self.assertTrue(decision.estimator_control_eligible, decision)
        self.assertLess(decision.attitude_variance_reduction_rad2[2], 0.0)

    def test_negative_imu_gap_and_invalid_full_covariance_fail_closed(self):
        snapshot = valid_snapshot()
        snapshot["post_release_ekf"]["max_imu_gap_ms"] = -0.001
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "estimate_numeric_contract_invalid",
        )

        snapshot = valid_snapshot()
        ekf = snapshot["post_release_ekf"]
        ekf["covariance_matrix"][0][1] = 0.001
        ekf["covariance_matrix"][1][0] = 0.001
        matrix = np.asarray(ekf["covariance_matrix"], dtype=float)
        ekf["covariance_min_eigenvalue"] = float(np.min(
            np.linalg.eigvalsh(matrix)
        ))
        self.assertEqual(
            self.evaluate(snapshot).reason, "covariance_contract_invalid"
        )

        snapshot = valid_snapshot()
        snapshot["post_release_ekf"]["covariance_matrix"][0][1] = 1e-4
        self.assertEqual(
            self.evaluate(snapshot).reason, "covariance_contract_invalid"
        )

    def test_covariance_eigendecomposition_failure_fails_closed(self):
        with mock.patch(
            "Interaction.post_release_estimator_gate.np.linalg.eigvalsh",
            side_effect=np.linalg.LinAlgError("did not converge"),
        ):
            decision = self.evaluate(valid_snapshot())

        self.assertFalse(decision.estimator_control_eligible)
        self.assertEqual(
            decision.reason, "covariance_eigendecomposition_failed"
        )

    def test_nonfinite_covariance_eigenvalues_fail_closed(self):
        eigenvalues = np.zeros(15, dtype=float)
        eigenvalues[-1] = np.nan
        with mock.patch(
            "Interaction.post_release_estimator_gate.np.linalg.eigvalsh",
            return_value=eigenvalues,
        ):
            decision = self.evaluate(valid_snapshot())

        self.assertFalse(decision.estimator_control_eligible)
        self.assertEqual(
            decision.reason, "covariance_eigendecomposition_invalid"
        )

    def test_rejections_imu_gap_and_covariance_each_fail_closed(self):
        mutations = (
            ("position_rejection_fraction_exceeded", lambda s: s["post_release_ekf"].update(rejected_position_count=10)),
            ("imu_gap_exceeded", lambda s: s["post_release_ekf"].update(max_imu_gap_ms=6.0)),
            ("position_uncertainty_exceeded", lambda s: self.set_covariance_diagonal(s, 0, 0.03 ** 2)),
            ("velocity_uncertainty_exceeded", lambda s: self.set_covariance_diagonal(s, 3, 0.21 ** 2)),
            ("attitude_uncertainty_exceeded", lambda s: self.set_covariance_diagonal(s, 6, 0.10 ** 2)),
            ("attitude_uncertainty_exceeded", lambda s: self.set_covariance_diagonal(s, 8, math.radians(6.0) ** 2)),
            ("gyro_bias_uncertainty_exceeded", lambda s: self.set_covariance_diagonal(s, 9, math.radians(1.1) ** 2)),
            ("accel_bias_uncertainty_exceeded", lambda s: self.set_covariance_diagonal(s, 12, 0.11 ** 2)),
        )
        for reason, mutate in mutations:
            with self.subTest(reason=reason):
                snapshot = valid_snapshot()
                mutate(snapshot)
                self.assertEqual(self.evaluate(snapshot).reason, reason)

    def test_malformed_numeric_state_never_raises_into_controller(self):
        for bad_value in (float("nan"), -1.0, "bad"):
            with self.subTest(value=bad_value):
                snapshot = copy.deepcopy(valid_snapshot())
                snapshot["post_release_ekf"]["covariance_diagonal"][0] = bad_value
                decision = self.evaluate(snapshot)
                self.assertFalse(decision.estimator_control_eligible)
                self.assertEqual(decision.reason, "estimate_numeric_contract_invalid")
        snapshot = valid_snapshot()
        snapshot["approximate_position_time_update_count"] = "bad"
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "position_update_count_contract_invalid",
        )
        snapshot = valid_snapshot()
        snapshot["post_release_ekf"]["position_update_count"] = True
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "position_update_counts_invalid",
        )
        snapshot = valid_snapshot()
        snapshot["post_release_ekf"]["position_update_count"] = 10.5
        self.assertEqual(
            self.evaluate(snapshot).reason,
            "position_update_counts_invalid",
        )


if __name__ == "__main__":
    unittest.main()
