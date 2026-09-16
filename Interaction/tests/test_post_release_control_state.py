import math
import unittest

from Interaction.log_manager import CONTACT_SOURCE_TIMESTAMP_BASIS
from Interaction.post_release_control_state import (
    post_release_control_state_candidate,
)
from Interaction.post_release_estimator_gate import (
    PostReleaseEstimatorGateConfig,
)
from Interaction.tests.test_post_release_estimator_gate import valid_snapshot


class PostReleaseControlStateTests(unittest.TestCase):
    def candidate(self, snapshot):
        return post_release_control_state_candidate(
            snapshot,
            now_cf_timestamp_ms=1000,
            now_unwrapped_cf_timestamp_ms=1000,
            now_timestamp_basis=CONTACT_SOURCE_TIMESTAMP_BASIS,
            now_host_receive_age_s=0.005,
            gate_config=PostReleaseEstimatorGateConfig(enabled=True),
        )

    def test_extracts_ekf_state_without_granting_command_authority(self):
        snapshot = valid_snapshot()
        snapshot["post_release_ekf"].update({
            "position_m": [0.1, -0.2, 0.9],
            "velocity_m_s": [0.3, 0.02, -0.01],
            "quaternion_wxyz": [
                math.cos(math.radians(5.0)),
                math.sin(math.radians(5.0)),
                0.0,
                0.0,
            ],
        })
        candidate = self.candidate(snapshot)
        self.assertTrue(candidate.ready)
        self.assertFalse(candidate.candidate_grants_command_authority)
        self.assertFalse(candidate.command_history_used)
        self.assertEqual(candidate.position_m, (0.1, -0.2, 0.9))
        self.assertEqual(candidate.velocity_m_s, (0.3, 0.02, -0.01))
        self.assertAlmostEqual(
            math.degrees(candidate.orientation_rpy_rad[0]), 10.0
        )
        self.assertEqual(candidate.angular_velocity_rad_s, (0.01, -0.02, 0.03))
        self.assertEqual(candidate.attitude_std_deg, (
            math.degrees(0.03),
            math.degrees(0.03),
            math.degrees(0.03),
        ))
        self.assertEqual(candidate.gyro_bias_std_deg_s, (
            math.degrees(0.01),
            math.degrees(0.01),
            math.degrees(0.01),
        ))
        self.assertIsNone(candidate.body_rate_std_deg_s)
        self.assertIsNone(
            candidate.body_rate_measurement_calibration_id
        )

    def test_named_body_rate_measurement_uncertainty_is_exposed(self):
        snapshot = valid_snapshot()
        snapshot["post_release_ekf"].update({
            "position_m": [0.1, -0.2, 0.9],
            "velocity_m_s": [0.3, 0.02, -0.01],
            "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
        })
        snapshot.update({
            "body_rate_measurement_calibrated": True,
            "body_rate_measurement_std_deg_s": [0.2, 0.3, 0.4],
            "body_rate_measurement_calibration_id": (
                "crazysim_exact_imu_quality_v1"
            ),
        })

        candidate = self.candidate(snapshot)

        self.assertTrue(candidate.ready)
        self.assertEqual(candidate.body_rate_std_deg_s, (0.2, 0.3, 0.4))
        self.assertEqual(
            candidate.body_rate_measurement_calibration_id,
            "crazysim_exact_imu_quality_v1",
        )
        self.assertFalse(candidate.candidate_grants_command_authority)

    def test_invalid_body_rate_calibration_fails_closed(self):
        for std, calibration_id in (
            ([0.2, -0.1, 0.4], "crazysim_exact_imu_quality_v1"),
            ([0.2, 0.0, 0.4], "crazysim_exact_imu_quality_v1"),
            ([0.2, 0.3, 0.4], "different-rate-fit-v1"),
        ):
            with self.subTest(std=std, calibration_id=calibration_id):
                snapshot = valid_snapshot()
                snapshot["post_release_ekf"].update({
                    "position_m": [0.1, -0.2, 0.9],
                    "velocity_m_s": [0.3, 0.02, -0.01],
                    "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                })
                snapshot.update({
                    "body_rate_measurement_calibrated": True,
                    "body_rate_measurement_std_deg_s": std,
                    "body_rate_measurement_calibration_id": calibration_id,
                })

                candidate = self.candidate(snapshot)

                self.assertFalse(candidate.ready)
                self.assertEqual(
                    candidate.reason, "body_rate_measurement_evidence_invalid"
                )

    def test_ineligible_estimator_never_silently_returns_another_state(self):
        snapshot = valid_snapshot()
        snapshot["post_release_velocity_process_command_independent"] = False
        snapshot["onboard_ekf"] = {
            "position_m": [9.0, 9.0, 9.0],
            "velocity_m_s": [9.0, 9.0, 9.0],
        }
        candidate = self.candidate(snapshot)
        self.assertFalse(candidate.ready)
        self.assertEqual(candidate.reason, "velocity_process_depends_on_command")
        self.assertIsNone(candidate.position_m)
        self.assertIsNone(candidate.velocity_m_s)

    def test_nonfinite_state_fails_closed_after_gate(self):
        snapshot = valid_snapshot()
        snapshot["post_release_ekf"].update({
            "position_m": [float("nan"), 0.0, 1.0],
            "velocity_m_s": [0.0, 0.0, 0.0],
        })
        candidate = self.candidate(snapshot)
        self.assertFalse(candidate.ready)
        self.assertEqual(candidate.reason, "estimate_state_invalid")

    def test_invalid_same_epoch_rate_never_falls_back(self):
        snapshot = valid_snapshot()
        snapshot["post_release_control_epoch"][
            "legacy_body_rate_rad_s"
        ] = [0.0, float("nan"), 0.0]
        snapshot["onboard_ekf"] = {
            "angular_velocity_rad_s": [9.0, 9.0, 9.0],
        }
        candidate = self.candidate(snapshot)
        self.assertFalse(candidate.ready)
        self.assertEqual(candidate.reason, "current_control_epoch_state_invalid")
        self.assertIsNone(candidate.angular_velocity_rad_s)

    def test_ineligible_candidate_preserves_available_uncertainty(self):
        snapshot = valid_snapshot()
        snapshot["post_release_ekf"]["max_imu_gap_ms"] = 6.0
        candidate = self.candidate(snapshot)
        self.assertFalse(candidate.ready)
        self.assertEqual(candidate.reason, "imu_gap_exceeded")
        self.assertIsNotNone(candidate.attitude_std_deg)
        self.assertIsNotNone(candidate.gyro_bias_std_deg_s)
        self.assertIsNone(candidate.position_m)


if __name__ == "__main__":
    unittest.main()
