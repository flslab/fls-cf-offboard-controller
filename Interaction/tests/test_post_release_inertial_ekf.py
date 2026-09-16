import math
import unittest

import numpy as np

from Interaction.contact_attitude_observer import (
    legacy_rpy_from_quaternion,
    quaternion_from_native_rpy,
)
from Interaction.post_release_inertial_ekf import (
    GRAVITY_M_S2,
    PostReleaseInertialEkf,
    error_state_reset_jacobian,
    rotation_matrix,
)


class PostReleaseInertialEkfTests(unittest.TestCase):
    def make_filter(self, **kwargs):
        defaults = dict(
            position_m=[0.0, 0.0, 1.0],
            velocity_m_s=[0.0, 0.0, 0.0],
            quaternion_wxyz=quaternion_from_native_rpy(0.0, 0.0, 0.0),
            gyro_bias_rad_s=[0.0, 0.0, 0.0],
            cf_timestamp_ms=100,
        )
        defaults.update(kwargs)
        return PostReleaseInertialEkf(**defaults)

    def test_accelerometer_drives_velocity_without_command_history(self):
        ekf = self.make_filter()
        initial = ekf.snapshot()
        self.assertEqual(len(initial.covariance_diagonal), 15)
        self.assertTrue(all(value > 0.0 for value in initial.covariance_diagonal))
        # A 0.2 g body-X specific-force component is integrated immediately.
        for timestamp in range(101, 111):
            result = ekf.propagate(timestamp, [0, 0, 0], [0.2, 0, 1.0])
        self.assertAlmostEqual(result.velocity_m_s[0], 0.2 * GRAVITY_M_S2 * 0.010)
        self.assertAlmostEqual(result.velocity_m_s[2], 0.0, places=12)

    def test_position_only_extpos_can_correct_attitude_through_inertial_coupling(self):
        true_pitch = math.radians(8.0)
        true_quaternion = quaternion_from_native_rpy(0.0, true_pitch, 0.0)
        true_rotation = rotation_matrix(true_quaternion)
        specific_force = np.array([0.0, 0.0, GRAVITY_M_S2])
        true_acceleration = true_rotation @ specific_force + [0.0, 0.0, -GRAVITY_M_S2]
        true_position = np.array([0.0, 0.0, 1.0])
        true_velocity = np.zeros(3)
        ekf = self.make_filter()

        dt = 0.001
        for step in range(1, 1001):
            true_position += true_velocity * dt + 0.5 * true_acceleration * dt * dt
            true_velocity += true_acceleration * dt
            ekf.propagate(100 + step, [0, 0, 0], [0, 0, 1.0])
            if step % 10 == 0:
                ekf.update_extpos(true_position, std_m=0.001)

        estimated_native_pitch = -legacy_rpy_from_quaternion(
            ekf.snapshot().quaternion_wxyz
        )[1]
        self.assertLess(abs(math.degrees(estimated_native_pitch - true_pitch)), 1.0)
        self.assertGreater(ekf.snapshot().position_update_count, 50)

    def test_first_imu_step_builds_analytic_position_attitude_cross_covariance(self):
        ekf = self.make_filter()
        initial_attitude_variance = math.radians(4.0) ** 2
        dt = 0.005

        ekf.propagate(105, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0])

        force = np.array([0.0, 0.0, GRAVITY_M_S2])
        phi_position_attitude = -0.5 * rotation_matrix(
            [1.0, 0.0, 0.0, 0.0]
        ) @ np.array([
            [0.0, -force[2], force[1]],
            [force[2], 0.0, -force[0]],
            [-force[1], force[0], 0.0],
        ]) * dt * dt
        expected_attitude_position = (
            initial_attitude_variance * phi_position_attitude.T
        )
        np.testing.assert_allclose(
            ekf.covariance[6:9, 0:3],
            expected_attitude_position,
            rtol=1e-12,
            atol=1e-18,
        )
        self.assertGreater(ekf.covariance[7, 0], 0.0)

        before = np.asarray(ekf.snapshot().quaternion_wxyz)
        ekf.update_extpos([0.001, 0.0, 1.0], std_m=0.001)
        after = np.asarray(ekf.snapshot().quaternion_wxyz)
        self.assertGreater(float(np.linalg.norm(after - before)), 1e-8)

    def test_accelerometer_white_noise_discretization_is_analytic_and_psd(self):
        ekf = self.make_filter(
            quaternion_wxyz=quaternion_from_native_rpy(0.3, -0.2, 0.7)
        )
        ekf.covariance = np.zeros((15, 15))
        dt = 0.005
        variance_density = ekf.config.accel_noise_m_s2_sqrt_hz ** 2

        ekf.propagate(105, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0])

        np.testing.assert_allclose(
            ekf.covariance[0:3, 0:3],
            np.eye(3) * variance_density * dt ** 3 / 3.0,
            rtol=1e-12,
            atol=1e-18,
        )
        np.testing.assert_allclose(
            ekf.covariance[0:3, 3:6],
            np.eye(3) * variance_density * dt * dt / 2.0,
            rtol=1e-12,
            atol=1e-18,
        )
        np.testing.assert_allclose(
            ekf.covariance[3:6, 0:3],
            np.eye(3) * variance_density * dt * dt / 2.0,
            rtol=1e-12,
            atol=1e-18,
        )
        np.testing.assert_allclose(
            ekf.covariance[3:6, 3:6],
            np.eye(3) * variance_density * dt,
            rtol=1e-12,
            atol=1e-18,
        )
        np.testing.assert_allclose(
            ekf.covariance, ekf.covariance.T, rtol=0.0, atol=1e-18
        )
        self.assertGreaterEqual(
            float(np.min(np.linalg.eigvalsh(ekf.covariance))), -1e-15
        )

    def test_position_update_never_uses_an_orientation_measurement(self):
        ekf = self.make_filter()
        ekf.propagate(101, [0, 0, 0], [0, 0, 1])
        before = ekf.snapshot()
        after = ekf.update_extpos([0.001, 0.0, 1.0])
        self.assertTrue(after.valid)
        self.assertEqual(after.reason, "extpos_fused")
        self.assertEqual(before.cf_timestamp_ms, after.cf_timestamp_ms)

    def test_duplicate_is_noop_and_gap_invalidates(self):
        ekf = self.make_filter()
        first = ekf.propagate(101, [0, 0, 0], [0, 0, 1])
        duplicate = ekf.propagate(101, [100, 0, 0], [1, 0, 1])
        self.assertTrue(duplicate.valid)
        self.assertEqual(duplicate.reason, "duplicate_timestamp")
        self.assertEqual(first.quaternion_wxyz, duplicate.quaternion_wxyz)
        failed = ekf.propagate(107, [0, 0, 0], [0, 0, 1])
        self.assertFalse(failed.valid)
        self.assertEqual(failed.reason, "imu_gap_exceeded")

    def test_extpos_outlier_is_rejected(self):
        ekf = self.make_filter()
        result = ekf.update_extpos([2.0, 0.0, 1.0])
        self.assertEqual(result.reason, "extpos_innovation_rejected")
        self.assertEqual(result.rejected_position_count, 1)

    def test_attitude_injection_resets_right_error_covariance_frame(self):
        correction = np.array([0.1, -0.2, 0.3])
        reset = error_state_reset_jacobian(correction)

        np.testing.assert_allclose(reset[:6, :6], np.eye(6))
        np.testing.assert_allclose(reset[9:, 9:], np.eye(6))
        np.testing.assert_allclose(
            reset[6:9, 6:9],
            [[1.0, 0.15, 0.10],
             [-0.15, 1.0, 0.05],
             [-0.10, -0.05, 1.0]],
        )

    def test_unwrapped_release_epoch_survives_counter_wrap(self):
        ekf = self.make_filter(
            cf_timestamp_ms=5,
            unwrapped_timestamp_ms=(1 << 24) + 5,
        )

        result = ekf.propagate(6, [0, 0, 0], [0, 0, 1])

        self.assertEqual(result.unwrapped_timestamp_ms, (1 << 24) + 6)


if __name__ == "__main__":
    unittest.main()
