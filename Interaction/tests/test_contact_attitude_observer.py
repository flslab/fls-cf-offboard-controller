import math
import unittest

import numpy as np

from Interaction.contact_attitude_observer import (
    CF_TIMESTAMP_MODULUS_MS,
    ContactAttitudeConfig,
    ContactAttitudeObserver,
    integrate_body_rate,
    legacy_rpy_from_quaternion,
    quaternion_from_accel_and_legacy_yaw,
    quaternion_from_native_rpy,
)


def quaternion_distance_deg(left, right):
    left = np.asarray(left, dtype=float)
    right = np.asarray(right, dtype=float)
    cosine = min(1.0, abs(float(left @ right)))
    return math.degrees(2.0 * math.acos(cosine))


class ContactAttitudeMathTests(unittest.TestCase):
    def test_accelerometer_recovers_native_tilt_and_legacy_pitch(self):
        native_roll = math.radians(12.0)
        native_pitch = math.radians(-7.0)
        yaw = math.radians(90.0)
        accel = np.array([
            -math.sin(native_pitch),
            math.sin(native_roll) * math.cos(native_pitch),
            math.cos(native_roll) * math.cos(native_pitch),
        ])
        quaternion = quaternion_from_accel_and_legacy_yaw(accel, yaw)
        legacy = legacy_rpy_from_quaternion(quaternion)
        np.testing.assert_allclose(
            legacy, [native_roll, -native_pitch, yaw], atol=1e-12
        )

    def test_exact_constant_body_rate_for_all_yaws_axes_and_signs(self):
        for yaw_deg in (0.0, 90.0, 180.0):
            for axis in range(3):
                for sign in (-1.0, 1.0):
                    with self.subTest(yaw=yaw_deg, axis=axis, sign=sign):
                        initial = quaternion_from_native_rpy(
                            0.0, 0.0, math.radians(yaw_deg)
                        )
                        rate = np.zeros(3)
                        rate[axis] = sign * math.radians(80.0)
                        actual = integrate_body_rate(initial, rate, 0.025)
                        delta = quaternion_from_native_rpy(
                            *(rate * 0.025)
                        ) if axis < 2 else quaternion_from_native_rpy(
                            0.0, 0.0, rate[2] * 0.025
                        )
                        # Expected body increment is initial * delta.
                        from Interaction.contact_attitude_observer import quaternion_multiply
                        expected = quaternion_multiply(initial, delta)
                        self.assertLess(quaternion_distance_deg(actual, expected), 1e-6)

    def test_legacy_rate_sign_matches_firmware_compressed_state(self):
        observer = self.make_ready_observer(yaw_deg=180.0)
        observer.begin_contact()
        result = observer.add_gyro_sample(302, [10.0, 20.0, -30.0])
        np.testing.assert_allclose(
            result.legacy_body_rate_rad_s,
            np.radians([10.0, -20.0, -30.0]), atol=1e-12,
        )

    @staticmethod
    def make_ready_observer(yaw_deg=0.0, start=0, bias=(0.0, 0.0, 0.0), **kwargs):
        config = ContactAttitudeConfig(
            alignment_window_ms=300,
            alignment_min_samples=4,
            alignment_max_sample_gap_ms=110,
            **kwargs,
        )
        observer = ContactAttitudeObserver(config)
        for offset in (0, 100, 200, 300):
            observer.add_alignment_sample(
                (start + offset) % CF_TIMESTAMP_MODULUS_MS,
                [0.0, 0.0, 1.0], bias, yaw_deg,
            )
        return observer


class ContactAttitudeObserverTests(unittest.TestCase):
    def make_ready(self, **kwargs):
        return ContactAttitudeMathTests.make_ready_observer(**kwargs)

    def test_alignment_estimates_bias_then_freezes_accelerometer(self):
        observer = self.make_ready(bias=(0.4, -0.2, 0.1))
        ready = observer.snapshot()
        self.assertTrue(ready.valid)
        np.testing.assert_allclose(ready.gyro_bias_deg_s, [0.4, -0.2, 0.1])
        observer.begin_contact()
        frozen = observer.add_alignment_sample(
            301, [0.8, 0.0, 0.6], [50.0, 0.0, 0.0], 45.0
        )
        self.assertEqual(frozen.reason, "alignment_frozen")
        np.testing.assert_allclose(frozen.legacy_rpy_rad, [0.0, 0.0, 0.0])

    def test_bias_corrected_full_three_axis_integration(self):
        observer = self.make_ready(bias=(0.4, -0.2, 0.1), yaw_deg=90.0)
        observer.begin_contact()
        for timestamp in range(301, 311):
            result = observer.add_gyro_sample(
                timestamp, [40.4, -20.2, 10.1]
            )
        # The final alignment sample at t=300 still contains only the bias;
        # trapezoidal integration correctly assigns half of the first 1 ms
        # step to the newly observed motion.
        expected = integrate_body_rate(
            quaternion_from_native_rpy(0.0, 0.0, math.pi / 2.0),
            np.radians([40.0, -20.0, 10.0]), 0.0095,
        )
        self.assertLess(
            quaternion_distance_deg(result.quaternion_wxyz, expected), 1e-5
        )
        self.assertEqual(result.sample_count, 10)

    def test_wrap_is_valid_but_duplicate_is_noop(self):
        start = CF_TIMESTAMP_MODULUS_MS - 300
        observer = self.make_ready(start=start)
        observer.begin_contact()
        first = observer.add_gyro_sample(1, [10.0, 0.0, 0.0])
        duplicate = observer.add_gyro_sample(1, [99.0, 0.0, 0.0])
        self.assertTrue(first.valid)
        self.assertTrue(duplicate.valid)
        self.assertEqual(duplicate.reason, "duplicate_timestamp")
        self.assertEqual(duplicate.sample_count, first.sample_count)

    def test_backward_timestamp_invalidates(self):
        observer = self.make_ready()
        observer.begin_contact()
        result = observer.add_gyro_sample(299, [0.0, 0.0, 0.0])
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "backward_timestamp")

    def test_gap_above_five_ms_invalidates_without_propagating(self):
        observer = self.make_ready(max_gyro_gap_ms=5.0)
        observer.begin_contact()
        before = observer.snapshot().quaternion_wxyz
        result = observer.add_gyro_sample(306, [100.0, 0.0, 0.0])
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "gyro_gap_exceeded")
        self.assertEqual(result.quaternion_wxyz, before)

    def test_release_latches_quaternion(self):
        observer = self.make_ready()
        observer.begin_contact()
        observer.add_gyro_sample(301, [30.0, 0.0, 0.0])
        released = observer.release()
        ignored = observer.add_gyro_sample(302, [300.0, 0.0, 0.0])
        self.assertTrue(released.valid)
        self.assertEqual(released.phase, "released")
        self.assertEqual(ignored.quaternion_wxyz, released.quaternion_wxyz)

    def test_recontact_can_seed_from_post_release_ekf_state(self):
        observer = ContactAttitudeObserver(ContactAttitudeConfig(
            alignment_window_ms=30, alignment_min_samples=3,
        ))
        seed = quaternion_from_native_rpy(
            math.radians(3.0), math.radians(-4.0), math.radians(20.0)
        )

        started = observer.begin_contact_from_state(
            seed,
            gyro_bias_deg_s=[0.2, -0.1, 0.05],
            cf_timestamp_ms=500,
            unwrapped_timestamp_ms=CF_TIMESTAMP_MODULUS_MS + 500,
            gyro_deg_s=[10.2, 19.9, 0.05],
        )
        propagated = observer.add_gyro_sample(
            501, [10.2, 19.9, 0.05]
        )

        self.assertTrue(started.valid)
        self.assertEqual(
            started.reason, "contact_reseeded_from_post_release_ekf"
        )
        self.assertEqual(started.phase, observer.CONTACT)
        self.assertEqual(
            propagated.unwrapped_timestamp_ms,
            CF_TIMESTAMP_MODULUS_MS + 501,
        )
        expected = integrate_body_rate(
            seed, np.radians([10.0, 20.0, 0.0]), 0.001
        )
        self.assertLess(
            quaternion_distance_deg(propagated.quaternion_wxyz, expected),
            1e-6,
        )

    def test_unstable_acceleration_resets_alignment_window(self):
        observer = ContactAttitudeObserver(ContactAttitudeConfig(
            alignment_window_ms=30, alignment_min_samples=3,
        ))
        observer.add_alignment_sample(0, [0, 0, 1], [0, 0, 0], 0)
        result = observer.add_alignment_sample(10, [0, 0, 1.3], [0, 0, 0], 0)
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "alignment_acceleration_unstable")

    def test_constant_rotation_is_not_learned_as_gyro_bias(self):
        observer = ContactAttitudeObserver(ContactAttitudeConfig(
            alignment_window_ms=20,
            alignment_min_samples=3,
            alignment_max_sample_gap_ms=10,
        ))
        for timestamp in (0, 10, 20):
            result = observer.add_alignment_sample(
                timestamp, [0, 0, 1], [6.0, 0.0, 0.0], 0.0
            )
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "alignment_mean_rotation_exceeded")

    def test_ready_alignment_survives_contact_onset_acceleration(self):
        observer = ContactAttitudeObserver(ContactAttitudeConfig(
            alignment_window_ms=20,
            alignment_min_samples=3,
            alignment_max_sample_gap_ms=10,
        ))
        for timestamp in (0, 10, 20):
            result = observer.add_alignment_sample(
                timestamp, [0, 0, 1], [0, 0, 0], 0.0
            )
        self.assertTrue(result.valid)
        self.assertEqual(result.phase, observer.READY)

        result = observer.add_alignment_sample(
            21, [0, 0, 1.3], [10, 0, 0], 0.0
        )

        self.assertTrue(result.valid)
        self.assertEqual(result.phase, observer.READY)
        self.assertEqual(result.cf_timestamp_ms, 21)
        self.assertEqual(
            result.reason,
            'ready_gyro_propagating_after_alignment_acceleration_unstable',
        )
        self.assertTrue(observer.begin_contact().valid)

    def test_alignment_does_not_bridge_an_imu_sample_hole(self):
        observer = ContactAttitudeObserver(ContactAttitudeConfig(
            alignment_window_ms=20,
            alignment_min_samples=3,
            alignment_max_sample_gap_ms=10,
        ))
        observer.add_alignment_sample(0, [0, 0, 1], [0, 0, 0], 0)
        result = observer.add_alignment_sample(
            20, [0, 0, 1], [0, 0, 0], 0
        )
        self.assertEqual(result.reason, "alignment_sample_gap")
        self.assertEqual(len(observer._alignment_samples), 1)

    def test_directionally_unstable_acceleration_is_not_alignment(self):
        observer = ContactAttitudeObserver(ContactAttitudeConfig(
            alignment_window_ms=20,
            alignment_min_samples=3,
            alignment_max_sample_gap_ms=10,
        ))
        samples = ([0, 0, 1], [0.12, 0, math.sqrt(1 - 0.12 ** 2)],
                   [-0.12, 0, math.sqrt(1 - 0.12 ** 2)])
        for timestamp, acceleration in zip((0, 10, 20), samples):
            result = observer.add_alignment_sample(
                timestamp, acceleration, [0, 0, 0], 0
            )
        self.assertFalse(result.valid)
        self.assertEqual(
            result.reason, "alignment_acceleration_direction_unstable"
        )

    def test_large_constant_tilt_is_not_treated_as_gravity_alignment(self):
        observer = ContactAttitudeObserver(ContactAttitudeConfig(
            alignment_window_ms=20,
            alignment_min_samples=3,
            alignment_max_sample_gap_ms=10,
        ))
        tilt = math.radians(15.0)
        for timestamp in (0, 10, 20):
            result = observer.add_alignment_sample(
                timestamp, [math.sin(tilt), 0, math.cos(tilt)],
                [0, 0, 0], 0,
            )
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "alignment_tilt_exceeded")

    def test_ambiguous_yaw_window_is_rejected(self):
        observer = ContactAttitudeObserver(ContactAttitudeConfig(
            alignment_window_ms=20,
            alignment_min_samples=3,
            alignment_max_sample_gap_ms=10,
        ))
        for timestamp, yaw in zip((0, 10, 20), (0.0, 120.0, 240.0)):
            result = observer.add_alignment_sample(
                timestamp, [0, 0, 1], [0, 0, 0], yaw
            )
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "alignment_yaw_ambiguous")

    def test_slow_yaw_drift_window_is_rejected(self):
        observer = ContactAttitudeObserver(ContactAttitudeConfig(
            alignment_window_ms=20,
            alignment_min_samples=3,
            alignment_max_sample_gap_ms=10,
        ))
        for timestamp, yaw in zip((0, 10, 20), (0.0, 6.0, 12.0)):
            result = observer.add_alignment_sample(
                timestamp, [0, 0, 1], [0, 0, 0], yaw
            )
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "alignment_yaw_unstable")

if __name__ == "__main__":
    unittest.main()
