import unittest

import numpy as np

from Interaction.jerk_limited_braking import (
    JerkLimitedBrakeError,
    make_jerk_limited_brake_profile,
)


class JerkLimitedBrakeProfileTests(unittest.TestCase):
    @staticmethod
    def _roll_out_delayed_first_order_plant(
            profile, initial_velocity_m_s, delay_s=0.15, tau_s=0.08,
            step_s=0.0005):
        acceleration = 0.0
        velocity = float(initial_velocity_m_s)
        minimum_velocity = velocity
        elapsed = 0.0
        horizon = profile.duration_s + delay_s + 10.0 * tau_s
        while elapsed < horizon - 1e-12:
            dt = min(step_s, horizon - elapsed)
            delayed_profile_time = elapsed - delay_s
            target_acceleration = (
                0.0
                if delayed_profile_time < 0.0
                else profile.sample(delayed_profile_time).acceleration_m_s2
            )
            decay = np.exp(-dt / tau_s)
            acceleration_error = acceleration - target_acceleration
            velocity += (
                target_acceleration * dt
                + acceleration_error * tau_s * (1.0 - decay)
            )
            acceleration = (
                target_acceleration + acceleration_error * decay
            )
            minimum_velocity = min(minimum_velocity, velocity)
            elapsed += dt
        return minimum_velocity, velocity, acceleration

    def test_triangular_profile_finishes_at_rest_without_reversal(self):
        profile = make_jerk_limited_brake_profile(0.12, 0.0, 1.0, 4.0)

        self.assertEqual(profile.profile_type, 'triangular')
        initial = profile.sample(0.0)
        terminal = profile.sample(profile.duration_s)
        self.assertAlmostEqual(initial.velocity_m_s, 0.12)
        self.assertAlmostEqual(initial.acceleration_m_s2, 0.0)
        self.assertAlmostEqual(terminal.velocity_m_s, 0.0)
        self.assertAlmostEqual(terminal.acceleration_m_s2, 0.0)
        self.assertEqual(terminal.jerk_m_s3, 0.0)
        samples = [
            profile.sample(value)
            for value in np.linspace(0.0, profile.duration_s, 501)
        ]
        self.assertGreaterEqual(
            min(sample.velocity_m_s for sample in samples), -1e-12
        )
        self.assertGreaterEqual(
            min(sample.position_m for sample in samples), -1e-12
        )

    def test_trapezoidal_profile_respects_acceleration_and_jerk_limits(self):
        profile = make_jerk_limited_brake_profile(0.60, 0.0, 1.0, 4.0)

        self.assertEqual(profile.profile_type, 'trapezoidal')
        samples = [
            profile.sample(value)
            for value in np.linspace(0.0, profile.duration_s, 1001)
        ]
        self.assertLessEqual(
            max(abs(sample.acceleration_m_s2) for sample in samples),
            1.0 + 1e-10,
        )
        self.assertLessEqual(
            max(abs(sample.jerk_m_s3) for sample in samples),
            4.0 + 1e-10,
        )
        self.assertAlmostEqual(samples[-1].velocity_m_s, 0.0)
        self.assertAlmostEqual(samples[-1].acceleration_m_s2, 0.0)

    def test_delayed_first_order_plant_settles_without_reversal(self):
        terminal_margin_m_s = 0.005
        for initial_velocity in (0.30, 0.60):
            for deceleration in (0.40, 0.981):
                with self.subTest(
                        velocity=initial_velocity,
                        deceleration=deceleration):
                    profile = make_jerk_limited_brake_profile(
                        initial_velocity - terminal_margin_m_s,
                        0.0,
                        deceleration,
                        4.0,
                    )
                    minimum, terminal, acceleration = (
                        self._roll_out_delayed_first_order_plant(
                            profile, initial_velocity
                        )
                    )
                    self.assertGreaterEqual(minimum, -0.0005)
                    self.assertAlmostEqual(
                        terminal, terminal_margin_m_s, delta=0.001
                    )
                    self.assertLess(abs(acceleration), 0.0001)

    def test_nonzero_initial_acceleration_is_continuous_and_terminal(self):
        profile = make_jerk_limited_brake_profile(0.45, 0.30, 1.0, 4.0)

        self.assertAlmostEqual(
            profile.sample(0.0).acceleration_m_s2, 0.30
        )
        for boundary in np.cumsum(profile.phase_durations_s)[:-1]:
            before = profile.sample(max(boundary - 1e-8, 0.0))
            after = profile.sample(boundary + 1e-8)
            self.assertAlmostEqual(
                before.position_m, after.position_m, places=6
            )
            self.assertAlmostEqual(
                before.velocity_m_s, after.velocity_m_s, places=6
            )
            self.assertAlmostEqual(
                before.acceleration_m_s2,
                after.acceleration_m_s2,
                places=6,
            )
        terminal = profile.sample(profile.duration_s + 1.0)
        self.assertAlmostEqual(terminal.velocity_m_s, 0.0)
        self.assertAlmostEqual(terminal.acceleration_m_s2, 0.0)

    def test_unavoidable_reverse_from_existing_brake_is_rejected(self):
        with self.assertRaisesRegex(
                JerkLimitedBrakeError, 'cannot be unwound before reversal'):
            make_jerk_limited_brake_profile(0.01, -0.50, 1.0, 4.0)

    def test_invalid_limits_and_reverse_initial_velocity_are_rejected(self):
        with self.assertRaises(ValueError):
            make_jerk_limited_brake_profile(0.2, 0.0, 0.0, 4.0)
        with self.assertRaises(ValueError):
            make_jerk_limited_brake_profile(0.2, 0.0, 1.0, np.nan)
        with self.assertRaises(JerkLimitedBrakeError):
            make_jerk_limited_brake_profile(-0.01, 0.0, 1.0, 4.0)


if __name__ == '__main__':
    unittest.main()
