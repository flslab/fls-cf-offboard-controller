import unittest

import numpy as np

from Interaction.jerk_limited_braking import (
    JerkLimitedBrakeError,
    make_jerk_limited_brake_profile,
    make_septic_brake_profile,
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


class SepticBrakeProfileTests(unittest.TestCase):
    def test_zero_acceleration_profile_has_smooth_terminal_conditions(self):
        profile = make_septic_brake_profile(
            0.60, 0.0, 1.0, 4.0, initial_position_m=-0.25
        )

        self.assertEqual(profile.profile_type, 'septic_trapezoidal')
        self.assertGreater(profile.phase_durations_s[0], 0.0)
        self.assertGreater(profile.phase_durations_s[1], 0.0)
        self.assertGreater(profile.phase_durations_s[2], 0.0)
        initial = profile.sample(0.0)
        terminal = profile.sample(profile.duration_s)
        self.assertAlmostEqual(initial.position_m, -0.25)
        self.assertAlmostEqual(initial.velocity_m_s, 0.60)
        self.assertAlmostEqual(initial.acceleration_m_s2, 0.0)
        self.assertAlmostEqual(initial.jerk_m_s3, 0.0)
        self.assertAlmostEqual(terminal.position_m, profile.stop_position_m)
        self.assertAlmostEqual(terminal.velocity_m_s, 0.0)
        self.assertAlmostEqual(terminal.acceleration_m_s2, 0.0)
        self.assertAlmostEqual(terminal.jerk_m_s3, 0.0)
        self.assertTrue(terminal.complete)

    def test_continuous_profile_respects_limits_and_never_reverses(self):
        cases = (
            (0.05, 0.0, 0.40, 2.0),
            (0.30, 0.0, 0.981, 4.0),
            (0.60, 0.0, 1.0, 4.0),
            (0.45, 0.25, 1.0, 4.0),
            (0.45, -0.25, 1.0, 4.0),
        )
        for velocity, acceleration, max_acceleration, max_jerk in cases:
            with self.subTest(
                    velocity=velocity, acceleration=acceleration):
                profile = make_septic_brake_profile(
                    velocity,
                    acceleration,
                    max_acceleration,
                    max_jerk,
                )
                dense_times = np.linspace(0.0, profile.duration_s, 4001)
                samples = [
                    profile.sample(value) for value in dense_times
                ]
                self.assertGreaterEqual(
                    min(sample.velocity_m_s for sample in samples), -1e-12
                )
                self.assertLessEqual(
                    max(abs(sample.acceleration_m_s2) for sample in samples),
                    max_acceleration + 1e-9,
                )
                self.assertLessEqual(
                    max(abs(sample.jerk_m_s3) for sample in samples),
                    max_jerk + 1e-8,
                )
                positions = np.asarray([
                    profile.sample(value).position_m
                    for value in dense_times
                ])
                self.assertTrue(np.all(np.diff(positions) >= -1e-12))
                self.assertAlmostEqual(
                    positions[-1], profile.stop_position_m
                )

    def test_all_phase_boundaries_are_continuous(self):
        profiles = (
            make_septic_brake_profile(0.45, -0.25, 1.0, 4.0),
            make_septic_brake_profile(0.60, 0.0, 1.0, 4.0),
        )
        self.assertEqual(profiles[0].profile_type, 'septic_triangular')
        self.assertEqual(profiles[1].profile_type, 'septic_trapezoidal')
        for profile in profiles:
            with self.subTest(profile_type=profile.profile_type):
                boundaries = np.unique(
                    np.cumsum(profile.phase_durations_s)[:-1]
                )
                for boundary in boundaries:
                    if boundary <= 0.0:
                        continue
                    at_boundary = profile.sample(boundary)
                    before = profile.sample(boundary - 1e-8)
                    after = profile.sample(boundary + 1e-8)
                    self.assertAlmostEqual(at_boundary.jerk_m_s3, 0.0)
                    for field in (
                            'position_m', 'velocity_m_s',
                            'acceleration_m_s2', 'jerk_m_s3'):
                        self.assertAlmostEqual(
                            getattr(before, field),
                            getattr(after, field),
                            places=6,
                        )

    def test_exact_zoh_samples_do_not_reverse_at_100_hz(self):
        for acceleration in (-0.25, 0.0, 0.25):
            with self.subTest(acceleration=acceleration):
                profile = make_septic_brake_profile(
                    0.45, acceleration, 1.0, 4.0
                )
                velocity = profile.initial_velocity_m_s
                elapsed = 0.0
                interval = 0.01
                minimum_velocity = velocity
                previous_acceleration = profile.initial_acceleration_m_s2
                while elapsed < profile.duration_s - 1e-12:
                    command = profile.sample_zoh(elapsed, interval)
                    self.assertLessEqual(
                        abs(command.acceleration_m_s2), 1.0 + 1e-12
                    )
                    self.assertLessEqual(
                        abs(command.jerk_m_s3), 4.0 + 1e-12
                    )
                    self.assertLessEqual(
                        abs(
                            command.acceleration_m_s2
                            - previous_acceleration
                        ) / interval,
                        4.0 + 1e-9,
                    )
                    velocity += command.acceleration_m_s2 * interval
                    elapsed += interval
                    minimum_velocity = min(minimum_velocity, velocity)
                    expected = profile.sample(elapsed).velocity_m_s
                    self.assertAlmostEqual(velocity, expected, places=12)
                    previous_acceleration = command.acceleration_m_s2
                self.assertGreaterEqual(minimum_velocity, -1e-12)
                self.assertAlmostEqual(velocity, 0.0, places=12)
                after = profile.sample_zoh(elapsed, interval)
                self.assertEqual(after.acceleration_m_s2, 0.0)
                self.assertEqual(after.jerk_m_s3, 0.0)

    def test_zoh_sample_crosses_phase_and_end_boundaries_exactly(self):
        profile = make_septic_brake_profile(0.60, 0.0, 1.0, 4.0)
        for boundary in (*np.cumsum(profile.phase_durations_s),):
            start = max(float(boundary) - 0.007, 0.0)
            interval = 0.013
            command = profile.sample_zoh(start, interval)
            expected_end = profile.sample(start + interval)
            actual_end_velocity = (
                command.velocity_m_s
                + command.acceleration_m_s2 * interval
            )
            self.assertAlmostEqual(
                actual_end_velocity, expected_end.velocity_m_s, places=12
            )

        completed = profile.sample_zoh(profile.duration_s + 1.0, 0.01)
        self.assertTrue(completed.complete)
        self.assertEqual(completed.acceleration_m_s2, 0.0)
        with self.assertRaises(ValueError):
            profile.sample_zoh(0.0, 0.0)

    def test_position_derivatives_match_reported_velocity_and_acceleration(self):
        profile = make_septic_brake_profile(0.45, 0.20, 1.0, 4.0)
        boundaries = (0.0, *np.cumsum(profile.phase_durations_s))
        step = 1e-5
        for elapsed in np.linspace(step, profile.duration_s - step, 31):
            if any(abs(elapsed - boundary) < 2.0 * step
                   for boundary in boundaries):
                continue
            before = profile.sample(elapsed - step)
            current = profile.sample(elapsed)
            after = profile.sample(elapsed + step)
            numerical_velocity = (
                after.position_m - before.position_m
            ) / (2.0 * step)
            numerical_acceleration = (
                after.velocity_m_s - before.velocity_m_s
            ) / (2.0 * step)
            self.assertAlmostEqual(
                numerical_velocity, current.velocity_m_s, places=7
            )
            self.assertAlmostEqual(
                numerical_acceleration,
                current.acceleration_m_s2,
                places=6,
            )

    def test_infeasible_or_out_of_limit_initial_state_is_rejected(self):
        with self.assertRaisesRegex(
                JerkLimitedBrakeError, 'smoothly unwound before reversal'):
            make_septic_brake_profile(0.005, -0.50, 1.0, 4.0)
        with self.assertRaisesRegex(
                JerkLimitedBrakeError, 'initial braking acceleration'):
            make_septic_brake_profile(0.30, -1.01, 1.0, 4.0)
        with self.assertRaises(JerkLimitedBrakeError):
            make_septic_brake_profile(-0.01, 0.0, 1.0, 4.0)
        with self.assertRaises(ValueError):
            make_septic_brake_profile(0.30, 0.0, 1.0, 0.0)

    def test_inherited_positive_acceleration_above_braking_cap_is_supported(self):
        initial_acceleration = 1.25
        profile = make_septic_brake_profile(
            0.30, initial_acceleration, 1.0, 4.0
        )
        samples = [
            profile.sample(value)
            for value in np.linspace(0.0, profile.duration_s, 4001)
        ]

        self.assertGreater(profile.duration_s, 0.0)
        self.assertLessEqual(
            max(sample.acceleration_m_s2 for sample in samples),
            initial_acceleration + 1e-12,
        )
        self.assertGreaterEqual(
            min(sample.acceleration_m_s2 for sample in samples),
            -1.0 - 1e-12,
        )
        self.assertLessEqual(
            max(abs(sample.jerk_m_s3) for sample in samples),
            4.0 + 1e-9,
        )
        self.assertGreaterEqual(
            min(sample.velocity_m_s for sample in samples), -1e-12
        )

    def test_exact_initial_brake_unwind_boundary_remains_feasible(self):
        acceleration = -0.50
        max_jerk = 4.0
        minimum_velocity = (
            (15.0 / 8.0) * acceleration ** 2 / (2.0 * max_jerk)
        )
        profile = make_septic_brake_profile(
            minimum_velocity, acceleration, 1.0, max_jerk
        )

        self.assertEqual(profile.phase_durations_s[0], 0.0)
        samples = [
            profile.sample(value)
            for value in np.linspace(0.0, profile.duration_s, 1001)
        ]
        self.assertGreaterEqual(
            min(sample.velocity_m_s for sample in samples), -1e-12
        )
        self.assertAlmostEqual(samples[-1].velocity_m_s, 0.0)

    def test_stationary_profile_is_exact(self):
        profile = make_septic_brake_profile(
            0.0, 0.0, 1.0, 4.0, initial_position_m=1.25
        )

        self.assertEqual(profile.profile_type, 'stationary')
        self.assertEqual(profile.duration_s, 0.0)
        self.assertEqual(profile.stop_position_m, 1.25)
        sample = profile.sample(123.0)
        self.assertEqual(sample.position_m, 1.25)
        self.assertEqual(sample.velocity_m_s, 0.0)
        self.assertEqual(sample.acceleration_m_s2, 0.0)
        self.assertEqual(sample.jerk_m_s3, 0.0)
        self.assertTrue(sample.complete)


if __name__ == '__main__':
    unittest.main()
