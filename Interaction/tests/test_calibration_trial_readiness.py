import unittest

from Interaction.calibration_trial_readiness import CalibrationTrialReadinessGate


class CalibrationTrialReadinessTests(unittest.TestCase):
    def setUp(self):
        self.gate = CalibrationTrialReadinessGate({})
        self.gate.begin(0, 10.0)

    def sample(self, now, speed=0.01, tilt=1.0, error=0.02, sample_time=None):
        return self.gate.update(
            0, now, now if sample_time is None else sample_time, speed, tilt, error
        )

    def test_requires_full_fresh_dwell_then_admits_once(self):
        for now in [10.0, 10.1, 10.2]:
            self.assertFalse(self.sample(now))
        self.assertTrue(self.sample(10.3))
        self.assertFalse(self.gate.waiting)
        self.assertTrue(self.gate.admitted(0))
        self.assertAlmostEqual(self.gate.total_wait_s(11.0), 0.3)
        self.gate.begin(0, 11.0)
        self.assertFalse(self.gate.waiting)

    def test_reported_point_059_waits_not_aborts(self):
        self.assertFalse(self.sample(10.0, speed=0.059, tilt=1.29, error=0.037))
        self.assertTrue(self.gate.waiting)
        for now in [10.1, 10.2, 10.3]:
            self.assertFalse(self.sample(now, speed=0.05))
        self.assertTrue(self.sample(10.4, speed=0.05))

    def test_each_failed_limit_restarts_dwell(self):
        for field, value in [('speed', 0.051), ('tilt', 4.01), ('error', 0.081)]:
            with self.subTest(field=field):
                self.setUp()
                self.sample(10.0)
                self.sample(10.1)
                self.assertFalse(self.sample(10.2, **{field: value}))
                for now in [10.3, 10.4, 10.5]:
                    self.assertFalse(self.sample(now))
                self.assertTrue(self.sample(10.6))

    def test_duplicates_and_dropout_cannot_count_as_dwell(self):
        self.sample(10.0)
        self.sample(10.1)
        self.assertFalse(self.sample(10.2, sample_time=10.1))
        self.assertAlmostEqual(self.gate.stable_elapsed_s, 0.1)
        self.assertFalse(self.sample(10.3))
        self.gate.invalidate(10.35)
        self.assertFalse(self.sample(10.4))
        self.assertAlmostEqual(self.gate.stable_elapsed_s, 0.0)
        self.assertFalse(self.sample(10.6))  # fresh, but gap too large
        self.assertAlmostEqual(self.gate.stable_elapsed_s, 0.0)

    def test_regular_duplicates_between_fresh_samples_do_not_starve_dwell(self):
        self.assertFalse(self.sample(10.0))
        for tick in range(1, 30):
            now = 10.0 + tick * 0.01
            if tick % 2:
                self.gate.no_new_sample(now)
            else:
                self.assertFalse(self.sample(now))
        self.assertTrue(self.sample(10.3))

    def test_duplicate_poll_exceeding_maximum_gap_resets_evidence(self):
        self.sample(10.0)
        self.sample(10.1)
        self.gate.no_new_sample(10.201)
        self.assertEqual(self.gate.stable_elapsed_s, 0.0)

    def test_wall_timeout_is_checked_without_fresh_samples(self):
        self.assertFalse(self.sample(10.0, speed=0.059))
        self.gate.poll(14.99)
        with self.assertRaisesRegex(TimeoutError, 'extra position hold'):
            self.gate.poll(15.0)

    def test_intentional_wait_accounting_is_cumulative_and_stops_at_admission(self):
        for now in [10.0, 10.1, 10.2, 10.3]:
            self.sample(now)
        self.gate.begin(1, 12.0)
        self.assertAlmostEqual(self.gate.total_wait_s(12.5), 0.8)
        self.gate.update(1, 12.5, 12.5, 0.01, 0, 0)
        for now in [12.6, 12.7, 12.8]:
            self.gate.update(1, now, now, 0.01, 0, 0)
        self.assertAlmostEqual(self.gate.total_wait_s(14.0), 1.1)

    def test_large_sensor_time_jump_does_not_fake_dwell(self):
        self.sample(10.0)
        self.assertFalse(self.sample(10.01, sample_time=10.3))
        self.assertAlmostEqual(self.gate.stable_elapsed_s, 0)

    def test_pending_trial_cannot_be_overwritten_or_clock_reversed(self):
        with self.assertRaises(ValueError):
            self.gate.begin(1, 10.1)
        with self.assertRaises(ValueError):
            self.gate.poll(9.9)

    def test_invalid_config_rejected(self):
        for config in [
            {'trial_start_dwell_s': 0}, {'trial_start_timeout_s': float('nan')},
            {'trial_start_dwell_s': 5.0}, {'trial_start_max_sample_gap_s': -1},
        ]:
            with self.subTest(config=config), self.assertRaises(ValueError):
                CalibrationTrialReadinessGate(config)


if __name__ == '__main__':
    unittest.main()
