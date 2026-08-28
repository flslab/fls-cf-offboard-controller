import unittest
from types import SimpleNamespace
import math

from Interaction.interactions import InteractionsControl
from Interaction.potentiometer_force_sensor import (
    SpringForceTrendDetector,
    parse_potentiometer_line,
)


class PotentiometerForceSensorParsingTest(unittest.TestCase):
    def test_parses_arduino_csv_and_computes_hooke_force(self):
        sample = parse_potentiometer_line(
            b"1234,925,925.25,4.522,7.100\n",
            spring_constant_n_per_mm=0.16,
            host_time=10.0,
        )

        self.assertIsNotNone(sample)
        self.assertEqual(sample.host_time, 10.0)
        self.assertEqual(sample.arduino_time_ms, 1234)
        self.assertEqual(sample.raw, 925)
        self.assertAlmostEqual(sample.filtered_raw, 925.25)
        self.assertAlmostEqual(sample.distance_mm, 7.1)
        self.assertAlmostEqual(sample.force_n, 1.136)

    def test_ignores_header_and_invalid_rows(self):
        self.assertIsNone(parse_potentiometer_line(
            "time_ms,raw,filtered,voltage,distance_mm"
        ))
        self.assertIsNone(parse_potentiometer_line("1,2,3"))
        self.assertIsNone(parse_potentiometer_line("1,1024,3,4,5"))
        self.assertIsNone(parse_potentiometer_line("1,500,500,2.4,-1"))

    def test_builds_axis_aligned_fresh_comparison_fields(self):
        sample = parse_potentiometer_line(
            "1234,925,925.25,4.522,7.100",
            spring_constant_n_per_mm=0.16,
            host_time=10.0,
        )
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.force_sensor = SimpleNamespace(
            latest=lambda: sample,
            spring_constant_n_per_mm=0.16,
        )
        controller.sense_axis = 'y'
        controller.sense_axis_index = 1
        controller.sense_sign = -1
        controller.sense_max_age_s = 0.25
        estimate = SimpleNamespace(
            external_force=[0.5, -0.2, 0.0],
            orientation_rpy=[0.0, 0.0, 0.0],
        )

        fields = controller._force_sensor_log_fields(estimate, now=10.05)

        self.assertTrue(fields['force_sensor_fresh'])
        self.assertEqual(
            fields['force_sensor_external_force_body_N'],
            [0.0, -1.136, 0.0],
        )
        self.assertEqual(fields['force_sensor_external_force_N'], [0.0, -1.136, 0.0])
        self.assertAlmostEqual(
            fields['estimated_external_force_along_sensor_N'], -0.2
        )
        self.assertAlmostEqual(fields['force_sensor_estimate_error_N'], 0.936)

        stale = controller._force_sensor_log_fields(estimate, now=10.5)
        self.assertFalse(stale['force_sensor_fresh'])
        self.assertIsNone(stale['force_sensor_estimate_error_N'])

    def test_rotates_body_sensor_axis_into_world_frame(self):
        sample = parse_potentiometer_line(
            "1234,925,925.25,4.522,7.100",
            spring_constant_n_per_mm=0.16,
            host_time=10.0,
        )
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.force_sensor = SimpleNamespace(latest=lambda: sample)
        controller.sense_axis = 'x'
        controller.sense_axis_index = 0
        controller.sense_sign = 1
        controller.sense_max_age_s = 0.25
        estimate = SimpleNamespace(
            external_force=[0.0, 1.0, 0.0],
            orientation_rpy=[0.0, 0.0, math.pi / 2.0],
        )

        fields = controller._force_sensor_log_fields(estimate, now=10.05)

        for actual, expected in zip(
                fields['force_sensor_external_force_N'], [0.0, 1.136, 0.0]):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(
            fields['estimated_external_force_along_sensor_N'], 1.0
        )

    def test_force_trend_starts_on_rise_and_releases_on_fall(self):
        detector = SpringForceTrendDetector(
            contact_force_n=0.02,
            rise_rate_n_s=0.05,
            release_rate_n_s=0.05,
            release_drop_n=0.01,
            onset_time_s=0.02,
            release_time_s=0.04,
        )
        direction = [0.0, 1.0, 0.0]

        self.assertFalse(detector.update(0.0, 0.00, direction).active)
        contact = detector.update(0.03, 0.02, direction)
        self.assertTrue(contact.started)
        self.assertTrue(contact.active)
        detector.update(0.08, 0.04, direction)

        release = detector.update(0.06, 0.06, direction)
        self.assertTrue(release.release_candidate_started)
        self.assertTrue(release.active)
        self.assertFalse(detector.update(0.04, 0.08, direction).ended)
        ended = detector.update(0.02, 0.10, direction)
        self.assertTrue(ended.ended)
        self.assertFalse(ended.active)

        # A new rise is ignored until braking completes and the controller
        # explicitly rearms the detector.
        self.assertFalse(detector.update(0.08, 0.12, direction).started)
        detector.reset(0.12)
        self.assertFalse(detector.update(0.02, 0.14, direction).started)
        self.assertTrue(detector.update(0.05, 0.17, direction).started)

    def test_force_trend_cancels_release_when_force_rises_again(self):
        detector = SpringForceTrendDetector(
            onset_time_s=0.02,
            release_time_s=0.08,
        )
        direction = [0.0, 1.0, 0.0]
        detector.update(0.0, 0.00, direction)
        detector.update(0.04, 0.02, direction)
        detector.update(0.08, 0.04, direction)
        candidate = detector.update(0.06, 0.06, direction)
        self.assertTrue(candidate.release_candidate_started)

        cancelled = detector.update(0.075, 0.08, direction)
        self.assertTrue(cancelled.release_candidate_cancelled)
        self.assertTrue(cancelled.active)


if __name__ == "__main__":
    unittest.main()
