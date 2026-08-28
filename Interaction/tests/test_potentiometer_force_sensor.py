import unittest
from types import SimpleNamespace
import math

import numpy as np

from Interaction.interactions import InteractionsControl
from Interaction.potentiometer_force_sensor import parse_potentiometer_line


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

    def test_fresh_sensor_force_is_selected_only_for_release_braking(self):
        estimate = SimpleNamespace(external_force=[0.3, 0.4, 0.0])
        sensor_fields = {
            'force_sensor_fresh': True,
            'force_sensor_external_force_N': [0.0, 0.8, 0.0],
        }

        force, source = InteractionsControl._release_braking_force(
            estimate, sensor_fields, sensor_enabled=True
        )

        np.testing.assert_allclose(force, [0.0, 0.8, 0.0])
        self.assertEqual(source, 'potentiometer_force_sensor')

    def test_stale_sensor_falls_back_to_observer_for_release_braking(self):
        estimate = SimpleNamespace(external_force=[0.3, 0.4, 0.0])
        sensor_fields = {
            'force_sensor_fresh': False,
            'force_sensor_external_force_N': [0.0, 0.8, 0.0],
        }

        force, source = InteractionsControl._release_braking_force(
            estimate, sensor_fields, sensor_enabled=True
        )

        np.testing.assert_allclose(force, estimate.external_force)
        self.assertEqual(source, 'wrench_observer')

if __name__ == "__main__":
    unittest.main()
