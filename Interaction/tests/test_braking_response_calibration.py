import unittest

import numpy as np

from Interaction.braking_response_calibration import PlanarBrakingCalibration


class PlanarBrakingCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.plan = PlanarBrakingCalibration({
            "enabled": True,
            "start_delay_s": 0.5,
            "directions_xy": [[0.0, 1.0], [0.0, -1.0]],
            "repetitions": 1,
            "tilt_deg": 8.0,
            "level_before_acceleration_s": 0.2,
            "accelerate_s": 0.3,
            "level_before_brake_s": 0.2,
            "brake_s": 0.4,
            "level_after_brake_s": 0.3,
            "recovery_s": 0.5,
        }, start_after_s=2.0)

    def test_schedule_contains_accelerate_level_brake_level_and_recovery(self):
        self.assertAlmostEqual(self.plan.maneuver_start_s, 2.5)
        self.assertAlmostEqual(self.plan.end_s, 6.3)
        cases = [
            (2.49, None, "waiting", False),
            (2.55, 0, "level_before_acceleration", True),
            (2.80, 0, "accelerate", True),
            (3.05, 0, "level_before_brake", True),
            (3.30, 0, "brake", True),
            (3.70, 0, "level_after_brake", True),
            (4.10, 0, "recovery", False),
            (4.70, 1, "accelerate", True),
            (6.31, None, "complete", False),
        ]
        for elapsed, segment, phase, attitude_control in cases:
            with self.subTest(elapsed=elapsed):
                command = self.plan.command(elapsed, yaw_deg=0.0)
                self.assertEqual(command.segment_id, segment)
                self.assertEqual(command.phase, phase)
                self.assertEqual(command.attitude_control, attitude_control)

    def test_attitude_is_rotated_by_yaw_and_brake_reverses_it(self):
        accelerate = self.plan.command(2.80, yaw_deg=0.0)
        brake = self.plan.command(3.30, yaw_deg=0.0)
        self.assertAlmostEqual(accelerate.roll_deg, -8.0)
        self.assertAlmostEqual(accelerate.pitch_deg, 0.0)
        self.assertAlmostEqual(brake.roll_deg, 8.0)
        self.assertAlmostEqual(brake.pitch_deg, 0.0)

        yawed = self.plan.command(2.80, yaw_deg=90.0)
        self.assertAlmostEqual(yawed.roll_deg, 0.0, places=10)
        self.assertAlmostEqual(yawed.pitch_deg, -8.0)

    def test_normalizes_direction_and_rejects_unsafe_configuration(self):
        normalized = PlanarBrakingCalibration({
            "enabled": True,
            "directions_xy": [[0.0, 2.0], [0.0, -3.0]],
        })
        np.testing.assert_allclose(normalized.directions[0], [0.0, 1.0])

        with self.assertRaises(ValueError):
            PlanarBrakingCalibration({
                "enabled": True,
                "directions_xy": [[0.0, 0.0]],
            })
        with self.assertRaisesRegex(ValueError, "opposed"):
            PlanarBrakingCalibration({
                "enabled": True,
                "directions_xy": [[0.0, 1.0], [1.0, 0.0]],
            })
        with self.assertRaises(ValueError):
            PlanarBrakingCalibration({
                "enabled": True,
                "directions_xy": [[0.0, 1.0]],
                "tilt_deg": 30.0,
            })


if __name__ == "__main__":
    unittest.main()
