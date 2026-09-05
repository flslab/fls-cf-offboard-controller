import unittest

import numpy as np

from Interaction.braking_response_calibration import PlanarBrakingCalibration


class PlanarBrakingCalibrationTests(unittest.TestCase):
    def test_fixed_twenty_degree_duration_sweep_has_paired_pulses(self):
        plan = PlanarBrakingCalibration({
            "enabled": True, "tilt_levels_deg": [20.0],
            "accelerate_durations_s": [0.16, 0.24, 0.32],
            "repetitions_per_duration": 1,
        })
        np.testing.assert_allclose(plan.trial_accelerate_s, [.16, .16, .24, .24, .32, .32])
        np.testing.assert_allclose(plan.trial_brake_s, plan.trial_accelerate_s)
        self.assertEqual(plan.repetitions, 3)
        for i, start in enumerate(plan.trial_start_s):
            accel_time = start + plan.level_before_acceleration_s
            accel = plan.command(accel_time + .001, yaw_deg=0)
            brake = plan.command(accel_time + plan.trial_accelerate_s[i]
                                 + plan.level_before_brake_s + .001, yaw_deg=0)
            self.assertEqual(accel.segment_id, i)
            self.assertEqual(brake.segment_id, i)
            self.assertEqual(accel.phase, 'accelerate')
            self.assertEqual(brake.phase, 'brake')
            self.assertAlmostEqual(abs(accel.roll_deg), 20)
            self.assertAlmostEqual(brake.roll_deg, -accel.roll_deg)
        self.assertEqual(plan.command(plan.end_s, yaw_deg=0).phase, 'complete')
        self.assertEqual(plan.timing_protocol()['accelerate_durations_s'], [.16, .24, .32])

    def test_duration_sweep_rejects_invalid_durations(self):
        for durations in ([], [0], [-1], [float('nan')], [.2, .1]):
            with self.subTest(durations=durations), self.assertRaises(ValueError):
                PlanarBrakingCalibration({'tilt_levels_deg': [20],
                                         'accelerate_durations_s': durations})

    def test_duration_sweep_allows_repeated_held_out_pair(self):
        plan = PlanarBrakingCalibration({
            'enabled': True,
            'tilt_levels_deg': [20],
            'accelerate_durations_s': [.16, .24, .32, .32],
        })
        self.assertEqual(len(plan.trial_directions), 8)
        np.testing.assert_allclose(
            plan.trial_accelerate_s,
            [.16, .16, .24, .24, .32, .32, .32, .32],
        )
        self.assertEqual(
            plan.timing_protocol()['accelerate_durations_s'],
            [.16, .24, .32, .32],
        )

    def test_duration_sweep_accepts_independent_pairwise_brake_schedule(self):
        plan = PlanarBrakingCalibration({
            "enabled": True,
            "tilt_levels_deg": [20],
            "accelerate_durations_s": [.32, .32, .32],
            "brake_durations_s": [.16, .20, .24],
            "repetitions_per_duration": 2,
        })
        np.testing.assert_allclose(plan.trial_accelerate_s, [.32] * 12)
        np.testing.assert_allclose(
            plan.trial_brake_s,
            [.16, .16, .20, .20, .24, .24] * 2,
        )
        self.assertEqual(plan.repetitions, 6)
        self.assertAlmostEqual(plan.accelerate_s, .32)
        self.assertAlmostEqual(plan.brake_s, .24)
        protocol = plan.timing_protocol()
        self.assertEqual(protocol["accelerate_durations_s"], [.32, .32, .32])
        self.assertEqual(protocol["brake_durations_s"], [.16, .20, .24])

        first_start = plan.trial_start_s[0]
        brake_start = (
            first_start + plan.level_before_acceleration_s
            + plan.trial_accelerate_s[0] + plan.level_before_brake_s
        )
        self.assertEqual(plan.command(brake_start + .159, 0).phase, "brake")
        self.assertEqual(plan.command(brake_start + .161, 0).phase,
                         "level_after_brake")

    def test_independent_brake_schedule_rejects_invalid_configuration(self):
        base = {
            "tilt_levels_deg": [20],
            "accelerate_durations_s": [.32, .32, .32],
        }
        invalid = ([], [.16], [.16, .20], [.16, 0, .24],
                   [.16, float("nan"), .24], [.24, .20, .16])
        for durations in invalid:
            with self.subTest(durations=durations), self.assertRaisesRegex(
                ValueError, "braking durations"
            ):
                PlanarBrakingCalibration(
                    base | {"brake_durations_s": durations}
                )
        with self.assertRaisesRegex(ValueError, "requires accelerate"):
            PlanarBrakingCalibration({
                "tilt_levels_deg": [20],
                "brake_durations_s": [.16, .20, .24],
            })

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

    def test_multi_level_schedule_runs_both_signs_at_each_tilt(self):
        plan = PlanarBrakingCalibration({
            "enabled": True,
            "start_delay_s": 0.0,
            "directions_xy": [[0.0, 1.0], [0.0, -1.0]],
            # Legacy defaults may remain after recursive config merging; the
            # explicit multi-level controls must take precedence.
            "repetitions": 3,
            "tilt_deg": 8.0,
            "tilt_levels_deg": [8.0, 14.0, 20.0],
            "repetitions_per_tilt": 1,
            "level_before_acceleration_s": 0.2,
            "accelerate_s": 0.32,
            "level_before_brake_s": 0.2,
            "brake_s": 0.32,
            "level_after_brake_s": 0.65,
            "recovery_s": 2.0,
        })

        self.assertEqual(plan.repetitions, 3)
        self.assertEqual(len(plan.trial_directions), 6)
        np.testing.assert_allclose(
            plan.trial_tilt_levels_deg,
            [8.0, 8.0, 14.0, 14.0, 20.0, 20.0],
        )
        observed = []
        for segment_id in range(6):
            local_acceleration_time = (
                plan.maneuver_start_s
                + segment_id * plan.trial_s
                + plan.level_before_acceleration_s
                + 0.01
            )
            command = plan.command(local_acceleration_time, yaw_deg=0.0)
            observed.append((
                command.segment_id,
                command.tilt_deg,
                tuple(command.direction_xy),
                float(np.hypot(command.roll_deg, command.pitch_deg)),
            ))
        self.assertEqual(
            [(entry[1], entry[2]) for entry in observed],
            [
                (8.0, (0.0, 1.0)), (8.0, (0.0, -1.0)),
                (14.0, (0.0, 1.0)), (14.0, (0.0, -1.0)),
                (20.0, (0.0, 1.0)), (20.0, (0.0, -1.0)),
            ],
        )
        np.testing.assert_allclose(
            [entry[3] for entry in observed],
            [8.0, 8.0, 14.0, 14.0, 20.0, 20.0],
        )

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
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            PlanarBrakingCalibration({
                "enabled": True,
                "tilt_levels_deg": [14.0, 8.0, 20.0],
            })


if __name__ == "__main__":
    unittest.main()
