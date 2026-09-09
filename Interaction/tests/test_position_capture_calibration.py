import copy
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from Interaction.position_capture_calibration import (
    PositionCaptureCalibration,
    position_capture_fit_is_current,
)


class PositionCaptureCalibrationTests(unittest.TestCase):
    def make_plan(self, **updates):
        config = {
            "enabled": True, "start_delay_s": 0,
            "directions_xy": [[0, 1], [0, -1]],
            "accelerate_durations_s": [0.10], "target_distances_m": [0.08],
            "capture_s": 1.0, "recovery_s": 0.20,
        }
        config.update(updates)
        return PositionCaptureCalibration(config)

    def successful_samples(self, plan):
        samples = []
        for trial in plan.trials:
            direction = np.asarray(trial["direction_xy"])
            plan.command(trial["start_s"] + plan.settle_s + 0.001, 0,
                         [0, 0, 1], [0, 0, 0], [0, 0, 0])
            velocity = np.r_[0.20 * direction, 0.0]
            command = plan.command(trial["capture_start_s"] + 0.001, 0,
                                   [0, 0, 1], velocity, [0, 0, 0])
            epoch = 1000 + 10 * trial["segment_id"]
            # Smooth motion reaches the fixed target by 0.5 s, then stays there.
            for elapsed in np.arange(0.02, plan.capture_s, 0.02):
                fraction = min(elapsed / 0.5, 1.0)
                position = np.array([0., 0., 1.])
                position[:2] += direction * trial["target_distance_m"] * fraction
                current_velocity = np.r_[direction * (0.16 if elapsed < 0.5 else 0), 0.0]
                samples.append({
                    "timestamp": epoch + elapsed, "command_started_at": epoch,
                    "segment_id": trial["segment_id"], "phase": "capture",
                    "position": position.tolist(), "velocity": current_velocity.tolist(),
                    "orientation_rpy": [0, 0, 0],
                    "position_target": command.position_target.tolist(),
                })
        return samples

    def test_default_trials_cover_four_signed_directions_and_three_levels(self):
        plan = PositionCaptureCalibration({"enabled": True})
        self.assertEqual(len(plan.trials), 12)
        self.assertAlmostEqual(plan.duration_s, 81.4)
        self.assertEqual({tuple(t["direction_xy"]) for t in plan.trials},
                         {(1, 0), (-1, 0), (0, 1), (0, -1)})

    def test_speed_no_longer_aborts_capture_or_recovery(self):
        plan = self.make_plan()
        recovery = plan.command(plan.trials[0]["capture_end_s"] + .01,
                                0, [0, .3, 1], [0, -.717, 0], [0, 0, 0])
        self.assertEqual(recovery.phase, "recovery")
        self.assertFalse(recovery.attitude_control)
        for elapsed in [.01, .45, .61]:
            plan = self.make_plan()
            if elapsed > .31:
                plan.command(.31, 0, [0, 0, 1], [0, 0, 0], [0, 0, 0])
            plan.command(elapsed, 0, [0, 0, 1], [0, .717, 0], [0, 0, 0])
        with self.assertRaisesRegex(ValueError, 'settled XY speed'):
            self.make_plan().command(.31, 0, [0, 0, 1], [0, .717, 0], [0, 0, 0])

    def test_capture_speed_quality_gate_remains_active(self):
        plan = self.make_plan()
        samples = self.successful_samples(plan)
        samples[10]["velocity"] = [0, .717, 0]
        report = plan.summarize(samples)
        self.assertFalse(report["usable"])
        self.assertTrue(any("speed exceeded" in reason for reason in report["quality_failures"]))

    def test_disabled_and_pure_schedule_do_not_capture_or_change_state(self):
        disabled = PositionCaptureCalibration({})
        self.assertEqual(disabled.duration_s, 0)
        self.assertFalse(disabled.attitude_phase_due(100))
        plan = self.make_plan()
        before = plan.protocol()
        self.assertTrue(plan.attitude_phase_due(0.31))
        self.assertTrue(plan.attitude_phase_due(0.50))
        self.assertFalse(plan.attitude_phase_due(0.80))
        self.assertEqual(plan.protocol(), before)

    def test_fixed_xyz_target_never_chases_measured_position(self):
        plan = self.make_plan()
        command = plan.command(0.31, 0, [0, 0, 1], [0, 0, 0], [0, 0, 0])
        self.assertTrue(command.attitude_control)
        self.assertAlmostEqual(command.roll_deg, -8)
        first = plan.command(0.61, 0, [0.2, 0.3, 1.2], [0, 0.2, 0], [0, 0, 0])
        second = plan.command(0.71, 0, [0.5, 0.7, 1.8], [0, 0.1, 0], [0, 0, 0])
        np.testing.assert_allclose(first.position_target, [0.2, 0.38, 1.2])
        np.testing.assert_array_equal(second.position_target, first.position_target)
        self.assertFalse(first.attitude_control)
        with self.assertRaises(ValueError):
            first.position_target[0] = 9

    def test_unsettled_or_skipped_start_and_wrong_direction_are_rejected(self):
        for velocity, attitude in [([0, .1, 0], [0, 0, 0]), ([0, 0, 0], [.1, 0, 0])]:
            with self.subTest(velocity=velocity, attitude=attitude), self.assertRaises(ValueError):
                self.make_plan().command(.31, 0, [0, 0, 1], velocity, attitude)
        with self.assertRaisesRegex(ValueError, "skipped"):
            self.make_plan().command(.61, 0, [0, 0, 1], [0, .2, 0], [0, 0, 0])
        plan = self.make_plan()
        plan.command(.31, 0, [0, 0, 1], [0, 0, 0], [0, 0, 0])
        with self.assertRaisesRegex(ValueError, "positive directional"):
            plan.command(.61, 0, [0, 0, 1], [0, -.2, 0], [0, 0, 0])

    def test_rejects_unsafe_and_malformed_configuration_and_states(self):
        for config in [
            {"tilt_deg": 35}, {"capture_s": .1}, {"max_sample_gap_s": float("nan")},
            {"accelerate_durations_s": [.1, .2]}, {"target_distances_m": [2.]},
            {"directions_xy": [[0, 1]]}, {"directions_xy": [[0, 0], [0, -1]]},
            {"repetitions": 1.5}, {"max_xy_speed_m_s": .02},
        ]:
            with self.subTest(config=config), self.assertRaises((ValueError, TypeError)):
                self.make_plan(**config)
        plan = self.make_plan()
        with self.assertRaises(ValueError):
            plan.command(.31, 0, [0, float("nan"), 1], [0, 0, 0], [0, 0, 0])
        with self.assertRaisesRegex(ValueError, "settled XY speed"):
            plan.command(.31, 0, [0, 0, 1], [0, .8, 0], [0, 0, 0])

    def test_complete_good_trials_are_empirical_not_continuous_certificate(self):
        plan = self.make_plan()
        samples = self.successful_samples(plan)
        fit = plan.summarize(samples)
        self.assertTrue(fit["usable"], fit["quality_failures"])
        self.assertTrue(position_capture_fit_is_current(fit))
        self.assertFalse(fit["continuous_capture_envelope_certified"])
        self.assertEqual(fit["kind"], "empirical_position_capture_trials")
        self.assertAlmostEqual(fit["trials"][0]["entry_speed_m_s"], .2)
        self.assertAlmostEqual(fit["trials"][0]["target_distance_m"], .08)

    def test_overshoot_then_return_is_not_hidden_by_final_settling(self):
        plan = self.make_plan()
        samples = self.successful_samples(plan)
        samples[15]["position"][1] = .12
        samples[16]["velocity"][1] = -.08
        fit = plan.summarize(samples)
        self.assertFalse(fit["usable"])
        self.assertAlmostEqual(fit["trials"][0]["peak_overshoot_m"], .04)
        self.assertAlmostEqual(fit["trials"][0]["peak_reverse_speed_m_s"], .08)

    def test_reverse_speed_fails_even_without_position_overshoot(self):
        plan = self.make_plan()
        samples = self.successful_samples(plan)
        samples[16]["velocity"][1] = -.08
        fit = plan.summarize(samples)
        self.assertFalse(fit["usable"])
        self.assertEqual(fit["trials"][0]["peak_overshoot_m"], 0.0)
        self.assertAlmostEqual(fit["trials"][0]["peak_reverse_speed_m_s"], .08)
        self.assertTrue(any("reverse speed" in reason for reason in fit["quality_failures"]))
        fit["usable"] = True
        fit["quality_failures"] = []
        fit["trials"][0]["passed"] = True
        fit["trials"][0]["quality_failures"] = []
        self.assertFalse(position_capture_fit_is_current(fit))

    def test_unsettled_or_transient_settle_is_rejected(self):
        for mode in ["moving", "far", "transient"]:
            plan = self.make_plan()
            samples = self.successful_samples(plan)
            for row in samples:
                if mode == "moving" or (mode == "transient" and row["timestamp"] % 10 > .85):
                    row["velocity"][1] = .10
                if mode == "far":
                    row["position"][0] = .20
            with self.subTest(mode=mode):
                self.assertFalse(plan.summarize(samples)["usable"])

    def test_missing_late_truncated_stale_and_bad_timestamp_samples_fail(self):
        for mode in ["missing", "truncated", "late", "stale", "duplicate", "reverse_time", "moving_epoch"]:
            plan = self.make_plan()
            samples = self.successful_samples(plan)
            if mode == "missing":
                samples = [row for row in samples if row["segment_id"] == 0]
            elif mode == "truncated":
                samples = [row for row in samples if row["timestamp"] - row["command_started_at"] < .75]
            elif mode == "late":
                samples = [row for row in samples if row["timestamp"] - row["command_started_at"] > .2]
            elif mode == "stale":
                del samples[10:20]
            elif mode == "duplicate":
                samples[10]["timestamp"] = samples[9]["timestamp"]
            elif mode == "reverse_time":
                samples[10], samples[11] = samples[11], samples[10]
            else:
                samples[10]["command_started_at"] += .02
            with self.subTest(mode=mode):
                self.assertFalse(plan.summarize(samples)["usable"])

    def test_bad_samples_and_target_drift_are_not_accepted(self):
        for field, value in [("position", [0, float("nan"), 1]), ("velocity", [1, 2]),
                             ("timestamp", float("inf")), ("position_target", [0, .081, 1]),
                             ("orientation_rpy", [None, 0, 0])]:
            plan = self.make_plan()
            samples = self.successful_samples(plan)
            samples[10][field] = value
            with self.subTest(field=field):
                self.assertFalse(plan.summarize(samples)["usable"])

    def test_saved_fit_validation_rejects_partial_or_forged_pass(self):
        plan = self.make_plan()
        fit = plan.summarize(self.successful_samples(plan))
        changes = [
            lambda f: f.update(continuous_capture_envelope_certified=True),
            lambda f: f["trials"].pop(),
            lambda f: f["trials"][0].update(peak_overshoot_m=.20),
            lambda f: f["trials"][0].update(observation_duration_s=.20),
            lambda f: f["trials"][0].update(entry_speed_m_s=float("nan")),
            lambda f: f["trials"][0].update(sample_count=1),
            lambda f: f["trials"][0].update(time_to_settle_s=None),
        ]
        for change in changes:
            forged = copy.deepcopy(fit)
            change(forged)
            self.assertFalse(position_capture_fit_is_current(forged))

    def test_real_persistence_roundtrip_retains_fit_and_rejects_bad_replacement(self):
        from Interaction.wrench_model_calibration import save_drone_calibration

        plan = self.make_plan()
        capture = plan.summarize(self.successful_samples(plan))
        capture["control_context"] = {"pid_parameters": {"posCtlPid": {"xKp": "1.9"}}}
        impulse = {
            "model_delay_s": [0., 0., 0.],
            "model_time_constant_s": [0., 0., 0.],
            "model_acceleration_scale": [1., 1., 1.],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "wrench_calibration.json"
            save_drone_calibration("lb11", impulse, {}, path, position_capture_fit=capture)
            saved = json.loads(path.read_text())["drones"]["lb11"]["position_capture_fit"]
            self.assertEqual(saved, capture)
            self.assertTrue(position_capture_fit_is_current(saved))
            save_drone_calibration("lb11", impulse, {}, path)
            self.assertEqual(json.loads(path.read_text())["drones"]["lb11"]["position_capture_fit"], capture)
            before = path.read_bytes()
            bad = copy.deepcopy(capture)
            bad["trials"][0]["peak_reverse_speed_m_s"] = .20
            with self.assertRaisesRegex(ValueError, "position capture"):
                save_drone_calibration("lb11", impulse, {}, path, position_capture_fit=bad)
            self.assertEqual(path.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
