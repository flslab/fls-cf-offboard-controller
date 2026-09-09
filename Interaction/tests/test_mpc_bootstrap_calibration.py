from copy import deepcopy
import unittest
from unittest.mock import patch

import numpy as np

from Interaction.mpc_bootstrap_calibration import (
    MPCBootstrapCalibrationConfig,
    MPCBootstrapCoverage,
    configure_mpc_bootstrap_mission,
    mpc_bootstrap_world_y_direction,
    prepare_mpc_bootstrap_mission,
)


class MPCBootstrapCalibrationTests(unittest.TestCase):
    def test_private_overlay_disables_every_nonbaseline_owner(self):
        mission = {
            "Interaction": {"action": "translation", "config": {
                "virtual_object": {
                    "inertia_command": "position",
                    "two_afc_friction": {"enabled": True},
                },
                "wrench_interaction": {
                    "shadow_mode": True,
                    "calibration_excitation": {"enabled": True},
                    "planar_braking_calibration": {"enabled": True},
                    "adaptive_braking_calibration": {"enabled": True},
                    "online_prediction_calibration": {"enabled": True},
                    "predictive_braking": {"enabled": True},
                    "learning_velocity_mpc_shadow": {
                        "enabled": True, "command_authority": True,
                        "direction_xy": [0.0, 1.0],
                    },
                    "control_handoff": {
                        "coast_velocity_braking_enabled": True,
                        "coast_velocity_predictive_unwind_enabled": True,
                        "coast_direct_position_handoff": True,
                    },
                },
            }},
        }
        before = deepcopy(mission)
        result = configure_mpc_bootstrap_mission(mission)
        self.assertEqual(mission, before)
        self.assertIsNot(result, mission)
        virtual = result["Interaction"]["config"]["virtual_object"]
        wrench = result["Interaction"]["config"]["wrench_interaction"]
        self.assertEqual(virtual["inertia_command"], "orientation")
        self.assertFalse(virtual["two_afc_friction"]["enabled"])
        self.assertEqual(
            virtual["release_behavior"]["mode"], "potentiometer_coast"
        )
        self.assertFalse(wrench["shadow_mode"])
        for name in (
            "calibration_excitation", "planar_braking_calibration",
            "adaptive_braking_calibration", "online_prediction_calibration",
            "predictive_braking",
        ):
            self.assertFalse(wrench[name]["enabled"])
        self.assertFalse(wrench["learning_velocity_mpc_shadow"]["enabled"])
        self.assertFalse(
            wrench["learning_velocity_mpc_shadow"]["command_authority"]
        )
        self.assertIsNone(
            wrench["learning_velocity_mpc_shadow"]["direction_xy"]
        )
        self.assertFalse(
            wrench["control_handoff"]["coast_velocity_braking_enabled"]
        )
        self.assertFalse(
            wrench["control_handoff"]["coast_direct_position_handoff"]
        )

    def test_config_rejects_overlapping_speed_windows(self):
        with self.assertRaisesRegex(ValueError, "overlap"):
            MPCBootstrapCalibrationConfig.from_mapping({
                "initial_speed_targets_m_s": [0.2, 0.3],
                "speed_tolerance_m_s": 0.06,
            })

    def test_all_direction_speed_cells_complete_independently(self):
        config = MPCBootstrapCalibrationConfig.from_mapping({
            "initial_speed_targets_m_s": [0.2, 0.4],
            "speed_tolerance_m_s": 0.04,
            "repetitions_per_cell": 1,
            "max_release_speed_m_s": 0.5,
        })
        coverage = MPCBootstrapCoverage(config)
        first = coverage.begin("a", [0, 1], [0.01, 0.21])
        self.assertTrue(first.countable)
        coverage.close("a", terminal_success=True, reason="terminal")
        self.assertEqual(coverage.current_target_speed_m_s, 0.2)
        second = coverage.begin("b", [0, -1], [-0.01, -0.19])
        self.assertTrue(second.countable)
        coverage.close("b", terminal_success=True, reason="terminal")
        self.assertEqual(coverage.current_target_speed_m_s, 0.4)
        for episode, direction, velocity in (
            ("c", [0, 1], [0, .4]),
            ("d", [0, -1], [0, -.4]),
        ):
            coverage.begin(episode, direction, velocity)
            coverage.close(episode, terminal_success=True, reason="terminal")
        self.assertTrue(coverage.complete)
        self.assertTrue(coverage.summary()["complete"])

    def test_higher_speed_cell_counts_while_failed_cell_does_not(self):
        config = MPCBootstrapCalibrationConfig.from_mapping({
            "initial_speed_targets_m_s": [0.2, 0.4],
            "speed_tolerance_m_s": 0.04,
            "repetitions_per_cell": 1,
            "max_release_speed_m_s": 0.5,
        })
        coverage = MPCBootstrapCoverage(config)
        fast = coverage.begin("fast", [0, 1], [0, .4])
        self.assertTrue(fast.countable)
        coverage.close("fast", terminal_success=True, reason="terminal")
        failed = coverage.begin("fail", [0, 1], [0, .2])
        self.assertTrue(failed.countable)
        coverage.close("fail", terminal_success=False, reason="overshoot")
        self.assertEqual(coverage.current_target_speed_m_s, 0.2)
        cells = coverage.summary()["cells"]
        counts = {
            (cell["direction_sign"], cell["target_speed_m_s"]):
                cell["success_count"]
            for cell in cells
        }
        self.assertEqual(counts[(1, 0.4)], 1)
        self.assertEqual(counts[(1, 0.2)], 0)
        self.assertEqual(counts[(-1, 0.2)], 0)
        self.assertEqual(counts[(-1, 0.4)], 0)

    def test_cross_speed_and_protocol_max_are_not_countable(self):
        config = MPCBootstrapCalibrationConfig.from_mapping({
            "initial_speed_targets_m_s": [0.2],
            "speed_tolerance_m_s": 0.04,
            "max_cross_speed_m_s": 0.05,
            "max_release_speed_m_s": 0.3,
        })
        coverage = MPCBootstrapCoverage(config)
        cross = coverage.begin("cross", [0, 1], [-.06, .2])
        self.assertEqual(cross.reason, "cross_speed_above_protocol_limit")
        coverage.close("cross", terminal_success=True, reason="terminal")
        fast = coverage.begin("too-fast", [0, 1], [0, .31])
        self.assertEqual(fast.reason, "release_speed_above_protocol_limit")

    def test_path_reversal_prevents_provisional_count(self):
        config = MPCBootstrapCalibrationConfig.from_mapping({
            "initial_speed_targets_m_s": [0.2],
            "speed_tolerance_m_s": 0.04,
            "repetitions_per_cell": 1,
            "max_release_speed_m_s": 0.3,
        })
        coverage = MPCBootstrapCoverage(config)
        coverage.begin("reverse", [0, 1], [0, .2])
        failures = coverage.observe(
            "reverse",
            velocity_xy_m_s=[0.0, -0.03],
            attitude_rp_rad=[0.0, 0.0],
            attitude_rate_rp_rad_s=[0.0, 0.0],
            boundary_margin_m=0.5,
            state_age_s=0.01,
            state_group_skew_s=0.01,
        )
        result = coverage.close(
            "reverse", terminal_success=True, reason="terminal"
        )
        self.assertIn("reverse_velocity_path_violation", failures)
        self.assertFalse(result["provisional_path_eligible"])
        self.assertFalse(result["counted"])
        self.assertFalse(coverage.complete)

    def test_prediction_step_is_part_of_logged_protocol(self):
        config = MPCBootstrapCalibrationConfig.from_mapping({
            "prediction_step_s": 0.01,
        })
        self.assertEqual(config.to_dict()["prediction_step_s"], 0.01)
        with self.assertRaisesRegex(ValueError, "prediction_step_s"):
            MPCBootstrapCalibrationConfig.from_mapping({
                "prediction_step_s": 0.03,
            })

    def test_default_protocol_uses_50_hz_and_tight_cross_speed(self):
        protocol = MPCBootstrapCalibrationConfig.from_mapping({}).to_dict()
        self.assertEqual(protocol["prediction_step_s"], 0.02)
        self.assertEqual(protocol["max_cross_speed_m_s"], 0.03)

    def test_world_y_direction_is_locked_only_for_on_axis_release(self):
        np.testing.assert_allclose(
            mpc_bootstrap_world_y_direction([0.02, 0.25], 0.03),
            [0.0, 1.0],
        )
        np.testing.assert_allclose(
            mpc_bootstrap_world_y_direction([-0.02, -0.25], 0.03),
            [0.0, -1.0],
        )
        self.assertIsNone(
            mpc_bootstrap_world_y_direction([0.031, 0.25], 0.03)
        )
        self.assertIsNone(
            mpc_bootstrap_world_y_direction([0.0, 0.049], 0.03)
        )

    def test_external_cadence_failure_is_sticky(self):
        config = MPCBootstrapCalibrationConfig.from_mapping({
            "initial_speed_targets_m_s": [0.2],
            "speed_tolerance_m_s": 0.04,
            "repetitions_per_cell": 1,
            "max_release_speed_m_s": 0.3,
        })
        coverage = MPCBootstrapCoverage(config)
        coverage.begin("late", [0, 1], [0, .2])
        coverage.mark_path_failure(
            "late", "decision_command_cadence_path_violation"
        )
        result = coverage.close(
            "late", terminal_success=True, reason="terminal"
        )
        self.assertFalse(result["counted"])
        self.assertIn(
            "decision_command_cadence_path_violation",
            result["path_failure_reasons"],
        )

    def test_prearm_preparation_applies_and_requires_baseline_fit(self):
        mission = {
            "drones": {"lb11": {"target": [0.0, -1.0, 1.0]}},
            "boundary_limits": {
                "x_min": -1.5, "x_max": 1.5,
                "y_min": -1.5, "y_max": 1.5,
            },
            "Interaction": {"action": "translation", "config": {
                "wrench_interaction": {
                    "calibration_nominal_position": [0.0, 0.0, 1.0],
                },
            }},
        }
        saved = {"planar_braking_fit": {"fit": "current"}}
        resolved = {"resolved": True, "mpc_bootstrap_calibration": {
            "enabled": True,
        }}
        with patch(
            "Interaction.mpc_bootstrap_calibration.apply_drone_calibration",
            return_value=(resolved, saved),
        ) as apply_fit, patch(
            "Interaction.mpc_bootstrap_calibration.planar_braking_fit_is_current",
            return_value=True,
        ):
            prepared = prepare_mpc_bootstrap_mission(
                mission, drone_id="lb11", sense_axis="y"
            )
        self.assertEqual(
            prepared["Interaction"]["config"]["wrench_interaction"],
            resolved,
        )
        apply_fit.assert_called_once()

        with patch(
            "Interaction.mpc_bootstrap_calibration.apply_drone_calibration",
            return_value=({}, None),
        ), self.assertRaisesRegex(ValueError, "run --calibrate first"):
            prepare_mpc_bootstrap_mission(
                mission, drone_id="lb11", sense_axis="y"
            )

    def test_prearm_preparation_rejects_wrong_sensor_axis(self):
        with self.assertRaisesRegex(ValueError, "sense-axis y"):
            prepare_mpc_bootstrap_mission(
                {}, drone_id="lb11", sense_axis="x"
            )


if __name__ == "__main__":
    unittest.main()
