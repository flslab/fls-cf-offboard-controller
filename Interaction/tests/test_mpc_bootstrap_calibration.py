from copy import deepcopy
import math
import unittest
from unittest.mock import patch

import numpy as np

from Interaction.mpc_bootstrap_calibration import (
    MPCBootstrapAutomaticAttempt,
    MPCBootstrapCalibrationConfig,
    MPCBootstrapCoverage,
    MPCBootstrapTargetCell,
    build_mpc_bootstrap_model_contracts,
    configure_mpc_bootstrap_mission,
    mpc_bootstrap_acceleration_attitude_deg,
    mpc_bootstrap_model_contract_for_direction,
    mpc_bootstrap_required_boundary_margin_m,
    mpc_bootstrap_world_y_direction,
    mpc_decision_state_age_is_fresh,
    prepare_mpc_bootstrap_mission,
    validate_mpc_bootstrap_model_contracts,
)
from Interaction.tests.test_predictive_brake_handoff import validated_model


class MPCBootstrapCalibrationTests(unittest.TestCase):
    @staticmethod
    def mission():
        return {
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

    @staticmethod
    def saved(*, positive_delay=0.03, negative_delay=0.05):
        prediction = validated_model()
        prediction["directional_models"]["positive_y"][
            "attitude_fit"
        ]["delay_s"] = positive_delay
        prediction["directional_models"]["negative_y"][
            "attitude_fit"
        ]["delay_s"] = negative_delay
        return {
            "planar_braking_fit": {
                "fit": "current", "command_delay_s": 0.12,
            },
            "prediction_model": prediction,
        }

    @staticmethod
    def automatic_config(**overrides):
        values = {
            "initial_speed_targets_m_s": [0.25],
            "speed_tolerance_m_s": 0.04,
            "repetitions_per_cell": 1,
            "max_release_speed_m_s": 0.40,
            "ready_dwell_s": 0.02,
            "level_warmup_min_s": 0.02,
            "prediction_step_s": 0.02,
            "max_acceleration_duration_s": 0.10,
            "max_maneuver_displacement_m": 0.20,
        }
        values.update(overrides)
        return MPCBootstrapCalibrationConfig.from_mapping(values)

    @classmethod
    def automatic_attempt(cls, *, sign=1, model_delay_s=0.03, **overrides):
        config = cls.automatic_config(**overrides)
        cell = MPCBootstrapTargetCell(
            direction_sign=sign,
            target_speed_m_s=0.25,
            success_count=0,
            required_count=1,
        )
        return MPCBootstrapAutomaticAttempt(
            config,
            cell,
            nominal_position_xy_m=[0.0, 0.0],
            directional_model_delay_s=model_delay_s,
        )

    @staticmethod
    def observe_automatic(
            attempt, time_s, *, position=(0.0, 0.0),
            velocity=(0.0, 0.0), attitude=(0.0, 0.0),
            rates=(0.0, 0.0), boundary=1.0, state_age=0.0,
            state_skew=0.0, z_error=0.0, sample_gap=None,
            rejected=False):
        return attempt.observe(
            time_s=time_s,
            position_xy_m=position,
            velocity_xy_m_s=velocity,
            attitude_rp_rad=attitude,
            attitude_rate_rp_rad_s=rates,
            boundary_margin_m=boundary,
            state_age_s=state_age,
            state_group_skew_s=state_skew,
            z_error_m=z_error,
            sample_gap_s=sample_gap,
            measurement_rejected=rejected,
        )

    @classmethod
    def advance_to_acceleration(cls, attempt):
        first = cls.observe_automatic(attempt, 0.0)
        second = cls.observe_automatic(attempt, 0.02)
        third = cls.observe_automatic(
            attempt, 0.02+attempt.level_warmup_required_s
        )
        assert first.command_kind == "position_hold"
        assert second.command_kind == "level_attitude_zdistance"
        assert third.command_kind == (
            "automatic_acceleration_attitude_zdistance"
        )
        return third.time_s

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
            virtual["release_behavior"]["mode"], "observer_brake"
        )
        self.assertEqual(
            virtual["contact_detection"]["source"], "wrench_observer"
        )
        self.assertFalse(virtual["force_rendering"]["enabled"])
        self.assertEqual(virtual["max_velocity_command_m_s"], 0.75)
        self.assertFalse(wrench["shadow_mode"])
        self.assertFalse(wrench["detection"]["translation"]["enabled"])
        self.assertFalse(wrench["detection"]["yaw"]["enabled"])
        self.assertTrue(wrench["safety"]["enforce_state_group_skew"])
        self.assertEqual(wrench["safety"]["max_state_age_s"], 0.10)
        self.assertEqual(wrench["safety"]["max_state_group_skew_s"], 0.03)
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
        self.assertEqual(
            wrench["control_handoff"][
                "coast_level_handoff_speed_m_s"
            ],
            0.10,
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
            "max_cross_speed_m_s": 0.03,
            "max_release_speed_m_s": 0.3,
        })
        coverage = MPCBootstrapCoverage(config)
        cross = coverage.begin("cross", [0, 1], [-.031, .2])
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
        self.assertEqual(protocol["automatic_acceleration_tilt_deg"], 8.0)
        self.assertEqual(protocol["automatic_release_tolerance_m_s"], 0.02)
        self.assertEqual(
            mpc_bootstrap_required_boundary_margin_m(
                MPCBootstrapCalibrationConfig.from_mapping({})
            ),
            0.70,
        )

    def test_automatic_scheduler_interleaves_minimum_count_cells(self):
        config = MPCBootstrapCalibrationConfig.from_mapping({
            "initial_speed_targets_m_s": [0.25, 0.45, 0.65],
            "repetitions_per_cell": 2,
        })
        coverage = MPCBootstrapCoverage(config)
        previous = None
        observed = []
        for index in range(12):
            cell = coverage.next_required_cell(previous)
            observed.append(cell.key)
            episode = f"auto-{index}"
            velocity = [0.0, cell.direction_sign*cell.target_speed_m_s]
            coverage.begin(episode, cell.direction_xy, velocity)
            coverage.close(
                episode, terminal_success=True, reason="terminal"
            )
            previous = cell
        self.assertEqual(observed, [
            (1, 0.25), (-1, 0.25),
            (1, 0.25), (-1, 0.25),
            (1, 0.45), (-1, 0.45),
            (1, 0.45), (-1, 0.45),
            (1, 0.65), (-1, 0.65),
            (1, 0.65), (-1, 0.65),
        ])
        self.assertTrue(coverage.complete)
        self.assertIsNone(coverage.next_required_cell(previous))

    def test_failed_automatic_cell_is_not_counted_and_retries_later(self):
        config = MPCBootstrapCalibrationConfig.from_mapping({
            "initial_speed_targets_m_s": [0.25, 0.45],
            "repetitions_per_cell": 1,
        })
        coverage = MPCBootstrapCoverage(config)
        failed = coverage.next_required_cell()
        self.assertEqual(failed.key, (1, 0.25))
        next_cell = coverage.next_required_cell(failed)
        self.assertEqual(next_cell.key, (-1, 0.25))
        self.assertEqual(
            coverage.summary()["cells"][1]["success_count"], 0
        )

    def test_attitude_acceleration_signs_are_world_y_correct(self):
        positive = mpc_bootstrap_acceleration_attitude_deg(1, 8.0)
        negative = mpc_bootstrap_acceleration_attitude_deg(-1, 8.0)
        self.assertEqual(positive, (-8.0, 0.0))
        self.assertEqual(negative, (8.0, 0.0))
        rotated = mpc_bootstrap_acceleration_attitude_deg(1, 8.0, 180.0)
        self.assertAlmostEqual(rotated[0], 8.0)
        self.assertAlmostEqual(rotated[1], 0.0, places=12)

    def test_automatic_attempt_waits_warms_accelerates_then_brakes(self):
        attempt = self.automatic_attempt(model_delay_s=0.06)
        self.assertEqual(attempt.level_warmup_required_s, 0.08)
        first = self.observe_automatic(attempt, 0.0)
        self.assertEqual(first.command_kind, "position_hold")
        warmup = self.observe_automatic(attempt, 0.02)
        self.assertEqual(warmup.phase, attempt.LEVEL_WARMUP)
        self.assertEqual(warmup.command_kind, "level_attitude_zdistance")
        not_long_enough = self.observe_automatic(attempt, 0.099)
        self.assertEqual(
            not_long_enough.command_kind, "level_attitude_zdistance"
        )
        accelerating = self.observe_automatic(attempt, 0.10)
        self.assertEqual(accelerating.phase, attempt.ACCELERATING)
        self.assertEqual(
            accelerating.command_kind,
            "automatic_acceleration_attitude_zdistance",
        )
        self.assertEqual(accelerating.command_roll_deg, -8.0)
        self.assertEqual(accelerating.command_pitch_deg, 0.0)
        below_window = self.observe_automatic(
            attempt, 0.11, velocity=(0.0, 0.229)
        )
        self.assertFalse(below_window.start_braking)
        release = self.observe_automatic(
            attempt, 0.12, velocity=(0.0, 0.23)
        )
        self.assertTrue(release.start_braking)
        self.assertEqual(release.phase, attempt.BRAKING)
        self.assertEqual(
            release.command_kind, "begin_legacy_attitude_coast"
        )
        self.assertEqual(release.velocity_xy_m_s, (0.0, 0.23))

    def test_ready_dwell_resets_until_position_velocity_and_level_are_stable(self):
        attempt = self.automatic_attempt()
        self.observe_automatic(attempt, 0.0)
        moving = self.observe_automatic(
            attempt, 0.02, velocity=(0.0, 0.04)
        )
        self.assertEqual(moving.command_kind, "position_hold")
        self.observe_automatic(attempt, 0.03)
        still_waiting = self.observe_automatic(attempt, 0.049)
        self.assertEqual(still_waiting.phase, attempt.READY_DWELL)
        warmup = self.observe_automatic(attempt, 0.05)
        self.assertEqual(warmup.phase, attempt.LEVEL_WARMUP)

    def test_automatic_prelude_failures_are_sticky_and_never_release(self):
        cases = (
            ("reverse", {"velocity": (0.0, -0.021)},
             "reverse_velocity_automatic_prelude_violation"),
            ("overspeed", {"velocity": (0.0, 0.271)},
             "target_window_skipped_automatic_prelude_violation"),
            ("cross", {"velocity": (0.031, 0.10)},
             "cross_velocity_automatic_prelude_violation"),
            ("timeout", {"time_offset": 0.101, "sample_gap": 0.02},
             "acceleration_timeout_automatic_prelude_violation"),
            ("tilt", {"attitude": (math.radians(12.1), 0.0)},
             "attitude_automatic_prelude_violation"),
            ("rate", {"rates": (math.radians(100.1), 0.0)},
             "attitude_rate_automatic_prelude_violation"),
            ("displacement", {"position": (0.201, 0.0)},
             "maneuver_displacement_automatic_prelude_violation"),
            ("state_age", {"state_age": 0.101},
             "state_age_automatic_prelude_violation"),
            ("state_skew", {"state_skew": 0.031},
             "state_group_skew_automatic_prelude_violation"),
            ("rejected", {"rejected": True},
             "measurement_rejected_automatic_prelude_violation"),
            ("boundary", {"boundary": 0.019},
             "boundary_margin_automatic_prelude_violation"),
        )
        for name, changes, expected in cases:
            with self.subTest(name=name):
                changes = dict(changes)
                attempt = self.automatic_attempt()
                acceleration_start = self.advance_to_acceleration(attempt)
                time_s = acceleration_start+changes.pop(
                    "time_offset", 0.01
                )
                failed = self.observe_automatic(
                    attempt, time_s, **changes
                )
                self.assertTrue(failed.abort_requested)
                self.assertFalse(failed.start_braking)
                self.assertIn(expected, failed.prelude_failure_reasons)
                later = self.observe_automatic(
                    attempt,
                    time_s+0.01,
                    velocity=(0.0, 0.25),
                )
                self.assertTrue(later.abort_requested)
                self.assertFalse(later.start_braking)
                self.assertIn(expected, later.prelude_failure_reasons)

    def test_untrusted_prelude_state_requests_level_and_land(self):
        attempt = self.automatic_attempt()
        acceleration_start = self.advance_to_acceleration(attempt)
        failed = self.observe_automatic(
            attempt, acceleration_start+0.01, state_age=0.101
        )
        self.assertEqual(failed.command_kind, "abort_level_and_land")
        self.assertEqual(failed.abort_action, "level_and_land")

    def test_overspeed_requests_bounded_legacy_braking_not_lmpc(self):
        attempt = self.automatic_attempt()
        acceleration_start = self.advance_to_acceleration(attempt)
        failed = self.observe_automatic(
            attempt,
            acceleration_start+0.01,
            velocity=(0.0, 0.28),
        )
        self.assertEqual(
            failed.command_kind, "abort_to_legacy_attitude_coast"
        )
        self.assertEqual(
            failed.abort_action, "legacy_attitude_coast_to_rest"
        )
        self.assertNotIn("velocity_hover", failed.command_kind)
        self.assertNotIn("lmpc", failed.command_kind.lower())

    def test_ready_gate_requires_z_and_has_a_five_second_timeout(self):
        attempt = self.automatic_attempt()
        high = self.observe_automatic(attempt, 0.0, z_error=0.051)
        self.assertEqual(high.phase, attempt.READY_DWELL)
        self.assertEqual(high.command_kind, "position_hold")
        self.observe_automatic(
            attempt, 0.02, velocity=(0.0, 0.04), sample_gap=0.02
        )
        timed_out = self.observe_automatic(
            attempt, 5.001, velocity=(0.0, 0.04), sample_gap=0.02
        )
        self.assertTrue(timed_out.abort_requested)
        self.assertEqual(timed_out.abort_action, "level_and_land")
        self.assertIn(
            "ready_timeout_automatic_prelude_violation",
            timed_out.prelude_failure_reasons,
        )

    def test_sample_gap_is_a_hard_untrusted_state_failure(self):
        attempt = self.automatic_attempt()
        acceleration_start = self.advance_to_acceleration(attempt)
        failed = self.observe_automatic(
            attempt,
            acceleration_start+0.02,
            sample_gap=0.101,
        )
        self.assertEqual(failed.abort_action, "level_and_land")
        self.assertIn(
            "sample_gap_automatic_prelude_violation",
            failed.prelude_failure_reasons,
        )

    def test_only_skipped_window_and_cross_speed_use_legacy_abort(self):
        for name, observation in (
            ("skipped", {"velocity": (0.0, 0.28)}),
            ("cross", {"velocity": (0.031, 0.10)}),
            ("release-context", {
                "velocity": (0.0, 0.25),
                "attitude": (math.radians(10.1), 0.0),
            }),
        ):
            with self.subTest(name=name):
                attempt = self.automatic_attempt()
                start = self.advance_to_acceleration(attempt)
                failed = self.observe_automatic(
                    attempt, start+0.01, **observation
                )
                self.assertEqual(
                    failed.abort_action,
                    "legacy_attitude_coast_to_rest",
                )

    def test_severe_automatic_failures_request_level_and_land(self):
        cases = (
            ("absolute-speed", {"velocity": (0.0, 0.401)}),
            ("wrong-way", {"velocity": (0.0, -0.051)}),
            ("timeout", {"time_offset": 0.101, "sample_gap": 0.02}),
            ("displacement", {"position": (0.201, 0.0)}),
            ("boundary", {"boundary": 0.099}),
            ("z", {"z_error": 0.051}),
        )
        for name, changes in cases:
            with self.subTest(name=name):
                changes = dict(changes)
                attempt = self.automatic_attempt()
                start = self.advance_to_acceleration(attempt)
                offset = changes.pop("time_offset", 0.01)
                failed = self.observe_automatic(
                    attempt, start+offset, **changes
                )
                self.assertEqual(failed.abort_action, "level_and_land")

    def test_automatic_config_caps_timeout_tilt_rate_and_boundary_budget(self):
        for values, expected in (
            ({"max_acceleration_duration_s": 0.851}, "0.85"),
            ({"max_automatic_tilt_deg": 12.1}, "12 degrees"),
            ({"max_automatic_rate_deg_s": 100.1}, "100 deg/s"),
            ({"ready_timeout_s": 5.1}, "5.0 s"),
            ({"max_cross_speed_m_s": 0.031}, "0.03 m/s"),
            ({"max_release_speed_m_s": 0.751}, "0.75 m/s"),
            ({"max_maneuver_displacement_m": 0.601}, "0.60 m"),
            ({"boundary_reserve_m": 0.099}, "0.10 m"),
        ):
            with self.subTest(values=values), self.assertRaisesRegex(
                ValueError, expected
            ):
                MPCBootstrapCalibrationConfig.from_mapping(values)

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

    def test_scheduled_send_rechecks_state_age_boundaries(self):
        self.assertTrue(mpc_decision_state_age_is_fresh(1.0, 1.0, 0.1))
        self.assertTrue(mpc_decision_state_age_is_fresh(1.0, 1.1, 0.1))
        self.assertFalse(mpc_decision_state_age_is_fresh(1.0, 1.1001, 0.1))
        self.assertFalse(mpc_decision_state_age_is_fresh(1.0, 0.999, 0.1))
        self.assertFalse(mpc_decision_state_age_is_fresh(1.0, float("nan"), 0.1))

    def test_prearm_preparation_applies_and_requires_baseline_fit(self):
        mission = self.mission()
        saved = self.saved()
        resolved = {
            "resolved": True,
            "mpc_bootstrap_calibration": {"enabled": True},
            "control_handoff": {
                "coast_attitude_response_delay_s": 0.12,
            },
        }
        with patch(
            "Interaction.mpc_bootstrap_calibration.apply_drone_calibration",
            return_value=(resolved, saved),
        ) as apply_fit, patch(
            "Interaction.mpc_bootstrap_calibration.planar_braking_fit_is_current",
            return_value=True,
        ):
            prepared = prepare_mpc_bootstrap_mission(
                mission, drone_id="lb11", controller_rate_hz=100,
            )
        wrench = prepared["Interaction"]["config"]["wrench_interaction"]
        self.assertTrue(wrench["resolved"])
        self.assertEqual(
            wrench["control_handoff"]["coast_attitude_response_delay_s"],
            0.12,
        )
        contracts = wrench["mpc_bootstrap_model_contracts"]
        self.assertEqual(contracts["positive_y"]["command_delay_s"], 0.03)
        self.assertEqual(contracts["negative_y"]["command_delay_s"], 0.05)
        self.assertNotEqual(
            contracts["positive_y"]["model_fingerprint"],
            contracts["negative_y"]["model_fingerprint"],
        )
        apply_fit.assert_called_once()

        with patch(
            "Interaction.mpc_bootstrap_calibration.apply_drone_calibration",
            return_value=({}, None),
        ), self.assertRaisesRegex(ValueError, "run --calibrate first"):
            prepare_mpc_bootstrap_mission(
                mission, drone_id="lb11", controller_rate_hz=100,
            )

    def test_directional_contract_selects_exact_delay_and_state_shape(self):
        contracts = build_mpc_bootstrap_model_contracts(
            self.saved(
                positive_delay=0.021,
                negative_delay=0.061,
            )["prediction_model"],
            prediction_step_s=0.02,
        )
        positive = mpc_bootstrap_model_contract_for_direction(
            contracts, [0.0, 1.0]
        )
        negative = mpc_bootstrap_model_contract_for_direction(
            contracts, [0.0, -1.0]
        )
        self.assertEqual(positive["command_delay_s"], 0.021)
        self.assertEqual(negative["command_delay_s"], 0.061)
        self.assertEqual(positive["state_dimension"], 5)
        self.assertEqual(negative["state_dimension"], 7)
        self.assertNotEqual(
            positive["model_fingerprint"], negative["model_fingerprint"]
        )
        self.assertEqual(
            positive["conditional_velocity_lmpc_config"][
                "prediction_step_s"
            ],
            0.02,
        )

    def test_directional_contract_tampering_fails_closed(self):
        contracts = build_mpc_bootstrap_model_contracts(
            self.saved()["prediction_model"], prediction_step_s=0.02
        )
        contracts["positive_y"]["command_delay_s"] = 0.031
        with self.assertRaisesRegex(ValueError, "does not match"):
            validate_mpc_bootstrap_model_contracts(
                contracts, expected_prediction_step_s=0.02
            )

    def test_prearm_rejects_missing_unvalidated_or_zero_directional_model(self):
        mission = self.mission()
        cases = []
        missing = self.saved()
        missing["prediction_model"]["directional_models"].pop("negative_y")
        cases.append(("missing", missing, "prediction_model"))
        unvalidated = self.saved()
        unvalidated["prediction_model"]["validation_passed"] = False
        unvalidated["prediction_model"]["validation"][
            "validation_passed"
        ] = False
        cases.append(("unvalidated", unvalidated, "not independently validated"))
        zero = self.saved(negative_delay=0.0)
        cases.append(("zero", zero, "positive directional command delay"))
        for name, saved, expected in cases:
            resolved = {
                "mpc_bootstrap_calibration": {"enabled": True},
                "control_handoff": {
                    "coast_attitude_response_delay_s": 0.12,
                },
            }
            with self.subTest(name=name), patch(
                "Interaction.mpc_bootstrap_calibration.apply_drone_calibration",
                return_value=(resolved, saved),
            ), patch(
                "Interaction.mpc_bootstrap_calibration.planar_braking_fit_is_current",
                return_value=True,
            ), self.assertRaisesRegex(ValueError, expected):
                prepare_mpc_bootstrap_mission(
                    mission, drone_id="lb11", controller_rate_hz=100,
                )

    def test_prearm_preparation_rejects_zero_fitted_delay(self):
        mission = {
            "drones": {"lb11": {"target": [0.0, 0.0, 1.0]}},
            "boundary_limits": {
                "x_min": -1.5, "x_max": 1.5,
                "y_min": -1.5, "y_max": 1.5,
            },
            "Interaction": {"action": "translation", "config": {
                "wrench_interaction": {},
            }},
        }
        resolved = {
            "mpc_bootstrap_calibration": {"enabled": True},
            "control_handoff": {
                "coast_attitude_response_delay_s": 0.0,
            },
        }
        saved = {"planar_braking_fit": {
            "fit": "current", "command_delay_s": 0.0,
        }}
        with patch(
            "Interaction.mpc_bootstrap_calibration.apply_drone_calibration",
            return_value=(resolved, saved),
        ), patch(
            "Interaction.mpc_bootstrap_calibration.planar_braking_fit_is_current",
            return_value=True,
        ), self.assertRaisesRegex(ValueError, "positive fitted command delay"):
            prepare_mpc_bootstrap_mission(
                mission, drone_id="lb11", controller_rate_hz=100,
            )

    def test_prearm_preparation_rejects_nonintegral_decision_rate(self):
        mission = {
            "drones": {"lb11": {"target": [0.0, 0.0, 1.0]}},
            "boundary_limits": {
                "x_min": -1.5, "x_max": 1.5,
                "y_min": -1.5, "y_max": 1.5,
            },
            "Interaction": {"action": "translation", "config": {
                "wrench_interaction": {
                    "mpc_bootstrap_calibration": {
                        "prediction_step_s": 0.015,
                    },
                },
            }},
        }
        with self.assertRaisesRegex(ValueError, "integral multiple"):
            prepare_mpc_bootstrap_mission(
                mission, drone_id="lb11", controller_rate_hz=100,
            )

    def test_prearm_requires_full_automatic_maneuver_boundary_budget(self):
        mission = self.mission()
        mission["boundary_limits"] = {
            "x_min": -1.5,
            "x_max": 1.5,
            "y_min": -0.699,
            "y_max": 1.5,
        }
        with self.assertRaisesRegex(
            ValueError, "0.700 m XY boundary margin"
        ):
            prepare_mpc_bootstrap_mission(
                mission, drone_id="lb11", controller_rate_hz=100,
            )


if __name__ == "__main__":
    unittest.main()
