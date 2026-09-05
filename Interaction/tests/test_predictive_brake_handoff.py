"""Pure tests for the reusable prediction brake-to-position episode."""
import copy
import math
import unittest
from unittest.mock import patch

import numpy as np

from Interaction.predictive_brake_handoff import (
    MOTION_RESIDUAL_CONFIG_KEYS,
    PredictiveBrakeToPosition,
    predictive_brake_to_position,
    projected_tilt_from_attitude_command,
    projected_tilt_from_world_acceleration,
    projected_tilt_history_from_world_acceleration,
    validated_prediction_model_for_interaction,
)
from Interaction.model_based_braking import DEFAULTS as BRAKING_DEFAULTS
from Interaction.tests.test_model_based_braking import model as base_model


def validated_model(*, approved=False, margin=0.01):
    source = base_model()
    quality = copy.deepcopy(source["identifiability"])
    source["directional_models"] = {
        label: {
            "direction_y": sign,
            "attitude_fit": copy.deepcopy(source["attitude_fit"]),
            "motion_gain": source["motion_gain"],
            "identifiability": copy.deepcopy(quality),
            "terminal_velocity_error_margin_m_s": margin,
        }
        for label, sign in (("positive_y", 1), ("negative_y", -1))
    }
    for row in source["data_ranges"]:
        row["phase_command_times_relative_s"] = {
            "brake": 0.50,
            "level_after_brake": 0.82,
        }
    required_gates = {
        "at_least_two_trials": True,
        "both_y_directions": True,
        "reported_segment_ids_match": True,
        "complete_per_trial_results": True,
        "per_trial_ids_match": True,
        "per_trial_directions_match": True,
        "parameters_identifiable": True,
        "no_active_parameter_bounds": True,
        "velocity_rmse_m_s": True,
        "terminal_error_m_s": True,
        "end_position_error_m": True,
        "no_reversal_misses_or_false_alarms": True,
        "directional_parameters_identifiable": True,
        "directional_no_active_parameter_bounds": True,
    }
    source.update({
        "control_eligible": True,
        "validation_passed": True,
        "independent_validation_complete": True,
        "runtime_enabled": approved,
        "deployment_approved": approved,
        "validation": {
            "training_segment_ids": [0, 1],
            "validation_segment_ids": [2, 3],
            "independent_validation": True,
            "validation_passed": True,
            "control_eligible": True,
            "failed_gates": [],
            "gates": required_gates,
            "per_trial": [
                {"segment_id": 2, "direction_y": 1},
                {"segment_id": 3, "direction_y": -1},
            ],
            "evaluation_scope": "conditional_on_executed_command_schedule",
            "runtime_enabled": approved,
            "deployment_approved": approved,
        },
    })
    return source


def state(
        now=10.0, *, x=0.0, y=0.0, vx=0.0, vy=0.0,
        roll=0.0, pitch=0.0, yaw=0.0, wx=0.0, wy=0.0, wz=0.0,
        acceleration=(0.0, 0.0)):
    return {
        "time_s": now,
        "position_xy": [x, y],
        "velocity_xy": [vx, vy],
        "orientation_rpy_rad": [roll, pitch, yaw],
        "angular_velocity_rad_s": [wx, wy, wz],
        "state_group_skew_s": 0.004,
        "battery_voltage_V": 7.4,
        "acceleration_xy": list(acceleration),
    }


class FakePredictor:
    def __init__(self, *args, **kwargs):
        self.records = []
        self.next = {
            "action": "brake",
            "reason": "rolling_prediction_continue_brake",
            "roll_deg": 20.0,
            "pitch_deg": 0.0,
            "projected_tilt_rad": -math.radians(20.0),
            "hard_terminal_constraints_satisfied": True,
            "state_extrapolates_training_range": False,
            "fallback_to_original_brake": False,
        }

    def record_command(self, sent_at_s, projected_tilt_rad):
        if self.records and sent_at_s <= self.records[-1][0]:
            raise ValueError("history_command_times_must_increase")
        self.records.append((sent_at_s, projected_tilt_rad))

    def decide(self, now_s, current_state):
        return copy.deepcopy(self.next)


def episode(
        *, direction=1, destination_y=0.30, config=None, model=None,
        history=None):
    options = {
        "allow_validated_experimental_model": True,
        "model_based_braking": {"max_compute_s": 1.0},
    }
    options.update(config or {})
    return predictive_brake_to_position(
        validated_model() if model is None else model,
        initial_state=state(vy=direction * 0.55),
        destination_position=[0.0, destination_y, 1.0],
        now_s=10.0,
        sent_command_history=(
            [(9.8, 0.0)] if history is None else history
        ),
        direction_xy=[0.0, direction],
        config=options,
    )


class PredictionModelLoadingTests(unittest.TestCase):
    def test_existing_world_acceleration_history_has_one_public_conversion(self):
        self.assertAlmostEqual(
            projected_tilt_from_world_acceleration(
                [0.0, -3.57055], [0.0, 1.0]
            ),
            -math.radians(20.0),
            places=5,
        )
        self.assertEqual(
            projected_tilt_history_from_world_acceleration(
                [(1.0, [0.0, 0.0]), (1.1, [0.0, -3.57055])],
                [0.0, 1.0],
            )[0],
            (1.0, 0.0),
        )
        negative = projected_tilt_from_world_acceleration(
            [0.0, 3.57055], [0.0, -1.0]
        )
        self.assertAlmostEqual(negative, -math.radians(20.0), places=5)
        self.assertAlmostEqual(
            projected_tilt_from_attitude_command(
                20.0, 0.0, 0.0, [0.0, 1.0]
            ),
            -math.radians(20.0),
        )

    def test_feature_off_is_backward_compatible(self):
        self.assertIsNone(validated_prediction_model_for_interaction(
            {}, enabled=False, direction_xy=[0.0, 1.0]
        ))

    def test_unapproved_model_requires_explicit_experimental_opt_in(self):
        source = validated_model()
        with self.assertRaisesRegex(ValueError, "not approved"):
            validated_prediction_model_for_interaction(
                {"prediction_model": source},
                enabled=True,
                direction_xy=[0.0, 1.0],
            )
        selected = validated_prediction_model_for_interaction(
            {"prediction_model": source},
            enabled=True,
            direction_xy=[0.0, 1.0],
            allow_validated_experimental_model=True,
        )
        self.assertEqual(selected, source)
        self.assertIsNot(selected, source)
        approved = validated_model(approved=True)
        self.assertEqual(
            validated_prediction_model_for_interaction(
                {"prediction_model": approved},
                enabled=True,
                direction_xy=[0.0, -1.0],
            ),
            approved,
        )

    def test_failed_or_incomplete_evidence_is_rejected(self):
        for mutate in (
                lambda item: item.update(control_eligible=False),
                lambda item: item["validation"]["failed_gates"].append("x"),
                lambda item: item["validation"]["gates"].update(x=False),
                lambda item: item.pop("validation")):
            source = validated_model()
            mutate(source)
            with self.subTest(source=source), self.assertRaises(ValueError):
                validated_prediction_model_for_interaction(
                    {"prediction_model": source},
                    enabled=True,
                    direction_xy=[0.0, 1.0],
                    allow_validated_experimental_model=True,
                )

    def test_simplified_or_internally_conflicting_evidence_is_rejected(self):
        cases = []
        source = validated_model()
        source["validation"]["gates"] = {"looks_good": True}
        cases.append(source)
        source = validated_model()
        source["validation"]["runtime_enabled"] = True
        cases.append(source)
        source = validated_model()
        source["validation"]["validation_segment_ids"] = [1, 2]
        cases.append(source)
        source = validated_model()
        source["validation"]["per_trial"][0]["segment_id"] = 99
        cases.append(source)
        for source in cases:
            with self.subTest(source=source), self.assertRaises(ValueError):
                validated_prediction_model_for_interaction(
                    {"prediction_model": source},
                    enabled=True,
                    direction_xy=[0.0, 1.0],
                    allow_validated_experimental_model=True,
                )

    def test_direction_margin_and_observed_envelope_are_hard_limits(self):
        with self.assertRaisesRegex(ValueError, "margin"):
            validated_prediction_model_for_interaction(
                {"prediction_model": validated_model(margin=0.05)},
                enabled=True,
                direction_xy=[0.0, 1.0],
                allow_validated_experimental_model=True,
            )
        with self.assertRaisesRegex(ValueError, "only world"):
            validated_prediction_model_for_interaction(
                {"prediction_model": validated_model()},
                enabled=True,
                direction_xy=[1.0, 0.0],
                allow_validated_experimental_model=True,
            )
        with self.assertRaisesRegex(ValueError, "tilt exceeds"):
            episode(config={"brake_tilt_deg": 20.1})
        with self.assertRaisesRegex(ValueError, "duration exceeds"):
            episode(config={"max_brake_duration_s": 0.321})
        asymmetric = validated_model()
        for row in asymmetric["data_ranges"]:
            if row["direction_y"] == 1:
                row["command_acceleration_m_s2"] = [-1.0, 6.0]
        with self.assertRaisesRegex(ValueError, "tilt exceeds"):
            episode(model=asymmetric, config={"brake_tilt_deg": 10.0})

        combined = validated_model()
        high_short = next(
            row for row in combined["data_ranges"] if row["direction_y"] == 1
        )
        high_short["phase_command_times_relative_s"]["level_after_brake"] = 0.66
        low_long = copy.deepcopy(high_short)
        low_long["command_acceleration_m_s2"] = [-1.3788, 1.3788]
        low_long["phase_command_times_relative_s"]["level_after_brake"] = 0.82
        combined["data_ranges"].append(low_long)
        with self.assertRaisesRegex(ValueError, "duration exceeds"):
            episode(
                model=combined,
                config={"brake_tilt_deg": 20.0, "max_brake_duration_s": 0.30},
            )

    def test_motion_residual_requires_exact_independent_hybrid_validation(self):
        residual_options = {
            "max_compute_s": 1.0,
            "motion_residual_observer_enabled": True,
        }
        with self.assertRaisesRegex(ValueError, "hybrid-predictor validation"):
            episode(config={"model_based_braking": residual_options})

        source = validated_model()
        source["motion_residual_validation"] = {
            "schema_version": 1,
            "predictor_kind": (
                "delayed_second_order_plus_causal_motion_residual"
            ),
            "independent_validation_complete": True,
            "validation_passed": True,
            "control_eligible": True,
            "failed_gates": [],
            "config": {
                key: BRAKING_DEFAULTS[key]
                for key in MOTION_RESIDUAL_CONFIG_KEYS
            },
        }
        with patch(
            "Interaction.predictive_brake_handoff.ModelBasedBrakingController",
            FakePredictor,
        ):
            item = episode(
                model=source,
                config={"model_based_braking": residual_options},
            )
        self.assertTrue(item._controller is not None)

        source["motion_residual_validation"]["config"][
            "motion_residual_apply_horizon_s"
        ] = .10
        with self.assertRaisesRegex(ValueError, "differs"):
            episode(
                model=source,
                config={"model_based_braking": residual_options},
            )

    def test_ambiguous_stationary_direction_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "cannot infer"):
            predictive_brake_to_position(
                validated_model(),
                initial_state=state(),
                destination_position=[0.0, 0.0, 1.0],
                now_s=10.0,
                sent_command_history=[(9.8, 0.0)],
                config={"allow_validated_experimental_model": True},
            )


class PredictiveBrakeToPositionTests(unittest.TestCase):
    @patch(
        "Interaction.predictive_brake_handoff.ModelBasedBrakingController",
        FakePredictor,
    )
    def test_only_successfully_sent_attitude_decisions_enter_history(self):
        item = episode()
        self.assertEqual(item._controller.records, [(9.8, 0.0)])
        decision = item.decide(10.0, state(10.0, vy=0.40))
        self.assertEqual(decision["action"], "brake")
        self.assertEqual(item._controller.records, [(9.8, 0.0)])
        item.record_sent(decision, 10.0)
        self.assertEqual(len(item._controller.records), 2)
        self.assertAlmostEqual(
            item._controller.records[-1][1], -math.radians(20.0)
        )

    @patch(
        "Interaction.predictive_brake_handoff.ModelBasedBrakingController",
        FakePredictor,
    )
    def test_decision_identity_and_validity_are_exposed_and_enforced(self):
        item = episode()
        decision = item.decide(10.0, state(10.0, vy=0.40))
        self.assertEqual(decision["decision_sequence"], 1)
        self.assertAlmostEqual(decision["valid_until_s"], 10.03)
        modified = dict(decision, projected_tilt_rad=0.0)
        self.assertFalse(item.record_sent(modified, 10.01))
        self.assertEqual(
            item.decide(10.02, state(10.02, vy=0.35))["action"],
            "abort_level",
        )

        item = episode()
        decision = item.decide(10.0, state(10.0, vy=0.40))
        modified = dict(decision, roll_deg=-999.0)
        self.assertFalse(item.record_sent(modified, 10.01))
        self.assertEqual(item._send_protocol_error,
                         "sent_attitude_did_not_match_decision")

        item = episode()
        decision = item.decide(10.0, state(10.0, vy=0.40))
        self.assertFalse(item.record_sent(
            decision,
            10.01,
            actual_attitude={
                "roll_deg": -20.0,
                "pitch_deg": 0.0,
                "yaw_rate_deg_s": 0.0,
                "projection_yaw_rad": 0.0,
            },
        ))
        self.assertAlmostEqual(
            item._controller.records[-1][1], math.radians(20.0)
        )

        item = episode()
        decision = item.decide(10.0, state(10.0, vy=0.40))
        self.assertFalse(item.record_sent(decision, 10.04))
        abort = item.decide(10.05, state(10.05, vy=0.35))
        self.assertEqual(abort["action"], "abort_level")
        self.assertEqual(
            abort["reason"], "attitude_decision_sent_outside_valid_phase"
        )

    @patch(
        "Interaction.predictive_brake_handoff.ModelBasedBrakingController",
        FakePredictor,
    )
    def test_superseded_brake_is_recorded_but_forces_abort(self):
        item = episode()
        old = item.decide(10.0, state(10.0, vy=0.40))
        current = item.decide(10.01, state(10.01, vy=0.35))
        self.assertGreater(
            current["decision_sequence"], old["decision_sequence"]
        )
        self.assertFalse(item.record_sent(old, 10.015))
        self.assertEqual(len(item._controller.records), 2)
        abort = item.decide(10.02, state(10.02, vy=0.30))
        self.assertEqual(abort["action"], "abort_level")

    @patch(
        "Interaction.predictive_brake_handoff.ModelBasedBrakingController",
        FakePredictor,
    )
    def test_unrecorded_decision_bookkeeping_is_bounded(self):
        item = episode()
        for index in range(100):
            now = 10.0 + index * 0.001
            item.decide(now, state(now, vy=0.40))
        self.assertLessEqual(len(item._issued_attitude_decisions), 64)

    @patch(
        "Interaction.predictive_brake_handoff.ModelBasedBrakingController",
        FakePredictor,
    )
    def test_prediction_failure_levels_and_aborts_without_fixed_pulse(self):
        for response in (
                {"action": "fallback", "reason": "invalid model"},
                {
                    "action": "brake",
                    "reason": "compute overrun",
                    "fallback_to_original_brake": True,
                }):
            item = episode()
            item._controller.next = response
            decision = item.decide(10.0, state(10.0, vy=0.40))
            self.assertEqual(decision["action"], "abort_level")
            self.assertEqual(decision["roll_deg"], 0.0)
            self.assertEqual(decision["pitch_deg"], 0.0)
            self.assertEqual(decision["projected_tilt_rad"], 0.0)
            self.assertIn("without_runtime_fixed_pulse", decision["reason"])

    @patch(
        "Interaction.predictive_brake_handoff.ModelBasedBrakingController",
        FakePredictor,
    )
    def test_acceleration_or_retry_action_is_never_forwarded(self):
        item = episode()
        item._controller.next["action"] = "acceleration"
        decision = item.decide(10.0, state(10.0, vy=0.40))
        self.assertEqual(decision["action"], "abort_level")
        self.assertEqual(decision["reason"], "invalid_prediction_action")

    @patch(
        "Interaction.predictive_brake_handoff.ModelBasedBrakingController",
        FakePredictor,
    )
    def test_extrapolated_state_aborts_even_when_model_opt_in_is_allowed(self):
        item = episode()
        item._controller.next["state_extrapolates_training_range"] = True
        decision = item.decide(10.0, state(10.0, vy=0.40))
        self.assertEqual(decision["action"], "abort_level")
        self.assertEqual(decision["reason"], "state_outside_identified_range")

    def test_high_yaw_rate_cannot_authorize_braking_or_position(self):
        item = episode()
        decision = item.decide(
            10.0,
            state(10.0, vy=0.40, wz=0.36),
        )
        self.assertEqual(decision["action"], "abort_level")
        self.assertEqual(decision["reason"], "yaw_rate_outside_prediction_domain")

    @patch(
        "Interaction.predictive_brake_handoff.ModelBasedBrakingController",
        FakePredictor,
    )
    def test_level_waits_for_response_measured_gates_and_dwell(self):
        item = episode(history=[(9.8, -math.radians(20.0))])
        item._controller.next.update({
            "action": "level",
            "reason": "rolling_prediction_selected_level",
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "projected_tilt_rad": 0.0,
        })
        too_soon = item.decide(10.0, state(10.0))
        self.assertEqual(too_soon["action"], "level")
        self.assertFalse(too_soon["response_queue_settled"])
        self.assertTrue(item.record_sent(too_soon, 10.0))
        ready_start = item.decide(10.40, state(10.40))
        self.assertEqual(ready_start["action"], "level")
        self.assertTrue(ready_start["response_queue_settled"])
        self.assertTrue(ready_start["measured_handoff_safe"])
        handed_off = item.decide(10.46, state(10.46, x=0.04, y=0.35))
        self.assertEqual(handed_off["action"], "position")
        self.assertEqual(handed_off["phase"], item.POSITION)
        self.assertAlmostEqual(handed_off["position_target"][0], 0.04)
        self.assertAlmostEqual(handed_off["position_target"][1], 0.35)
        self.assertAlmostEqual(handed_off["position_target"][2], 1.0)
        self.assertTrue(handed_off["target_clamped_to_actual"])
        self.assertTrue(handed_off["lateral_target_latched_to_actual"])
        latched = item.decide(10.50, state(10.50, x=0.04, y=0.38))
        self.assertGreater(
            latched["decision_sequence"], handed_off["decision_sequence"]
        )
        self.assertAlmostEqual(latched["valid_until_s"], 10.53)
        self.assertEqual(latched["position_target"], [0.04, 0.38, 1.0])
        self.assertEqual(latched["reason"], "position_target_ratchet_advanced")
        moved_back = item.decide(10.51, state(10.51, x=0.04, y=0.37))
        self.assertEqual(moved_back["position_target"], [0.04, 0.38, 1.0])

    @patch(
        "Interaction.predictive_brake_handoff.ModelBasedBrakingController",
        FakePredictor,
    )
    def test_position_ratchet_rejects_stale_localization(self):
        item = episode()
        item.phase = item.POSITION
        item.position_target = np.array([0.0, 0.30, 1.0])
        item.position_target.setflags(write=False)
        decision = item.decide(10.5, state(10.0, y=0.5))
        self.assertEqual(decision["action"], "abort_level")
        self.assertEqual(decision["reason"], "invalid_position_handoff_state")

    @patch(
        "Interaction.predictive_brake_handoff.ModelBasedBrakingController",
        FakePredictor,
    )
    def test_position_ratchet_rejects_fresh_localization_jump(self):
        item = episode()
        item.phase = item.POSITION
        item.position_target = np.array([0.0, 0.30, 1.0])
        item.position_target.setflags(write=False)
        item._last_position_state_time_s = 10.0
        item._last_position_actual_xy = np.array([0.0, 0.30])
        decision = item.decide(10.01, state(10.01, y=100.0))
        self.assertEqual(decision["action"], "abort_level")
        self.assertEqual(decision["position_target"], [0.0, 0.30, 1.0])

    @patch(
        "Interaction.predictive_brake_handoff.ModelBasedBrakingController",
        FakePredictor,
    )
    def test_position_ratchet_has_a_total_forward_extension_limit(self):
        item = episode(config={"max_total_forward_extension_m": 0.05})
        item.phase = item.POSITION
        item.position_target = np.array([0.0, 0.30, 1.0])
        item.position_target.setflags(write=False)
        item._last_position_state_time_s = 10.0
        item._last_position_actual_xy = np.array([0.0, 0.30])
        first = item.decide(10.02, state(10.02, y=0.34))
        self.assertEqual(first["position_target"], [0.0, 0.34, 1.0])
        second = item.decide(10.04, state(10.04, y=0.36))
        self.assertEqual(second["action"], "abort_level")
        self.assertEqual(second["position_target"], [0.0, 0.34, 1.0])

    @patch(
        "Interaction.predictive_brake_handoff.ModelBasedBrakingController",
        FakePredictor,
    )
    def test_initial_position_clamp_cannot_bypass_total_extension_limit(self):
        item = episode(
            config={"max_total_forward_extension_m": 0.05},
            history=[(9.8, -math.radians(20.0))],
        )
        item._controller.next.update({
            "action": "level",
            "reason": "rolling_prediction_selected_level",
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "projected_tilt_rad": 0.0,
        })
        first = item.decide(10.0, state(10.0))
        item.record_sent(first, 10.0)
        item.decide(10.40, state(10.40))
        handoff = item.decide(10.46, state(10.46, y=0.39))
        self.assertEqual(handoff["action"], "abort_level")
        self.assertEqual(handoff["reason"], "invalid_position_handoff_state")

    def test_slow_model_that_cannot_settle_before_timeout_is_rejected(self):
        source = validated_model()
        for component in source["directional_models"].values():
            component["attitude_fit"]["wn_rad_s"] = 5.0
            component["attitude_fit"]["zeta"] = 0.2
        with self.assertRaisesRegex(ValueError, "cannot cover"):
            episode(model=source, config={"max_attitude_phase_s": 3.0})

    @patch(
        "Interaction.predictive_brake_handoff.ModelBasedBrakingController",
        FakePredictor,
    )
    def test_unsafe_measured_state_prevents_position_handoff(self):
        item = episode()
        item._controller.next.update({
            "action": "level",
            "reason": "rolling_prediction_selected_level",
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "projected_tilt_rad": 0.0,
        })
        decision = item.decide(
            10.5,
            state(10.5, vy=0.08, roll=math.radians(4.0)),
        )
        self.assertEqual(decision["action"], "level")
        self.assertFalse(decision["measured_handoff_safe"])
        self.assertIsNone(decision["handoff_ready_elapsed_s"])

    def test_real_prediction_engine_uses_current_state_and_destination(self):
        item = episode()
        decision = item.decide(
            10.0,
            state(
                10.0,
                y=0.0,
                vy=0.55,
                roll=-0.15,
                wx=1.0,
            ),
        )
        self.assertIn(decision["action"], ("brake", "level"))
        self.assertNotEqual(decision["action"], "abort_level")
        self.assertEqual(
            decision["prediction"]["target_projected_position_m"], 0.30
        )
        self.assertEqual(
            decision["prediction"]["selected_directional_model"],
            "positive_y",
        )
        self.assertFalse(decision["position_handoff_latched"])

    def test_real_prediction_fails_closed_without_effective_send_history(self):
        item = episode(history=[])
        decision = item.decide(10.0, state(10.0, vy=0.55))
        self.assertEqual(decision["action"], "abort_level")
        self.assertIn("without_runtime_fixed_pulse", decision["reason"])

    def test_real_predictor_selects_the_frozen_negative_y_component(self):
        item = episode(direction=-1, destination_y=-0.30)
        decision = item.decide(
            10.0,
            state(10.0, vy=-0.55, roll=0.15, wx=-1.0),
        )
        self.assertIn(decision["action"], ("brake", "level"))
        self.assertEqual(
            decision["prediction"]["selected_directional_model"],
            "negative_y",
        )

    def test_target_and_model_are_frozen_for_episode(self):
        source = validated_model()
        destination = [0.0, 0.30, 1.0]
        item = predictive_brake_to_position(
            source,
            initial_state=state(vy=0.55),
            destination_position=destination,
            now_s=10.0,
            sent_command_history=[(9.8, 0.0)],
            direction_xy=[0.0, 1.0],
            config={"allow_validated_experimental_model": True},
        )
        source["attitude_fit"]["delay_s"] = 0.15
        destination[1] = 9.0
        np.testing.assert_array_equal(
            item.destination_position, [0.0, 0.30, 1.0]
        )
        self.assertEqual(item.model["attitude_fit"]["delay_s"], 0.03)


if __name__ == "__main__":
    unittest.main()
