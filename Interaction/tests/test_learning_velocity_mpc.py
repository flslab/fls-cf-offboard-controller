"""Synthetic tests only; this suite never imports cflib or sends commands."""
import math
import unittest

from Interaction.learning_velocity_mpc import (
    CausalAccelerationResidualLearner,
    LearningVelocityMPC,
    VelocityMPCConfig,
    VelocityMPCState,
    frozen_velocity_model_from_prediction_model,
)
from Interaction.offline_braking_selector import FrozenTiltModel
from Interaction.model_based_braking import _second_order_transition


def model():
    return FrozenTiltModel(
        delay_s=0.03, wn_rad_s=14.0, zeta=0.8,
        command_gain=1.0, motion_gain=1.0,
    )


def state(time_s=0.0, *, velocity=(0.0, 0.0), roll=0.0, pitch=0.0,
          yaw=0.0, roll_rate=0.0, pitch_rate=0.0, skew=0.004):
    return VelocityMPCState(
        time_s=time_s,
        velocity_xy=velocity,
        orientation_rpy_rad=(roll, pitch, yaw),
        angular_velocity_rad_s=(roll_rate, pitch_rate, 0.0),
        state_group_skew_s=skew,
    )


class LearningVelocityMPCTests(unittest.TestCase):
    def controller(self, target=0.6, direction=(0.0, 1.0), **config):
        item = LearningVelocityMPC(
            model(), direction_xy=direction, target_velocity_m_s=target,
            config=VelocityMPCConfig(**config),
        )
        item.record_sent_command(-0.05, 0.0)
        return item

    def test_default_configuration_is_bounded(self):
        for update in (
            dict(max_acceleration_tilt_deg=30.0),
            dict(max_tilt_deg=30.0),
            dict(terminal_velocity_tolerance_m_s=0.11),
            dict(terminal_tilt_tolerance_deg=5.1),
            dict(velocity_uncertainty_margin_m_s=0.03,
                 overshoot_tolerance_m_s=0.02),
            dict(max_state_age_s=0.11),
            dict(include_selected_trace=1),
        ):
            with self.subTest(update=update), self.assertRaises(ValueError):
                VelocityMPCConfig(**update).validate()

    def test_saved_directional_prediction_model_can_seed_shadow_mpc(self):
        prediction = {
            "schema_version": 1,
            "kind": "delayed_second_order_planar_prediction",
            "prediction_scope": "attitude_command_only",
            "identifiability": {
                "identifiable": True, "bound_active_parameters": [],
            },
            "data_ranges": [{
                "direction_y": sign,
                "command_acceleration_m_s2": [-3.0, 3.0],
                "velocity_m_s": [-1.0, 1.0],
                "theta_rad": [-0.4, 0.4],
            } for sign in (-1, 1)],
            "directional_models": {
                "positive_y": {
                    "direction_y": 1,
                    "identifiability": {
                        "identifiable": True, "bound_active_parameters": [],
                    },
                    "attitude_fit": {
                        "model": "second_order", "delay_s": 0.03,
                        "wn_rad_s": 14.0, "zeta": 0.8, "gain": 1.0,
                        "bias_world_y_rad": 0.01,
                    },
                    "motion_gain": 0.9,
                },
                "negative_y": {
                    "direction_y": -1,
                    "identifiability": {
                        "identifiable": True, "bound_active_parameters": [],
                    },
                    "attitude_fit": {
                        "model": "second_order", "delay_s": 0.04,
                        "wn_rad_s": 13.0, "zeta": 0.9, "gain": 0.95,
                        "bias_world_y_rad": -0.01,
                    },
                    "motion_gain": 0.8,
                },
            },
        }
        frozen, label = frozen_velocity_model_from_prediction_model(
            prediction, direction_y=-1,
        )
        self.assertEqual(label, "negative_y")
        self.assertEqual(frozen.delay_s, 0.04)
        self.assertEqual(frozen.motion_gain, 0.8)
        with self.assertRaisesRegex(ValueError, "independently validated"):
            frozen_velocity_model_from_prediction_model(
                prediction,
                direction_y=-1,
                require_validated_evidence=True,
            )

    def test_missing_sent_history_fails_closed_without_command(self):
        item = LearningVelocityMPC(
            model(), direction_xy=(0.0, 1.0), target_velocity_m_s=0.5,
        )
        result = item.decide(0.0, state())
        self.assertEqual(result["action"], "fallback_level")
        self.assertEqual(result["reason"], "missing_actual_sent_command_history")
        self.assertIsNone(result["roll_deg"])

    def test_accelerates_toward_positive_y_with_slew_limited_roll(self):
        item = self.controller(target=0.6)
        result = item.decide(0.0, state())
        self.assertEqual(result["action"], "accelerate")
        self.assertGreater(result["projected_command_tilt_rad"], 0.0)
        self.assertLess(result["roll_deg"], 0.0)
        self.assertAlmostEqual(result["pitch_deg"], 0.0, places=9)
        self.assertLessEqual(
            abs(math.degrees(result["projected_command_tilt_rad"])), 1.8+1e-9
        )
        self.assertTrue(result["hard_path_constraints_satisfied"])
        self.assertLessEqual(result["predicted_max_signed_overshoot_m_s"], 0.02+1e-9)

    def test_online_selection_can_skip_duplicate_diagnostic_trace(self):
        item = self.controller(
            target=0.6,
            include_selected_trace=False,
            prediction_horizon_s=1.0,
            pulse_grid_step_s=0.02,
        )
        result = item.decide(0.0, state())
        self.assertEqual(result["action"], "accelerate")
        self.assertEqual(result["selected_trace"], [])
        self.assertEqual(result["candidate_count"], 47)
        self.assertTrue(result["hard_path_constraints_satisfied"])

    def test_negative_target_uses_opposite_attitude(self):
        item = self.controller(target=-0.5)
        result = item.decide(0.0, state())
        self.assertEqual(result["action"], "accelerate")
        self.assertLess(result["projected_command_tilt_rad"], 0.0)
        self.assertGreater(result["roll_deg"], 0.0)

    def test_world_x_direction_maps_to_pitch(self):
        item = self.controller(target=0.5, direction=(1.0, 0.0))
        result = item.decide(0.0, state())
        self.assertEqual(result["action"], "accelerate")
        self.assertAlmostEqual(result["roll_deg"], 0.0, places=9)
        self.assertLess(result["pitch_deg"], 0.0)

    def test_target_state_commands_level_and_checks_terminal_attitude(self):
        item = self.controller(target=0.5)
        result = item.decide(0.0, state(velocity=(0.0, 0.5)))
        self.assertEqual(result["action"], "hold_level")
        self.assertTrue(result["hard_terminal_constraints_satisfied"])
        self.assertEqual(result["roll_deg"], 0.0)
        self.assertEqual(result["pitch_deg"], 0.0)

    def test_target_speed_with_nonlevel_attitude_is_not_terminal(self):
        item = self.controller(target=0.5)
        result = item.decide(
            0.0, state(velocity=(0.0, 0.5), roll=math.radians(-8.0))
        )
        self.assertEqual(result["action"], "hold_level")
        self.assertFalse(result["hard_terminal_constraints_satisfied"])

    def test_stale_or_cross_axis_state_fails_closed(self):
        stale = self.controller().decide(0.2, state(0.0))
        self.assertEqual(stale["action"], "fallback_level")
        cross = self.controller().decide(0.0, state(velocity=(0.2, 0.0)))
        self.assertEqual(cross["action"], "fallback_level")

    def test_residual_learner_waits_then_changes_forecast_model(self):
        learner = CausalAccelerationResidualLearner(
            learning_rate=1.0, max_abs_acceleration_m_s2=1.0,
            min_samples=3,
        )
        for _ in range(2):
            snapshot = learner.update(
                dt_s=0.02, velocity_before_m_s=0.0,
                velocity_after_m_s=0.01, mean_projected_tilt_rad=0.0,
                motion_gain=1.0,
            )
            self.assertFalse(snapshot["ready"])
        snapshot = learner.update(
            dt_s=0.02, velocity_before_m_s=0.0,
            velocity_after_m_s=0.01, mean_projected_tilt_rad=0.0,
            motion_gain=1.0,
        )
        self.assertTrue(snapshot["ready"])
        self.assertAlmostEqual(snapshot["acceleration_m_s2"], 0.5)
        rejected = learner.update(
            dt_s=0.02, velocity_before_m_s=0.0,
            velocity_after_m_s=1.0, mean_projected_tilt_rad=0.0,
            motion_gain=1.0,
        )
        self.assertEqual(rejected["status"], "outlier_rejected")
        self.assertAlmostEqual(rejected["acceleration_m_s2"], 0.5)

    def test_receding_horizon_closed_loop_reaches_speed_level_without_overshoot(self):
        item = self.controller(target=0.6)
        dt = 0.01
        transition = _second_order_transition(14.0, 0.8, dt)
        delayed_commands = [0.0, 0.0, 0.0]
        velocity = tilt = tilt_rate = 0.0
        maximum_velocity = velocity
        final = None
        for index in range(250):
            stamp = index*dt
            snapshot = state(
                stamp, velocity=(0.0, velocity), roll=-tilt,
                roll_rate=-tilt_rate,
            )
            decision = item.decide(stamp, snapshot)
            self.assertNotEqual(decision["action"], "fallback_level", decision)
            command = decision["projected_command_tilt_rad"]
            item.record_sent_command(stamp, command)
            delayed_commands.append(command)
            effective = delayed_commands.pop(0)
            equilibrium = effective
            centered = transition @ [tilt-equilibrium, tilt_rate]
            next_tilt = float(centered[0]+equilibrium)
            next_rate = float(centered[1])
            a0 = 9.81*math.tan(tilt)
            a1 = 9.81*math.tan(next_tilt)
            velocity += 0.5*(a0+a1)*dt
            tilt, tilt_rate = next_tilt, next_rate
            maximum_velocity = max(maximum_velocity, velocity)
            final = decision
            if (abs(velocity-0.6) <= 0.05
                    and abs(math.degrees(tilt)) <= 3.0
                    and abs(math.degrees(tilt_rate)) <= 20.0
                    and abs(math.degrees(command)) <= 3.0):
                break
        else:
            self.fail("closed-loop MPC did not reach the terminal velocity set")
        self.assertLessEqual(maximum_velocity, 0.62+1e-9)
        self.assertTrue(final["hard_path_constraints_satisfied"])


if __name__ == "__main__":
    unittest.main()
