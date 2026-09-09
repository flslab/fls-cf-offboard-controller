"""Offline-only tests for the paper-structured conditional LMPC core."""
import math
import unittest

import numpy as np

from Interaction.conditional_velocity_lmpc import (
    ConditionalVelocityLMPCConfig,
    ConditionalVelocityLMPCState,
    OfflineConditionalVelocityLMPC,
    conditional_velocity_lmpc_fingerprint,
)
from Interaction.offline_braking_selector import FrozenTiltModel
from Interaction.velocity_lmpc_safe_set import (
    LMPCContext,
    SafeSetLimits,
    TERMINAL_OUTCOME,
    VelocityLMPCEpisode,
    VelocityLMPCSafeSet,
    VelocityLMPCSample,
)


def model(delay_s=0.04):
    return FrozenTiltModel(
        delay_s=delay_s,
        wn_rad_s=14.0,
        zeta=0.8,
        command_gain=1.0,
        motion_gain=1.0,
    )


class ConditionalVelocityLMPCTests(unittest.TestCase):
    def planner(self, **updates):
        return OfflineConditionalVelocityLMPC(
            model(), ConditionalVelocityLMPCConfig(**updates)
        )

    def initial_state(self, velocity=0.4, *, queue=(0.0, 0.0),
                      available_distance=2.0):
        return ConditionalVelocityLMPCState(
            velocity_m_s=velocity,
            projected_tilt_rad=0.0,
            projected_tilt_rate_rad_s=0.0,
            previous_command_rad=queue[-1],
            pending_commands_rad=queue,
            available_distance_m=available_distance,
        )

    def baseline_commands(self):
        return np.radians([-3, -6, -8, -8, -6, -4, -2, 0, 0, 0])

    def test_delay_queue_is_part_of_markov_state(self):
        planner = self.planner()
        state = self.initial_state(queue=(0.0, math.radians(-3.0)))
        vector = state.vector(delay_steps=planner.delay_steps)
        self.assertEqual(planner.state_dimension, 5)
        self.assertEqual(vector.shape, (5,))
        self.assertAlmostEqual(vector[-1], math.radians(-3.0))
        with self.assertRaisesRegex(ValueError, "previous command"):
            ConditionalVelocityLMPCState(
                velocity_m_s=0.4,
                projected_tilt_rad=0.0,
                projected_tilt_rate_rad_s=0.0,
                previous_command_rad=0.0,
                pending_commands_rad=(0.0, math.radians(-3.0)),
            ).vector(delay_steps=planner.delay_steps)

    def test_pending_command_changes_prediction_before_new_command_arrives(self):
        planner = self.planner()
        level, _ = planner.rollout(
            self.initial_state(queue=(0.0, 0.0)), [0.0, 0.0, 0.0]
        )
        pending, _ = planner.rollout(
            self.initial_state(queue=(math.radians(-8.0),)*2),
            [0.0, 0.0, 0.0],
        )
        self.assertLess(pending[-1, 0], level[-1, 0])
        self.assertLess(pending[-1, 1], level[-1, 1])

    def test_fractional_delay_switches_effective_command_inside_step(self):
        fractional = OfflineConditionalVelocityLMPC(model(delay_s=0.03))
        rounded = OfflineConditionalVelocityLMPC(model(delay_s=0.04))
        queue = (math.radians(-8.0), 0.0)
        state = self.initial_state(queue=queue)
        fractional_states, _ = fractional.rollout(state, [0.0])
        rounded_states, _ = rounded.rollout(state, [0.0])
        self.assertAlmostEqual(fractional.delay_remainder_s, 0.01)
        # 30 ms delay applies the old command for half this 20 ms prediction
        # step; rounding it to 40 ms would incorrectly apply it for the whole.
        self.assertGreater(fractional_states[-1, 1], rounded_states[-1, 1])

    def test_forward_distance_bound_does_not_hide_zero_crossing(self):
        distance = OfflineConditionalVelocityLMPC._forward_distance_upper_bound(
            0.05, -0.05, 0.02
        )
        self.assertGreater(distance, 0.0)
        self.assertAlmostEqual(distance, 0.001)

    def test_solves_terminal_convex_safe_set_and_remains_offline(self):
        planner = self.planner()
        initial = self.initial_state()
        warm = self.baseline_commands()
        baseline_states, _ = planner.rollout(initial, warm)
        terminal = baseline_states[-1]
        # The symmetric neighbors make the feasible warm-start terminal state
        # an interior convex combination rather than only a safe-set vertex.
        delta = np.asarray([0.01, 0.001, 0.002, 0.0, 0.0])
        safe_states = np.vstack([terminal, terminal+delta, terminal-delta])
        plan = planner.solve(
            initial,
            safe_states=safe_states,
            safe_cost_to_go=[0.20, 0.30, 0.40],
            warm_start_commands_rad=warm,
        )
        self.assertTrue(plan.optimizer_feasible, plan)
        self.assertEqual(
            plan.reason, "offline_relaxed_conditional_safe_set_solution"
        )
        self.assertTrue(plan.offline_only)
        self.assertFalse(plan.flight_command_dispatched)
        self.assertFalse(plan.safety_certified)
        self.assertIsNotNone(plan.first_command_rad)
        self.assertLess(plan.first_command_rad, 0.0)
        self.assertAlmostEqual(sum(plan.terminal_weights), 1.0, places=7)
        self.assertLessEqual(plan.equality_residual_inf, 2e-5)
        self.assertGreaterEqual(plan.minimum_inequality_margin, -2e-6)

        predicted_terminal = np.asarray(plan.predicted_states[-1])
        convex_terminal = safe_states.T@np.asarray(plan.terminal_weights)
        np.testing.assert_allclose(
            predicted_terminal, convex_terminal, atol=2e-5, rtol=0.0
        )

    def test_empty_or_wrong_dimension_safe_set_fails_closed(self):
        planner = self.planner()
        for safe in ([], [[0.0, 0.0, 0.0]]):
            with self.subTest(safe=safe):
                result = planner.solve(
                    self.initial_state(),
                    safe_states=safe,
                    safe_cost_to_go=[],
                    warm_start_commands_rad=self.baseline_commands(),
                )
                self.assertFalse(result.optimizer_feasible)
                self.assertTrue(result.offline_only)
                self.assertFalse(result.flight_command_dispatched)
                self.assertIn("invalid_problem", result.reason)

    def test_varying_release_speed_forces_conditional_interpolation(self):
        planner = self.planner()
        initial = self.initial_state()
        warm = self.baseline_commands()
        baseline_states, _ = planner.rollout(initial, warm)
        terminal = baseline_states[-1]
        delta = np.asarray([0.01, 0.001, 0.002, 0.0, 0.0])

        plan = planner.solve(
            initial,
            safe_states=[terminal+delta, terminal-delta],
            safe_cost_to_go=[0.20, 0.40],
            safe_release_speeds_m_s=[0.40, 0.80],
            release_initial_speed_m_s=0.60,
            warm_start_commands_rad=warm,
        )

        self.assertTrue(plan.optimizer_feasible, plan)
        self.assertAlmostEqual(plan.terminal_weights[0], 0.5, places=7)
        self.assertAlmostEqual(plan.terminal_weights[1], 0.5, places=7)

    def test_conditional_release_speed_refuses_extrapolation(self):
        planner = self.planner()
        initial = self.initial_state()
        warm = self.baseline_commands()
        states, _ = planner.rollout(initial, warm)
        result = planner.solve(
            initial,
            safe_states=[states[-1], states[-1]],
            safe_cost_to_go=[0.20, 0.20],
            safe_release_speeds_m_s=[0.40, 0.80],
            release_initial_speed_m_s=0.81,
            warm_start_commands_rad=warm,
        )
        self.assertFalse(result.optimizer_feasible)
        self.assertIn("outside safe-set coverage", result.reason)

    def test_solver_deadline_fails_closed(self):
        planner = self.planner()
        initial = self.initial_state()
        warm = self.baseline_commands()
        states, _ = planner.rollout(initial, warm)
        result = planner.solve(
            initial,
            safe_states=[states[-1]],
            safe_cost_to_go=[0.0],
            warm_start_commands_rad=warm,
            solver_deadline_s=1e-12,
        )
        self.assertFalse(result.optimizer_feasible)
        self.assertEqual(result.reason, "solver_deadline_exceeded")
        self.assertIsNone(result.first_command_rad)

    def test_initial_state_and_pending_command_bounds_fail_closed(self):
        planner = self.planner()
        safe = [[0.0, 0.0, 0.0, 0.0, 0.0]]
        invalid = (
            self.initial_state(velocity=-0.021),
            ConditionalVelocityLMPCState(
                velocity_m_s=0.4,
                projected_tilt_rad=math.radians(15.1),
                projected_tilt_rate_rad_s=0.0,
                previous_command_rad=0.0,
                pending_commands_rad=(0.0, 0.0),
            ),
            self.initial_state(
                queue=(0.0, math.radians(8.1))
            ),
        )
        for state in invalid:
            with self.subTest(state=state):
                result = planner.solve(
                    state,
                    safe_states=safe,
                    safe_cost_to_go=[0.0],
                    warm_start_commands_rad=np.zeros(10),
                )
                self.assertFalse(result.optimizer_feasible)
                self.assertIn("hard bound", result.reason)

    def test_unavoidable_intermediate_velocity_limit_violation_fails_closed(self):
        planner = self.planner()
        positive_limit = math.radians(
            planner.config.max_command_tilt_deg
        )
        initial = self.initial_state(
            velocity=1.99,
            queue=(positive_limit, positive_limit),
            available_distance=10.0,
        )
        warm = np.r_[
            np.full(5, -positive_limit),
            np.zeros(5),
        ]
        states, _ = planner.rollout(initial, warm)
        self.assertGreater(np.max(states[:, 0]), 2.0)
        self.assertLess(states[-1, 0], 2.0)

        plan = planner.solve(
            initial,
            safe_states=[states[-1]],
            safe_cost_to_go=[0.0],
            warm_start_commands_rad=warm,
        )

        self.assertFalse(plan.optimizer_feasible)
        self.assertIsNone(plan.first_command_rad)

    def test_safe_tail_distance_is_reserved_inside_workspace_limit(self):
        planner = self.planner()
        initial = self.initial_state(available_distance=0.05)
        warm = self.baseline_commands()
        states, _ = planner.rollout(initial, warm)
        result = planner.solve(
            initial,
            safe_states=[states[-1]],
            safe_cost_to_go=[0.20],
            safe_tail_forward_distances_m=[0.10],
            warm_start_commands_rad=warm,
        )
        self.assertFalse(result.optimizer_feasible)

    def test_configuration_rejects_unbounded_attitude(self):
        with self.assertRaises(ValueError):
            ConditionalVelocityLMPCConfig(
                max_command_tilt_deg=30.0
            ).validate()

    def test_fingerprint_binds_model_timing_and_hard_constraints(self):
        default = ConditionalVelocityLMPCConfig()
        self.assertEqual(
            conditional_velocity_lmpc_fingerprint(model(), default),
            conditional_velocity_lmpc_fingerprint(model(), default),
        )
        self.assertNotEqual(
            conditional_velocity_lmpc_fingerprint(model(), default),
            conditional_velocity_lmpc_fingerprint(
                model(), ConditionalVelocityLMPCConfig(
                    max_command_tilt_deg=7.0
                )
            ),
        )
        self.assertNotEqual(
            conditional_velocity_lmpc_fingerprint(model(), default),
            conditional_velocity_lmpc_fingerprint(
                model(), default,
                SafeSetLimits(terminal_velocity_tolerance_m_s=0.04),
            ),
        )

    def test_planner_rejects_safe_set_with_different_admission_limits(self):
        planner = self.planner()
        artifact = VelocityLMPCSafeSet(
            state_dimension=planner.state_dimension,
            command_dimension=1,
            aligned_velocity_state_index=0,
            state_scales=(0.05, 0.02, 0.2, 0.1, 0.1),
            limits=SafeSetLimits(terminal_velocity_tolerance_m_s=0.04),
        )
        context = LMPCContext(
            direction_sign=1,
            initial_speed_m_s=0.4,
            cross_speed_m_s=0.0,
            roll_rad=0.0,
            pitch_rad=0.0,
            roll_rate_rad_s=0.0,
            pitch_rate_rad_s=0.0,
            boundary_margin_m=2.0,
            battery_voltage_v=7.8,
            model_fingerprint=planner.model_fingerprint,
        )
        plan = planner.solve_from_safe_set(
            self.initial_state(), artifact, context
        )
        self.assertFalse(plan.optimizer_feasible)
        self.assertEqual(plan.reason, "safe_set_state_contract_mismatch")

    def test_validated_safe_set_query_connects_both_speed_brackets(self):
        planner = self.planner()
        initial = self.initial_state(available_distance=2.0)
        warm = self.baseline_commands()
        predicted, _ = planner.rollout(initial, warm)
        terminal = predicted[-1]
        delta = np.asarray([0.01, 0.001, 0.002, 0.0, 0.0])
        artifact = VelocityLMPCSafeSet(
            state_dimension=planner.state_dimension,
            command_dimension=1,
            state_scales=(0.05, 0.02, 0.2, 0.1, 0.1),
        )

        def add_episode(speed, state, episode_id):
            context = LMPCContext(
                direction_sign=1,
                initial_speed_m_s=speed,
                cross_speed_m_s=0.0,
                roll_rad=0.0,
                pitch_rad=0.0,
                roll_rate_rad_s=0.0,
                pitch_rate_rad_s=0.0,
                boundary_margin_m=2.0,
                battery_voltage_v=7.8,
                model_fingerprint=planner.model_fingerprint,
            )
            rows = (
                (np.asarray([speed, 0.0, 0.0, 0.0, 0.0]), speed, 0.00),
                (state, float(state[0]), 0.03),
                (np.asarray([0.04, 0.0, 0.0, 0.0, 0.0]), 0.04, 0.06),
                (np.asarray([0.02, 0.0, 0.0, 0.0, 0.0]), 0.02, 0.07),
                (np.asarray([0.01, 0.0, 0.0, 0.0, 0.0]), 0.01, 0.08),
            )
            samples = []
            for index, (vector, velocity, position) in enumerate(rows):
                samples.append(VelocityLMPCSample(
                    state=tuple(vector),
                    command=(0.0,),
                    dt_s=0.0 if index == len(rows)-1 else 0.04,
                    aligned_velocity_m_s=velocity,
                    cross_velocity_m_s=0.0,
                    position_m=(0.0, position, 1.0),
                    aligned_position_m=position,
                    roll_rad=0.0 if index != 1 else float(vector[1]),
                    pitch_rad=0.0,
                    roll_rate_rad_s=(
                        0.0 if index != 1 else float(vector[2])
                    ),
                    pitch_rate_rad_s=0.0,
                    boundary_margin_m=2.0,
                ))
            artifact.add_episode(VelocityLMPCEpisode.from_samples(
                episode_id=episode_id,
                context=context,
                outcome=TERMINAL_OUTCOME,
                terminal_handoff_position_m=(0.0, rows[-1][2], 1.0),
                samples=samples,
            ))

        add_episode(0.4, terminal+delta, "slow")
        add_episode(0.8, terminal-delta, "fast")
        query_context = LMPCContext(
            direction_sign=1,
            initial_speed_m_s=0.6,
            cross_speed_m_s=0.0,
            roll_rad=0.0,
            pitch_rad=0.0,
            roll_rate_rad_s=0.0,
            pitch_rate_rad_s=0.0,
            boundary_margin_m=2.0,
            battery_voltage_v=7.8,
            model_fingerprint=planner.model_fingerprint,
        )
        plan = planner.solve_from_safe_set(
            initial,
            artifact,
            query_context,
            neighbors_per_bracket=1,
            warm_start_commands_rad=warm,
        )
        self.assertTrue(plan.optimizer_feasible, plan)
        self.assertEqual(len(plan.terminal_weights), 2)
        self.assertAlmostEqual(plan.terminal_weights[0], 0.5, places=6)
        self.assertAlmostEqual(plan.terminal_weights[1], 0.5, places=6)


if __name__ == "__main__":
    unittest.main()
