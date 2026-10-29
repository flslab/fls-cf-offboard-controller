"""Conservation, feasibility, and plant checks for the offline collision demo."""

from dataclasses import replace
import json
import math
import unittest

from Interaction.virtual_collision_sim import (
    CollisionConfig,
    plan_collision,
    simulate_collision,
)


class VirtualCollisionReferenceTests(unittest.TestCase):
    def setUp(self):
        self.config = CollisionConfig()

    def test_equal_mass_elastic_collision_exchanges_velocity(self):
        config = self.config
        plan = plan_collision(config)
        start = plan.sample(0.0)
        end = plan.sample(plan.duration_s)
        self.assertAlmostEqual(start["v1_m_s"], config.velocity1_m_s)
        self.assertAlmostEqual(start["v2_m_s"], config.velocity2_m_s)
        self.assertAlmostEqual(end["v1_m_s"], config.velocity2_m_s)
        self.assertAlmostEqual(end["v2_m_s"], config.velocity1_m_s)
        self.assertAlmostEqual(start["x1_m"], -plan.trigger_distance_m / 2)
        self.assertAlmostEqual(start["x2_m"], plan.trigger_distance_m / 2)
        for key in ("a1_m_s2", "a2_m_s2", "j1_m_s3", "j2_m_s3"):
            self.assertAlmostEqual(start[key], 0.0)
            self.assertAlmostEqual(end[key], 0.0)

    def test_shared_curve_preserves_momentum_with_unequal_masses(self):
        config = replace(self.config, mass1_kg=0.1, mass2_kg=0.2,
                         restitution=0.8)
        plan = plan_collision(config)
        momentum = (config.mass1_kg * config.velocity1_m_s
                    + config.mass2_kg * config.velocity2_m_s)
        for index in range(501):
            sample = plan.sample(plan.duration_s * index / 500)
            actual = (config.mass1_kg * sample["v1_m_s"]
                      + config.mass2_kg * sample["v2_m_s"])
            self.assertAlmostEqual(actual, momentum, places=11)
            self.assertAlmostEqual(
                config.mass1_kg * sample["a1_m_s2"]
                + config.mass2_kg * sample["a2_m_s2"], 0.0, places=11)
        self.assertAlmostEqual(plan.target_velocity1_m_s, -0.84)
        self.assertAlmostEqual(plan.target_velocity2_m_s, 0.12)

    def test_restitution_impulse_and_energy_loss(self):
        for restitution in (0.0, 0.35, 0.8, 1.0):
            with self.subTest(restitution=restitution):
                config = replace(self.config, mass1_kg=0.12, mass2_kg=0.21,
                                 velocity1_m_s=0.7, velocity2_m_s=-0.4,
                                 restitution=restitution)
                plan = plan_collision(config)
                end = plan.sample(plan.duration_s)
                closing = config.velocity1_m_s - config.velocity2_m_s
                reduced_mass = 1 / (1 / config.mass1_kg + 1 / config.mass2_kg)
                impulse = (1 + restitution) * closing * reduced_mass
                self.assertAlmostEqual(
                    end["v2_m_s"] - end["v1_m_s"], restitution * closing)
                self.assertAlmostEqual(
                    config.mass1_kg * (end["v1_m_s"] - config.velocity1_m_s),
                    -impulse)
                self.assertAlmostEqual(
                    config.mass2_kg * (end["v2_m_s"] - config.velocity2_m_s),
                    impulse)
                before = 0.5 * (config.mass1_kg * config.velocity1_m_s ** 2
                                + config.mass2_kg * config.velocity2_m_s ** 2)
                after = 0.5 * (config.mass1_kg * end["v1_m_s"] ** 2
                               + config.mass2_kg * end["v2_m_s"] ** 2)
                expected_loss = 0.5 * reduced_mass * (1 - restitution ** 2) * closing ** 2
                self.assertAlmostEqual(before - after, expected_loss, places=11)

    def test_derivative_limits_apply_to_both_masses(self):
        for masses in ((0.15, 0.15), (0.07, 0.3), (0.3, 0.07)):
            with self.subTest(masses=masses):
                config = replace(self.config, mass1_kg=masses[0], mass2_kg=masses[1])
                plan = plan_collision(config)
                for index in range(1001):
                    sample = plan.sample(plan.duration_s * index / 1000)
                    for drone in (1, 2):
                        self.assertLessEqual(abs(sample[f"a{drone}_m_s2"]),
                                             config.acceleration_limit_m_s2 + 1e-10)
                        self.assertLessEqual(abs(sample[f"j{drone}_m_s3"]),
                                             config.jerk_limit_m_s3 + 1e-10)

    def test_positions_velocities_and_accelerations_are_consistent(self):
        plan = plan_collision(replace(self.config, mass1_kg=0.1, mass2_kg=0.2))
        h = 1e-6
        for fraction in (0.07, 0.23, 0.47, 0.71, 0.91):
            elapsed = fraction * plan.duration_s
            left, sample, right = (plan.sample(elapsed + offset)
                                   for offset in (-h, 0.0, h))
            for drone in (1, 2):
                numerical_v = (right[f"x{drone}_m"] - left[f"x{drone}_m"]) / (2 * h)
                numerical_a = (right[f"v{drone}_m_s"] - left[f"v{drone}_m_s"]) / (2 * h)
                numerical_j = (right[f"a{drone}_m_s2"] - left[f"a{drone}_m_s2"]) / (2 * h)
                self.assertAlmostEqual(numerical_v, sample[f"v{drone}_m_s"], places=7)
                self.assertAlmostEqual(numerical_a, sample[f"a{drone}_m_s2"], places=7)
                self.assertAlmostEqual(numerical_j, sample[f"j{drone}_m_s3"], places=6)

    def test_trigger_keeps_reference_outside_safe_separation(self):
        for restitution in (0.0, 0.5, 1.0):
            with self.subTest(restitution=restitution):
                config = replace(self.config, restitution=restitution)
                plan = plan_collision(config)
                minimum = min(
                    sample["x2_m"] - sample["x1_m"]
                    for sample in (plan.sample(plan.duration_s * i / 2000)
                                   for i in range(2001)))
                self.assertGreaterEqual(minimum + 1e-10,
                                        config.minimum_separation_m + config.separation_margin_m)
                self.assertLess(plan.trigger_distance_m, config.initial_separation_m)

    def test_completed_reference_cruises_at_outgoing_velocity(self):
        plan = plan_collision(self.config)
        end = plan.sample(plan.duration_s)
        later = plan.sample(plan.duration_s + 0.7)
        for drone in (1, 2):
            self.assertAlmostEqual(later[f"v{drone}_m_s"], end[f"v{drone}_m_s"])
            self.assertAlmostEqual(later[f"x{drone}_m"] - end[f"x{drone}_m"],
                                   0.7 * end[f"v{drone}_m_s"])
            self.assertAlmostEqual(later[f"a{drone}_m_s2"], 0.0)
            self.assertAlmostEqual(later[f"j{drone}_m_s3"], 0.0)

    def test_invalid_and_nonfinite_configurations_are_rejected(self):
        invalid = (
            {"mass1_kg": 0.0}, {"mass2_kg": -0.2},
            {"restitution": -0.01}, {"restitution": 1.01},
            {"velocity1_m_s": -0.6, "velocity2_m_s": -0.6},
            {"velocity1_m_s": -0.7, "velocity2_m_s": 0.6},
            {"acceleration_limit_m_s2": 0.0}, {"jerk_limit_m_s3": 0.0},
            {"initial_separation_m": 0.01},
            {"mass1_kg": math.nan}, {"velocity1_m_s": math.inf},
            {"delay1_s": math.nan}, {"response_tau2_s": math.inf},
            {"delay1_s": -0.01}, {"dt_s": 0.0},
        )
        for values in invalid:
            with self.subTest(values=values), self.assertRaises(ValueError):
                plan_collision(replace(self.config, **values))


class VirtualCollisionPlantTests(unittest.TestCase):
    def test_result_can_be_written_as_strict_json(self):
        result = simulate_collision(CollisionConfig())
        restored = json.loads(json.dumps(result, allow_nan=False))
        self.assertTrue(restored['summary']['checks']['separation'])

    def test_default_delayed_plant_completes(self):
        config = CollisionConfig()
        result = simulate_collision(config)
        summary = result["summary"]
        self.assertEqual(summary["status"], "passed")
        self.assertGreater(len(result["samples"]), 10)
        self.assertLess(summary["max_momentum_error_kg_m_s"], 1e-9)
        self.assertLess(summary["terminal_velocity_error_m_s"], 1e-5)
        self.assertGreaterEqual(summary["minimum_separation_m"], config.minimum_separation_m)
        self.assertGreater(summary["minimum_surface_gap_m"], 0.0)
        self.assertAlmostEqual(summary["energy_after_j"], summary["energy_before_j"], places=7)
        self.assertTrue(summary["simulation_only"])
        self.assertFalse(summary["flight_validated"])
        self.assertFalse(summary["firmware_sitl"])

    def test_asymmetric_response_reports_transient_momentum_error(self):
        config = replace(CollisionConfig(), delay1_s=0.01, delay2_s=0.13,
                         response_tau1_s=0.02, response_tau2_s=0.16,
                         trigger_delay_budget_s=0.4, settle_s=2.0)
        result = simulate_collision(config)
        summary = result["summary"]
        self.assertGreater(summary["max_momentum_error_kg_m_s"], 1e-3)
        self.assertFalse(summary["checks"]["transient_momentum"])
        self.assertTrue(summary["checks"]["terminal_velocity"])
        self.assertTrue(summary["checks"]["separation"])
        self.assertLess(summary["terminal_velocity_error_m_s"], 1e-4)
        self.assertLess(abs(summary["terminal_momentum_error_kg_m_s"]), 1e-5)
        self.assertEqual(summary["status"], "failed")

    def test_response_outside_trigger_budget_is_reported_as_failure(self):
        config = replace(CollisionConfig(), delay1_s=0.12, delay2_s=0.12,
                         response_tau1_s=0.1, response_tau2_s=0.1)
        summary = simulate_collision(config)["summary"]
        self.assertFalse(summary["checks"]["response_within_budget"])
        self.assertEqual(summary["status"], "failed")

    def test_profile_shorter_than_time_step_preserves_impulse_and_energy(self):
        for speed in (1e-4, 1e-6):
            with self.subTest(speed=speed):
                config = replace(CollisionConfig(), velocity1_m_s=speed,
                                 velocity2_m_s=-speed, dt_s=0.01,
                                 delay1_s=0.0, delay2_s=0.0,
                                 response_tau1_s=0.0, response_tau2_s=0.0,
                                 trigger_delay_budget_s=0.0)
                plan = plan_collision(config)
                self.assertLess(plan.duration_s, config.dt_s)
                # Start at the trigger, avoiding an intentionally long approach.
                config = replace(config, initial_separation_m=plan.trigger_distance_m)
                summary = simulate_collision(config)["summary"]
                self.assertEqual(summary["status"], "passed")
                self.assertAlmostEqual(summary["terminal_velocity1_m_s"], -speed,
                                       delta=speed * 1e-10)
                self.assertAlmostEqual(summary["terminal_velocity2_m_s"], speed,
                                       delta=speed * 1e-10)
                self.assertAlmostEqual(summary["energy_after_j"], summary["energy_before_j"],
                                       delta=summary["energy_before_j"] * 1e-10)

    def test_unbounded_simulation_allocations_are_rejected(self):
        configs = (
            replace(CollisionConfig(), dt_s=1e-12),
            replace(CollisionConfig(), velocity1_m_s=1e-8, velocity2_m_s=-1e-8),
        )
        for config in configs:
            with self.subTest(config=config), self.assertRaises(ValueError):
                simulate_collision(config)


if __name__ == "__main__":
    unittest.main()
