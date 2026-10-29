"""Configuration propagation checks; importing this module starts no simulator."""

from dataclasses import replace
import math
import unittest

from Interaction.simulate_virtual_collision_gazebo import (
    CollisionExperimentOptions,
    cad_surface_clearance_checks,
    make_cad_safety_geometry,
    select_trigger_distance,
)
from Interaction.virtual_collision_sim import CollisionConfig, plan_collision


class GazeboSurfaceGapTests(unittest.TestCase):
    def setUp(self):
        self.radius = 0.11462632707959478
        self.options = CollisionExperimentOptions(
            geometry_close_pass=True, trigger_reference="prior-accepted-run")
        self.bounds = [{"bounding_sphere_radius_m": self.radius}] * 2

    def test_omitted_gap_keeps_the_previous_cad_scenario(self):
        self.options.validate()
        geometry = make_cad_safety_geometry(self.options, self.bounds)
        self.assertEqual(self.options.resolved_minimum_surface_gap_m, .03)
        self.assertEqual(geometry["minimum_surface_margin_m"], .03)
        self.assertAlmostEqual(geometry["minimum_center_separation_m"],
                               .25925265415918956)

    def test_configured_margin_reaches_both_floor_and_trigger(self):
        empirical = {"incoming_relative_velocity_m_s": 1.2,
                     "measured_projected_closure_m": .3}
        distances = []
        for margin in (.01, .03, .08, .12):
            with self.subTest(margin=margin):
                options = replace(self.options, minimum_surface_gap_m=margin)
                options.validate()
                geometry = make_cad_safety_geometry(options, self.bounds)
                self.assertAlmostEqual(geometry["minimum_center_separation_m"],
                                       2 * self.radius + margin)
                plan = plan_collision(replace(CollisionConfig(),
                    minimum_separation_m=geometry["minimum_center_separation_m"],
                    drone_radius_m=self.radius))
                selected, _ = select_trigger_distance(plan, empirical, .04)
                distances.append(selected - margin)
        for value in distances[1:]:
            self.assertAlmostEqual(value, distances[0])

    def test_saved_data_checks_enforce_requested_margin_not_old_default(self):
        checks = cad_surface_clearance_checks(.06, .055, .03)
        self.assertTrue(all(checks.values()))
        checks = cad_surface_clearance_checks(.06, .055, .08)
        self.assertFalse(any(checks.values()))
        checks = cad_surface_clearance_checks(.06, .055, .058)
        self.assertTrue(checks["cad_surface_clearance"])
        self.assertFalse(checks["cad_clearance_with_allowance"])

    def test_gap_requires_cad_mode_and_finite_positive_input(self):
        for value in (0., -.01, .50001, math.nan, math.inf, -math.inf):
            with self.subTest(value=value), self.assertRaises(ValueError):
                replace(self.options, minimum_surface_gap_m=value).validate()
        with self.assertRaises(ValueError):
            CollisionExperimentOptions(minimum_surface_gap_m=.12).validate()
        # Normal mode is still usable with no gap option.
        CollisionExperimentOptions().validate()
        replace(self.options, minimum_surface_gap_m=.50).validate()


if __name__ == "__main__":
    unittest.main()
