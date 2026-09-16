import inspect
import unittest

from Interaction.braking_swept_envelope import (
    FACE_NAMES,
    WorldTrajectorySample,
    certify_braking_swept_envelope,
)


def state(time_s, position, velocity=(0.0, 0.0, 0.0)):
    return WorldTrajectorySample(time_s, position, velocity)


class BrakingSweptEnvelopeTests(unittest.TestCase):
    def certificate(self, **changes):
        arguments = dict(
            bounds_m={
                "x": (-1.0, 1.0),
                "y": (-1.0, 1.0),
                "z": (0.0, 2.0),
            },
            current_position_m=(0.0, 0.0, 1.0),
            current_velocity_m_s=(0.0, 0.0, 0.0),
            position_uncertainty_m=(0.0, 0.0, 0.0),
            velocity_uncertainty_m_s=(0.0, 0.0, 0.0),
            vehicle_radius_m=0.05,
            boundary_reserve_m=0.05,
            transport_tail_samples=(),
            inner_loop_tail_samples=(),
            candidate_samples=(state(1.0, (0.0, 0.0, 1.0)),),
        )
        arguments.update(changes)
        return certify_braking_swept_envelope(**arguments)

    def test_safe_world_trajectory_reports_every_face_margin(self):
        result = self.certificate(
            bounds_m={axis: (-2.0, 2.0) for axis in ("x", "y", "z")},
            current_position_m=(0.0, 0.0, 0.0),
            current_velocity_m_s=(0.8, 0.0, 0.0),
            position_uncertainty_m=(0.05, 0.05, 0.05),
            velocity_uncertainty_m_s=(0.02, 0.02, 0.02),
            vehicle_radius_m=0.1,
            boundary_reserve_m=0.1,
            transport_tail_samples=(state(0.1, (0.08, 0.0, 0.0), (0.8, 0.0, 0.0)),),
            inner_loop_tail_samples=(state(0.2, (0.16, 0.0, 0.0), (0.8, 0.0, 0.0)),),
            candidate_samples=(
                state(0.5, (0.35, 0.0, 0.0), (0.4, 0.0, 0.0)),
                state(1.0, (0.45, 0.0, 0.0)),
            ),
        )

        self.assertTrue(result.feasible)
        self.assertEqual(result.reason, "certified")
        self.assertEqual(set(result.face_min_margin_m), set(FACE_NAMES))
        self.assertTrue(all(value > 0.0 for value in result.face_min_margin_m.values()))
        self.assertEqual(result.checked_knot_count, 5)
        self.assertEqual(result.checked_segment_count, 4)
        self.assertEqual(result.horizon_s, 1.0)
        self.assertTrue(result.shadow_only)
        self.assertFalse(result.command_authorized)

    def test_positive_and_negative_xy_faces_are_checked(self):
        cases = (
            ("x", 0, 0.95, "x_max"),
            ("x", 0, -0.95, "x_min"),
            ("y", 1, 0.95, "y_max"),
            ("y", 1, -0.95, "y_min"),
        )
        for _axis, index, displacement, face in cases:
            with self.subTest(face=face):
                position = [0.0, 0.0, 1.0]
                position[index] = displacement
                result = self.certificate(
                    candidate_samples=(state(0.5, position),)
                )
                self.assertFalse(result.feasible)
                self.assertEqual(result.reason, "boundary_violation")
                self.assertIn(face, result.violating_faces)
                self.assertAlmostEqual(result.face_min_margin_m[face], -0.05)

    def test_world_frame_check_has_no_yaw_input_or_body_rotation(self):
        signature = inspect.signature(certify_braking_swept_envelope)
        self.assertNotIn("yaw", signature.parameters)
        self.assertNotIn("attitude", signature.parameters)

        world_path = (state(0.5, (0.95, 0.0, 1.0)),)
        result_labelled_yaw_zero = self.certificate(candidate_samples=world_path)
        result_labelled_yaw_ninety = self.certificate(candidate_samples=world_path)
        self.assertEqual(
            dict(result_labelled_yaw_zero.face_min_margin_m),
            dict(result_labelled_yaw_ninety.face_min_margin_m),
        )
        self.assertIn("x_max", result_labelled_yaw_ninety.violating_faces)

    def test_diagonal_corner_checks_both_world_faces(self):
        result = self.certificate(
            vehicle_radius_m=0.10,
            candidate_samples=(state(0.5, (0.88, 0.88, 1.0)),),
        )

        self.assertFalse(result.feasible)
        self.assertAlmostEqual(result.face_min_margin_m["x_max"], -0.03)
        self.assertAlmostEqual(result.face_min_margin_m["y_max"], -0.03)
        self.assertIn("x_max", result.violating_faces)
        self.assertIn("y_max", result.violating_faces)

    def test_cubic_sweep_finds_reverse_overshoot_between_safe_endpoints(self):
        result = self.certificate(
            bounds_m={"x": (-0.8, 0.8), "y": (-1.0, 1.0), "z": (0.0, 2.0)},
            current_velocity_m_s=(4.0, 0.0, 0.0),
            vehicle_radius_m=0.0,
            boundary_reserve_m=0.0,
            candidate_samples=(state(1.0, (0.0, 0.0, 1.0), (-4.0, 0.0, 0.0)),),
        )

        self.assertFalse(result.feasible)
        self.assertAlmostEqual(result.face_min_margin_m["x_max"], -0.2)
        self.assertAlmostEqual(result.face_min_time_s["x_max"], 0.5)
        # An endpoint-only test would incorrectly accept both x == 0 knots.
        self.assertGreater(result.face_min_margin_m["x_min"], 0.0)

    def test_z_face_is_checked(self):
        result = self.certificate(
            candidate_samples=(state(0.5, (0.0, 0.0, 1.95)),)
        )

        self.assertFalse(result.feasible)
        self.assertIn("z_max", result.violating_faces)
        self.assertAlmostEqual(result.face_min_margin_m["z_max"], -0.05)

    def test_transport_and_inner_loop_tails_cannot_be_hidden_by_safe_candidate(self):
        for phase_name in ("transport_tail_samples", "inner_loop_tail_samples"):
            with self.subTest(phase=phase_name):
                changes = {
                    phase_name: (state(0.2, (0.95, 0.0, 1.0)),),
                    "candidate_samples": (state(0.5, (0.3, 0.0, 1.0)),),
                }
                result = self.certificate(**changes)
                self.assertFalse(result.feasible)
                self.assertIn("x_max", result.violating_faces)

    def test_uncertainty_erodes_the_bounds(self):
        path = (state(0.5, (0.75, 0.0, 1.0)),)
        nominal = self.certificate(candidate_samples=path)
        uncertain = self.certificate(
            candidate_samples=path,
            position_uncertainty_m=(0.16, 0.0, 0.0),
        )

        self.assertTrue(nominal.feasible)
        self.assertFalse(uncertain.feasible)
        self.assertAlmostEqual(uncertain.face_min_margin_m["x_max"], -0.01)

    def test_velocity_uncertainty_grows_over_the_delay_tail(self):
        result = self.certificate(
            current_position_m=(0.65, 0.0, 1.0),
            velocity_uncertainty_m_s=(0.4, 0.0, 0.0),
            transport_tail_samples=(state(1.0, (0.65, 0.0, 1.0)),),
            candidate_samples=(state(1.1, (0.65, 0.0, 1.0)),),
        )

        self.assertFalse(result.feasible)
        self.assertIn("x_max", result.violating_faces)
        self.assertAlmostEqual(result.face_min_margin_m["x_max"], -0.19)
        self.assertAlmostEqual(result.face_min_time_s["x_max"], 1.1)

    def test_invalid_bounds_fail_closed(self):
        invalid_bounds = (
            {"x": (-1.0, 1.0), "y": (-1.0, 1.0)},
            {"x": (-1.0, 1.0), "y": (-1.0, 1.0), "z": (2.0, 0.0)},
            {"x": (-1.0, float("nan")), "y": (-1.0, 1.0), "z": (0.0, 2.0)},
            {"x": (-1.0, 1.0), "y": (-1.0, 1.0), "z": (0.0, 2.0), "yaw": (-1, 1)},
            None,
        )
        for bounds in invalid_bounds:
            with self.subTest(bounds=bounds):
                result = self.certificate(bounds_m=bounds)
                self.assertFalse(result.feasible)
                self.assertEqual(result.reason, "invalid_input")
                self.assertTrue(all(
                    value is None for value in result.face_min_margin_m.values()
                ))
                self.assertFalse(result.command_authorized)

    def test_nan_negative_uncertainty_and_missing_trajectory_fail_closed(self):
        changes = (
            {"current_position_m": (float("nan"), 0.0, 1.0)},
            {"current_velocity_m_s": (0.0, float("inf"), 0.0)},
            {"position_uncertainty_m": (-0.01, 0.0, 0.0)},
            {"velocity_uncertainty_m_s": (0.0, float("nan"), 0.0)},
            {"vehicle_radius_m": -0.01},
            {"boundary_reserve_m": float("inf")},
            {"transport_tail_samples": None},
            {"inner_loop_tail_samples": None},
            {"candidate_samples": None},
            {"candidate_samples": ()},
        )
        for change in changes:
            with self.subTest(change=change):
                result = self.certificate(**change)
                self.assertFalse(result.feasible)
                self.assertEqual(result.reason, "invalid_input")

    def test_phase_order_and_same_time_discontinuity_fail_closed(self):
        cases = (
            dict(
                transport_tail_samples=(state(0.3, (0.1, 0.0, 1.0)),),
                inner_loop_tail_samples=(state(0.2, (0.2, 0.0, 1.0)),),
            ),
            dict(
                candidate_samples=(
                    state(0.5, (0.1, 0.0, 1.0)),
                    state(0.4, (0.2, 0.0, 1.0)),
                ),
            ),
            dict(
                transport_tail_samples=(state(0.0, (0.1, 0.0, 1.0)),),
            ),
        )
        for change in cases:
            with self.subTest(change=change):
                result = self.certificate(**change)
                self.assertFalse(result.feasible)
                self.assertEqual(result.reason, "invalid_input")


if __name__ == "__main__":
    unittest.main()
