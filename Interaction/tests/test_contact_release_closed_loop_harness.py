import dataclasses
import inspect
import unittest

from Interaction.contact_release_closed_loop_harness import (
    ClosedLoopFaults,
    ClosedLoopHarnessConfig,
    run_contact_release_closed_loop_harness,
)
from Interaction.post_release_inertial_ekf import PostReleaseInertialEkf


class ContactReleaseClosedLoopHarnessTests(unittest.TestCase):
    def test_nominal_chain_passes_every_hard_gate(self):
        result = run_contact_release_closed_loop_harness()

        self.assertTrue(result.passed)
        self.assertTrue(result.contact_started)
        self.assertTrue(result.release_detected)
        self.assertEqual(result.detector_source, "potentiometer")
        self.assertEqual(result.release_boundary_sample_id, 220)
        self.assertEqual(result.release_confirmation_sample_id, 260)
        self.assertAlmostEqual(result.release_boundary_time_s, 0.220)
        self.assertAlmostEqual(result.release_confirmation_time_s, 0.260)
        self.assertLess(
            result.release_boundary_time_s,
            result.release_confirmation_time_s,
        )
        self.assertEqual(result.braking_start_time_s, result.release_confirmation_time_s)
        self.assertEqual(result.candidate_boundary_sample_ids, (220,))
        preview = next(
            sample for sample in result.trace
            if result.release_boundary_time_s <= sample.time_s
            < result.release_confirmation_time_s
        )
        self.assertTrue(preview.release_candidate_active)
        self.assertFalse(preview.release_confirmed)
        self.assertIsNotNone(preview.estimated_velocity_m_s)
        self.assertEqual(preview.commanded_acceleration_m_s2, 0.0)
        self.assertGreater(result.command_count, 100)
        self.assertGreater(result.position_update_count, 100)
        self.assertTrue(all(gate.passed for gate in result.gates.values()))
        self.assertEqual(
            result.estimator_process_inputs,
            ("gyro", "accelerometer_specific_force", "position_only"),
        )
        self.assertTrue(result.simulation_only)
        self.assertFalse(result.authoritative)
        self.assertFalse(result.live_crazysim_coupled)
        self.assertFalse(result.contact_attitude_seed_validated)
        self.assertIn("plant_truth", result.estimator_seed_source)
        self.assertIn("open_loop", result.profile_feedback_scope)
        self.assertEqual(
            result.swept_envelope_scope,
            "post_hoc_realized_trajectory_validation_only",
        )
        self.assertIn("CrazySim/SITL", result.remaining_live_gate)

    def test_rebounded_release_candidate_is_discarded_before_second_boundary(self):
        result = run_contact_release_closed_loop_harness(faults=ClosedLoopFaults(
            release_candidate_rebound=True,
        ))

        self.assertTrue(result.passed)
        self.assertEqual(result.release_candidate_started_count, 2)
        self.assertEqual(result.release_candidate_cancelled_count, 1)
        self.assertEqual(result.candidate_boundary_sample_ids, (220, 260))
        self.assertEqual(result.release_boundary_sample_id, 260)
        self.assertEqual(result.release_confirmation_sample_id, 300)
        self.assertAlmostEqual(result.braking_start_time_s, 0.300)

    def test_run_is_bitwise_deterministic(self):
        first = run_contact_release_closed_loop_harness()
        second = run_contact_release_closed_loop_harness()

        self.assertEqual(first, second)

    def test_nonzero_imu_bias_is_fused_with_position_only_updates(self):
        result = run_contact_release_closed_loop_harness(faults=ClosedLoopFaults(
            gyro_bias_deg_s=(0.0, 0.6, 0.0),
            accel_bias_g=(0.008, 0.0, 0.0),
        ))

        self.assertTrue(result.passed)
        self.assertTrue(result.gates["estimator_valid"].passed)
        final = result.trace[-1]
        self.assertAlmostEqual(
            final.estimated_position_m[0], final.true_position_m[0], delta=0.003
        )
        self.assertAlmostEqual(
            final.estimated_velocity_m_s[0], final.true_velocity_m_s[0],
            delta=0.006,
        )
        parameters = inspect.signature(PostReleaseInertialEkf.propagate).parameters
        self.assertNotIn("command", parameters)
        self.assertNotIn("setpoint", parameters)
        self.assertNotIn("command_history", parameters)

    def test_imu_gap_invalidates_ekf_and_latches_authority_off(self):
        result = run_contact_release_closed_loop_harness(faults=ClosedLoopFaults(
            imu_gap_start_s=0.12,
            imu_gap_duration_s=0.012,
        ))

        self.assertFalse(result.passed)
        self.assertFalse(result.gates["estimator_valid"].passed)
        self.assertEqual(result.gates["estimator_valid"].reason, "imu_gap_exceeded")
        self.assertTrue(any(sample.authority_latched_off for sample in result.trace))

    def test_imu_outlier_fails_range_gate_before_ekf_fusion(self):
        result = run_contact_release_closed_loop_harness(faults=ClosedLoopFaults(
            imu_outlier_time_s=0.15,
            imu_outlier_g=(3.0, 0.0, 0.0),
        ))

        self.assertFalse(result.passed)
        self.assertFalse(result.gates["imu_quality"].passed)
        self.assertIn("latched off", result.gates["imu_quality"].reason)
        self.assertTrue(result.gates["estimator_valid"].passed)

    def test_position_delay_drop_and_reorder_are_contained(self):
        result = run_contact_release_closed_loop_harness(faults=ClosedLoopFaults(
            position_delay_s=0.001,
            position_drop_every=3,
            position_reorder_index=4,
            position_reorder_extra_delay_s=0.030,
        ))

        self.assertTrue(result.passed)
        self.assertGreater(result.position_drop_count, 0)
        self.assertEqual(result.position_reorder_rejection_count, 1)
        self.assertEqual(result.position_stale_rejection_count, 0)
        self.assertTrue(result.gates["position_stream"].passed)

    def test_excessive_position_delay_fails_closed(self):
        result = run_contact_release_closed_loop_harness(faults=ClosedLoopFaults(
            position_delay_s=0.080,
        ))

        self.assertFalse(result.passed)
        self.assertFalse(result.gates["position_stream"].passed)
        self.assertGreater(result.position_stale_rejection_count, 0)
        self.assertTrue(any(sample.authority_latched_off for sample in result.trace))

    def test_host_send_stall_exceeds_maximum_hold(self):
        result = run_contact_release_closed_loop_harness(faults=ClosedLoopFaults(
            host_stall_start_s=0.12,
            host_stall_duration_s=0.080,
        ))

        gate = result.gates["command_hold"]
        self.assertFalse(result.passed)
        self.assertFalse(gate.passed)
        self.assertGreater(gate.observed, gate.limit)
        self.assertGreater(result.skipped_command_count, 0)

    def test_swept_xyz_boundary_checks_vehicle_and_uncertainty(self):
        config = dataclasses.replace(
            ClosedLoopHarnessConfig(),
            bounds_m={
                "x": (-1.0, 0.50), "y": (-1.0, 1.0), "z": (0.0, 2.0),
            },
        )
        result = run_contact_release_closed_loop_harness(config=config)

        gate = result.gates["realized_swept_xyz_boundary_posthoc"]
        self.assertFalse(result.passed)
        self.assertFalse(gate.passed)
        self.assertLess(gate.observed, 0.0)
        self.assertEqual(gate.reason, "boundary_violation")

    def test_tilt_rate_and_jerk_are_independent_hard_gates(self):
        config = dataclasses.replace(
            ClosedLoopHarnessConfig(),
            maximum_tilt_deg=4.0,
            maximum_tilt_rate_deg_s=10.0,
            maximum_command_jerk_m_s3=3.0,
        )
        result = run_contact_release_closed_loop_harness(config=config)

        self.assertFalse(result.gates["tilt"].passed)
        self.assertFalse(result.gates["tilt_rate"].passed)
        self.assertFalse(result.gates["command_jerk"].passed)
        for name in ("tilt", "tilt_rate", "command_jerk"):
            self.assertGreater(result.gates[name].observed, result.gates[name].limit)

    def test_unmodelled_acceleration_exposes_reverse_motion(self):
        result = run_contact_release_closed_loop_harness(faults=ClosedLoopFaults(
            plant_acceleration_disturbance_m_s2=-0.05,
        ))

        gate = result.gates["no_reverse"]
        self.assertFalse(result.passed)
        self.assertFalse(gate.passed)
        self.assertGreater(gate.observed, gate.limit)

    def test_fault_and_timing_inputs_fail_closed_at_construction(self):
        with self.assertRaises(ValueError):
            ClosedLoopFaults(position_drop_every=-1)
        with self.assertRaises(ValueError):
            ClosedLoopFaults(accel_bias_g=(0.0, 0.0))
        with self.assertRaises(ValueError):
            ClosedLoopFaults(position_drop_every=1.5)
        with self.assertRaises(ValueError):
            ClosedLoopFaults(host_stall_start_s=-0.1)
        with self.assertRaises(ValueError):
            ClosedLoopFaults(release_candidate_rebound=1)
        with self.assertRaises(ValueError):
            ClosedLoopHarnessConfig(command_period_s=0.0)
        with self.assertRaises(ValueError):
            ClosedLoopHarnessConfig(bounds_m={"x": (-1.0, 1.0)})
        with self.assertRaises(ValueError):
            ClosedLoopHarnessConfig(transport_delay_s=float("nan"))
        with self.assertRaises(ValueError):
            ClosedLoopHarnessConfig(
                bounds_m={
                    "x": (1.0, -1.0), "y": (-1.0, 1.0), "z": (0.0, 2.0),
                }
            )


if __name__ == "__main__":
    unittest.main()
