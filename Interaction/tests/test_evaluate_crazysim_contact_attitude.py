import copy
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np

from Interaction.contact_attitude_observer import (
    CF_TIMESTAMP_MODULUS_MS,
    integrate_body_rate,
)
from Interaction.evaluate_crazysim_contact_attitude import (
    EXPECTED_CLOCK_BASIS,
    TRACE_SCHEMA,
    TraceValidationError,
    _alignment_seed,
    _run_estimator,
    evaluate_records,
    main,
    parse_records,
)
from Interaction.post_release_inertial_ekf import GRAVITY_M_S2, rotation_matrix


def _xyzw(quaternion_wxyz):
    w, x, y, z = quaternion_wxyz
    return [float(x), float(y), float(z), float(w)]


def make_physics_trace(post_release_ms=1200):
    """Return a deterministic, physically consistent, wrapping-clock trace."""
    start_ms = CF_TIMESTAMP_MODULUS_MS - 200
    release_offset_ms = 403
    contact_offset_ms = 303
    release_ms = start_ms + release_offset_ms
    records = [{
        "type": "meta",
        "schema": TRACE_SCHEMA,
        "position_only": True,
        "alignment_yaw_deg": 0.0,
        "sim_time_basis": EXPECTED_CLOCK_BASIS,
        "external_pose_mode": "position_only",
        "host_arrival_time_used": False,
        "odom_position_role": "position_only_estimator_input",
        "odom_velocity_role": (
            "plant_diagnostics_only_not_estimator_input"
        ),
        "odom_quaternion_role": "evaluation_truth_only",
        "command_history_is_estimator_input": False,
        "release_velocity_seed_policy": (
            "causal_position_finite_difference"
        ),
        "contact_start_sim_time_ns": (
            start_ms + contact_offset_ms
        ) * 1_000_000,
        "release_sim_time_ns": release_ms * 1_000_000,
        "simulator": "deterministic_test_fixture",
    }]
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])
    position = np.array([0.0, 0.0, 1.0])
    velocity = np.zeros(3)
    previous_rate = np.zeros(3)
    for offset in range(release_offset_ms + post_release_ms + 1):
        sim_ms = start_ms + offset
        sim_ns = sim_ms * 1_000_000
        specific_force = np.array([0.0, 0.0, GRAVITY_M_S2])
        if offset <= release_offset_ms:
            rate = np.zeros(3)
            if offset > contact_offset_ms:
                specific_force = np.array([1.0, 0.0, GRAVITY_M_S2])
                world_acceleration = (
                    rotation_matrix(quaternion) @ specific_force
                    + np.array([0.0, 0.0, -GRAVITY_M_S2])
                )
                dt = 0.001
                position += (
                    velocity * dt + 0.5 * world_acceleration * dt * dt
                )
                velocity += world_acceleration * dt
        else:
            post = offset - release_offset_ms
            rate = np.radians([
                25.0 if post <= 300 else 0.0,
                -12.0 if 150 < post <= 500 else 0.0,
                5.0 if post <= 400 else 0.0,
            ])
            specific_force = np.array([0.0, 0.0, GRAVITY_M_S2])
            world_acceleration = (
                rotation_matrix(quaternion) @ specific_force
                + np.array([0.0, 0.0, -GRAVITY_M_S2])
            )
            dt = 0.001
            position += velocity * dt + 0.5 * world_acceleration * dt * dt
            velocity += world_acceleration * dt
            quaternion = integrate_body_rate(
                quaternion, 0.5 * (previous_rate + rate), dt
            )
        if offset == contact_offset_ms:
            records.append({
                "type": "event", "name": "contact",
                "sim_time_ns": sim_ns,
            })
        if offset == release_offset_ms:
            records.append({
                "type": "event", "name": "release",
                "sim_time_ns": sim_ns,
            })
        records.append({
            "type": "imu", "sim_time_ns": sim_ns,
            "angular_velocity_rad_s": [float(item) for item in rate],
            "linear_acceleration_m_s2": [
                float(item) for item in specific_force
            ],
        })
        if offset % 5 == 0:
            records.append({
                "type": "odom", "sim_time_ns": sim_ns,
                "position_m": [float(item) for item in position],
                "velocity_m_s": [float(item) for item in velocity],
                "quaternion_xyzw": _xyzw(quaternion),
            })
        previous_rate = rate
    return records


class CrazySimContactAttitudeEvaluationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.records = make_physics_trace()
        cls.report = evaluate_records(
            cls.records, gyro_bias_stress_deg_s=(1.0, -0.6, 0.0)
        )

    def test_physics_trace_passes_fused_vs_gyro_only_gates(self):
        report = self.report
        self.assertEqual(report["gate_verdict"], "PASS", report["failed_gates"])
        self.assertEqual(report["failed_gates"], [])
        self.assertTrue(report["clock_mapping"]["release_cf_timestamp_ms"] < 300)
        self.assertGreaterEqual(report["metrics"]["position_update_count"], 100)
        self.assertEqual(report["metrics"]["release_seed_skew_ms"], 3.0)
        fused = report["metrics"]["fused_attitude"]
        baseline = report["metrics"]["gyro_only_attitude"]
        self.assertLess(fused["rmse_deg"], baseline["rmse_deg"])
        self.assertFalse(report["command_history_used"])
        self.assertEqual(report["post_release_odometry_update_fields"], ["position_m"])
        self.assertEqual(report["odom_quaternion_use"], "scoring_only")
        self.assertEqual(report["odom_velocity_use"],
                         "ignored_plant_diagnostics_only")
        self.assertFalse(
            report["release_state_seed"]["odom_velocity_field_used"]
        )
        self.assertEqual(
            report["release_state_seed"]["finite_difference_dt_ms"], 10.0
        )

    def test_truth_leakage_canary_hashes_mutated_and_removed_truth(self):
        canary = self.report["truth_leakage_canary"]
        self.assertTrue(canary["passed"])
        self.assertEqual(
            canary["nominal_estimator_output_sha256"],
            canary["mutated_truth_estimator_output_sha256"],
        )
        self.assertEqual(
            canary["nominal_estimator_output_sha256"],
            canary["removed_truth_estimator_output_sha256"],
        )

    def test_nominal_accuracy_does_not_require_artificial_ab_improvement(self):
        report = evaluate_records(self.records)
        self.assertEqual(report["gate_verdict"], "PASS", report["failed_gates"])
        self.assertFalse(report["ab_improvement_gate_applicable"])
        self.assertFalse(report["gates"]["ab_rmse_improvement_deg"]["applicable"])
        self.assertTrue(report["gates"]["ab_rmse_improvement_deg"]["passed"])

    def test_external_truth_changes_scores_but_not_estimator_output(self):
        parsed = parse_records(self.records)
        alignment = _alignment_seed(parsed, self.report_config())
        nominal = _run_estimator(parsed, alignment, (1.0, -0.6, 0.0))
        mutated = copy.deepcopy(self.records)
        for record in mutated:
            if record.get("type") == "odom":
                record["velocity_m_s"] = [999.0, -888.0, 777.0]
                record["quaternion_xyzw"] = [0.0, math.sqrt(0.5), 0.0,
                                               math.sqrt(0.5)]
        mutated_parsed = parse_records(mutated)
        changed = _run_estimator(
            mutated_parsed, alignment, (1.0, -0.6, 0.0)
        )
        self.assertEqual(
            nominal["estimator_output_sha256"],
            changed["estimator_output_sha256"],
        )
        self.assertNotEqual(
            nominal["fused_attitude"]["rmse_deg"],
            changed["fused_attitude"]["rmse_deg"],
        )

    @staticmethod
    def report_config():
        from Interaction.evaluate_crazysim_contact_attitude import EvaluationConfig
        return EvaluationConfig()

    def test_malformed_metadata_and_clock_fail_closed(self):
        bad_meta = copy.deepcopy(self.records)
        bad_meta[0]["position_only"] = False
        with self.assertRaisesRegex(TraceValidationError, "position_only"):
            parse_records(bad_meta)

        missing_provenance = copy.deepcopy(self.records)
        del missing_provenance[0]["host_arrival_time_used"]
        with self.assertRaisesRegex(TraceValidationError, "host_arrival"):
            parse_records(missing_provenance)

        mismatched_event = copy.deepcopy(self.records)
        mismatched_event[0]["contact_start_sim_time_ns"] += 1_000_000
        with self.assertRaisesRegex(TraceValidationError, "does not match"):
            parse_records(mismatched_event)

        bad_clock = copy.deepcopy(self.records)
        imu = next(record for record in bad_clock if record.get("type") == "imu")
        imu["sim_time_ns"] += 1
        with self.assertRaisesRegex(TraceValidationError, "millisecond epoch"):
            parse_records(bad_clock)

        bad_truth = copy.deepcopy(self.records)
        odom = next(record for record in bad_truth if record.get("type") == "odom")
        odom["quaternion_xyzw"] = [0.0, 0.0, 0.0, 0.0]
        with self.assertRaisesRegex(TraceValidationError, "non-zero"):
            parse_records(bad_truth)

    def test_gap_and_missing_common_epoch_are_reported_as_gate_failures(self):
        damaged = []
        parsed = parse_records(self.records)
        gap_start = parsed.release_time_ns + 500_000_000
        for record in self.records:
            if record.get("type") == "imu" and (
                    gap_start <= record["sim_time_ns"] < gap_start + 7_000_000):
                continue
            damaged.append(record)
        report = evaluate_records(
            damaged, gyro_bias_stress_deg_s=(1.0, -0.6, 0.0)
        )
        self.assertEqual(report["gate_verdict"], "FAIL")
        self.assertIn("maximum_imu_gap_ms", report["failed_gates"])
        self.assertIn("common_sim_clock", report["failed_gates"])
        self.assertIn("estimator_valid", report["failed_gates"])

    def test_cli_emits_json_and_refuses_to_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            trace = directory / "trace.jsonl"
            output = directory / "report.json"
            trace.write_text("".join(
                json.dumps(record, separators=(",", ":")) + "\n"
                for record in self.records
            ))
            stdout = io.StringIO()
            with mock.patch("sys.stdout", stdout):
                result = main([
                    str(trace), "--gyro-bias-stress-deg-s", "1", "-0.6", "0",
                    "--output", str(output),
                ])
            self.assertEqual(result, 0)
            emitted = json.loads(stdout.getvalue())
            written = json.loads(output.read_text())
            self.assertEqual(emitted, written)
            before = output.read_bytes()
            with mock.patch("sys.stderr", io.StringIO()):
                with self.assertRaises(SystemExit):
                    main([str(trace), "--output", str(output)])
            self.assertEqual(output.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
