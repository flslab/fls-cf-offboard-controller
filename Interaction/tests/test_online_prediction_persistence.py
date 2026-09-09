"""Prediction evidence is optional, conservative, and never control settings."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

from Interaction.online_dynamics_model import (
    PARAMETER_BOUNDS, evaluate_predictive_model, fit_predictive_model,
)
from Interaction.online_prediction_calibration import _CalibrationAccumulator
from Interaction.tests.test_online_dynamics_model import synthetic_samples
from Interaction.wrench_model_calibration import (
    _PREDICTION_PARAMETER_BOUNDS, save_drone_calibration,
)


LEGACY_FIT = dict(model_delay_s=[.01, .02, .03],
                  model_time_constant_s=[.05, .06, .07],
                  model_acceleration_scale=[1., 1.1, 1.2])


class OnlinePredictionPersistenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Exercise the real producer schema and independent synthetic plant,
        # rather than accepting a report fixture invented by persistence code.
        with tempfile.TemporaryDirectory() as directory:
            accumulator = _CalibrationAccumulator({}, Path(directory)/"online.json", "lb11", list(range(6)),
                fit_predictive_model, evaluate_predictive_model)
            try:
                for segment in range(6):
                    accumulator.receive(synthetic_samples((segment,)))
                accumulator.finish(dict(accepted_submissions=6, rejected_submissions=0))
                cls.report = copy.deepcopy(accumulator.report)
            finally:
                accumulator.close()
        assert cls.report["validation_passed"]

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name)/"wrench_calibration.json"
        self.previous = {
            "prediction_model": {"old_model": "preserve exactly", "runtime_enabled": False},
            "control_handoff": {"coast_attitude_response_delay_s": .12, "owner": "existing"},
            "position_capture_fit": {"previous_capture": True},
            "custom_field": {"user_owned": [1, 2, 3]},
            "online_prediction_attempt": {"old_attempt": "preserve when omitted"},
        }
        self.path.write_text(json.dumps({"schema_version": 1, "drones": {
            "lb11": self.previous, "other_drone": {"untouched": True}}}))

    def save(self, report=None):
        return save_drone_calibration("lb11", LEGACY_FIT, {"rotor_count": 4}, self.path,
                                      online_prediction_report=report)[1]

    def assert_preserved(self, entry):
        self.assertEqual(entry["prediction_model"], self.previous["prediction_model"])
        self.assertFalse(entry["online_prediction_attempt"]["prediction_model_updated"])
        self.assertEqual(entry["fit"], LEGACY_FIT)
        self.assertEqual(entry["control_handoff"], self.previous["control_handoff"])
        self.assertEqual(entry["position_capture_fit"], self.previous["position_capture_fit"])
        self.assertEqual(entry["custom_field"], self.previous["custom_field"])
        json.dumps(entry, allow_nan=False)

    def test_lightweight_persistence_bounds_match_numerical_core(self):
        self.assertEqual(_PREDICTION_PARAMETER_BOUNDS, PARAMETER_BOUNDS)

    def test_old_caller_preserves_model_attempt_and_unrelated_fields(self):
        entry = self.save()
        for key, value in self.previous.items():
            self.assertEqual(entry[key], value)
        document = json.loads(self.path.read_text())
        self.assertEqual(document["drones"]["other_drone"], {"untouched": True})

    def test_real_completed_report_saves_only_frozen_independently_validated_model(self):
        report = copy.deepcopy(self.report)
        before = json.dumps(report, sort_keys=True, allow_nan=False)
        entry = self.save(report)
        self.assertEqual(before, json.dumps(report, sort_keys=True, allow_nan=False))
        selected = entry["prediction_model"]
        self.assertEqual(selected["train_segment_ids"], [0, 1, 2, 3])
        self.assertEqual(selected["validation"]["validation_segment_ids"], [4, 5])
        self.assertEqual(report["candidate"]["model"]["train_segment_ids"], [0, 1, 2, 3, 4, 5])
        self.assertEqual(selected["attitude_fit"], report["validated_candidate"]["model"]["attitude_fit"])
        self.assertTrue(selected["independent_validation_complete"])
        self.assertFalse(selected["runtime_enabled"])
        self.assertFalse(selected["deployment_approved"])
        self.assertTrue(entry["online_prediction_attempt"]["prediction_model_updated"])
        self.assertNotIn("rolling_horizons", selected["validation"])
        self.assertNotIn("model", entry["online_prediction_attempt"])
        self.assertNotIn("metrics", entry["online_prediction_attempt"])
        self.assertLess(len(json.dumps(entry["online_prediction_attempt"])), 2000)
        self.assertEqual(entry["control_handoff"], self.previous["control_handoff"])
        self.assertEqual(entry["position_capture_fit"], self.previous["position_capture_fit"])
        self.assertEqual(entry["impulse_estimator"], LEGACY_FIT)
        self.assertEqual(entry["custom_field"], self.previous["custom_field"])

    def test_incomplete_failed_and_nonreport_values_do_not_abort_legacy_save(self):
        for change in ({"status": "partial"}, {"status": "failed"}, {"data_complete": False},
                       {"validation_passed": False}, {"validated_candidate": None},
                       {"worker_error": "worker crashed"}, {"drone_id": "another-drone"},
                       {"kind": "unrelated"}, {"schema_version": True},
                       {"received_segment_ids": [0, 1, 2, 3]},
                       {"rejected_trials": [{"segment_id": 0}]}):
            report = copy.deepcopy(self.report)
            report.update(change)
            with self.subTest(change=change):
                self.assert_preserved(self.save(report))
        for report in ([], "failed", 123, {"status": object()}):
            with self.subTest(report=report):
                self.assert_preserved(self.save(report))

    def test_latest_pair_and_full_training_prefix_are_required(self):
        for which in ("old_pair", "overlap", "duplicate", "all_data_candidate", "model_provenance", "boolean_id"):
            report = copy.deepcopy(self.report)
            validated = report["validated_candidate"]
            if which == "old_pair":
                report["validated_candidate"] = copy.deepcopy(report["validation_history"][0])
            elif which == "overlap":
                validated["training_segment_ids"].append(4)
            elif which == "duplicate":
                report["expected_segment_ids"][1] = 0
                report["received_segment_ids"][1] = 0
            elif which == "all_data_candidate":
                validated["model"] = copy.deepcopy(report["candidate"]["model"])
            elif which == "boolean_id":
                validated["model"]["train_segment_ids"][0] = False
            else:
                validated["model"]["attitude_fit"]["training_segments"] = [2, 3]
            with self.subTest(which=which):
                self.assert_preserved(self.save(report))

    def test_unidentifiable_active_bounds_and_out_of_bounds_are_rejected(self):
        for which in ("unidentifiable", "bound_flag", "active_mask", "bound_value", "outside",
                      "nan", "infinite_gain", "weak_singular_values", "bad_condition",
                      "directional_margin", "directional_sign",
                      "directional_unidentifiable"):
            report = copy.deepcopy(self.report)
            model = report["validated_candidate"]["model"]
            if which == "unidentifiable":
                model["identifiability"]["identifiable"] = False
            elif which == "bound_flag":
                model["identifiability"]["bound_active_parameters"] = ["delay_s"]
            elif which == "active_mask":
                model["attitude_fit"]["active_bounds"] = [1, 0, 0, 0, 0]
            elif which == "bound_value":
                model["attitude_fit"]["delay_s"] = .15
            elif which == "outside":
                model["attitude_fit"]["zeta"] = -1.
            elif which == "nan":
                model["attitude_fit"]["gain"] = float("nan")
            elif which == "infinite_gain":
                model["motion_gain"] = float("inf")
            elif which == "weak_singular_values":
                model["identifiability"]["singular_values"][-1] = 1e-8
            elif which == "bad_condition":
                model["identifiability"]["condition_number"] = 1e10
            elif which == "directional_margin":
                model["directional_models"]["negative_y"][
                    "terminal_velocity_error_margin_m_s"
                ] = float("nan")
            elif which == "directional_sign":
                model["directional_models"]["negative_y"]["direction_y"] = 1
            else:
                model["directional_models"]["positive_y"][
                    "identifiability"
                ]["identifiable"] = False
            with self.subTest(which=which):
                self.assert_preserved(self.save(report))

    def test_failed_metric_cannot_be_hidden_by_true_gate_booleans(self):
        for field in ("velocity_rmse_m_s", "terminal_error_m_s", "end_position_error_m"):
            for value in (.07, -.07, float("nan"), float("inf"), None, True):
                report = copy.deepcopy(self.report)
                report["validated_candidate"]["metrics"]["per_trial"][0][field] = value
                with self.subTest(field=field, value=value):
                    self.assert_preserved(self.save(report))

    def test_invalid_declared_limits_cannot_bypass_quality_gates(self):
        for limit in (0, -.01, float("nan"), float("inf"), 1000., True, "0.06"):
            report = copy.deepcopy(self.report)
            report["validated_candidate"]["limits"]["velocity_rmse_m_s"] = limit
            with self.subTest(limit=limit):
                self.assert_preserved(self.save(report))

    def test_missing_opposed_directions_wrong_ids_or_reversal_mismatch_rejected(self):
        for which in ("train_direction", "held_direction", "held_id", "reversal", "nonbool_reversal",
                      "future_measurement", "updated_parameters", "nonfinite_nested"):
            report = copy.deepcopy(self.report)
            validated = report["validated_candidate"]
            if which == "train_direction":
                for row in validated["model"]["data_ranges"]:
                    row["direction_y"] = 1
            elif which == "held_direction":
                for row in validated["metrics"]["per_trial"]:
                    row["direction_y"] = 1
            elif which == "held_id":
                validated["metrics"]["per_trial"][0]["segment_id"] = 99
            elif which == "reversal":
                row = validated["metrics"]["per_trial"][0]
                row["predicted_reverse"] = not row["actual_reverse"]
            elif which == "nonbool_reversal":
                row = validated["metrics"]["per_trial"][0]
                row["predicted_reverse"] = int(row["actual_reverse"])
            elif which == "future_measurement":
                validated["metrics"]["future_measurements_used_for_prediction"] = True
            elif which == "updated_parameters":
                validated["metrics"]["model_parameters_updated"] = True
            else:
                validated["metrics"]["aggregates"]["velocity_rmse_m_s"] = float("nan")
            with self.subTest(which=which):
                self.assert_preserved(self.save(report))

    def test_previous_valid_prediction_survives_later_failed_attempt(self):
        accepted = self.save(copy.deepcopy(self.report))
        previous_model = copy.deepcopy(accepted["prediction_model"])
        bad = copy.deepcopy(self.report)
        bad["validation_passed"] = False
        saved = self.save(bad)
        self.assertEqual(saved["prediction_model"], previous_model)
        self.assertFalse(saved["online_prediction_attempt"]["prediction_model_updated"])
        self.assertFalse(saved["online_prediction_attempt"]["reported_validation_passed"])

    def test_complete_failed_validation_is_saved_but_marked_ineligible(self):
        report = copy.deepcopy(self.report)
        validated = report["validated_candidate"]
        report["validation_passed"] = False
        validated["validation_passed"] = False
        validated["gates"]["terminal_error_m_s"] = False
        row = validated["metrics"]["per_trial"][0]
        row["terminal_error_m_s"] = .20
        label = "positive_y" if row["direction_y"] > 0 else "negative_y"
        validated["model"]["directional_models"][label][
            "terminal_velocity_error_margin_m_s"
        ] = .20

        saved = self.save(report)
        selected = saved["prediction_model"]
        self.assertFalse(selected["control_eligible"])
        self.assertFalse(selected["validation_passed"])
        self.assertFalse(selected["validation"]["control_eligible"])
        self.assertEqual(
            selected["validation"]["failed_gates"],
            ["terminal_error_m_s"],
        )
        self.assertEqual(
            selected["candidate_status"],
            "independently_validated_failed_not_control_eligible",
        )
        self.assertTrue(saved["online_prediction_attempt"]["prediction_model_updated"])
        self.assertEqual(
            saved["online_prediction_attempt"]["selection_reason"],
            "saved_latest_frozen_candidate_control_ineligible",
        )

    def test_bad_attempt_without_previous_model_does_not_create_one(self):
        self.path.unlink()
        entry = self.save({"status": "failed"})
        self.assertNotIn("prediction_model", entry)
        self.assertNotIn("control_handoff", entry)
        self.assertEqual(entry["fit"], LEGACY_FIT)


if __name__ == "__main__":
    unittest.main()
