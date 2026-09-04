"""Offline replay CLI contracts; all session interaction is mocked, no hardware."""

import copy
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

from Interaction.fit_online_prediction import main, samples_from_log, summary_markdown


MODULE = "Interaction.fit_online_prediction"


def _observer(timestamp, scale=1.):
    return {"type": "wrench_observer", "data": {
        "state_time": timestamp,
        "orientation_rpy_rad": [.11 * scale, -.22 * scale, .33 * scale],
        "angular_velocity_rad_s": [.41 * scale, -.52 * scale, .63 * scale],
        "position_m": [.71 * scale, -.82 * scale, 1.03 * scale],
        "velocity_m_s": [.14 * scale, -.25 * scale, .36 * scale],
        "battery_voltage_V": 3.94 * scale, "state_group_skew_s": .007 * scale,
    }}


def _report():
    return {
        "schema_version": 1, "kind": "online_prediction_calibration",
        "status": "completed", "data_complete": True, "validation_passed": True,
        "runtime_enabled": False, "flight_approved": False,
        "candidate": {"training_segment_ids": list(range(6)),
                      "independent_validation": False,
                      "model": {"attitude_fit": {"gain": 9.999}, "motion_gain": 8.888}},
        "validation_history": [
            {"candidate_version": 1, "training_segment_ids": [0, 1],
             "validation_segment_ids": [2, 3], "validation_passed": False},
            {"candidate_version": 2, "training_segment_ids": [0, 1, 2, 3],
             "validation_segment_ids": [4, 5], "validation_passed": True},
        ],
        "validated_candidate": {
            "candidate_version": 2, "training_segment_ids": [0, 1, 2, 3],
            "validation_segment_ids": [4, 5], "validation_passed": True,
            "model": {"attitude_fit": {"gain": 1.07}, "motion_gain": 1.13,
                      "identifiability": {"identifiable": True}},
            "metrics": {"per_trial": [
                {"segment_id": 4, "theta_rmse_deg": .1234,
                 "velocity_rmse_m_s": .0123, "terminal_error_m_s": -.0234,
                 "end_position_error_m": .0345, "actual_reverse": False,
                 "predicted_reverse": False},
                {"segment_id": 5, "theta_rmse_deg": .2345,
                 "velocity_rmse_m_s": .0234, "terminal_error_m_s": .0345,
                 "end_position_error_m": -.0456, "actual_reverse": True,
                 "predicted_reverse": True},
            ]},
        },
    }


class ReplaySampleAugmentationTests(unittest.TestCase):
    def test_actual_vectors_rates_battery_and_first_duplicate_are_preserved(self):
        first = _observer(100.)
        second = _observer(101., scale=.5)
        records = [first, _observer(100., scale=9.),
                   {"type": "command", "data": {"state_time": 100.}}, second]
        original_records = copy.deepcopy(records)
        extracted = [
            {"segment_id": 7, "timestamp": 100., "phase": "accelerate",
             "direction_xy": [0., -1.], "command_started_at": 99.995,
             "command_acceleration_xy": [0., -3.],
             "position_xy": [0., .82], "velocity_xy": [0., .25]},
            {"segment_id": 8, "timestamp": 101., "phase": "brake",
             "direction_xy": [0., 1.], "command_started_at": 100.992,
             "command_acceleration_xy": [0., -2.]},
        ]
        metadata = {"command_time_method": "approximate reconstructed send anchors",
                    "clock_offset_s": .012, "duplicate_state_count": 1}
        with mock.patch(MODULE + ".extract", return_value=(
                {"legacy": "not used"}, [SimpleNamespace(segment=7), SimpleNamespace(segment=8)],
                extracted, metadata)) as extractor:
            samples, segments, received_metadata = samples_from_log(records, max_sample_gap_s=.045)
        extractor.assert_called_once_with(records, max_gap_s=.045)
        self.assertEqual(segments, [7, 8])
        self.assertEqual(received_metadata, metadata)
        for sample, record in zip(samples, (first, second)):
            row = record["data"]
            self.assertEqual(sample["actual_attitude_rpy_rad"], row["orientation_rpy_rad"])
            self.assertEqual(sample["angular_velocity_rad_s"], row["angular_velocity_rad_s"])
            self.assertEqual(sample["position_xy"], row["position_m"][:2])
            self.assertEqual(sample["velocity_xy"], row["velocity_m_s"][:2])
            self.assertEqual(sample["battery_voltage_V"], row["battery_voltage_V"])
            self.assertEqual(sample["state_group_skew_s"], row["state_group_skew_s"])
        # World-X components and signed world-Y are retained, not direction-projected.
        self.assertEqual(samples[0]["position_xy"], [.71, -.82])
        self.assertEqual(samples[0]["velocity_xy"], [.14, -.25])
        self.assertEqual(samples[0]["direction_xy"], [0., -1.])
        self.assertEqual(samples[0]["command_started_at"], 99.995)
        self.assertEqual(samples[0]["command_acceleration_xy"], [0., -3.])
        self.assertEqual(records, original_records)

    def test_required_measurement_is_never_synthesized_from_partial_row(self):
        for missing in ("angular_velocity_rad_s", "battery_voltage_V", "state_group_skew_s"):
            with self.subTest(missing=missing):
                record = _observer(10.)
                del record["data"][missing]
                with mock.patch(MODULE + ".extract", return_value=(
                        {}, [SimpleNamespace(segment=0)], [{"timestamp": 10., "segment_id": 0}], {})):
                    with self.assertRaises(KeyError):
                        samples_from_log([record])

    def test_extractor_rejections_propagate_without_silent_sample_repair(self):
        with mock.patch(MODULE + ".extract", side_effect=ValueError("state gap exceeds limit")):
            with self.assertRaisesRegex(ValueError, "state gap exceeds limit"):
                samples_from_log([])


class ReplaySummaryTests(unittest.TestCase):
    def test_reconstructed_provenance_follows_every_frozen_and_refitted_model(self):
        from Interaction.online_prediction_calibration import _CalibrationAccumulator
        from Interaction.tests.test_online_prediction_calibration import (
            _fake_evaluate, _fake_fit, _samples,
        )

        def fit(samples, **kwargs):
            model = _fake_fit(samples, **kwargs)
            model["source_provenance"] = {
                "source": "raw_calibration_samples",
                "command_clock": "actual_phase_send_host_time",
                "completion_certified_by": "caller_after_actual_recovery_send",
                "sample_sha256": "sample-hash-retained",
            }
            return model

        metadata = {
            "mode": "offline_sequential_replay", "source_path": "/offline/flight.json",
            "source_sha256": "a" * 64,
            "command_clock_method": "median phase-event minus next-command timestamp; approximate send anchor",
            "completion_certification": "logged_actual_recovery_commands",
        }
        with tempfile.TemporaryDirectory() as directory:
            accumulator = _CalibrationAccumulator(
                {}, Path(directory) / "report.json", "lb11", list(range(6)),
                fit, _fake_evaluate, metadata=metadata)
            try:
                for segment in range(6):
                    accumulator.receive(_samples(segment))
                accumulator.finish({"accepted_submissions": 6, "rejected_submissions": 0})
                report = json.loads(accumulator.path.read_text())
            finally:
                accumulator.close()
        self.assertTrue(report["validation_passed"])
        self.assertEqual(report["context"], metadata)
        nested_models = [report["candidate"]["model"], report["validated_candidate"]["model"]]
        nested_models += [validation["model"] for validation in report["validation_history"]]
        self.assertEqual(len(nested_models), 4)
        for model in nested_models:
            provenance = model["source_provenance"]
            self.assertEqual(provenance["source"], "reconstructed_flight_log_samples")
            self.assertEqual(provenance["command_clock"], metadata["command_clock_method"])
            self.assertEqual(provenance["completion_certified_by"], "logged_actual_recovery_commands")
            self.assertEqual(provenance["source_log_path"], "/offline/flight.json")
            self.assertEqual(provenance["source_log_sha256"], "a" * 64)
            self.assertEqual(provenance["sample_sha256"], "sample-hash-retained")

    def test_summary_labels_diagnostics_and_uses_frozen_not_all_data_model(self):
        report = _report()
        original = copy.deepcopy(report)
        summary = summary_markdown(report)
        self.assertIn("Offline replay only; no flight commands or active calibration updates.", summary)
        self.assertIn("held-out diagnostic gates passed: True", summary)
        self.assertIn("last all-data refit is NOT independently validated", summary)
        self.assertIn("runtime_enabled=false and flight_approved=false", summary)
        self.assertIn("| 1 | [0, 1] | [2, 3] | False |", summary)
        self.assertIn("| 2 | [0, 1, 2, 3] | [4, 5] | True |", summary)
        self.assertIn('"motion_gain": 1.13', summary)
        self.assertNotIn("9.999", summary)
        self.assertNotIn("8.888", summary)
        heldout = summary.split("## Final held-out trials", 1)[1]
        self.assertIn("| 4 | 0.1234 | 0.0123 | -0.0234 | 0.0345 | False / False |", heldout)
        self.assertIn("| 5 | 0.2345 | 0.0234 | 0.0345 | -0.0456 | True / True |", heldout)
        self.assertIn("Position-controller capture, X/Z response", summary)
        self.assertIn("no future measured states initialize forecasts", summary)
        self.assertEqual(report, original)

    def test_partial_summary_does_not_invent_heldout_metrics(self):
        summary = summary_markdown({"status": "partial", "data_complete": False,
                                    "validation_passed": False})
        self.assertIn("Status: partial; complete data: False", summary)
        self.assertIn("held-out diagnostic gates passed: False", summary)
        self.assertNotIn("## Last frozen model", summary)
        self.assertNotIn("## Final held-out trials", summary)
        self.assertIn("NOT independently validated", summary)


class ReplayCliTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.source = self.directory / "flight.json"
        self.source.write_text('[{"type":"example","data":{}}]', encoding="utf-8")
        self.output = self.directory / "new_replay"
        self.samples = [{"segment_id": segment, "timestamp": 100. + segment}
                        for segment in range(6)]

    def tearDown(self):
        self.temporary.cleanup()

    def test_existing_output_directory_refused_without_overwriting_artifacts(self):
        self.output.mkdir()
        existing = self.output / "report.json"
        existing.write_text('{"previous":true}', encoding="utf-8")
        with mock.patch(MODULE + ".samples_from_log", return_value=(self.samples, list(range(6)), {})), \
                mock.patch(MODULE + ".OnlinePredictionCalibration") as session_class:
            with self.assertRaises(FileExistsError):
                main([str(self.source), "--output", str(self.output)])
        session_class.assert_not_called()
        self.assertEqual(existing.read_text(), '{"previous":true}')
        self.assertEqual(list(self.output.iterdir()), [existing])

    def test_replay_keeps_clock_metadata_provenance_and_only_writes_new_diagnostics(self):
        active = self.directory / "wrench_calibration.json"
        active.write_text('{"lb11":{"retained":42}}', encoding="utf-8")
        active_before = active.read_bytes()
        source_before = self.source.read_bytes()
        metadata = {"command_time_method": "approximate reconstructed phase sends",
                    "clock_offset_s": .021, "duplicate_state_count": 3}
        session = mock.Mock()
        session.start.return_value = True
        session.submit_trial.return_value = True
        session.request_finish.return_value = True
        session.poll.return_value = []
        session.latest_report = _report()
        with mock.patch(MODULE + ".samples_from_log", return_value=(
                self.samples, list(range(6)), metadata)) as augment, \
                mock.patch(MODULE + ".OnlinePredictionCalibration", return_value=session) as session_class, \
                mock.patch("Interaction.wrench_model_calibration.save_drone_calibration") as save_active, \
                mock.patch("builtins.print"):
            result = main([str(self.source), "--output", str(self.output), "--drone-id", "lb_test"])
        self.assertEqual(result, 0)
        augment.assert_called_once_with(json.loads(source_before))
        save_active.assert_not_called()
        self.assertEqual(active.read_bytes(), active_before)
        self.assertEqual(self.source.read_bytes(), source_before)
        args, kwargs = session_class.call_args
        self.assertEqual(args, ({}, self.output.resolve() / "report.json", "lb_test", list(range(6))))
        context = kwargs["metadata"]
        self.assertEqual(context["mode"], "offline_sequential_replay")
        self.assertEqual(context["source_path"], str(self.source.resolve()))
        self.assertEqual(context["source_sha256"], hashlib.sha256(source_before).hexdigest())
        self.assertEqual(context["clock_scope"], "host_receive_effective_delay_with_reconstructed_send_anchors")
        self.assertEqual(context["completion_certification"], "logged_actual_recovery_commands")
        self.assertEqual(context["command_time_method"], "approximate reconstructed phase sends")
        self.assertEqual(context["clock_offset_s"], .021)
        self.assertEqual(context["duplicate_state_count"], 3)
        self.assertEqual(context["prediction_scope"], "attitude_command_only")
        self.assertIs(context["position_capture_model_identified"], False)
        self.assertEqual(session.submit_trial.call_args_list,
                         [mock.call([sample]) for sample in self.samples])
        session.close.assert_called_once_with()
        # The mock fitter emits no files: the CLI itself writes only this summary.
        self.assertEqual([path.name for path in self.output.iterdir()], ["summary.md"])
        self.assertIn("NOT independently validated", (self.output / "summary.md").read_text())

    def test_queue_rejection_closes_session_and_does_not_claim_completed_replay(self):
        session = mock.Mock()
        session.start.return_value = True
        session.submit_trial.side_effect = [True, False]
        with mock.patch(MODULE + ".samples_from_log", return_value=(self.samples, list(range(6)), {})), \
                mock.patch(MODULE + ".OnlinePredictionCalibration", return_value=session):
            with self.assertRaisesRegex(RuntimeError, "failed to queue segment 1"):
                main([str(self.source), "--output", str(self.output)])
        session.close.assert_called_once_with()
        self.assertFalse((self.output / "summary.md").exists())

    def test_incomplete_trial_count_rejected_before_start_or_directory_creation(self):
        for segments in ([0, 1], [0, 1, 2], [0, 1, 2, 3, 4]):
            with self.subTest(segments=segments), \
                    mock.patch(MODULE + ".samples_from_log", return_value=(self.samples, segments, {})), \
                    mock.patch(MODULE + ".OnlinePredictionCalibration") as session_class, \
                    mock.patch("sys.stderr"):
                with self.assertRaises(SystemExit):
                    main([str(self.source), "--output", str(self.output)])
                session_class.assert_not_called()
                self.assertFalse(self.output.exists())


if __name__ == "__main__":
    unittest.main()
