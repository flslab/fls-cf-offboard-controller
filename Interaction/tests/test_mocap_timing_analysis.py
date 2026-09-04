"""Offline timing: no motion-fitted clock offsets or silent extrapolation."""
from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from Interaction.analyze_mocap_timing import (
    WRAP_MS, analyze, load_records, main, unwrap_cf,
)


def records(*, wrap=False):
    result = []
    base = WRAP_MS-50 if wrap else 1000
    for i in range(15):
        host = 100 + i*.01
        result += [
            dict(type="state", group="VEL_ORI", data=dict(time=host,
                cf_timestamp_ms=(base+i*10) % WRAP_MS,
                **{"stateEstimate.vx": 0., "stateEstimate.vy": 1.})),
            dict(type="state", group="POS_ACC", data=dict(time=host+.001,
                cf_timestamp_ms=(base+i*10+1) % WRAP_MS,
                **{"stateEstimate.x": 0., "stateEstimate.y": i*.01+.001})),
            dict(type="frames", data=dict(time=host, frame_id=i, tvec=[0., i*.01, 1.])),
        ]
    result += [
        dict(type="events", name="Planar Braking Calibration Phase",
             data=dict(segment_id=0, phase="brake", time=100.019, direction_xy=[0., 1.])),
        dict(type="events", name="Planar Braking Calibration Phase",
             data=dict(segment_id=0, phase="recovery", time=100.131, direction_xy=[0., 1.])),
    ]
    return sorted(result, key=lambda r:r["data"]["time"])


class MocapTimingAnalysisTests(unittest.TestCase):
    def test_constant_velocity_both_clocks_same_endpoints(self):
        report = analyze(records())
        row = report["brake_windows"][0]
        self.assertTrue(row["available"])
        self.assertEqual(row["sample_count"], 12)
        for clock in ("host_clock", "device_clock"):
            self.assertTrue(row[clock]["available"])
            self.assertAlmostEqual(row[clock]["velocity_integral_m"], .11)
            self.assertAlmostEqual(row[clock]["integral_minus_displacement_m"], 0.)
        self.assertTrue(row["device_clock"]["same_velocity_packet_endpoints_as_host"])

    def test_cf_wrap_is_unwrapped_without_fitted_lag(self):
        clock, wraps = unwrap_cf([WRAP_MS-10, 0, 10])
        np.testing.assert_allclose(np.diff(clock), [.01, .01])
        self.assertEqual(wraps, 1)
        row = analyze(records(wrap=True))["brake_windows"][0]
        self.assertTrue(row["device_clock"]["available"])
        self.assertAlmostEqual(row["device_clock"]["integral_minus_displacement_m"], 0., places=8)

    def test_group_starts_on_other_side_of_wrap(self):
        source = records(wrap=True)
        source = [r for r in source if not (r.get("group") == "POS_ACC" and r["data"]["time"] < 100.05)]
        for r in source:
            if r.get("name") == "Planar Braking Calibration Phase" and r["data"]["phase"] == "brake":
                r["data"]["time"] = 100.059
        row = analyze(source)["brake_windows"][0]
        self.assertTrue(row["device_clock"]["available"])
        self.assertAlmostEqual(row["device_clock"]["integral_minus_displacement_m"], 0., places=8)

    def test_missing_groups_are_unavailable(self):
        source = [r for r in records() if r.get("group") != "POS_ACC"]
        row = analyze(source)["brake_windows"][0]
        self.assertFalse(row["available"])
        self.assertIn("missing POS_ACC", row["reason"])

    def test_nonoverlapping_group_is_not_extrapolated(self):
        source = records()
        for r in source:
            if r.get("group") == "POS_ACC":
                r["data"]["time"] += 50
                r["data"]["cf_timestamp_ms"] += 50000
        row = analyze(source)["brake_windows"][0]
        for clock in ("host_clock", "device_clock"):
            self.assertFalse(row[clock]["available"])
            self.assertIn("no extrapolation", row[clock]["reason"])

    def test_old_logs_without_cf_metadata_keep_host_result(self):
        source = records()
        for r in source:
            r["data"].pop("cf_timestamp_ms", None)
        row = analyze(source)["brake_windows"][0]
        self.assertTrue(row["host_clock"]["available"])
        self.assertFalse(row["device_clock"]["available"])

    def test_duplicate_device_timestamp_is_visible_not_fabricated_wrap(self):
        source = records()
        rows = [r["data"] for r in source if r.get("group") == "VEL_ORI"]
        rows[4]["cf_timestamp_ms"] = rows[3]["cf_timestamp_ms"]
        report = analyze(source)
        self.assertEqual(report["packet_groups"]["VEL_ORI"]["device_clock"]["nonpositive_intervals"], 1)
        self.assertFalse(report["brake_windows"][0]["device_clock"]["available"])
        with self.assertRaises(ValueError):
            unwrap_cf([True, 5])

    def test_unrelated_startup_duplicate_does_not_invalidate_later_window(self):
        source = records()
        rows = [r["data"] for r in source if r.get("group") == "POS_ACC"]
        rows[1]["cf_timestamp_ms"] = rows[0]["cf_timestamp_ms"]
        for r in source:
            if r["data"].get("phase") == "brake":
                r["data"]["time"] = 100.059
        report = analyze(source)
        self.assertEqual(report["packet_groups"]["POS_ACC"]["device_clock"]["nonpositive_intervals"], 1)
        self.assertTrue(report["brake_windows"][0]["device_clock"]["available"])

    def test_position_duplicate_inside_trial_is_rejected(self):
        source = records()
        rows = [r["data"] for r in source if r.get("group") == "POS_ACC"]
        rows[5]["cf_timestamp_ms"] = rows[4]["cf_timestamp_ms"]
        self.assertFalse(analyze(source)["brake_windows"][0]["device_clock"]["available"])

    def test_frame_gaps_and_bursts_do_not_infer_camera_loss(self):
        source = [dict(type="frames", data=dict(time=t, frame_id=i))
                  for i,t in enumerate([1., 1.01, 1.09, 1.0903])]
        report = analyze(source)
        self.assertEqual(report["frame_arrivals"]["gap_count"], 1)
        self.assertEqual(report["frame_arrivals"]["burst_count"], 1)
        self.assertIn("local wait-loop counter", report["caveats"][0])

    def test_sorting_does_not_hide_original_record_order_clock_reversal(self):
        source = [dict(type="frames", data=dict(time=t, frame_id=i))
                  for i,t in enumerate([1., 1.02, 1.01, 1.01])]
        summary = analyze(source)["frame_arrivals"]
        self.assertEqual(summary["nonpositive_intervals"], 1)
        self.assertEqual(summary["original_record_order_nonpositive_intervals"], 2)
        self.assertIn("host-time order", summary["sorting_note"])
        packet_source = [dict(r, type="state", group="POS_ACC") for r in source]
        summary = analyze(packet_source)["packet_groups"]["POS_ACC"]
        self.assertEqual(summary["original_record_order_nonpositive_intervals"], 2)

    def test_optional_instrumentation_missing_is_not_zero(self):
        source = records()
        source.append(dict(type="mocap_timing", data=dict(time=100., frame_id=0,
            wait_duration_s=.08, processing_duration_s=.002, callback_duration_s=.001,
            callback_count=1, local_loop_interval_s=.081)))
        frame = next(r["data"] for r in source if r["type"] == "frames")
        frame["mocap_timing"] = dict(extpos_send_duration_s=.0004, wait_return_to_send_s=.001)
        timing = analyze(source)["optional_instrumentation"]
        self.assertAlmostEqual(timing["timing_fields"]["wait_duration_s"]["maximum"], .08)
        self.assertEqual(timing["frame_send_metadata_count"], 1)
        self.assertEqual(timing["frames_without_send_metadata"], 14)
        missing = analyze([])["optional_instrumentation"]["send_fields"]["extpos_send_duration_s"]
        self.assertFalse(missing["available"])
        self.assertIsNone(missing["maximum"])

    def test_new_loop_send_and_sampled_queue_fields(self):
        source = records()
        for i in range(2):
            source.append(dict(type="mocap_timing", data=dict(time=100.+i,
                processing_excluding_callbacks_s=.001,
                max_callback_duration_s=.002, between_processing_and_wait_s=.003,
                logger_queue=dict(accepted_records=100*(i+1), written_records=100*i,
                    queued_records=80-i, in_flight_records=20,
                    max_queue_depth=100, queue_capacity=20000,
                    overflow_errors=0, writer_errors=i))))
        frame = next(r["data"] for r in source if r["type"] == "frames")
        frame["mocap_timing"] = dict(extpos_send_interval_s=.08)
        timing = analyze(source)["optional_instrumentation"]
        self.assertEqual(timing["logger_queue_snapshot_count"], 2)
        self.assertEqual(timing["logger_queue_fields"]["accepted_records"]["last_observed"], 200)
        self.assertEqual(timing["logger_queue_fields"]["writer_errors"]["maximum"], 1)
        self.assertEqual(timing["send_fields"]["extpos_send_interval_s"]["maximum"], .08)
        self.assertEqual(timing["timing_fields"]["between_processing_and_wait_s"]["count"], 2)
        missing = analyze([])["optional_instrumentation"]["logger_queue_fields"]["queued_records"]
        self.assertFalse(missing["available"])
        self.assertIsNone(missing["last_observed"])

    def test_partial_window_remains_flagged(self):
        source = [r for r in records() if r["data"].get("phase") != "recovery"]
        row = analyze(source)["brake_windows"][0]
        self.assertFalse(row["available"])
        self.assertIn("missing", row["reason"])

    def test_complete_prefix_recovery_does_not_skip_malformed_middle(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"partial.json"
            path.write_text('[{"type":"frames","data":{"time":1}},\n{"broken":')
            recovered, info = load_records(path)
            self.assertEqual(len(recovered), 1)
            self.assertTrue(info["recovered"])
            path.write_text('[{"type":"frames"},garbage,{"type":"state"}]')
            recovered, _ = load_records(path)
            self.assertEqual(len(recovered), 1)
            path.write_text('{"not":"array"}')
            with self.assertRaises(ValueError):
                load_records(path)

    def test_cli_multiple_inputs_is_offline_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"log.json"
            original = json.dumps(records())
            path.write_text(original)
            output = Path(tmp)/"audit"
            with redirect_stdout(io.StringIO()):
                self.assertEqual(main([str(path), str(path), "--output", str(output)]), 0)
            report = json.loads((output/"report.json").read_text())
            self.assertEqual(len(report["runs"]), 2)
            self.assertEqual(path.read_text(), original)
            self.assertTrue((output/"README.md").exists())
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main([str(path), "--output", str(output)])


if __name__ == "__main__":
    unittest.main()
