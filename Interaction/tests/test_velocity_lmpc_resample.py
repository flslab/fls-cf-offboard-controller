"""Tests for fail-closed offline decision-time flight-log resampling."""
from contextlib import redirect_stdout
from copy import deepcopy
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

from Interaction.tests.test_velocity_lmpc_replay import (
    complete_records,
    config,
)
from Interaction.velocity_lmpc_replay import (
    CLOSE_EVENT,
    START_EVENT,
    TERMINAL_DWELL_EVENT,
    extract_velocity_lmpc_episodes,
)
from Interaction.velocity_lmpc_resample import (
    RESAMPLE_METHOD,
    main,
    resample_velocity_lmpc_records,
)


RAW_SHA256 = "b"*64


def raw_records(*, episode_id="lb11-raw-release-1", sign=1):
    """Return a raw state-before-send fixture with a real upper bracket."""
    records = complete_records(
        decision_phase_s=0.005,
        episode_id=episode_id,
        sign=sign,
    )
    start = next(
        record for record in records if record.get("name") == START_EVENT
    )
    start["data"].update({
        "release_dataset_prediction_step_s": 0.02,
        "release_dataset_command_delay_s": 0.04,
    })
    start["data"]["release_command_effective_at_state"][
        "effective_query_time"
    ] = 0.96
    # The flight collector defers the terminal event until one fresh,
    # command-free state brackets the position handoff send. The ordinary
    # replay fixture puts its lingering row after that event; turn it into the
    # collector's explicit pre-event resampling bracket.
    terminal = records.pop(-2)
    bracket = records[-1]
    final_position = records[-2]["data"][
        "actual_commands_sent_since_previous_state"
    ][0]
    bracket["data"].update({
        "time": 1.22,
        "state_time": 1.22,
        "state_age_s": 0.0,
        "position_m": [0.1, sign*0.10, 1.0],
        "actual_commands_sent_since_previous_state": [],
        "release_dataset_resample_upper_bracket": True,
        "release_dataset_final_position_sent_at": final_position["sent_at"],
        "release_dataset_final_position_sequence": final_position["sequence"],
    })
    terminal["data"]["time"] = 1.221
    terminal["data"]["resample_upper_bracket_state_time"] = 1.22
    terminal["data"]["final_position_sent_at"] = final_position["sent_at"]
    terminal["data"]["final_position_sequence"] = final_position["sequence"]
    records.append(terminal)
    return records


def observer_rows(records):
    return [
        record for record in records
        if record.get("type") == "wrench_observer"
    ]


def shift_timeline(records, offset_s):
    shifted = deepcopy(records)

    def visit(value):
        if isinstance(value, dict):
            for key, item in tuple(value.items()):
                if key in {
                    "time", "state_time", "release_state_time",
                    "terminal_state_time", "resample_upper_bracket_state_time",
                    "effective_query_time", "final_position_sent_at",
                    "release_dataset_final_position_sent_at", "sent_at",
                } and isinstance(item, (int, float)):
                    value[key] = item+offset_s
                else:
                    visit(item)
        elif isinstance(value, list):
            for item in value:
                visit(item)

    visit(shifted)
    return shifted


class SuccessfulResampleTests(unittest.TestCase):
    def test_interpolates_at_actual_send_epochs_and_feeds_replay(self):
        raw = raw_records()
        raw_rows = observer_rows(raw)
        raw_rows[0]["data"]["angular_velocity_rad_s"] = [0.0, 0.0, 0.0]
        raw_rows[1]["data"]["angular_velocity_rad_s"] = [0.04, 0.08, 0.0]
        raw_rows[0]["data"]["battery_voltage_V"] = 4.0
        raw_rows[1]["data"]["battery_voltage_V"] = 3.6
        original_first_send = deepcopy(
            raw_rows[0]["data"]
            ["actual_commands_sent_since_previous_state"][0]
        )
        original_first_send["radio_packet_id"] = "actual-packet-4"
        raw_rows[0]["data"][
            "actual_commands_sent_since_previous_state"
        ][0]["radio_packet_id"] = "actual-packet-4"
        original_final_send = deepcopy(
            raw_rows[-2]["data"]
            ["actual_commands_sent_since_previous_state"][0]
        )

        result = resample_velocity_lmpc_records(
            raw,
            prediction_step_s=0.02,
            command_delay_s=0.04,
            raw_sha256=RAW_SHA256,
        )

        self.assertEqual(result.episode_ids, ("lb11-raw-release-1",))
        self.assertEqual(result.rejections, ())
        self.assertNotIn("effective_query_time", json.dumps(result.records))
        self.assertNotIn("effective_query_time", json.dumps(result.records))
        rows = observer_rows(result.records)
        self.assertEqual(len(rows), 11)
        self.assertTrue(all(
            row["data"]["time"] == row["data"]["state_time"]
            for row in rows
        ))
        self.assertEqual(
            rows[0]["data"]["actual_commands_sent_since_previous_state"],
            [original_first_send],
        )
        self.assertEqual(
            rows[-1]["data"]["actual_commands_sent_since_previous_state"],
            [original_final_send],
        )
        self.assertAlmostEqual(rows[0]["data"]["velocity_m_s"][1], 0.3775)
        self.assertAlmostEqual(rows[0]["data"]["position_m"][1], 0.0025)
        self.assertAlmostEqual(
            rows[0]["data"]["orientation_rpy_rad"][0],
            math.radians(2.25),
        )
        for actual, expected in zip(
            rows[0]["data"]["angular_velocity_rad_s"],
            (0.01, 0.02, 0.0),
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(rows[0]["data"]["battery_voltage_V"], 3.9)
        provenance = rows[0]["data"]["resample_provenance"]
        self.assertEqual(provenance["raw_sha256"], RAW_SHA256)
        self.assertEqual(provenance["source_record_indices"], [1, 2])
        self.assertEqual(provenance["source_state_times_s"], [1.0, 1.02])
        self.assertAlmostEqual(provenance["interpolation_weight"], 0.25)
        self.assertEqual(provenance["method"], RESAMPLE_METHOD)
        terminal = next(
            record for record in result.records
            if record.get("name") == TERMINAL_DWELL_EVENT
        )
        self.assertTrue(terminal["data"]["terminal_gate"]["complete"])

        replay = extract_velocity_lmpc_episodes(result.records, config())
        self.assertEqual(len(replay.episodes), 1)
        self.assertEqual(replay.rejections, ())
        self.assertAlmostEqual(
            replay.episodes[0].samples[0].aligned_velocity_m_s,
            0.3775,
        )

    def test_attitude_angles_are_unwrapped_before_interpolation(self):
        raw = raw_records()
        rows = observer_rows(raw)
        rows[0]["data"]["orientation_rpy_rad"][2] = math.radians(179.0)
        for index, row in enumerate(rows[1:]):
            row["data"]["orientation_rpy_rad"][2] = math.radians(
                -179.0+2.0*index
            )

        result = resample_velocity_lmpc_records(
            raw,
            prediction_step_s=0.02,
            command_delay_s=0.04,
            raw_sha256=RAW_SHA256,
        )

        self.assertEqual(result.rejections, ())
        output_yaws = [
            row["data"]["orientation_rpy_rad"][2]
            for row in observer_rows(result.records)
        ]
        self.assertAlmostEqual(math.degrees(output_yaws[0]), 179.5, places=8)
        self.assertAlmostEqual(math.degrees(output_yaws[1]), 181.5, places=8)
        self.assertTrue(all(
            after > before
            for before, after in zip(output_yaws, output_yaws[1:])
        ))

    def test_effective_lookup_annotation_does_not_change_command_identity(self):
        raw = raw_records()
        start = next(
            record for record in raw
            if record.get("name") == START_EVENT
        )
        start["data"]["release_command_effective_at_state"][
            "effective_query_time"
        ] = 0.96

        result = resample_velocity_lmpc_records(
            raw,
            prediction_step_s=0.02,
            command_delay_s=0.04,
            raw_sha256=RAW_SHA256,
        )

        self.assertEqual(result.episode_ids, ("lb11-raw-release-1",))
        self.assertEqual(result.rejections, ())
        self.assertNotIn("effective_query_time", json.dumps(result.records))

    def test_one_bad_episode_does_not_contaminate_another(self):
        bad = raw_records(episode_id="bad-release")
        observer_rows(bad)[2]["data"]["xy_boundary_margin_m"] = -0.01
        good = shift_timeline(
            raw_records(episode_id="good-release", sign=-1), 10.0
        )

        result = resample_velocity_lmpc_records(
            bad+good,
            prediction_step_s=0.02,
            command_delay_s=0.04,
            raw_sha256=RAW_SHA256,
        )

        self.assertEqual(result.episode_ids, ("good-release",))
        self.assertEqual(len(result.rejections), 1)
        self.assertEqual(result.rejections[0].episode_id, "bad-release")
        names = [record.get("name") for record in result.records]
        self.assertEqual(names.count(CLOSE_EVENT), 1)
        self.assertEqual(names.count(TERMINAL_DWELL_EVENT), 1)
        replay = extract_velocity_lmpc_episodes(result.records, config())
        self.assertEqual(
            tuple(episode.episode_id for episode in replay.episodes),
            ("good-release",),
        )


class FailClosedResampleTests(unittest.TestCase):
    def assert_rejected(self, raw, pattern=None):
        result = resample_velocity_lmpc_records(
            raw,
            prediction_step_s=0.02,
            command_delay_s=0.04,
            raw_sha256=RAW_SHA256,
        )
        self.assertEqual(result.episode_ids, ())
        self.assertEqual(len(result.rejections), 1)
        if pattern is not None:
            self.assertRegex(result.rejections[0].reason, pattern)
        self.assertFalse(any(
            record.get("name") == TERMINAL_DWELL_EVENT
            for record in result.records
        ))
        self.assertTrue(any(
            record.get("name") == CLOSE_EVENT
            for record in result.records
        ))
        return result

    def test_missing_upper_source_bracket_forbids_extrapolation(self):
        raw = raw_records()
        raw[:] = [
            record for record in raw
            if not record.get("data", {}).get(
                "release_dataset_resample_upper_bracket"
            )
        ]

        result = self.assert_rejected(raw, "explicit resample upper bracket")

        replay = extract_velocity_lmpc_episodes(result.records, config())
        self.assertEqual(replay.episodes, ())

    def test_upper_bracket_after_terminal_marker_is_rejected(self):
        raw = raw_records()
        bracket = raw.pop(-2)
        raw.append(bracket)

        self.assert_rejected(raw, "explicit resample upper bracket")

    def test_terminal_marker_must_bind_the_upper_bracket_time(self):
        raw = raw_records()
        terminal = next(
            record for record in raw
            if record.get("name") == TERMINAL_DWELL_EVENT
        )
        terminal["data"]["resample_upper_bracket_state_time"] += 0.01

        self.assert_rejected(raw, "does not identify")

    def test_unknown_duplicate_command_payload_difference_is_rejected(self):
        raw = raw_records()
        start = next(
            record for record in raw
            if record.get("name") == START_EVENT
        )
        start["data"]["release_command_effective_at_state"][
            "zdistance_m"
        ] = 1.0

        self.assert_rejected(raw, "conflicting raw payloads")

    def test_cli_command_delay_must_match_release_timing_identity(self):
        raw = raw_records()
        result = resample_velocity_lmpc_records(
            raw,
            prediction_step_s=0.02,
            command_delay_s=0.03,
            raw_sha256=RAW_SHA256,
        )

        self.assertEqual(result.episode_ids, ())
        self.assertRegex(result.rejections[0].reason, "command_delay_s")

    def test_effective_query_time_must_match_logged_delay(self):
        raw = raw_records()
        start = next(
            record for record in raw
            if record.get("name") == START_EVENT
        )
        start["data"]["release_command_effective_at_state"][
            "effective_query_time"
        ] = 0.97

        self.assert_rejected(raw, "effective_query_time")

    def test_logged_terminal_success_cannot_replace_measured_terminal(self):
        raw = raw_records()
        rows = observer_rows(raw)
        rows[-2]["data"]["velocity_m_s"][1] = 0.20
        rows[-1]["data"]["velocity_m_s"][1] = 0.20

        self.assert_rejected(raw, "measured terminal dwell")

    def test_stale_source_is_rejected(self):
        raw = raw_records()
        data = observer_rows(raw)[2]["data"]
        data["state_age_s"] = 0.11
        data["time"] = data["state_time"]+0.11
        self.assert_rejected(raw, "stale")

    def test_skewed_source_is_rejected(self):
        raw = raw_records()
        observer_rows(raw)[2]["data"]["state_group_skew_s"] = 0.031
        self.assert_rejected(raw, "skew")

    def test_boundary_source_is_rejected(self):
        raw = raw_records()
        observer_rows(raw)[2]["data"]["xy_boundary_margin_m"] = -0.01
        self.assert_rejected(raw, "boundary")

    def test_hazard_event_is_rejected(self):
        raw = raw_records()
        raw.insert(3, {
            "type": "events",
            "name": "Emergency Stop",
            "data": {"time": 1.03},
        })
        self.assert_rejected(raw, "hazard")

    def test_non_attitude_action_is_rejected(self):
        raw = raw_records()
        action = observer_rows(raw)[3]["data"][
            "actual_commands_sent_since_previous_state"
        ][0]
        action["kind"] = "velocity_hover"
        self.assert_rejected(raw, "non-final actual command")

    def test_actual_command_cadence_gap_is_rejected(self):
        raw = raw_records()
        action = observer_rows(raw)[3]["data"][
            "actual_commands_sent_since_previous_state"
        ][0]
        action["sent_at"] += 0.004
        self.assert_rejected(raw, "decision command interval")

    def test_source_measurement_gap_is_rejected(self):
        raw = raw_records()
        first = observer_rows(raw)[0]["data"]
        first["state_time"] = 0.94
        first["time"] = 0.945
        self.assert_rejected(raw, "source observer gap")


class CommandLineTests(unittest.TestCase):
    def test_cli_writes_new_array_and_never_overwrites(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/"raw.json"
            output = Path(directory)/"resampled.json"
            source.write_text(json.dumps(raw_records()), encoding="utf-8")
            argv = [
                "--input", str(source),
                "--output", str(output),
                "--prediction-step-s", "0.02",
                "--command-delay-s", "0.04",
            ]
            stdout = io.StringIO()
            with redirect_stdout(stdout):
                first_status = main(argv)
            first_bytes = output.read_bytes()

            self.assertEqual(first_status, 0)
            report = json.loads(stdout.getvalue())
            self.assertEqual(report["status"], "written")
            written = json.loads(first_bytes)
            replay = extract_velocity_lmpc_episodes(written, config())
            self.assertEqual(len(replay.episodes), 1)

            with redirect_stdout(io.StringIO()):
                second_status = main(argv)
            self.assertEqual(second_status, 2)
            self.assertEqual(output.read_bytes(), first_bytes)


if __name__ == "__main__":
    unittest.main()
