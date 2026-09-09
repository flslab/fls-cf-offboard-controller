"""Tests for strict offline flight-log to velocity-LMPC replay."""
from contextlib import redirect_stdout
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

from Interaction.velocity_lmpc_replay import (
    CLOSE_EVENT,
    LEGACY_HANDOFF_EVENT,
    ReplayValidationError,
    START_EVENT,
    TERMINAL_DWELL_EVENT,
    VelocityLMPCReplayConfig,
    build_safe_set_artifact,
    extract_velocity_lmpc_episodes,
    load_complete_flight_records,
    main,
    save_new_safe_set_artifact,
)
from Interaction.velocity_lmpc_safe_set import (
    STATE_ACTION_PHASE_CONTRACT,
    StageCostSpec,
    VelocityLMPCSafeSet,
)


FINGERPRINT = "reduced-v3:"+("a"*64)


def event(name, data):
    return {"type": "events", "name": name, "data": dict(data)}


def attitude_command(sent_at, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0,
                     sequence=1):
    return {
        "kind": "attitude_zdistance",
        "sent_at": sent_at,
        "sequence": sequence,
        "roll_deg": roll_deg,
        "pitch_deg": pitch_deg,
        "yaw_deg": yaw_deg,
    }


def position_command(sent_at, sequence=1, position=(0.0, 0.0, 1.0)):
    return {
        "kind": "position",
        "sent_at": sent_at,
        "sequence": sequence,
        "position_m": list(position),
    }


def terminal_gate(*, complete=False):
    return {
        "complete": complete,
        "reason": (
            "terminal_dwell_complete" if complete else
            "sample_outside_terminal_set"
        ),
        "violations": [] if complete else ["velocity_xy"],
    }


def observer(*, stamp, state_time=None, episode_id=None, direction=None,
             velocity=(0.0, 0.0, 0.0), position=(0.0, 0.0, 1.0),
             rpy=(0.0, 0.0, 0.0), rates=(0.0, 0.0, 0.0),
             applied=None, sent_batch=None,
             command_owner="attitude_coast",
             gate=None, pending_outcome=None,
             state_age=0.005, skew=0.005, boundary=0.5, battery=4.0,
             **extra):
    if state_time is None:
        state_time = stamp
    if applied is None:
        applied = attitude_command(state_time-0.04)
    if sent_batch is None:
        sent_batch = [attitude_command(state_time+0.005)]
    data = {
        "time": stamp,
        "state_time": state_time,
        "state_age_s": state_age,
        "state_group_skew_s": skew,
        "release_dataset_episode_id": episode_id,
        "release_dataset_direction_xy": direction,
        "release_dataset_command_owner": command_owner,
        "release_dataset_terminal_gate": (
            terminal_gate() if gate is None else gate
        ),
        "release_dataset_pending_outcome": pending_outcome,
        "actual_command_applied_at_state": applied,
        "actual_commands_sent_since_previous_state": sent_batch,
        "position_m": list(position),
        "velocity_m_s": list(velocity),
        "orientation_rpy_rad": list(rpy),
        "angular_velocity_rad_s": list(rates),
        "xy_boundary_margin_m": boundary,
        "battery_voltage_V": battery,
        "measurement_rejected": False,
    }
    data.update(extra)
    return {"type": "wrench_observer", "name": None, "data": data}


def complete_records(*, sign=1, episode_id="lb11-release-1",
                     direction_x=0.0, duplicate_before_state_at=None,
                     duplicate_roll_deg=7.0, measured_direction=None,
                     decision_phase_s=0.0, command_delay_s=0.04):
    direction = [direction_x, float(sign)]
    if measured_direction is None:
        measured_direction = [0.0, float(sign)]
    records = []
    # START owns the complete delay-window seed. Ordinary pre-release observer
    # rows intentionally do not duplicate this command history.
    history = [
        (0.96, sign*2.0, 2),
        (0.98, sign*3.0, 3),
    ]
    sequence = 3
    records.append(event(START_EVENT, {
        "time": 0.999,
        "release_dataset_episode_id": episode_id,
        "release_dataset_direction_xy": direction,
        "release_dataset_measured_sensor_axis_world_xy": list(
            measured_direction
        ),
        "release_state_time": 1.0,
        "release_command_effective_at_state": attitude_command(
            0.96, sign*2.0, sequence=2
        ),
        "release_pending_command_history": [
            attitude_command(0.96, sign*2.0, sequence=2),
            attitude_command(0.98, sign*3.0, sequence=3),
        ],
        "offline_lmpc_dataset_only": True,
    }))

    velocities = [
        0.40, 0.31, 0.22, 0.11, 0.045, 0.035,
        0.025, 0.015, 0.010, 0.007, 0.005,
    ]
    command_roll = [sign*5.0, sign*5.0, sign*4.0, sign*2.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    measured_roll = [sign*math.radians(2.0), sign*math.radians(3.0),
                     sign*math.radians(2.0), sign*math.radians(1.0),
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    for index, (speed, sent_roll, roll) in enumerate(zip(
            velocities, command_roll, measured_roll)):
        state_time = 1.00+0.02*index
        effective_time = state_time-command_delay_s
        applied_sent_at, applied_roll, applied_sequence = max(
            item for item in history if item[0] <= effective_time+1e-12
        )
        final = index == len(velocities)-1
        sent_at = state_time+decision_phase_s
        sent_batch = []
        if index == duplicate_before_state_at:
            sequence += 1
            duplicate_command = attitude_command(
                state_time-0.005,
                sign*duplicate_roll_deg,
                sequence=sequence,
            )
            sent_batch.append(duplicate_command)
            history.append((
                duplicate_command["sent_at"],
                sign*duplicate_roll_deg,
                sequence,
            ))
        sequence += 1
        sent_command = (
            position_command(
                sent_at,
                sequence,
                position=(0.1, sign*(0.01*index), 1.0),
            ) if final else
            attitude_command(sent_at, sent_roll, sequence=sequence)
        )
        sent_batch.append(sent_command)
        gate = terminal_gate(complete=final)
        records.append(observer(
            stamp=state_time+decision_phase_s,
            state_time=state_time,
            episode_id=episode_id,
            direction=direction,
            velocity=(0.01, sign*speed, 0.0),
            position=(0.1, sign*(0.01*index), 1.0),
            rpy=(roll, 0.0, 0.0),
            rates=(0.0, 0.0, 0.0),
            applied=attitude_command(
                applied_sent_at, applied_roll, sequence=applied_sequence
            ),
            sent_batch=sent_batch,
            state_age=decision_phase_s,
            command_owner=("position_hold" if final else "attitude_coast"),
            gate=gate,
            pending_outcome=("terminal_handoff" if final else None),
        ))
        if not final:
            history.append((sent_at, sent_roll, sequence))
        if index == len(velocities)-2:
            # This old transition event is deliberately not evidence and does
            # not delimit the dataset episode.
            records.append(event(LEGACY_HANDOFF_EVENT, {
                "time": 1.152,
                "release_dataset_episode_id": episode_id,
                "release_dataset_outcome": "terminal_handoff",
            }))
    records.append(event(TERMINAL_DWELL_EVENT, {
        "time": 1.211,
        "release_dataset_episode_id": episode_id,
        "release_dataset_outcome": "terminal_handoff",
        "reason": "full_measured_terminal_dwell_complete",
        "terminal_gate": terminal_gate(complete=True),
        "terminal_state_time": 1.20,
    }))
    # This is expected flight-loop behavior: the completed ID lingers until
    # the next release, but the post-handoff position row is not in the episode.
    records.append(observer(
        stamp=1.24,
        state_time=1.24,
        episode_id=episode_id,
        direction=direction,
        velocity=(0.0, 0.0, 0.0),
        applied=position_command(
            1.205, sequence, position=(0.1, sign*0.10, 1.0)
        ),
        sent_batch=[position_command(
            1.245, sequence+1, position=(0.1, sign*0.10, 1.0)
        )],
    ))
    return records


def config(**updates):
    values = dict(
        model_fingerprint=FINGERPRINT,
        state_dimension=5,
        prediction_step_s=0.02,
        command_delay_s=0.04,
    )
    values.update(updates)
    return VelocityLMPCReplayConfig(**values)


class SuccessfulExtractionTests(unittest.TestCase):
    def test_extracts_aligned_state_command_and_oldest_first_delay_queue(self):
        result = extract_velocity_lmpc_episodes(complete_records(), config())
        self.assertTrue(result.offline_only)
        self.assertFalse(result.flight_commands_generated)
        self.assertEqual(
            result.state_action_phase_contract,
            STATE_ACTION_PHASE_CONTRACT,
        )
        self.assertEqual(len(result.episodes), 1)
        episode = result.episodes[0]
        first = episode.samples[0]
        self.assertEqual(len(first.state), 5)
        self.assertAlmostEqual(first.state[0], 0.40)
        self.assertAlmostEqual(first.state[1], math.radians(-2.0), places=5)
        self.assertAlmostEqual(first.command[0], math.radians(-5.0), places=5)
        self.assertAlmostEqual(first.state[3], math.radians(-2.0), places=5)
        self.assertAlmostEqual(first.state[4], math.radians(-3.0), places=5)
        self.assertAlmostEqual(first.cross_velocity_m_s, -0.01)
        self.assertAlmostEqual(episode.samples[-1].dt_s, 0.0)
        self.assertAlmostEqual(episode.samples[-1].command[0], 0.0)
        self.assertAlmostEqual(episode.samples[-1].aligned_position_m, 0.10)
        self.assertAlmostEqual(episode.context.cross_speed_m_s, -0.01)
        self.assertEqual(episode.context.direction_sign, 1)

    def test_negative_world_y_is_normalized_to_positive_episode_speed(self):
        result = extract_velocity_lmpc_episodes(
            complete_records(sign=-1, episode_id="lb11-release-neg"), config()
        )
        episode = result.episodes[0]
        self.assertEqual(episode.context.direction_sign, -1)
        self.assertAlmostEqual(episode.context.initial_speed_m_s, 0.40)
        self.assertAlmostEqual(episode.samples[0].state[1], math.radians(-2.0), places=5)
        self.assertAlmostEqual(episode.samples[0].command[0], math.radians(-5.0), places=5)
        self.assertAlmostEqual(episode.samples[-1].aligned_position_m, 0.10)

    def test_fractional_delay_on_aligned_decision_grid_is_preserved(self):
        result = extract_velocity_lmpc_episodes(
            complete_records(command_delay_s=0.03),
            config(command_delay_s=0.03),
        )

        self.assertEqual(len(result.episodes), 1)
        self.assertEqual(result.command_delay_s, 0.03)
        self.assertEqual(len(result.episodes[0].samples[0].state[3:]), 2)

    def test_small_real_sensor_axis_bias_is_preserved_and_accepted(self):
        measured = [0.10, math.sqrt(1.0-0.10**2)]
        records = complete_records(measured_direction=measured)

        result = extract_velocity_lmpc_episodes(records, config())

        self.assertEqual(len(result.episodes), 1)
        start = next(
            record for record in records if record.get("name") == START_EVENT
        )
        self.assertEqual(
            start["data"][
                "release_dataset_measured_sensor_axis_world_xy"
            ],
            measured,
        )

    def test_delay_queue_preserves_orthogonal_command_projection(self):
        records = complete_records()
        start = records[0]["data"]
        start["release_command_effective_at_state"]["pitch_deg"] = 0.25
        start["release_pending_command_history"][0]["pitch_deg"] = 0.25
        start["release_pending_command_history"][1]["pitch_deg"] = 0.50
        first_observer = next(
            record for record in records
            if record.get("type") == "wrench_observer"
            and record["data"].get("release_dataset_episode_id")
        )
        first_observer["data"]["actual_command_applied_at_state"][
            "pitch_deg"
        ] = 0.25
        active_rows = [
            record for record in records
            if record.get("type") == "wrench_observer"
            and record["data"].get("release_dataset_episode_id")
        ]
        active_rows[1]["data"]["actual_command_applied_at_state"][
            "pitch_deg"
        ] = 0.50

        result = extract_velocity_lmpc_episodes(records, config())

        samples = result.episodes[0].samples
        for actual, expected in zip(
            samples[0].pending_orthogonal_commands_rad,
            (math.radians(0.25), math.radians(0.50)),
        ):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(
            samples[1].pending_orthogonal_commands_rad,
            (math.radians(0.50), samples[0].orthogonal_command_rad),
        ):
            self.assertAlmostEqual(actual, expected)

    def test_multiple_sends_inside_one_prediction_step_are_rejected(self):
        result = extract_velocity_lmpc_episodes(
            complete_records(
                duplicate_before_state_at=2,
                duplicate_roll_deg=7.0,
            ),
            config(),
        )
        self.assertEqual(result.episodes, ())
        self.assertRegex(result.rejections[0].reason, "multiple flight commands")

    def test_raw_callback_record_may_interleave_before_terminal_event(self):
        records = complete_records()
        terminal_index = next(
            index for index, record in enumerate(records)
            if record.get("name") == TERMINAL_DWELL_EVENT
        )
        records.insert(terminal_index, {
            "type": "state",
            "name": "stateEstimate",
            "data": {"time": 1.2005},
        })
        result = extract_velocity_lmpc_episodes(records, config())
        self.assertEqual(len(result.episodes), 1)

    def test_result_builds_valid_artifact_without_mutating_existing(self):
        result = extract_velocity_lmpc_episodes(complete_records(), config())
        first = build_safe_set_artifact(result)
        second_result = extract_velocity_lmpc_episodes(
            complete_records(sign=-1, episode_id="lb11-release-2"), config()
        )
        combined = build_safe_set_artifact(second_result, existing=first)
        self.assertEqual(len(first.episodes), 1)
        self.assertEqual(len(combined.episodes), 2)


class StrictRejectionTests(unittest.TestCase):
    def active_rows(self, records):
        start = next(index for index, record in enumerate(records)
                     if record.get("name") == START_EVENT)
        close = next(index for index, record in enumerate(records)
                     if record.get("name") in (
                         TERMINAL_DWELL_EVENT, CLOSE_EVENT
                     ))
        return [record for record in records[start+1:close] if (
            record.get("type") == "wrench_observer"
        )]

    def assert_rejected(self, records, pattern):
        result = extract_velocity_lmpc_episodes(records, config())
        self.assertEqual(result.episodes, ())
        self.assertTrue(result.rejections)
        self.assertRegex(
            " | ".join(item.reason for item in result.rejections), pattern
        )
        return result

    def test_rejects_missing_actual_command_timestamp(self):
        records = complete_records()
        row = self.active_rows(records)[2]
        row["data"]["actual_commands_sent_since_previous_state"][0].pop(
            "sent_at"
        )
        self.assert_rejected(records, "sent_since_previous_state.*sent_at")

    def test_rejects_command_or_state_gap(self):
        records = complete_records()
        rows = self.active_rows(records)
        rows[3]["data"]["actual_commands_sent_since_previous_state"][0][
            "sent_at"
        ] += 0.08
        self.assert_rejected(
            records, "duplicate or non-monotonic|timestamp gap"
        )

        records = complete_records()
        rows = self.active_rows(records)
        rows[3]["data"]["state_time"] += 0.08
        self.assert_rejected(
            records,
            "state timestamp gap|sent-after-state timestamp|"
            "inconsistent state/observation timing",
        )

    def test_rejects_incomplete_command_sequence(self):
        records = complete_records()
        self.active_rows(records)[3]["data"][
            "actual_commands_sent_since_previous_state"
        ][0]["sequence"] += 1
        self.assert_rejected(records, "command sequence.*incomplete")

    def test_rejects_forged_applied_command_on_every_observer_row(self):
        records = complete_records()
        forged = self.active_rows(records)[1]["data"][
            "actual_command_applied_at_state"
        ]
        forged.update({
            "sent_at": 0.90,
            "sequence": 999,
            "roll_deg": 29.0,
        })

        self.assert_rejected(
            records,
            "actual_command_applied_at_state disagrees.*delayed send history",
        )

    def test_raw_post_observation_command_phase_requires_resampling(self):
        for phase_s in (0.0005, 0.005):
            with self.subTest(phase_s=phase_s):
                self.assert_rejected(
                    complete_records(
                        decision_phase_s=phase_s,
                        command_delay_s=0.03,
                    ),
                    "not on the LMPC decision-time grid.*explicit resampling",
                )

    def test_complete_raw_queue_must_match_fixed_grid_successor(self):
        records = complete_records()
        rows = self.active_rows(records)
        shifted = rows[2]["data"]
        shifted["state_time"] = 1.0395
        shifted["time"] = 1.0395
        shifted["state_age_s"] = 0.0
        shifted["actual_commands_sent_since_previous_state"][0][
            "sent_at"
        ] = 1.0395
        shifted["actual_command_applied_at_state"] = dict(
            rows[1]["data"]["actual_command_applied_at_state"]
        )

        self.assert_rejected(
            records,
            "complete delayed command history disagrees.*fixed-grid",
        )

    def test_rejects_duplicate_and_incomplete_ids(self):
        duplicate = complete_records()
        duplicate.extend(complete_records())
        result = extract_velocity_lmpc_episodes(duplicate, config())
        self.assertEqual(len(result.episodes), 1)
        self.assertRegex(result.rejections[-1].reason, "duplicate")

        incomplete = complete_records()[:-2]
        self.assert_rejected(incomplete, "no terminal-dwell success")

    def test_rejects_non_y_direction_and_hazard_event(self):
        self.assert_rejected(
            complete_records(direction_x=0.1), "world \+/-Y"
        )

        records = complete_records()
        records[0]["data"]["release_dataset_direction_xy"] = [0.0, 2.0]
        self.assert_rejected(records, "unit world \+/-Y")

        records = complete_records()
        end = next(index for index, record in enumerate(records)
                   if record.get("name") == TERMINAL_DWELL_EVENT)
        records.insert(end, event("Workspace Boundary Safety Stop", {
            "time": 1.17,
        }))
        self.assert_rejected(records, "hazard event")

        records = complete_records()
        end = next(index for index, record in enumerate(records)
                   if record.get("name") == TERMINAL_DWELL_EVENT)
        records.insert(end, event(None, {
            "time": 1.17,
            "name": "emergency_stop",
        }))
        self.assert_rejected(records, "hazard event")

        records = complete_records()
        end = next(index for index, record in enumerate(records)
                   if record.get("name") == TERMINAL_DWELL_EVENT)
        records.insert(end, event(None, {
            "time": 1.17,
            "name": "battery_critical",
        }))
        self.assert_rejected(records, "hazard event")

    def test_rejects_diagonal_or_reversed_measured_release_axis(self):
        diagonal = math.sqrt(0.5)
        cases = (
            ([diagonal, diagonal], "signed alignment"),
            ([0.0, -1.0], "signed alignment"),
            ([0.0, 0.99], "finite unit direction"),
        )
        for measured_direction, pattern in cases:
            with self.subTest(measured_direction=measured_direction):
                self.assert_rejected(
                    complete_records(measured_direction=measured_direction),
                    pattern,
                )

    def test_rejects_missing_measured_release_axis(self):
        records = complete_records()
        records[0]["data"].pop(
            "release_dataset_measured_sensor_axis_world_xy"
        )

        self.assert_rejected(records, "measured_sensor_axis_world_xy")

    def test_rejects_stale_boundary_and_rejected_measurement_rows(self):
        for field, value, message in (
            ("state_age_s", 0.11, "stale/future"),
            ("xy_boundary_margin_m", 0.01, "boundary margin"),
            ("measurement_rejected", True, "rejected measurement"),
        ):
            records = complete_records()
            self.active_rows(records)[3]["data"][field] = value
            with self.subTest(field=field):
                self.assert_rejected(records, message)

    def test_rejects_inconsistent_observation_timestamp_and_state_age(self):
        records = complete_records()
        self.active_rows(records)[3]["data"]["time"] += 0.001
        self.assert_rejected(records, "inconsistent state/observation timing")

    def test_terminal_marker_cannot_bypass_full_xy_speed_constraint(self):
        records = complete_records()
        self.active_rows(records)[-1]["data"]["velocity_m_s"] = [0.1, 0.005, 0.0]
        self.assert_rejected(records, "terminal set")

    def test_rejects_velocity_hover_command_and_wrong_terminal_order(self):
        records = complete_records()
        self.active_rows(records)[3]["data"][
            "actual_commands_sent_since_previous_state"
        ] = [{"kind": "velocity_hover", "sent_at": 1.065, "sequence": 7}]
        self.assert_rejected(records, "velocity_hover")

        records = complete_records()
        final_record = self.active_rows(records)[-1]
        terminal_index = next(index for index, record in enumerate(records)
                              if record.get("name") == TERMINAL_DWELL_EVENT)
        terminal = records.pop(terminal_index)
        final_index = records.index(final_record)
        records.insert(final_index, terminal)
        self.assert_rejected(
            records,
            "state timestamp disagrees|terminal set|POSITION_HOLD|pending "
            "terminal_handoff",
        )

    def test_success_requires_actual_position_handoff_after_final_state(self):
        records = complete_records()
        final = self.active_rows(records)[-1]["data"]
        position = final["actual_commands_sent_since_previous_state"][0]
        final["actual_commands_sent_since_previous_state"] = [
            attitude_command(
                position["sent_at"], 0.0, sequence=position["sequence"]
            )
        ]
        final["release_dataset_command_owner"] = "attitude_coast"
        self.assert_rejected(records, "POSITION_HOLD position command")

    def test_position_handoff_target_must_equal_final_measured_position(self):
        mutations = (
            lambda command: command.__setitem__(
                "position_m", [0.1, 0.12, 1.0]
            ),
            lambda command: command.pop("position_m"),
            lambda command: command.__setitem__(
                "position_m", [0.1, float("inf"), 1.0]
            ),
        )
        for case, mutate in enumerate(mutations):
            records = complete_records()
            final = self.active_rows(records)[-1]["data"]
            mutate(final["actual_commands_sent_since_previous_state"][0])
            with self.subTest(case=case):
                self.assert_rejected(records, "POSITION_HOLD position command")

    def test_rejects_nonlevel_send_between_terminal_state_and_observation(self):
        for sent_at in (1.20, 1.2005):
            records = complete_records()
            rows = self.active_rows(records)
            final = rows[-1]["data"]
            position = final[
                "actual_commands_sent_since_previous_state"
            ][0]
            final["time"] = 1.201
            final["state_age_s"] = 0.001
            position["sent_at"] = 1.202
            position["sequence"] += 1
            final["actual_commands_sent_since_previous_state"] = [
                attitude_command(
                    sent_at,
                    10.0,
                    sequence=position["sequence"]-1,
                ),
                position,
            ]
            with self.subTest(sent_at=sent_at):
                self.assert_rejected(
                    records,
                    "between the terminal state and fresh-state observation",
                )

    def test_rejects_forged_terminal_gate_audit(self):
        for field, value, message in (
            ("complete", False, "complete is not true"),
            ("reason", "tracking", "reason is not"),
            ("violations", ["velocity_xy"], "violations is not empty"),
        ):
            records = complete_records()
            marker = next(
                record for record in records
                if record.get("name") == TERMINAL_DWELL_EVENT
            )
            marker["data"]["terminal_gate"][field] = value
            with self.subTest(field=field):
                self.assert_rejected(records, message)

    def test_terminal_marker_cannot_hide_nonlevel_pending_queue(self):
        records = complete_records()
        self.active_rows(records)[-2]["data"][
            "actual_commands_sent_since_previous_state"
        ][0]["roll_deg"] = 10.0
        self.assert_rejected(records, "full measured terminal set")

    def test_rejects_missing_release_queue_seed(self):
        records = complete_records()
        records[0]["data"].pop("release_pending_command_history")
        self.assert_rejected(records, "pending_command_history.*array")

    def test_rejects_reordered_release_queue_seed(self):
        records = complete_records()
        records[0]["data"]["release_pending_command_history"].reverse()
        self.assert_rejected(
            records,
            "pending command seed is not strictly ordered",
        )

    def test_nonterminal_episode_is_reported_without_blocking_later_success(self):
        bad = complete_records(episode_id="lb11-release-bad")
        close = next(
            record for record in bad
            if record.get("name") == TERMINAL_DWELL_EVENT
        )
        close["name"] = CLOSE_EVENT
        close["data"]["release_dataset_outcome"] = "safety_abort"

        good = json.loads(json.dumps(complete_records(
            episode_id="lb11-release-good"
        )))
        for record in good:
            data = record.get("data")
            if not isinstance(data, dict):
                continue
            for key in (
                "time", "state_time", "release_state_time",
                "terminal_state_time",
            ):
                if isinstance(data.get(key), (int, float)):
                    data[key] += 10.0
            for key in (
                "actual_command_applied_at_state",
                "release_command_effective_at_state",
            ):
                if isinstance(data.get(key), dict):
                    data[key]["sent_at"] += 10.0
            for key in (
                "actual_commands_sent_since_previous_state",
                "release_pending_command_history",
            ):
                batch = data.get(key)
                if not isinstance(batch, list):
                    continue
                for command in batch:
                    command["sent_at"] += 10.0
        result = extract_velocity_lmpc_episodes(bad+good, config())
        self.assertEqual(
            [episode.episode_id for episode in result.episodes],
            ["lb11-release-good"],
        )
        self.assertEqual(result.rejections[0].episode_id, "lb11-release-bad")
        self.assertIn("without a successful terminal handoff",
                      result.rejections[0].reason)

    def test_layout_must_match_delay(self):
        with self.assertRaisesRegex(ReplayValidationError, "state_dimension"):
            config(state_dimension=4)
        with self.assertRaisesRegex(
            ReplayValidationError, "exact delay identity"
        ):
            config(command_delay_s=None)
        with self.assertRaisesRegex(
            ReplayValidationError, "must be positive.*zero-delay"
        ):
            config(state_dimension=4, command_delay_s=0.0)


class FileAndCLITests(unittest.TestCase):
    def test_strict_loader_rejects_truncated_and_nonfinite_json(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"flight.json"
            path.write_text("[{", encoding="utf-8")
            with self.assertRaisesRegex(ReplayValidationError, "incomplete"):
                load_complete_flight_records(path)
            path.write_text('[{"data": NaN}]', encoding="utf-8")
            with self.assertRaisesRegex(ReplayValidationError, "non-finite"):
                load_complete_flight_records(path)

    def test_atomic_save_never_overwrites(self):
        result = extract_velocity_lmpc_episodes(complete_records(), config())
        artifact = build_safe_set_artifact(result)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"safe-set.json"
            save_new_safe_set_artifact(artifact, path)
            original = path.read_bytes()
            with self.assertRaisesRegex(ReplayValidationError, "overwrite"):
                save_new_safe_set_artifact(artifact, path)
            self.assertEqual(path.read_bytes(), original)
            loaded = VelocityLMPCSafeSet.load(path)
            self.assertEqual(len(loaded.episodes), 1)

    def test_cli_outputs_offline_report_and_refuses_second_write(self):
        with tempfile.TemporaryDirectory() as directory:
            flight = Path(directory)/"flight.json"
            output = Path(directory)/"safe-set.json"
            flight.write_text(
                json.dumps(complete_records()), encoding="utf-8"
            )
            argv = [
                "--input", str(flight),
                "--output", str(output),
                "--model-fingerprint", FINGERPRINT,
                "--state-dimension", "5",
                "--prediction-step-s", "0.02",
                "--command-delay-s", "0.04",
            ]
            stream = io.StringIO()
            with redirect_stdout(stream):
                self.assertEqual(main(argv), 0)
            report = json.loads(stream.getvalue())
            self.assertTrue(report["offline_only"])
            self.assertFalse(report["flight_commands_generated"])
            self.assertEqual(report["episode_count"], 1)

            stream = io.StringIO()
            with redirect_stdout(stream):
                self.assertEqual(main(argv), 2)
            rejection = json.loads(stream.getvalue())
            self.assertTrue(rejection["offline_only"])
            self.assertEqual(rejection["status"], "rejected")

    def test_cli_requires_explicit_delay_for_a_new_artifact(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            flight = directory/"flight.json"
            flight.write_text(
                json.dumps(complete_records()), encoding="utf-8"
            )
            stream = io.StringIO()
            with redirect_stdout(stream):
                self.assertEqual(main([
                    "--input", str(flight),
                    "--output", str(directory/"safe-set.json"),
                    "--model-fingerprint", FINGERPRINT,
                    "--state-dimension", "5",
                ]), 2)
            report = json.loads(stream.getvalue())
            self.assertRegex(report["reason"], "command-delay-s is required")

    def test_cli_extends_existing_artifact_contract_and_checks_step(self):
        custom_spec = StageCostSpec(effort_weight=0.0025)
        existing = VelocityLMPCSafeSet(
            state_dimension=5,
            command_dimension=1,
            prediction_step_s=0.02,
            command_delay_s=0.04,
            state_scales=(
                1.0,
                math.radians(10.0),
                math.radians(100.0),
                math.radians(10.0),
                math.radians(10.0),
            ),
            stage_cost_spec=custom_spec,
        )
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            flight = directory/"flight.json"
            existing_path = directory/"existing.json"
            output = directory/"extended.json"
            mismatch_output = directory/"mismatch.json"
            flight.write_text(
                json.dumps(complete_records()), encoding="utf-8"
            )
            save_new_safe_set_artifact(existing, existing_path)
            base_argv = [
                "--input", str(flight),
                "--model-fingerprint", FINGERPRINT,
                "--state-dimension", "5",
                "--existing-artifact", str(existing_path),
            ]
            stream = io.StringIO()
            with redirect_stdout(stream):
                self.assertEqual(
                    main([*base_argv, "--output", str(output)]), 0
                )
            loaded = VelocityLMPCSafeSet.load(output)
            self.assertEqual(loaded.stage_cost_spec, custom_spec)

            stream = io.StringIO()
            with redirect_stdout(stream):
                self.assertEqual(main([
                    *base_argv,
                    "--output", str(mismatch_output),
                    "--prediction-step-s", "0.03",
                ]), 2)
            report = json.loads(stream.getvalue())
            self.assertRegex(report["reason"], "does not match")

            stream = io.StringIO()
            with redirect_stdout(stream):
                self.assertEqual(main([
                    *base_argv,
                    "--output", str(mismatch_output),
                    "--command-delay-s", "0.03",
                ]), 2)
            report = json.loads(stream.getvalue())
            self.assertRegex(report["reason"], "does not match")


if __name__ == "__main__":
    unittest.main()
