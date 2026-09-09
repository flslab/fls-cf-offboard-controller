"""Strict offline validation of decision-grid release-to-rest LMPC episodes.

The flight loop records measured state and the attitude command that was
actually sent, but those events have a real post-observation computation/send
phase.  That raw phase is not silently folded into the identified attitude
delay.  This validator admits only records that have already been explicitly
resampled onto the command decision-time grid: every non-terminal state and
its newly sent action have the same timestamp, the complete delayed-command
queue obeys its fixed-grid successor, and the raw effective command inferred
from the lossless send history agrees with every logged value.  Run
``Interaction.velocity_lmpc_resample`` first; unmodified flight-loop records
still fail closed rather than create a phase-blind safe set.

``actual_command_applied_at_state`` is a modeled effective-input lookup from
the configured delay and actual send history, not a radio or motor hardware
acknowledgement.  This module has no commander, radio, or device imports and
every public report is explicitly offline-only.

Only complete world ``+/-Y`` episodes beginning at
``Potentiometer Release Coasting Started`` are considered.  A dedicated
``Release Dataset Terminal Dwell Complete`` marker is the successful close;
``Release Dataset Episode Closed`` records a rejected/non-terminal close.
The successful marker, the measured samples, the applied/pending command
timeline, and the actual position-handoff send must all agree.  The older
position-hold event is deliberately ignored: a controller transition is not
measured terminal-state evidence.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import dataclass, field
import json
import math
import numbers
import os
from pathlib import Path
import re
import tempfile
from typing import Sequence

import numpy as np

from Interaction.predictive_brake_handoff import (
    projected_tilt_from_attitude_command,
)
from Interaction.release_lmpc_terminal_gate import (
    classify_terminal_post_state_commands,
)
from Interaction.velocity_lmpc_safe_set import (
    LMPCContext,
    SafeSetLimits,
    SafeSetValidationError,
    STATE_ACTION_PHASE_CONTRACT,
    StageCostSpec,
    TERMINAL_OUTCOME,
    VelocityLMPCEpisode,
    VelocityLMPCSafeSet,
    VelocityLMPCSample,
)


START_EVENT = "Potentiometer Release Coasting Started"
TERMINAL_DWELL_EVENT = "Release Dataset Terminal Dwell Complete"
CLOSE_EVENT = "Release Dataset Episode Closed"
# Export the legacy name for callers that used the first draft, but never use
# it as an extraction boundary or as evidence of success.
LEGACY_HANDOFF_EVENT = "Translation Position Hold Resumed"

_HAZARD_EVENT_RE = re.compile(
    r"(?:safety|boundar|stale|locali[sz]ation|emergency|"
    r"battery[_ ]?(?:critical|abort|low))",
    re.IGNORECASE,
)
_MIN_DIRECTION_ALIGNMENT_DOT = 0.98


class ReplayValidationError(ValueError):
    """The flight log cannot safely contribute an LMPC episode."""


def _finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ReplayValidationError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ReplayValidationError(f"{name} must be a finite number")
    return result


def _vector(value, length, name):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ReplayValidationError(f"{name} must be a {length}-element array")
    return np.asarray([
        _finite_number(item, f"{name}[{index}]")
        for index, item in enumerate(value)
    ], dtype=float)


def _record_data(record, index):
    if not isinstance(record, dict):
        raise ReplayValidationError(f"record {index} must be an object")
    data = record.get("data")
    if not isinstance(data, dict):
        raise ReplayValidationError(f"record {index}.data must be an object")
    return data


def _episode_id(value, name):
    if not isinstance(value, str) or not value:
        raise ReplayValidationError(f"{name} must be a nonempty string")
    return value


def _bool_flag(data, key):
    value = data.get(key)
    if value is None:
        return False
    if type(value) is not bool:
        raise ReplayValidationError(f"{key} must be boolean when present")
    return value


@dataclass(frozen=True)
class VelocityLMPCReplayConfig:
    """Model-bound layout and hard gates for offline replay.

    ``state_dimension`` is the core state dimension.  It must be
    ``3 + max(1, ceil(command_delay_s / prediction_step_s))``.  The exact
    physical delay is mandatory: queue dimension alone cannot distinguish a
    fractional delay from a rounded whole-step delay.
    """

    model_fingerprint: str
    state_dimension: int
    prediction_step_s: float = 0.02
    command_delay_s: float | None = None
    direction_x_tolerance: float = 1e-6
    direction_norm_tolerance: float = 1e-6
    stage_cost_spec: StageCostSpec = field(default_factory=StageCostSpec)
    limits: SafeSetLimits = field(default_factory=SafeSetLimits)

    def __post_init__(self):
        if not isinstance(self.model_fingerprint, str):
            raise ReplayValidationError("model_fingerprint must be a string")
        if isinstance(self.state_dimension, bool) or not isinstance(
                self.state_dimension, numbers.Integral):
            raise ReplayValidationError("state_dimension must be an integer")
        object.__setattr__(self, "state_dimension", int(self.state_dimension))
        if self.state_dimension < 4:
            raise ReplayValidationError("state_dimension must be at least 4")
        for name in (
            "prediction_step_s", "direction_x_tolerance",
            "direction_norm_tolerance",
        ):
            value = _finite_number(getattr(self, name), name)
            object.__setattr__(self, name, value)
            if value <= 0:
                raise ReplayValidationError(f"{name} must be positive")
        if self.command_delay_s is None:
            raise ReplayValidationError(
                "command_delay_s is required for exact delay identity"
            )
        delay = _finite_number(self.command_delay_s, "command_delay_s")
        if delay <= 0:
            raise ReplayValidationError(
                "command_delay_s must be positive for strict decision-grid "
                "replay; zero-delay state-before-send ordering is unsupported"
            )
        object.__setattr__(self, "command_delay_s", delay)
        if not isinstance(self.limits, SafeSetLimits):
            raise ReplayValidationError("limits must be SafeSetLimits")
        if not isinstance(self.stage_cost_spec, StageCostSpec):
            raise ReplayValidationError(
                "stage_cost_spec must be StageCostSpec"
            )

        expected = 3+max(1, self.delay_steps)
        if self.state_dimension != expected:
            raise ReplayValidationError(
                "state_dimension does not match command delay and prediction "
                f"step (expected {expected})"
            )

    @property
    def delay_steps(self):
        ratio = self.command_delay_s/self.prediction_step_s
        return int(math.ceil(ratio-1e-12))

    @property
    def queue_slots(self):
        return max(1, self.delay_steps)

    @property
    def effective_command_delay_s(self):
        return self.command_delay_s

    @property
    def state_scales(self):
        return (
            1.0,
            math.radians(10.0),
            math.radians(100.0),
            *([math.radians(10.0)]*self.queue_slots),
        )


@dataclass(frozen=True)
class VelocityLMPCReplayRejection:
    episode_id: str | None
    reason: str
    start_index: int | None = None
    end_index: int | None = None
    offline_only: bool = True

    def to_dict(self):
        return {
            "offline_only": True,
            "episode_id": self.episode_id,
            "reason": self.reason,
            "start_index": self.start_index,
            "end_index": self.end_index,
        }


@dataclass(frozen=True)
class VelocityLMPCReplayResult:
    episodes: tuple[VelocityLMPCEpisode, ...]
    rejections: tuple[VelocityLMPCReplayRejection, ...]
    source_record_count: int
    model_fingerprint: str
    state_dimension: int
    prediction_step_s: float
    command_delay_s: float
    stage_cost_spec: StageCostSpec
    state_action_phase_contract: str = STATE_ACTION_PHASE_CONTRACT
    offline_only: bool = True
    flight_commands_generated: bool = False

    def to_dict(self):
        return {
            "offline_only": True,
            "flight_commands_generated": False,
            "source_record_count": self.source_record_count,
            "episode_count": len(self.episodes),
            "episode_ids": [episode.episode_id for episode in self.episodes],
            "rejected_episode_count": len(self.rejections),
            "rejections": [item.to_dict() for item in self.rejections],
            "model_fingerprint": self.model_fingerprint,
            "state_dimension": self.state_dimension,
            "prediction_step_s": self.prediction_step_s,
            "command_delay_s": self.command_delay_s,
            "state_action_phase_contract": self.state_action_phase_contract,
            "stage_cost_spec": self.stage_cost_spec.to_dict(),
        }


@dataclass(frozen=True)
class _Segment:
    episode_id: str
    direction: np.ndarray
    measured_sensor_axis_world_xy: np.ndarray
    start_index: int
    release_state_time: float
    terminal_dwell_index: int
    terminal_state_time: float
    terminal_gate: dict
    end_index: int
    observer_indices: tuple[int, ...]
    release_effective_command: dict
    release_pending_commands: tuple[dict, ...]


@dataclass(frozen=True)
class _Command:
    record_index: int
    sequence: int
    sent_at: float
    kind: str
    roll_deg: float | None = None
    pitch_deg: float | None = None
    yaw_rad: float | None = None


def _direction(value, config, name):
    raw = _vector(value, 2, name)
    norm = float(np.linalg.norm(raw))
    if (
        abs(norm-1.0) > config.direction_norm_tolerance
        or abs(raw[0]) > config.direction_x_tolerance
        or abs(abs(raw[1])-1.0) > config.direction_norm_tolerance
    ):
        raise ReplayValidationError(
            f"{name} is not a unit world +/-Y direction"
        )
    return np.asarray([0.0, float(np.sign(raw[1]))], dtype=float)


def _measured_sensor_axis(value, direction, config, name):
    measured = _vector(value, 2, name)
    norm = float(np.linalg.norm(measured))
    if abs(norm-1.0) > config.direction_norm_tolerance:
        raise ReplayValidationError(f"{name} must be a finite unit direction")
    signed_alignment = float(measured @ direction)
    if signed_alignment+1e-12 < _MIN_DIRECTION_ALIGNMENT_DOT:
        raise ReplayValidationError(
            f"{name} signed alignment with release_dataset_direction_xy must "
            f"be at least {_MIN_DIRECTION_ALIGNMENT_DOT:.2f}"
        )
    return measured


def _is_hazard_event(record):
    if record.get("type") != "events":
        return False
    names = [record.get("name")]
    data = record.get("data")
    if isinstance(data, dict):
        names.append(data.get("name"))
    return any(
        isinstance(name, str) and _HAZARD_EVENT_RE.search(name) is not None
        for name in names
    )


def _segments(records, config):
    active = None
    seen_starts = set()
    lingering_id = None
    segments = []
    rejections = []
    unknown_observer_ids = set()

    def reject(episode_id, reason, start_index=None, end_index=None):
        rejections.append(VelocityLMPCReplayRejection(
            episode_id=episode_id,
            reason=str(reason),
            start_index=start_index,
            end_index=end_index,
        ))

    def reject_active(reason, end_index):
        nonlocal active, lingering_id
        if active is not None:
            reject(
                active["episode_id"], reason,
                active["start_index"], end_index,
            )
            lingering_id = active["episode_id"]
            active = None

    for index, record in enumerate(records):
        if not isinstance(record, dict):
            raise ReplayValidationError(f"record {index} must be an object")
        record_type = record.get("type")
        name = record.get("name")
        if active is not None and _is_hazard_event(record):
            active["errors"].append(f"contains hazard event {name!r}")

        if record_type == "events" and name == START_EVENT:
            if active is not None:
                reject_active(
                    "episode is incomplete because another release started "
                    "before it closed",
                    index,
                )
            try:
                data = _record_data(record, index)
                episode_id = _episode_id(
                    data.get("release_dataset_episode_id"),
                    f"record {index} release_dataset_episode_id",
                )
                direction = _direction(
                    data.get("release_dataset_direction_xy"), config,
                    f"record {index} release_dataset_direction_xy",
                )
                measured_sensor_axis = _measured_sensor_axis(
                    data.get(
                        "release_dataset_measured_sensor_axis_world_xy"
                    ),
                    direction,
                    config,
                    f"record {index} "
                    "release_dataset_measured_sensor_axis_world_xy",
                )
                _finite_number(data.get("time"), f"record {index} event time")
                release_state_time = _finite_number(
                    data.get("release_state_time"),
                    f"record {index} release_state_time",
                )
                release_effective_command = data.get(
                    "release_command_effective_at_state"
                )
                release_pending_commands = data.get(
                    "release_pending_command_history"
                )
                if not isinstance(release_effective_command, dict):
                    raise ReplayValidationError(
                        f"record {index} release_command_effective_at_state "
                        "must be an object"
                    )
                if not isinstance(release_pending_commands, list):
                    raise ReplayValidationError(
                        f"record {index} release_pending_command_history "
                        "must be an array"
                    )
            except ReplayValidationError as error:
                reject(None, str(error), index, index)
                lingering_id = None
                continue
            errors = []
            if episode_id in seen_starts:
                errors.append(
                    f"duplicate release dataset episode ID: {episode_id}"
                )
            active = {
                "episode_id": episode_id,
                "direction": direction,
                "measured_sensor_axis": measured_sensor_axis,
                "start_index": index,
                "release_state_time": release_state_time,
                "terminal_dwell_index": None,
                "observer_indices": [],
                "errors": errors,
                "release_effective_command": release_effective_command,
                "release_pending_commands": tuple(release_pending_commands),
            }
            seen_starts.add(episode_id)
            lingering_id = None
            continue

        if record_type == "events" and name == TERMINAL_DWELL_EVENT:
            try:
                data = _record_data(record, index)
                episode_id = _episode_id(
                    data.get("release_dataset_episode_id"),
                    f"record {index} release_dataset_episode_id",
                )
                _finite_number(data.get("time"), f"record {index} event time")
                outcome = data.get("release_dataset_outcome")
                terminal_gate = data.get("terminal_gate")
                terminal_state_time = _finite_number(
                    data.get("terminal_state_time"),
                    f"record {index} terminal_state_time",
                )
            except ReplayValidationError as error:
                reject(None, str(error), None, index)
                continue
            if active is None or episode_id != active["episode_id"]:
                reject(
                    episode_id,
                    "terminal-dwell marker references an incomplete/unknown "
                    "episode",
                    None if active is None else active["start_index"],
                    index,
                )
            elif active["terminal_dwell_index"] is not None:
                active["errors"].append("duplicate terminal-dwell marker")
            else:
                active["terminal_dwell_index"] = index
                if outcome != TERMINAL_OUTCOME:
                    active["errors"].append(
                        "terminal-dwell event does not declare "
                        "terminal_handoff"
                    )
                if not isinstance(terminal_gate, dict):
                    active["errors"].append(
                        "terminal-dwell event has no terminal_gate audit"
                    )
                else:
                    if terminal_gate.get("complete") is not True:
                        active["errors"].append(
                            "terminal_gate.complete is not true"
                        )
                    if terminal_gate.get("reason") != "terminal_dwell_complete":
                        active["errors"].append(
                            "terminal_gate.reason is not terminal_dwell_complete"
                        )
                    if terminal_gate.get("violations") != []:
                        active["errors"].append(
                            "terminal_gate.violations is not empty"
                        )
                if len(active["observer_indices"]) < 2:
                    active["errors"].append(
                        "episode has fewer than two observer rows"
                    )
                if active["errors"]:
                    reject_active("; ".join(active["errors"]), index)
                else:
                    segments.append(_Segment(
                        episode_id=episode_id,
                        direction=active["direction"],
                        measured_sensor_axis_world_xy=(
                            active["measured_sensor_axis"]
                        ),
                        start_index=active["start_index"],
                        release_state_time=active["release_state_time"],
                        terminal_dwell_index=index,
                        terminal_state_time=terminal_state_time,
                        terminal_gate=terminal_gate,
                        end_index=index,
                        observer_indices=tuple(active["observer_indices"]),
                        release_effective_command=(
                            active["release_effective_command"]
                        ),
                        release_pending_commands=(
                            active["release_pending_commands"]
                        ),
                    ))
                    active = None
                    lingering_id = episode_id
            continue

        if record_type == "events" and name == CLOSE_EVENT:
            try:
                data = _record_data(record, index)
                episode_id = _episode_id(
                    data.get("release_dataset_episode_id"),
                    f"record {index} release_dataset_episode_id",
                )
                _finite_number(data.get("time"), f"record {index} event time")
                outcome = data.get("release_dataset_outcome")
                if not isinstance(outcome, str) or not outcome:
                    raise ReplayValidationError(
                        f"record {index} release_dataset_outcome must be a "
                        "nonempty string"
                    )
            except ReplayValidationError as error:
                reject(None, str(error), None, index)
                continue
            if active is None or episode_id != active["episode_id"]:
                reject(
                    episode_id,
                    "close marker references an incomplete/unknown episode",
                    None if active is None else active["start_index"],
                    index,
                )
                continue
            reason = data.get("reason")
            active["errors"].append(
                f"episode closed without a successful terminal handoff: "
                f"outcome {outcome!r}"
                + (" ("+str(reason)+")" if reason else "")
            )
            reject_active("; ".join(active["errors"]), index)
            continue

        if record_type == "wrench_observer":
            data = _record_data(record, index)
            row_id = data.get("release_dataset_episode_id")
            if active is not None:
                if row_id != active["episode_id"]:
                    active["errors"].append(
                        f"observer record {index} has missing or mismatched "
                        "episode ID"
                    )
                    continue
                try:
                    row_direction = _direction(
                        data.get("release_dataset_direction_xy"), config,
                        f"record {index} release_dataset_direction_xy",
                    )
                except ReplayValidationError as error:
                    active["errors"].append(str(error))
                    continue
                if not np.array_equal(row_direction, active["direction"]):
                    active["errors"].append(
                        f"observer record {index} changes episode direction"
                    )
                    continue
                active["observer_indices"].append(index)
            elif row_id is not None:
                # The flight loop intentionally retains the completed ID until
                # the next release.  Any other non-null ID means the supposedly
                # complete file began or ended in the middle of an episode.
                if lingering_id is None or row_id != lingering_id:
                    if row_id not in unknown_observer_ids:
                        reject(
                            str(row_id),
                            f"observer record {index} references an "
                            "incomplete/unknown episode",
                            None,
                            index,
                        )
                        unknown_observer_ids.add(row_id)

    if active is not None:
        reject_active(
            "episode has no terminal-dwell success or rejected-close marker",
            len(records)-1 if records else None,
        )
    return tuple(segments), tuple(rejections)


def _logged_command(value, *, record_index, field_name):
    name = f"record {record_index} {field_name}"
    if not isinstance(value, dict):
        raise ReplayValidationError(f"{name} must be an object")
    kind = value.get("kind")
    if not isinstance(kind, str) or not kind:
        raise ReplayValidationError(f"{name}.kind must be a nonempty string")
    sent_at = _finite_number(value.get("sent_at"), f"{name}.sent_at")
    sequence = value.get("sequence")
    if (
        isinstance(sequence, bool)
        or not isinstance(sequence, numbers.Integral)
        or int(sequence) <= 0
    ):
        raise ReplayValidationError(f"{name}.sequence must be a positive integer")
    sequence = int(sequence)
    if kind == "attitude_zdistance":
        return _Command(
            record_index=record_index,
            sequence=sequence,
            sent_at=sent_at,
            kind=kind,
            roll_deg=_finite_number(value.get("roll_deg"), f"{name}.roll_deg"),
            pitch_deg=_finite_number(
                value.get("pitch_deg"), f"{name}.pitch_deg"
            ),
            yaw_rad=math.radians(_finite_number(
                value.get("yaw_deg"), f"{name}.yaw_deg"
            )),
        )
    return _Command(
        record_index=record_index,
        sequence=sequence,
        sent_at=sent_at,
        kind=kind,
    )


def _project_command(command, direction):
    if command.kind != "attitude_zdistance":
        raise ReplayValidationError(
            f"record {command.record_index} uses non-attitude command kind "
            f"{command.kind!r} where an attitude command is required"
        )
    return float(projected_tilt_from_attitude_command(
        command.roll_deg,
        command.pitch_deg,
        command.yaw_rad,
        direction,
    ))


def _orthogonal_command(command, direction):
    if command.kind != "attitude_zdistance":
        raise ReplayValidationError(
            f"record {command.record_index} uses non-attitude command kind "
            f"{command.kind!r} where an attitude command is required"
        )
    return float(projected_tilt_from_attitude_command(
        command.roll_deg,
        command.pitch_deg,
        command.yaw_rad-math.pi/2.0,
        direction,
    ))


def _same_command(first, second):
    return bool(
        first.sequence == second.sequence
        and math.isclose(
            first.sent_at, second.sent_at, rel_tol=0.0, abs_tol=1e-9
        )
        and first.kind == second.kind
        and first.roll_deg == second.roll_deg
        and first.pitch_deg == second.pitch_deg
        and first.yaw_rad == second.yaw_rad
    )


def _observer_commands(record, index):
    data = _record_data(record, index)
    applied = _logged_command(
        data.get("actual_command_applied_at_state"),
        record_index=index,
        field_name="actual_command_applied_at_state",
    )
    batch = data.get("actual_commands_sent_since_previous_state")
    if not isinstance(batch, list):
        raise ReplayValidationError(
            f"record {index} actual_commands_sent_since_previous_state "
            "must be an array"
        )
    sent = tuple(
        _logged_command(
            value,
            record_index=index,
            field_name=(
                f"actual_commands_sent_since_previous_state[{batch_index}]"
            ),
        )
        for batch_index, value in enumerate(batch)
    )
    return applied, sent


def _sent_command_history(records, segment, config):
    """Flatten lossless command batches into one strict global timeline."""
    commands = []
    last_stamp = None
    last_sequence = None
    seen_sequences = {}

    def append(command):
        nonlocal last_stamp, last_sequence
        duplicate = seen_sequences.get(command.sequence)
        if duplicate is not None:
            if (
                duplicate.sent_at != command.sent_at
                or duplicate.kind != command.kind
                or duplicate.roll_deg != command.roll_deg
                or duplicate.pitch_deg != command.pitch_deg
                or duplicate.yaw_rad != command.yaw_rad
            ):
                raise ReplayValidationError(
                    "same actual command sequence has inconsistent payloads"
                )
            return
        if last_stamp is not None and command.sent_at <= last_stamp:
            raise ReplayValidationError(
                "actual command timestamps are duplicate or non-monotonic "
                f"at observer record {command.record_index}"
            )
        if (
            last_stamp is not None
            and command.sent_at-last_stamp
            > config.limits.max_sample_dt_s+1e-12
        ):
            raise ReplayValidationError(
                "actual command timeline contains a timestamp gap of "
                f"{command.sent_at-last_stamp:.6f}s at observer record "
                f"{command.record_index}"
            )
        if last_sequence is not None and command.sequence != last_sequence+1:
            raise ReplayValidationError(
                "actual command sequence is duplicated, out of order, or "
                f"incomplete at observer record {command.record_index}"
            )
        last_stamp = command.sent_at
        last_sequence = command.sequence
        seen_sequences[command.sequence] = command
        commands.append(command)

    release_effective_command = _logged_command(
        segment.release_effective_command,
        record_index=segment.start_index,
        field_name="release_command_effective_at_state",
    )
    release_pending_commands = tuple(
        _logged_command(
            value,
            record_index=segment.start_index,
            field_name=f"release_pending_command_history[{seed_index}]",
        )
        for seed_index, value in enumerate(segment.release_pending_commands)
    )
    for first, second in zip(
        release_pending_commands, release_pending_commands[1:]
    ):
        if (
            second.sequence != first.sequence+1
            or second.sent_at <= first.sent_at
        ):
            raise ReplayValidationError(
                "release pending command seed is not strictly ordered and "
                "sequence-complete"
            )
    seed_commands = (
        release_effective_command,
        *release_pending_commands,
    )
    for command in seed_commands:
        append(command)

    for index in segment.observer_indices:
        data = _record_data(records[index], index)
        raw = data.get("actual_commands_sent_since_previous_state")
        if not isinstance(raw, list):
            raise ReplayValidationError(
                f"record {index} actual_commands_sent_since_previous_state "
                "must be an array"
            )
        for batch_index, value in enumerate(raw):
            command = _logged_command(
                value,
                record_index=index,
                field_name=(
                    "actual_commands_sent_since_previous_state"
                    f"[{batch_index}]"
                ),
            )
            append(command)
    if not commands:
        raise ReplayValidationError(
            f"episode {segment.episode_id} has no actual command history"
        )
    return tuple(commands)


def _command_at_or_before(commands, stamp):
    timestamps = [item.sent_at for item in commands]
    position = bisect_right(timestamps, stamp+1e-12)-1
    if position < 0:
        raise ReplayValidationError(
            "actual sent command history does not cover the LMPC delay queue"
        )
    return commands[position]


def _queue_for_sample(commands, *, sample_stamp, config):
    if config.delay_steps == 0:
        previous = _command_at_or_before(commands, sample_stamp)
        if sample_stamp-previous.sent_at > (
                config.limits.max_sample_dt_s+1e-12):
            raise ReplayValidationError(
                "previous actual sent command is separated by a logging gap"
            )
        return (previous,)

    result = []
    delay = config.effective_command_delay_s
    for offset in range(config.delay_steps):
        effective_time = (
            sample_stamp-delay+offset*config.prediction_step_s
        )
        selected = _command_at_or_before(commands, effective_time)
        if effective_time-selected.sent_at > (
                config.limits.max_sample_dt_s+1e-12):
            raise ReplayValidationError(
                "actual sent command history contains a delay-queue gap"
            )
        result.append(selected)
    return tuple(result)


def _observer_values(record, index, segment, config):
    data = _record_data(record, index)
    position = _vector(
        data.get("position_m"), 3, f"record {index} position_m"
    )
    velocity = _vector(
        data.get("velocity_m_s"), 3, f"record {index} velocity_m_s"
    )
    rpy = _vector(
        data.get("orientation_rpy_rad"), 3,
        f"record {index} orientation_rpy_rad",
    )
    rates = _vector(
        data.get("angular_velocity_rad_s"), 3,
        f"record {index} angular_velocity_rad_s",
    )
    row_time = _finite_number(
        data.get("time"), f"record {index} time"
    )
    state_time = _finite_number(
        data.get("state_time"), f"record {index} state_time"
    )
    state_age = _finite_number(
        data.get("state_age_s"), f"record {index} state_age_s"
    )
    skew = _finite_number(
        data.get("state_group_skew_s"),
        f"record {index} state_group_skew_s",
    )
    boundary = _finite_number(
        data.get("xy_boundary_margin_m"),
        f"record {index} xy_boundary_margin_m",
    )
    battery = _finite_number(
        data.get("battery_voltage_V"),
        f"record {index} battery_voltage_V",
    )
    if state_age < 0 or state_age > config.limits.max_state_age_s:
        raise ReplayValidationError(
            f"observer record {index} has stale/future state telemetry"
        )
    if not math.isclose(
        row_time-state_time, state_age, rel_tol=0.0, abs_tol=1e-9
    ):
        raise ReplayValidationError(
            f"observer record {index} has inconsistent state/observation "
            "timing"
        )
    if skew < 0 or skew > config.limits.max_state_group_skew_s:
        raise ReplayValidationError(
            f"observer record {index} has excessive state-group skew"
        )
    if boundary < config.limits.min_path_boundary_margin_m:
        raise ReplayValidationError(
            f"observer record {index} violates the XY boundary margin"
        )
    if battery <= 0:
        raise ReplayValidationError(
            f"observer record {index} has invalid battery voltage"
        )
    for flag in (
        "safety_violation", "boundary_violation", "localization_stale",
        "localization_invalid", "emergency_stop",
    ):
        if _bool_flag(data, flag):
            raise ReplayValidationError(
                f"observer record {index} sets {flag}"
            )
    if _bool_flag(data, "measurement_rejected"):
        raise ReplayValidationError(
            f"observer record {index} contains a rejected measurement"
        )

    direction_sign = float(segment.direction[1])
    yaw = float(rpy[2])
    cosine, sine = math.cos(yaw), math.sin(yaw)
    if abs(abs(float(rpy[1]))-math.pi/2.0) <= 1e-6:
        raise ReplayValidationError(
            f"observer record {index} has singular pitch"
        )
    projected_tilt = math.atan(
        -direction_sign*(
            cosine*math.tan(float(rpy[0]))/math.cos(float(rpy[1]))
            + sine*math.tan(float(rpy[1]))
        )
    )
    projected_rate = -direction_sign*(
        cosine*float(rates[0])+sine*float(rates[1])
    )
    cross_direction = np.asarray([-direction_sign, 0.0])
    return {
        "position": position,
        "velocity": velocity,
        "rpy": rpy,
        "rates": rates,
        "row_time": row_time,
        "state_time": state_time,
        "state_age": state_age,
        "skew": skew,
        "boundary": boundary,
        "battery": battery,
        "aligned_velocity": float(velocity[:2]@segment.direction),
        "cross_velocity": float(velocity[:2]@cross_direction),
        "projected_tilt": float(projected_tilt),
        "projected_rate": float(projected_rate),
    }


def _sample_in_terminal_set(sample, limits):
    """Mirror the public safe-set terminal contract without private calls."""
    return bool(
        math.hypot(
            sample.aligned_velocity_m_s,
            sample.cross_velocity_m_s,
        ) <= limits.terminal_velocity_tolerance_m_s
        and abs(sample.roll_rad) <= limits.terminal_tilt_tolerance_rad
        and abs(sample.pitch_rad) <= limits.terminal_tilt_tolerance_rad
        and abs(sample.roll_rate_rad_s)
        <= limits.terminal_rate_tolerance_rad_s
        and abs(sample.pitch_rate_rad_s)
        <= limits.terminal_rate_tolerance_rad_s
        and max(abs(value) for value in sample.command)
        <= limits.terminal_command_tolerance
    )


def _is_level_attitude_command(command, limits):
    return bool(
        command.kind == "attitude_zdistance"
        and math.radians(max(
            abs(command.roll_deg), abs(command.pitch_deg)
        )) <= limits.terminal_command_tolerance+1e-12
    )


def _extract_segment(records, segment, config):
    if segment.observer_indices[-1] >= segment.terminal_dwell_index:
        raise ReplayValidationError(
            "terminal-dwell event must follow the final measured observer row"
        )
    commands = _sent_command_history(records, segment, config)
    rows = []
    previous_state_time = None
    previous_send_time = None
    release_position = None
    terminal_handoff_position = None
    modeled_queue_commands = None
    previous_action_command = None

    for row_number, index in enumerate(segment.observer_indices):
        final = row_number == len(segment.observer_indices)-1
        values = _observer_values(records[index], index, segment, config)
        applied, sent_batch = _observer_commands(records[index], index)
        row_data = _record_data(records[index], index)
        row_terminal_gate = row_data.get("release_dataset_terminal_gate")
        if not isinstance(row_terminal_gate, dict):
            raise ReplayValidationError(
                f"observer record {index} has no terminal-gate audit"
            )
        pending_outcome = row_data.get("release_dataset_pending_outcome")
        if final:
            if pending_outcome != TERMINAL_OUTCOME:
                raise ReplayValidationError(
                    f"final observer record {index} does not declare the "
                    "pending terminal_handoff outcome"
                )
            if row_terminal_gate != segment.terminal_gate:
                raise ReplayValidationError(
                    f"final observer record {index} terminal-gate audit "
                    "disagrees with the success event"
                )
            if not math.isclose(
                values["state_time"], segment.terminal_state_time,
                rel_tol=0.0, abs_tol=1e-9,
            ):
                raise ReplayValidationError(
                    f"final observer record {index} state timestamp disagrees "
                    "with the terminal-dwell event"
                )
        elif pending_outcome is not None:
            raise ReplayValidationError(
                f"non-final observer record {index} unexpectedly declares a "
                "pending dataset outcome"
            )
        if row_number == 0 and not math.isclose(
            values["state_time"], segment.release_state_time,
            rel_tol=0.0, abs_tol=1e-9,
        ):
            raise ReplayValidationError(
                f"first observer record {index} state timestamp disagrees "
                "with the release event"
            )
        if applied.kind == "velocity_hover" or any(
                command.kind == "velocity_hover" for command in sent_batch):
            raise ReplayValidationError(
                f"observer record {index} contains unsupported velocity_hover "
                "command ownership"
            )
        sent_after_state = tuple(
            command for command in sent_batch
            if command.sent_at >= values["row_time"]
        )
        raw_batch = row_data["actual_commands_sent_since_previous_state"]
        raw_sent_after_state = tuple(
            raw for raw, command in zip(raw_batch, sent_batch)
            if command.sent_at >= values["row_time"]
        )
        projected_applied = _project_command(applied, segment.direction)
        if final:
            pre_observation_commands = tuple(
                command for command in commands
                if (
                    values["state_time"]-1e-12 <= command.sent_at
                    < values["row_time"]
                )
            )
            if not all(
                _is_level_attitude_command(command, config.limits)
                for command in pre_observation_commands
            ):
                raise ReplayValidationError(
                    f"final observer record {index} has a non-level or "
                    "non-attitude command between the terminal state and "
                    "fresh-state observation"
                )
            classification = classify_terminal_post_state_commands(
                raw_sent_after_state,
                row_data.get("release_dataset_command_owner"),
                config.limits,
                terminal_position_m=tuple(
                    float(value) for value in values["position"]
                ),
            )
            if classification != "position_handoff":
                raise ReplayValidationError(
                    f"final observer record {index} must send the actual "
                    "POSITION_HOLD position command after the terminal state"
                )
            terminal_handoff_position = tuple(
                float(value)
                for value in raw_sent_after_state[-1]["position_m"]
            )
            sent_after = sent_after_state[0]
            projected_command = projected_applied
            sample_command = applied
        else:
            if len(sent_after_state) != 1:
                raise ReplayValidationError(
                    f"observer record {index} must contain exactly one "
                    "unambiguous command sent after the fresh state"
                )
            sent_after = sent_after_state[0]
            if sent_after.kind != "attitude_zdistance":
                raise ReplayValidationError(
                    f"non-final observer record {index} must send an "
                    "attitude_zdistance command after the state"
                )
            projected_command = _project_command(sent_after, segment.direction)
            sample_command = sent_after

        orthogonal_command = _orthogonal_command(
            sample_command, segment.direction
        )

        if (
            not final
            and not math.isclose(
                sent_after.sent_at,
                values["state_time"],
                rel_tol=0.0,
                abs_tol=1e-9,
            )
        ):
            raise ReplayValidationError(
                f"observer record {index} is not on the LMPC decision-time "
                "grid: the new attitude command timestamp must equal the "
                "sample state timestamp after explicit resampling"
            )

        if applied.sent_at > values["state_time"]+1e-9:
            raise ReplayValidationError(
                f"observer record {index} claims a command was applied before "
                "it was sent"
            )
        if sent_after.sent_at <= applied.sent_at:
            raise ReplayValidationError(
                f"observer record {index} command ordering is not causal"
            )
        if previous_send_time is not None:
            command_dt = sent_after.sent_at-previous_send_time
            state_dt = values["state_time"]-previous_state_time
            intervening_commands = tuple(
                command for command in commands
                if (
                    command.sent_at > previous_send_time+1e-12
                    and command.sent_at < values["state_time"]-1e-12
                )
            )
            if intervening_commands:
                raise ReplayValidationError(
                    f"observer record {index} contains multiple flight "
                    "commands within one fixed prediction step"
                )
            if previous_send_time >= values["state_time"]-1e-12:
                raise ReplayValidationError(
                    f"observer record {index} previous command was not sent "
                    "before the next fresh state"
                )
            if not (
                config.limits.min_sample_dt_s
                <= command_dt
                <= config.limits.max_sample_dt_s
            ):
                raise ReplayValidationError(
                    f"episode {segment.episode_id} has a command timestamp gap "
                    f"of {command_dt:.6f}s"
                )
            if not (
                config.limits.min_sample_dt_s
                <= state_dt
                <= config.limits.max_sample_dt_s
            ):
                raise ReplayValidationError(
                    f"episode {segment.episode_id} has a state timestamp gap "
                    f"of {state_dt:.6f}s"
                )
        previous_send_time = sent_after.sent_at
        previous_state_time = values["state_time"]
        if release_position is None:
            release_position = values["position"][:2].copy()
        raw_queue_commands = _queue_for_sample(
            commands,
            sample_stamp=values["state_time"],
            config=config,
        )
        if not _same_command(raw_queue_commands[0], applied):
            raise ReplayValidationError(
                f"observer record {index} actual_command_applied_at_state "
                "disagrees with the independently reconstructed delayed "
                "send history"
            )

        # The safe-set state lives on a fixed decision-time grid, so its
        # delayed-input memory has the exact discrete successor
        # ``q[k+1] = shift(q[k], u[k])``.  Preserve complete command objects
        # until after validating the applied input; scalar projections alone
        # cannot detect a forged sequence/timestamp with the same tilt.
        if modeled_queue_commands is None:
            modeled_queue_commands = raw_queue_commands
            release_effective_command = _logged_command(
                segment.release_effective_command,
                record_index=segment.start_index,
                field_name="release_command_effective_at_state",
            )
            if not _same_command(
                release_effective_command, modeled_queue_commands[0]
            ):
                raise ReplayValidationError(
                    "release command effective at the first state disagrees "
                    "with the complete delayed send history"
                )
        else:
            modeled_queue_commands = (
                *modeled_queue_commands[1:], previous_action_command,
            )
        if (
            len(raw_queue_commands) != len(modeled_queue_commands)
            or any(
                not _same_command(raw, modeled)
                for raw, modeled in zip(
                    raw_queue_commands, modeled_queue_commands
                )
            )
        ):
            raise ReplayValidationError(
                f"observer record {index} complete delayed command history "
                "disagrees with the fixed-grid command-memory successor; "
                "raw post-observation state/command phase must be resampled "
                "onto the decision-time grid before admission"
            )
        queue = tuple(
            _project_command(command, segment.direction)
            for command in modeled_queue_commands
        )
        orthogonal_queue = tuple(
            _orthogonal_command(command, segment.direction)
            for command in modeled_queue_commands
        )
        state = (
            values["aligned_velocity"],
            values["projected_tilt"],
            values["projected_rate"],
            *queue,
        )
        if len(state) != config.state_dimension:
            raise ReplayValidationError(
                "constructed state does not match the configured core dimension"
            )
        aligned_position = float(
            (values["position"][:2]-release_position)@segment.direction
        )
        rows.append((
            index, values["state_time"], projected_command, state,
            aligned_position, orthogonal_command, orthogonal_queue, values
        ))
        previous_action_command = sample_command

    samples = []
    for row_number, row in enumerate(rows):
        (
            index, send_time, command, state, position,
            orthogonal_command, orthogonal_queue, values,
        ) = row
        dt_s = (
            0.0 if row_number == len(rows)-1
            else rows[row_number+1][1]-send_time
        )
        samples.append(VelocityLMPCSample(
            state=tuple(float(item) for item in state),
            command=(float(command),),
            dt_s=float(dt_s),
            aligned_velocity_m_s=values["aligned_velocity"],
            cross_velocity_m_s=values["cross_velocity"],
            projected_tilt_rad=values["projected_tilt"],
            projected_tilt_rate_rad_s=values["projected_rate"],
            orthogonal_command_rad=float(orthogonal_command),
            pending_orthogonal_commands_rad=tuple(
                float(item) for item in orthogonal_queue
            ),
            position_m=tuple(float(value) for value in values["position"]),
            aligned_position_m=position,
            roll_rad=float(values["rpy"][0]),
            pitch_rad=float(values["rpy"][1]),
            roll_rate_rad_s=float(values["rates"][0]),
            pitch_rate_rad_s=float(values["rates"][1]),
            boundary_margin_m=values["boundary"],
            state_age_s=values["state_age"],
            state_group_skew_s=values["skew"],
            safety_violation=False,
        ))

    first = rows[0][7]
    if first["aligned_velocity"] <= 0:
        raise ReplayValidationError(
            f"episode {segment.episode_id} does not start with positive "
            "release-aligned speed"
        )
    context = LMPCContext(
        direction_sign=int(np.sign(segment.direction[1])),
        initial_speed_m_s=first["aligned_velocity"],
        cross_speed_m_s=first["cross_velocity"],
        roll_rad=float(first["rpy"][0]),
        pitch_rad=float(first["rpy"][1]),
        roll_rate_rad_s=float(first["rates"][0]),
        pitch_rate_rad_s=float(first["rates"][1]),
        boundary_margin_m=first["boundary"],
        battery_voltage_v=first["battery"],
        model_fingerprint=config.model_fingerprint,
    )
    episode = VelocityLMPCEpisode.from_samples(
        episode_id=segment.episode_id,
        context=context,
        outcome=TERMINAL_OUTCOME,
        terminal_handoff_position_m=terminal_handoff_position,
        samples=tuple(samples),
        stage_cost_spec=config.stage_cost_spec,
    )

    # A terminal marker only identifies a candidate.  Re-run the complete
    # admission contract now so callers cannot accidentally persist an episode
    # that merely *claims* success.
    validator = VelocityLMPCSafeSet(
        state_dimension=config.state_dimension,
        command_dimension=1,
        prediction_step_s=config.prediction_step_s,
        command_delay_s=config.effective_command_delay_s,
        state_scales=config.state_scales,
        stage_cost_spec=config.stage_cost_spec,
        limits=config.limits,
    )
    try:
        validator.validate_episode(episode)
    except SafeSetValidationError as error:
        raise ReplayValidationError(
            f"episode {segment.episode_id} failed safe-set admission: {error}"
        ) from error
    terminal_with_level_queue = [
        bool(
            _sample_in_terminal_set(sample, config.limits)
            and max(abs(value) for value in sample.state[3:])
            <= config.limits.terminal_command_tolerance
        )
        for sample in episode.samples
    ]
    if not terminal_with_level_queue[-1]:
        raise ReplayValidationError(
            f"episode {segment.episode_id} final delay queue is not level"
        )
    terminal_start = len(terminal_with_level_queue)-1
    while (
        terminal_start > 0
        and terminal_with_level_queue[terminal_start-1]
    ):
        terminal_start -= 1
    queue_safe_dwell = sum(
        sample.dt_s for sample in episode.samples[terminal_start:-1]
    )
    if queue_safe_dwell+1e-12 < config.limits.terminal_dwell_s:
        raise ReplayValidationError(
            f"episode {segment.episode_id} terminal dwell does not keep every "
            "pending delay-queue command level"
        )
    return episode


def extract_velocity_lmpc_episodes(records: Sequence[dict], config):
    """Purely transform one complete flight-record array into safe episodes."""
    if not isinstance(config, VelocityLMPCReplayConfig):
        raise ReplayValidationError("config must be VelocityLMPCReplayConfig")
    if not isinstance(records, (list, tuple)):
        raise ReplayValidationError("flight records must be a complete array")
    records = tuple(records)
    segments, parse_rejections = _segments(records, config)
    episodes = []
    rejections = list(parse_rejections)
    for segment in segments:
        try:
            episodes.append(_extract_segment(records, segment, config))
        except (ReplayValidationError, SafeSetValidationError) as error:
            rejections.append(VelocityLMPCReplayRejection(
                episode_id=segment.episode_id,
                reason=str(error),
                start_index=segment.start_index,
                end_index=segment.end_index,
            ))
    return VelocityLMPCReplayResult(
        episodes=tuple(episodes),
        rejections=tuple(rejections),
        source_record_count=len(records),
        model_fingerprint=config.model_fingerprint,
        state_dimension=config.state_dimension,
        prediction_step_s=config.prediction_step_s,
        command_delay_s=config.effective_command_delay_s,
        stage_cost_spec=config.stage_cost_spec,
    )


# Short alias for programmatic callers.
extract_episodes = extract_velocity_lmpc_episodes


def build_safe_set_artifact(result, *, existing=None, state_scales=None,
                            limits=None):
    """Add extracted episodes to a compatible in-memory safe-set artifact."""
    if not isinstance(result, VelocityLMPCReplayResult):
        raise ReplayValidationError("result must be VelocityLMPCReplayResult")
    if existing is None:
        scales = tuple(state_scales or (
            1.0, math.radians(10.0), math.radians(100.0),
            *([math.radians(10.0)]*(result.state_dimension-3)),
        ))
        artifact = VelocityLMPCSafeSet(
            state_dimension=result.state_dimension,
            command_dimension=1,
            prediction_step_s=result.prediction_step_s,
            command_delay_s=result.command_delay_s,
            state_scales=scales,
            stage_cost_spec=result.stage_cost_spec,
            limits=limits or SafeSetLimits(),
        )
    else:
        if not isinstance(existing, VelocityLMPCSafeSet):
            raise ReplayValidationError(
                "existing artifact must be VelocityLMPCSafeSet"
            )
        if (
            existing.state_dimension != result.state_dimension
            or existing.command_dimension != 1
            or existing.aligned_velocity_state_index != 0
            or not math.isclose(
                existing.prediction_step_s,
                result.prediction_step_s,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
            or not math.isclose(
                existing.command_delay_s,
                result.command_delay_s,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
            or existing.stage_cost_spec != result.stage_cost_spec
        ):
            raise ReplayValidationError(
                "existing artifact state/command contract is incompatible"
            )
        # Clone through the strict schema so a failure cannot partially mutate
        # the caller's existing in-memory artifact.
        artifact = VelocityLMPCSafeSet.from_dict(existing.to_dict())
    try:
        for episode in result.episodes:
            artifact.add_episode(episode)
    except SafeSetValidationError as error:
        raise ReplayValidationError(
            "could not add extracted episode to safe set: "+str(error)
        ) from error
    return artifact


def save_new_safe_set_artifact(artifact, output_path):
    """Atomically create ``output_path`` and never replace an existing file."""
    if not isinstance(artifact, VelocityLMPCSafeSet):
        raise ReplayValidationError("artifact must be VelocityLMPCSafeSet")
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=path.name+".", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(artifact.dumps())
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, path)
        except FileExistsError as error:
            raise ReplayValidationError(
                f"refusing to overwrite existing output: {path}"
            ) from error
        try:
            directory_fd = os.open(path.parent, os.O_RDONLY)
        except OSError:
            directory_fd = None
        if directory_fd is not None:
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return path


def _json_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ReplayValidationError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _json_constant(value):
    raise ReplayValidationError(f"non-finite JSON constant: {value}")


def load_complete_flight_records(path):
    """Strictly load one closed JSON array; truncated logs are rejected."""
    try:
        records = json.loads(
            Path(path).read_text(encoding="utf-8"),
            object_pairs_hook=_json_object,
            parse_constant=_json_constant,
        )
    except ReplayValidationError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
        raise ReplayValidationError(
            "flight log is missing, malformed, or incomplete: "+str(error)
        ) from error
    if not isinstance(records, list):
        raise ReplayValidationError("flight log root must be a JSON array")
    return records


def _parse_scales(text, dimension):
    if text is None:
        return None
    try:
        values = tuple(float(item.strip()) for item in text.split(","))
    except ValueError as error:
        raise ReplayValidationError(
            "--state-scales must be comma-separated finite numbers"
        ) from error
    if len(values) != dimension or not all(
            math.isfinite(value) and value > 0 for value in values):
        raise ReplayValidationError(
            "--state-scales must contain one positive value per state"
        )
    return values


def _parser():
    parser = argparse.ArgumentParser(
        description=(
            "Validate decision-time-resampled release-to-zero LMPC episodes "
            "offline. Raw post-observation flight rows fail closed."
        )
    )
    parser.add_argument(
        "--input",
        required=True,
        help="complete decision-time-resampled flight JSON",
    )
    parser.add_argument(
        "--output", required=True,
        help="new safe-set JSON; an existing path is never overwritten",
    )
    parser.add_argument("--model-fingerprint", required=True)
    parser.add_argument("--state-dimension", required=True, type=int)
    parser.add_argument(
        "--prediction-step-s", type=float,
        help=(
            "fixed replay step (default 0.02 for a new artifact; an existing "
            "artifact supplies this value and any explicit value must match)"
        ),
    )
    parser.add_argument(
        "--command-delay-s", type=float,
        help=(
            "strictly positive configured physical command delay (required "
            "for a new artifact; an existing artifact supplies it and any "
            "explicit value must match)"
        ),
    )
    parser.add_argument(
        "--existing-artifact",
        help="optional existing safe set to copy and extend",
    )
    parser.add_argument(
        "--state-scales",
        help="comma-separated scales for a new artifact",
    )
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    try:
        existing = (
            None if args.existing_artifact is None else
            VelocityLMPCSafeSet.load(args.existing_artifact)
        )
        limits = existing.limits if existing is not None else SafeSetLimits()
        if existing is None and args.command_delay_s is None:
            raise ReplayValidationError(
                "--command-delay-s is required when creating a new artifact"
            )
        if (
            existing is not None
            and args.prediction_step_s is not None
            and not math.isclose(
                args.prediction_step_s,
                existing.prediction_step_s,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
        ):
            raise ReplayValidationError(
                "--prediction-step-s does not match the existing artifact"
            )
        prediction_step_s = (
            existing.prediction_step_s
            if existing is not None else
            0.02 if args.prediction_step_s is None else
            args.prediction_step_s
        )
        if (
            existing is not None
            and args.command_delay_s is not None
            and not math.isclose(
                args.command_delay_s,
                existing.command_delay_s,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
        ):
            raise ReplayValidationError(
                "--command-delay-s does not match the existing artifact"
            )
        command_delay_s = (
            existing.command_delay_s
            if existing is not None else args.command_delay_s
        )
        config = VelocityLMPCReplayConfig(
            model_fingerprint=args.model_fingerprint,
            state_dimension=args.state_dimension,
            prediction_step_s=prediction_step_s,
            command_delay_s=command_delay_s,
            stage_cost_spec=(
                StageCostSpec()
                if existing is None else existing.stage_cost_spec
            ),
            limits=limits,
        )
        records = load_complete_flight_records(args.input)
        result = extract_velocity_lmpc_episodes(records, config)
        if not result.episodes:
            report = result.to_dict()
            report.update({
                "status": "rejected",
                "reason": "no episode passed the offline replay gates",
                "output": None,
            })
            print(json.dumps(report, sort_keys=True, allow_nan=False))
            return 2
        scales = _parse_scales(args.state_scales, args.state_dimension)
        artifact = build_safe_set_artifact(
            result,
            existing=existing,
            state_scales=scales or config.state_scales,
            limits=limits,
        )
        output = save_new_safe_set_artifact(artifact, args.output)
        report = result.to_dict()
        report.update({
            "status": "accepted",
            "output": str(output),
            "existing_artifact": args.existing_artifact,
            "total_episode_count": len(artifact.episodes),
        })
        print(json.dumps(report, sort_keys=True, allow_nan=False))
        return 0
    except (OSError, SafeSetValidationError, ReplayValidationError) as error:
        print(json.dumps({
            "offline_only": True,
            "flight_commands_generated": False,
            "status": "rejected",
            "reason": str(error),
        }, sort_keys=True, allow_nan=False))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
