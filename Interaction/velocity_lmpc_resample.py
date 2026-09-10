"""Offline resampling of raw release episodes onto actual decision epochs.

Raw flight rows observe state before the controller sends its next attitude
command.  This module moves each state to the corresponding *actual* attitude
send timestamp using only bracketing fresh ``wrench_observer`` measurements.
It never extrapolates, never invents a successful terminal sample, and never
overwrites an output file.  The resulting JSON remains a flight-record array
and can be passed directly to :mod:`Interaction.velocity_lmpc_replay`.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
import numbers
import os
from pathlib import Path
import re
import tempfile
from typing import Sequence

import numpy as np

from Interaction.release_lmpc_terminal_gate import (
    ReleaseLMPCTerminalGate,
    ReleaseLMPCTerminalSample,
    classify_terminal_post_state_commands,
)
from Interaction.velocity_lmpc_replay import (
    BOOTSTRAP_ATTEMPT_CLOSED_EVENT,
    CLOSE_EVENT,
    START_EVENTS,
    TERMINAL_DWELL_EVENT,
    ReplayValidationError,
    VelocityLMPCReplayConfig,
    VelocityLMPCReplayRejection,
    _direction,
    _is_hazard_event,
    _json_constant,
    _json_object,
    _logged_command,
    _observer_values,
    _queue_for_sample,
    _record_data,
    _same_command,
    _segments,
    _sent_command_history,
)
from Interaction.velocity_lmpc_safe_set import SafeSetLimits


RESAMPLE_METHOD = "linear_fresh_wrench_observer_angle_unwrap_v1"
DIRECTION_SIGNS = ("positive-y", "negative-y")
_SHA256_RE = re.compile(r"[0-9a-f]{64}\Z")
_CONDITIONAL_FINGERPRINT_RE = re.compile(r"reduced-v3:[0-9a-f]{64}\Z")
_TIME_ATOL_S = 1e-9


class ResampleValidationError(ValueError):
    """A raw file or episode cannot be resampled without fabrication."""


@dataclass(frozen=True)
class VelocityLMPCResampleResult:
    records: tuple[dict, ...]
    episode_ids: tuple[str, ...]
    rejections: tuple[VelocityLMPCReplayRejection, ...]
    direction_sign: str
    skipped_opposite_direction_episode_ids: tuple[str, ...]
    raw_sha256: str
    source_record_count: int
    prediction_step_s: float
    command_delay_s: float
    model_fingerprint: str | None
    state_dimension: int
    offline_only: bool = True
    flight_commands_generated: bool = False

    def to_dict(self):
        return {
            "offline_only": True,
            "flight_commands_generated": False,
            "source_record_count": self.source_record_count,
            "output_record_count": len(self.records),
            "episode_count": len(self.episode_ids),
            "episode_ids": list(self.episode_ids),
            "direction_sign": self.direction_sign,
            "skipped_opposite_direction_episode_count": len(
                self.skipped_opposite_direction_episode_ids
            ),
            "skipped_opposite_direction_episode_ids": list(
                self.skipped_opposite_direction_episode_ids
            ),
            "rejected_episode_count": len(self.rejections),
            "rejections": [item.to_dict() for item in self.rejections],
            "raw_sha256": self.raw_sha256,
            "prediction_step_s": self.prediction_step_s,
            "command_delay_s": self.command_delay_s,
            "model_fingerprint": self.model_fingerprint,
            "state_dimension": self.state_dimension,
            "resample_method": RESAMPLE_METHOD,
        }


@dataclass(frozen=True)
class _SourceState:
    index: int
    state_time_s: float
    values: dict


def _finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ResampleValidationError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ResampleValidationError(f"{name} must be a finite number")
    return result


def _validated_direction_sign(value):
    if value not in DIRECTION_SIGNS:
        raise ResampleValidationError(
            "direction_sign must be exactly 'positive-y' or 'negative-y'"
        )
    return value


def _direction_sign(direction):
    return "positive-y" if float(direction[1]) > 0.0 else "negative-y"


def _start_direction_sign(records, start_index, config):
    """Best-effort direction classification for filtering parse rejections."""
    if start_index is None or not 0 <= start_index < len(records):
        return None
    record = records[start_index]
    if (
        record.get("type") != "events"
        or record.get("name") not in START_EVENTS
    ):
        return None
    try:
        data = _record_data(record, start_index)
        direction = _direction(
            data.get("release_dataset_direction_xy"),
            config,
            f"record {start_index} release_dataset_direction_xy",
        )
    except (ReplayValidationError, ResampleValidationError):
        return None
    return _direction_sign(direction)


def _rejection_direction_sign(records, rejection, config):
    direct = _start_direction_sign(records, rejection.start_index, config)
    if direct is not None:
        return direct
    if not isinstance(rejection.episode_id, str) or not rejection.episode_id:
        return None
    matches = set()
    for index, record in enumerate(records):
        if (
            record.get("type") != "events"
            or record.get("name") not in START_EVENTS
        ):
            continue
        data = record.get("data")
        if not isinstance(data, dict) or data.get(
            "release_dataset_episode_id"
        ) != rejection.episode_id:
            continue
        sign = _start_direction_sign(records, index, config)
        if sign is not None:
            matches.add(sign)
    return next(iter(matches)) if len(matches) == 1 else None


def _raw_digest(records):
    try:
        encoded = json.dumps(
            records,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as error:
        raise ResampleValidationError(
            "raw records are not finite JSON values"
        ) from error
    return hashlib.sha256(encoded).hexdigest()


def _config(prediction_step_s, command_delay_s, limits):
    step = _finite_number(prediction_step_s, "prediction_step_s")
    delay = _finite_number(command_delay_s, "command_delay_s")
    if step <= 0.0:
        raise ResampleValidationError("prediction_step_s must be positive")
    if delay <= 0.0:
        raise ResampleValidationError(
            "command_delay_s must be positive for raw state-before-send data"
        )
    if not isinstance(limits, SafeSetLimits):
        raise ResampleValidationError("limits must be SafeSetLimits")
    delay_ratio = delay/step
    if not math.isfinite(delay_ratio):
        raise ResampleValidationError(
            "command_delay_s/prediction_step_s is too large"
        )
    delay_steps = math.ceil(delay_ratio-1e-12)
    try:
        return VelocityLMPCReplayConfig(
            model_fingerprint="decision-time-resample-v1",
            state_dimension=3+max(1, delay_steps),
            prediction_step_s=step,
            command_delay_s=delay,
            limits=limits,
        )
    except ReplayValidationError as error:
        raise ResampleValidationError(str(error)) from error


def _validate_raw_timing_identity(records, segment, config):
    """Bind CLI timing to the immutable per-release flight evidence."""
    start_data = _record_data(records[segment.start_index], segment.start_index)
    logged_step = _finite_number(
        start_data.get("release_dataset_prediction_step_s"),
        "release_dataset_prediction_step_s",
    )
    logged_delay = _finite_number(
        start_data.get("release_dataset_command_delay_s"),
        "release_dataset_command_delay_s",
    )
    if not math.isclose(
        logged_step,
        config.prediction_step_s,
        rel_tol=0.0,
        abs_tol=_TIME_ATOL_S,
    ):
        raise ResampleValidationError(
            "CLI prediction_step_s does not match the release timing identity"
        )
    if not math.isclose(
        logged_delay,
        config.command_delay_s,
        rel_tol=0.0,
        abs_tol=_TIME_ATOL_S,
    ):
        raise ResampleValidationError(
            "CLI command_delay_s does not match the release timing identity"
        )
    effective_raw = segment.release_effective_command
    effective_query_time = _finite_number(
        effective_raw.get("effective_query_time"),
        "release_command_effective_at_state.effective_query_time",
    )
    expected_query_time = segment.release_state_time-logged_delay
    if not math.isclose(
        effective_query_time,
        expected_query_time,
        rel_tol=0.0,
        abs_tol=_TIME_ATOL_S,
    ):
        raise ResampleValidationError(
            "release effective_query_time does not match release state minus "
            "the logged command delay"
        )
    model_fingerprint = start_data.get(
        "release_dataset_model_fingerprint"
    )
    if (
        not isinstance(model_fingerprint, str)
        or _CONDITIONAL_FINGERPRINT_RE.fullmatch(model_fingerprint) is None
    ):
        raise ResampleValidationError(
            "release_dataset_model_fingerprint must identify the frozen "
            "conditional LMPC model contract"
        )
    state_dimension = start_data.get("release_dataset_state_dimension")
    if (
        isinstance(state_dimension, bool)
        or not isinstance(state_dimension, numbers.Integral)
        or int(state_dimension) != config.state_dimension
    ):
        raise ResampleValidationError(
            "release_dataset_state_dimension does not match the directional "
            "model delay contract"
        )
    return model_fingerprint, int(state_dimension)


def _validated_bootstrap_eligibility(records, segment):
    """Require the flight-side sticky path audit before offline admission."""
    matching_indices = [
        index
        for index in range(segment.start_index, segment.terminal_dwell_index+1)
        if (
            records[index].get("type") == "events"
            and records[index].get("name") == BOOTSTRAP_ATTEMPT_CLOSED_EVENT
        )
    ]
    if len(matching_indices) != 1:
        raise ResampleValidationError(
            "episode must contain exactly one Learning MPC Bootstrap Attempt "
            "Closed event before its terminal marker"
        )
    record_index = matching_indices[0]
    data = _record_data(records[record_index], record_index)
    if data.get("episode_id") != segment.episode_id:
        raise ResampleValidationError(
            "Learning MPC Bootstrap Attempt Closed episode_id does not match "
            "the release episode"
        )
    required_true = (
        "terminal_success",
        "countable",
        "provisional_path_eligible",
        "counted",
    )
    if data.get("path_failure_reasons") != []:
        raise ResampleValidationError(
            "Learning MPC Bootstrap Attempt Closed path_failure_reasons must "
            "be exactly an empty array"
        )
    for field_name in required_true:
        if data.get(field_name) is not True:
            raise ResampleValidationError(
                "Learning MPC Bootstrap Attempt Closed "
                f"{field_name} must be true"
            )
    return {
        "event_name": BOOTSTRAP_ATTEMPT_CLOSED_EVENT,
        "source_record_index": record_index,
        "episode_id": segment.episode_id,
        **{field_name: True for field_name in required_true},
        "path_failure_reasons": [],
    }


def _command_inventory(records, segment):
    """Return original command dictionaries keyed by lossless sequence ID."""
    inventory = {}

    def add(raw, record_index, field_name):
        command = _logged_command(
            raw, record_index=record_index, field_name=field_name
        )
        normalized_raw = deepcopy(raw)
        if "effective_query_time" in normalized_raw:
            if field_name != "release_command_effective_at_state":
                raise ResampleValidationError(
                    "effective_query_time is allowed only on the release "
                    "effective-command lookup"
                )
            _finite_number(
                normalized_raw.pop("effective_query_time"),
                "release effective_query_time",
            )
        existing = inventory.get(command.sequence)
        if existing is not None:
            existing_command, existing_raw = existing
            if (
                not _same_command(existing_command, command)
                or existing_raw != normalized_raw
            ):
                raise ResampleValidationError(
                    "same command sequence has conflicting raw payloads"
                )
            return command
        # ``effective_query_time`` describes a lookup view, not a radio send.
        # Strip it before storing the physical command so the old query epoch
        # cannot leak into resampled pending/applied command views. Every other
        # field must agree exactly whenever a sequence appears more than once.
        inventory[command.sequence] = (command, normalized_raw)
        return command

    add(
        segment.release_effective_command,
        segment.start_index,
        "release_command_effective_at_state",
    )
    for offset, raw in enumerate(segment.release_pending_commands):
        add(
            raw,
            segment.start_index,
            f"release_pending_command_history[{offset}]",
        )
    batch_commands = []
    seen_batch_sequences = set()
    for index in segment.observer_indices:
        data = _record_data(records[index], index)
        batch = data.get("actual_commands_sent_since_previous_state")
        if not isinstance(batch, list):
            raise ResampleValidationError(
                f"record {index} actual command batch must be an array"
            )
        for offset, raw in enumerate(batch):
            command = add(
                raw,
                index,
                f"actual_commands_sent_since_previous_state[{offset}]",
            )
            if command.sequence in seen_batch_sequences:
                raise ResampleValidationError(
                    "actual command appears in more than one lossless batch"
                )
            seen_batch_sequences.add(command.sequence)
            batch_commands.append(command)
    return inventory, tuple(batch_commands)


def _raw_command(command, inventory):
    try:
        stored, raw = inventory[command.sequence]
    except KeyError as error:
        raise ResampleValidationError(
            "reconstructed command is absent from the raw inventory"
        ) from error
    if not _same_command(stored, command):
        raise ResampleValidationError(
            "reconstructed command payload disagrees with raw inventory"
        )
    return deepcopy(raw)


def _decision_commands(records, segment, config):
    try:
        timeline = _sent_command_history(records, segment, config)
        inventory, batch_commands = _command_inventory(records, segment)
    except ReplayValidationError as error:
        raise ResampleValidationError(str(error)) from error
    if not batch_commands:
        raise ResampleValidationError("episode has no actual command sends")
    if any(
        after.sent_at <= before.sent_at
        for before, after in zip(batch_commands, batch_commands[1:])
    ):
        raise ResampleValidationError(
            "actual command batch timeline is not strictly increasing"
        )
    positions = [
        index for index, command in enumerate(batch_commands)
        if command.kind == "position"
    ]
    if positions != [len(batch_commands)-1]:
        raise ResampleValidationError(
            "episode must end with exactly one final position command"
        )
    actions = batch_commands[:-1]
    if not actions:
        raise ResampleValidationError("episode has no attitude decision")
    for command in actions:
        if command.kind != "attitude_zdistance":
            raise ResampleValidationError(
                "every non-final actual command must be attitude_zdistance"
            )
    final_position = batch_commands[-1]
    if final_position.record_index >= segment.terminal_dwell_index:
        raise ResampleValidationError(
            "final current-position handoff must precede the terminal marker"
        )
    epochs = [command.sent_at for command in actions]
    epochs.append(final_position.sent_at)
    for index, (before, after) in enumerate(zip(epochs, epochs[1:])):
        interval = after-before
        if abs(interval-config.prediction_step_s) > (
            config.limits.sample_step_tolerance_s+1e-12
        ):
            raise ResampleValidationError(
                f"decision command interval {index} is {interval:.6f}s, not "
                f"the configured {config.prediction_step_s:.6f}s step"
            )
    return (
        timeline,
        inventory,
        actions,
        final_position,
        tuple(epochs),
    )


def _source_states(
        records, segment, config, first_epoch, final_epoch, final_sequence):
    bracket_rows = []
    for index in segment.observer_indices:
        data = _record_data(records[index], index)
        marker = data.get("release_dataset_resample_upper_bracket", False)
        if type(marker) is not bool:
            raise ResampleValidationError(
                "release_dataset_resample_upper_bracket must be boolean"
            )
        if marker:
            bracket_rows.append(index)
    if len(bracket_rows) != 1:
        raise ResampleValidationError(
            "episode must contain exactly one explicit resample upper bracket"
        )
    bracket_index = bracket_rows[0]
    if bracket_index != segment.observer_indices[-1]:
        raise ResampleValidationError(
            "resample upper bracket must be the final observer before the "
            "terminal marker"
        )
    bracket_data = _record_data(records[bracket_index], bracket_index)
    if bracket_data.get("actual_commands_sent_since_previous_state") != []:
        raise ResampleValidationError(
            "resample upper bracket must be command-free"
        )
    bracket_state_time = _finite_number(
        bracket_data.get("state_time"),
        f"record {bracket_index} upper bracket state_time",
    )
    bracket_final_sent_at = _finite_number(
        bracket_data.get("release_dataset_final_position_sent_at"),
        "upper bracket final position sent_at",
    )
    bracket_final_sequence = bracket_data.get(
        "release_dataset_final_position_sequence"
    )
    if (
        isinstance(bracket_final_sequence, bool)
        or not isinstance(bracket_final_sequence, numbers.Integral)
        or int(bracket_final_sequence) != final_sequence
        or not math.isclose(
            bracket_final_sent_at,
            final_epoch,
            rel_tol=0.0,
            abs_tol=_TIME_ATOL_S,
        )
    ):
        raise ResampleValidationError(
            "resample upper bracket does not identify the final position send"
        )
    if bracket_state_time <= segment.terminal_state_time+_TIME_ATOL_S:
        raise ResampleValidationError(
            "resample upper bracket must be strictly newer than the terminal "
            "handoff state"
        )
    terminal_data = _record_data(
        records[segment.terminal_dwell_index], segment.terminal_dwell_index
    )
    logged_bracket_time = _finite_number(
        terminal_data.get("resample_upper_bracket_state_time"),
        "terminal resample_upper_bracket_state_time",
    )
    terminal_final_sent_at = _finite_number(
        terminal_data.get("final_position_sent_at"),
        "terminal final_position_sent_at",
    )
    terminal_final_sequence = terminal_data.get("final_position_sequence")
    if (
        isinstance(terminal_final_sequence, bool)
        or not isinstance(terminal_final_sequence, numbers.Integral)
        or int(terminal_final_sequence) != final_sequence
        or not math.isclose(
            terminal_final_sent_at,
            final_epoch,
            rel_tol=0.0,
            abs_tol=_TIME_ATOL_S,
        )
    ):
        raise ResampleValidationError(
            "terminal marker does not identify the final position send"
        )
    if not math.isclose(
        logged_bracket_time,
        bracket_state_time,
        rel_tol=0.0,
        abs_tol=_TIME_ATOL_S,
    ):
        raise ResampleValidationError(
            "terminal marker does not identify the explicit resample upper "
            "bracket"
        )

    candidates = []
    next_start_seen = False
    for index in range(segment.start_index+1, len(records)):
        record = records[index]
        if (
            record.get("type") == "events"
            and record.get("name") in START_EVENTS
        ):
            next_start_seen = True
            break
        if record.get("type") != "wrench_observer":
            continue
        data = _record_data(record, index)
        if data.get("release_dataset_episode_id") != segment.episode_id:
            continue
        state_time = _finite_number(
            data.get("state_time"), f"record {index} state_time"
        )
        candidates.append((index, state_time))
        if state_time >= final_epoch-_TIME_ATOL_S:
            break
    if not candidates:
        raise ResampleValidationError(
            "episode has no matching fresh wrench_observer source states"
        )
    if candidates[0][1] > first_epoch+_TIME_ATOL_S:
        raise ResampleValidationError(
            "first decision epoch would require state extrapolation"
        )
    if candidates[-1][1] < final_epoch-_TIME_ATOL_S:
        suffix = " before the next release" if next_start_seen else ""
        raise ResampleValidationError(
            "final decision epoch would require state extrapolation"+suffix
        )

    sources = []
    previous_time = None
    for index, state_time in candidates:
        if previous_time is not None and state_time <= previous_time+_TIME_ATOL_S:
            raise ResampleValidationError(
                "source state timestamps are duplicate or non-monotonic"
            )
        previous_time = state_time
        data = _record_data(records[index], index)
        if data.get("resample_provenance") is not None:
            raise ResampleValidationError(
                "source row is already resampled; raw measurements are required"
            )
        try:
            row_direction = _direction(
                data.get("release_dataset_direction_xy"),
                config,
                f"record {index} release_dataset_direction_xy",
            )
            values = _observer_values(records[index], index, segment, config)
        except ReplayValidationError as error:
            raise ResampleValidationError(str(error)) from error
        if not np.array_equal(row_direction, segment.direction):
            raise ResampleValidationError(
                f"source observer record {index} changes episode direction"
            )
        sources.append(_SourceState(index, state_time, values))
    for before, after in zip(sources, sources[1:]):
        gap = after.state_time_s-before.state_time_s
        if gap > config.limits.max_sample_dt_s+1e-12:
            raise ResampleValidationError(
                f"source observer gap of {gap:.6f}s exceeds the fresh-data limit"
            )
    upper_index = sources[-1].index
    if any(
        _is_hazard_event(records[index])
        for index in range(segment.start_index, upper_index+1)
    ):
        raise ResampleValidationError(
            "episode interpolation interval contains a hazard event"
        )
    # Unwrap the complete measured sequence before interpolation.  Pairwise
    # shortest-arc interpolation alone would be numerically equivalent modulo
    # 2*pi, but could put consecutive output rows on opposite wrap branches.
    unwrapped_rpy = np.unwrap(np.stack([
        source.values["rpy"] for source in sources
    ]), axis=0)
    return tuple(
        _SourceState(
            source.index,
            source.state_time_s,
            {**source.values, "rpy": unwrapped_rpy[index]},
        )
        for index, source in enumerate(sources)
    )


def _interpolate_source(sources, epoch, raw_sha256):
    times = [source.state_time_s for source in sources]
    position = bisect_left(times, epoch)
    if position < len(sources) and math.isclose(
        times[position], epoch, rel_tol=0.0, abs_tol=_TIME_ATOL_S
    ):
        lower = upper = sources[position]
        weight = 0.0
    else:
        if position == 0 or position == len(sources):
            raise ResampleValidationError(
                f"decision epoch {epoch:.9f} would require state extrapolation"
            )
        lower = sources[position-1]
        upper = sources[position]
        gap = upper.state_time_s-lower.state_time_s
        if gap <= 0.0:
            raise ResampleValidationError("invalid source interpolation gap")
        weight = (epoch-lower.state_time_s)/gap
        if not 0.0 <= weight <= 1.0:
            raise ResampleValidationError("interpolation weight is outside [0, 1]")

    first = lower.values
    second = upper.values

    def linear(name):
        return np.asarray(first[name], dtype=float)+weight*(
            np.asarray(second[name], dtype=float)
            - np.asarray(first[name], dtype=float)
        )

    rpy = linear("rpy")
    result = {
        "position": linear("position"),
        "velocity": linear("velocity"),
        "rpy": rpy,
        "rates": linear("rates"),
        "battery": float(
            first["battery"]+weight*(second["battery"]-first["battery"])
        ),
        # Do not make safety margins look better through interpolation.
        "boundary": float(min(first["boundary"], second["boundary"])),
        "skew": float(max(first["skew"], second["skew"])),
        "provenance": {
            "raw_sha256": raw_sha256,
            "source_record_indices": [lower.index, upper.index],
            "source_state_times_s": [
                lower.state_time_s, upper.state_time_s,
            ],
            "target_decision_time_s": float(epoch),
            "interpolation_weight": float(weight),
            "method": RESAMPLE_METHOD,
        },
    }
    if not all(math.isfinite(float(value)) for values in (
        result["position"], result["velocity"], result["rpy"],
        result["rates"],
    ) for value in values):
        raise ResampleValidationError("interpolated state is not finite")
    if not math.isfinite(result["battery"]) or result["battery"] <= 0.0:
        raise ResampleValidationError("interpolated battery is invalid")
    return result


def _gate_status(gate, state, queue, inventory):
    raw_queue = [_raw_command(command, inventory) for command in queue]
    applied = raw_queue[0]
    status = gate.update(ReleaseLMPCTerminalSample(
        state_time_s=state["provenance"]["target_decision_time_s"],
        velocity_xy_m_s=tuple(float(value) for value in state["velocity"][:2]),
        attitude_rp_rad=tuple(float(value) for value in state["rpy"][:2]),
        attitude_rate_rp_rad_s=tuple(
            float(value) for value in state["rates"][:2]
        ),
        applied_command_kind=applied.get("kind"),
        applied_attitude_rp_rad=(
            math.radians(_finite_number(
                applied.get("roll_deg"), "applied roll_deg"
            )),
            math.radians(_finite_number(
                applied.get("pitch_deg"), "applied pitch_deg"
            )),
        ),
        state_age_s=0.0,
        state_group_skew_s=state["skew"],
        boundary_margin_m=state["boundary"],
        pending_command_kinds=tuple(
            command.get("kind") for command in raw_queue
        ),
        pending_attitude_rp_rad=tuple((
            math.radians(_finite_number(
                command.get("roll_deg"), "pending roll_deg"
            )),
            math.radians(_finite_number(
                command.get("pitch_deg"), "pending pitch_deg"
            )),
        ) for command in raw_queue),
    ))
    return status, applied, raw_queue


def _resampled_observer(
        *, segment, epoch, state, applied, sent, owner, gate_status,
        final=False):
    return {
        "type": "wrench_observer",
        "name": None,
        "data": {
            "time": float(epoch),
            "state_time": float(epoch),
            "state_age_s": 0.0,
            "state_group_skew_s": state["skew"],
            "release_dataset_episode_id": segment.episode_id,
            "release_dataset_direction_xy": [
                float(value) for value in segment.direction
            ],
            "release_dataset_command_owner": owner,
            "release_dataset_terminal_gate": gate_status.to_dict(),
            "release_dataset_pending_outcome": (
                "terminal_handoff" if final else None
            ),
            "actual_command_applied_at_state": deepcopy(applied),
            "actual_commands_sent_since_previous_state": [deepcopy(sent)],
            "position_m": [float(value) for value in state["position"]],
            "velocity_m_s": [float(value) for value in state["velocity"]],
            "orientation_rpy_rad": [float(value) for value in state["rpy"]],
            "angular_velocity_rad_s": [
                float(value) for value in state["rates"]
            ],
            "xy_boundary_margin_m": state["boundary"],
            "battery_voltage_V": state["battery"],
            "measurement_rejected": False,
            "decision_time_resampled": True,
            "offline_lmpc_dataset_only": True,
            "resample_provenance": deepcopy(state["provenance"]),
        },
    }


def _resample_segment(records, segment, config, raw_sha256):
    bootstrap_eligibility = _validated_bootstrap_eligibility(records, segment)
    model_contract = _validate_raw_timing_identity(records, segment, config)
    (
        timeline, inventory, actions, final_position, epochs,
    ) = _decision_commands(records, segment, config)
    sources = _source_states(
        records,
        segment,
        config,
        epochs[0],
        epochs[-1],
        final_position.sequence,
    )
    states = tuple(
        _interpolate_source(sources, epoch, raw_sha256)
        for epoch in epochs
    )

    queues = []
    modeled_queue = None
    for index, epoch in enumerate(epochs):
        try:
            raw_queue = _queue_for_sample(
                timeline, sample_stamp=epoch, config=config
            )
        except ReplayValidationError as error:
            raise ResampleValidationError(str(error)) from error
        if modeled_queue is None:
            modeled_queue = raw_queue
        else:
            modeled_queue = (*modeled_queue[1:], actions[index-1])
        if len(raw_queue) != len(modeled_queue) or any(
            not _same_command(raw, modeled)
            for raw, modeled in zip(raw_queue, modeled_queue)
        ):
            raise ResampleValidationError(
                "actual send timeline cannot form the fixed command-memory "
                "successor at every decision epoch"
            )
        if raw_queue[0].kind != "attitude_zdistance" or any(
            command.kind != "attitude_zdistance" for command in raw_queue
        ):
            raise ResampleValidationError(
                "effective and pending commands must all be attitude_zdistance"
            )
        queues.append(raw_queue)

    gate = ReleaseLMPCTerminalGate(config.limits)
    gate.start()
    output_rows = []
    final_gate_status = None
    for index, (epoch, state, queue) in enumerate(zip(
        epochs, states, queues
    )):
        final = index == len(epochs)-1
        gate_status, applied, _ = _gate_status(
            gate, state, queue, inventory
        )
        if final:
            final_gate_status = gate_status
            sent = _raw_command(final_position, inventory)
            raw_owner = _record_data(
                records[final_position.record_index],
                final_position.record_index,
            ).get("release_dataset_command_owner")
            if raw_owner != "position_hold":
                raise ResampleValidationError(
                    "final actual command owner is not position_hold"
                )
            if classify_terminal_post_state_commands(
                [sent],
                raw_owner,
                config.limits,
                terminal_position_m=tuple(
                    float(value) for value in state["position"]
                ),
            ) != "position_handoff":
                raise ResampleValidationError(
                    "actual final current-position handoff does not match "
                    "the interpolated terminal position"
                )
            owner = raw_owner
        else:
            sent = _raw_command(actions[index], inventory)
            owner = _record_data(
                records[actions[index].record_index],
                actions[index].record_index,
            ).get("release_dataset_command_owner")
            if not isinstance(owner, str) or not owner:
                raise ResampleValidationError(
                    "attitude action has no command owner provenance"
                )
        output_rows.append(_resampled_observer(
            segment=segment,
            epoch=epoch,
            state=state,
            applied=applied,
            sent=sent,
            owner=owner,
            gate_status=gate_status,
            final=final,
        ))
    if final_gate_status is None or not final_gate_status.complete:
        reason = (
            "missing final terminal gate"
            if final_gate_status is None else final_gate_status.reason
        )
        raise ResampleValidationError(
            "resampled final state lacks measured terminal dwell: "+reason
        )

    first_queue = queues[0]
    start = deepcopy(records[segment.start_index])
    start_data = _record_data(start, segment.start_index)
    start_data["release_state_time"] = float(epochs[0])
    start_data["release_command_effective_at_state"] = _raw_command(
        first_queue[0], inventory
    )
    start_data["release_pending_command_history"] = [
        _raw_command(command, inventory) for command in first_queue
    ]
    start_data["decision_time_resampled"] = True
    start_data["offline_lmpc_dataset_only"] = True
    start_data["resample_provenance"] = {
        "raw_sha256": raw_sha256,
        "source_record_indices": [segment.start_index],
        "source_state_times_s": [segment.release_state_time],
        "target_decision_time_s": float(epochs[0]),
        "interpolation_weight": None,
        "method": RESAMPLE_METHOD,
    }

    terminal = deepcopy(records[segment.terminal_dwell_index])
    terminal_data = _record_data(terminal, segment.terminal_dwell_index)
    terminal_data["terminal_state_time"] = float(epochs[-1])
    terminal_data["terminal_gate"] = final_gate_status.to_dict()
    terminal_data["release_dataset_outcome"] = "terminal_handoff"
    terminal_data["decision_time_resampled"] = True
    terminal_data["offline_lmpc_dataset_only"] = True
    terminal_data["release_dataset_bootstrap_eligibility"] = (
        bootstrap_eligibility
    )
    terminal_data["resample_provenance"] = {
        "raw_sha256": raw_sha256,
        "source_record_indices": [segment.terminal_dwell_index],
        "source_state_times_s": [segment.terminal_state_time],
        "target_decision_time_s": float(epochs[-1]),
        "interpolation_weight": None,
        "method": RESAMPLE_METHOD,
    }
    return (start, *output_rows, terminal), model_contract


def _rejected_block(records, rejection, raw_sha256):
    if rejection.start_index is None or not (
        0 <= rejection.start_index < len(records)
    ):
        return ()
    start = deepcopy(records[rejection.start_index])
    if (
        start.get("type") != "events"
        or start.get("name") not in START_EVENTS
    ):
        return ()
    start_data = start.get("data")
    if not isinstance(start_data, dict):
        return ()
    episode_id = rejection.episode_id or start_data.get(
        "release_dataset_episode_id"
    )
    if not isinstance(episode_id, str) or not episode_id:
        return ()
    source_time = start_data.get("time")
    if rejection.end_index is not None and (
        0 <= rejection.end_index < len(records)
    ):
        end_data = records[rejection.end_index].get("data")
        if isinstance(end_data, dict) and isinstance(
            end_data.get("time"), numbers.Real
        ):
            source_time = end_data["time"]
    try:
        close_time = _finite_number(source_time, "rejected close time")
    except ResampleValidationError:
        return ()
    close = {
        "type": "events",
        "name": CLOSE_EVENT,
        "data": {
            "time": close_time,
            "release_dataset_episode_id": episode_id,
            "release_dataset_outcome": "resample_rejected",
            "reason": rejection.reason,
            "offline_lmpc_dataset_only": True,
            "decision_time_resampled": False,
            "resample_provenance": {
                "raw_sha256": raw_sha256,
                "source_record_indices": [
                    rejection.start_index, rejection.end_index,
                ],
                "method": "rejected_without_interpolation_v1",
            },
        },
    }
    return start, close


def resample_velocity_lmpc_records(
        records: Sequence[dict], *, prediction_step_s, command_delay_s,
        direction_sign, raw_sha256=None, limits=None):
    """Resample only one explicit world-Y direction at a time."""
    if not isinstance(records, (list, tuple)):
        raise ResampleValidationError("flight records must be a complete array")
    records = tuple(records)
    direction_sign = _validated_direction_sign(direction_sign)
    limits = SafeSetLimits() if limits is None else limits
    config = _config(prediction_step_s, command_delay_s, limits)
    digest = _raw_digest(records) if raw_sha256 is None else raw_sha256
    if not isinstance(digest, str) or _SHA256_RE.fullmatch(digest) is None:
        raise ResampleValidationError("raw_sha256 must be 64 lowercase hex digits")
    try:
        segments, parse_rejections = _segments(records, config)
    except ReplayValidationError as error:
        raise ResampleValidationError(str(error)) from error

    blocks = []
    rejections = []
    skipped_opposite_ids = []
    for rejection in parse_rejections:
        rejection_sign = _rejection_direction_sign(
            records, rejection, config
        )
        if rejection_sign is not None and rejection_sign != direction_sign:
            if rejection.episode_id is not None:
                skipped_opposite_ids.append(rejection.episode_id)
            continue
        rejections.append(rejection)
    successful_ids = []
    selected_model_contract = None
    for segment in segments:
        if _direction_sign(segment.direction) != direction_sign:
            skipped_opposite_ids.append(segment.episode_id)
            continue
        try:
            block, model_contract = _resample_segment(
                records, segment, config, digest
            )
            if (
                selected_model_contract is not None
                and model_contract != selected_model_contract
            ):
                raise ResampleValidationError(
                    "selected direction contains multiple frozen model "
                    "contracts; resample them into separate artifacts"
                )
        except (ResampleValidationError, ReplayValidationError) as error:
            rejection = VelocityLMPCReplayRejection(
                episode_id=segment.episode_id,
                reason=str(error),
                start_index=segment.start_index,
                end_index=segment.end_index,
            )
            rejections.append(rejection)
            block = _rejected_block(records, rejection, digest)
        else:
            selected_model_contract = model_contract
            successful_ids.append(segment.episode_id)
        blocks.append((segment.start_index, block))
    segment_starts = {
        segment.start_index for segment in segments
        if _direction_sign(segment.direction) == direction_sign
    }
    for rejection in rejections:
        if rejection.start_index in segment_starts:
            continue
        block = _rejected_block(records, rejection, digest)
        if block:
            blocks.append((rejection.start_index, block))
    blocks.sort(key=lambda item: item[0])
    output = tuple(
        record for _, block in blocks for record in block
    )
    return VelocityLMPCResampleResult(
        records=output,
        episode_ids=tuple(successful_ids),
        rejections=tuple(rejections),
        direction_sign=direction_sign,
        skipped_opposite_direction_episode_ids=tuple(dict.fromkeys(
            skipped_opposite_ids
        )),
        raw_sha256=digest,
        source_record_count=len(records),
        prediction_step_s=config.prediction_step_s,
        command_delay_s=config.command_delay_s,
        model_fingerprint=(
            None
            if selected_model_contract is None else
            selected_model_contract[0]
        ),
        state_dimension=config.state_dimension,
    )


def save_new_resampled_records(result, output_path):
    """Atomically publish a new JSON array without replacing any file."""
    if not isinstance(result, VelocityLMPCResampleResult):
        raise ResampleValidationError(
            "result must be VelocityLMPCResampleResult"
        )
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=path.name+".", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(
                list(result.records), stream, indent=2,
                sort_keys=True, allow_nan=False,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, path)
        except FileExistsError as error:
            raise ResampleValidationError(
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


def _parser():
    parser = argparse.ArgumentParser(
        description=(
            "Offline-only raw flight-log resampling onto actual attitude "
            "decision timestamps."
        )
    )
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--prediction-step-s", required=True, type=float)
    parser.add_argument("--command-delay-s", required=True, type=float)
    parser.add_argument(
        "--direction-sign", required=True, choices=DIRECTION_SIGNS,
        help="emit only releases in the selected world-Y direction",
    )
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    try:
        source = Path(args.input)
        raw_bytes = source.read_bytes()
        records = json.loads(
            raw_bytes,
            object_pairs_hook=_json_object,
            parse_constant=_json_constant,
        )
        if not isinstance(records, list):
            raise ResampleValidationError(
                "flight log root must be a JSON array"
            )
        result = resample_velocity_lmpc_records(
            records,
            prediction_step_s=args.prediction_step_s,
            command_delay_s=args.command_delay_s,
            direction_sign=args.direction_sign,
            raw_sha256=hashlib.sha256(raw_bytes).hexdigest(),
        )
        save_new_resampled_records(result, args.output)
    except (
        OSError, UnicodeError, ReplayValidationError,
        ResampleValidationError, ValueError,
    ) as error:
        print(json.dumps({
            "offline_only": True,
            "flight_commands_generated": False,
            "status": "rejected",
            "reason": str(error),
        }, sort_keys=True))
        return 2
    report = result.to_dict()
    report["status"] = "written"
    report["output"] = str(args.output)
    print(json.dumps(report, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
