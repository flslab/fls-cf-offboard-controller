"""Validated sampled safe sets for conditional iterative velocity LMPC.

This module is intentionally independent of cflib and flight command code.  It
stores only complete, successful release-to-rest episodes and exposes a local
nearest-neighbour query for an optimizer.  The initial release speed is a task
parameter: queries may interpolate between successful speeds, but they may
never extrapolate beyond their convex hull.

The state and command vectors are deliberately generic so the numerical LMPC
core can evolve without weakening artifact validation.  Required measured
velocity/attitude fields remain explicit on every sample so admission does not
trust an opaque vector or a caller-provided ``passed`` flag.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import numbers
import os
from pathlib import Path
import re
import tempfile

import numpy as np


SCHEMA_VERSION = 1
ARTIFACT_KIND = "conditional_velocity_lmpc_safe_set"
TERMINAL_OUTCOME = "terminal_handoff"

_FINGERPRINT_RE = re.compile(r"[A-Za-z0-9._:+-]{1,128}\Z")
_EPISODE_ID_RE = re.compile(r"[A-Za-z0-9._:+-]{1,128}\Z")


class SafeSetValidationError(ValueError):
    """The artifact or an episode violates the safe-set contract."""


class NoSafeSetCoverageError(LookupError):
    """No validated conditional safe set covers the requested context."""


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise SafeSetValidationError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise SafeSetValidationError(f"{name} must be a finite number")
    return result


def _integer(value, name, *, minimum=None):
    if isinstance(value, bool) or not isinstance(value, numbers.Integral):
        raise SafeSetValidationError(f"{name} must be an integer")
    result = int(value)
    if minimum is not None and result < minimum:
        raise SafeSetValidationError(f"{name} must be at least {minimum}")
    return result


def _exact_keys(value, expected, name):
    if not isinstance(value, dict):
        raise SafeSetValidationError(f"{name} must be an object")
    actual = set(value)
    expected = set(expected)
    missing = sorted(expected-actual)
    unknown = sorted(actual-expected)
    if missing or unknown:
        details = []
        if missing:
            details.append("missing " + ", ".join(missing))
        if unknown:
            details.append("unknown " + ", ".join(unknown))
        raise SafeSetValidationError(f"{name} has " + "; ".join(details))


def _vector(value, name, *, nonempty=True):
    if not isinstance(value, (list, tuple)):
        raise SafeSetValidationError(f"{name} must be an array")
    result = tuple(_number(item, f"{name}[{index}]")
                   for index, item in enumerate(value))
    if nonempty and not result:
        raise SafeSetValidationError(f"{name} must not be empty")
    return result


@dataclass(frozen=True)
class LMPCContext:
    """Release condition that selects one conditional sampled safe set."""

    direction_sign: int
    initial_speed_m_s: float
    cross_speed_m_s: float
    roll_rad: float
    pitch_rad: float
    roll_rate_rad_s: float
    pitch_rate_rad_s: float
    boundary_margin_m: float
    battery_voltage_v: float
    model_fingerprint: str

    _KEYS = (
        "direction_sign", "initial_speed_m_s", "cross_speed_m_s",
        "roll_rad", "pitch_rad", "roll_rate_rad_s", "pitch_rate_rad_s",
        "boundary_margin_m", "battery_voltage_v", "model_fingerprint",
    )

    def __post_init__(self):
        direction = _integer(self.direction_sign, "context.direction_sign")
        if direction not in (-1, 1):
            raise SafeSetValidationError("context.direction_sign must be -1 or 1")
        object.__setattr__(self, "direction_sign", direction)
        for field in self._KEYS[1:-1]:
            object.__setattr__(
                self, field, _number(getattr(self, field), "context."+field)
            )
        if self.initial_speed_m_s <= 0:
            raise SafeSetValidationError(
                "context.initial_speed_m_s must be positive"
            )
        if self.boundary_margin_m <= 0:
            raise SafeSetValidationError(
                "context.boundary_margin_m must be positive"
            )
        if self.battery_voltage_v <= 0:
            raise SafeSetValidationError(
                "context.battery_voltage_v must be positive"
            )
        if (
            not isinstance(self.model_fingerprint, str)
            or _FINGERPRINT_RE.fullmatch(self.model_fingerprint) is None
        ):
            raise SafeSetValidationError(
                "context.model_fingerprint has an invalid format"
            )

    def to_dict(self):
        return {field: getattr(self, field) for field in self._KEYS}

    @classmethod
    def from_dict(cls, value):
        _exact_keys(value, cls._KEYS, "context")
        return cls(**value)


@dataclass(frozen=True)
class VelocityLMPCSample:
    """One measured state and actually sent command in an episode.

    ``dt_s`` is the transition duration from this state to the next state.  It
    must be positive for every non-final sample and exactly zero for the final
    sample.  ``aligned_velocity_m_s`` is positive in the release direction;
    negative values therefore mean reversal/overshoot for a zero-speed target.
    ``cross_velocity_m_s`` remains an explicit measured safety value even
    though the reduced optimizer state is one-dimensional.  Terminal
    admission uses full planar speed, not only the release-axis projection.
    """

    state: tuple[float, ...]
    command: tuple[float, ...]
    dt_s: float
    aligned_velocity_m_s: float
    cross_velocity_m_s: float
    position_m: tuple[float, float, float]
    roll_rad: float
    pitch_rad: float
    roll_rate_rad_s: float
    pitch_rate_rad_s: float
    aligned_position_m: float = 0.0
    boundary_margin_m: float = 1.0
    state_age_s: float = 0.0
    state_group_skew_s: float = 0.0
    safety_violation: bool = False

    _KEYS = (
        "state", "command", "dt_s", "aligned_velocity_m_s",
        "cross_velocity_m_s",
        "position_m", "aligned_position_m", "roll_rad", "pitch_rad",
        "roll_rate_rad_s",
        "pitch_rate_rad_s",
        "boundary_margin_m", "state_age_s", "state_group_skew_s",
        "safety_violation",
    )

    def __post_init__(self):
        object.__setattr__(self, "state", _vector(self.state, "sample.state"))
        object.__setattr__(
            self, "command", _vector(self.command, "sample.command")
        )
        position = _vector(self.position_m, "sample.position_m")
        if len(position) != 3:
            raise SafeSetValidationError(
                "sample.position_m must contain three coordinates"
            )
        object.__setattr__(self, "position_m", position)
        for field in self._KEYS[2:-1]:
            if field == "position_m":
                continue
            object.__setattr__(
                self, field, _number(getattr(self, field), "sample."+field)
            )
        if type(self.safety_violation) is not bool:
            raise SafeSetValidationError(
                "sample.safety_violation must be boolean"
            )
        if self.dt_s < 0:
            raise SafeSetValidationError("sample.dt_s must be non-negative")
        if self.boundary_margin_m <= 0:
            raise SafeSetValidationError(
                "sample.boundary_margin_m must be positive"
            )
        if self.state_age_s < 0 or self.state_group_skew_s < 0:
            raise SafeSetValidationError(
                "sample state age and group skew must be non-negative"
            )

    def to_dict(self):
        result = {field: getattr(self, field) for field in self._KEYS}
        result["state"] = list(self.state)
        result["command"] = list(self.command)
        result["position_m"] = list(self.position_m)
        return result

    @classmethod
    def from_dict(cls, value):
        _exact_keys(value, cls._KEYS, "sample")
        return cls(**value)


def reverse_cost_to_go(samples):
    """Return minimum-time stage cost from each sample to the final sample."""
    samples = tuple(samples)
    if not samples:
        raise SafeSetValidationError("episode.samples must not be empty")
    costs = [0.0]*len(samples)
    for index in range(len(samples)-2, -1, -1):
        costs[index] = float(samples[index].dt_s+costs[index+1])
    return tuple(costs)


@dataclass(frozen=True)
class VelocityLMPCEpisode:
    episode_id: str
    context: LMPCContext
    outcome: str
    terminal_handoff_position_m: tuple[float, float, float]
    samples: tuple[VelocityLMPCSample, ...]
    cost_to_go_s: tuple[float, ...]

    _KEYS = (
        "episode_id", "context", "outcome", "terminal_handoff_position_m",
        "samples", "cost_to_go_s",
    )

    def __post_init__(self):
        if (
            not isinstance(self.episode_id, str)
            or _EPISODE_ID_RE.fullmatch(self.episode_id) is None
        ):
            raise SafeSetValidationError("episode_id has an invalid format")
        if not isinstance(self.context, LMPCContext):
            raise SafeSetValidationError("episode.context must be LMPCContext")
        if not isinstance(self.outcome, str) or not self.outcome:
            raise SafeSetValidationError("episode.outcome must be a string")
        terminal_handoff = _vector(
            self.terminal_handoff_position_m,
            "episode.terminal_handoff_position_m",
        )
        if len(terminal_handoff) != 3:
            raise SafeSetValidationError(
                "episode.terminal_handoff_position_m must contain three "
                "coordinates"
            )
        object.__setattr__(
            self, "terminal_handoff_position_m", terminal_handoff
        )
        samples = tuple(self.samples)
        if not samples or any(
            not isinstance(sample, VelocityLMPCSample) for sample in samples
        ):
            raise SafeSetValidationError(
                "episode.samples must contain VelocityLMPCSample values"
            )
        object.__setattr__(self, "samples", samples)
        costs = tuple(
            _number(item, f"episode.cost_to_go_s[{index}]")
            for index, item in enumerate(self.cost_to_go_s)
        )
        object.__setattr__(self, "cost_to_go_s", costs)

    @classmethod
    def from_samples(cls, *, episode_id, context, outcome,
                     terminal_handoff_position_m, samples):
        samples = tuple(samples)
        return cls(
            episode_id=episode_id,
            context=context,
            outcome=outcome,
            terminal_handoff_position_m=terminal_handoff_position_m,
            samples=samples,
            cost_to_go_s=reverse_cost_to_go(samples),
        )

    def to_dict(self):
        return {
            "episode_id": self.episode_id,
            "context": self.context.to_dict(),
            "outcome": self.outcome,
            "terminal_handoff_position_m": list(
                self.terminal_handoff_position_m
            ),
            "samples": [sample.to_dict() for sample in self.samples],
            "cost_to_go_s": list(self.cost_to_go_s),
        }

    @classmethod
    def from_dict(cls, value):
        _exact_keys(value, cls._KEYS, "episode")
        if not isinstance(value["samples"], list):
            raise SafeSetValidationError("episode.samples must be an array")
        if not isinstance(value["cost_to_go_s"], list):
            raise SafeSetValidationError(
                "episode.cost_to_go_s must be an array"
            )
        return cls(
            episode_id=value["episode_id"],
            context=LMPCContext.from_dict(value["context"]),
            outcome=value["outcome"],
            terminal_handoff_position_m=value[
                "terminal_handoff_position_m"
            ],
            samples=tuple(
                VelocityLMPCSample.from_dict(sample)
                for sample in value["samples"]
            ),
            cost_to_go_s=tuple(value["cost_to_go_s"]),
        )


@dataclass(frozen=True)
class SafeSetLimits:
    min_sample_dt_s: float = 0.001
    max_sample_dt_s: float = 0.05
    initial_speed_consistency_tolerance_m_s: float = 0.01
    initial_cross_speed_consistency_tolerance_m_s: float = 0.01
    aligned_position_consistency_tolerance_m: float = 1e-6
    max_abs_command: float = math.radians(30.0)
    max_path_tilt_rad: float = math.radians(30.0)
    max_path_rate_rad_s: float = math.radians(300.0)
    max_abs_aligned_velocity_m_s: float = 2.0
    terminal_velocity_tolerance_m_s: float = 0.05
    terminal_tilt_tolerance_rad: float = math.radians(3.0)
    terminal_rate_tolerance_rad_s: float = math.radians(20.0)
    terminal_command_tolerance: float = math.radians(3.0)
    terminal_position_handoff_tolerance_m: float = 0.01
    terminal_dwell_s: float = 0.08
    reverse_velocity_tolerance_m_s: float = 0.02
    min_path_boundary_margin_m: float = 0.02
    max_state_age_s: float = 0.10
    max_state_group_skew_s: float = 0.03
    initial_attitude_consistency_tolerance_rad: float = math.radians(0.5)
    initial_rate_consistency_tolerance_rad_s: float = math.radians(5.0)
    initial_boundary_consistency_tolerance_m: float = 0.01
    max_context_cross_speed_m_s: float = 0.15
    max_context_initial_tilt_rad: float = math.radians(10.0)
    max_context_initial_rate_rad_s: float = math.radians(100.0)
    max_context_cross_speed_delta_m_s: float = 0.05
    max_context_tilt_delta_rad: float = math.radians(3.0)
    max_context_rate_delta_rad_s: float = math.radians(30.0)
    max_context_battery_delta_v: float = 0.50

    _KEYS = (
        "min_sample_dt_s", "max_sample_dt_s",
        "initial_speed_consistency_tolerance_m_s",
        "initial_cross_speed_consistency_tolerance_m_s",
        "aligned_position_consistency_tolerance_m", "max_abs_command",
        "max_path_tilt_rad", "max_path_rate_rad_s",
        "max_abs_aligned_velocity_m_s",
        "terminal_velocity_tolerance_m_s", "terminal_tilt_tolerance_rad",
        "terminal_rate_tolerance_rad_s", "terminal_command_tolerance",
        "terminal_position_handoff_tolerance_m",
        "terminal_dwell_s", "reverse_velocity_tolerance_m_s",
        "min_path_boundary_margin_m", "max_state_age_s",
        "max_state_group_skew_s",
        "initial_attitude_consistency_tolerance_rad",
        "initial_rate_consistency_tolerance_rad_s",
        "initial_boundary_consistency_tolerance_m",
        "max_context_cross_speed_m_s", "max_context_initial_tilt_rad",
        "max_context_initial_rate_rad_s",
        "max_context_cross_speed_delta_m_s", "max_context_tilt_delta_rad",
        "max_context_rate_delta_rad_s", "max_context_battery_delta_v",
    )

    def __post_init__(self):
        for field in self._KEYS:
            object.__setattr__(
                self, field, _number(getattr(self, field), "limits."+field)
            )
        if self.min_sample_dt_s <= 0:
            raise SafeSetValidationError("limits.min_sample_dt_s must be positive")
        if self.max_sample_dt_s < self.min_sample_dt_s:
            raise SafeSetValidationError(
                "limits.max_sample_dt_s must cover min_sample_dt_s"
            )
        positive = (
            "initial_speed_consistency_tolerance_m_s",
            "initial_cross_speed_consistency_tolerance_m_s",
            "aligned_position_consistency_tolerance_m", "max_abs_command",
            "max_path_tilt_rad", "max_path_rate_rad_s",
            "max_abs_aligned_velocity_m_s",
            "terminal_velocity_tolerance_m_s", "terminal_tilt_tolerance_rad",
            "terminal_rate_tolerance_rad_s", "terminal_command_tolerance",
            "terminal_position_handoff_tolerance_m",
            "terminal_dwell_s", "min_path_boundary_margin_m",
            "max_state_age_s", "max_state_group_skew_s",
            "initial_attitude_consistency_tolerance_rad",
            "initial_rate_consistency_tolerance_rad_s",
            "initial_boundary_consistency_tolerance_m",
            "max_context_cross_speed_m_s", "max_context_initial_tilt_rad",
            "max_context_initial_rate_rad_s",
            "max_context_cross_speed_delta_m_s", "max_context_tilt_delta_rad",
            "max_context_rate_delta_rad_s", "max_context_battery_delta_v",
        )
        if any(getattr(self, field) <= 0 for field in positive):
            raise SafeSetValidationError(
                "consistency, command, path, terminal, and dwell limits must be positive"
            )
        if self.reverse_velocity_tolerance_m_s < 0:
            raise SafeSetValidationError(
                "limits.reverse_velocity_tolerance_m_s must be non-negative"
            )
        if self.terminal_tilt_tolerance_rad > self.max_path_tilt_rad:
            raise SafeSetValidationError(
                "terminal tilt tolerance exceeds the path tilt limit"
            )
        if self.terminal_rate_tolerance_rad_s > self.max_path_rate_rad_s:
            raise SafeSetValidationError(
                "terminal rate tolerance exceeds the path rate limit"
            )
        if self.terminal_command_tolerance > self.max_abs_command:
            raise SafeSetValidationError(
                "terminal command tolerance exceeds the command limit"
            )

    def to_dict(self):
        return {field: getattr(self, field) for field in self._KEYS}

    @classmethod
    def from_dict(cls, value):
        _exact_keys(value, cls._KEYS, "limits")
        return cls(**value)


@dataclass(frozen=True)
class SafeSetPoint:
    episode_id: str
    sample_index: int
    initial_speed_m_s: float
    state: tuple[float, ...]
    command: tuple[float, ...]
    cost_to_go_s: float
    distance: float
    remaining_forward_distance_m: float
    tail_max_abs_aligned_velocity_m_s: float
    tail_max_abs_command: float
    tail_max_abs_tilt_rad: float
    tail_max_abs_rate_rad_s: float
    tail_max_command_slew_rad_s: float


@dataclass(frozen=True)
class LocalSafeSetQuery:
    direction_sign: int
    model_fingerprint: str
    lower_initial_speed_m_s: float
    upper_initial_speed_m_s: float
    upper_interpolation_weight: float
    points: tuple[SafeSetPoint, ...]


class VelocityLMPCSafeSet:
    """In-memory database containing only episodes that pass hard admission."""

    _KEYS = (
        "schema_version", "kind", "state_dimension", "command_dimension",
        "aligned_velocity_state_index", "state_scales", "limits", "episodes",
    )

    def __init__(self, *, state_dimension, command_dimension, state_scales,
                 aligned_velocity_state_index=0, limits=None):
        self.state_dimension = _integer(
            state_dimension, "state_dimension", minimum=1
        )
        self.command_dimension = _integer(
            command_dimension, "command_dimension", minimum=1
        )
        self.aligned_velocity_state_index = _integer(
            aligned_velocity_state_index,
            "aligned_velocity_state_index",
            minimum=0,
        )
        if self.aligned_velocity_state_index >= self.state_dimension:
            raise SafeSetValidationError(
                "aligned_velocity_state_index is outside the state vector"
            )
        self.state_scales = _vector(state_scales, "state_scales")
        if len(self.state_scales) != self.state_dimension:
            raise SafeSetValidationError(
                "state_scales length must match state_dimension"
            )
        if any(scale <= 0 for scale in self.state_scales):
            raise SafeSetValidationError("state_scales must be positive")
        self.limits = limits or SafeSetLimits()
        if not isinstance(self.limits, SafeSetLimits):
            raise SafeSetValidationError("limits must be SafeSetLimits")
        self._episodes = []
        self._episode_ids = set()

    @property
    def episodes(self):
        return tuple(self._episodes)

    def _terminal(self, sample):
        limits = self.limits
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
            and (
                self.state_dimension <= 3
                or max(abs(value) for value in sample.state[3:])
                <= limits.terminal_command_tolerance
            )
        )

    def validate_episode(self, episode):
        if not isinstance(episode, VelocityLMPCEpisode):
            raise SafeSetValidationError(
                "episode must be VelocityLMPCEpisode"
            )
        if episode.outcome != TERMINAL_OUTCOME:
            raise SafeSetValidationError(
                "only terminal_handoff episodes may enter the safe set"
            )
        context = episode.context
        limits = self.limits
        if abs(context.cross_speed_m_s) > limits.max_context_cross_speed_m_s:
            raise SafeSetValidationError(
                "episode initial cross speed exceeds the context gate"
            )
        if max(abs(context.roll_rad), abs(context.pitch_rad)) > (
            limits.max_context_initial_tilt_rad
        ):
            raise SafeSetValidationError(
                "episode initial attitude exceeds the context gate"
            )
        if max(
            abs(context.roll_rate_rad_s), abs(context.pitch_rate_rad_s)
        ) > limits.max_context_initial_rate_rad_s:
            raise SafeSetValidationError(
                "episode initial attitude rate exceeds the context gate"
            )
        if len(episode.samples) < 2:
            raise SafeSetValidationError("episode needs at least two samples")
        if len(episode.cost_to_go_s) != len(episode.samples):
            raise SafeSetValidationError(
                "cost_to_go_s length must match episode samples"
            )
        expected_costs = reverse_cost_to_go(episode.samples)
        if not np.allclose(
            episode.cost_to_go_s, expected_costs, rtol=0.0, atol=1e-12
        ):
            raise SafeSetValidationError(
                "cost_to_go_s does not match reverse minimum-time cost"
            )
        if episode.samples[-1].dt_s != 0.0:
            raise SafeSetValidationError("the final sample dt_s must be zero")
        zero_dt_entry_change_index = None
        for index, sample in enumerate(episode.samples):
            if len(sample.state) != self.state_dimension:
                raise SafeSetValidationError(
                    f"sample {index} state dimension does not match artifact"
                )
            if len(sample.command) != self.command_dimension:
                raise SafeSetValidationError(
                    f"sample {index} command dimension does not match artifact"
                )
            if abs(
                sample.state[self.aligned_velocity_state_index]
                - sample.aligned_velocity_m_s
            ) > self.limits.initial_speed_consistency_tolerance_m_s:
                raise SafeSetValidationError(
                    f"sample {index} opaque state disagrees with measured velocity"
                )
            if index < len(episode.samples)-1 and not (
                self.limits.min_sample_dt_s
                <= sample.dt_s
                <= self.limits.max_sample_dt_s
            ):
                raise SafeSetValidationError(
                    f"sample {index} dt_s is outside the complete-data interval"
                )
            if max(abs(value) for value in sample.command) > self.limits.max_abs_command:
                raise SafeSetValidationError(
                    f"sample {index} command exceeds the path limit"
                )
            if abs(sample.aligned_velocity_m_s) > (
                self.limits.max_abs_aligned_velocity_m_s+1e-12
            ):
                raise SafeSetValidationError(
                    f"sample {index} aligned velocity exceeds the path limit"
                )
            if self.state_dimension > 3:
                delayed_queue = sample.state[3:]
                if max(abs(value) for value in delayed_queue) > (
                    self.limits.max_abs_command+1e-12
                ):
                    raise SafeSetValidationError(
                        f"sample {index} delayed command queue exceeds the "
                        "path limit"
                    )
                entry_change = abs(
                    sample.command[0]-delayed_queue[-1]
                )
                if sample.dt_s <= 0.0 and entry_change > 1e-12:
                    zero_dt_entry_change_index = index
            if max(abs(sample.roll_rad), abs(sample.pitch_rad)) > self.limits.max_path_tilt_rad:
                raise SafeSetValidationError(
                    f"sample {index} attitude exceeds the path limit"
                )
            if max(
                abs(sample.roll_rate_rad_s), abs(sample.pitch_rate_rad_s)
            ) > self.limits.max_path_rate_rad_s:
                raise SafeSetValidationError(
                    f"sample {index} attitude rate exceeds the path limit"
                )
            if (
                sample.aligned_velocity_m_s
                < -self.limits.reverse_velocity_tolerance_m_s
            ):
                raise SafeSetValidationError(
                    f"sample {index} reverses past the allowed tolerance"
                )
            if sample.boundary_margin_m < limits.min_path_boundary_margin_m:
                raise SafeSetValidationError(
                    f"sample {index} violates the workspace boundary margin"
                )
            if sample.state_age_s > limits.max_state_age_s:
                raise SafeSetValidationError(
                    f"sample {index} has stale state telemetry"
                )
            if sample.state_group_skew_s > limits.max_state_group_skew_s:
                raise SafeSetValidationError(
                    f"sample {index} has skewed state telemetry"
                )
            if sample.safety_violation:
                raise SafeSetValidationError(
                    f"sample {index} records a safety violation"
                )
        release_position = episode.samples[0].position_m
        for index, sample in enumerate(episode.samples):
            measured_aligned_position = (
                context.direction_sign
                * (sample.position_m[1]-release_position[1])
            )
            if abs(
                sample.aligned_position_m-measured_aligned_position
            ) > limits.aligned_position_consistency_tolerance_m:
                raise SafeSetValidationError(
                    f"sample {index} aligned position disagrees with measured "
                    "world position"
                )
        if abs(
            episode.samples[0].aligned_velocity_m_s
            - episode.context.initial_speed_m_s
        ) > self.limits.initial_speed_consistency_tolerance_m_s:
            raise SafeSetValidationError(
                "context initial speed does not match the first measured sample"
            )
        first = episode.samples[0]
        if abs(
            first.cross_velocity_m_s-context.cross_speed_m_s
        ) > limits.initial_cross_speed_consistency_tolerance_m_s:
            raise SafeSetValidationError(
                "context initial cross speed does not match the first "
                "measured sample"
            )
        if max(
            abs(first.roll_rad-context.roll_rad),
            abs(first.pitch_rad-context.pitch_rad),
        ) > limits.initial_attitude_consistency_tolerance_rad:
            raise SafeSetValidationError(
                "context initial attitude does not match the first sample"
            )
        if max(
            abs(first.roll_rate_rad_s-context.roll_rate_rad_s),
            abs(first.pitch_rate_rad_s-context.pitch_rate_rad_s),
        ) > limits.initial_rate_consistency_tolerance_rad_s:
            raise SafeSetValidationError(
                "context initial attitude rate does not match the first sample"
            )
        if abs(
            first.boundary_margin_m-context.boundary_margin_m
        ) > limits.initial_boundary_consistency_tolerance_m:
            raise SafeSetValidationError(
                "context boundary margin does not match the first sample"
            )
        if math.dist(
            episode.terminal_handoff_position_m,
            episode.samples[-1].position_m,
        ) > limits.terminal_position_handoff_tolerance_m+1e-12:
            raise SafeSetValidationError(
                "terminal position handoff target is not the final measured "
                "position"
            )
        terminal_flags = [self._terminal(sample) for sample in episode.samples]
        if not terminal_flags[-1]:
            raise SafeSetValidationError(
                "episode does not end inside the full measured terminal set"
            )
        terminal_start = len(terminal_flags)-1
        while terminal_start > 0 and terminal_flags[terminal_start-1]:
            terminal_start -= 1
        terminal_dwell = sum(
            sample.dt_s for sample in episode.samples[terminal_start:-1]
        )
        if terminal_dwell+1e-12 < self.limits.terminal_dwell_s:
            raise SafeSetValidationError(
                "episode terminal-state dwell is too short"
            )
        if zero_dt_entry_change_index is not None:
            raise SafeSetValidationError(
                f"sample {zero_dt_entry_change_index} has a nonzero safe-tail "
                "entry command change at zero dt_s"
            )
        return {
            "passed": True,
            "terminal_dwell_s": float(terminal_dwell),
            "duration_s": float(expected_costs[0]),
            "minimum_aligned_velocity_m_s": float(min(
                sample.aligned_velocity_m_s for sample in episode.samples
            )),
        }

    def add_episode(self, episode):
        """Validate completely before mutating the database."""
        if isinstance(episode, VelocityLMPCEpisode):
            episode_id = episode.episode_id
        else:
            episode_id = None
        if episode_id in self._episode_ids:
            raise SafeSetValidationError(
                f"duplicate episode_id: {episode_id}"
            )
        result = self.validate_episode(episode)
        self._episodes.append(episode)
        self._episode_ids.add(episode.episode_id)
        return result

    @staticmethod
    def _tail_envelopes(episode):
        """Compute certified suffix bounds once for every safe-set state."""
        samples = episode.samples
        count = len(samples)
        transition_slew = [0.0]*count
        entry_slew = [0.0]*count
        for index in range(count-1):
            transition_slew[index] = max(
                abs(after-before)/samples[index].dt_s
                for before, after in zip(
                    samples[index].command,
                    samples[index+1].command,
                )
            )
        for index, sample in enumerate(samples):
            if len(sample.state) <= 3:
                continue
            entry_change = abs(sample.command[0]-sample.state[-1])
            if sample.dt_s > 0.0:
                entry_slew[index] = entry_change/sample.dt_s
            elif entry_change > 1e-12:
                # Admission rejects this case. Keep the derived envelope
                # fail-closed if an invalid episode ever reaches this helper.
                entry_slew[index] = math.inf

        envelopes = [None]*count
        suffix_max_position = -math.inf
        suffix_max_velocity = 0.0
        suffix_max_command = 0.0
        suffix_max_tilt = 0.0
        suffix_max_rate = 0.0
        suffix_max_slew = 0.0
        for index in range(count-1, -1, -1):
            sample = samples[index]
            suffix_max_position = max(
                suffix_max_position, sample.aligned_position_m
            )
            suffix_max_velocity = max(
                suffix_max_velocity, abs(sample.aligned_velocity_m_s)
            )
            suffix_max_command = max(
                suffix_max_command,
                max(abs(value) for value in sample.command),
            )
            suffix_max_tilt = max(
                suffix_max_tilt,
                abs(sample.roll_rad),
                abs(sample.pitch_rad),
            )
            suffix_max_rate = max(
                suffix_max_rate,
                abs(sample.roll_rate_rad_s),
                abs(sample.pitch_rate_rad_s),
            )
            suffix_max_slew = max(
                suffix_max_slew,
                transition_slew[index],
                entry_slew[index],
            )
            envelopes[index] = {
                "remaining_forward_distance_m": float(max(
                    0.0,
                    suffix_max_position-sample.aligned_position_m,
                )),
                "tail_max_abs_aligned_velocity_m_s": float(
                    suffix_max_velocity
                ),
                "tail_max_abs_command": float(suffix_max_command),
                "tail_max_abs_tilt_rad": float(suffix_max_tilt),
                "tail_max_abs_rate_rad_s": float(suffix_max_rate),
                "tail_max_command_slew_rad_s": float(suffix_max_slew),
            }
        return tuple(envelopes)

    def query(self, context, current_state, *, neighbors_per_bracket=8):
        """Return local safe points without initial-speed extrapolation.

        Direction and model fingerprint are exact partitions.  When the query
        speed lies between two observed speeds, nearest points are returned
        independently from each bracket so an optimizer cannot accidentally
        discard one side of the conditional interpolation.
        """
        if not isinstance(context, LMPCContext):
            raise SafeSetValidationError("query context must be LMPCContext")
        state = _vector(current_state, "current_state")
        if len(state) != self.state_dimension:
            raise SafeSetValidationError(
                "current_state dimension does not match artifact"
            )
        count = _integer(
            neighbors_per_bracket, "neighbors_per_bracket", minimum=1
        )
        limits = self.limits
        if abs(context.cross_speed_m_s) > limits.max_context_cross_speed_m_s:
            raise NoSafeSetCoverageError(
                "current cross speed is outside the certified context gate"
            )
        if max(abs(context.roll_rad), abs(context.pitch_rad)) > (
            limits.max_context_initial_tilt_rad
        ):
            raise NoSafeSetCoverageError(
                "current attitude is outside the certified context gate"
            )
        if max(
            abs(context.roll_rate_rad_s), abs(context.pitch_rate_rad_s)
        ) > limits.max_context_initial_rate_rad_s:
            raise NoSafeSetCoverageError(
                "current attitude rate is outside the certified context gate"
            )
        matching = [episode for episode in self._episodes if (
            episode.context.direction_sign == context.direction_sign
            and episode.context.model_fingerprint == context.model_fingerprint
            and context.boundary_margin_m+1e-12
            >= episode.context.boundary_margin_m
            and abs(
                context.battery_voltage_v
                - episode.context.battery_voltage_v
            ) <= limits.max_context_battery_delta_v+1e-12
            and abs(
                context.cross_speed_m_s-episode.context.cross_speed_m_s
            ) <= limits.max_context_cross_speed_delta_m_s+1e-12
            and max(
                abs(context.roll_rad-episode.context.roll_rad),
                abs(context.pitch_rad-episode.context.pitch_rad),
            ) <= limits.max_context_tilt_delta_rad+1e-12
            and max(
                abs(
                    context.roll_rate_rad_s
                    - episode.context.roll_rate_rad_s
                ),
                abs(
                    context.pitch_rate_rad_s
                    - episode.context.pitch_rate_rad_s
                ),
            ) <= limits.max_context_rate_delta_rad_s+1e-12
        )]
        if not matching:
            raise NoSafeSetCoverageError(
                "no safe episodes match direction, model, and release context"
            )
        speeds = sorted({episode.context.initial_speed_m_s for episode in matching})
        speed = context.initial_speed_m_s
        epsilon = 1e-12
        if speed < speeds[0]-epsilon or speed > speeds[-1]+epsilon:
            raise NoSafeSetCoverageError(
                "initial speed is outside the successful trajectory convex hull"
            )
        lower = max(value for value in speeds if value <= speed+epsilon)
        upper = min(value for value in speeds if value >= speed-epsilon)
        if math.isclose(lower, upper, rel_tol=0.0, abs_tol=epsilon):
            upper = lower
            weight = 0.0
            brackets = (lower,)
        else:
            weight = float((speed-lower)/(upper-lower))
            brackets = (lower, upper)
        current = np.asarray(state, dtype=float)
        scales = np.asarray(self.state_scales, dtype=float)
        selected = []
        for bracket in brackets:
            candidates = []
            for episode in matching:
                if not math.isclose(
                    episode.context.initial_speed_m_s, bracket,
                    rel_tol=0.0, abs_tol=epsilon,
                ):
                    continue
                tail_envelopes = self._tail_envelopes(episode)
                for index, sample in enumerate(episode.samples):
                    distance = float(np.linalg.norm(
                        (np.asarray(sample.state)-current)/scales
                    ))
                    candidates.append(SafeSetPoint(
                        episode_id=episode.episode_id,
                        sample_index=index,
                        initial_speed_m_s=episode.context.initial_speed_m_s,
                        state=sample.state,
                        command=sample.command,
                        cost_to_go_s=episode.cost_to_go_s[index],
                        distance=distance,
                        **tail_envelopes[index],
                    ))
            candidates.sort(key=lambda item: (
                item.distance, item.cost_to_go_s,
                item.episode_id, item.sample_index,
            ))
            selected.extend(candidates[:count])
        if not selected:
            raise NoSafeSetCoverageError("matching safe-set partition is empty")
        return LocalSafeSetQuery(
            direction_sign=context.direction_sign,
            model_fingerprint=context.model_fingerprint,
            lower_initial_speed_m_s=float(lower),
            upper_initial_speed_m_s=float(upper),
            upper_interpolation_weight=weight,
            points=tuple(selected),
        )

    def to_dict(self):
        return {
            "schema_version": SCHEMA_VERSION,
            "kind": ARTIFACT_KIND,
            "state_dimension": self.state_dimension,
            "command_dimension": self.command_dimension,
            "aligned_velocity_state_index": (
                self.aligned_velocity_state_index
            ),
            "state_scales": list(self.state_scales),
            "limits": self.limits.to_dict(),
            "episodes": [episode.to_dict() for episode in self._episodes],
        }

    @classmethod
    def from_dict(cls, value):
        _exact_keys(value, cls._KEYS, "safe-set artifact")
        version = _integer(value["schema_version"], "schema_version")
        if version != SCHEMA_VERSION:
            raise SafeSetValidationError(
                f"unsupported safe-set schema version: {version}"
            )
        if value["kind"] != ARTIFACT_KIND:
            raise SafeSetValidationError("unexpected safe-set artifact kind")
        if not isinstance(value["episodes"], list):
            raise SafeSetValidationError("episodes must be an array")
        result = cls(
            state_dimension=value["state_dimension"],
            command_dimension=value["command_dimension"],
            aligned_velocity_state_index=value[
                "aligned_velocity_state_index"
            ],
            state_scales=value["state_scales"],
            limits=SafeSetLimits.from_dict(value["limits"]),
        )
        for raw_episode in value["episodes"]:
            result.add_episode(VelocityLMPCEpisode.from_dict(raw_episode))
        return result

    @staticmethod
    def _json_object(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise SafeSetValidationError(f"duplicate JSON key: {key}")
            result[key] = value
        return result

    @staticmethod
    def _json_constant(value):
        raise SafeSetValidationError(f"non-finite JSON constant: {value}")

    @classmethod
    def loads(cls, text):
        try:
            value = json.loads(
                text,
                object_pairs_hook=cls._json_object,
                parse_constant=cls._json_constant,
            )
        except SafeSetValidationError:
            raise
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            raise SafeSetValidationError(
                "invalid safe-set JSON: " + str(error)
            ) from error
        return cls.from_dict(value)

    @classmethod
    def load(cls, path):
        return cls.loads(Path(path).read_text(encoding="utf-8"))

    def dumps(self):
        return json.dumps(
            self.to_dict(), indent=2, sort_keys=True, allow_nan=False
        ) + "\n"

    def save(self, path):
        """Atomically publish a complete artifact; never expose partial JSON."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix=path.name+".", suffix=".tmp", dir=str(path.parent)
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                stream.write(self.dumps())
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
