"""Validated sampled safe sets for conditional iterative velocity LMPC.

This module is intentionally independent of cflib and flight command code.  It
stores only complete, successful release-to-rest episodes and exposes a local
nearest-neighbour query for an optimizer.  The initial release speed is a task
parameter: queries may interpolate between successful speeds, but they may
never extrapolate beyond their convex hull.

The reduced state has the fixed layout ``[v, theta, theta_rate, pending...]``.
Required measured velocity/attitude fields remain explicit on every sample so
admission does not trust an opaque vector or a caller-provided ``passed`` flag.
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


SCHEMA_VERSION = 3
ARTIFACT_KIND = "conditional_velocity_lmpc_safe_set"
TERMINAL_OUTCOME = "terminal_handoff"
STAGE_COST_KIND = "aligned_velocity_tilt_rate_pending_sigmoid_v1"
STATE_ACTION_PHASE_CONTRACT = "coincident_decision_time_v1"

_FINGERPRINT_RE = re.compile(r"[A-Za-z0-9._:+-]{1,128}\Z")
_EPISODE_ID_RE = re.compile(r"[A-Za-z0-9._:+-]{1,128}\Z")
_REDUCED_STATE_CONSISTENCY_ATOL = 1e-9


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
class StageCostSpec:
    """Dimensionless running-cost rate whose integral is stored in seconds."""

    kind: str = STAGE_COST_KIND
    velocity_scale_m_s: float = 0.05
    tilt_scale_rad: float = math.radians(3.0)
    tilt_rate_scale_rad_s: float = math.radians(20.0)
    pending_command_scale_rad: float = math.radians(3.0)
    command_scale_rad: float = math.radians(8.0)
    command_slew_rate_scale_rad_s: float = math.radians(180.0)
    effort_weight: float = 1e-3
    slew_weight: float = 2e-3

    _KEYS = (
        "kind", "velocity_scale_m_s", "tilt_scale_rad",
        "tilt_rate_scale_rad_s", "pending_command_scale_rad",
        "command_scale_rad", "command_slew_rate_scale_rad_s",
        "effort_weight", "slew_weight",
    )

    def __post_init__(self):
        if self.kind != STAGE_COST_KIND:
            raise SafeSetValidationError(
                "stage_cost_spec.kind must be " + STAGE_COST_KIND
            )
        for field in self._KEYS[1:]:
            object.__setattr__(
                self,
                field,
                _number(getattr(self, field), "stage_cost_spec."+field),
            )
        scales = self._KEYS[1:7]
        if any(getattr(self, field) <= 0.0 for field in scales):
            raise SafeSetValidationError(
                "stage_cost_spec scales must be positive"
            )
        if self.effort_weight < 0.0 or self.slew_weight < 0.0:
            raise SafeSetValidationError(
                "stage_cost_spec weights must be non-negative"
            )

    def to_dict(self):
        return {field: getattr(self, field) for field in self._KEYS}

    @classmethod
    def from_dict(cls, value):
        _exact_keys(value, cls._KEYS, "stage_cost_spec")
        return cls(**value)


def minimum_time_stage_cost_s(state, command, dt_s, spec):
    """Return ``dt * running_cost_rate`` for the fixed reduced LMPC state.

    The sigmoid-like progress term is zero only at the target and approaches
    one far from it.  Effort and command-slew terms use the same normalized
    units in both horizon optimization and reverse safe-set cost-to-go.
    """
    if not isinstance(spec, StageCostSpec):
        raise SafeSetValidationError("spec must be StageCostSpec")
    state = _vector(state, "stage_cost.state")
    command = _vector(command, "stage_cost.command")
    dt_s = _number(dt_s, "stage_cost.dt_s")
    if len(state) < 4:
        raise SafeSetValidationError(
            "stage_cost.state must use [v, tilt, tilt_rate, pending...]"
        )
    if len(command) != 1:
        raise SafeSetValidationError(
            "stage_cost.command must contain one aligned command"
        )
    if dt_s < 0.0:
        raise SafeSetValidationError("stage_cost.dt_s must be non-negative")
    if dt_s == 0.0:
        return 0.0

    try:
        normalized_state = (
            state[0]/spec.velocity_scale_m_s,
            state[1]/spec.tilt_scale_rad,
            state[2]/spec.tilt_rate_scale_rad_s,
            *(value/spec.pending_command_scale_rad for value in state[3:]),
        )
        state_squares = tuple(value*value for value in normalized_state)
        if not all(math.isfinite(value) for value in state_squares):
            raise SafeSetValidationError("normalized stage state is too large")
        radius_squared = math.fsum(state_squares)
        progress_rate = radius_squared/math.hypot(radius_squared, 1.0)
        normalized_command = command[0]/spec.command_scale_rad
        command_slew_rate = (command[0]-state[-1])/dt_s
        normalized_slew_rate = (
            command_slew_rate/spec.command_slew_rate_scale_rad_s
        )
        effort_rate = (
            spec.effort_weight*normalized_command*normalized_command
        )
        slew_rate = spec.slew_weight*normalized_slew_rate*normalized_slew_rate
        cost_rate = progress_rate+effort_rate+slew_rate
        result = dt_s*cost_rate
    except SafeSetValidationError:
        raise
    except (OverflowError, ValueError, ZeroDivisionError) as error:
        raise SafeSetValidationError(
            "stage cost arithmetic is outside the finite range"
        ) from error
    if not all(math.isfinite(value) for value in (
        progress_rate, effort_rate, slew_rate, result
    )):
        raise SafeSetValidationError("stage cost is not finite")
    return float(result)


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
    projected_tilt_rad: float
    projected_tilt_rate_rad_s: float
    orthogonal_command_rad: float
    pending_orthogonal_commands_rad: tuple[float, ...]
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
        "cross_velocity_m_s", "projected_tilt_rad",
        "projected_tilt_rate_rad_s", "orthogonal_command_rad",
        "pending_orthogonal_commands_rad",
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
        object.__setattr__(
            self,
            "pending_orthogonal_commands_rad",
            _vector(
                self.pending_orthogonal_commands_rad,
                "sample.pending_orthogonal_commands_rad",
            ),
        )
        position = _vector(self.position_m, "sample.position_m")
        if len(position) != 3:
            raise SafeSetValidationError(
                "sample.position_m must contain three coordinates"
            )
        object.__setattr__(self, "position_m", position)
        for field in self._KEYS[2:-1]:
            if field in ("pending_orthogonal_commands_rad", "position_m"):
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
        result["pending_orthogonal_commands_rad"] = list(
            self.pending_orthogonal_commands_rad
        )
        result["position_m"] = list(self.position_m)
        return result

    @classmethod
    def from_dict(cls, value):
        _exact_keys(value, cls._KEYS, "sample")
        return cls(**value)


def reverse_cost_to_go(samples, spec):
    """Return shared running-stage cost from each sample to the terminal."""
    if not isinstance(spec, StageCostSpec):
        raise SafeSetValidationError("spec must be StageCostSpec")
    samples = tuple(samples)
    if not samples:
        raise SafeSetValidationError("episode.samples must not be empty")
    if any(not isinstance(sample, VelocityLMPCSample) for sample in samples):
        raise SafeSetValidationError(
            "samples must contain VelocityLMPCSample values"
        )
    costs = [0.0]*len(samples)
    for index in range(len(samples)-2, -1, -1):
        sample = samples[index]
        costs[index] = float(
            minimum_time_stage_cost_s(
                sample.state,
                sample.command,
                sample.dt_s,
                spec,
            )+costs[index+1]
        )
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
                     terminal_handoff_position_m, samples,
                     stage_cost_spec):
        samples = tuple(samples)
        return cls(
            episode_id=episode_id,
            context=context,
            outcome=outcome,
            terminal_handoff_position_m=terminal_handoff_position_m,
            samples=samples,
            cost_to_go_s=reverse_cost_to_go(samples, stage_cost_spec),
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
    sample_step_tolerance_s: float = 0.001
    initial_speed_consistency_tolerance_m_s: float = 0.01
    initial_cross_speed_consistency_tolerance_m_s: float = 0.01
    aligned_position_consistency_tolerance_m: float = 1e-6
    max_abs_command: float = math.radians(30.0)
    max_abs_orthogonal_command_rad: float = math.radians(1.0)
    max_path_tilt_rad: float = math.radians(30.0)
    max_path_rate_rad_s: float = math.radians(300.0)
    max_abs_aligned_velocity_m_s: float = 2.0
    max_abs_cross_velocity_m_s: float = 0.15
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
        "min_sample_dt_s", "max_sample_dt_s", "sample_step_tolerance_s",
        "initial_speed_consistency_tolerance_m_s",
        "initial_cross_speed_consistency_tolerance_m_s",
        "aligned_position_consistency_tolerance_m", "max_abs_command",
        "max_abs_orthogonal_command_rad",
        "max_path_tilt_rad", "max_path_rate_rad_s",
        "max_abs_aligned_velocity_m_s", "max_abs_cross_velocity_m_s",
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
        if not 0.0 <= self.sample_step_tolerance_s <= 0.001:
            raise SafeSetValidationError(
                "limits.sample_step_tolerance_s must be in [0, 0.001]"
            )
        positive = (
            "initial_speed_consistency_tolerance_m_s",
            "initial_cross_speed_consistency_tolerance_m_s",
            "aligned_position_consistency_tolerance_m", "max_abs_command",
            "max_abs_orthogonal_command_rad",
            "max_path_tilt_rad", "max_path_rate_rad_s",
            "max_abs_aligned_velocity_m_s", "max_abs_cross_velocity_m_s",
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
        if self.max_abs_orthogonal_command_rad > self.max_abs_command:
            raise SafeSetValidationError(
                "orthogonal command limit exceeds the aligned command limit"
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
    tail_min_aligned_velocity_m_s: float
    tail_max_abs_command: float
    tail_max_abs_tilt_rad: float
    tail_max_abs_rate_rad_s: float
    tail_max_command_step_rad: float


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
        "aligned_velocity_state_index", "prediction_step_s",
        "command_delay_s", "state_action_phase_contract", "state_scales",
        "stage_cost_spec", "limits", "episodes",
    )

    def __init__(self, *, state_dimension, command_dimension, state_scales,
                 command_delay_s, aligned_velocity_state_index=0,
                 prediction_step_s=0.02, stage_cost_spec=None, limits=None):
        self.state_dimension = _integer(
            state_dimension, "state_dimension", minimum=4
        )
        self.command_dimension = _integer(
            command_dimension, "command_dimension", minimum=1
        )
        if self.command_dimension != 1:
            raise SafeSetValidationError(
                "command_dimension must be one aligned command"
            )
        self.aligned_velocity_state_index = _integer(
            aligned_velocity_state_index,
            "aligned_velocity_state_index",
            minimum=0,
        )
        if self.aligned_velocity_state_index != 0:
            raise SafeSetValidationError(
                "aligned_velocity_state_index must be zero for the fixed state layout"
            )
        self.prediction_step_s = _number(
            prediction_step_s, "prediction_step_s"
        )
        if self.prediction_step_s <= 0.0:
            raise SafeSetValidationError("prediction_step_s must be positive")
        self.command_delay_s = _number(command_delay_s, "command_delay_s")
        if self.command_delay_s < 0.0:
            raise SafeSetValidationError("command_delay_s must be non-negative")
        try:
            delay_ratio = self.command_delay_s/self.prediction_step_s
            if not math.isfinite(delay_ratio):
                raise SafeSetValidationError(
                    "command delay ratio must be finite"
                )
            self.delay_steps = max(1, math.ceil(delay_ratio-1e-12))
        except OverflowError as error:
            raise SafeSetValidationError(
                "command delay ratio is outside the supported range"
            ) from error
        expected_state_dimension = 3+self.delay_steps
        if self.state_dimension != expected_state_dimension:
            raise SafeSetValidationError(
                "state_dimension must equal 3 plus the command delay queue "
                f"length ({expected_state_dimension} for this timing)"
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
        if not (
            self.limits.min_sample_dt_s
            <= self.prediction_step_s
            <= self.limits.max_sample_dt_s
        ):
            raise SafeSetValidationError(
                "prediction_step_s must be inside the sample dt limits"
            )
        if self.limits.sample_step_tolerance_s >= self.prediction_step_s:
            raise SafeSetValidationError(
                "sample_step_tolerance_s must be smaller than prediction_step_s"
            )
        self.stage_cost_spec = stage_cost_spec or StageCostSpec()
        if not isinstance(self.stage_cost_spec, StageCostSpec):
            raise SafeSetValidationError(
                "stage_cost_spec must be StageCostSpec"
            )
        self.state_action_phase_contract = STATE_ACTION_PHASE_CONTRACT
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
            and abs(sample.projected_tilt_rad)
            <= limits.terminal_tilt_tolerance_rad
            and abs(sample.projected_tilt_rate_rad_s)
            <= limits.terminal_rate_tolerance_rad_s
            and abs(sample.roll_rad) <= limits.terminal_tilt_tolerance_rad
            and abs(sample.pitch_rad) <= limits.terminal_tilt_tolerance_rad
            and abs(sample.roll_rate_rad_s)
            <= limits.terminal_rate_tolerance_rad_s
            and abs(sample.pitch_rate_rad_s)
            <= limits.terminal_rate_tolerance_rad_s
            and max(abs(value) for value in sample.command)
            <= limits.terminal_command_tolerance
            and abs(sample.orthogonal_command_rad)
            <= limits.terminal_command_tolerance
            and max(abs(value) for value in (
                sample.pending_orthogonal_commands_rad
            )) <= limits.terminal_command_tolerance
            and max(abs(value) for value in sample.state[3:])
            <= limits.terminal_command_tolerance
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
        expected_costs = reverse_cost_to_go(
            episode.samples, self.stage_cost_spec
        )
        if not np.allclose(
            episode.cost_to_go_s, expected_costs, rtol=0.0, atol=1e-12
        ):
            raise SafeSetValidationError(
                "cost_to_go_s does not match reverse minimum-time cost"
            )
        if episode.samples[-1].dt_s != 0.0:
            raise SafeSetValidationError("the final sample dt_s must be zero")
        for index, sample in enumerate(episode.samples):
            if len(sample.state) != self.state_dimension:
                raise SafeSetValidationError(
                    f"sample {index} state dimension does not match artifact"
                )
            if len(sample.command) != self.command_dimension:
                raise SafeSetValidationError(
                    f"sample {index} command dimension does not match artifact"
                )
            if len(sample.pending_orthogonal_commands_rad) != (
                len(sample.state)-3
            ):
                raise SafeSetValidationError(
                    f"sample {index} orthogonal command-memory dimension "
                    "does not match the aligned pending queue"
                )
            if abs(
                sample.state[self.aligned_velocity_state_index]
                - sample.aligned_velocity_m_s
            ) > _REDUCED_STATE_CONSISTENCY_ATOL:
                raise SafeSetValidationError(
                    f"sample {index} opaque state disagrees with measured velocity"
                )
            if abs(
                sample.state[1]-sample.projected_tilt_rad
            ) > _REDUCED_STATE_CONSISTENCY_ATOL:
                raise SafeSetValidationError(
                    f"sample {index} opaque state disagrees with projected tilt"
                )
            if abs(
                sample.state[2]-sample.projected_tilt_rate_rad_s
            ) > _REDUCED_STATE_CONSISTENCY_ATOL:
                raise SafeSetValidationError(
                    f"sample {index} opaque state disagrees with projected tilt rate"
                )
            if index < len(episode.samples)-1 and not (
                self.limits.min_sample_dt_s
                <= sample.dt_s
                <= self.limits.max_sample_dt_s
            ):
                raise SafeSetValidationError(
                    f"sample {index} dt_s is outside the complete-data interval"
                )
            if index < len(episode.samples)-1 and abs(
                sample.dt_s-self.prediction_step_s
            ) > self.limits.sample_step_tolerance_s+1e-12:
                raise SafeSetValidationError(
                    f"sample {index} dt_s does not match prediction_step_s"
                )
            if max(abs(value) for value in sample.command) > self.limits.max_abs_command:
                raise SafeSetValidationError(
                    f"sample {index} command exceeds the path limit"
                )
            if abs(sample.orthogonal_command_rad) > (
                self.limits.max_abs_orthogonal_command_rad+1e-12
            ):
                raise SafeSetValidationError(
                    f"sample {index} orthogonal command exceeds the path limit"
                )
            if max(abs(value) for value in (
                sample.pending_orthogonal_commands_rad
            )) > self.limits.max_abs_orthogonal_command_rad+1e-12:
                raise SafeSetValidationError(
                    f"sample {index} delayed orthogonal command queue exceeds "
                    "the path limit"
                )
            if abs(sample.projected_tilt_rad) > (
                self.limits.max_path_tilt_rad+1e-12
            ):
                raise SafeSetValidationError(
                    f"sample {index} projected tilt exceeds the path limit"
                )
            if abs(sample.projected_tilt_rate_rad_s) > (
                self.limits.max_path_rate_rad_s+1e-12
            ):
                raise SafeSetValidationError(
                    f"sample {index} projected tilt rate exceeds the path limit"
                )
            if abs(sample.aligned_velocity_m_s) > (
                self.limits.max_abs_aligned_velocity_m_s+1e-12
            ):
                raise SafeSetValidationError(
                    f"sample {index} aligned velocity exceeds the path limit"
                )
            if abs(sample.cross_velocity_m_s) > (
                self.limits.max_abs_cross_velocity_m_s+1e-12
            ):
                raise SafeSetValidationError(
                    f"sample {index} cross velocity exceeds the path limit"
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
        for index, (sample, successor) in enumerate(zip(
            episode.samples[:-1], episode.samples[1:]
        )):
            expected_pending = (*sample.state[4:], sample.command[0])
            if any(
                abs(actual-expected) > _REDUCED_STATE_CONSISTENCY_ATOL
                for actual, expected in zip(
                    successor.state[3:], expected_pending
                )
            ):
                raise SafeSetValidationError(
                    f"sample {index+1} violates the fixed command-memory successor"
                )
            expected_orthogonal_pending = (
                *sample.pending_orthogonal_commands_rad[1:],
                sample.orthogonal_command_rad,
            )
            if any(
                abs(actual-expected) > _REDUCED_STATE_CONSISTENCY_ATOL
                for actual, expected in zip(
                    successor.pending_orthogonal_commands_rad,
                    expected_orthogonal_pending,
                )
            ):
                raise SafeSetValidationError(
                    f"sample {index+1} violates the fixed orthogonal "
                    "command-memory successor"
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
        return {
            "passed": True,
            "terminal_dwell_s": float(terminal_dwell),
            "duration_s": float(math.fsum(
                sample.dt_s for sample in episode.samples
            )),
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
        transition_step = [0.0]*count
        entry_step = [0.0]*count
        # The final sample.command is the old command observed as applied at
        # handoff, not a newly issued LMPC action.  Compare only consecutive
        # non-final actions; final queue-internal steps are covered below.
        for index in range(count-2):
            transition_step[index] = max(
                abs(after-before)
                for before, after in zip(
                    samples[index].command,
                    samples[index+1].command,
                )
            )
        for index, sample in enumerate(samples):
            pending = sample.state[3:]
            internal_steps = [
                abs(after-before)
                for before, after in zip(pending[:-1], pending[1:])
            ]
            if index < count-1:
                internal_steps.append(abs(sample.command[0]-pending[-1]))
            entry_step[index] = max(internal_steps, default=0.0)

        envelopes = [None]*count
        release_y = samples[0].position_m[1]
        suffix_max_position = float(
            episode.context.direction_sign
            * (episode.terminal_handoff_position_m[1]-release_y)
        )
        suffix_max_velocity = 0.0
        suffix_min_velocity = math.inf
        suffix_max_command = 0.0
        suffix_max_tilt = 0.0
        suffix_max_rate = 0.0
        suffix_max_step = 0.0
        for index in range(count-1, -1, -1):
            sample = samples[index]
            suffix_max_position = max(
                suffix_max_position, sample.aligned_position_m
            )
            suffix_max_velocity = max(
                suffix_max_velocity, abs(sample.aligned_velocity_m_s)
            )
            suffix_min_velocity = min(
                suffix_min_velocity, sample.aligned_velocity_m_s
            )
            suffix_max_command = max(
                suffix_max_command,
                max(abs(value) for value in sample.command),
                max(abs(value) for value in sample.state[3:]),
            )
            suffix_max_tilt = max(
                suffix_max_tilt,
                abs(sample.roll_rad),
                abs(sample.pitch_rad),
                abs(sample.projected_tilt_rad),
            )
            suffix_max_rate = max(
                suffix_max_rate,
                abs(sample.roll_rate_rad_s),
                abs(sample.pitch_rate_rad_s),
                abs(sample.projected_tilt_rate_rad_s),
            )
            suffix_max_step = max(
                suffix_max_step,
                transition_step[index],
                entry_step[index],
            )
            envelopes[index] = {
                "remaining_forward_distance_m": float(max(
                    0.0,
                    suffix_max_position-sample.aligned_position_m,
                )),
                "tail_max_abs_aligned_velocity_m_s": float(
                    suffix_max_velocity
                ),
                "tail_min_aligned_velocity_m_s": float(
                    suffix_min_velocity
                ),
                "tail_max_abs_command": float(suffix_max_command),
                "tail_max_abs_tilt_rad": float(suffix_max_tilt),
                "tail_max_abs_rate_rad_s": float(suffix_max_rate),
                "tail_max_command_step_rad": float(suffix_max_step),
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
            "prediction_step_s": self.prediction_step_s,
            "command_delay_s": self.command_delay_s,
            "state_action_phase_contract": self.state_action_phase_contract,
            "state_scales": list(self.state_scales),
            "stage_cost_spec": self.stage_cost_spec.to_dict(),
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
        if (
            value["state_action_phase_contract"]
            != STATE_ACTION_PHASE_CONTRACT
        ):
            raise SafeSetValidationError(
                "unsupported safe-set state/action phase contract"
            )
        if not isinstance(value["episodes"], list):
            raise SafeSetValidationError("episodes must be an array")
        result = cls(
            state_dimension=value["state_dimension"],
            command_dimension=value["command_dimension"],
            aligned_velocity_state_index=value[
                "aligned_velocity_state_index"
            ],
            prediction_step_s=value["prediction_step_s"],
            command_delay_s=value["command_delay_s"],
            state_scales=value["state_scales"],
            stage_cost_spec=StageCostSpec.from_dict(
                value["stage_cost_spec"]
            ),
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
