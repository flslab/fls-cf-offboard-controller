"""Fail-closed terminal dwell gate for release-to-rest LMPC datasets.

The gate has no device, logging, or commander dependencies.  It admits a
    release episode only after *measured* state, the command actually applied to
    the vehicle, and every command still pending inside the identified delay
    window have remained in the complete terminal set for a continuous dwell.
All thresholds come from :class:`SafeSetLimits`, so live collection and offline
safe-set admission can share one contract.

``update`` never raises for malformed telemetry.  Instead it breaks the dwell
and returns an auditable status with a stable reason code.  Construction errors
still raise because they are programmer/configuration errors rather than flight
samples.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import numbers

from Interaction.velocity_lmpc_safe_set import SafeSetLimits


ATTITUDE_ZDISTANCE_COMMAND = "attitude_zdistance"
POSITION_COMMAND = "position"


def classify_terminal_post_state_commands(
        commands, controller_mode, limits=None, terminal_position_m=None):
    """Classify sends made after a measured terminal state.

    A successful iteration must transfer to the position terminal controller
    with every sent position target within the configured tolerance of the
    final measured position.  That makes the terminal backup locally
    invariant instead of allowing an ahead-of-vehicle setpoint to accelerate
    the vehicle again.  Continuing with level attitude is safe to keep
    observing but does not close the episode.  Missing, mixed, velocity,
    malformed, or non-level attitude commands fail closed as unsafe.
    """
    limits = SafeSetLimits() if limits is None else limits
    if not isinstance(limits, SafeSetLimits):
        raise TypeError("limits must be SafeSetLimits")
    if not isinstance(commands, (tuple, list)) or not commands:
        return "unsafe"
    if controller_mode == "position_hold":
        measured_position = _finite_vector(terminal_position_m, 3)
        if measured_position is None:
            return "unsafe"
        for command in commands:
            if (
                not isinstance(command, dict)
                or command.get("kind") != POSITION_COMMAND
            ):
                return "unsafe"
            target = _finite_vector(command.get("position_m"), 3)
            if target is None or math.dist(target, measured_position) > (
                limits.terminal_position_handoff_tolerance_m+1e-12
            ):
                return "unsafe"
        return "position_handoff"
    for command in commands:
        if (
            not isinstance(command, dict)
            or command.get("kind") != ATTITUDE_ZDISTANCE_COMMAND
        ):
            return "unsafe"
        roll = _finite_scalar(command.get("roll_deg"))
        pitch = _finite_scalar(command.get("pitch_deg"))
        if roll is None or pitch is None:
            return "unsafe"
        if math.radians(max(abs(roll), abs(pitch))) > (
            limits.terminal_command_tolerance+1e-12
        ):
            return "unsafe"
    return "level_attitude_hold"


@dataclass(frozen=True)
class ReleaseLMPCTerminalSample:
    """One measured-state/applied-command sample.

    Vector fields deliberately remain unvalidated here.  This lets
    :meth:`ReleaseLMPCTerminalGate.update` fail closed with a status instead of
    forcing callers to catch exceptions while handling imperfect log data.
    Angles and angular rates are in radians and radians/second.
    """

    state_time_s: float
    velocity_xy_m_s: tuple[float, float]
    attitude_rp_rad: tuple[float, float]
    attitude_rate_rp_rad_s: tuple[float, float]
    applied_command_kind: str
    applied_attitude_rp_rad: tuple[float, float]
    state_age_s: float
    state_group_skew_s: float
    boundary_margin_m: float
    pending_command_kinds: tuple[str, ...] = ()
    pending_attitude_rp_rad: tuple[tuple[float, float], ...] = ()


@dataclass(frozen=True)
class ReleaseLMPCTerminalGateStatus:
    """Immutable audit result returned by every lifecycle operation."""

    phase: str
    reason: str
    active: bool
    complete: bool
    sample_in_terminal_set: bool
    dwell_s: float
    consecutive_terminal_samples: int
    update_count: int
    last_state_time_s: float | None
    sample_dt_s: float | None
    violations: tuple[str, ...] = ()

    def to_dict(self):
        """Return a JSON-ready representation for flight-log auditing."""
        result = asdict(self)
        result["violations"] = list(self.violations)
        return result


def _finite_scalar(value):
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _finite_pair(value):
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        return None
    first = _finite_scalar(value[0])
    second = _finite_scalar(value[1])
    if first is None or second is None:
        return None
    return first, second


def _finite_vector(value, length):
    if not isinstance(value, (tuple, list)) or len(value) != length:
        return None
    result = tuple(_finite_scalar(item) for item in value)
    if any(item is None for item in result):
        return None
    return result


class ReleaseLMPCTerminalGate:
    """Track continuous measured release-to-rest terminal dwell.

    ``start`` must be called for each release.  A qualifying first sample starts
    the dwell at zero; only intervals to later qualifying samples are credited.
    A timestamp gap or too-small/non-increasing timestamp breaks continuity,
    but the current valid terminal sample may seed a new dwell.  Completion is
    revalidated on every later sample until the real position handoff closes
    the episode; any violation returns the gate to tracking.
    """

    def __init__(self, limits=None):
        self.limits = SafeSetLimits() if limits is None else limits
        if not isinstance(self.limits, SafeSetLimits):
            raise TypeError("limits must be SafeSetLimits")
        self._phase = "idle"
        self._dwell_s = 0.0
        self._consecutive_terminal_samples = 0
        self._update_count = 0
        self._last_state_time_s = None
        self._last_sample_dt_s = None
        self._last_reason = "reset"
        self._last_sample_in_terminal_set = False
        self._last_violations = ()

    def _status(
            self,
            reason,
            *,
            sample_in_terminal_set=False,
            violations=(),
    ):
        self._last_reason = reason
        self._last_sample_in_terminal_set = bool(sample_in_terminal_set)
        self._last_violations = tuple(violations)
        return ReleaseLMPCTerminalGateStatus(
            phase=self._phase,
            reason=reason,
            active=self._phase != "idle",
            complete=self._phase == "complete",
            sample_in_terminal_set=self._last_sample_in_terminal_set,
            dwell_s=float(self._dwell_s),
            consecutive_terminal_samples=(
                self._consecutive_terminal_samples
            ),
            update_count=self._update_count,
            last_state_time_s=self._last_state_time_s,
            sample_dt_s=self._last_sample_dt_s,
            violations=self._last_violations,
        )

    def _clear_tracking(self):
        self._dwell_s = 0.0
        self._consecutive_terminal_samples = 0
        self._update_count = 0
        self._last_state_time_s = None
        self._last_sample_dt_s = None

    def _break_dwell(self):
        self._dwell_s = 0.0
        self._consecutive_terminal_samples = 0

    @property
    def status(self):
        """Return the current state without changing the gate."""
        return ReleaseLMPCTerminalGateStatus(
            phase=self._phase,
            reason=self._last_reason,
            active=self._phase != "idle",
            complete=self._phase == "complete",
            sample_in_terminal_set=self._last_sample_in_terminal_set,
            dwell_s=float(self._dwell_s),
            consecutive_terminal_samples=(
                self._consecutive_terminal_samples
            ),
            update_count=self._update_count,
            last_state_time_s=self._last_state_time_s,
            sample_dt_s=self._last_sample_dt_s,
            violations=self._last_violations,
        )

    def reset(self):
        """Disarm the gate and discard all timestamp/dwell history."""
        self._clear_tracking()
        self._phase = "idle"
        return self._status("reset")

    def start(self):
        """Arm a fresh release episode and discard any previous history."""
        self._clear_tracking()
        self._phase = "tracking"
        return self._status("started")

    def _malformed(self, violations):
        # A malformed record cannot provide a trustworthy state-time anchor.
        # Dropping it forces the next good record to establish a fresh streak.
        self._break_dwell()
        if self._phase != "idle":
            self._phase = "tracking"
        self._last_state_time_s = None
        self._last_sample_dt_s = None
        return self._status(
            violations[0],
            sample_in_terminal_set=False,
            violations=violations,
        )

    def update(self, sample):
        """Consume one sample and return a fail-closed audit status.

        Malformed/non-finite input, stale or skewed state, unsafe boundary,
        non-terminal measured motion, and a non-attitude applied command all
        reset the continuous dwell.  No such sample can report completion.
        """
        if self._phase == "idle":
            return self._status(
                "not_started", violations=("not_started",)
            )
        self._update_count += 1
        if not isinstance(sample, ReleaseLMPCTerminalSample):
            return self._malformed(("invalid_sample_type",))

        scalar_names = (
            "state_time_s", "state_age_s", "state_group_skew_s",
            "boundary_margin_m",
        )
        scalars = {
            name: _finite_scalar(getattr(sample, name))
            for name in scalar_names
        }
        malformed = []
        for name, value in scalars.items():
            if value is None:
                malformed.append("invalid_"+name.removesuffix("_s"))

        velocity = _finite_pair(sample.velocity_xy_m_s)
        attitude = _finite_pair(sample.attitude_rp_rad)
        attitude_rate = _finite_pair(sample.attitude_rate_rp_rad_s)
        command = _finite_pair(sample.applied_attitude_rp_rad)
        pending_kinds = sample.pending_command_kinds
        pending_commands = sample.pending_attitude_rp_rad
        if velocity is None:
            malformed.append("invalid_velocity_xy")
        if attitude is None:
            malformed.append("invalid_attitude_rp")
        if attitude_rate is None:
            malformed.append("invalid_attitude_rate_rp")
        if command is None:
            malformed.append("invalid_applied_attitude_rp")
        if not isinstance(sample.applied_command_kind, str):
            malformed.append("invalid_applied_command_kind")
        if (
            not isinstance(pending_kinds, (tuple, list))
            or not isinstance(pending_commands, (tuple, list))
            or len(pending_kinds) != len(pending_commands)
        ):
            malformed.append("invalid_pending_command_history")
            pending_kinds = ()
            pending_commands = ()
        else:
            for index, (kind, pending_command) in enumerate(zip(
                    pending_kinds, pending_commands)):
                if not isinstance(kind, str):
                    malformed.append(
                        f"invalid_pending_command_kind_{index}"
                    )
                if _finite_pair(pending_command) is None:
                    malformed.append(
                        f"invalid_pending_attitude_rp_{index}"
                    )
        if malformed:
            return self._malformed(tuple(malformed))

        state_time = scalars["state_time_s"]
        timing_reason = None
        sample_dt = None
        if self._last_state_time_s is not None:
            sample_dt = state_time-self._last_state_time_s
            if sample_dt < self.limits.min_sample_dt_s-1e-12:
                timing_reason = "sample_dt_below_min"
            elif sample_dt > self.limits.max_sample_dt_s+1e-12:
                timing_reason = "sample_dt_above_max"
        self._last_state_time_s = state_time
        self._last_sample_dt_s = sample_dt

        violations = []
        if timing_reason is not None:
            violations.append(timing_reason)
        state_age = scalars["state_age_s"]
        if state_age < 0.0 or state_age > self.limits.max_state_age_s:
            violations.append("state_not_fresh")
        group_skew = scalars["state_group_skew_s"]
        if (
            group_skew < 0.0
            or group_skew > self.limits.max_state_group_skew_s
        ):
            violations.append("state_group_skew_exceeded")
        if (
            scalars["boundary_margin_m"]
            < self.limits.min_path_boundary_margin_m
        ):
            violations.append("boundary_margin_too_small")

        xy_speed = math.hypot(*velocity)
        if xy_speed > self.limits.terminal_velocity_tolerance_m_s:
            violations.append("xy_speed_above_terminal_limit")
        if max(abs(value) for value in attitude) > (
            self.limits.terminal_tilt_tolerance_rad
        ):
            violations.append("attitude_above_terminal_limit")
        if max(abs(value) for value in attitude_rate) > (
            self.limits.terminal_rate_tolerance_rad_s
        ):
            violations.append("attitude_rate_above_terminal_limit")
        if sample.applied_command_kind != ATTITUDE_ZDISTANCE_COMMAND:
            violations.append("applied_command_kind_not_attitude_zdistance")
        if max(abs(value) for value in command) > (
            self.limits.terminal_command_tolerance
        ):
            violations.append("applied_attitude_above_terminal_limit")
        if any(
            kind != ATTITUDE_ZDISTANCE_COMMAND for kind in pending_kinds
        ):
            violations.append("pending_command_kind_not_attitude_zdistance")
        if any(
            max(abs(value) for value in _finite_pair(pending_command))
            > self.limits.terminal_command_tolerance
            for pending_command in pending_commands
        ):
            violations.append("pending_attitude_above_terminal_limit")

        # A bad interval breaks the old streak, but if the current sample is
        # otherwise terminal it is a valid zero-duration start for a new one.
        non_timing_violations = tuple(
            item for item in violations if item != timing_reason
        )
        if non_timing_violations:
            self._break_dwell()
            self._phase = "tracking"
            return self._status(
                non_timing_violations[0],
                sample_in_terminal_set=False,
                violations=tuple(violations),
            )

        if timing_reason is not None:
            self._break_dwell()
            self._phase = "tracking"
            self._consecutive_terminal_samples = 1
            return self._status(
                timing_reason,
                sample_in_terminal_set=True,
                violations=(timing_reason,),
            )

        if self._consecutive_terminal_samples == 0:
            self._consecutive_terminal_samples = 1
            self._dwell_s = 0.0
            return self._status(
                "terminal_dwell_started", sample_in_terminal_set=True
            )

        # At this point there is a previous qualifying sample and the interval
        # has been checked against both sample-dt limits.
        self._consecutive_terminal_samples += 1
        self._dwell_s += sample_dt
        if self._dwell_s+1e-12 >= self.limits.terminal_dwell_s:
            self._phase = "complete"
            return self._status(
                "terminal_dwell_complete", sample_in_terminal_set=True
            )
        return self._status(
            "terminal_dwell_accumulating", sample_in_terminal_set=True
        )
