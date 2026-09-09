"""Independent, offline-only delayed tilt plant for braking validation.

This numerical plant is deliberately separate from the candidate predictor's
analytic transfer-function solution. It does not fit a model, connect to a
vehicle, or generate flight commands. Its one-dimensional state is projected on
the original travel axis; negative tilt accelerates against that axis.

The model is only a diagnostic hypothesis. It omits localization corrections,
cross-axis coupling, battery/thrust limits and position-controller behaviour.
Passing a simulation is not evidence of a physical stopping guarantee.
"""
from __future__ import annotations

from bisect import bisect_right
from dataclasses import asdict
import math

from Interaction.offline_braking_selector import BrakingSnapshot, FrozenTiltModel


class DelayedTiltPlant:
    """RK4 plant with exact command boundaries and causal state history.

    ``command_history`` contains ordered, already SENT ``(time_s, tilt_rad)``
    pairs, including commands still pending at the initial snapshot. Empty
    history assumes a level command. A new command may be sent only at the
    current simulated time: future commands must not be disclosed to a causal
    controller before they are sent.

    The total plant command delay is ``model.delay_s + transport_delay_s``.
    The optional constant acceleration disturbance is an explicit sensitivity
    input, not a fitted drag or estimator correction.
    """

    offline_only = True
    flight_command_generated = False

    def __init__(
        self,
        model: FrozenTiltModel,
        initial_snapshot: BrakingSnapshot,
        command_history=(),
        *,
        integration_step_s=0.001,
        transport_delay_s=0.0,
        disturbance_acceleration_m_s2=0.0,
    ):
        try:
            numeric = [*asdict(model).values(), *asdict(initial_snapshot).values(),
                       integration_step_s, transport_delay_s,
                       disturbance_acceleration_m_s2]
            if not all(math.isfinite(float(value)) for value in numeric):
                raise ValueError("plant inputs must be finite")
        except (TypeError, AttributeError, OverflowError) as exc:
            raise ValueError("invalid plant model or snapshot") from exc
        if (model.delay_s < 0 or model.wn_rad_s <= 0 or model.zeta <= 0
                or model.command_gain <= 0 or model.motion_gain <= 0
                or abs(model.projected_bias_rad) >= math.radians(80)
                or abs(initial_snapshot.tilt_rad) >= math.radians(80)
                or not 0 < integration_step_s <= 0.002
                or transport_delay_s < 0):
            raise ValueError("plant model, state or integration settings outside domain")

        self.model = model
        self.integration_step_s = float(integration_step_s)
        self.transport_delay_s = float(transport_delay_s)
        self.disturbance_acceleration_m_s2 = float(disturbance_acceleration_m_s2)
        self._total_delay_s = float(model.delay_s + transport_delay_s)
        if not math.isfinite(self._total_delay_s):
            raise ValueError("total command delay is not finite")
        self._current = initial_snapshot
        self._states = [initial_snapshot]
        self._state_times = [float(initial_snapshot.time_s)]
        self._commands = []
        self._effective_times = []
        try:
            for pair in command_history:
                sent_time_s, tilt_rad = pair
                sent_time_s, tilt_rad = self._validate_command(sent_time_s, tilt_rad)
                if sent_time_s > initial_snapshot.time_s:
                    raise ValueError("constructor history contains a future-sent command")
                if self._commands and sent_time_s < self._commands[-1][0]:
                    raise ValueError("command history must be time ordered")
                self._append_command(sent_time_s, tilt_rad)
        except (TypeError, OverflowError) as exc:
            raise ValueError("malformed sent-command history") from exc

    @property
    def current_snapshot(self):
        return self._current

    @property
    def command_history(self):
        """Immutable copy of SENT commands; no unsent future actions."""
        return tuple(self._commands)

    @property
    def snapshot_history(self):
        """Immutable snapshots at integration boundaries, for offline scoring."""
        return tuple(self._states)

    @staticmethod
    def _validate_command(time_s, tilt_rad):
        time_s, tilt_rad = float(time_s), float(tilt_rad)
        if (not math.isfinite(time_s) or not math.isfinite(tilt_rad)
                or abs(tilt_rad) >= math.radians(80)):
            raise ValueError("command time/angle is invalid or outside domain")
        return time_s, tilt_rad

    def _append_command(self, time_s, tilt_rad):
        effective_time_s = time_s + self._total_delay_s
        if not math.isfinite(effective_time_s):
            raise ValueError("effective command time is not finite")
        self._commands.append((time_s, tilt_rad))
        self._effective_times.append(effective_time_s)

    def send_command(self, time_s, tilt_rad):
        """Record a command sent now, preserving any earlier pending commands."""
        time_s, tilt_rad = self._validate_command(time_s, tilt_rad)
        if not math.isclose(time_s, self._current.time_s, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError("send commands only at the current simulated time")
        # Normalize a roundoff-only discrepancy to the actual send instant.
        self._append_command(float(self._current.time_s), tilt_rad)

    def snapshot_at_or_before(self, time_s):
        """Return only an available past state, without future interpolation.

        ``None`` means the query predates the available simulation history.
        Requesting a state after the current simulated time is an error.
        """
        time_s = float(time_s)
        if not math.isfinite(time_s) or time_s > self._current.time_s:
            raise ValueError("state query must be finite and not in the future")
        index = bisect_right(self._state_times, time_s) - 1
        return None if index < 0 else self._states[index]

    def get_delayed_snapshot(self, delay_s):
        delay_s = float(delay_s)
        if not math.isfinite(delay_s) or delay_s < 0:
            raise ValueError("measurement delay must be finite and nonnegative")
        return self.snapshot_at_or_before(self._current.time_s - delay_s)

    def _derivative(self, state, command_rad):
        position, velocity, tilt, rate = state
        del position
        if (not all(math.isfinite(x) for x in state)
                or abs(tilt) >= math.radians(80)):
            raise ValueError("plant integration stage left the finite small-tilt domain")
        target = self.model.command_gain * command_rad + self.model.projected_bias_rad
        return (
            velocity,
            self.model.motion_gain * 9.81 * math.tan(tilt)
            + self.disturbance_acceleration_m_s2,
            rate,
            self.model.wn_rad_s ** 2 * (target - tilt)
            - 2 * self.model.zeta * self.model.wn_rad_s * rate,
        )

    def _rk4_step(self, state, dt, command_rad):
        # command_rad is fixed throughout this half-open integration interval.
        # In particular k4 must not apply the next command at an event endpoint.
        k1 = self._derivative(state, command_rad)
        k2 = self._derivative(tuple(x + 0.5 * dt * k for x, k in zip(state, k1)), command_rad)
        k3 = self._derivative(tuple(x + 0.5 * dt * k for x, k in zip(state, k2)), command_rad)
        k4 = self._derivative(tuple(x + dt * k for x, k in zip(state, k3)), command_rad)
        return tuple(x + dt * (a + 2 * b + 2 * c + d) / 6
                     for x, a, b, c, d in zip(state, k1, k2, k3, k4))

    def advance_to(self, time_s):
        """Integrate forward, splitting exactly at delayed command switches."""
        time_s = float(time_s)
        if not math.isfinite(time_s) or time_s < self._current.time_s:
            raise ValueError("advance time must be finite and nondecreasing")
        while self._current.time_s < time_s:
            now = float(self._current.time_s)
            next_event_index = bisect_right(self._effective_times, now)
            command_rad = self._commands[next_event_index - 1][1] if next_event_index else 0.0
            end_time = min(time_s, now + self.integration_step_s)
            if next_event_index < len(self._effective_times):
                end_time = min(end_time, self._effective_times[next_event_index])
            dt = end_time - now
            if dt <= 0:
                raise ValueError("integration step cannot advance at this time scale")
            state = (self._current.position_m, self._current.velocity_m_s,
                     self._current.tilt_rad, self._current.tilt_rate_rad_s)
            try:
                position, velocity, tilt, rate = self._rk4_step(state, dt, command_rad)
            except OverflowError as exc:
                raise ValueError("plant integration exceeded numeric domain") from exc
            if (not all(math.isfinite(x) for x in (position, velocity, tilt, rate))
                    or abs(tilt) >= math.radians(80)):
                raise ValueError("plant trajectory left the finite small-tilt domain")
            self._current = BrakingSnapshot(end_time, position, velocity, tilt, rate)
            self._states.append(self._current)
            self._state_times.append(end_time)
        return self._current
