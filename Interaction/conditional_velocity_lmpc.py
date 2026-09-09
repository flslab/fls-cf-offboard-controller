"""Offline conditional Learning MPC for post-release velocity braking.

This module implements the defining optimization structure from Learning MPC:

* successful iterations supply sampled safe states and cost-to-go values;
* the prediction horizon terminates in a convex combination of local safe
  states; and
* that same convex combination supplies the terminal cost.

The repeated task here is adapted from a fixed-start quadrotor maneuver to
post-release braking with a varying initial speed.  Direction and release
context are therefore selected by :mod:`Interaction.velocity_lmpc_safe_set`
before this optimizer is called.  The plant model is the identified closed-loop
attitude response used by the existing Crazyflie roll/pitch command interface,
not the paper's thrust/body-rate interface.

The SciPy SLSQP implementation is deliberately offline-only.  It has no device
I/O and its output must never be sent to a vehicle.  A future online adapter
needs a bounded-time solver, a recoverability filter, validated safe-set
coverage, and a legacy braking fallback.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
import time
from typing import Sequence

import numpy as np

from Interaction.model_based_braking import _second_order_transition
from Interaction.offline_braking_selector import FrozenTiltModel


def conditional_velocity_lmpc_fingerprint(
        model: FrozenTiltModel,
        config: "ConditionalVelocityLMPCConfig",
        safe_set_limits=None) -> str:
    """Bind a safe-set partition to dynamics, timing, and hard constraints."""
    from Interaction.velocity_lmpc_safe_set import SafeSetLimits

    limits = SafeSetLimits() if safe_set_limits is None else safe_set_limits
    if not isinstance(limits, SafeSetLimits):
        raise TypeError("safe_set_limits must be SafeSetLimits")
    payload = {
        "schema": "conditional_velocity_lmpc_reduced_v2",
        "model": asdict(model),
        "prediction_step_s": config.prediction_step_s,
        "max_integration_substep_s": config.max_integration_substep_s,
        "max_command_tilt_deg": config.max_command_tilt_deg,
        "max_command_slew_deg_s": config.max_command_slew_deg_s,
        "max_tilt_deg": config.max_tilt_deg,
        "max_tilt_rate_deg_s": config.max_tilt_rate_deg_s,
        "reverse_velocity_tolerance_m_s": (
            config.reverse_velocity_tolerance_m_s
        ),
        "max_abs_velocity_m_s": config.max_abs_velocity_m_s,
        "safe_set_limits": limits.to_dict(),
    }
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return "reduced-v2:"+hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class ConditionalVelocityLMPCConfig:
    """Bounds and numerical settings for the offline reduced-order LMPC."""

    prediction_step_s: float = 0.02
    max_integration_substep_s: float = 0.001
    horizon_steps: int = 10
    max_command_tilt_deg: float = 8.0
    max_command_slew_deg_s: float = 180.0
    max_tilt_deg: float = 15.0
    max_tilt_rate_deg_s: float = 300.0
    reverse_velocity_tolerance_m_s: float = 0.02
    max_abs_velocity_m_s: float = 2.0
    effort_weight: float = 1e-3
    slew_weight: float = 2e-3
    solver_max_iterations: int = 150
    solver_ftol: float = 1e-9
    equality_tolerance: float = 2e-5
    inequality_tolerance: float = 2e-6
    default_solver_deadline_s: float = 0.50

    def validate(self) -> None:
        values = asdict(self)
        integer_keys = {"horizon_steps", "solver_max_iterations"}
        for key, value in values.items():
            if key in integer_keys:
                if isinstance(value, bool) or int(value) != value:
                    raise ValueError(f"{key} must be an integer")
            elif isinstance(value, bool) or not math.isfinite(float(value)):
                raise ValueError(f"{key} must be finite")
        if not 0.005 <= self.prediction_step_s <= 0.10:
            raise ValueError("prediction_step_s must be in [0.005, 0.10]")
        if not 0.0005 <= self.max_integration_substep_s <= min(
            0.001, self.prediction_step_s
        ):
            raise ValueError(
                "max_integration_substep_s must be in [0.0005, 0.001] and "
                "not exceed prediction_step_s"
            )
        if not 2 <= self.horizon_steps <= 100:
            raise ValueError("horizon_steps must be in [2, 100]")
        if not 0 < self.max_command_tilt_deg < 30:
            raise ValueError("max_command_tilt_deg must be in (0, 30)")
        if not self.max_command_tilt_deg <= self.max_tilt_deg < 30:
            raise ValueError("max_tilt_deg must cover commands and be below 30")
        if not 10 <= self.max_command_slew_deg_s <= 720:
            raise ValueError("max_command_slew_deg_s must be in [10, 720]")
        if not 0 < self.max_tilt_rate_deg_s <= 1000:
            raise ValueError("max_tilt_rate_deg_s must be in (0, 1000]")
        if not 0 <= self.reverse_velocity_tolerance_m_s <= 0.05:
            raise ValueError("reverse velocity tolerance must be in [0, 0.05]")
        if not 0 < self.max_abs_velocity_m_s <= 5.0:
            raise ValueError("max_abs_velocity_m_s must be in (0, 5]")
        if self.effort_weight < 0 or self.slew_weight < 0:
            raise ValueError("objective weights cannot be negative")
        if not 10 <= self.solver_max_iterations <= 1000:
            raise ValueError("solver_max_iterations must be in [10, 1000]")
        if not 0 < self.solver_ftol <= 1e-3:
            raise ValueError("solver_ftol must be in (0, 1e-3]")
        if not 0 < self.equality_tolerance <= 1e-2:
            raise ValueError("equality_tolerance must be in (0, 1e-2]")
        if not 0 <= self.inequality_tolerance <= 1e-3:
            raise ValueError("inequality_tolerance must be in [0, 1e-3]")
        if not 0.01 <= self.default_solver_deadline_s <= 10:
            raise ValueError("default_solver_deadline_s must be in [0.01, 10]")


@dataclass(frozen=True)
class ConditionalVelocityLMPCState:
    """Markov state in the release-aligned one-dimensional braking frame.

    Positive velocity is the measured release direction.  A braking command is
    normally negative.  ``pending_commands_rad`` is ordered oldest/effective
    first and must contain ``ceil(model.delay_s / dt)`` commands.  Including
    the delayed-command memory prevents the optimizer from treating a
    delayed/slew-limited system as memoryless.  When the queue is nonempty its
    newest entry is also the previous command, so that duplicate coordinate is
    omitted from the optimizer state.
    """

    velocity_m_s: float
    projected_tilt_rad: float
    projected_tilt_rate_rad_s: float
    previous_command_rad: float
    pending_commands_rad: tuple[float, ...] = ()
    available_distance_m: float = math.inf

    def vector(self, *, delay_steps: int) -> np.ndarray:
        queue = np.asarray(self.pending_commands_rad, dtype=float)
        values = np.asarray([
            self.velocity_m_s,
            self.projected_tilt_rad,
            self.projected_tilt_rate_rad_s,
            self.previous_command_rad,
            *queue,
            self.available_distance_m,
        ], dtype=float)
        if queue.shape != (delay_steps,):
            raise ValueError(
                "pending command count does not match the model delay"
            )
        if not np.isfinite(values[:-1]).all():
            raise ValueError("LMPC state contains nonfinite dynamic values")
        if not (math.isinf(self.available_distance_m)
                or math.isfinite(self.available_distance_m)):
            raise ValueError("available distance must be finite or +inf")
        if self.available_distance_m <= 0:
            raise ValueError("available distance must be positive")
        if delay_steps:
            if not math.isclose(
                    self.previous_command_rad, float(queue[-1]),
                    rel_tol=0.0, abs_tol=1e-9):
                raise ValueError(
                    "previous command does not match the newest delayed command"
                )
            return np.r_[values[:3], queue]
        return values[:4]


@dataclass(frozen=True)
class ConditionalVelocityLMPCPlan:
    """Auditable optimizer result; feasible plans remain offline-only.

    A convex terminal safe-set relaxation for this nonlinear model is not a
    physical safety certificate.  ``optimizer_feasible`` means only that the
    configured numerical problem passed its explicit residual checks.
    """

    optimizer_feasible: bool
    reason: str
    offline_only: bool
    flight_command_dispatched: bool
    safety_certified: bool
    commands_rad: tuple[float, ...]
    terminal_weights: tuple[float, ...]
    predicted_states: tuple[tuple[float, ...], ...]
    predicted_distance_m: float | None
    objective: float | None
    equality_residual_inf: float | None
    minimum_inequality_margin: float | None
    solve_time_s: float
    iterations: int

    @property
    def first_command_rad(self) -> float | None:
        return (
            self.commands_rad[0]
            if self.optimizer_feasible and self.commands_rad else None
        )


class _SolverDeadlineExceeded(RuntimeError):
    pass


class OfflineConditionalVelocityLMPC:
    """Paper-structured LMPC optimizer for one conditional safe-set query."""

    def __init__(self, model: FrozenTiltModel,
                 config: ConditionalVelocityLMPCConfig | None = None,
                 safe_set_limits=None):
        from Interaction.velocity_lmpc_safe_set import SafeSetLimits

        self.model = model
        self.config = config or ConditionalVelocityLMPCConfig()
        self.config.validate()
        self.safe_set_limits = (
            SafeSetLimits() if safe_set_limits is None else safe_set_limits
        )
        if not isinstance(self.safe_set_limits, SafeSetLimits):
            raise TypeError("safe_set_limits must be SafeSetLimits")
        self.model_fingerprint = conditional_velocity_lmpc_fingerprint(
            model, self.config, self.safe_set_limits
        )
        model_values = np.asarray([
            model.delay_s,
            model.wn_rad_s,
            model.zeta,
            model.command_gain,
            model.motion_gain,
            model.projected_bias_rad,
        ], dtype=float)
        if (not np.isfinite(model_values).all() or model.delay_s < 0
                or model.wn_rad_s <= 0 or model.zeta <= 0
                or model.command_gain <= 0 or model.motion_gain <= 0
                or abs(model.projected_bias_rad) >= math.radians(30)):
            raise ValueError("frozen model is outside the supported domain")
        delay_ratio = model.delay_s/self.config.prediction_step_s
        self.delay_steps = int(math.ceil(delay_ratio-1e-12))
        whole_delay_steps = int(math.floor(delay_ratio+1e-12))
        self.delay_remainder_s = float(
            model.delay_s-whole_delay_steps*self.config.prediction_step_s
        )
        if self.delay_remainder_s < 1e-12:
            self.delay_remainder_s = 0.0
        self.state_dimension = 3+max(1, self.delay_steps)
        self._transition = _second_order_transition(
            model.wn_rad_s,
            model.zeta,
            self.config.prediction_step_s,
        )

    @staticmethod
    def _forward_distance_upper_bound(velocity_before, velocity_after, dt):
        """Upper-bound displacement of one piecewise-linear velocity segment."""
        return float(max(0.0, velocity_before, velocity_after)*dt)

    def _propagate_segment(self, velocity, tilt, tilt_rate,
                           effective_command, dt):
        """Integrate one constant-input segment and retain every audit node.

        The attitude transition is exact for each constant-input substep.  The
        translational state uses the module's trapezoidal discretization.  A
        small bounded substep prevents a fractional-delay switch or a coarse
        prediction node from hiding a numerical-model attitude/rate/velocity
        constraint excursion.  These nodes are still a discretized-model
        contract, not a continuous-time physical certificate.
        """
        if dt <= 0:
            state = np.asarray([velocity, tilt, tilt_rate], dtype=float)
            return (*state, 0.0, np.empty((0, 3), dtype=float))
        substep_count = max(
            1, int(math.ceil(dt/self.config.max_integration_substep_s-1e-12))
        )
        substep_s = float(dt/substep_count)
        transition = _second_order_transition(
            self.model.wn_rad_s, self.model.zeta, substep_s
        )
        equilibrium = (
            self.model.command_gain*effective_command
            + self.model.projected_bias_rad
        )
        current_velocity = float(velocity)
        current_tilt = float(tilt)
        current_rate = float(tilt_rate)
        forward_distance = 0.0
        audit_nodes = []
        for _ in range(substep_count):
            centered = transition @ np.asarray([
                current_tilt-equilibrium, current_rate,
            ])
            next_tilt = float(centered[0]+equilibrium)
            next_rate = float(centered[1])
            acceleration_before = (
                self.model.motion_gain*9.81*math.tan(current_tilt)
            )
            acceleration_after = (
                self.model.motion_gain*9.81*math.tan(next_tilt)
            )
            next_velocity = float(
                current_velocity
                + 0.5*(acceleration_before+acceleration_after)*substep_s
            )
            forward_distance += self._forward_distance_upper_bound(
                current_velocity, next_velocity, substep_s
            )
            audit_nodes.append((next_velocity, next_tilt, next_rate))
            current_velocity = next_velocity
            current_tilt = next_tilt
            current_rate = next_rate
        return (
            current_velocity,
            current_tilt,
            current_rate,
            float(forward_distance),
            np.asarray(audit_nodes, dtype=float),
        )

    def _next_state_detailed(
        self, state: np.ndarray, command: float
    ) -> tuple[np.ndarray, float, np.ndarray]:
        dt = self.config.prediction_step_s
        velocity, tilt, tilt_rate = state[:3]
        queue = state[3:] if self.delay_steps else np.empty(0, dtype=float)
        audit_parts = []
        if self.delay_steps and self.delay_remainder_s > 0:
            (
                next_velocity, next_tilt, next_rate, distance_first,
                audit_first,
            ) = (
                self._propagate_segment(
                    velocity, tilt, tilt_rate, queue[0],
                    self.delay_remainder_s,
                )
            )
            audit_parts.append(audit_first)
            second_effective = queue[1] if self.delay_steps > 1 else command
            (
                next_velocity, next_tilt, next_rate, distance_second,
                audit_second,
            ) = (
                self._propagate_segment(
                    next_velocity, next_tilt, next_rate, second_effective,
                    dt-self.delay_remainder_s,
                )
            )
            audit_parts.append(audit_second)
            distance = distance_first+distance_second
        else:
            effective_command = queue[0] if self.delay_steps else command
            next_velocity, next_tilt, next_rate, distance, audit = (
                self._propagate_segment(
                    velocity, tilt, tilt_rate, effective_command, dt
                )
            )
            audit_parts.append(audit)
        if self.delay_steps:
            next_queue = np.r_[queue[1:], command]
        else:
            next_queue = np.empty(0, dtype=float)
        command_memory = next_queue if self.delay_steps else [command]
        next_state = np.r_[
            next_velocity, next_tilt, next_rate, command_memory,
        ]
        audit_nodes = np.vstack([
            part for part in audit_parts if part.size
        ]) if any(part.size for part in audit_parts) else np.empty((0, 3))
        return next_state, float(distance), audit_nodes

    def _next_state(self, state: np.ndarray, command: float) -> tuple[np.ndarray, float]:
        next_state, distance, _audit_nodes = self._next_state_detailed(
            state, command
        )
        return next_state, distance

    def _rollout_detailed(
        self, initial_state: ConditionalVelocityLMPCState,
        commands_rad: Sequence[float],
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        state = initial_state.vector(delay_steps=self.delay_steps)
        commands = np.asarray(commands_rad, dtype=float)
        if commands.ndim != 1 or not np.isfinite(commands).all():
            raise ValueError("commands must be a finite one-dimensional sequence")
        states = [state.copy()]
        distances = []
        audit_nodes = []
        for command in commands:
            state, distance, nodes = self._next_state_detailed(
                state, float(command)
            )
            states.append(state)
            distances.append(distance)
            audit_nodes.extend(nodes)
        return (
            np.asarray(states),
            np.asarray(distances),
            np.asarray(audit_nodes, dtype=float).reshape((-1, 3)),
        )

    def rollout(self, initial_state: ConditionalVelocityLMPCState,
                commands_rad: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
        """Roll out the learned closed-loop model without solving an NLP."""
        states, distances, _audit_nodes = self._rollout_detailed(
            initial_state, commands_rad
        )
        return states, distances

    def _prepare_warm_commands(
        self,
        initial_vector: np.ndarray,
        warm_start_commands_rad: Sequence[float] | None,
    ) -> np.ndarray:
        """Return the exact bounded/slew-limited sequence used for local query.

        LMPC selects local terminal safe states around the predicted end of the
        warm trajectory.  Preparing that trajectory in one place keeps the
        query and nonlinear program consistent, including when no previous
        receding-horizon solution was supplied.
        """
        c = self.config
        horizon = c.horizon_steps
        command_limit = math.radians(c.max_command_tilt_deg)
        slew_step = math.radians(c.max_command_slew_deg_s)*c.prediction_step_s
        previous = float(initial_vector[-1])
        if warm_start_commands_rad is not None:
            warm = np.asarray(warm_start_commands_rad, dtype=float)
            if warm.shape != (horizon,) or not np.isfinite(warm).all():
                raise ValueError("invalid warm start commands")
            if np.any(np.abs(warm) > command_limit+1e-12):
                raise ValueError("warm start command exceeds the hard bound")
            changes = np.diff(np.r_[previous, warm])
            if np.any(np.abs(changes) > slew_step+1e-12):
                raise ValueError("warm start command exceeds the slew bound")
            return warm.copy()

        braking_steps = max(1, horizon//2)
        target = -command_limit
        values = []
        for index in range(horizon):
            desired = target if index < braking_steps else 0.0
            previous += float(np.clip(
                desired-previous, -slew_step, slew_step
            ))
            previous = float(np.clip(previous, -command_limit, command_limit))
            values.append(previous)
        return np.asarray(values, dtype=float)

    def _failure(self, reason: str, *, started_at: float,
                 iterations: int = 0) -> ConditionalVelocityLMPCPlan:
        return ConditionalVelocityLMPCPlan(
            optimizer_feasible=False,
            reason=reason,
            offline_only=True,
            flight_command_dispatched=False,
            safety_certified=False,
            commands_rad=(),
            terminal_weights=(),
            predicted_states=(),
            predicted_distance_m=None,
            objective=None,
            equality_residual_inf=None,
            minimum_inequality_margin=None,
            solve_time_s=float(time.perf_counter()-started_at),
            iterations=int(iterations),
        )

    def solve(self, initial_state: ConditionalVelocityLMPCState, *,
              safe_states: Sequence[Sequence[float]],
              safe_cost_to_go: Sequence[float],
              safe_release_speeds_m_s: Sequence[float] | None = None,
              release_initial_speed_m_s: float | None = None,
              safe_tail_forward_distances_m: Sequence[float] | None = None,
              warm_start_commands_rad: Sequence[float] | None = None,
              solver_deadline_s: float | None = None,
              ) -> ConditionalVelocityLMPCPlan:
        """Solve one offline receding-horizon LMPC problem.

        ``safe_states`` must already have been selected for the current release
        context.  Their rows use the same Markov state layout as
        :class:`ConditionalVelocityLMPCState`.  The terminal equality and cost
        are respectively ``x_N = safe_states.T @ lambda`` and
        ``Q_N = safe_cost_to_go @ lambda`` with simplex-constrained weights.
        For a release speed between two demonstrated tasks, also pass
        ``safe_release_speeds_m_s`` and ``release_initial_speed_m_s``.  The
        optimizer then enforces the paper-style augmented terminal equality
        ``v_release = safe_release_speeds @ lambda``; this prevents it from
        silently discarding either conditional speed bracket.
        """
        started_at = time.perf_counter()
        c = self.config
        command_limit = math.radians(c.max_command_tilt_deg)
        try:
            from scipy.optimize import minimize
        except ImportError:
            return self._failure("scipy_solver_unavailable", started_at=started_at)

        try:
            initial_vector = initial_state.vector(delay_steps=self.delay_steps)
            safe = np.asarray(safe_states, dtype=float)
            costs = np.asarray(safe_cost_to_go, dtype=float)
            if (safe.ndim != 2 or safe.shape[0] < 1
                    or safe.shape[1] != self.state_dimension):
                raise ValueError("safe states have the wrong shape")
            if costs.shape != (safe.shape[0],):
                raise ValueError("safe cost-to-go has the wrong shape")
            if not np.isfinite(safe).all() or not np.isfinite(costs).all():
                raise ValueError("safe set contains nonfinite values")
            if np.any(costs < 0):
                raise ValueError("safe cost-to-go cannot be negative")
            if (initial_vector[0] < -c.reverse_velocity_tolerance_m_s
                    or abs(initial_vector[0]) > c.max_abs_velocity_m_s
                    or abs(initial_vector[1]) > math.radians(c.max_tilt_deg)
                    or abs(initial_vector[2])
                    > math.radians(c.max_tilt_rate_deg_s)
                    or np.any(np.abs(initial_vector[3:]) > command_limit)):
                raise ValueError(
                    "initial state or delayed command violates a hard bound"
                )
            if (np.any(safe[:, 0] < -c.reverse_velocity_tolerance_m_s)
                    or np.any(np.abs(safe[:, 0]) > c.max_abs_velocity_m_s)
                    or np.any(np.abs(safe[:, 1])
                              > math.radians(c.max_tilt_deg))
                    or np.any(np.abs(safe[:, 2])
                              > math.radians(c.max_tilt_rate_deg_s))
                    or np.any(np.abs(safe[:, 3:]) > command_limit)):
                raise ValueError(
                    "safe terminal state violates the current planner bounds"
                )
            if safe_tail_forward_distances_m is None:
                safe_tail_distances = np.zeros(safe.shape[0], dtype=float)
            else:
                safe_tail_distances = np.asarray(
                    safe_tail_forward_distances_m, dtype=float
                )
                if (safe_tail_distances.shape != (safe.shape[0],)
                        or not np.isfinite(safe_tail_distances).all()
                        or np.any(safe_tail_distances < 0)):
                    raise ValueError("safe-tail forward distances are invalid")
            if ((safe_release_speeds_m_s is None)
                    != (release_initial_speed_m_s is None)):
                raise ValueError(
                    "safe and requested release speeds must be supplied together"
                )
            if safe_release_speeds_m_s is None:
                safe_release_speeds = None
                requested_release_speed = None
                condition_speed_equality = False
            else:
                safe_release_speeds = np.asarray(
                    safe_release_speeds_m_s, dtype=float
                )
                requested_release_speed = float(release_initial_speed_m_s)
                if (safe_release_speeds.shape != (safe.shape[0],)
                        or not np.isfinite(safe_release_speeds).all()
                        or not math.isfinite(requested_release_speed)
                        or requested_release_speed <= 0
                        or requested_release_speed > c.max_abs_velocity_m_s
                        or np.any(safe_release_speeds <= 0)):
                    raise ValueError("conditional release speeds are invalid")
                if (requested_release_speed
                        < float(np.min(safe_release_speeds))-1e-12
                        or requested_release_speed
                        > float(np.max(safe_release_speeds))+1e-12):
                    raise ValueError(
                        "requested release speed is outside safe-set coverage"
                    )
                condition_speed_equality = bool(
                    np.ptp(safe_release_speeds) > 1e-12
                )
        except (TypeError, ValueError) as error:
            return self._failure(
                "invalid_problem: "+str(error), started_at=started_at,
            )

        horizon = c.horizon_steps
        count = safe.shape[0]
        deadline_s = (
            c.default_solver_deadline_s
            if solver_deadline_s is None else float(solver_deadline_s)
        )
        if not math.isfinite(deadline_s) or deadline_s <= 0:
            return self._failure("invalid_solver_deadline", started_at=started_at)
        deadline_at = started_at+deadline_s

        def check_deadline() -> None:
            if time.perf_counter() > deadline_at:
                raise _SolverDeadlineExceeded

        try:
            warm = self._prepare_warm_commands(
                initial_vector, warm_start_commands_rad
            )
        except (TypeError, ValueError) as error:
            return self._failure(
                "invalid_warm_start_commands: "+str(error),
                started_at=started_at,
            )
        warm_states, _ = self.rollout(initial_state, warm)
        scales = np.maximum(np.std(safe, axis=0), np.asarray([
            0.05,
            math.radians(1.0),
            math.radians(10.0),
            *([math.radians(1.0)]*self.delay_steps),
            *([] if self.delay_steps else [math.radians(1.0)]),
        ]))
        distances = np.linalg.norm(
            (safe-warm_states[-1])/scales,
            axis=1,
        )
        # Start inside the simplex.  A one-hot vertex frequently gives SLSQP a
        # rank-deficient active-bound Jacobian before it can use the local
        # convex hull, even when the supplied command warm start is feasible.
        inverse_distance = 1.0/np.maximum(distances, 1e-6)
        if condition_speed_equality:
            lower = float(np.max(
                safe_release_speeds[
                    safe_release_speeds <= requested_release_speed+1e-12
                ]
            ))
            upper = float(np.min(
                safe_release_speeds[
                    safe_release_speeds >= requested_release_speed-1e-12
                ]
            ))
            if math.isclose(lower, upper, rel_tol=0.0, abs_tol=1e-12):
                lower_mass, upper_mass = 1.0, 0.0
            else:
                upper_mass = (requested_release_speed-lower)/(upper-lower)
                lower_mass = 1.0-upper_mass
            lambda0 = np.zeros(count, dtype=float)
            for value, mass in ((lower, lower_mass), (upper, upper_mass)):
                if mass <= 0:
                    continue
                mask = np.isclose(
                    safe_release_speeds, value, rtol=0.0, atol=1e-12
                )
                local = inverse_distance[mask]
                lambda0[mask] += mass*local/np.sum(local)
        else:
            lambda0 = inverse_distance/np.sum(inverse_distance)
        guess = np.r_[warm, lambda0]
        previous_command = float(initial_vector[-1])

        def unpack(variables):
            return variables[:horizon], variables[horizon:]

        def simulate(commands):
            check_deadline()
            return self._rollout_detailed(initial_state, commands)

        def objective(variables):
            commands, weights = unpack(variables)
            states, _, _ = simulate(commands)
            del states
            changes = np.diff(np.r_[previous_command, commands])
            return float(
                costs@weights
                + c.effort_weight*c.prediction_step_s*(commands@commands)
                + c.slew_weight*(changes@changes)
            )

        def terminal_equalities(variables):
            commands, weights = unpack(variables)
            states, _, _ = simulate(commands)
            equalities = [states[-1]-safe.T@weights, [np.sum(weights)-1.0]]
            if condition_speed_equality:
                equalities.append([
                    safe_release_speeds@weights-requested_release_speed
                ])
            return np.concatenate(equalities)

        def path_margins(variables):
            commands, _weights = unpack(variables)
            _states, step_distances, audit_nodes = simulate(commands)
            velocities = audit_nodes[:, 0]
            tilts = audit_nodes[:, 1]
            rates = audit_nodes[:, 2]
            changes = np.diff(np.r_[previous_command, commands])
            slew_limit = math.radians(c.max_command_slew_deg_s)*c.prediction_step_s
            margins = [
                velocities+c.reverse_velocity_tolerance_m_s,
                c.max_abs_velocity_m_s-np.abs(velocities),
                math.radians(c.max_tilt_deg)-np.abs(tilts),
                math.radians(c.max_tilt_rate_deg_s)-np.abs(rates),
                slew_limit-np.abs(changes),
            ]
            if math.isfinite(initial_state.available_distance_m):
                cumulative = np.cumsum(step_distances)
                # Every returned local point retains a real successful suffix.
                # Reserving the largest candidate suffix is conservative under
                # the nonlinear convex-terminal relaxation: any selected tail
                # still fits within the available forward workspace margin.
                tail_reserve = float(np.max(safe_tail_distances))
                margins.append(
                    initial_state.available_distance_m
                    - cumulative-tail_reserve
                )
            return np.concatenate(margins)

        bounds = [(-command_limit, command_limit)]*horizon+[(0.0, 1.0)]*count
        try:
            result = minimize(
                objective,
                guess,
                method="SLSQP",
                bounds=bounds,
                constraints=(
                    {"type": "eq", "fun": terminal_equalities},
                    {"type": "ineq", "fun": path_margins},
                ),
                options={
                    "maxiter": c.solver_max_iterations,
                    "ftol": c.solver_ftol,
                    "disp": False,
                },
            )
        except _SolverDeadlineExceeded:
            return self._failure("solver_deadline_exceeded", started_at=started_at)
        except (FloatingPointError, OverflowError, ValueError) as error:
            return self._failure(
                "solver_exception: "+str(error), started_at=started_at,
            )

        iterations = int(getattr(result, "nit", 0))
        try:
            check_deadline()
        except _SolverDeadlineExceeded:
            return self._failure(
                "solver_deadline_exceeded",
                started_at=started_at,
                iterations=iterations,
            )
        if not bool(result.success):
            return self._failure(
                "solver_failed: "+str(result.message),
                started_at=started_at,
                iterations=iterations,
            )
        variables = np.asarray(result.x, dtype=float)
        if not np.isfinite(variables).all():
            return self._failure(
                "solver_returned_nonfinite_values",
                started_at=started_at,
                iterations=iterations,
            )
        commands, weights = unpack(variables)
        try:
            states, step_distances, _audit_nodes = self._rollout_detailed(
                initial_state, commands
            )
            equalities = terminal_equalities(variables)
            margins = path_margins(variables)
            check_deadline()
        except _SolverDeadlineExceeded:
            return self._failure(
                "solver_deadline_exceeded",
                started_at=started_at,
                iterations=iterations,
            )
        numerical_outputs = np.r_[
            variables, states.ravel(), step_distances, equalities, margins,
            float(result.fun),
        ]
        if not np.isfinite(numerical_outputs).all():
            return self._failure(
                "post_solver_validation_found_nonfinite_values",
                started_at=started_at,
                iterations=iterations,
            )
        if (np.any(commands < -command_limit-c.inequality_tolerance)
                or np.any(commands > command_limit+c.inequality_tolerance)
                or np.any(weights < -c.inequality_tolerance)
                or np.any(weights > 1.0+c.inequality_tolerance)):
            return self._failure(
                "solver_returned_values_outside_bounds",
                started_at=started_at,
                iterations=iterations,
            )
        equality_residual = float(np.max(np.abs(equalities)))
        minimum_margin = float(np.min(margins))
        if equality_residual > c.equality_tolerance:
            return self._failure(
                "terminal_safe_set_residual_exceeded",
                started_at=started_at,
                iterations=iterations,
            )
        if minimum_margin < -c.inequality_tolerance:
            return self._failure(
                "hard_path_constraint_violated",
                started_at=started_at,
                iterations=iterations,
            )
        if (np.min(weights) < -c.inequality_tolerance
                or abs(float(np.sum(weights))-1.0) > c.equality_tolerance):
            return self._failure(
                "terminal_weights_not_on_simplex",
                started_at=started_at,
                iterations=iterations,
            )
        return ConditionalVelocityLMPCPlan(
            optimizer_feasible=True,
            reason="offline_relaxed_conditional_safe_set_solution",
            offline_only=True,
            flight_command_dispatched=False,
            safety_certified=False,
            commands_rad=tuple(float(value) for value in commands),
            terminal_weights=tuple(float(value) for value in weights),
            predicted_states=tuple(
                tuple(float(value) for value in row) for row in states
            ),
            predicted_distance_m=float(np.sum(step_distances)),
            objective=float(result.fun),
            equality_residual_inf=equality_residual,
            minimum_inequality_margin=minimum_margin,
            solve_time_s=float(time.perf_counter()-started_at),
            iterations=iterations,
        )

    def solve_local_query(self, initial_state: ConditionalVelocityLMPCState,
                          local_query, *,
                          warm_start_commands_rad: Sequence[float] | None = None,
                          solver_deadline_s: float | None = None,
                          ) -> ConditionalVelocityLMPCPlan:
        """Solve directly from a validated conditional safe-set query."""
        from Interaction.velocity_lmpc_safe_set import LocalSafeSetQuery

        started_at = time.perf_counter()
        if not isinstance(local_query, LocalSafeSetQuery):
            return self._failure(
                "invalid_local_safe_set_query", started_at=started_at
            )
        if local_query.model_fingerprint != self.model_fingerprint:
            return self._failure(
                "local_safe_set_model_or_constraint_fingerprint_mismatch",
                started_at=started_at,
            )
        if not local_query.points:
            return self._failure(
                "local_safe_set_query_is_empty", started_at=started_at
            )
        command_limit = math.radians(self.config.max_command_tilt_deg)
        tilt_limit = math.radians(self.config.max_tilt_deg)
        rate_limit = math.radians(self.config.max_tilt_rate_deg_s)
        slew_limit = math.radians(self.config.max_command_slew_deg_s)
        if any(
            len(point.command) != 1
            or point.tail_max_abs_aligned_velocity_m_s
            > self.config.max_abs_velocity_m_s+1e-12
            or point.tail_max_abs_command > command_limit+1e-12
            or point.tail_max_abs_tilt_rad > tilt_limit+1e-12
            or point.tail_max_abs_rate_rad_s > rate_limit+1e-12
            or point.tail_max_command_slew_rad_s > slew_limit+1e-12
            for point in local_query.points
        ):
            return self._failure(
                "stored_safe_tail_violates_current_planner_bounds",
                started_at=started_at,
            )
        release_speed = (
            local_query.lower_initial_speed_m_s
            + local_query.upper_interpolation_weight
            * (
                local_query.upper_initial_speed_m_s
                - local_query.lower_initial_speed_m_s
            )
        )
        return self.solve(
            initial_state,
            safe_states=[point.state for point in local_query.points],
            safe_cost_to_go=[
                point.cost_to_go_s for point in local_query.points
            ],
            safe_release_speeds_m_s=[
                point.initial_speed_m_s for point in local_query.points
            ],
            release_initial_speed_m_s=release_speed,
            safe_tail_forward_distances_m=[
                point.remaining_forward_distance_m
                for point in local_query.points
            ],
            warm_start_commands_rad=warm_start_commands_rad,
            solver_deadline_s=solver_deadline_s,
        )

    def solve_from_safe_set(self, initial_state: ConditionalVelocityLMPCState,
                            safe_set, context, *, neighbors_per_bracket=8,
                            warm_start_commands_rad: Sequence[float] | None = None,
                            solver_deadline_s: float | None = None,
                            ) -> ConditionalVelocityLMPCPlan:
        """Query a fully validated artifact, then solve one offline problem."""
        from Interaction.velocity_lmpc_safe_set import (
            LMPCContext,
            NoSafeSetCoverageError,
            SafeSetValidationError,
            VelocityLMPCSafeSet,
        )

        started_at = time.perf_counter()
        if (not isinstance(safe_set, VelocityLMPCSafeSet)
                or not isinstance(context, LMPCContext)):
            return self._failure(
                "invalid_safe_set_or_context", started_at=started_at
            )
        if (safe_set.state_dimension != self.state_dimension
                or safe_set.command_dimension != 1
                or safe_set.aligned_velocity_state_index != 0
                or safe_set.limits != self.safe_set_limits):
            return self._failure(
                "safe_set_state_contract_mismatch", started_at=started_at
            )
        if context.model_fingerprint != self.model_fingerprint:
            return self._failure(
                "safe_set_model_or_constraint_fingerprint_mismatch",
                started_at=started_at,
            )
        try:
            current_state = initial_state.vector(delay_steps=self.delay_steps)
            warm = self._prepare_warm_commands(
                current_state, warm_start_commands_rad
            )
            # The paper's local safe set is selected around the predicted
            # horizon terminal state, including on the first call where this
            # implementation supplies its deterministic bounded warm rollout.
            query_state = self.rollout(initial_state, warm)[0][-1]
            local_query = safe_set.query(
                context,
                tuple(float(value) for value in query_state),
                neighbors_per_bracket=neighbors_per_bracket,
            )
        except (NoSafeSetCoverageError, SafeSetValidationError, ValueError) as error:
            return self._failure(
                "safe_set_query_failed: "+str(error), started_at=started_at
            )
        total_deadline = (
            self.config.default_solver_deadline_s
            if solver_deadline_s is None else float(solver_deadline_s)
        )
        remaining_deadline = total_deadline-(time.perf_counter()-started_at)
        if not math.isfinite(remaining_deadline) or remaining_deadline <= 0:
            return self._failure(
                "solver_deadline_exceeded", started_at=started_at
            )
        return self.solve_local_query(
            initial_state,
            local_query,
            warm_start_commands_rad=warm,
            solver_deadline_s=remaining_deadline,
        )
