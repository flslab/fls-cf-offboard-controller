"""Dependency-free one-dimensional jerk-limited braking profiles.

The profile is expressed in signed coordinates: positive velocity is motion
along the latched interaction direction and negative acceleration brakes that
motion. The original generator uses at most three constant-jerk segments. The
septic generator smooths each acceleration transition so position, velocity,
acceleration and jerk remain continuous at the phase boundaries.
"""

from dataclasses import dataclass

import numpy as np


class JerkLimitedBrakeError(ValueError):
    """The requested no-reverse terminal state is not feasible."""


@dataclass(frozen=True)
class JerkLimitedBrakeSample:
    elapsed_s: float
    position_m: float
    velocity_m_s: float
    acceleration_m_s2: float
    jerk_m_s3: float
    complete: bool


@dataclass(frozen=True)
class JerkLimitedBrakeProfile:
    initial_position_m: float
    initial_velocity_m_s: float
    initial_acceleration_m_s2: float
    max_deceleration_m_s2: float
    max_jerk_m_s3: float
    peak_deceleration_m_s2: float
    phase_durations_s: tuple
    phase_jerks_m_s3: tuple
    duration_s: float
    stop_position_m: float
    profile_type: str

    def sample(self, elapsed_s):
        """Sample the analytic profile without accumulating loop-time error."""
        elapsed_s = float(elapsed_s)
        if not np.isfinite(elapsed_s):
            raise ValueError('jerk-limited profile time must be finite')
        elapsed_s = max(elapsed_s, 0.0)
        if elapsed_s >= self.duration_s:
            return JerkLimitedBrakeSample(
                elapsed_s=self.duration_s,
                position_m=self.stop_position_m,
                velocity_m_s=0.0,
                acceleration_m_s2=0.0,
                jerk_m_s3=0.0,
                complete=True,
            )

        position = self.initial_position_m
        velocity = self.initial_velocity_m_s
        acceleration = self.initial_acceleration_m_s2
        remaining = elapsed_s
        active_jerk = 0.0
        for duration, jerk in zip(
                self.phase_durations_s, self.phase_jerks_m_s3):
            step = min(max(remaining, 0.0), duration)
            if step > 0.0:
                position += (
                    velocity * step
                    + 0.5 * acceleration * step ** 2
                    + jerk * step ** 3 / 6.0
                )
                velocity += (
                    acceleration * step + 0.5 * jerk * step ** 2
                )
                acceleration += jerk * step
            remaining -= step
            active_jerk = jerk
            if remaining <= 1e-12:
                break

        return JerkLimitedBrakeSample(
            elapsed_s=elapsed_s,
            position_m=float(position),
            velocity_m_s=float(max(velocity, 0.0)),
            acceleration_m_s2=float(acceleration),
            jerk_m_s3=float(active_jerk),
            complete=False,
        )


def make_jerk_limited_brake_profile(
        initial_velocity_m_s,
        initial_acceleration_m_s2,
        max_deceleration_m_s2,
        max_jerk_m_s3,
        *,
        initial_position_m=0.0):
    """Create a time-optimal triangular or trapezoidal acceleration profile.

    The terminal constraints are ``velocity == 0`` and ``acceleration == 0``.
    If an already-negative initial acceleration contains more unavoidable
    impulse than the remaining forward velocity, reaching that terminal state
    without first reversing is physically impossible and is rejected.
    """
    velocity = float(initial_velocity_m_s)
    acceleration = float(initial_acceleration_m_s2)
    max_deceleration = float(max_deceleration_m_s2)
    max_jerk = float(max_jerk_m_s3)
    initial_position = float(initial_position_m)
    values = np.asarray([
        velocity,
        acceleration,
        max_deceleration,
        max_jerk,
        initial_position,
    ], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError('jerk-limited braking inputs must be finite')
    if velocity < -1e-9:
        raise JerkLimitedBrakeError(
            'initial velocity is already opposite the interaction direction'
        )
    if max_deceleration <= 0.0 or max_jerk <= 0.0:
        raise ValueError(
            'jerk-limited braking limits must be strictly positive'
        )
    if acceleration < -max_deceleration - 1e-9:
        raise JerkLimitedBrakeError(
            'initial braking acceleration exceeds the configured limit'
        )
    if (
        acceleration < 0.0
        and velocity + 1e-12 < acceleration ** 2 / (2.0 * max_jerk)
    ):
        raise JerkLimitedBrakeError(
            'existing braking acceleration cannot be unwound before reversal'
        )

    velocity = max(velocity, 0.0)
    if velocity <= 1e-12 and abs(acceleration) <= 1e-12:
        return JerkLimitedBrakeProfile(
            initial_position_m=initial_position,
            initial_velocity_m_s=0.0,
            initial_acceleration_m_s2=0.0,
            max_deceleration_m_s2=max_deceleration,
            max_jerk_m_s3=max_jerk,
            peak_deceleration_m_s2=0.0,
            phase_durations_s=(0.0, 0.0, 0.0),
            phase_jerks_m_s3=(-max_jerk, 0.0, max_jerk),
            duration_s=0.0,
            stop_position_m=initial_position,
            profile_type='stationary',
        )

    required_peak = float(np.sqrt(
        max_jerk * velocity + 0.5 * acceleration ** 2
    ))
    if required_peak <= max_deceleration + 1e-12:
        peak = min(required_peak, max_deceleration)
        ramp_down_s = (acceleration + peak) / max_jerk
        hold_s = 0.0
        ramp_up_s = peak / max_jerk
        profile_type = 'triangular'
    else:
        peak = max_deceleration
        ramp_down_s = (acceleration + peak) / max_jerk
        ramp_velocity_delta = (
            acceleration ** 2 - 2.0 * peak ** 2
        ) / (2.0 * max_jerk)
        hold_s = (velocity + ramp_velocity_delta) / peak
        ramp_up_s = peak / max_jerk
        profile_type = 'trapezoidal'

    durations = tuple(float(max(value, 0.0)) for value in (
        ramp_down_s, hold_s, ramp_up_s
    ))
    jerks = (-max_jerk, 0.0, max_jerk)
    position = initial_position
    terminal_velocity = velocity
    terminal_acceleration = acceleration
    for duration, jerk in zip(durations, jerks):
        position += (
            terminal_velocity * duration
            + 0.5 * terminal_acceleration * duration ** 2
            + jerk * duration ** 3 / 6.0
        )
        terminal_velocity += (
            terminal_acceleration * duration
            + 0.5 * jerk * duration ** 2
        )
        terminal_acceleration += jerk * duration

    numerical_tolerance = 1e-8 * max(1.0, velocity, max_deceleration)
    if (
        abs(terminal_velocity) > numerical_tolerance
        or abs(terminal_acceleration) > numerical_tolerance
        or position < initial_position - numerical_tolerance
    ):
        raise JerkLimitedBrakeError(
            'analytic jerk-limited profile failed its terminal constraints'
        )

    return JerkLimitedBrakeProfile(
        initial_position_m=initial_position,
        initial_velocity_m_s=velocity,
        initial_acceleration_m_s2=acceleration,
        max_deceleration_m_s2=max_deceleration,
        max_jerk_m_s3=max_jerk,
        peak_deceleration_m_s2=peak,
        phase_durations_s=durations,
        phase_jerks_m_s3=jerks,
        duration_s=float(sum(durations)),
        stop_position_m=float(position),
        profile_type=profile_type,
    )


# A quintic smootherstep acceleration ramp integrates to a seventh-order
# position trajectory. Its normalized derivative peaks at 15 / 8.
_SEPTIC_JERK_SHAPE_PEAK = 15.0 / 8.0


def _smootherstep5(u):
    """Quintic 0-to-1 interpolation with zero endpoint derivatives."""
    return u ** 3 * (10.0 + u * (-15.0 + 6.0 * u))


def _smootherstep5_derivative(u):
    return 30.0 * u ** 2 * (1.0 - u) ** 2


def _smootherstep5_integral(u):
    return u ** 4 * (2.5 + u * (-3.0 + u))


def _smootherstep5_double_integral(u):
    return u ** 5 * (0.5 + u * (-0.5 + u / 7.0))


def _sample_septic_acceleration_ramp(
        position_m,
        velocity_m_s,
        start_acceleration_m_s2,
        end_acceleration_m_s2,
        duration_s,
        elapsed_s):
    """Integrate one smooth acceleration ramp analytically.

    Acceleration is quintic in normalized time; velocity is sixth-order and
    position is seventh-order. Position, velocity, acceleration and jerk are
    continuous at both ends of adjacent ramps and holds.
    """
    u = min(max(float(elapsed_s) / duration_s, 0.0), 1.0)
    acceleration_delta = (
        end_acceleration_m_s2 - start_acceleration_m_s2
    )
    position = (
        position_m
        + velocity_m_s * duration_s * u
        + duration_s ** 2 * (
            0.5 * start_acceleration_m_s2 * u ** 2
            + acceleration_delta * _smootherstep5_double_integral(u)
        )
    )
    velocity = (
        velocity_m_s
        + duration_s * (
            start_acceleration_m_s2 * u
            + acceleration_delta * _smootherstep5_integral(u)
        )
    )
    acceleration = (
        start_acceleration_m_s2
        + acceleration_delta * _smootherstep5(u)
    )
    jerk = (
        acceleration_delta
        * _smootherstep5_derivative(u)
        / duration_s
    )
    return tuple(float(value) for value in (
        position, velocity, acceleration, jerk
    ))


@dataclass(frozen=True)
class SepticBrakeProfile:
    """Analytic no-reverse brake with zero jerk at every phase boundary.

    Acceleration follows a smooth ramp-down, an optional constant-deceleration
    hold, and a smooth ramp-up. Each acceleration ramp is quintic, making its
    analytic position curve seventh-order. ``phase_durations_s`` contains the
    three phase durations in that order.
    """

    initial_position_m: float
    initial_velocity_m_s: float
    initial_acceleration_m_s2: float
    max_deceleration_m_s2: float
    max_jerk_m_s3: float
    peak_deceleration_m_s2: float
    peak_jerk_m_s3: float
    phase_durations_s: tuple
    duration_s: float
    stop_position_m: float
    profile_type: str

    def sample(self, elapsed_s):
        """Sample the profile analytically without integration drift."""
        elapsed_s = float(elapsed_s)
        if not np.isfinite(elapsed_s):
            raise ValueError('septic braking profile time must be finite')
        elapsed_s = max(elapsed_s, 0.0)
        if elapsed_s >= self.duration_s:
            return JerkLimitedBrakeSample(
                elapsed_s=self.duration_s,
                position_m=self.stop_position_m,
                velocity_m_s=0.0,
                acceleration_m_s2=0.0,
                jerk_m_s3=0.0,
                complete=True,
            )

        ramp_down_s, hold_s, ramp_up_s = self.phase_durations_s
        peak_acceleration = -self.peak_deceleration_m_s2
        position = self.initial_position_m
        velocity = self.initial_velocity_m_s

        if ramp_down_s > 0.0:
            if elapsed_s <= ramp_down_s:
                position, velocity, acceleration, jerk = (
                    _sample_septic_acceleration_ramp(
                        position,
                        velocity,
                        self.initial_acceleration_m_s2,
                        peak_acceleration,
                        ramp_down_s,
                        elapsed_s,
                    )
                )
                return JerkLimitedBrakeSample(
                    elapsed_s=elapsed_s,
                    position_m=position,
                    velocity_m_s=velocity,
                    acceleration_m_s2=acceleration,
                    jerk_m_s3=jerk,
                    complete=False,
                )
            position, velocity, _, _ = _sample_septic_acceleration_ramp(
                position,
                velocity,
                self.initial_acceleration_m_s2,
                peak_acceleration,
                ramp_down_s,
                ramp_down_s,
            )

        remaining = elapsed_s - ramp_down_s
        if remaining <= hold_s and hold_s > 0.0:
            return JerkLimitedBrakeSample(
                elapsed_s=elapsed_s,
                position_m=float(
                    position
                    + velocity * remaining
                    + 0.5 * peak_acceleration * remaining ** 2
                ),
                velocity_m_s=float(
                    velocity + peak_acceleration * remaining
                ),
                acceleration_m_s2=peak_acceleration,
                jerk_m_s3=0.0,
                complete=False,
            )

        if hold_s > 0.0:
            position += (
                velocity * hold_s
                + 0.5 * peak_acceleration * hold_s ** 2
            )
            velocity += peak_acceleration * hold_s

        remaining -= hold_s
        if ramp_up_s > 0.0:
            position, velocity, acceleration, jerk = (
                _sample_septic_acceleration_ramp(
                    position,
                    velocity,
                    peak_acceleration,
                    0.0,
                    ramp_up_s,
                    remaining,
                )
            )
            return JerkLimitedBrakeSample(
                elapsed_s=elapsed_s,
                position_m=position,
                velocity_m_s=velocity,
                acceleration_m_s2=acceleration,
                jerk_m_s3=jerk,
                complete=False,
            )

        return JerkLimitedBrakeSample(
            elapsed_s=elapsed_s,
            position_m=float(position),
            velocity_m_s=float(velocity),
            acceleration_m_s2=0.0,
            jerk_m_s3=0.0,
            complete=False,
        )

    def sample_zoh(self, elapsed_s, interval_s):
        """Return an exact zero-order-hold command for one interval.

        Position and velocity describe the interval start. Acceleration and
        jerk are averages of the analytic acceleration and jerk over the
        interval; ``jerk_m_s3`` is not the derivative of the returned ZOH
        acceleration command. Holding the returned acceleration for exactly
        ``interval_s`` reaches the analytic velocity at the next command
        boundary, including intervals that cross a phase boundary or profile
        end. Time after completion contributes zero acceleration.

        This helper does not make a delayed or unexpectedly extended physical
        hold safe. The caller must use the actual conservative hold interval
        and retain the delayed-plant no-reverse validation. ZOH position also
        need not equal the continuous profile between command boundaries.
        """
        interval_s = float(interval_s)
        if not np.isfinite(interval_s) or interval_s <= 0.0:
            raise ValueError(
                'septic zero-order-hold interval must be finite and positive'
            )
        start = self.sample(elapsed_s)
        if start.complete:
            return start
        end = self.sample(start.elapsed_s + interval_s)
        return JerkLimitedBrakeSample(
            elapsed_s=start.elapsed_s,
            position_m=start.position_m,
            velocity_m_s=start.velocity_m_s,
            acceleration_m_s2=float(
                (end.velocity_m_s - start.velocity_m_s) / interval_s
            ),
            jerk_m_s3=float(
                (end.acceleration_m_s2 - start.acceleration_m_s2)
                / interval_s
            ),
            complete=False,
        )


@dataclass(frozen=True)
class ReleaseStateSepticBrakeProfile:
    """One seventh-degree free stop including release angular inertia.

    The polynomial has no prescribed terminal position.  Its eight boundary
    conditions are p/v/a/j at release and v/a/j/snap equal to zero at the
    terminal time.  In particular, the initial jerk is measured from release
    attitude rate instead of being silently reset to zero.
    """

    initial_position_m: float
    initial_velocity_m_s: float
    initial_acceleration_m_s2: float
    initial_jerk_m_s3: float
    max_deceleration_m_s2: float
    max_jerk_m_s3: float
    peak_deceleration_m_s2: float
    peak_jerk_m_s3: float
    phase_durations_s: tuple
    duration_s: float
    stop_position_m: float
    profile_type: str
    normalized_position_coefficients: tuple

    def sample(self, elapsed_s):
        elapsed = float(elapsed_s)
        if not np.isfinite(elapsed):
            raise ValueError('release-state septic profile time must be finite')
        elapsed = max(elapsed, 0.0)
        if elapsed >= self.duration_s:
            return JerkLimitedBrakeSample(
                elapsed_s=self.duration_s,
                position_m=self.stop_position_m,
                velocity_m_s=0.0,
                acceleration_m_s2=0.0,
                jerk_m_s3=0.0,
                complete=True,
            )
        u = elapsed/self.duration_s
        coefficients = np.asarray(
            self.normalized_position_coefficients, dtype=float
        )
        position = float(np.polynomial.polynomial.polyval(u, coefficients))
        velocity = float(np.polynomial.polynomial.polyval(
            u, np.polynomial.polynomial.polyder(coefficients, 1)
        )/self.duration_s)
        acceleration = float(np.polynomial.polynomial.polyval(
            u, np.polynomial.polynomial.polyder(coefficients, 2)
        )/self.duration_s**2)
        jerk = float(np.polynomial.polynomial.polyval(
            u, np.polynomial.polynomial.polyder(coefficients, 3)
        )/self.duration_s**3)
        return JerkLimitedBrakeSample(
            elapsed_s=elapsed,
            position_m=position,
            velocity_m_s=velocity,
            acceleration_m_s2=acceleration,
            jerk_m_s3=jerk,
            complete=False,
        )

    def sample_zoh(self, elapsed_s, interval_s):
        interval = float(interval_s)
        if not np.isfinite(interval) or interval <= 0.0:
            raise ValueError(
                'septic zero-order-hold interval must be finite and positive'
            )
        start = self.sample(elapsed_s)
        if start.complete:
            return start
        end = self.sample(start.elapsed_s+interval)
        return JerkLimitedBrakeSample(
            elapsed_s=start.elapsed_s,
            position_m=start.position_m,
            velocity_m_s=start.velocity_m_s,
            acceleration_m_s2=(
                end.velocity_m_s-start.velocity_m_s
            )/interval,
            jerk_m_s3=(
                end.acceleration_m_s2-start.acceleration_m_s2
            )/interval,
            complete=False,
        )


def _release_state_septic_coefficients(
        position, velocity, acceleration, jerk, duration_s,
        seventh_coefficient=None):
    """Return normalized-time coefficients for the release endpoint rules.

    With ``seventh_coefficient=None`` terminal snap is zero, preserving the
    original closed form. Otherwise the seventh-order coefficient is the one
    free shape variable left by terminal v/a/j=0.
    """
    duration = float(duration_s)
    coefficients = np.zeros(8, dtype=float)
    coefficients[:4] = (
        position,
        velocity*duration,
        0.5*acceleration*duration**2,
        jerk*duration**3/6.0,
    )
    if seventh_coefficient is None:
        terminal_matrix = np.array([
            [4.0, 5.0, 6.0, 7.0],
            [12.0, 20.0, 30.0, 42.0],
            [24.0, 60.0, 120.0, 210.0],
            [24.0, 120.0, 360.0, 840.0],
        ])
        terminal_rhs = -np.array([
            coefficients[1]+2.0*coefficients[2]+3.0*coefficients[3],
            2.0*coefficients[2]+6.0*coefficients[3],
            6.0*coefficients[3],
            0.0,
        ])
        coefficients[4:] = np.linalg.solve(terminal_matrix, terminal_rhs)
    else:
        coefficients[7] = float(seventh_coefficient)
        terminal_matrix = np.array([
            [4.0, 5.0, 6.0],
            [12.0, 20.0, 30.0],
            [24.0, 60.0, 120.0],
        ])
        terminal_rhs = -np.array([
            coefficients[1]+2.0*coefficients[2]+3.0*coefficients[3]
            + 7.0*coefficients[7],
            2.0*coefficients[2]+6.0*coefficients[3]
            + 42.0*coefficients[7],
            6.0*coefficients[3]+210.0*coefficients[7],
        ])
        coefficients[4:7] = np.linalg.solve(
            terminal_matrix, terminal_rhs
        )
    return coefficients


def _polynomial_extrema_values(coefficients, derivative_order):
    derivative = np.polynomial.polynomial.polyder(
        coefficients, derivative_order
    )
    next_derivative = np.polynomial.polynomial.polyder(derivative)
    roots = np.polynomial.polynomial.polyroots(next_derivative)
    locations = [0.0, 1.0]
    locations.extend(
        float(root.real) for root in roots
        if abs(root.imag) <= 1e-8 and 0.0 < root.real < 1.0
    )
    return np.asarray([
        np.polynomial.polynomial.polyval(value, derivative)
        for value in locations
    ], dtype=float)


def make_release_state_septic_brake_profile(
        initial_velocity_m_s, initial_acceleration_m_s2,
        initial_jerk_m_s3, max_deceleration_m_s2, max_jerk_m_s3, *,
        initial_position_m=0.0, min_duration_s=0.0, max_duration_s=4.0):
    """Find the shortest constrained seventh-degree free-stop polynomial."""
    velocity = float(initial_velocity_m_s)
    acceleration = float(initial_acceleration_m_s2)
    jerk = float(initial_jerk_m_s3)
    max_deceleration = float(max_deceleration_m_s2)
    max_jerk = float(max_jerk_m_s3)
    position = float(initial_position_m)
    min_duration = float(min_duration_s)
    max_duration = float(max_duration_s)
    values = np.asarray([
        velocity, acceleration, jerk, max_deceleration, max_jerk,
        position, min_duration, max_duration,
    ])
    if not np.all(np.isfinite(values)):
        raise ValueError('release-state septic inputs must be finite')
    if velocity < -1e-9:
        raise JerkLimitedBrakeError(
            'initial velocity is already opposite the interaction direction'
        )
    if (
        max_deceleration <= 0.0
        or max_jerk <= 0.0
        or min_duration < 0.0
        or max_duration <= 0.0
        or min_duration > max_duration
    ):
        raise ValueError('release-state septic limits must be positive')
    effective_max_deceleration = max(max_deceleration, abs(acceleration))
    effective_max_forward_acceleration = max(acceleration, 0.0)
    # Release angular rate determines the physical jerk at t=0. It is an
    # observed boundary condition, not a command the planner can retroactively
    # clamp. Keep that endpoint exact and bound all later extrema by at least
    # its unavoidable magnitude.
    effective_max_jerk = max(max_jerk, abs(jerk))

    def candidate(duration):
        coefficients = _release_state_septic_coefficients(
            position, max(velocity, 0.0), acceleration, jerk, duration
        )
        velocity_values = (
            _polynomial_extrema_values(coefficients, 1)/duration
        )
        acceleration_values = (
            _polynomial_extrema_values(coefficients, 2)/duration**2
        )
        jerk_values = (
            _polynomial_extrema_values(coefficients, 3)/duration**3
        )
        tolerance = 1e-8
        feasible = bool(
            np.min(velocity_values) >= -tolerance
            and np.max(acceleration_values) <= (
                effective_max_forward_acceleration+tolerance
            )
            and np.min(acceleration_values) >= (
                -effective_max_deceleration-tolerance
            )
            and np.max(np.abs(jerk_values)) <= effective_max_jerk+tolerance
        )
        return feasible, coefficients, velocity_values, acceleration_values, jerk_values

    def free_shape_result(duration, seventh_coefficient,
                          acceleration_limit, jerk_limit):
        coefficients = _release_state_septic_coefficients(
            position, max(velocity, 0.0), acceleration, jerk, duration,
            seventh_coefficient=seventh_coefficient,
        )
        velocity_values = (
            _polynomial_extrema_values(coefficients, 1)/duration
        )
        acceleration_values = (
            _polynomial_extrema_values(coefficients, 2)/duration**2
        )
        jerk_values = (
            _polynomial_extrema_values(coefficients, 3)/duration**3
        )
        feasible = bool(
            np.min(velocity_values) >= -1e-7
            and np.max(np.abs(acceleration_values))
            <= acceleration_limit+1e-7
            and np.max(np.abs(jerk_values)) <= jerk_limit+1e-7
        )
        return (
            feasible, coefficients, velocity_values,
            acceleration_values, jerk_values,
        )

    # Root finding dominates this path and runs synchronously in the first
    # post-release command cycle. Bracket coarsely, then refine, so profile
    # construction does not starve the 100 Hz command stream.
    strict_start_duration = max(0.02, min_duration)
    previous_duration = strict_start_duration
    selected = None
    boundary_feasible_candidates = []
    release_state_feasible_candidates = []
    # When the measured release jerk already exceeds the nominal command
    # limit, the strict endpoint-shape family cannot reduce that boundary
    # condition.  Scanning it performs thousands of polynomial root solves on
    # the command thread before inevitably selecting the free-shape family.
    # Go directly to that family while preserving the exact measured jerk.
    if abs(jerk) <= max_jerk + 1e-8:
        for duration in np.linspace(
                strict_start_duration, max_duration, 97):
            result = candidate(float(duration))
            _, _, velocity_values, acceleration_values, jerk_values = result
            if (
                np.min(velocity_values) >= -1e-8
                and np.max(np.abs(jerk_values)) <= effective_max_jerk+1e-8
            ):
                release_state_feasible_candidates.append((
                    float(np.max(np.abs(acceleration_values))),
                    float(duration), result,
                ))
            if (
                np.min(velocity_values) >= -1e-8
                and np.max(acceleration_values) <= (
                    effective_max_forward_acceleration+1e-8
                )
                and np.max(np.abs(jerk_values)) <= effective_max_jerk+1e-8
            ):
                boundary_feasible_candidates.append((
                    float(-np.min(acceleration_values)), float(duration), result
                ))
            if result[0]:
                lower = max(previous_duration, min_duration)
                upper = float(duration)
                if upper-lower <= 1e-12:
                    # With a bandwidth-derived minimum duration the first
                    # candidate is commonly feasible.  Re-evaluating the same
                    # endpoint 24 times blocks the 100 Hz command thread and
                    # makes its velocity sample stale before the first send.
                    selected = (upper, result)
                else:
                    for _ in range(24):
                        midpoint = 0.5*(lower+upper)
                        if candidate(midpoint)[0]:
                            upper = midpoint
                        else:
                            lower = midpoint
                    selected = (upper, candidate(upper))
                break
            previous_duration = float(duration)
    if selected is None:
        # Terminal v/a/j impose only seven conditions on a seventh-degree
        # position curve. Do not waste its remaining shape degree of freedom
        # by forcing terminal snap to zero: use it to turn an inherited
        # forward jerk promptly and keep stopping distance bounded.
        # The free shape is an extra polynomial degree of freedom, not
        # permission to relax the physical envelope.  In particular, the old
        # 2.5x expansion could return a nominal ``2 m/s^2`` stop whose true
        # peak was almost ``5 m/s^2``.  That profile is mathematically smooth
        # but cannot be followed by the calibrated attitude loop, so its
        # reference reaches zero while the aircraft is still moving quickly.
        # Preserve an out-of-limit measured endpoint exactly (we cannot change
        # the past), but never ask the future trajectory to exceed it or the
        # configured limits.
        dynamic_acceleration_limit = effective_max_deceleration
        # A release can arrive with jerk already outside the nominal command
        # envelope.  Permit the free shape enough snap authority to turn that
        # inherited boundary condition, while keeping the acceleration (and
        # therefore requested tilt) at the real configured limit.
        dynamic_jerk_limit = 2.5*effective_max_jerk
        sample_u = np.linspace(0.0, 1.0, 97)
        orders = np.arange(8, dtype=float)
        velocity_basis = np.vstack([
            np.zeros_like(sample_u) if order == 0
            else order*sample_u**(int(order)-1)
            for order in orders
        ])
        acceleration_basis = np.vstack([
            np.zeros_like(sample_u) if order < 2
            else order*(order-1)*sample_u**(int(order)-2)
            for order in orders
        ])
        jerk_basis = np.vstack([
            np.zeros_like(sample_u) if order < 3
            else order*(order-1)*(order-2)*sample_u**(int(order)-3)
            for order in orders
        ])
        shape_factors = np.linspace(-4.0, 4.0, 81)
        for duration in np.linspace(
                max(0.08, min_duration), max_duration, 97):
            duration = float(duration)
            fixed = _release_state_septic_coefficients(
                position, max(velocity, 0.0), acceleration, jerk, duration
            )
            zero = _release_state_septic_coefficients(
                position, max(velocity, 0.0), acceleration, jerk, duration,
                seventh_coefficient=0.0,
            )
            one = _release_state_septic_coefficients(
                position, max(velocity, 0.0), acceleration, jerk, duration,
                seventh_coefficient=1.0,
            )
            span = max(abs(fixed[7]), abs(velocity*duration), 0.1)
            seventh_values = fixed[7]+shape_factors*span
            coefficient_rows = (
                zero[None, :]
                + seventh_values[:, None]*(one-zero)[None, :]
            )
            with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
                sampled_velocity = (
                    coefficient_rows @ velocity_basis / duration
                )
                sampled_acceleration = (
                    coefficient_rows @ acceleration_basis / duration**2
                )
                sampled_jerk = coefficient_rows @ jerk_basis / duration**3
            feasible_rows = np.flatnonzero(
                (np.min(sampled_velocity, axis=1) >= -1e-6)
                & (np.max(np.abs(sampled_acceleration), axis=1)
                   <= dynamic_acceleration_limit+1e-6)
                & (np.max(np.abs(sampled_jerk), axis=1)
                   <= dynamic_jerk_limit+1e-6)
            )
            for row in feasible_rows:
                result = free_shape_result(
                    duration, seventh_values[row],
                    dynamic_acceleration_limit, dynamic_jerk_limit,
                )
                if result[0]:
                    selected = (duration, result)
                    acceleration_values = result[3]
                    jerk_values = result[4]
                    effective_max_deceleration = max(
                        effective_max_deceleration,
                        float(-np.min(acceleration_values)),
                    )
                    effective_max_forward_acceleration = max(
                        effective_max_forward_acceleration,
                        float(np.max(acceleration_values)),
                    )
                    effective_max_jerk = max(
                        effective_max_jerk,
                        float(np.max(np.abs(jerk_values))),
                    )
                    break
            if selected is not None:
                break

    if selected is None:
        if boundary_feasible_candidates:
            required_peak, duration, result = min(
                boundary_feasible_candidates, key=lambda item: item[0]
            )
        elif release_state_feasible_candidates:
            _, duration, result = min(
                release_state_feasible_candidates, key=lambda item: item[0]
            )
            acceleration_values = result[3]
            required_peak = float(-np.min(acceleration_values))
            effective_max_forward_acceleration = max(
                effective_max_forward_acceleration,
                float(np.max(acceleration_values)),
            )
        else:
            raise JerkLimitedBrakeError(
                'release v/a/j cannot reach a no-reverse seventh-order stop '
                'within the configured duration'
            )
        # A large release angular rate can make the configured acceleration
        # limit incompatible with the exact initial jerk. Select the physically
        # admissible curve requiring the smallest peak deceleration instead of
        # discarding the measured release state or falling back.
        effective_max_deceleration = required_peak
        selected = (duration, result)
    duration, (_, coefficients, _, acceleration_values, jerk_values) = selected
    stop_position = float(np.sum(coefficients))
    return ReleaseStateSepticBrakeProfile(
        initial_position_m=position,
        initial_velocity_m_s=max(velocity, 0.0),
        initial_acceleration_m_s2=acceleration,
        initial_jerk_m_s3=jerk,
        max_deceleration_m_s2=effective_max_deceleration,
        max_jerk_m_s3=effective_max_jerk,
        peak_deceleration_m_s2=float(-np.min(acceleration_values)),
        peak_jerk_m_s3=float(np.max(np.abs(jerk_values))),
        phase_durations_s=(duration, 0.0, 0.0),
        duration_s=duration,
        stop_position_m=stop_position,
        profile_type='release_state_septic_free_stop',
        normalized_position_coefficients=tuple(map(float, coefficients)),
    )


def make_septic_brake_profile(
        initial_velocity_m_s,
        initial_acceleration_m_s2,
        max_deceleration_m_s2,
        max_jerk_m_s3,
        *,
        initial_position_m=0.0):
    """Create an endpoint-smoothed, constraint-respecting brake profile.

    This is the smooth analogue of the existing triangular/trapezoidal
    constant-jerk profile. Acceleration changes use quintic smootherstep ramps,
    so their exact double integrals are seventh-order position curves. Ramp
    durations saturate the jerk constraint while peak deceleration is chosen
    to remove exactly the available velocity without reversing.
    """
    velocity = float(initial_velocity_m_s)
    acceleration = float(initial_acceleration_m_s2)
    max_deceleration = float(max_deceleration_m_s2)
    max_jerk = float(max_jerk_m_s3)
    initial_position = float(initial_position_m)
    values = np.asarray([
        velocity,
        acceleration,
        max_deceleration,
        max_jerk,
        initial_position,
    ], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError('septic braking inputs must be finite')
    if velocity < -1e-9:
        raise JerkLimitedBrakeError(
            'initial velocity is already opposite the interaction direction'
        )
    if max_deceleration <= 0.0 or max_jerk <= 0.0:
        raise ValueError('septic braking limits must be strictly positive')
    if acceleration < -max_deceleration - 1e-9:
        raise JerkLimitedBrakeError(
            'initial braking acceleration exceeds the configured limit'
        )

    velocity = max(velocity, 0.0)
    acceleration = 0.0 if abs(acceleration) <= 1e-12 else acceleration
    if velocity <= 1e-12 and acceleration == 0.0:
        return SepticBrakeProfile(
            initial_position_m=initial_position,
            initial_velocity_m_s=0.0,
            initial_acceleration_m_s2=0.0,
            max_deceleration_m_s2=max_deceleration,
            max_jerk_m_s3=max_jerk,
            peak_deceleration_m_s2=0.0,
            peak_jerk_m_s3=0.0,
            phase_durations_s=(0.0, 0.0, 0.0),
            duration_s=0.0,
            stop_position_m=initial_position,
            profile_type='stationary',
        )

    minimum_unwind_velocity = (
        _SEPTIC_JERK_SHAPE_PEAK * acceleration ** 2
        / (2.0 * max_jerk)
    )
    if acceleration < 0.0 and velocity < minimum_unwind_velocity:
        raise JerkLimitedBrakeError(
            'existing braking acceleration cannot be smoothly unwound '
            'before reversal'
        )

    # A smooth ramp from a0 to -P followed by a smooth ramp from -P to
    # zero changes velocity by C * (a0**2 - 2*P**2) / (2*J), where
    # C = 15/8. Solving that impulse equation for a zero terminal velocity
    # gives the triangular peak below. If it exceeds the acceleration limit,
    # the difference is supplied by a constant-deceleration hold.
    required_peak = float(np.sqrt(
        max_jerk * velocity / _SEPTIC_JERK_SHAPE_PEAK
        + 0.5 * acceleration ** 2
    ))
    if required_peak <= max_deceleration:
        peak_deceleration = required_peak
        hold_s = 0.0
        profile_type = 'septic_triangular'
    else:
        peak_deceleration = max_deceleration
        hold_s = (
            velocity
            + _SEPTIC_JERK_SHAPE_PEAK
            * (acceleration ** 2 - 2.0 * peak_deceleration ** 2)
            / (2.0 * max_jerk)
        ) / peak_deceleration
        profile_type = 'septic_trapezoidal'

    ramp_down_s = (
        _SEPTIC_JERK_SHAPE_PEAK
        * (acceleration + peak_deceleration)
        / max_jerk
    )
    ramp_up_s = (
        _SEPTIC_JERK_SHAPE_PEAK * peak_deceleration / max_jerk
    )
    durations = tuple(float(max(value, 0.0)) for value in (
        ramp_down_s, hold_s, ramp_up_s
    ))

    position = initial_position
    terminal_velocity = velocity
    if durations[0] > 0.0:
        position, terminal_velocity, _, _ = (
            _sample_septic_acceleration_ramp(
                position,
                terminal_velocity,
                acceleration,
                -peak_deceleration,
                durations[0],
                durations[0],
            )
        )
    if durations[1] > 0.0:
        position += (
            terminal_velocity * durations[1]
            - 0.5 * peak_deceleration * durations[1] ** 2
        )
        terminal_velocity -= peak_deceleration * durations[1]
    if durations[2] > 0.0:
        position, terminal_velocity, terminal_acceleration, terminal_jerk = (
            _sample_septic_acceleration_ramp(
                position,
                terminal_velocity,
                -peak_deceleration,
                0.0,
                durations[2],
                durations[2],
            )
        )
    else:
        terminal_acceleration = 0.0
        terminal_jerk = 0.0

    peak_jerk = max_jerk if peak_deceleration > 0.0 else 0.0
    stop_position = position
    duration = float(sum(durations))
    numerical_tolerance = 1e-9 * max(
        1.0, velocity, max_deceleration, max_jerk
    )
    if (
        abs(terminal_velocity) > numerical_tolerance
        or abs(terminal_acceleration) > numerical_tolerance
        or abs(terminal_jerk) > numerical_tolerance
        or peak_deceleration > max_deceleration + numerical_tolerance
        or stop_position < initial_position - numerical_tolerance
    ):
        raise JerkLimitedBrakeError(
            'analytic septic profile failed its constraints'
        )

    return SepticBrakeProfile(
        initial_position_m=initial_position,
        initial_velocity_m_s=velocity,
        initial_acceleration_m_s2=acceleration,
        max_deceleration_m_s2=max_deceleration,
        max_jerk_m_s3=max_jerk,
        peak_deceleration_m_s2=float(peak_deceleration),
        peak_jerk_m_s3=float(peak_jerk),
        phase_durations_s=durations,
        duration_s=duration,
        stop_position_m=float(stop_position),
        profile_type=profile_type,
    )
