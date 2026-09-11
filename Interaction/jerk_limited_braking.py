"""Dependency-free one-dimensional jerk-limited braking profiles.

The profile is expressed in signed coordinates: positive velocity is motion
along the latched interaction direction and negative acceleration brakes that
motion.  It supports a finite initial acceleration, then uses at most three
constant-jerk segments to reach exactly zero velocity and acceleration.
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
