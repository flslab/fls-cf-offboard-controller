"""Planar attitude-step calibration used by release braking.

The maneuver is intentionally simple and observable: accelerate with a small
constant tilt, command level attitude, decelerate with the opposite tilt, then
command level attitude again before returning to position hold.  It is used
by the explicit ``--calibrate`` and data-only ``--braking-test`` flight paths.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class BrakingCalibrationCommand:
    """One command in the deterministic planar braking maneuver."""

    active: bool
    attitude_control: bool
    phase: str
    segment_id: int | None
    direction_xy: np.ndarray
    command_acceleration_xy: np.ndarray
    tilt_deg: float
    roll_deg: float
    pitch_deg: float


class PlanarBrakingCalibration:
    """Generate accelerate -> level -> brake -> level calibration trials."""

    ATTITUDE_PHASES = frozenset({
        "level_before_acceleration", "accelerate", "level_before_brake",
        "brake", "level_after_brake",
    })

    def __init__(self, config, start_after_s=0.0, *, require_opposed_directions=True):
        config = dict(config or {})
        self.enabled = bool(config.get("enabled", False))
        self.start_after_s = float(start_after_s)
        self.start_delay_s = float(config.get("start_delay_s", 1.0))
        self.level_before_acceleration_s = float(
            config.get("level_before_acceleration_s", 0.20)
        )
        self.accelerate_s = float(config.get("accelerate_s", 0.35))
        self.level_before_brake_s = float(
            config.get("level_before_brake_s", 0.20)
        )
        self.brake_s = float(config.get("brake_s", 0.45))
        self.level_after_brake_s = float(
            config.get("level_after_brake_s", 0.60)
        )
        self.recovery_s = float(config.get("recovery_s", 1.50))
        configured_tilt_levels = config.get("tilt_levels_deg")
        if configured_tilt_levels is None:
            tilt_levels = np.asarray([
                abs(float(config.get("tilt_deg", 8.0)))
            ], dtype=float)
            self.repetitions_per_tilt = int(config.get("repetitions", 3))
        else:
            tilt_levels = np.asarray(configured_tilt_levels, dtype=float)
            # A recursively merged config may still contain the legacy scalar
            # ``tilt_deg``/``repetitions`` defaults.  Explicit multi-level
            # fields take precedence and avoid accidentally multiplying the
            # six intended trials into eighteen.
            self.repetitions_per_tilt = int(
                config.get("repetitions_per_tilt", 1)
            )
        if tilt_levels.ndim != 1 or len(tilt_levels) == 0:
            raise ValueError(
                "braking calibration tilt_levels_deg must be a non-empty list"
            )
        if (
            not np.all(np.isfinite(tilt_levels))
            or np.any(tilt_levels <= 0.0)
            or np.any(tilt_levels >= 30.0)
        ):
            raise ValueError(
                "braking calibration tilt levels must be between 0 and 30 deg"
            )
        if len(np.unique(np.round(tilt_levels, decimals=9))) != len(tilt_levels):
            raise ValueError("braking calibration tilt levels must be unique")
        if np.any(np.diff(tilt_levels) <= 0.0):
            raise ValueError(
                "braking calibration tilt levels must be strictly increasing"
            )
        self.tilt_levels_deg = tilt_levels.copy()
        self.tilt_deg = float(np.max(self.tilt_levels_deg))
        self.accelerate_durations_s = None
        self.brake_durations_s = None
        if "accelerate_durations_s" in config:
            durations = np.asarray(config["accelerate_durations_s"], dtype=float)
            repeats = config.get("repetitions_per_duration", 1)
            if (durations.ndim != 1 or len(durations) == 0
                    or not np.all(np.isfinite(durations))
                    or np.any(durations <= 0) or np.any(np.diff(durations) < 0)
                    or isinstance(repeats, bool) or not np.isfinite(float(repeats))
                    or int(repeats) != repeats or repeats <= 0):
                raise ValueError(
                    "acceleration durations must be positive, nondecreasing "
                    "with integer repeats"
                )
            if len(self.tilt_levels_deg) != 1:
                raise ValueError("duration sweep requires one fixed tilt angle")
            self.accelerate_durations_s = durations.copy()
            if "brake_durations_s" in config:
                brake_durations = np.asarray(
                    config["brake_durations_s"], dtype=float
                )
                if (
                    brake_durations.ndim != 1
                    or brake_durations.shape != durations.shape
                    or not np.all(np.isfinite(brake_durations))
                    or np.any(brake_durations <= 0)
                    or np.any(np.diff(brake_durations) < 0)
                ):
                    raise ValueError(
                        "braking durations must be positive, nondecreasing, "
                        "and match acceleration durations"
                    )
                self.brake_durations_s = brake_durations.copy()
            self.repetitions_per_duration = int(repeats)
            # Each duration is a repetition at the same tilt for the existing
            # signed-gain quality checks. Brake times match acceleration times
            # unless an explicit pairwise brake schedule is configured.
            self.repetitions_per_tilt = len(durations) * int(repeats)
            self.accelerate_s = float(max(durations))
            self.brake_s = float(max(
                durations if self.brake_durations_s is None
                else self.brake_durations_s
            ))
        elif "brake_durations_s" in config:
            raise ValueError(
                "brake_durations_s requires accelerate_durations_s"
            )
        # ``repetitions`` remains the number of trials per direction.  Keeping
        # this aggregate makes the persisted protocol compatible with the
        # existing signed-quality contract while each individual level is also
        # recorded explicitly.
        self.repetitions = int(
            len(self.tilt_levels_deg) * self.repetitions_per_tilt
        )
        self.max_xy_speed_m_s = float(
            config.get("max_xy_speed_m_s", 0.70)
        )
        self.max_displacement_m = float(
            config.get("max_displacement_m", 0.45)
        )
        self.trial_start_max_xy_speed_m_s = float(
            config.get("trial_start_max_xy_speed_m_s", 0.05)
        )
        self.trial_start_max_tilt_deg = float(
            config.get("trial_start_max_tilt_deg", 4.0)
        )
        directions = np.asarray(
            config.get("directions_xy", [[0.0, 1.0], [0.0, -1.0]]),
            dtype=float,
        )
        timing = np.asarray([
            self.start_after_s,
            self.start_delay_s,
            self.level_before_acceleration_s,
            self.accelerate_s,
            self.level_before_brake_s,
            self.brake_s,
            self.level_after_brake_s,
            self.recovery_s,
            self.tilt_deg,
            self.max_xy_speed_m_s,
            self.max_displacement_m,
            self.trial_start_max_xy_speed_m_s,
            self.trial_start_max_tilt_deg,
        ])
        if not np.all(np.isfinite(timing)) or np.any(timing < 0.0):
            raise ValueError("braking calibration timing and limits must be finite")
        if (
            self.repetitions <= 0
            or self.repetitions_per_tilt <= 0
            or self.level_before_acceleration_s <= 0.0
            or self.accelerate_s <= 0.0
            or self.level_before_brake_s <= 0.0
            or self.brake_s <= 0.0
            or self.level_after_brake_s <= 0.0
            or self.recovery_s <= 0.0
            or self.max_xy_speed_m_s <= 0.0
            or self.max_displacement_m <= 0.0
            or self.trial_start_max_xy_speed_m_s <= 0.0
            or self.trial_start_max_tilt_deg <= 0.0
        ):
            raise ValueError("braking calibration durations and limits must be positive")
        if directions.ndim != 2 or directions.shape[1] != 2 or len(directions) == 0:
            raise ValueError("braking calibration directions_xy must contain XY pairs")
        if not np.all(np.isfinite(directions)):
            raise ValueError("braking calibration directions must be finite")
        norms = np.linalg.norm(directions, axis=1)
        if np.any(norms <= 1e-9):
            raise ValueError("braking calibration directions cannot be zero")
        self.directions = directions / norms[:, None]
        direction_projection = self.directions @ self.directions[0]
        # Single-direction data collection is explicit at the call site, not a
        # YAML switch that could weaken the ordinary calibration contract.
        single_direction_test = (
            require_opposed_directions is False and len(self.directions) == 1
        )
        if not single_direction_test and (
            np.any(np.abs(direction_projection) < 0.98)
            or not np.any(direction_projection > 0.98)
            or not np.any(direction_projection < -0.98)
        ):
            raise ValueError(
                "braking calibration requires one pair of opposed directions"
            )
        repeated_levels = np.tile(
            self.tilt_levels_deg, self.repetitions_per_tilt
        )
        self.trial_directions = np.tile(
            self.directions, (len(repeated_levels), 1)
        )
        self.trial_tilt_levels_deg = np.repeat(
            repeated_levels, len(self.directions)
        )
        self.attitude_trial_s = (
            self.level_before_acceleration_s
            + self.accelerate_s
            + self.level_before_brake_s
            + self.brake_s
            + self.level_after_brake_s
        )
        self.trial_s = self.attitude_trial_s + self.recovery_s
        self.maneuver_start_s = self.start_after_s + self.start_delay_s
        count = len(self.trial_directions)
        self.trial_accelerate_s = np.full(count, self.accelerate_s)
        self.trial_brake_s = np.full(count, self.brake_s)
        if self.accelerate_durations_s is not None:
            self.trial_accelerate_s = np.repeat(np.tile(
                self.accelerate_durations_s, self.repetitions_per_duration
            ), len(self.directions))
            scheduled_brake_s = (
                self.accelerate_durations_s
                if self.brake_durations_s is None
                else self.brake_durations_s
            )
            self.trial_brake_s = np.repeat(np.tile(
                scheduled_brake_s, self.repetitions_per_duration
            ), len(self.directions))
        self.trial_attitude_durations_s = (
            self.level_before_acceleration_s + self.trial_accelerate_s
            + self.level_before_brake_s + self.trial_brake_s
            + self.level_after_brake_s
        )
        self.trial_durations_s = self.trial_attitude_durations_s + self.recovery_s
        self.trial_start_s = self.maneuver_start_s + np.r_[
            0.0, np.cumsum(self.trial_durations_s)[:-1]
        ]
        self.end_s = float(self.maneuver_start_s + sum(self.trial_durations_s))

    def timing_protocol(self):
        """Exact per-trial timing; scalar durations alone cannot describe a sweep."""
        return {
            "accelerate_durations_s": (
                None if self.accelerate_durations_s is None
                else self.accelerate_durations_s.tolist()
            ),
            "brake_durations_s": (
                None if self.brake_durations_s is None
                else self.brake_durations_s.tolist()
            ),
            "repetitions_per_duration": getattr(self, "repetitions_per_duration", None),
            "trial_accelerate_s": self.trial_accelerate_s.tolist(),
            "trial_brake_s": self.trial_brake_s.tolist(),
            "trial_start_s": self.trial_start_s.tolist(),
            "trial_durations_s": self.trial_durations_s.tolist(),
        }

    @property
    def duration_s(self):
        if not self.enabled:
            return 0.0
        return self.end_s

    @staticmethod
    def _world_acceleration_to_attitude(acceleration_xy, yaw_deg):
        acceleration = np.asarray(acceleration_xy, dtype=float)
        norm = float(np.linalg.norm(acceleration))
        if norm <= 1e-9:
            return 0.0, 0.0
        yaw_rad = np.radians(float(yaw_deg))
        cos_y = np.cos(yaw_rad)
        sin_y = np.sin(yaw_rad)
        acceleration_body = np.array([
            acceleration[0] * cos_y + acceleration[1] * sin_y,
            -acceleration[0] * sin_y + acceleration[1] * cos_y,
        ])
        tilt_deg = float(np.degrees(np.arctan2(norm, 9.81)))
        pitch_deg = -tilt_deg * float(acceleration_body[0] / norm)
        roll_deg = -tilt_deg * float(acceleration_body[1] / norm)
        return roll_deg, pitch_deg

    def command(self, elapsed_s, yaw_deg):
        """Return the scheduled command for an elapsed calibration time."""
        elapsed_s = float(elapsed_s)
        yaw_deg = float(yaw_deg)
        if not np.all(np.isfinite([elapsed_s, yaw_deg])):
            raise ValueError("braking calibration command time/yaw must be finite")
        zero = np.zeros(2)
        if not self.enabled or elapsed_s < self.maneuver_start_s:
            return BrakingCalibrationCommand(
                active=False,
                attitude_control=False,
                phase="waiting",
                segment_id=None,
                direction_xy=zero,
                command_acceleration_xy=zero,
                tilt_deg=0.0,
                roll_deg=0.0,
                pitch_deg=0.0,
            )
        if elapsed_s >= self.end_s:
            return BrakingCalibrationCommand(
                active=False,
                attitude_control=False,
                phase="complete",
                segment_id=None,
                direction_xy=zero,
                command_acceleration_xy=zero,
                tilt_deg=0.0,
                roll_deg=0.0,
                pitch_deg=0.0,
            )

        segment_id = int(np.searchsorted(self.trial_start_s, elapsed_s, side="right") - 1)
        local_s = elapsed_s - self.trial_start_s[segment_id]
        accelerate_s = self.trial_accelerate_s[segment_id]
        brake_s = self.trial_brake_s[segment_id]
        direction = self.trial_directions[segment_id].copy()
        tilt_deg = float(self.trial_tilt_levels_deg[segment_id])
        acceleration_m_s2 = 9.81 * np.tan(np.radians(tilt_deg))

        if local_s < self.level_before_acceleration_s:
            phase = "level_before_acceleration"
            command_acceleration = zero
        elif local_s < self.level_before_acceleration_s + accelerate_s:
            phase = "accelerate"
            command_acceleration = acceleration_m_s2 * direction
        elif local_s < (
            self.level_before_acceleration_s
            + accelerate_s
            + self.level_before_brake_s
        ):
            phase = "level_before_brake"
            command_acceleration = zero
        elif local_s < (
            self.level_before_acceleration_s
            + accelerate_s
            + self.level_before_brake_s
            + brake_s
        ):
            phase = "brake"
            command_acceleration = -acceleration_m_s2 * direction
        elif local_s < self.trial_attitude_durations_s[segment_id]:
            phase = "level_after_brake"
            command_acceleration = zero
        else:
            phase = "recovery"
            command_acceleration = zero

        attitude_control = phase in self.ATTITUDE_PHASES
        roll_deg, pitch_deg = self._world_acceleration_to_attitude(
            command_acceleration, yaw_deg
        )
        return BrakingCalibrationCommand(
            active=True,
            attitude_control=attitude_control,
            phase=phase,
            segment_id=segment_id,
            direction_xy=direction,
            command_acceleration_xy=command_acceleration.copy(),
            tilt_deg=tilt_deg,
            roll_deg=float(roll_deg),
            pitch_deg=float(pitch_deg),
        )
