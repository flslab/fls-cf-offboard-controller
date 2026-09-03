"""Planar attitude-step calibration used by release braking.

The maneuver is intentionally simple and observable: accelerate with a small
constant tilt, command level attitude, decelerate with the opposite tilt, then
command level attitude again before returning to position hold.  It is used
only by the explicit ``--calibrate`` flight path.
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
    roll_deg: float
    pitch_deg: float


class PlanarBrakingCalibration:
    """Generate accelerate -> level -> brake -> level calibration trials."""

    ATTITUDE_PHASES = frozenset({
        "level_before_acceleration", "accelerate", "level_before_brake",
        "brake", "level_after_brake",
    })

    def __init__(self, config, start_after_s=0.0):
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
        self.tilt_deg = abs(float(config.get("tilt_deg", 8.0)))
        self.repetitions = int(config.get("repetitions", 3))
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
            or self.tilt_deg <= 0.0
            or self.tilt_deg >= 30.0
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
        if (
            np.any(np.abs(direction_projection) < 0.98)
            or not np.any(direction_projection > 0.98)
            or not np.any(direction_projection < -0.98)
        ):
            raise ValueError(
                "braking calibration requires one pair of opposed directions"
            )
        self.trial_directions = np.tile(self.directions, (self.repetitions, 1))
        self.attitude_trial_s = (
            self.level_before_acceleration_s
            + self.accelerate_s
            + self.level_before_brake_s
            + self.brake_s
            + self.level_after_brake_s
        )
        self.trial_s = self.attitude_trial_s + self.recovery_s
        self.maneuver_start_s = self.start_after_s + self.start_delay_s
        self.end_s = (
            self.maneuver_start_s
            + len(self.trial_directions) * self.trial_s
        )

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
                roll_deg=0.0,
                pitch_deg=0.0,
            )

        maneuver_elapsed = elapsed_s - self.maneuver_start_s
        segment_id = min(
            int(maneuver_elapsed // self.trial_s),
            len(self.trial_directions) - 1,
        )
        local_s = maneuver_elapsed - segment_id * self.trial_s
        direction = self.trial_directions[segment_id].copy()
        acceleration_m_s2 = 9.81 * np.tan(np.radians(self.tilt_deg))

        if local_s < self.level_before_acceleration_s:
            phase = "level_before_acceleration"
            command_acceleration = zero
        elif local_s < self.level_before_acceleration_s + self.accelerate_s:
            phase = "accelerate"
            command_acceleration = acceleration_m_s2 * direction
        elif local_s < (
            self.level_before_acceleration_s
            + self.accelerate_s
            + self.level_before_brake_s
        ):
            phase = "level_before_brake"
            command_acceleration = zero
        elif local_s < (
            self.level_before_acceleration_s
            + self.accelerate_s
            + self.level_before_brake_s
            + self.brake_s
        ):
            phase = "brake"
            command_acceleration = -acceleration_m_s2 * direction
        elif local_s < self.attitude_trial_s:
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
            roll_deg=float(roll_deg),
            pitch_deg=float(pitch_deg),
        )
