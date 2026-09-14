"""Contact-period attitude propagation from raw Crazyflie IMU samples.

This module is deliberately pure math.  It has no cflib, commander, handoff,
or controller imports, which keeps the first implementation shadow-only and
makes the exact same golden vectors reusable by a later firmware port.

The stored quaternion is the Crazyflie estimator's native body-to-world
quaternion in ``[w, x, y, z]`` order.  Crazyflie's legacy Euler convention
negates pitch, while the native quaternion does not.  Consequently raw sensor
rates map to legacy rates as ``[gyro.x, -gyro.y, gyro.z]`` even though the
quaternion integrates the native ``[gyro.x, gyro.y, gyro.z]`` body rates.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


CF_TIMESTAMP_MODULUS_MS = 1 << 24


def _vector3(value: Sequence[float], name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain three finite values")
    return result


def _normalize_quaternion(quaternion: Sequence[float]) -> np.ndarray:
    result = np.asarray(quaternion, dtype=float)
    if result.shape != (4,) or not np.all(np.isfinite(result)):
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(result))
    if norm <= 1e-12:
        raise ValueError("quaternion must have non-zero norm")
    result = result / norm
    # A canonical hemisphere makes logs and Python/C golden vectors stable.
    return -result if result[0] < 0.0 else result


def quaternion_multiply(left: Sequence[float], right: Sequence[float]) -> np.ndarray:
    """Hamilton product of two ``[w, x, y, z]`` quaternions."""
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array([
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ], dtype=float)


def quaternion_from_native_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return body-to-world quaternion for intrinsic XYZ / Rz Ry Rx Euler."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return _normalize_quaternion([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def native_rpy_from_quaternion(quaternion: Sequence[float]) -> np.ndarray:
    """Return native quaternion roll/pitch/yaw, before legacy pitch negation."""
    w, x, y, z = _normalize_quaternion(quaternion)
    roll = math.atan2(2.0 * (w * x + y * z),
                      1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    yaw = math.atan2(2.0 * (w * z + x * y),
                     1.0 - 2.0 * (y * y + z * z))
    return np.array([roll, pitch, yaw], dtype=float)


def legacy_rpy_from_quaternion(quaternion: Sequence[float]) -> np.ndarray:
    """Return Crazyflie ``stateEstimate`` roll/pitch/yaw in radians."""
    native = native_rpy_from_quaternion(quaternion)
    native[1] *= -1.0
    return native


def quaternion_from_accel_and_legacy_yaw(
        accel_g: Sequence[float], legacy_yaw_rad: float) -> np.ndarray:
    """Build a native quaternion from a stationary accelerometer and EKF yaw.

    At stable hover the normalized accelerometer is the world-up direction
    expressed in body axes.  It determines roll/pitch but cannot observe yaw,
    so the concurrent EKF yaw supplies that final degree of freedom.
    """
    acceleration = _vector3(accel_g, "accel_g")
    norm = float(np.linalg.norm(acceleration))
    if norm <= 1e-6 or not math.isfinite(legacy_yaw_rad):
        raise ValueError("acceleration norm and yaw must be valid")
    ax, ay, az = acceleration / norm
    native_roll = math.atan2(ay, az)
    native_pitch = math.atan2(-ax, math.hypot(ay, az))
    return quaternion_from_native_rpy(
        native_roll, native_pitch, float(legacy_yaw_rad)
    )


def integrate_body_rate(
        quaternion: Sequence[float], body_rate_rad_s: Sequence[float], dt_s: float,
) -> np.ndarray:
    """Integrate a constant native body rate with the exact exponential map."""
    q = _normalize_quaternion(quaternion)
    omega = _vector3(body_rate_rad_s, "body_rate_rad_s")
    dt_s = float(dt_s)
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("dt_s must be finite and positive")
    rotation = omega * dt_s
    angle = float(np.linalg.norm(rotation))
    if angle <= 1e-12:
        delta = np.array([1.0, *(0.5 * rotation)], dtype=float)
    else:
        half_angle = 0.5 * angle
        delta = np.array([
            math.cos(half_angle),
            *(math.sin(half_angle) * rotation / angle),
        ], dtype=float)
    # Body-frame increments post-multiply a body-to-world quaternion.
    return _normalize_quaternion(quaternion_multiply(q, delta))


@dataclass(frozen=True)
class ContactAttitudeConfig:
    timestamp_modulus_ms: int = CF_TIMESTAMP_MODULUS_MS
    max_gyro_gap_ms: float = 5.0
    alignment_window_ms: float = 300.0
    alignment_min_samples: int = 20
    alignment_accel_norm_tolerance_g: float = 0.12
    alignment_mean_accel_norm_tolerance_g: float = 0.05
    alignment_max_accel_direction_rms_deg: float = 2.0
    alignment_max_tilt_deg: float = 12.0
    alignment_max_gyro_std_deg_s: float = 1.5
    alignment_max_mean_gyro_deg_s: float = 3.0
    alignment_max_sample_gap_ms: float = 5.0
    alignment_min_yaw_resultant: float = 0.98
    alignment_max_yaw_deviation_deg: float = 5.0

    def __post_init__(self):
        if self.timestamp_modulus_ms <= 2:
            raise ValueError("timestamp_modulus_ms must exceed two")
        if self.max_gyro_gap_ms <= 0.0 or self.alignment_window_ms <= 0.0:
            raise ValueError("timestamp windows must be positive")
        if self.alignment_min_samples < 2:
            raise ValueError("alignment_min_samples must be at least two")
        positive = (
            self.alignment_accel_norm_tolerance_g,
            self.alignment_mean_accel_norm_tolerance_g,
            self.alignment_max_accel_direction_rms_deg,
            self.alignment_max_tilt_deg,
            self.alignment_max_gyro_std_deg_s,
            self.alignment_max_mean_gyro_deg_s,
            self.alignment_max_sample_gap_ms,
            self.alignment_min_yaw_resultant,
            self.alignment_max_yaw_deviation_deg,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("alignment limits must be finite and positive")
        if self.alignment_min_yaw_resultant > 1.0:
            raise ValueError("alignment yaw resultant cannot exceed one")


@dataclass(frozen=True)
class ContactAttitudeEstimate:
    valid: bool
    reason: str
    phase: str
    cf_timestamp_ms: int | None
    unwrapped_timestamp_ms: int | None
    quaternion_wxyz: tuple[float, float, float, float] | None
    legacy_rpy_rad: tuple[float, float, float] | None
    native_body_rate_rad_s: tuple[float, float, float] | None
    legacy_body_rate_rad_s: tuple[float, float, float] | None
    gyro_bias_deg_s: tuple[float, float, float] | None
    sample_count: int
    max_gap_ms: float


class ContactAttitudeObserver:
    """Initialize in stable hover, then propagate gyro-only during contact."""

    ALIGNING = "aligning"
    READY = "ready"
    CONTACT = "contact"
    RELEASED = "released"
    INVALID = "invalid"

    def __init__(self, config: ContactAttitudeConfig | None = None):
        self.config = config or ContactAttitudeConfig()
        self.reset()

    def reset(self) -> None:
        self.phase = self.ALIGNING
        self.reason = "alignment_not_ready"
        self._alignment_samples = []
        self._quaternion = None
        self._bias_deg_s = None
        self._last_raw_timestamp = None
        self._unwrapped_timestamp = None
        self._last_gyro_deg_s = None
        self._native_rate_rad_s = None
        self._sample_count = 0
        self._max_gap_ms = 0.0

    def _unwrap(self, raw_timestamp_ms: int) -> tuple[int | None, str | None]:
        raw = int(raw_timestamp_ms)
        modulus = self.config.timestamp_modulus_ms
        if raw < 0 or raw >= modulus:
            return None, "timestamp_out_of_range"
        if self._last_raw_timestamp is None:
            self._last_raw_timestamp = raw
            self._unwrapped_timestamp = raw
            return raw, None
        raw_delta = (raw - self._last_raw_timestamp) % modulus
        if raw_delta == 0:
            return None, "duplicate_timestamp"
        if raw_delta >= modulus // 2:
            return None, "backward_timestamp"
        self._last_raw_timestamp = raw
        self._unwrapped_timestamp += raw_delta
        return self._unwrapped_timestamp, None

    def add_alignment_sample(
            self, cf_timestamp_ms: int, accel_g: Sequence[float],
            gyro_deg_s: Sequence[float], ekf_legacy_yaw_deg: float,
    ) -> ContactAttitudeEstimate:
        if self.phase in (self.CONTACT, self.RELEASED, self.INVALID):
            return self.snapshot("alignment_frozen")
        acceleration = _vector3(accel_g, "accel_g")
        gyro = _vector3(gyro_deg_s, "gyro_deg_s")
        yaw = float(ekf_legacy_yaw_deg)
        unwrapped, timestamp_error = self._unwrap(cf_timestamp_ms)
        if timestamp_error:
            return self.snapshot(timestamp_error)
        if not math.isfinite(yaw):
            return self.snapshot("invalid_alignment_yaw")
        if abs(float(np.linalg.norm(acceleration)) - 1.0) > (
                self.config.alignment_accel_norm_tolerance_g):
            self._alignment_samples.clear()
            self.phase = self.ALIGNING
            self.reason = "alignment_acceleration_unstable"
            return self.snapshot()
        self._alignment_samples.append((unwrapped, acceleration, gyro, yaw))
        cutoff = unwrapped - self.config.alignment_window_ms
        self._alignment_samples = [
            sample for sample in self._alignment_samples if sample[0] >= cutoff
        ]
        if len(self._alignment_samples) >= 2:
            latest_gap_ms = (
                self._alignment_samples[-1][0]
                - self._alignment_samples[-2][0]
            )
            if latest_gap_ms > self.config.alignment_max_sample_gap_ms:
                # Never certify a window that silently bridges missing IMU
                # history.  Keep only the first sample after the hole so a new,
                # contiguous stable-hover interval can form.
                self._alignment_samples = [self._alignment_samples[-1]]
                self.phase = self.ALIGNING
                self.reason = "alignment_sample_gap"
                return self.snapshot()
        if len(self._alignment_samples) < self.config.alignment_min_samples:
            self.reason = "alignment_insufficient_samples"
            return self.snapshot()
        span_ms = self._alignment_samples[-1][0] - self._alignment_samples[0][0]
        if span_ms < self.config.alignment_window_ms:
            self.reason = "alignment_window_short"
            return self.snapshot()
        gyros = np.stack([sample[2] for sample in self._alignment_samples])
        if np.any(np.std(gyros, axis=0) > self.config.alignment_max_gyro_std_deg_s):
            self.reason = "alignment_gyro_unstable"
            return self.snapshot()
        mean_gyro = np.mean(gyros, axis=0)
        if float(np.linalg.norm(mean_gyro)) > (
                self.config.alignment_max_mean_gyro_deg_s):
            # A constant rotation has zero standard deviation and must not be
            # mistaken for gyro bias during the stationary alignment phase.
            self.reason = "alignment_mean_rotation_exceeded"
            return self.snapshot()
        accelerations = np.stack([sample[1] for sample in self._alignment_samples])
        yaws = np.radians([sample[3] for sample in self._alignment_samples])
        accel_norms = np.linalg.norm(accelerations, axis=1)
        mean_acceleration = np.mean(accelerations, axis=0)
        mean_accel_norm = float(np.linalg.norm(mean_acceleration))
        if abs(mean_accel_norm - 1.0) > (
                self.config.alignment_mean_accel_norm_tolerance_g):
            self.reason = "alignment_mean_acceleration_not_gravity"
            return self.snapshot()
        unit_accelerations = accelerations / accel_norms[:, None]
        mean_direction = mean_acceleration / mean_accel_norm
        direction_errors_deg = np.degrees(np.arccos(np.clip(
            unit_accelerations @ mean_direction, -1.0, 1.0
        )))
        direction_rms_deg = float(np.sqrt(np.mean(direction_errors_deg ** 2)))
        if direction_rms_deg > (
                self.config.alignment_max_accel_direction_rms_deg):
            self.reason = "alignment_acceleration_direction_unstable"
            return self.snapshot()
        tilt_deg = math.degrees(math.acos(float(np.clip(
            mean_direction[2], -1.0, 1.0
        ))))
        if tilt_deg > self.config.alignment_max_tilt_deg:
            self.reason = "alignment_tilt_exceeded"
            return self.snapshot()
        mean_sin = float(np.mean(np.sin(yaws)))
        mean_cos = float(np.mean(np.cos(yaws)))
        yaw_resultant = math.hypot(mean_sin, mean_cos)
        if yaw_resultant < self.config.alignment_min_yaw_resultant:
            self.reason = "alignment_yaw_ambiguous"
            return self.snapshot()
        mean_yaw = math.atan2(mean_sin, mean_cos)
        yaw_deviations_deg = np.degrees(np.arctan2(
            np.sin(yaws - mean_yaw), np.cos(yaws - mean_yaw)
        ))
        if float(np.max(np.abs(yaw_deviations_deg))) > (
                self.config.alignment_max_yaw_deviation_deg):
            self.reason = "alignment_yaw_unstable"
            return self.snapshot()
        self._quaternion = quaternion_from_accel_and_legacy_yaw(
            mean_acceleration, mean_yaw
        )
        self._bias_deg_s = mean_gyro
        self._last_gyro_deg_s = gyros[-1].copy()
        self.phase = self.READY
        self.reason = "ready"
        return self.snapshot()

    def begin_contact(self) -> ContactAttitudeEstimate:
        """Freeze accelerometer alignment at contact-candidate start."""
        if self.phase != self.READY or self._quaternion is None:
            return self._invalidate("contact_before_alignment_ready")
        self.phase = self.CONTACT
        self.reason = "contact_started"
        return self.snapshot()

    def begin_contact_from_state(
            self, quaternion_wxyz: Sequence[float],
            gyro_bias_deg_s: Sequence[float], cf_timestamp_ms: int,
            gyro_deg_s: Sequence[float],
            unwrapped_timestamp_ms: int | None = None,
    ) -> ContactAttitudeEstimate:
        """Start a later contact from a released inertial-EKF state.

        This is the recontact counterpart of stable-hover alignment.  The
        post-release EKF has already fused acceleration and position, so its
        attitude and gyro bias become the next gyro-only contact seed without
        replaying command history or consuming an external-pose quaternion.
        """
        quaternion = _normalize_quaternion(quaternion_wxyz)
        bias = _vector3(gyro_bias_deg_s, "gyro_bias_deg_s")
        gyro = _vector3(gyro_deg_s, "gyro_deg_s")
        raw = int(cf_timestamp_ms)
        if not 0 <= raw < self.config.timestamp_modulus_ms:
            return self._invalidate("timestamp_out_of_range")
        unwrapped = raw if unwrapped_timestamp_ms is None else int(
            unwrapped_timestamp_ms
        )
        if unwrapped < raw:
            return self._invalidate("unwrapped_timestamp_before_raw")
        self._alignment_samples = []
        self._quaternion = quaternion
        self._bias_deg_s = bias
        self._last_raw_timestamp = raw
        self._unwrapped_timestamp = unwrapped
        self._last_gyro_deg_s = gyro
        corrected_rad_s = np.radians(gyro - bias)
        self._native_rate_rad_s = corrected_rad_s
        self._sample_count = 0
        self._max_gap_ms = 0.0
        self.phase = self.CONTACT
        self.reason = "contact_reseeded_from_post_release_ekf"
        return self.snapshot()

    def add_gyro_sample(
            self, cf_timestamp_ms: int, gyro_deg_s: Sequence[float],
    ) -> ContactAttitudeEstimate:
        if self.phase != self.CONTACT:
            return self.snapshot("gyro_outside_contact")
        gyro = _vector3(gyro_deg_s, "gyro_deg_s")
        previous_unwrapped = self._unwrapped_timestamp
        unwrapped, timestamp_error = self._unwrap(cf_timestamp_ms)
        if timestamp_error:
            if timestamp_error == "duplicate_timestamp":
                return self.snapshot(timestamp_error)
            return self._invalidate(timestamp_error)
        gap_ms = float(unwrapped - previous_unwrapped)
        self._max_gap_ms = max(self._max_gap_ms, gap_ms)
        if gap_ms > self.config.max_gyro_gap_ms:
            return self._invalidate("gyro_gap_exceeded")
        corrected = gyro - self._bias_deg_s
        # Trapezoidal rate integration reduces quantization error without
        # inventing data through a packet gap.
        previous_corrected = self._last_gyro_deg_s - self._bias_deg_s
        mean_rate_rad_s = np.radians(0.5 * (previous_corrected + corrected))
        self._quaternion = integrate_body_rate(
            self._quaternion, mean_rate_rad_s, gap_ms / 1000.0
        )
        self._last_gyro_deg_s = gyro
        self._native_rate_rad_s = np.radians(corrected)
        self._sample_count += 1
        self.reason = "propagating"
        return self.snapshot()

    def release(self) -> ContactAttitudeEstimate:
        if self.phase != self.CONTACT:
            return self._invalidate("release_outside_contact")
        self.phase = self.RELEASED
        self.reason = "released"
        return self.snapshot()

    def _invalidate(self, reason: str) -> ContactAttitudeEstimate:
        self.phase = self.INVALID
        self.reason = reason
        return self.snapshot()

    def snapshot(self, transient_reason: str | None = None) -> ContactAttitudeEstimate:
        quaternion = None if self._quaternion is None else tuple(
            float(value) for value in self._quaternion
        )
        legacy_rpy = None if self._quaternion is None else tuple(
            float(value) for value in legacy_rpy_from_quaternion(self._quaternion)
        )
        native_rate = None if self._native_rate_rad_s is None else tuple(
            float(value) for value in self._native_rate_rad_s
        )
        legacy_rate = None if native_rate is None else (
            native_rate[0], -native_rate[1], native_rate[2]
        )
        bias = None if self._bias_deg_s is None else tuple(
            float(value) for value in self._bias_deg_s
        )
        return ContactAttitudeEstimate(
            valid=self.phase in (self.READY, self.CONTACT, self.RELEASED),
            reason=transient_reason or self.reason,
            phase=self.phase,
            cf_timestamp_ms=self._last_raw_timestamp,
            unwrapped_timestamp_ms=self._unwrapped_timestamp,
            quaternion_wxyz=quaternion,
            legacy_rpy_rad=legacy_rpy,
            native_body_rate_rad_s=native_rate,
            legacy_body_rate_rad_s=legacy_rate,
            gyro_bias_deg_s=bias,
            sample_count=self._sample_count,
            max_gap_ms=self._max_gap_ms,
        )
