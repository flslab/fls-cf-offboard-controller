"""Shadow-only post-release inertial EKF.

The filter uses measured IMU specific force as a process input and a
time-aligned position-only localization stream as its observation.  It
intentionally does not consume command history.  Position innovations correct
position, velocity, attitude, and IMU biases through the propagated error-state
covariance.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from Interaction.contact_attitude_observer import (
    CF_TIMESTAMP_MODULUS_MS,
    _normalize_quaternion,
    _vector3,
    integrate_body_rate,
    quaternion_multiply,
)


GRAVITY_M_S2 = 9.80665


def _skew(vector: Sequence[float]) -> np.ndarray:
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def error_state_reset_jacobian(attitude_error: Sequence[float]) -> np.ndarray:
    """Map posterior covariance after right-error attitude injection."""
    correction = _vector3(attitude_error, "attitude_error")
    reset = np.eye(15)
    reset[6:9, 6:9] = np.eye(3) - 0.5 * _skew(correction)
    return reset


def rotation_matrix(quaternion_wxyz: Sequence[float]) -> np.ndarray:
    """Body-to-world rotation matrix."""
    w, x, y, z = _normalize_quaternion(quaternion_wxyz)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


@dataclass(frozen=True)
class PostReleaseEkfConfig:
    max_imu_gap_ms: float = 5.0
    gyro_noise_rad_s_sqrt_hz: float = math.radians(0.35)
    accel_noise_m_s2_sqrt_hz: float = 0.18
    gyro_bias_walk_rad_s2_sqrt_hz: float = math.radians(0.02)
    accel_bias_walk_m_s3_sqrt_hz: float = 0.02
    extpos_std_m: float = 0.003
    extpos_innovation_gate_sigma: float = 6.0

    def __post_init__(self):
        values = (
            self.max_imu_gap_ms, self.gyro_noise_rad_s_sqrt_hz,
            self.accel_noise_m_s2_sqrt_hz,
            self.gyro_bias_walk_rad_s2_sqrt_hz,
            self.accel_bias_walk_m_s3_sqrt_hz, self.extpos_std_m,
            self.extpos_innovation_gate_sigma,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("all post-release EKF configuration values must be positive")


@dataclass(frozen=True)
class PostReleaseEkfEstimate:
    valid: bool
    reason: str
    cf_timestamp_ms: int
    unwrapped_timestamp_ms: int
    position_m: tuple[float, float, float]
    velocity_m_s: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    gyro_bias_rad_s: tuple[float, float, float]
    accel_bias_m_s2: tuple[float, float, float]
    world_acceleration_m_s2: tuple[float, float, float]
    position_update_count: int
    rejected_position_count: int
    max_imu_gap_ms: float
    covariance_diagonal: tuple[float, ...]


class PostReleaseInertialEkf:
    """15-state error-state INS initialized from the release snapshot.

    Error-state order is position, velocity, right-multiplicative body attitude
    error, gyro bias, accelerometer bias.  The nominal quaternion is always
    propagated from measured gyro, while accelerometer specific force drives
    velocity directly.  Extpos is position-only; no external quaternion enters.
    """

    def __init__(
            self, position_m: Sequence[float], velocity_m_s: Sequence[float],
            quaternion_wxyz: Sequence[float], gyro_bias_rad_s: Sequence[float],
            cf_timestamp_ms: int, config: PostReleaseEkfConfig | None = None,
            unwrapped_timestamp_ms: int | None = None,
            initial_gyro_deg_s: Sequence[float] | None = None,
    ):
        self.config = config or PostReleaseEkfConfig()
        self.position = _vector3(position_m, "position_m")
        self.velocity = _vector3(velocity_m_s, "velocity_m_s")
        self.quaternion = _normalize_quaternion(quaternion_wxyz)
        self.gyro_bias = _vector3(gyro_bias_rad_s, "gyro_bias_rad_s")
        self._last_gyro_rad_s = (
            None if initial_gyro_deg_s is None
            else np.radians(_vector3(
                initial_gyro_deg_s, "initial_gyro_deg_s"
            ))
        )
        self.accel_bias = np.zeros(3)
        self._raw_timestamp = int(cf_timestamp_ms)
        if not 0 <= self._raw_timestamp < CF_TIMESTAMP_MODULUS_MS:
            raise ValueError("cf_timestamp_ms outside 24-bit range")
        self._unwrapped_timestamp = (
            self._raw_timestamp
            if unwrapped_timestamp_ms is None
            else int(unwrapped_timestamp_ms)
        )
        if (
            self._unwrapped_timestamp < self._raw_timestamp
            or self._unwrapped_timestamp % CF_TIMESTAMP_MODULUS_MS
            != self._raw_timestamp
        ):
            raise ValueError(
                "unwrapped_timestamp_ms must match the raw 24-bit epoch"
            )
        self._valid = True
        self._reason = "initialized"
        self._world_acceleration = np.zeros(3)
        self._position_update_count = 0
        self._rejected_position_count = 0
        self._max_imu_gap_ms = 0.0
        # Release uncertainty: position is the causal forwarded extpos sample,
        # while velocity is the atomic onboard-EKF sample. Neither is treated as
        # independent attitude truth; attitude/bias retain freedom. These priors
        # still require hardware-log calibration before any future authority.
        standard_deviations = np.r_[
            [0.004] * 3, [0.08] * 3, [math.radians(4.0)] * 3,
            [math.radians(0.8)] * 3, [0.03] * 3,
        ]
        self.covariance = np.diag(standard_deviations ** 2)

    def _advance_timestamp(self, raw_timestamp_ms: int) -> tuple[float | None, str | None]:
        raw = int(raw_timestamp_ms)
        if not 0 <= raw < CF_TIMESTAMP_MODULUS_MS:
            return None, "timestamp_out_of_range"
        delta = (raw - self._raw_timestamp) % CF_TIMESTAMP_MODULUS_MS
        if delta == 0:
            return None, "duplicate_timestamp"
        if delta >= CF_TIMESTAMP_MODULUS_MS // 2:
            return None, "backward_timestamp"
        if delta > self.config.max_imu_gap_ms:
            self._max_imu_gap_ms = max(self._max_imu_gap_ms, float(delta))
            return None, "imu_gap_exceeded"
        self._raw_timestamp = raw
        self._unwrapped_timestamp += delta
        self._max_imu_gap_ms = max(self._max_imu_gap_ms, float(delta))
        return delta / 1000.0, None

    def propagate(
            self, cf_timestamp_ms: int, gyro_deg_s: Sequence[float],
            accel_g: Sequence[float],
    ) -> PostReleaseEkfEstimate:
        if not self._valid:
            return self.snapshot()
        gyro = np.radians(_vector3(gyro_deg_s, "gyro_deg_s"))
        specific_force = GRAVITY_M_S2 * _vector3(accel_g, "accel_g")
        dt, error = self._advance_timestamp(cf_timestamp_ms)
        if error == "duplicate_timestamp":
            return self.snapshot(error)
        if error:
            self._valid = False
            self._reason = error
            return self.snapshot()

        # Match the contact observer's trapezoidal gyro integration exactly at
        # handoff.  This avoids a silent quaternion discontinuity when the EKF
        # replays the few IMU samples between the common state seed and release.
        mean_gyro = (
            gyro if self._last_gyro_rad_s is None
            else 0.5 * (self._last_gyro_rad_s + gyro)
        )
        omega = mean_gyro - self.gyro_bias
        self._last_gyro_rad_s = gyro
        corrected_force = specific_force - self.accel_bias
        rotation = rotation_matrix(self.quaternion)
        world_acceleration = rotation @ corrected_force + np.array([
            0.0, 0.0, -GRAVITY_M_S2
        ])
        self.position += self.velocity * dt + 0.5 * world_acceleration * dt * dt
        self.velocity += world_acceleration * dt
        self.quaternion = integrate_body_rate(self.quaternion, omega, dt)
        self._world_acceleration = world_acceleration

        transition = np.eye(15)
        transition[0:3, 3:6] = np.eye(3) * dt
        acceleration_attitude_jacobian = -rotation @ _skew(corrected_force)
        acceleration_bias_jacobian = -rotation
        half_dt_squared = 0.5 * dt * dt
        # Match the nominal p <- p + v*dt + 0.5*a*dt^2 propagation.  Without
        # these blocks, the first position innovation after release cannot use
        # the physically present position/attitude or position/bias
        # cross-covariance.
        transition[0:3, 6:9] = (
            acceleration_attitude_jacobian * half_dt_squared
        )
        transition[0:3, 12:15] = (
            acceleration_bias_jacobian * half_dt_squared
        )
        transition[3:6, 6:9] = acceleration_attitude_jacobian * dt
        transition[3:6, 12:15] = acceleration_bias_jacobian * dt
        transition[6:9, 6:9] -= _skew(omega) * dt
        transition[6:9, 9:12] = -np.eye(3) * dt

        process_covariance = np.zeros((15, 15))
        accel_noise_body = (
            np.eye(3) * self.config.accel_noise_m_s2_sqrt_hz ** 2
        )
        # The accelerometer noise density is specified in the body frame.  It
        # is isotropic today, so R Q R^T is algebraically unchanged, but retain
        # the frame transform here to keep the discretization explicit and
        # correct if axis-specific calibration is introduced later.
        accel_noise_world = rotation @ accel_noise_body @ rotation.T
        process_covariance[0:3, 0:3] = (
            accel_noise_world * dt ** 3 / 3.0
        )
        process_covariance[0:3, 3:6] = (
            accel_noise_world * dt * dt / 2.0
        )
        process_covariance[3:6, 0:3] = (
            accel_noise_world * dt * dt / 2.0
        )
        process_covariance[3:6, 3:6] = accel_noise_world * dt
        process_covariance[6:9, 6:9] = (
            np.eye(3) * self.config.gyro_noise_rad_s_sqrt_hz ** 2 * dt
        )
        process_covariance[9:12, 9:12] = (
            np.eye(3) * self.config.gyro_bias_walk_rad_s2_sqrt_hz ** 2 * dt
        )
        process_covariance[12:15, 12:15] = (
            np.eye(3) * self.config.accel_bias_walk_m_s3_sqrt_hz ** 2 * dt
        )
        self.covariance = np.einsum(
            "ij,jk,lk->il", transition, self.covariance, transition
        ) + process_covariance
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self._reason = "propagating"
        return self.snapshot()

    def update_extpos(
            self, position_m: Sequence[float], std_m: float | None = None,
    ) -> PostReleaseEkfEstimate:
        """Fuse a position-only localization observation.

        The legacy method name is retained for callers and golden-vector
        compatibility; the runtime adapter labels the actual position source.
        """
        if not self._valid:
            return self.snapshot()
        measurement = _vector3(position_m, "position_m")
        std = self.config.extpos_std_m if std_m is None else float(std_m)
        if not math.isfinite(std) or std <= 0.0:
            raise ValueError("extpos std_m must be finite and positive")
        observation = np.zeros((3, 15))
        observation[:, 0:3] = np.eye(3)
        innovation = measurement - self.position
        innovation_covariance = np.einsum(
            "ij,jk,lk->il", observation, self.covariance, observation
        ) + np.eye(3) * std * std
        mahalanobis_sq = float(
            innovation @ np.linalg.solve(innovation_covariance, innovation)
        )
        if mahalanobis_sq > self.config.extpos_innovation_gate_sigma ** 2:
            self._rejected_position_count += 1
            return self.snapshot("extpos_innovation_rejected")
        gain = np.linalg.solve(
            innovation_covariance,
            (self.covariance @ observation.T).T,
        ).T
        correction = gain @ innovation
        self.position += correction[0:3]
        self.velocity += correction[3:6]
        attitude_error = correction[6:9]
        angle = float(np.linalg.norm(attitude_error))
        if angle > 1e-12:
            delta = np.r_[math.cos(angle / 2.0),
                          math.sin(angle / 2.0) * attitude_error / angle]
            self.quaternion = _normalize_quaternion(
                quaternion_multiply(self.quaternion, delta)
            )
        self.gyro_bias += correction[9:12]
        self.accel_bias += correction[12:15]
        identity = np.eye(15)
        residual = identity - np.einsum("ij,jk->ik", gain, observation)
        measurement_covariance = np.eye(3) * std * std
        self.covariance = (
            np.einsum("ij,jk,lk->il", residual, self.covariance, residual)
            + np.einsum("ij,jk,lk->il", gain, measurement_covariance, gain)
        )
        # The Joseph update above is expressed around the pre-injection
        # nominal attitude.  Reset the local right-multiplicative error frame
        # after q <- q * Exp(dtheta); omitting this map quietly corrupts the
        # cross-covariance that lets later position innovations correct tilt.
        reset = error_state_reset_jacobian(attitude_error)
        self.covariance = np.einsum(
            "ij,jk,lk->il", reset, self.covariance, reset
        )
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self._position_update_count += 1
        self._reason = "extpos_fused"
        return self.snapshot()

    def snapshot(self, transient_reason: str | None = None) -> PostReleaseEkfEstimate:
        return PostReleaseEkfEstimate(
            valid=self._valid,
            reason=transient_reason or self._reason,
            cf_timestamp_ms=self._raw_timestamp,
            unwrapped_timestamp_ms=self._unwrapped_timestamp,
            position_m=tuple(float(value) for value in self.position),
            velocity_m_s=tuple(float(value) for value in self.velocity),
            quaternion_wxyz=tuple(float(value) for value in self.quaternion),
            gyro_bias_rad_s=tuple(float(value) for value in self.gyro_bias),
            accel_bias_m_s2=tuple(float(value) for value in self.accel_bias),
            world_acceleration_m_s2=tuple(
                float(value) for value in self._world_acceleration
            ),
            position_update_count=self._position_update_count,
            rejected_position_count=self._rejected_position_count,
            max_imu_gap_ms=self._max_imu_gap_ms,
            covariance_diagonal=tuple(
                float(value) for value in np.diag(self.covariance)
            ),
        )
