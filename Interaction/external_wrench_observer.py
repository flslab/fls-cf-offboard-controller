"""Model-based external force and torque estimation for interaction flight."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


GRAVITY = 9.81


def _as_vector(value, size: int, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.ndim == 0:
        array = np.full(size, float(array))
    if array.shape != (size,):
        raise ValueError(f"{name} must contain {size} values")
    return array


def quaternion_to_euler(quaternion: Sequence[float]) -> np.ndarray:
    """Return intrinsic XYZ roll/pitch/yaw in radians from [x, y, z, w]."""
    x, y, z, w = (float(component) for component in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return np.array([roll, pitch, yaw], dtype=float)


def unwrap_angles(previous: np.ndarray | None, current: np.ndarray) -> np.ndarray:
    if previous is None:
        return current.copy()
    delta = (current - previous + np.pi) % (2.0 * np.pi) - np.pi
    return previous + delta


def body_z_world(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Body Z direction in the world frame for Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        cy * sp * cr + sy * sr,
        sy * sp * cr - cy * sr,
        cp * cr,
    ])


class MotorWrenchModel:
    """Configurable PWM/voltage approximation of actuator acceleration.

    The default collective model is normalized so four motors at hover PWM and
    hover voltage produce one gravity of thrust. Angular acceleration scales
    remain explicit calibration parameters because they depend strongly on the
    LightBender inertia and motor layout.
    """

    DEFAULT_MIXER = np.array([
        [1.0, 1.0, -1.0, -1.0],       # roll
        [1.0, -1.0, -1.0, 1.0],       # pitch
        [-1.0, 1.0, -1.0, 1.0],       # yaw
    ]) / 4.0

    def __init__(
            self,
            hover_pwm: float = 31900.0,
            hover_voltage: float = 7.8,
            angular_accel_scale: Sequence[float] = (0.0, 0.0, 0.0),
            motor_mixer: Sequence[Sequence[float]] | None = None,
            roll_sign_for_world_thrust: float = 1.0,
            pitch_sign_for_world_thrust: float = 1.0,
    ):
        self.hover_pwm = float(hover_pwm)
        self.hover_voltage = float(hover_voltage)
        if self.hover_pwm <= 0 or self.hover_voltage <= 0:
            raise ValueError("hover_pwm and hover_voltage must be positive")
        self.angular_accel_scale = _as_vector(angular_accel_scale, 3, "angular_accel_scale")
        self.roll_sign_for_world_thrust = float(roll_sign_for_world_thrust)
        self.pitch_sign_for_world_thrust = float(pitch_sign_for_world_thrust)
        if (
            self.roll_sign_for_world_thrust not in (-1.0, 1.0)
            or self.pitch_sign_for_world_thrust not in (-1.0, 1.0)
        ):
            raise ValueError(
                "roll_sign_for_world_thrust and "
                "pitch_sign_for_world_thrust must be -1 or 1"
            )
        self.motor_mixer = np.asarray(
            self.DEFAULT_MIXER if motor_mixer is None else motor_mixer, dtype=float
        )
        if self.motor_mixer.shape != (3, 4):
            raise ValueError("motor_mixer must be 3x4")

    def expected_accelerations(
            self,
            attitude_rpy: Sequence[float],
            motor_pwm: Sequence[float] | None,
            battery_voltage: float | None,
    ) -> tuple[np.ndarray, np.ndarray]:
        roll, pitch, yaw = _as_vector(attitude_rpy, 3, "attitude_rpy")
        direction = body_z_world(
            self.roll_sign_for_world_thrust * roll,
            self.pitch_sign_for_world_thrust * pitch,
            yaw,
        )

        valid_motors = motor_pwm is not None and len(motor_pwm) == 4 and max(motor_pwm) > 0
        if valid_motors:
            voltage = float(battery_voltage or self.hover_voltage)
            normalized = np.asarray(motor_pwm, dtype=float) / self.hover_pwm
            normalized *= voltage / self.hover_voltage
            motor_thrust = np.square(np.clip(normalized, 0.0, None))
            thrust_accel = GRAVITY * float(np.mean(motor_thrust))
            angular_accel = self.angular_accel_scale * (self.motor_mixer @ motor_thrust)
        else:
            # Safe fallback for offline/droneless use: assume altitude-hold
            # collective thrust. It predicts XY actuation but cannot identify a
            # static vertical force without real motor data.
            thrust_accel = GRAVITY / max(direction[2], 0.35)
            angular_accel = np.zeros(3)

        linear_accel = thrust_accel * direction - np.array([0.0, 0.0, GRAVITY])
        return linear_accel, angular_accel


class KinematicDisturbanceObserver:
    """Augmented Kalman observer for position, velocity, and acceleration bias."""

    def __init__(
            self,
            dimensions: int,
            measurement_std: Sequence[float] | float,
            disturbance_process_std: Sequence[float] | float,
            velocity_process_std: Sequence[float] | float = 0.10,
            initial_velocity_std: float = 1.0,
            initial_disturbance_std: float = 2.0,
            max_measurement_nis: float = 1e4,
    ):
        self.dimensions = dimensions
        self.measurement_std = _as_vector(measurement_std, dimensions, "measurement_std")
        self.disturbance_process_std = _as_vector(
            disturbance_process_std, dimensions, "disturbance_process_std"
        )
        self.velocity_process_std = _as_vector(velocity_process_std, dimensions, "velocity_process_std")
        self.initial_velocity_std = float(initial_velocity_std)
        self.initial_disturbance_std = float(initial_disturbance_std)
        self.max_measurement_nis = float(max_measurement_nis)
        self.state = np.zeros(3 * dimensions)
        self.covariance = np.eye(3 * dimensions)
        self.initialized = False
        self.last_innovation = np.zeros(dimensions)
        self.last_nis = 0.0
        self.measurement_rejected = False

    @property
    def position(self) -> np.ndarray:
        return self.state[:self.dimensions].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.state[self.dimensions:2 * self.dimensions].copy()

    @property
    def disturbance(self) -> np.ndarray:
        return self.state[2 * self.dimensions:].copy()

    @property
    def disturbance_covariance(self) -> np.ndarray:
        return self.covariance[2 * self.dimensions:, 2 * self.dimensions:].copy()

    def update(self, measurement: Sequence[float], expected_acceleration: Sequence[float], dt: float) -> None:
        measurement = _as_vector(measurement, self.dimensions, "measurement")
        expected_acceleration = _as_vector(
            expected_acceleration, self.dimensions, "expected_acceleration"
        )
        dt = min(max(float(dt), 1e-4), 0.1)
        n = self.dimensions

        if not self.initialized:
            self.state[:n] = measurement
            self.covariance = np.diag(np.concatenate([
                np.square(self.measurement_std),
                np.full(n, self.initial_velocity_std ** 2),
                np.full(n, self.initial_disturbance_std ** 2),
            ]))
            self.initialized = True
            return

        identity = np.eye(n)
        transition = np.block([
            [identity, dt * identity, 0.5 * dt * dt * identity],
            [np.zeros((n, n)), identity, dt * identity],
            [np.zeros((n, n)), np.zeros((n, n)), identity],
        ])
        control = np.vstack([0.5 * dt * dt * identity, dt * identity, np.zeros((n, n))])
        process_noise = np.diag(np.concatenate([
            np.square(0.5 * dt * dt * self.velocity_process_std),
            np.square(dt * self.velocity_process_std),
            np.square(math.sqrt(dt) * self.disturbance_process_std),
        ]))

        # NumPy 2.2 on the Raspberry Pi/macOS Accelerate builds can emit
        # spurious floating-point warnings from BLAS matmul even when the
        # resulting finite matrices are well conditioned. Validate the result
        # explicitly so a real numerical failure still stops the flight loop.
        with np.errstate(divide='ignore', over='ignore', invalid='ignore'):
            predicted_state = transition @ self.state + control @ expected_acceleration
            predicted_covariance = transition @ self.covariance @ transition.T + process_noise
        if not np.all(np.isfinite(predicted_state)) or not np.all(np.isfinite(predicted_covariance)):
            raise FloatingPointError('disturbance observer prediction became non-finite')
        observation = np.hstack([identity, np.zeros((n, 2 * n))])
        measurement_noise = np.diag(np.square(self.measurement_std))
        innovation = measurement - observation @ predicted_state
        innovation_covariance = observation @ predicted_covariance @ observation.T + measurement_noise
        innovation_covariance_inv = np.linalg.pinv(innovation_covariance)
        nis = float(innovation.T @ innovation_covariance_inv @ innovation)

        self.last_innovation = innovation
        self.last_nis = nis
        self.measurement_rejected = nis > self.max_measurement_nis
        if self.measurement_rejected:
            self.state = predicted_state
            self.covariance = predicted_covariance
            return

        gain = predicted_covariance @ observation.T @ innovation_covariance_inv
        self.state = predicted_state + gain @ innovation
        # Joseph form preserves positive semi-definiteness better in long runs.
        residual = np.eye(3 * n) - gain @ observation
        with np.errstate(divide='ignore', over='ignore', invalid='ignore'):
            updated_covariance = (
                residual @ predicted_covariance @ residual.T
                + gain @ measurement_noise @ gain.T
            )
        if not np.all(np.isfinite(updated_covariance)):
            raise FloatingPointError('disturbance observer covariance became non-finite')
        self.covariance = 0.5 * (updated_covariance + updated_covariance.T)


@dataclass(frozen=True)
class WrenchEstimate:
    timestamp: float
    position: np.ndarray
    velocity: np.ndarray
    orientation_rpy: np.ndarray
    angular_velocity: np.ndarray
    external_force: np.ndarray
    external_torque: np.ndarray
    force_covariance: np.ndarray
    torque_covariance: np.ndarray
    position_innovation: np.ndarray
    orientation_innovation: np.ndarray
    position_nis: float
    orientation_nis: float
    measurement_rejected: bool


class ExternalWrenchObserver:
    """Estimate XYZ external force and roll/pitch/yaw external torque."""

    def __init__(
            self,
            mass: float,
            inertia: Sequence[float],
            position_measurement_std: Sequence[float] = (0.004, 0.004, 0.006),
            orientation_measurement_std: Sequence[float] = (0.015, 0.015, 0.02),
            linear_disturbance_process_std: Sequence[float] = (1.5, 1.5, 2.0),
            angular_disturbance_process_std: Sequence[float] = (4.0, 4.0, 3.0),
            max_measurement_nis: float = 1e4,
    ):
        self.mass = float(mass)
        self.inertia = _as_vector(inertia, 3, "inertia")
        if self.mass <= 0 or np.any(self.inertia <= 0):
            raise ValueError("mass and inertia must be positive")
        self.linear = KinematicDisturbanceObserver(
            3, position_measurement_std, linear_disturbance_process_std,
            max_measurement_nis=max_measurement_nis,
        )
        self.angular = KinematicDisturbanceObserver(
            3, orientation_measurement_std, angular_disturbance_process_std,
            velocity_process_std=0.5,
            initial_velocity_std=2.0,
            initial_disturbance_std=8.0,
            max_measurement_nis=max_measurement_nis,
        )
        self.last_timestamp: float | None = None
        self.last_unwrapped_orientation: np.ndarray | None = None

    def update(
            self,
            position: Sequence[float],
            quaternion: Sequence[float],
            expected_linear_acceleration: Sequence[float],
            expected_angular_acceleration: Sequence[float],
            timestamp: float,
    ) -> WrenchEstimate:
        timestamp = float(timestamp)
        orientation = unwrap_angles(
            self.last_unwrapped_orientation, quaternion_to_euler(quaternion)
        )
        if self.last_timestamp is None:
            dt = 0.01
        else:
            dt = timestamp - self.last_timestamp
            if dt <= 0:
                dt = 1e-4
        self.last_timestamp = timestamp
        self.last_unwrapped_orientation = orientation

        self.linear.update(position, expected_linear_acceleration, dt)
        self.angular.update(orientation, expected_angular_acceleration, dt)
        inertia_matrix = np.diag(self.inertia)
        # At the small roll/pitch angles used by position control, Euler rates
        # closely approximate body rates. Retain the rigid-body gyroscopic term
        # so deliberate rotation is less likely to appear as external torque.
        gyroscopic_torque = np.cross(
            self.angular.velocity, self.inertia * self.angular.velocity
        )
        return WrenchEstimate(
            timestamp=timestamp,
            position=self.linear.position,
            velocity=self.linear.velocity,
            orientation_rpy=self.angular.position,
            angular_velocity=self.angular.velocity,
            external_force=self.mass * self.linear.disturbance,
            external_torque=(
                self.inertia * self.angular.disturbance + gyroscopic_torque
            ),
            force_covariance=(self.mass ** 2) * self.linear.disturbance_covariance,
            torque_covariance=(
                inertia_matrix @ self.angular.disturbance_covariance @ inertia_matrix
            ),
            position_innovation=self.linear.last_innovation.copy(),
            orientation_innovation=self.angular.last_innovation.copy(),
            position_nis=self.linear.last_nis,
            orientation_nis=self.angular.last_nis,
            measurement_rejected=self.linear.measurement_rejected or self.angular.measurement_rejected,
        )
