"""Onboard-state external-wrench estimation for interaction flight.

This path deliberately does not filter Vicon position or differentiate the
Crazyflie's already-estimated velocity.  It compares measured linear/angular
momentum with the momentum predicted by the motor model.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Sequence

import numpy as np

from Interaction.external_wrench_observer import WrenchEstimate, _as_vector
from Interaction.wrench_interaction_pipeline import PipelineOutput, WrenchInteractionPipeline


DEFAULT_MOMENTUM_OBSERVER_CONFIG = {
    # Observer bandwidths in 1/s.  These tune residual convergence, not a
    # low-pass filter applied to the onboard velocity estimate.
    "linear_gain": [8.0, 8.0, 8.0],
    "angular_gain": [12.0, 12.0, 12.0],
    "force_measurement_std": [0.020, 0.020, 0.030],
    "torque_measurement_std": [0.00020, 0.00020, 0.00025],
    "max_dt_s": 0.05,
}


def _merge_momentum_config(overrides: dict | None) -> dict:
    result = dict(DEFAULT_MOMENTUM_OBSERVER_CONFIG)
    result.update(overrides or {})
    return result


class OnboardMomentumWrenchObserver:
    """Estimate external force/torque from onboard momentum residuals.

    Linear momentum is ``m * velocity`` and angular momentum is ``I * omega``.
    The observer integrates the actuator model and feeds the momentum error
    back as an external-wrench residual.  It consumes the Crazyflie's onboard
    estimator output directly and therefore adds no second position/velocity
    Kalman filter.
    """

    def __init__(
            self,
            mass: float,
            inertia: Sequence[float],
            linear_gain: Sequence[float] = (8.0, 8.0, 8.0),
            angular_gain: Sequence[float] = (12.0, 12.0, 12.0),
            force_measurement_std: Sequence[float] = (0.020, 0.020, 0.030),
            torque_measurement_std: Sequence[float] = (0.00020, 0.00020, 0.00025),
            max_dt_s: float = 0.05,
    ):
        self.mass = float(mass)
        self.inertia = _as_vector(inertia, 3, "inertia")
        self.linear_gain = _as_vector(linear_gain, 3, "linear_gain")
        self.angular_gain = _as_vector(angular_gain, 3, "angular_gain")
        self.force_measurement_std = _as_vector(
            force_measurement_std, 3, "force_measurement_std"
        )
        self.torque_measurement_std = _as_vector(
            torque_measurement_std, 3, "torque_measurement_std"
        )
        self.max_dt_s = float(max_dt_s)
        if self.mass <= 0 or np.any(self.inertia <= 0):
            raise ValueError("mass and inertia must be positive")
        if np.any(self.linear_gain <= 0) or np.any(self.angular_gain <= 0):
            raise ValueError("momentum observer gains must be positive")
        if self.max_dt_s <= 0:
            raise ValueError("max_dt_s must be positive")

        self.last_timestamp: float | None = None
        self.predicted_linear_momentum = np.zeros(3)
        self.predicted_angular_momentum = np.zeros(3)
        self.force_residual = np.zeros(3)
        self.torque_residual = np.zeros(3)

    def _estimate(
            self,
            position,
            velocity,
            attitude_rpy,
            angular_velocity,
            timestamp,
            linear_error,
            angular_error,
            rejected,
    ) -> WrenchEstimate:
        force_covariance = np.diag(np.square(self.force_measurement_std))
        torque_covariance = np.diag(np.square(self.torque_measurement_std))
        return WrenchEstimate(
            timestamp=float(timestamp),
            position=np.asarray(position, dtype=float).copy(),
            velocity=np.asarray(velocity, dtype=float).copy(),
            orientation_rpy=np.asarray(attitude_rpy, dtype=float).copy(),
            angular_velocity=np.asarray(angular_velocity, dtype=float).copy(),
            external_force=self.force_residual.copy(),
            external_torque=self.torque_residual.copy(),
            force_covariance=force_covariance,
            torque_covariance=torque_covariance,
            # Preserve the common estimate interface.  In this observer these
            # fields contain momentum errors rather than pose innovations.
            position_innovation=np.asarray(linear_error, dtype=float).copy(),
            orientation_innovation=np.asarray(angular_error, dtype=float).copy(),
            position_nis=0.0,
            orientation_nis=0.0,
            measurement_rejected=bool(rejected),
        )

    def update(
            self,
            position: Sequence[float],
            velocity: Sequence[float],
            attitude_rpy: Sequence[float],
            angular_velocity: Sequence[float],
            expected_linear_acceleration: Sequence[float],
            expected_angular_acceleration: Sequence[float],
            timestamp: float,
    ) -> WrenchEstimate:
        position = _as_vector(position, 3, "position")
        velocity = _as_vector(velocity, 3, "velocity")
        attitude_rpy = _as_vector(attitude_rpy, 3, "attitude_rpy")
        angular_velocity = _as_vector(angular_velocity, 3, "angular_velocity")
        expected_linear_acceleration = _as_vector(
            expected_linear_acceleration, 3, "expected_linear_acceleration"
        )
        expected_angular_acceleration = _as_vector(
            expected_angular_acceleration, 3, "expected_angular_acceleration"
        )
        timestamp = float(timestamp)

        measured_linear_momentum = self.mass * velocity
        measured_angular_momentum = self.inertia * angular_velocity
        if self.last_timestamp is None:
            self.last_timestamp = timestamp
            self.predicted_linear_momentum = measured_linear_momentum.copy()
            self.predicted_angular_momentum = measured_angular_momentum.copy()
            return self._estimate(
                position, velocity, attitude_rpy, angular_velocity, timestamp,
                np.zeros(3), np.zeros(3), False,
            )

        dt = timestamp - self.last_timestamp
        self.last_timestamp = timestamp
        if dt <= 0.0 or dt > self.max_dt_s:
            # A duplicate or stale packet must not create an impulse residual.
            self.predicted_linear_momentum = measured_linear_momentum.copy()
            self.predicted_angular_momentum = measured_angular_momentum.copy()
            self.force_residual.fill(0.0)
            self.torque_residual.fill(0.0)
            return self._estimate(
                position, velocity, attitude_rpy, angular_velocity, timestamp,
                np.zeros(3), np.zeros(3), True,
            )

        model_force = self.mass * expected_linear_acceleration
        model_torque = self.inertia * expected_angular_acceleration
        gyroscopic_torque = np.cross(
            angular_velocity, measured_angular_momentum
        )
        self.predicted_linear_momentum += dt * (
            model_force + self.force_residual
        )
        self.predicted_angular_momentum += dt * (
            model_torque - gyroscopic_torque + self.torque_residual
        )

        linear_error = measured_linear_momentum - self.predicted_linear_momentum
        angular_error = measured_angular_momentum - self.predicted_angular_momentum
        self.force_residual = self.linear_gain * linear_error
        self.torque_residual = self.angular_gain * angular_error

        finite = all(np.all(np.isfinite(value)) for value in (
            linear_error,
            angular_error,
            self.force_residual,
            self.torque_residual,
        ))
        if not finite:
            raise FloatingPointError("momentum observer became non-finite")
        return self._estimate(
            position, velocity, attitude_rpy, angular_velocity, timestamp,
            linear_error, angular_error, False,
        )


class OnboardMomentumWrenchPipeline(WrenchInteractionPipeline):
    """Independent wrench pipeline driven by Crazyflie onboard estimates."""

    def __init__(self, config: dict | None = None):
        # Reuse calibration, contact, admittance, and safety behavior without
        # changing the existing mocap WrenchInteractionPipeline.
        super().__init__(config)
        observer_config = _merge_momentum_config(
            self.config.get("momentum_observer")
        )
        self.observer = OnboardMomentumWrenchObserver(
            mass=self.config["mass"],
            inertia=self.config["inertia"],
            **observer_config,
        )

    def update(
            self,
            position: Sequence[float],
            velocity: Sequence[float],
            attitude_rpy: Sequence[float],
            angular_velocity: Sequence[float],
            motor_pwm: Sequence[float] | None,
            battery_voltage: float | None,
            timestamp: float,
            yaw_control_command: float | None = None,
    ) -> PipelineOutput:
        timestamp = float(timestamp)
        if self._start_timestamp is None:
            self._start_timestamp = timestamp
        dt = 0.01 if self._last_timestamp is None else max(
            timestamp - self._last_timestamp, 1e-4
        )
        self._last_timestamp = timestamp

        attitude_rpy = _as_vector(attitude_rpy, 3, "attitude_rpy")
        has_motor_data = self.motor_data_available(motor_pwm)
        expected_linear, expected_angular = self.motor_model.expected_accelerations(
            attitude_rpy,
            motor_pwm if has_motor_data else None,
            battery_voltage,
        )
        yaw_model = self.yaw_command_model
        if yaw_model["enabled"]:
            if not isinstance(yaw_control_command, (int, float)) or not np.isfinite(
                    yaw_control_command
            ):
                raise ValueError(
                    "enabled yaw_command_model requires controller.cmd_yaw"
                )
            expected_angular[2] = (
                float(yaw_model["accel_per_command"]) * float(yaw_control_command)
                - float(yaw_model["damping_per_s"]) * float(angular_velocity[2])
                + float(yaw_model["bias_rad_s2"])
            )
        raw = self.observer.update(
            position=position,
            velocity=velocity,
            attitude_rpy=attitude_rpy,
            angular_velocity=angular_velocity,
            expected_linear_acceleration=expected_linear,
            expected_angular_acceleration=expected_angular,
            timestamp=timestamp,
        )

        if not self.calibrated:
            self._update_bias(raw)
        estimate = replace(
            raw,
            external_force=raw.external_force - self.force_bias,
            external_torque=raw.external_torque - self.torque_bias,
        )

        contacts = None
        force_input = np.zeros(3)
        yaw_input = 0.0
        if self.calibrated:
            contacts = self.detector.update(estimate)
            if not estimate.measurement_rejected:
                force_input, yaw_input = self._response_inputs(estimate, contacts)
        admittance = self.admittance.step(force_input, yaw_input, dt)
        return PipelineOutput(
            raw_estimate=raw,
            estimate=estimate,
            contacts=contacts,
            admittance=admittance,
            expected_linear_acceleration=expected_linear.copy(),
            expected_angular_acceleration=expected_angular.copy(),
            motor_data_available=has_motor_data,
            calibrated=self.calibrated,
            calibration_samples=self.calibration_samples,
        )
