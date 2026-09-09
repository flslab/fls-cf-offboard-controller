"""Onboard-state external-wrench estimation for interaction flight.

This path deliberately does not filter Vicon position or differentiate the
Crazyflie's already-estimated velocity.  It compares measured linear/angular
momentum with the momentum predicted by the motor model.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
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

DEFAULT_IMPULSE_ESTIMATOR_CONFIG = {
    "window_s": 0.08,
    "minimum_window_s": 0.05,
    "max_dt_s": 0.05,
    # stateEstimate.v* is a filtered state whose physical response can lag the
    # motor/attitude packets even when their packet timestamps are close.  The
    # delay is kept at zero by default and identified per vehicle from a
    # contact-free excitation flight.
    "model_delay_s": 0.0,
    "model_time_constant_s": 0.0,
    "model_acceleration_scale": 1.0,
}


def _merge_momentum_config(overrides: dict | None) -> dict:
    result = dict(DEFAULT_MOMENTUM_OBSERVER_CONFIG)
    result.update(overrides or {})
    return result


def _merge_impulse_config(overrides: dict | None) -> dict:
    result = dict(DEFAULT_IMPULSE_ESTIMATOR_CONFIG)
    result.update(overrides or {})
    return result


@dataclass(frozen=True)
class ImpulseForceEstimate:
    external_force: np.ndarray
    external_impulse: np.ndarray
    predicted_velocity: np.ndarray
    window_s: float
    ready: bool
    rejected: bool


class CausalAccelerationAligner:
    """Align actuator-model acceleration with the onboard velocity estimate.

    ``model_delay_s`` represents the aggregate motor/attitude-to-velocity
    latency observed in flight.  The optional first-order state represents
    actuator and estimator response dynamics; it is applied to acceleration,
    never to the measured velocity used for contact detection.
    """

    def __init__(
            self,
            model_delay_s: Sequence[float] | float = 0.0,
            model_time_constant_s: Sequence[float] | float = 0.0,
            max_dt_s: float = 0.05,
    ):
        self.model_delay_s = _as_vector(model_delay_s, 3, "model_delay_s")
        self.model_time_constant_s = _as_vector(
            model_time_constant_s, 3, "model_time_constant_s"
        )
        self.max_dt_s = float(max_dt_s)
        if (
            np.any(self.model_delay_s < 0.0)
            or np.any(self.model_time_constant_s < 0.0)
        ):
            raise ValueError("model delay and time constant cannot be negative")
        if self.max_dt_s <= 0.0:
            raise ValueError("max_dt_s must be positive")
        self._history = deque()
        self._last_timestamp: float | None = None
        self._filtered_acceleration: np.ndarray | None = None

    def reset(self) -> None:
        self._history.clear()
        self._last_timestamp = None
        self._filtered_acceleration = None

    def update(
            self,
            acceleration: Sequence[float],
            timestamp: float,
    ) -> tuple[np.ndarray, bool]:
        acceleration = _as_vector(acceleration, 3, "acceleration")
        timestamp = float(timestamp)
        if self._last_timestamp is not None:
            dt = timestamp - self._last_timestamp
            if dt <= 0.0 or dt > self.max_dt_s:
                self.reset()
        dt = None if self._last_timestamp is None else timestamp - self._last_timestamp
        self._last_timestamp = timestamp

        if self._filtered_acceleration is None:
            self._filtered_acceleration = acceleration.copy()
        else:
            alpha = np.ones(3)
            dynamic_axes = self.model_time_constant_s > 0.0
            alpha[dynamic_axes] = 1.0 - np.exp(
                -dt / self.model_time_constant_s[dynamic_axes]
            )
            self._filtered_acceleration += alpha * (
                acceleration - self._filtered_acceleration
            )
        self._history.append((timestamp, self._filtered_acceleration.copy()))

        target_times = timestamp - self.model_delay_s
        earliest_target = float(np.min(target_times))
        while len(self._history) >= 3 and self._history[1][0] <= earliest_target:
            self._history.popleft()
        if self._history[0][0] > earliest_target:
            return self._filtered_acceleration.copy(), False

        history = list(self._history)
        aligned = np.zeros(3)
        for axis, target_time in enumerate(target_times):
            if target_time >= history[-1][0]:
                aligned[axis] = history[-1][1][axis]
                continue
            after_index = next(
                index for index, sample in enumerate(history)
                if sample[0] >= target_time
            )
            if history[after_index][0] == target_time:
                aligned[axis] = history[after_index][1][axis]
                continue
            before, after = history[after_index - 1], history[after_index]
            span = after[0] - before[0]
            if span <= 0.0 or span > self.max_dt_s:
                self.reset()
                return acceleration.copy(), False
            fraction = (target_time - before[0]) / span
            aligned[axis] = before[1][axis] + fraction * (
                after[1][axis] - before[1][axis]
            )
        return aligned, True


class FiniteWindowMomentumForceEstimator:
    """Estimate force from momentum balance under the no-contact hypothesis.

    The momentum at the beginning of the window carries all existing inertia.
    Only modeled motor/gravity force is integrated through the window; a prior
    external-force estimate is deliberately not propagated into the prediction.
    """

    def __init__(
            self,
            mass: float,
            window_s: float = 0.08,
            minimum_window_s: float = 0.05,
            max_dt_s: float = 0.05,
            model_delay_s: Sequence[float] | float = 0.0,
            model_time_constant_s: Sequence[float] | float = 0.0,
            model_acceleration_scale: Sequence[float] | float = 1.0,
    ):
        self.mass = float(mass)
        self.window_s = float(window_s)
        self.minimum_window_s = float(minimum_window_s)
        self.max_dt_s = float(max_dt_s)
        self.model_acceleration_scale = _as_vector(
            model_acceleration_scale, 3, "model_acceleration_scale"
        )
        if (
            self.mass <= 0.0
            or self.minimum_window_s <= 0.0
            or self.window_s < self.minimum_window_s
            or self.max_dt_s <= 0.0
            or np.any(self.model_acceleration_scale <= 0.0)
        ):
            raise ValueError(
                "impulse estimator mass, window, minimum window, and max dt "
                "must be positive; window must cover the minimum window"
            )
        self._samples = deque()
        self._last_timestamp: float | None = None
        self.aligner = CausalAccelerationAligner(
            model_delay_s=model_delay_s,
            model_time_constant_s=model_time_constant_s,
            max_dt_s=max_dt_s,
        )
        self.last_aligned_acceleration = np.zeros(3)
        self.last_alignment_ready = False

    def _result(
            self,
            impulse: np.ndarray,
            predicted_velocity: np.ndarray,
            duration: float,
            ready: bool,
            rejected: bool,
    ) -> ImpulseForceEstimate:
        force = impulse / duration if ready else np.zeros(3)
        if not all(np.all(np.isfinite(value)) for value in (
            force, impulse, predicted_velocity,
        )):
            raise FloatingPointError("finite-window momentum estimate became non-finite")
        return ImpulseForceEstimate(
            external_force=force.copy(),
            external_impulse=impulse.copy(),
            predicted_velocity=predicted_velocity.copy(),
            window_s=float(duration),
            ready=bool(ready),
            rejected=bool(rejected),
        )

    def update(
            self,
            velocity: Sequence[float],
            expected_linear_acceleration: Sequence[float],
            timestamp: float,
    ) -> ImpulseForceEstimate:
        velocity = _as_vector(velocity, 3, "velocity")
        expected_linear_acceleration = _as_vector(
            expected_linear_acceleration, 3, "expected_linear_acceleration"
        )
        expected_linear_acceleration = (
            expected_linear_acceleration * self.model_acceleration_scale
        )
        timestamp = float(timestamp)
        aligned_acceleration, alignment_ready = self.aligner.update(
            expected_linear_acceleration, timestamp
        )
        self.last_aligned_acceleration = aligned_acceleration.copy()
        self.last_alignment_ready = alignment_ready
        momentum = self.mass * velocity
        model_force = self.mass * aligned_acceleration

        discontinuity = False
        if self._last_timestamp is not None:
            dt = timestamp - self._last_timestamp
            discontinuity = dt <= 0.0 or dt > self.max_dt_s
        self._last_timestamp = timestamp
        if discontinuity:
            self._samples.clear()

        if not alignment_ready:
            self._samples.clear()
            return self._result(
                np.zeros(3), velocity, 0.0, False, True,
            )

        self._samples.append((timestamp, momentum.copy(), model_force.copy()))
        while (
            len(self._samples) >= 3
            and timestamp - self._samples[1][0] >= self.window_s
        ):
            self._samples.popleft()

        if len(self._samples) < 2:
            return self._result(
                np.zeros(3), velocity, 0.0, False, True,
            )

        duration = timestamp - self._samples[0][0]
        modeled_impulse = np.zeros(3)
        samples = list(self._samples)
        for previous, current in zip(samples, samples[1:]):
            interval = current[0] - previous[0]
            if interval <= 0.0 or interval > self.max_dt_s:
                self._samples.clear()
                self._samples.append((
                    timestamp, momentum.copy(), model_force.copy()
                ))
                return self._result(
                    np.zeros(3), velocity, 0.0, False, True,
                )
            modeled_impulse += previous[2] * interval

        predicted_momentum = self._samples[0][1] + modeled_impulse
        external_impulse = momentum - predicted_momentum
        predicted_velocity = predicted_momentum / self.mass
        ready = duration >= self.minimum_window_s and not discontinuity
        return self._result(
            external_impulse,
            predicted_velocity,
            duration,
            ready,
            not ready,
        )


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
        self.last_measured_linear_momentum: np.ndarray | None = None
        self.last_measured_angular_momentum: np.ndarray | None = None
        self.last_model_force: np.ndarray | None = None
        self.last_model_torque: np.ndarray | None = None
        self.last_gyroscopic_torque: np.ndarray | None = None
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
        model_force = self.mass * expected_linear_acceleration
        model_torque = self.inertia * expected_angular_acceleration
        gyroscopic_torque = np.cross(
            angular_velocity, measured_angular_momentum
        )
        if self.last_timestamp is None:
            self.last_timestamp = timestamp
            self.predicted_linear_momentum = measured_linear_momentum.copy()
            self.predicted_angular_momentum = measured_angular_momentum.copy()
            self.last_measured_linear_momentum = measured_linear_momentum.copy()
            self.last_measured_angular_momentum = measured_angular_momentum.copy()
            self.last_model_force = model_force.copy()
            self.last_model_torque = model_torque.copy()
            self.last_gyroscopic_torque = gyroscopic_torque.copy()
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
            self.last_measured_linear_momentum = measured_linear_momentum.copy()
            self.last_measured_angular_momentum = measured_angular_momentum.copy()
            self.last_model_force = model_force.copy()
            self.last_model_torque = model_torque.copy()
            self.last_gyroscopic_torque = gyroscopic_torque.copy()
            return self._estimate(
                position, velocity, attitude_rpy, angular_velocity, timestamp,
                np.zeros(3), np.zeros(3), True,
            )

        # Causal one-step prediction: the actuator state sampled at k-1 acts
        # over [t[k-1], t[k]].  Anchor each prediction at the previous measured
        # momentum so old model errors cannot accumulate into a false contact.
        self.predicted_linear_momentum = (
            self.last_measured_linear_momentum + dt * (
                self.last_model_force + self.force_residual
            )
        )
        self.predicted_angular_momentum = (
            self.last_measured_angular_momentum + dt * (
                self.last_model_torque
                - self.last_gyroscopic_torque
                + self.torque_residual
            )
        )

        linear_error = measured_linear_momentum - self.predicted_linear_momentum
        angular_error = measured_angular_momentum - self.predicted_angular_momentum
        # Innovation feedback estimates the unmodelled wrench.  This is the
        # disturbance-observer state update, not a second filter on velocity.
        self.force_residual += self.linear_gain * linear_error
        self.torque_residual += self.angular_gain * angular_error

        self.last_measured_linear_momentum = measured_linear_momentum.copy()
        self.last_measured_angular_momentum = measured_angular_momentum.copy()
        self.last_model_force = model_force.copy()
        self.last_model_torque = model_torque.copy()
        self.last_gyroscopic_torque = gyroscopic_torque.copy()

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
        impulse_config = _merge_impulse_config(
            self.config.get("impulse_estimator")
        )
        self.config["impulse_estimator"] = impulse_config
        self.impulse_estimator = FiniteWindowMomentumForceEstimator(
            mass=self.config["mass"],
            **impulse_config,
        )
        self.last_recursive_external_force = np.zeros(3)
        self.last_external_impulse = np.zeros(3)
        self.last_impulse_window_s = 0.0
        self.last_no_contact_predicted_velocity = np.zeros(3)
        self.last_impulse_ready = False
        self.last_aligned_expected_linear_acceleration = np.zeros(3)
        self.last_model_alignment_ready = False

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
        recursive_raw = self.observer.update(
            position=position,
            velocity=velocity,
            attitude_rpy=attitude_rpy,
            angular_velocity=angular_velocity,
            expected_linear_acceleration=expected_linear,
            expected_angular_acceleration=expected_angular,
            timestamp=timestamp,
        )
        impulse = self.impulse_estimator.update(
            velocity=velocity,
            expected_linear_acceleration=expected_linear,
            timestamp=timestamp,
        )
        self.last_aligned_expected_linear_acceleration = (
            self.impulse_estimator.last_aligned_acceleration.copy()
        )
        self.last_model_alignment_ready = (
            self.impulse_estimator.last_alignment_ready
        )
        self.last_recursive_external_force = recursive_raw.external_force.copy()
        self.last_external_impulse = impulse.external_impulse.copy()
        self.last_impulse_window_s = impulse.window_s
        self.last_no_contact_predicted_velocity = impulse.predicted_velocity.copy()
        self.last_impulse_ready = impulse.ready
        # XYZ contact detection uses the finite-window no-contact innovation.
        # Retain recursive angular momentum for the dormant yaw channel.
        raw = replace(
            recursive_raw,
            external_force=impulse.external_force,
            position_innovation=impulse.external_impulse,
            measurement_rejected=(
                recursive_raw.measurement_rejected or impulse.rejected
            ),
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
