"""Composition of wrench estimation, contact detection, and admittance."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Sequence

import numpy as np

from Interaction.admittance_controller import AdmittanceController3DYaw, AdmittanceState
from Interaction.external_wrench_observer import (
    ExternalWrenchObserver,
    MotorWrenchModel,
    WrenchEstimate,
    quaternion_to_euler,
)
from Interaction.wrench_contact_detector import WrenchContactDetector, WrenchContactState


DEFAULT_WRENCH_INTERACTION_CONFIG = {
    # The motor/torque model and thresholds must be identified from flight data
    # before this is disabled. Shadow mode estimates and logs, but never moves
    # the position/yaw reference in response to a detected contact.
    "shadow_mode": True,
    "mass": 0.17,
    "inertia": [0.002, 0.002, 0.003],
    # Normal interaction runs can skip the per-flight zero-bias collection.
    # Dedicated model calibration still forces this on before excitation.
    "startup_bias_calibration_enabled": True,
    "observer_settle_s": 1.0,
    "bias_calibration_s": 1.0,
    "bias_calibration_timeout_s": 12.0,
    "calibration_max_speed_m_s": 0.06,
    "calibration_max_angular_rate_rad_s": 0.25,
    "minimum_bias_samples": 30,
    # Keep the first contact detector disarmed until the vehicle has actually
    # settled after the initial go-to.  This prevents residual approach
    # velocity/model transients from being interpreted as a user touch.
    "initial_contact_arming": {
        "enabled": True,
        "max_xy_speed_m_s": 0.03,
        "stationary_dwell_s": 0.50,
    },
    "calibration_excitation": {
        "enabled": False,
        "start_delay_s": 1.0,
        "duration_s": 30.0,
        "translation_amplitude_m": [0.12, 0.12, 0.07],
        "translation_frequency_hz": [0.35, 0.35, 0.25],
        "translation_profile": "sequential_chirp",
        "translation_chirp_end_hz": [1.80, 1.80, 1.10],
        "translation_axis_rest_s": 1.0,
        "translation_ramp_s": 0.6,
        "yaw_amplitude_deg": 10.0,
        "yaw_frequency_hz": 0.23,
    },
    # Second stage of --calibrate.  Each trial commands a small planar tilt,
    # level attitude, the opposite tilt, and level attitude again.  The
    # resulting command-to-velocity response is fitted separately from the
    # wrench observer model and saved in the same per-drone calibration file.
    "planar_braking_calibration": {
        "enabled": False,
        "start_delay_s": 1.0,
        # The current force sensor and interaction are mounted on world Y.
        # Repeat both signs to estimate that axis without pretending a pooled
        # scalar is a valid roll/pitch anisotropy model.
        "directions_xy": [[0.0, 1.0], [0.0, -1.0]],
        "repetitions": 3,
        "tilt_deg": 8.0,
        "level_before_acceleration_s": 0.20,
        "accelerate_s": 0.35,
        "level_before_brake_s": 0.20,
        "brake_s": 0.45,
        "level_after_brake_s": 0.60,
        "recovery_s": 1.50,
        "max_xy_speed_m_s": 0.70,
        "max_displacement_m": 0.45,
        "trial_start_max_xy_speed_m_s": 0.05,
        "trial_start_max_tilt_deg": 4.0,
        "fit_window_s": 0.08,
        "max_fit_delay_s": 0.25,
        "max_fit_time_constant_s": 0.25,
        "minimum_fit_r_squared": 0.70,
        "minimum_validation_r_squared": 0.50,
        "minimum_acceleration_scale": 0.20,
        "maximum_acceleration_scale": 2.50,
        "minimum_trials_per_direction": 3,
        "minimum_windows_per_trial": 12,
        "minimum_direction_r_squared": 0.70,
        "minimum_direction_validation_r_squared": 0.50,
        "maximum_direction_nrmse": 0.20,
        "maximum_direction_gain_ratio": 1.25,
        "maximum_repeat_gain_deviation": 0.20,
        # Runtime acceleration is capped to this multiple of the calibrated
        # 8-degree step response; the configured 5 m/s^2 remains an upper bound.
        "maximum_acceleration_extrapolation_ratio": 1.25,
    },
    "observer": {
        "position_measurement_std": [0.004, 0.004, 0.006],
        "orientation_measurement_std": [0.015, 0.015, 0.020],
        "linear_disturbance_process_std": [1.5, 1.5, 2.0],
        "angular_disturbance_process_std": [4.0, 4.0, 3.0],
        "max_measurement_nis": 1.0e4,
    },
    "motor_model": {
        # Initial collective calibration from the 2026-07-24 lb11 hover log.
        "hover_pwm": 31900.0,
        "hover_voltage": 7.8,
        # Keep the generic mathematical convention by default. Set either sign
        # to -1 when flight identification shows that the corresponding
        # Crazyflie stateEstimate attitude/thrust convention is reversed.
        "roll_sign_for_world_thrust": 1.0,
        "pitch_sign_for_world_thrust": 1.0,
        # Must be identified for reliable rotation detection while the vehicle
        # is deliberately rotating. Zero is suitable only for shadow trials.
        "angular_accel_scale": [0.0, 0.0, 0.0],
        # Preferred onboard yaw model driven by the PID actuator output. It is
        # deliberately disabled until a contact-free chirp fit generalizes.
        "yaw_command_model": {
            "enabled": False,
            "accel_per_command": 0.0,
            "damping_per_s": 0.0,
            "bias_rad_s2": 0.0,
        },
    },
    "detection": {
        "translation": {
            "component_thresholds": [0.08, 0.08, 0.12],
            "covariance_floor": [0.015, 0.015, 0.025],
            "confidence_sigma": 2.5,
            "onset_evidence_s": 0.05,
            "release_time_s": 0.15,
            "release_ratio": 0.55,
            # By default all translation axes participate in contact onset.
            # Missions may select a subset, for example [0, 1] for lateral
            # interaction when persistent vertical residuals are irrelevant.
            "onset_axes": None,
            # After onset, lock the configured motion direction. Release when
            # the signed projected force falls below the positive threshold,
            # including when it becomes negative.
            "release_projection_axes": None,
            "release_direction_min_norm": 0.02,
        },
        "yaw": {
            "enabled": True,
            "component_thresholds": [0.0010],
            "covariance_floor": [0.00015],
            "confidence_sigma": 2.5,
            "onset_evidence_s": 0.05,
            "release_time_s": 0.15,
            "release_ratio": 0.55,
        },
    },
    "control_handoff": {
        # The two legacy fields remain accepted by TranslationControlHandoff so
        # archived mission files still load; attitude braking does not use them.
        "brake_xy_acceleration_m_s2": 0.8,
        "position_brake_offset_m": 0.05,
        "brake_settle_s": 0.30,
        # At Contact End apply velocity-proportional attitude damping along the
        # locked interaction direction, then capture current-position hold near
        # zero projected speed/reversal.
        "brake_xy_speed_m_s": 0.20,
        # Apply the minimum to total tilt, not independently to roll/pitch. It
        # tapers toward zero near the braking completion speed.
        "brake_min_attitude_deg": 3.0,
        "brake_min_attitude_taper_speed_m_s": 0.25,
        # 5 m/s^2 requires about 27 degrees of tilt; retain a 30-degree cap.
        "brake_max_attitude_deg": 30.0,
        "brake_timeout_s": 1.5,
        "brake_velocity_gain_s": 2.0,
        # At release, freeze the zero-force virtual stopping point. Attitude
        # braking uses the remaining distance and a conservative actuator-delay
        # lookahead; position control receives the frozen point unless the
        # vehicle has already passed it.
        "coast_position_gain_s2": 4.0,
        "coast_velocity_gain_s": 2.5,
        "coast_max_acceleration_m_s2": 5.0,
        "coast_attitude_response_delay_s": 0.12,
        "coast_attitude_time_constant_s": 0.08,
        "coast_attitude_acceleration_scale": 1.0,
        "coast_calibrated_direction_xy": None,
        # Level slightly before the model predicts zero terminal velocity.
        # This buffer absorbs fit/discretization error without preventing the
        # 0.04 m/s state-settled handoff below.
        "coast_level_terminal_speed_m_s": 0.03,
        "coast_command_period_s": 0.02,
        "coast_command_acceleration_deadband_m_s2": 0.02,
        # A release candidate is reversible, so its response-tail correction
        # is deliberately gentler than confirmed coasting.
        "coast_candidate_tail_cancellation_max_acceleration_m_s2": 1.0,
        "coast_acceleration_filter_time_constant_s": 0.08,
        # Position control receives ownership only after measured translation,
        # attitude, and acceleration have settled. Longitudinal motion uses the
        # tight stop threshold; bounded transverse drift is handed to position
        # control with its target latched at the measured lateral coordinate.
        # brake_xy_speed_m_s is retained for the legacy observer-brake path.
        "coast_handoff_speed_m_s": 0.04,
        "coast_handoff_max_lateral_speed_m_s": 0.15,
        "coast_handoff_max_tilt_deg": 3.0,
        "coast_handoff_max_acceleration_m_s2": 0.35,
        "coast_alignment_position_tolerance_m": 0.04,
        "coast_alignment_velocity_tolerance_m_s": 0.08,
        "coast_alignment_dwell_s": 0.05,
        # Diagnostic only; timeout cannot force a moving vehicle into position
        # control because doing so would pull it back toward the handoff point.
        "coast_attitude_timeout_s": 1.5,
    },
    "admittance": {
        "translation_mass": [0.30, 0.30, 0.45],
        "translation_damping": [1.20, 1.20, 1.80],
        "translation_stiffness": [0.0, 0.0, 0.0],
        "max_offset": [0.50, 0.50, 0.30],
        "max_velocity": [0.40, 0.40, 0.20],
        "max_acceleration": [1.0, 1.0, 0.6],
        "yaw_inertia": 0.010,
        "yaw_damping": 0.040,
        "yaw_stiffness": 0.0,
        "max_yaw_offset": 0.785,
        "max_yaw_rate": 0.50,
        "max_yaw_acceleration": 1.20,
    },
    "safety": {
        "max_frame_age_s": 0.10,
        # During shadow calibration only, skip transient stale state packets
        # while retaining position hold. Active interaction keeps the strict
        # max_state_age_s cutoff.
        "calibration_state_dropout_timeout_s": 0.25,
        "max_motor_age_s": 0.15,
        "max_motor_pose_skew_s": 0.03,
        "startup_timeout_s": 5.0,
        "require_motor_data": True,
        "require_calibrated_angular_model_when_active": True,
    },
}


def _deep_merge(defaults: dict, overrides: dict | None) -> dict:
    result = {}
    overrides = overrides or {}
    for key, value in defaults.items():
        if isinstance(value, dict):
            result[key] = _deep_merge(value, overrides.get(key))
        else:
            result[key] = overrides.get(key, value)
    for key, value in overrides.items():
        if key not in result:
            result[key] = value
    return result


@dataclass(frozen=True)
class PipelineOutput:
    raw_estimate: WrenchEstimate
    estimate: WrenchEstimate
    contacts: WrenchContactState | None
    admittance: AdmittanceState
    expected_linear_acceleration: np.ndarray
    expected_angular_acceleration: np.ndarray
    motor_data_available: bool
    calibrated: bool
    calibration_samples: int


class WrenchInteractionPipeline:
    """Stateful model-based interaction pipeline.

    Translation force can drive X/Y/Z admittance and yaw torque can drive yaw
    admittance. Roll and pitch remain state inputs but are not contact channels.
    """

    def __init__(self, config: dict | None = None):
        self.config = _deep_merge(DEFAULT_WRENCH_INTERACTION_CONFIG, config)
        self.shadow_mode = bool(self.config["shadow_mode"])

        observer_config = dict(self.config["observer"])
        self.observer = ExternalWrenchObserver(
            mass=self.config["mass"],
            inertia=self.config["inertia"],
            **observer_config,
        )
        motor_model_config = dict(self.config["motor_model"])
        self.yaw_command_model = motor_model_config.pop("yaw_command_model")
        self.motor_model = MotorWrenchModel(**motor_model_config)
        # Select the supported channels explicitly so an older mission file
        # containing the removed roll_pitch block remains launch-compatible.
        self.detector = WrenchContactDetector(
            translation=self.config["detection"]["translation"],
            yaw=self.config["detection"]["yaw"],
        )
        self.admittance = AdmittanceController3DYaw(**self.config["admittance"])

        self._bias_calibration_enabled = bool(
            self.config["startup_bias_calibration_enabled"]
        )
        self._settle_s = float(self.config["observer_settle_s"])
        self._calibration_s = float(self.config["bias_calibration_s"])
        self._calibration_timeout_s = float(self.config["bias_calibration_timeout_s"])
        self._calibration_max_speed = float(self.config["calibration_max_speed_m_s"])
        self._calibration_max_angular_rate = float(
            self.config["calibration_max_angular_rate_rad_s"]
        )
        self._minimum_bias_samples = int(self.config["minimum_bias_samples"])
        if (
            self._settle_s < 0
            or self._calibration_s <= 0
            or self._calibration_timeout_s <= self._calibration_s
            or self._calibration_max_speed <= 0
            or self._calibration_max_angular_rate <= 0
            or self._minimum_bias_samples <= 0
        ):
            raise ValueError(
                "observer_settle_s must be non-negative and bias calibration "
                "duration, timeout, stationarity limits, and sample count must "
                "be positive; timeout must exceed the calibration duration"
            )
        self._start_timestamp: float | None = None
        self._stationary_since: float | None = None
        self._last_timestamp: float | None = None
        self._force_samples: list[np.ndarray] = []
        self._torque_samples: list[np.ndarray] = []
        self.force_bias = np.zeros(3)
        self.torque_bias = np.zeros(3)
        self.calibrated = not self._bias_calibration_enabled

        safety = self.config["safety"]
        excitation = self.config["calibration_excitation"]
        if excitation["enabled"] and not self.shadow_mode:
            raise ValueError("calibration_excitation is allowed only in shadow_mode")
        angular_scale = np.asarray(self.config["motor_model"]["angular_accel_scale"], dtype=float)
        yaw_command_model = self.yaw_command_model
        yaw_command_model_ready = (
            bool(yaw_command_model["enabled"])
            and np.isfinite(float(yaw_command_model["accel_per_command"]))
            and not np.isclose(float(yaw_command_model["accel_per_command"]), 0.0)
            and np.isfinite(float(yaw_command_model["damping_per_s"]))
            and float(yaw_command_model["damping_per_s"]) >= 0.0
            and np.isfinite(float(yaw_command_model["bias_rad_s2"]))
        )
        legacy_yaw_model_ready = (
            angular_scale.shape == (3,)
            and np.isfinite(angular_scale[2])
            and not np.isclose(angular_scale[2], 0.0)
        )
        if (
            not self.shadow_mode
            and bool(self.config["detection"]["yaw"].get("enabled", True))
            and safety["require_calibrated_angular_model_when_active"]
            and not (yaw_command_model_ready or legacy_yaw_model_ready)
        ):
            raise ValueError(
                "active yaw interaction requires a calibrated yaw_command_model "
                "or legacy angular_accel_scale yaw value; run shadow_mode first"
            )

    @property
    def calibration_samples(self) -> int:
        return len(self._force_samples)

    @staticmethod
    def motor_data_available(motor_pwm: Sequence[float] | None) -> bool:
        if motor_pwm is None or len(motor_pwm) != 4:
            return False
        try:
            motors = np.asarray(motor_pwm, dtype=float)
        except (TypeError, ValueError):
            return False
        return bool(np.all(np.isfinite(motors)) and np.max(motors) > 0)

    def _update_bias(self, estimate: WrenchEstimate) -> None:
        elapsed = estimate.timestamp - self._start_timestamp
        if elapsed < self._settle_s:
            return

        if elapsed > self._settle_s + self._calibration_timeout_s:
            raise RuntimeError(
                "wrench bias calibration timed out while waiting for "
                f"{self._calibration_s:.2f}s of continuous stationarity "
                f"(|v| <= {self._calibration_max_speed:.3f}m/s, "
                f"|omega| <= {self._calibration_max_angular_rate:.3f}rad/s)"
            )

        stationary = (
            not estimate.measurement_rejected
            and float(np.linalg.norm(estimate.velocity)) <= self._calibration_max_speed
            and float(np.linalg.norm(estimate.angular_velocity))
            <= self._calibration_max_angular_rate
        )
        if not stationary:
            self._stationary_since = None
            self._force_samples.clear()
            self._torque_samples.clear()
            return

        if self._stationary_since is None:
            self._stationary_since = estimate.timestamp
        stationary_elapsed = estimate.timestamp - self._stationary_since
        if stationary_elapsed <= self._calibration_s:
            self._force_samples.append(estimate.external_force.copy())
            self._torque_samples.append(estimate.external_torque.copy())
            return
        if len(self._force_samples) < self._minimum_bias_samples:
            raise RuntimeError(
                f"wrench bias calibration collected only {len(self._force_samples)} samples; "
                f"need {self._minimum_bias_samples}"
            )
        self.force_bias = np.median(np.asarray(self._force_samples), axis=0)
        self.torque_bias = np.median(np.asarray(self._torque_samples), axis=0)
        self.calibrated = True

    @staticmethod
    def _response_inputs(
            estimate: WrenchEstimate,
            contacts: WrenchContactState,
    ) -> tuple[np.ndarray, float]:
        force = estimate.external_force if contacts.translation.active else np.zeros(3)
        yaw_torque = float(estimate.external_torque[2]) if contacts.yaw.active else 0.0
        return force, yaw_torque

    def update(
            self,
            position: Sequence[float],
            quaternion: Sequence[float],
            motor_pwm: Sequence[float] | None,
            battery_voltage: float | None,
            timestamp: float,
    ) -> PipelineOutput:
        timestamp = float(timestamp)
        if self._start_timestamp is None:
            self._start_timestamp = timestamp
        dt = 0.01 if self._last_timestamp is None else max(timestamp - self._last_timestamp, 1e-4)
        self._last_timestamp = timestamp

        attitude = quaternion_to_euler(quaternion)
        has_motor_data = self.motor_data_available(motor_pwm)
        expected_linear, expected_angular = self.motor_model.expected_accelerations(
            attitude, motor_pwm if has_motor_data else None, battery_voltage
        )
        raw = self.observer.update(
            position,
            quaternion,
            expected_linear,
            expected_angular,
            timestamp,
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
