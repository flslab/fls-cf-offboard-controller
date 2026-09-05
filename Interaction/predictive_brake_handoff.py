"""Reusable prediction-model brake-to-position handoff with no device I/O.

This module is deliberately separate from calibration scheduling.  An episode
starts from the measured release state and a frozen destination; there is no
acceleration phase and no fixed-pulse fallback.  Its only command decisions are
full calibrated reverse tilt, level attitude, repeated position commands, or
an explicit level-and-abort result.

The caller owns command transmission.  Every successfully sent attitude
decision must be returned through :meth:`PredictiveBrakeToPosition.record_sent`
so delayed commands remain part of the prediction.  A position decision is not
emitted until the model predicts a safe level tail *and* measured speed, tilt,
angular rate, acceleration, response settling, and dwell gates all pass.

Nothing here enables the policy in normal interaction.  Persisted prediction
models remain deployment-disabled by default; accepting a held-out validated
experimental model requires an explicit constructor option.
"""
from __future__ import annotations

import copy
import math
from typing import Iterable, Mapping

import numpy as np

from Interaction.model_based_braking import (
    DEFAULTS as MODEL_BASED_BRAKING_DEFAULTS,
    ModelBasedBrakingController,
    _validated_model,
)

__all__ = [
    "PredictiveBrakeToPosition",
    "predictive_brake_to_position",
    "projected_tilt_from_attitude_command",
    "projected_tilt_from_world_acceleration",
    "projected_tilt_history_from_world_acceleration",
    "validated_prediction_model_for_interaction",
]


DEFAULTS = {
    "allow_validated_experimental_model": False,
    "allow_state_extrapolation": False,
    "brake_tilt_deg": None,
    "max_brake_duration_s": None,
    "max_attitude_phase_s": 1.5,
    "direction_inference_speed_m_s": 0.03,
    "handoff_longitudinal_speed_m_s": 0.04,
    "handoff_lateral_speed_m_s": 0.15,
    "handoff_tilt_deg": 3.0,
    "handoff_angular_rate_rad_s": 0.50,
    "max_yaw_rate_rad_s": 0.35,
    "handoff_acceleration_m_s2": 0.35,
    "handoff_dwell_s": 0.05,
    "acceleration_filter_time_constant_s": 0.08,
    "max_acceleration_sample_gap_s": 0.10,
    "response_settle_time_constants": 4.0,
    "minimum_response_settle_s": 0.05,
    "max_position_ratchet_step_m": 0.10,
    "max_position_ratchet_rate_m_s": 3.0,
    "max_total_forward_extension_m": 0.50,
    "latch_lateral_position_to_actual": True,
    "prevent_position_pullback": True,
    "model_based_braking": {},
}

REQUIRED_VALIDATION_GATES = {
    "at_least_two_trials",
    "both_y_directions",
    "reported_segment_ids_match",
    "complete_per_trial_results",
    "per_trial_ids_match",
    "per_trial_directions_match",
    "parameters_identifiable",
    "no_active_parameter_bounds",
    "velocity_rmse_m_s",
    "terminal_error_m_s",
    "end_position_error_m",
    "no_reversal_misses_or_false_alarms",
    "directional_parameters_identifiable",
    "directional_no_active_parameter_bounds",
}

MOTION_RESIDUAL_CONFIG_KEYS = (
    "motion_residual_window_s",
    "motion_residual_min_window_s",
    "motion_residual_max_sample_gap_s",
    "motion_residual_filter_tau_s",
    "motion_residual_max_abs_accel_m_s2",
    "motion_residual_sigma_floor_m_s2",
    "motion_residual_sigma_multiplier",
    "motion_residual_apply_horizon_s",
    "motion_residual_min_samples",
)


def _number(value, name):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(name + " must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(name + " must be a finite number")
    return result


def _vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(name + " must contain finite values")
    return result


def _direction(direction_xy):
    result = _vector(direction_xy, 2, "braking direction")
    if abs(result[0]) > 1e-6 or abs(abs(result[1]) - 1.0) > 1e-6:
        raise ValueError("prediction-model braking supports only world +/-Y")
    return np.array([0.0, float(np.sign(result[1]))])


def _direction_label(direction_xy):
    return "positive_y" if float(direction_xy[1]) > 0 else "negative_y"


def projected_tilt_from_world_acceleration(
        acceleration_xy, direction_xy, gravity_m_s2=9.81):
    """Convert one actually sent world-XY acceleration to model input tilt.

    Existing interaction code stores sent attitude commands as their equivalent
    world-frame acceleration.  The prediction model consumes the signed tilt
    projected onto the frozen braking direction instead.  This public helper
    keeps that conversion identical for future adapters.
    """
    acceleration = _vector(acceleration_xy, 2, "world acceleration")
    direction = _direction(direction_xy)
    gravity = _number(gravity_m_s2, "gravity")
    if gravity <= 0:
        raise ValueError("gravity must be positive")
    return float(math.atan(float(acceleration @ direction) / gravity))


def projected_tilt_from_attitude_command(
        roll_deg, pitch_deg, projection_yaw_rad, direction_xy):
    """Project an actually sent body attitude onto frozen world +/-Y."""
    roll = math.radians(_number(roll_deg, "sent roll"))
    pitch = math.radians(_number(pitch_deg, "sent pitch"))
    yaw = _number(projection_yaw_rad, "projection yaw")
    if abs(abs(pitch) - math.pi / 2.0) <= 1e-6:
        raise ValueError("sent pitch is singular")
    sign = _direction(direction_xy)[1]
    return float(math.atan(
        -sign * (
            math.cos(yaw) * math.tan(roll) / math.cos(pitch)
            + math.sin(yaw) * math.tan(pitch)
        )
    ))


def projected_tilt_history_from_world_acceleration(
        sent_acceleration_history: Iterable, direction_xy):
    """Convert ``(actual_send_time, acceleration_xy)`` history for an episode."""
    result = []
    for item in sent_acceleration_history:
        try:
            stamp, acceleration = item
        except (TypeError, ValueError) as error:
            raise ValueError(
                "sent acceleration history must contain (time, acceleration_xy)"
            ) from error
        result.append((
            _number(stamp, "sent command time"),
            projected_tilt_from_world_acceleration(
                acceleration, direction_xy
            ),
        ))
    return result


def _segment_ids(value, name):
    if (
        not isinstance(value, list)
        or not value
        or any(
            isinstance(item, bool) or not isinstance(item, int) or item < 0
            for item in value
        )
        or len(set(value)) != len(value)
    ):
        raise ValueError(name + " must contain unique nonnegative integer IDs")
    return list(value)


def _validated_evidence(model, allow_experimental):
    """Reject incomplete/failed evidence before constructing a controller."""
    if not isinstance(model, dict):
        raise ValueError("prediction model must be a dictionary")
    if (
        model.get("control_eligible") is not True
        or model.get("validation_passed") is not True
        or model.get("independent_validation_complete") is not True
    ):
        raise ValueError(
            "prediction model is not independently validated and control eligible"
        )
    validation = model.get("validation")
    if not isinstance(validation, dict):
        raise ValueError("prediction model lacks held-out validation evidence")
    if (
        validation.get("independent_validation") is not True
        or validation.get("validation_passed") is not True
        or validation.get("control_eligible") is not True
        or validation.get("failed_gates") != []
    ):
        raise ValueError("prediction model held-out validation did not pass")
    gates = validation.get("gates")
    if (
        not isinstance(gates, dict)
        or not REQUIRED_VALIDATION_GATES.issubset(gates)
        or any(type(value) is not bool for value in gates.values())
        or any(value is not True for value in gates.values())
    ):
        raise ValueError("prediction model has missing or failed validation gates")
    training_ids = _segment_ids(
        validation.get("training_segment_ids"), "validation training IDs"
    )
    validation_ids = _segment_ids(
        validation.get("validation_segment_ids"), "held-out validation IDs"
    )
    if set(training_ids) & set(validation_ids):
        raise ValueError("training and held-out validation IDs overlap")
    if _segment_ids(model.get("train_segment_ids"), "model training IDs") != training_ids:
        raise ValueError("model training IDs disagree with validation evidence")
    per_trial = validation.get("per_trial")
    if (
        not isinstance(per_trial, list)
        or len(per_trial) != len(validation_ids)
        or any(not isinstance(row, dict) for row in per_trial)
        or [row.get("segment_id") for row in per_trial] != validation_ids
        or {row.get("direction_y") for row in per_trial} != {-1, 1}
        or validation.get("evaluation_scope")
        != "conditional_on_executed_command_schedule"
    ):
        raise ValueError("held-out trial provenance is incomplete or inconsistent")
    if not isinstance(model.get("directional_models"), dict):
        raise ValueError("direction-specific prediction models are required")
    for key in ("runtime_enabled", "deployment_approved"):
        if type(model.get(key)) is not bool or validation.get(key) is not model[key]:
            raise ValueError(key + " disagrees between model and validation evidence")
    if not allow_experimental and (
        model.get("runtime_enabled") is not True
        or model.get("deployment_approved") is not True
    ):
        raise ValueError(
            "prediction model is validated but not approved for runtime deployment"
        )


def _validate_motion_residual_evidence(model, engine_options):
    """Require validation of the exact hybrid predictor before runtime use."""
    if engine_options.get("motion_residual_observer_enabled", False) is not True:
        return
    evidence = model.get("motion_residual_validation")
    if (
        not isinstance(evidence, dict)
        or evidence.get("schema_version") != 1
        or evidence.get("predictor_kind")
        != "delayed_second_order_plus_causal_motion_residual"
        or evidence.get("independent_validation_complete") is not True
        or evidence.get("validation_passed") is not True
        or evidence.get("control_eligible") is not True
        or evidence.get("failed_gates") != []
    ):
        raise ValueError(
            "motion residual observer lacks independent hybrid-predictor validation"
        )
    recorded = evidence.get("config")
    if not isinstance(recorded, dict):
        raise ValueError("motion residual validation config is missing")
    effective = dict(MODEL_BASED_BRAKING_DEFAULTS)
    effective.update(engine_options)
    for key in MOTION_RESIDUAL_CONFIG_KEYS:
        if key not in recorded:
            raise ValueError("motion residual validation config is incomplete")
        expected, actual = effective[key], recorded[key]
        if isinstance(expected, int):
            matches = type(actual) is int and actual == expected
        else:
            try:
                matches = math.isclose(
                    _number(actual, key), float(expected),
                    rel_tol=0., abs_tol=1e-12,
                )
            except (TypeError, ValueError):
                matches = False
        if not matches:
            raise ValueError(
                "motion residual runtime config differs from held-out validation"
            )


def _observed_command_envelope(
        model, direction_xy, requested_tilt_deg=None):
    """Return observed brake tilt and a jointly supported pulse duration."""
    sign = int(np.sign(direction_xy[1]))
    rows = [
        row for row in model.get("data_ranges", [])
        if isinstance(row, dict) and row.get("direction_y") == sign
    ]
    if not rows:
        raise ValueError("prediction model has no data for requested direction")
    observations = []
    for row in rows:
        command_range = _vector(
            row.get("command_acceleration_m_s2"),
            2,
            "observed command acceleration range",
        )
        observed_brake_m_s2 = max(0.0, -float(command_range[0]))
        observed_tilt_deg = (
            math.degrees(math.atan(observed_brake_m_s2 / 9.81))
            if observed_brake_m_s2 > 0 else None
        )
        phase_times = row.get("phase_command_times_relative_s")
        if isinstance(phase_times, dict):
            try:
                brake = _number(phase_times["brake"], "brake phase time")
                level = _number(
                    phase_times["level_after_brake"],
                    "level-after-brake phase time",
                )
            except (KeyError, TypeError, ValueError):
                continue
            if observed_tilt_deg is not None and level > brake:
                observations.append((observed_tilt_deg, level - brake))
    if not observations:
        raise ValueError(
            "prediction model lacks an observed brake duration envelope"
        )
    maximum_tilt = float(max(tilt for tilt, _ in observations))
    requested = (
        maximum_tilt
        if requested_tilt_deg is None
        else _number(requested_tilt_deg, "requested observed brake tilt")
    )
    durations = [
        duration for tilt, duration in observations
        if tilt + 1e-6 >= requested
    ]
    if not durations:
        raise ValueError(
            "no observed trial jointly covers requested brake tilt and duration"
        )
    return maximum_tilt, float(max(durations))


def validated_prediction_model_for_interaction(
        calibration_entry, *, enabled, direction_xy,
        allow_validated_experimental_model=False,
        terminal_velocity_tolerance_m_s=0.05):
    """Select a frozen model for a future interaction integration point.

    When ``enabled`` is false, old calibration documents remain compatible and
    ``None`` is returned.  Enabling is fail-closed: a missing, failed, or
    unapproved model raises before a release episode can begin.
    """
    if type(enabled) is not bool:
        raise ValueError("prediction-model interaction enabled must be boolean")
    if not enabled:
        return None
    if not isinstance(calibration_entry, dict):
        raise ValueError("no saved calibration entry for prediction braking")
    model = calibration_entry.get("prediction_model")
    allow = allow_validated_experimental_model
    if type(allow) is not bool:
        raise ValueError("allow_validated_experimental_model must be boolean")
    direction = _direction(direction_xy)
    tolerance = _number(
        terminal_velocity_tolerance_m_s,
        "terminal velocity tolerance",
    )
    if not 0 < tolerance <= 0.10:
        raise ValueError("terminal velocity tolerance must be in (0, 0.10]")
    _validated_evidence(model, allow)
    _, _, margin, _ = _validated_model(model, allow, direction[1])
    if margin >= tolerance:
        raise ValueError(
            f"prediction model {_direction_label(direction)} margin "
            f"{margin:.6f}m/s is not below {tolerance:.6f}m/s"
        )
    _observed_command_envelope(model, direction)
    return copy.deepcopy(model)


class PredictiveBrakeToPosition:
    """Stateful BRAKE -> LEVEL/SETTLE -> POSITION episode.

    The class returns decisions and never talks to a commander.  Direction is
    frozen from measured velocity (preferred), otherwise from the destination.
    The target is frozen too; at handoff it may only be clamped forward to the
    actual stop point, never moved behind the vehicle to create pullback.
    """

    BRAKING = "attitude_braking"
    POSITION = "position_control"
    ABORTED = "abort_level"

    def __init__(
            self, model, *, initial_state, destination_position,
            now_s, sent_command_history: Iterable = (),
            direction_xy=None, config: Mapping | None = None,
            clock=None):
        if config is not None and not isinstance(config, Mapping):
            raise ValueError("predictive brake config must be a mapping")
        self.config = copy.deepcopy(DEFAULTS)
        self.config.update(copy.deepcopy(dict(config or {})))
        for key in (
                "allow_validated_experimental_model",
                "allow_state_extrapolation",
                "latch_lateral_position_to_actual",
                "prevent_position_pullback"):
            if type(self.config[key]) is not bool:
                raise ValueError(key + " must be boolean")
        engine_options = self.config.get("model_based_braking")
        if not isinstance(engine_options, dict):
            raise ValueError("model_based_braking must be a mapping")
        _validate_motion_residual_evidence(model, engine_options)
        numeric_keys = (
            "max_attitude_phase_s",
            "direction_inference_speed_m_s",
            "handoff_longitudinal_speed_m_s",
            "handoff_lateral_speed_m_s",
            "handoff_tilt_deg",
            "handoff_angular_rate_rad_s",
            "max_yaw_rate_rad_s",
            "handoff_acceleration_m_s2",
            "handoff_dwell_s",
            "acceleration_filter_time_constant_s",
            "max_acceleration_sample_gap_s",
            "response_settle_time_constants",
            "minimum_response_settle_s",
            "max_position_ratchet_step_m",
            "max_position_ratchet_rate_m_s",
            "max_total_forward_extension_m",
        )
        for key in numeric_keys:
            self.config[key] = _number(self.config[key], key)
        if (
            any(self.config[key] <= 0 for key in numeric_keys)
            or self.config["handoff_dwell_s"] > 1.0
            or self.config["handoff_tilt_deg"] > 5.0
            or self.config["handoff_longitudinal_speed_m_s"] > 0.10
            or self.config["max_yaw_rate_rad_s"] > 1.0
            or self.config["max_attitude_phase_s"] > 3.0
            or self.config["max_position_ratchet_step_m"] > 0.25
            or self.config["max_position_ratchet_rate_m_s"] > 5.0
            or self.config["max_total_forward_extension_m"] > 1.0
        ):
            raise ValueError("predictive brake config is outside its bounded domain")

        self.started_at_s = _number(now_s, "episode start time")
        position = _vector(initial_state["position_xy"], 2, "initial position")
        velocity = _vector(initial_state["velocity_xy"], 2, "initial velocity")
        destination = _vector(destination_position, 3, "destination position")
        self.destination_position = destination.copy()
        self.destination_position.setflags(write=False)
        if direction_xy is None:
            threshold = self.config["direction_inference_speed_m_s"]
            if abs(float(velocity[1])) >= threshold:
                sign = float(np.sign(velocity[1]))
                self.direction_source = "measured_velocity"
            elif abs(float(destination[1] - position[1])) > 1e-6:
                sign = float(np.sign(destination[1] - position[1]))
                self.direction_source = "destination_delta"
            else:
                raise ValueError(
                    "cannot infer braking direction from stationary state and target"
                )
            direction_xy = [0.0, sign]
        else:
            self.direction_source = "explicit"
        self.direction_xy = _direction(direction_xy)
        self.direction_xy.setflags(write=False)
        self.direction_label = _direction_label(self.direction_xy)

        allow = self.config["allow_validated_experimental_model"]
        tolerance = _number(
            engine_options.get("terminal_velocity_tolerance_m_s", 0.05),
            "terminal velocity tolerance",
        )
        self.model = validated_prediction_model_for_interaction(
            {"prediction_model": model},
            enabled=True,
            direction_xy=self.direction_xy,
            allow_validated_experimental_model=allow,
            terminal_velocity_tolerance_m_s=tolerance,
        )
        params, _, margin, selected = _validated_model(
            self.model, allow, self.direction_xy[1]
        )
        self.terminal_velocity_error_margin_m_s = margin
        self.selected_directional_model = selected
        observed_tilt_deg, _ = _observed_command_envelope(
            self.model, self.direction_xy
        )
        requested_tilt = self.config["brake_tilt_deg"]
        self.brake_tilt_deg = (
            min(20.0, observed_tilt_deg)
            if requested_tilt is None
            else _number(requested_tilt, "brake_tilt_deg")
        )
        if not 0 < self.brake_tilt_deg <= observed_tilt_deg + 1e-6:
            raise ValueError("requested brake tilt exceeds observed model envelope")
        _, observed_duration_s = _observed_command_envelope(
            self.model, self.direction_xy, self.brake_tilt_deg
        )
        requested_duration = self.config["max_brake_duration_s"]
        self.max_brake_duration_s = (
            observed_duration_s
            if requested_duration is None
            else _number(requested_duration, "max_brake_duration_s")
        )
        if (
            self.max_brake_duration_s <= 0
            or self.max_brake_duration_s > observed_duration_s + 1e-6
            or self.max_brake_duration_s > 0.50
        ):
            raise ValueError(
                "requested brake duration exceeds observed/bounded model envelope"
            )
        if self.config["max_attitude_phase_s"] <= self.max_brake_duration_s:
            raise ValueError(
                "max_attitude_phase_s must exceed maximum brake duration"
            )
        self.response_settle_s = max(
            self.config["minimum_response_settle_s"],
            params["delay_s"]
            + self.config["response_settle_time_constants"]
            / (params["zeta"] * params["wn_rad_s"]),
        )
        required_attitude_phase_s = (
            self.max_brake_duration_s
            + self.response_settle_s
            + self.config["handoff_dwell_s"]
        )
        if self.config["max_attitude_phase_s"] <= required_attitude_phase_s:
            raise ValueError(
                "max_attitude_phase_s cannot cover brake, response settle, and dwell"
            )
        self.command_valid_for_s = _number(
            engine_options.get("command_valid_for_s", 0.03),
            "command_valid_for_s",
        )
        self.max_state_age_s = _number(
            engine_options.get("max_state_age_s", 0.04),
            "max_state_age_s",
        )
        self.max_state_group_skew_s = _number(
            engine_options.get("max_state_group_skew_s", 0.03),
            "max_state_group_skew_s",
        )
        if not 0 < self.command_valid_for_s <= 0.10:
            raise ValueError("command_valid_for_s must be in (0, 0.10]")
        if (
            not 0 < self.max_state_age_s <= 0.10
            or not 0 < self.max_state_group_skew_s <= 0.03
        ):
            raise ValueError("state age/skew limits exceed bounded domain")

        controller_config = copy.deepcopy(engine_options)
        controller_config.update({
            "enabled": True,
            # The wrapper performs strict validation and independently rejects
            # every state-range extrapolation before a command is returned.
            "experimental_calibration": allow,
            "calibration_one_way_latch": True,
            "brake_tilt_deg": self.brake_tilt_deg,
        })
        self._controller = ModelBasedBrakingController(
            self.model,
            target_position_xy=self.destination_position[:2],
            direction_xy=self.direction_xy,
            brake_deadline_s=self.started_at_s + self.max_brake_duration_s,
            config=controller_config,
            **({} if clock is None else {"clock": clock}),
        )
        self.phase = self.BRAKING
        self.abort_reason = None
        self.position_target = None
        self.target_clamped_to_actual = False
        self.lateral_target_latched_to_actual = False
        self._last_decision_time = None
        self._level_command_since = None
        self._last_projected_tilt = None
        self._handoff_ready_since = None
        self._previous_velocity = None
        self._previous_state_time = None
        self._filtered_acceleration = np.zeros(2)
        self._acceleration_valid = False
        self._decision_sequence = 0
        self._issued_attitude_decisions = {}
        self._sent_decision_sequences = set()
        self._send_protocol_error = None
        self._position_handoff_at_s = None
        self._last_position_state_time_s = None
        self._last_position_actual_xy = None
        for item in sent_command_history:
            try:
                stamp, tilt = item
            except (TypeError, ValueError) as error:
                raise ValueError(
                    "sent command history must contain (time, projected tilt)"
                ) from error
            self._record_projected_tilt(stamp, tilt)

    def _record_projected_tilt(self, sent_at_s, projected_tilt_rad):
        try:
            stamp = _number(sent_at_s, "sent command time")
        except (TypeError, ValueError):
            self._send_protocol_error = "invalid_sent_attitude_timestamp"
            raise
        tilt = _number(projected_tilt_rad, "sent projected tilt")
        self._controller.record_command(stamp, tilt)
        if abs(tilt) <= 1e-12:
            if self._last_projected_tilt is None or abs(
                    self._last_projected_tilt) > 1e-12:
                self._level_command_since = stamp
        else:
            self._level_command_since = None
            self._handoff_ready_since = None
        self._last_projected_tilt = tilt

    def record_sent(self, decision, sent_at_s, *, actual_attitude=None):
        """Record one returned attitude decision after it was actually sent.

        Record each send before asking for the next decision.  A late or
        superseded decision is still recorded because the actuator really received
        it, but the episode is marked unsafe and the next decision aborts level.
        If a lower layer altered the command, pass a mapping containing its actual
        ``roll_deg``, ``pitch_deg``, ``yaw_rate_deg_s`` and projection yaw through
        ``actual_attitude``; that real projected input is retained before aborting.
        This prevents stale commands from disappearing from delayed prediction
        history while also preventing control from continuing normally.
        """
        if not isinstance(decision, dict) or decision.get("action") not in (
                "brake", "level", "abort_level"):
            self._send_protocol_error = "invalid_sent_attitude_decision"
            raise ValueError("only a returned attitude decision can be recorded")
        try:
            stamp = _number(sent_at_s, "sent command time")
        except (TypeError, ValueError):
            self._send_protocol_error = "invalid_sent_attitude_timestamp"
            raise
        sequence = decision.get("decision_sequence")
        if isinstance(sequence, bool) or not isinstance(sequence, int):
            self._send_protocol_error = "invalid_sent_attitude_decision"
            raise ValueError("attitude decision has no valid episode sequence")
        issued = self._issued_attitude_decisions.get(sequence)
        if issued is None:
            self._send_protocol_error = "unknown_sent_attitude_decision"
            raise ValueError("attitude decision was not issued by this episode")
        if sequence in self._sent_decision_sequences:
            self._send_protocol_error = "duplicate_sent_attitude_decision"
            raise ValueError("attitude decision was already recorded as sent")
        if actual_attitude is not None and not isinstance(actual_attitude, Mapping):
            self._send_protocol_error = "invalid_actual_sent_attitude"
            raise ValueError("actual_attitude must be a mapping")
        actual = decision if actual_attitude is None else actual_attitude
        try:
            decision_intact = (
                decision.get("action") == issued["action"]
                and all(
                    math.isclose(
                        _number(decision.get(key), "decision " + key),
                        issued[key],
                        rel_tol=0.0,
                        abs_tol=1e-12,
                    )
                    for key in (
                        "projected_tilt_rad",
                        "roll_deg",
                        "pitch_deg",
                        "yaw_rate_deg_s",
                        "projection_yaw_rad",
                    )
                )
            )
            actual_unchanged = all(
                math.isclose(
                    _number(actual.get(key), "actual sent " + key),
                    issued[key],
                    rel_tol=0.0,
                    abs_tol=1e-12,
                )
                for key in (
                    "roll_deg",
                    "pitch_deg",
                    "yaw_rate_deg_s",
                    "projection_yaw_rad",
                )
            )
        except (TypeError, ValueError):
            decision_intact = False
            actual_unchanged = False
        if not decision_intact or not actual_unchanged:
            self._send_protocol_error = "sent_attitude_did_not_match_decision"
            try:
                actual_tilt = projected_tilt_from_attitude_command(
                    actual.get("roll_deg"),
                    actual.get("pitch_deg"),
                    actual.get(
                        "projection_yaw_rad", issued["projection_yaw_rad"]
                    ),
                    self.direction_xy,
                )
                self._record_projected_tilt(stamp, actual_tilt)
            except (TypeError, ValueError):
                pass
            self._sent_decision_sequences.add(sequence)
            return False
        try:
            self._record_projected_tilt(stamp, issued["projected_tilt_rad"])
        except (TypeError, ValueError):
            self._send_protocol_error = "invalid_actual_sent_attitude_history"
            raise
        self._sent_decision_sequences.add(sequence)
        if (
            stamp < issued["decision_time_s"] - 1e-9
            or stamp > issued["valid_until_s"] + 1e-9
            or sequence != self._decision_sequence
            or (
                self._position_handoff_at_s is not None
                and stamp >= self._position_handoff_at_s - 1e-9
                and issued["action"] != "abort_level"
            )
        ):
            self._send_protocol_error = "attitude_decision_sent_outside_valid_phase"
            return False
        return True

    def _update_acceleration(self, state):
        velocity = _vector(state["velocity_xy"], 2, "velocity")
        stamp = _number(state["time_s"], "state time")
        supplied = state.get("acceleration_xy")
        if supplied is not None:
            self._filtered_acceleration = _vector(
                supplied, 2, "acceleration"
            ).copy()
            self._acceleration_valid = True
        elif self._previous_velocity is not None:
            dt = stamp - self._previous_state_time
            if 0 < dt <= self.config["max_acceleration_sample_gap_s"]:
                raw = (velocity - self._previous_velocity) / dt
                alpha = 1.0 - math.exp(
                    -dt / self.config[
                        "acceleration_filter_time_constant_s"
                    ]
                )
                self._filtered_acceleration = (
                    alpha * raw
                    + (1.0 - alpha) * self._filtered_acceleration
                )
                self._acceleration_valid = True
            else:
                self._filtered_acceleration.fill(0.0)
                self._acceleration_valid = False
                self._handoff_ready_since = None
        self._previous_velocity = velocity.copy()
        self._previous_state_time = stamp

    def _measured_handoff_state(self, state):
        velocity = _vector(state["velocity_xy"], 2, "velocity")
        rpy = _vector(state["orientation_rpy_rad"], 3, "orientation")
        angular = _vector(
            state["angular_velocity_rad_s"], 3, "angular velocity"
        )
        longitudinal = float(velocity @ self.direction_xy)
        lateral = float(np.linalg.norm(
            velocity - longitudinal * self.direction_xy
        ))
        return {
            "longitudinal_speed_m_s": longitudinal,
            "lateral_speed_m_s": lateral,
            "tilt_deg": float(np.degrees(np.hypot(rpy[0], rpy[1]))),
            "angular_rate_rad_s": float(np.linalg.norm(angular)),
            "yaw_rad": float(rpy[2]),
            "yaw_rate_rad_s": float(angular[2]),
            "acceleration_m_s2": float(np.linalg.norm(
                self._filtered_acceleration
            )),
            "acceleration_valid": self._acceleration_valid,
        }

    def _validated_position_sample(self, state, now_s):
        actual = _vector(state["position_xy"], 2, "position")
        state_time = _number(state["time_s"], "state time")
        skew = _number(state["state_group_skew_s"], "state group skew")
        if (
            not 0 <= now_s - state_time <= self.max_state_age_s
            or not 0 <= skew <= self.max_state_group_skew_s
        ):
            raise ValueError("stale or skewed position handoff state")
        if self._last_position_state_time_s is not None:
            dt = state_time - self._last_position_state_time_s
            displacement = float(np.linalg.norm(
                actual - self._last_position_actual_xy
            ))
            if dt < 0 or (dt == 0 and displacement > 1e-9):
                raise ValueError("nonmonotonic position handoff state")
            if displacement > self.config["max_position_ratchet_step_m"]:
                raise ValueError("position handoff sample jump exceeds step limit")
            if (
                dt > 0
                and displacement / dt
                > self.config["max_position_ratchet_rate_m_s"]
            ):
                raise ValueError("position handoff sample jump exceeds rate limit")
        self._last_position_state_time_s = state_time
        self._last_position_actual_xy = actual.copy()
        return actual

    def _position_handoff_target(self, state, now_s):
        actual = self._validated_position_sample(state, now_s)
        target = self.destination_position.copy()
        direction = self.direction_xy
        if self.config["latch_lateral_position_to_actual"]:
            target_xy = target[:2]
            target_projection = float(target_xy @ direction)
            actual_projection = float(actual @ direction)
            target[:2] = actual + (
                target_projection - actual_projection
            ) * direction
            self.lateral_target_latched_to_actual = True
        if self.config["prevent_position_pullback"]:
            target_projection = float(target[:2] @ direction)
            actual_projection = float(actual @ direction)
            if target_projection < actual_projection:
                extension = actual_projection - target_projection
                if extension > min(
                        self.config["max_position_ratchet_step_m"],
                        self.config["max_total_forward_extension_m"]):
                    raise ValueError(
                        "initial position handoff clamp exceeds extension limit"
                    )
                target[:2] += (
                    extension
                ) * direction
                self.target_clamped_to_actual = True
        return target

    def _advance_position_target_to_actual(self, state, now_s):
        """Ratchet a latched target forward so it never becomes a pullback."""
        actual = self._validated_position_sample(state, now_s)
        if not self.config["prevent_position_pullback"]:
            return False
        actual_projection = float(actual @ self.direction_xy)
        target_projection = float(self.position_target[:2] @ self.direction_xy)
        if actual_projection <= target_projection + 1e-9:
            return False
        destination_projection = float(
            self.destination_position[:2] @ self.direction_xy
        )
        if (
            actual_projection - destination_projection
            > self.config["max_total_forward_extension_m"]
        ):
            raise ValueError("position target ratchet exceeds total extension limit")
        target = self.position_target.copy()
        target[:2] += (
            actual_projection - target_projection
        ) * self.direction_xy
        target.setflags(write=False)
        self.position_target = target
        self.target_clamped_to_actual = True
        return True

    def _base_result(self, action, reason, now_s):
        return {
            "action": action,
            "reason": reason,
            "phase": self.phase,
            "roll_deg": None,
            "pitch_deg": None,
            "yaw_rate_deg_s": 0.0,
            "projection_yaw_rad": None,
            "projected_tilt_rad": None,
            "position_target": (
                None if self.position_target is None
                else self.position_target.tolist()
            ),
            "direction_xy": self.direction_xy.tolist(),
            "direction_source": self.direction_source,
            "destination_position": self.destination_position.tolist(),
            "selected_directional_model": self.selected_directional_model,
            "terminal_velocity_error_margin_m_s": (
                self.terminal_velocity_error_margin_m_s
            ),
            "max_brake_duration_s": self.max_brake_duration_s,
            "response_settle_s": self.response_settle_s,
            "elapsed_s": max(0.0, now_s - self.started_at_s),
            "target_clamped_to_actual": self.target_clamped_to_actual,
            "lateral_target_latched_to_actual": (
                self.lateral_target_latched_to_actual
            ),
            "position_handoff_latched": self.phase == self.POSITION,
            "requires_external_abort_handling": self.phase == self.ABORTED,
        }

    def _issue_attitude_result(self, result, now_s, source_valid_until=None):
        self._decision_sequence += 1
        valid_until = now_s + self.command_valid_for_s
        if source_valid_until is not None:
            try:
                source_limit = _number(source_valid_until, "prediction valid_until_s")
            except (TypeError, ValueError):
                source_limit = now_s
            valid_until = min(valid_until, source_limit)
        result.update(
            decision_sequence=self._decision_sequence,
            decision_time_s=now_s,
            valid_until_s=valid_until,
        )
        self._issued_attitude_decisions[self._decision_sequence] = {
            "action": result["action"],
            "projected_tilt_rad": float(result["projected_tilt_rad"]),
            "roll_deg": float(result["roll_deg"]),
            "pitch_deg": float(result["pitch_deg"]),
            "yaw_rate_deg_s": float(result["yaw_rate_deg_s"]),
            "projection_yaw_rad": float(result["projection_yaw_rad"]),
            "decision_time_s": now_s,
            "valid_until_s": valid_until,
        }
        while len(self._issued_attitude_decisions) > 64:
            oldest = min(self._issued_attitude_decisions)
            self._issued_attitude_decisions.pop(oldest)
            self._sent_decision_sequences.discard(oldest)
        return result

    def _issue_position_result(self, result, now_s):
        self._decision_sequence += 1
        result.update(
            decision_sequence=self._decision_sequence,
            decision_time_s=now_s,
            valid_until_s=now_s + self.command_valid_for_s,
        )
        return result

    def _abort(self, reason, now_s, detail=None):
        self.phase = self.ABORTED
        self.abort_reason = reason
        result = self._base_result("abort_level", reason, now_s)
        result.update(
            roll_deg=0.0,
            pitch_deg=0.0,
            projected_tilt_rad=0.0,
            projection_yaw_rad=0.0,
            detail=detail,
        )
        return self._issue_attitude_result(result, now_s)

    def decide(self, now_s, state):
        """Return the next command decision without sending it."""
        now = _number(now_s, "decision time")
        if self.phase == self.ABORTED:
            return self._abort(self.abort_reason, now)
        if self._send_protocol_error is not None:
            return self._abort(self._send_protocol_error, now)
        if now < self.started_at_s:
            return self._abort("decision_before_episode_start", now)
        if self._last_decision_time is not None and now <= self._last_decision_time:
            return self._abort("nonincreasing_decision_time", now)
        self._last_decision_time = now
        if self.phase == self.POSITION:
            try:
                advanced = self._advance_position_target_to_actual(state, now)
            except (KeyError, TypeError, ValueError) as error:
                return self._abort("invalid_position_handoff_state", now, str(error))
            result = self._base_result(
                "position",
                (
                    "position_target_ratchet_advanced"
                    if advanced else "position_handoff_already_latched"
                ),
                now,
            )
            return self._issue_position_result(result, now)
        if now - self.started_at_s > self.config["max_attitude_phase_s"]:
            return self._abort("attitude_phase_timeout", now)
        try:
            self._update_acceleration(state)
            measured = self._measured_handoff_state(state)
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            return self._abort("invalid_runtime_state", now, str(error))
        if abs(measured["yaw_rate_rad_s"]) > self.config["max_yaw_rate_rad_s"]:
            return self._abort(
                "yaw_rate_outside_prediction_domain",
                now,
                measured["yaw_rate_rad_s"],
            )
        try:
            prediction = self._controller.decide(now, state)
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            return self._abort("invalid_runtime_state", now, str(error))
        if (
            prediction.get("action") == "fallback"
            or prediction.get("fallback_to_original_brake") is True
        ):
            return self._abort(
                "prediction_failure_without_runtime_fixed_pulse_fallback",
                now,
                prediction.get("reason"),
            )
        if (
            prediction.get("state_extrapolates_training_range") is True
            and not self.config["allow_state_extrapolation"]
        ):
            return self._abort(
                "state_outside_identified_range", now,
                "runtime extrapolation is disabled",
            )
        if prediction.get("action") not in ("brake", "level"):
            return self._abort(
                "invalid_prediction_action", now, prediction.get("action")
            )

        action = prediction["action"]
        prediction_safe = bool(
            action == "level"
            and prediction.get("hard_terminal_constraints_satisfied") is True
        )
        level_elapsed = (
            None if self._level_command_since is None
            else max(0.0, now - self._level_command_since)
        )
        measured_safe = bool(
            measured["acceleration_valid"]
            and abs(measured["longitudinal_speed_m_s"])
            <= self.config["handoff_longitudinal_speed_m_s"]
            and measured["lateral_speed_m_s"]
            <= self.config["handoff_lateral_speed_m_s"]
            and measured["tilt_deg"] <= self.config["handoff_tilt_deg"]
            and measured["angular_rate_rad_s"]
            <= self.config["handoff_angular_rate_rad_s"]
            and measured["acceleration_m_s2"]
            <= self.config["handoff_acceleration_m_s2"]
        )
        response_settled = bool(
            level_elapsed is not None
            and level_elapsed >= self.response_settle_s
        )
        if action == "level" and prediction_safe and measured_safe and response_settled:
            if self._handoff_ready_since is None:
                self._handoff_ready_since = now
        else:
            self._handoff_ready_since = None
        ready_elapsed = (
            None if self._handoff_ready_since is None
            else max(0.0, now - self._handoff_ready_since)
        )
        if (
            ready_elapsed is not None
            and ready_elapsed >= self.config["handoff_dwell_s"]
        ):
            try:
                self.position_target = self._position_handoff_target(state, now)
            except (KeyError, TypeError, ValueError) as error:
                return self._abort(
                    "invalid_position_handoff_state", now, str(error)
                )
            self.position_target.setflags(write=False)
            self.phase = self.POSITION
            self._position_handoff_at_s = now
            result = self._base_result(
                "position", "measured_and_predicted_handoff_gates_passed", now
            )
            result.update(
                measured_handoff_state=measured,
                prediction=copy.deepcopy(prediction),
                level_command_elapsed_s=level_elapsed,
                handoff_ready_elapsed_s=ready_elapsed,
            )
            return self._issue_position_result(result, now)

        result = self._base_result(action, prediction["reason"], now)
        result.update(
            roll_deg=float(prediction["roll_deg"]),
            pitch_deg=float(prediction["pitch_deg"]),
            projected_tilt_rad=float(prediction["projected_tilt_rad"]),
            projection_yaw_rad=measured["yaw_rad"],
            measured_handoff_state=measured,
            prediction=copy.deepcopy(prediction),
            predicted_handoff_safe=prediction_safe,
            measured_handoff_safe=measured_safe,
            response_queue_settled=response_settled,
            level_command_elapsed_s=level_elapsed,
            handoff_ready_elapsed_s=ready_elapsed,
        )
        return self._issue_attitude_result(
            result, now, prediction.get("valid_until_s")
        )


def predictive_brake_to_position(
        prediction_model, *, initial_state, destination_position, now_s,
        sent_command_history=(), direction_xy=None, config=None, clock=None):
    """Create a reusable stateful brake-to-position episode.

    Call ``episode.decide(...)``, transmit its returned command, then call
    ``episode.record_sent(...)`` only after a successful attitude send.
    """
    return PredictiveBrakeToPosition(
        prediction_model,
        initial_state=initial_state,
        destination_position=destination_position,
        now_s=now_s,
        sent_command_history=sent_command_history,
        direction_xy=direction_xy,
        config=config,
        clock=clock,
    )
