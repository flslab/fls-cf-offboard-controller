"""Configuration and coverage bookkeeping for LMPC bootstrap flights.

The bootstrap is deliberately a data-collection mode.  It never evaluates an
LMPC policy and never grants an optimizer command authority.  The normal,
bounded attitude coast controller stops the vehicle after each real release;
this module only freezes the private mission overlay and records which
world-Y/initial-speed cells produced complete terminal trajectories.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import numbers

import numpy as np

from Interaction.conditional_velocity_lmpc import (
    ConditionalVelocityLMPCConfig,
    OfflineConditionalVelocityLMPC,
    conditional_velocity_lmpc_fingerprint,
)
from Interaction.learning_velocity_mpc import (
    frozen_velocity_model_from_prediction_model,
)
from Interaction.offline_braking_selector import FrozenTiltModel
from Interaction.velocity_lmpc_safe_set import SafeSetLimits, StageCostSpec
from Interaction.wrench_model_calibration import (
    DEFAULT_CALIBRATION_PATH,
    apply_drone_calibration,
    planar_braking_fit_is_current,
)


def _positive_float(value, name):
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ValueError(f"{name} must be a finite positive number")
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be a finite positive number")
    return value


def _positive_integer(value, name):
    if (
        isinstance(value, bool)
        or not isinstance(value, numbers.Integral)
        or int(value) <= 0
    ):
        raise ValueError(f"{name} must be a positive integer")
    return int(value)


@dataclass(frozen=True)
class MPCBootstrapCalibrationConfig:
    """The release-speed cells required before online LMPC work can begin."""

    initial_speed_targets_m_s: tuple[float, ...] = (0.25, 0.45, 0.65)
    speed_tolerance_m_s: float = 0.06
    repetitions_per_cell: int = 2
    max_cross_speed_m_s: float = 0.03
    max_release_speed_m_s: float = 0.75
    prediction_step_s: float = 0.02

    @classmethod
    def from_mapping(cls, value):
        value = dict(value or {})
        unknown = set(value) - {
            "enabled",
            "initial_speed_targets_m_s",
            "speed_tolerance_m_s",
            "repetitions_per_cell",
            "max_cross_speed_m_s",
            "max_release_speed_m_s",
            "prediction_step_s",
        }
        if unknown:
            raise ValueError(
                "unknown mpc_bootstrap_calibration keys: "
                + ", ".join(sorted(unknown))
            )
        enabled = value.get("enabled", False)
        if type(enabled) is not bool:
            raise ValueError("mpc_bootstrap_calibration.enabled must be boolean")
        raw_targets = value.get(
            "initial_speed_targets_m_s", cls.initial_speed_targets_m_s
        )
        if not isinstance(raw_targets, (tuple, list)) or not raw_targets:
            raise ValueError(
                "initial_speed_targets_m_s must be a non-empty increasing list"
            )
        targets = tuple(
            _positive_float(item, f"initial_speed_targets_m_s[{index}]")
            for index, item in enumerate(raw_targets)
        )
        if any(second <= first for first, second in zip(targets, targets[1:])):
            raise ValueError(
                "initial_speed_targets_m_s must be strictly increasing"
            )
        tolerance = _positive_float(
            value.get("speed_tolerance_m_s", cls.speed_tolerance_m_s),
            "speed_tolerance_m_s",
        )
        if any(
            second-first <= 2.0*tolerance
            for first, second in zip(targets, targets[1:])
        ):
            raise ValueError(
                "speed target windows must not overlap; reduce "
                "speed_tolerance_m_s"
            )
        repetitions = _positive_integer(
            value.get("repetitions_per_cell", cls.repetitions_per_cell),
            "repetitions_per_cell",
        )
        cross = _positive_float(
            value.get("max_cross_speed_m_s", cls.max_cross_speed_m_s),
            "max_cross_speed_m_s",
        )
        maximum = _positive_float(
            value.get("max_release_speed_m_s", cls.max_release_speed_m_s),
            "max_release_speed_m_s",
        )
        if maximum+1e-12 < targets[-1]+tolerance:
            raise ValueError(
                "max_release_speed_m_s must cover the highest target window"
            )
        prediction_step = _positive_float(
            value.get("prediction_step_s", cls.prediction_step_s),
            "prediction_step_s",
        )
        if not 0.005 <= prediction_step <= 0.02:
            raise ValueError("prediction_step_s must be in [0.005, 0.02]")
        return cls(
            initial_speed_targets_m_s=targets,
            speed_tolerance_m_s=tolerance,
            repetitions_per_cell=repetitions,
            max_cross_speed_m_s=cross,
            max_release_speed_m_s=maximum,
            prediction_step_s=prediction_step,
        )

    def to_dict(self):
        return {
            "initial_speed_targets_m_s": list(self.initial_speed_targets_m_s),
            "speed_tolerance_m_s": self.speed_tolerance_m_s,
            "repetitions_per_cell": self.repetitions_per_cell,
            "max_cross_speed_m_s": self.max_cross_speed_m_s,
            "max_release_speed_m_s": self.max_release_speed_m_s,
            "prediction_step_s": self.prediction_step_s,
        }


def mpc_bootstrap_world_y_direction(
        velocity_xy_m_s, max_cross_speed_m_s, minimum_aligned_speed_m_s=0.05):
    """Return a locked world-Y direction only for an on-axis release."""
    velocity = np.asarray(velocity_xy_m_s, dtype=float)
    maximum_cross = _positive_float(
        max_cross_speed_m_s, "max_cross_speed_m_s"
    )
    minimum_aligned = _positive_float(
        minimum_aligned_speed_m_s, "minimum_aligned_speed_m_s"
    )
    if velocity.shape != (2,) or not np.all(np.isfinite(velocity)):
        raise ValueError("MPC bootstrap release velocity must be finite XY")
    if (
        abs(float(velocity[0])) > maximum_cross+1e-12
        or abs(float(velocity[1])) < minimum_aligned-1e-12
    ):
        return None
    return np.asarray([0.0, float(np.sign(velocity[1]))])


def mpc_decision_state_age_is_fresh(
        state_time_s, checked_at_s, max_state_age_s):
    """Return whether a scheduled send still has a fresh causal state."""
    if any(
        isinstance(value, bool) or not isinstance(value, numbers.Real)
        for value in (state_time_s, checked_at_s, max_state_age_s)
    ):
        return False
    values = np.asarray([
        state_time_s, checked_at_s, max_state_age_s,
    ], dtype=float)
    if not np.all(np.isfinite(values)) or float(max_state_age_s) <= 0.0:
        return False
    age_s = float(checked_at_s)-float(state_time_s)
    return 0.0 <= age_s <= float(max_state_age_s)+1e-12


def _prediction_model_sha256(prediction_model):
    try:
        encoded = json.dumps(
            prediction_model,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as error:
        raise ValueError(
            "--mpc saved prediction_model is not canonical JSON"
        ) from error
    return "sha256:"+hashlib.sha256(encoded).hexdigest()


def build_mpc_bootstrap_model_contracts(
        prediction_model, *, prediction_step_s):
    """Freeze exact per-direction LMPC contracts before the vehicle arms."""
    if not isinstance(prediction_model, dict):
        raise ValueError(
            "--mpc requires a saved independently validated directional "
            "prediction_model; run the prediction calibration first"
        )
    prediction_step = _positive_float(
        prediction_step_s, "prediction_step_s"
    )
    lmpc_config = ConditionalVelocityLMPCConfig(
        prediction_step_s=prediction_step
    )
    lmpc_config.validate()
    safe_set_limits = SafeSetLimits()
    stage_cost_spec = StageCostSpec()
    source_sha256 = _prediction_model_sha256(prediction_model)
    contracts = {}
    for expected_label, direction_sign in (
            ("positive_y", 1), ("negative_y", -1)):
        try:
            frozen_model, selected_label = (
                frozen_velocity_model_from_prediction_model(
                    prediction_model,
                    direction_y=direction_sign,
                    require_validated_evidence=True,
                )
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f"--mpc {expected_label} prediction_model is missing or "
                "not independently validated"
            ) from error
        if selected_label != expected_label:
            raise ValueError(
                f"--mpc {expected_label} prediction_model selected "
                f"unexpected direction {selected_label!r}"
            )
        if not math.isfinite(float(frozen_model.delay_s)) or (
                float(frozen_model.delay_s) <= 0.0):
            raise ValueError(
                f"--mpc {expected_label} prediction_model requires a "
                "positive directional command delay"
            )
        planner = OfflineConditionalVelocityLMPC(
            frozen_model,
            config=lmpc_config,
            safe_set_limits=safe_set_limits,
            stage_cost_spec=stage_cost_spec,
        )
        model_fingerprint = conditional_velocity_lmpc_fingerprint(
            frozen_model,
            lmpc_config,
            safe_set_limits,
            stage_cost_spec,
        )
        if planner.model_fingerprint != model_fingerprint:
            raise RuntimeError("conditional LMPC fingerprint implementation drift")
        contracts[expected_label] = {
            "schema_version": 1,
            "kind": "directional_conditional_velocity_lmpc_contract",
            "direction_label": expected_label,
            "direction_sign": direction_sign,
            "direction_xy": [0.0, float(direction_sign)],
            "command_delay_s": float(frozen_model.delay_s),
            "prediction_step_s": prediction_step,
            "model_fingerprint": model_fingerprint,
            "state_dimension": int(planner.state_dimension),
            "delay_steps": int(planner.delay_steps),
            "delay_remainder_s": float(planner.delay_remainder_s),
            "frozen_model": asdict(frozen_model),
            "conditional_velocity_lmpc_config": asdict(lmpc_config),
            "safe_set_limits": safe_set_limits.to_dict(),
            "stage_cost_spec": stage_cost_spec.to_dict(),
            "source_prediction_model_sha256": source_sha256,
            "source_prediction_model_schema_version": (
                prediction_model.get("schema_version")
            ),
            "source_prediction_model_kind": prediction_model.get("kind"),
        }
    return contracts


def validate_mpc_bootstrap_model_contracts(
        value, *, expected_prediction_step_s=None):
    """Validate stored contracts by rebuilding their exact fingerprints."""
    if not isinstance(value, dict) or set(value) != {
            "positive_y", "negative_y"}:
        raise ValueError(
            "--mpc requires positive_y and negative_y model contracts"
        )
    expected_step = (
        None if expected_prediction_step_s is None else
        _positive_float(
            expected_prediction_step_s, "expected_prediction_step_s"
        )
    )
    validated = {}
    for label, direction_sign in (("positive_y", 1), ("negative_y", -1)):
        contract = value[label]
        if not isinstance(contract, dict):
            raise ValueError(f"--mpc {label} model contract must be a mapping")
        try:
            if (
                contract["schema_version"] != 1
                or contract["kind"]
                != "directional_conditional_velocity_lmpc_contract"
                or contract["direction_label"] != label
                or contract["direction_sign"] != direction_sign
                or contract["direction_xy"] != [0.0, float(direction_sign)]
            ):
                raise ValueError("direction identity mismatch")
            frozen_model = FrozenTiltModel(**contract["frozen_model"])
            lmpc_config = ConditionalVelocityLMPCConfig(
                **contract["conditional_velocity_lmpc_config"]
            )
            lmpc_config.validate()
            safe_set_limits = SafeSetLimits.from_dict(
                contract["safe_set_limits"]
            )
            stage_cost_spec = StageCostSpec.from_dict(
                contract["stage_cost_spec"]
            )
            planner = OfflineConditionalVelocityLMPC(
                frozen_model,
                config=lmpc_config,
                safe_set_limits=safe_set_limits,
                stage_cost_spec=stage_cost_spec,
            )
            exact_fingerprint = conditional_velocity_lmpc_fingerprint(
                frozen_model,
                lmpc_config,
                safe_set_limits,
                stage_cost_spec,
            )
            command_delay = float(contract["command_delay_s"])
            prediction_step = float(contract["prediction_step_s"])
            valid = (
                math.isfinite(command_delay)
                and command_delay > 0.0
                and math.isclose(
                    command_delay,
                    float(frozen_model.delay_s),
                    rel_tol=0.0,
                    abs_tol=1e-12,
                )
                and math.isclose(
                    prediction_step,
                    float(lmpc_config.prediction_step_s),
                    rel_tol=0.0,
                    abs_tol=1e-12,
                )
                and (
                    expected_step is None
                    or math.isclose(
                        prediction_step,
                        expected_step,
                        rel_tol=0.0,
                        abs_tol=1e-12,
                    )
                )
                and contract["model_fingerprint"] == exact_fingerprint
                and contract["state_dimension"] == planner.state_dimension
                and contract["delay_steps"] == planner.delay_steps
                and math.isclose(
                    float(contract["delay_remainder_s"]),
                    planner.delay_remainder_s,
                    rel_tol=0.0,
                    abs_tol=1e-12,
                )
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f"--mpc {label} model contract is malformed"
            ) from error
        if not valid:
            raise ValueError(
                f"--mpc {label} model contract does not match its exact "
                "directional LMPC model/configuration"
            )
        validated[label] = deepcopy(contract)
    return validated


def mpc_bootstrap_model_contract_for_direction(contracts, direction_xy):
    """Select the already-frozen contract for one locked world-Y release."""
    direction = np.asarray(direction_xy, dtype=float)
    if (
        direction.shape != (2,)
        or not np.all(np.isfinite(direction))
        or abs(float(direction[0])) > 1e-9
        or abs(abs(float(direction[1]))-1.0) > 1e-9
    ):
        raise ValueError("--mpc release direction must be world +Y or -Y")
    validated = validate_mpc_bootstrap_model_contracts(contracts)
    label = "positive_y" if direction[1] > 0.0 else "negative_y"
    return deepcopy(validated[label])


@dataclass(frozen=True)
class MPCBootstrapEpisodeAssignment:
    episode_id: str
    direction_sign: int
    initial_speed_m_s: float
    cross_speed_m_s: float
    target_speed_m_s: float | None
    target_error_m_s: float | None
    countable: bool
    reason: str

    def to_dict(self):
        return {
            "episode_id": self.episode_id,
            "direction_sign": self.direction_sign,
            "initial_speed_m_s": self.initial_speed_m_s,
            "cross_speed_m_s": self.cross_speed_m_s,
            "target_speed_m_s": self.target_speed_m_s,
            "target_error_m_s": self.target_error_m_s,
            "countable": self.countable,
            "reason": self.reason,
        }


class MPCBootstrapCoverage:
    """Track successful cells without changing the flight controller.

    Every configured speed/sign cell is independent. ``current_target`` is an
    operator hint, not a gate: a valid faster release still counts even when a
    lower cell is incomplete. This lets short battery-bounded flights collect
    different subsets without silently discarding higher-speed evidence.
    """

    def __init__(self, config):
        if not isinstance(config, MPCBootstrapCalibrationConfig):
            raise TypeError("config must be MPCBootstrapCalibrationConfig")
        self.config = config
        self._successes = {
            (sign, speed): 0
            for speed in config.initial_speed_targets_m_s
            for sign in (-1, 1)
        }
        self._active = {}
        self._path_failures = {}
        self._attempts = []
        self.limits = SafeSetLimits()

    @property
    def current_target_speed_m_s(self):
        for speed in self.config.initial_speed_targets_m_s:
            if any(
                self._successes[(sign, speed)]
                < self.config.repetitions_per_cell
                for sign in (-1, 1)
            ):
                return speed
        return None

    @property
    def complete(self):
        return self.current_target_speed_m_s is None

    def begin(self, episode_id, direction_xy, velocity_xy):
        if not isinstance(episode_id, str) or not episode_id:
            raise ValueError("episode_id must be a non-empty string")
        if episode_id in self._active:
            raise ValueError("duplicate active MPC bootstrap episode")
        direction = np.asarray(direction_xy, dtype=float)
        velocity = np.asarray(velocity_xy, dtype=float)
        if (
            direction.shape != (2,)
            or velocity.shape != (2,)
            or not np.all(np.isfinite(direction))
            or not np.all(np.isfinite(velocity))
        ):
            raise ValueError("bootstrap direction and velocity must be finite XY")
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-9:
            raise ValueError("bootstrap direction cannot be zero")
        direction = direction/norm
        if abs(direction[0]) > 1e-6 or abs(abs(direction[1])-1.0) > 1e-6:
            raise ValueError("MPC bootstrap supports only world +Y/-Y releases")
        sign = int(np.sign(direction[1]))
        cross_direction = np.asarray([-float(sign), 0.0])
        initial_speed = float(velocity@direction)
        cross_speed = float(velocity@cross_direction)
        candidates = [
            (abs(initial_speed-target), target)
            for target in self.config.initial_speed_targets_m_s
            if abs(initial_speed-target) <= (
                self.config.speed_tolerance_m_s+1e-12
            )
        ]
        target = min(candidates)[1] if candidates else None
        target_error = None if target is None else initial_speed-target
        reason = "eligible"
        countable = True
        if self.complete:
            countable = False
            reason = "coverage_already_complete"
        elif initial_speed <= 0.0:
            countable = False
            reason = "nonpositive_release_aligned_speed"
        elif initial_speed > self.config.max_release_speed_m_s:
            countable = False
            reason = "release_speed_above_protocol_limit"
        elif abs(cross_speed) > self.config.max_cross_speed_m_s:
            countable = False
            reason = "cross_speed_above_protocol_limit"
        elif target is None:
            countable = False
            reason = "release_speed_outside_all_target_windows"
        elif (
            self._successes[(sign, target)]
            >= self.config.repetitions_per_cell
        ):
            countable = False
            reason = "direction_cell_already_complete"
        assignment = MPCBootstrapEpisodeAssignment(
            episode_id=episode_id,
            direction_sign=sign,
            initial_speed_m_s=initial_speed,
            cross_speed_m_s=cross_speed,
            target_speed_m_s=target,
            target_error_m_s=target_error,
            countable=countable,
            reason=reason,
        )
        self._active[episode_id] = assignment
        self._path_failures[episode_id] = []
        return assignment

    def observe(
            self, episode_id, *, velocity_xy_m_s, attitude_rp_rad,
            attitude_rate_rp_rad_s, boundary_margin_m, state_age_s,
            state_group_skew_s, measurement_rejected=False):
        """Keep a sticky audit of raw path violations for this attempt.

        This is deliberately only a provisional flight-side screen.  Passing
        it never admits an episode to the safe set; decision-time resampling
        and strict offline replay remain mandatory.
        """
        assignment = self._active.get(episode_id)
        if assignment is None:
            raise ValueError("unknown MPC bootstrap episode")
        velocity = np.asarray(velocity_xy_m_s, dtype=float)
        attitude = np.asarray(attitude_rp_rad, dtype=float)
        rates = np.asarray(attitude_rate_rp_rad_s, dtype=float)
        scalars = np.asarray([
            boundary_margin_m, state_age_s, state_group_skew_s,
        ], dtype=float)
        failures = []
        if (
            velocity.shape != (2,)
            or attitude.shape != (2,)
            or rates.shape != (2,)
            or not np.all(np.isfinite(velocity))
            or not np.all(np.isfinite(attitude))
            or not np.all(np.isfinite(rates))
            or not np.all(np.isfinite(scalars))
        ):
            failures.append("nonfinite_path_state")
        else:
            direction = np.asarray(
                [0.0, float(assignment.direction_sign)], dtype=float
            )
            cross_direction = np.asarray([
                -float(assignment.direction_sign), 0.0,
            ])
            aligned_speed = float(velocity@direction)
            cross_speed = float(velocity@cross_direction)
            if aligned_speed < -self.limits.reverse_velocity_tolerance_m_s:
                failures.append("reverse_velocity_path_violation")
            if abs(aligned_speed) > self.limits.max_abs_aligned_velocity_m_s:
                failures.append("aligned_velocity_path_violation")
            if abs(cross_speed) > min(
                    self.config.max_cross_speed_m_s,
                    self.limits.max_abs_cross_velocity_m_s):
                failures.append("cross_velocity_path_violation")
            if float(np.max(np.abs(attitude))) > self.limits.max_path_tilt_rad:
                failures.append("attitude_path_violation")
            if float(np.max(np.abs(rates))) > self.limits.max_path_rate_rad_s:
                failures.append("attitude_rate_path_violation")
            if boundary_margin_m < self.limits.min_path_boundary_margin_m:
                failures.append("boundary_margin_path_violation")
            if not 0.0 <= state_age_s <= self.limits.max_state_age_s:
                failures.append("state_age_path_violation")
            if not 0.0 <= state_group_skew_s <= (
                    self.limits.max_state_group_skew_s):
                failures.append("state_group_skew_path_violation")
        if bool(measurement_rejected):
            failures.append("measurement_rejected_path_violation")
        sticky = self._path_failures[episode_id]
        for failure in failures:
            if failure not in sticky:
                sticky.append(failure)
        return tuple(sticky)

    def close(self, episode_id, *, terminal_success, reason):
        assignment = self._active.pop(episode_id, None)
        if assignment is None:
            raise ValueError("unknown MPC bootstrap episode")
        path_failures = tuple(self._path_failures.pop(episode_id, ()))
        path_eligible = not path_failures
        counted = bool(
            terminal_success and assignment.countable and path_eligible
        )
        if counted:
            key = (assignment.direction_sign, assignment.target_speed_m_s)
            self._successes[key] += 1
        result = {
            **assignment.to_dict(),
            "terminal_success": bool(terminal_success),
            "provisional_path_eligible": path_eligible,
            "path_failure_reasons": list(path_failures),
            "counted": counted,
            "close_reason": str(reason),
        }
        self._attempts.append(result)
        return result

    def mark_path_failure(self, episode_id, reason):
        """Make an externally audited protocol failure sticky for a path."""
        if episode_id not in self._active:
            raise ValueError("unknown MPC bootstrap episode")
        if not isinstance(reason, str) or not reason:
            raise ValueError("path failure reason must be a non-empty string")
        sticky = self._path_failures[episode_id]
        if reason not in sticky:
            sticky.append(reason)
        return tuple(sticky)

    def summary(self):
        cells = []
        for speed in self.config.initial_speed_targets_m_s:
            for sign in (-1, 1):
                count = self._successes[(sign, speed)]
                cells.append({
                    "direction_sign": sign,
                    "target_speed_m_s": speed,
                    "success_count": count,
                    "required_count": self.config.repetitions_per_cell,
                    "complete": count >= self.config.repetitions_per_cell,
                })
        return {
            "offline_only": True,
            "provisional_raw_collection_coverage": True,
            "requires_offline_resampling_and_replay": True,
            "command_authority": "legacy_attitude_coast_only",
            "lmpc_command_authority": False,
            "complete": self.complete,
            "current_target_speed_m_s": self.current_target_speed_m_s,
            "cells": cells,
            "attempt_count": len(self._attempts)+len(self._active),
            "closed_attempt_count": len(self._attempts),
            "active_episode_ids": sorted(self._active),
        }


def configure_mpc_bootstrap_mission(mission):
    """Return a private mission copy for raw LMPC bootstrap collection."""

    configured = deepcopy(mission)
    interaction = configured.setdefault("Interaction", {})
    if interaction.get("action", "translation") != "translation":
        raise ValueError("--mpc requires Interaction.action: translation")
    interaction["action"] = "translation"
    translation = interaction.setdefault("config", {})
    translation["detection_method"] = "momentum_impulse"
    virtual = translation.setdefault("virtual_object", {})
    virtual["inertia_command"] = "orientation"
    virtual.setdefault("force_rendering", {})["enabled"] = True
    virtual.setdefault("contact_detection", {})["source"] = "potentiometer"
    virtual.setdefault("release_behavior", {})["mode"] = "potentiometer_coast"
    virtual.setdefault("two_afc_friction", {})["enabled"] = False

    wrench = translation.setdefault("wrench_interaction", {})
    wrench["state_source"] = "onboard"
    wrench["shadow_mode"] = False
    wrench["startup_bias_calibration_enabled"] = False
    bootstrap = wrench.setdefault("mpc_bootstrap_calibration", {})
    bootstrap["enabled"] = True
    # Validate the overlay structure here; the controller performs the full
    # calibration-file/geometry check before arming. Preserve selected targets.
    MPCBootstrapCalibrationConfig.from_mapping(bootstrap)

    wrench.setdefault("calibration_excitation", {})["enabled"] = False
    wrench.setdefault("planar_braking_calibration", {})["enabled"] = False
    wrench.setdefault("adaptive_braking_calibration", {})["enabled"] = False
    wrench.setdefault("online_prediction_calibration", {})["enabled"] = False
    wrench.setdefault("predictive_braking", {})["enabled"] = False
    lmpc = wrench.setdefault("learning_velocity_mpc_shadow", {})
    lmpc["enabled"] = False
    lmpc["command_authority"] = False
    lmpc["direction_xy"] = None
    handoff = wrench.setdefault("control_handoff", {})
    handoff["coast_velocity_braking_enabled"] = False
    handoff["coast_velocity_predictive_unwind_enabled"] = False
    handoff["coast_direct_position_handoff"] = False
    return configured


def prepare_mpc_bootstrap_mission(
        mission, *, drone_id, sense_axis, controller_rate_hz):
    """Build and fully validate the private mission before arming.

    In addition to the mode overlay, this verifies the world-Y sensor geometry,
    clear calibration volume, and the existing bounded baseline dynamics that
    own the bootstrap flight.  No calibration file is written.
    """
    if sense_axis != "y":
        raise ValueError("--mpc requires --sense-axis y")
    configured = configure_mpc_bootstrap_mission(mission)
    try:
        translation = configured["Interaction"]["config"]
        drone_target = list(configured["drones"][str(drone_id)]["target"])
    except (KeyError, TypeError) as error:
        raise ValueError("--mpc mission does not contain the selected drone") from error
    if len(drone_target) < 3:
        raise ValueError("--mpc drone target must contain XYZ")
    wrench = translation["wrench_interaction"]
    controller_rate = _positive_float(
        controller_rate_hz, "controller_rate_hz"
    )
    bootstrap_config = MPCBootstrapCalibrationConfig.from_mapping(
        wrench.get("mpc_bootstrap_calibration")
    )
    decision_ratio = bootstrap_config.prediction_step_s*controller_rate
    if (
        decision_ratio < 2.0-1e-12
        or not math.isclose(
            decision_ratio,
            round(decision_ratio),
            rel_tol=0.0,
            abs_tol=1e-12,
        )
    ):
        raise ValueError(
            "--mpc prediction_step_s must be an integral multiple of at "
            "least two controller periods"
        )
    target = list(wrench.get("calibration_nominal_position", drone_target[:3]))
    if len(target) != 3 or not np.all(np.isfinite(np.asarray(target, dtype=float))):
        raise ValueError("--mpc calibration_nominal_position must be finite XYZ")
    nominal_yaw_deg = float(
        drone_target[3]
        if len(drone_target) > 3 else wrench.get("nominal_yaw_deg", 0.0)
    )
    if not math.isfinite(nominal_yaw_deg) or abs(
            math.sin(math.radians(nominal_yaw_deg))) > 1e-6:
        raise ValueError(
            "--mpc requires body Y aligned with the world-Y axis at nominal yaw"
        )
    bounds = configured.get("boundary_limits")
    if not isinstance(bounds, dict):
        raise ValueError("--mpc requires explicit boundary_limits")
    try:
        margin = min(
            target[0]-float(bounds["x_min"]),
            float(bounds["x_max"])-target[0],
            target[1]-float(bounds["y_min"]),
            float(bounds["y_max"])-target[1],
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("--mpc boundary_limits must contain finite XY bounds") from error
    if not math.isfinite(margin) or margin < 0.30:
        raise ValueError(
            "--mpc needs at least 0.30 m XY boundary margin around "
            "calibration_nominal_position"
        )

    calibration_path = translation.get(
        "wrench_calibration_file", str(DEFAULT_CALIBRATION_PATH)
    )
    resolved, saved = apply_drone_calibration(
        wrench,
        drone_id,
        calibration_path,
        runtime_interaction_direction_xy=[0.0, 1.0],
    )
    saved_fit = None if saved is None else saved.get("planar_braking_fit")
    if not planar_braking_fit_is_current(saved_fit):
        raise ValueError(
            "--mpc requires a current quality-gated planar braking "
            "calibration; run --calibrate first"
        )
    try:
        saved_delay_s = float(saved_fit["command_delay_s"])
        runtime_delay_s = float(
            resolved["control_handoff"][
                "coast_attitude_response_delay_s"
            ]
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            "--mpc baseline calibration has no usable command delay; "
            "rerun --calibrate"
        ) from error
    if (
        not np.all(np.isfinite([saved_delay_s, runtime_delay_s]))
        or saved_delay_s <= 0.0
        or runtime_delay_s <= 0.0
        or not math.isclose(
            saved_delay_s,
            runtime_delay_s,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
    ):
        raise ValueError(
            "--mpc requires a positive fitted command delay that matches "
            "the runtime baseline; rerun --calibrate"
        )
    prediction_model = None if saved is None else saved.get("prediction_model")
    contracts = build_mpc_bootstrap_model_contracts(
        prediction_model,
        prediction_step_s=bootstrap_config.prediction_step_s,
    )
    # These learned timing contracts are logging/offline-replay metadata only.
    # The planar fit above remains the legacy attitude-coast flight authority.
    resolved["mpc_bootstrap_model_contracts"] = contracts
    configured["Interaction"]["config"]["wrench_interaction"] = resolved
    return configured
