"""Configuration and coverage bookkeeping for LMPC bootstrap flights.

The bootstrap is deliberately a data-collection mode.  It never evaluates an
LMPC policy and never grants an optimizer command authority.  The normal,
bounded attitude coast controller stops the vehicle after each release.  A
small pure state machine can generate the release automatically with bounded
attitude commands; it remains separate from both Crazyflie I/O and the LMPC
policy.  This module also freezes the private mission overlay and records which
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
    automatic_acceleration_tilt_deg: float = 8.0
    automatic_release_tolerance_m_s: float = 0.02
    ready_speed_tolerance_m_s: float = 0.03
    ready_position_tolerance_m: float = 0.05
    ready_z_tolerance_m: float = 0.05
    ready_dwell_s: float = 0.30
    ready_timeout_s: float = 5.0
    level_warmup_min_s: float = 0.10
    max_sample_gap_s: float = 0.10
    max_acceleration_duration_s: float = 0.85
    max_maneuver_displacement_m: float = 0.60
    boundary_reserve_m: float = 0.10
    max_automatic_tilt_deg: float = 12.0
    max_automatic_rate_deg_s: float = 100.0
    wrong_way_land_speed_m_s: float = 0.05

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
            "automatic_acceleration_tilt_deg",
            "automatic_release_tolerance_m_s",
            "ready_speed_tolerance_m_s",
            "ready_position_tolerance_m",
            "ready_z_tolerance_m",
            "ready_dwell_s",
            "ready_timeout_s",
            "level_warmup_min_s",
            "max_sample_gap_s",
            "max_acceleration_duration_s",
            "max_maneuver_displacement_m",
            "boundary_reserve_m",
            "max_automatic_tilt_deg",
            "max_automatic_rate_deg_s",
            "wrong_way_land_speed_m_s",
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
        if cross > 0.03+1e-12:
            raise ValueError("max_cross_speed_m_s cannot exceed 0.03 m/s")
        maximum = _positive_float(
            value.get("max_release_speed_m_s", cls.max_release_speed_m_s),
            "max_release_speed_m_s",
        )
        if maximum > 0.75+1e-12:
            raise ValueError("max_release_speed_m_s cannot exceed 0.75 m/s")
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
        acceleration_tilt = _positive_float(
            value.get(
                "automatic_acceleration_tilt_deg",
                cls.automatic_acceleration_tilt_deg,
            ),
            "automatic_acceleration_tilt_deg",
        )
        if acceleration_tilt > 8.0:
            raise ValueError(
                "automatic_acceleration_tilt_deg must stay inside the "
                "8 degree calibrated envelope"
            )
        release_tolerance = _positive_float(
            value.get(
                "automatic_release_tolerance_m_s",
                cls.automatic_release_tolerance_m_s,
            ),
            "automatic_release_tolerance_m_s",
        )
        if release_tolerance > tolerance:
            raise ValueError(
                "automatic_release_tolerance_m_s cannot exceed "
                "speed_tolerance_m_s"
            )
        if release_tolerance > 0.02+1e-12:
            raise ValueError(
                "automatic_release_tolerance_m_s cannot exceed 0.02 m/s"
            )
        ready_speed = _positive_float(
            value.get(
                "ready_speed_tolerance_m_s", cls.ready_speed_tolerance_m_s
            ),
            "ready_speed_tolerance_m_s",
        )
        if ready_speed > cross:
            raise ValueError(
                "ready_speed_tolerance_m_s cannot exceed "
                "max_cross_speed_m_s"
            )
        ready_position = _positive_float(
            value.get(
                "ready_position_tolerance_m",
                cls.ready_position_tolerance_m,
            ),
            "ready_position_tolerance_m",
        )
        ready_z = _positive_float(
            value.get("ready_z_tolerance_m", cls.ready_z_tolerance_m),
            "ready_z_tolerance_m",
        )
        if ready_z > 0.05+1e-12:
            raise ValueError("ready_z_tolerance_m cannot exceed 0.05 m")
        ready_dwell = _positive_float(
            value.get("ready_dwell_s", cls.ready_dwell_s),
            "ready_dwell_s",
        )
        ready_timeout = _positive_float(
            value.get("ready_timeout_s", cls.ready_timeout_s),
            "ready_timeout_s",
        )
        if ready_timeout > 5.0+1e-12:
            raise ValueError("ready_timeout_s cannot exceed 5.0 s")
        level_warmup = _positive_float(
            value.get("level_warmup_min_s", cls.level_warmup_min_s),
            "level_warmup_min_s",
        )
        if level_warmup < prediction_step-1e-12:
            raise ValueError(
                "level_warmup_min_s must cover at least one prediction step"
            )
        max_sample_gap = _positive_float(
            value.get("max_sample_gap_s", cls.max_sample_gap_s),
            "max_sample_gap_s",
        )
        if not prediction_step <= max_sample_gap <= 0.10:
            raise ValueError(
                "max_sample_gap_s must cover prediction_step_s and cannot "
                "exceed 0.10 s"
            )
        acceleration_timeout = _positive_float(
            value.get(
                "max_acceleration_duration_s",
                cls.max_acceleration_duration_s,
            ),
            "max_acceleration_duration_s",
        )
        if acceleration_timeout > 0.85+1e-12:
            raise ValueError(
                "max_acceleration_duration_s cannot exceed 0.85 s"
            )
        displacement = _positive_float(
            value.get(
                "max_maneuver_displacement_m",
                cls.max_maneuver_displacement_m,
            ),
            "max_maneuver_displacement_m",
        )
        if displacement <= ready_position:
            raise ValueError(
                "max_maneuver_displacement_m must exceed "
                "ready_position_tolerance_m"
            )
        if displacement > 0.60+1e-12:
            raise ValueError(
                "max_maneuver_displacement_m cannot exceed 0.60 m"
            )
        boundary_reserve = _positive_float(
            value.get("boundary_reserve_m", cls.boundary_reserve_m),
            "boundary_reserve_m",
        )
        safe_limits = SafeSetLimits()
        if boundary_reserve < 0.10-1e-12:
            raise ValueError(
                "boundary_reserve_m cannot be smaller than the 0.10 m "
                "automatic-maneuver reserve"
            )
        max_automatic_tilt = _positive_float(
            value.get(
                "max_automatic_tilt_deg", cls.max_automatic_tilt_deg
            ),
            "max_automatic_tilt_deg",
        )
        if not acceleration_tilt <= max_automatic_tilt <= 12.0:
            raise ValueError(
                "max_automatic_tilt_deg must cover the acceleration command "
                "and cannot exceed 12 degrees"
            )
        max_automatic_rate = _positive_float(
            value.get(
                "max_automatic_rate_deg_s", cls.max_automatic_rate_deg_s
            ),
            "max_automatic_rate_deg_s",
        )
        if max_automatic_rate > 100.0+1e-12:
            raise ValueError(
                "max_automatic_rate_deg_s cannot exceed 100 deg/s"
            )
        wrong_way_land_speed = _positive_float(
            value.get(
                "wrong_way_land_speed_m_s", cls.wrong_way_land_speed_m_s
            ),
            "wrong_way_land_speed_m_s",
        )
        if wrong_way_land_speed < (
                safe_limits.reverse_velocity_tolerance_m_s-1e-12):
            raise ValueError(
                "wrong_way_land_speed_m_s must cover the reverse-velocity "
                "tolerance"
            )
        if wrong_way_land_speed > 0.05+1e-12:
            raise ValueError(
                "wrong_way_land_speed_m_s cannot exceed 0.05 m/s"
            )
        return cls(
            initial_speed_targets_m_s=targets,
            speed_tolerance_m_s=tolerance,
            repetitions_per_cell=repetitions,
            max_cross_speed_m_s=cross,
            max_release_speed_m_s=maximum,
            prediction_step_s=prediction_step,
            automatic_acceleration_tilt_deg=acceleration_tilt,
            automatic_release_tolerance_m_s=release_tolerance,
            ready_speed_tolerance_m_s=ready_speed,
            ready_position_tolerance_m=ready_position,
            ready_z_tolerance_m=ready_z,
            ready_dwell_s=ready_dwell,
            ready_timeout_s=ready_timeout,
            level_warmup_min_s=level_warmup,
            max_sample_gap_s=max_sample_gap,
            max_acceleration_duration_s=acceleration_timeout,
            max_maneuver_displacement_m=displacement,
            boundary_reserve_m=boundary_reserve,
            max_automatic_tilt_deg=max_automatic_tilt,
            max_automatic_rate_deg_s=max_automatic_rate,
            wrong_way_land_speed_m_s=wrong_way_land_speed,
        )

    def to_dict(self):
        return {
            "initial_speed_targets_m_s": list(self.initial_speed_targets_m_s),
            "speed_tolerance_m_s": self.speed_tolerance_m_s,
            "repetitions_per_cell": self.repetitions_per_cell,
            "max_cross_speed_m_s": self.max_cross_speed_m_s,
            "max_release_speed_m_s": self.max_release_speed_m_s,
            "prediction_step_s": self.prediction_step_s,
            "automatic_acceleration_tilt_deg": (
                self.automatic_acceleration_tilt_deg
            ),
            "automatic_release_tolerance_m_s": (
                self.automatic_release_tolerance_m_s
            ),
            "ready_speed_tolerance_m_s": self.ready_speed_tolerance_m_s,
            "ready_position_tolerance_m": self.ready_position_tolerance_m,
            "ready_z_tolerance_m": self.ready_z_tolerance_m,
            "ready_dwell_s": self.ready_dwell_s,
            "ready_timeout_s": self.ready_timeout_s,
            "level_warmup_min_s": self.level_warmup_min_s,
            "max_sample_gap_s": self.max_sample_gap_s,
            "max_acceleration_duration_s": (
                self.max_acceleration_duration_s
            ),
            "max_maneuver_displacement_m": (
                self.max_maneuver_displacement_m
            ),
            "boundary_reserve_m": self.boundary_reserve_m,
            "max_automatic_tilt_deg": self.max_automatic_tilt_deg,
            "max_automatic_rate_deg_s": self.max_automatic_rate_deg_s,
            "wrong_way_land_speed_m_s": self.wrong_way_land_speed_m_s,
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


def mpc_bootstrap_required_boundary_margin_m(config):
    """Return the pre-arm XY clearance reserved for one full maneuver."""
    if not isinstance(config, MPCBootstrapCalibrationConfig):
        raise TypeError("config must be MPCBootstrapCalibrationConfig")
    return (
        config.max_maneuver_displacement_m+config.boundary_reserve_m
    )


def mpc_bootstrap_acceleration_attitude_deg(
        direction_sign, tilt_deg, nominal_yaw_deg=0.0):
    """Map a world-Y acceleration into Crazyflie roll/pitch commands.

    Positive world-Y acceleration is negative roll at zero yaw.  Keeping this
    mapping in the pure protocol makes the sign convention directly testable
    without arming a vehicle.
    """
    if (
        isinstance(direction_sign, bool)
        or not isinstance(direction_sign, numbers.Integral)
        or int(direction_sign) not in (-1, 1)
    ):
        raise ValueError("direction_sign must be +1 or -1")
    tilt = _positive_float(tilt_deg, "tilt_deg")
    yaw = float(nominal_yaw_deg)
    if not math.isfinite(yaw):
        raise ValueError("nominal_yaw_deg must be finite")
    yaw_rad = math.radians(yaw)
    world_acceleration = np.asarray([0.0, float(direction_sign)])
    body_x = (
        world_acceleration[0]*math.cos(yaw_rad)
        + world_acceleration[1]*math.sin(yaw_rad)
    )
    body_y = (
        -world_acceleration[0]*math.sin(yaw_rad)
        + world_acceleration[1]*math.cos(yaw_rad)
    )
    pitch_deg = -tilt*body_x
    roll_deg = -tilt*body_y
    return float(roll_deg), float(pitch_deg)


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
class MPCBootstrapTargetCell:
    """One automatically scheduled world-Y release-speed cell."""

    direction_sign: int
    target_speed_m_s: float
    success_count: int
    required_count: int

    @property
    def direction_xy(self):
        return (0.0, float(self.direction_sign))

    @property
    def key(self):
        return self.direction_sign, self.target_speed_m_s

    def to_dict(self):
        return {
            "direction_sign": self.direction_sign,
            "direction_xy": list(self.direction_xy),
            "target_speed_m_s": self.target_speed_m_s,
            "success_count": self.success_count,
            "required_count": self.required_count,
        }


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

    def next_required_cell(self, previous_cell=None):
        """Choose the least-sampled sign in the lowest incomplete speed tier.

        Both signs and all repetitions at a lower speed are completed before
        the vehicle is exposed to a higher-speed maneuver.  Within that tier,
        equal-count signs alternate.  A failed attempt therefore stays
        required but, when possible, the opposite sign runs before its retry.
        """
        if self.complete:
            return None
        speed = next(
            speed for speed in self.config.initial_speed_targets_m_s
            if any(
                self._successes[(sign, speed)]
                < self.config.repetitions_per_cell
                for sign in (1, -1)
            )
        )
        ordered_keys = [(sign, speed) for sign in (1, -1)]
        minimum_count = min(
            self._successes[key]
            for key in ordered_keys
            if self._successes[key] < self.config.repetitions_per_cell
        )
        eligible = [
            key for key in ordered_keys
            if self._successes[key] == minimum_count
            and self._successes[key] < self.config.repetitions_per_cell
        ]
        previous_key = None
        if previous_cell is not None:
            if isinstance(previous_cell, MPCBootstrapTargetCell):
                previous_key = previous_cell.key
            elif (
                isinstance(previous_cell, (tuple, list))
                and len(previous_cell) == 2
            ):
                previous_key = (int(previous_cell[0]), float(previous_cell[1]))
            else:
                raise TypeError(
                    "previous_cell must be MPCBootstrapTargetCell or "
                    "(direction_sign, target_speed_m_s)"
                )
        if previous_key in eligible and len(eligible) > 1:
            index = eligible.index(previous_key)
            eligible = eligible[index+1:]+eligible[:index+1]
        sign, speed = eligible[0]
        return MPCBootstrapTargetCell(
            direction_sign=sign,
            target_speed_m_s=speed,
            success_count=self._successes[(sign, speed)],
            required_count=self.config.repetitions_per_cell,
        )

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
            "command_authority": (
                "automatic_attitude_acceleration_then_legacy_attitude_coast"
            ),
            "lmpc_command_authority": False,
            "complete": self.complete,
            "current_target_speed_m_s": self.current_target_speed_m_s,
            "cells": cells,
            "attempt_count": len(self._attempts)+len(self._active),
            "closed_attempt_count": len(self._attempts),
            "active_episode_ids": sorted(self._active),
        }


@dataclass(frozen=True)
class MPCBootstrapAutomaticDecision:
    """One deterministic output from the automatic-release state machine."""

    time_s: float | None
    phase: str
    command_kind: str
    direction_sign: int
    direction_xy: tuple[float, float]
    target_speed_m_s: float
    command_roll_deg: float | None
    command_pitch_deg: float | None
    start_braking: bool
    abort_requested: bool
    abort_action: str | None
    prelude_failure_reasons: tuple[str, ...]
    velocity_xy_m_s: tuple[float, float] | None
    aligned_speed_m_s: float | None
    cross_speed_m_s: float | None
    displacement_from_nominal_m: float | None
    z_error_m: float | None
    sample_gap_s: float | None
    ready_elapsed_s: float
    level_warmup_elapsed_s: float
    level_warmup_required_s: float
    acceleration_elapsed_s: float

    def to_dict(self):
        return {
            "time_s": self.time_s,
            "phase": self.phase,
            "command_kind": self.command_kind,
            "direction_sign": self.direction_sign,
            "direction_xy": list(self.direction_xy),
            "target_speed_m_s": self.target_speed_m_s,
            "command_roll_deg": self.command_roll_deg,
            "command_pitch_deg": self.command_pitch_deg,
            "start_braking": self.start_braking,
            "abort_requested": self.abort_requested,
            "abort_action": self.abort_action,
            "prelude_failure_reasons": list(
                self.prelude_failure_reasons
            ),
            "velocity_xy_m_s": (
                None
                if self.velocity_xy_m_s is None
                else list(self.velocity_xy_m_s)
            ),
            "aligned_speed_m_s": self.aligned_speed_m_s,
            "cross_speed_m_s": self.cross_speed_m_s,
            "displacement_from_nominal_m": (
                self.displacement_from_nominal_m
            ),
            "z_error_m": self.z_error_m,
            "sample_gap_s": self.sample_gap_s,
            "ready_elapsed_s": self.ready_elapsed_s,
            "level_warmup_elapsed_s": self.level_warmup_elapsed_s,
            "level_warmup_required_s": self.level_warmup_required_s,
            "acceleration_elapsed_s": self.acceleration_elapsed_s,
        }


class MPCBootstrapAutomaticAttempt:
    """Pure fail-closed state machine for one automatic release attempt.

    The state machine owns no I/O.  Its outputs intentionally distinguish
    position hold, level attitude warm-up, automatic attitude acceleration,
    and handoff to the existing legacy attitude-coast brake.  The caller must
    send the requested command kind and must never give an LMPC optimizer
    command authority during this collection protocol.
    """

    READY_DWELL = "ready_dwell"
    LEVEL_WARMUP = "level_warmup"
    ACCELERATING = "accelerating"
    BRAKING = "braking"
    ABORTED = "aborted"
    FINISHED = "finished"

    def __init__(
            self, config, target_cell, *, nominal_position_xy_m,
            directional_model_delay_s, nominal_yaw_deg=0.0):
        if not isinstance(config, MPCBootstrapCalibrationConfig):
            raise TypeError("config must be MPCBootstrapCalibrationConfig")
        if not isinstance(target_cell, MPCBootstrapTargetCell):
            raise TypeError("target_cell must be MPCBootstrapTargetCell")
        if target_cell.direction_sign not in (-1, 1):
            raise ValueError("target cell direction must be +Y or -Y")
        if not any(
            math.isclose(
                target_cell.target_speed_m_s,
                target,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
            for target in config.initial_speed_targets_m_s
        ):
            raise ValueError("target cell speed is not in the protocol")
        nominal = np.asarray(nominal_position_xy_m, dtype=float)
        if nominal.shape != (2,) or not np.all(np.isfinite(nominal)):
            raise ValueError("nominal_position_xy_m must be finite XY")
        model_delay = _positive_float(
            directional_model_delay_s, "directional_model_delay_s"
        )
        yaw = float(nominal_yaw_deg)
        if not math.isfinite(yaw):
            raise ValueError("nominal_yaw_deg must be finite")
        self.config = config
        self.target_cell = target_cell
        self.nominal_position_xy_m = nominal.copy()
        self.directional_model_delay_s = model_delay
        self.nominal_yaw_deg = yaw
        self.limits = SafeSetLimits()
        self.level_warmup_required_s = max(
            config.level_warmup_min_s,
            model_delay+config.prediction_step_s,
        )
        self.phase = self.READY_DWELL
        self._ready_since_s = None
        self._attempt_started_s = None
        self._level_warmup_started_s = None
        self._acceleration_started_s = None
        self._last_time_s = None
        self._prelude_failure_reasons = []
        self._last_decision = None

    @property
    def prelude_failure_reasons(self):
        return tuple(self._prelude_failure_reasons)

    @property
    def direction_xy(self):
        return self.target_cell.direction_xy

    @property
    def target_speed_m_s(self):
        return self.target_cell.target_speed_m_s

    def _elapsed(self, now_s, started_s):
        if started_s is None or now_s is None:
            return 0.0
        return max(0.0, float(now_s)-float(started_s))

    def _decision(
            self, *, now_s, command_kind, metrics=None,
            command_roll_deg=None, command_pitch_deg=None,
            start_braking=False, abort_requested=False,
            abort_action=None):
        metrics = dict(metrics or {})
        decision = MPCBootstrapAutomaticDecision(
            time_s=now_s,
            phase=self.phase,
            command_kind=command_kind,
            direction_sign=self.target_cell.direction_sign,
            direction_xy=self.target_cell.direction_xy,
            target_speed_m_s=self.target_cell.target_speed_m_s,
            command_roll_deg=command_roll_deg,
            command_pitch_deg=command_pitch_deg,
            start_braking=bool(start_braking),
            abort_requested=bool(abort_requested),
            abort_action=abort_action,
            prelude_failure_reasons=tuple(
                self._prelude_failure_reasons
            ),
            velocity_xy_m_s=metrics.get("velocity_xy_m_s"),
            aligned_speed_m_s=metrics.get("aligned_speed_m_s"),
            cross_speed_m_s=metrics.get("cross_speed_m_s"),
            displacement_from_nominal_m=metrics.get(
                "displacement_from_nominal_m"
            ),
            z_error_m=metrics.get("z_error_m"),
            sample_gap_s=metrics.get("sample_gap_s"),
            ready_elapsed_s=self._elapsed(now_s, self._ready_since_s),
            level_warmup_elapsed_s=self._elapsed(
                now_s, self._level_warmup_started_s
            ),
            level_warmup_required_s=self.level_warmup_required_s,
            acceleration_elapsed_s=self._elapsed(
                now_s, self._acceleration_started_s
            ),
        )
        self._last_decision = decision
        return decision

    def _mark_failures(self, reasons):
        for reason in reasons:
            if reason not in self._prelude_failure_reasons:
                self._prelude_failure_reasons.append(reason)

    def _abort(self, *, now_s, metrics, reasons, state_trustworthy):
        self._mark_failures(reasons)
        self.phase = self.ABORTED
        if state_trustworthy:
            command_kind = "abort_to_legacy_attitude_coast"
            abort_action = "legacy_attitude_coast_to_rest"
        else:
            command_kind = "abort_level_and_land"
            abort_action = "level_and_land"
        return self._decision(
            now_s=now_s,
            command_kind=command_kind,
            metrics=metrics,
            abort_requested=True,
            abort_action=abort_action,
        )

    def _acceleration_decision(self, now_s, metrics):
        roll_deg, pitch_deg = mpc_bootstrap_acceleration_attitude_deg(
            self.target_cell.direction_sign,
            self.config.automatic_acceleration_tilt_deg,
            self.nominal_yaw_deg,
        )
        return self._decision(
            now_s=now_s,
            command_kind="automatic_acceleration_attitude_zdistance",
            metrics=metrics,
            command_roll_deg=roll_deg,
            command_pitch_deg=pitch_deg,
        )

    def observe(
            self, *, time_s, position_xy_m, velocity_xy_m_s,
            attitude_rp_rad, attitude_rate_rp_rad_s, boundary_margin_m,
            state_age_s, state_group_skew_s, z_error_m,
            sample_gap_s=None, measurement_rejected=False):
        """Advance from one fresh, measured controller state.

        ``start_braking`` is emitted only on the first fresh sample inside the
        requested speed window.  Once any prelude limit fails, the attempt is
        permanently aborted and can never later emit that transition.
        """
        if self.phase in (self.ABORTED, self.FINISHED):
            return self._last_decision
        if self.phase == self.BRAKING:
            return self._decision(
                now_s=self._last_decision.time_s,
                command_kind="continue_legacy_attitude_coast",
                metrics={
                    "velocity_xy_m_s": self._last_decision.velocity_xy_m_s,
                    "aligned_speed_m_s": self._last_decision.aligned_speed_m_s,
                    "cross_speed_m_s": self._last_decision.cross_speed_m_s,
                    "displacement_from_nominal_m": (
                        self._last_decision.displacement_from_nominal_m
                    ),
                    "z_error_m": self._last_decision.z_error_m,
                    "sample_gap_s": self._last_decision.sample_gap_s,
                },
            )

        try:
            now_s = float(time_s)
            position = np.asarray(position_xy_m, dtype=float)
            velocity = np.asarray(velocity_xy_m_s, dtype=float)
            attitude = np.asarray(attitude_rp_rad, dtype=float)
            rates = np.asarray(attitude_rate_rp_rad_s, dtype=float)
            boundary_margin = float(boundary_margin_m)
            state_age = float(state_age_s)
            state_skew = float(state_group_skew_s)
            z_error = float(z_error_m)
            sample_gap = (
                None if sample_gap_s is None else float(sample_gap_s)
            )
        except (TypeError, ValueError):
            return self._abort(
                now_s=None,
                metrics={},
                reasons=("nonfinite_automatic_prelude_state",),
                state_trustworthy=False,
            )
        finite_state = (
            math.isfinite(now_s)
            and position.shape == (2,)
            and velocity.shape == (2,)
            and attitude.shape == (2,)
            and rates.shape == (2,)
            and np.all(np.isfinite(position))
            and np.all(np.isfinite(velocity))
            and np.all(np.isfinite(attitude))
            and np.all(np.isfinite(rates))
            and np.all(np.isfinite([
                boundary_margin, state_age, state_skew, z_error,
            ]))
            and (sample_gap is None or math.isfinite(sample_gap))
        )
        if not finite_state:
            return self._abort(
                now_s=now_s if math.isfinite(now_s) else None,
                metrics={},
                reasons=("nonfinite_automatic_prelude_state",),
                state_trustworthy=False,
            )

        direction = np.asarray(self.target_cell.direction_xy)
        cross_direction = np.asarray([
            -float(self.target_cell.direction_sign), 0.0,
        ])
        aligned_speed = float(velocity@direction)
        cross_speed = float(velocity@cross_direction)
        displacement = float(np.linalg.norm(
            position-self.nominal_position_xy_m
        ))
        if sample_gap is None:
            sample_gap = (
                0.0 if self._last_time_s is None
                else now_s-self._last_time_s
            )
        metrics = {
            "velocity_xy_m_s": tuple(float(value) for value in velocity),
            "aligned_speed_m_s": aligned_speed,
            "cross_speed_m_s": cross_speed,
            "displacement_from_nominal_m": displacement,
            "z_error_m": z_error,
            "sample_gap_s": sample_gap,
        }
        common_failures = []
        if self._last_time_s is not None and now_s < self._last_time_s-1e-12:
            common_failures.append("nonmonotonic_automatic_protocol_time")
        if not 0.0 <= state_age <= self.limits.max_state_age_s+1e-12:
            common_failures.append(
                "state_age_automatic_prelude_violation"
            )
        if not 0.0 <= state_skew <= (
                self.limits.max_state_group_skew_s+1e-12):
            common_failures.append(
                "state_group_skew_automatic_prelude_violation"
            )
        if bool(measurement_rejected):
            common_failures.append(
                "measurement_rejected_automatic_prelude_violation"
            )
        if not 0.0 <= sample_gap <= self.config.max_sample_gap_s+1e-12:
            common_failures.append(
                "sample_gap_automatic_prelude_violation"
            )
        self._last_time_s = now_s
        if self._attempt_started_s is None:
            self._attempt_started_s = now_s
        if common_failures:
            return self._abort(
                now_s=now_s,
                metrics=metrics,
                reasons=common_failures,
                state_trustworthy=False,
            )

        max_tilt = float(np.max(np.abs(attitude)))
        max_rate = float(np.max(np.abs(rates)))
        if self.phase == self.READY_DWELL:
            ready_failures = []
            if self._elapsed(now_s, self._attempt_started_s) > (
                    self.config.ready_timeout_s+1e-12):
                ready_failures.append(
                    "ready_timeout_automatic_prelude_violation"
                )
            if boundary_margin < (
                    mpc_bootstrap_required_boundary_margin_m(self.config)
                    - 1e-12):
                ready_failures.append(
                    "insufficient_runtime_maneuver_boundary_budget"
                )
            if max_tilt > self.limits.max_path_tilt_rad+1e-12:
                ready_failures.append(
                    "attitude_automatic_prelude_violation"
                )
            if max_rate > self.limits.max_path_rate_rad_s+1e-12:
                ready_failures.append(
                    "attitude_rate_automatic_prelude_violation"
                )
            if ready_failures:
                return self._abort(
                    now_s=now_s,
                    metrics=metrics,
                    reasons=ready_failures,
                    state_trustworthy=False,
                )
            ready = (
                float(np.linalg.norm(velocity))
                <= self.config.ready_speed_tolerance_m_s+1e-12
                and displacement
                <= self.config.ready_position_tolerance_m+1e-12
                and abs(z_error) <= self.config.ready_z_tolerance_m+1e-12
                and max_tilt
                <= self.limits.terminal_tilt_tolerance_rad+1e-12
                and max_rate
                <= self.limits.terminal_rate_tolerance_rad_s+1e-12
            )
            if not ready:
                self._ready_since_s = None
                return self._decision(
                    now_s=now_s,
                    command_kind="position_hold",
                    metrics=metrics,
                )
            if self._ready_since_s is None:
                self._ready_since_s = now_s
            if self._elapsed(now_s, self._ready_since_s)+1e-12 < (
                    self.config.ready_dwell_s):
                return self._decision(
                    now_s=now_s,
                    command_kind="position_hold",
                    metrics=metrics,
                )
            self.phase = self.LEVEL_WARMUP
            self._level_warmup_started_s = now_s
            return self._decision(
                now_s=now_s,
                command_kind="level_attitude_zdistance",
                metrics=metrics,
                command_roll_deg=0.0,
                command_pitch_deg=0.0,
            )

        if self.phase == self.LEVEL_WARMUP:
            warmup_failures = []
            if boundary_margin < (
                    mpc_bootstrap_required_boundary_margin_m(self.config)
                    - 1e-12):
                warmup_failures.append(
                    "insufficient_runtime_maneuver_boundary_budget"
                )
            if aligned_speed < (
                    -self.limits.reverse_velocity_tolerance_m_s-1e-12):
                warmup_failures.append(
                    "reverse_velocity_automatic_prelude_violation"
                )
            if abs(cross_speed) > (
                    self.config.ready_speed_tolerance_m_s+1e-12):
                warmup_failures.append(
                    "cross_velocity_automatic_prelude_violation"
                )
            if aligned_speed > (
                    self.config.ready_speed_tolerance_m_s+1e-12):
                warmup_failures.append(
                    "level_warmup_motion_automatic_prelude_violation"
                )
            if displacement > (
                    self.config.ready_position_tolerance_m+1e-12):
                warmup_failures.append(
                    "level_warmup_position_automatic_prelude_violation"
                )
            if abs(z_error) > self.config.ready_z_tolerance_m+1e-12:
                warmup_failures.append(
                    "z_error_automatic_prelude_violation"
                )
            if max_tilt > (
                    self.limits.terminal_tilt_tolerance_rad+1e-12):
                warmup_failures.append(
                    "attitude_automatic_prelude_violation"
                )
            if max_rate > (
                    self.limits.terminal_rate_tolerance_rad_s+1e-12):
                warmup_failures.append(
                    "attitude_rate_automatic_prelude_violation"
                )
            if warmup_failures:
                trustworthy = not any(
                    reason in {
                        "insufficient_runtime_maneuver_boundary_budget",
                        "z_error_automatic_prelude_violation",
                        "attitude_automatic_prelude_violation",
                        "attitude_rate_automatic_prelude_violation",
                    }
                    for reason in warmup_failures
                )
                return self._abort(
                    now_s=now_s,
                    metrics=metrics,
                    reasons=warmup_failures,
                    state_trustworthy=trustworthy,
                )
            if self._elapsed(
                    now_s, self._level_warmup_started_s
            )+1e-12 < self.level_warmup_required_s:
                return self._decision(
                    now_s=now_s,
                    command_kind="level_attitude_zdistance",
                    metrics=metrics,
                    command_roll_deg=0.0,
                    command_pitch_deg=0.0,
                )
            self.phase = self.ACCELERATING
            self._acceleration_started_s = now_s
            return self._acceleration_decision(now_s, metrics)

        if self.phase != self.ACCELERATING:
            raise RuntimeError("unexpected automatic MPC bootstrap phase")
        acceleration_failures = []
        land_failures = set()
        if abs(z_error) > self.config.ready_z_tolerance_m+1e-12:
            acceleration_failures.append(
                "z_error_automatic_prelude_violation"
            )
            land_failures.add("z_error_automatic_prelude_violation")
        if abs(aligned_speed) > self.config.max_release_speed_m_s+1e-12:
            acceleration_failures.append(
                "absolute_aligned_speed_automatic_prelude_violation"
            )
            land_failures.add(
                "absolute_aligned_speed_automatic_prelude_violation"
            )
        if aligned_speed < (
                -self.config.wrong_way_land_speed_m_s-1e-12):
            acceleration_failures.append(
                "wrong_way_speed_automatic_prelude_violation"
            )
            land_failures.add(
                "wrong_way_speed_automatic_prelude_violation"
            )
        elif aligned_speed < (
                -self.limits.reverse_velocity_tolerance_m_s-1e-12):
            acceleration_failures.append(
                "reverse_velocity_automatic_prelude_violation"
            )
        if (
            aligned_speed
            > self.target_cell.target_speed_m_s
            + self.config.automatic_release_tolerance_m_s+1e-12
            and aligned_speed <= self.config.max_release_speed_m_s+1e-12
        ):
            acceleration_failures.append(
                "target_window_skipped_automatic_prelude_violation"
            )
        if abs(cross_speed) > self.config.max_cross_speed_m_s+1e-12:
            acceleration_failures.append(
                "cross_velocity_automatic_prelude_violation"
            )
        if max_tilt > math.radians(
                self.config.max_automatic_tilt_deg)+1e-12:
            acceleration_failures.append(
                "attitude_automatic_prelude_violation"
            )
            land_failures.add("attitude_automatic_prelude_violation")
        if max_rate > math.radians(
                self.config.max_automatic_rate_deg_s)+1e-12:
            acceleration_failures.append(
                "attitude_rate_automatic_prelude_violation"
            )
            land_failures.add("attitude_rate_automatic_prelude_violation")
        if boundary_margin < self.config.boundary_reserve_m-1e-12:
            acceleration_failures.append(
                "boundary_margin_automatic_prelude_violation"
            )
            land_failures.add(
                "boundary_margin_automatic_prelude_violation"
            )
        if displacement > self.config.max_maneuver_displacement_m+1e-12:
            acceleration_failures.append(
                "maneuver_displacement_automatic_prelude_violation"
            )
            land_failures.add(
                "maneuver_displacement_automatic_prelude_violation"
            )
        if self._elapsed(now_s, self._acceleration_started_s) > (
                self.config.max_acceleration_duration_s+1e-12):
            acceleration_failures.append(
                "acceleration_timeout_automatic_prelude_violation"
            )
            land_failures.add(
                "acceleration_timeout_automatic_prelude_violation"
            )
        release_lower = (
            self.target_cell.target_speed_m_s
            - self.config.automatic_release_tolerance_m_s
        )
        release_upper = (
            self.target_cell.target_speed_m_s
            + self.config.automatic_release_tolerance_m_s
        )
        in_release_window = (
            release_lower-1e-12 <= aligned_speed <= release_upper+1e-12
        )
        if (
            in_release_window
            and max_tilt
            > self.limits.max_context_initial_tilt_rad+1e-12
        ):
            acceleration_failures.append(
                "release_context_attitude_automatic_prelude_violation"
            )
        if acceleration_failures:
            return self._abort(
                now_s=now_s,
                metrics=metrics,
                reasons=acceleration_failures,
                state_trustworthy=not bool(land_failures),
            )
        if in_release_window:
            self.phase = self.BRAKING
            return self._decision(
                now_s=now_s,
                command_kind="begin_legacy_attitude_coast",
                metrics=metrics,
                start_braking=True,
            )
        return self._acceleration_decision(now_s, metrics)

    def finish(self):
        """Seal a handled braking/abort attempt before it is discarded."""
        if self.phase not in (self.BRAKING, self.ABORTED):
            raise RuntimeError("automatic attempt is not ready to finish")
        self.phase = self.FINISHED
        return self.phase


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
    virtual["max_velocity_command_m_s"] = 0.75
    virtual.setdefault("force_rendering", {})["enabled"] = False
    virtual.setdefault("contact_detection", {})["source"] = "wrench_observer"
    virtual.setdefault("release_behavior", {})["mode"] = "observer_brake"
    virtual.setdefault("two_afc_friction", {})["enabled"] = False

    wrench = translation.setdefault("wrench_interaction", {})
    wrench["state_source"] = "onboard"
    wrench["shadow_mode"] = False
    wrench["startup_bias_calibration_enabled"] = False
    # Automatic acceleration and braking use position, velocity, attitude, and
    # angular-rate callbacks in the same decision.  Freeze the same strict
    # synchronization limits as the offline safe-set contract; the normal
    # translation mission is allowed to keep its more permissive logging-only
    # policy outside this private --mpc copy.
    bootstrap_limits = SafeSetLimits()
    safety = wrench.setdefault("safety", {})
    safety["enforce_state_group_skew"] = True
    safety["max_state_age_s"] = bootstrap_limits.max_state_age_s
    safety["max_state_group_skew_s"] = (
        bootstrap_limits.max_state_group_skew_s
    )
    wrench.setdefault("detection", {}).setdefault("translation", {})[
        "enabled"
    ] = False
    wrench.setdefault("detection", {}).setdefault("yaw", {})[
        "enabled"
    ] = False
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
    # The normal mission may use a much higher early-level threshold.  The
    # bootstrap must keep the legacy attitude brake active through every
    # 0.25/0.45/0.65 m/s release and only allow its low-speed level tail.
    handoff["coast_level_handoff_speed_m_s"] = 0.10
    return configured


def prepare_mpc_bootstrap_mission(
        mission, *, drone_id, controller_rate_hz, sense_axis=None):
    """Build and fully validate the private mission before arming.

    In addition to the mode overlay, this verifies a full automatic maneuver
    volume and the existing bounded baseline dynamics that own the bootstrap
    flight.  ``sense_axis`` is accepted only for backward-compatible callers;
    automatic collection does not use a contact sensor.  No calibration file
    is written.
    """
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
    required_margin = mpc_bootstrap_required_boundary_margin_m(
        bootstrap_config
    )
    if not math.isfinite(margin) or margin < required_margin-1e-12:
        raise ValueError(
            "--mpc needs at least "
            f"{required_margin:.3f} m XY boundary margin around "
            "calibration_nominal_position for the complete automatic "
            "acceleration/braking maneuver"
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
