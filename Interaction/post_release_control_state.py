"""Extract an estimator-backed braking-state candidate without authority.

The helper intentionally has no fallback to command history, setpoints, or a
different estimator. When the requested estimate is not eligible it returns no
state. ``ready`` means only that the state may be presented to independent
control-safety gates; this helper cannot authorize a setpoint.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from Interaction.contact_attitude_observer import legacy_rpy_from_quaternion
from Interaction.post_release_estimator_gate import (
    PostReleaseEstimatorGateConfig,
    PostReleaseEstimatorGateDecision,
    evaluate_post_release_estimator,
)


@dataclass(frozen=True)
class PostReleaseControlStateCandidate:
    ready: bool
    reason: str
    source: Optional[str]
    position_m: Optional[tuple[float, float, float]]
    velocity_m_s: Optional[tuple[float, float, float]]
    orientation_rpy_rad: Optional[tuple[float, float, float]]
    angular_velocity_rad_s: Optional[tuple[float, float, float]]
    position_std_m: Optional[tuple[float, float, float]]
    velocity_std_m_s: Optional[tuple[float, float, float]]
    attitude_std_deg: Optional[tuple[float, float, float]]
    gyro_bias_std_deg_s: Optional[tuple[float, float, float]]
    estimator_gate: PostReleaseEstimatorGateDecision
    body_rate_std_deg_s: Optional[tuple[float, float, float]] = None
    body_rate_measurement_calibration_id: Optional[str] = None
    candidate_grants_command_authority: bool = False
    command_history_used: bool = False


def post_release_control_state_candidate(
        snapshot, *, now_cf_timestamp_ms, now_timestamp_basis,
        now_unwrapped_cf_timestamp_ms=None,
        now_host_receive_age_s=None,
        gate_config: PostReleaseEstimatorGateConfig | None = None,
) -> PostReleaseControlStateCandidate:
    """Return one finite, same-epoch p/v/RPY/rate state after all gates."""

    gate = evaluate_post_release_estimator(
        snapshot,
        now_cf_timestamp_ms=now_cf_timestamp_ms,
        now_timestamp_basis=now_timestamp_basis,
        now_unwrapped_cf_timestamp_ms=now_unwrapped_cf_timestamp_ms,
        now_host_receive_age_s=now_host_receive_age_s,
        config=gate_config,
    )
    if not gate.estimator_control_eligible:
        return PostReleaseControlStateCandidate(
            ready=False,
            reason=gate.reason,
            source=None,
            position_m=None,
            velocity_m_s=None,
            orientation_rpy_rad=None,
            angular_velocity_rad_s=None,
            position_std_m=gate.position_std_m,
            velocity_std_m_s=gate.velocity_std_m_s,
            attitude_std_deg=gate.attitude_std_deg,
            gyro_bias_std_deg_s=gate.gyro_bias_std_deg_s,
            estimator_gate=gate,
        )
    rate_calibrated = snapshot.get('body_rate_measurement_calibrated')
    rate_std_raw = snapshot.get('body_rate_measurement_std_deg_s')
    rate_calibration_id = snapshot.get(
        'body_rate_measurement_calibration_id'
    )
    if rate_calibrated is True:
        try:
            rate_std = np.asarray(rate_std_raw, dtype=float)
        except (TypeError, ValueError):
            rate_std = np.empty(0)
        if (
            rate_std.shape != (3,)
            or not np.all(np.isfinite(rate_std))
            or np.any(rate_std <= 0.0)
            or not isinstance(rate_calibration_id, str)
            or not rate_calibration_id.strip()
            or rate_calibration_id.strip()
            != snapshot.get('post_release_imu_quality_provenance_id')
        ):
            return PostReleaseControlStateCandidate(
                ready=False,
                reason='body_rate_measurement_evidence_invalid',
                source=None,
                position_m=None,
                velocity_m_s=None,
                orientation_rpy_rad=None,
                angular_velocity_rad_s=None,
                position_std_m=gate.position_std_m,
                velocity_std_m_s=gate.velocity_std_m_s,
                attitude_std_deg=gate.attitude_std_deg,
                gyro_bias_std_deg_s=gate.gyro_bias_std_deg_s,
                estimator_gate=gate,
            )
        body_rate_std = tuple(float(value) for value in rate_std)
        rate_calibration_id = rate_calibration_id.strip()
    elif (
        rate_calibrated in (None, False)
        and rate_std_raw is None
        and rate_calibration_id is None
    ):
        # State eligibility and command eligibility remain separate.  The
        # candidate can be inspected, but the runtime terminal gate receives
        # no body-rate uncertainty and therefore cannot authorize a send.
        body_rate_std = None
        rate_calibration_id = None
    else:
        return PostReleaseControlStateCandidate(
            ready=False,
            reason='body_rate_measurement_evidence_invalid',
            source=None,
            position_m=None,
            velocity_m_s=None,
            orientation_rpy_rad=None,
            angular_velocity_rad_s=None,
            position_std_m=gate.position_std_m,
            velocity_std_m_s=gate.velocity_std_m_s,
            attitude_std_deg=gate.attitude_std_deg,
            gyro_bias_std_deg_s=gate.gyro_bias_std_deg_s,
            estimator_gate=gate,
        )
    try:
        estimate = snapshot["post_release_ekf"]
        position = np.asarray(estimate["position_m"], dtype=float)
        velocity = np.asarray(estimate["velocity_m_s"], dtype=float)
        quaternion = np.asarray(estimate["quaternion_wxyz"], dtype=float)
        angular_velocity = np.asarray(
            snapshot["post_release_control_epoch"][
                "legacy_body_rate_rad_s"
            ],
            dtype=float,
        )
        orientation = np.asarray(
            legacy_rpy_from_quaternion(quaternion), dtype=float
        )
    except (KeyError, TypeError, ValueError, OverflowError):
        return PostReleaseControlStateCandidate(
            ready=False,
            reason="estimate_state_missing",
            source=None,
            position_m=None,
            velocity_m_s=None,
            orientation_rpy_rad=None,
            angular_velocity_rad_s=None,
            position_std_m=gate.position_std_m,
            velocity_std_m_s=gate.velocity_std_m_s,
            attitude_std_deg=gate.attitude_std_deg,
            gyro_bias_std_deg_s=gate.gyro_bias_std_deg_s,
            estimator_gate=gate,
        )
    if (
        position.shape != (3,)
        or velocity.shape != (3,)
        or orientation.shape != (3,)
        or angular_velocity.shape != (3,)
        or not np.all(np.isfinite(position))
        or not np.all(np.isfinite(velocity))
        or not np.all(np.isfinite(orientation))
        or not np.all(np.isfinite(angular_velocity))
    ):
        return PostReleaseControlStateCandidate(
            ready=False,
            reason="estimate_state_invalid",
            source=None,
            position_m=None,
            velocity_m_s=None,
            orientation_rpy_rad=None,
            angular_velocity_rad_s=None,
            position_std_m=gate.position_std_m,
            velocity_std_m_s=gate.velocity_std_m_s,
            attitude_std_deg=gate.attitude_std_deg,
            gyro_bias_std_deg_s=gate.gyro_bias_std_deg_s,
            estimator_gate=gate,
        )
    return PostReleaseControlStateCandidate(
        ready=True,
        reason="eligible_estimator_candidate",
        source="post_release_inertial_ekf_position_only",
        position_m=tuple(float(value) for value in position),
        velocity_m_s=tuple(float(value) for value in velocity),
        orientation_rpy_rad=tuple(float(value) for value in orientation),
        angular_velocity_rad_s=tuple(
            float(value) for value in angular_velocity
        ),
        position_std_m=gate.position_std_m,
        velocity_std_m_s=gate.velocity_std_m_s,
        attitude_std_deg=gate.attitude_std_deg,
        gyro_bias_std_deg_s=gate.gyro_bias_std_deg_s,
        estimator_gate=gate,
        body_rate_std_deg_s=body_rate_std,
        body_rate_measurement_calibration_id=rate_calibration_id,
    )
