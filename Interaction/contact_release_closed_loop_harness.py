"""Deterministic, simulation-only contact-to-brake validation harness.

This module connects the production potentiometer contact/release detectors,
post-release inertial EKF, septic braking profile, and a post-hoc swept-envelope
checker to a small deterministic plant.  It is deliberately *not* a
CrazySim/SITL or hardware result: no radio, commander, firmware, or real vehicle
is used and a passing result never grants control authority.  Its purpose is to
make the remaining closed-loop safety requirements executable before a
timestamp-correct CrazySim run exists.

The estimator receives only measured gyro/specific force and position-only
observations.  Commands are sent solely to the simulated plant; neither the
command nor the braking profile is supplied to the EKF process model.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math
from types import MappingProxyType
from typing import Mapping

import numpy as np

from Interaction.braking_swept_envelope import (
    WorldTrajectorySample,
    certify_braking_swept_envelope,
)
from Interaction.contact_attitude_observer import CF_TIMESTAMP_MODULUS_MS
from Interaction.jerk_limited_braking import make_septic_brake_profile
from Interaction.post_release_inertial_ekf import (
    GRAVITY_M_S2,
    PostReleaseEkfConfig,
    PostReleaseInertialEkf,
)
from Interaction.potentiometer_force_sensor import (
    PotentiometerContactDetector,
    PotentiometerReleaseDetector,
)


@dataclass(frozen=True)
class ClosedLoopHarnessConfig:
    """Fixed limits and plant parameters for one deterministic scenario."""

    integration_step_s: float = 0.001
    horizon_s: float = 1.65
    detector_period_s: float = 0.005
    command_period_s: float = 0.010
    position_period_s: float = 0.010
    position_delay_s: float = 0.0
    # Simulation-only trial: extrapolate a delayed position observation to
    # estimator-now using the EKF velocity and an assumed fixed delay. The
    # default false path is byte-for-byte equivalent to the prior harness.
    position_delay_compensation_enabled: bool = False
    maximum_position_age_s: float = 0.035
    maximum_position_gap_s: float = 0.055
    contact_start_s: float = 0.050
    contact_end_s: float = 0.220
    contact_force_n: float = 2.0
    initial_position_m: tuple[float, float, float] = (0.0, 0.0, 1.0)
    initial_velocity_m_s: tuple[float, float, float] = (0.45, 0.0, 0.0)
    transport_delay_s: float = 0.012
    attitude_natural_frequency_rad_s: float = 25.0
    attitude_damping_ratio: float = 0.95
    planner_max_deceleration_m_s2: float = 1.0
    planner_max_jerk_m_s3: float = 4.0
    terminal_velocity_reserve_m_s: float = 0.0
    maximum_tilt_deg: float = 12.0
    maximum_tilt_rate_deg_s: float = 90.0
    maximum_command_jerk_m_s3: float = 4.05
    maximum_command_hold_s: float = 0.031
    reverse_velocity_tolerance_m_s: float = 0.006
    maximum_stop_overshoot_m: float = 0.005
    maximum_terminal_speed_m_s: float = 0.012
    imu_accel_norm_bounds_g: tuple[float, float] = (0.55, 1.45)
    imu_max_gyro_deg_s: float = 500.0
    bounds_m: Mapping[str, tuple[float, float]] = field(default_factory=lambda: {
        "x": (-1.0, 1.0), "y": (-1.0, 1.0), "z": (0.0, 2.0),
    })
    position_uncertainty_m: tuple[float, float, float] = (0.006, 0.006, 0.008)
    velocity_uncertainty_m_s: tuple[float, float, float] = (0.035, 0.035, 0.040)
    vehicle_radius_m: float = 0.065
    boundary_reserve_m: float = 0.050

    def __post_init__(self) -> None:
        positive = (
            self.integration_step_s,
            self.horizon_s,
            self.detector_period_s,
            self.command_period_s,
            self.position_period_s,
            self.maximum_position_age_s,
            self.maximum_position_gap_s,
            self.contact_force_n,
            self.attitude_natural_frequency_rad_s,
            self.attitude_damping_ratio,
            self.planner_max_deceleration_m_s2,
            self.planner_max_jerk_m_s3,
            self.maximum_tilt_deg,
            self.maximum_tilt_rate_deg_s,
            self.maximum_command_jerk_m_s3,
            self.maximum_command_hold_s,
            self.maximum_stop_overshoot_m,
            self.maximum_terminal_speed_m_s,
            self.imu_max_gyro_deg_s,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("closed-loop positive configuration values must be finite")
        nonnegative = (
            self.transport_delay_s,
            self.position_delay_s,
            self.contact_start_s,
            self.terminal_velocity_reserve_m_s,
            self.reverse_velocity_tolerance_m_s,
            self.vehicle_radius_m,
            self.boundary_reserve_m,
        )
        if (
            any(not math.isfinite(value) or value < 0.0
                for value in nonnegative)
            or type(self.position_delay_compensation_enabled) is not bool
            or self.integration_step_s > min(
                self.detector_period_s,
                self.command_period_s,
                self.position_period_s,
            )
            or self.contact_end_s <= self.contact_start_s
            or self.contact_end_s >= self.horizon_s
        ):
            raise ValueError("closed-loop timing or nonnegative limit is invalid")
        accel_min, accel_max = self.imu_accel_norm_bounds_g
        if (
            not math.isfinite(accel_min)
            or not math.isfinite(accel_max)
            or not 0.0 < accel_min < accel_max
        ):
            raise ValueError("imu acceleration norm bounds must be ordered and positive")
        if not isinstance(self.bounds_m, Mapping) or set(self.bounds_m) != {
                "x", "y", "z"}:
            raise ValueError("bounds_m must contain exactly x, y and z")
        vectors = (
            self.initial_position_m,
            self.initial_velocity_m_s,
            self.position_uncertainty_m,
            self.velocity_uncertainty_m_s,
        )
        try:
            invalid_vector = (
                any(len(vector) != 3 for vector in vectors)
                or any(
                    not math.isfinite(float(value))
                    for vector in vectors for value in vector
                )
            )
        except (TypeError, ValueError, OverflowError):
            invalid_vector = True
        if invalid_vector:
            raise ValueError("state and uncertainty inputs must be finite three-vectors")
        if any(
                float(value) < 0.0
                for vector in (self.position_uncertainty_m,
                               self.velocity_uncertainty_m_s)
                for value in vector):
            raise ValueError("state uncertainty must be nonnegative")
        try:
            bounds_valid = all(
                len(self.bounds_m[axis]) == 2
                and all(math.isfinite(float(value))
                        for value in self.bounds_m[axis])
                and float(self.bounds_m[axis][0]) < float(self.bounds_m[axis][1])
                for axis in ("x", "y", "z")
            )
        except (TypeError, ValueError, OverflowError):
            bounds_valid = False
        if not bounds_valid:
            raise ValueError("each workspace bound must be finite with lower < upper")


@dataclass(frozen=True)
class ClosedLoopFaults:
    """Deterministic fault injection, all times relative to confirmed release."""

    gyro_bias_deg_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    accel_bias_g: tuple[float, float, float] = (0.0, 0.0, 0.0)
    imu_gap_start_s: float | None = None
    imu_gap_duration_s: float = 0.0
    imu_outlier_time_s: float | None = None
    imu_outlier_g: tuple[float, float, float] = (0.0, 0.0, 0.0)
    position_delay_s: float | None = None
    position_drop_every: int = 0
    position_reorder_index: int | None = None
    position_reorder_extra_delay_s: float = 0.0
    host_stall_start_s: float | None = None
    host_stall_duration_s: float = 0.0
    plant_acceleration_disturbance_m_s2: float = 0.0
    release_candidate_rebound: bool = False

    def __post_init__(self) -> None:
        if not isinstance(self.release_candidate_rebound, bool):
            raise ValueError("release_candidate_rebound must be boolean")
        vectors = (self.gyro_bias_deg_s, self.accel_bias_g, self.imu_outlier_g)
        if any(len(vector) != 3 for vector in vectors):
            raise ValueError("fault bias and outlier inputs must be three-vectors")
        scalars = (
            *self.gyro_bias_deg_s,
            *self.accel_bias_g,
            *self.imu_outlier_g,
            self.imu_gap_duration_s,
            self.position_reorder_extra_delay_s,
            self.host_stall_duration_s,
            self.plant_acceleration_disturbance_m_s2,
        )
        optional = (
            self.imu_gap_start_s,
            self.imu_outlier_time_s,
            self.position_delay_s,
            self.host_stall_start_s,
        )
        if any(not math.isfinite(value) for value in scalars):
            raise ValueError("fault inputs must be finite")
        if any(value is not None and not math.isfinite(value) for value in optional):
            raise ValueError("optional fault times must be finite")
        if (
            isinstance(self.position_drop_every, bool)
            or not isinstance(self.position_drop_every, int)
            or isinstance(self.position_reorder_index, bool)
            or (self.position_reorder_index is not None
                and not isinstance(self.position_reorder_index, int))
            or self.imu_gap_duration_s < 0.0
            or self.position_reorder_extra_delay_s < 0.0
            or self.host_stall_duration_s < 0.0
            or (self.position_delay_s is not None and self.position_delay_s < 0.0)
            or self.position_drop_every < 0
            or (self.position_reorder_index is not None
                and self.position_reorder_index < 0)
            or any(value is not None and value < 0.0 for value in optional)
        ):
            raise ValueError("fault durations, delays and indices must be nonnegative")


@dataclass(frozen=True)
class ClosedLoopTraceSample:
    time_s: float
    contact_active: bool
    release_candidate_active: bool
    release_confirmed: bool
    true_position_m: tuple[float, float, float]
    true_velocity_m_s: tuple[float, float, float]
    estimated_position_m: tuple[float, float, float] | None
    estimated_velocity_m_s: tuple[float, float, float] | None
    pitch_deg: float
    pitch_rate_deg_s: float
    commanded_acceleration_m_s2: float
    command_age_s: float | None
    estimator_valid: bool | None
    authority_latched_off: bool


@dataclass(frozen=True)
class HardGateResult:
    passed: bool
    reason: str
    observed: float | None = None
    limit: float | None = None
    units: str | None = None


@dataclass(frozen=True)
class ClosedLoopValidationResult:
    passed: bool
    gates: Mapping[str, HardGateResult]
    trace: tuple[ClosedLoopTraceSample, ...]
    contact_started: bool
    release_detected: bool
    release_boundary_time_s: float | None
    release_confirmation_time_s: float | None
    braking_start_time_s: float | None
    release_boundary_sample_id: int | None
    release_confirmation_sample_id: int | None
    candidate_boundary_sample_ids: tuple[int, ...]
    release_candidate_started_count: int
    release_candidate_cancelled_count: int
    command_count: int
    skipped_command_count: int
    position_update_count: int
    position_drop_count: int
    position_reorder_rejection_count: int
    position_stale_rejection_count: int
    profile_duration_s: float | None
    profile_stop_position_m: float | None
    delay_compensated_stop_position_m: float | None
    estimator_process_inputs: tuple[str, ...]
    detector_source: str = "potentiometer"
    estimator_seed_source: str = (
        "simulated_plant_truth_at_committed_candidate_first_unloaded_sample"
    )
    contact_attitude_seed_validated: bool = False
    profile_feedback_scope: str = (
        "EKF_release_confirmation_snapshot_then_time_parameterized_open_loop"
    )
    swept_envelope_scope: str = "post_hoc_realized_trajectory_validation_only"
    simulation_only: bool = True
    authoritative: bool = False
    live_crazysim_coupled: bool = False
    remaining_live_gate: str = (
        "Run the same detector-to-commander chain against timestamp-correct "
        "CrazySim/SITL, including the real send path and onboard watchdog."
    )


class _PlanarAttitudePlant:
    """Small deterministic pitch/translation plant used only by the harness."""

    def __init__(self, config: ClosedLoopHarnessConfig, faults: ClosedLoopFaults):
        self.config = config
        self.faults = faults
        self.time_s = 0.0
        self.position = np.asarray(config.initial_position_m, dtype=float).copy()
        self.velocity = np.asarray(config.initial_velocity_m_s, dtype=float).copy()
        self.pitch_rad = 0.0
        self.pitch_rate_rad_s = 0.0
        self._target_pitch_rad = 0.0
        self._pending_commands: list[tuple[float, int, float]] = []
        self._command_sequence = 0

    def send_acceleration(self, acceleration_m_s2: float) -> None:
        target = math.atan(float(acceleration_m_s2) / GRAVITY_M_S2)
        heapq.heappush(
            self._pending_commands,
            (
                self.time_s + self.config.transport_delay_s,
                self._command_sequence,
                target,
            ),
        )
        self._command_sequence += 1

    def _apply_pending(self) -> None:
        while self._pending_commands and self._pending_commands[0][0] <= self.time_s + 1e-12:
            _, _, self._target_pitch_rad = heapq.heappop(self._pending_commands)

    def acceleration_world(self) -> np.ndarray:
        return np.array([
            GRAVITY_M_S2 * math.tan(self.pitch_rad)
            + self.faults.plant_acceleration_disturbance_m_s2,
            0.0,
            0.0,
        ])

    def quaternion_wxyz(self) -> tuple[float, float, float, float]:
        half = 0.5 * self.pitch_rad
        return (math.cos(half), 0.0, math.sin(half), 0.0)

    def imu(self) -> tuple[np.ndarray, np.ndarray]:
        """Return ideal gyro deg/s and body specific force in g."""
        acceleration = self.acceleration_world()
        world_specific_force = acceleration + np.array([0.0, 0.0, GRAVITY_M_S2])
        cosine, sine = math.cos(self.pitch_rad), math.sin(self.pitch_rad)
        rotation_body_to_world = np.array([
            [cosine, 0.0, sine],
            [0.0, 1.0, 0.0],
            [-sine, 0.0, cosine],
        ])
        body_specific_force = rotation_body_to_world.T @ world_specific_force
        gyro = np.array([0.0, math.degrees(self.pitch_rate_rad_s), 0.0])
        return gyro, body_specific_force / GRAVITY_M_S2

    def advance(self) -> None:
        self._apply_pending()
        dt = self.config.integration_step_s
        wn = self.config.attitude_natural_frequency_rad_s
        zeta = self.config.attitude_damping_ratio

        def derivative(state):
            position_x, velocity_x, pitch, pitch_rate = state
            del position_x
            acceleration_x = (
                GRAVITY_M_S2 * math.tan(pitch)
                + self.faults.plant_acceleration_disturbance_m_s2
            )
            return np.array([
                velocity_x,
                acceleration_x,
                pitch_rate,
                wn * wn * (self._target_pitch_rad - pitch)
                - 2.0 * zeta * wn * pitch_rate,
            ])

        state = np.array([
            self.position[0], self.velocity[0], self.pitch_rad,
            self.pitch_rate_rad_s,
        ])
        k1 = derivative(state)
        k2 = derivative(state + 0.5 * dt * k1)
        k3 = derivative(state + 0.5 * dt * k2)
        k4 = derivative(state + dt * k3)
        state += dt * (k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0
        self.position[0], self.velocity[0], self.pitch_rad, self.pitch_rate_rad_s = state
        self.time_s += dt


def _inside_interval(value: float, start: float | None, duration: float) -> bool:
    return start is not None and start <= value < start + duration


def _gate(
        passed: bool, reason: str, observed: float | None = None,
        limit: float | None = None, units: str | None = None,
) -> HardGateResult:
    return HardGateResult(bool(passed), reason, observed, limit, units)


def run_contact_release_closed_loop_harness(
        config: ClosedLoopHarnessConfig | None = None,
        faults: ClosedLoopFaults | None = None,
) -> ClosedLoopValidationResult:
    """Run one reproducible, non-authoritative contact/release/brake scenario."""
    config = config or ClosedLoopHarnessConfig()
    faults = faults or ClosedLoopFaults()
    plant = _PlanarAttitudePlant(config, faults)
    contact_detector = PotentiometerContactDetector(
        force_threshold_n=0.08,
        onset_dwell_s=0.030,
        max_sample_gap_s=0.020,
    )
    release_detector = PotentiometerReleaseDetector(
        force_drop_n=0.040,
        candidate_lead_drop_n=0.010,
        decrease_rate_n_s=0.05,
        unloaded_force_n=0.050,
        unloaded_dwell_s=0.040,
        max_sample_gap_s=0.020,
        candidate_stall_timeout_s=0.20,
    )
    ekf: PostReleaseInertialEkf | None = None
    estimate = None
    profile = None
    contact_started = False
    release_detected = False
    release_candidate_active = False
    release_boundary_time_s = None
    release_confirmation_time_s = None
    release_boundary_sample_id = None
    release_confirmation_sample_id = None
    candidate_boundary_sample_ids: list[int] = []
    release_candidate_started_count = 0
    release_candidate_cancelled_count = 0
    authority_latched_off = False
    imu_quality_ok = True
    position_stream_ok = True
    command_hold_ok = True
    command_count = 0
    skipped_command_count = 0
    position_drop_count = 0
    position_reorder_rejections = 0
    position_stale_rejections = 0
    position_capture_index = 0
    position_sequence = 0
    pending_positions: list[tuple[float, int, float, tuple[float, ...]]] = []
    last_fused_position_capture_s = None
    max_fused_position_gap_s = 0.0
    next_detector_s = 0.0
    next_command_s = math.inf
    next_position_s = math.inf
    last_command_send_s = None
    last_command_acceleration = 0.0
    max_command_hold_s = 0.0
    max_command_jerk_m_s3 = 0.0
    max_tilt_deg = 0.0
    max_tilt_rate_deg_s = 0.0
    imu_outlier_injected = False
    trace: list[ClosedLoopTraceSample] = []
    braking_trajectory: list[WorldTrajectorySample] = []

    while plant.time_s <= config.horizon_s + 1e-12:
        now = plant.time_s
        if now + 1e-12 >= next_detector_s:
            force_n = (
                config.contact_force_n
                if config.contact_start_s <= now < config.contact_end_s
                else 0.0
            )
            # A one-sample unload followed by renewed compression exercises the
            # production detector's reversible candidate lifecycle.  The next
            # sustained unload becomes the only committed release boundary.
            if (
                faults.release_candidate_rebound
                and config.contact_end_s + 0.5*config.detector_period_s <= now
                < config.contact_end_s + 0.040
            ):
                force_n = config.contact_force_n
            contact_decision = contact_detector.update(force_n, now)
            contact_started = contact_started or contact_decision.started
            if contact_decision.started:
                release_detector.arm(
                    force_n, now, peak_force_n=contact_decision.peak_force_n
                )

            release_decision = None
            if (
                release_detector.armed
                and not release_detected
                and not contact_decision.started
            ):
                sample_id = int(round(now*1000.0))
                release_decision = release_detector.update(
                    force_n, now, sample_id=sample_id
                )
                release_candidate_active = release_decision.candidate_active
                if release_decision.candidate_started:
                    release_candidate_started_count += 1
                if release_decision.candidate_cancelled:
                    release_candidate_cancelled_count += 1
                    # The first-unloaded preview is reversible.  Discard its
                    # plant-truth seed and every provisional observation; it
                    # must never leak into a later confirmed release.
                    ekf = None
                    estimate = None
                    release_boundary_time_s = None
                    release_boundary_sample_id = None
                    pending_positions.clear()
                    last_fused_position_capture_s = None
                    max_fused_position_gap_s = 0.0
                    next_position_s = math.inf
                    position_capture_index = 0

                boundary_sample_id = (
                    release_decision.unloaded_started_sample_id
                )
                if (
                    boundary_sample_id is not None
                    and boundary_sample_id != release_boundary_sample_id
                ):
                    release_boundary_time_s = (
                        release_decision.unloaded_started_at_s
                    )
                    release_boundary_sample_id = int(boundary_sample_id)
                    candidate_boundary_sample_ids.append(
                        release_boundary_sample_id
                    )
                    raw_timestamp = (
                        int(round(release_boundary_time_s*1000.0))
                        % CF_TIMESTAMP_MODULUS_MS
                    )
                    gyro, _ = plant.imu()
                    ekf = PostReleaseInertialEkf(
                        # This scaffold deliberately uses plant truth here. It
                        # does not validate the contact gyro attitude seed.
                        position_m=tuple(
                            float(value) for value in plant.position
                        ),
                        velocity_m_s=tuple(
                            float(value) for value in plant.velocity
                        ),
                        quaternion_wxyz=plant.quaternion_wxyz(),
                        gyro_bias_rad_s=(0.0, 0.0, 0.0),
                        cf_timestamp_ms=raw_timestamp,
                        initial_gyro_deg_s=(
                            gyro + np.asarray(faults.gyro_bias_deg_s)
                        ),
                        config=PostReleaseEkfConfig(max_imu_gap_ms=5.0),
                    )
                    estimate = ekf.snapshot()
                    next_position_s = now
                    pending_positions.clear()
                    last_fused_position_capture_s = None
                    max_fused_position_gap_s = 0.0
                    position_capture_index = 0

                if release_decision.released:
                    release_detected = True
                    release_confirmation_time_s = (
                        release_decision.release_confirmed_at_s
                    )
                    release_confirmation_sample_id = (
                        release_decision.release_confirmed_sample_id
                    )
                    release_candidate_active = False
                    contact_detector.mark_released()
                    next_command_s = now

            next_detector_s += config.detector_period_s

        # The provisional EKF starts at the immutable first-unloaded sample,
        # while command authority remains off until the later dwell
        # confirmation.  Every sample therefore has a causal capture epoch.
        if release_boundary_time_s is not None:
            boundary_elapsed = now-release_boundary_time_s
            fault_elapsed = (
                -math.inf
                if release_confirmation_time_s is None
                else now-release_confirmation_time_s
            )
            # Generate the 1 kHz IMU in the plant capture-time domain.  A gap
            # advances the next delivered timestamp, exercising the EKF's own
            # maximum-gap rejection instead of hiding the fault in host time.
            if ekf is not None and boundary_elapsed > 1e-12 and not _inside_interval(
                    fault_elapsed, faults.imu_gap_start_s,
                    faults.imu_gap_duration_s):
                gyro, accel = plant.imu()
                gyro += np.asarray(faults.gyro_bias_deg_s)
                accel += np.asarray(faults.accel_bias_g)
                if (
                    faults.imu_outlier_time_s is not None
                    and not imu_outlier_injected
                    and fault_elapsed + 0.5 * config.integration_step_s
                    >= faults.imu_outlier_time_s
                ):
                    accel += np.asarray(faults.imu_outlier_g)
                    imu_outlier_injected = True
                accel_norm = float(np.linalg.norm(accel))
                gyro_norm = float(np.linalg.norm(gyro))
                accel_min, accel_max = config.imu_accel_norm_bounds_g
                if not (
                    np.all(np.isfinite(gyro))
                    and np.all(np.isfinite(accel))
                    and accel_min <= accel_norm <= accel_max
                    and gyro_norm <= config.imu_max_gyro_deg_s
                ):
                    imu_quality_ok = False
                    authority_latched_off = True
                else:
                    raw_timestamp = int(round(now * 1000.0)) % CF_TIMESTAMP_MODULUS_MS
                    estimate = ekf.propagate(raw_timestamp, gyro, accel)
                    if not estimate.valid:
                        authority_latched_off = True

            # Capture position independently from arrival, then accept only
            # fresh, monotonically captured samples.  Quaternion and velocity
            # are intentionally absent from this queue.
            if now + 1e-12 >= next_position_s:
                should_drop = (
                    faults.position_drop_every > 0
                    and (position_capture_index + 1) % faults.position_drop_every == 0
                )
                if should_drop:
                    position_drop_count += 1
                else:
                    delay = (
                        config.position_delay_s
                        if faults.position_delay_s is None
                        else faults.position_delay_s
                    )
                    if position_capture_index == faults.position_reorder_index:
                        delay += faults.position_reorder_extra_delay_s
                    heapq.heappush(
                        pending_positions,
                        (
                            now + delay,
                            position_sequence,
                            now,
                            tuple(float(value) for value in plant.position),
                        ),
                    )
                    position_sequence += 1
                position_capture_index += 1
                next_position_s += config.position_period_s

            while pending_positions and pending_positions[0][0] <= now + 1e-12:
                _, _, capture_time, measured_position = heapq.heappop(pending_positions)
                age = now - capture_time
                if (
                    last_fused_position_capture_s is not None
                    and capture_time <= last_fused_position_capture_s + 1e-12
                ):
                    position_reorder_rejections += 1
                    continue
                if age > config.maximum_position_age_s + 1e-12:
                    position_stale_rejections += 1
                    position_stream_ok = False
                    authority_latched_off = True
                    continue
                if last_fused_position_capture_s is not None:
                    max_fused_position_gap_s = max(
                        max_fused_position_gap_s,
                        capture_time - last_fused_position_capture_s,
                    )
                last_fused_position_capture_s = capture_time
                if ekf is not None and estimate is not None and estimate.valid:
                    if config.position_delay_compensation_enabled:
                        # Never use plant truth or the simulator's exact
                        # capture-to-arrival age as the correction. This is
                        # only the configured fixed-delay hypothesis.
                        delay = config.position_delay_s
                        corrected_position = (
                            np.asarray(measured_position)
                            + np.asarray(estimate.velocity_m_s) * delay
                        )
                        velocity_uncertainty = max(
                            config.velocity_uncertainty_m_s
                        )
                        std = math.hypot(
                            ekf.config.extpos_std_m,
                            velocity_uncertainty * delay,
                        )
                        estimate = ekf.update_extpos(
                            corrected_position, std_m=std,
                        )
                    else:
                        estimate = ekf.update_extpos(measured_position)

            position_gap = (
                boundary_elapsed if last_fused_position_capture_s is None
                else now - last_fused_position_capture_s
            )
            if (
                release_detected
                and position_gap > config.maximum_position_gap_s + 1e-12
            ):
                position_stream_ok = False
                authority_latched_off = True

        if release_detected:
            assert release_confirmation_time_s is not None
            elapsed = now-release_confirmation_time_s
            if profile is None:
                if estimate is None:
                    authority_latched_off = True
                else:
                    profile_velocity = max(
                        estimate.velocity_m_s[0]
                        - config.terminal_velocity_reserve_m_s,
                        0.0,
                    )
                    profile = make_septic_brake_profile(
                        profile_velocity,
                        0.0,
                        config.planner_max_deceleration_m_s2,
                        config.planner_max_jerk_m_s3,
                        initial_position_m=estimate.position_m[0],
                    )

            if now + 1e-12 >= next_command_s:
                stalled = _inside_interval(
                    elapsed, faults.host_stall_start_s,
                    faults.host_stall_duration_s,
                )
                if stalled:
                    skipped_command_count += 1
                else:
                    if authority_latched_off or estimate is None or not estimate.valid:
                        command_acceleration = 0.0
                    else:
                        command_acceleration = (
                            0.0 if profile is None else
                            profile.sample_zoh(
                                elapsed, config.command_period_s
                            ).acceleration_m_s2
                        )
                    if last_command_send_s is not None:
                        interval = now - last_command_send_s
                        if interval > 0.0:
                            max_command_jerk_m_s3 = max(
                                max_command_jerk_m_s3,
                                abs(command_acceleration-last_command_acceleration)
                                / interval,
                            )
                    plant.send_acceleration(command_acceleration)
                    last_command_send_s = now
                    last_command_acceleration = command_acceleration
                    command_count += 1
                next_command_s += config.command_period_s

            if last_command_send_s is not None:
                command_age = now-last_command_send_s
                max_command_hold_s = max(max_command_hold_s, command_age)
                if command_age > config.maximum_command_hold_s + 1e-12:
                    command_hold_ok = False
                    authority_latched_off = True
            else:
                command_age = None

            braking_trajectory.append(WorldTrajectorySample(
                elapsed,
                tuple(float(value) for value in plant.position),
                tuple(float(value) for value in plant.velocity),
            ))
        else:
            command_age = None

        max_tilt_deg = max(max_tilt_deg, abs(math.degrees(plant.pitch_rad)))
        max_tilt_rate_deg_s = max(
            max_tilt_rate_deg_s, abs(math.degrees(plant.pitch_rate_rad_s))
        )
        if not trace or now-trace[-1].time_s >= config.command_period_s-1e-12:
            trace.append(ClosedLoopTraceSample(
                time_s=now,
                contact_active=contact_detector.active,
                release_candidate_active=release_candidate_active,
                release_confirmed=release_detected,
                true_position_m=tuple(float(value) for value in plant.position),
                true_velocity_m_s=tuple(float(value) for value in plant.velocity),
                estimated_position_m=(None if estimate is None else estimate.position_m),
                estimated_velocity_m_s=(None if estimate is None else estimate.velocity_m_s),
                pitch_deg=math.degrees(plant.pitch_rad),
                pitch_rate_deg_s=math.degrees(plant.pitch_rate_rad_s),
                commanded_acceleration_m_s2=last_command_acceleration,
                command_age_s=command_age,
                estimator_valid=(None if estimate is None else estimate.valid),
                authority_latched_off=authority_latched_off,
            ))
        if now >= config.horizon_s-1e-12:
            break
        plant.advance()

    estimator_valid = bool(estimate is not None and estimate.valid)
    release_samples = tuple(braking_trajectory)
    if release_samples:
        tail_end = config.transport_delay_s
        inner_end = tail_end + 4.0/config.attitude_natural_frequency_rad_s
        transport = tuple(sample for sample in release_samples if 0.0 < sample.time_s <= tail_end)
        inner = tuple(sample for sample in release_samples if tail_end < sample.time_s <= inner_end)
        candidate = tuple(sample for sample in release_samples if sample.time_s > inner_end)
        if not candidate:
            candidate = (release_samples[-1],)
        certificate = certify_braking_swept_envelope(
            bounds_m=config.bounds_m,
            current_position_m=release_samples[0].position_m,
            current_velocity_m_s=release_samples[0].velocity_m_s,
            position_uncertainty_m=config.position_uncertainty_m,
            velocity_uncertainty_m_s=config.velocity_uncertainty_m_s,
            vehicle_radius_m=config.vehicle_radius_m,
            boundary_reserve_m=config.boundary_reserve_m,
            transport_tail_samples=transport,
            inner_loop_tail_samples=inner,
            candidate_samples=candidate,
        )
    else:
        certificate = None

    post_release_velocity = [
        sample.velocity_m_s[0] for sample in release_samples
    ]
    post_release_position = [
        sample.position_m[0] for sample in release_samples
    ]
    minimum_velocity = min(post_release_velocity, default=math.inf)
    final_speed = abs(post_release_velocity[-1]) if post_release_velocity else math.inf
    actual_stop_position = max(post_release_position, default=math.inf)
    planned_stop_position = None if profile is None else profile.stop_position_m
    # For a unit-DC-gain second-order attitude response, 2*zeta/wn is
    # the exact first-moment delay.  Adding it to the explicit transport
    # delay predicts the stopping-position shift caused by the known plant
    # tail; the remaining allowance below is only numerical/model tolerance.
    delay_compensated_stop_position = (
        None if profile is None else
        profile.stop_position_m
        + profile.initial_velocity_m_s * (
            config.transport_delay_s
            + 2.0 * config.attitude_damping_ratio
            / config.attitude_natural_frequency_rad_s
        )
    )
    stop_overshoot = (
        math.inf if delay_compensated_stop_position is None
        else actual_stop_position-delay_compensated_stop_position
    )
    no_reverse = minimum_velocity >= -config.reverse_velocity_tolerance_m_s
    no_overshoot = stop_overshoot <= config.maximum_stop_overshoot_m

    position_gate_ok = (
        position_stream_ok
        and max_fused_position_gap_s <= config.maximum_position_gap_s
    )
    lifecycle_ok = bool(
        contact_started
        and release_detected
        and release_boundary_time_s is not None
        and release_confirmation_time_s is not None
        and release_boundary_sample_id is not None
        and release_confirmation_sample_id is not None
        and release_boundary_time_s < release_confirmation_time_s
        and release_boundary_sample_id < release_confirmation_sample_id
    )
    gates = {
        "contact_release": _gate(
            lifecycle_ok,
            "potentiometer onset, first-unloaded boundary, and later dwell "
            "confirmation observed" if lifecycle_ok else
            "potentiometer contact/release lifecycle incomplete",
        ),
        "estimator_valid": _gate(
            estimator_valid,
            "post-release EKF remained valid" if estimator_valid else
            ("post-release EKF invalid or absent" if estimate is None else estimate.reason),
        ),
        "imu_quality": _gate(
            imu_quality_ok,
            "all delivered IMU samples passed finite range checks"
            if imu_quality_ok else "IMU sample rejected; authority latched off",
        ),
        "position_stream": _gate(
            position_gate_ok,
            "fresh monotonic position-only samples maintained"
            if position_gate_ok else "position stream became stale",
            max(max_fused_position_gap_s, 0.0),
            config.maximum_position_gap_s,
            "s",
        ),
        "command_hold": _gate(
            command_hold_ok,
            "command refresh stayed within the maximum hold"
            if command_hold_ok else "host send stall exceeded maximum command hold",
            max_command_hold_s,
            config.maximum_command_hold_s,
            "s",
        ),
        "no_reverse": _gate(
            no_reverse,
            "signed velocity never reversed" if no_reverse else
            "reverse velocity observed",
            max(0.0, -minimum_velocity),
            config.reverse_velocity_tolerance_m_s,
            "m/s",
        ),
        "stop_overshoot": _gate(
            no_overshoot,
            "trajectory stayed within the delay-compensated stop bound"
            if no_overshoot else "delay-compensated stop bound exceeded",
            stop_overshoot,
            config.maximum_stop_overshoot_m,
            "m",
        ),
        "terminal_speed": _gate(
            final_speed <= config.maximum_terminal_speed_m_s,
            "terminal speed settled" if final_speed <= config.maximum_terminal_speed_m_s
            else "terminal speed did not settle",
            final_speed,
            config.maximum_terminal_speed_m_s,
            "m/s",
        ),
        "realized_swept_xyz_boundary_posthoc": _gate(
            bool(certificate is not None and certificate.feasible),
            "post-hoc realized XYZ swept envelope passed"
            if certificate is not None and certificate.feasible else
            ("braking trajectory absent" if certificate is None else certificate.reason),
            None if certificate is None else certificate.limiting_margin_m,
            0.0,
            "m_margin",
        ),
        "tilt": _gate(
            max_tilt_deg <= config.maximum_tilt_deg,
            "tilt stayed within limit" if max_tilt_deg <= config.maximum_tilt_deg
            else "tilt limit exceeded",
            max_tilt_deg,
            config.maximum_tilt_deg,
            "deg",
        ),
        "tilt_rate": _gate(
            max_tilt_rate_deg_s <= config.maximum_tilt_rate_deg_s,
            "tilt rate stayed within limit"
            if max_tilt_rate_deg_s <= config.maximum_tilt_rate_deg_s
            else "tilt-rate limit exceeded",
            max_tilt_rate_deg_s,
            config.maximum_tilt_rate_deg_s,
            "deg/s",
        ),
        "command_jerk": _gate(
            max_command_jerk_m_s3 <= config.maximum_command_jerk_m_s3,
            "sent acceleration sequence stayed within jerk limit"
            if max_command_jerk_m_s3 <= config.maximum_command_jerk_m_s3
            else "sent acceleration jerk limit exceeded",
            max_command_jerk_m_s3,
            config.maximum_command_jerk_m_s3,
            "m/s^3",
        ),
    }
    immutable_gates = MappingProxyType(gates)
    return ClosedLoopValidationResult(
        passed=all(item.passed for item in gates.values()),
        gates=immutable_gates,
        trace=tuple(trace),
        contact_started=contact_started,
        release_detected=release_detected,
        release_boundary_time_s=release_boundary_time_s,
        release_confirmation_time_s=release_confirmation_time_s,
        braking_start_time_s=release_confirmation_time_s,
        release_boundary_sample_id=release_boundary_sample_id,
        release_confirmation_sample_id=release_confirmation_sample_id,
        candidate_boundary_sample_ids=tuple(candidate_boundary_sample_ids),
        release_candidate_started_count=release_candidate_started_count,
        release_candidate_cancelled_count=release_candidate_cancelled_count,
        command_count=command_count,
        skipped_command_count=skipped_command_count,
        position_update_count=(0 if estimate is None else estimate.position_update_count),
        position_drop_count=position_drop_count,
        position_reorder_rejection_count=position_reorder_rejections,
        position_stale_rejection_count=position_stale_rejections,
        profile_duration_s=None if profile is None else profile.duration_s,
        profile_stop_position_m=planned_stop_position,
        delay_compensated_stop_position_m=delay_compensated_stop_position,
        estimator_process_inputs=("gyro", "accelerometer_specific_force", "position_only"),
    )
