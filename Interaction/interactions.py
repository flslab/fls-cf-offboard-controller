import logging
import random
import threading
import time
import traceback
from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

import cflib.crazyflie
import numpy as np
import zmq

from Interaction.command_wrapper import CommandWrapper
from Interaction.commander_handoff import HandoffError, handoff_to_high_level
from Interaction.adaptive_braking_calibration import AdaptiveBrakingCalibration
from Interaction.braking_response_calibration import (
    PlanarBrakingCalibration,
)
from Interaction.braking_repeat_test import (
    calibration_reference,
    repeat_test_config,
    repeat_test_protocol,
    repeat_test_result,
    resolve_repeat_test_selection,
)
from Interaction.position_capture_calibration import PositionCaptureCalibration
from Interaction.online_prediction_calibration import OnlinePredictionCalibration
from Interaction.calibration_trial_readiness import CalibrationTrialReadinessGate
from Interaction.flight_behaviors import load_commands
from Interaction.learning_velocity_mpc import (
    LearningVelocityMPC,
    VelocityMPCConfig,
    VelocityMPCState,
    frozen_velocity_model_from_prediction_model,
)
from Interaction.mpc_bootstrap_calibration import (
    MPCBootstrapAutomaticAttempt,
    MPCBootstrapCalibrationConfig,
    MPCBootstrapCoverage,
    mpc_bootstrap_required_boundary_margin_m,
    mpc_bootstrap_model_contract_for_direction,
    mpc_bootstrap_world_y_direction,
    mpc_decision_state_age_is_fresh,
    validate_mpc_bootstrap_model_contracts,
)
from Interaction.onboard_wrench_interaction_pipeline import OnboardMomentumWrenchPipeline
from Interaction.potentiometer_force_sensor import (
    PotentiometerContactDetector,
    PotentiometerReleaseDetector,
)
from Interaction.predictive_brake_handoff import (
    PredictiveBrakeToPosition,
    projected_tilt_from_world_acceleration,
    projected_tilt_history_from_world_acceleration,
)
from Interaction.release_lmpc_terminal_gate import (
    ATTITUDE_ZDISTANCE_COMMAND,
    ReleaseLMPCTerminalGate,
    ReleaseLMPCTerminalSample,
    classify_terminal_post_state_commands,
)
from Interaction.wrench_interaction_pipeline import WrenchInteractionPipeline
from Interaction.wrench_model_calibration import (
    DEFAULT_CALIBRATION_PATH,
    apply_drone_calibration,
    identify_planar_braking_response,
    identify_xyz_alignment,
    load_drone_calibration,
    planar_braking_fit_is_current,
    save_drone_calibration,
)

# from Interaction.collision_avoidance.simulation import apf_velocity

logger = logging.getLogger(__name__)


class PairedFrictionRandomizer:
    """Randomize high/low friction within each consecutive interaction pair."""

    def __init__(self, config=None, rng=None):
        config = config or {}
        if not isinstance(config, dict):
            raise ValueError('virtual_object.two_afc_friction must be a mapping')
        enabled = config.get('enabled', False)
        if type(enabled) is not bool:
            raise ValueError('two_afc_friction.enabled must be boolean')
        self.enabled = enabled
        self.high_mu = float(config.get('high_mu', 0.10))
        self.low_mu = float(config.get('low_mu', 0.01))
        if (
            not np.all(np.isfinite([self.high_mu, self.low_mu]))
            or self.low_mu < 0.0
            or self.high_mu <= self.low_mu
        ):
            raise ValueError(
                'two_afc_friction requires finite coefficients with '
                'high_mu > low_mu >= 0'
            )
        seed = config.get('random_seed')
        if seed is not None and type(seed) is not int:
            raise ValueError('two_afc_friction.random_seed must be an integer or null')
        self.random_seed = seed
        self._rng = rng if rng is not None else random.Random(seed)
        self._pending_pair = []
        self.actual_sequence = []

    def begin_interaction(self):
        """Lock and record the condition for one accepted interaction start."""
        if not self.enabled:
            return None
        if not self._pending_pair:
            high_first = bool(self._rng.getrandbits(1))
            self._pending_pair = (
                [('high', self.high_mu), ('low', self.low_mu)]
                if high_first else
                [('low', self.low_mu), ('high', self.high_mu)]
            )
        condition, mu = self._pending_pair.pop(0)
        interaction_number = len(self.actual_sequence) + 1
        record = {
            'interaction_number': interaction_number,
            'pair_number': (interaction_number + 1) // 2,
            'position_in_pair': 1 if interaction_number % 2 else 2,
            'condition': condition,
            'mu': mu,
        }
        self.actual_sequence.append(record)
        return dict(record)

    def summary(self):
        return {
            'enabled': self.enabled,
            'high_mu': self.high_mu,
            'low_mu': self.low_mu,
            'random_seed': self.random_seed,
            'interaction_count': len(self.actual_sequence),
            'actual_sequence': [dict(row) for row in self.actual_sequence],
        }

    def formatted_sequence(self):
        if not self.actual_sequence:
            return '(no accepted contact starts)'
        return ' -> '.join(
            f"{row['interaction_number']}:{row['condition']}(mu={row['mu']:.2f})"
            for row in self.actual_sequence
        )


def _planar_fit_for_calibration_save(
        attempted_fit, adaptive_enabled, drone_id, calibration_path):
    """Choose the legacy fit independently from adaptive model persistence."""
    if attempted_fit is None:
        return None, None, None
    # Older callers and test doubles predate the explicit usability field;
    # only an explicit False denotes a quality-gated diagnostic result.
    if attempted_fit.get('usable', True):
        return attempted_fit, 'current_run', None
    if not adaptive_enabled:
        # The identifier normally raises before this point. Keep this guard so
        # a diagnostic result can never silently weaken fixed-baseline mode.
        failures = attempted_fit.get('quality_failures', [])
        raise ValueError(
            'planar braking response fit failed quality gates: '
            + '; '.join(str(item) for item in failures)
        )
    previous_entry = load_drone_calibration(drone_id, calibration_path)
    previous_fit = (
        previous_entry.get('planar_braking_fit')
        if isinstance(previous_entry, dict) else None
    )
    if not planar_braking_fit_is_current(previous_fit):
        raise ValueError(
            'adaptive planar braking fit failed quality gates and no current '
            'legacy planar fit is available to preserve; first run '
            '--calibrate --no-adaptive-braking-calibration'
        )
    return (
        None,
        'preserved_previous_after_adaptive_quality_rejection',
        previous_fit,
    )


def prediction_calibration_report_path(calibration_path, drone_id):
    """Unique diagnostic output; never replace the active calibration file."""
    safe_id = ''.join(c if c.isalnum() or c in '-_' else '_'
                      for c in str(drone_id))[:40] or 'drone'
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')
    return (Path(calibration_path).expanduser().resolve().parent
            / 'prediction_calibration_runs'
            / f'{safe_id}_{stamp}_{uuid4().hex[:8]}' / 'report.json')


def poll_prediction_calibration(session, log_event):
    """Nonblocking delivery; fit failures do not change control ownership."""
    if session is None:
        return
    for item in session.poll():
        event = 'Online Prediction ' + item['event'].replace('_', ' ').title()
        data = item['data']
        log_event(event, data)
        logger.info('%s: %s', event, data)


def velocity_inertia_mass_class(current_mass, virtual_mass, mass_tolerance=1e-6):
    """Classify a virtual mass relative to the physical LightBender mass."""
    current_mass = float(current_mass)
    virtual_mass = float(virtual_mass)
    mass_tolerance = abs(float(mass_tolerance))
    if current_mass <= 0.0 or virtual_mass <= 0.0:
        raise ValueError('current_mass and virtual_mass must be positive')
    if virtual_mass < current_mass - mass_tolerance:
        return 'light'
    if virtual_mass > current_mass + mass_tolerance:
        return 'heavy'
    return 'matched'


def kinetic_energy_velocity(
        measured_velocity,
        current_mass,
        virtual_mass,
        max_energy_gain=4.0,
):
    """Map velocity by equal kinetic energy: v_virtual=sqrt(m/m_v)*v."""
    measured_velocity = np.asarray(measured_velocity, dtype=float)
    if measured_velocity.shape != (3,):
        raise ValueError('measured_velocity must contain XYZ')
    current_mass = float(current_mass)
    virtual_mass = float(virtual_mass)
    max_energy_gain = float(max_energy_gain)
    if current_mass <= 0.0 or virtual_mass <= 0.0:
        raise ValueError('current_mass and virtual_mass must be positive')
    if max_energy_gain < 1.0:
        raise ValueError('max_energy_gain must be at least 1')
    raw_gain = float(np.sqrt(current_mass / virtual_mass))
    applied_gain = min(raw_gain, max_energy_gain)
    return (
        measured_velocity * applied_gain,
        raw_gain,
        applied_gain,
        applied_gain < raw_gain,
    )


def inertia_position_target(interaction_origin, measured_position, energy_gain):
    """Map measured displacement to an equal-energy virtual displacement.

    Anchoring the reference at contact onset makes a heavy target lag behind
    the hand-driven vehicle and a light target lead it.  Computing the mapped
    displacement directly also avoids integrating the same stale mocap
    velocity more than once.
    """
    interaction_origin = np.asarray(interaction_origin, dtype=float)
    measured_position = np.asarray(measured_position, dtype=float)
    if interaction_origin.shape != (3,) or measured_position.shape != (3,):
        raise ValueError('interaction origin and position must contain XYZ')
    energy_gain = float(energy_gain)
    if energy_gain <= 0.0:
        raise ValueError('energy_gain must be positive')
    return interaction_origin + energy_gain * (
        measured_position - interaction_origin
    )


def release_coast_initial_velocity(
        measured_velocity,
        last_force,
        mass,
        force_memory_s=0.02,
        max_velocity_m_s=None,
):
    """Initialize coasting from measured speed plus the last force impulse."""
    velocity = np.asarray(measured_velocity, dtype=float)
    force = np.asarray(last_force, dtype=float)
    mass = float(mass)
    force_memory_s = float(force_memory_s)
    if (
        velocity.shape != (3,)
        or force.shape != (3,)
        or not np.all(np.isfinite(velocity))
        or not np.all(np.isfinite(force))
        or not np.isfinite(mass)
        or mass <= 0.0
        or not np.isfinite(force_memory_s)
        or force_memory_s < 0.0
    ):
        raise ValueError('release coast inputs must be finite with positive mass')
    coast_velocity = velocity.copy()
    coast_velocity[:2] += force[:2] / mass * force_memory_s
    coast_velocity[2] = 0.0
    if max_velocity_m_s is not None:
        max_speed = float(max_velocity_m_s)
        if not np.isfinite(max_speed) or max_speed <= 0.0:
            raise ValueError('release coast max velocity must be positive')
        speed = float(np.linalg.norm(coast_velocity[:2]))
        if speed > max_speed:
            coast_velocity[:2] *= max_speed / speed
    return coast_velocity


def potentiometer_release_direction(force_world, measured_velocity):
    """Prefer the spring-force axis over transient velocity at release onset."""
    force = np.asarray(force_world, dtype=float)
    velocity = np.asarray(measured_velocity, dtype=float)
    if (
        force.shape != (3,)
        or velocity.shape != (3,)
        or not np.all(np.isfinite(force))
        or not np.all(np.isfinite(velocity))
    ):
        raise ValueError('release direction inputs must be finite XYZ')
    direction = force.copy()
    direction[2] = 0.0
    source = 'potentiometer_force_world'
    norm = float(np.linalg.norm(direction[:2]))
    if norm <= 1e-9:
        direction = velocity.copy()
        direction[2] = 0.0
        source = 'measured_velocity_fallback'
        norm = float(np.linalg.norm(direction[:2]))
    if norm > 1e-9:
        direction /= norm
    else:
        direction.fill(0.0)
    return direction, source


def release_dataset_world_y_directions(
        task_axis_xy,
        measured_sensor_axis_world_xy,
        *,
        sensor_axis_is_unsigned=False,
):
    """Return snapped task direction and unsnapped measured release axis.

    Dataset projection is defined only for a configured/runtime task axis that
    lies on the world-Y line.  The measured sensor axis is normalized but never
    snapped, so replay can independently verify the real release alignment.
    This helper is deliberately fail-closed and has no flight-control effect.
    """
    try:
        task_axis = np.asarray(task_axis_xy, dtype=float)
        measured_axis = np.asarray(
            measured_sensor_axis_world_xy, dtype=float
        )
    except (TypeError, ValueError):
        return None, None
    if (
        measured_axis.shape != (2,)
        or not np.all(np.isfinite(measured_axis))
    ):
        return None, None
    measured_norm = float(np.linalg.norm(measured_axis))
    if measured_norm <= 1e-9:
        return None, None
    measured_unit = measured_axis/measured_norm
    if (
        task_axis.shape != (2,)
        or not np.all(np.isfinite(task_axis))
    ):
        return None, measured_unit
    task_norm = float(np.linalg.norm(task_axis))
    if task_norm <= 1e-9:
        return None, measured_unit
    task_unit = task_axis/task_norm
    if abs(float(task_unit[1]))+1e-12 < 0.98:
        return None, measured_unit
    task_direction = np.array([
        0.0, float(np.sign(task_unit[1])),
    ])
    signed_alignment = float(task_direction @ measured_unit)
    if sensor_axis_is_unsigned and abs(signed_alignment)+1e-12 >= 0.98:
        # A single-axis potentiometer measures compression magnitude.  During
        # the two-direction LMPC bootstrap, orient that measured axis line by
        # the actual release-velocity sign while retaining the raw signed axis
        # separately in the flight log.
        measured_unit = measured_unit * (
            1.0 if signed_alignment >= 0.0 else -1.0
        )
        signed_alignment = float(task_direction @ measured_unit)
    if signed_alignment+1e-12 < 0.98:
        return None, measured_unit
    return task_direction, measured_unit


def resolve_release_mode(
        configured_mode,
        force_sensor_available,
        calibration_mode=False,
):
    """Resolve release behavior without coupling model calibration to sensing."""
    configured_mode = str(configured_mode)
    if configured_mode not in ('observer_brake', 'potentiometer_coast'):
        raise ValueError(
            'virtual_object.release_behavior.mode must be '
            'observer_brake or potentiometer_coast'
        )
    # --calibrate identifies only the onboard wrench/motor model.  It runs in
    # shadow mode and must not require or consume the interaction force sensor.
    if calibration_mode:
        return 'observer_brake'
    if configured_mode == 'potentiometer_coast' and not force_sensor_available:
        raise ValueError(
            'potentiometer_coast release requires --sense and a fresh '
            'Arduino force-sensor reader'
        )
    return configured_mode


def resolve_wrench_nominal_target(
        mission_target,
        wrench_config,
        calibration_mode=False,
):
    """Use an optional clear-volume target only for calibration flights."""
    target = list(mission_target)
    if len(target) < 3:
        raise ValueError('wrench mission target must contain X, Y, and Z')
    if calibration_mode:
        calibration_target = dict(wrench_config or {}).get(
            'calibration_nominal_position'
        )
        if calibration_target is not None:
            calibration_target = np.asarray(calibration_target, dtype=float)
            if (
                calibration_target.shape != (3,)
                or not np.all(np.isfinite(calibration_target))
            ):
                raise ValueError(
                    'calibration_nominal_position must contain finite XYZ'
                )
            target[:3] = calibration_target.tolist()
    if not np.all(np.isfinite(np.asarray(target[:3], dtype=float))):
        raise ValueError('wrench mission target XYZ must be finite')
    return target


def release_candidate_sensor_stale_watchdog(
        candidate_pending,
        sensor_fresh,
        timestamp_s,
        stale_since_s,
        timeout_s,
):
    """Track a bounded sensor dropout while release owns the attitude path.

    A release candidate deliberately suppresses force rendering. If the UART
    stream disappears at that moment, neither confirmation nor cancellation can
    advance, so allowing the candidate to persist would leave it in control for
    the rest of the flight. Return the start of the current stale interval and
    whether it has exceeded the configured fail-safe timeout.
    """
    timestamp_s = float(timestamp_s)
    timeout_s = float(timeout_s)
    if (
        not np.isfinite(timestamp_s)
        or not np.isfinite(timeout_s)
        or timeout_s <= 0.0
    ):
        raise ValueError(
            'release-candidate sensor watchdog time and timeout must be finite; '
            'timeout must be positive'
        )
    if not bool(candidate_pending) or bool(sensor_fresh):
        return None, False
    if stale_since_s is None:
        stale_since_s = timestamp_s
    else:
        stale_since_s = float(stale_since_s)
        if not np.isfinite(stale_since_s):
            raise ValueError(
                'release-candidate sensor stale start must be finite'
            )
        if timestamp_s < stale_since_s:
            # A backwards wall-clock adjustment must not turn into an
            # indefinitely negative timeout interval.
            stale_since_s = timestamp_s
    return (
        stale_since_s,
        timestamp_s - stale_since_s >= timeout_s,
    )


def calibration_state_dropout_tolerated(
        state_age_s,
        max_state_age_s,
        dropout_timeout_s,
        calibration_mode=False,
):
    """Return whether calibration may safely wait for a fresh state packet."""
    state_age_s = float(state_age_s)
    max_state_age_s = float(max_state_age_s)
    dropout_timeout_s = float(dropout_timeout_s)
    if (
        not np.all(np.isfinite([
            state_age_s, max_state_age_s, dropout_timeout_s
        ]))
        or max_state_age_s <= 0.0
    ):
        raise ValueError(
            'state age limits must be finite with positive max_state_age_s'
        )
    if not calibration_mode:
        return False
    if dropout_timeout_s <= max_state_age_s:
        raise ValueError(
            'calibration state dropout timeout must be greater than '
            'max_state_age_s'
        )
    return bool(max_state_age_s < state_age_s <= dropout_timeout_s)


def calibration_state_group_skew_tolerated(
        state_group_skew_s,
        max_state_group_skew_s,
        dropout_elapsed_s,
        dropout_timeout_s,
        calibration_mode=False,
        planar_attitude_active=False,
):
    """Allow only a brief calibration hold for mismatched state groups.

    The skewed packet set is never passed to the observer or calibration fit.
    Normal interaction remains fail-fast, and an open-loop planar attitude
    trial is aborted rather than resumed from an uncertain actuator state.
    """
    values = np.asarray([
        state_group_skew_s,
        max_state_group_skew_s,
        dropout_elapsed_s,
        dropout_timeout_s,
    ], dtype=float)
    if (
        not np.all(np.isfinite(values))
        or state_group_skew_s < 0.0
        or max_state_group_skew_s <= 0.0
        or dropout_elapsed_s < 0.0
        or dropout_timeout_s <= 0.0
        or (
            calibration_mode
            and dropout_timeout_s <= max_state_group_skew_s
        )
    ):
        raise ValueError(
            'state-group skew limits and elapsed time must be finite; '
            'the calibration timeout must exceed the normal skew limit'
        )
    return bool(
        calibration_mode
        and not planar_attitude_active
        and max_state_group_skew_s < state_group_skew_s
        <= dropout_timeout_s
        and dropout_elapsed_s <= dropout_timeout_s
    )


def _validated_attitude_limit(max_attitude_deg, context):
    """Return a finite tilt limit in the physically valid tangent range."""
    max_attitude_deg = float(max_attitude_deg)
    if (
        not np.isfinite(max_attitude_deg)
        or max_attitude_deg <= 0.0
        or max_attitude_deg >= 90.0
    ):
        raise ValueError(
            f'{context} max_attitude_deg must be finite and between 0 and 90'
        )
    return max_attitude_deg


def calibrated_force_render_attitude_limit(
        requested_attitude_deg,
        calibration_attitude_deg,
):
    """Keep contact rendering inside the attitude-response fit envelope."""
    requested = _validated_attitude_limit(
        requested_attitude_deg, 'force rendering'
    )
    calibrated = _validated_attitude_limit(
        calibration_attitude_deg, 'planar calibration'
    )
    return min(requested, calibrated)


class InitialContactArmingGate:
    """Arm initial contact detection after continuous low XY speed."""

    def __init__(
            self,
            max_xy_speed_m_s=0.03,
            stationary_dwell_s=0.5,
            enabled=True,
            apply_after_each_interaction=True,
    ):
        self.enabled = bool(enabled)
        self.apply_after_each_interaction = bool(
            apply_after_each_interaction
        )
        self.max_xy_speed_m_s = float(max_xy_speed_m_s)
        self.stationary_dwell_s = float(stationary_dwell_s)
        if (
            not np.isfinite(self.max_xy_speed_m_s)
            or self.max_xy_speed_m_s <= 0.0
        ):
            raise ValueError(
                'initial contact arming max_xy_speed_m_s must be positive'
            )
        if (
            not np.isfinite(self.stationary_dwell_s)
            or self.stationary_dwell_s < 0.0
        ):
            raise ValueError(
                'initial contact arming stationary_dwell_s must be non-negative'
            )
        self.armed = not self.enabled
        self.stationary_since = None
        self.stationary_elapsed_s = 0.0
        self.xy_speed_m_s = None
        self._last_timestamp = None

    def reset(self, after_interaction=False):
        """Reset the gate, optionally skipping repeated post-contact dwell."""
        require_stationary_dwell = bool(
            self.enabled
            and (
                not after_interaction
                or self.apply_after_each_interaction
            )
        )
        self.armed = not require_stationary_dwell
        self.stationary_since = None
        self.stationary_elapsed_s = 0.0
        self.xy_speed_m_s = None
        self._last_timestamp = None

    def update(self, velocity, timestamp):
        """Return true only on the sample that completes the initial dwell."""
        velocity = np.asarray(velocity, dtype=float)
        timestamp = float(timestamp)
        if (
            velocity.shape != (3,)
            or not np.all(np.isfinite(velocity))
            or not np.isfinite(timestamp)
        ):
            raise ValueError(
                'initial contact arming requires finite XYZ velocity and time'
            )
        self.xy_speed_m_s = float(np.linalg.norm(velocity[:2]))
        if self.armed:
            self._last_timestamp = timestamp
            return False
        if (
            self._last_timestamp is not None
            and timestamp < self._last_timestamp
        ):
            self.stationary_since = None
            self.stationary_elapsed_s = 0.0
        self._last_timestamp = timestamp
        if self.xy_speed_m_s >= self.max_xy_speed_m_s:
            self.stationary_since = None
            self.stationary_elapsed_s = 0.0
            return False
        if self.stationary_since is None:
            self.stationary_since = timestamp
        self.stationary_elapsed_s = max(
            0.0, timestamp - self.stationary_since
        )
        if self.stationary_elapsed_s < self.stationary_dwell_s:
            return False
        self.armed = True
        return True


def inertia_command_mode(mass_class, requested_mode=None):
    """Validate and normalize the preferred slow-response rendering mode."""
    aliases = {
        'pos': 'position',
        'position': 'position',
        'vel': 'velocity',
        'velocity': 'velocity',
        'ori': 'orientation',
        'attitude': 'orientation',
        'orientation': 'orientation',
    }
    if requested_mode is None:
        return 'orientation' if mass_class == 'heavy' else 'position'
    normalized = aliases.get(str(requested_mode).strip().lower())
    if normalized is None:
        raise ValueError(
            'inertia_command must be position, velocity, or orientation'
        )
    return normalized


def world_to_body_xy(world_velocity_xy, yaw_deg):
    """Convert an XY velocity command into send_hover_setpoint body axes."""
    world_velocity_xy = np.asarray(world_velocity_xy, dtype=float)
    if world_velocity_xy.shape != (2,):
        raise ValueError('world_velocity_xy must contain X and Y')
    yaw_rad = np.radians(float(yaw_deg))
    cos_y = np.cos(yaw_rad)
    sin_y = np.sin(yaw_rad)
    return np.array([
        world_velocity_xy[0] * cos_y + world_velocity_xy[1] * sin_y,
        -world_velocity_xy[0] * sin_y + world_velocity_xy[1] * cos_y,
    ])


def reset_pid_integrators_without_ack(crazyflie, parameter_names):
    """Reset uint8 PID flags without occupying the parameter reply queue.

    Crazyflie ``Param.set_value`` serializes acknowledged writes with log
    traffic.  The first pair of handoff resets produced a roughly 0.52 second
    telemetry/command gap in flight.  ``set_value_raw`` uses the documented
    by-name no-ack path, which is appropriate for transient uint8 reset flags.
    Test doubles and older cflib versions retain the acknowledged fallback.
    """
    names = tuple(str(name) for name in parameter_names)
    if not names:
        return 'none'
    parameter_api = getattr(crazyflie, 'param', None)
    if parameter_api is None:
        raise AttributeError('Crazyflie parameter API is unavailable')
    raw_setter = getattr(parameter_api, 'set_value_raw', None)
    if callable(raw_setter):
        for complete_name in names:
            # cflib ParamTocElement type 0x08 is uint8_t. Both resetI flags are
            # PARAM_UINT8 in the Crazyflie controller firmware.
            raw_setter(complete_name, 0x08, 1)
        return 'raw_by_name_no_ack'
    setter = getattr(parameter_api, 'set_value', None)
    if not callable(setter):
        raise AttributeError('Crazyflie parameter setter is unavailable')
    for complete_name in names:
        setter(complete_name, '1')
    return 'acknowledged_fallback'


def coast_braking_attitude(
        current_velocity_xy,
        brake_direction_xy,
        yaw_deg,
        velocity_gain_s=2.5,
        max_acceleration_m_s2=2.0,
        max_attitude_deg=20.0,
):
    """Return bounded attitude damping that can only remove kinetic energy.

    Acceleration is always opposite the measured planar velocity, including
    after a small overshoot. Therefore it can brake reverse motion but can
    never add kinetic energy or chase an old position trajectory.
    Crazyflie positive pitch/roll produces force opposite the corresponding
    body axis in this command path, hence the negative attitude mapping.
    """
    current_velocity_xy = np.asarray(current_velocity_xy, dtype=float)
    brake_direction_xy = np.asarray(brake_direction_xy, dtype=float)
    if current_velocity_xy.shape != (2,) or brake_direction_xy.shape != (2,):
        raise ValueError('coast braking velocity/direction must contain XY')
    if (
        not np.all(np.isfinite(current_velocity_xy))
        or not np.all(np.isfinite(brake_direction_xy))
    ):
        raise ValueError('coast braking velocity/direction must be finite')
    velocity_gain_s = float(velocity_gain_s)
    max_acceleration_m_s2 = float(max_acceleration_m_s2)
    max_attitude_deg = _validated_attitude_limit(
        max_attitude_deg, 'coast braking'
    )
    parameters = np.asarray([
        velocity_gain_s,
        max_acceleration_m_s2,
        max_attitude_deg,
        yaw_deg,
    ])
    if not np.all(np.isfinite(parameters)) or np.any(parameters[:3] <= 0.0):
        raise ValueError('coast braking gain and limits must be positive')

    direction_norm = float(np.linalg.norm(brake_direction_xy))
    if direction_norm <= 1e-9:
        forward_speed = 0.0
    else:
        direction = brake_direction_xy / direction_norm
        forward_speed = float(current_velocity_xy @ direction)
    velocity_to_brake = current_velocity_xy.copy()

    requested_acceleration = -velocity_gain_s * velocity_to_brake
    requested_norm = float(np.linalg.norm(requested_acceleration))
    attitude_acceleration_limit = float(
        9.81 * np.tan(np.radians(max_attitude_deg))
    )
    effective_acceleration_limit = min(
        max_acceleration_m_s2,
        attitude_acceleration_limit,
    )
    applied_acceleration = requested_acceleration.copy()
    if requested_norm > effective_acceleration_limit:
        applied_acceleration *= effective_acceleration_limit / requested_norm
    applied_norm = float(np.linalg.norm(applied_acceleration))

    if applied_norm <= 1e-9:
        roll_deg = 0.0
        pitch_deg = 0.0
        raw_tilt_deg = 0.0
    else:
        acceleration_body = world_to_body_xy(applied_acceleration, yaw_deg)
        raw_tilt_deg = float(np.degrees(np.arctan2(
            applied_norm, 9.81
        )))
        applied_tilt_deg = min(raw_tilt_deg, max_attitude_deg)
        pitch_deg = -applied_tilt_deg * float(
            acceleration_body[0] / applied_norm
        )
        roll_deg = -applied_tilt_deg * float(
            acceleration_body[1] / applied_norm
        )

    power = float(applied_acceleration @ current_velocity_xy)
    return {
        'roll_deg': float(roll_deg),
        'pitch_deg': float(pitch_deg),
        'raw_tilt_deg': float(raw_tilt_deg),
        'forward_speed_m_s': float(forward_speed),
        'velocity_to_brake_m_s': velocity_to_brake,
        'requested_acceleration_m_s2': requested_acceleration,
        'applied_acceleration_m_s2': applied_acceleration,
        'action': 'decelerating' if power < -1e-4 else 'holding',
        'power_w_per_kg': power,
        'acceleration_saturated': (
            requested_norm > effective_acceleration_limit
        ),
    }


def attitude_to_world_acceleration(roll_deg, pitch_deg, yaw_deg):
    """Invert the planar attitude convention used by zdistance commands."""
    values = np.asarray([roll_deg, pitch_deg, yaw_deg], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError('attitude command must be finite')
    roll_deg, pitch_deg, yaw_deg = values
    tilt_deg = float(np.hypot(roll_deg, pitch_deg))
    if tilt_deg <= 1e-12:
        return np.zeros(2)
    acceleration_norm = float(9.81 * np.tan(np.radians(tilt_deg)))
    body = -acceleration_norm * np.array([
        pitch_deg / tilt_deg,
        roll_deg / tilt_deg,
    ])
    yaw_rad = np.radians(yaw_deg)
    cos_y = np.cos(yaw_rad)
    sin_y = np.sin(yaw_rad)
    return np.array([
        body[0] * cos_y - body[1] * sin_y,
        body[0] * sin_y + body[1] * cos_y,
    ])


def integrate_rate_limited_leveling_velocity_delta(
        orientation_rpy,
        angular_velocity,
        direction_xy,
        response_delay_s,
        leveling_rate_deg_s,
        integration_step_s,
        acceleration_scale=1.0,
        attitude_limit_deg=30.0,
):
    """Integrate longitudinal acceleration while roll/pitch return to level."""
    orientation = np.asarray(orientation_rpy, dtype=float).copy()
    rates = np.asarray(angular_velocity, dtype=float)
    direction = np.asarray(direction_xy, dtype=float).copy()
    values = np.asarray([
        response_delay_s,
        leveling_rate_deg_s,
        integration_step_s,
        acceleration_scale,
        attitude_limit_deg,
    ], dtype=float)
    if (
        orientation.shape != (3,)
        or rates.shape != (3,)
        or direction.shape != (2,)
        or not np.all(np.isfinite(orientation))
        or not np.all(np.isfinite(rates))
        or not np.all(np.isfinite(direction))
        or not np.all(np.isfinite(values))
        or response_delay_s < 0.0
        or leveling_rate_deg_s <= 0.0
        or integration_step_s <= 0.0
        or acceleration_scale <= 0.0
        or attitude_limit_deg <= 0.0
    ):
        raise ValueError('rate-limited leveling inputs must be finite and valid')
    direction_norm = float(np.linalg.norm(direction))
    if direction_norm <= 1e-9:
        raise ValueError('leveling projection direction must be nonzero')
    direction /= direction_norm

    attitude_limit_rad = np.radians(attitude_limit_deg)
    orientation[:2] = np.clip(
        orientation[:2], -attitude_limit_rad, attitude_limit_rad
    )
    leveling_rate_rad_s = np.radians(leveling_rate_deg_s)

    def projected_acceleration(rpy):
        acceleration_xy = acceleration_scale * attitude_to_world_acceleration(
            np.degrees(rpy[0]),
            np.degrees(rpy[1]),
            np.degrees(rpy[2]),
        )
        return float(acceleration_xy @ direction)

    velocity_delta = 0.0
    elapsed_s = 0.0
    current_acceleration = projected_acceleration(orientation)

    remaining_delay_s = float(response_delay_s)
    while remaining_delay_s > 1e-12:
        step_s = min(integration_step_s, remaining_delay_s)
        next_orientation = orientation.copy()
        rate_xy = rates[:2].copy()
        rate_norm = float(np.linalg.norm(rate_xy))
        if rate_norm > leveling_rate_rad_s:
            rate_xy *= leveling_rate_rad_s / rate_norm
        next_orientation[:2] += rate_xy * step_s
        next_orientation[:2] = np.clip(
            next_orientation[:2], -attitude_limit_rad, attitude_limit_rad
        )
        next_acceleration = projected_acceleration(next_orientation)
        velocity_delta += 0.5 * (
            current_acceleration + next_acceleration
        ) * step_s
        orientation = next_orientation
        current_acceleration = next_acceleration
        elapsed_s += step_s
        remaining_delay_s -= step_s

    while float(np.linalg.norm(orientation[:2])) > 1e-12:
        tilt_rad = float(np.linalg.norm(orientation[:2]))
        step_s = min(integration_step_s, tilt_rad / leveling_rate_rad_s)
        next_tilt_rad = max(0.0, tilt_rad - leveling_rate_rad_s * step_s)
        next_orientation = orientation.copy()
        next_orientation[:2] *= next_tilt_rad / tilt_rad
        next_acceleration = projected_acceleration(next_orientation)
        velocity_delta += 0.5 * (
            current_acceleration + next_acceleration
        ) * step_s
        orientation = next_orientation
        current_acceleration = next_acceleration
        elapsed_s += step_s

    return {
        'velocity_delta_m_s': float(velocity_delta),
        'duration_s': float(elapsed_s),
        'final_projected_acceleration_m_s2': float(current_acceleration),
    }


def predict_delayed_zero_crossing(
        current_velocity_xy,
        measured_acceleration_xy,
        motion_direction_xy,
        command_history,
        timestamp,
        response_delay_s,
        response_time_constant_s,
        acceleration_scale,
        future_command_acceleration_xy=None,
        future_command_started_at=None,
        continue_after_crossing=False,
        step_s=0.01,
        horizon_s=4.0,
):
    """Roll out the calibrated delayed first-order attitude response.

    The returned distance is measured along ``motion_direction_xy`` from the
    current position to the first zero crossing.  Commands are nominal
    horizontal accelerations implied by roll/pitch; ``acceleration_scale``
    maps them to measured acceleration.  A future command is held after the
    known delay queue has drained.
    """
    velocity = np.asarray(current_velocity_xy, dtype=float)
    acceleration = np.asarray(measured_acceleration_xy, dtype=float)
    direction = np.asarray(motion_direction_xy, dtype=float)
    future_command = np.asarray(
        np.zeros(2)
        if future_command_acceleration_xy is None
        else future_command_acceleration_xy,
        dtype=float,
    )
    if any(value.shape != (2,) for value in (
            velocity, acceleration, direction, future_command)):
        raise ValueError('delayed response rollout inputs must contain XY')
    if not all(np.all(np.isfinite(value)) for value in (
            velocity, acceleration, direction, future_command)):
        raise ValueError('delayed response rollout inputs must be finite')
    timestamp = float(timestamp)
    response_delay_s = float(response_delay_s)
    response_time_constant_s = float(response_time_constant_s)
    acceleration_scale = float(acceleration_scale)
    future_command_started_at = float(
        timestamp
        if future_command_started_at is None
        else future_command_started_at
    )
    step_s = float(step_s)
    horizon_s = float(horizon_s)
    continue_after_crossing = bool(continue_after_crossing)
    scalars = np.asarray([
        timestamp,
        response_delay_s,
        response_time_constant_s,
        acceleration_scale,
        future_command_started_at,
        step_s,
        horizon_s,
    ])
    if (
        not np.all(np.isfinite(scalars))
        or response_delay_s < 0.0
        or response_time_constant_s < 0.0
        or acceleration_scale <= 0.0
        or step_s <= 0.0
        or horizon_s <= 0.0
    ):
        raise ValueError('delayed response rollout parameters are invalid')
    direction_norm = float(np.linalg.norm(direction))
    if direction_norm <= 1e-9:
        raise ValueError('motion direction cannot be zero')
    direction = direction / direction_norm
    speed = float(velocity @ direction)
    if speed <= 0.0 and not continue_after_crossing:
        return {
            'crossed': True,
            'distance_m': 0.0,
            'time_s': 0.0,
            'final_speed_m_s': speed,
        }

    history = []
    for history_time, history_command in command_history or ():
        history_time = float(history_time)
        history_command = np.asarray(history_command, dtype=float)
        if (
            not np.isfinite(history_time)
            or history_command.shape != (2,)
            or not np.all(np.isfinite(history_command))
        ):
            raise ValueError('command history must contain finite time/XY pairs')
        history.append((history_time, history_command.copy()))
    history.sort(key=lambda item: item[0])

    history_times = [item[0] for item in history]
    history_projection = [float(item[1] @ direction) for item in history]
    future_projection = float(future_command @ direction)
    history_index = -1
    response_events = sorted({
        float(command_time + response_delay_s - timestamp)
        for command_time, _ in history
        if (
            command_time < future_command_started_at
            and 0.0
            < command_time + response_delay_s - timestamp
            < horizon_s
        )
    } | ({
        float(future_command_started_at + response_delay_s - timestamp)
    } if (
        0.0
        < future_command_started_at + response_delay_s - timestamp
        < horizon_s
    ) else set()))
    response_event_index = 0

    projected_acceleration = float(acceleration @ direction)
    distance = 0.0
    crossing_distance = 0.0 if speed <= 0.0 else None
    crossing_time = 0.0 if speed <= 0.0 else None
    elapsed = 0.0
    while elapsed < horizon_s:
        dt = min(step_s, horizon_s - elapsed)
        while (
            response_event_index < len(response_events)
            and response_events[response_event_index] <= elapsed + 1e-12
        ):
            response_event_index += 1
        if response_event_index < len(response_events):
            dt = min(
                dt,
                response_events[response_event_index] - elapsed,
            )
        if dt <= 1e-12:
            if response_event_index < len(response_events):
                elapsed = response_events[response_event_index]
            else:
                elapsed = horizon_s
            continue
        query_time = timestamp + elapsed - response_delay_s
        while (
            history_index + 1 < len(history_times)
            and history_times[history_index + 1] <= query_time + 1e-12
        ):
            history_index += 1
        if query_time >= future_command_started_at - 1e-12:
            delayed_projection = future_projection
        elif history_index >= 0:
            delayed_projection = history_projection[history_index]
        else:
            # A recorded step starts at its timestamp; it must not be extended
            # backward when the query precedes the first history event. Callers
            # that know a command was held earlier seed that prior command
            # explicitly (for example at confirmed release).
            delayed_projection = 0.0
        target_acceleration = acceleration_scale * delayed_projection
        if response_time_constant_s <= 1e-9:
            next_acceleration = target_acceleration
            velocity_delta = target_acceleration * dt
            distance_delta = (
                speed * dt + 0.5 * target_acceleration * dt ** 2
            )

            def state_delta_at(local_dt):
                return (
                    target_acceleration * local_dt,
                    speed * local_dt
                    + 0.5 * target_acceleration * local_dt ** 2,
                )
        else:
            decay = np.exp(-dt / response_time_constant_s)
            acceleration_error = (
                projected_acceleration - target_acceleration
            )
            next_acceleration = (
                target_acceleration + acceleration_error * decay
            )
            velocity_delta = (
                target_acceleration * dt
                + acceleration_error
                * response_time_constant_s * (1.0 - decay)
            )
            distance_delta = (
                speed * dt
                + 0.5 * target_acceleration * dt ** 2
                + acceleration_error * response_time_constant_s
                * (
                    dt
                    - response_time_constant_s * (1.0 - decay)
                )
            )

            def state_delta_at(local_dt):
                local_decay = np.exp(
                    -local_dt / response_time_constant_s
                )
                local_velocity_delta = (
                    target_acceleration * local_dt
                    + acceleration_error * response_time_constant_s
                    * (1.0 - local_decay)
                )
                local_distance_delta = (
                    speed * local_dt
                    + 0.5 * target_acceleration * local_dt ** 2
                    + acceleration_error * response_time_constant_s
                    * (
                        local_dt
                        - response_time_constant_s
                        * (1.0 - local_decay)
                    )
                )
                return local_velocity_delta, local_distance_delta

        next_speed = speed + velocity_delta
        next_distance = distance + distance_delta
        crossing_bracket_high = None
        if crossing_time is None and speed > 0.0:
            if next_speed <= 0.0:
                crossing_bracket_high = dt
            elif (
                response_time_constant_s > 1e-9
                and projected_acceleration < 0.0
                and next_acceleration > 0.0
            ):
                # Velocity can cross zero twice inside one interval while both
                # endpoint samples remain positive.  Check the interior minimum
                # where the first-order acceleration changes sign.
                acceleration_error = (
                    projected_acceleration - target_acceleration
                )
                extremum_ratio = (
                    -target_acceleration / acceleration_error
                    if abs(acceleration_error) > 1e-12 else -1.0
                )
                if 0.0 < extremum_ratio < 1.0:
                    extremum_dt = -response_time_constant_s * np.log(
                        extremum_ratio
                    )
                    if 0.0 < extremum_dt < dt:
                        extremum_velocity_delta, _ = state_delta_at(
                            extremum_dt
                        )
                        if speed + extremum_velocity_delta <= 0.0:
                            crossing_bracket_high = extremum_dt
        if crossing_bracket_high is not None:
            # Solve the continuous first-order response inside this interval.
            # Bisection is deterministic and avoids the step-size-dependent
            # half-impulse introduced by trapezoiding an instantaneous command.
            crossing_low = 0.0
            crossing_high = crossing_bracket_high
            for _ in range(40):
                crossing_mid = 0.5 * (crossing_low + crossing_high)
                mid_velocity_delta, _ = state_delta_at(crossing_mid)
                if speed + mid_velocity_delta > 0.0:
                    crossing_low = crossing_mid
                else:
                    crossing_high = crossing_mid
            crossing_dt = crossing_high
            _, crossing_distance_delta = state_delta_at(crossing_dt)
            crossing_time = elapsed + crossing_dt
            crossing_distance = distance + crossing_distance_delta
            if not continue_after_crossing:
                return {
                    'crossed': True,
                    'distance_m': float(max(crossing_distance, 0.0)),
                    'time_s': float(crossing_time),
                    'final_speed_m_s': 0.0,
                }
        speed = next_speed
        distance = next_distance
        projected_acceleration = next_acceleration
        elapsed += dt
    return {
        'crossed': crossing_time is not None,
        'distance_m': float(max(
            distance if crossing_distance is None else crossing_distance,
            0.0,
        )),
        'time_s': float(
            horizon_s if crossing_time is None else crossing_time
        ),
        # Unlike the first-zero-crossing fields above, this is the projected
        # speed after the complete delayed command/lag tail has settled.  It
        # tells the controller to level *before* the zero crossing whenever
        # the commands already in flight contain enough braking impulse.
        'final_speed_m_s': float(speed),
    }


def release_tail_neutralization_attitude(
        current_velocity_xy,
        brake_direction_xy,
        yaw_deg,
        response_delay_s=0.12,
        response_time_constant_s=0.08,
        acceleration_scale=1.0,
        terminal_speed_margin_m_s=0.03,
        command_hold_s=0.02,
        max_acceleration_m_s2=5.0,
        max_attitude_deg=30.0,
        measured_acceleration_xy=None,
        command_history=None,
        timestamp=None,
        future_command_started_at=None,
):
    """Level early and cancel only the predicted residual braking tail.

    A force-rendering command can remain effective after the spring starts to
    unload because attitude commands have transport delay and first-order lag.
    During a release candidate, commanding level alone can therefore still
    make a slowly moving vehicle reverse.  This helper predicts the terminal
    speed if level is commanded now and adds one bounded cancellation impulse
    whenever the residual tail would leave the interval between current speed
    and rest. This handles both a braking-tail reversal and a queued forward
    rebound without asking for motion farther from rest.
    """
    velocity = np.asarray(current_velocity_xy, dtype=float)
    direction = np.asarray(brake_direction_xy, dtype=float)
    measured_acceleration = np.asarray(
        np.zeros(2)
        if measured_acceleration_xy is None else measured_acceleration_xy,
        dtype=float,
    )
    if any(value.shape != (2,) for value in (
            velocity, direction, measured_acceleration)):
        raise ValueError('release-tail states must contain XY')
    if not all(np.all(np.isfinite(value)) for value in (
            velocity, direction, measured_acceleration)):
        raise ValueError('release-tail states must be finite')
    max_attitude_deg = _validated_attitude_limit(
        max_attitude_deg, 'release tail'
    )
    parameters = np.asarray([
        yaw_deg,
        response_delay_s,
        response_time_constant_s,
        acceleration_scale,
        terminal_speed_margin_m_s,
        command_hold_s,
        max_acceleration_m_s2,
        max_attitude_deg,
    ], dtype=float)
    if (
        not np.all(np.isfinite(parameters))
        or float(response_delay_s) < 0.0
        or float(response_time_constant_s) < 0.0
        or float(acceleration_scale) <= 0.0
        or float(terminal_speed_margin_m_s) < 0.0
        or float(command_hold_s) <= 0.0
        or float(max_acceleration_m_s2) <= 0.0
    ):
        raise ValueError('release-tail gains and limits are invalid')

    direction_norm = float(np.linalg.norm(direction))
    if direction_norm <= 1e-9:
        velocity_norm = float(np.linalg.norm(velocity))
        direction = (
            velocity / velocity_norm
            if velocity_norm > 1e-9 else np.zeros(2)
        )
    else:
        direction = direction / direction_norm

    forward_speed = float(velocity @ direction)
    predictive_model_used = bool(
        command_history is not None
        and timestamp is not None
        and np.linalg.norm(direction) > 1e-9
    )
    level_tail = None
    terminal_speed = forward_speed
    if predictive_model_used:
        level_tail = predict_delayed_zero_crossing(
            velocity,
            measured_acceleration,
            direction,
            command_history,
            timestamp,
            response_delay_s,
            response_time_constant_s,
            acceleration_scale,
            future_command_started_at=future_command_started_at,
            continue_after_crossing=True,
        )
        terminal_speed = float(level_tail['final_speed_m_s'])

    # Level-now is safe only when its eventual speed stays between the current
    # speed and zero.  Clamp the prediction to that interval and cancel only
    # the excess. This is symmetric at zero and after a small reversal, unlike
    # a one-sided "forward pulse" special case.
    target_terminal_speed = float(np.clip(
        terminal_speed,
        min(forward_speed, 0.0),
        max(forward_speed, 0.0),
    ))
    attitude_physical_limit = float(
        acceleration_scale
        * 9.81
        * np.tan(np.radians(max_attitude_deg))
    )
    physical_limit = min(
        float(max_acceleration_m_s2), attitude_physical_limit
    )
    terminal_speed_correction = target_terminal_speed - terminal_speed
    cancellation_signed_acceleration = 0.0
    if abs(terminal_speed_correction) > 1e-6:
        cancellation_signed_acceleration = float(np.clip(
            terminal_speed_correction / float(command_hold_s),
            -physical_limit,
            physical_limit,
        ))
    cancellation_acceleration = abs(cancellation_signed_acceleration)
    predicted_terminal_after_pulse = float(
        terminal_speed
        + cancellation_signed_acceleration * float(command_hold_s)
    )
    physical_acceleration = cancellation_signed_acceleration * direction
    command_acceleration = physical_acceleration / float(acceleration_scale)
    command_norm = float(np.linalg.norm(command_acceleration))
    if command_norm <= 1e-9:
        roll_deg = 0.0
        pitch_deg = 0.0
        raw_tilt_deg = 0.0
        action = 'leveling_release_candidate'
    else:
        acceleration_body = world_to_body_xy(command_acceleration, yaw_deg)
        raw_tilt_deg = float(np.degrees(np.arctan2(
            command_norm, 9.81
        )))
        applied_tilt_deg = min(raw_tilt_deg, max_attitude_deg)
        pitch_deg = -applied_tilt_deg * float(
            acceleration_body[0] / command_norm
        )
        roll_deg = -applied_tilt_deg * float(
            acceleration_body[1] / command_norm
        )
        action = (
            'canceling_predicted_reverse_tail'
            if cancellation_signed_acceleration > 0.0
            else 'canceling_predicted_forward_tail'
        )
    return {
        'roll_deg': float(roll_deg),
        'pitch_deg': float(pitch_deg),
        'raw_tilt_deg': float(raw_tilt_deg),
        'action': action,
        'forward_speed_m_s': float(forward_speed),
        'target_terminal_speed_m_s': float(target_terminal_speed),
        'predicted_level_terminal_speed_m_s': float(terminal_speed),
        'predicted_level_stop_distance_m': (
            None if level_tail is None else float(level_tail['distance_m'])
        ),
        'tail_cancellation_acceleration_m_s2': float(
            cancellation_acceleration
        ),
        'tail_cancellation_signed_acceleration_m_s2': float(
            cancellation_signed_acceleration
        ),
        'predicted_terminal_after_pulse_m_s': (
            predicted_terminal_after_pulse
        ),
        'command_acceleration_m_s2': command_acceleration,
        'applied_acceleration_m_s2': physical_acceleration,
        'command_hold_s': float(command_hold_s),
        'predictive_response_model_used': predictive_model_used,
        'power_w_per_kg': float(physical_acceleration @ velocity),
    }


def coast_target_braking_attitude(
        current_position_xy,
        current_velocity_xy,
        target_position_xy,
        brake_direction_xy,
        yaw_deg,
        response_delay_s=0.12,
        response_time_constant_s=0.0,
        acceleration_scale=1.0,
        terminal_speed_margin_m_s=0.03,
        command_hold_s=0.02,
        velocity_gain_s=2.5,
        max_acceleration_m_s2=5.0,
        max_attitude_deg=30.0,
        measured_acceleration_xy=None,
        command_history=None,
        timestamp=None,
        future_command_started_at=None,
):
    """Brake toward a frozen stop target with actuator-delay lookahead.

    The longitudinal command is the constant deceleration required to remove
    the measured forward speed in the distance that will remain after the
    configured attitude-to-acceleration delay. Reverse motion is damped along
    that calibrated line; transverse attitude is left level because this
    scalar response fit does not identify the perpendicular axis. The sole
    non-dissipative exception is a bounded
    one-command-period pulse that cancels a predicted residual braking tail;
    its terminal-speed target cannot exceed the already measured speed.
    """
    position = np.asarray(current_position_xy, dtype=float)
    velocity = np.asarray(current_velocity_xy, dtype=float)
    target = np.asarray(target_position_xy, dtype=float)
    direction = np.asarray(brake_direction_xy, dtype=float)
    measured_acceleration = np.asarray(
        np.zeros(2)
        if measured_acceleration_xy is None else measured_acceleration_xy,
        dtype=float,
    )
    if any(value.shape != (2,) for value in (
            position, velocity, target, direction, measured_acceleration)):
        raise ValueError('target coast states must contain XY')
    if not all(np.all(np.isfinite(value)) for value in (
            position, velocity, target, direction, measured_acceleration)):
        raise ValueError('target coast states must be finite')
    response_delay_s = float(response_delay_s)
    response_time_constant_s = float(response_time_constant_s)
    acceleration_scale = float(acceleration_scale)
    terminal_speed_margin_m_s = float(terminal_speed_margin_m_s)
    command_hold_s = float(command_hold_s)
    velocity_gain_s = float(velocity_gain_s)
    max_acceleration_m_s2 = float(max_acceleration_m_s2)
    max_attitude_deg = _validated_attitude_limit(
        max_attitude_deg, 'target coast'
    )
    parameters = np.asarray([
        response_delay_s,
        response_time_constant_s,
        acceleration_scale,
        terminal_speed_margin_m_s,
        command_hold_s,
        velocity_gain_s,
        max_acceleration_m_s2,
        max_attitude_deg,
        yaw_deg,
    ])
    if (
        not np.all(np.isfinite(parameters))
        or response_delay_s < 0.0
        or response_time_constant_s < 0.0
        or acceleration_scale <= 0.0
        or terminal_speed_margin_m_s < 0.0
        or command_hold_s <= 0.0
        or velocity_gain_s <= 0.0
        or max_acceleration_m_s2 <= 0.0
    ):
        raise ValueError('target coast gains and limits must be positive')

    direction_norm = float(np.linalg.norm(direction))
    if direction_norm <= 1e-9:
        velocity_norm = float(np.linalg.norm(velocity))
        direction = (
            velocity / velocity_norm
            if velocity_norm > 1e-9 else np.zeros(2)
        )
    else:
        direction = direction / direction_norm

    forward_speed = float(velocity @ direction)
    remaining_distance = float((target - position) @ direction)
    response_horizon_s = response_delay_s + response_time_constant_s
    attitude_physical_acceleration_limit = float(
        acceleration_scale * 9.81 * np.tan(np.radians(max_attitude_deg))
    )
    effective_physical_acceleration_limit = min(
        max_acceleration_m_s2,
        attitude_physical_acceleration_limit,
    )
    measured_deceleration = min(max(
        -float(measured_acceleration @ direction),
        0.0,
    ), max_acceleration_m_s2)
    predictive_model_used = command_history is not None and timestamp is not None
    level_tail = None
    if (
        predictive_model_used
        and np.linalg.norm(direction) > 1e-9
    ):
        level_tail = predict_delayed_zero_crossing(
            velocity,
            measured_acceleration,
            direction,
            command_history,
            timestamp,
            response_delay_s,
            response_time_constant_s,
            acceleration_scale,
            future_command_started_at=future_command_started_at,
            continue_after_crossing=True,
        )
        future_forward_speed = max(
            float(level_tail['final_speed_m_s']), 0.0
        )
        delay_reserved_distance = float(level_tail['distance_m'])
    else:
        future_forward_speed = max(
            forward_speed - measured_deceleration * response_horizon_s,
            0.0,
        )
        delay_reserved_distance = max(
            max(forward_speed, 0.0) * response_horizon_s
            - 0.5 * measured_deceleration * response_horizon_s ** 2,
            0.0,
        )
    effective_remaining_distance = max(
        remaining_distance - delay_reserved_distance,
        0.0,
    )
    impulse_safe_deceleration = effective_physical_acceleration_limit
    if level_tail is not None:
        # The command chosen below will be reconsidered after one control
        # period, not held forever as the target-distance solve assumes. Its
        # total eventual velocity change is gain * command * hold time,
        # independent of first-order lag. Cap this one-frame impulse so a
        # nearly sufficient delay queue cannot receive a final oversized
        # braking pulse.
        impulse_safe_deceleration = min(
            max(
                float(level_tail['final_speed_m_s'])
                - terminal_speed_margin_m_s,
                0.0,
            ) / command_hold_s,
            effective_physical_acceleration_limit,
        )

    required_deceleration = 0.0
    tail_cancellation_acceleration = 0.0
    tail_cancellation_signed_acceleration = 0.0
    tail_terminal_target_speed = None
    predicted_terminal_after_pulse = None
    longitudinal_acceleration = np.zeros(2)
    action = 'holding'
    if level_tail is not None:
        # Level-now is safe only when its eventual longitudinal speed remains
        # between the current speed and zero.  Any queued response outside that
        # interval either crosses through zero or accelerates farther away from
        # rest. Retain bounded tail cancellation for reversal/zero-speed
        # protection. While still moving forward, however, a forward-growing
        # tail must be handled by the stopping controller below: canceling only
        # that extra speed would otherwise starve distance-aware braking.
        predicted_level_terminal_speed = float(
            level_tail['final_speed_m_s']
        )
        safe_terminal_speed_low = min(forward_speed, 0.0)
        safe_terminal_speed_high = max(forward_speed, 0.0)
        tail_terminal_target_speed = float(np.clip(
            predicted_level_terminal_speed,
            safe_terminal_speed_low,
            safe_terminal_speed_high,
        ))
        terminal_speed_correction = (
            tail_terminal_target_speed - predicted_level_terminal_speed
        )
        forward_tail_should_use_stop_controller = bool(
            forward_speed > 1e-9
            and predicted_level_terminal_speed > forward_speed
        )
        if (
            abs(terminal_speed_correction) > 1e-6
            and not forward_tail_should_use_stop_controller
        ):
            tail_cancellation_signed_acceleration = float(np.clip(
                terminal_speed_correction / command_hold_s,
                -effective_physical_acceleration_limit,
                effective_physical_acceleration_limit,
            ))
            tail_cancellation_acceleration = abs(
                tail_cancellation_signed_acceleration
            )
            longitudinal_acceleration = (
                tail_cancellation_signed_acceleration * direction
            )
            predicted_terminal_after_pulse = float(
                predicted_level_terminal_speed
                + tail_cancellation_signed_acceleration * command_hold_s
            )
            action = (
                'canceling_predicted_reverse_tail'
                if tail_cancellation_signed_acceleration > 0.0
                else 'canceling_predicted_forward_tail'
            )

    if tail_cancellation_acceleration > 0.0:
        pass
    elif forward_speed > 1e-9:
        level_now_has_enough_braking_impulse = bool(
            level_tail is not None
            and level_tail['final_speed_m_s']
            <= terminal_speed_margin_m_s
        )
        if level_now_has_enough_braking_impulse:
            # Commands already in the calibrated delay queue and lag state are
            # sufficient. Level before the zero crossing so their tail cannot
            # stack into a reversal. If the target is already unavoidable,
            # stopping a little beyond it is safer than adding more braking.
            action = 'leveling_for_response_tail'
        elif remaining_distance <= 0.0:
            # Once the frozen target has been passed, position error is no
            # longer allowed to shape the command. Continue damping only the
            # measured forward velocity; otherwise a level command could let
            # the vehicle coast forever and prevent the strict handoff.
            required_deceleration = min(
                velocity_gain_s * forward_speed,
                impulse_safe_deceleration,
            )
            longitudinal_acceleration = -required_deceleration * direction
            action = 'damping_forward_motion_after_target'
        elif predictive_model_used:
            # Pick the smallest constant physical deceleration whose delayed
            # first-order rollout reaches zero no later than the target.  This
            # is recomputed every fresh state sample; once level-now is enough,
            # the branch above removes the command before the zero crossing.
            low = 0.0
            high = effective_physical_acceleration_limit
            maximum_command = -(high / acceleration_scale) * direction
            maximum_stop = predict_delayed_zero_crossing(
                velocity,
                measured_acceleration,
                direction,
                command_history,
                timestamp,
                response_delay_s,
                response_time_constant_s,
                acceleration_scale,
                future_command_acceleration_xy=maximum_command,
                future_command_started_at=future_command_started_at,
            )
            if (
                maximum_stop['crossed']
                and maximum_stop['distance_m'] <= remaining_distance
            ):
                for _ in range(9):
                    midpoint = 0.5 * (low + high)
                    trial = predict_delayed_zero_crossing(
                        velocity,
                        measured_acceleration,
                        direction,
                        command_history,
                        timestamp,
                        response_delay_s,
                        response_time_constant_s,
                        acceleration_scale,
                        future_command_acceleration_xy=(
                            -(midpoint / acceleration_scale) * direction
                        ),
                        future_command_started_at=(
                            future_command_started_at
                        ),
                    )
                    if (
                        not trial['crossed']
                        or trial['distance_m'] > remaining_distance
                    ):
                        low = midpoint
                    else:
                        high = midpoint
                required_deceleration = high
            else:
                required_deceleration = high
            required_deceleration = min(
                required_deceleration,
                impulse_safe_deceleration,
            )
            longitudinal_acceleration = -required_deceleration * direction
            action = 'decelerating'
        elif future_forward_speed > 1e-9:
            if effective_remaining_distance <= 1e-6:
                required_deceleration = effective_physical_acceleration_limit
            else:
                required_deceleration = min(
                    future_forward_speed ** 2
                    / (2.0 * effective_remaining_distance),
                    effective_physical_acceleration_limit,
                )
            longitudinal_acceleration = -required_deceleration * direction
            action = 'decelerating'
        else:
            action = 'leveling_for_response_tail'
    elif forward_speed < -1e-9:
        # A small reversal can still be caused by unavoidable actuator tail.
        # Work in the original interaction frame. A positive level-tail terminal
        # means queued commands would rebound through zero and relaunch forward;
        # a negative terminal means reverse motion still needs dissipating.
        predicted_terminal_speed = (
            None
            if level_tail is None
            else float(level_tail['final_speed_m_s'])
        )
        reverse_impulse_safe_deceleration = (
            effective_physical_acceleration_limit
            if predicted_terminal_speed is None else min(
                max(
                    -predicted_terminal_speed - terminal_speed_margin_m_s,
                    0.0,
                ) / command_hold_s,
                effective_physical_acceleration_limit,
            )
        )
        if (
            predicted_terminal_speed is None
            or predicted_terminal_speed < -terminal_speed_margin_m_s
        ):
            required_deceleration = min(
                velocity_gain_s * abs(forward_speed),
                reverse_impulse_safe_deceleration,
            )
            longitudinal_acceleration = required_deceleration * direction
            action = 'damping_reverse_motion'
        else:
            action = 'leveling_for_response_tail'
    lateral_velocity = velocity - forward_speed * direction
    # The braking fit is one-dimensional. Reusing its Y (or X) gain for the
    # perpendicular body/world response can introduce an unmeasured sign or
    # scale error, so do not command lateral attitude. The strict full-XY speed
    # gate still prevents a moving vehicle from handing off to position control.
    lateral_acceleration = np.zeros(2)
    longitudinal_acceleration_norm = float(np.linalg.norm(
        longitudinal_acceleration
    ))
    lateral_limit = float(np.sqrt(max(
        effective_physical_acceleration_limit ** 2
        - longitudinal_acceleration_norm ** 2,
        0.0,
    )))
    lateral_norm = float(np.linalg.norm(lateral_acceleration))
    if lateral_norm > lateral_limit and lateral_norm > 1e-12:
        lateral_acceleration *= lateral_limit / lateral_norm
    requested_physical_acceleration = (
        longitudinal_acceleration + lateral_acceleration
    )
    requested_physical_norm = float(np.linalg.norm(
        requested_physical_acceleration
    ))
    applied_physical_acceleration = requested_physical_acceleration.copy()
    command_acceleration = applied_physical_acceleration / acceleration_scale
    command_norm = float(np.linalg.norm(command_acceleration))

    if command_norm <= 1e-9:
        roll_deg = 0.0
        pitch_deg = 0.0
        raw_tilt_deg = 0.0
    else:
        acceleration_body = world_to_body_xy(command_acceleration, yaw_deg)
        raw_tilt_deg = float(np.degrees(np.arctan2(
            command_norm, 9.81
        )))
        applied_tilt_deg = min(raw_tilt_deg, max_attitude_deg)
        pitch_deg = -applied_tilt_deg * float(
            acceleration_body[0] / command_norm
        )
        roll_deg = -applied_tilt_deg * float(
            acceleration_body[1] / command_norm
        )

    power = float(applied_physical_acceleration @ velocity)
    return {
        'roll_deg': float(roll_deg),
        'pitch_deg': float(pitch_deg),
        'raw_tilt_deg': float(raw_tilt_deg),
        'forward_speed_m_s': float(forward_speed),
        'remaining_distance_m': float(remaining_distance),
        'delay_reserved_distance_m': float(delay_reserved_distance),
        'effective_remaining_distance_m': float(
            effective_remaining_distance
        ),
        'required_deceleration_m_s2': float(required_deceleration),
        'tail_cancellation_acceleration_m_s2': float(
            tail_cancellation_acceleration
        ),
        'tail_cancellation_signed_acceleration_m_s2': float(
            tail_cancellation_signed_acceleration
        ),
        'tail_terminal_target_speed_m_s': tail_terminal_target_speed,
        'predicted_terminal_after_pulse_m_s': (
            predicted_terminal_after_pulse
        ),
        'measured_deceleration_m_s2': float(measured_deceleration),
        'predicted_forward_speed_after_delay_m_s': float(
            future_forward_speed
        ),
        'response_horizon_s': float(response_horizon_s),
        'predictive_response_model_used': bool(predictive_model_used),
        'predicted_level_zero_crossing': (
            None if level_tail is None else bool(level_tail['crossed'])
        ),
        'predicted_level_stop_distance_m': (
            None if level_tail is None else float(level_tail['distance_m'])
        ),
        'predicted_level_stop_time_s': (
            None if level_tail is None else float(level_tail['time_s'])
        ),
        'predicted_level_terminal_speed_m_s': (
            None
            if level_tail is None
            else float(level_tail['final_speed_m_s'])
        ),
        'acceleration_scale': float(acceleration_scale),
        'terminal_speed_margin_m_s': float(terminal_speed_margin_m_s),
        'command_hold_s': float(command_hold_s),
        'impulse_safe_deceleration_m_s2': float(
            impulse_safe_deceleration
        ),
        'effective_acceleration_limit_m_s2': float(
            effective_physical_acceleration_limit
        ),
        'velocity_to_brake_m_s': velocity.copy(),
        'uncontrolled_lateral_velocity_m_s': lateral_velocity.copy(),
        'requested_acceleration_m_s2': requested_physical_acceleration,
        'applied_acceleration_m_s2': applied_physical_acceleration,
        'command_acceleration_m_s2': command_acceleration,
        'action': action,
        'power_w_per_kg': power,
        'acceleration_saturated': (
            max(required_deceleration, tail_cancellation_acceleration)
            >= effective_physical_acceleration_limit - 1e-9
            or lateral_norm > lateral_limit + 1e-9
        ),
        'target_passed': bool(remaining_distance <= 0.0),
    }


def heavy_inertia_attitude(
        delta_velocity_xy,
        dt,
        yaw_deg,
        current_mass,
        virtual_mass,
        max_attitude_deg=20.0,
):
    """Return pitch/roll feedback that opposes acceleration of a heavy object.

    The sign mapping follows the existing Crazyflie interaction convention.
    Verify the signs in a restrained flight test whenever the body-frame
    convention changes.
    """
    delta_velocity_xy = np.asarray(delta_velocity_xy, dtype=float)
    if delta_velocity_xy.shape != (2,):
        raise ValueError('delta_velocity_xy must contain X and Y')
    dt = float(dt)
    current_mass = float(current_mass)
    virtual_mass = float(virtual_mass)
    max_attitude_deg = _validated_attitude_limit(
        max_attitude_deg, 'heavy inertia'
    )
    if dt <= 0.0:
        raise ValueError('dt must be positive')
    if current_mass <= 0.0 or virtual_mass <= 0.0:
        raise ValueError('current_mass and virtual_mass must be positive')

    yaw_rad = np.radians(float(yaw_deg))
    cos_y = np.cos(yaw_rad)
    sin_y = np.sin(yaw_rad)
    body_dv_x = delta_velocity_xy[0] * cos_y + delta_velocity_xy[1] * sin_y
    body_dv_y = -delta_velocity_xy[0] * sin_y + delta_velocity_xy[1] * cos_y

    mass_ratio = max(virtual_mass / current_mass, 1.0)
    sin_pitch = -(1.0 - mass_ratio) * body_dv_x / (9.81 * dt)
    sin_roll = -(1.0 - mass_ratio) * body_dv_y / (9.81 * dt)
    pitch = np.degrees(np.arcsin(np.clip(sin_pitch, -1.0, 1.0)))
    roll = np.degrees(np.arcsin(np.clip(sin_roll, -1.0, 1.0)))
    pitch = float(np.clip(pitch, -max_attitude_deg, max_attitude_deg))
    roll = float(np.clip(roll, -max_attitude_deg, max_attitude_deg))
    return pitch, roll


def virtual_resistance_force(
        velocity_xy,
        virtual_mass,
        kinetic_friction_coefficient=0.0,
        drag_coefficient=0.0,
        frontal_area=0.019,
        air_density=1.225,
        friction_min_speed_m_s=0.02,
        static_friction_coefficient=0.0,
        external_force_xy=None,
):
    """Return virtual friction/drag force magnitudes and motion direction.

    The returned vector points along velocity.  The attitude command convention
    turns this requested counter-force vector into physical force opposite the
    motion direction.
    """
    velocity_xy = np.asarray(velocity_xy, dtype=float)
    if velocity_xy.shape != (2,) or not np.all(np.isfinite(velocity_xy)):
        raise ValueError('virtual resistance velocity must be finite XY')
    values = np.asarray([
        virtual_mass,
        kinetic_friction_coefficient,
        static_friction_coefficient,
        drag_coefficient,
        frontal_area,
        air_density,
        friction_min_speed_m_s,
    ], dtype=float)
    if not np.all(np.isfinite(values)) or values[0] <= 0.0:
        raise ValueError('virtual resistance mass must be positive and finite')
    if np.any(values[1:] < 0.0):
        raise ValueError('virtual resistance parameters cannot be negative')

    speed = float(np.linalg.norm(velocity_xy))
    if speed < float(friction_min_speed_m_s):
        external_force = np.asarray(
            [0.0, 0.0] if external_force_xy is None else external_force_xy,
            dtype=float,
        )
        if external_force.shape != (2,) or not np.all(np.isfinite(external_force)):
            raise ValueError('external force for static friction must be finite XY')
        force_norm = float(np.linalg.norm(external_force))
        static_limit = (
            float(static_friction_coefficient) * float(virtual_mass) * 9.81
        )
        if force_norm <= 1e-9 or static_limit <= 0.0:
            return np.zeros(2), 0.0, 0.0
        static_force = min(force_norm, static_limit)
        return (
            external_force / force_norm * static_force,
            static_force,
            0.0,
        )
    direction = velocity_xy / speed
    friction_force = (
        float(kinetic_friction_coefficient) * float(virtual_mass) * 9.81
        if speed >= float(friction_min_speed_m_s) else 0.0
    )
    drag_force = (
        0.5 * float(air_density) * float(drag_coefficient)
        * float(frontal_area) * speed ** 2
    )
    return direction * (friction_force + drag_force), friction_force, drag_force


def select_inertia_render_mode(
        external_force_xy,
        velocity_xy,
        current_mass,
        virtual_mass,
        preferred_mode,
        virtual_resistance_force_xy=None,
        acceleration_tolerance_m_s2=0.02,
):
    """Choose position for faster virtual motion, otherwise honor priority.

    ``external_force_xy / current_mass`` is the measured native-drone baseline;
    it already contains forces arising in the real flight environment.  The
    virtual acceleration additionally accounts for configured resistance.
    The decision is projected onto the interaction/force direction so an
    unrelated transverse component cannot switch the rendering technique.
    """
    external_force_xy = np.asarray(external_force_xy, dtype=float)
    velocity_xy = np.asarray(velocity_xy, dtype=float)
    resistance = np.asarray(
        [0.0, 0.0]
        if virtual_resistance_force_xy is None
        else virtual_resistance_force_xy,
        dtype=float,
    )
    if any(value.shape != (2,) for value in (
            external_force_xy, velocity_xy, resistance)):
        raise ValueError('force, velocity, and resistance must contain XY')
    current_mass = float(current_mass)
    virtual_mass = float(virtual_mass)
    tolerance = abs(float(acceleration_tolerance_m_s2))
    if current_mass <= 0.0 or virtual_mass <= 0.0:
        raise ValueError('current and virtual mass must be positive')
    preferred_mode = inertia_command_mode('matched', preferred_mode)
    if preferred_mode == 'velocity':
        raise ValueError(
            'momentum force rendering priority must be position or orientation'
        )

    direction = external_force_xy.copy()
    if np.linalg.norm(direction) <= 1e-9:
        direction = velocity_xy.copy()
    direction_norm = float(np.linalg.norm(direction))
    if direction_norm > 1e-9:
        direction /= direction_norm
    else:
        direction = np.zeros(2)

    native_acceleration = external_force_xy / current_mass
    virtual_acceleration = (external_force_xy - resistance) / virtual_mass
    native_projected = float(native_acceleration @ direction)
    virtual_projected = float(virtual_acceleration @ direction)
    faster = virtual_projected > native_projected + tolerance
    return {
        'mode': 'position' if faster else preferred_mode,
        'relation': 'faster' if faster else 'slower_or_equal',
        'direction': direction,
        'native_acceleration': native_acceleration,
        'virtual_acceleration': virtual_acceleration,
        'native_projected_acceleration': native_projected,
        'virtual_projected_acceleration': virtual_projected,
    }


def constrain_predictive_coast_render_mode(
        render_selection,
        release_mode,
        shadow_mode,
):
    """Keep active predictive coasting on its calibrated attitude path."""
    constrained = dict(render_selection)
    if (
        str(release_mode).strip().lower() == 'potentiometer_coast'
        and not bool(shadow_mode)
        and constrained.get('mode') != 'orientation'
    ):
        previous_relation = constrained.get('relation', 'unknown')
        constrained['mode'] = 'orientation'
        constrained['relation'] = (
            f'calibrated_orientation_override:{previous_relation}'
        )
    return constrained


class VirtualObjectPlanarMotion:
    """Bounded XY virtual dynamics used by position rendering and coast."""

    def __init__(
            self,
            mass,
            max_velocity_m_s,
            max_offset_xy,
            kinetic_friction_coefficient=0.0,
            static_friction_coefficient=0.0,
            drag_coefficient=0.0,
            frontal_area=0.019,
            air_density=1.225,
            friction_min_speed_m_s=0.02,
    ):
        self.mass = float(mass)
        self.max_velocity_m_s = abs(float(max_velocity_m_s))
        self.max_offset_xy = np.abs(np.asarray(max_offset_xy, dtype=float))
        if self.mass <= 0.0 or self.max_velocity_m_s <= 0.0:
            raise ValueError('virtual mass and max velocity must be positive')
        if self.max_offset_xy.shape != (2,) or np.any(self.max_offset_xy <= 0.0):
            raise ValueError('virtual max offset must contain positive XY')
        self.resistance_config = {
            'virtual_mass': self.mass,
            'kinetic_friction_coefficient': float(
                kinetic_friction_coefficient
            ),
            'static_friction_coefficient': float(
                static_friction_coefficient
            ),
            'drag_coefficient': float(drag_coefficient),
            'frontal_area': float(frontal_area),
            'air_density': float(air_density),
            'friction_min_speed_m_s': float(friction_min_speed_m_s),
        }
        self.origin = np.zeros(2)
        self.position = np.zeros(2)
        self.velocity = np.zeros(2)

    def set_friction_coefficients(self, kinetic, static):
        values = np.asarray([kinetic, static], dtype=float)
        if not np.all(np.isfinite(values)) or np.any(values < 0.0):
            raise ValueError('virtual friction coefficients must be finite and non-negative')
        self.resistance_config['kinetic_friction_coefficient'] = float(values[0])
        self.resistance_config['static_friction_coefficient'] = float(values[1])

    def reset(self, position_xy, velocity_xy):
        self.origin = np.asarray(position_xy, dtype=float).copy()
        self.position = self.origin.copy()
        self.velocity = np.asarray(velocity_xy, dtype=float).copy()
        if self.origin.shape != (2,) or self.velocity.shape != (2,):
            raise ValueError('virtual reset position and velocity must contain XY')

    def predict_stop(self, dt=0.005, max_duration_s=10.0):
        """Simulate the zero-force virtual trajectory without mutating it."""
        dt = float(dt)
        max_duration_s = float(max_duration_s)
        if (
            not np.isfinite(dt)
            or not np.isfinite(max_duration_s)
            or dt <= 0.0
            or max_duration_s <= 0.0
        ):
            raise ValueError('virtual stop prediction timing must be positive')
        dt = min(dt, 0.05)
        position = self.position.copy()
        velocity = self.velocity.copy()
        elapsed = 0.0
        stopped = False
        stop_speed = max(
            float(self.resistance_config['friction_min_speed_m_s']),
            1e-3,
        )
        while elapsed < max_duration_s:
            speed = float(np.linalg.norm(velocity))
            if speed <= stop_speed:
                velocity.fill(0.0)
                stopped = True
                break
            resistance, _, _ = virtual_resistance_force(
                velocity,
                external_force_xy=np.zeros(2),
                **self.resistance_config,
            )
            acceleration = -resistance / self.mass
            previous_velocity = velocity.copy()
            proposed_velocity = previous_velocity + acceleration * dt
            for axis in range(2):
                if previous_velocity[axis] * proposed_velocity[axis] < 0.0:
                    proposed_velocity[axis] = 0.0
            proposed_position = position + proposed_velocity * dt
            offset = np.clip(
                proposed_position - self.origin,
                -self.max_offset_xy,
                self.max_offset_xy,
            )
            clipped = proposed_position != self.origin + offset
            proposed_velocity[clipped] = 0.0
            position = self.origin + offset
            velocity = proposed_velocity
            elapsed += dt
        return {
            'position': position.copy(),
            'velocity': velocity.copy(),
            'duration_s': float(elapsed),
            'stopped': bool(stopped or np.linalg.norm(velocity) <= stop_speed),
        }

    def resistance(self, external_force_xy=None):
        return virtual_resistance_force(
            self.velocity,
            external_force_xy=external_force_xy,
            **self.resistance_config,
        )

    def step(self, external_force_xy, dt):
        force = np.asarray(external_force_xy, dtype=float)
        if force.shape != (2,) or not np.all(np.isfinite(force)):
            raise ValueError('virtual external force must be finite XY')
        dt = min(max(float(dt), 1e-4), 0.05)
        resistance, friction, drag = self.resistance(force)
        acceleration = (force - resistance) / self.mass
        previous_velocity = self.velocity.copy()
        proposed_velocity = previous_velocity + acceleration * dt
        # Coulomb friction may stop motion, but must never reverse it by itself.
        if np.linalg.norm(force) <= 1e-9:
            for axis in range(2):
                if previous_velocity[axis] * proposed_velocity[axis] < 0.0:
                    proposed_velocity[axis] = 0.0
        speed = float(np.linalg.norm(proposed_velocity))
        if speed > self.max_velocity_m_s:
            proposed_velocity *= self.max_velocity_m_s / speed
        proposed_position = self.position + proposed_velocity * dt
        offset = np.clip(
            proposed_position - self.origin,
            -self.max_offset_xy,
            self.max_offset_xy,
        )
        clipped = proposed_position != self.origin + offset
        proposed_velocity[clipped] = 0.0
        self.position = self.origin + offset
        self.velocity = proposed_velocity
        return {
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'acceleration': acceleration.copy(),
            'resistance': resistance.copy(),
            'friction_force_N': float(friction),
            'drag_force_N': float(drag),
        }


def force_inertia_attitude(
        external_force_xy,
        yaw_deg,
        current_mass,
        virtual_mass,
        max_attitude_deg=20.0,
        virtual_resistance_force_xy=None,
):
    """Convert estimated external force/resistance into counter-tilt.

    For a desired virtual acceleration F/m_virtual, the flight controller must
    oppose the remaining fraction ``1 - m_current/m_virtual`` of the applied
    force. A matched virtual mass therefore removes the inertia term while
    retaining configured friction/drag. The sign convention matches
    ``heavy_inertia_attitude`` and the existing Crazyflie
    ``send_zdistance_setpoint`` path.
    """
    external_force_xy = np.asarray(external_force_xy, dtype=float)
    if external_force_xy.shape != (2,):
        raise ValueError('external_force_xy must contain X and Y')
    current_mass = float(current_mass)
    virtual_mass = float(virtual_mass)
    max_attitude_deg = _validated_attitude_limit(
        max_attitude_deg, 'force inertia'
    )
    if current_mass <= 0.0 or virtual_mass <= 0.0:
        raise ValueError('force inertia attitude masses must be positive')
    force_body = world_to_body_xy(external_force_xy, yaw_deg)
    resistance_force_xy = np.asarray(
        [0.0, 0.0]
        if virtual_resistance_force_xy is None
        else virtual_resistance_force_xy,
        dtype=float,
    )
    if (
        resistance_force_xy.shape != (2,)
        or not np.all(np.isfinite(resistance_force_xy))
    ):
        raise ValueError('virtual resistance force must be finite XY')
    resistance_force_body = world_to_body_xy(
        resistance_force_xy, yaw_deg
    )
    counter_force_body = (
        1.0 - current_mass / virtual_mass
    ) * force_body + (
        current_mass / virtual_mass
    ) * resistance_force_body
    force_norm = float(np.linalg.norm(counter_force_body))
    if force_norm <= 0.0:
        return 0.0, 0.0, 0.0, False

    raw_tilt_deg = float(np.degrees(np.arctan2(
        force_norm, current_mass * 9.81
    )))
    applied_tilt_deg = min(raw_tilt_deg, max_attitude_deg)
    tilt_direction = counter_force_body / force_norm
    pitch = applied_tilt_deg * float(tilt_direction[0])
    roll = applied_tilt_deg * float(tilt_direction[1])
    return pitch, roll, raw_tilt_deg, raw_tilt_deg > max_attitude_deg


class BoundaryExceededError(Exception):
    """Exception raised when the drone leaves the defined interaction space."""
    pass


class StaleLocalizationError(Exception):
    """Exception raised when mocap stops producing fresh position frames."""
    pass


class GuidedTouchProtocol:
    """Generate one-shot terminal/log prompts for repeatable touch trials."""

    def __init__(self, config=None):
        config = config or {}
        self.enabled = bool(config.get('enabled', False))
        self._next_event = 0
        self.events = []
        if not self.enabled:
            return

        countdown_s = int(config.get('countdown_s', 3))
        touch_s = float(config.get('touch_s', 2.0))
        rest_s = float(config.get('rest_s', 3.0))
        trials = config.get('trials', ['X', 'Y', 'Z'])
        if countdown_s <= 0 or touch_s <= 0 or rest_s < 0:
            raise ValueError(
                'guided_touch_test countdown/touch durations must be positive '
                'and rest_s must be non-negative'
            )
        if not isinstance(trials, list) or not trials:
            raise ValueError('guided_touch_test trials must be a non-empty list')

        elapsed_s = 0.0
        trial_count = len(trials)
        for index, trial in enumerate(trials, start=1):
            label = str(trial).strip()
            if not label:
                raise ValueError('guided_touch_test trial labels cannot be empty')
            common = {
                'trial_index': index,
                'trial_count': trial_count,
                'axis': label,
            }
            prefix = f'[XYZ TOUCH {index}/{trial_count} · {label}]'
            for remaining_s in range(countdown_s, 0, -1):
                self.events.append((
                    elapsed_s + countdown_s - remaining_s,
                    'Guided Touch Countdown',
                    f'{prefix} {remaining_s}',
                    {**common, 'countdown_s': remaining_s},
                    False,
                ))
            touch_start_s = elapsed_s + countdown_s
            self.events.append((
                touch_start_s,
                'Guided Touch Start Expected',
                f'{prefix} 0 — TOUCH NOW; hold for {touch_s:.1f} s',
                {**common, 'expected_touch_duration_s': touch_s},
                True,
            ))
            self.events.append((
                touch_start_s + touch_s,
                'Guided Touch Release Expected',
                f'{prefix} RELEASE NOW — hands off',
                common,
                True,
            ))
            elapsed_s += countdown_s + touch_s + rest_s

        self.required_duration_s = elapsed_s
        self.events.append((
            elapsed_s,
            'Guided Touch Test Complete',
            '[XYZ TOUCH] Test sequence complete — keep hands off',
            {'trial_count': trial_count},
            True,
        ))

    def due(self, elapsed_s):
        """Return prompts whose scheduled times have passed exactly once."""
        due_events = []
        while (
            self._next_event < len(self.events)
            and float(elapsed_s) >= self.events[self._next_event][0]
        ):
            due_events.append(self.events[self._next_event])
            self._next_event += 1
        return due_events


class TranslationControlHandoff:
    """Switch translation through contact, braking, and position-hold modes.

    The legacy coast path uses a one-way timed handoff: level at a configured
    signed longitudinal speed, hold that actually sent level command for a
    configured delay, then give the frozen release target to position control.
    Missions may instead request an immediate current-position handoff at the
    speed threshold so no intermediate level-attitude command is sent.
    """

    POSITION_HOLD = 'position_hold'
    CONTACT_POSITION = 'position_interaction'
    CONTACT_ZDISTANCE = 'attitude_zdistance'
    MPC_BOOTSTRAP_ACCELERATION = 'mpc_bootstrap_acceleration'
    ATTITUDE_COAST = 'attitude_coast'
    VELOCITY_COAST = 'velocity_coast'
    POSITION_COAST = 'position_coast'
    ATTITUDE_BRAKING = 'attitude_braking'

    def __init__(
            self,
            initial_position,
            yaw_deg,
            shadow_mode,
            brake_xy_acceleration_m_s2=0.8,
            brake_xy_speed_m_s=0.2,
            brake_settle_s=0.30,
            position_brake_offset_m=0.05,
            brake_min_attitude_deg=3.0,
            brake_max_attitude_deg=30.0,
            brake_timeout_s=1.5,
            brake_velocity_gain_s=2.0,
            brake_min_attitude_taper_speed_m_s=0.25,
            coast_position_gain_s2=4.0,
            coast_velocity_gain_s=2.5,
            coast_max_acceleration_m_s2=5.0,
            coast_attitude_response_delay_s=0.12,
            coast_attitude_time_constant_s=0.08,
            coast_attitude_acceleration_scale=1.0,
            coast_calibrated_direction_xy=None,
            coast_level_terminal_speed_m_s=0.03,
            coast_level_handoff_speed_m_s=0.10,
            coast_level_handoff_delay_s=0.30,
            coast_velocity_braking_enabled=False,
            coast_velocity_handoff_speed_m_s=0.03,
            coast_velocity_handoff_position_offset_m=0.0,
            coast_velocity_predictive_unwind_enabled=False,
            coast_velocity_unwind_terminal_speed_m_s=0.10,
            coast_velocity_unwind_prediction_margin_s=0.15,
            coast_velocity_unwind_command_switch_delay_s=0.0,
            coast_velocity_unwind_integrated_leveling_enabled=False,
            coast_velocity_unwind_tail_calibration_scale=1.0,
            coast_velocity_unwind_direct_level_attitude_enabled=False,
            coast_velocity_unwind_position_control_enabled=False,
            coast_velocity_unwind_leveling_rate_deg_s=100.0,
            coast_velocity_unwind_integration_step_s=0.01,
            coast_velocity_unwind_min_deceleration_m_s2=0.30,
            coast_velocity_unwind_filter_time_constant_s=0.03,
            coast_velocity_unwind_max_target_error_m_s=0.15,
            coast_velocity_unwind_one_step_lookahead_enabled=False,
            coast_velocity_unwind_one_step_max_dt_s=0.03,
            coast_velocity_unwind_low_speed_fallback_m_s=0.03,
            coast_velocity_rebrake_enabled=True,
            coast_velocity_rebrake_speed_m_s=0.04,
            coast_velocity_handoff_min_projected_speed_m_s=-0.03,
            coast_velocity_handoff_max_rate_deg_s=5.0,
            coast_state_kinematic_guard_enabled=False,
            coast_state_max_kinematic_residual_m=0.03,
            coast_state_max_implied_acceleration_m_s2=20.0,
            coast_state_max_sample_gap_s=0.05,
            coast_direct_position_handoff=False,
            coast_command_period_s=0.02,
            coast_command_acceleration_deadband_m_s2=0.02,
            coast_candidate_tail_cancellation_max_acceleration_m_s2=1.0,
            coast_acceleration_filter_time_constant_s=0.08,
            coast_handoff_speed_m_s=0.04,
            coast_handoff_max_lateral_speed_m_s=0.15,
            coast_handoff_max_tilt_deg=0.5,
            coast_handoff_max_acceleration_m_s2=0.35,
            coast_alignment_position_tolerance_m=0.04,
            coast_alignment_velocity_tolerance_m_s=0.08,
            coast_alignment_dwell_s=0.08,
            coast_attitude_timeout_s=1.5,
            rearm_delay_s=0.0,
    ):
        self.hold_position = np.asarray(initial_position, dtype=float).copy()
        if (
            self.hold_position.shape != (3,)
            or not np.all(np.isfinite(self.hold_position))
        ):
            raise ValueError(
                'initial translation hold position must contain finite XYZ'
            )
        self.yaw_deg = float(yaw_deg)
        if not np.isfinite(self.yaw_deg):
            raise ValueError('initial translation yaw must be finite')
        self.shadow_mode = bool(shadow_mode)
        self.brake_xy_acceleration_m_s2 = float(brake_xy_acceleration_m_s2)
        self.brake_xy_speed_m_s = float(brake_xy_speed_m_s)
        self.brake_settle_s = float(brake_settle_s)
        self.position_brake_offset_m = float(position_brake_offset_m)
        self.brake_min_attitude_deg = float(brake_min_attitude_deg)
        self.brake_max_attitude_deg = _validated_attitude_limit(
            brake_max_attitude_deg, 'translation braking'
        )
        self.brake_timeout_s = float(brake_timeout_s)
        self.brake_velocity_gain_s = float(brake_velocity_gain_s)
        self.brake_min_attitude_taper_speed_m_s = float(
            brake_min_attitude_taper_speed_m_s
        )
        self.coast_position_gain_s2 = float(coast_position_gain_s2)
        self.coast_velocity_gain_s = float(coast_velocity_gain_s)
        self.coast_max_acceleration_m_s2 = float(
            coast_max_acceleration_m_s2
        )
        self.coast_attitude_response_delay_s = float(
            coast_attitude_response_delay_s
        )
        self.coast_attitude_time_constant_s = float(
            coast_attitude_time_constant_s
        )
        self.coast_attitude_acceleration_scale = float(
            coast_attitude_acceleration_scale
        )
        if coast_calibrated_direction_xy is None:
            self.coast_calibrated_direction_xy = None
        else:
            calibrated_direction = np.asarray(
                coast_calibrated_direction_xy, dtype=float
            )
            if (
                calibrated_direction.shape != (2,)
                or not np.all(np.isfinite(calibrated_direction))
                or np.linalg.norm(calibrated_direction) <= 1e-9
            ):
                raise ValueError(
                    'coast calibrated direction must be finite, nonzero XY'
                )
            self.coast_calibrated_direction_xy = (
                calibrated_direction / np.linalg.norm(calibrated_direction)
            )
        self.coast_level_terminal_speed_m_s = float(
            coast_level_terminal_speed_m_s
        )
        self.coast_level_handoff_speed_m_s = float(
            coast_level_handoff_speed_m_s
        )
        self.coast_level_handoff_delay_s = float(
            coast_level_handoff_delay_s
        )
        self.coast_velocity_braking_enabled = bool(
            coast_velocity_braking_enabled
        )
        self.coast_velocity_handoff_speed_m_s = float(
            coast_velocity_handoff_speed_m_s
        )
        self.coast_velocity_handoff_position_offset_m = float(
            coast_velocity_handoff_position_offset_m
        )
        self.coast_velocity_predictive_unwind_enabled = bool(
            coast_velocity_predictive_unwind_enabled
        )
        self.coast_velocity_unwind_terminal_speed_m_s = float(
            coast_velocity_unwind_terminal_speed_m_s
        )
        self.coast_velocity_unwind_prediction_margin_s = float(
            coast_velocity_unwind_prediction_margin_s
        )
        self.coast_velocity_unwind_command_switch_delay_s = float(
            coast_velocity_unwind_command_switch_delay_s
        )
        self.coast_velocity_unwind_integrated_leveling_enabled = bool(
            coast_velocity_unwind_integrated_leveling_enabled
        )
        self.coast_velocity_unwind_tail_calibration_scale = float(
            coast_velocity_unwind_tail_calibration_scale
        )
        self.coast_velocity_unwind_direct_level_attitude_enabled = bool(
            coast_velocity_unwind_direct_level_attitude_enabled
        )
        self.coast_velocity_unwind_position_control_enabled = bool(
            coast_velocity_unwind_position_control_enabled
        )
        self.coast_velocity_unwind_leveling_rate_deg_s = float(
            coast_velocity_unwind_leveling_rate_deg_s
        )
        self.coast_velocity_unwind_integration_step_s = float(
            coast_velocity_unwind_integration_step_s
        )
        self.coast_velocity_unwind_min_deceleration_m_s2 = float(
            coast_velocity_unwind_min_deceleration_m_s2
        )
        self.coast_velocity_unwind_filter_time_constant_s = float(
            coast_velocity_unwind_filter_time_constant_s
        )
        self.coast_velocity_unwind_max_target_error_m_s = float(
            coast_velocity_unwind_max_target_error_m_s
        )
        self.coast_velocity_unwind_one_step_lookahead_enabled = bool(
            coast_velocity_unwind_one_step_lookahead_enabled
        )
        self.coast_velocity_unwind_one_step_max_dt_s = float(
            coast_velocity_unwind_one_step_max_dt_s
        )
        self.coast_velocity_unwind_low_speed_fallback_m_s = float(
            coast_velocity_unwind_low_speed_fallback_m_s
        )
        self.coast_velocity_rebrake_enabled = bool(
            coast_velocity_rebrake_enabled
        )
        self.coast_velocity_rebrake_speed_m_s = float(
            coast_velocity_rebrake_speed_m_s
        )
        self.coast_velocity_handoff_min_projected_speed_m_s = float(
            coast_velocity_handoff_min_projected_speed_m_s
        )
        self.coast_velocity_handoff_max_rate_deg_s = float(
            coast_velocity_handoff_max_rate_deg_s
        )
        self.coast_state_kinematic_guard_enabled = bool(
            coast_state_kinematic_guard_enabled
        )
        self.coast_state_max_kinematic_residual_m = float(
            coast_state_max_kinematic_residual_m
        )
        self.coast_state_max_implied_acceleration_m_s2 = float(
            coast_state_max_implied_acceleration_m_s2
        )
        self.coast_state_max_sample_gap_s = float(
            coast_state_max_sample_gap_s
        )
        self.coast_direct_position_handoff = bool(
            coast_direct_position_handoff
        )
        self.coast_command_period_s = float(coast_command_period_s)
        self.coast_command_acceleration_deadband_m_s2 = float(
            coast_command_acceleration_deadband_m_s2
        )
        self.coast_candidate_tail_cancellation_max_acceleration_m_s2 = float(
            coast_candidate_tail_cancellation_max_acceleration_m_s2
        )
        self.coast_acceleration_filter_time_constant_s = float(
            coast_acceleration_filter_time_constant_s
        )
        self.coast_handoff_speed_m_s = float(coast_handoff_speed_m_s)
        self.coast_handoff_max_lateral_speed_m_s = float(
            coast_handoff_max_lateral_speed_m_s
        )
        self.coast_handoff_max_tilt_deg = float(
            coast_handoff_max_tilt_deg
        )
        self.coast_handoff_max_acceleration_m_s2 = float(
            coast_handoff_max_acceleration_m_s2
        )
        self.coast_alignment_position_tolerance_m = float(
            coast_alignment_position_tolerance_m
        )
        self.coast_alignment_velocity_tolerance_m_s = float(
            coast_alignment_velocity_tolerance_m_s
        )
        self.coast_alignment_dwell_s = float(coast_alignment_dwell_s)
        self.coast_attitude_timeout_s = float(coast_attitude_timeout_s)
        self.rearm_delay_s = float(rearm_delay_s)
        if (
            self.coast_velocity_unwind_direct_level_attitude_enabled
            and not self.coast_velocity_predictive_unwind_enabled
        ):
            raise ValueError(
                'direct level-attitude unwind requires predictive unwind'
            )
        if (
            self.coast_velocity_unwind_direct_level_attitude_enabled
            and self.coast_velocity_unwind_position_control_enabled
        ):
            raise ValueError(
                'direct level-attitude unwind and position-controlled unwind '
                'are mutually exclusive'
            )
        translation_limits = np.asarray([
            self.brake_xy_acceleration_m_s2,
            self.brake_xy_speed_m_s,
            self.brake_settle_s,
            self.position_brake_offset_m,
            self.brake_min_attitude_deg,
            self.brake_timeout_s,
            self.brake_velocity_gain_s,
            self.brake_min_attitude_taper_speed_m_s,
            self.coast_position_gain_s2,
            self.coast_velocity_gain_s,
            self.coast_max_acceleration_m_s2,
            self.coast_attitude_response_delay_s,
            self.coast_attitude_time_constant_s,
            self.coast_attitude_acceleration_scale,
            self.coast_level_terminal_speed_m_s,
            self.coast_level_handoff_speed_m_s,
            self.coast_level_handoff_delay_s,
            self.coast_velocity_handoff_speed_m_s,
            self.coast_velocity_handoff_position_offset_m,
            self.coast_velocity_unwind_terminal_speed_m_s,
            self.coast_velocity_unwind_prediction_margin_s,
            self.coast_velocity_unwind_command_switch_delay_s,
            self.coast_velocity_unwind_tail_calibration_scale,
            self.coast_velocity_unwind_leveling_rate_deg_s,
            self.coast_velocity_unwind_integration_step_s,
            self.coast_velocity_unwind_min_deceleration_m_s2,
            self.coast_velocity_unwind_filter_time_constant_s,
            self.coast_velocity_unwind_max_target_error_m_s,
            self.coast_velocity_unwind_one_step_max_dt_s,
            self.coast_velocity_unwind_low_speed_fallback_m_s,
            self.coast_velocity_rebrake_speed_m_s,
            self.coast_velocity_handoff_min_projected_speed_m_s,
            self.coast_velocity_handoff_max_rate_deg_s,
            self.coast_state_max_kinematic_residual_m,
            self.coast_state_max_implied_acceleration_m_s2,
            self.coast_state_max_sample_gap_s,
            self.coast_command_period_s,
            self.coast_command_acceleration_deadband_m_s2,
            self.coast_candidate_tail_cancellation_max_acceleration_m_s2,
            self.coast_acceleration_filter_time_constant_s,
            self.coast_handoff_speed_m_s,
            self.coast_handoff_max_lateral_speed_m_s,
            self.coast_handoff_max_tilt_deg,
            self.coast_handoff_max_acceleration_m_s2,
            self.coast_alignment_position_tolerance_m,
            self.coast_alignment_velocity_tolerance_m_s,
            self.coast_alignment_dwell_s,
            self.coast_attitude_timeout_s,
            self.rearm_delay_s,
        ], dtype=float)
        if (
            not np.all(np.isfinite(translation_limits))
            or self.brake_xy_acceleration_m_s2 <= 0
            or self.brake_xy_speed_m_s <= 0
            or self.brake_settle_s < 0
            or self.position_brake_offset_m < 0
            or self.brake_min_attitude_deg < 0
            or self.brake_min_attitude_deg > self.brake_max_attitude_deg
            or self.brake_timeout_s <= 0
            or self.brake_velocity_gain_s <= 0
            or self.brake_min_attitude_taper_speed_m_s
            <= self.brake_xy_speed_m_s
            or self.coast_position_gain_s2 <= 0
            or self.coast_velocity_gain_s <= 0
            or self.coast_max_acceleration_m_s2 <= 0
            or self.coast_attitude_response_delay_s < 0
            or self.coast_attitude_time_constant_s < 0
            or self.coast_attitude_acceleration_scale <= 0
            or self.coast_level_terminal_speed_m_s < 0
            or self.coast_level_terminal_speed_m_s
            >= self.coast_handoff_speed_m_s
            or self.coast_level_handoff_speed_m_s <= 0
            or self.coast_level_handoff_delay_s < 0
            or self.coast_velocity_handoff_speed_m_s <= 0
            or self.coast_velocity_handoff_position_offset_m < 0
            or self.coast_velocity_unwind_terminal_speed_m_s < 0
            or self.coast_velocity_unwind_prediction_margin_s < 0
            or self.coast_velocity_unwind_command_switch_delay_s < 0
            or self.coast_velocity_unwind_tail_calibration_scale <= 0
            or self.coast_velocity_unwind_leveling_rate_deg_s <= 0
            or self.coast_velocity_unwind_integration_step_s <= 0
            or self.coast_velocity_unwind_min_deceleration_m_s2 <= 0
            or self.coast_velocity_unwind_filter_time_constant_s <= 0
            or self.coast_velocity_unwind_max_target_error_m_s <= 0
            or self.coast_velocity_unwind_one_step_max_dt_s <= 0
            or self.coast_velocity_unwind_low_speed_fallback_m_s <= 0
            or (
                self.coast_velocity_predictive_unwind_enabled
                and self.coast_velocity_rebrake_enabled
                and self.coast_velocity_rebrake_speed_m_s
                <= self.coast_velocity_handoff_speed_m_s
            )
            or self.coast_velocity_handoff_min_projected_speed_m_s > 0
            or self.coast_velocity_handoff_max_rate_deg_s <= 0
            or self.coast_state_max_kinematic_residual_m <= 0
            or self.coast_state_max_implied_acceleration_m_s2 <= 0
            or self.coast_state_max_sample_gap_s <= 0
            or self.coast_command_period_s <= 0
            or self.coast_command_acceleration_deadband_m_s2 < 0
            or self.coast_candidate_tail_cancellation_max_acceleration_m_s2
            <= 0
            or self.coast_acceleration_filter_time_constant_s <= 0
            or self.coast_handoff_speed_m_s <= 0
            or self.coast_handoff_max_lateral_speed_m_s <= 0
            or self.coast_handoff_max_tilt_deg <= 0
            or self.coast_handoff_max_tilt_deg >= 90
            or self.coast_handoff_max_acceleration_m_s2 <= 0
            or self.coast_alignment_position_tolerance_m <= 0
            or self.coast_alignment_velocity_tolerance_m_s <= 0
            or self.coast_alignment_dwell_s < 0
            or self.coast_attitude_timeout_s <= 0
            or self.rearm_delay_s < 0
        ):
            raise ValueError(
                'translation braking limits must be positive; legacy settle '
                'time and position offset cannot be negative'
            )
        self.mode = self.POSITION_HOLD
        self._brake_started_at = None
        self._detector_rearm_at = None
        self.brake_direction = np.zeros(3)
        self.brake_direction_source = None
        self.brake_projected_speed_m_s = 0.0
        self.brake_completion_reason = None
        self.brake_command_tilt_deg = 0.0
        self.brake_force_feedforward_acceleration_m_s2 = 0.0
        self.release_force_N = np.zeros(3)
        self.release_momentum_kg_m_s = None
        self.release_position_m = None
        self.stopping_position_m = None
        self.release_mass_kg = None
        self.hover_z = float(self.hold_position[2])
        # Velocity-coast always uses the task's nominal altitude. Contact and
        # release updates may refresh hover_z for attitude commands, but must
        # not turn the zero-velocity brake into a release-height command.
        self.velocity_coast_fixed_zdistance_m = float(self.hold_position[2])
        self.contact_roll_deg = 0.0
        self.contact_pitch_deg = 0.0
        self.contact_yaw_rate_deg_s = 0.0
        self._pending_attitude_yaw_deg = self.yaw_deg
        self._release_candidate_mode = None
        self._coast_alignment_since = None
        self._coast_position_settle_since = None
        self.coast_tracking_action = None
        self.coast_tracking_position_error_m = None
        self.coast_tracking_velocity_error_m_s = None
        self.coast_tracking_acceleration_m_s2 = None
        self.coast_tracking_acceleration_saturated = False
        self.coast_tracking_power_w_per_kg = None
        self.coast_handoff_reason = None
        self.coast_stop_target_position_m = None
        self.coast_handoff_actual_position_m = None
        self.coast_target_clamped_to_actual = False
        self.coast_lateral_target_latched_to_actual = False
        self.coast_lateral_speed_m_s = None
        self.coast_target_remaining_distance_m = None
        self.coast_delay_reserved_distance_m = None
        self.coast_required_deceleration_m_s2 = None
        self.coast_measured_deceleration_m_s2 = None
        self.coast_predicted_forward_speed_after_delay_m_s = None
        self.coast_response_horizon_s = None
        self.coast_actual_tilt_deg = None
        self.coast_handoff_state_ready = False
        self.coast_response_queue_settled = False
        self.coast_response_queue_settle_elapsed_s = None
        self.coast_response_queue_settle_required_s = None
        self.coast_command_acceleration_m_s2 = None
        self.coast_predicted_level_stop_distance_m = None
        self.coast_predicted_level_stop_time_s = None
        self.coast_predicted_level_terminal_speed_m_s = None
        self.coast_impulse_safe_deceleration_m_s2 = None
        self.coast_tail_cancellation_acceleration_m_s2 = None
        self.coast_tail_cancellation_signed_acceleration_m_s2 = None
        self.coast_tail_terminal_target_speed_m_s = None
        self.coast_predicted_terminal_after_pulse_m_s = None
        self.coast_command_hold_s = None
        self._coast_previous_position_xy = None
        self._coast_previous_velocity_xy = None
        self._coast_previous_timestamp = None
        self._coast_filtered_acceleration_xy = np.zeros(2)
        self._coast_model_acceleration_xy = np.zeros(2)
        self._coast_acceleration_valid = False
        self._coast_command_history = []
        self._last_attitude_send_timestamp = None
        self._attitude_send_intervals_s = []
        # Generic send metadata is separate from the attitude-model history.
        # It lets offline replay distinguish the command already applied at a
        # measured state from the new command sent after that state, including
        # duplicate-state resend cycles that do not produce an observer row.
        self._last_sent_command = None
        self._sent_command_sequence = 0
        self._sent_command_history = deque(maxlen=2048)
        self._level_attitude_command_started_at = None
        self._coast_level_handoff_latched = False
        self._tail_neutralization_deadline = None
        self._tail_neutralization_needs_send_anchor = False
        self.release_candidate_action = None
        self.release_candidate_predicted_level_terminal_speed_m_s = None
        self.release_candidate_tail_cancellation_acceleration_m_s2 = None
        self.release_candidate_tail_cancellation_signed_acceleration_m_s2 = None
        self.release_candidate_target_terminal_speed_m_s = None
        self.release_candidate_predicted_terminal_after_pulse_m_s = None
        self.release_candidate_command_hold_s = None
        self.coast_velocity_phase = 'inactive'
        self.coast_velocity_command_xy_m_s = np.zeros(2)
        self.coast_velocity_predicted_unwind_terminal_speed_m_s = None
        self.coast_velocity_predicted_next_step_terminal_speed_m_s = None
        self.coast_velocity_dynamic_unwind_threshold_m_s = None
        self.coast_velocity_dynamic_unwind_step_guard_m_s = None
        self.coast_velocity_unwind_decision_reason = None
        self.coast_velocity_projected_acceleration_m_s2 = None
        self.coast_velocity_unwind_response_horizon_s = None
        self.coast_velocity_unwind_observed_decision_latency_s = None
        self.coast_velocity_unwind_total_response_delay_s = None
        self.coast_velocity_unwind_position_target_m = None
        self.coast_velocity_unwind_position_progress_m = None
        self.coast_velocity_unwind_lateral_error_m = None
        self.coast_velocity_unwind_raw_integrated_velocity_delta_m_s = None
        self.coast_velocity_unwind_integrated_velocity_delta_m_s = None
        self.coast_velocity_unwind_leveling_duration_s = None
        self.coast_velocity_unwind_started_at = None
        self.coast_velocity_handoff_tilt_ready = False
        self.coast_velocity_handoff_rate_ready = False
        self.coast_velocity_handoff_speed_ready = False
        self.coast_velocity_rebrake_count = 0
        self.coast_state_sample_valid = True
        self.coast_state_rejection_reason = None
        self.coast_state_kinematic_residual_m = None
        self.coast_state_implied_acceleration_m_s2 = None
        self.coast_state_sample_gap_s = None
        self.coast_state_rejection_count = 0
        self._coast_velocity_pid_reset_pending = False
        self._coast_velocity_rebrake_pending = False
        self._coast_state_rejection_pending = None

    def _validate_calibrated_braking_direction(self, direction_xy):
        if self.coast_calibrated_direction_xy is None:
            return
        direction_xy = np.asarray(direction_xy, dtype=float)
        norm = float(np.linalg.norm(direction_xy))
        direction_matches = False
        if (
            direction_xy.shape == (2,)
            and np.all(np.isfinite(direction_xy))
            and norm > 1e-9
        ):
            unit_direction = direction_xy / norm
            direction_matches = bool(abs(float(
                unit_direction @ self.coast_calibrated_direction_xy
            )) >= 0.98)
        if (
            direction_xy.shape != (2,)
            or not np.all(np.isfinite(direction_xy))
            or norm <= 1e-9
            or not direction_matches
        ):
            raise ValueError(
                'actual release direction is outside the calibrated planar '
                'braking axis; rerun --calibrate for this direction'
            )

    def _transition_mode(self, new_mode, log_details=None):
        self.mode = new_mode
        message = {
            self.CONTACT_POSITION: 'HANDLING INTERACTION',
            self.CONTACT_ZDISTANCE: 'HANDLING INTERACTION',
            self.MPC_BOOTSTRAP_ACCELERATION: (
                'LMPC BOOTSTRAP: AUTOMATIC ACCELERATION'
            ),
            self.ATTITUDE_COAST: 'COASTING WITH ATTITUDE',
            self.VELOCITY_COAST: 'VELOCITY COAST: BRAKE / UNWIND',
            self.POSITION_COAST: 'COASTING',
            self.ATTITUDE_BRAKING: 'BRAKING',
            self.POSITION_HOLD: 'HOVER',
        }[new_mode]
        if log_details:
            message += f' | {log_details}'
        logger.info(message)

    def start_contact(
            self, render_mode='orientation', current_position=None,
            log_details=None, allow_coast_reentry=False):
        # Detector residuals during braking are expected controller/model
        # transients. Only an explicitly confirmed potentiometer recontact may
        # take command ownership back from the velocity-coast state.
        coast_reentry = bool(
            allow_coast_reentry and self.mode == self.VELOCITY_COAST
        )
        if (
            self.shadow_mode
            or (self.mode != self.POSITION_HOLD and not coast_reentry)
        ):
            return False
        if render_mode not in (
                'position', 'orientation', 'mpc_bootstrap_acceleration'):
            raise ValueError(
                'contact render mode must be position, orientation, or '
                'mpc_bootstrap_acceleration'
            )
        if current_position is not None:
            position = np.asarray(current_position, dtype=float)
            if position.shape != (3,) or not np.all(np.isfinite(position)):
                raise ValueError('contact position must be finite XYZ')
            self.hold_position = position.copy()
        self.hover_z = float(self.hold_position[2])
        self._coast_command_history = []
        self.set_contact_attitude(0.0, 0.0, 0.0)
        self._release_candidate_mode = None
        self.brake_direction.fill(0.0)
        self.brake_direction_source = None
        self.release_force_N.fill(0.0)
        self.release_momentum_kg_m_s = None
        self.release_position_m = None
        self.stopping_position_m = None
        self.release_mass_kg = None
        self.coast_stop_target_position_m = None
        self.coast_handoff_actual_position_m = None
        self.coast_target_clamped_to_actual = False
        self.coast_lateral_target_latched_to_actual = False
        self.coast_lateral_speed_m_s = None
        self.coast_target_remaining_distance_m = None
        self.coast_delay_reserved_distance_m = None
        self.coast_required_deceleration_m_s2 = None
        self.coast_measured_deceleration_m_s2 = None
        self.coast_predicted_forward_speed_after_delay_m_s = None
        self.coast_response_horizon_s = None
        self.coast_actual_tilt_deg = None
        self.coast_handoff_state_ready = False
        self.coast_response_queue_settled = False
        self.coast_response_queue_settle_elapsed_s = None
        self.coast_response_queue_settle_required_s = None
        self.coast_command_acceleration_m_s2 = None
        self.coast_predicted_level_stop_distance_m = None
        self.coast_predicted_level_stop_time_s = None
        self.coast_predicted_level_terminal_speed_m_s = None
        self.coast_impulse_safe_deceleration_m_s2 = None
        self.coast_tail_cancellation_acceleration_m_s2 = None
        self.coast_tail_cancellation_signed_acceleration_m_s2 = None
        self.coast_tail_terminal_target_speed_m_s = None
        self.coast_predicted_terminal_after_pulse_m_s = None
        self.coast_command_hold_s = None
        self._coast_previous_position_xy = None
        self._coast_previous_velocity_xy = None
        self._coast_previous_timestamp = None
        self._coast_filtered_acceleration_xy.fill(0.0)
        self._coast_model_acceleration_xy.fill(0.0)
        self._coast_acceleration_valid = False
        self._last_attitude_send_timestamp = None
        self._attitude_send_intervals_s = []
        self._level_attitude_command_started_at = None
        self._coast_level_handoff_latched = False
        self._tail_neutralization_deadline = None
        self._tail_neutralization_needs_send_anchor = False
        self.release_candidate_action = None
        self.release_candidate_predicted_level_terminal_speed_m_s = None
        self.release_candidate_tail_cancellation_acceleration_m_s2 = None
        self.release_candidate_tail_cancellation_signed_acceleration_m_s2 = None
        self.release_candidate_target_terminal_speed_m_s = None
        self.release_candidate_predicted_terminal_after_pulse_m_s = None
        self.release_candidate_command_hold_s = None
        self.coast_velocity_phase = 'inactive'
        self.coast_velocity_command_xy_m_s.fill(0.0)
        self.coast_velocity_predicted_unwind_terminal_speed_m_s = None
        self.coast_velocity_predicted_next_step_terminal_speed_m_s = None
        self.coast_velocity_dynamic_unwind_threshold_m_s = None
        self.coast_velocity_dynamic_unwind_step_guard_m_s = None
        self.coast_velocity_unwind_decision_reason = None
        self.coast_velocity_projected_acceleration_m_s2 = None
        self.coast_velocity_unwind_response_horizon_s = None
        self.coast_velocity_unwind_observed_decision_latency_s = None
        self.coast_velocity_unwind_total_response_delay_s = None
        self.coast_velocity_unwind_position_target_m = None
        self.coast_velocity_unwind_position_progress_m = None
        self.coast_velocity_unwind_lateral_error_m = None
        self.coast_velocity_unwind_raw_integrated_velocity_delta_m_s = None
        self.coast_velocity_unwind_integrated_velocity_delta_m_s = None
        self.coast_velocity_unwind_leveling_duration_s = None
        self.coast_velocity_unwind_started_at = None
        self.coast_velocity_handoff_tilt_ready = False
        self.coast_velocity_handoff_rate_ready = False
        self.coast_velocity_handoff_speed_ready = False
        self.coast_velocity_rebrake_count = 0
        self.coast_state_sample_valid = True
        self.coast_state_rejection_reason = None
        self.coast_state_kinematic_residual_m = None
        self.coast_state_implied_acceleration_m_s2 = None
        self.coast_state_sample_gap_s = None
        self.coast_state_rejection_count = 0
        self._coast_velocity_pid_reset_pending = False
        self._coast_velocity_rebrake_pending = False
        self._coast_state_rejection_pending = None
        self._transition_mode(
            (
                self.CONTACT_POSITION
                if render_mode == 'position' else
                self.MPC_BOOTSTRAP_ACCELERATION
                if render_mode == 'mpc_bootstrap_acceleration' else
                self.CONTACT_ZDISTANCE
            ),
            log_details=log_details,
        )
        return True

    def set_contact_position(self, position):
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError('contact position command must be finite XYZ')
        self.hold_position = position.copy()

    def set_contact_attitude(
            self,
            roll_deg,
            pitch_deg,
            yaw_rate_deg_s=0.0,
            yaw_deg=None,
    ):
        values = np.asarray(
            [roll_deg, pitch_deg, yaw_rate_deg_s], dtype=float
        )
        if values.shape != (3,) or not np.all(np.isfinite(values)):
            raise ValueError('contact attitude command must be finite')
        self.contact_roll_deg = float(values[0])
        self.contact_pitch_deg = float(values[1])
        self.contact_yaw_rate_deg_s = float(values[2])
        self._pending_attitude_yaw_deg = float(
            self.yaw_deg if yaw_deg is None else yaw_deg
        )

    def sent_attitude_acceleration_history(self):
        """Return a detached copy of actual sent world-XY attitude inputs."""
        return [
            (float(timestamp), np.asarray(acceleration, dtype=float).copy())
            for timestamp, acceleration in self._coast_command_history
        ]

    def acquire_external_attitude_coast(self):
        """Transfer an existing coast episode to external roll/pitch control."""
        if self.shadow_mode or self.mode not in (
                self.ATTITUDE_COAST, self.VELOCITY_COAST):
            return False
        self.set_contact_attitude(0.0, 0.0, 0.0)
        if self.mode != self.ATTITUDE_COAST:
            self._transition_mode(self.ATTITUDE_COAST)
        return True

    def set_predictive_position_target(self, position, timestamp):
        """Transfer a predictive coast episode to its position target."""
        if self.shadow_mode or self.mode not in (
                self.ATTITUDE_COAST, self.VELOCITY_COAST,
                self.POSITION_HOLD):
            return False
        position = np.asarray(position, dtype=float)
        timestamp = float(timestamp)
        if (
            position.shape != (3,)
            or not np.all(np.isfinite(position))
            or not np.isfinite(timestamp)
        ):
            raise ValueError('predictive position target must be finite XYZ')
        self.hold_position = position.copy()
        self.hover_z = float(position[2])
        self.stopping_position_m = position.copy()
        self.set_contact_attitude(0.0, 0.0, 0.0)
        self.brake_command_tilt_deg = 0.0
        self.brake_completion_reason = 'predictive_model_position_handoff'
        self._brake_started_at = None
        if self.mode != self.POSITION_HOLD:
            self._detector_rearm_at = timestamp + self.rearm_delay_s
            self._transition_mode(self.POSITION_HOLD)
        return True

    def _record_attitude_command(self, timestamp, yaw_deg):
        timestamp = float(timestamp)
        if not np.isfinite(timestamp):
            raise ValueError('attitude command timestamp must be finite')
        if self._last_attitude_send_timestamp is not None:
            send_interval_s = timestamp - self._last_attitude_send_timestamp
            if 1e-4 <= send_interval_s <= 0.10:
                self._attitude_send_intervals_s.append(float(send_interval_s))
                self._attitude_send_intervals_s = (
                    self._attitude_send_intervals_s[-8:]
                )
            elif send_interval_s < 0.0:
                self._attitude_send_intervals_s = []
        self._last_attitude_send_timestamp = timestamp
        command = attitude_to_world_acceleration(
            self.contact_roll_deg,
            self.contact_pitch_deg,
            yaw_deg,
        )
        if (
            self._coast_command_history
            and timestamp < self._coast_command_history[-1][0]
        ):
            self._coast_command_history = []
            self._level_attitude_command_started_at = None
        if (
            self._coast_command_history
            and abs(timestamp - self._coast_command_history[-1][0]) <= 1e-9
        ):
            previous_command = self._coast_command_history[-1][1]
            self._coast_command_history[-1] = (timestamp, command)
            command_is_level = bool(np.linalg.norm(command) <= 1e-9)
            previous_was_level = bool(
                np.linalg.norm(previous_command) <= 1e-9
            )
            if command_is_level and not previous_was_level:
                self._level_attitude_command_started_at = timestamp
            elif not command_is_level:
                self._level_attitude_command_started_at = None
        elif (
            self._coast_command_history
            and np.allclose(
                command,
                self._coast_command_history[-1][1],
                rtol=0.0,
                atol=1e-12,
            )
        ):
            # Re-sending an unchanged attitude does not create a new step.
            # Retaining its original timestamp is essential to model delay.
            # A handoff may seed an already-level command directly; start the
            # conservative queue-settle clock at its first real send.
            if (
                np.linalg.norm(command) <= 1e-9
                and self._level_attitude_command_started_at is None
            ):
                self._level_attitude_command_started_at = timestamp
            return
        else:
            previous_command = (
                None
                if not self._coast_command_history
                else self._coast_command_history[-1][1]
            )
            self._coast_command_history.append((timestamp, command))
            command_is_level = bool(np.linalg.norm(command) <= 1e-9)
            previous_was_level = bool(
                previous_command is not None
                and np.linalg.norm(previous_command) <= 1e-9
            )
            if command_is_level:
                if previous_command is None or not previous_was_level:
                    self._level_attitude_command_started_at = timestamp
            else:
                self._level_attitude_command_started_at = None
        history_window_s = max(
            1.0,
            self.coast_attitude_response_delay_s
            + 8.0 * self.coast_attitude_time_constant_s,
        )
        cutoff = timestamp - history_window_s
        while (
            len(self._coast_command_history) > 2
            and self._coast_command_history[1][0] < cutoff
        ):
            self._coast_command_history.pop(0)

    def _estimated_attitude_command_hold_s(self):
        """Estimate the next actual send interval without one-gap poisoning."""
        if not self._attitude_send_intervals_s:
            return float(self.coast_command_period_s)
        observed_period_s = float(np.percentile(
            self._attitude_send_intervals_s[-8:], 75.0
        ))
        return max(float(self.coast_command_period_s), observed_period_s)

    def expire_tail_neutralization(self, timestamp):
        """End a finite cancellation pulse even when state telemetry stalls."""
        timestamp = float(timestamp)
        if not np.isfinite(timestamp):
            raise ValueError('tail-neutralization timestamp must be finite')
        if (
            self._tail_neutralization_deadline is None
            or timestamp < self._tail_neutralization_deadline
        ):
            return False
        self._tail_neutralization_deadline = None
        self._tail_neutralization_needs_send_anchor = False
        self.set_contact_attitude(0.0, 0.0, 0.0)
        if self.mode == self.CONTACT_ZDISTANCE:
            self.release_candidate_action = (
                'leveling_after_tail_neutralization_pulse'
            )
        elif self.mode == self.ATTITUDE_COAST:
            self.coast_tracking_action = (
                'leveling_after_tail_neutralization_pulse'
            )
            self.coast_tracking_acceleration_m_s2 = np.zeros(2)
            self.coast_tracking_power_w_per_kg = 0.0
        return True

    def cancel_tail_neutralization(self):
        """Cancel a pending finite pulse when a release candidate rebounds."""
        pulse_was_pending = self._tail_neutralization_deadline is not None
        self._tail_neutralization_deadline = None
        self._tail_neutralization_needs_send_anchor = False
        if self.mode == self.CONTACT_ZDISTANCE:
            self.release_candidate_action = 'release_candidate_cancelled'
        return pulse_was_pending

    def update_release_candidate_attitude(
            self,
            current_velocity,
            current_orientation_rpy,
            interaction_direction,
            timestamp,
            command_timestamp=None,
    ):
        """Remove render tilt early and neutralize its calibrated response tail."""
        if self.shadow_mode or self.mode != self.CONTACT_ZDISTANCE:
            return None
        velocity = np.asarray(current_velocity, dtype=float)
        orientation_rpy = np.asarray(current_orientation_rpy, dtype=float)
        direction = np.asarray(interaction_direction, dtype=float)
        if velocity.shape not in ((2,), (3,)):
            raise ValueError('candidate velocity must contain XY or XYZ')
        if orientation_rpy.shape != (3,):
            raise ValueError('candidate orientation must contain roll/pitch/yaw')
        if direction.shape not in ((2,), (3,)):
            raise ValueError('candidate direction must contain XY or XYZ')
        if not all(np.all(np.isfinite(value)) for value in (
                velocity, orientation_rpy, direction)):
            raise ValueError('candidate state must be finite')
        timestamp = float(timestamp)
        command_timestamp = float(
            timestamp if command_timestamp is None else command_timestamp
        )
        if not np.all(np.isfinite([timestamp, command_timestamp])):
            raise ValueError('candidate command time must be finite')
        self._validate_calibrated_braking_direction(direction[:2])
        yaw_deg = float(np.degrees(orientation_rpy[2]))
        model_acceleration_xy = attitude_to_world_acceleration(
            np.degrees(orientation_rpy[0]),
            np.degrees(orientation_rpy[1]),
            yaw_deg,
        )
        if not self._coast_command_history:
            history_start = timestamp - max(
                1.0,
                self.coast_attitude_response_delay_s
                + 8.0 * self.coast_attitude_time_constant_s,
            )
            self._coast_command_history.append((
                history_start,
                model_acceleration_xy
                / self.coast_attitude_acceleration_scale,
            ))
        tracking = release_tail_neutralization_attitude(
            velocity[:2],
            direction[:2],
            yaw_deg,
            response_delay_s=self.coast_attitude_response_delay_s,
            response_time_constant_s=self.coast_attitude_time_constant_s,
            acceleration_scale=self.coast_attitude_acceleration_scale,
            terminal_speed_margin_m_s=self.coast_level_terminal_speed_m_s,
            command_hold_s=self._estimated_attitude_command_hold_s(),
            max_acceleration_m_s2=min(
                self.coast_max_acceleration_m_s2,
                self.coast_candidate_tail_cancellation_max_acceleration_m_s2,
            ),
            max_attitude_deg=self.brake_max_attitude_deg,
            measured_acceleration_xy=model_acceleration_xy,
            command_history=self._coast_command_history,
            timestamp=timestamp,
            future_command_started_at=command_timestamp,
        )
        self.set_contact_attitude(
            tracking['roll_deg'],
            tracking['pitch_deg'],
            0.0,
            yaw_deg=yaw_deg,
        )
        self.release_candidate_action = tracking['action']
        self.release_candidate_predicted_level_terminal_speed_m_s = float(
            tracking['predicted_level_terminal_speed_m_s']
        )
        self.release_candidate_tail_cancellation_acceleration_m_s2 = float(
            tracking['tail_cancellation_acceleration_m_s2']
        )
        self.release_candidate_tail_cancellation_signed_acceleration_m_s2 = (
            float(tracking['tail_cancellation_signed_acceleration_m_s2'])
        )
        self.release_candidate_target_terminal_speed_m_s = float(
            tracking['target_terminal_speed_m_s']
        )
        self.release_candidate_predicted_terminal_after_pulse_m_s = float(
            tracking['predicted_terminal_after_pulse_m_s']
        )
        self.release_candidate_command_hold_s = float(
            tracking['command_hold_s']
        )
        self._tail_neutralization_deadline = (
            command_timestamp + tracking['command_hold_s']
            if tracking['tail_cancellation_acceleration_m_s2'] > 0.0
            else None
        )
        self._tail_neutralization_needs_send_anchor = bool(
            tracking['tail_cancellation_acceleration_m_s2'] > 0.0
        )
        return tracking

    def _set_velocity_brake_attitude(
            self, projected_speed, yaw_rad,
            projected_force_n=0.0, current_mass_kg=None,
    ):
        force_feedforward = 0.0
        if current_mass_kg is not None:
            mass = float(current_mass_kg)
            if not np.isfinite(mass) or mass <= 0.0:
                raise ValueError('braking mass must be finite and positive')
            force_feedforward = max(float(projected_force_n), 0.0) / mass
        self.brake_force_feedforward_acceleration_m_s2 = force_feedforward
        desired_deceleration = min(
            self.brake_velocity_gain_s * max(float(projected_speed), 0.0)
            + force_feedforward,
            self.brake_xy_acceleration_m_s2,
        )
        raw_tilt_deg = float(np.degrees(np.arctan2(
            desired_deceleration, 9.81
        )))
        if projected_speed > self.brake_xy_speed_m_s:
            taper_fraction = min(max(
                (
                    float(projected_speed) - self.brake_xy_speed_m_s
                ) / (
                    self.brake_min_attitude_taper_speed_m_s
                    - self.brake_xy_speed_m_s
                ),
                0.0,
            ), 1.0)
            tapered_min_tilt_deg = (
                self.brake_min_attitude_deg * taper_fraction
            )
            self.brake_command_tilt_deg = min(
                max(raw_tilt_deg, tapered_min_tilt_deg),
                self.brake_max_attitude_deg,
            )
        else:
            self.brake_command_tilt_deg = 0.0
        direction_body = world_to_body_xy(
            self.brake_direction[:2], np.degrees(float(yaw_rad))
        )
        self.set_contact_attitude(
            self.brake_command_tilt_deg * float(direction_body[1]),
            self.brake_command_tilt_deg * float(direction_body[0]),
            0.0,
        )

    def end_contact(
            self,
            current_position,
            current_velocity,
            timestamp,
            interaction_direction=None,
            current_orientation_rpy=None,
            current_force=None,
            current_mass_kg=None,
            coast=False,
    ):
        if self.shadow_mode or self.mode not in (
                self.CONTACT_POSITION,
                self.CONTACT_ZDISTANCE,
                self.MPC_BOOTSTRAP_ACCELERATION,
        ):
            return False
        coast = bool(coast)
        released_from_position = self.mode == self.CONTACT_POSITION
        if (
            coast
            and released_from_position
            and self.coast_calibrated_direction_xy is not None
        ):
            raise RuntimeError(
                'calibrated predictive coasting requires orientation-rendered '
                'contact so its sent attitude-command history is observable'
            )
        self._release_candidate_mode = (
            'position' if released_from_position else 'orientation'
        )
        position = np.asarray(current_position, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError('translation release position must be finite XYZ')
        velocity = np.asarray(current_velocity, dtype=float)
        if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
            raise ValueError('translation release velocity must be finite XYZ')
        timestamp = float(timestamp)
        if not np.isfinite(timestamp):
            raise ValueError('translation release timestamp must be finite')
        orientation_rpy = np.asarray(
            [0.0, 0.0, 0.0]
            if current_orientation_rpy is None else current_orientation_rpy,
            dtype=float,
        )
        if orientation_rpy.shape != (3,) or not np.all(np.isfinite(orientation_rpy)):
            raise ValueError('translation release orientation must be finite RPY')
        force = np.asarray(
            np.zeros(3) if current_force is None else current_force,
            dtype=float,
        )
        if force.shape != (3,) or not np.all(np.isfinite(force)):
            raise ValueError('translation release force must be finite XYZ')
        mass = None if current_mass_kg is None else float(current_mass_kg)
        if mass is not None and (not np.isfinite(mass) or mass <= 0.0):
            raise ValueError('translation release mass must be positive')
        interaction_direction = (
            np.asarray(interaction_direction, dtype=float)
            if interaction_direction is not None else np.zeros(3)
        )
        if (
            interaction_direction.shape != (3,)
            or not np.all(np.isfinite(interaction_direction))
        ):
            raise ValueError('interaction direction must be finite XYZ')
        direction = interaction_direction.copy()
        direction[2] = 0.0
        interaction_direction_norm = float(np.linalg.norm(direction[:2]))
        if interaction_direction_norm > 1e-9:
            self.brake_direction_source = 'locked_interaction_direction'
        else:
            direction[:2] = velocity[:2]
            self.brake_direction_source = 'release_velocity_fallback'
        direction[2] = 0.0
        direction_norm = float(np.linalg.norm(direction[:2]))
        if direction_norm <= 1e-9:
            direction[:2] = velocity[:2]
            direction_norm = float(np.linalg.norm(direction[:2]))
        if direction_norm > 1e-9:
            direction /= direction_norm
        else:
            direction.fill(0.0)
            self.brake_direction_source = 'unknown_latch_actual_at_handoff'
        if coast:
            self._validate_calibrated_braking_direction(direction[:2])

        self.hold_position = position.copy()
        self.release_position_m = position.copy()
        self.stopping_position_m = None
        self.release_force_N = force.copy()
        self.release_mass_kg = mass
        self.release_momentum_kg_m_s = (
            None if mass is None else mass * velocity.copy()
        )
        self.brake_direction = direction.copy()
        self.brake_projected_speed_m_s = float(
            velocity[:2] @ self.brake_direction[:2]
        )
        self.hover_z = float(position[2])
        self._coast_alignment_since = None
        self._coast_position_settle_since = None
        self.coast_tracking_action = None
        self.coast_tracking_position_error_m = None
        self.coast_tracking_velocity_error_m_s = None
        self.coast_tracking_acceleration_m_s2 = None
        self.coast_tracking_acceleration_saturated = False
        self.coast_tracking_power_w_per_kg = None
        self.coast_handoff_reason = None
        self.coast_handoff_actual_position_m = None
        self.coast_target_clamped_to_actual = False
        self.coast_lateral_target_latched_to_actual = False
        self.coast_lateral_speed_m_s = None
        self.coast_response_horizon_s = None
        self.coast_actual_tilt_deg = None
        self.coast_handoff_state_ready = False
        self.coast_response_queue_settled = False
        self.coast_response_queue_settle_elapsed_s = None
        self.coast_response_queue_settle_required_s = None
        self.coast_command_acceleration_m_s2 = None
        self.coast_predicted_level_stop_distance_m = None
        self.coast_predicted_level_stop_time_s = None
        self.coast_predicted_level_terminal_speed_m_s = None
        self.coast_impulse_safe_deceleration_m_s2 = None
        self.coast_tail_cancellation_acceleration_m_s2 = None
        self.coast_tail_cancellation_signed_acceleration_m_s2 = None
        self.coast_tail_terminal_target_speed_m_s = None
        self.coast_predicted_terminal_after_pulse_m_s = None
        self.coast_command_hold_s = None
        self._coast_previous_position_xy = position[:2].copy()
        self._coast_previous_velocity_xy = velocity[:2].copy()
        self._coast_previous_timestamp = timestamp
        self._coast_filtered_acceleration_xy.fill(0.0)
        self._coast_model_acceleration_xy.fill(0.0)
        self._coast_acceleration_valid = False
        self._level_attitude_command_started_at = None
        self._coast_level_handoff_latched = False
        self._tail_neutralization_deadline = None
        self._tail_neutralization_needs_send_anchor = False
        self.coast_velocity_phase = (
            'fast_brake'
            if coast
            and self.coast_velocity_braking_enabled
            and self.coast_velocity_predictive_unwind_enabled
            else 'zero_velocity'
            if coast and self.coast_velocity_braking_enabled
            else 'inactive'
        )
        self.coast_velocity_command_xy_m_s.fill(0.0)
        self.coast_velocity_predicted_unwind_terminal_speed_m_s = None
        self.coast_velocity_predicted_next_step_terminal_speed_m_s = None
        self.coast_velocity_dynamic_unwind_threshold_m_s = None
        self.coast_velocity_dynamic_unwind_step_guard_m_s = None
        self.coast_velocity_unwind_decision_reason = None
        self.coast_velocity_projected_acceleration_m_s2 = None
        self.coast_velocity_unwind_response_horizon_s = None
        self.coast_velocity_unwind_observed_decision_latency_s = None
        self.coast_velocity_unwind_total_response_delay_s = None
        self.coast_velocity_unwind_position_target_m = None
        self.coast_velocity_unwind_position_progress_m = None
        self.coast_velocity_unwind_lateral_error_m = None
        self.coast_velocity_unwind_raw_integrated_velocity_delta_m_s = None
        self.coast_velocity_unwind_integrated_velocity_delta_m_s = None
        self.coast_velocity_unwind_leveling_duration_s = None
        self.coast_velocity_unwind_started_at = None
        self.coast_velocity_handoff_tilt_ready = False
        self.coast_velocity_handoff_rate_ready = False
        self.coast_velocity_handoff_speed_ready = False
        self.coast_velocity_rebrake_count = 0
        self.coast_state_sample_valid = True
        self.coast_state_rejection_reason = None
        self.coast_state_kinematic_residual_m = None
        self.coast_state_implied_acceleration_m_s2 = None
        self.coast_state_sample_gap_s = None
        self.coast_state_rejection_count = 0
        self._coast_velocity_pid_reset_pending = False
        self._coast_velocity_rebrake_pending = False
        self._coast_state_rejection_pending = None
        if coast:
            yaw_deg = float(np.degrees(orientation_rpy[2]))
            if not self._coast_command_history:
                prior_command = (
                    attitude_to_world_acceleration(
                        np.degrees(orientation_rpy[0]),
                        np.degrees(orientation_rpy[1]),
                        yaw_deg,
                    ) / self.coast_attitude_acceleration_scale
                    if released_from_position
                    else attitude_to_world_acceleration(
                        self.contact_roll_deg,
                        self.contact_pitch_deg,
                        yaw_deg,
                    )
                )
                history_start = timestamp - max(
                    1.0,
                    self.coast_attitude_response_delay_s
                    + 8.0 * self.coast_attitude_time_constant_s,
                )
                self._coast_command_history.append(
                    (history_start, prior_command)
                )
            self.set_contact_attitude(
                0.0,
                0.0,
                0.0,
                yaw_deg=yaw_deg,
            )
            self.brake_force_feedforward_acceleration_m_s2 = 0.0
        else:
            # Legacy observer release uses measured velocity and force for
            # bounded counter-tilt braking.
            self._set_velocity_brake_attitude(
                self.brake_projected_speed_m_s,
                orientation_rpy[2],
                projected_force_n=float(
                    force[:2] @ self.brake_direction[:2]
                ),
                current_mass_kg=mass,
            )
        self._brake_started_at = timestamp
        self.brake_completion_reason = None
        self._transition_mode(
            (
                self.VELOCITY_COAST
                if coast and self.coast_velocity_braking_enabled
                else self.ATTITUDE_COAST
                if coast
                else self.ATTITUDE_BRAKING
            )
        )
        return True

    def cancel_release_candidate(self, current_position) -> bool:
        """Resume the interrupted interaction after a transient force dip."""
        if self.shadow_mode or self._release_candidate_mode is None:
            return False
        if self.mode not in (
                self.ATTITUDE_BRAKING,
                self.ATTITUDE_COAST,
                self.POSITION_HOLD):
            return False
        position = np.asarray(current_position, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError('release-cancel position must be finite XYZ')
        render_mode = self._release_candidate_mode
        self._release_candidate_mode = None
        self._brake_started_at = None
        self._detector_rearm_at = None
        self.brake_completion_reason = None
        self.brake_command_tilt_deg = 0.0
        self.brake_force_feedforward_acceleration_m_s2 = 0.0
        self._coast_previous_position_xy = None
        self._coast_previous_velocity_xy = None
        self._coast_previous_timestamp = None
        self._coast_filtered_acceleration_xy.fill(0.0)
        self._coast_model_acceleration_xy.fill(0.0)
        self._coast_acceleration_valid = False
        self._level_attitude_command_started_at = None
        self._coast_level_handoff_latched = False
        self._tail_neutralization_deadline = None
        self._tail_neutralization_needs_send_anchor = False
        self.release_candidate_action = None
        self.release_candidate_predicted_level_terminal_speed_m_s = None
        self.release_candidate_tail_cancellation_acceleration_m_s2 = None
        self.release_candidate_tail_cancellation_signed_acceleration_m_s2 = None
        self.release_candidate_target_terminal_speed_m_s = None
        self.release_candidate_predicted_terminal_after_pulse_m_s = None
        self.release_candidate_command_hold_s = None
        self.coast_velocity_predicted_unwind_terminal_speed_m_s = None
        self.coast_velocity_predicted_next_step_terminal_speed_m_s = None
        self.coast_velocity_dynamic_unwind_threshold_m_s = None
        self.coast_velocity_dynamic_unwind_step_guard_m_s = None
        self.coast_velocity_unwind_decision_reason = None
        self.coast_state_sample_valid = True
        self.coast_state_rejection_reason = None
        self.coast_state_kinematic_residual_m = None
        self.coast_state_implied_acceleration_m_s2 = None
        self.coast_state_sample_gap_s = None
        self.coast_state_rejection_count = 0
        self._coast_state_rejection_pending = None
        self.hold_position = position.copy()
        self.hover_z = float(position[2])
        self.set_contact_attitude(0.0, 0.0, 0.0)
        self._transition_mode(
            self.CONTACT_POSITION
            if render_mode == 'position' else self.CONTACT_ZDISTANCE
        )
        return True

    def confirm_release_candidate(
            self,
            current_position=None,
            current_velocity=None,
            current_force=None,
            timestamp=None,
    ) -> None:
        """Make an early release handoff permanent after detector dwell."""
        if current_position is not None:
            position = np.asarray(current_position, dtype=float)
            if position.shape != (3,) or not np.all(np.isfinite(position)):
                raise ValueError('confirmed release position must be finite XYZ')
            self.release_position_m = position.copy()
        if current_velocity is not None:
            velocity = np.asarray(current_velocity, dtype=float)
            if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
                raise ValueError('confirmed release velocity must be finite XYZ')
            if self.release_mass_kg is not None:
                self.release_momentum_kg_m_s = (
                    self.release_mass_kg * velocity.copy()
                )
        if current_force is not None:
            force = np.asarray(current_force, dtype=float)
            if force.shape != (3,) or not np.all(np.isfinite(force)):
                raise ValueError('confirmed release force must be finite XYZ')
            self.release_force_N = force.copy()
        if timestamp is not None:
            timestamp = float(timestamp)
            if not np.isfinite(timestamp):
                raise ValueError('confirmed release timestamp must be finite')
            self._brake_started_at = timestamp
            self._coast_alignment_since = None
            # A reversible release candidate may already have sent level
            # commands.  The requested 0.30 s interval starts no earlier than
            # the confirmed release and is anchored by the next actual send.
            self._level_attitude_command_started_at = None
        self._release_candidate_mode = None

    def _update_predictive_unwind_position_target(
            self, position, response_delay_s):
        """Keep a delay-compensated capture point on the release line."""
        if self.release_position_m is None:
            raise RuntimeError(
                'position unwind requires a latched release position'
            )
        direction_xy = self.brake_direction[:2].copy()
        direction_norm = float(np.linalg.norm(direction_xy))
        if direction_norm <= 1e-9:
            raise RuntimeError(
                'position unwind requires a locked interaction direction'
            )
        direction_xy /= direction_norm
        lateral_xy = np.array([-direction_xy[1], direction_xy[0]])
        release_xy = self.release_position_m[:2]
        offset_xy = position[:2] - release_xy
        current_progress_m = float(offset_xy @ direction_xy)
        current_lateral_error_m = float(offset_xy @ lateral_xy)

        # Place the target where the current state is expected to be when this
        # command can affect the vehicle. Never move that scalar target back
        # toward release: if residual attitude starts a reversal, position
        # control continues pulling toward the furthest capture point instead
        # of following the reverse motion.
        candidate_progress_m = float(
            current_progress_m
            + max(self.brake_projected_speed_m_s, 0.0) * response_delay_s
        )
        previous_progress_m = 0.0
        if self.coast_velocity_unwind_position_target_m is not None:
            previous_progress_m = float(
                (
                    self.coast_velocity_unwind_position_target_m[:2]
                    - release_xy
                ) @ direction_xy
            )
        target_progress_m = max(
            0.0,
            current_progress_m,
            candidate_progress_m,
            previous_progress_m,
        )
        target = self.release_position_m.copy()
        target[:2] = release_xy + target_progress_m * direction_xy
        target[2] = self.velocity_coast_fixed_zdistance_m
        self.hold_position = target.copy()
        self.stopping_position_m = target.copy()
        self.coast_velocity_unwind_position_target_m = target.copy()
        self.coast_velocity_unwind_position_progress_m = target_progress_m
        self.coast_velocity_unwind_lateral_error_m = (
            current_lateral_error_m
        )
        self.coast_tracking_position_error_m = target - position
        self.coast_tracking_velocity_error_m_s = None
        self.coast_target_clamped_to_actual = False
        self.coast_lateral_target_latched_to_actual = False
        self.coast_tracking_action = (
            'track_delay_compensated_release_line_position'
        )

    def update_coast_velocity(
            self,
            current_position,
            current_velocity,
            timestamp,
            current_orientation_rpy=None,
            current_angular_velocity=None,
            allow_position_handoff=True,
            command_timestamp=None,
    ):
        """Brake in velocity mode and hand off only after a level unwind.

        The legacy experimental path remains a zero-velocity command followed
        by a speed-only handoff.  The predictive-unwind variant first uses that
        same aggressive command, then either changes the velocity target to the
        measured velocity or transfers to a delay-compensated position target
        on the release line before the zero crossing. The position variant
        resets both cascade integrators once at that transition and keeps its
        longitudinal target from following a later reversal.
        """
        if self.shadow_mode or self.mode != self.VELOCITY_COAST:
            return False
        position = np.asarray(current_position, dtype=float)
        velocity = np.asarray(current_velocity, dtype=float)
        orientation_rpy = np.asarray(
            [0.0, 0.0, 0.0]
            if current_orientation_rpy is None else current_orientation_rpy,
            dtype=float,
        )
        angular_velocity = np.asarray(
            [0.0, 0.0, 0.0]
            if current_angular_velocity is None else current_angular_velocity,
            dtype=float,
        )
        timestamp = float(timestamp)
        command_timestamp = float(
            timestamp if command_timestamp is None else command_timestamp
        )
        if (
            position.shape != (3,)
            or velocity.shape != (3,)
            or orientation_rpy.shape != (3,)
            or angular_velocity.shape != (3,)
            or not np.all(np.isfinite(position))
            or not np.all(np.isfinite(velocity))
            or not np.all(np.isfinite(orientation_rpy))
            or not np.all(np.isfinite(angular_velocity))
            or not np.isfinite(timestamp)
            or not np.isfinite(command_timestamp)
        ):
            raise ValueError(
                'velocity coast state must be finite XYZ/RPY/angular-rate'
            )

        previous_position_xy = self._coast_previous_position_xy
        previous_velocity_xy = self._coast_previous_velocity_xy
        previous_timestamp = self._coast_previous_timestamp
        sample_gap_s = (
            None
            if previous_timestamp is None
            else timestamp - previous_timestamp
        )
        coast_update_dt_s = (
            self.coast_command_period_s
            if sample_gap_s is None or sample_gap_s <= 0.0
            else sample_gap_s
        )
        self.coast_state_sample_valid = True
        self.coast_state_rejection_reason = None
        self.coast_state_kinematic_residual_m = None
        self.coast_state_implied_acceleration_m_s2 = None
        self.coast_state_sample_gap_s = sample_gap_s

        if (
            self.coast_state_kinematic_guard_enabled
            and previous_position_xy is not None
            and previous_velocity_xy is not None
            and previous_timestamp is not None
        ):
            if sample_gap_s <= 0.0:
                rejection_reason = 'non_monotonic_timestamp'
            elif sample_gap_s > self.coast_state_max_sample_gap_s:
                rejection_reason = 'sample_gap'
            else:
                expected_displacement_xy = 0.5 * (
                    previous_velocity_xy + velocity[:2]
                ) * sample_gap_s
                measured_displacement_xy = (
                    position[:2] - previous_position_xy
                )
                self.coast_state_kinematic_residual_m = float(np.linalg.norm(
                    measured_displacement_xy - expected_displacement_xy
                ))
                self.coast_state_implied_acceleration_m_s2 = float(
                    np.linalg.norm(
                        velocity[:2] - previous_velocity_xy
                    ) / sample_gap_s
                )
                if (
                    self.coast_state_kinematic_residual_m
                    > self.coast_state_max_kinematic_residual_m
                ):
                    rejection_reason = 'position_velocity_inconsistency'
                elif (
                    self.coast_state_implied_acceleration_m_s2
                    > self.coast_state_max_implied_acceleration_m_s2
                ):
                    rejection_reason = 'implied_acceleration'
                else:
                    rejection_reason = None
            if rejection_reason is not None:
                self.coast_state_sample_valid = False
                self.coast_state_rejection_reason = rejection_reason
                self.coast_state_rejection_count += 1
                self._coast_state_rejection_pending = {
                    'reason': rejection_reason,
                    'sample_gap_s': sample_gap_s,
                    'kinematic_residual_m': (
                        self.coast_state_kinematic_residual_m
                    ),
                    'implied_acceleration_m_s2': (
                        self.coast_state_implied_acceleration_m_s2
                    ),
                    'position_xy_m': position[:2].tolist(),
                    'velocity_xy_m_s': velocity[:2].tolist(),
                    'previous_position_xy_m': previous_position_xy.tolist(),
                    'previous_velocity_xy_m_s': previous_velocity_xy.tolist(),
                    'rejection_count': self.coast_state_rejection_count,
                }
                self._coast_alignment_since = None
                self.coast_handoff_state_ready = False
                self.coast_response_queue_settled = False
                self.coast_response_queue_settle_elapsed_s = None
                self.coast_velocity_handoff_speed_ready = False
                self.coast_velocity_handoff_tilt_ready = False
                self.coast_velocity_handoff_rate_ready = False
                self.coast_tracking_action = (
                    'hold_previous_command_invalid_coast_state'
                )

        # Rebase even a rejected sample. This prevents a single estimator jump
        # from poisoning every subsequent comparison while the already-active
        # safe coast command remains unchanged for the rejected update.
        self._coast_previous_position_xy = position[:2].copy()
        self._coast_previous_velocity_xy = velocity[:2].copy()
        self._coast_previous_timestamp = timestamp
        if not self.coast_state_sample_valid:
            return False

        xy_speed = float(np.linalg.norm(velocity[:2]))
        self.brake_projected_speed_m_s = float(
            velocity[:2] @ self.brake_direction[:2]
        )
        self.coast_lateral_speed_m_s = float(np.sqrt(max(
            0.0,
            xy_speed ** 2 - self.brake_projected_speed_m_s ** 2,
        )))
        self.coast_actual_tilt_deg = float(np.degrees(np.arccos(np.clip(
            np.cos(orientation_rpy[0]) * np.cos(orientation_rpy[1]),
            -1.0,
            1.0,
        ))))
        self._coast_model_acceleration_xy = (
            self.coast_attitude_acceleration_scale
            * attitude_to_world_acceleration(
                np.degrees(orientation_rpy[0]),
                np.degrees(orientation_rpy[1]),
                np.degrees(orientation_rpy[2]),
            )
        )
        projected_acceleration = float(
            self._coast_model_acceleration_xy @ self.brake_direction[:2]
        )
        self.coast_velocity_projected_acceleration_m_s2 = (
            projected_acceleration
        )
        self.coast_tracking_acceleration_m_s2 = None
        self.coast_tracking_acceleration_saturated = False
        self.coast_tracking_power_w_per_kg = None

        if not self.coast_velocity_predictive_unwind_enabled:
            self.coast_velocity_phase = 'zero_velocity'
            self.coast_velocity_command_xy_m_s.fill(0.0)
            self.coast_tracking_action = 'zero_world_velocity_command'
            self.coast_tracking_velocity_error_m_s = -velocity[:2].copy()
            self.coast_handoff_state_ready = bool(
                xy_speed <= self.coast_velocity_handoff_speed_m_s
            )
            if not self.coast_handoff_state_ready or not bool(
                    allow_position_handoff):
                return False
            handoff_reason = 'velocity_zero_position_handoff'
        else:
            # The estimator state is already old when this decision is made.
            # Add both the observed state-to-decision latency and the measured
            # brake-to-unwind command-switch delay to the calibrated attitude
            # response delay before integrating the remaining braking impulse.
            # All three terms therefore share the state-time reference.
            decision_latency_s = max(0.0, command_timestamp - timestamp)
            response_delay_s = (
                self.coast_attitude_response_delay_s
                + self.coast_velocity_unwind_command_switch_delay_s
                + decision_latency_s
            )
            self.coast_velocity_unwind_observed_decision_latency_s = (
                decision_latency_s
            )
            self.coast_velocity_unwind_total_response_delay_s = (
                response_delay_s
            )
            if self.coast_velocity_unwind_integrated_leveling_enabled:
                leveling_prediction = (
                    integrate_rate_limited_leveling_velocity_delta(
                        orientation_rpy,
                        angular_velocity,
                        self.brake_direction[:2],
                        response_delay_s,
                        self.coast_velocity_unwind_leveling_rate_deg_s,
                        self.coast_velocity_unwind_integration_step_s,
                        acceleration_scale=(
                            self.coast_attitude_acceleration_scale
                        ),
                        attitude_limit_deg=self.brake_max_attitude_deg,
                    )
                )
                raw_tail_velocity_delta = leveling_prediction[
                    'velocity_delta_m_s'
                ]
                # The rate-limited kinematic model under-predicted the
                # measured post-switch braking tail in real flight. Keep the
                # physical integration intact, then apply a dedicated,
                # auditable calibration only to its longitudinal impulse.
                tail_velocity_delta = raw_tail_velocity_delta
                if raw_tail_velocity_delta < 0.0:
                    tail_velocity_delta *= (
                        self.coast_velocity_unwind_tail_calibration_scale
                    )
                response_horizon_s = leveling_prediction['duration_s']
                future_projected_acceleration = leveling_prediction[
                    'final_projected_acceleration_m_s2'
                ]
                self.coast_velocity_unwind_raw_integrated_velocity_delta_m_s = (
                    raw_tail_velocity_delta
                )
                self.coast_velocity_unwind_integrated_velocity_delta_m_s = (
                    tail_velocity_delta
                )
                self.coast_velocity_unwind_leveling_duration_s = (
                    response_horizon_s
                )
            else:
                response_decay_s = (
                    self.coast_attitude_time_constant_s
                    + self.coast_velocity_unwind_prediction_margin_s
                )
                response_horizon_s = response_delay_s + response_decay_s
                future_orientation = orientation_rpy.copy()
                future_orientation[:2] += (
                    angular_velocity[:2] * response_delay_s
                )
                attitude_limit_rad = np.radians(self.brake_max_attitude_deg)
                future_orientation[:2] = np.clip(
                    future_orientation[:2],
                    -attitude_limit_rad,
                    attitude_limit_rad,
                )
                future_acceleration_xy = (
                    self.coast_attitude_acceleration_scale
                    * attitude_to_world_acceleration(
                        np.degrees(future_orientation[0]),
                        np.degrees(future_orientation[1]),
                        np.degrees(future_orientation[2]),
                    )
                )
                future_projected_acceleration = float(
                    future_acceleration_xy @ self.brake_direction[:2]
                )
                tail_velocity_delta = (
                    0.5 * (
                        projected_acceleration + future_projected_acceleration
                    ) * response_delay_s
                    + future_projected_acceleration * response_decay_s
                )
                self.coast_velocity_unwind_raw_integrated_velocity_delta_m_s = (
                    None
                )
                self.coast_velocity_unwind_integrated_velocity_delta_m_s = None
                self.coast_velocity_unwind_leveling_duration_s = None
            self.coast_velocity_unwind_response_horizon_s = response_horizon_s
            predicted_terminal_speed = float(
                self.brake_projected_speed_m_s + tail_velocity_delta
            )
            self.coast_velocity_predicted_unwind_terminal_speed_m_s = (
                predicted_terminal_speed
            )

            # Compensate for one fixed decision/send interval using the
            # current longitudinal deceleration.  Unwind when the predicted
            # attitude tail is below ``terminal + deceleration * delay``.
            # This keeps the guard interpretable and prevents angular-rate
            # extrapolation from producing a very large early-unwind margin.
            one_step_dt_s = (
                self.coast_velocity_unwind_one_step_max_dt_s
            )
            current_deceleration_m_s2 = max(
                -projected_acceleration,
                0.0,
            )
            dynamic_step_guard = (
                current_deceleration_m_s2 * one_step_dt_s
            )
            predicted_next_step_terminal_speed = float(
                predicted_terminal_speed - dynamic_step_guard
            )
            self.coast_velocity_predicted_next_step_terminal_speed_m_s = (
                predicted_next_step_terminal_speed
            )
            self.coast_velocity_dynamic_unwind_step_guard_m_s = (
                dynamic_step_guard
            )
            self.coast_velocity_dynamic_unwind_threshold_m_s = (
                self.coast_velocity_unwind_terminal_speed_m_s
                + (
                    dynamic_step_guard
                    if self.coast_velocity_unwind_one_step_lookahead_enabled
                    else 0.0
                )
            )
            angular_rate_xy_deg_s = float(np.linalg.norm(
                np.degrees(angular_velocity[:2])
            ))

            # A deliberately conservative early unwind can leave useful
            # forward velocity after the vehicle has become level. Re-enter a
            # short zero-velocity brake only from a settled attitude and only
            # for residual speed along the locked interaction direction.
            # Lateral drift may delay handoff, but must not trigger a full
            # longitudinal re-brake.
            rebrake_started = False
            if (
                self.coast_velocity_phase == 'predictive_unwind'
                and self.coast_velocity_rebrake_enabled
                and not self.coast_velocity_unwind_position_control_enabled
                and self.brake_projected_speed_m_s
                >= self.coast_velocity_rebrake_speed_m_s
                and self.coast_actual_tilt_deg
                <= self.coast_handoff_max_tilt_deg
                and angular_rate_xy_deg_s
                <= self.coast_velocity_handoff_max_rate_deg_s
            ):
                self.coast_velocity_phase = 'fast_brake'
                self.coast_velocity_command_xy_m_s.fill(0.0)
                self.coast_velocity_rebrake_count += 1
                self._coast_velocity_rebrake_pending = True
                self._coast_alignment_since = None
                self.coast_velocity_unwind_decision_reason = None
                rebrake_started = True

            # Always send at least one zero-velocity sample after a re-brake.
            # Without this guard, a tail prediction can switch back to unwind
            # in the same update and no actual braking command is transmitted.
            if self.coast_velocity_phase == 'fast_brake' and not rebrake_started:
                deceleration_ready = bool(
                    min(
                        projected_acceleration,
                        future_projected_acceleration,
                    )
                    <= -self.coast_velocity_unwind_min_deceleration_m_s2
                )
                fixed_tail_ready = bool(
                    deceleration_ready
                    and predicted_terminal_speed
                    <= self.coast_velocity_unwind_terminal_speed_m_s
                )
                one_step_tail_ready = bool(
                    self.coast_velocity_unwind_one_step_lookahead_enabled
                    and deceleration_ready
                    and predicted_terminal_speed
                    <= self.coast_velocity_dynamic_unwind_threshold_m_s
                )
                low_speed_fallback = bool(
                    xy_speed
                    <= self.coast_velocity_unwind_low_speed_fallback_m_s
                )
                if (
                    fixed_tail_ready
                    or one_step_tail_ready
                    or low_speed_fallback
                ):
                    self.coast_velocity_phase = 'predictive_unwind'
                    self.coast_velocity_unwind_started_at = timestamp
                    self.coast_velocity_command_xy_m_s = velocity[:2].copy()
                    self._coast_velocity_pid_reset_pending = True
                    self._coast_alignment_since = None
                    self.coast_velocity_unwind_decision_reason = (
                        'current_tail_prediction'
                        if fixed_tail_ready
                        else 'one_step_tail_prediction'
                        if one_step_tail_ready
                        else 'low_speed_fallback'
                    )

            if self.coast_velocity_phase == 'predictive_unwind':
                if self.coast_velocity_unwind_direct_level_attitude_enabled:
                    self.coast_velocity_command_xy_m_s.fill(0.0)
                    self.set_contact_attitude(0.0, 0.0, 0.0)
                    self.hover_z = self.velocity_coast_fixed_zdistance_m
                    self.coast_tracking_action = (
                        'direct_level_attitude_unwind'
                    )
                elif self.coast_velocity_unwind_position_control_enabled:
                    self.coast_velocity_command_xy_m_s.fill(0.0)
                    self._update_predictive_unwind_position_target(
                        position,
                        response_delay_s,
                    )
                else:
                    update_dt = coast_update_dt_s
                    alpha = 1.0 - np.exp(
                        -update_dt
                        / self.coast_velocity_unwind_filter_time_constant_s
                    )
                    filtered_target = (
                        alpha * velocity[:2]
                        + (1.0 - alpha)
                        * self.coast_velocity_command_xy_m_s
                    )
                    target_error = filtered_target - velocity[:2]
                    target_error_norm = float(np.linalg.norm(target_error))
                    if (
                        target_error_norm
                        > self.coast_velocity_unwind_max_target_error_m_s
                    ):
                        target_error *= (
                            self.coast_velocity_unwind_max_target_error_m_s
                            / target_error_norm
                        )
                    self.coast_velocity_command_xy_m_s = (
                        velocity[:2] + target_error
                    )
                    if (
                        self.brake_projected_speed_m_s
                        < self.coast_velocity_handoff_min_projected_speed_m_s
                    ):
                        # Do not let measured-velocity tracking settle below
                        # the signed handoff boundary. Correct only to that
                        # boundary; using the unwind terminal-speed target here
                        # would turn a tiny negative estimate into an
                        # unnecessary +0.10 m/s forward command. Lateral
                        # velocity remains bumpless.
                        brake_direction_xy = self.brake_direction[:2].copy()
                        direction_norm = float(
                            np.linalg.norm(brake_direction_xy)
                        )
                        if direction_norm > 1e-9:
                            brake_direction_xy /= direction_norm
                            recovery_error_m_s = min(
                                self.coast_velocity_unwind_max_target_error_m_s,
                                max(
                                    0.0,
                                    self.coast_velocity_handoff_min_projected_speed_m_s
                                    - self.brake_projected_speed_m_s,
                                ),
                            )
                            self.coast_velocity_command_xy_m_s = (
                                velocity[:2]
                                + recovery_error_m_s * brake_direction_xy
                            )
                        self.coast_tracking_action = (
                            'recover_reverse_velocity_while_unwinding'
                        )
                    else:
                        self.coast_tracking_action = (
                            'track_measured_velocity_to_unwind_attitude'
                        )
            else:
                self.coast_velocity_command_xy_m_s.fill(0.0)
                self.coast_tracking_action = (
                    'predictive_zero_world_velocity_rebrake'
                    if self.coast_velocity_rebrake_count > 0
                    else 'predictive_zero_world_velocity_brake'
                )

            if not (
                (
                    self.coast_velocity_unwind_position_control_enabled
                    or self.coast_velocity_unwind_direct_level_attitude_enabled
                )
                and self.coast_velocity_phase == 'predictive_unwind'
            ):
                self.coast_tracking_velocity_error_m_s = (
                    self.coast_velocity_command_xy_m_s - velocity[:2]
                )
            self.coast_velocity_handoff_speed_ready = bool(
                xy_speed <= self.coast_velocity_handoff_speed_m_s
                and self.brake_projected_speed_m_s
                >= self.coast_velocity_handoff_min_projected_speed_m_s
            )
            self.coast_velocity_handoff_tilt_ready = bool(
                self.coast_actual_tilt_deg <= self.coast_handoff_max_tilt_deg
            )
            self.coast_velocity_handoff_rate_ready = bool(
                angular_rate_xy_deg_s
                <= self.coast_velocity_handoff_max_rate_deg_s
            )
            gate_ready = bool(
                self.coast_velocity_phase == 'predictive_unwind'
                and self.coast_velocity_handoff_speed_ready
                and self.coast_velocity_handoff_tilt_ready
                and self.coast_velocity_handoff_rate_ready
                and bool(allow_position_handoff)
            )
            if gate_ready:
                if self._coast_alignment_since is None:
                    self._coast_alignment_since = timestamp
            else:
                self._coast_alignment_since = None
            self.coast_response_queue_settle_required_s = (
                self.coast_alignment_dwell_s
            )
            self.coast_response_queue_settle_elapsed_s = (
                None
                if self._coast_alignment_since is None
                else max(timestamp - self._coast_alignment_since, 0.0)
            )
            self.coast_response_queue_settled = bool(
                self.coast_response_queue_settle_elapsed_s is not None
                and self.coast_response_queue_settle_elapsed_s
                >= self.coast_alignment_dwell_s
            )
            self.coast_handoff_state_ready = (
                self.coast_response_queue_settled
            )
            if not self.coast_handoff_state_ready:
                return False
            handoff_reason = (
                'velocity_predictive_unwind_attitude_handoff'
                if self.coast_velocity_unwind_direct_level_attitude_enabled
                else 'velocity_predictive_unwind_position_handoff'
            )

        position_unwind_active = bool(
            self.coast_velocity_unwind_position_control_enabled
            and self.coast_velocity_phase == 'predictive_unwind'
            and self.coast_velocity_unwind_position_target_m is not None
        )
        interaction_direction_xy = self.brake_direction[:2].copy()
        interaction_direction_norm = float(np.linalg.norm(
            interaction_direction_xy
        ))
        offset_handoff_target = bool(
            self.coast_velocity_handoff_position_offset_m > 0.0
            and interaction_direction_norm > 1e-9
        )
        if offset_handoff_target:
            interaction_direction_xy /= interaction_direction_norm
            self.hold_position = position.copy()
            self.hold_position[:2] += (
                self.coast_velocity_handoff_position_offset_m
                * interaction_direction_xy
            )
            self.hover_z = float(position[2])
            self.stopping_position_m = self.hold_position.copy()
            self.coast_target_clamped_to_actual = False
            self.coast_lateral_target_latched_to_actual = True
        elif position_unwind_active:
            self.hold_position = (
                self.coast_velocity_unwind_position_target_m.copy()
            )
            self.hover_z = self.velocity_coast_fixed_zdistance_m
            self.stopping_position_m = self.hold_position.copy()
            self.coast_target_clamped_to_actual = False
            self.coast_lateral_target_latched_to_actual = False
        else:
            self.hold_position = position.copy()
            self.hover_z = float(position[2])
            self.stopping_position_m = self.hold_position.copy()
            self.coast_target_clamped_to_actual = True
            self.coast_lateral_target_latched_to_actual = True
        self.coast_handoff_actual_position_m = position.copy()
        self.coast_response_queue_settled = True
        self.coast_velocity_phase = 'position_handoff'
        self.coast_handoff_reason = handoff_reason
        self.brake_completion_reason = self.coast_handoff_reason
        self._brake_started_at = None
        self._detector_rearm_at = timestamp + self.rearm_delay_s
        self._transition_mode(self.POSITION_HOLD)
        return True

    def consume_velocity_pid_reset_request(self):
        """Return true once when predictive unwind needs a bumpless reset."""
        pending = bool(self._coast_velocity_pid_reset_pending)
        self._coast_velocity_pid_reset_pending = False
        return pending

    def consume_velocity_rebrake_request(self):
        """Return true once when a settled but fast coast resumes braking."""
        pending = bool(self._coast_velocity_rebrake_pending)
        self._coast_velocity_rebrake_pending = False
        return pending

    def consume_coast_state_rejection(self):
        """Return one rejected coast sample for event logging."""
        pending = self._coast_state_rejection_pending
        self._coast_state_rejection_pending = None
        return pending

    def update_coast_attitude(
            self,
            current_position,
            current_velocity,
            target_position,
            target_velocity,
            timestamp,
            current_orientation_rpy=None,
            allow_position_handoff=True,
            latch_current_position_on_handoff=False,
            command_timestamp=None,
    ):
        """Brake with attitude toward a stop target frozen at release.

        Returns true only on the sample that transitions to position control.
        Once signed speed reaches ``coast_level_handoff_speed_m_s``, attitude
        is latched level and cannot resume reverse-motion damping. Position
        handoff follows ``coast_level_handoff_delay_s`` after the first actual
        level send (and never before release confirmation).
        The stop target is retained only along the interaction axis; the
        measured perpendicular coordinate is latched at handoff. If measured
        motion already passed the stop, the complete measured position is
        latched to preserve the no-pullback invariant.
        """
        if self.shadow_mode or self.mode != self.ATTITUDE_COAST:
            return False
        position = np.asarray(current_position, dtype=float)
        velocity = np.asarray(current_velocity, dtype=float)
        target_position = np.asarray(target_position, dtype=float)
        target_velocity = np.asarray(target_velocity, dtype=float)
        if any(value.shape != (3,) for value in (
                position, velocity, target_position, target_velocity)):
            raise ValueError('coast attitude states must contain XYZ')
        if not all(np.all(np.isfinite(value)) for value in (
                position, velocity, target_position, target_velocity)):
            raise ValueError('coast attitude states must be finite')
        timestamp = float(timestamp)
        command_timestamp = float(
            timestamp if command_timestamp is None else command_timestamp
        )
        orientation_rpy = np.asarray(
            [0.0, 0.0, 0.0]
            if current_orientation_rpy is None else current_orientation_rpy,
            dtype=float,
        )
        if (
            not np.all(np.isfinite([timestamp, command_timestamp]))
            or orientation_rpy.shape != (3,)
            or not np.all(np.isfinite(orientation_rpy))
        ):
            raise ValueError('coast attitude time/orientation must be finite')

        # The frozen stop target shapes longitudinal braking and becomes the
        # position target after handoff when it is still ahead of the vehicle.
        self.coast_tracking_position_error_m = (
            target_position[:2] - position[:2]
        )
        self.coast_tracking_velocity_error_m_s = (
            target_velocity[:2] - velocity[:2]
        )
        self.brake_projected_speed_m_s = float(
            velocity[:2] @ self.brake_direction[:2]
        )
        brake_direction_norm = float(np.linalg.norm(
            self.brake_direction[:2]
        ))
        if brake_direction_norm > 1e-9:
            brake_direction_xy = (
                self.brake_direction[:2] / brake_direction_norm
            )
            lateral_velocity_xy = (
                velocity[:2]
                - self.brake_projected_speed_m_s * brake_direction_xy
            )
            lateral_speed = float(np.linalg.norm(lateral_velocity_xy))
        else:
            # With no trustworthy release axis, retain the original full-XY
            # stop gate and latch the complete measured position at handoff.
            brake_direction_xy = np.zeros(2)
            lateral_speed = 0.0
        self.coast_lateral_speed_m_s = lateral_speed
        if (
            not self._coast_level_handoff_latched
            and self.brake_projected_speed_m_s
            <= self.coast_level_handoff_speed_m_s
        ):
            self._coast_level_handoff_latched = True
            self._level_attitude_command_started_at = None
            self._coast_alignment_since = None
            if self.coast_direct_position_handoff:
                # Transfer ownership directly to the native position
                # controller. Latch the measured pose rather than the frozen
                # virtual stop so the handoff cannot request a forward pull or
                # stale-target correction. No level attitude is sent first.
                self.coast_stop_target_position_m = target_position.copy()
                self.coast_target_remaining_distance_m = float(
                    (target_position[:2] - position[:2]) @ brake_direction_xy
                )
                self.coast_delay_reserved_distance_m = 0.0
                self.coast_required_deceleration_m_s2 = 0.0
                self.coast_command_acceleration_m_s2 = np.zeros(2)
                self.coast_actual_tilt_deg = float(np.degrees(np.arccos(
                    np.clip(
                        np.cos(orientation_rpy[0])
                        * np.cos(orientation_rpy[1]),
                        -1.0,
                        1.0,
                    )
                )))
                self.coast_handoff_actual_position_m = position.copy()
                self.coast_target_clamped_to_actual = True
                self.coast_lateral_target_latched_to_actual = True
                self.hold_position = position.copy()
                self.hover_z = float(position[2])
                self.stopping_position_m = position.copy()
                self.set_contact_attitude(0.0, 0.0, 0.0)
                self.brake_command_tilt_deg = 0.0
                self.coast_tracking_action = 'direct_position_handoff'
                self.coast_tracking_acceleration_m_s2 = np.zeros(2)
                self.coast_tracking_acceleration_saturated = False
                self.coast_tracking_power_w_per_kg = 0.0
                self.coast_response_queue_settled = False
                self.coast_handoff_state_ready = True
                self.coast_handoff_reason = (
                    'direct_current_position_handoff'
                )
                self.brake_completion_reason = self.coast_handoff_reason
                self._brake_started_at = None
                self._detector_rearm_at = timestamp + self.rearm_delay_s
                self._transition_mode(self.POSITION_HOLD)
                return True
            # The legacy timer is based on the first real level command sent
            # after the speed threshold, not on this state timestamp.
        attitude_timed_out = bool(
            self._brake_started_at is not None
            and timestamp - self._brake_started_at
            >= min(self.coast_attitude_timeout_s, self.brake_timeout_s)
        )
        acceleration_dt = None
        if (
            self._coast_previous_velocity_xy is not None
            and self._coast_previous_timestamp is not None
        ):
            acceleration_dt = timestamp - self._coast_previous_timestamp
            if 1e-4 <= acceleration_dt <= 0.10:
                raw_acceleration = (
                    velocity[:2] - self._coast_previous_velocity_xy
                ) / acceleration_dt
                raw_acceleration = np.clip(
                    raw_acceleration,
                    -2.0 * self.coast_max_acceleration_m_s2,
                    2.0 * self.coast_max_acceleration_m_s2,
                )
                filter_alpha = 1.0 - np.exp(
                    -acceleration_dt
                    / self.coast_acceleration_filter_time_constant_s
                )
                self._coast_filtered_acceleration_xy = (
                    filter_alpha * raw_acceleration
                    + (1.0 - filter_alpha)
                    * self._coast_filtered_acceleration_xy
                )
                self._coast_acceleration_valid = True
            else:
                self._coast_filtered_acceleration_xy.fill(0.0)
                self._coast_acceleration_valid = False
                self._coast_alignment_since = None
        self._coast_previous_velocity_xy = velocity[:2].copy()
        self._coast_previous_timestamp = timestamp
        # The predictor already contains the calibrated command-to-attitude
        # lag. Feeding it a second slow velocity-difference filter would count
        # that lag twice and can trigger a late max-brake pulse. Actual tilt is
        # the instantaneous plant state; retain the slower velocity-derived
        # acceleration separately for the noise-tolerant handoff gate.
        self._coast_model_acceleration_xy = attitude_to_world_acceleration(
            np.degrees(orientation_rpy[0]),
            np.degrees(orientation_rpy[1]),
            np.degrees(orientation_rpy[2]),
        )
        model_acceleration_norm = float(np.linalg.norm(
            self._coast_model_acceleration_xy
        ))
        model_acceleration_limit = 2.0 * self.coast_max_acceleration_m_s2
        if model_acceleration_norm > model_acceleration_limit:
            self._coast_model_acceleration_xy *= (
                model_acceleration_limit / model_acceleration_norm
            )
        tracking = coast_target_braking_attitude(
            position[:2],
            velocity[:2],
            target_position[:2],
            self.brake_direction[:2],
            np.degrees(orientation_rpy[2]),
            response_delay_s=self.coast_attitude_response_delay_s,
            response_time_constant_s=(
                self.coast_attitude_time_constant_s
            ),
            acceleration_scale=self.coast_attitude_acceleration_scale,
            terminal_speed_margin_m_s=(
                self.coast_level_terminal_speed_m_s
            ),
            velocity_gain_s=self.coast_velocity_gain_s,
            max_acceleration_m_s2=self.coast_max_acceleration_m_s2,
            max_attitude_deg=self.brake_max_attitude_deg,
            measured_acceleration_xy=(
                self._coast_model_acceleration_xy
            ),
            command_history=self._coast_command_history,
            timestamp=timestamp,
            future_command_started_at=command_timestamp,
            command_hold_s=self._estimated_attitude_command_hold_s(),
        )
        if (
            np.linalg.norm(tracking['applied_acceleration_m_s2'])
            < self.coast_command_acceleration_deadband_m_s2
        ):
            # Snap sub-physical/noise-scale commands to true level.  Queue
            # settlement must be based on an actual zero command; otherwise a
            # 1 mm/s lateral estimate can keep resetting the response timer.
            tracking['roll_deg'] = 0.0
            tracking['pitch_deg'] = 0.0
            tracking['raw_tilt_deg'] = 0.0
            tracking['required_deceleration_m_s2'] = 0.0
            tracking['tail_cancellation_acceleration_m_s2'] = 0.0
            tracking['tail_cancellation_signed_acceleration_m_s2'] = 0.0
            tracking['predicted_terminal_after_pulse_m_s'] = (
                tracking['predicted_level_terminal_speed_m_s']
            )
            tracking['requested_acceleration_m_s2'] = np.zeros(2)
            tracking['applied_acceleration_m_s2'] = np.zeros(2)
            tracking['command_acceleration_m_s2'] = np.zeros(2)
            tracking['action'] = 'leveling_command_deadband'
            tracking['power_w_per_kg'] = 0.0
            tracking['acceleration_saturated'] = False
        if self._coast_level_handoff_latched:
            # Once the signed longitudinal speed first reaches 0.10 m/s, never
            # command reverse-motion cleanup.  Send true level attitude for the
            # fixed handoff delay, even if the measured speed subsequently
            # bounces across the threshold while the old response decays.
            tracking['roll_deg'] = 0.0
            tracking['pitch_deg'] = 0.0
            tracking['raw_tilt_deg'] = 0.0
            tracking['required_deceleration_m_s2'] = 0.0
            tracking['tail_cancellation_acceleration_m_s2'] = 0.0
            tracking['tail_cancellation_signed_acceleration_m_s2'] = 0.0
            tracking['predicted_terminal_after_pulse_m_s'] = (
                tracking['predicted_level_terminal_speed_m_s']
            )
            tracking['requested_acceleration_m_s2'] = np.zeros(2)
            tracking['applied_acceleration_m_s2'] = np.zeros(2)
            tracking['command_acceleration_m_s2'] = np.zeros(2)
            tracking['action'] = 'level_for_timed_position_handoff'
            tracking['power_w_per_kg'] = 0.0
            tracking['acceleration_saturated'] = False
        self.coast_stop_target_position_m = target_position.copy()
        self.coast_target_remaining_distance_m = float(
            tracking['remaining_distance_m']
        )
        self.coast_delay_reserved_distance_m = float(
            tracking['delay_reserved_distance_m']
        )
        self.coast_required_deceleration_m_s2 = float(
            tracking['required_deceleration_m_s2']
        )
        self.coast_measured_deceleration_m_s2 = float(
            tracking['measured_deceleration_m_s2']
        )
        self.coast_predicted_forward_speed_after_delay_m_s = float(
            tracking['predicted_forward_speed_after_delay_m_s']
        )
        self.coast_response_horizon_s = float(
            tracking['response_horizon_s']
        )
        self.coast_command_acceleration_m_s2 = (
            tracking['command_acceleration_m_s2'].copy()
        )
        self.coast_predicted_level_stop_distance_m = (
            tracking['predicted_level_stop_distance_m']
        )
        self.coast_predicted_level_stop_time_s = (
            tracking['predicted_level_stop_time_s']
        )
        self.coast_predicted_level_terminal_speed_m_s = (
            tracking['predicted_level_terminal_speed_m_s']
        )
        self.coast_impulse_safe_deceleration_m_s2 = float(
            tracking['impulse_safe_deceleration_m_s2']
        )
        self.coast_tail_cancellation_acceleration_m_s2 = float(
            tracking['tail_cancellation_acceleration_m_s2']
        )
        self.coast_tail_cancellation_signed_acceleration_m_s2 = float(
            tracking['tail_cancellation_signed_acceleration_m_s2']
        )
        self.coast_tail_terminal_target_speed_m_s = (
            tracking['tail_terminal_target_speed_m_s']
        )
        self.coast_predicted_terminal_after_pulse_m_s = (
            tracking['predicted_terminal_after_pulse_m_s']
        )
        self.coast_command_hold_s = float(tracking['command_hold_s'])
        self._tail_neutralization_deadline = (
            command_timestamp + tracking['command_hold_s']
            if tracking['tail_cancellation_acceleration_m_s2'] > 0.0
            else None
        )
        self._tail_neutralization_needs_send_anchor = bool(
            tracking['tail_cancellation_acceleration_m_s2'] > 0.0
        )
        self.set_contact_attitude(
            tracking['roll_deg'],
            tracking['pitch_deg'],
            0.0,
            yaw_deg=np.degrees(orientation_rpy[2]),
        )
        self.brake_command_tilt_deg = float(np.hypot(
            tracking['roll_deg'], tracking['pitch_deg']
        ))
        self.coast_tracking_action = tracking['action']
        self.coast_tracking_acceleration_m_s2 = (
            tracking['applied_acceleration_m_s2'].copy()
        )
        self.coast_tracking_acceleration_saturated = bool(
            tracking['acceleration_saturated']
        )
        self.coast_tracking_power_w_per_kg = float(
            tracking['power_w_per_kg']
        )
        self.coast_actual_tilt_deg = float(np.degrees(np.arccos(np.clip(
            np.cos(orientation_rpy[0]) * np.cos(orientation_rpy[1]),
            -1.0,
            1.0,
        ))))
        planned_command_level = bool(
            np.linalg.norm(tracking['command_acceleration_m_s2']) <= 1e-9
        )
        history_command_level = bool(
            self._coast_command_history
            and np.linalg.norm(self._coast_command_history[-1][1]) <= 1e-9
        )
        self.coast_response_queue_settle_required_s = float(
            self.coast_level_handoff_delay_s
        )
        self.coast_response_queue_settle_elapsed_s = (
            None
            if self._level_attitude_command_started_at is None
            else max(
                command_timestamp
                - self._level_attitude_command_started_at,
                0.0,
            )
        )
        self.coast_response_queue_settled = bool(
            self._coast_level_handoff_latched
            and planned_command_level
            and history_command_level
            and self.coast_response_queue_settle_elapsed_s is not None
            and self.coast_response_queue_settle_elapsed_s
            >= self.coast_response_queue_settle_required_s
        )
        self.coast_handoff_state_ready = bool(
            self.coast_response_queue_settled
        )
        handoff_ready = self.coast_handoff_state_ready
        if not handoff_ready or not bool(allow_position_handoff):
            if attitude_timed_out:
                self.coast_handoff_reason = (
                    'waiting_for_timed_level_handoff_after_timeout'
                )
            return False

        self.coast_handoff_reason = (
            'terminal_current_position_handoff'
            if latch_current_position_on_handoff else
            'timed_level_to_position_handoff'
        )
        # Hold the stop point frozen at release only along the calibrated
        # interaction line. The perpendicular coordinate is always latched to
        # the measured position, allowing native position control to damp small
        # transverse hover drift without chasing a stale lateral target. If lag
        # already carried the vehicle past the longitudinal stop point, latch
        # the complete measured position so handoff cannot command pullback.
        self.coast_handoff_actual_position_m = position.copy()
        target_is_behind = bool(
            np.linalg.norm(self.brake_direction[:2]) <= 1e-9
            or self.coast_target_remaining_distance_m < 0.0
        )
        latch_current_position = bool(
            target_is_behind or latch_current_position_on_handoff
        )
        self.coast_target_clamped_to_actual = latch_current_position
        if latch_current_position:
            self.hold_position = position.copy()
            self.coast_lateral_target_latched_to_actual = True
        else:
            self.hold_position = position.copy()
            self.hold_position[:2] += (
                self.coast_target_remaining_distance_m * brake_direction_xy
            )
            self.hold_position[2] = target_position[2]
            self.coast_lateral_target_latched_to_actual = True
        self.hover_z = float(self.hold_position[2])
        self.stopping_position_m = self.hold_position.copy()
        self.set_contact_attitude(0.0, 0.0, 0.0)
        self.brake_command_tilt_deg = 0.0
        self.coast_tracking_action = 'position_handoff'
        self.coast_tracking_acceleration_m_s2 = np.zeros(2)
        self.coast_tracking_acceleration_saturated = False
        self.coast_tracking_power_w_per_kg = 0.0
        self.brake_completion_reason = self.coast_handoff_reason
        self._brake_started_at = None
        self._detector_rearm_at = timestamp + self.rearm_delay_s
        self._transition_mode(self.POSITION_HOLD)
        return True

    def update_braking(
            self, current_position, velocity, timestamp,
            current_orientation_rpy=None, coast_position=None,
            coast_velocity=None,
            current_force=None, current_mass_kg=None,
    ):
        if self.shadow_mode or self.mode not in (
                self.POSITION_COAST, self.ATTITUDE_BRAKING):
            return False
        position_coast = self.mode == self.POSITION_COAST
        position = np.asarray(current_position, dtype=float)
        velocity = np.asarray(velocity, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError('current translation hold position must be finite XYZ')
        if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
            raise ValueError('translation braking velocity must be finite XYZ')
        timestamp = float(timestamp)
        if not np.isfinite(timestamp):
            raise ValueError('translation braking timestamp must be finite')
        orientation_rpy = np.asarray(
            [0.0, 0.0, 0.0]
            if current_orientation_rpy is None else current_orientation_rpy,
            dtype=float,
        )
        if orientation_rpy.shape != (3,) or not np.all(np.isfinite(orientation_rpy)):
            raise ValueError('translation braking orientation must be finite RPY')
        force = np.asarray(
            np.zeros(3) if current_force is None else current_force,
            dtype=float,
        )
        if force.shape != (3,) or not np.all(np.isfinite(force)):
            raise ValueError('translation braking force must be finite XYZ')
        mass = self.release_mass_kg if current_mass_kg is None else float(current_mass_kg)
        if mass is not None and (not np.isfinite(mass) or mass <= 0.0):
            raise ValueError('translation braking mass must be positive')
        braking_velocity = velocity
        position_state_settled = False
        if position_coast:
            # ``hold_position`` was latched from the actual state at the
            # attitude handoff. The virtual coast remains log-only and must
            # never overwrite this target, otherwise position control pulls
            # the vehicle back toward an obsolete simulated stop point.
            position_error_xy = self.hold_position[:2] - position[:2]
            position_state_settled = bool(
                np.linalg.norm(position_error_xy)
                <= self.coast_alignment_position_tolerance_m
                and np.linalg.norm(velocity[:2]) <= self.brake_xy_speed_m_s
            )
            if position_state_settled:
                if self._coast_position_settle_since is None:
                    self._coast_position_settle_since = timestamp
            else:
                self._coast_position_settle_since = None
        projected_speed = float(
            braking_velocity[:2] @ self.brake_direction[:2]
        )
        self.brake_projected_speed_m_s = projected_speed
        timed_out = bool(
            self._brake_started_at is not None
            and timestamp - self._brake_started_at >= self.brake_timeout_s
        )

        stopped_or_reversed = bool(
            projected_speed <= self.brake_xy_speed_m_s
        )
        position_settle_complete = bool(
            position_coast
            and self._coast_position_settle_since is not None
            and timestamp - self._coast_position_settle_since
            >= self.brake_settle_s
        )
        if not stopped_or_reversed and not timed_out and not position_coast:
            self._set_velocity_brake_attitude(
                projected_speed,
                orientation_rpy[2],
                projected_force_n=float(
                    force[:2] @ self.brake_direction[:2]
                ),
                current_mass_kg=mass,
            )
            return False
        if position_coast:
            if not position_settle_complete and not timed_out:
                return False
        elif not stopped_or_reversed and not timed_out:
            return False

        # Both braking paths finish at a measured/latching position. The
        # virtual coast is comparison-only and never owns the final hold.
        if not position_coast:
            self.hold_position = position.copy()
        self.stopping_position_m = self.hold_position.copy()
        self.set_contact_attitude(0.0, 0.0, 0.0)
        self.brake_command_tilt_deg = 0.0
        self._brake_started_at = None
        self._detector_rearm_at = timestamp + self.rearm_delay_s
        if position_coast:
            self.brake_completion_reason = (
                'actual_state_settled'
                if position_settle_complete else 'braking_timeout'
            )
        else:
            self.brake_completion_reason = (
                'projected_velocity_zero_or_reversed'
                if stopped_or_reversed else 'braking_timeout'
            )
        self._transition_mode(self.POSITION_HOLD)
        return True

    def consume_detector_rearm(self, timestamp):
        """Return true once when the post-braking detector delay expires."""
        timestamp = float(timestamp)
        if not np.isfinite(timestamp):
            raise ValueError('detector rearm timestamp must be finite')
        if (
            self.mode != self.POSITION_HOLD
            or self._detector_rearm_at is None
            or timestamp < self._detector_rearm_at
        ):
            return False
        self._detector_rearm_at = None
        return True

    @property
    def attitude_mode(self):
        return self.mode == self.CONTACT_ZDISTANCE

    @property
    def mpc_bootstrap_acceleration_mode(self):
        return self.mode == self.MPC_BOOTSTRAP_ACCELERATION

    @property
    def position_interaction_mode(self):
        return self.mode == self.CONTACT_POSITION

    @property
    def braking_mode(self):
        return self.mode in (
            self.ATTITUDE_COAST,
            self.VELOCITY_COAST,
            self.POSITION_COAST,
            self.ATTITUDE_BRAKING,
        )

    @property
    def uses_position_setpoint(self):
        return bool(
            self.shadow_mode
            or self.mode in (
                self.POSITION_HOLD,
                self.CONTACT_POSITION,
                self.POSITION_COAST,
            )
            or (
                self.mode == self.VELOCITY_COAST
                and self.coast_velocity_phase == 'predictive_unwind'
                and self.coast_velocity_unwind_position_control_enabled
                and self.coast_velocity_unwind_position_target_m is not None
            )
        )

    @property
    def direct_level_unwind_active(self):
        return bool(
            self.mode == self.VELOCITY_COAST
            and self.coast_velocity_phase == 'predictive_unwind'
            and self.coast_velocity_unwind_direct_level_attitude_enabled
        )

    @property
    def command_mode(self):
        if self.shadow_mode:
            return 'shadow_position_hold'
        if (
            self.mode == self.VELOCITY_COAST
            and self.coast_velocity_phase == 'predictive_unwind'
            and self.coast_velocity_unwind_direct_level_attitude_enabled
        ):
            return 'predictive_unwind_attitude_zdistance'
        if (
            self.mode == self.VELOCITY_COAST
            and self.coast_velocity_phase == 'predictive_unwind'
            and self.coast_velocity_unwind_position_control_enabled
        ):
            return 'predictive_unwind_position'
        return self.mode

    def _record_sent_command(self, kind, sent_at, **values):
        sent_at = float(sent_at)
        if not np.isfinite(sent_at):
            raise ValueError('actual command timestamp must be finite')
        self._sent_command_sequence += 1
        command = {
            'kind': str(kind),
            'sent_at': sent_at,
            'sequence': self._sent_command_sequence,
        }
        command.update(values)
        self._last_sent_command = command
        self._sent_command_history.append(command)

    def sent_command_snapshot(self):
        """Return JSON-safe metadata for the most recent actual send."""
        if self._last_sent_command is None:
            return None
        return {
            key: (list(value) if isinstance(value, list) else value)
            for key, value in self._last_sent_command.items()
        }

    def sent_commands_after_sequence(self, sequence):
        """Return every actual send after ``sequence`` in send order."""
        if isinstance(sequence, bool) or int(sequence) != sequence:
            raise ValueError('actual command sequence must be an integer')
        return [
            {
                key: (list(value) if isinstance(value, list) else value)
                for key, value in command.items()
            }
            for command in self._sent_command_history
            if command['sequence'] > int(sequence)
        ]

    def sent_command_effective_at(self, state_time, delay_s=0.0):
        """Return the last command at the delayed input time of a state."""
        state_time = float(state_time)
        delay_s = float(delay_s)
        if not np.all(np.isfinite([state_time, delay_s])) or delay_s < 0.0:
            raise ValueError('state command lookup needs finite time/delay')
        effective_time = state_time-delay_s
        for command in reversed(self._sent_command_history):
            if command['sent_at'] <= effective_time+1e-12:
                result = {
                    key: (list(value) if isinstance(value, list) else value)
                    for key, value in command.items()
                }
                result['effective_query_time'] = effective_time
                return result
        return None

    def sent_commands_in_window(self, start_time, end_time):
        """Return actual sends in the closed host-time interval."""
        start_time = float(start_time)
        end_time = float(end_time)
        if (
            not np.all(np.isfinite([start_time, end_time]))
            or end_time < start_time
        ):
            raise ValueError('command-history window must be finite/ordered')
        return [
            {
                key: (list(value) if isinstance(value, list) else value)
                for key, value in command.items()
            }
            for command in self._sent_command_history
            if start_time-1e-12 <= command['sent_at'] <= end_time+1e-12
        ]

    def send(self, commander, command_timestamp=None, yaw_deg=None):
        if self.uses_position_setpoint:
            commander.send_position_setpoint(
                *self.hold_position, self.yaw_deg
            )
            sent_at = float(
                time.time()
                if command_timestamp is None else command_timestamp
            )
            self._record_sent_command(
                'position', sent_at,
                position_m=self.hold_position.tolist(),
                yaw_deg=float(self.yaw_deg),
            )
            return sent_at
        elif self.direct_level_unwind_active:
            commander.send_zdistance_setpoint(
                0.0,
                0.0,
                0.0,
                self.velocity_coast_fixed_zdistance_m,
            )
            sent_at = float(
                time.time()
                if command_timestamp is None else command_timestamp
            )
            current_yaw_deg = float(
                self.yaw_deg if yaw_deg is None else yaw_deg
            )
            self._record_attitude_command(sent_at, current_yaw_deg)
            self._record_sent_command(
                'attitude_zdistance', sent_at,
                roll_deg=0.0,
                pitch_deg=0.0,
                yaw_rate_deg_s=0.0,
                zdistance_m=float(
                    self.velocity_coast_fixed_zdistance_m
                ),
                yaw_deg=current_yaw_deg,
            )
            return sent_at
        elif self.mode in (
            self.CONTACT_ZDISTANCE,
            self.MPC_BOOTSTRAP_ACCELERATION,
            self.ATTITUDE_COAST,
            self.ATTITUDE_BRAKING,
        ):
            commander.send_zdistance_setpoint(
                self.contact_roll_deg,
                self.contact_pitch_deg,
                self.contact_yaw_rate_deg_s,
                self.hover_z,
            )
            # Runtime callers normally omit command_timestamp so history uses
            # the actual wall-clock send instant rather than the earlier plan
            # instant. Tests/simulations may provide an explicit clock.
            sent_at = float(
                time.time()
                if command_timestamp is None else command_timestamp
            )
            attitude_yaw_deg = float(
                self._pending_attitude_yaw_deg
                if yaw_deg is None else yaw_deg
            )
            self._record_attitude_command(
                sent_at,
                attitude_yaw_deg,
            )
            self._record_sent_command(
                'attitude_zdistance', sent_at,
                roll_deg=float(self.contact_roll_deg),
                pitch_deg=float(self.contact_pitch_deg),
                yaw_rate_deg_s=float(self.contact_yaw_rate_deg_s),
                zdistance_m=float(self.hover_z),
                yaw_deg=attitude_yaw_deg,
            )
            if self._tail_neutralization_needs_send_anchor:
                pulse_hold_s = (
                    self.release_candidate_command_hold_s
                    if self.mode == self.CONTACT_ZDISTANCE
                    else self.coast_command_hold_s
                )
                if pulse_hold_s is None or pulse_hold_s <= 0.0:
                    raise RuntimeError(
                        'tail-neutralization pulse has no valid hold duration'
                    )
                # The predictor is evaluated before the command is sent. Anchor
                # finite-pulse expiry at the real send time so computation and
                # logging latency cannot silently shorten or extend it.
                self._tail_neutralization_deadline = (
                    sent_at + float(pulse_hold_s)
                )
                self._tail_neutralization_needs_send_anchor = False
            return sent_at
        elif self.mode == self.VELOCITY_COAST:
            current_yaw_deg = float(
                self.yaw_deg if yaw_deg is None else yaw_deg
            )
            body_velocity_xy = world_to_body_xy(
                self.coast_velocity_command_xy_m_s,
                current_yaw_deg,
            )
            commander.send_hover_setpoint(
                float(body_velocity_xy[0]),
                float(body_velocity_xy[1]),
                0.0,
                self.velocity_coast_fixed_zdistance_m,
            )
            sent_at = float(
                time.time()
                if command_timestamp is None else command_timestamp
            )
            self._record_sent_command(
                'velocity_hover', sent_at,
                world_velocity_xy_m_s=(
                    self.coast_velocity_command_xy_m_s.tolist()
                ),
                body_velocity_xy_m_s=body_velocity_xy.tolist(),
                yaw_rate_deg_s=0.0,
                zdistance_m=float(
                    self.velocity_coast_fixed_zdistance_m
                ),
                yaw_deg=current_yaw_deg,
            )
            return sent_at
        return None


def calculate_tilt(roll, pitch, degrees=True):
    if degrees:
        roll = np.radians(roll)
        pitch = np.radians(pitch)

    # Calculate the cosine of the total tilt
    cos_tilt = np.cos(roll) * np.cos(pitch)
    tilt_rad = np.arccos(np.clip(cos_tilt, -1.0, 1.0))

    return np.degrees(tilt_rad) if degrees else tilt_rad


class InteractionsControl:

    def __init__(self, cf, sleep_function, log_manager, mission, ctrl_rate, log_command=True, execute=True,
                 leader_info=None, pub_socket=None, sub_socket=None, drone_id=None, set_color=None,
                 orchestrator_ip=None, force_sensor=None, sense_axis='x',
                 sense_sign=1, sense_max_age_s=0.25, *args, **kwargs):
        self.cf = cf
        self.log_manager = log_manager
        self.mission = mission
        self.ctrl_rate = ctrl_rate
        self.pub_socket = pub_socket
        self.sub_socket = sub_socket
        self.drone_id = drone_id
        self.set_color = set_color
        self.orchestrator_ip = orchestrator_ip
        self.force_sensor = force_sensor
        self.sense_axis = str(sense_axis).lower()
        if self.sense_axis not in ('x', 'y', 'z'):
            raise ValueError('sense_axis must be x, y, or z')
        self.sense_axis_index = {'x': 0, 'y': 1, 'z': 2}[self.sense_axis]
        self.sense_sign = int(sense_sign)
        if self.sense_sign not in (-1, 1):
            raise ValueError('sense_sign must be +1 or -1')
        self.sense_max_age_s = float(sense_max_age_s)
        if self.sense_max_age_s <= 0.0:
            raise ValueError('sense_max_age_s must be positive')
        # Network followers use their own 'frames' position, not the leader's mocap group
        self.pos_group_name = 'frames' if (leader_info is None or sub_socket is not None) else f"{leader_info['id']}"

        log_function = log_manager.add_log_entry if log_command else None
        offset = np.zeros(3) if leader_info is None else np.array(leader_info['offset'])
        self.hl_commander = CommandWrapper(self.cf.high_level_commander, log_function=log_function, execute=execute,
                                           offset=offset)
        self.lo_commander = CommandWrapper(self.cf.commander, log_function=log_function, execute=execute, offset=offset)
        self._safe_sleep = sleep_function
        self.bounds = self.mission.get('boundary_limits', None)

    def _force_sensor_log_fields(self, estimate, now):
        """Return time-aligned potentiometer/observer comparison fields."""
        sensor = getattr(self, 'force_sensor', None)
        if sensor is None:
            return {}

        power_fields = self._rpi_power_log_fields(sensor, now)

        axis = getattr(self, 'sense_axis', 'x')
        axis_index = getattr(
            self, 'sense_axis_index', {'x': 0, 'y': 1, 'z': 2}[axis]
        )
        sign = getattr(self, 'sense_sign', 1)
        max_age_s = getattr(self, 'sense_max_age_s', 0.25)
        sample = sensor.latest()
        if sample is None:
            return {
                'force_sensor_fresh': False,
                'force_sensor_axis': axis,
                'force_sensor_sign': sign,
                **power_fields,
            }

        age_s = float(now) - float(sample.host_time)
        fresh = -0.5 <= age_s <= max_age_s
        signed_force_n = sign * float(sample.force_n)
        force_body = np.zeros(3)
        force_body[axis_index] = signed_force_n
        roll, pitch, yaw = np.asarray(
            getattr(estimate, 'orientation_rpy', np.zeros(3)), dtype=float
        )
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        body_to_world = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ])
        force_world = body_to_world @ force_body
        estimated_force_body = (
            body_to_world.T @ np.asarray(estimate.external_force, dtype=float)
        )
        estimated_axis_force = float(estimated_force_body[axis_index])
        return {
            'force_sensor_fresh': bool(fresh),
            'force_sensor_axis': axis,
            'force_sensor_sign': sign,
            'force_sensor_sample_time': float(sample.host_time),
            'force_sensor_sample_age_s': age_s,
            'force_sensor_arduino_time_ms': int(sample.arduino_time_ms),
            'force_sensor_raw': int(sample.raw),
            'force_sensor_filtered_raw': float(sample.filtered_raw),
            'force_sensor_voltage_V': float(sample.voltage_v),
            'force_sensor_supply_voltage_V': (
                None
                if sample.supply_voltage_v is None
                else float(sample.supply_voltage_v)
            ),
            'force_sensor_compression_mm': float(sample.compression_mm),
            'force_sensor_length_mm': float(sample.length_mm),
            'force_sensor_compression_force_N': float(sample.force_n),
            'force_sensor_external_force_body_N': force_body.tolist(),
            'force_sensor_external_force_N': force_world.tolist(),
            'estimated_external_force_along_sensor_N': estimated_axis_force,
            'force_sensor_estimate_error_N': (
                estimated_axis_force - signed_force_n if fresh else None
            ),
            **power_fields,
        }

    @staticmethod
    def _rpi_power_log_fields(sensor, now):
        """Return the latest non-blocking Raspberry Pi power-health sample."""
        monitor = getattr(sensor, 'rpi_power_monitor', None)
        if monitor is None:
            return {}
        sample = monitor.latest()
        if sample is None:
            return {'rpi_power_monitor_available': False}
        return {
            'rpi_power_monitor_available': True,
            'rpi_power_sample_time': float(sample.host_time),
            'rpi_power_sample_age_s': float(now) - float(sample.host_time),
            'rpi_power_flags': int(sample.flags),
            'rpi_power_flags_hex': f'0x{sample.flags:x}',
            'rpi_under_voltage_now': bool(sample.under_voltage_now),
            'rpi_under_voltage_occurred': bool(
                sample.under_voltage_occurred
            ),
            'rpi_frequency_capped_now': bool(sample.frequency_capped_now),
            'rpi_frequency_capped_occurred': bool(
                sample.frequency_capped_occurred
            ),
            'rpi_throttled_now': bool(sample.throttled_now),
            'rpi_throttled_occurred': bool(sample.throttled_occurred),
            'rpi_soft_temperature_limit_now': bool(
                sample.soft_temperature_limit_now
            ),
            'rpi_soft_temperature_limit_occurred': bool(
                sample.soft_temperature_limit_occurred
            ),
        }

    def _force_sensor_config(self):
        sensor = getattr(self, 'force_sensor', None)
        return {
            'enabled': sensor is not None,
            'axis': getattr(self, 'sense_axis', 'x'),
            'sign': getattr(self, 'sense_sign', 1),
            'max_sample_age_s': getattr(self, 'sense_max_age_s', 0.25),
            'control_source': 'wrench_observer',
            'contact_detection_source': 'wrench_observer',
            'release_braking_force_source': (
                'potentiometer_force_sensor'
                if sensor is not None else 'wrench_observer'
            ),
            'spring_constant_n_per_mm': (
                sensor.spring_constant_n_per_mm if sensor is not None else None
            ),
            'max_extension_mm': (
                getattr(sensor, 'max_extension_mm', None)
                if sensor is not None else None
            ),
            'arduino_supply_voltage_recorded': bool(
                sensor is not None
                and getattr(sensor.latest(), 'supply_voltage_v', None)
                is not None
            ),
            'rpi_power_monitor': {
                'enabled': bool(
                    sensor is not None
                    and getattr(sensor, 'rpi_power_monitor', None) is not None
                ),
                'poll_interval_s': (
                    getattr(
                        getattr(sensor, 'rpi_power_monitor', None),
                        'poll_interval_s',
                        None,
                    )
                    if sensor is not None else None
                ),
            },
        }

    def _force_sensor_axis_world(self, estimate):
        """Return the signed unit sensor axis rotated into the world frame."""
        axis_body = np.zeros(3)
        axis_body[self.sense_axis_index] = self.sense_sign
        roll, pitch, yaw = np.asarray(estimate.orientation_rpy, dtype=float)
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]) @ axis_body

    @staticmethod
    def _release_braking_force(estimate, sensor_fields, sensor_enabled):
        """Select relative sensor force only for the release-braking phase."""
        observer_force = np.asarray(estimate.external_force, dtype=float)
        if (
            sensor_enabled
            and bool(sensor_fields.get('force_sensor_fresh'))
        ):
            return (
                np.asarray(
                    sensor_fields['force_sensor_external_force_N'],
                    dtype=float,
                ),
                'potentiometer_force_sensor',
            )
        return observer_force.copy(), 'wrench_observer'

    def run(self) -> None:

        action = self.mission.get('Interaction', {}).get('action')

        if action == 'peer_latency_test':
            self._run_peer_latency_test()
            return

        # When both sockets are present the behaviour depends on whether avoidance
        # is configured.  With avoidance: UI-LB runs translation + APF broadcast.
        # Without: symmetric peer translation (all drones equal).
        if self.pub_socket is not None and self.sub_socket is not None:
            if self.mission.get('avoidance'):
                # self.run_translation_broadcast()
                pass
            else:
                self._run_peer_translation()
            return

        if action == 'rotation_test':
            self._run_rotation_limit()
        elif action == 'translation':
            self._run_translation()

    def run_calibration(self) -> None:
        """Run contact-free wrench and planar braking identification."""
        if self.mission.get('Interaction', {}).get('action') != 'translation':
            raise ValueError('--calibrate requires Interaction.action: translation')
        self._run_translation(calibration_mode=True)

    def run_mpc_calibration(self) -> None:
        """Collect automatic accelerate-to-rest trajectories for LMPC admission.

        This is not the legacy plant fit and it does not run an LMPC policy.
        A private controller-side mission overlay automatically accelerates on
        world +/-Y with bounded attitude commands, then selects the existing
        bounded attitude coast controller. This method only activates that
        bootstrap coverage/audit path.
        """
        if self.mission.get('Interaction', {}).get('action') != 'translation':
            raise ValueError('--mpc requires Interaction.action: translation')
        if self.ctrl_rate != 100:
            raise ValueError('--mpc requires a 100 Hz control rate')
        self._run_translation(mpc_calibration_mode=True)

    def run_braking_test(self, *, direction=None, repetitions=None) -> None:
        """Run selected fixed-duration attitude repeats, without fitting or saving."""
        if self.mission.get('Interaction', {}).get('action') != 'translation':
            raise ValueError('--braking-test requires Interaction.action: translation')
        if self.ctrl_rate < 50:
            raise ValueError('--braking-test requires a control rate of at least 50 Hz')
        direction, repetitions = resolve_repeat_test_selection(direction, repetitions)
        self._run_translation(
            calibration_mode=True, braking_test_mode=True,
            braking_test_direction=direction, braking_test_repetitions=repetitions,
        )

    def check_interaction_boundary(self, pos=None):
        if self.bounds is None:
            return

        if pos is None:
            pos = self._get_latest_pos()

        if pos is None or len(pos) < 3:
            logger.warning("Could not retrieve position for boundary check.")
            return

        x, y, z = pos[0], pos[1], pos[2]

        if not (self.bounds['x_min'] <= x <= self.bounds['x_max']):
            raise BoundaryExceededError(
                f"X position ({x:.3f}) breached bounds [{self.bounds['x_min']}, {self.bounds['x_max']}]")

        if not (self.bounds['y_min'] <= y <= self.bounds['y_max']):
            raise BoundaryExceededError(
                f"Y position ({y:.3f}) breached bounds [{self.bounds['y_min']}, {self.bounds['y_max']}]")

        if not (self.bounds['z_min'] <= z <= self.bounds['z_max']):
            raise BoundaryExceededError(
                f"Z position ({z:.3f}) breached bounds [{self.bounds['z_min']}, {self.bounds['z_max']}]")

    def test_flight(self):

        try:
            st = time.time()
            while time.time() < st + 10:
                self.lo_commander.send_position_setpoint(1, 1, 1, 0)
                self._safe_sleep(0.01)
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Test Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()

    def _run_rotation_limit(self) -> None:
        """Execute the force-render haptic interaction."""
        try:
            setting = self.mission['Interaction']['config']
            rads_to_deg = 57.3
            yawrate = setting['rads_per_sec'] * rads_to_deg
            self.test_rotation_limit(yawrate=yawrate, duration=setting['duration'])
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Render Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()

    def _run_force_render(self) -> None:
        """Execute the force-render haptic interaction."""
        try:
            # self.hover()
            # self.test_rotation_limit()
            self.force_render()
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Render Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()

    def run_unit_test(self, command_type='lo'):
        distance_to_test = [0.01, 0.02, 0.2, 0.5, 1, 2]
        dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01

        self._safe_sleep(2)

        for d in distance_to_test:
            pos, vel = self._get_latest_pos(vel=True)
            hover_pos = [pos[0], pos[1] + d, 1]
            travel_time = d * 3

            if command_type == 'hi':
                self.hl_commander.go_to(hover_pos[0], hover_pos[1], hover_pos[2], 0, travel_time, relative=False)
                self._safe_sleep(travel_time + 3)
            else:
                start_time = time.time()
                while time.time() < start_time + travel_time + 3:
                    self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                    self._safe_sleep(dt)

        self.lo_commander.send_notify_setpoint_stop()
        return

    def run_recap(self, file) -> None:
        """Replay a single recorded command log. File selection and takeoff/land
        orchestration are handled by controller.py before calling IC.run()."""
        try:
            cmds = load_commands(file)
            self.execute_commands(cmds)
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Recap Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()

    def _run_translation(self, calibration_mode=False, braking_test_mode=False,
                         mpc_calibration_mode=False,
                         braking_test_direction=None, braking_test_repetitions=None) -> None:
        """Run model-based interaction, with a legacy velocity-mode fallback."""
        prediction_session = None
        self._translation_exit_target = None
        self._translation_high_level_active = False
        try:
            if mpc_calibration_mode and (calibration_mode or braking_test_mode):
                raise ValueError('--mpc is separate from legacy calibration modes')
            if braking_test_mode and not calibration_mode:
                raise ValueError('braking repeat test requires the calibration control path')
            translation_setting = self.mission['Interaction']['config']
            wrench_config = translation_setting.get('wrench_interaction')
            detection_method = translation_setting.get('detection_method')
            if detection_method is None:
                # Preserve old missions: a wrench block selected model-based
                # detection, while its absence selected legacy velocity mode.
                detection_method = (
                    (
                        'momentum_impulse'
                        if wrench_config.get('state_source') == 'onboard'
                        else 'mocap_wrench'
                    )
                    if wrench_config is not None else 'velocity'
                )
            if detection_method not in (
                    'velocity', 'momentum_impulse', 'mocap_wrench'):
                raise ValueError(
                    'translation detection_method must be velocity or '
                    'momentum_impulse (mocap_wrench is retained for legacy use)'
                )
            if calibration_mode and detection_method != 'momentum_impulse':
                raise ValueError(
                    '--calibrate requires detection_method: momentum_impulse'
                )
            if mpc_calibration_mode and detection_method != 'momentum_impulse':
                raise ValueError(
                    '--mpc requires detection_method: momentum_impulse'
                )
            if detection_method in ('momentum_impulse', 'mocap_wrench'):
                if wrench_config is None:
                    raise ValueError(
                        f'{detection_method} detection requires '
                        'wrench_interaction config'
                    )
                if (
                    detection_method == 'momentum_impulse'
                    and wrench_config.get('state_source', 'mocap') != 'onboard'
                ):
                    raise ValueError(
                        'momentum_impulse detection requires state_source: onboard'
                    )
                calibration_path = translation_setting.get(
                    'wrench_calibration_file', str(DEFAULT_CALIBRATION_PATH)
                )
                target = resolve_wrench_nominal_target(
                    self.mission['drones'][self.drone_id]['target'],
                    wrench_config,
                    calibration_mode=(
                        calibration_mode or mpc_calibration_mode
                    ),
                )
                nominal_yaw = (
                    target[3]
                    if len(target) > 3
                    else wrench_config.get('nominal_yaw_deg', 0.0)
                )
                virtual_object_setting = dict(
                    translation_setting.get('virtual_object') or {}
                )
                force_sensor_available = bool(
                    getattr(self, 'force_sensor', None) is not None
                    and not calibration_mode
                )
                default_release_mode = (
                    'potentiometer_coast'
                    if force_sensor_available else 'observer_brake'
                )
                configured_release_mode = str(
                    dict(virtual_object_setting.get('release_behavior') or {})
                    .get('mode', default_release_mode)
                ).strip().lower()
                effective_release_mode = resolve_release_mode(
                    configured_release_mode,
                    force_sensor_available,
                    calibration_mode=calibration_mode,
                )
                runtime_braking_direction_xy = None
                if mpc_calibration_mode:
                    # The automatic protocol exercises both signed world-Y
                    # models in one run.  Resolve the saved calibration against
                    # that task axis even though no physical sensor selects it.
                    runtime_braking_direction_xy = np.array([0.0, 1.0])
                if effective_release_mode == 'potentiometer_coast':
                    sense_axis = getattr(self, 'sense_axis', None)
                    sense_sign = float(getattr(self, 'sense_sign', 1))
                    yaw_rad = np.radians(float(nominal_yaw))
                    if sense_axis == 'x':
                        runtime_braking_direction_xy = sense_sign * np.array([
                            np.cos(yaw_rad), np.sin(yaw_rad),
                        ])
                    elif sense_axis == 'y':
                        runtime_braking_direction_xy = sense_sign * np.array([
                            -np.sin(yaw_rad), np.cos(yaw_rad),
                        ])
                    else:
                        raise ValueError(
                            'potentiometer coasting requires a planar x/y '
                            'sensor axis'
                        )
                if calibration_mode:
                    wrench_config = (
                        repeat_test_config(
                            wrench_config, direction=braking_test_direction,
                            repetitions=braking_test_repetitions,
                        )
                        if braking_test_mode else deepcopy(wrench_config)
                    )
                    wrench_config['shadow_mode'] = True
                    wrench_config['startup_bias_calibration_enabled'] = True
                    wrench_config.setdefault('calibration_excitation', {})[
                        'enabled'
                    ] = not braking_test_mode
                    excitation = wrench_config['calibration_excitation']
                    excitation_end_s = (
                        float(excitation.get('start_delay_s', 1.0))
                        + float(excitation.get('duration_s', 30.0))
                    )
                    if braking_test_mode:
                        excitation_end_s = 0.0
                    braking_config = wrench_config.setdefault(
                        'planar_braking_calibration', {}
                    )
                    braking_config['enabled'] = True
                    braking_plan = PlanarBrakingCalibration(
                        braking_config,
                        start_after_s=excitation_end_s,
                        require_opposed_directions=not braking_test_mode,
                    )
                    # Retired experiment: old mission settings must not
                    # silently re-enable position-capture trials.
                    wrench_config.pop('position_capture_calibration', None)
                    # Validate readiness timing before entering any maneuver.
                    CalibrationTrialReadinessGate(braking_config)
                    adaptive_options = wrench_config.get('adaptive_braking_calibration', {})
                    adaptive_preflight = AdaptiveBrakingCalibration(
                        adaptive_options, braking_plan, getattr(self, 'ctrl_rate', 0.0), None,
                    )
                    if adaptive_preflight.enabled:
                        if braking_test_mode:
                            raise ValueError('adaptive braking is only available with --calibrate, not --braking-test')
                        AdaptiveBrakingCalibration.validate_online_worker_config(
                            wrench_config.get('online_prediction_calibration'))
                    interaction_duration = braking_plan.end_s + 0.5
                else:
                    wrench_config, saved_calibration = apply_drone_calibration(
                        wrench_config,
                        self.drone_id,
                        calibration_path,
                        runtime_interaction_direction_xy=(
                            runtime_braking_direction_xy
                        ),
                    )
                    saved_planar_fit = (
                        None
                        if saved_calibration is None
                        else saved_calibration.get('planar_braking_fit')
                    )
                    current_planar_fit = planar_braking_fit_is_current(
                        saved_planar_fit
                    )
                    if (
                        effective_release_mode == 'potentiometer_coast'
                        and not bool(wrench_config.get('shadow_mode', True))
                        and not current_planar_fit
                    ):
                        raise ValueError(
                            'active potentiometer_coast requires a current, '
                            'quality-gated planar braking calibration; run '
                            '--calibrate before --interaction'
                        )
                    if (
                        effective_release_mode == 'potentiometer_coast'
                        and not bool(wrench_config.get('shadow_mode', True))
                        and current_planar_fit
                    ):
                        calibrated_tilt_deg = float(
                            saved_planar_fit['protocol']['tilt_deg']
                        )
                        requested_render_tilt_deg = float(
                            virtual_object_setting.get(
                                'max_attitude_deg', 20.0
                            )
                        )
                        virtual_object_setting['max_attitude_deg'] = (
                            calibrated_force_render_attitude_limit(
                            requested_render_tilt_deg,
                            calibrated_tilt_deg,
                            )
                        )
                        if (
                            requested_render_tilt_deg
                            > virtual_object_setting['max_attitude_deg']
                        ):
                            logger.info(
                                'Capped force-render tilt from %.2f to %.2f '
                                'deg to stay inside planar calibration.',
                                requested_render_tilt_deg,
                                virtual_object_setting['max_attitude_deg'],
                            )
                    # A normal interaction starts immediately. The dedicated
                    # --calibrate flow retains stationary bias collection.
                    wrench_config['startup_bias_calibration_enabled'] = False
                    wrench_config.setdefault('calibration_excitation', {})[
                        'enabled'
                    ] = False
                    wrench_config.setdefault('online_prediction_calibration', {})[
                        'enabled'
                    ] = False
                    interaction_duration = translation_setting['duration']
                    if saved_calibration is None:
                        logger.warning(
                            'No saved wrench model calibration for %s at %s; '
                            'using mission/default alignment parameters.',
                            self.drone_id, calibration_path,
                        )
                    else:
                        logger.info('Loaded wrench calibration: %s', calibration_path)
                if mpc_calibration_mode:
                    bootstrap_config = MPCBootstrapCalibrationConfig.from_mapping(
                        wrench_config.get('mpc_bootstrap_calibration')
                    )
                    decision_ratio = (
                        bootstrap_config.prediction_step_s*self.ctrl_rate
                    )
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
                            '--mpc prediction_step_s must be an integral '
                            'multiple of at least two 100 Hz control periods'
                        )
                    if getattr(self, 'bounds', None) is not None:
                        x, y = float(target[0]), float(target[1])
                        available_margin = min(
                            x-self.bounds['x_min'],
                            self.bounds['x_max']-x,
                            y-self.bounds['y_min'],
                            self.bounds['y_max']-y,
                        )
                        required_margin = (
                            mpc_bootstrap_required_boundary_margin_m(
                                bootstrap_config
                            )
                        )
                        if available_margin < required_margin-1e-12:
                            raise ValueError(
                                '--mpc needs at least '
                                f'{required_margin:.3f} m XY boundary margin '
                                'around calibration_nominal_position'
                            )
                    logger.warning(
                        'LMPC BOOTSTRAP ONLY: the drone will automatically '
                        'accelerate to speed targets %s m/s in both world-Y '
                        'directions (%d successful episodes per cell), then '
                        'the bounded legacy attitude coast controller will '
                        'brake to rest. LMPC command authority is disabled.',
                        list(bootstrap_config.initial_speed_targets_m_s),
                        bootstrap_config.repetitions_per_cell,
                    )
                if calibration_mode:
                    if getattr(self, 'bounds', None) is not None:
                        margin = braking_plan.max_displacement_m
                        x, y = float(target[0]), float(target[1])
                        if not (
                            self.bounds['x_min'] <= x - margin
                            and x + margin <= self.bounds['x_max']
                            and self.bounds['y_min'] <= y - margin
                            and y + margin <= self.bounds['y_max']
                        ):
                            raise ValueError(
                                'planar braking calibration '
                                'needs at least '
                                f'{margin:.2f} m XY boundary margin around '
                                'the nominal hover point'
                            )
                    logger.warning(
                        'Calibration includes %d %s trials at '
                        'tilt levels %s deg (maximum %.1f); keep the full '
                        '%.2f m XY safety radius clear and do not touch the '
                        'vehicle.',
                        len(braking_plan.trial_directions),
                        ('bounded adaptive-eligible attitude' if adaptive_preflight.enabled
                         else 'open-loop planar'),
                        ', '.join(
                            f'{value:g}'
                            for value in braking_plan.tilt_levels_deg
                        ),
                        braking_plan.tilt_deg,
                        braking_plan.max_displacement_m,
                    )
                    logger.warning(
                        'Calibration schedules maximum acceleration/braking durations '
                        '%s s; total scheduled motion time %.1fs plus startup '
                        'bias collection and readiness holds. No position '
                        'capture trials are run.',
                        braking_plan.trial_accelerate_s.tolist(),
                        interaction_duration,
                    )
                    if adaptive_preflight.enabled:
                        logger.warning(
                            'ACTIVE MODEL-GUIDED CALIBRATION: a frozen model '
                            'may shorten braking only after its own later opposed '
                            'pair validates it, and only for directions inside '
                            'the terminal-error margin. Experimental target '
                            'is %.2fm ahead of brake start; it is not a new '
                            'POSITION command. Existing safety limits remain active.',
                            adaptive_preflight.target_distance_m,
                        )
                interaction_function = (
                    self.interaction_onboard_wrench_admittance
                    if detection_method == 'momentum_impulse'
                    else self.interaction_wrench_admittance
                )
                prediction_config = dict(
                    (wrench_config.get('online_prediction_calibration') or {})
                    if calibration_mode and not braking_test_mode else {}
                )
                prediction_enabled = prediction_config.get('enabled', False)
                if type(prediction_enabled) is not bool:
                    raise ValueError('online_prediction_calibration.enabled must be boolean')
                if calibration_mode and not braking_test_mode and prediction_enabled:
                    if (len(braking_plan.trial_directions) < 4
                            or np.any(np.abs(braking_plan.directions[:, 0]) > 1e-6)
                            or np.any(np.abs(braking_plan.directions[:, 1]) < .999)):
                        raise ValueError(
                            'online prediction calibration currently requires '
                            'at least four complete opposed world-Y trials'
                        )
                    report_path = prediction_calibration_report_path(
                        calibration_path, self.drone_id,
                    )
                    prediction_session = OnlinePredictionCalibration(
                        prediction_config, report_path, self.drone_id,
                        expected_segment_ids=list(range(len(braking_plan.trial_directions))),
                        metadata={
                            'nominal_position_m': list(target[:3]),
                            'nominal_yaw_deg': float(nominal_yaw),
                            'control_rate_hz': float(self.ctrl_rate),
                            'protocol': braking_plan.timing_protocol(),
                            'clock_scope': 'host_receive_effective_delay',
                            'prediction_scope': 'attitude_command_only',
                            'position_capture_model_identified': False,
                            'adaptive_braking_calibration': deepcopy(adaptive_options),
                        },
                    )
                    prediction_worker_started = prediction_session.start()
                    self._log_event('Online Prediction Calibration Started', {
                        'report_path': str(report_path),
                        'fit_scope': 'attitude_command_to_tilt_velocity_position',
                        'runtime_enabled': adaptive_preflight.enabled,
                        'runtime_scope': ('adaptive_calibration_only'
                                          if adaptive_preflight.enabled else 'diagnostic_only'),
                        'worker_started': bool(prediction_worker_started),
                        'fit_updates': 'completed opposed trial pairs',
                    })
                    if prediction_worker_started:
                        logger.info('Online prediction fitting enabled in a background '
                                    'process; adaptive braking=%s. Report: %s',
                                    adaptive_preflight.enabled,
                                    report_path)
                    else:
                        if adaptive_preflight.enabled:
                            raise RuntimeError('adaptive calibration requires a running prediction worker')
                        logger.warning('Online prediction worker could not start; '
                                       'continuing the unchanged calibration protocol.')
                interaction_function(
                    duration=interaction_duration,
                    nominal_position=target[:3],
                    nominal_yaw_deg=nominal_yaw,
                    config=wrench_config,
                    virtual_object_config=(
                        virtual_object_setting
                        if detection_method == 'momentum_impulse' else None
                    ),
                    rearm_delay_s=translation_setting.get('grace_time', 0),
                    calibration_mode=calibration_mode,
                    mpc_calibration_mode=mpc_calibration_mode,
                    calibration_path=calibration_path,
                    **({'prediction_calibration': prediction_session}
                       if prediction_session is not None else {}),
                    **({
                        'braking_test_mode': True,
                        'braking_test_direction': braking_test_direction,
                        'braking_test_repetitions': braking_test_repetitions,
                    } if braking_test_mode else {}),
                )
                return

            current_mass = translation_setting.get('current_mass', translation_setting.get('mass_lightbender', 1.0))
            virtual_mass = translation_setting.get('virtual_mass', translation_setting.get('mass_virtual', 1.0))
            self.interaction_translation_vel(
                vel_threshold=translation_setting['delta_v'],
                acc_threshold=translation_setting.get(
                    'delta_a',
                    translation_setting.get(
                        'acceleration_threshold',
                        translation_setting.get('acc_threshold', None)
                    )
                ),
                z=translation_setting.get('z', None),
                fric_coe=translation_setting['friction_coefficient'],
                base_attitude=translation_setting['base_attitude'],
                duration=translation_setting['duration'],
                v_scalar=translation_setting['v_scalar'],
                grace_time=translation_setting.get('grace_time', 0),
                alpha_vel=translation_setting.get('alpha_vel', 1),
                pub_socket=self.pub_socket,
                current_mass=current_mass,
                virtual_mass=virtual_mass,
                virtual_object_config=translation_setting.get('virtual_object', None),
                init_hover=self.mission['drones'][self.drone_id]['target'][:3],
                blender_port=translation_setting.get('blender_port', None)
            )
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Translation Error: {e}\nTraceback:\n{tb_info}")
            if calibration_mode or mpc_calibration_mode:
                raise
        finally:
            try:
                exit_target = self._translation_exit_target
                if exit_target is not None:
                    self._handoff_translation_hold(*exit_target)
                elif not calibration_mode:
                    # Legacy paths have their own control lifecycle.
                    self.lo_commander.send_notify_setpoint_stop()
            finally:
                if prediction_session is not None:
                    prediction_session.close()

    def _handoff_translation_hold(self, position, yaw_deg):
        """Transfer ownership once, before blocking fits or leaving translation.

        Never send another LL setpoint after this: firmware would stop the HLC
        planner. A failed acknowledgement propagates to Controller.land(),
        which can perform a continuously streamed LL descent.
        """
        if getattr(self, '_translation_high_level_active', False):
            return
        low, high = self.lo_commander, self.hl_commander
        if isinstance(low, CommandWrapper):
            low = low.for_safety_cleanup()
        if isinstance(high, CommandWrapper):
            high = high.for_safety_cleanup()
        try:
            result = handoff_to_high_level(
                low, high, 'go_to', *position, float(np.radians(yaw_deg)),
                1.0, relative=False,
                dry_run=all(isinstance(c, CommandWrapper) and c.execution is False
                            for c in (low, high)),
            )
        except HandoffError:
            # The HLC command might have started even though its reply was
            # lost. Reassert LL ownership immediately and refresh the
            # watchdog before closing the prediction worker or unwinding.
            low.send_position_setpoint(
                *position, float(yaw_deg),
            )
            raise
        self._translation_high_level_active = True
        # Ownership is already safe even if writing this event fails.
        self._log_event('Translation High Level Hold Acquired', {
            'position_m': list(position), 'yaw_deg': float(yaw_deg),
            'handoff': result,
        })

    @staticmethod
    def _contact_log(decision):
        if decision is None:
            return None
        return {
            'active': bool(decision.active),
            'started': bool(decision.started),
            'ended': bool(decision.ended),
            'magnitude': float(decision.magnitude),
            'normalized_magnitude': float(decision.normalized_magnitude),
            'confidence_sigma': float(decision.confidence_sigma),
            'evidence': float(decision.evidence),
            'release_projected_force_N': decision.release_projected_value,
            'release_projection_normalized': (
                decision.release_projection_normalized
            ),
            'release_direction': (
                None
                if decision.release_direction is None
                else list(decision.release_direction)
            ),
            'release_direction_source': decision.release_direction_source,
            'release_candidate_active': bool(
                decision.release_candidate_active
            ),
            'release_candidate_started': bool(
                decision.release_candidate_started
            ),
            'release_candidate_cancelled': bool(
                decision.release_candidate_cancelled
            ),
            'release_elapsed_s': float(decision.release_elapsed_s),
        }

    def _bounded_wrench_reference(self, position):
        position = np.asarray(position, dtype=float)
        if self.bounds is None:
            return position
        return np.clip(
            position,
            [self.bounds['x_min'], self.bounds['y_min'], self.bounds['z_min']],
            [self.bounds['x_max'], self.bounds['y_max'], self.bounds['z_max']],
        )

    def _emit_guided_touch_prompts(self, protocol, elapsed_s, state_source):
        """Print and log scheduled human-touch ground-truth markers."""
        for scheduled_s, event_name, message, data, emphasize in protocol.due(
                elapsed_s):
            payload = {
                **data,
                'scheduled_after_calibration_s': scheduled_s,
                'state_source': state_source,
            }
            self._log_event(event_name, payload)
            (logger.warning if emphasize else logger.info)(message)

    def _calibration_excitation_reference(
            self, nominal_position, nominal_yaw_deg, config, elapsed_s,
    ):
        """Return a bounded contact-free XYZ/yaw identification reference."""
        amplitudes = np.asarray(config['translation_amplitude_m'], dtype=float)
        frequencies = np.asarray(config['translation_frequency_hz'], dtype=float)
        if amplitudes.shape != (3,) or frequencies.shape != (3,):
            raise ValueError(
                'calibration_excitation translation amplitude/frequency '
                'must each contain X, Y, and Z'
            )
        elapsed_s = float(elapsed_s)
        duration_s = float(config['duration_s'])
        if duration_s <= 0.0:
            raise ValueError('calibration_excitation duration_s must be positive')
        translation_profile = config.get('translation_profile', 'sine')
        if translation_profile == 'sine':
            translation_phase = 2.0 * np.pi * frequencies * elapsed_s
            translation_offset = amplitudes * np.sin(translation_phase)
        elif translation_profile == 'chirp':
            end_frequencies = np.asarray(
                config['translation_chirp_end_hz'], dtype=float
            )
            if end_frequencies.shape != (3,) or np.any(end_frequencies <= 0.0):
                raise ValueError(
                    'translation_chirp_end_hz must contain positive XYZ values'
                )
            sweep_rates = (end_frequencies - frequencies) / duration_s
            translation_phase = 2.0 * np.pi * (
                frequencies * elapsed_s
                + 0.5 * sweep_rates * elapsed_s ** 2
            )
            translation_offset = amplitudes * np.sin(translation_phase)
        elif translation_profile == 'sequential_chirp':
            end_frequencies = np.asarray(
                config['translation_chirp_end_hz'], dtype=float
            )
            rest_s = float(config.get('translation_axis_rest_s', 1.0))
            ramp_s = float(config.get('translation_ramp_s', 0.6))
            segment_s = (duration_s - 2.0 * rest_s) / 3.0
            if (
                end_frequencies.shape != (3,)
                or np.any(end_frequencies <= 0.0)
                or rest_s < 0.0
                or ramp_s < 0.0
                or segment_s <= 2.0 * ramp_s
            ):
                raise ValueError(
                    'sequential chirp requires positive XYZ end frequencies '
                    'and enough duration for three ramped axis segments'
                )
            translation_offset = np.zeros(3)
            block_s = segment_s + rest_s
            axis = min(int(elapsed_s // block_s), 2)
            local_s = elapsed_s - axis * block_s
            if 0.0 <= local_s < segment_s:
                sweep_rate = (
                    end_frequencies[axis] - frequencies[axis]
                ) / segment_s
                phase = 2.0 * np.pi * (
                    frequencies[axis] * local_s
                    + 0.5 * sweep_rate * local_s ** 2
                )
                envelope = (
                    min(local_s / ramp_s, (segment_s - local_s) / ramp_s, 1.0)
                    if ramp_s > 0.0 else 1.0
                )
                translation_offset[axis] = (
                    amplitudes[axis] * max(envelope, 0.0) * np.sin(phase)
                )
        else:
            raise ValueError(
                f'Unsupported translation excitation profile: {translation_profile}'
            )
        position = self._bounded_wrench_reference(
            np.asarray(nominal_position, dtype=float)
            + translation_offset
        )

        yaw_amplitude_deg = float(config['yaw_amplitude_deg'])
        yaw_profile = config.get('yaw_profile', 'sine')
        if yaw_profile == 'sine':
            phase = 2.0 * np.pi * float(config['yaw_frequency_hz']) * elapsed_s
            envelope = 1.0
        elif yaw_profile == 'chirp':
            start_hz = float(config['yaw_chirp_start_hz'])
            end_hz = float(config['yaw_chirp_end_hz'])
            if start_hz <= 0.0 or end_hz <= 0.0:
                raise ValueError('yaw chirp frequencies must be positive')
            sweep_rate = (end_hz - start_hz) / duration_s
            phase = 2.0 * np.pi * (
                start_hz * elapsed_s + 0.5 * sweep_rate * elapsed_s ** 2
            )
            ramp_s = min(
                max(float(config.get('yaw_ramp_s', 1.0)), 0.0),
                duration_s / 2.0,
            )
            if ramp_s > 0.0:
                ramp_in = 0.5 * (1.0 - np.cos(
                    np.pi * min(elapsed_s / ramp_s, 1.0)
                ))
                remaining_s = max(duration_s - elapsed_s, 0.0)
                ramp_out = 0.5 * (1.0 - np.cos(
                    np.pi * min(remaining_s / ramp_s, 1.0)
                ))
                envelope = min(ramp_in, ramp_out)
            else:
                envelope = 1.0
        else:
            raise ValueError(f'Unsupported yaw excitation profile: {yaw_profile}')
        yaw_deg = (
            float(nominal_yaw_deg)
            + yaw_amplitude_deg * envelope * np.sin(phase)
        )
        return position, yaw_deg

    def interaction_wrench_admittance(
            self,
            duration,
            nominal_position,
            nominal_yaw_deg=0.0,
            config=None,
            virtual_object_config=None,
            rearm_delay_s=0.0,
            calibration_mode=False,
            mpc_calibration_mode=False,
            calibration_path=DEFAULT_CALIBRATION_PATH,
    ):
        """Estimate external wrench and generate bounded XYZ/yaw references.

        The Crazyflie position PID remains the flight controller. External XYZ
        force feeds a virtual mass/damper/spring reference generator. Optional
        yaw interaction uses a separate yaw admittance when enabled.
        """
        pipeline = WrenchInteractionPipeline(config)
        config = pipeline.config
        safety = config['safety']
        dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01
        config['control_handoff']['coast_command_period_s'] = max(
            float(config['control_handoff']['coast_command_period_s']), dt
        )
        duration = float(duration)
        nominal_position = np.asarray(nominal_position, dtype=float)
        if nominal_position.shape != (3,):
            raise ValueError('nominal_position must contain X, Y, and Z')
        nominal_position = self._bounded_wrench_reference(nominal_position)
        nominal_yaw_deg = float(nominal_yaw_deg)

        if config.get('shadow_mode', True):
            logger.warning(
                'Wrench interaction is in shadow mode: contacts and proposed '
                'responses are logged, but the reference remains fixed.'
            )
        if config.get('blender_port'):
            logger.warning('Blender edit streaming is not used by wrench interaction mode.')

        self.log_manager.add_log_entry(
            'configs',
            {
                'pipeline': 'external_wrench_admittance_pid',
                'detection_method': 'mocap_wrench',
                'translation_response_axes': ['x', 'y', 'z'],
                'force_sensor_comparison': self._force_sensor_config(),
                'rotation_response_axes': (
                    ['yaw'] if config['detection']['yaw'].get('enabled', True)
                    else []
                ),
                'nominal_position': nominal_position.tolist(),
                'nominal_yaw_deg': nominal_yaw_deg,
                'translation_rearm_delay_s': float(rearm_delay_s),
                'config': config,
            },
            name='Wrench Interaction Config',
        )

        self._translation_exit_target = (nominal_position.tolist(), nominal_yaw_deg)
        self._translation_high_level_active = False
        self.hl_commander.go_to(
            nominal_position[0], nominal_position[1], nominal_position[2],
            nominal_yaw_deg, 2.0, relative=False,
        )
        self._safe_sleep(2.0)

        startup_deadline = time.time() + float(safety['startup_timeout_s'])
        while True:
            frames = self.log_manager.groups.get(self.pos_group_name, [])
            if frames:
                frame = frames[-1]
                if frame.get('quat') is not None and frame.get('tvec') is not None:
                    break
            if time.time() >= startup_deadline:
                raise StaleLocalizationError(
                    'No full-pose mocap frame received. Use rigidbody localization '
                    'with --vicon-full-pose and verify the Vicon object label.'
                )
            self.lo_commander.send_position_setpoint(
                *nominal_position, nominal_yaw_deg
            )
            self._safe_sleep(dt)

        bias_calibration_enabled = bool(
            config['startup_bias_calibration_enabled']
        )
        if bias_calibration_enabled:
            self._log_event('Wrench Calibration Started', {
                'instruction': 'Do not touch the drone until calibration completes.',
                'shadow_mode': pipeline.shadow_mode,
            })
            logger.info(
                'Calibrating the external-wrench observer; do not touch the drone.'
            )

        last_frame_marker = None
        interaction_start = None
        calibration_announced = False
        last_command_position = nominal_position.copy()
        last_command_yaw = nominal_yaw_deg
        translation_control = TranslationControlHandoff(
            nominal_position,
            nominal_yaw_deg,
            pipeline.shadow_mode,
            rearm_delay_s=rearm_delay_s,
            **config['control_handoff'],
        )
        excitation_config = config['calibration_excitation']
        guided_touch = GuidedTouchProtocol(config.get('guided_touch_test'))
        if guided_touch.enabled and duration < guided_touch.required_duration_s:
            raise ValueError(
                f'interaction duration {duration:.1f}s is shorter than the '
                f'guided touch sequence ({guided_touch.required_duration_s:.1f}s)'
            )
        excitation_started = False
        excitation_finished = False

        while interaction_start is None or time.time() - interaction_start < duration:
            now = time.time()
            frames = self.log_manager.groups.get(self.pos_group_name, [])
            if not frames:
                raise StaleLocalizationError('Full-pose mocap frame buffer is empty')
            frame = frames[-1]
            frame_time = float(frame.get('time', 0.0))
            frame_age = now - frame_time
            if frame_age < -0.5 or frame_age > float(safety['max_frame_age_s']):
                raise StaleLocalizationError(
                    f'Mocap frame is {frame_age:.3f}s old '
                    f"(limit {float(safety['max_frame_age_s']):.3f}s)"
                )
            if frame.get('quat') is None:
                raise StaleLocalizationError(
                    'Mocap frame has no quaternion; rigidbody full-pose is required'
                )

            frame_marker = (frame.get('frame_id'), frame_time)
            if frame_marker == last_frame_marker:
                translation_control.send(self.lo_commander)
                self._safe_sleep(dt)
                continue
            last_frame_marker = frame_marker

            position = np.asarray(frame['tvec'], dtype=float)
            self.check_interaction_boundary(position)
            motor_state, motor_pose_skew = self.log_manager.get_nearest_group_log_data(
                'MOT_BAT', frame_time
            )
            motor_state = motor_state or {}
            motor_pwm = [motor_state.get(f'motor.m{i}') for i in range(1, 5)]
            if not WrenchInteractionPipeline.motor_data_available(motor_pwm):
                motor_pwm = None
            battery_voltage = motor_state.get('pm.vbat')
            battery_available = (
                isinstance(battery_voltage, (int, float))
                and np.isfinite(battery_voltage)
                and battery_voltage > 0
            )
            if not battery_available:
                battery_voltage = None
            motor_log_time = motor_state.get('time')
            motor_age = None if motor_log_time is None else now - motor_log_time

            motor_is_stale = (
                motor_age is None
                or motor_age < -0.5
                or motor_age > float(safety['max_motor_age_s'])
            )
            motor_is_unsynchronized = (
                motor_pose_skew is None
                or motor_pose_skew > float(safety['max_motor_pose_skew_s'])
            )
            if safety['require_motor_data'] and (
                motor_pwm is None
                or battery_voltage is None
                or motor_is_stale
                or motor_is_unsynchronized
            ):
                raise RuntimeError(
                    'Fresh, pose-synchronized motor PWM and battery data are required '
                    'for wrench estimation; '
                    'check MOT_BAT logging and the Crazyflie connection'
                )

            output = pipeline.update(
                position=position,
                quaternion=frame['quat'],
                motor_pwm=motor_pwm,
                battery_voltage=battery_voltage,
                timestamp=frame_time,
            )

            if output.calibrated and not calibration_announced:
                calibration_announced = True
                interaction_start = time.time()
                if bias_calibration_enabled:
                    self._log_event('Wrench Calibration Complete', {
                        'samples': output.calibration_samples,
                        'force_bias_N': pipeline.force_bias.tolist(),
                        'torque_bias_Nm': pipeline.torque_bias.tolist(),
                    })
                self._log_event('Waiting For User Interaction')
                logger.info('Interaction detection is active.')

            contacts = output.contacts
            if contacts is not None:
                transitions = (
                    ('Translation Contact', contacts.translation),
                    ('Yaw Contact', contacts.yaw),
                )
                for event_name, decision in transitions:
                    if decision.started or decision.ended:
                        self._log_event(
                            f"{event_name} {'Start' if decision.started else 'End'}",
                            {
                                'force_N': output.estimate.external_force.tolist(),
                                'torque_Nm': output.estimate.external_torque.tolist(),
                                'confidence_sigma': decision.confidence_sigma,
                                'release_projected_force_N': (
                                    decision.release_projected_value
                                ),
                                'release_projection_normalized': (
                                    decision.release_projection_normalized
                                ),
                                'release_direction': decision.release_direction,
                                'release_direction_source': (
                                    decision.release_direction_source
                                ),
                                'response_enabled': not pipeline.shadow_mode,
                            },
                        )
                        if event_name == 'Translation Contact':
                            if decision.started and translation_control.start_contact():
                                pipeline.admittance.reset()
                                self._log_event(
                                    'Translation Attitude Control Started',
                                    {
                                        'zdistance_m': translation_control.hover_z,
                                        'roll_deg': 0.0,
                                        'pitch_deg': 0.0,
                                        'yaw_rate_deg_s': 0.0,
                                    },
                                )
                            elif decision.ended and translation_control.end_contact(
                                    self._bounded_wrench_reference(position),
                                    output.estimate.velocity,
                                    frame_time,
                                    decision.release_direction,
                                    output.estimate.orientation_rpy):
                                pipeline.admittance.reset()
                                self._log_event(
                                    'Translation Attitude Braking Started',
                                    {
                                        'xy_speed_m_s': float(np.linalg.norm(
                                            output.estimate.velocity[:2]
                                        )),
                                        'projected_speed_m_s': (
                                            translation_control.brake_projected_speed_m_s
                                        ),
                                        'interaction_direction': (
                                            decision.release_direction
                                        ),
                                        'brake_direction': (
                                            translation_control.brake_direction.tolist()
                                        ),
                                        'brake_direction_source': (
                                            translation_control.brake_direction_source
                                        ),
                                        'brake_roll_deg': (
                                            translation_control.contact_roll_deg
                                        ),
                                        'brake_pitch_deg': (
                                            translation_control.contact_pitch_deg
                                        ),
                                        'brake_timeout_s': (
                                            translation_control.brake_timeout_s
                                        ),
                                        'brake_velocity_gain_s': (
                                            translation_control.brake_velocity_gain_s
                                        ),
                                        'brake_min_attitude_deg': (
                                            translation_control.brake_min_attitude_deg
                                        ),
                                        'brake_command_tilt_deg': (
                                            translation_control.brake_command_tilt_deg
                                        ),
                                    },
                                )

            if translation_control.update_braking(
                    self._bounded_wrench_reference(position),
                    output.estimate.velocity,
                    frame_time,
                    output.estimate.orientation_rpy):
                last_command_position = translation_control.hold_position.copy()
                self._log_event(
                    'Translation Position Hold Resumed',
                    {
                        'hold_position_m': last_command_position.tolist(),
                        'xy_speed_m_s': float(np.linalg.norm(
                            output.estimate.velocity[:2]
                        )),
                        'projected_speed_m_s': (
                            translation_control.brake_projected_speed_m_s
                        ),
                        'brake_completion_reason': (
                            translation_control.brake_completion_reason
                        ),
                        'detector_rearm_delay_s': (
                            translation_control.rearm_delay_s
                        ),
                    },
                )

            if translation_control.consume_detector_rearm(frame_time):
                pipeline.detector.translation.reset(frame_time)
                self._log_event(
                    'Translation Contact Detector Rearmed',
                    {'rearm_delay_s': translation_control.rearm_delay_s},
                )

            if interaction_start is not None:
                self._emit_guided_touch_prompts(
                    guided_touch,
                    time.time() - interaction_start,
                    'mocap_full_pose',
                )

            baseline_position = nominal_position.copy()
            baseline_yaw = nominal_yaw_deg
            excitation_active = False
            if interaction_start is not None and excitation_config['enabled']:
                excitation_elapsed = time.time() - interaction_start
                excitation_time = excitation_elapsed - float(excitation_config['start_delay_s'])
                excitation_duration = float(excitation_config['duration_s'])
                if 0.0 <= excitation_time < excitation_duration:
                    excitation_active = True
                    baseline_position, baseline_yaw = (
                        self._calibration_excitation_reference(
                            nominal_position, nominal_yaw_deg,
                            excitation_config, excitation_time,
                        )
                    )
                    if not excitation_started:
                        excitation_started = True
                        self._log_event('Wrench Calibration Excitation Started', {
                            'instruction': 'Do not touch the drone during this motion.',
                        })
                elif excitation_started and not excitation_finished:
                    excitation_finished = True
                    self._log_event('Wrench Calibration Excitation Complete')

            proposed_position = self._bounded_wrench_reference(
                baseline_position + output.admittance.translation_offset
            )
            proposed_yaw = baseline_yaw + float(np.degrees(output.admittance.yaw_offset))
            if pipeline.shadow_mode or not output.calibrated:
                command_position = baseline_position
                command_yaw = baseline_yaw
                translation_control.hold_position = np.asarray(
                    command_position, dtype=float
                ).copy()
                translation_control.yaw_deg = float(command_yaw)
                translation_control.send(self.lo_commander)
            elif not translation_control.uses_position_setpoint:
                command_position = None
                command_yaw = translation_control.yaw_deg
                translation_control.send(self.lo_commander)
            else:
                command_position = translation_control.hold_position.copy()
                command_yaw = translation_control.yaw_deg
                last_command_position = command_position.copy()
                last_command_yaw = float(command_yaw)
                translation_control.send(self.lo_commander)

            estimate = output.estimate
            raw = output.raw_estimate
            self.log_manager.add_log_entry('wrench_observer', {
                'time': now,
                'frame_time': frame_time,
                'frame_age_s': frame_age,
                'position_m': position.tolist(),
                'orientation_rpy_rad': estimate.orientation_rpy.tolist(),
                'velocity_m_s': estimate.velocity.tolist(),
                'angular_velocity_rad_s': estimate.angular_velocity.tolist(),
                'expected_linear_acceleration_m_s2': output.expected_linear_acceleration.tolist(),
                'expected_angular_acceleration_rad_s2': output.expected_angular_acceleration.tolist(),
                'raw_external_force_N': raw.external_force.tolist(),
                'raw_external_torque_Nm': raw.external_torque.tolist(),
                'force_bias_N': pipeline.force_bias.tolist(),
                'torque_bias_Nm': pipeline.torque_bias.tolist(),
                'external_force_N': estimate.external_force.tolist(),
                **self._force_sensor_log_fields(estimate, now),
                'external_torque_Nm': estimate.external_torque.tolist(),
                'force_covariance': estimate.force_covariance.tolist(),
                'torque_covariance': estimate.torque_covariance.tolist(),
                'position_innovation_m': estimate.position_innovation.tolist(),
                'orientation_innovation_rad': estimate.orientation_innovation.tolist(),
                'position_nis': estimate.position_nis,
                'orientation_nis': estimate.orientation_nis,
                'measurement_rejected': bool(estimate.measurement_rejected),
                'motor_data_available': bool(output.motor_data_available),
                'battery_data_available': bool(battery_available),
                'motor_data_age_s': motor_age,
                'motor_pose_skew_s': motor_pose_skew,
                'motor_pwm': motor_pwm,
                'battery_voltage_V': battery_voltage,
                'calibrated': bool(output.calibrated),
                'calibration_samples': output.calibration_samples,
                'translation_contact': self._contact_log(
                    contacts.translation if contacts else None
                ),
                'yaw_contact': self._contact_log(contacts.yaw if contacts else None),
                'translation_offset_m': output.admittance.translation_offset.tolist(),
                'translation_reference_velocity_m_s': output.admittance.translation_velocity.tolist(),
                'yaw_offset_rad': output.admittance.yaw_offset,
                'yaw_reference_rate_rad_s': output.admittance.yaw_rate,
                'baseline_position_m': baseline_position.tolist(),
                'baseline_yaw_deg': baseline_yaw,
                'calibration_excitation_active': excitation_active,
                'proposed_position_m': proposed_position.tolist(),
                'proposed_yaw_deg': proposed_yaw,
                'command_mode': translation_control.command_mode,
                'command_position_m': (
                    None if command_position is None
                    else np.asarray(command_position, dtype=float).tolist()
                ),
                'command_zdistance_m': (
                    translation_control.hover_z
                    if not translation_control.uses_position_setpoint else None
                ),
                'command_roll_deg': (
                    translation_control.contact_roll_deg
                    if not translation_control.uses_position_setpoint else None
                ),
                'command_pitch_deg': (
                    translation_control.contact_pitch_deg
                    if not translation_control.uses_position_setpoint else None
                ),
                'brake_projected_speed_m_s': (
                    translation_control.brake_projected_speed_m_s
                    if translation_control.braking_mode else None
                ),
                'brake_command_tilt_deg': (
                    translation_control.brake_command_tilt_deg
                    if translation_control.braking_mode else None
                ),
                'command_xy_velocity_m_s': None,
                'command_xy_velocity_world_m_s': None,
                'command_yaw_deg': float(command_yaw),
                'shadow_mode': pipeline.shadow_mode,
            })
            self._safe_sleep(dt)

        self._log_event('Wrench Interaction Complete')

    def _get_synchronized_onboard_wrench_state(self):
        """Return time-aligned Crazyflie state-estimate and actuator packets."""
        state_time = self.log_manager.get_latest_group_log_time('VEL_ORI')
        if state_time is None:
            return None

        velocity_attitude, _ = self.log_manager.get_nearest_group_log_data(
            'VEL_ORI', state_time
        )
        position_acceleration, position_skew = (
            self.log_manager.get_nearest_group_log_data('POS_ACC', state_time)
        )
        angular_rate, angular_rate_skew = self.log_manager.get_nearest_group_log_data(
            'RATE_EST', state_time
        )
        yaw_control, yaw_control_skew = self.log_manager.get_nearest_group_log_data(
            'YAW_CTL', state_time
        )
        motor_state, motor_skew = self.log_manager.get_nearest_group_log_data(
            'MOT_BAT', state_time
        )
        if not all((
                velocity_attitude, position_acceleration, angular_rate,
                yaw_control, motor_state,
        )):
            return None

        try:
            position = np.asarray([
                position_acceleration['stateEstimate.x'],
                position_acceleration['stateEstimate.y'],
                position_acceleration['stateEstimate.z'],
            ], dtype=float)
            velocity = np.asarray([
                velocity_attitude['stateEstimate.vx'],
                velocity_attitude['stateEstimate.vy'],
                velocity_attitude['stateEstimate.vz'],
            ], dtype=float)
            attitude_rpy = np.radians(np.asarray([
                velocity_attitude['stateEstimate.roll'],
                velocity_attitude['stateEstimate.pitch'],
                velocity_attitude['stateEstimate.yaw'],
            ], dtype=float))
            # stateEstimateZ angular rates are compressed milliradians/second.
            angular_velocity = 0.001 * np.asarray([
                angular_rate['stateEstimateZ.rateRoll'],
                angular_rate['stateEstimateZ.ratePitch'],
                angular_rate['stateEstimateZ.rateYaw'],
            ], dtype=float)
            yaw_control_command = float(yaw_control['controller.cmd_yaw'])
            controller_yaw_rate = float(yaw_control['controller.r_yaw'])
        except (KeyError, TypeError, ValueError):
            return None
        if not all(np.all(np.isfinite(value)) for value in (
            position, velocity, attitude_rpy, angular_velocity,
            yaw_control_command, controller_yaw_rate,
        )):
            return None

        return {
            'time': float(state_time),
            'position': position,
            'velocity': velocity,
            'attitude_rpy': attitude_rpy,
            'angular_velocity': angular_velocity,
            'position_skew_s': position_skew,
            'angular_rate_skew_s': angular_rate_skew,
            'yaw_control_skew_s': yaw_control_skew,
            'yaw_control_command': yaw_control_command,
            'controller_yaw_rate': controller_yaw_rate,
            'motor_skew_s': motor_skew,
            'motor_state': motor_state,
        }

    def interaction_onboard_wrench_admittance(
            self,
            duration,
            nominal_position,
            nominal_yaw_deg=0.0,
            config=None,
            virtual_object_config=None,
            rearm_delay_s=0.0,
            calibration_mode=False,
            mpc_calibration_mode=False,
            calibration_path=DEFAULT_CALIBRATION_PATH,
            braking_test_mode=False,
            braking_test_direction=None,
            braking_test_repetitions=None,
            prediction_calibration=None,
    ):
        """Run wrench interaction from synchronized onboard state estimates.

        This is intentionally separate from ``interaction_wrench_admittance``.
        The original full-pose mocap/Kalman observer path remains available by
        selecting ``state_source: mocap``.
        """
        if mpc_calibration_mode and calibration_mode:
            raise ValueError('--mpc cannot use the legacy calibration path')
        if mpc_calibration_mode and braking_test_mode:
            raise ValueError('--mpc cannot use the braking repeat-test path')
        if prediction_calibration is not None and (not calibration_mode or braking_test_mode):
            raise ValueError('online prediction fitting is only supported by --calibrate')
        if braking_test_mode:
            if not calibration_mode or self.ctrl_rate < 50:
                raise ValueError('braking repeat test needs calibration mode and at least 50 Hz')
            config = repeat_test_config(
                config or {}, direction=braking_test_direction,
                repetitions=braking_test_repetitions,
            )
        pipeline = OnboardMomentumWrenchPipeline(config)
        config = pipeline.config
        bootstrap_coverage = None
        bootstrap_model_contracts = None
        if mpc_calibration_mode:
            bootstrap_enabled = dict(
                config.get('mpc_bootstrap_calibration') or {}
            ).get('enabled', False)
            if bootstrap_enabled is not True:
                raise ValueError(
                    '--mpc requires mpc_bootstrap_calibration.enabled=true '
                    'in its private mission overlay'
                )
            bootstrap_coverage = MPCBootstrapCoverage(
                MPCBootstrapCalibrationConfig.from_mapping(
                    config.get('mpc_bootstrap_calibration')
                )
            )
            bootstrap_model_contracts = (
                validate_mpc_bootstrap_model_contracts(
                    config.get('mpc_bootstrap_model_contracts'),
                    expected_prediction_step_s=(
                        bootstrap_coverage.config.prediction_step_s
                    ),
                )
            )
            if pipeline.shadow_mode:
                raise ValueError('--mpc requires shadow_mode=false')
        force_sensor_available = bool(
            getattr(self, 'force_sensor', None) is not None
            and not calibration_mode
        )
        safety = config['safety']
        dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01
        config['control_handoff']['coast_command_period_s'] = (
            bootstrap_coverage.config.prediction_step_s
            if bootstrap_coverage is not None else max(
                float(config['control_handoff']['coast_command_period_s']), dt
            )
        )
        duration = float(duration)
        nominal_position = np.asarray(nominal_position, dtype=float)
        if nominal_position.shape != (3,):
            raise ValueError('nominal_position must contain X, Y, and Z')
        nominal_position = self._bounded_wrench_reference(nominal_position)
        nominal_yaw_deg = float(nominal_yaw_deg)

        virtual_object_config = virtual_object_config or {}
        preferred_render_mode = 'position'
        force_current_mass = float(config['mass'])
        force_virtual_mass = force_current_mass
        force_max_attitude_deg = 20.0
        force_kinetic_friction_coefficient = 0.0
        force_static_friction_coefficient = 0.0
        force_drag_coefficient = 0.0
        force_frontal_area = 0.019
        force_air_density = 1.225
        force_friction_min_speed_m_s = 0.02
        render_acceleration_tolerance_m_s2 = 0.02
        virtual_max_velocity_m_s = 0.60
        force_rendering_enabled = False
        default_release_mode = (
            'potentiometer_coast'
            if force_sensor_available else 'observer_brake'
        )
        release_mode = default_release_mode
        release_force_drop_n = 0.01
        release_decrease_rate_n_s = 0.05
        release_unloaded_force_n = 0.05
        release_unloaded_dwell_s = 0.05
        release_max_sample_gap_s = 0.15
        release_candidate_stall_timeout_s = 0.50
        release_candidate_sensor_stale_timeout_s = 0.25
        release_candidate_lead_drop_n = None
        release_force_memory_s = 0.02
        configured_contact_detection_source = 'wrench_observer'
        contact_detection_source = configured_contact_detection_source
        potentiometer_contact_force_n = 0.08
        potentiometer_contact_dwell_s = 0.03
        if virtual_object_config:
            force_current_mass = float(virtual_object_config.get(
                'current_mass', config['mass']
            ))
            force_virtual_mass = float(virtual_object_config.get(
                'mass', force_current_mass
            ))
            mass_class = velocity_inertia_mass_class(
                force_current_mass, force_virtual_mass
            )
            requested_mode = inertia_command_mode(
                mass_class, virtual_object_config.get('inertia_command')
            )
            preferred_render_mode = requested_mode
            if preferred_render_mode not in ('position', 'orientation'):
                raise ValueError(
                    'momentum force rendering inertia_command must be '
                    'position or orientation'
                )
            force_max_attitude_deg = float(
                virtual_object_config.get('max_attitude_deg', 20.0)
            )
            force_kinetic_friction_coefficient = float(
                virtual_object_config.get(
                    'kinetic_friction_coefficient',
                    virtual_object_config.get('friction_coefficient', 0.0),
                )
            )
            force_static_friction_coefficient = float(
                virtual_object_config.get(
                    'static_friction_coefficient', 0.0
                )
            )
            force_drag_coefficient = float(
                virtual_object_config.get('drag_coefficient', 0.0)
            )
            force_frontal_area = float(
                virtual_object_config.get('frontal_area', 0.019)
            )
            force_air_density = float(
                virtual_object_config.get('air_density', 1.225)
            )
            force_friction_min_speed_m_s = float(
                virtual_object_config.get(
                    'friction_min_speed_m_s', 0.02
                )
            )
            render_acceleration_tolerance_m_s2 = float(
                virtual_object_config.get(
                    'render_acceleration_tolerance_m_s2', 0.02
                )
            )
            virtual_max_velocity_m_s = float(
                virtual_object_config.get(
                    'max_velocity_command_m_s', 0.60
                )
            )
            force_rendering_config = virtual_object_config.get(
                'force_rendering', {}
            )
            if not isinstance(force_rendering_config, dict):
                raise ValueError('virtual_object.force_rendering must be a mapping')
            force_rendering_enabled = bool(
                force_rendering_config.get('enabled', False)
            )
            release_config = virtual_object_config.get(
                'release_behavior', {}
            )
            if not isinstance(release_config, dict):
                raise ValueError('virtual_object.release_behavior must be a mapping')
            release_mode = str(
                release_config.get('mode', default_release_mode)
            )
            release_force_drop_n = float(
                release_config.get('force_drop_n', 0.01)
            )
            release_decrease_rate_n_s = float(
                release_config.get('decrease_rate_n_s', 0.05)
            )
            release_unloaded_force_n = float(
                release_config.get('unloaded_force_n', 0.05)
            )
            release_unloaded_dwell_s = float(
                release_config.get('unloaded_dwell_s', 0.05)
            )
            release_max_sample_gap_s = float(
                release_config.get('max_sample_gap_s', 0.15)
            )
            release_candidate_stall_timeout_s = float(
                release_config.get('candidate_stall_timeout_s', 0.50)
            )
            release_candidate_sensor_stale_timeout_s = float(
                release_config.get(
                    'candidate_sensor_stale_timeout_s', 0.25
                )
            )
            configured_candidate_lead_drop_n = release_config.get(
                'candidate_lead_drop_n'
            )
            release_candidate_lead_drop_n = (
                None
                if configured_candidate_lead_drop_n is None
                else float(configured_candidate_lead_drop_n)
            )
            release_force_memory_s = float(
                release_config.get('force_memory_s', 0.02)
            )
            contact_detection_config = virtual_object_config.get(
                'contact_detection', {}
            )
            if not isinstance(contact_detection_config, dict):
                raise ValueError(
                    'virtual_object.contact_detection must be a mapping'
                )
            configured_contact_detection_source = str(
                contact_detection_config.get(
                    'source', configured_contact_detection_source
                )
            ).strip().lower()
            potentiometer_contact_force_n = float(
                contact_detection_config.get('force_threshold_n', 0.08)
            )
            potentiometer_contact_dwell_s = float(
                contact_detection_config.get('onset_dwell_s', 0.03)
            )
        two_afc_config = virtual_object_config.get('two_afc_friction', {})
        if not isinstance(two_afc_config, dict):
            raise ValueError('virtual_object.two_afc_friction must be a mapping')
        two_afc_config = dict(two_afc_config)
        two_afc_configured_enabled = two_afc_config.get('enabled', False)
        if type(two_afc_configured_enabled) is not bool:
            raise ValueError('two_afc_friction.enabled must be boolean')
        if calibration_mode:
            two_afc_config['enabled'] = False
        two_afc_friction = PairedFrictionRandomizer(two_afc_config)
        force_max_attitude_deg = _validated_attitude_limit(
            force_max_attitude_deg, 'force rendering'
        )
        configured_release_mode = release_mode
        release_mode = resolve_release_mode(
            configured_release_mode,
            force_sensor_available,
            calibration_mode=calibration_mode,
        )
        velocity_coast_braking_enabled = config['control_handoff'].get(
            'coast_velocity_braking_enabled', False
        )
        if type(velocity_coast_braking_enabled) is not bool:
            raise ValueError(
                'control_handoff.coast_velocity_braking_enabled must be '
                'boolean'
            )
        velocity_predictive_unwind_enabled = config['control_handoff'].get(
            'coast_velocity_predictive_unwind_enabled', False
        )
        if type(velocity_predictive_unwind_enabled) is not bool:
            raise ValueError(
                'control_handoff.coast_velocity_predictive_unwind_enabled '
                'must be boolean'
            )
        velocity_rebrake_enabled = config['control_handoff'].get(
            'coast_velocity_rebrake_enabled', True
        )
        if type(velocity_rebrake_enabled) is not bool:
            raise ValueError(
                'control_handoff.coast_velocity_rebrake_enabled must be '
                'boolean'
            )
        velocity_unwind_direct_level_attitude_enabled = (
            config['control_handoff'].get(
                'coast_velocity_unwind_direct_level_attitude_enabled', False
            )
        )
        if type(velocity_unwind_direct_level_attitude_enabled) is not bool:
            raise ValueError(
                'control_handoff.'
                'coast_velocity_unwind_direct_level_attitude_enabled must be '
                'boolean'
            )
        velocity_unwind_position_control_enabled = (
            config['control_handoff'].get(
                'coast_velocity_unwind_position_control_enabled', False
            )
        )
        if type(velocity_unwind_position_control_enabled) is not bool:
            raise ValueError(
                'control_handoff.'
                'coast_velocity_unwind_position_control_enabled must be '
                'boolean'
            )
        if (
            velocity_predictive_unwind_enabled
            and not velocity_coast_braking_enabled
        ):
            raise ValueError(
                'predictive velocity unwind requires '
                'coast_velocity_braking_enabled'
            )
        if (
            velocity_unwind_position_control_enabled
            and not velocity_predictive_unwind_enabled
        ):
            raise ValueError(
                'position-controlled velocity unwind requires '
                'coast_velocity_predictive_unwind_enabled'
            )
        if (
            velocity_unwind_direct_level_attitude_enabled
            and not velocity_predictive_unwind_enabled
        ):
            raise ValueError(
                'direct level-attitude velocity unwind requires '
                'coast_velocity_predictive_unwind_enabled'
            )
        if (
            velocity_unwind_direct_level_attitude_enabled
            and velocity_unwind_position_control_enabled
        ):
            raise ValueError(
                'direct level-attitude unwind and position-controlled unwind '
                'cannot both be enabled'
            )
        velocity_coast_handoff_speed_m_s = float(
            config['control_handoff'].get(
                'coast_velocity_handoff_speed_m_s', 0.03
            )
        )
        if (
            not np.isfinite(velocity_coast_handoff_speed_m_s)
            or velocity_coast_handoff_speed_m_s <= 0.0
        ):
            raise ValueError(
                'control_handoff.coast_velocity_handoff_speed_m_s must be '
                'finite and positive'
            )
        velocity_coast_handoff_position_offset_m = float(
            config['control_handoff'].get(
                'coast_velocity_handoff_position_offset_m', 0.0
            )
        )
        if (
            not np.isfinite(velocity_coast_handoff_position_offset_m)
            or velocity_coast_handoff_position_offset_m < 0.0
        ):
            raise ValueError(
                'control_handoff.'
                'coast_velocity_handoff_position_offset_m must be finite and '
                'non-negative'
            )
        predictive_braking_config = deepcopy(
            config.get('predictive_braking', {})
        )
        if not isinstance(predictive_braking_config, dict):
            raise ValueError('predictive_braking must be a mapping')
        predictive_braking_enabled = predictive_braking_config.pop(
            'enabled', True
        )
        if type(predictive_braking_enabled) is not bool:
            raise ValueError('predictive_braking.enabled must be boolean')
        velocity_mpc_shadow_config = deepcopy(
            config.get('learning_velocity_mpc_shadow', {})
        )
        if not isinstance(velocity_mpc_shadow_config, dict):
            raise ValueError('learning_velocity_mpc_shadow must be a mapping')
        velocity_mpc_shadow_enabled = velocity_mpc_shadow_config.pop(
            'enabled', False
        )
        if type(velocity_mpc_shadow_enabled) is not bool:
            raise ValueError(
                'learning_velocity_mpc_shadow.enabled must be boolean'
            )
        velocity_mpc_command_authority = velocity_mpc_shadow_config.pop(
            'command_authority', False
        )
        if type(velocity_mpc_command_authority) is not bool:
            raise ValueError(
                'learning_velocity_mpc_shadow.command_authority must be boolean'
            )
        if velocity_mpc_command_authority and not velocity_mpc_shadow_enabled:
            raise ValueError(
                'learning velocity MPC command authority requires enabled=true'
            )
        velocity_mpc_online_enabled = bool(
            velocity_mpc_shadow_enabled and velocity_mpc_command_authority
        )
        velocity_mpc_shadow_target_m_s = float(
            velocity_mpc_shadow_config.pop('target_velocity_m_s', 0.0)
        )
        velocity_mpc_shadow_log_interval_s = float(
            velocity_mpc_shadow_config.pop('log_interval_s', 0.10)
        )
        velocity_mpc_max_decision_time_s = float(
            velocity_mpc_shadow_config.pop('max_decision_time_s', 0.008)
        )
        velocity_mpc_shadow_direction_config = (
            velocity_mpc_shadow_config.pop('direction_xy', None)
        )
        velocity_mpc_controller_config = velocity_mpc_shadow_config.pop(
            'controller', {}
        )
        if velocity_mpc_shadow_config:
            raise ValueError(
                'unknown learning_velocity_mpc_shadow keys: '
                + ', '.join(sorted(velocity_mpc_shadow_config))
            )
        if (
            not np.isfinite(velocity_mpc_shadow_target_m_s)
            or not np.isfinite(velocity_mpc_shadow_log_interval_s)
            or velocity_mpc_shadow_log_interval_s <= 0.0
            or not np.isfinite(velocity_mpc_max_decision_time_s)
            or not 0.001 <= velocity_mpc_max_decision_time_s <= 0.020
        ):
            raise ValueError(
                'learning velocity MPC target/log interval/runtime budget is '
                'invalid; decision budget must be in [0.001, 0.020] s'
            )
        if velocity_mpc_online_enabled:
            if abs(velocity_mpc_shadow_target_m_s) > 1e-9:
                raise ValueError(
                    'online learning velocity MPC currently supports only the '
                    'zero-velocity target before position handoff'
                )
            if predictive_braking_enabled:
                raise ValueError(
                    'online learning velocity MPC and predictive_braking '
                    'cannot both own release control'
                )
            if pipeline.shadow_mode:
                raise ValueError(
                    'online learning velocity MPC requires shadow_mode=false'
                )
        if not isinstance(velocity_mpc_controller_config, dict):
            raise ValueError(
                'learning_velocity_mpc_shadow.controller must be a mapping'
            )
        velocity_mpc_config_object = VelocityMPCConfig(
            **velocity_mpc_controller_config
        )
        velocity_mpc_config_object.validate()
        if (
            velocity_mpc_online_enabled
            and velocity_mpc_config_object.include_selected_trace
        ):
            raise ValueError(
                'online learning velocity MPC must disable selected traces to '
                'protect the control-loop compute budget'
            )
        if velocity_mpc_shadow_direction_config is not None:
            configured_velocity_direction = np.asarray(
                velocity_mpc_shadow_direction_config, dtype=float
            )
            if (
                configured_velocity_direction.shape != (2,)
                or not np.all(np.isfinite(configured_velocity_direction))
                or abs(configured_velocity_direction[0]) > 1e-9
                or abs(abs(configured_velocity_direction[1])-1.0) > 1e-9
            ):
                raise ValueError(
                    'learning velocity MPC fitted model direction must be '
                    'world [0,+/-1]'
                )
            velocity_mpc_shadow_direction_config = (
                configured_velocity_direction
            )
        release_dataset_configured_task_axis_xy = (
            None
            if velocity_mpc_shadow_direction_config is None else
            velocity_mpc_shadow_direction_config.copy()
        )
        prediction_model_consumer_enabled = bool(
            velocity_mpc_shadow_enabled
            or (
                predictive_braking_enabled
                and not velocity_coast_braking_enabled
            )
        )
        predictive_calibration_entry = (
            load_drone_calibration(self.drone_id, calibration_path)
            if (
                prediction_model_consumer_enabled
                and not calibration_mode
                and release_mode == 'potentiometer_coast'
                and not pipeline.shadow_mode
            ) else None
        )
        predictive_braking_model = (
            predictive_calibration_entry.get('prediction_model')
            if isinstance(predictive_calibration_entry, dict) else None
        )
        predictive_braking_available = bool(
            predictive_braking_model is not None
        )
        velocity_mpc_shadow_available = bool(
            velocity_mpc_shadow_enabled and predictive_braking_available
        )
        if velocity_mpc_shadow_enabled and not velocity_mpc_shadow_available:
            logger.warning(
                'Learning velocity MPC requested without a saved '
                'prediction_model; using the legacy coast controller.'
            )
        if (
            predictive_braking_enabled
            and not velocity_coast_braking_enabled
            and not calibration_mode
            and release_mode == 'potentiometer_coast'
            and not pipeline.shadow_mode
            and not predictive_braking_available
        ):
            logger.warning(
                'No saved prediction_model is available; potentiometer '
                'release will use the legacy coast controller.'
            )
        if configured_contact_detection_source not in (
                'wrench_observer', 'potentiometer'):
            raise ValueError(
                'virtual_object.contact_detection.source must be '
                'wrench_observer or potentiometer'
            )
        contact_detection_source = (
            'wrench_observer'
            if calibration_mode else configured_contact_detection_source
        )
        if (
            contact_detection_source == 'potentiometer'
            and not force_sensor_available
        ):
            raise ValueError(
                'potentiometer contact detection requires --sense and a '
                'fresh Arduino force sensor'
            )
        if (
            contact_detection_source == 'potentiometer'
            and release_mode != 'potentiometer_coast'
        ):
            raise ValueError(
                'potentiometer contact detection requires '
                'release_behavior.mode: potentiometer_coast'
            )
        if (
            contact_detection_source == 'potentiometer'
            and release_unloaded_force_n >= potentiometer_contact_force_n
        ):
            raise ValueError(
                'release_behavior.unloaded_force_n must be below '
                'contact_detection.force_threshold_n'
            )
        if (
            not np.isfinite(release_force_memory_s)
            or release_force_memory_s < 0.0
        ):
            raise ValueError('release force_memory_s must be finite and non-negative')
        if (
            not np.isfinite(release_candidate_sensor_stale_timeout_s)
            or release_candidate_sensor_stale_timeout_s <= 0.0
        ):
            raise ValueError(
                'release candidate_sensor_stale_timeout_s must be finite '
                'and positive'
            )
        potentiometer_release_detector = (
            PotentiometerReleaseDetector(
                force_drop_n=release_force_drop_n,
                decrease_rate_n_s=release_decrease_rate_n_s,
                unloaded_force_n=release_unloaded_force_n,
                unloaded_dwell_s=release_unloaded_dwell_s,
                max_sample_gap_s=release_max_sample_gap_s,
                candidate_stall_timeout_s=(
                    release_candidate_stall_timeout_s
                ),
                candidate_lead_drop_n=release_candidate_lead_drop_n,
            )
            if release_mode == 'potentiometer_coast' else None
        )
        potentiometer_contact_detector = (
            PotentiometerContactDetector(
                force_threshold_n=potentiometer_contact_force_n,
                onset_dwell_s=potentiometer_contact_dwell_s,
            )
            if contact_detection_source == 'potentiometer' else None
        )
        resistance_parameters = np.asarray([
            force_kinetic_friction_coefficient,
            force_static_friction_coefficient,
            force_drag_coefficient,
            force_frontal_area,
            force_air_density,
            force_friction_min_speed_m_s,
            render_acceleration_tolerance_m_s2,
            virtual_max_velocity_m_s,
        ])
        if (
            not np.all(np.isfinite(resistance_parameters))
            or np.any(resistance_parameters < 0.0)
        ):
            raise ValueError(
                'virtual-object resistance/render parameters must be finite '
                'and non-negative'
            )
        if virtual_max_velocity_m_s <= 0.0:
            raise ValueError('virtual max velocity must be positive')
        if (
            force_rendering_enabled
            and preferred_render_mode == 'orientation'
            and not np.isclose(
                force_current_mass, float(config['mass'])
            )
        ):
            raise ValueError(
                'virtual_object.current_mass must match wrench_interaction.mass '
                'for force-based orientation inertia'
            )
        if (
            release_mode == 'potentiometer_coast'
            and not pipeline.shadow_mode
            and force_rendering_enabled
            and preferred_render_mode != 'orientation'
        ):
            raise ValueError(
                'active potentiometer_coast with force rendering requires '
                'virtual_object.inertia_command: orientation so the calibrated '
                'attitude-command response remains observable'
            )

        max_state_age_s = float(safety.get(
            'max_state_age_s', safety['max_frame_age_s']
        ))
        enforce_state_group_skew = safety.get(
            'enforce_state_group_skew', False
        )
        if not isinstance(enforce_state_group_skew, bool):
            raise ValueError(
                'safety.enforce_state_group_skew must be a boolean'
            )
        calibration_state_dropout_timeout_s = float(safety.get(
            'calibration_state_dropout_timeout_s', 0.25
        ))
        if (
            calibration_mode
            and calibration_state_dropout_timeout_s <= max_state_age_s
        ):
            raise ValueError(
                'safety.calibration_state_dropout_timeout_s must be greater '
                'than safety.max_state_age_s'
            )
        max_state_group_skew_s = float(safety.get(
            'max_state_group_skew_s', safety['max_motor_pose_skew_s']
        ))
        calibration_state_group_skew_timeout_s = float(safety.get(
            'calibration_state_group_skew_timeout_s',
            calibration_state_dropout_timeout_s,
        ))
        calibration_max_protocol_clock_lag_s = float(safety.get(
            'calibration_max_protocol_clock_lag_s', 10.0
        ))
        state_sync_limits = np.asarray([
            max_state_age_s,
            calibration_state_dropout_timeout_s,
            max_state_group_skew_s,
            calibration_state_group_skew_timeout_s,
            calibration_max_protocol_clock_lag_s,
        ], dtype=float)
        if not np.all(np.isfinite(state_sync_limits)) or np.any(
                state_sync_limits <= 0.0):
            raise ValueError(
                'onboard state age/skew safety limits must be finite and '
                'positive'
            )
        if (
            calibration_mode
            and calibration_state_group_skew_timeout_s
            <= max_state_group_skew_s
        ):
            raise ValueError(
                'safety.calibration_state_group_skew_timeout_s must be '
                'greater than safety.max_state_group_skew_s'
            )
        max_motor_state_skew_s = float(safety.get(
            'max_motor_state_skew_s', safety['max_motor_pose_skew_s']
        ))
        initial_contact_arming_config = dict(
            config.get('initial_contact_arming', {})
        )
        initial_contact_gate = InitialContactArmingGate(
            **initial_contact_arming_config
        )
        translation_detector_requested = bool(
            pipeline.detector.translation.enabled
        )

        if config.get('shadow_mode', True):
            logger.warning(
                'Onboard wrench interaction is in shadow mode: contacts and '
                'proposed responses are logged, but the reference remains fixed.'
            )
        if not enforce_state_group_skew:
            logger.warning(
                'Onboard state-group skew enforcement is disabled; skew is '
                'still logged, but it will not stop interaction or calibration.'
            )
        self.log_manager.add_log_entry(
            'configs',
            {
                'pipeline': 'onboard_momentum_wrench_admittance_pid',
                'detection_method': 'momentum_impulse',
                'state_source': 'crazyflie_state_estimate',
                'orientation_feedback_source': (
                    'estimated_external_force'
                    if (
                        force_rendering_enabled
                        and preferred_render_mode == 'orientation'
                    ) else 'none'
                ),
                'virtual_object': {
                        'current_mass': force_current_mass,
                        'mass': force_virtual_mass,
                        'inertia_command': preferred_render_mode,
                        'render_policy': (
                            'position_when_faster_otherwise_priority'
                        ),
                        'render_acceleration_tolerance_m_s2': (
                            render_acceleration_tolerance_m_s2
                        ),
                        'max_attitude_deg': force_max_attitude_deg,
                        'kinetic_friction_coefficient': (
                            force_kinetic_friction_coefficient
                        ),
                        'static_friction_coefficient': (
                            force_static_friction_coefficient
                        ),
                        'two_afc_friction': {
                            **two_afc_friction.summary(),
                            'configured_enabled': (
                                two_afc_configured_enabled
                            ),
                            'disabled_during_calibration': bool(
                                calibration_mode
                                and two_afc_configured_enabled
                            ),
                        },
                        'drag_coefficient': force_drag_coefficient,
                        'frontal_area': force_frontal_area,
                        'air_density': force_air_density,
                        'friction_min_speed_m_s': (
                            force_friction_min_speed_m_s
                        ),
                        'force_rendering': {
                            'enabled': force_rendering_enabled,
                        },
                        'contact_detection': {
                            'source': contact_detection_source,
                            'configured_source': (
                                configured_contact_detection_source
                            ),
                            'ignored_during_calibration': bool(
                                calibration_mode
                            ),
                            'force_threshold_n': (
                                potentiometer_contact_force_n
                            ),
                            'onset_dwell_s': (
                                potentiometer_contact_dwell_s
                            ),
                        },
                        'release_behavior': {
                            'mode': release_mode,
                            'configured_mode': configured_release_mode,
                            'coast_control_policy': (
                                (
                                    (
                                        (
                                            'direct_level_attitude_unwind_then_position'
                                            if velocity_unwind_direct_level_attitude_enabled
                                            else 'release_line_position_unwind'
                                            if velocity_unwind_position_control_enabled
                                            else 'predictive_velocity_unwind_then_position'
                                        )
                                        if velocity_predictive_unwind_enabled
                                        else 'zero_world_velocity_then_position'
                                    )
                                    if velocity_coast_braking_enabled
                                    else (
                                        'predictive_model_brake_to_position'
                                        if predictive_braking_available
                                        else 'target_aware_no_pullback'
                                    )
                                )
                                if release_mode == 'potentiometer_coast' else None
                            ),
                            'velocity_handoff_speed_m_s': (
                                velocity_coast_handoff_speed_m_s
                                if velocity_coast_braking_enabled else None
                            ),
                            'velocity_handoff_position_offset_m': (
                                velocity_coast_handoff_position_offset_m
                                if velocity_coast_braking_enabled else None
                            ),
                            'velocity_predictive_unwind_enabled': (
                                velocity_predictive_unwind_enabled
                            ),
                            'velocity_unwind_prediction_margin_s': (
                                config['control_handoff'].get(
                                    'coast_velocity_unwind_prediction_margin_s',
                                    0.15,
                                )
                            ),
                            'velocity_unwind_command_switch_delay_s': (
                                config['control_handoff'].get(
                                    'coast_velocity_unwind_command_switch_delay_s',
                                    0.0,
                                )
                            ),
                            'velocity_unwind_integrated_leveling_enabled': (
                                config['control_handoff'].get(
                                    'coast_velocity_unwind_integrated_leveling_enabled',
                                    False,
                                )
                            ),
                            'velocity_unwind_tail_calibration_scale': (
                                config['control_handoff'].get(
                                    'coast_velocity_unwind_tail_calibration_scale',
                                    1.0,
                                )
                            ),
                            'velocity_unwind_direct_level_attitude_enabled': (
                                velocity_unwind_direct_level_attitude_enabled
                            ),
                            'velocity_unwind_position_control_enabled': (
                                velocity_unwind_position_control_enabled
                            ),
                            'velocity_unwind_leveling_rate_deg_s': (
                                config['control_handoff'].get(
                                    'coast_velocity_unwind_leveling_rate_deg_s',
                                    100.0,
                                )
                            ),
                            'velocity_unwind_integration_step_s': (
                                config['control_handoff'].get(
                                    'coast_velocity_unwind_integration_step_s',
                                    0.01,
                                )
                            ),
                            'velocity_unwind_one_step_lookahead_enabled': (
                                config['control_handoff'].get(
                                    'coast_velocity_unwind_one_step_lookahead_enabled',
                                    False,
                                )
                            ),
                            'coast_state_kinematic_guard_enabled': (
                                config['control_handoff'].get(
                                    'coast_state_kinematic_guard_enabled',
                                    False,
                                )
                            ),
                            'velocity_rebrake_speed_m_s': (
                                config['control_handoff'].get(
                                    'coast_velocity_rebrake_speed_m_s', 0.04
                                )
                            ),
                            'velocity_rebrake_enabled': (
                                velocity_rebrake_enabled
                            ),
                            'ignored_during_calibration': bool(
                                calibration_mode
                            ),
                            'force_drop_n': release_force_drop_n,
                            'decrease_rate_n_s': (
                                release_decrease_rate_n_s
                            ),
                            'unloaded_force_n': release_unloaded_force_n,
                            'unloaded_dwell_s': release_unloaded_dwell_s,
                            'max_sample_gap_s': release_max_sample_gap_s,
                            'candidate_stall_timeout_s': (
                                release_candidate_stall_timeout_s
                            ),
                            'candidate_sensor_stale_timeout_s': (
                                release_candidate_sensor_stale_timeout_s
                            ),
                            'candidate_lead_drop_n': (
                                release_candidate_lead_drop_n
                                if release_candidate_lead_drop_n is not None
                                else release_force_drop_n
                            ),
                            'force_memory_s': release_force_memory_s,
                            'force_memory_usage': (
                                'comparison_only_virtual_model'
                                if release_mode == 'potentiometer_coast'
                                else None
                            ),
                        },
                    },
                'translation_response_axes': ['x', 'y', 'z'],
                'force_sensor_comparison': {
                    **self._force_sensor_config(),
                    'control_source': 'wrench_observer',
                    'contact_detection_source': contact_detection_source,
                    'release_braking_force_source': (
                        'measured_xy_velocity'
                        if release_mode == 'potentiometer_coast'
                        else self._force_sensor_config()[
                            'release_braking_force_source'
                        ]
                    ),
                    'controls_translation': (
                        contact_detection_source == 'potentiometer'
                        or release_mode == 'potentiometer_coast'
                    ),
                    'used_for_release_detection': (
                        release_mode == 'potentiometer_coast'
                    ),
                    'observer_recorded_for_comparison': (
                        getattr(self, 'force_sensor', None) is not None
                    ),
                },
                'rotation_response_axes': (
                    ['yaw'] if config['detection']['yaw'].get('enabled', True)
                    else []
                ),
                'nominal_position': nominal_position.tolist(),
                'nominal_yaw_deg': nominal_yaw_deg,
                'translation_rearm_delay_s': float(rearm_delay_s),
                'initial_contact_arming': {
                    'enabled': initial_contact_gate.enabled,
                    'apply_after_each_interaction': (
                        initial_contact_gate.apply_after_each_interaction
                    ),
                    'max_xy_speed_m_s': (
                        initial_contact_gate.max_xy_speed_m_s
                    ),
                    'stationary_dwell_s': (
                        initial_contact_gate.stationary_dwell_s
                    ),
                },
                'config': config,
            },
            name='Onboard Wrench Interaction Config',
        )
        if bootstrap_coverage is not None:
            self._log_event('Learning MPC Bootstrap Calibration Started', {
                **bootstrap_coverage.summary(),
                'protocol': bootstrap_coverage.config.to_dict(),
                'instruction': (
                    'Keep the workspace clear. The drone will automatically '
                    'accelerate along world +/-Y to each target speed, then '
                    'the legacy attitude-coast controller will brake to rest.'
                ),
                'offline_only': True,
                'actual_flight_controller': (
                    'automatic_attitude_acceleration_then_legacy_attitude_coast'
                ),
                'lmpc_command_authority': False,
                'velocity_hover_disabled': True,
                'prediction_step_s': (
                    bootstrap_coverage.config.prediction_step_s
                ),
                'legacy_coast_command_delay_s': float(
                    config['control_handoff'][
                        'coast_attitude_response_delay_s'
                    ]
                ),
                'directional_model_contracts': {
                    label: {
                        'direction_xy': contract['direction_xy'],
                        'command_delay_s': contract['command_delay_s'],
                        'model_fingerprint': contract['model_fingerprint'],
                        'state_dimension': contract['state_dimension'],
                    }
                    for label, contract in bootstrap_model_contracts.items()
                },
                'coverage_is_provisional_until_offline_replay': True,
                'state_source': 'crazyflie_state_estimate',
            })
            logger.warning(
                'LMPC bootstrap target: %.2f m/s; collect %d successful '
                '+Y and -Y release(s) before moving to the next speed tier.',
                bootstrap_coverage.current_target_speed_m_s,
                bootstrap_coverage.config.repetitions_per_cell,
            )

        self._translation_exit_target = (nominal_position.tolist(), nominal_yaw_deg)
        self._translation_high_level_active = False
        self.hl_commander.go_to(
            nominal_position[0], nominal_position[1], nominal_position[2],
            nominal_yaw_deg, 2.0, relative=False,
        )
        self._safe_sleep(2.0)

        startup_deadline = time.time() + float(safety['startup_timeout_s'])
        while True:
            state = self._get_synchronized_onboard_wrench_state()
            now = time.time()
            if state is not None:
                state_age = now - state['time']
                state_skew = max(
                    float(state['position_skew_s']),
                    float(state['angular_rate_skew_s']),
                    float(state['yaw_control_skew_s']),
                )
                if (
                    -0.5 <= state_age <= max_state_age_s
                    and (
                        not enforce_state_group_skew
                        or state_skew <= max_state_group_skew_s
                    )
                ):
                    break
            if now >= startup_deadline:
                raise StaleLocalizationError(
                    'No fresh synchronized onboard state received from '
                    'VEL_ORI, POS_ACC, RATE_EST, YAW_CTL, and MOT_BAT'
                )
            self.lo_commander.send_position_setpoint(
                *nominal_position, nominal_yaw_deg
            )
            self._safe_sleep(dt)

        bias_calibration_enabled = bool(
            config['startup_bias_calibration_enabled']
        )
        if bias_calibration_enabled:
            self._log_event('Wrench Calibration Started', {
                'instruction': 'Do not touch the drone until calibration completes.',
                'shadow_mode': pipeline.shadow_mode,
                'state_source': 'crazyflie_state_estimate',
            })
            logger.info(
                'Calibrating onboard momentum observer; do not touch the drone.'
            )

        last_state_time = None
        interaction_start = None
        calibration_announced = False
        last_command_position = nominal_position.copy()
        last_command_yaw = nominal_yaw_deg
        translation_control = TranslationControlHandoff(
            nominal_position,
            nominal_yaw_deg,
            pipeline.shadow_mode,
            rearm_delay_s=rearm_delay_s,
            **config['control_handoff'],
        )
        virtual_motion = VirtualObjectPlanarMotion(
            mass=force_virtual_mass,
            max_velocity_m_s=virtual_max_velocity_m_s,
            max_offset_xy=config['admittance']['max_offset'][:2],
            kinetic_friction_coefficient=(
                force_kinetic_friction_coefficient
            ),
            static_friction_coefficient=(
                force_static_friction_coefficient
            ),
            drag_coefficient=force_drag_coefficient,
            frontal_area=force_frontal_area,
            air_density=force_air_density,
            friction_min_speed_m_s=force_friction_min_speed_m_s,
        )
        active_two_afc_condition = None

        def begin_two_afc_interaction():
            nonlocal force_kinetic_friction_coefficient
            nonlocal force_static_friction_coefficient
            nonlocal active_two_afc_condition
            condition = two_afc_friction.begin_interaction()
            if condition is None:
                return None
            mu = float(condition['mu'])
            force_kinetic_friction_coefficient = mu
            force_static_friction_coefficient = mu
            virtual_motion.set_friction_coefficients(mu, mu)
            active_two_afc_condition = condition
            self._log_event('2AFC Friction Condition Started', {
                **condition,
                'kinetic_friction_coefficient': mu,
                'static_friction_coefficient': mu,
                'state_source': 'crazyflie_state_estimate',
            })
            return condition

        def current_interaction_log_details():
            details = (
                f'current_mass={force_current_mass:.3f} kg, '
                f'virtual_mass={force_virtual_mass:.3f} kg, '
                f'kinetic_mu={force_kinetic_friction_coefficient:.3f}, '
                f'static_mu={force_static_friction_coefficient:.3f}'
            )
            if active_two_afc_condition is not None:
                details += (
                    f", 2AFC={active_two_afc_condition['condition']}"
                )
            return details
        potentiometer_release_decision = None
        potentiometer_release_processed = False
        potentiometer_release_pending = False
        candidate_release_force_world = None
        candidate_release_direction = None
        candidate_release_attitude_deg = None
        candidate_release_sensor_stale_logged = False
        candidate_release_sensor_stale_since = None
        potentiometer_contact_decision = None
        coast_initial_velocity = None
        selected_render_mode = None
        render_relation = None
        render_selection = None
        virtual_motion_state = None
        coast_stop_prediction = None
        predictive_brake_episode = None
        predictive_brake_decision = None
        predictive_brake_abort_after_send = False
        predictive_position_handoff_logged = False
        predictive_last_logged_signature = None
        velocity_mpc_shadow_episode = None
        velocity_mpc_shadow_direction = None
        velocity_mpc_shadow_previous_state = None
        velocity_mpc_shadow_last_decision = None
        velocity_mpc_shadow_last_logged_signature = None
        velocity_mpc_shadow_last_log_time = None
        velocity_mpc_terminal_since = None
        # Raw release episodes are logged for post-flight LMPC admission.  The
        # in-flight loop never mutates or publishes a sampled safe set.
        release_dataset_episode_sequence = 0
        release_dataset_episode_id = None
        release_dataset_direction_xy = None
        release_dataset_model_contract = None
        release_dataset_measured_sensor_axis_world_xy = None
        release_dataset_last_logged_command_sequence = 0
        release_dataset_terminal_gate = ReleaseLMPCTerminalGate()
        release_dataset_terminal_status = release_dataset_terminal_gate.status
        release_dataset_terminal_finalize_pending = None
        mpc_last_decision_send_monotonic = None
        mpc_next_decision_deadline_monotonic = None
        mpc_automatic_attempt = None
        mpc_automatic_retry_counts = {}
        mpc_automatic_release_confirmed = False
        mpc_automatic_abort_after_safe_stop = None
        mpc_automatic_last_phase = None
        mpc_automatic_previous_cell = None

        def close_bootstrap_episode(episode_id, *, terminal_success, reason):
            if bootstrap_coverage is None or episode_id is None:
                return None
            result = bootstrap_coverage.close(
                episode_id,
                terminal_success=terminal_success,
                reason=reason,
            )
            self._log_event('Learning MPC Bootstrap Attempt Closed', {
                **result,
                'coverage': bootstrap_coverage.summary(),
                'offline_only': True,
                'lmpc_command_authority': False,
                'state_source': 'crazyflie_state_estimate',
            })
            next_speed = bootstrap_coverage.current_target_speed_m_s
            if bootstrap_coverage.complete:
                logger.info(
                    'LMPC bootstrap raw speed-cell coverage is complete. '
                    'The raw log still requires offline resampling and replay.'
                )
            elif result['counted']:
                logger.info(
                    'LMPC bootstrap attempt counted; next target remains '
                    '%.2f m/s until both directions/repetitions are complete.',
                    next_speed,
                )
            else:
                rejection_reason = (
                    result['close_reason']
                    if not result['terminal_success'] else
                    (
                        result['path_failure_reasons'][0]
                        if result['path_failure_reasons'] else
                        result['reason']
                    )
                )
                logger.warning(
                    'LMPC bootstrap attempt did not count (%s); retry the '
                    'current %.2f m/s cell.',
                    rejection_reason,
                    next_speed,
                )
            return result

        def finish_mpc_automatic_attempt(*, counted, reason):
            """Return to nominal only after one automatic attempt is safe."""
            nonlocal mpc_automatic_attempt
            nonlocal mpc_automatic_release_confirmed
            nonlocal mpc_automatic_abort_after_safe_stop
            nonlocal mpc_automatic_last_phase
            nonlocal mpc_automatic_previous_cell
            nonlocal mpc_last_decision_send_monotonic
            nonlocal mpc_next_decision_deadline_monotonic
            if mpc_automatic_attempt is None:
                return
            cell = mpc_automatic_attempt.target_cell
            mpc_automatic_previous_cell = cell
            cell_key = (cell.direction_sign, cell.target_speed_m_s)
            if counted:
                mpc_automatic_retry_counts.pop(cell_key, None)
            else:
                attempts = mpc_automatic_retry_counts.get(cell_key, 0)+1
                mpc_automatic_retry_counts[cell_key] = attempts
                if attempts >= 3:
                    mpc_automatic_abort_after_safe_stop = (
                        'automatic_cell_failed_three_times:' + str(reason)
                    )
            if mpc_automatic_attempt.phase in (
                    mpc_automatic_attempt.BRAKING,
                    mpc_automatic_attempt.ABORTED,
            ):
                mpc_automatic_attempt.finish()
            return_target_m = (
                translation_control.hold_position.copy()
                if mpc_automatic_abort_after_safe_stop is not None else
                nominal_position.copy()
            )
            self._log_event(
                'Learning MPC Bootstrap Automatic Attempt Finished',
                {
                    **cell.to_dict(),
                    'counted': bool(counted),
                    'reason': str(reason),
                    'retry_count_for_cell': mpc_automatic_retry_counts.get(
                        cell_key, 0
                    ),
                    'return_target_m': return_target_m.tolist(),
                    'offline_only': True,
                    'lmpc_command_authority': False,
                    'state_source': 'crazyflie_state_estimate',
                },
            )
            mpc_automatic_attempt = None
            mpc_automatic_release_confirmed = False
            mpc_automatic_last_phase = None
            mpc_last_decision_send_monotonic = None
            mpc_next_decision_deadline_monotonic = None
            if mpc_automatic_abort_after_safe_stop is None:
                translation_control.hold_position = nominal_position.copy()
                translation_control.yaw_deg = nominal_yaw_deg

        def level_mpc_attitude_before_fault(reason, yaw_deg=None):
            """Synchronously replace a latched automatic/coast tilt."""
            if translation_control.mode not in (
                    translation_control.MPC_BOOTSTRAP_ACCELERATION,
                    translation_control.ATTITUDE_COAST,
            ):
                return False
            translation_control.set_contact_attitude(
                0.0, 0.0, 0.0, yaw_deg=yaw_deg
            )
            sent_at = translation_control.send(
                self.lo_commander, yaw_deg=yaw_deg
            )
            self._log_event(
                'Learning MPC Bootstrap Fault Level Command Sent',
                {
                    'reason': str(reason),
                    'command_sent_at': sent_at,
                    'previous_phase': (
                        None
                        if mpc_automatic_attempt is None else
                        mpc_automatic_attempt.phase
                    ),
                    'offline_only': True,
                    'lmpc_command_authority': False,
                    'state_source': 'crazyflie_state_estimate',
                },
            )
            return True

        excitation_config = config['calibration_excitation']
        excitation_end_s = (
            float(excitation_config['start_delay_s'])
            + float(excitation_config['duration_s'])
        )
        if braking_test_mode:
            excitation_end_s = 0.0
        planar_braking_config = config['planar_braking_calibration']
        planar_braking_plan = PlanarBrakingCalibration(
            planar_braking_config,
            start_after_s=excitation_end_s,
            require_opposed_directions=not braking_test_mode,
        )
        repeat_reference = None
        if braking_test_mode:
            repeat_reference = calibration_reference(calibration_path)
            self._log_event('Planar Braking Repeat Test Started', {
                'protocol': repeat_test_protocol(planar_braking_plan, planar_braking_config),
                'calibration_reference': repeat_reference,
                'offline_only': True,
                'nominal_position_m': nominal_position.tolist(),
                'nominal_yaw_deg': nominal_yaw_deg,
                'control_rate_hz': self.ctrl_rate,
            })
            logger.info('BRAKING REPEAT TEST: %d trials at 20 deg / 0.24s; '
                        'directions=%s, repeats per direction=%d; '
                        'no XYZ excitation and no calibration file updates.',
                        len(planar_braking_plan.trial_directions),
                        planar_braking_plan.directions.tolist(),
                        planar_braking_plan.repetitions_per_duration)
        # Keep historical report support, but remove this experiment from all
        # live calibration paths, including callers with an older mission.
        position_capture_config = {}
        position_capture_plan = PositionCaptureCalibration(
            position_capture_config,
            start_after_s=planar_braking_plan.duration_s,
        )
        if (
            calibration_mode
            and planar_braking_plan.enabled
            and duration < planar_braking_plan.duration_s
        ):
            raise ValueError(
                f'interaction duration {duration:.1f}s is shorter than the '
                'complete planar braking calibration '
                f'({planar_braking_plan.duration_s:.1f}s)'
            )
        if (
            calibration_mode
            and position_capture_plan.enabled
            and duration < position_capture_plan.duration_s
        ):
            raise ValueError(
                f'interaction duration {duration:.1f}s is shorter than the '
                'complete position capture calibration '
                f'({position_capture_plan.duration_s:.1f}s)'
            )
        guided_touch = GuidedTouchProtocol(config.get('guided_touch_test'))
        if guided_touch.enabled and duration < guided_touch.required_duration_s:
            raise ValueError(
                f'interaction duration {duration:.1f}s is shorter than the '
                f'guided touch sequence ({guided_touch.required_duration_s:.1f}s)'
            )
        excitation_started = False
        excitation_finished = False
        model_calibration_samples = []
        planar_braking_samples = []
        adaptive_braking = AdaptiveBrakingCalibration(
            config.get('adaptive_braking_calibration', {}) if calibration_mode else {},
            planar_braking_plan, self.ctrl_rate, self._log_event,
        )
        if adaptive_braking.enabled and (prediction_calibration is None or braking_test_mode):
            raise ValueError('adaptive braking requires --calibrate with an online prediction worker')
        prediction_submitted_segments = set()
        prediction_finish_requested = False
        active_planar_braking_command = None
        active_planar_braking_command_since = None
        last_planar_braking_phase = None
        position_capture_samples = []
        active_position_capture_command = None
        active_position_capture_command_since = None
        last_position_capture_phase = None
        calibration_dropout_active = False
        calibration_dropout_max_age_s = 0.0
        calibration_group_skew_dropout_active = False
        calibration_group_skew_dropout_started_at = None
        calibration_group_skew_dropout_max_s = 0.0
        calibration_elapsed_s = 0.0
        # A gate owns only the interval before a new trial. Once an attitude
        # command has been sent, its fixed-duration protocol is never paused
        # for readiness. Both stages share the same paused protocol clock.
        calibration_trial_gates = []
        calibration_trial_boundaries = []
        if calibration_mode and planar_braking_plan.enabled:
            gate = CalibrationTrialReadinessGate(planar_braking_config)
            calibration_trial_gates.append(gate)
            for segment_id in range(len(planar_braking_plan.trial_directions)):
                calibration_trial_boundaries.append((
                    float(planar_braking_plan.trial_start_s[segment_id]),
                    'Planar Braking', segment_id, gate, planar_braking_plan,
                ))
        if calibration_mode and position_capture_plan.enabled:
            gate = CalibrationTrialReadinessGate(position_capture_config)
            calibration_trial_gates.append(gate)
            for trial in getattr(position_capture_plan, 'trials', []):
                calibration_trial_boundaries.append((
                    trial['start_s'] + position_capture_plan.settle_s,
                    'Position Capture', trial['segment_id'], gate,
                    position_capture_plan,
                ))
        calibration_trial_boundaries.sort(key=lambda trial: trial[0])
        calibration_trial_wait = None

        def begin_trial_wait(wall_time):
            for boundary in calibration_trial_boundaries:
                boundary_s, label, segment_id, gate, _plan = boundary
                if (calibration_elapsed_s + 1e-9 >= boundary_s
                        and not gate.admitted(segment_id)):
                    if not gate.waiting:
                        gate.begin(segment_id, wall_time)
                        self._log_event(label + ' Calibration Trial Wait Started', {
                            'segment_id': segment_id,
                            'protocol_elapsed_s': calibration_elapsed_s,
                            'hold_position_m': nominal_position.tolist(),
                            'state_source': 'crazyflie_state_estimate',
                        })
                        logger.info('%s calibration trial %s: holding nominal '
                                    'position until the start state is stable.',
                                    label, segment_id)
                    return boundary
            return None

        def check_trial_wait(wall_time, *, invalidate=False, duplicate=False):
            if calibration_trial_wait is None:
                return
            _, label, segment_id, gate, _plan = calibration_trial_wait
            try:
                if invalidate:
                    gate.invalidate(wall_time)
                elif duplicate:
                    gate.no_new_sample(wall_time)
                else:
                    gate.poll(wall_time)
            except TimeoutError as exc:
                self._log_event(label + ' Calibration Trial Wait Timeout', {
                    'segment_id': segment_id,
                    'protocol_elapsed_s': calibration_elapsed_s,
                    'wait_elapsed_s': gate.wait_elapsed_s(wall_time),
                    'reason': str(exc),
                    'state_source': 'crazyflie_state_estimate',
                })
                raise

        while True:
            now = time.time()
            if (
                bootstrap_coverage is not None
                and release_dataset_episode_id is None
                and (
                    bootstrap_coverage.complete
                    or (
                        mpc_automatic_abort_after_safe_stop is not None
                        and mpc_automatic_attempt is None
                        and translation_control.mode
                        == translation_control.POSITION_HOLD
                    )
                )
            ):
                break
            attitude_sent_at = None
            release_dataset_close_after_row = None
            mpc_scheduled_decision_deadline = None
            mpc_decision_state_age_at_send_s = None
            mpc_automatic_decision = None
            mpc_automatic_finish_after_handoff_send_reason = None
            calibration_wait_this_cycle = False
            if calibration_mode and interaction_start is not None:
                calibration_trial_wait = begin_trial_wait(now)
                check_trial_wait(now)
            planar_attitude_active = bool(
                calibration_mode
                and (
                    (active_planar_braking_command is not None
                     and active_planar_braking_command.attitude_control)
                    or (active_position_capture_command is not None
                        and active_position_capture_command.attitude_control)
                )
            )
            mpc_attitude_motion_active = bool(
                mpc_calibration_mode
                and translation_control.mode in (
                    translation_control.MPC_BOOTSTRAP_ACCELERATION,
                    translation_control.ATTITUDE_COAST,
                )
            )
            if (
                calibration_mode
                and interaction_start is not None
                and not planar_attitude_active
                and calibration_trial_wait is None
            ):
                # Also treat an attitude phase that became due during a
                # callback gap as unsafe.  The previous command may still be
                # latched, or the scheduled open-loop phase may otherwise be
                # entered without a synchronized velocity sample.
                planar_attitude_active = bool(
                    planar_braking_plan.command(
                        calibration_elapsed_s, 0.0
                    ).attitude_control
                    or position_capture_plan.attitude_phase_due(
                        calibration_elapsed_s
                    )
                )
            if interaction_start is not None:
                interaction_elapsed_s = (
                    calibration_elapsed_s
                    if calibration_mode
                    else now - interaction_start
                )
                if interaction_elapsed_s >= duration:
                    if mpc_attitude_motion_active:
                        level_mpc_attitude_before_fault(
                            'mission_duration_expired_during_automatic_motion'
                        )
                        raise RuntimeError(
                            'automatic LMPC bootstrap duration expired during '
                            'an active attitude maneuver'
                        )
                    break
            calibration_clock_lag_s = (
                now - interaction_start - calibration_elapsed_s
                - sum(gate.total_wait_s(now) for gate in calibration_trial_gates)
                if calibration_mode and interaction_start is not None else 0.0
            )
            if (
                calibration_mode
                and interaction_start is not None
                and calibration_clock_lag_s > calibration_max_protocol_clock_lag_s
            ):
                if planar_attitude_active:
                    self.lo_commander.send_zdistance_setpoint(
                        0.0, 0.0, 0.0, float(nominal_position[2])
                    )
                raise StaleLocalizationError(
                    'Calibration protocol clock is '
                    f'{calibration_clock_lag_s:.3f}s '
                    'behind wall time '
                    f'(limit {calibration_max_protocol_clock_lag_s:.3f}s)'
                )
            state = self._get_synchronized_onboard_wrench_state()
            if state is None:
                check_trial_wait(now, invalidate=True)
                if mpc_attitude_motion_active:
                    level_mpc_attitude_before_fault(
                        'onboard_state_packet_set_incomplete'
                    )
                if planar_attitude_active:
                    self.lo_commander.send_zdistance_setpoint(
                        0.0, 0.0, 0.0, float(nominal_position[2])
                    )
                raise StaleLocalizationError('Onboard state packet set is incomplete')
            # ``_get_synchronized_onboard_wrench_state`` reads data populated by
            # asynchronous Crazyflie callbacks.  The control thread can be
            # descheduled after the loop-entry timestamp above while callbacks
            # continue to publish newer packets.  Refresh wall time after the
            # snapshot so a fresh packet is never compared with a stale ``now``
            # and misclassified as hundreds of milliseconds into the future.
            now = time.time()
            state_observed_at = now
            state_time = state['time']
            state_age = now - state_time
            release_dataset_effective_command_delay_s = (
                float(release_dataset_model_contract['command_delay_s'])
                if release_dataset_model_contract is not None else
                translation_control.coast_attitude_response_delay_s
            )
            actual_command_applied_at_state = (
                translation_control.sent_command_effective_at(
                    state_time,
                    release_dataset_effective_command_delay_s,
                )
            )
            if state_age < -0.5:
                check_trial_wait(now, invalidate=True)
                if mpc_attitude_motion_active:
                    level_mpc_attitude_before_fault(
                        'onboard_state_timestamp_in_future'
                    )
                if planar_attitude_active:
                    self.lo_commander.send_zdistance_setpoint(
                        0.0, 0.0, 0.0, float(nominal_position[2])
                    )
                raise StaleLocalizationError(
                    f'Onboard state is {state_age:.3f}s old '
                    f'(limit {max_state_age_s:.3f}s)'
                )
            if state_age > max_state_age_s:
                check_trial_wait(now, invalidate=True)
                if mpc_attitude_motion_active:
                    level_mpc_attitude_before_fault(
                        'onboard_state_stale'
                    )
                    raise StaleLocalizationError(
                        'Onboard state became stale during automatic LMPC '
                        f'bootstrap motion ({state_age:.3f}s)'
                    )
                if planar_attitude_active:
                    # A stale velocity sample makes open-loop attitude
                    # calibration unsafe. Level immediately and abort so a
                    # prior tilt is never held through a telemetry dropout.
                    self.lo_commander.send_zdistance_setpoint(
                        0.0, 0.0, 0.0, float(nominal_position[2])
                    )
                    raise StaleLocalizationError(
                        'Onboard state became stale during planar attitude '
                        f'calibration ({state_age:.3f}s)'
                    )
                if calibration_state_dropout_tolerated(
                        state_age,
                        max_state_age_s,
                        calibration_state_dropout_timeout_s,
                        calibration_mode=calibration_mode):
                    calibration_dropout_max_age_s = max(
                        calibration_dropout_max_age_s, state_age
                    )
                    if not calibration_dropout_active:
                        calibration_dropout_active = True
                        self._log_event(
                            'Calibration State Dropout Started',
                            {
                                'state_age_s': state_age,
                                'normal_limit_s': max_state_age_s,
                                'dropout_timeout_s': (
                                    calibration_state_dropout_timeout_s
                                ),
                                'state_source': (
                                    'crazyflie_state_estimate'
                                ),
                            },
                        )
                        logger.warning(
                            'Skipping transient %.3fs-old calibration state; '
                            'holding position while waiting for a fresh packet.',
                            state_age,
                        )
                    translation_control.send(self.lo_commander)
                    self._safe_sleep(dt)
                    continue
                effective_limit = (
                    calibration_state_dropout_timeout_s
                    if calibration_mode else max_state_age_s
                )
                raise StaleLocalizationError(
                    f'Onboard state is {state_age:.3f}s old '
                    f'(limit {effective_limit:.3f}s)'
                )
            if calibration_dropout_active:
                self._log_event(
                    'Calibration State Dropout Recovered',
                    {
                        'recovered_state_age_s': state_age,
                        'maximum_stale_age_s': (
                            calibration_dropout_max_age_s
                        ),
                        'state_source': 'crazyflie_state_estimate',
                    },
                )
                logger.info(
                    'Calibration state stream recovered at %.3fs age.',
                    state_age,
                )
                calibration_dropout_active = False
                calibration_dropout_max_age_s = 0.0
            state_group_skews = {
                'position_skew_s': float(state['position_skew_s']),
                'angular_rate_skew_s': float(state['angular_rate_skew_s']),
                'yaw_control_skew_s': float(state['yaw_control_skew_s']),
            }
            state_group_skew_values = np.asarray(
                list(state_group_skews.values()), dtype=float
            )
            if (
                not np.all(np.isfinite(state_group_skew_values))
                or np.any(state_group_skew_values < 0.0)
            ):
                check_trial_wait(now, invalidate=True)
                if mpc_attitude_motion_active:
                    level_mpc_attitude_before_fault(
                        'invalid_onboard_state_group_skew'
                    )
                if planar_attitude_active:
                    self.lo_commander.send_zdistance_setpoint(
                        0.0, 0.0, 0.0, float(nominal_position[2])
                    )
                raise StaleLocalizationError(
                    'Onboard state-group skew contains an invalid value'
                )
            state_group_skew = float(np.max(state_group_skew_values))
            active_state_group_skew_limit_s = (
                release_dataset_terminal_gate.limits.max_state_group_skew_s
                if mpc_attitude_motion_active else max_state_group_skew_s
            )
            if (
                (enforce_state_group_skew or mpc_attitude_motion_active)
                and state_group_skew > active_state_group_skew_limit_s
            ):
                check_trial_wait(now, invalidate=True)
                if mpc_attitude_motion_active:
                    level_mpc_attitude_before_fault(
                        'onboard_state_groups_unsynchronized'
                    )
                    raise StaleLocalizationError(
                        'Onboard state groups became unsynchronized during '
                        'automatic LMPC bootstrap motion '
                        f'({state_group_skew:.3f}s; limit '
                        f'{active_state_group_skew_limit_s:.3f}s)'
                    )
                if planar_attitude_active:
                    # Never resume an open-loop tilt after losing synchronized
                    # state.  Level immediately and make the operator restart
                    # the calibration trial from a known state.
                    self.lo_commander.send_zdistance_setpoint(
                        0.0, 0.0, 0.0, float(nominal_position[2])
                    )
                    raise StaleLocalizationError(
                        'Onboard state groups became unsynchronized during '
                        'planar attitude calibration '
                        f'({state_group_skew:.3f}s)'
                    )
                if calibration_group_skew_dropout_started_at is None:
                    calibration_group_skew_dropout_started_at = now
                dropout_elapsed_s = max(
                    now - calibration_group_skew_dropout_started_at, 0.0
                )
                if calibration_state_group_skew_tolerated(
                        state_group_skew,
                        max_state_group_skew_s,
                        dropout_elapsed_s,
                        calibration_state_group_skew_timeout_s,
                        calibration_mode=calibration_mode,
                        planar_attitude_active=planar_attitude_active):
                    calibration_group_skew_dropout_max_s = max(
                        calibration_group_skew_dropout_max_s,
                        state_group_skew,
                    )
                    if not calibration_group_skew_dropout_active:
                        calibration_group_skew_dropout_active = True
                        self._log_event(
                            'Calibration State Group Skew Started',
                            {
                                **state_group_skews,
                                'maximum_group_skew_s': state_group_skew,
                                'normal_limit_s': max_state_group_skew_s,
                                'dropout_timeout_s': (
                                    calibration_state_group_skew_timeout_s
                                ),
                                'state_source': (
                                    'crazyflie_state_estimate'
                                ),
                            },
                        )
                        logger.warning(
                            'Skipping transient %.3fs calibration state-group '
                            'skew; holding position while groups resynchronize.',
                            state_group_skew,
                        )
                    translation_control.send(self.lo_commander)
                    self._safe_sleep(dt)
                    continue
                effective_limit = (
                    calibration_state_group_skew_timeout_s
                    if calibration_mode else max_state_group_skew_s
                )
                raise StaleLocalizationError(
                    f'Onboard state-group skew is {state_group_skew:.3f}s '
                    f'(limit {effective_limit:.3f}s; invalid for '
                    f'{dropout_elapsed_s:.3f}s)'
                )
            if calibration_group_skew_dropout_active:
                dropout_elapsed_s = max(
                    now - calibration_group_skew_dropout_started_at, 0.0
                )
                self._log_event(
                    'Calibration State Group Skew Recovered',
                    {
                        **state_group_skews,
                        'recovered_group_skew_s': state_group_skew,
                        'maximum_group_skew_s': (
                            calibration_group_skew_dropout_max_s
                        ),
                        'dropout_elapsed_s': dropout_elapsed_s,
                        'state_source': 'crazyflie_state_estimate',
                    },
                )
                logger.info(
                    'Calibration state groups resynchronized at %.3fs skew '
                    'after %.3fs.',
                    state_group_skew,
                    dropout_elapsed_s,
                )
                calibration_group_skew_dropout_active = False
                calibration_group_skew_dropout_started_at = None
                calibration_group_skew_dropout_max_s = 0.0
            if state_time == last_state_time:
                check_trial_wait(now, duplicate=True)
                if mpc_calibration_mode:
                    # A duplicated callback state is not a new LMPC decision
                    # epoch. Keep the previous LL setpoint latched so the raw
                    # actual-send timeline stays one command per fresh state.
                    pass
                elif predictive_brake_episode is not None:
                    # The predictive episode may only issue a new attitude
                    # decision from a new synchronized state. Do not resend a
                    # stale decision behind its back; the last Crazyflie
                    # setpoint remains latched until the next fresh sample.
                    pass
                elif (
                    calibration_mode
                    and active_position_capture_command is not None
                    and active_position_capture_command.attitude_control
                ):
                    self.lo_commander.send_zdistance_setpoint(
                        active_position_capture_command.roll_deg,
                        active_position_capture_command.pitch_deg,
                        0.0,
                        float(nominal_position[2]),
                    )
                elif (
                    calibration_mode
                    and active_planar_braking_command is not None
                    and active_planar_braking_command.attitude_control
                ):
                    self.lo_commander.send_zdistance_setpoint(
                        active_planar_braking_command.roll_deg,
                        active_planar_braking_command.pitch_deg,
                        0.0,
                        float(nominal_position[2]),
                    )
                else:
                    translation_control.expire_tail_neutralization(time.time())
                    translation_control.send(
                        self.lo_commander,
                        yaw_deg=np.degrees(state['attitude_rpy'][2]),
                    )
                self._safe_sleep(dt)
                continue
            protocol_state_step_s = dt
            if last_state_time is not None:
                protocol_state_step_s = float(state_time-last_state_time)
                if (
                    not np.isfinite(protocol_state_step_s)
                    or protocol_state_step_s <= 0.0
                ):
                    check_trial_wait(now, invalidate=True)
                    if mpc_attitude_motion_active:
                        level_mpc_attitude_before_fault(
                            'nonincreasing_onboard_state_time'
                        )
                    if planar_attitude_active:
                        self.lo_commander.send_zdistance_setpoint(
                            0.0, 0.0, 0.0, float(nominal_position[2])
                        )
                    raise StaleLocalizationError(
                        'Onboard state time did not increase during '
                        'calibration'
                    )
                # A large discontinuity is a telemetry/host pause, not
                # elapsed maneuver time. Active attitude already aborts on
                # stale or unsynchronized state above. A fresh packet after a
                # blocked host loop can hide that gap from the state-age test,
                # so reject it explicitly while attitude control is active.
                # Position-held phases instead resume with one nominal control
                # step rather than skipping a scheduled calibration phase.
                if protocol_state_step_s > max_state_age_s:
                    if mpc_attitude_motion_active:
                        level_mpc_attitude_before_fault(
                            'onboard_state_sample_gap'
                        )
                        raise StaleLocalizationError(
                            'Onboard state sample gap during automatic LMPC '
                            f'bootstrap motion was {protocol_state_step_s:.3f}s '
                            f'(limit {max_state_age_s:.3f}s)'
                        )
                    if planar_attitude_active:
                        self.lo_commander.send_zdistance_setpoint(
                            0.0, 0.0, 0.0, float(nominal_position[2])
                        )
                        raise StaleLocalizationError(
                            'Onboard state sample gap during planar attitude '
                            f'calibration was {protocol_state_step_s:.3f}s '
                            f'(limit {max_state_age_s:.3f}s)'
                        )
                    protocol_state_step_s = dt
            last_state_time = state_time

            position = state['position']
            try:
                self.check_interaction_boundary(position)
            except BoundaryExceededError:
                if mpc_attitude_motion_active:
                    level_mpc_attitude_before_fault(
                        'interaction_boundary_exceeded',
                        yaw_deg=np.degrees(state['attitude_rpy'][2]),
                    )
                raise
            # An interaction can finish far from nominal. Exit into a hold at
            # the latest valid position; do not introduce a new return motion.
            self._translation_exit_target = (position.tolist(), nominal_yaw_deg)
            motor_state = state['motor_state']
            motor_pwm = [motor_state.get(f'motor.m{i}') for i in range(1, 5)]
            if not OnboardMomentumWrenchPipeline.motor_data_available(motor_pwm):
                motor_pwm = None
            battery_voltage = motor_state.get('pm.vbat')
            battery_available = (
                isinstance(battery_voltage, (int, float))
                and np.isfinite(battery_voltage)
                and battery_voltage > 0
            )
            if not battery_available:
                battery_voltage = None
            motor_log_time = motor_state.get('time')
            motor_age = None if motor_log_time is None else now - motor_log_time
            motor_is_stale = (
                motor_age is None
                or motor_age < -0.5
                or motor_age > float(safety['max_motor_age_s'])
            )
            motor_is_unsynchronized = (
                enforce_state_group_skew
                and (
                    state['motor_skew_s'] is None
                    or state['motor_skew_s'] > max_motor_state_skew_s
                )
            )
            if safety['require_motor_data'] and (
                motor_pwm is None
                or battery_voltage is None
                or motor_is_stale
                or motor_is_unsynchronized
            ):
                if mpc_attitude_motion_active:
                    level_mpc_attitude_before_fault(
                        'motor_or_battery_state_unavailable',
                        yaw_deg=np.degrees(state['attitude_rpy'][2]),
                    )
                if planar_attitude_active:
                    self.lo_commander.send_zdistance_setpoint(
                        0.0, 0.0, 0.0, float(nominal_position[2])
                    )
                raise RuntimeError(
                    'Fresh, state-synchronized motor PWM and battery data are '
                    'required for onboard wrench estimation'
                )

            initial_contact_just_armed = False
            if calibration_announced:
                initial_contact_just_armed = initial_contact_gate.update(
                    state['velocity'], state_time
                )
            pipeline.detector.translation.enabled = bool(
                translation_detector_requested
                and initial_contact_gate.armed
                and contact_detection_source == 'wrench_observer'
            )

            try:
                output = pipeline.update(
                    position=position,
                    velocity=state['velocity'],
                    attitude_rpy=state['attitude_rpy'],
                    angular_velocity=state['angular_velocity'],
                    motor_pwm=motor_pwm,
                    battery_voltage=battery_voltage,
                    timestamp=state_time,
                    yaw_control_command=state['yaw_control_command'],
                )
            except Exception:
                if mpc_attitude_motion_active:
                    level_mpc_attitude_before_fault(
                        'wrench_pipeline_update_failed',
                        yaw_deg=np.degrees(state['attitude_rpy'][2]),
                    )
                raise

            if release_dataset_terminal_finalize_pending is not None:
                pending = release_dataset_terminal_finalize_pending
                if pending['episode_id'] != release_dataset_episode_id:
                    raise RuntimeError(
                        'LMPC terminal bracket episode identity changed'
                    )
                if state_time < pending['final_position_sent_at']-1e-12:
                    # The callback is fresh relative to the preceding state,
                    # but its measurement can still predate the wall-clock
                    # position send. Keep that command latched and wait for a
                    # state that actually brackets the send. Preserve this
                    # command-free measurement as ordinary path evidence; do
                    # not fabricate an upper bound or add another command.
                    prebracket_boundary_margin_m = float(min(
                        position[0]-self.bounds['x_min'],
                        self.bounds['x_max']-position[0],
                        position[1]-self.bounds['y_min'],
                        self.bounds['y_max']-position[1],
                    ))
                    bootstrap_coverage.observe(
                        release_dataset_episode_id,
                        velocity_xy_m_s=output.estimate.velocity[:2],
                        attitude_rp_rad=output.estimate.orientation_rpy[:2],
                        attitude_rate_rp_rad_s=(
                            output.estimate.angular_velocity[:2]
                        ),
                        boundary_margin_m=prebracket_boundary_margin_m,
                        state_age_s=state_age,
                        state_group_skew_s=state_group_skew,
                        measurement_rejected=(
                            output.estimate.measurement_rejected
                        ),
                    )
                    self.log_manager.add_log_entry('wrench_observer', {
                        'time': now,
                        'state_source': 'crazyflie_state_estimate',
                        'state_time': state_time,
                        'state_age_s': state_age,
                        'state_group_skew_s': state_group_skew,
                        'release_dataset_episode_id': (
                            release_dataset_episode_id
                        ),
                        'release_dataset_direction_xy': (
                            release_dataset_direction_xy.tolist()
                        ),
                        'release_dataset_command_owner': (
                            translation_control.command_mode
                        ),
                        'release_dataset_terminal_gate': (
                            pending['terminal_gate']
                        ),
                        'release_dataset_pending_outcome': (
                            'awaiting_resample_upper_bracket'
                        ),
                        'release_dataset_resample_upper_bracket': False,
                        'release_dataset_final_position_sent_at': (
                            pending['final_position_sent_at']
                        ),
                        'release_dataset_final_position_sequence': (
                            pending['final_position_sequence']
                        ),
                        'actual_command_applied_at_state': (
                            actual_command_applied_at_state
                        ),
                        'actual_commands_sent_since_previous_state': [],
                        'xy_boundary_margin_m': (
                            prebracket_boundary_margin_m
                        ),
                        'position_m': position.tolist(),
                        'velocity_m_s': output.estimate.velocity.tolist(),
                        'orientation_rpy_rad': (
                            output.estimate.orientation_rpy.tolist()
                        ),
                        'angular_velocity_rad_s': (
                            output.estimate.angular_velocity.tolist()
                        ),
                        'battery_voltage_V': battery_voltage,
                        'measurement_rejected': bool(
                            output.estimate.measurement_rejected
                        ),
                        'offline_lmpc_dataset_only': True,
                    })
                    self._safe_sleep(max(dt - (time.time() - now), 0.0))
                    continue
                commands_after_terminal_row = (
                    translation_control.sent_commands_after_sequence(
                        pending['final_position_sequence']
                    )
                )
                if commands_after_terminal_row:
                    raise RuntimeError(
                        'an unlogged command was sent before the LMPC '
                        'terminal interpolation bracket'
                    )
                upper_boundary_margin_m = float(min(
                    position[0]-self.bounds['x_min'],
                    self.bounds['x_max']-position[0],
                    position[1]-self.bounds['y_min'],
                    self.bounds['y_max']-position[1],
                ))
                bootstrap_coverage.observe(
                    release_dataset_episode_id,
                    velocity_xy_m_s=output.estimate.velocity[:2],
                    attitude_rp_rad=output.estimate.orientation_rpy[:2],
                    attitude_rate_rp_rad_s=(
                        output.estimate.angular_velocity[:2]
                    ),
                    boundary_margin_m=upper_boundary_margin_m,
                    state_age_s=state_age,
                    state_group_skew_s=state_group_skew,
                    measurement_rejected=(
                        output.estimate.measurement_rejected
                    ),
                )
                self.log_manager.add_log_entry('wrench_observer', {
                    'time': now,
                    'state_source': 'crazyflie_state_estimate',
                    'state_time': state_time,
                    'state_age_s': state_age,
                    'state_group_skew_s': state_group_skew,
                    'release_dataset_episode_id': (
                        release_dataset_episode_id
                    ),
                    'release_dataset_direction_xy': (
                        release_dataset_direction_xy.tolist()
                    ),
                    'release_dataset_command_owner': (
                        translation_control.command_mode
                    ),
                    'release_dataset_terminal_gate': pending['terminal_gate'],
                    'release_dataset_pending_outcome': None,
                    'release_dataset_resample_upper_bracket': True,
                    'release_dataset_final_position_sent_at': (
                        pending['final_position_sent_at']
                    ),
                    'release_dataset_final_position_sequence': (
                        pending['final_position_sequence']
                    ),
                    'actual_command_applied_at_state': (
                        actual_command_applied_at_state
                    ),
                    'actual_commands_sent_since_previous_state': [],
                    'xy_boundary_margin_m': upper_boundary_margin_m,
                    'position_m': position.tolist(),
                    'velocity_m_s': output.estimate.velocity.tolist(),
                    'orientation_rpy_rad': (
                        output.estimate.orientation_rpy.tolist()
                    ),
                    'angular_velocity_rad_s': (
                        output.estimate.angular_velocity.tolist()
                    ),
                    'battery_voltage_V': battery_voltage,
                    'measurement_rejected': bool(
                        output.estimate.measurement_rejected
                    ),
                    'offline_lmpc_dataset_only': True,
                })
                automatic_close_result = close_bootstrap_episode(
                    release_dataset_episode_id,
                    terminal_success=True,
                    reason=pending['reason'],
                )
                self._log_event(pending['event_name'], {
                    'release_dataset_episode_id': (
                        release_dataset_episode_id
                    ),
                    'release_dataset_outcome': pending['outcome'],
                    'reason': pending['reason'],
                    'terminal_gate': pending['terminal_gate'],
                    'terminal_state_time': pending['terminal_state_time'],
                    'final_position_sent_at': (
                        pending['final_position_sent_at']
                    ),
                    'final_position_sequence': (
                        pending['final_position_sequence']
                    ),
                    'resample_upper_bracket_state_time': state_time,
                    'offline_lmpc_dataset_only': True,
                    'state_source': 'crazyflie_state_estimate',
                })
                if mpc_automatic_attempt is not None:
                    finish_mpc_automatic_attempt(
                        counted=bool(automatic_close_result['counted']),
                        reason=(
                            automatic_close_result['close_reason']
                            if automatic_close_result['counted'] else
                            (
                                automatic_close_result[
                                    'path_failure_reasons'
                                ][0]
                                if automatic_close_result[
                                    'path_failure_reasons'
                                ] else
                                automatic_close_result['reason']
                            )
                        ),
                    )
                release_dataset_episode_id = None
                release_dataset_direction_xy = None
                release_dataset_model_contract = None
                release_dataset_measured_sensor_axis_world_xy = None
                release_dataset_terminal_status = (
                    release_dataset_terminal_gate.reset()
                )
                release_dataset_terminal_finalize_pending = None

            # Attribute this measurement to the command that was actually
            # latched before the sample, never to the next scheduled phase.
            # In particular, retain the final capture sample before recovery.
            if (
                calibration_mode
                and active_position_capture_command is not None
                and active_position_capture_command.phase == 'capture'
                and active_position_capture_command_since is not None
                and state_time >= active_position_capture_command_since
            ):
                position_capture_samples.append({
                    'timestamp': float(state_time),
                    'command_started_at': float(
                        active_position_capture_command_since
                    ),
                    'segment_id': active_position_capture_command.segment_id,
                    'phase': active_position_capture_command.phase,
                    'position': position.tolist(),
                    'velocity': output.estimate.velocity.tolist(),
                    'orientation_rpy': (
                        output.estimate.orientation_rpy.tolist()
                    ),
                    'position_target': (
                        active_position_capture_command.position_target.tolist()
                    ),
                    'battery_voltage_V': battery_voltage,
                })

            if (
                calibration_mode
                and active_planar_braking_command is not None
                and active_planar_braking_command.attitude_control
                and active_planar_braking_command_since is not None
                and state_time >= active_planar_braking_command_since
            ):
                planar_braking_samples.append({
                    'segment_id': (
                        active_planar_braking_command.segment_id
                    ),
                    'timestamp': float(state_time),
                    'command_started_at': float(
                        active_planar_braking_command_since
                    ),
                    'phase': active_planar_braking_command.phase,
                    'direction_xy': (
                        active_planar_braking_command
                        .direction_xy.tolist()
                    ),
                    'command_acceleration_xy': (
                        active_planar_braking_command
                        .command_acceleration_xy.tolist()
                    ),
                    'command_tilt_deg': float(
                        active_planar_braking_command.tilt_deg
                    ),
                    'command_roll_deg': float(
                        active_planar_braking_command.roll_deg
                    ),
                    'command_pitch_deg': float(
                        active_planar_braking_command.pitch_deg
                    ),
                    'actual_attitude_rpy_rad': (
                        output.estimate.orientation_rpy.tolist()
                    ),
                    'angular_velocity_rad_s': state['angular_velocity'].tolist(),
                    'state_group_skew_s': float(state_group_skew),
                    'velocity_xy': output.estimate.velocity[:2].tolist(),
                    'position_xy': position[:2].tolist(),
                })

            sensor_fields = self._force_sensor_log_fields(output.estimate, now)
            control_force_world = output.estimate.external_force.copy()
            force_control_source = 'wrench_observer'
            braking_force_world, braking_force_source = (
                self._release_braking_force(
                    output.estimate,
                    sensor_fields,
                    force_sensor_available
                    and release_mode == 'observer_brake',
                )
            )
            if (
                release_mode == 'potentiometer_coast'
                and potentiometer_release_processed
                and translation_control.release_position_m is not None
            ):
                braking_force_world = np.zeros(3)
                braking_force_source = 'measured_xy_velocity'
            contacts = output.contacts
            if (
                potentiometer_release_detector is not None
                and bool(sensor_fields.get('force_sensor_fresh'))
            ):
                sensor_force_n = float(
                    sensor_fields['force_sensor_compression_force_N']
                )
                sensor_sample_time = float(
                    sensor_fields['force_sensor_sample_time']
                )
                if (
                    potentiometer_contact_detector is not None
                    and not potentiometer_release_detector.armed
                ):
                    potentiometer_contact_decision = (
                        potentiometer_contact_detector.update(
                            sensor_force_n,
                            sensor_sample_time,
                            enabled=(
                                (
                                    initial_contact_gate.armed
                                    and translation_control.mode
                                    == translation_control.POSITION_HOLD
                                )
                                or (
                                    potentiometer_release_processed
                                    and translation_control.mode
                                    == translation_control.VELOCITY_COAST
                                )
                                or (
                                    release_dataset_episode_id is not None
                                    and translation_control.mode in (
                                        translation_control.ATTITUDE_COAST,
                                        translation_control.ATTITUDE_BRAKING,
                                    )
                                )
                            ),
                        )
                    )
                elif potentiometer_release_detector.armed:
                    potentiometer_release_decision = (
                        potentiometer_release_detector.update(
                            sensor_force_n, sensor_sample_time
                        )
                    )
                    if potentiometer_release_decision.candidate_started:
                        self._log_event(
                            'Potentiometer Release Candidate Started',
                            {
                                'compression_force_N': sensor_force_n,
                                'pre_release_force_N': (
                                    potentiometer_release_decision
                                    .pre_release_force_n
                                ),
                                'peak_force_N': (
                                    potentiometer_release_decision.peak_force_n
                                ),
                                'force_drop_N': (
                                    potentiometer_release_decision.force_drop_n
                                ),
                                'force_rate_N_s': (
                                    potentiometer_release_decision.force_rate_n_s
                                ),
                                'unloaded_force_threshold_N': (
                                    release_unloaded_force_n
                                ),
                                'unloaded_dwell_s': release_unloaded_dwell_s,
                                'candidate_stall_timeout_s': (
                                    release_candidate_stall_timeout_s
                                ),
                                'state_source': 'crazyflie_state_estimate',
                            },
                        )
                    elif potentiometer_release_decision.candidate_cancelled:
                        self._log_event(
                            'Potentiometer Release Candidate Cancelled',
                            {
                                'compression_force_N': sensor_force_n,
                                'peak_force_N': (
                                    potentiometer_release_decision.peak_force_n
                                ),
                                'force_drop_N': (
                                    potentiometer_release_decision.force_drop_n
                                ),
                                'candidate_elapsed_s': (
                                    potentiometer_release_decision
                                    .candidate_elapsed_s
                                ),
                                'reason': (
                                    potentiometer_release_decision
                                    .candidate_cancel_reason
                                ),
                                'state_source': 'crazyflie_state_estimate',
                            },
                        )

            sensor_fresh = bool(sensor_fields.get('force_sensor_fresh'))
            (
                candidate_release_sensor_stale_since,
                candidate_release_sensor_stale_timed_out,
            ) = release_candidate_sensor_stale_watchdog(
                potentiometer_release_pending,
                sensor_fresh,
                now,
                candidate_release_sensor_stale_since,
                release_candidate_sensor_stale_timeout_s,
            )
            if (
                release_mode == 'potentiometer_coast'
                and potentiometer_release_pending
                and not sensor_fresh
            ):
                stale_elapsed_s = max(
                    now - candidate_release_sensor_stale_since, 0.0
                )
                if not candidate_release_sensor_stale_logged:
                    self._log_event(
                        'Potentiometer Release Candidate Sensor Stale',
                        {
                            'reason': 'force_sensor_stale',
                            'sample_age_s': sensor_fields.get(
                                'force_sensor_sample_age_s'
                            ),
                            'maximum_sample_age_s': getattr(
                                self, 'sense_max_age_s', 0.25
                            ),
                            'candidate_sensor_stale_timeout_s': (
                                release_candidate_sensor_stale_timeout_s
                            ),
                            'command_mode_unchanged': True,
                            'render_scale_forced_to_zero': True,
                            'state_source': 'crazyflie_state_estimate',
                        },
                    )
                    candidate_release_sensor_stale_logged = True
                if candidate_release_sensor_stale_timed_out:
                    translation_control.cancel_tail_neutralization()
                    if translation_control.attitude_mode:
                        translation_control.set_contact_attitude(
                            0.0,
                            0.0,
                            0.0,
                            yaw_deg=np.degrees(
                                output.estimate.orientation_rpy[2]
                            ),
                        )
                    # Send one safe level or latched-position command before
                    # propagating the fault to the controller's landing path.
                    translation_control.send(
                        self.lo_commander,
                        yaw_deg=np.degrees(
                            output.estimate.orientation_rpy[2]
                        ),
                    )
                    self._log_event(
                        'Potentiometer Release Candidate Sensor Timeout',
                        {
                            'reason': 'force_sensor_stale_timeout',
                            'stale_elapsed_s': stale_elapsed_s,
                            'timeout_s': (
                                release_candidate_sensor_stale_timeout_s
                            ),
                            'safe_command': (
                                'level_attitude'
                                if translation_control.attitude_mode
                                else 'latched_position'
                            ),
                            'state_source': 'crazyflie_state_estimate',
                        },
                    )
                    raise RuntimeError(
                        'force sensor remained stale for '
                        f'{stale_elapsed_s:.3f}s during a release candidate '
                        f'(limit '
                        f'{release_candidate_sensor_stale_timeout_s:.3f}s)'
                    )
                # Keep the detector candidate and locked render direction.
                # A recovered sample-gap update will either restart unloaded
                # dwell or explicitly cancel it before this bounded watchdog
                # expires; missing UART data itself never restores counter-tilt.
                potentiometer_release_decision = None
            elif sensor_fresh:
                candidate_release_sensor_stale_logged = False

            if output.calibrated and not calibration_announced:
                calibration_announced = True
                interaction_start = time.time()
                if bias_calibration_enabled:
                    self._log_event('Wrench Calibration Complete', {
                        'samples': output.calibration_samples,
                        'force_bias_N': pipeline.force_bias.tolist(),
                        'torque_bias_Nm': pipeline.torque_bias.tolist(),
                        'state_source': 'crazyflie_state_estimate',
                    })
                self._log_event(
                    (
                        'Learning MPC Bootstrap Waiting For Automatic Readiness'
                        if mpc_calibration_mode else
                        'Waiting For User Interaction'
                    ), {
                    'contact_detector_armed': initial_contact_gate.armed,
                    'max_xy_speed_m_s': (
                        initial_contact_gate.max_xy_speed_m_s
                    ),
                    'stationary_dwell_s': (
                        initial_contact_gate.stationary_dwell_s
                    ),
                    'state_source': 'crazyflie_state_estimate',
                    },
                )
                if initial_contact_gate.armed:
                    logger.info('Interaction detection is active.')
                else:
                    logger.info(
                        'Waiting for XY speed < %.3f m/s for %.2f s '
                        'before enabling interaction detection.',
                        initial_contact_gate.max_xy_speed_m_s,
                        initial_contact_gate.stationary_dwell_s,
                    )

            if initial_contact_just_armed:
                self._log_event('Initial Contact Detector Armed', {
                    'xy_speed_m_s': initial_contact_gate.xy_speed_m_s,
                    'max_xy_speed_m_s': (
                        initial_contact_gate.max_xy_speed_m_s
                    ),
                    'stationary_elapsed_s': (
                        initial_contact_gate.stationary_elapsed_s
                    ),
                    'stationary_dwell_s': (
                        initial_contact_gate.stationary_dwell_s
                    ),
                    'state_source': 'crazyflie_state_estimate',
                })
                logger.info(
                    'Initial contact detector armed after %.2f s at '
                    'XY speed %.3f m/s.',
                    initial_contact_gate.stationary_elapsed_s,
                    initial_contact_gate.xy_speed_m_s,
                )

            if (
                potentiometer_contact_decision is not None
                and potentiometer_contact_decision.started
                and bool(sensor_fields.get('force_sensor_fresh'))
                and release_dataset_episode_id is not None
                and translation_control.mode in (
                    translation_control.ATTITUDE_COAST,
                    translation_control.ATTITUDE_BRAKING,
                )
            ):
                self._log_event(
                    'Release Dataset Episode Closed',
                    {
                        'release_dataset_episode_id': (
                            release_dataset_episode_id
                        ),
                        'release_dataset_outcome': 'rejected',
                        'reason': 'recontact_before_terminal_dwell',
                        'compression_force_N': sensor_fields.get(
                            'force_sensor_compression_force_N'
                        ),
                        'terminal_gate': (
                            release_dataset_terminal_status.to_dict()
                        ),
                        'offline_lmpc_dataset_only': True,
                        'flight_control_mode_unchanged': True,
                        'state_source': 'crazyflie_state_estimate',
                    },
                )
                close_bootstrap_episode(
                    release_dataset_episode_id,
                    terminal_success=False,
                    reason='recontact_before_terminal_dwell',
                )
                release_dataset_episode_id = None
                release_dataset_direction_xy = None
                release_dataset_model_contract = None
                release_dataset_measured_sensor_axis_world_xy = None
                release_dataset_terminal_status = (
                    release_dataset_terminal_gate.reset()
                )
                # Dataset admission observes the second press but does not
                # expand the legacy controller's recontact ownership rules.
                potentiometer_contact_decision = None

            if (
                potentiometer_contact_decision is not None
                and potentiometer_contact_decision.started
                and bool(sensor_fields.get('force_sensor_fresh'))
            ):
                coast_recontact = bool(
                    potentiometer_release_processed
                    and translation_control.mode
                    == translation_control.VELOCITY_COAST
                )
                if (
                    not pipeline.shadow_mode
                    and translation_control.mode in (
                        translation_control.POSITION_HOLD,
                        translation_control.VELOCITY_COAST,
                    )
                ):
                    begin_two_afc_interaction()
                if coast_recontact:
                    if release_dataset_episode_id is not None:
                        self._log_event(
                            'Release Dataset Episode Closed',
                            {
                                'release_dataset_episode_id': (
                                    release_dataset_episode_id
                                ),
                                'release_dataset_outcome': 'rejected',
                                'reason': (
                                    'recontact_before_terminal_dwell'
                                ),
                                'terminal_gate': (
                                    release_dataset_terminal_status.to_dict()
                                ),
                                'offline_lmpc_dataset_only': True,
                                'state_source': (
                                    'crazyflie_state_estimate'
                                ),
                            },
                        )
                        close_bootstrap_episode(
                            release_dataset_episode_id,
                            terminal_success=False,
                            reason='recontact_before_terminal_dwell',
                        )
                        release_dataset_episode_id = None
                        release_dataset_direction_xy = None
                        release_dataset_model_contract = None
                        release_dataset_measured_sensor_axis_world_xy = None
                        release_dataset_terminal_status = (
                            release_dataset_terminal_gate.reset()
                        )
                    predictive_brake_episode = None
                    predictive_brake_decision = None
                    predictive_brake_abort_after_send = False
                    predictive_position_handoff_logged = False
                    predictive_last_logged_signature = None
                    velocity_mpc_shadow_episode = None
                    velocity_mpc_shadow_direction = None
                    velocity_mpc_shadow_previous_state = None
                    velocity_mpc_shadow_last_decision = None
                    velocity_mpc_shadow_last_logged_signature = None
                    velocity_mpc_shadow_last_log_time = None
                    velocity_mpc_terminal_since = None
                potentiometer_release_processed = False
                potentiometer_release_pending = False
                candidate_release_force_world = None
                candidate_release_direction = None
                candidate_release_attitude_deg = None
                candidate_release_sensor_stale_logged = False
                potentiometer_release_decision = None
                coast_initial_velocity = None
                coast_stop_prediction = None
                sensor_force_n = float(
                    sensor_fields['force_sensor_compression_force_N']
                )
                sensor_sample_time = float(
                    sensor_fields['force_sensor_sample_time']
                )
                potentiometer_release_detector.arm(
                    sensor_force_n,
                    sensor_sample_time,
                    peak_force_n=(
                        potentiometer_contact_decision.peak_force_n
                    ),
                )
                sensor_force_world = (
                    self._force_sensor_axis_world(output.estimate)
                    * sensor_force_n
                )
                if force_rendering_enabled:
                    selection_resistance, _, _ = (
                        virtual_resistance_force(
                            output.estimate.velocity[:2],
                            force_virtual_mass,
                            force_kinetic_friction_coefficient,
                            force_drag_coefficient,
                            force_frontal_area,
                            force_air_density,
                            force_friction_min_speed_m_s,
                            force_static_friction_coefficient,
                            control_force_world[:2],
                        )
                    )
                    render_selection = select_inertia_render_mode(
                        control_force_world[:2],
                        output.estimate.velocity[:2],
                        force_current_mass,
                        force_virtual_mass,
                        preferred_render_mode,
                        selection_resistance,
                        render_acceleration_tolerance_m_s2,
                    )
                else:
                    render_selection = {
                        'mode': 'orientation',
                        'relation': 'force_rendering_disabled',
                        'native_projected_acceleration': 0.0,
                        'virtual_projected_acceleration': 0.0,
                    }
                render_selection = constrain_predictive_coast_render_mode(
                    render_selection, release_mode, pipeline.shadow_mode
                )
                selected_render_mode = render_selection['mode']
                render_relation = render_selection['relation']
                virtual_motion.reset(
                    position[:2], output.estimate.velocity[:2]
                )
                if translation_control.start_contact(
                        selected_render_mode,
                        self._bounded_wrench_reference(position),
                        log_details=current_interaction_log_details(),
                        allow_coast_reentry=coast_recontact):
                    pipeline.admittance.reset()
                    if coast_recontact:
                        self._log_event(
                            'Potentiometer Coast Recontact Detected',
                            {
                                'compression_force_N': sensor_force_n,
                                'contact_force_threshold_N': (
                                    potentiometer_contact_force_n
                                ),
                                'contact_onset_dwell_s': (
                                    potentiometer_contact_dwell_s
                                ),
                                'previous_control_mode': (
                                    translation_control.VELOCITY_COAST
                                ),
                                'new_control_mode': (
                                    translation_control.command_mode
                                ),
                                'previous_coast_episode_cancelled': True,
                                'state_source': (
                                    'crazyflie_state_estimate'
                                ),
                            },
                        )
                    self._log_event(
                        'Translation Contact Start',
                        {
                            'force_N': sensor_force_world.tolist(),
                            'estimated_force_N': (
                                output.estimate.external_force.tolist()
                            ),
                            'force_control_source': (
                                'potentiometer_force_sensor'
                            ),
                            'contact_detection_source': 'potentiometer',
                            'coast_recontact': coast_recontact,
                            'compression_force_N': sensor_force_n,
                            'compression_mm': sensor_fields[
                                'force_sensor_compression_mm'
                            ],
                            'contact_force_threshold_N': (
                                potentiometer_contact_force_n
                            ),
                            'contact_onset_dwell_s': (
                                potentiometer_contact_dwell_s
                            ),
                            'contact_peak_force_N': (
                                potentiometer_contact_decision.peak_force_n
                            ),
                            'interaction_direction': (
                                self._force_sensor_axis_world(
                                    output.estimate
                                ).tolist()
                            ),
                            'response_enabled': not pipeline.shadow_mode,
                            'two_afc_friction_condition': (
                                active_two_afc_condition
                            ),
                            'state_source': 'crazyflie_state_estimate',
                        },
                    )
                    self._log_event(
                        'Translation Rendering Started',
                        {
                            'selected_mode': selected_render_mode,
                            'preferred_mode': preferred_render_mode,
                            'motion_relation': render_relation,
                            'contact_detection_source': 'potentiometer',
                            'native_projected_acceleration_m_s2': (
                                render_selection[
                                    'native_projected_acceleration'
                                ]
                            ),
                            'virtual_projected_acceleration_m_s2': (
                                render_selection[
                                    'virtual_projected_acceleration'
                                ]
                            ),
                            'state_source': 'crazyflie_state_estimate',
                        },
                    )
                potentiometer_contact_decision = None

            if contacts is not None:
                transitions = (
                    ('Translation Contact', contacts.translation),
                    ('Yaw Contact', contacts.yaw),
                )
                for event_name, decision in transitions:
                    if (
                        event_name == 'Translation Contact'
                        and contact_detection_source == 'potentiometer'
                    ):
                        continue
                    if (
                        decision.started
                        or decision.ended
                        or decision.release_candidate_started
                        or decision.release_candidate_cancelled
                    ):
                        if decision.started or decision.ended:
                            transition_event_name = (
                                f"{event_name} "
                                f"{'Start' if decision.started else 'End'}"
                            )
                            if (
                                event_name == 'Translation Contact'
                                and release_mode == 'potentiometer_coast'
                                and (
                                    decision.ended
                                    or translation_control.mode
                                    != translation_control.POSITION_HOLD
                                )
                            ):
                                transition_event_name = (
                                    'Wrench Observer Translation Contact '
                                    f"{'Start' if decision.started else 'End'} "
                                    '(Comparison)'
                                )
                            self._log_event(
                                transition_event_name,
                                {
                                    'force_N': control_force_world.tolist(),
                                    'estimated_force_N': (
                                        output.estimate.external_force.tolist()
                                    ),
                                    'force_control_source': force_control_source,
                                    'torque_Nm': (
                                        output.estimate.external_torque.tolist()
                                    ),
                                    'confidence_sigma': (
                                        decision.confidence_sigma
                                    ),
                                    'release_projected_force_N': (
                                        decision.release_projected_value
                                    ),
                                    'release_projection_normalized': (
                                        decision.release_projection_normalized
                                    ),
                                    'release_direction': (
                                        decision.release_direction
                                    ),
                                    'release_direction_source': (
                                        decision.release_direction_source
                                    ),
                                    'response_enabled': not pipeline.shadow_mode,
                                    'state_source': 'crazyflie_state_estimate',
                                },
                            )
                        if event_name == 'Translation Contact':
                            if decision.started:
                                if (
                                    not pipeline.shadow_mode
                                    and translation_control.mode
                                    == translation_control.POSITION_HOLD
                                ):
                                    begin_two_afc_interaction()
                                if force_rendering_enabled:
                                    selection_resistance, _, _ = (
                                        virtual_resistance_force(
                                            output.estimate.velocity[:2],
                                            force_virtual_mass,
                                            force_kinetic_friction_coefficient,
                                            force_drag_coefficient,
                                            force_frontal_area,
                                            force_air_density,
                                            force_friction_min_speed_m_s,
                                            force_static_friction_coefficient,
                                            control_force_world[:2],
                                        )
                                    )
                                    render_selection = select_inertia_render_mode(
                                        control_force_world[:2],
                                        output.estimate.velocity[:2],
                                        force_current_mass,
                                        force_virtual_mass,
                                        preferred_render_mode,
                                        selection_resistance,
                                        render_acceleration_tolerance_m_s2,
                                    )
                                else:
                                    render_selection = {
                                        'mode': 'orientation',
                                        'relation': 'force_rendering_disabled',
                                        'native_projected_acceleration': 0.0,
                                        'virtual_projected_acceleration': 0.0,
                                    }
                                render_selection = (
                                    constrain_predictive_coast_render_mode(
                                        render_selection,
                                        release_mode,
                                        pipeline.shadow_mode,
                                    )
                                )
                                if translation_control.start_contact(
                                        render_selection['mode'],
                                        self._bounded_wrench_reference(position),
                                        log_details=(
                                            current_interaction_log_details()
                                        )):
                                    # Commit contact-owned state only after the
                                    # handoff accepts the observer onset. A
                                    # detector re-start during release braking
                                    # must not orphan the active pot candidate.
                                    potentiometer_release_processed = False
                                    potentiometer_release_pending = False
                                    candidate_release_force_world = None
                                    candidate_release_direction = None
                                    candidate_release_attitude_deg = None
                                    candidate_release_sensor_stale_logged = False
                                    potentiometer_release_decision = None
                                    coast_initial_velocity = None
                                    coast_stop_prediction = None
                                    selected_render_mode = (
                                        render_selection['mode']
                                    )
                                    render_relation = (
                                        render_selection['relation']
                                    )
                                    virtual_motion.reset(
                                        position[:2],
                                        output.estimate.velocity[:2],
                                    )
                                    if (
                                        potentiometer_release_detector
                                        is not None
                                        and bool(sensor_fields.get(
                                            'force_sensor_fresh'
                                        ))
                                    ):
                                        potentiometer_release_detector.arm(
                                            float(sensor_fields[
                                                'force_sensor_compression_force_N'
                                            ]),
                                            float(sensor_fields[
                                                'force_sensor_sample_time'
                                            ]),
                                        )
                                    pipeline.admittance.reset()
                                    self._log_event(
                                        'Translation Rendering Started',
                                        {
                                            'selected_mode': selected_render_mode,
                                            'preferred_mode': preferred_render_mode,
                                            'motion_relation': render_relation,
                                            'native_projected_acceleration_m_s2': (
                                                render_selection[
                                                    'native_projected_acceleration'
                                                ]
                                            ),
                                            'virtual_projected_acceleration_m_s2': (
                                                render_selection[
                                                    'virtual_projected_acceleration'
                                                ]
                                            ),
                                            'two_afc_friction_condition': (
                                                active_two_afc_condition
                                            ),
                                            'state_source': (
                                                'crazyflie_state_estimate'
                                            ),
                                        },
                                    )
                            if (
                                decision.release_candidate_started
                                and release_mode == 'observer_brake'
                            ):
                                self._log_event(
                                    'Translation Release Candidate',
                                    {
                                        'release_projected_force_N': (
                                            decision.release_projected_value
                                        ),
                                        'release_projection_normalized': (
                                            decision.release_projection_normalized
                                        ),
                                        'confirmation_dwell_s': (
                                            pipeline.detector.translation.release_time_s
                                        ),
                                        'state_source': 'crazyflie_state_estimate',
                                    },
                                )
                            if (
                                release_mode == 'observer_brake'
                                and decision.release_candidate_started
                                and translation_control.end_contact(
                                    self._bounded_wrench_reference(position),
                                    output.estimate.velocity,
                                    state_time,
                                    decision.release_direction,
                                    output.estimate.orientation_rpy,
                                    current_force=braking_force_world,
                                    current_mass_kg=force_current_mass,
                                )
                            ):
                                pipeline.admittance.reset()
                                self._log_event(
                                    'Translation Braking Started',
                                    {
                                        'render_mode': selected_render_mode,
                                        'xy_speed_m_s': float(np.linalg.norm(
                                            output.estimate.velocity[:2]
                                        )),
                                        'projected_speed_m_s': (
                                            translation_control.brake_projected_speed_m_s
                                        ),
                                        'interaction_direction': (
                                            decision.release_direction
                                        ),
                                        'brake_direction': (
                                            translation_control.brake_direction.tolist()
                                        ),
                                        'brake_direction_source': (
                                            translation_control.brake_direction_source
                                        ),
                                        'brake_roll_deg': (
                                            translation_control.contact_roll_deg
                                        ),
                                        'brake_pitch_deg': (
                                            translation_control.contact_pitch_deg
                                        ),
                                        'brake_timeout_s': (
                                            translation_control.brake_timeout_s
                                        ),
                                        'brake_velocity_gain_s': (
                                            translation_control.brake_velocity_gain_s
                                        ),
                                        'brake_min_attitude_deg': (
                                            translation_control.brake_min_attitude_deg
                                        ),
                                        'brake_command_tilt_deg': (
                                            translation_control.brake_command_tilt_deg
                                        ),
                                        'release_force_N': (
                                            translation_control.release_force_N.tolist()
                                        ),
                                        'release_braking_force_source': (
                                            braking_force_source
                                        ),
                                        'release_momentum_kg_m_s': (
                                            translation_control.release_momentum_kg_m_s.tolist()
                                        ),
                                        'release_position_m': (
                                            translation_control.release_position_m.tolist()
                                        ),
                                        'force_feedforward_acceleration_m_s2': (
                                            translation_control.brake_force_feedforward_acceleration_m_s2
                                        ),
                                        'state_source': 'crazyflie_state_estimate',
                                    },
                                )
                            if (
                                decision.release_candidate_cancelled
                                and release_mode == 'observer_brake'
                            ):
                                self._log_event(
                                    'Translation Release Candidate Cancelled',
                                    {
                                        'release_projected_force_N': (
                                            decision.release_projected_value
                                        ),
                                        'release_projection_normalized': (
                                            decision.release_projection_normalized
                                        ),
                                        'state_source': 'crazyflie_state_estimate',
                                    },
                                )
                                if translation_control.cancel_release_candidate(
                                        self._bounded_wrench_reference(position)):
                                    pipeline.admittance.reset()
                                    self._log_event(
                                        'Translation Rendering Resumed',
                                        {
                                            'selected_mode': selected_render_mode,
                                            'motion_relation': render_relation,
                                            'state_source': (
                                                'crazyflie_state_estimate'
                                            ),
                                        },
                                    )
                            if (
                                decision.ended
                                and release_mode == 'observer_brake'
                            ):
                                translation_control.confirm_release_candidate()
                                if (
                                    translation_control.mode
                                    == translation_control.POSITION_HOLD
                                ):
                                    selected_render_mode = None
                                    render_relation = None

            if (
                release_mode == 'potentiometer_coast'
                and potentiometer_release_decision is not None
                and potentiometer_release_decision.candidate_started
                and bool(sensor_fields.get('force_sensor_fresh'))
                and not potentiometer_release_pending
                and (
                    translation_control.attitude_mode
                    or translation_control.position_interaction_mode
                )
            ):
                candidate_force_n = float(
                    potentiometer_release_decision.pre_release_force_n
                    if potentiometer_release_decision.pre_release_force_n
                    is not None
                    else potentiometer_release_decision.last_force_n
                )
                candidate_force_world = (
                    self._force_sensor_axis_world(output.estimate)
                    * candidate_force_n
                )
                # The force sensor defines the user's push direction. The
                # instantaneous velocity at candidate onset can still be a
                # small diagonal hover transient and must not define the
                # zero-crossing direction used by release braking.
                candidate_direction, candidate_direction_source = (
                    potentiometer_release_direction(
                        candidate_force_world,
                        output.estimate.velocity,
                    )
                )
                if bootstrap_coverage is not None:
                    measured_direction = np.asarray(
                        output.estimate.velocity, dtype=float
                    ).copy()
                    measured_direction[2] = 0.0
                    measured_norm = float(np.linalg.norm(
                        measured_direction[:2]
                    ))
                    if measured_norm >= 0.05:
                        locked_direction = mpc_bootstrap_world_y_direction(
                            measured_direction[:2],
                            bootstrap_coverage.config.max_cross_speed_m_s,
                        )
                        if locked_direction is None:
                            candidate_direction = (
                                measured_direction/measured_norm
                            )
                            candidate_direction_source = (
                                'measured_release_velocity_off_axis_fallback'
                            )
                        else:
                            candidate_direction = np.asarray([
                                locked_direction[0], locked_direction[1], 0.0,
                            ])
                            candidate_direction_source = (
                                'locked_world_y_mpc_bootstrap'
                            )
                # Candidate onset is not yet a release, but it is the earliest
                # moment at which the old force-rendering counter-tilt can be
                # removed.  The active attitude path remains command owner and
                # will level/neutralize its calibrated response tail below.
                potentiometer_release_pending = True
                candidate_release_force_world = candidate_force_world.copy()
                candidate_release_direction = candidate_direction.copy()
                candidate_release_attitude_deg = np.array([
                    translation_control.contact_roll_deg,
                    translation_control.contact_pitch_deg,
                ])
                if translation_control.position_interaction_mode:
                    # A position-rendered object can have a target behind the
                    # vehicle when unloading begins. Latch the measured point
                    # immediately so the position PID cannot add pullback while
                    # the reversible release decision is pending.
                    translation_control.set_contact_position(
                        self._bounded_wrench_reference(position)
                    )
                    virtual_motion.reset(
                        position[:2], output.estimate.velocity[:2]
                    )
                candidate_release_sensor_stale_logged = False
                self._log_event(
                    'Potentiometer Release Candidate Tilt Adjustment Started',
                    {
                        'pre_release_force_N': candidate_force_n,
                        'measured_velocity_m_s': (
                            output.estimate.velocity.tolist()
                        ),
                        'candidate_position_m': position.tolist(),
                        'brake_direction_xy': (
                            candidate_direction[:2].tolist()
                        ),
                        'brake_direction_source': (
                            candidate_direction_source
                        ),
                        'command_mode': translation_control.command_mode,
                        'command_owner': (
                            'force_rendering'
                            if force_rendering_enabled
                            else 'level_contact_attitude'
                        ),
                        'initial_render_scale': 0.0,
                        'adjustment': (
                            'level_with_calibrated_tail_neutralization'
                            if translation_control.attitude_mode
                            else 'latch_actual_position'
                        ),
                        'locked_render_roll_deg': float(
                            candidate_release_attitude_deg[0]
                        ),
                        'locked_render_pitch_deg': float(
                            candidate_release_attitude_deg[1]
                        ),
                        'state_source': 'crazyflie_state_estimate',
                    },
                )
            if (
                release_mode == 'potentiometer_coast'
                and potentiometer_release_decision is not None
                and potentiometer_release_decision.candidate_cancelled
                and potentiometer_release_pending
            ):
                # The candidate was reversible.  Its short tail-cancellation
                # pulse must not expire later and overwrite newly resumed force
                # rendering with an unrelated level command.
                translation_control.cancel_tail_neutralization()
                potentiometer_release_pending = False
                candidate_release_force_world = None
                candidate_release_direction = None
                candidate_release_attitude_deg = None
                candidate_release_sensor_stale_logged = False
                coast_initial_velocity = None
                coast_stop_prediction = None
                if translation_control.position_interaction_mode:
                    virtual_motion.reset(
                        position[:2], output.estimate.velocity[:2]
                    )
                self._log_event(
                    'Translation Rendering Continued',
                    {
                        'selected_mode': selected_render_mode,
                        'motion_relation': render_relation,
                        'reason': (
                            'potentiometer_release_candidate_cancelled'
                        ),
                        'command_mode_unchanged': True,
                        'state_source': 'crazyflie_state_estimate',
                    },
                )

            if (
                release_mode == 'potentiometer_coast'
                and potentiometer_release_decision is not None
                and potentiometer_release_decision.released
                and bool(sensor_fields.get('force_sensor_fresh'))
                and not potentiometer_release_processed
                and (
                    translation_control.attitude_mode
                    or translation_control.position_interaction_mode
                )
            ):
                last_force_n = float(
                    potentiometer_release_decision.last_force_n
                )
                last_force_world = (
                    self._force_sensor_axis_world(output.estimate)
                    * last_force_n
                )
                pre_release_force_n = float(
                    potentiometer_release_decision.pre_release_force_n
                    if potentiometer_release_decision.pre_release_force_n
                    is not None else last_force_n
                )
                candidate_force_vector_locked = bool(
                    potentiometer_release_pending
                    and candidate_release_force_world is not None
                )
                if candidate_force_vector_locked:
                    # Preserve the candidate-start world direction.  Rotating
                    # the same scalar again at confirmation would let attitude
                    # changes during unloaded dwell alter the recorded impulse.
                    pre_release_force_world = (
                        candidate_release_force_world.copy()
                    )
                else:
                    pre_release_force_world = (
                        self._force_sensor_axis_world(output.estimate)
                        * pre_release_force_n
                    )
                coast_initial_velocity = release_coast_initial_velocity(
                    output.estimate.velocity,
                    pre_release_force_world,
                    force_current_mass,
                    force_memory_s=release_force_memory_s,
                    max_velocity_m_s=virtual_max_velocity_m_s,
                )
                if bootstrap_coverage is not None:
                    # The potentiometer is a one-axis compression magnitude;
                    # its configured sign cannot represent both directions in
                    # one bootstrap run.  Use the measured release velocity
                    # for the conservative baseline trajectory and retain the
                    # physical sensor axis separately in the audit log.
                    coast_initial_velocity = np.asarray(
                        output.estimate.velocity, dtype=float
                    ).copy()
                virtual_motion.reset(
                    position[:2], coast_initial_velocity[:2]
                )
                coast_stop_prediction = virtual_motion.predict_stop()
                coast_direction = (
                    candidate_release_direction.copy()
                    if candidate_release_direction is not None
                    else output.estimate.velocity.copy()
                )
                if np.linalg.norm(coast_direction[:2]) <= 1e-9:
                    coast_direction = pre_release_force_world.copy()
                if bootstrap_coverage is not None:
                    measured_release_direction = np.asarray(
                        output.estimate.velocity, dtype=float
                    ).copy()
                    measured_release_direction[2] = 0.0
                    measured_release_norm = float(np.linalg.norm(
                        measured_release_direction[:2]
                    ))
                    if measured_release_norm > 1e-9:
                        locked_direction = mpc_bootstrap_world_y_direction(
                            measured_release_direction[:2],
                            bootstrap_coverage.config.max_cross_speed_m_s,
                        )
                        if locked_direction is None:
                            coast_direction = (
                                measured_release_direction
                                / measured_release_norm
                            )
                        else:
                            coast_direction = np.asarray([
                                locked_direction[0], locked_direction[1], 0.0,
                            ])
                release_started = False
                if translation_control.end_contact(
                    self._bounded_wrench_reference(position),
                    output.estimate.velocity,
                    state_time,
                    coast_direction,
                    output.estimate.orientation_rpy,
                    current_force=pre_release_force_world,
                    current_mass_kg=force_current_mass,
                    coast=True,
                ):
                    translation_control.confirm_release_candidate(
                        self._bounded_wrench_reference(position),
                        output.estimate.velocity,
                        pre_release_force_world,
                        state_time,
                    )
                    release_started = True
                else:
                    raise RuntimeError(
                        'confirmed potentiometer release could not transfer '
                        'control from the active interaction'
                    )
                if release_started:
                    if release_dataset_episode_id is not None:
                        self._log_event(
                            'Release Dataset Episode Closed',
                            {
                                'release_dataset_episode_id': (
                                    release_dataset_episode_id
                                ),
                                'release_dataset_outcome': 'rejected',
                                'reason': 'superseded_by_new_release',
                                'terminal_gate': (
                                    release_dataset_terminal_status.to_dict()
                                ),
                                'offline_lmpc_dataset_only': True,
                                'state_source': (
                                    'crazyflie_state_estimate'
                                ),
                            },
                        )
                        close_bootstrap_episode(
                            release_dataset_episode_id,
                            terminal_success=False,
                            reason='superseded_by_new_release',
                        )
                        release_dataset_episode_id = None
                        release_dataset_direction_xy = None
                        release_dataset_model_contract = None
                        release_dataset_measured_sensor_axis_world_xy = None
                        release_dataset_terminal_status = (
                            release_dataset_terminal_gate.reset()
                        )
                    raw_sensor_axis_world_xy = self._force_sensor_axis_world(
                        output.estimate
                    )[:2]
                    (
                        release_dataset_direction_xy,
                        release_dataset_measured_sensor_axis_world_xy,
                    ) = release_dataset_world_y_directions(
                        (
                            coast_direction[:2]
                            if release_dataset_configured_task_axis_xy is None
                            else release_dataset_configured_task_axis_xy
                        ),
                        raw_sensor_axis_world_xy,
                        sensor_axis_is_unsigned=(
                            bootstrap_coverage is not None
                        ),
                    )
                    if release_dataset_direction_xy is not None:
                        if bootstrap_coverage is not None:
                            release_dataset_model_contract = (
                                mpc_bootstrap_model_contract_for_direction(
                                    bootstrap_model_contracts,
                                    release_dataset_direction_xy,
                                )
                            )
                            release_dataset_effective_command_delay_s = float(
                                release_dataset_model_contract[
                                    'command_delay_s'
                                ]
                            )
                            # This is still the release-state sample. Rebind
                            # its causal command history to the exact learned
                            # directional delay before emitting START.
                            actual_command_applied_at_state = (
                                translation_control.sent_command_effective_at(
                                    state_time,
                                    release_dataset_effective_command_delay_s,
                                )
                            )
                        release_dataset_episode_sequence += 1
                        release_dataset_episode_id = (
                            f'{self.drone_id}-release-'
                            f'{int(time.time()*1e6)}-'
                            f'{release_dataset_episode_sequence}'
                        )
                        release_dataset_terminal_status = (
                            release_dataset_terminal_gate.start()
                        )
                        if bootstrap_coverage is not None:
                            mpc_last_decision_send_monotonic = None
                            mpc_next_decision_deadline_monotonic = None
                            bootstrap_assignment = bootstrap_coverage.begin(
                                release_dataset_episode_id,
                                release_dataset_direction_xy,
                                output.estimate.velocity[:2],
                            )
                            self._log_event(
                                'Learning MPC Bootstrap Release Classified',
                                {
                                    **bootstrap_assignment.to_dict(),
                                    'release_dataset_model_label': (
                                        release_dataset_model_contract[
                                            'direction_label'
                                        ]
                                    ),
                                    'release_dataset_model_fingerprint': (
                                        release_dataset_model_contract[
                                            'model_fingerprint'
                                        ]
                                    ),
                                    'release_dataset_state_dimension': (
                                        release_dataset_model_contract[
                                            'state_dimension'
                                        ]
                                    ),
                                    'release_dataset_command_delay_s': (
                                        release_dataset_model_contract[
                                            'command_delay_s'
                                        ]
                                    ),
                                    'coverage': bootstrap_coverage.summary(),
                                    'offline_only': True,
                                    'lmpc_command_authority': False,
                                    'state_source': (
                                        'crazyflie_state_estimate'
                                    ),
                                },
                            )
                            if bootstrap_assignment.countable:
                                logger.info(
                                    'LMPC bootstrap release %.3f m/s matched '
                                    'the %.2f m/s %sY cell.',
                                    bootstrap_assignment.initial_speed_m_s,
                                    bootstrap_assignment.target_speed_m_s,
                                    '+' if (
                                        bootstrap_assignment.direction_sign > 0
                                    ) else '-',
                                )
                            else:
                                logger.warning(
                                    'LMPC bootstrap release %.3f m/s will not '
                                    'count: %s.',
                                    bootstrap_assignment.initial_speed_m_s,
                                    bootstrap_assignment.reason,
                                )
                    else:
                        release_dataset_episode_id = None
                        release_dataset_direction_xy = None
                        release_dataset_model_contract = None
                        release_dataset_terminal_status = (
                            release_dataset_terminal_gate.reset()
                        )
                    if predictive_braking_available:
                        predicted_destination = self._bounded_wrench_reference(
                            np.array([
                                coast_stop_prediction['position'][0],
                                coast_stop_prediction['position'][1],
                                translation_control.hover_z,
                            ])
                        )
                        direction_y = float(coast_direction[1])
                        try:
                            if abs(direction_y) <= 1e-9:
                                raise ValueError(
                                    'release has no world-Y braking direction'
                                )
                            prediction_started_at = time.time()
                            predictive_brake_episode = PredictiveBrakeToPosition(
                                predictive_braking_model,
                                initial_state={
                                    'time_s': state_time,
                                    'position_xy': position[:2],
                                    'velocity_xy': (
                                        output.estimate.velocity[:2]
                                    ),
                                    'orientation_rpy_rad': (
                                        output.estimate.orientation_rpy
                                    ),
                                    'angular_velocity_rad_s': (
                                        state['angular_velocity']
                                    ),
                                    'state_group_skew_s': state_group_skew,
                                },
                                destination_position=predicted_destination,
                                now_s=prediction_started_at,
                                sent_command_history=(
                                    projected_tilt_history_from_world_acceleration(
                                        translation_control
                                        .sent_attitude_acceleration_history(),
                                        [0.0, float(np.sign(direction_y))],
                                    )
                                ),
                                direction_xy=[
                                    0.0, float(np.sign(direction_y))
                                ],
                                config=predictive_braking_config,
                            )
                            predictive_brake_decision = None
                            predictive_brake_abort_after_send = False
                            predictive_position_handoff_logged = False
                            predictive_last_logged_signature = None
                            self._log_event(
                                'Predictive Brake Started',
                                {
                                    'direction_xy': [
                                        0.0, float(np.sign(direction_y))
                                    ],
                                    'release_position_m': position.tolist(),
                                    'release_velocity_m_s': (
                                        output.estimate.velocity.tolist()
                                    ),
                                    'destination_position_m': (
                                        predicted_destination.tolist()
                                    ),
                                    'accept_failed_validation': bool(
                                        predictive_braking_config.get(
                                            'accept_failed_validation', False
                                        )
                                    ),
                                    'state_source': (
                                        'crazyflie_state_estimate'
                                    ),
                                },
                            )
                        except (KeyError, TypeError, ValueError) as error:
                            predictive_brake_episode = None
                            self._log_event(
                                'Predictive Brake Unavailable',
                                {
                                    'reason': str(error),
                                    'fallback': 'legacy_coast_controller',
                                    'state_source': (
                                        'crazyflie_state_estimate'
                                    ),
                                },
                            )
                            logger.warning(
                                'Predictive brake could not start: %s; using '
                                'legacy coast controller.', error,
                            )
                    if velocity_mpc_shadow_available:
                        try:
                            if velocity_mpc_shadow_direction_config is None:
                                direction_y = float(coast_direction[1])
                                if abs(direction_y) <= 1e-9:
                                    raise ValueError(
                                        'release has no world-Y shadow direction'
                                    )
                                velocity_mpc_shadow_direction = np.array([
                                    0.0, float(np.sign(direction_y))
                                ])
                            else:
                                velocity_mpc_shadow_direction = (
                                    velocity_mpc_shadow_direction_config.copy()
                                )
                            frozen_model, selected_model = (
                                frozen_velocity_model_from_prediction_model(
                                    predictive_braking_model,
                                    direction_y=float(
                                        velocity_mpc_shadow_direction[1]
                                    ),
                                    require_validated_evidence=(
                                        velocity_mpc_online_enabled
                                    ),
                                )
                            )
                            velocity_mpc_shadow_episode = LearningVelocityMPC(
                                frozen_model,
                                direction_xy=velocity_mpc_shadow_direction,
                                target_velocity_m_s=(
                                    velocity_mpc_shadow_target_m_s
                                ),
                                config=velocity_mpc_config_object,
                            )
                            if (
                                velocity_mpc_online_enabled
                                and not translation_control
                                .acquire_external_attitude_coast()
                            ):
                                raise ValueError(
                                    'online MPC could not acquire the active '
                                    'coast attitude command path'
                                )
                            sent_history = (
                                projected_tilt_history_from_world_acceleration(
                                    translation_control
                                    .sent_attitude_acceleration_history(),
                                    velocity_mpc_shadow_direction,
                                )
                            )
                            for sent_time, projected_tilt in sent_history:
                                velocity_mpc_shadow_episode.record_sent_command(
                                    sent_time, projected_tilt
                                )
                            velocity_mpc_shadow_previous_state = None
                            velocity_mpc_shadow_last_decision = None
                            velocity_mpc_shadow_last_logged_signature = None
                            velocity_mpc_shadow_last_log_time = None
                            velocity_mpc_terminal_since = None
                            self._log_event(
                                (
                                    'Learning Velocity MPC Online Started'
                                    if velocity_mpc_online_enabled else
                                    'Learning Velocity MPC Shadow Started'
                                ),
                                {
                                    'offline_only': not velocity_mpc_online_enabled,
                                    'command_authority': (
                                        velocity_mpc_online_enabled
                                    ),
                                    'actual_flight_controller_unchanged': (
                                        not velocity_mpc_online_enabled
                                    ),
                                    'direction_xy': (
                                        velocity_mpc_shadow_direction.tolist()
                                    ),
                                    'target_velocity_m_s': (
                                        velocity_mpc_shadow_target_m_s
                                    ),
                                    'selected_directional_model': selected_model,
                                    'model_source': (
                                        'saved_prediction_model_plus_causal_'
                                        'acceleration_residual'
                                    ),
                                    'seeded_sent_command_count': len(sent_history),
                                    'max_decision_time_s': (
                                        velocity_mpc_max_decision_time_s
                                    ),
                                    'state_source': (
                                        'crazyflie_state_estimate'
                                    ),
                                },
                            )
                        except (KeyError, TypeError, ValueError) as error:
                            velocity_mpc_shadow_episode = None
                            velocity_mpc_shadow_direction = None
                            self._log_event(
                                'Learning Velocity MPC Unavailable',
                                {
                                    'reason': str(error),
                                    'offline_only': (
                                        not velocity_mpc_online_enabled
                                    ),
                                    'command_authority': False,
                                    'fallback': 'legacy_coast_controller',
                                    'actual_flight_controller_unchanged': True,
                                    'state_source': (
                                        'crazyflie_state_estimate'
                                    ),
                                },
                            )
                            logger.warning(
                                'Learning velocity MPC could not start: %s; '
                                'using the legacy coast controller.',
                                error,
                            )
                    potentiometer_release_processed = True
                    potentiometer_release_pending = False
                    candidate_release_force_world = None
                    candidate_release_direction = None
                    candidate_release_attitude_deg = None
                    candidate_release_sensor_stale_logged = False
                    if potentiometer_contact_detector is not None:
                        potentiometer_contact_detector.mark_released()
                    if potentiometer_release_detector is not None:
                        # The completed release no longer owns sensor updates.
                        # Re-enable the contact detector during the coast so a
                        # real second press starts a new interaction instead of
                        # being folded into the old braking episode.
                        potentiometer_release_detector.disarm()
                    braking_force_world = np.zeros(3)
                    braking_force_source = 'measured_xy_velocity'
                    pipeline.admittance.reset()
                    self._log_event(
                        'Translation Contact End',
                        {
                            'force_N': (
                                self._force_sensor_axis_world(output.estimate)
                                * potentiometer_release_decision.current_force_n
                            ).tolist(),
                            'estimated_force_N': (
                                output.estimate.external_force.tolist()
                            ),
                            'force_control_source': (
                                'potentiometer_force_sensor'
                            ),
                            'contact_detection_source': (
                                contact_detection_source
                            ),
                            'compression_force_N': (
                                potentiometer_release_decision.current_force_n
                            ),
                            'peak_force_N': (
                                potentiometer_release_decision.peak_force_n
                            ),
                            'force_drop_N': (
                                potentiometer_release_decision.force_drop_n
                            ),
                            'force_rate_N_s': (
                                potentiometer_release_decision.force_rate_n_s
                            ),
                            'unloaded_elapsed_s': (
                                potentiometer_release_decision
                                .unloaded_elapsed_s
                            ),
                            'unloaded_force_threshold_N': (
                                release_unloaded_force_n
                            ),
                            'pre_release_force_N': pre_release_force_n,
                            'pre_release_force_vector_locked_at_candidate': (
                                candidate_force_vector_locked
                            ),
                            'state_source': 'crazyflie_state_estimate',
                        },
                    )
                    self._log_event(
                        'Potentiometer Release Coasting Started',
                        {
                            'last_force_N': pre_release_force_world.tolist(),
                            'last_compression_force_N': pre_release_force_n,
                            'pre_release_compression_force_N': (
                                pre_release_force_n
                            ),
                            'confirmed_unloaded_force_N': (
                                potentiometer_release_decision.current_force_n
                            ),
                            'force_drop_N': (
                                potentiometer_release_decision.force_drop_n
                            ),
                            'force_rate_N_s': (
                                potentiometer_release_decision.force_rate_n_s
                            ),
                            'measured_velocity_m_s': (
                                output.estimate.velocity.tolist()
                            ),
                            'coast_initial_velocity_m_s': (
                                coast_initial_velocity.tolist()
                            ),
                            'predicted_stop_position_m': (
                                None if coast_stop_prediction is None
                                else [
                                    float(coast_stop_prediction['position'][0]),
                                    float(coast_stop_prediction['position'][1]),
                                    float(translation_control.hover_z),
                                ]
                            ),
                            'predicted_stop_duration_s': (
                                None if coast_stop_prediction is None
                                else coast_stop_prediction['duration_s']
                            ),
                            'predicted_stop_reached': (
                                None if coast_stop_prediction is None
                                else coast_stop_prediction['stopped']
                            ),
                            'release_position_m': position.tolist(),
                            'release_dataset_episode_id': (
                                release_dataset_episode_id
                            ),
                            'release_dataset_direction_xy': (
                                None
                                if release_dataset_direction_xy is None else
                                release_dataset_direction_xy.tolist()
                            ),
                            'release_dataset_measured_sensor_axis_world_xy': (
                                None
                                if release_dataset_measured_sensor_axis_world_xy
                                is None else
                                release_dataset_measured_sensor_axis_world_xy
                                .tolist()
                            ),
                            'release_dataset_raw_sensor_axis_world_xy': (
                                raw_sensor_axis_world_xy.tolist()
                            ),
                            'release_direction_source': (
                                'measured_velocity_and_unsigned_sensor_axis'
                                if bootstrap_coverage is not None else
                                'signed_sensor_axis'
                            ),
                            'release_state_time': state_time,
                            'release_orientation_rpy_rad': (
                                output.estimate.orientation_rpy.tolist()
                            ),
                            'release_angular_velocity_rad_s': (
                                state['angular_velocity'].tolist()
                            ),
                            'release_state_age_s': state_age,
                            'release_state_group_skew_s': state_group_skew,
                            'release_battery_voltage_V': battery_voltage,
                            'release_dataset_prediction_step_s': (
                                None
                                if bootstrap_coverage is None else
                                bootstrap_coverage.config.prediction_step_s
                            ),
                            'release_dataset_model_label': (
                                None
                                if release_dataset_model_contract is None else
                                release_dataset_model_contract[
                                    'direction_label'
                                ]
                            ),
                            'release_dataset_model_fingerprint': (
                                None
                                if release_dataset_model_contract is None else
                                release_dataset_model_contract[
                                    'model_fingerprint'
                                ]
                            ),
                            'release_dataset_state_dimension': (
                                None
                                if release_dataset_model_contract is None else
                                release_dataset_model_contract[
                                    'state_dimension'
                                ]
                            ),
                            'release_dataset_command_delay_s': (
                                None
                                if release_dataset_model_contract is None else
                                release_dataset_effective_command_delay_s
                            ),
                            'release_command_effective_at_state': (
                                actual_command_applied_at_state
                            ),
                            'release_pending_command_history': (
                                translation_control.sent_commands_in_window(
                                    state_time
                                    - release_dataset_effective_command_delay_s,
                                    state_observed_at,
                                )
                            ),
                            'offline_lmpc_dataset_only': True,
                            'force_memory_s': release_force_memory_s,
                            'unloaded_elapsed_s': (
                                potentiometer_release_decision
                                .unloaded_elapsed_s
                            ),
                            'initial_command_mode': (
                                translation_control.command_mode
                            ),
                            'velocity_coast_fixed_zdistance_m': (
                                translation_control
                                .velocity_coast_fixed_zdistance_m
                            ),
                            'state_source': 'crazyflie_state_estimate',
                        },
                    )

            if (
                mpc_calibration_mode
                and output.calibrated
                and interaction_start is not None
                and release_dataset_terminal_finalize_pending is None
            ):
                if (
                    mpc_automatic_attempt is None
                    and release_dataset_episode_id is None
                    and translation_control.mode
                    == translation_control.POSITION_HOLD
                    and not bootstrap_coverage.complete
                    and mpc_automatic_abort_after_safe_stop is None
                    and not output.estimate.measurement_rejected
                ):
                    automatic_cell = bootstrap_coverage.next_required_cell(
                        mpc_automatic_previous_cell
                    )
                    automatic_contract = (
                        mpc_bootstrap_model_contract_for_direction(
                            bootstrap_model_contracts,
                            automatic_cell.direction_xy,
                        )
                    )
                    mpc_automatic_attempt = MPCBootstrapAutomaticAttempt(
                        bootstrap_coverage.config,
                        automatic_cell,
                        nominal_position_xy_m=nominal_position[:2],
                        directional_model_delay_s=(
                            automatic_contract['command_delay_s']
                        ),
                        nominal_yaw_deg=nominal_yaw_deg,
                    )
                    self._log_event(
                        'Learning MPC Bootstrap Automatic Attempt Scheduled',
                        {
                            **automatic_cell.to_dict(),
                            'direction_xy': list(
                                automatic_cell.direction_xy
                            ),
                            'acceleration_tilt_deg': (
                                bootstrap_coverage.config
                                .automatic_acceleration_tilt_deg
                            ),
                            'release_tolerance_m_s': (
                                bootstrap_coverage.config
                                .automatic_release_tolerance_m_s
                            ),
                            'directional_model_delay_s': (
                                automatic_contract['command_delay_s']
                            ),
                            'level_warmup_required_s': (
                                mpc_automatic_attempt
                                .level_warmup_required_s
                            ),
                            'offline_only': True,
                            'lmpc_command_authority': False,
                            'state_source': 'crazyflie_state_estimate',
                        },
                    )

                if (
                    mpc_automatic_attempt is not None
                    and mpc_automatic_attempt.phase in (
                        mpc_automatic_attempt.READY_DWELL,
                        mpc_automatic_attempt.LEVEL_WARMUP,
                        mpc_automatic_attempt.ACCELERATING,
                    )
                ):
                    automatic_boundary_margin_m = (
                        float(min(
                            position[0]-self.bounds['x_min'],
                            self.bounds['x_max']-position[0],
                            position[1]-self.bounds['y_min'],
                            self.bounds['y_max']-position[1],
                        ))
                        if self.bounds is not None else float('-inf')
                    )
                    mpc_automatic_decision = mpc_automatic_attempt.observe(
                        time_s=state_time,
                        position_xy_m=position[:2],
                        velocity_xy_m_s=output.estimate.velocity[:2],
                        attitude_rp_rad=(
                            output.estimate.orientation_rpy[:2]
                        ),
                        attitude_rate_rp_rad_s=(
                            output.estimate.angular_velocity[:2]
                        ),
                        boundary_margin_m=automatic_boundary_margin_m,
                        state_age_s=state_age,
                        state_group_skew_s=state_group_skew,
                        z_error_m=float(position[2]-nominal_position[2]),
                        sample_gap_s=protocol_state_step_s,
                        measurement_rejected=(
                            output.estimate.measurement_rejected
                        ),
                    )
                    automatic_phase_changed = bool(
                        mpc_automatic_last_phase
                        != mpc_automatic_decision.phase
                    )
                    if (
                        automatic_phase_changed
                        or mpc_automatic_decision.start_braking
                        or mpc_automatic_decision.abort_requested
                    ):
                        self._log_event(
                            'Learning MPC Bootstrap Automatic Decision',
                            {
                                **mpc_automatic_decision.to_dict(),
                                'offline_only': True,
                                'lmpc_command_authority': False,
                                'state_source': (
                                    'crazyflie_state_estimate'
                                ),
                            },
                        )
                    mpc_automatic_last_phase = (
                        mpc_automatic_decision.phase
                    )
                    automatic_command_kind = (
                        mpc_automatic_decision.command_kind
                    )

                    if automatic_command_kind == 'position_hold':
                        if (
                            translation_control.mode
                            != translation_control.POSITION_HOLD
                        ):
                            level_mpc_attitude_before_fault(
                                'automatic_ready_phase_lost_position_owner',
                                yaw_deg=np.degrees(
                                    output.estimate.orientation_rpy[2]
                                ),
                            )
                            raise RuntimeError(
                                'automatic LMPC readiness lost position '
                                'command ownership'
                            )
                        translation_control.hold_position = (
                            nominal_position.copy()
                        )
                        translation_control.yaw_deg = nominal_yaw_deg
                    elif automatic_command_kind in (
                        'level_attitude_zdistance',
                        'automatic_acceleration_attitude_zdistance',
                    ):
                        if (
                            translation_control.mode
                            == translation_control.POSITION_HOLD
                        ):
                            if not translation_control.start_contact(
                                'mpc_bootstrap_acceleration',
                                current_position=(
                                    self._bounded_wrench_reference(position)
                                ),
                                log_details=(
                                    'automatic offline LMPC bootstrap; '
                                    'target={:.2f}m/s, direction={:+d}Y'.format(
                                        mpc_automatic_decision.target_speed_m_s,
                                        mpc_automatic_decision.direction_sign,
                                    )
                                ),
                            ):
                                raise RuntimeError(
                                    'automatic LMPC bootstrap could not '
                                    'acquire attitude command ownership'
                                )
                            translation_control.hover_z = float(
                                nominal_position[2]
                            )
                            mpc_last_decision_send_monotonic = None
                            mpc_next_decision_deadline_monotonic = None
                        elif (
                            translation_control.mode
                            != translation_control
                            .MPC_BOOTSTRAP_ACCELERATION
                        ):
                            level_mpc_attitude_before_fault(
                                'automatic_acceleration_command_owner_changed',
                                yaw_deg=np.degrees(
                                    output.estimate.orientation_rpy[2]
                                ),
                            )
                            raise RuntimeError(
                                'automatic LMPC acceleration lost attitude '
                                'command ownership'
                            )
                        translation_control.set_contact_attitude(
                            mpc_automatic_decision.command_roll_deg,
                            mpc_automatic_decision.command_pitch_deg,
                            0.0,
                            yaw_deg=np.degrees(
                                output.estimate.orientation_rpy[2]
                            ),
                        )
                    elif automatic_command_kind in (
                        'begin_legacy_attitude_coast',
                        'abort_to_legacy_attitude_coast',
                    ):
                        automatic_release_velocity = np.asarray(
                            output.estimate.velocity, dtype=float
                        ).copy()
                        coast_initial_velocity = (
                            automatic_release_velocity.copy()
                        )
                        virtual_motion.reset(
                            position[:2], coast_initial_velocity[:2]
                        )
                        coast_stop_prediction = virtual_motion.predict_stop()
                        automatic_coast_direction = np.asarray([
                            0.0,
                            float(mpc_automatic_decision.direction_sign),
                            0.0,
                        ])
                        if not translation_control.end_contact(
                            self._bounded_wrench_reference(position),
                            automatic_release_velocity,
                            state_time,
                            automatic_coast_direction,
                            output.estimate.orientation_rpy,
                            current_force=np.zeros(3),
                            current_mass_kg=force_current_mass,
                            coast=True,
                        ):
                            level_mpc_attitude_before_fault(
                                'automatic_release_handoff_failed',
                                yaw_deg=np.degrees(
                                    output.estimate.orientation_rpy[2]
                                ),
                            )
                            raise RuntimeError(
                                'automatic LMPC release could not transfer '
                                'to the legacy attitude-coast controller'
                            )
                        translation_control.hover_z = float(
                            nominal_position[2]
                        )
                        translation_control.confirm_release_candidate(
                            self._bounded_wrench_reference(position),
                            automatic_release_velocity,
                            np.zeros(3),
                            state_time,
                        )
                        mpc_automatic_release_confirmed = True
                        braking_force_world = np.zeros(3)
                        braking_force_source = 'measured_xy_velocity'
                        pipeline.admittance.reset()

                        if automatic_command_kind == (
                            'begin_legacy_attitude_coast'
                        ):
                            release_dataset_direction_xy = np.asarray(
                                mpc_automatic_decision.direction_xy,
                                dtype=float,
                            )
                            release_dataset_measured_sensor_axis_world_xy = (
                                None
                            )
                            release_dataset_model_contract = (
                                mpc_bootstrap_model_contract_for_direction(
                                    bootstrap_model_contracts,
                                    release_dataset_direction_xy,
                                )
                            )
                            release_dataset_effective_command_delay_s = float(
                                release_dataset_model_contract[
                                    'command_delay_s'
                                ]
                            )
                            actual_command_applied_at_state = (
                                translation_control
                                .sent_command_effective_at(
                                    state_time,
                                    release_dataset_effective_command_delay_s,
                                )
                            )
                            if actual_command_applied_at_state is None:
                                level_mpc_attitude_before_fault(
                                    'automatic_release_has_no_causal_command',
                                    yaw_deg=np.degrees(
                                        output.estimate.orientation_rpy[2]
                                    ),
                                )
                                raise RuntimeError(
                                    'automatic LMPC release has no causal '
                                    'attitude command at the learned delay'
                                )
                            release_dataset_episode_sequence += 1
                            release_dataset_episode_id = (
                                f'{self.drone_id}-automatic-release-'
                                f'{int(time.time()*1e6)}-'
                                f'{release_dataset_episode_sequence}'
                            )
                            release_dataset_terminal_status = (
                                release_dataset_terminal_gate.start()
                            )
                            bootstrap_assignment = bootstrap_coverage.begin(
                                release_dataset_episode_id,
                                release_dataset_direction_xy,
                                automatic_release_velocity[:2],
                            )
                            self._log_event(
                                'Learning MPC Bootstrap Release Classified',
                                {
                                    **bootstrap_assignment.to_dict(),
                                    'release_dataset_model_label': (
                                        release_dataset_model_contract[
                                            'direction_label'
                                        ]
                                    ),
                                    'release_dataset_model_fingerprint': (
                                        release_dataset_model_contract[
                                            'model_fingerprint'
                                        ]
                                    ),
                                    'release_dataset_state_dimension': (
                                        release_dataset_model_contract[
                                            'state_dimension'
                                        ]
                                    ),
                                    'release_dataset_command_delay_s': (
                                        release_dataset_effective_command_delay_s
                                    ),
                                    'coverage': bootstrap_coverage.summary(),
                                    'automatic_release': True,
                                    'offline_only': True,
                                    'lmpc_command_authority': False,
                                    'state_source': (
                                        'crazyflie_state_estimate'
                                    ),
                                },
                            )
                            self._log_event(
                                'Learning MPC Bootstrap Braking Started',
                                {
                                    'release_dataset_episode_id': (
                                        release_dataset_episode_id
                                    ),
                                    'release_dataset_direction_xy': (
                                        release_dataset_direction_xy.tolist()
                                    ),
                                    'release_dataset_axis_source': (
                                        'automatic_world_y_profile'
                                    ),
                                    'release_dataset_measured_sensor_axis_world_xy': (
                                        None
                                    ),
                                    'measured_velocity_m_s': (
                                        automatic_release_velocity.tolist()
                                    ),
                                    'coast_initial_velocity_m_s': (
                                        automatic_release_velocity.tolist()
                                    ),
                                    'release_position_m': position.tolist(),
                                    'release_state_time': state_time,
                                    'release_orientation_rpy_rad': (
                                        output.estimate.orientation_rpy.tolist()
                                    ),
                                    'release_angular_velocity_rad_s': (
                                        output.estimate.angular_velocity.tolist()
                                    ),
                                    'release_state_age_s': state_age,
                                    'release_state_group_skew_s': (
                                        state_group_skew
                                    ),
                                    'release_battery_voltage_V': (
                                        battery_voltage
                                    ),
                                    'release_dataset_prediction_step_s': (
                                        bootstrap_coverage.config
                                        .prediction_step_s
                                    ),
                                    'release_dataset_model_label': (
                                        release_dataset_model_contract[
                                            'direction_label'
                                        ]
                                    ),
                                    'release_dataset_model_fingerprint': (
                                        release_dataset_model_contract[
                                            'model_fingerprint'
                                        ]
                                    ),
                                    'release_dataset_state_dimension': (
                                        release_dataset_model_contract[
                                            'state_dimension'
                                        ]
                                    ),
                                    'release_dataset_command_delay_s': (
                                        release_dataset_effective_command_delay_s
                                    ),
                                    'release_command_effective_at_state': (
                                        actual_command_applied_at_state
                                    ),
                                    'release_pending_command_history': (
                                        translation_control
                                        .sent_commands_in_window(
                                            state_time
                                            - release_dataset_effective_command_delay_s,
                                            state_observed_at,
                                        )
                                    ),
                                    'initial_command_mode': (
                                        translation_control.command_mode
                                    ),
                                    'offline_lmpc_dataset_only': True,
                                    'lmpc_command_authority': False,
                                    'state_source': (
                                        'crazyflie_state_estimate'
                                    ),
                                },
                            )
                        else:
                            self._log_event(
                                'Learning MPC Bootstrap Automatic Prelude '
                                'Rejected',
                                {
                                    **mpc_automatic_decision.to_dict(),
                                    'safe_fallback': (
                                        'legacy_attitude_coast_to_rest'
                                    ),
                                    'dataset_started': False,
                                    'offline_only': True,
                                    'lmpc_command_authority': False,
                                    'state_source': (
                                        'crazyflie_state_estimate'
                                    ),
                                },
                            )
                    elif automatic_command_kind == 'abort_level_and_land':
                        level_mpc_attitude_before_fault(
                            'automatic_prelude_hard_safety_violation',
                            yaw_deg=np.degrees(
                                output.estimate.orientation_rpy[2]
                            ),
                        )
                        self._log_event(
                            'Learning MPC Bootstrap Automatic Safety Abort',
                            {
                                **mpc_automatic_decision.to_dict(),
                                'safe_command': 'level_attitude_then_land',
                                'dataset_started': False,
                                'offline_only': True,
                                'lmpc_command_authority': False,
                                'state_source': (
                                    'crazyflie_state_estimate'
                                ),
                            },
                        )
                        raise RuntimeError(
                            'automatic LMPC bootstrap safety abort: '
                            + ', '.join(
                                mpc_automatic_decision
                                .prelude_failure_reasons
                            )
                        )
                    else:
                        raise RuntimeError(
                            'unknown automatic LMPC bootstrap command: '
                            + str(automatic_command_kind)
                        )

            force_target_pitch = 0.0
            force_target_roll = 0.0
            force_raw_tilt_deg = 0.0
            force_attitude_saturated = False
            virtual_motion_state = None
            force_virtual_friction_N = 0.0
            force_virtual_drag_N = 0.0
            force_virtual_resistance_xy = np.zeros(2)
            release_candidate_render_scale = (
                0.0 if potentiometer_release_pending else 1.0
            )
            release_candidate_tail_tracking = None
            attitude_command_planned_at = None
            if (
                output.calibrated
                and force_rendering_enabled
                and (
                    translation_control.attitude_mode
                    or translation_control.position_interaction_mode
                )
                and not output.estimate.measurement_rejected
            ):
                (
                    force_virtual_resistance_xy,
                    force_virtual_friction_N,
                    force_virtual_drag_N,
                ) = virtual_resistance_force(
                    output.estimate.velocity[:2],
                    force_virtual_mass,
                    force_kinetic_friction_coefficient,
                    force_drag_coefficient,
                    force_frontal_area,
                    force_air_density,
                    force_friction_min_speed_m_s,
                    force_static_friction_coefficient,
                    control_force_world[:2],
                )
                if translation_control.attitude_mode:
                    if not potentiometer_release_pending:
                        (
                            force_target_pitch,
                            force_target_roll,
                            force_raw_tilt_deg,
                            force_attitude_saturated,
                        ) = force_inertia_attitude(
                            control_force_world[:2],
                            np.degrees(
                                output.estimate.orientation_rpy[2]
                            ),
                            force_current_mass,
                            force_virtual_mass,
                            force_max_attitude_deg,
                            force_virtual_resistance_xy,
                        )
                else:
                    # A position-rendered candidate freezes its last target.
                    # Advancing the virtual object after support is fading can
                    # create the same pullback through position control.
                    if not potentiometer_release_pending:
                        virtual_motion_state = virtual_motion.step(
                            control_force_world[:2], dt
                        )
                        position_target = np.array([
                            virtual_motion_state['position'][0],
                            virtual_motion_state['position'][1],
                            translation_control.hover_z,
                        ])
                        translation_control.set_contact_position(
                            self._bounded_wrench_reference(position_target)
                        )
            if (
                potentiometer_release_pending
                and translation_control.attitude_mode
                and candidate_release_direction is not None
            ):
                # Candidate onset is the earliest evidence that user support is
                # disappearing. Stop force rendering immediately, then use the
                # calibrated command queue to add only enough sign-symmetric
                # impulse to keep the residual tail between current speed and
                # rest.
                attitude_command_planned_at = time.time()
                release_candidate_tail_tracking = (
                    translation_control.update_release_candidate_attitude(
                        output.estimate.velocity,
                        output.estimate.orientation_rpy,
                        candidate_release_direction,
                        state_time,
                        command_timestamp=attitude_command_planned_at,
                    )
                )
                if release_candidate_tail_tracking is not None:
                    force_target_roll = float(
                        release_candidate_tail_tracking['roll_deg']
                    )
                    force_target_pitch = float(
                        release_candidate_tail_tracking['pitch_deg']
                    )
                    force_raw_tilt_deg = float(
                        release_candidate_tail_tracking['raw_tilt_deg']
                    )
                    force_attitude_saturated = bool(
                        force_raw_tilt_deg
                        >= translation_control.brake_max_attitude_deg - 1e-9
                    )
            if translation_control.attitude_mode:
                translation_control.set_contact_attitude(
                    force_target_roll,
                    force_target_pitch,
                    0.0,
                    yaw_deg=np.degrees(output.estimate.orientation_rpy[2]),
                )

            braking_kwargs = {}
            coast_handoff_completed = False
            velocity_mpc_handoff_completed = False
            predictive_brake_decision = None
            if (
                velocity_mpc_shadow_episode is not None
                and potentiometer_release_processed
            ):
                velocity_mpc_shadow_now = time.time()
                velocity_mpc_shadow_state = VelocityMPCState(
                    time_s=state_time,
                    velocity_xy=tuple(output.estimate.velocity[:2]),
                    orientation_rpy_rad=tuple(
                        output.estimate.orientation_rpy
                    ),
                    angular_velocity_rad_s=tuple(state['angular_velocity']),
                    state_group_skew_s=state_group_skew,
                )
                if velocity_mpc_shadow_previous_state is not None:
                    try:
                        velocity_mpc_shadow_episode.observe_transition(
                            velocity_mpc_shadow_previous_state,
                            velocity_mpc_shadow_state,
                        )
                    except (TypeError, ValueError) as error:
                        logger.debug(
                            'Learning velocity MPC shadow residual sample '
                            'rejected: %s', error,
                        )
                velocity_mpc_shadow_previous_state = (
                    velocity_mpc_shadow_state
                )
                velocity_mpc_decision_started = time.perf_counter()
                velocity_mpc_shadow_last_decision = (
                    velocity_mpc_shadow_episode.decide(
                        velocity_mpc_shadow_now,
                        velocity_mpc_shadow_state,
                    )
                )
                velocity_mpc_decision_elapsed_s = (
                    time.perf_counter() - velocity_mpc_decision_started
                )
                if velocity_mpc_online_enabled:
                    velocity_mpc_action = (
                        velocity_mpc_shadow_last_decision.get('action')
                    )
                    velocity_mpc_fallback_reason = None
                    if (
                        velocity_mpc_decision_elapsed_s
                        > velocity_mpc_max_decision_time_s
                    ):
                        velocity_mpc_fallback_reason = (
                            'decision_runtime_budget_exceeded'
                        )
                    elif velocity_mpc_action == 'fallback_level':
                        velocity_mpc_fallback_reason = (
                            velocity_mpc_shadow_last_decision.get('reason')
                            or 'controller_requested_fallback'
                        )
                    if velocity_mpc_fallback_reason is not None:
                        self._log_event(
                            'Learning Velocity MPC Online Fallback',
                            {
                                'reason': velocity_mpc_fallback_reason,
                                'decision_elapsed_s': (
                                    velocity_mpc_decision_elapsed_s
                                ),
                                'max_decision_time_s': (
                                    velocity_mpc_max_decision_time_s
                                ),
                                'fallback': 'legacy_coast_controller',
                                'state_source': (
                                    'crazyflie_state_estimate'
                                ),
                            },
                        )
                        velocity_mpc_shadow_episode = None
                        velocity_mpc_terminal_since = None
                    else:
                        translation_control.set_contact_attitude(
                            velocity_mpc_shadow_last_decision['roll_deg'],
                            velocity_mpc_shadow_last_decision['pitch_deg'],
                            0.0,
                            yaw_deg=np.degrees(
                                output.estimate.orientation_rpy[2]
                            ),
                        )
                        attitude_command_planned_at = (
                            velocity_mpc_shadow_now
                        )
                        projected_velocity_m_s = float(
                            np.dot(
                                output.estimate.velocity[:2],
                                velocity_mpc_shadow_direction,
                            )
                        )
                        actual_tilt_deg = float(np.linalg.norm(
                            np.degrees(
                                output.estimate.orientation_rpy[:2]
                            )
                        ))
                        actual_tilt_rate_deg_s = float(np.linalg.norm(
                            np.degrees(state['angular_velocity'][:2])
                        ))
                        commanded_tilt_deg = float(np.linalg.norm([
                            velocity_mpc_shadow_last_decision['roll_deg'],
                            velocity_mpc_shadow_last_decision['pitch_deg'],
                        ]))
                        measured_terminal = bool(
                            abs(
                                projected_velocity_m_s
                                - velocity_mpc_shadow_target_m_s
                            )
                            <= velocity_mpc_config_object
                            .terminal_velocity_tolerance_m_s
                            and actual_tilt_deg
                            <= velocity_mpc_config_object
                            .terminal_tilt_tolerance_deg
                            and actual_tilt_rate_deg_s
                            <= velocity_mpc_config_object
                            .terminal_tilt_rate_tolerance_deg_s
                            and commanded_tilt_deg
                            <= velocity_mpc_config_object
                            .terminal_tilt_tolerance_deg
                        )
                        if measured_terminal:
                            if velocity_mpc_terminal_since is None:
                                velocity_mpc_terminal_since = state_time
                        else:
                            velocity_mpc_terminal_since = None
                        terminal_dwell_s = (
                            0.0
                            if velocity_mpc_terminal_since is None else
                            max(0.0, state_time-velocity_mpc_terminal_since)
                        )
                        if (
                            measured_terminal
                            and terminal_dwell_s
                            >= velocity_mpc_config_object.terminal_dwell_s
                        ):
                            position_reset_started_at = time.time()
                            position_reset_method = (
                                reset_pid_integrators_without_ack(
                                    self.cf,
                                    (
                                        'posCtlPid.resetI',
                                        'velCtlPid.resetI',
                                    ),
                                )
                            )
                            handoff_target = np.array([
                                position[0], position[1],
                                translation_control.hover_z,
                            ])
                            translation_control.set_predictive_position_target(
                                self._bounded_wrench_reference(handoff_target),
                                state_time,
                            )
                            velocity_mpc_handoff_completed = True
                            self._log_event(
                                'Learning Velocity MPC Position Handoff',
                                {
                                    'reason': (
                                        'measured_terminal_state_dwell'
                                    ),
                                    'target_velocity_m_s': (
                                        velocity_mpc_shadow_target_m_s
                                    ),
                                    'measured_projected_velocity_m_s': (
                                        projected_velocity_m_s
                                    ),
                                    'actual_tilt_deg': actual_tilt_deg,
                                    'actual_tilt_rate_deg_s': (
                                        actual_tilt_rate_deg_s
                                    ),
                                    'terminal_dwell_s': terminal_dwell_s,
                                    'hold_position_m': (
                                        translation_control
                                        .hold_position.tolist()
                                    ),
                                    'position_integrators_reset': True,
                                    'integrator_reset_method': (
                                        position_reset_method
                                    ),
                                    'integrator_reset_elapsed_s': (
                                        time.time()-position_reset_started_at
                                    ),
                                    'state_source': (
                                        'crazyflie_state_estimate'
                                    ),
                                },
                            )
                            velocity_mpc_shadow_episode = None
                            velocity_mpc_terminal_since = None
                velocity_mpc_signature = (
                    velocity_mpc_shadow_last_decision.get('action'),
                    velocity_mpc_shadow_last_decision.get('reason'),
                )
                velocity_mpc_log_due = bool(
                    velocity_mpc_shadow_last_log_time is None
                    or velocity_mpc_shadow_now
                    - velocity_mpc_shadow_last_log_time
                    >= velocity_mpc_shadow_log_interval_s
                )
                if (
                    velocity_mpc_signature
                    != velocity_mpc_shadow_last_logged_signature
                    or velocity_mpc_log_due
                ):
                    self._log_event(
                        (
                            'Learning Velocity MPC Online Decision'
                            if velocity_mpc_online_enabled else
                            'Learning Velocity MPC Shadow Decision'
                        ),
                        {
                            'offline_only': not velocity_mpc_online_enabled,
                            'command_authority': (
                                velocity_mpc_online_enabled
                            ),
                            'actual_flight_controller_unchanged': (
                                not velocity_mpc_online_enabled
                            ),
                            'decision_elapsed_s': (
                                velocity_mpc_decision_elapsed_s
                            ),
                            'action': (
                                velocity_mpc_shadow_last_decision.get(
                                    'action'
                                )
                            ),
                            'reason': (
                                velocity_mpc_shadow_last_decision.get(
                                    'reason'
                                )
                            ),
                            'direction_xy': (
                                velocity_mpc_shadow_direction.tolist()
                            ),
                            'measured_velocity_m_s': (
                                velocity_mpc_shadow_last_decision.get(
                                    'measured_velocity_m_s'
                                )
                            ),
                            'target_velocity_m_s': (
                                velocity_mpc_shadow_target_m_s
                            ),
                            'hypothetical_roll_deg': (
                                velocity_mpc_shadow_last_decision.get(
                                    'roll_deg'
                                )
                            ),
                            'hypothetical_pitch_deg': (
                                velocity_mpc_shadow_last_decision.get(
                                    'pitch_deg'
                                )
                            ),
                            'predicted_arrival_time_s': (
                                velocity_mpc_shadow_last_decision.get(
                                    'predicted_arrival_time_s'
                                )
                            ),
                            'predicted_terminal_velocity_m_s': (
                                velocity_mpc_shadow_last_decision.get(
                                    'predicted_terminal_velocity_m_s'
                                )
                            ),
                            'predicted_terminal_tilt_deg': (
                                velocity_mpc_shadow_last_decision.get(
                                    'predicted_terminal_tilt_deg'
                                )
                            ),
                            'predicted_max_signed_overshoot_m_s': (
                                velocity_mpc_shadow_last_decision.get(
                                    'predicted_max_signed_overshoot_m_s'
                                )
                            ),
                            'hard_terminal_constraints_satisfied': (
                                velocity_mpc_shadow_last_decision.get(
                                    'hard_terminal_constraints_satisfied'
                                )
                            ),
                            'residual_learning': (
                                velocity_mpc_shadow_last_decision.get(
                                    'residual_learning'
                                )
                            ),
                            'state_source': 'crazyflie_state_estimate',
                        },
                    )
                    velocity_mpc_shadow_last_logged_signature = (
                        velocity_mpc_signature
                    )
                    velocity_mpc_shadow_last_log_time = (
                        velocity_mpc_shadow_now
                    )
            if (
                predictive_brake_episode is not None
                and potentiometer_release_processed
            ):
                predictive_decision_time = time.time()
                predictive_runtime_state = {
                    'time_s': state_time,
                    'position_xy': position[:2],
                    'velocity_xy': output.estimate.velocity[:2],
                    'orientation_rpy_rad': output.estimate.orientation_rpy,
                    'angular_velocity_rad_s': state['angular_velocity'],
                    'state_group_skew_s': state_group_skew,
                }
                predictive_brake_decision = (
                    predictive_brake_episode.decide(
                        predictive_decision_time,
                        predictive_runtime_state,
                    )
                )
                predictive_action = predictive_brake_decision['action']
                predictive_signature = (
                    predictive_action,
                    predictive_brake_decision.get('reason'),
                )
                if predictive_signature != predictive_last_logged_signature:
                    self._log_event(
                        'Predictive Brake Decision',
                        {
                            'action': predictive_action,
                            'reason': predictive_brake_decision.get('reason'),
                            'phase': predictive_brake_decision.get('phase'),
                            'elapsed_s': predictive_brake_decision.get(
                                'elapsed_s'
                            ),
                            'selected_directional_model': (
                                predictive_brake_decision.get(
                                    'selected_directional_model'
                                )
                            ),
                            'target_position_m': (
                                predictive_brake_decision.get(
                                    'position_target'
                                )
                            ),
                            'state_source': 'crazyflie_state_estimate',
                        },
                    )
                    predictive_last_logged_signature = predictive_signature
                if predictive_action in ('brake', 'level', 'abort_level'):
                    translation_control.set_contact_attitude(
                        predictive_brake_decision['roll_deg'],
                        predictive_brake_decision['pitch_deg'],
                        predictive_brake_decision['yaw_rate_deg_s'],
                        yaw_deg=np.degrees(
                            output.estimate.orientation_rpy[2]
                        ),
                    )
                    attitude_command_planned_at = predictive_decision_time
                    predictive_brake_abort_after_send = bool(
                        predictive_action == 'abort_level'
                    )
                elif predictive_action == 'position':
                    predictive_target = np.asarray(
                        predictive_brake_decision['position_target'],
                        dtype=float,
                    )
                    translation_control.set_predictive_position_target(
                        self._bounded_wrench_reference(predictive_target),
                        state_time,
                    )
                    if not predictive_position_handoff_logged:
                        self._log_event(
                            'Predictive Brake Position Handoff',
                            {
                                'reason': predictive_brake_decision.get(
                                    'reason'
                                ),
                                'actual_position_m': position.tolist(),
                                'actual_velocity_m_s': (
                                    output.estimate.velocity.tolist()
                                ),
                                'target_position_m': (
                                    translation_control
                                    .hold_position.tolist()
                                ),
                                'target_clamped_to_actual': (
                                    predictive_brake_decision.get(
                                        'target_clamped_to_actual'
                                    )
                                ),
                                'state_source': (
                                    'crazyflie_state_estimate'
                                ),
                            },
                        )
                        predictive_position_handoff_logged = True
                else:
                    raise RuntimeError(
                        'unknown predictive braking action: '
                        + str(predictive_action)
                    )
            # Revalidate the terminal set with this exact measured state before
            # allowing the coast controller to transfer command ownership.
            # The gate deliberately drops out of ``complete`` on any later
            # velocity/tilt/rate/command violation, so using the prior loop's
            # status here would create a one-sample early-handoff race.
            if release_dataset_episode_id is not None:
                if self.bounds is None:
                    release_dataset_boundary_margin_m = float('nan')
                else:
                    release_dataset_boundary_margin_m = float(min(
                        position[0]-self.bounds['x_min'],
                        self.bounds['x_max']-position[0],
                        position[1]-self.bounds['y_min'],
                        self.bounds['y_max']-position[1],
                    ))
                if bootstrap_coverage is not None:
                    bootstrap_coverage.observe(
                        release_dataset_episode_id,
                        velocity_xy_m_s=output.estimate.velocity[:2],
                        attitude_rp_rad=(
                            output.estimate.orientation_rpy[:2]
                        ),
                        attitude_rate_rp_rad_s=(
                            output.estimate.angular_velocity[:2]
                        ),
                        boundary_margin_m=(
                            release_dataset_boundary_margin_m
                        ),
                        state_age_s=state_age,
                        state_group_skew_s=state_group_skew,
                        measurement_rejected=(
                            output.estimate.measurement_rejected
                        ),
                    )
                applied_command_kind = ''
                applied_attitude_rp_rad = (float('nan'), float('nan'))
                if actual_command_applied_at_state is not None:
                    applied_command_kind = str(
                        actual_command_applied_at_state.get('kind', '')
                    )
                    if applied_command_kind == ATTITUDE_ZDISTANCE_COMMAND:
                        try:
                            applied_attitude_rp_rad = tuple(np.radians([
                                actual_command_applied_at_state['roll_deg'],
                                actual_command_applied_at_state['pitch_deg'],
                            ]))
                        except (KeyError, TypeError, ValueError):
                            applied_attitude_rp_rad = (
                                float('nan'), float('nan')
                            )
                pending_dataset_commands = (
                    translation_control.sent_commands_in_window(
                        state_time
                        - release_dataset_effective_command_delay_s,
                        state_observed_at,
                    )
                )
                pending_dataset_command_kinds = tuple(
                    str(command.get('kind', ''))
                    for command in pending_dataset_commands
                )
                pending_dataset_attitudes_rp_rad = []
                for command in pending_dataset_commands:
                    if command.get('kind') == ATTITUDE_ZDISTANCE_COMMAND:
                        try:
                            pending_dataset_attitudes_rp_rad.append(
                                tuple(np.radians([
                                    command['roll_deg'],
                                    command['pitch_deg'],
                                ]))
                            )
                        except (KeyError, TypeError, ValueError):
                            pending_dataset_attitudes_rp_rad.append((
                                float('nan'), float('nan')
                            ))
                    else:
                        pending_dataset_attitudes_rp_rad.append((0.0, 0.0))
                release_dataset_terminal_status = (
                    release_dataset_terminal_gate.update(
                        ReleaseLMPCTerminalSample(
                            state_time_s=state_time,
                            velocity_xy_m_s=tuple(
                                output.estimate.velocity[:2]
                            ),
                            attitude_rp_rad=tuple(
                                output.estimate.orientation_rpy[:2]
                            ),
                            attitude_rate_rp_rad_s=tuple(
                                output.estimate.angular_velocity[:2]
                            ),
                            applied_command_kind=applied_command_kind,
                            applied_attitude_rp_rad=(
                                applied_attitude_rp_rad
                            ),
                            pending_command_kinds=(
                                pending_dataset_command_kinds
                            ),
                            pending_attitude_rp_rad=tuple(
                                pending_dataset_attitudes_rp_rad
                            ),
                            state_age_s=state_age,
                            state_group_skew_s=state_group_skew,
                            boundary_margin_m=(
                                release_dataset_boundary_margin_m
                            ),
                        )
                    )
                )
            if translation_control.mode in (
                    translation_control.ATTITUDE_COAST,
                    translation_control.VELOCITY_COAST,
                    translation_control.POSITION_COAST):
                if (
                    mpc_calibration_mode
                    and mpc_automatic_attempt is not None
                ):
                    automatic_maneuver_displacement_m = float(
                        np.linalg.norm(position[:2]-nominal_position[:2])
                    )
                    automatic_maneuver_boundary_margin_m = (
                        float(min(
                            position[0]-self.bounds['x_min'],
                            self.bounds['x_max']-position[0],
                            position[1]-self.bounds['y_min'],
                            self.bounds['y_max']-position[1],
                        ))
                        if self.bounds is not None else float('-inf')
                    )
                    automatic_aligned_speed_m_s = float(
                        output.estimate.velocity[:2]
                        @ np.asarray(
                            mpc_automatic_attempt.direction_xy, dtype=float
                        )
                    )
                    automatic_cross_speed_m_s = float(
                        output.estimate.velocity[0]
                        * -mpc_automatic_attempt.target_cell.direction_sign
                    )
                    automatic_actual_tilt_rad = float(np.max(np.abs(
                        output.estimate.orientation_rpy[:2]
                    )))
                    automatic_actual_rate_rad_s = float(np.max(np.abs(
                        output.estimate.angular_velocity[:2]
                    )))
                    automatic_safe_set_limits = (
                        release_dataset_terminal_gate.limits
                    )
                    automatic_motion_failures = []
                    if automatic_maneuver_displacement_m > (
                        bootstrap_coverage.config
                        .max_maneuver_displacement_m+1e-12
                    ):
                        automatic_motion_failures.append(
                            'maneuver_displacement_limit_exceeded'
                        )
                    if automatic_maneuver_boundary_margin_m < (
                        bootstrap_coverage.config.boundary_reserve_m-1e-12
                    ):
                        automatic_motion_failures.append(
                            'maneuver_boundary_reserve_exhausted'
                        )
                    if abs(float(position[2]-nominal_position[2])) > (
                        bootstrap_coverage.config.ready_z_tolerance_m+1e-12
                    ):
                        automatic_motion_failures.append(
                            'maneuver_z_error_limit_exceeded'
                        )
                    if abs(automatic_aligned_speed_m_s) > (
                        bootstrap_coverage.config.max_release_speed_m_s+1e-12
                    ):
                        automatic_motion_failures.append(
                            'maneuver_aligned_speed_limit_exceeded'
                        )
                    if automatic_aligned_speed_m_s < (
                        -bootstrap_coverage.config.wrong_way_land_speed_m_s
                        - 1e-12
                    ):
                        automatic_motion_failures.append(
                            'maneuver_wrong_way_speed_limit_exceeded'
                        )
                    if abs(automatic_cross_speed_m_s) > (
                        automatic_safe_set_limits.max_abs_cross_velocity_m_s
                        + 1e-12
                    ):
                        automatic_motion_failures.append(
                            'maneuver_cross_speed_limit_exceeded'
                        )
                    if automatic_actual_tilt_rad > (
                        automatic_safe_set_limits.max_path_tilt_rad+1e-12
                    ):
                        automatic_motion_failures.append(
                            'maneuver_attitude_limit_exceeded'
                        )
                    if automatic_actual_rate_rad_s > (
                        automatic_safe_set_limits.max_path_rate_rad_s+1e-12
                    ):
                        automatic_motion_failures.append(
                            'maneuver_attitude_rate_limit_exceeded'
                        )
                    if automatic_motion_failures:
                        if release_dataset_episode_id is not None:
                            for automatic_failure in automatic_motion_failures:
                                bootstrap_coverage.mark_path_failure(
                                    release_dataset_episode_id,
                                    automatic_failure,
                                )
                        level_mpc_attitude_before_fault(
                            automatic_motion_failures[0],
                            yaw_deg=np.degrees(
                                output.estimate.orientation_rpy[2]
                            ),
                        )
                        self._log_event(
                            'Learning MPC Bootstrap Automatic Safety Abort',
                            {
                                'reasons': automatic_motion_failures,
                                'displacement_from_nominal_m': (
                                    automatic_maneuver_displacement_m
                                ),
                                'boundary_margin_m': (
                                    automatic_maneuver_boundary_margin_m
                                ),
                                'z_error_m': float(
                                    position[2]-nominal_position[2]
                                ),
                                'aligned_speed_m_s': (
                                    automatic_aligned_speed_m_s
                                ),
                                'cross_speed_m_s': automatic_cross_speed_m_s,
                                'actual_tilt_deg': float(np.degrees(
                                    automatic_actual_tilt_rad
                                )),
                                'actual_attitude_rate_deg_s': float(np.degrees(
                                    automatic_actual_rate_rad_s
                                )),
                                'safe_command': 'level_attitude_then_land',
                                'offline_only': True,
                                'lmpc_command_authority': False,
                                'state_source': (
                                    'crazyflie_state_estimate'
                                ),
                            },
                        )
                        raise RuntimeError(
                            'automatic LMPC bootstrap maneuver safety abort: '
                            + ', '.join(automatic_motion_failures)
                        )
                virtual_motion_state = virtual_motion.step(np.zeros(2), dt)
                coast_position = self._bounded_wrench_reference(np.array([
                    virtual_motion_state['position'][0],
                    virtual_motion_state['position'][1],
                    translation_control.hover_z,
                ]))
                predicted_stop_position = coast_position
                if coast_stop_prediction is not None:
                    predicted_stop_position = self._bounded_wrench_reference(
                        np.array([
                            coast_stop_prediction['position'][0],
                            coast_stop_prediction['position'][1],
                            translation_control.hover_z,
                        ])
                    )
                braking_kwargs = {
                    'coast_position': coast_position,
                    'coast_velocity': np.array([
                        virtual_motion_state['velocity'][0],
                        virtual_motion_state['velocity'][1],
                        0.0,
                    ]),
                }
                force_virtual_resistance_xy = (
                    virtual_motion_state['resistance']
                )
                force_virtual_friction_N = (
                    virtual_motion_state['friction_force_N']
                )
                force_virtual_drag_N = virtual_motion_state['drag_force_N']
                attitude_command_planned_at = time.time()
                if (
                    mpc_calibration_mode
                    and mpc_next_decision_deadline_monotonic is not None
                    and translation_control.mode
                    == translation_control.ATTITUDE_COAST
                ):
                    # Finish the delayed-response calculation before the send
                    # deadline, while telling the predictor when its selected
                    # command will actually begin.  Sleeping before this
                    # calculation would add its runtime to every 20 ms command
                    # interval and systematically violate the offline cadence
                    # contract on the Pi.
                    mpc_scheduled_decision_deadline = (
                        mpc_next_decision_deadline_monotonic
                    )
                    attitude_command_planned_at += max(
                        mpc_scheduled_decision_deadline-time.monotonic(), 0.0
                    )
                coast_handoff_completed = False
                if (
                    predictive_brake_episode is None
                    and not (
                        velocity_mpc_online_enabled
                        and velocity_mpc_shadow_episode is not None
                    )
                    and translation_control.mode
                    == translation_control.VELOCITY_COAST
                ):
                    coast_handoff_completed = bool(
                        translation_control.update_coast_velocity(
                            self._bounded_wrench_reference(position),
                            output.estimate.velocity,
                            state_time,
                            output.estimate.orientation_rpy,
                            output.estimate.angular_velocity,
                            allow_position_handoff=(
                                potentiometer_release_processed
                                or mpc_automatic_release_confirmed
                            ),
                            command_timestamp=attitude_command_planned_at,
                        )
                    )
                    coast_state_rejection = (
                        translation_control.consume_coast_state_rejection()
                    )
                    if coast_state_rejection is not None:
                        self._log_event(
                            'Velocity Coast State Sample Rejected',
                            {
                                **coast_state_rejection,
                                'command_held': (
                                    translation_control.command_mode
                                ),
                                'state_source': (
                                    'crazyflie_state_estimate'
                                ),
                            },
                        )
                    if translation_control.consume_velocity_pid_reset_request():
                        direct_level_unwind = bool(
                            translation_control.direct_level_unwind_active
                        )
                        position_unwind = bool(
                            translation_control
                            .coast_velocity_unwind_position_control_enabled
                            and translation_control.uses_position_setpoint
                        )
                        integrator_names = (
                            ('posCtlPid.resetI', 'velCtlPid.resetI')
                            if position_unwind
                            else ('velCtlPid.resetI',)
                        )
                        reset_started_at = time.time()
                        velocity_reset_method = (
                            reset_pid_integrators_without_ack(
                                self.cf, integrator_names
                            )
                        )
                        velocity_reset_elapsed_s = (
                            time.time() - reset_started_at
                        )
                        self._log_event(
                            'Velocity Coast Predictive Unwind Started',
                            {
                                'measured_velocity_m_s': (
                                    output.estimate.velocity.tolist()
                                ),
                                'control_owner': (
                                    'native_position_pid'
                                    if position_unwind
                                    else 'direct_attitude_zdistance'
                                    if direct_level_unwind
                                    else 'native_velocity_pid'
                                ),
                                'position_target_m': (
                                    translation_control.hold_position.tolist()
                                    if position_unwind else None
                                ),
                                'attitude_target_rp_deg': (
                                    [0.0, 0.0]
                                    if direct_level_unwind else None
                                ),
                                'zdistance_target_m': (
                                    translation_control
                                    .velocity_coast_fixed_zdistance_m
                                    if direct_level_unwind else None
                                ),
                                'velocity_target_m_s': (
                                    None
                                    if (
                                        position_unwind
                                        or direct_level_unwind
                                    )
                                    else [
                                        *translation_control
                                        .coast_velocity_command_xy_m_s.tolist(),
                                        0.0,
                                    ]
                                ),
                                'projected_speed_m_s': (
                                    translation_control
                                    .brake_projected_speed_m_s
                                ),
                                'projected_acceleration_m_s2': (
                                    translation_control
                                    .coast_velocity_projected_acceleration_m_s2
                                ),
                                'predicted_terminal_speed_m_s': (
                                    translation_control
                                    .coast_velocity_predicted_unwind_terminal_speed_m_s
                                ),
                                'terminal_speed_target_m_s': (
                                    translation_control
                                    .coast_velocity_unwind_terminal_speed_m_s
                                ),
                                'predicted_next_step_terminal_speed_m_s': (
                                    translation_control
                                    .coast_velocity_predicted_next_step_terminal_speed_m_s
                                ),
                                'dynamic_unwind_threshold_m_s': (
                                    translation_control
                                    .coast_velocity_dynamic_unwind_threshold_m_s
                                ),
                                'dynamic_one_step_guard_m_s': (
                                    translation_control
                                    .coast_velocity_dynamic_unwind_step_guard_m_s
                                ),
                                'unwind_decision_reason': (
                                    translation_control
                                    .coast_velocity_unwind_decision_reason
                                ),
                                'response_horizon_s': (
                                    translation_control
                                    .coast_velocity_unwind_response_horizon_s
                                ),
                                'observed_state_to_decision_latency_s': (
                                    translation_control
                                    .coast_velocity_unwind_observed_decision_latency_s
                                ),
                                'configured_attitude_response_delay_s': (
                                    translation_control
                                    .coast_attitude_response_delay_s
                                ),
                                'configured_command_switch_delay_s': (
                                    translation_control
                                    .coast_velocity_unwind_command_switch_delay_s
                                ),
                                'modeled_total_response_delay_s': (
                                    translation_control
                                    .coast_velocity_unwind_total_response_delay_s
                                ),
                                'release_line_position_target_m': (
                                    None
                                    if translation_control
                                    .coast_velocity_unwind_position_target_m
                                    is None
                                    else translation_control
                                    .coast_velocity_unwind_position_target_m
                                    .tolist()
                                ),
                                'release_line_target_progress_m': (
                                    translation_control
                                    .coast_velocity_unwind_position_progress_m
                                ),
                                'release_line_lateral_error_m': (
                                    translation_control
                                    .coast_velocity_unwind_lateral_error_m
                                ),
                                'integrated_velocity_delta_m_s': (
                                    translation_control
                                    .coast_velocity_unwind_integrated_velocity_delta_m_s
                                ),
                                'raw_integrated_velocity_delta_m_s': (
                                    translation_control
                                    .coast_velocity_unwind_raw_integrated_velocity_delta_m_s
                                ),
                                'tail_calibration_scale': (
                                    translation_control
                                    .coast_velocity_unwind_tail_calibration_scale
                                ),
                                'leveling_duration_s': (
                                    translation_control
                                    .coast_velocity_unwind_leveling_duration_s
                                ),
                                'leveling_rate_deg_s': (
                                    translation_control
                                    .coast_velocity_unwind_leveling_rate_deg_s
                                ),
                                'integration_step_s': (
                                    translation_control
                                    .coast_velocity_unwind_integration_step_s
                                ),
                                'velocity_integrator_reset': True,
                                'position_integrator_reset': position_unwind,
                                'integrator_reset_method': (
                                    velocity_reset_method
                                ),
                                'integrator_reset_elapsed_s': (
                                    velocity_reset_elapsed_s
                                ),
                                'state_source': 'crazyflie_state_estimate',
                            },
                        )
                    if translation_control.consume_velocity_rebrake_request():
                        self._log_event(
                            'Velocity Coast Fast Brake Resumed',
                            {
                                'rebrake_count': (
                                    translation_control
                                    .coast_velocity_rebrake_count
                                ),
                                'measured_velocity_m_s': (
                                    output.estimate.velocity.tolist()
                                ),
                                'xy_speed_m_s': float(np.linalg.norm(
                                    output.estimate.velocity[:2]
                                )),
                                'projected_speed_m_s': (
                                    translation_control
                                    .brake_projected_speed_m_s
                                ),
                                'rebrake_speed_threshold_m_s': (
                                    translation_control
                                    .coast_velocity_rebrake_speed_m_s
                                ),
                                'rebrake_speed_source': (
                                    'projected_interaction_direction'
                                ),
                                'actual_tilt_deg': (
                                    translation_control.coast_actual_tilt_deg
                                ),
                                'state_source': 'crazyflie_state_estimate',
                            },
                        )
                elif (
                    predictive_brake_episode is None
                    and not (
                        velocity_mpc_online_enabled
                        and velocity_mpc_shadow_episode is not None
                    )
                    and translation_control.mode
                    == translation_control.ATTITUDE_COAST
                ):
                    coast_handoff_completed = bool(
                        translation_control.update_coast_attitude(
                            self._bounded_wrench_reference(position),
                            output.estimate.velocity,
                            predicted_stop_position,
                            braking_kwargs['coast_velocity'],
                            state_time,
                            output.estimate.orientation_rpy,
                            allow_position_handoff=(
                                (
                                    potentiometer_release_processed
                                    or mpc_automatic_release_confirmed
                                )
                                and (
                                    not mpc_calibration_mode
                                    or release_dataset_episode_id is None
                                    or release_dataset_terminal_status.complete
                                )
                            ),
                            latch_current_position_on_handoff=(
                                mpc_calibration_mode
                            ),
                            command_timestamp=attitude_command_planned_at,
                        )
                    )
                if coast_handoff_completed:
                    position_integrators_reset = False
                    position_integrator_reset_method = None
                    position_integrator_reset_elapsed_s = None
                    if (
                        translation_control.coast_handoff_reason
                        in (
                            'direct_current_position_handoff',
                            'terminal_current_position_handoff',
                            'velocity_zero_position_handoff',
                            'velocity_predictive_unwind_position_handoff',
                            'velocity_predictive_unwind_attitude_handoff',
                        )
                        and not (
                            translation_control.coast_handoff_reason
                            == 'velocity_predictive_unwind_position_handoff'
                            and translation_control
                            .coast_velocity_unwind_position_control_enabled
                        )
                    ):
                        # Clear both controller integrators without adding two
                        # acknowledged parameter transactions to the handoff
                        # command path. The first staged flight showed a 0.52 s
                        # command/telemetry gap at this exact transition.
                        reset_started_at = time.time()
                        position_integrator_reset_method = (
                            reset_pid_integrators_without_ack(
                                self.cf,
                                (
                                    'posCtlPid.resetI',
                                    'velCtlPid.resetI',
                                ),
                            )
                        )
                        position_integrator_reset_elapsed_s = (
                            time.time() - reset_started_at
                        )
                        position_integrators_reset = True
                    self._log_event(
                        'Coast Position Control Handoff',
                        {
                            'release_dataset_episode_id': (
                                release_dataset_episode_id
                            ),
                            'reason': (
                                translation_control.coast_handoff_reason
                            ),
                            'actual_position_m': position.tolist(),
                            'target_position_m': (
                                translation_control.hold_position.tolist()
                            ),
                            'virtual_target_position_m': (
                                coast_position.tolist()
                            ),
                            'predicted_stop_position_m': (
                                predicted_stop_position.tolist()
                            ),
                            'predicted_stop_duration_s': (
                                None if coast_stop_prediction is None
                                else coast_stop_prediction['duration_s']
                            ),
                            'predicted_stop_reached': (
                                None if coast_stop_prediction is None
                                else coast_stop_prediction['stopped']
                            ),
                            'target_clamped_to_actual': (
                                translation_control
                                .coast_target_clamped_to_actual
                            ),
                            'lateral_target_latched_to_actual': (
                                translation_control
                                .coast_lateral_target_latched_to_actual
                            ),
                            'lateral_speed_m_s': (
                                translation_control.coast_lateral_speed_m_s
                            ),
                            'target_remaining_distance_m': (
                                translation_control
                                .coast_target_remaining_distance_m
                            ),
                            'attitude_response_delay_s': (
                                translation_control
                                .coast_attitude_response_delay_s
                            ),
                            'attitude_time_constant_s': (
                                translation_control
                                .coast_attitude_time_constant_s
                            ),
                            'attitude_acceleration_scale': (
                                translation_control
                                .coast_attitude_acceleration_scale
                            ),
                            'actual_tilt_deg': (
                                translation_control.coast_actual_tilt_deg
                            ),
                            'measured_acceleration_m_s2': (
                                translation_control
                                ._coast_filtered_acceleration_xy.tolist()
                            ),
                            'model_acceleration_from_attitude_m_s2': (
                                translation_control
                                ._coast_model_acceleration_xy.tolist()
                            ),
                            'predicted_level_stop_distance_m': (
                                translation_control
                                .coast_predicted_level_stop_distance_m
                            ),
                            'predicted_level_terminal_speed_m_s': (
                                translation_control
                                .coast_predicted_level_terminal_speed_m_s
                            ),
                            'impulse_safe_deceleration_m_s2': (
                                translation_control
                                .coast_impulse_safe_deceleration_m_s2
                            ),
                            'tail_cancellation_acceleration_m_s2': (
                                translation_control
                                .coast_tail_cancellation_acceleration_m_s2
                            ),
                            'tail_cancellation_signed_acceleration_m_s2': (
                                translation_control
                                .coast_tail_cancellation_signed_acceleration_m_s2
                            ),
                            'tail_terminal_target_speed_m_s': (
                                translation_control
                                .coast_tail_terminal_target_speed_m_s
                            ),
                            'predicted_terminal_after_pulse_m_s': (
                                translation_control
                                .coast_predicted_terminal_after_pulse_m_s
                            ),
                            'command_hold_s': (
                                translation_control.coast_command_hold_s
                            ),
                            'actual_velocity_m_s': (
                                output.estimate.velocity.tolist()
                            ),
                            'handoff_control_already_position': bool(
                                translation_control
                                .coast_velocity_unwind_position_control_enabled
                                and translation_control
                                .coast_velocity_unwind_position_target_m
                                is not None
                            ),
                            'target_velocity_m_s': (
                                None
                                if (
                                    translation_control
                                    .coast_velocity_unwind_position_control_enabled
                                    or translation_control
                                    .coast_velocity_unwind_direct_level_attitude_enabled
                                )
                                else [
                                    *translation_control
                                    .coast_velocity_command_xy_m_s.tolist(),
                                    0.0,
                                ]
                            ),
                            'release_line_position_target_m': (
                                None
                                if translation_control
                                .coast_velocity_unwind_position_target_m
                                is None
                                else translation_control
                                .coast_velocity_unwind_position_target_m
                                .tolist()
                            ),
                            'release_line_target_progress_m': (
                                translation_control
                                .coast_velocity_unwind_position_progress_m
                            ),
                            'release_line_lateral_error_m': (
                                translation_control
                                .coast_velocity_unwind_lateral_error_m
                            ),
                            'virtual_target_velocity_m_s': (
                                braking_kwargs['coast_velocity'].tolist()
                            ),
                            'virtual_position_error_m': (
                                None
                                if translation_control
                                .coast_tracking_position_error_m is None
                                else translation_control
                                .coast_tracking_position_error_m.tolist()
                            ),
                            'virtual_velocity_error_m_s': (
                                None
                                if translation_control
                                .coast_tracking_velocity_error_m_s is None
                                else translation_control
                                .coast_tracking_velocity_error_m_s.tolist()
                            ),
                            'position_pullback_disabled': bool(
                                not translation_control
                                .coast_velocity_unwind_position_control_enabled
                                or translation_control
                                .coast_velocity_unwind_position_target_m
                                is not None
                            ),
                            'tail_neutralization_allowed': True,
                            'response_queue_settled': (
                                translation_control
                                .coast_response_queue_settled
                            ),
                            'response_queue_settle_elapsed_s': (
                                translation_control
                                .coast_response_queue_settle_elapsed_s
                            ),
                            'response_queue_settle_required_s': (
                                translation_control
                                .coast_response_queue_settle_required_s
                            ),
                            'level_handoff_speed_threshold_m_s': (
                                translation_control
                                .coast_level_handoff_speed_m_s
                            ),
                            'level_handoff_delay_s': (
                                translation_control
                                .coast_level_handoff_delay_s
                            ),
                            'velocity_handoff_speed_threshold_m_s': (
                                translation_control
                                .coast_velocity_handoff_speed_m_s
                                if velocity_coast_braking_enabled else None
                            ),
                            'velocity_handoff_position_offset_m': (
                                translation_control
                                .coast_velocity_handoff_position_offset_m
                                if velocity_coast_braking_enabled else None
                            ),
                            'velocity_handoff_interaction_direction_xy': (
                                translation_control.brake_direction[:2]
                                .tolist()
                                if velocity_coast_braking_enabled else None
                            ),
                            'target_offset_from_actual_m': (
                                (
                                    translation_control.hold_position
                                    - translation_control
                                    .coast_handoff_actual_position_m
                                ).tolist()
                            ),
                            'velocity_handoff_max_tilt_deg': (
                                translation_control.coast_handoff_max_tilt_deg
                                if velocity_coast_braking_enabled else None
                            ),
                            'velocity_handoff_max_rate_deg_s': (
                                translation_control
                                .coast_velocity_handoff_max_rate_deg_s
                                if velocity_coast_braking_enabled else None
                            ),
                            'actual_angular_rate_xy_deg_s': (
                                float(np.linalg.norm(np.degrees(
                                    output.estimate.angular_velocity[:2]
                                )))
                                if velocity_coast_braking_enabled else None
                            ),
                            'velocity_phase': (
                                translation_control.coast_velocity_phase
                            ),
                            'velocity_rebrake_count': (
                                translation_control
                                .coast_velocity_rebrake_count
                            ),
                            'velocity_handoff_tilt_ready': (
                                translation_control
                                .coast_velocity_handoff_tilt_ready
                            ),
                            'velocity_handoff_rate_ready': (
                                translation_control
                                .coast_velocity_handoff_rate_ready
                            ),
                            'velocity_handoff_speed_ready': (
                                translation_control
                                .coast_velocity_handoff_speed_ready
                            ),
                            'level_handoff_latched': (
                                translation_control
                                ._coast_level_handoff_latched
                            ),
                            'command_mode': (
                                translation_control.command_mode
                            ),
                            'position_integrators_reset': (
                                position_integrators_reset
                            ),
                            'integrator_reset_method': (
                                position_integrator_reset_method
                            ),
                            'integrator_reset_elapsed_s': (
                                position_integrator_reset_elapsed_s
                            ),
                            'state_source': 'crazyflie_state_estimate',
                        },
                    )

            braking_completed = bool(
                coast_handoff_completed or velocity_mpc_handoff_completed
            )
            if not braking_completed:
                braking_completed = translation_control.update_braking(
                        self._bounded_wrench_reference(position),
                        output.estimate.velocity,
                        state_time,
                        output.estimate.orientation_rpy,
                        current_force=braking_force_world,
                        current_mass_kg=force_current_mass,
                        **braking_kwargs)
            if braking_completed:
                if (
                    release_dataset_episode_id is not None
                    and release_dataset_close_after_row is None
                ):
                    if release_dataset_terminal_status.complete:
                        # A terminal dwell only authorizes the legacy coast
                        # controller to hand off.  Its delayed level-command
                        # queue may still need to settle, so count the sample
                        # only on the loop where position control actually
                        # takes ownership and sends the final command.
                        release_dataset_close_after_row = {
                            'event_name': (
                                'Release Dataset Terminal Dwell Complete'
                            ),
                            'release_dataset_outcome': 'terminal_handoff',
                            'reason': (
                                'full_measured_terminal_dwell_complete'
                            ),
                        }
                    else:
                        release_dataset_close_after_row = {
                            'event_name': 'Release Dataset Episode Closed',
                            'release_dataset_outcome': 'rejected',
                            'reason': (
                                'position_handoff_before_measured_terminal_dwell'
                            ),
                        }
                last_command_position = translation_control.hold_position.copy()
                completed_render_mode = selected_render_mode
                completed_render_relation = render_relation
                release_confirmed = bool(
                    release_mode == 'potentiometer_coast'
                    or contacts is None
                    or not contacts.translation.active
                )
                if release_confirmed:
                    selected_render_mode = None
                    render_relation = None
                self._log_event(
                    'Translation Position Hold Resumed',
                    {
                        'release_dataset_episode_id': (
                            release_dataset_episode_id
                        ),
                        'render_mode': completed_render_mode,
                        'motion_relation': completed_render_relation,
                        'hold_position_m': last_command_position.tolist(),
                        'stopping_position_m': (
                            translation_control.stopping_position_m.tolist()
                        ),
                        'actual_handoff_position_m': (
                            None
                            if translation_control
                            .coast_handoff_actual_position_m is None
                            else translation_control
                            .coast_handoff_actual_position_m.tolist()
                        ),
                        'target_clamped_to_actual': (
                            translation_control
                            .coast_target_clamped_to_actual
                        ),
                        'release_force_N': (
                            translation_control.release_force_N.tolist()
                        ),
                        'release_momentum_kg_m_s': (
                            translation_control.release_momentum_kg_m_s.tolist()
                        ),
                        'release_position_m': (
                            translation_control.release_position_m.tolist()
                        ),
                        'xy_speed_m_s': float(np.linalg.norm(
                            output.estimate.velocity[:2]
                        )),
                        'projected_speed_m_s': (
                            translation_control.brake_projected_speed_m_s
                        ),
                        'brake_completion_reason': (
                            translation_control.brake_completion_reason
                        ),
                        'detector_rearm_delay_s': (
                            translation_control.rearm_delay_s
                        ),
                        'state_source': 'crazyflie_state_estimate',
                    },
                )
                if (
                    mpc_automatic_attempt is not None
                    and mpc_automatic_release_confirmed
                    and release_dataset_episode_id is None
                ):
                    # Do not clear the attempt yet: the handoff's first
                    # current-position command must be sent below before the
                    # return-to-nominal target is installed for the next trial.
                    mpc_automatic_finish_after_handoff_send_reason = (
                        mpc_automatic_attempt.prelude_failure_reasons[0]
                        if mpc_automatic_attempt.prelude_failure_reasons else
                        'automatic_attempt_stopped_without_dataset'
                    )

            if (
                (
                    contact_detection_source == 'potentiometer'
                    or contacts is None
                    or not contacts.translation.active
                )
                and translation_control.consume_detector_rearm(state_time)
            ):
                if (
                    release_dataset_episode_id is not None
                    and release_dataset_close_after_row is None
                ):
                    release_dataset_close_after_row = {
                        'event_name': 'Release Dataset Episode Closed',
                        'release_dataset_outcome': 'rejected',
                        'reason': (
                            'detector_rearm_before_measured_terminal_dwell'
                        ),
                    }
                predictive_brake_episode = None
                predictive_brake_decision = None
                predictive_brake_abort_after_send = False
                velocity_mpc_shadow_episode = None
                velocity_mpc_shadow_direction = None
                velocity_mpc_shadow_previous_state = None
                velocity_mpc_shadow_last_decision = None
                velocity_mpc_terminal_since = None
                pipeline.detector.translation.reset(state_time)
                initial_contact_gate.reset(after_interaction=True)
                if potentiometer_contact_detector is not None:
                    potentiometer_contact_detector.reset()
                if potentiometer_release_detector is not None:
                    potentiometer_release_detector.disarm()
                post_interaction_dwell_required = bool(
                    initial_contact_gate.enabled
                    and initial_contact_gate.apply_after_each_interaction
                )
                self._log_event(
                    (
                        'Translation Contact Detector Rearm Started'
                        if post_interaction_dwell_required else
                        'Translation Contact Detector Rearmed'
                    ),
                    {
                        'rearm_delay_s': translation_control.rearm_delay_s,
                        'requires_stationary_dwell': (
                            post_interaction_dwell_required
                        ),
                        'state_source': 'crazyflie_state_estimate',
                    },
                )
                if not post_interaction_dwell_required:
                    logger.info(
                        'Translation contact detection active after %.2f s '
                        'rearm delay; post-interaction stationary dwell is '
                        'disabled.',
                        translation_control.rearm_delay_s,
                    )

            if interaction_start is not None:
                self._emit_guided_touch_prompts(
                    guided_touch,
                    (
                        calibration_elapsed_s
                        if calibration_mode
                        else time.time() - interaction_start
                    ),
                    'crazyflie_state_estimate',
                )

            baseline_position = nominal_position.copy()
            baseline_yaw = nominal_yaw_deg
            excitation_active = False
            if interaction_start is not None and excitation_config['enabled']:
                excitation_elapsed = (
                    calibration_elapsed_s
                    if calibration_mode
                    else time.time() - interaction_start
                )
                excitation_time = excitation_elapsed - float(
                    excitation_config['start_delay_s']
                )
                excitation_duration = float(excitation_config['duration_s'])
                if 0.0 <= excitation_time < excitation_duration:
                    excitation_active = True
                    baseline_position, baseline_yaw = (
                        self._calibration_excitation_reference(
                            nominal_position, nominal_yaw_deg,
                            excitation_config, excitation_time,
                        )
                    )
                    if not excitation_started:
                        excitation_started = True
                        self._log_event('Wrench Calibration Excitation Started', {
                            'instruction': 'Do not touch the drone during this motion.',
                        })
                elif excitation_started and not excitation_finished:
                    excitation_finished = True
                    self._log_event('Wrench Calibration Excitation Complete')

            if calibration_mode and interaction_start is not None:
                # Also covers a trial at t=0 in the same cycle that startup
                # bias calibration finishes. This is before any trial command.
                calibration_trial_wait = begin_trial_wait(now)
                if calibration_trial_wait is not None:
                    _, label, segment_id, gate, plan = calibration_trial_wait
                    xy_speed = float(np.linalg.norm(output.estimate.velocity[:2]))
                    xy_displacement = float(np.linalg.norm(
                        position[:2] - nominal_position[:2]
                    ))
                    actual_tilt_deg = float(np.linalg.norm(
                        np.degrees(output.estimate.orientation_rpy[:2])
                    ))
                    # Waiting is not permission to exceed the flight envelope.
                    if ((plan is planar_braking_plan and xy_speed > plan.max_xy_speed_m_s)
                            or xy_displacement > plan.max_displacement_m):
                        raise RuntimeError(
                            f'{label} calibration exceeded its safety limit '
                            'while waiting for trial readiness '
                            f'(speed={xy_speed:.3f}m/s, '
                            f'displacement={xy_displacement:.3f}m)'
                        )
                    check_trial_wait(now)
                    wait_elapsed_s = gate.wait_elapsed_s(now)
                    ready = gate.update(
                        segment_id, now, state_time, xy_speed,
                        actual_tilt_deg, xy_displacement,
                    )
                    if ready:
                        self._log_event(label + ' Calibration Trial Ready', {
                            'segment_id': segment_id,
                            'protocol_elapsed_s': calibration_elapsed_s,
                            'wait_elapsed_s': wait_elapsed_s,
                            'xy_speed_m_s': xy_speed,
                            'tilt_deg': actual_tilt_deg,
                            'position_error_m': xy_displacement,
                            'state_source': 'crazyflie_state_estimate',
                        })
                        logger.info('%s calibration trial %s ready after %.2fs '
                                    'at XY speed %.3fm/s and tilt %.2fdeg.',
                                    label, segment_id, wait_elapsed_s,
                                    xy_speed, actual_tilt_deg)
                        calibration_trial_wait = None
                    else:
                        calibration_wait_this_cycle = True

            planar_braking_command = None
            if (calibration_mode and interaction_start is not None
                    and not calibration_wait_this_cycle):
                planar_braking_command = planar_braking_plan.command(
                    calibration_elapsed_s,
                    np.degrees(output.estimate.orientation_rpy[2]),
                )
                if adaptive_braking.enabled:
                    planar_braking_command = adaptive_braking.modify(
                        planar_braking_command, time.time(), {
                            'time_s': state_time,
                            'position_xy': position[:2],
                            'velocity_xy': output.estimate.velocity[:2],
                            'orientation_rpy_rad': output.estimate.orientation_rpy,
                            'angular_velocity_rad_s': state['angular_velocity'],
                            'state_group_skew_s': state_group_skew,
                        }, prediction_calibration.latest_report,
                    )
                phase_marker = (
                    planar_braking_command.segment_id,
                    planar_braking_command.phase,
                )
                if (
                    planar_braking_command.phase != 'waiting'
                    and phase_marker != last_planar_braking_phase
                ):
                    last_planar_braking_phase = phase_marker
                    self._log_event('Planar Braking Calibration Phase', {
                        'segment_id': planar_braking_command.segment_id,
                        'phase': planar_braking_command.phase,
                        'direction_xy': (
                            planar_braking_command.direction_xy.tolist()
                        ),
                        'command_acceleration_xy_m_s2': (
                            planar_braking_command
                            .command_acceleration_xy.tolist()
                        ),
                        'command_roll_deg': planar_braking_command.roll_deg,
                        'command_pitch_deg': planar_braking_command.pitch_deg,
                        'command_tilt_deg': planar_braking_command.tilt_deg,
                        'state_source': 'crazyflie_state_estimate',
                    })

            position_capture_command = None
            if (calibration_mode and interaction_start is not None
                    and not calibration_wait_this_cycle):
                try:
                    position_capture_command = position_capture_plan.command(
                        calibration_elapsed_s,
                        np.degrees(output.estimate.orientation_rpy[2]),
                        position,
                        output.estimate.velocity,
                        output.estimate.orientation_rpy,
                    )
                except ValueError as exc:
                    if planar_attitude_active:
                        self.lo_commander.send_zdistance_setpoint(
                            0.0, 0.0, 0.0, float(nominal_position[2])
                        )
                    self._log_event('Position Capture Calibration Rejected', {
                        'reason': str(exc),
                        'protocol_elapsed_s': calibration_elapsed_s,
                        'state_source': 'crazyflie_state_estimate',
                    })
                    raise
                capture_phase_marker = (
                    position_capture_command.segment_id,
                    position_capture_command.phase,
                )
                if (
                    position_capture_command.phase != 'waiting'
                    and capture_phase_marker != last_position_capture_phase
                ):
                    last_position_capture_phase = capture_phase_marker
                    self._log_event('Position Capture Calibration Phase', {
                        'segment_id': position_capture_command.segment_id,
                        'phase': position_capture_command.phase,
                        'direction_xy': (
                            position_capture_command.direction_xy.tolist()
                        ),
                        'command_roll_deg': position_capture_command.roll_deg,
                        'command_pitch_deg': position_capture_command.pitch_deg,
                        'fixed_target_m': (
                            None
                            if position_capture_command.position_target is None
                            else position_capture_command.position_target.tolist()
                        ),
                        'entry_position_m': position.tolist(),
                        'entry_velocity_m_s': output.estimate.velocity.tolist(),
                        'entry_orientation_rpy_rad': (
                            output.estimate.orientation_rpy.tolist()
                        ),
                        'protocol_elapsed_s': calibration_elapsed_s,
                        'state_source': 'crazyflie_state_estimate',
                    })

            if calibration_mode and excitation_active:
                model_calibration_samples.append((
                    float(state_time),
                    output.expected_linear_acceleration.copy(),
                    output.estimate.velocity.copy(),
                ))

            proposed_position = self._bounded_wrench_reference(
                baseline_position + output.admittance.translation_offset
            )
            proposed_yaw = baseline_yaw + float(
                np.degrees(output.admittance.yaw_offset)
            )
            if (
                planar_braking_command is not None
                and planar_braking_command.active
            ):
                xy_speed = float(np.linalg.norm(output.estimate.velocity[:2]))
                xy_displacement = float(np.linalg.norm(
                    position[:2] - nominal_position[:2]
                ))
                actual_tilt_deg = float(np.degrees(np.arccos(np.clip(
                    np.cos(output.estimate.orientation_rpy[0])
                    * np.cos(output.estimate.orientation_rpy[1]),
                    -1.0,
                    1.0,
                ))))
                trial_start_unsettled = bool(
                    planar_braking_command.phase
                    == 'level_before_acceleration'
                    and (
                        active_planar_braking_command is None
                        or active_planar_braking_command.segment_id
                        != planar_braking_command.segment_id
                        or active_planar_braking_command.phase
                        != 'level_before_acceleration'
                    )
                    and (
                        xy_speed
                        > planar_braking_plan.trial_start_max_xy_speed_m_s
                        or actual_tilt_deg
                        > planar_braking_plan.trial_start_max_tilt_deg
                    )
                )
                if (
                    xy_speed > planar_braking_plan.max_xy_speed_m_s
                    or xy_displacement
                    > planar_braking_plan.max_displacement_m
                    or trial_start_unsettled
                ):
                    self.lo_commander.send_zdistance_setpoint(
                        0.0, 0.0, 0.0, float(nominal_position[2])
                    )
                    raise RuntimeError(
                        'Planar braking calibration exceeded its safety '
                        f'limit (speed={xy_speed:.3f}m/s, '
                        f'displacement={xy_displacement:.3f}m, '
                        f'tilt={actual_tilt_deg:.2f}deg, '
                        f'phase={planar_braking_command.phase})'
                    )
            if (
                position_capture_command is not None
                and position_capture_command.active
            ):
                xy_speed = float(np.linalg.norm(output.estimate.velocity[:2]))
                xy_displacement = float(np.linalg.norm(
                    position[:2] - nominal_position[:2]
                ))
                first_acceleration_command = bool(
                    position_capture_command.phase == 'accelerate'
                    and (
                        active_position_capture_command is None
                        or active_position_capture_command.segment_id
                        != position_capture_command.segment_id
                        or active_position_capture_command.phase != 'accelerate'
                    )
                )
                # Each trial must begin near the same nominal origin. Merely
                # being slow at the preceding capture target is insufficient.
                start_position_tolerance_m = (
                    position_capture_plan.trial_start_max_position_error_m
                )
                invalid_target = False
                target_error = None
                if position_capture_command.position_target is not None:
                    capture_target = np.asarray(
                        position_capture_command.position_target, dtype=float
                    )
                    try:
                        self.check_interaction_boundary(capture_target)
                    except BoundaryExceededError as exc:
                        invalid_target = True
                        target_error = str(exc)
                    invalid_target = bool(
                        invalid_target
                        or np.linalg.norm(
                            capture_target[:2] - nominal_position[:2]
                        ) > position_capture_plan.max_displacement_m
                    )
                if (
                    xy_displacement > position_capture_plan.max_displacement_m
                    or (first_acceleration_command
                        and xy_displacement > start_position_tolerance_m)
                    or invalid_target
                ):
                    self.lo_commander.send_zdistance_setpoint(
                        0.0, 0.0, 0.0, float(nominal_position[2])
                    )
                    failure = (
                        'Position capture calibration exceeded its safety '
                        f'limit (speed={xy_speed:.3f}m/s, '
                        f'displacement={xy_displacement:.3f}m, '
                        f'phase={position_capture_command.phase}, '
                        f'invalid_target={invalid_target}, '
                        f'target_error={target_error})'
                    )
                    self._log_event('Position Capture Calibration Rejected', {
                        'reason': failure,
                        'protocol_elapsed_s': calibration_elapsed_s,
                        'segment_id': position_capture_command.segment_id,
                        'state_source': 'crazyflie_state_estimate',
                    })
                    raise RuntimeError(failure)
            mpc_control_grid_active = bool(
                mpc_calibration_mode
                and (
                    release_dataset_episode_id is not None
                    or (
                        mpc_automatic_attempt is not None
                        and translation_control.mode in (
                            translation_control.MPC_BOOTSTRAP_ACCELERATION,
                            translation_control.ATTITUDE_COAST,
                        )
                    )
                )
            )
            if (
                mpc_control_grid_active
                and mpc_next_decision_deadline_monotonic is not None
            ):
                mpc_scheduled_decision_deadline = (
                    mpc_next_decision_deadline_monotonic
                )
                self._safe_sleep(max(
                    mpc_scheduled_decision_deadline-time.monotonic(), 0.0
                ))
            if mpc_control_grid_active:
                # The state was sampled before the wall-clock pacing wait.
                # Check every decision, including the first immediate send,
                # directly before the sole command sender. A host stall must
                # never turn an otherwise fresh sample into a stale attitude
                # decision that is merely rejected offline.
                mpc_decision_send_check_time = time.time()
                mpc_decision_state_age_at_send_s = (
                    mpc_decision_send_check_time-state_time
                )
                mpc_state_age_limit_s = (
                    release_dataset_terminal_gate.limits.max_state_age_s
                )
                if (
                    not mpc_decision_state_age_is_fresh(
                        state_time,
                        mpc_decision_send_check_time,
                        mpc_state_age_limit_s,
                    )
                ):
                    if release_dataset_episode_id is not None:
                        bootstrap_coverage.mark_path_failure(
                            release_dataset_episode_id,
                            'decision_state_age_path_violation',
                        )
                    level_mpc_attitude_before_fault(
                        'state_stale_before_scheduled_mpc_send',
                        yaw_deg=np.degrees(
                            output.estimate.orientation_rpy[2]
                        ),
                    )
                    self._log_event(
                        'Learning MPC Bootstrap Decision State Rejected',
                        {
                            'release_dataset_episode_id': (
                                release_dataset_episode_id
                            ),
                            'decision_state_age_at_send_s': (
                                mpc_decision_state_age_at_send_s
                            ),
                            'maximum_state_age_s': mpc_state_age_limit_s,
                            'command_sent': False,
                            'reason': 'state_stale_before_scheduled_send',
                            'offline_only': True,
                            'state_source': 'crazyflie_state_estimate',
                        },
                    )
                    raise StaleLocalizationError(
                        'LMPC bootstrap state became stale before its '
                        'scheduled command send '
                        f'({mpc_decision_state_age_at_send_s:.3f}s; '
                        f'limit {mpc_state_age_limit_s:.3f}s)'
                    )

            if calibration_wait_this_cycle:
                command_position = nominal_position.copy()
                command_yaw = nominal_yaw_deg
                translation_control.hold_position = command_position.copy()
                translation_control.yaw_deg = nominal_yaw_deg
                translation_control.send(self.lo_commander)
                # These samples belong to position hold, not the preceding
                # attitude/capture trial's response-identification data.
                active_planar_braking_command = None
                active_planar_braking_command_since = None
                active_position_capture_command = None
                active_position_capture_command_since = None
            elif (
                position_capture_command is not None
                and position_capture_command.active
            ):
                command_sent_at = time.time()
                if position_capture_command.attitude_control:
                    self.lo_commander.send_zdistance_setpoint(
                        position_capture_command.roll_deg,
                        position_capture_command.pitch_deg,
                        0.0,
                        float(nominal_position[2]),
                    )
                    command_position = None
                else:
                    # Capture uses the once-latched target, while settle and
                    # recovery return to nominal. Keep handoff's hold target
                    # identical so duplicate/skew retries cannot silently
                    # replace the capture target with the nominal position.
                    command_position = (
                        nominal_position.copy()
                        if position_capture_command.position_target is None
                        else position_capture_command.position_target.copy()
                    )
                    translation_control.hold_position = command_position.copy()
                    translation_control.yaw_deg = nominal_yaw_deg
                    translation_control.send(self.lo_commander)
                command_yaw = nominal_yaw_deg
                if (
                    active_position_capture_command is None
                    or (
                        active_position_capture_command.segment_id,
                        active_position_capture_command.phase,
                    ) != (
                        position_capture_command.segment_id,
                        position_capture_command.phase,
                    )
                ):
                    active_position_capture_command_since = command_sent_at
                active_position_capture_command = position_capture_command
                active_planar_braking_command = None
                active_planar_braking_command_since = None
            elif (
                planar_braking_command is not None
                and planar_braking_command.attitude_control
            ):
                command_sent_at = time.time()
                self.lo_commander.send_zdistance_setpoint(
                    planar_braking_command.roll_deg,
                    planar_braking_command.pitch_deg,
                    0.0,
                    float(nominal_position[2]),
                )
                adaptive_braking.record_sent(planar_braking_command, command_sent_at)
                command_position = None
                command_yaw = nominal_yaw_deg
                if (
                    active_planar_braking_command is None
                    or (
                        active_planar_braking_command.segment_id,
                        active_planar_braking_command.phase,
                    ) != (
                        planar_braking_command.segment_id,
                        planar_braking_command.phase,
                    )
                ):
                    active_planar_braking_command_since = command_sent_at
                active_planar_braking_command = planar_braking_command
            elif pipeline.shadow_mode or not output.calibrated:
                command_position = baseline_position
                command_yaw = baseline_yaw
                translation_control.hold_position = np.asarray(
                    command_position, dtype=float
                ).copy()
                translation_control.yaw_deg = float(command_yaw)
                translation_control.send(self.lo_commander)
                if (
                    position_capture_command is not None
                    and position_capture_command.phase == 'complete'
                ):
                    active_position_capture_command = None
                    active_position_capture_command_since = None
                if (
                    planar_braking_command is not None
                    and planar_braking_command.active
                ):
                    active_planar_braking_command = planar_braking_command
                    active_planar_braking_command_since = time.time()
                elif (
                    planar_braking_command is not None
                    and planar_braking_command.phase == 'complete'
                ):
                    active_planar_braking_command = None
                    active_planar_braking_command_since = None
            elif not translation_control.uses_position_setpoint:
                command_position = None
                command_yaw = translation_control.yaw_deg
                attitude_sent_at = translation_control.send(
                    self.lo_commander,
                    yaw_deg=np.degrees(output.estimate.orientation_rpy[2]),
                )
                if velocity_mpc_shadow_episode is not None:
                    try:
                        sent_attitude_history = (
                            translation_control
                            .sent_attitude_acceleration_history()
                        )
                        if not sent_attitude_history:
                            raise ValueError(
                                'actual attitude send was not recorded'
                            )
                        shadow_sent_time, shadow_sent_acceleration = (
                            sent_attitude_history[-1]
                        )
                        shadow_sent_tilt = (
                            projected_tilt_from_world_acceleration(
                                shadow_sent_acceleration,
                                velocity_mpc_shadow_direction,
                            )
                        )
                        velocity_mpc_shadow_episode.record_sent_command(
                            shadow_sent_time, shadow_sent_tilt
                        )
                    except (TypeError, ValueError) as error:
                        self._log_event(
                            (
                                'Learning Velocity MPC Online Fallback'
                                if velocity_mpc_online_enabled else
                                'Learning Velocity MPC Shadow Stopped'
                            ),
                            {
                                'reason': str(error),
                                'offline_only': (
                                    not velocity_mpc_online_enabled
                                ),
                                'command_authority': False,
                                'fallback': 'legacy_coast_controller',
                                'actual_flight_controller_unchanged': (
                                    not velocity_mpc_online_enabled
                                ),
                                'state_source': (
                                    'crazyflie_state_estimate'
                                ),
                            },
                        )
                        logger.warning(
                            'Learning velocity MPC command-history update '
                            'failed: %s; returning to legacy coast control.',
                            error,
                        )
                        velocity_mpc_shadow_episode = None
                        velocity_mpc_terminal_since = None
                if (
                    predictive_brake_episode is not None
                    and predictive_brake_decision is not None
                    and predictive_brake_decision.get('action') in (
                        'brake', 'level', 'abort_level'
                    )
                ):
                    try:
                        predictive_send_accepted = (
                            predictive_brake_episode.record_sent(
                                predictive_brake_decision,
                                attitude_sent_at,
                            )
                        )
                    except (TypeError, ValueError) as error:
                        predictive_send_accepted = False
                        logger.warning(
                            'Predictive brake command-history update failed: '
                            '%s; returning to legacy coast control.', error,
                        )
                    if (
                        not predictive_send_accepted
                        or predictive_brake_abort_after_send
                    ):
                        self._log_event(
                            'Predictive Brake Fallback',
                            {
                                'reason': (
                                    predictive_brake_decision.get('reason')
                                    if predictive_brake_abort_after_send
                                    else 'sent_command_protocol_rejected'
                                ),
                                'safe_command_sent': 'level_attitude',
                                'fallback': 'legacy_coast_controller',
                                'state_source': (
                                    'crazyflie_state_estimate'
                                ),
                            },
                        )
                        predictive_brake_episode = None
                        predictive_brake_decision = None
                        predictive_brake_abort_after_send = False
            else:
                if velocity_mpc_shadow_episode is not None:
                    self._log_event(
                        (
                            'Learning Velocity MPC Online Fallback'
                            if velocity_mpc_online_enabled else
                            'Learning Velocity MPC Shadow Stopped'
                        ),
                        {
                            'reason': 'actual_controller_entered_position_mode',
                            'offline_only': not velocity_mpc_online_enabled,
                            'command_authority': False,
                            'fallback': 'position_controller',
                            'actual_flight_controller_unchanged': (
                                not velocity_mpc_online_enabled
                            ),
                            'state_source': 'crazyflie_state_estimate',
                        },
                    )
                    velocity_mpc_shadow_episode = None
                    velocity_mpc_terminal_since = None
                command_position = translation_control.hold_position.copy()
                command_yaw = translation_control.yaw_deg
                last_command_position = command_position.copy()
                last_command_yaw = float(command_yaw)
                translation_control.send(self.lo_commander)
                if (
                    mpc_automatic_finish_after_handoff_send_reason is not None
                ):
                    safe_handoff_command = (
                        translation_control.sent_command_snapshot()
                    )
                    if (
                        safe_handoff_command is None
                        or safe_handoff_command.get('kind') != 'position'
                    ):
                        raise RuntimeError(
                            'automatic LMPC safe-stop handoff produced no '
                            'position command'
                        )
                    finish_mpc_automatic_attempt(
                        counted=False,
                        reason=(
                            mpc_automatic_finish_after_handoff_send_reason
                        ),
                    )

            if mpc_control_grid_active:
                mpc_command = translation_control.sent_command_snapshot()
                if (
                    mpc_command is None
                    or mpc_command['sequence']
                    <= release_dataset_last_logged_command_sequence
                ):
                    raise RuntimeError(
                        'LMPC decision epoch produced no actual command send'
                    )
                mpc_sent_at = float(mpc_command['sent_at'])
                mpc_sent_monotonic = time.monotonic()
                mpc_decision_state_age_at_send_s = mpc_sent_at-state_time
                if (
                    not mpc_decision_state_age_is_fresh(
                        state_time,
                        mpc_sent_at,
                        release_dataset_terminal_gate.limits.max_state_age_s,
                    )
                ):
                    if release_dataset_episode_id is not None:
                        bootstrap_coverage.mark_path_failure(
                            release_dataset_episode_id,
                            'decision_state_age_path_violation',
                        )
                    level_mpc_attitude_before_fault(
                        'command_send_completed_with_stale_mpc_state',
                        yaw_deg=np.degrees(
                            output.estimate.orientation_rpy[2]
                        ),
                    )
                    raise StaleLocalizationError(
                        'LMPC bootstrap command send completed outside the '
                        'fresh-state window'
                    )
                if mpc_last_decision_send_monotonic is not None:
                    mpc_send_interval_s = (
                        mpc_sent_monotonic
                        - mpc_last_decision_send_monotonic
                    )
                    mpc_send_error_s = (
                        mpc_send_interval_s
                        - bootstrap_coverage.config.prediction_step_s
                    )
                    if abs(mpc_send_error_s) > (
                        release_dataset_terminal_gate.limits
                        .sample_step_tolerance_s+1e-12
                    ):
                        if release_dataset_episode_id is not None:
                            bootstrap_coverage.mark_path_failure(
                                release_dataset_episode_id,
                                'decision_command_cadence_path_violation',
                            )
                        self._log_event(
                            'Learning MPC Bootstrap Command Cadence Rejected',
                            {
                                'release_dataset_episode_id': (
                                    release_dataset_episode_id
                                ),
                                'actual_interval_s': mpc_send_interval_s,
                                'prediction_step_s': (
                                    bootstrap_coverage.config
                                    .prediction_step_s
                                ),
                                'interval_error_s': mpc_send_error_s,
                                'scheduled_deadline_monotonic_s': (
                                    mpc_scheduled_decision_deadline
                                ),
                                'decision_state_age_at_send_s': (
                                    mpc_decision_state_age_at_send_s
                                ),
                                'offline_only': True,
                                'state_source': (
                                    'crazyflie_state_estimate'
                                ),
                            },
                        )
                        if release_dataset_episode_id is None:
                            level_mpc_attitude_before_fault(
                                'automatic_prelude_command_cadence_violation',
                                yaw_deg=np.degrees(
                                    output.estimate.orientation_rpy[2]
                                ),
                            )
                            raise RuntimeError(
                                'automatic LMPC prelude command cadence '
                                'violated its 50 Hz grid'
                            )
                mpc_last_decision_send_monotonic = mpc_sent_monotonic
                mpc_next_decision_deadline_monotonic = (
                    mpc_sent_monotonic
                    + bootstrap_coverage.config.prediction_step_s
                )

            if prediction_calibration is not None:
                # Only a fully ended attitude trial is eligible. Send the
                # recovery POSITION command above before copying/enqueuing it;
                # optimization never extends an open-loop attitude phase.
                if (not calibration_wait_this_cycle
                        and planar_braking_command is not None
                        and planar_braking_command.phase == 'recovery'
                        and command_position is not None
                        and planar_braking_command.segment_id
                        not in prediction_submitted_segments):
                    completed_id = planar_braking_command.segment_id
                    completed_samples = [sample for sample in planar_braking_samples
                                         if sample['segment_id'] == completed_id]
                    accepted = prediction_calibration.submit_trial(completed_samples)
                    prediction_submitted_segments.add(completed_id)
                    self._log_event('Online Prediction Trial Submitted', {
                        'segment_id': completed_id,
                        'sample_count': len(completed_samples),
                        'accepted': bool(accepted),
                        'recovery_position_command_sent': True,
                    })
                if (len(prediction_submitted_segments)
                        == len(planar_braking_plan.trial_directions)
                        and not prediction_finish_requested):
                    prediction_finish_requested = prediction_calibration.request_finish()
                poll_prediction_calibration(prediction_calibration, self._log_event)

            estimate = output.estimate
            raw = output.raw_estimate
            actual_commands_sent_since_previous_state = (
                translation_control.sent_commands_after_sequence(
                    release_dataset_last_logged_command_sequence
                )
            )
            if (
                release_dataset_episode_id is not None
                and release_dataset_close_after_row is not None
                and release_dataset_close_after_row[
                    'release_dataset_outcome'
                ] == 'terminal_handoff'
            ):
                commands_sent_after_state = [
                    command
                    for command in actual_commands_sent_since_previous_state
                    if command['sent_at'] >= state_observed_at-1e-12
                ]

                post_terminal_command_state = (
                    classify_terminal_post_state_commands(
                        commands_sent_after_state,
                        translation_control.mode,
                        release_dataset_terminal_gate.limits,
                        terminal_position_m=position.tolist(),
                    )
                )
                if post_terminal_command_state == 'position_handoff':
                    pass
                elif post_terminal_command_state == 'level_attitude_hold':
                    # The measured terminal set is ready, but a successful
                    # iteration ends only when the terminal position
                    # controller actually takes ownership.
                    release_dataset_close_after_row = None
                else:
                    release_dataset_close_after_row = {
                        'event_name': 'Release Dataset Episode Closed',
                        'release_dataset_outcome': 'rejected',
                        'reason': (
                            'unsafe_command_sent_after_terminal_state'
                        ),
                    }
            calibration_attitude_command = (
                position_capture_command
                if (position_capture_command is not None
                    and position_capture_command.attitude_control)
                else (
                    planar_braking_command
                    if (planar_braking_command is not None
                        and planar_braking_command.attitude_control)
                    else None
                )
            )
            self.log_manager.add_log_entry('wrench_observer', {
                'time': now,
                'state_source': 'crazyflie_state_estimate',
                'state_time': state_time,
                'state_age_s': state_age,
                'state_group_skew_s': state_group_skew,
                'mpc_automatic_phase': (
                    None
                    if mpc_automatic_attempt is None else
                    mpc_automatic_attempt.phase
                ),
                'mpc_automatic_target_cell': (
                    None
                    if mpc_automatic_attempt is None else
                    mpc_automatic_attempt.target_cell.to_dict()
                ),
                'mpc_automatic_decision': (
                    None
                    if mpc_automatic_decision is None else
                    mpc_automatic_decision.to_dict()
                ),
                'release_dataset_episode_id': release_dataset_episode_id,
                'release_dataset_direction_xy': (
                    None
                    if release_dataset_direction_xy is None else
                    release_dataset_direction_xy.tolist()
                ),
                'release_dataset_command_owner': (
                    translation_control.command_mode
                    if release_dataset_episode_id is not None else None
                ),
                'release_dataset_terminal_gate': (
                    release_dataset_terminal_status.to_dict()
                    if release_dataset_episode_id is not None else None
                ),
                'release_dataset_pending_outcome': (
                    None
                    if release_dataset_close_after_row is None else
                    release_dataset_close_after_row[
                        'release_dataset_outcome'
                    ]
                ),
                'actual_command_applied_at_state': (
                    actual_command_applied_at_state
                    if release_dataset_episode_id is not None else None
                ),
                'actual_commands_sent_since_previous_state': (
                    actual_commands_sent_since_previous_state
                    if release_dataset_episode_id is not None else None
                ),
                'release_dataset_decision_state_age_at_send_s': (
                    mpc_decision_state_age_at_send_s
                    if release_dataset_episode_id is not None else None
                ),
                'xy_boundary_margin_m': (
                    None
                    if self.bounds is None else float(min(
                        position[0]-self.bounds['x_min'],
                        self.bounds['x_max']-position[0],
                        position[1]-self.bounds['y_min'],
                        self.bounds['y_max']-position[1],
                    ))
                ),
                # Backward-compatible analyzer aliases.
                'frame_time': state_time,
                'frame_age_s': state_age,
                'motor_pose_skew_s': state['motor_skew_s'],
                'position_m': position.tolist(),
                'orientation_rpy_rad': estimate.orientation_rpy.tolist(),
                'velocity_m_s': estimate.velocity.tolist(),
                'angular_velocity_rad_s': estimate.angular_velocity.tolist(),
                'controller_yaw_command': state['yaw_control_command'],
                'controller_yaw_rate_rad_s': state['controller_yaw_rate'],
                'controller_yaw_skew_s': state['yaw_control_skew_s'],
                'expected_linear_acceleration_m_s2': output.expected_linear_acceleration.tolist(),
                'aligned_expected_linear_acceleration_m_s2': (
                    pipeline.last_aligned_expected_linear_acceleration.tolist()
                ),
                'model_alignment_ready': pipeline.last_model_alignment_ready,
                'model_delay_s': pipeline.config['impulse_estimator']['model_delay_s'],
                'model_time_constant_s': (
                    pipeline.config['impulse_estimator']['model_time_constant_s']
                ),
                'model_acceleration_scale': (
                    pipeline.config['impulse_estimator'][
                        'model_acceleration_scale'
                    ]
                ),
                'expected_angular_acceleration_rad_s2': output.expected_angular_acceleration.tolist(),
                'raw_external_force_N': raw.external_force.tolist(),
                'recursive_external_force_N': (
                    pipeline.last_recursive_external_force.tolist()
                ),
                'external_impulse_N_s': pipeline.last_external_impulse.tolist(),
                'impulse_window_s': pipeline.last_impulse_window_s,
                'impulse_estimate_ready': pipeline.last_impulse_ready,
                'raw_external_torque_Nm': raw.external_torque.tolist(),
                'force_bias_N': pipeline.force_bias.tolist(),
                'torque_bias_Nm': pipeline.torque_bias.tolist(),
                'external_force_N': estimate.external_force.tolist(),
                **sensor_fields,
                'force_control_source': force_control_source,
                'control_external_force_N': control_force_world.tolist(),
                'release_braking_force_source': braking_force_source,
                'release_braking_external_force_N': (
                    braking_force_world.tolist()
                ),
                'force_rendering_enabled': force_rendering_enabled,
                'contact_detection_source': contact_detection_source,
                'release_behavior_mode': release_mode,
                'coast_control_policy': (
                    (
                        (
                            (
                                'direct_level_attitude_unwind_then_position'
                                if velocity_unwind_direct_level_attitude_enabled
                                else 'release_line_position_unwind'
                                if velocity_unwind_position_control_enabled
                                else 'predictive_velocity_unwind_then_position'
                            )
                            if velocity_predictive_unwind_enabled
                            else 'zero_world_velocity_then_position'
                        )
                        if velocity_coast_braking_enabled
                        else (
                            'predictive_model_brake_to_position'
                            if predictive_braking_available
                            else 'target_aware_no_pullback'
                        )
                    )
                    if release_mode == 'potentiometer_coast' else None
                ),
                'predictive_braking_enabled': predictive_braking_enabled,
                'predictive_braking_model_available': (
                    predictive_braking_available
                ),
                'predictive_braking_active': (
                    predictive_brake_episode is not None
                ),
                'predictive_braking_action': (
                    None if predictive_brake_decision is None
                    else predictive_brake_decision.get('action')
                ),
                'predictive_braking_reason': (
                    None if predictive_brake_decision is None
                    else predictive_brake_decision.get('reason')
                ),
                'predictive_braking_position_target_m': (
                    None if predictive_brake_decision is None
                    else predictive_brake_decision.get('position_target')
                ),
                'initial_contact_detector_armed': (
                    initial_contact_gate.armed
                ),
                'initial_contact_xy_speed_m_s': (
                    initial_contact_gate.xy_speed_m_s
                ),
                'initial_contact_stationary_elapsed_s': (
                    initial_contact_gate.stationary_elapsed_s
                ),
                'potentiometer_release_detected': (
                    None
                    if potentiometer_release_decision is None
                    else potentiometer_release_decision.released
                ),
                'potentiometer_release_candidate_active': (
                    None
                    if potentiometer_release_decision is None
                    else potentiometer_release_decision.candidate_active
                ),
                'potentiometer_release_candidate_braking_active': (
                    potentiometer_release_pending
                    and translation_control.braking_mode
                ),
                'potentiometer_release_candidate_pending': (
                    potentiometer_release_pending
                ),
                'potentiometer_release_candidate_render_scale': (
                    release_candidate_render_scale
                ),
                'potentiometer_release_candidate_attitude_action': (
                    translation_control.release_candidate_action
                ),
                'potentiometer_release_candidate_predicted_level_terminal_speed_m_s': (
                    translation_control
                    .release_candidate_predicted_level_terminal_speed_m_s
                ),
                'potentiometer_release_candidate_tail_cancellation_acceleration_m_s2': (
                    translation_control
                    .release_candidate_tail_cancellation_acceleration_m_s2
                ),
                'potentiometer_release_candidate_tail_cancellation_signed_acceleration_m_s2': (
                    translation_control
                    .release_candidate_tail_cancellation_signed_acceleration_m_s2
                ),
                'potentiometer_release_candidate_target_terminal_speed_m_s': (
                    translation_control
                    .release_candidate_target_terminal_speed_m_s
                ),
                'potentiometer_release_candidate_predicted_terminal_after_pulse_m_s': (
                    translation_control
                    .release_candidate_predicted_terminal_after_pulse_m_s
                ),
                'potentiometer_release_candidate_command_hold_s': (
                    translation_control.release_candidate_command_hold_s
                ),
                'potentiometer_release_candidate_lead_drop_N': (
                    release_candidate_lead_drop_n
                    if release_candidate_lead_drop_n is not None
                    else release_force_drop_n
                ),
                'potentiometer_release_unloaded_elapsed_s': (
                    None
                    if potentiometer_release_decision is None
                    else potentiometer_release_decision.unloaded_elapsed_s
                ),
                'potentiometer_release_unloaded_force_threshold_N': (
                    release_unloaded_force_n
                    if potentiometer_release_detector is not None else None
                ),
                'potentiometer_release_unloaded_dwell_s': (
                    release_unloaded_dwell_s
                    if potentiometer_release_detector is not None else None
                ),
                'potentiometer_release_candidate_elapsed_s': (
                    None
                    if potentiometer_release_decision is None
                    else potentiometer_release_decision.candidate_elapsed_s
                ),
                'potentiometer_release_candidate_cancel_reason': (
                    None
                    if potentiometer_release_decision is None
                    else (
                        potentiometer_release_decision
                        .candidate_cancel_reason
                    )
                ),
                'potentiometer_release_candidate_stall_timeout_s': (
                    release_candidate_stall_timeout_s
                    if potentiometer_release_detector is not None else None
                ),
                'potentiometer_release_candidate_sensor_stale_timeout_s': (
                    release_candidate_sensor_stale_timeout_s
                    if potentiometer_release_detector is not None else None
                ),
                'potentiometer_release_candidate_sensor_stale_elapsed_s': (
                    None
                    if candidate_release_sensor_stale_since is None
                    else max(now - candidate_release_sensor_stale_since, 0.0)
                ),
                'potentiometer_force_rate_N_s': (
                    None
                    if potentiometer_release_decision is None
                    else potentiometer_release_decision.force_rate_n_s
                ),
                'potentiometer_force_drop_N': (
                    None
                    if potentiometer_release_decision is None
                    else potentiometer_release_decision.force_drop_n
                ),
                'potentiometer_contact_ready': (
                    None
                    if potentiometer_contact_detector is None
                    else potentiometer_contact_detector.ready
                ),
                'potentiometer_contact_active': (
                    None
                    if potentiometer_contact_detector is None
                    else potentiometer_contact_detector.active
                ),
                'potentiometer_contact_force_threshold_N': (
                    potentiometer_contact_force_n
                    if potentiometer_contact_detector is not None else None
                ),
                'external_torque_Nm': estimate.external_torque.tolist(),
                'force_covariance': estimate.force_covariance.tolist(),
                'torque_covariance': estimate.torque_covariance.tolist(),
                'linear_momentum_error_kg_m_s': raw.position_innovation.tolist(),
                'angular_momentum_error_kg_m2_s': raw.orientation_innovation.tolist(),
                'no_contact_predicted_velocity_m_s': (
                    pipeline.last_no_contact_predicted_velocity.tolist()
                ),
                'momentum_prediction_input': (
                    'finite_window_time_aligned_actuator_states_without_external_force'
                ),
                'measurement_rejected': bool(estimate.measurement_rejected),
                'motor_data_available': bool(output.motor_data_available),
                'battery_data_available': bool(battery_available),
                'motor_data_age_s': motor_age,
                'motor_pwm': motor_pwm,
                'battery_voltage_V': battery_voltage,
                'calibrated': bool(output.calibrated),
                'calibration_samples': output.calibration_samples,
                'translation_contact': self._contact_log(
                    contacts.translation if contacts else None
                ),
                'yaw_contact': self._contact_log(
                    contacts.yaw if contacts else None
                ),
                'translation_offset_m': output.admittance.translation_offset.tolist(),
                'translation_reference_velocity_m_s': output.admittance.translation_velocity.tolist(),
                'yaw_offset_rad': output.admittance.yaw_offset,
                'yaw_reference_rate_rad_s': output.admittance.yaw_rate,
                'baseline_position_m': baseline_position.tolist(),
                'baseline_yaw_deg': baseline_yaw,
                'calibration_excitation_active': excitation_active,
                'planar_braking_calibration_active': bool(
                    planar_braking_command is not None
                    and planar_braking_command.attitude_control
                ),
                'planar_braking_calibration_phase': (
                    None
                    if planar_braking_command is None
                    else planar_braking_command.phase
                ),
                'planar_braking_calibration_segment_id': (
                    None
                    if planar_braking_command is None
                    else planar_braking_command.segment_id
                ),
                'planar_braking_command_acceleration_xy_m_s2': (
                    None
                    if planar_braking_command is None
                    else planar_braking_command
                    .command_acceleration_xy.tolist()
                ),
                'position_capture_calibration_active': bool(
                    position_capture_command is not None
                    and position_capture_command.active
                ),
                'position_capture_calibration_phase': (
                    None if position_capture_command is None
                    else position_capture_command.phase
                ),
                'position_capture_calibration_segment_id': (
                    None if position_capture_command is None
                    else position_capture_command.segment_id
                ),
                'position_capture_fixed_target_m': (
                    None
                    if (position_capture_command is None
                        or position_capture_command.position_target is None)
                    else position_capture_command.position_target.tolist()
                ),
                'position_capture_command_started_at': (
                    active_position_capture_command_since
                ),
                'calibration_protocol_elapsed_s': (
                    calibration_elapsed_s if calibration_mode else None
                ),
                'calibration_protocol_state_step_s': (
                    protocol_state_step_s if calibration_mode else None
                ),
                'calibration_trial_waiting': calibration_wait_this_cycle,
                'calibration_trial_wait_stage': (
                    None if calibration_trial_wait is None
                    else calibration_trial_wait[1]
                ),
                'calibration_trial_wait_segment_id': (
                    None if calibration_trial_wait is None
                    else calibration_trial_wait[2]
                ),
                'calibration_intentional_wait_s': (
                    sum(gate.total_wait_s(now)
                        for gate in calibration_trial_gates)
                    if calibration_mode else None
                ),
                'proposed_position_m': proposed_position.tolist(),
                'proposed_yaw_deg': proposed_yaw,
                'command_mode': (
                    ('position_capture_attitude_calibration'
                     if position_capture_command.attitude_control
                     else 'position_capture_calibration')
                    if (position_capture_command is not None
                        and position_capture_command.active)
                    else translation_control.command_mode
                ),
                'command_position_m': (
                    None if command_position is None
                    else np.asarray(command_position, dtype=float).tolist()
                ),
                'command_zdistance_m': (
                    float(nominal_position[2])
                    if calibration_attitude_command is not None
                    else (
                        translation_control.hover_z
                        if (
                            not translation_control.uses_position_setpoint
                            and (
                                translation_control.mode
                                != translation_control.VELOCITY_COAST
                                or translation_control
                                .direct_level_unwind_active
                            )
                        )
                        else None
                    )
                ),
                'command_roll_deg': (
                    calibration_attitude_command.roll_deg
                    if calibration_attitude_command is not None
                    else (
                        translation_control.contact_roll_deg
                        if (
                            not translation_control.uses_position_setpoint
                            and (
                                translation_control.mode
                                != translation_control.VELOCITY_COAST
                                or translation_control
                                .direct_level_unwind_active
                            )
                        )
                        else None
                    )
                ),
                'command_pitch_deg': (
                    calibration_attitude_command.pitch_deg
                    if calibration_attitude_command is not None
                    else (
                        translation_control.contact_pitch_deg
                        if (
                            not translation_control.uses_position_setpoint
                            and (
                                translation_control.mode
                                != translation_control.VELOCITY_COAST
                                or translation_control
                                .direct_level_unwind_active
                            )
                        )
                        else None
                    )
                ),
                'brake_projected_speed_m_s': (
                    translation_control.brake_projected_speed_m_s
                    if translation_control.braking_mode else None
                ),
                'brake_command_tilt_deg': (
                    translation_control.brake_command_tilt_deg
                    if translation_control.braking_mode else None
                ),
                'brake_force_feedforward_acceleration_m_s2': (
                    translation_control.brake_force_feedforward_acceleration_m_s2
                    if translation_control.braking_mode else None
                ),
                'release_force_N': (
                    translation_control.release_force_N.tolist()
                    if translation_control.release_position_m is not None else None
                ),
                'release_momentum_kg_m_s': (
                    None
                    if translation_control.release_momentum_kg_m_s is None
                    else translation_control.release_momentum_kg_m_s.tolist()
                ),
                'release_position_m': (
                    None
                    if translation_control.release_position_m is None
                    else translation_control.release_position_m.tolist()
                ),
                'stopping_position_m': (
                    None
                    if translation_control.stopping_position_m is None
                    else translation_control.stopping_position_m.tolist()
                ),
                'command_xy_velocity_m_s': (
                    translation_control
                    .coast_velocity_command_xy_m_s.tolist()
                    if (
                        translation_control.mode
                        == translation_control.VELOCITY_COAST
                        and not translation_control.uses_position_setpoint
                        and not translation_control.direct_level_unwind_active
                    )
                    else None
                ),
                'command_xy_velocity_world_m_s': (
                    translation_control
                    .coast_velocity_command_xy_m_s.tolist()
                    if (
                        translation_control.mode
                        == translation_control.VELOCITY_COAST
                        and not translation_control.uses_position_setpoint
                        and not translation_control.direct_level_unwind_active
                    )
                    else None
                ),
                'command_yaw_deg': float(command_yaw),
                'preferred_render_mode': preferred_render_mode,
                'selected_render_mode': selected_render_mode,
                'virtual_motion_relation': render_relation,
                'force_orientation_enabled': (
                    force_rendering_enabled
                    and translation_control.attitude_mode
                ),
                'coast_initial_velocity_m_s': (
                    None
                    if coast_initial_velocity is None
                    else coast_initial_velocity.tolist()
                ),
                'coast_tracking_action': (
                    translation_control.coast_tracking_action
                ),
                'coast_tracking_position_error_m': (
                    None
                    if translation_control.coast_tracking_position_error_m
                    is None
                    else translation_control
                    .coast_tracking_position_error_m.tolist()
                ),
                'coast_tracking_velocity_error_m_s': (
                    None
                    if translation_control.coast_tracking_velocity_error_m_s
                    is None
                    else translation_control
                    .coast_tracking_velocity_error_m_s.tolist()
                ),
                'coast_tracking_acceleration_m_s2': (
                    None
                    if translation_control.coast_tracking_acceleration_m_s2
                    is None
                    else translation_control
                    .coast_tracking_acceleration_m_s2.tolist()
                ),
                'coast_tracking_acceleration_saturated': (
                    translation_control.coast_tracking_acceleration_saturated
                ),
                'coast_tracking_power_W_per_kg': (
                    translation_control.coast_tracking_power_w_per_kg
                ),
                'coast_stop_target_position_m': (
                    None
                    if translation_control.coast_stop_target_position_m is None
                    else translation_control
                    .coast_stop_target_position_m.tolist()
                ),
                'coast_handoff_actual_position_m': (
                    None
                    if translation_control.coast_handoff_actual_position_m
                    is None
                    else translation_control
                    .coast_handoff_actual_position_m.tolist()
                ),
                'coast_velocity_handoff_position_offset_m': (
                    translation_control
                    .coast_velocity_handoff_position_offset_m
                ),
                'coast_target_clamped_to_actual': (
                    translation_control.coast_target_clamped_to_actual
                ),
                'coast_lateral_target_latched_to_actual': (
                    translation_control
                    .coast_lateral_target_latched_to_actual
                ),
                'coast_lateral_speed_m_s': (
                    translation_control.coast_lateral_speed_m_s
                ),
                'coast_target_remaining_distance_m': (
                    translation_control.coast_target_remaining_distance_m
                ),
                'coast_delay_reserved_distance_m': (
                    translation_control.coast_delay_reserved_distance_m
                ),
                'coast_required_deceleration_m_s2': (
                    translation_control.coast_required_deceleration_m_s2
                ),
                'coast_measured_deceleration_m_s2': (
                    translation_control.coast_measured_deceleration_m_s2
                ),
                'coast_predicted_forward_speed_after_delay_m_s': (
                    translation_control
                    .coast_predicted_forward_speed_after_delay_m_s
                ),
                'coast_attitude_response_delay_s': (
                    translation_control.coast_attitude_response_delay_s
                ),
                'coast_velocity_unwind_observed_decision_latency_s': (
                    translation_control
                    .coast_velocity_unwind_observed_decision_latency_s
                ),
                'coast_velocity_unwind_command_switch_delay_s': (
                    translation_control
                    .coast_velocity_unwind_command_switch_delay_s
                ),
                'coast_velocity_unwind_total_response_delay_s': (
                    translation_control
                    .coast_velocity_unwind_total_response_delay_s
                ),
                'coast_velocity_unwind_position_control_enabled': (
                    translation_control
                    .coast_velocity_unwind_position_control_enabled
                ),
                'coast_velocity_unwind_direct_level_attitude_enabled': (
                    translation_control
                    .coast_velocity_unwind_direct_level_attitude_enabled
                ),
                'coast_velocity_unwind_position_target_m': (
                    None
                    if translation_control
                    .coast_velocity_unwind_position_target_m is None
                    else translation_control
                    .coast_velocity_unwind_position_target_m.tolist()
                ),
                'coast_velocity_unwind_position_progress_m': (
                    translation_control
                    .coast_velocity_unwind_position_progress_m
                ),
                'coast_velocity_unwind_lateral_error_m': (
                    translation_control
                    .coast_velocity_unwind_lateral_error_m
                ),
                'coast_attitude_time_constant_s': (
                    translation_control.coast_attitude_time_constant_s
                ),
                'coast_attitude_acceleration_scale': (
                    translation_control.coast_attitude_acceleration_scale
                ),
                'coast_command_acceleration_m_s2': (
                    None
                    if translation_control.coast_command_acceleration_m_s2
                    is None
                    else translation_control
                    .coast_command_acceleration_m_s2.tolist()
                ),
                'coast_predicted_level_stop_distance_m': (
                    translation_control
                    .coast_predicted_level_stop_distance_m
                ),
                'coast_predicted_level_stop_time_s': (
                    translation_control.coast_predicted_level_stop_time_s
                ),
                'coast_predicted_level_terminal_speed_m_s': (
                    translation_control
                    .coast_predicted_level_terminal_speed_m_s
                ),
                'coast_impulse_safe_deceleration_m_s2': (
                    translation_control
                    .coast_impulse_safe_deceleration_m_s2
                ),
                'coast_tail_cancellation_acceleration_m_s2': (
                    translation_control
                    .coast_tail_cancellation_acceleration_m_s2
                ),
                'coast_tail_cancellation_signed_acceleration_m_s2': (
                    translation_control
                    .coast_tail_cancellation_signed_acceleration_m_s2
                ),
                'coast_tail_terminal_target_speed_m_s': (
                    translation_control.coast_tail_terminal_target_speed_m_s
                ),
                'coast_predicted_terminal_after_pulse_m_s': (
                    translation_control
                    .coast_predicted_terminal_after_pulse_m_s
                ),
                'coast_command_hold_s': (
                    translation_control.coast_command_hold_s
                ),
                'coast_actual_tilt_deg': (
                    translation_control.coast_actual_tilt_deg
                ),
                'coast_acceleration_estimate_valid': (
                    translation_control._coast_acceleration_valid
                ),
                'coast_model_acceleration_from_attitude_m_s2': (
                    translation_control
                    ._coast_model_acceleration_xy.tolist()
                ),
                'coast_handoff_state_ready': (
                    translation_control.coast_handoff_state_ready
                ),
                'coast_response_queue_settled': (
                    translation_control.coast_response_queue_settled
                ),
                'coast_response_queue_settle_elapsed_s': (
                    translation_control.coast_response_queue_settle_elapsed_s
                ),
                'coast_response_queue_settle_required_s': (
                    translation_control.coast_response_queue_settle_required_s
                ),
                'coast_level_handoff_speed_threshold_m_s': (
                    translation_control.coast_level_handoff_speed_m_s
                ),
                'coast_level_handoff_delay_s': (
                    translation_control.coast_level_handoff_delay_s
                ),
                'coast_level_handoff_latched': (
                    translation_control._coast_level_handoff_latched
                ),
                'coast_handoff_reason': (
                    translation_control.coast_handoff_reason
                ),
                'coast_velocity_phase': (
                    translation_control.coast_velocity_phase
                ),
                'coast_velocity_rebrake_count': (
                    translation_control.coast_velocity_rebrake_count
                ),
                'coast_velocity_rebrake_enabled': (
                    translation_control.coast_velocity_rebrake_enabled
                ),
                'coast_velocity_predicted_unwind_terminal_speed_m_s': (
                    translation_control
                    .coast_velocity_predicted_unwind_terminal_speed_m_s
                ),
                'coast_velocity_predicted_next_step_terminal_speed_m_s': (
                    translation_control
                    .coast_velocity_predicted_next_step_terminal_speed_m_s
                ),
                'coast_velocity_dynamic_unwind_threshold_m_s': (
                    translation_control
                    .coast_velocity_dynamic_unwind_threshold_m_s
                ),
                'coast_velocity_dynamic_unwind_step_guard_m_s': (
                    translation_control
                    .coast_velocity_dynamic_unwind_step_guard_m_s
                ),
                'coast_velocity_unwind_decision_reason': (
                    translation_control.coast_velocity_unwind_decision_reason
                ),
                'coast_velocity_projected_acceleration_m_s2': (
                    translation_control
                    .coast_velocity_projected_acceleration_m_s2
                ),
                'coast_velocity_unwind_response_horizon_s': (
                    translation_control
                    .coast_velocity_unwind_response_horizon_s
                ),
                'coast_velocity_unwind_integrated_velocity_delta_m_s': (
                    translation_control
                    .coast_velocity_unwind_integrated_velocity_delta_m_s
                ),
                'coast_velocity_unwind_raw_integrated_velocity_delta_m_s': (
                    translation_control
                    .coast_velocity_unwind_raw_integrated_velocity_delta_m_s
                ),
                'coast_velocity_unwind_tail_calibration_scale': (
                    translation_control
                    .coast_velocity_unwind_tail_calibration_scale
                ),
                'coast_velocity_unwind_leveling_duration_s': (
                    translation_control
                    .coast_velocity_unwind_leveling_duration_s
                ),
                'coast_velocity_handoff_tilt_ready': (
                    translation_control.coast_velocity_handoff_tilt_ready
                ),
                'coast_velocity_handoff_rate_ready': (
                    translation_control.coast_velocity_handoff_rate_ready
                ),
                'coast_velocity_handoff_speed_ready': (
                    translation_control.coast_velocity_handoff_speed_ready
                ),
                'coast_state_sample_valid': (
                    translation_control.coast_state_sample_valid
                ),
                'coast_state_rejection_reason': (
                    translation_control.coast_state_rejection_reason
                ),
                'coast_state_kinematic_residual_m': (
                    translation_control.coast_state_kinematic_residual_m
                ),
                'coast_state_implied_acceleration_m_s2': (
                    translation_control.coast_state_implied_acceleration_m_s2
                ),
                'coast_state_sample_gap_s': (
                    translation_control.coast_state_sample_gap_s
                ),
                'coast_state_rejection_count': (
                    translation_control.coast_state_rejection_count
                ),
                'force_target_roll_deg': force_target_roll,
                'force_target_pitch_deg': force_target_pitch,
                'force_raw_tilt_deg': force_raw_tilt_deg,
                'force_attitude_saturated': force_attitude_saturated,
                'virtual_friction_force_N': force_virtual_friction_N,
                'virtual_air_drag_force_N': force_virtual_drag_N,
                'virtual_resistance_force_xy_N': (
                    force_virtual_resistance_xy.tolist()
                ),
                'applied_resistance_counter_force_xy_N': (
                    (
                        force_current_mass / force_virtual_mass
                    ) * force_virtual_resistance_xy
                ).tolist(),
                'virtual_position_m': (
                    None if virtual_motion_state is None else [
                        float(virtual_motion_state['position'][0]),
                        float(virtual_motion_state['position'][1]),
                        float(translation_control.hover_z),
                    ]
                ),
                'virtual_velocity_m_s': (
                    None if virtual_motion_state is None else [
                        float(virtual_motion_state['velocity'][0]),
                        float(virtual_motion_state['velocity'][1]),
                        0.0,
                    ]
                ),
                'shadow_mode': pipeline.shadow_mode,
            })
            if actual_commands_sent_since_previous_state:
                release_dataset_last_logged_command_sequence = max(
                    command['sequence']
                    for command in actual_commands_sent_since_previous_state
                )
            if (
                release_dataset_episode_id is not None
                and release_dataset_close_after_row is not None
            ):
                terminal_handoff = bool(
                    release_dataset_close_after_row[
                        'release_dataset_outcome'
                    ] == 'terminal_handoff'
                )
                if bootstrap_coverage is not None and terminal_handoff:
                    # The position handoff is sent after this row's measured
                    # state. Keep the episode open until one strictly newer
                    # fresh state provides the no-extrapolation upper bracket
                    # required by the offline decision-time resampler.
                    final_position_commands = [
                        command
                        for command in actual_commands_sent_since_previous_state
                        if command.get('kind') == 'position'
                    ]
                    if len(final_position_commands) != 1:
                        raise RuntimeError(
                            'LMPC terminal row must contain exactly one final '
                            'position send'
                        )
                    final_position_command = final_position_commands[0]
                    release_dataset_terminal_finalize_pending = {
                        'episode_id': release_dataset_episode_id,
                        'event_name': release_dataset_close_after_row[
                            'event_name'
                        ],
                        'outcome': release_dataset_close_after_row[
                            'release_dataset_outcome'
                        ],
                        'reason': release_dataset_close_after_row['reason'],
                        'terminal_gate': (
                            release_dataset_terminal_status.to_dict()
                        ),
                        'terminal_state_time': state_time,
                        'final_position_sent_at': float(
                            final_position_command['sent_at']
                        ),
                        'final_position_sequence': int(
                            final_position_command['sequence']
                        ),
                    }
                else:
                    automatic_close_result = close_bootstrap_episode(
                        release_dataset_episode_id,
                        terminal_success=terminal_handoff,
                        reason=release_dataset_close_after_row['reason'],
                    )
                    self._log_event(
                        release_dataset_close_after_row['event_name'],
                        {
                            'release_dataset_episode_id': (
                                release_dataset_episode_id
                            ),
                            'release_dataset_outcome': (
                                release_dataset_close_after_row[
                                    'release_dataset_outcome'
                                ]
                            ),
                            'reason': release_dataset_close_after_row['reason'],
                            'terminal_gate': (
                                release_dataset_terminal_status.to_dict()
                            ),
                            'terminal_state_time': state_time,
                            'offline_lmpc_dataset_only': True,
                            'state_source': 'crazyflie_state_estimate',
                        },
                    )
                    if mpc_automatic_attempt is not None:
                        finish_mpc_automatic_attempt(
                            counted=bool(automatic_close_result['counted']),
                            reason=(
                                automatic_close_result[
                                    'path_failure_reasons'
                                ][0]
                                if automatic_close_result[
                                    'path_failure_reasons'
                                ] else
                                automatic_close_result['close_reason']
                            ),
                        )
                    release_dataset_episode_id = None
                    release_dataset_direction_xy = None
                    release_dataset_model_contract = None
                    release_dataset_measured_sensor_axis_world_xy = None
                    release_dataset_terminal_status = (
                        release_dataset_terminal_gate.reset()
                    )
            self._safe_sleep(max(dt - (time.time() - now), 0.0))
            if (calibration_mode and interaction_start is not None
                    and not calibration_wait_this_cycle):
                # Advance only after one complete, fresh, synchronized
                # sample/control cycle, but use the measured interval between
                # admitted states rather than assuming the requested control
                # period. Otherwise 50-70 Hz state delivery to a 100 Hz loop
                # silently turns a nominal 0.45 s attitude pulse into a much
                # longer real command. Duplicate timestamps never reach this
                # branch, and a large telemetry discontinuity is reduced to
                # one nominal step above, so pauses still cannot skip phases.
                next_elapsed_s = (
                    calibration_elapsed_s + protocol_state_step_s
                )
                # Never step past an unadmitted start boundary: otherwise a
                # short level phase could be skipped while waiting for a trial.
                for boundary_s, _label, key, gate, _plan in calibration_trial_boundaries:
                    if (not gate.admitted(key)
                            and calibration_elapsed_s < boundary_s <= next_elapsed_s):
                        next_elapsed_s = min(next_elapsed_s, boundary_s)
                calibration_elapsed_s = next_elapsed_s

        if bootstrap_coverage is not None:
            if release_dataset_episode_id is not None:
                self._log_event('Release Dataset Episode Closed', {
                    'release_dataset_episode_id': release_dataset_episode_id,
                    'release_dataset_outcome': 'rejected',
                    'reason': 'mpc_bootstrap_collection_ended_mid_episode',
                    'terminal_gate': (
                        release_dataset_terminal_status.to_dict()
                    ),
                    'offline_lmpc_dataset_only': True,
                    'state_source': 'crazyflie_state_estimate',
                })
                close_bootstrap_episode(
                    release_dataset_episode_id,
                    terminal_success=False,
                    reason='mpc_bootstrap_collection_ended_mid_episode',
                )
                release_dataset_episode_id = None
                release_dataset_direction_xy = None
                release_dataset_model_contract = None
                release_dataset_measured_sensor_axis_world_xy = None
            summary = bootstrap_coverage.summary()
            self._log_event('Learning MPC Bootstrap Calibration Complete', {
                **summary,
                'protocol': bootstrap_coverage.config.to_dict(),
                'raw_log_only': True,
                'requires_offline_resampling': True,
                'requires_strict_replay': True,
                'safe_set_published_in_flight': False,
                'state_source': 'crazyflie_state_estimate',
            })
            if summary['complete']:
                logger.info(
                    'LMPC BOOTSTRAP COLLECTION COMPLETE. No safe set was '
                    'published in flight; run the offline resampler/replay.'
                )
            else:
                logger.warning(
                    'LMPC BOOTSTRAP COLLECTION INCOMPLETE. Next target is '
                    '%s m/s; inspect the complete raw log before another run.',
                    summary['current_target_speed_m_s'],
                )

        # Legacy final identification/save can take seconds. The onboard HLC
        # must own a live hover trajectory BEFORE the LL stream stops, not in
        # an outer finally after those calculations have finished.
        if calibration_mode:
            self._handoff_translation_hold(nominal_position, nominal_yaw_deg)
        elif mpc_calibration_mode and self._translation_exit_target is not None:
            exit_position, exit_yaw = self._translation_exit_target
            self._handoff_translation_hold(exit_position, exit_yaw)

        if braking_test_mode:
            result = repeat_test_result(
                planar_braking_plan, planar_braking_config,
                planar_braking_samples, repeat_reference,
            )
            if calibration_reference(calibration_path) != repeat_reference:
                raise RuntimeError('calibration file changed externally during braking repeat test')
            self._log_event('Planar Braking Repeat Test Complete', result)
            logger.info('BRAKING REPEAT TEST COMPLETE: %d trials, %d samples; '
                        'calibration unchanged. Analyze the flight log offline.',
                        result['maneuver_count'], result['sample_count'])
        elif calibration_mode:
            fit = identify_xyz_alignment(
                model_calibration_samples,
                window_s=float(config['impulse_estimator']['window_s']),
            )
            planar_braking_fit = None
            planar_braking_fit_source = None
            if planar_braking_plan.enabled:
                planar_braking_fit = identify_planar_braking_response(
                    planar_braking_samples,
                    window_s=float(planar_braking_config['fit_window_s']),
                    max_delay_s=float(
                        planar_braking_config['max_fit_delay_s']
                    ),
                    max_time_constant_s=float(
                        planar_braking_config[
                            'max_fit_time_constant_s'
                        ]
                    ),
                    expected_maneuver_count=len(
                        planar_braking_plan.trial_directions
                    ),
                    minimum_r_squared=float(
                        planar_braking_config['minimum_fit_r_squared']
                    ),
                    minimum_validation_r_squared=float(
                        planar_braking_config[
                            'minimum_validation_r_squared'
                        ]
                    ),
                    minimum_acceleration_scale=float(
                        planar_braking_config[
                            'minimum_acceleration_scale'
                        ]
                    ),
                    maximum_acceleration_scale=float(
                        planar_braking_config[
                            'maximum_acceleration_scale'
                        ]
                    ),
                    minimum_trials_per_direction=int(
                        planar_braking_config[
                            'minimum_trials_per_direction'
                        ]
                    ),
                    minimum_windows_per_trial=int(
                        planar_braking_config['minimum_windows_per_trial']
                    ),
                    minimum_direction_r_squared=float(
                        planar_braking_config[
                            'minimum_direction_r_squared'
                        ]
                    ),
                    minimum_direction_validation_r_squared=float(
                        planar_braking_config[
                            'minimum_direction_validation_r_squared'
                        ]
                    ),
                    maximum_direction_nrmse=float(
                        planar_braking_config['maximum_direction_nrmse']
                    ),
                    maximum_direction_gain_ratio=float(
                        planar_braking_config[
                            'maximum_direction_gain_ratio'
                        ]
                    ),
                    maximum_repeat_gain_deviation=float(
                        planar_braking_config[
                            'maximum_repeat_gain_deviation'
                        ]
                    ),
                    maximum_acceleration_extrapolation_ratio=float(
                        planar_braking_config[
                            'maximum_acceleration_extrapolation_ratio'
                        ]
                    ),
                    raise_on_quality_failure=not adaptive_braking.enabled,
                )
                planar_braking_fit['protocol'] = {
                    **planar_braking_plan.timing_protocol(),
                    'adaptive_braking_calibration': {
                        'enabled': adaptive_braking.enabled,
                        'target_distance_m': adaptive_braking.config.get('target_distance_m'),
                        'timing_values_are_maximum_scheduled_pulses': adaptive_braking.enabled,
                        'actual_commands_recorded_in_samples': True,
                    },
                    'directions_xy': (
                        planar_braking_plan.directions.tolist()
                    ),
                    'repetitions': planar_braking_plan.repetitions,
                    'tilt_deg': planar_braking_plan.tilt_deg,
                    'tilt_levels_deg': (
                        planar_braking_plan.tilt_levels_deg.tolist()
                    ),
                    'repetitions_per_tilt': (
                        planar_braking_plan.repetitions_per_tilt
                    ),
                    'level_before_acceleration_s': (
                        planar_braking_plan.level_before_acceleration_s
                    ),
                    'accelerate_s': planar_braking_plan.accelerate_s,
                    'level_before_brake_s': (
                        planar_braking_plan.level_before_brake_s
                    ),
                    'brake_s': planar_braking_plan.brake_s,
                    'level_after_brake_s': (
                        planar_braking_plan.level_after_brake_s
                    ),
                    'recovery_s': planar_braking_plan.recovery_s,
                    'max_xy_speed_m_s': (
                        planar_braking_plan.max_xy_speed_m_s
                    ),
                    'max_displacement_m': (
                        planar_braking_plan.max_displacement_m
                    ),
                    **{
                        name: getattr(calibration_trial_gates[0], name)
                        for name in (
                            'trial_start_max_xy_speed_m_s',
                            'trial_start_max_tilt_deg',
                            'trial_start_max_position_error_m',
                            'trial_start_dwell_s',
                            'trial_start_timeout_s',
                            'trial_start_max_sample_gap_s',
                        )
                    },
                    'calibrated_axes': [
                        axis_name
                        for axis_index, axis_name in enumerate(('x', 'y'))
                        if np.any(np.abs(
                            planar_braking_plan.directions[:, axis_index]
                        ) > 1e-9)
                    ],
                }
                attempted_planar_braking_fit = planar_braking_fit
                (
                    planar_braking_fit,
                    planar_braking_fit_source,
                    preserved_planar_braking_fit,
                ) = _planar_fit_for_calibration_save(
                    attempted_planar_braking_fit,
                    adaptive_braking.enabled,
                    self.drone_id,
                    calibration_path,
                )
                if preserved_planar_braking_fit is not None:
                    self._log_event(
                        'Adaptive Planar Calibration Fit Preserved', {
                            'reason': (
                                'planar braking response fit failed quality gates: '
                                + '; '.join(
                                    attempted_planar_braking_fit.get(
                                        'quality_failures', []
                                    )
                                )
                            ),
                            'quality_failures': attempted_planar_braking_fit.get(
                                'quality_failures', []
                            ),
                            'attempted_fit': attempted_planar_braking_fit,
                            'preserved_planar_braking_fit': (
                                preserved_planar_braking_fit
                            ),
                            'previous_calibration_preserved': True,
                            'prediction_model_save_continues': True,
                            'path': str(calibration_path),
                            'state_source': 'crazyflie_state_estimate',
                        }
                    )
            position_capture_fit = None
            if position_capture_plan.enabled:
                position_capture_fit = position_capture_plan.summarize(
                    position_capture_samples
                )
                cached_parameters = getattr(
                    getattr(getattr(self, 'cf', None), 'param', None),
                    'values', None,
                )
                parameter_groups = (
                    'posCtlPid', 'velCtlPid', 'pid_attitude', 'pid_rate',
                    'stabilizer',
                )
                controller_parameters = {
                    group: deepcopy(cached_parameters[group])
                    for group in parameter_groups
                    if (isinstance(cached_parameters, dict)
                        and isinstance(cached_parameters.get(group), dict))
                }
                position_capture_fit['control_context'] = {
                    'nominal_yaw_deg': nominal_yaw_deg,
                    'nominal_position_m': nominal_position.tolist(),
                    'mass_kg': float(config['mass']),
                    'control_rate_hz': self.ctrl_rate,
                    'controller_parameter_source': (
                        'crazyflie_cached_parameters'
                        if controller_parameters else 'unavailable'
                    ),
                    'controller_parameters': controller_parameters,
                    'controller_parameter_groups_unavailable': [
                        group for group in parameter_groups
                        if group not in controller_parameters
                    ],
                    'automatic_interaction_handoff_enabled': False,
                }
                self._log_event('Position Capture Calibration Evaluated', {
                    'position_capture_fit': position_capture_fit,
                    'state_source': 'crazyflie_state_estimate',
                })
                if not position_capture_fit.get('usable', False):
                    # Evaluate the entire capture window, including any late
                    # rebound, before saving anything. A failed new stage must
                    # not replace a previously usable calibration file.
                    self._log_event('Position Capture Calibration Rejected', {
                        'reason': 'fixed-target capture quality gates failed',
                        'position_capture_fit': position_capture_fit,
                        'previous_calibration_preserved': True,
                        'state_source': 'crazyflie_state_estimate',
                    })
                    raise ValueError(
                        'position capture calibration failed quality gates; '
                        'previous calibration file was preserved; inspect '
                        'Position Capture Calibration Evaluated in the flight log'
                    )
            prediction_report = None
            if prediction_calibration is not None:
                # Optimizers are never joined here. A late or rejected result
                # remains a separate diagnostic artifact, not a live model.
                poll_prediction_calibration(prediction_calibration, self._log_event)
                prediction_report = prediction_calibration.latest_report
            saved_path, saved_entry = save_drone_calibration(
                self.drone_id,
                fit,
                config['motor_model'],
                calibration_path,
                planar_braking_fit=planar_braking_fit,
                position_capture_fit=position_capture_fit,
                **({'online_prediction_report': prediction_report}
                   if prediction_report is not None else {}),
            )
            self._log_event('Wrench Model Calibration Saved', {
                'state_source': 'crazyflie_state_estimate',
                'path': str(saved_path),
                'impulse_estimator': saved_entry['impulse_estimator'],
                'fit': fit,
                'control_handoff': saved_entry.get('control_handoff'),
                'planar_braking_fit': saved_entry.get('planar_braking_fit'),
                'planar_braking_fit_source': planar_braking_fit_source,
                'position_capture_fit': position_capture_fit,
            })
            logger.info('CALIBRATED %s', saved_path)
        else:
            two_afc_summary = two_afc_friction.summary()
            if two_afc_friction.enabled:
                self._log_event('2AFC Friction Sequence Complete', {
                    **two_afc_summary,
                    'state_source': 'crazyflie_state_estimate',
                })
                logger.info(
                    '2AFC ACTUAL FRICTION ORDER: %s',
                    two_afc_friction.formatted_sequence(),
                )
            self._log_event('Wrench Interaction Complete', {
                'state_source': 'crazyflie_state_estimate',
                'two_afc_friction': two_afc_summary,
            })

    def _run_peer_translation(self) -> None:
        """Run symmetric peer interaction — every drone can push and follow."""
        try:
            translation_setting = self.mission['Interaction']['config']
            self.interaction_peer_translation_vel(
                drone_id=self.drone_id,
                vel_threshold=translation_setting['delta_v'],
                z=translation_setting['z'],
                fric_coe=translation_setting['friction_coefficient'],
                base_attitude=translation_setting['base_attitude'],
                duration=translation_setting['duration'],
                v_scalar=translation_setting['v_scalar'],
                grace_time=translation_setting['grace_time'],
                pub_socket=self.pub_socket,
                sub_socket=self.sub_socket,
            )
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Peer Translation Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()

    # def run_translation_broadcast(self) -> None:
    #     """UI-LB mode: run translation interaction while broadcasting APF avoidance
    #     commands to all passive I-LBs.
    #
    #     The translation loop runs in a background thread (the user-facing interaction).
    #     The main thread runs the APF loop at the avoidance control rate:
    #       1. Drain sub_socket for the latest position report from each I-LB.
    #       2. Fetch own position from log_manager (UI-LB position).
    #       3. Call apf_velocity() for every I-LB — no PID, no simulation step.
    #       4. Integrate v_cmd * dt to get the desired absolute position.
    #       5. Broadcast {"type": "avoid_cmd", "commands": {lb_id: [x,y,z], ...}}
    #          via pub_socket so each I-LB can apply the setpoint directly.
    #
    #     Stops when the translation thread finishes (interaction duration elapsed).
    #     """
    #     avoidance_cfg = self.mission.get('avoidance', {})
    #     eta      = avoidance_cfg.get('eta',       0.5)
    #     zeta     = avoidance_cfg.get('zeta',      0.0)
    #     d_detect = avoidance_cfg.get('d_detect',  0.47)
    #     v_max    = avoidance_cfg.get('v_max',     2.0)
    #     rate     = avoidance_cfg.get('ctrl_rate', self.ctrl_rate if self.ctrl_rate > 0 else 50)
    #     dt       = 1.0 / rate
    #
    #     # Goal positions for each passive I-LB (their static hover targets)
    #     drone_mission = self.mission.get('drones', {})
    #     lb_goals = {
    #         lb_id: np.array(cfg['target'][:3])
    #         for lb_id, cfg in drone_mission.items()
    #         if cfg.get('interaction') == 'avoid'
    #     }
    #     # Last known positions start at goal (I-LBs are initially at target)
    #     lb_positions = {lb_id: goal.copy() for lb_id, goal in lb_goals.items()}
    #     pos_lock = threading.Lock()
    #
    #     translation_thread = threading.Thread(target=self._run_translation, daemon=True)
    #     translation_thread.start()
    #
    #     try:
    #         while translation_thread.is_alive():
    #             # 1. Receive latest position reports from passive I-LBs
    #             if self.sub_socket is not None:
    #                 msg = self.sub_socket.recv_latest()
    #                 if msg is not None and msg.get('type') == 'position':
    #                     lb_id = msg.get('drone_id')
    #                     pos   = msg.get('pos')
    #                     if lb_id in lb_positions and pos is not None:
    #                         with pos_lock:
    #                             lb_positions[lb_id] = np.array(pos)
    #
    #             # 2. Own (UI-LB) position
    #             try:
    #                 ui_pos = self._get_latest_pos()
    #             except Exception:
    #                 time.sleep(dt)
    #                 continue
    #
    #             # 3-4. APF velocity → position offset for each I-LB
    #             cmds = {}
    #             with pos_lock:
    #                 for lb_id, lb_pos in lb_positions.items():
    #                     v_cmd = apf_velocity(lb_pos, lb_goals[lb_id], ui_pos,
    #                                          eta, zeta, d_detect, v_max)
    #                     offset = v_cmd * dt
    #                     cmds[lb_id] = offset.tolist()
    #                     # Advance local position estimate for next APF step
    #                     lb_positions[lb_id] = lb_pos + offset
    #
    #             # 5. Broadcast to all passive I-LBs
    #             if self.pub_socket is not None and cmds:
    #                 self.pub_socket.send_json({'type': 'avoid_cmd', 'commands': cmds})
    #
    #             time.sleep(dt)
    #     except Exception as e:
    #         tb_info = traceback.format_exc()
    #         logging.error(f"Avoidance Broadcast Error: {e}\nTraceback:\n{tb_info}")
    #     finally:
    #         translation_thread.join(timeout=2.0)
    #
    # def run_passive_avoidance(self) -> None:
    #     """Passive I-LB mode: publish own position to UI-LB and execute APF commands.
    #
    #     Receiving is non-blocking (recv_latest drains the ZMQ/UDP buffer and
    #     returns the newest message, or None if nothing arrived).
    #
    #     Message protocol:
    #       publish  → {"type": "position",  "drone_id": <id>,  "pos": [x, y, z]}
    #       receive  ← {"type": "avoid_cmd", "commands": {<id>: [dx, dy, dz], ...}}
    #
    #     Each command is a position *offset* (delta) that is added to the current
    #     desired hover position, not an absolute setpoint.
    #     """
    #     avoidance_cfg = self.mission.get('avoidance', {})
    #     rate     = avoidance_cfg.get('ctrl_rate', self.ctrl_rate if self.ctrl_rate > 0 else 50)
    #     dt       = 1.0 / rate
    #     duration = self.mission.get('drones', {}).get(self.drone_id, {}).get('delta_t', 60)
    #
    #     drone_cfg = self.mission['drones'][self.drone_id]
    #     hover_pos = np.array(drone_cfg['target'][:3], dtype=float)
    #
    #     start_t = time.time()
    #     try:
    #         while time.time() - start_t < duration:
    #             # Publish own position so the UI-LB can run APF for this drone
    #             try:
    #                 my_pos = self._get_latest_pos()
    #                 if self.pub_socket is not None:
    #                     self.pub_socket.send_json({
    #                         'type':     'position',
    #                         'drone_id': self.drone_id,
    #                         'pos':      my_pos.tolist(),
    #                     })
    #             except Exception:
    #                 pass
    #
    #             # Apply latest avoidance offset if one has arrived (non-blocking)
    #             if self.sub_socket is not None:
    #                 msg = self.sub_socket.recv_latest()
    #                 if msg is not None and msg.get('type') == 'avoid_cmd':
    #                     cmds = msg.get('commands', {})
    #                     if self.drone_id in cmds:
    #                         hover_pos += np.array(cmds[self.drone_id], dtype=float)
    #
    #             self.lo_commander.send_position_setpoint(
    #                 hover_pos[0], hover_pos[1], hover_pos[2], 0)
    #             time.sleep(dt)
    #     except Exception as e:
    #         tb_info = traceback.format_exc()
    #         logging.error(f"Passive Avoidance Error: {e}\nTraceback:\n{tb_info}")
    #     finally:
    #         self.lo_commander.send_notify_setpoint_stop()

    def _run_peer_latency_test(self) -> None:
        """Peer TCP latency test — comparable to live interaction transport."""
        cfg = self.mission['Interaction']['config']
        num_packets = cfg.get('num_packets', 1_000_000)
        role = self.mission['drones'][self.drone_id].get('role', 'sender')

        payload = {
            "type": "push",
            "drone_id": self.drone_id,
            "accumulated_offset": [0.0, 0.0, 0.0],
            "push_start_time": 0.0,
        }

        try:
            if role == 'receiver':
                logger.info(f"[Latency Test] Receiver — waiting for {num_packets:,} packets...")
                first_arrival = None
                last_arrival = None
                received = 0
                timed_out = False
                while received < num_packets:
                    msg = self.sub_socket.recv_one_timeout(10.0)
                    if msg is None:
                        logger.warning(
                            f"[Latency Test] No packet for 10s — terminating early "
                            f"({received:,}/{num_packets:,} received, "
                            f"{num_packets - received:,} lost)."
                        )
                        timed_out = True
                        break
                    t = time.perf_counter()
                    if first_arrival is None:
                        first_arrival = t
                    last_arrival = t
                    received += 1

                total_time = (last_arrival - first_arrival) if (first_arrival and last_arrival) else 0.0
                avg_iat = total_time / (received - 1) * 1e6 if received > 1 else 0.0
                logger.info(f"[Latency Test] Receiver Results:")
                logger.info(f"  Packets received             : {received:,} / {num_packets:,}"
                            + (" (INCOMPLETE — packet loss)" if timed_out else ""))
                logger.info(f"  Total reception time         : {total_time:.4f} s")
                logger.info(f"  Avg packet inter-arrival time: {avg_iat:.4f} us")

            else:
                logger.info(f"[Latency Test] Sender — sending {num_packets:,} packets...")
                time.sleep(1)  # give receiver SUB socket time to connect
                s = time.perf_counter()
                for i in range(num_packets):
                    payload["push_start_time"] = i
                    self.pub_socket.send_json(payload)
                e = time.perf_counter() - s

                logger.info(f"[Latency Test] Sender Results:")
                logger.info(f"  Total elapsed time           : {e:.4f} s")
                logger.info(f"  Avg per-packet send time     : {e / num_packets * 1e6:.4f} us")
                logger.info(f"  Throughput                   : {num_packets / e:.0f} msgs/s")

        except Exception as ex:
            tb_info = traceback.format_exc()
            logging.error(f"Latency Test Error: {ex}\nTraceback:\n{tb_info}")

    def _run_network_follow(self) -> None:
        """Run as a network follower, mirroring the interaction drone's state."""
        try:
            translation_setting = self.mission['Interaction']['config']
            self.interaction_follow_network(
                sub_socket=self.sub_socket,
                z=translation_setting['z'],
                fric_coe=translation_setting['friction_coefficient'],
                base_attitude=translation_setting['base_attitude'],
                duration=translation_setting['duration'],
                v_scalar=translation_setting['v_scalar'],
            )
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Network Follow Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()

    def _run_gimbal(self) -> None:
        """Run the gimbal test."""
        try:
            # self.motor_test()
            self.gimbal_test()
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Gimbal Test Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()

    def _run_HRI_tunnel(self) -> None:
        """Run the HRI tunnel test."""
        try:
            self.test_HRI_tunnel()
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"HRI Tunnel Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()

    def _log_event(self, event_name, data=None):
        if data is None:
            data = {}
        data["time"] = round(time.time(), 6)

        self.log_manager.add_log_entry('events', data, name=event_name)

    def _get_latest_drone_state(self):
        return self.log_manager.get_latest_group_log_data()

    def _get_latest_pos(self, vel=False):
        if vel:
            return np.array(self.log_manager.groups[self.pos_group_name][-1]["tvec"]), np.array(
                self.log_manager.groups[self.pos_group_name][-1].get("vel", None))
        else:
            return np.array(self.log_manager.groups[self.pos_group_name][-1]["tvec"])

    def test_HRI_tunnel(self):
        hover_pos = [1, 2, 3]
        self.hl_commander.go_to(hover_pos[0], hover_pos[1], hover_pos[2], 0, 5, relative=False)
        self._safe_sleep(5)

    def test_rotation_limit(self, yawrate, duration=5):
        dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01
        start_t = time.time()

        while time.time() - start_t < 2:
            self.lo_commander.send_position_setpoint(0.0, 0.0, 1.0, 0)
            self._safe_sleep(dt)

        start_t = time.time()
        while time.time() - start_t < duration:
            self.lo_commander.send_hover_setpoint(0.0, 0.0, yawrate, 1.0)
            self._safe_sleep(dt)

        while time.time() - start_t < duration + 2:
            self.lo_commander.send_position_setpoint(0.0, 0.0, 1.0, 0)
            self._safe_sleep(dt)

    def _get_drone_by_id(self, drone_id):
        for drone in self.manifest['drones']:
            if drone['id'] == drone_id:
                return drone

    def interaction_translation_vel(
            self,
            vel_threshold=0.01,
            acc_threshold=None,
            z=1,
            fric_coe=-1.0,
            base_attitude=1,
            duration=60,
            grace_time=1,
            v_scalar=None,
            alpha_vel=1,
            pub_socket=None,
            current_mass=1.0,
            virtual_mass=1.0,
            virtual_object_config=None,
            init_hover=None,
            blender_port=None
    ):
        if v_scalar is None:
            v_scalar = np.array([10, 10, 2])
        else:
            v_scalar = np.array(v_scalar)
        dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01
        if acc_threshold is not None:
            try:
                acc_threshold = float(acc_threshold)
            except (TypeError, ValueError):
                logger.error(f"Invalid acceleration threshold: {acc_threshold}")
                acc_threshold = None

        virtual_object_config = virtual_object_config or {}
        use_virtual_stopping_model = bool(virtual_object_config)

        def get_virtual_config_float(name, default):
            value = virtual_object_config.get(name, default)
            try:
                return float(value)
            except (TypeError, ValueError):
                logger.error(f"Invalid virtual object config {name}: {value}")
                return float(default)

        current_mass = get_virtual_config_float('current_mass', current_mass)
        virtual_mass = get_virtual_config_float('mass', virtual_mass)
        virtual_friction_coe = get_virtual_config_float(
            'kinetic_friction_coefficient',
            virtual_object_config.get('friction_coefficient', 0.01)
        )
        virtual_drag_coe = get_virtual_config_float('drag_coefficient', 1.0)
        virtual_frontal_area = get_virtual_config_float('frontal_area', 0.019)
        virtual_air_density = get_virtual_config_float('air_density', 1.225)
        virtual_fallback_distance = get_virtual_config_float('fallback_stopping_distance', 0.08)
        virtual_max_distance = virtual_object_config.get('max_stopping_distance', None)
        if virtual_max_distance is not None:
            try:
                virtual_max_distance = float(virtual_max_distance)
            except (TypeError, ValueError):
                logger.error(f"Invalid virtual object config max_stopping_distance: {virtual_max_distance}")
                virtual_max_distance = None
        max_energy_gain = get_virtual_config_float(
            'max_energy_gain',
            virtual_object_config.get('max_light_inertia_gain', 4.0),
        )
        max_attitude_deg = get_virtual_config_float('max_attitude_deg', 20.0)
        max_velocity_command = get_virtual_config_float(
            'max_velocity_command_m_s', 1.0
        )
        if max_energy_gain < 1.0:
            raise ValueError('max_energy_gain must be at least 1')
        if max_attitude_deg <= 0.0:
            raise ValueError('max_attitude_deg must be positive')
        if max_velocity_command <= 0.0:
            raise ValueError('max_velocity_command_m_s must be positive')
        mass_class = velocity_inertia_mass_class(current_mass, virtual_mass)
        command_mode = inertia_command_mode(
            mass_class, virtual_object_config.get('inertia_command')
        )

        self.log_manager.add_log_entry(group_name="configs",
                                       entry={'detection_method': 'velocity',
                                              'delta_v': vel_threshold, 'Delta': dt, 'delta': v_scalar[0] * dt,
                                              "Orientation CMD": base_attitude, 'Grace Period': grace_time,
                                              'current_mass': current_mass, 'virtual_mass': virtual_mass,
                                              'delta_a': acc_threshold,
                                              'virtual_object': {
                                                  'enabled': use_virtual_stopping_model,
                                                  'current_mass': current_mass,
                                                  'mass': virtual_mass,
                                                  'kinetic_friction_coefficient': virtual_friction_coe,
                                                  'drag_coefficient': virtual_drag_coe,
                                                  'frontal_area': virtual_frontal_area,
                                                  'air_density': virtual_air_density,
                                                  'fallback_stopping_distance': virtual_fallback_distance,
                                                  'max_stopping_distance': virtual_max_distance,
                                                  'mass_class': mass_class,
                                                  'inertia_command': command_mode,
                                                  'energy_model': 'equal_kinetic_energy',
                                                  'max_energy_gain': max_energy_gain,
                                                  'max_attitude_deg': max_attitude_deg,
                                                  'max_velocity_command_m_s': max_velocity_command,
                                              }},
                                       name='Translation Config')

        status = 0

        def check_external_force(vel_vec, pitch, roll):
            tilt_vec = np.array([-np.sin(np.radians(pitch)), np.sin(np.radians(roll))])

            vel_vec = np.array(vel_vec[:2])

            dot_product = np.dot(tilt_vec, vel_vec)

            if dot_product < -0.1:  # Threshold to ignore noise
                return True
            return False

        def detect_speed_threshold(s):
            if s > vel_threshold:
                return True
            return False

        def detect_user_disengage(s, accel):
            if acc_threshold is not None:
                return accel < acc_threshold
            return not detect_speed_threshold(s)

        def calculate_virtual_hover_pos(cur_pos, heading, initial_speed):
            heading_norm = np.linalg.norm(heading)
            if heading_norm <= 0:
                return cur_pos.copy(), 0.0, None

            if use_virtual_stopping_model:
                stopping_distance, trajectory = self.calculate_virtual_stopping_distance(
                    initial_speed=initial_speed,
                    mass=virtual_mass,
                    friction_coefficient=virtual_friction_coe,
                    drag_coefficient=virtual_drag_coe,
                    frontal_area=virtual_frontal_area,
                    air_density=virtual_air_density,
                    fallback_distance=initial_speed * dt,
                    max_distance=virtual_max_distance,
                    dt=dt,
                    cur_pos=cur_pos,
                    heading=heading
                )
                final_pos = cur_pos + (heading / heading_norm) * stopping_distance
                return final_pos, stopping_distance, trajectory
            else:
                stopping_distance = initial_speed * dt
                return cur_pos + heading / heading_norm * stopping_distance, stopping_distance, None

        if init_hover:
            last_pos = init_hover
        else:
            while True:
                try:
                    last_pos = self._get_latest_pos()
                    break
                except Exception as e:
                    time.sleep(0.001)

        if z is not None:
            if z < 0:
                z = last_pos[2]
            hover_pos = [last_pos[0], last_pos[1], z]
        else:
            hover_pos = [last_pos[0], last_pos[1], last_pos[2]]

        self.hl_commander.go_to(hover_pos[0], hover_pos[1], hover_pos[2], 0, 2)
        self._safe_sleep(5)

        logger.info("Starting Force Feedback Interaction mode...")
        self._log_event('Waiting For User Interaction')

        interaction_heading = np.zeros(3)
        interaction_origin = np.asarray(hover_pos, dtype=float)
        v_virtual = np.zeros(3)  # virtual-object velocity, integrated from F/m_virtual
        prev_interact_vel = np.zeros(3)  # previous tick's interact_vel, for differentiation

        if blender_port:
            try:
                blender_port = int(blender_port)
            except (TypeError, ValueError):
                logger.error(f"Invalid Blender TCP port: {blender_port}")
                blender_port = None
        blender_state = None
        if blender_port:
            import json as _json
            import socket as _socket

            blender_state = {
                'edit_active': False,
                'edit_end_time': 0.0,
                'finish_requested': False,
                'stop_at_next_zero': False,
                'status': 0,
                'sending_positions': False,
                'sock': None,
                'sock_lock': threading.Lock(),
            }
            worker_done = threading.Event()

            def blender_worker():
                host_candidates = []
                if self.orchestrator_ip:
                    host_candidates.append(self.orchestrator_ip)
                for fallback_host in ("127.0.0.1", "localhost"):
                    if fallback_host not in host_candidates:
                        host_candidates.append(fallback_host)

                recv_buffer = ""
                next_log_time = 0.0

                try:
                    while not worker_done.is_set():
                        with blender_state['sock_lock']:
                            sock = blender_state['sock']

                        if sock is None:
                            for host in host_candidates:
                                try:
                                    candidate = _socket.create_connection((host, blender_port), timeout=1.0)
                                    candidate.settimeout(0.1)
                                    with blender_state['sock_lock']:
                                        blender_state['sock'] = candidate
                                    recv_buffer = ""
                                    logger.info(f"Connected to Blender at {host}:{blender_port}")
                                    break
                                except OSError as exc:
                                    now = time.time()
                                    if now >= next_log_time:
                                        logger.info(f"Waiting for Blender at {host}:{blender_port} ({exc})")
                                        next_log_time = now + 2.0

                            with blender_state['sock_lock']:
                                if blender_state['sock'] is None:
                                    time.sleep(0.25)
                                    continue
                                sock = blender_state['sock']

                        try:
                            data = sock.recv(1024)
                            if not data:
                                logger.info("Blender connection closed. Retrying.")
                                with blender_state['sock_lock']:
                                    blender_state['sock'] = None
                                try:
                                    sock.close()
                                except OSError:
                                    pass
                                recv_buffer = ""
                                time.sleep(0.1)
                                continue

                            recv_buffer += data.decode('utf-8', errors='ignore')
                            while "\n" in recv_buffer:
                                raw_line, recv_buffer = recv_buffer.split("\n", 1)
                                raw_line = raw_line.strip()
                                if not raw_line:
                                    continue
                                try:
                                    msg = _json.loads(raw_line)
                                except _json.JSONDecodeError:
                                    logger.debug(f"Ignoring malformed Blender message: {raw_line!r}")
                                    continue
                                if not isinstance(msg, dict):
                                    continue
                                cmd = msg.get("cmd")
                                if cmd == "start_edit":
                                    edit_dur = float(msg.get("duration", 10.0))
                                    blender_state['edit_active'] = True
                                    blender_state['edit_end_time'] = time.time() + edit_dur
                                    blender_state['finish_requested'] = False
                                    blender_state['stop_at_next_zero'] = False
                                    blender_state['sending_positions'] = True
                                    logger.info(f"Edit mode started for {edit_dur}s, streaming positions.")
                                elif cmd == "finish_edit":
                                    blender_state['finish_requested'] = True
                                    blender_state['sending_positions'] = True
                                    blender_state['stop_at_next_zero'] = True
                                    logger.info("Finish edit received; will stream until next status 0.")

                        except _socket.timeout:
                            pass
                        except (BlockingIOError, InterruptedError):
                            pass
                        except OSError as exc:
                            logger.info(f"Blender socket error, reconnecting: {exc}")
                            with blender_state['sock_lock']:
                                try:
                                    blender_state['sock'].close()
                                except OSError:
                                    pass
                                blender_state['sock'] = None
                            recv_buffer = ""

                finally:
                    with blender_state['sock_lock']:
                        if blender_state['sock'] is not None:
                            try:
                                blender_state['sock'].close()
                            except OSError:
                                pass
                            blender_state['sock'] = None

            blender_thread = threading.Thread(target=blender_worker, daemon=True)
            blender_thread.start()

            def send_blender_position(pos_to_send):
                with blender_state['sock_lock']:
                    sock = blender_state['sock']
                if sock is None:
                    return
                try:
                    resp = {
                        "id": self.drone_id,
                        "position": [round(float(x), 3) for x in pos_to_send],
                    }
                    sock.sendall((_json.dumps(resp) + "\n").encode('utf-8'))
                except Exception:
                    with blender_state['sock_lock']:
                        try:
                            if blender_state['sock'] is not None:
                                blender_state['sock'].close()
                        except OSError:
                            pass
                        blender_state['sock'] = None

        # --- Unified main loop ---
        # When blender_state is None the edit-mode pause is skipped and elapsed
        # time accumulates every tick, matching the original no-blender behaviour.
        elapsed_non_edit = 0.0
        loop_tick_time = time.time()
        last_blender_send_time = 0.0

        self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
        while elapsed_non_edit < duration:
            now = time.time()
            tick = now - loop_tick_time
            loop_tick_time = now

            if blender_state is not None:
                if blender_state['edit_active'] and now >= blender_state['edit_end_time']:
                    blender_state['finish_requested'] = True
                    blender_state['sending_positions'] = True
                    blender_state['stop_at_next_zero'] = True
                    blender_state['edit_end_time'] = float('inf')
                    logger.info("Edit duration expired; will stream until next status 0.")

                if blender_state['finish_requested'] and status == 0:
                    send_blender_position(hover_pos)
                    last_blender_send_time = now
                    blender_state['sending_positions'] = False
                    blender_state['finish_requested'] = False
                    blender_state['stop_at_next_zero'] = False
                    blender_state['edit_active'] = False
                    logger.info("Sent final hover position, stopped streaming.")

                if not blender_state['edit_active']:
                    # Pause interaction during Blender edit; only accumulate non-edit time.
                    elapsed_non_edit += tick
                    status = 0
                    self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                    self._safe_sleep(dt)
                    continue
            else:
                elapsed_non_edit += tick

            state = self._get_latest_drone_state()
            if not state:
                state = {}

            current_pitch = state.get('stateEstimate.pitch', 0.0)
            current_roll = state.get('stateEstimate.roll', 0.0)
            current_yaw = state.get('stateEstimate.yaw', 0.0)

            state_vx = state.get('stateEstimate.vx', 0.0)
            state_vy = state.get('stateEstimate.vy', 0.0)
            state_vz = state.get('stateEstimate.vz', 0.0)
            state_vel = np.array([state_vx, state_vy, state_vz])

            pos, vel = self._get_latest_pos(vel=True)

            vel = (alpha_vel * vel) + ((1.0 - alpha_vel) * state_vel)
            self.check_interaction_boundary(pos)
            if z is not None:
                pos[2] = z
                vel[2] = 0

            speed = np.linalg.norm(vel)

            if status == 0:  # wait for user interaction
                if blender_state is not None:
                    blender_state['status'] = 0

                if detect_speed_threshold(speed):
                    logger.info(f"Switching to Translation From {status}.")
                    if self.set_color:
                        self.set_color([0, 255, 0])
                    status = 1
                    interaction_heading = vel
                    interaction_origin = pos.copy()

                    v_virtual = np.zeros(3)
                    prev_interact_vel = vel.copy()
                    continue
                else:
                    self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)

            elif status == 1:  # pushed by user
                if blender_state is not None:
                    blender_state['status'] = 1
                if np.linalg.norm(interaction_heading) > 0 > np.dot(vel, interaction_heading):
                    logger.info("Ignoring interaction: Direction change > 90 degrees.")
                    interact_vel = np.array([0.0, 0.0, 0.0])
                    speed = 0
                else:
                    interact_vel = vel

                dv_lb = interact_vel - prev_interact_vel
                interaction_heading_norm = np.linalg.norm(interaction_heading)
                if interaction_heading_norm > 0:
                    acceleration = np.dot(dv_lb / dt, interaction_heading / interaction_heading_norm)
                else:
                    acceleration = 0.0
                (
                    v_virtual,
                    raw_energy_gain,
                    applied_energy_gain,
                    energy_gain_saturated,
                ) = kinetic_energy_velocity(
                    interact_vel,
                    current_mass,
                    virtual_mass,
                    max_energy_gain,
                )
                prev_interact_vel = interact_vel.copy()
                if command_mode == 'position':
                    target_pos = self._bounded_wrench_reference(
                        inertia_position_target(
                            interaction_origin, pos, applied_energy_gain
                        )
                    )
                else:
                    target_pos = self._bounded_wrench_reference(
                        pos + v_virtual * dt * v_scalar
                    )

                if detect_user_disengage(speed, acceleration):
                    v_virtual = np.zeros(3)
                    prev_interact_vel = np.zeros(3)
                    if fric_coe > 0:
                        logger.info(f"Switching to Coasting From {status}.")
                        if self.set_color:
                            self.set_color([255, 255, 0])
                        status = 2
                        continue
                    else:
                        logger.info(f"Switching to Grace Hover From {status}.")
                        if command_mode == 'position':
                            hover_pos = target_pos.copy()
                            status = 3
                        else:
                            status = 4

                        self.cf.param.set_value("posCtlPid.resetI", "1")
                        self.cf.param.set_value("velCtlPid.resetI", "1")

                        log_data = {
                            "speed": round(speed, 3),
                            "acceleration": round(acceleration, 3),
                            "vel": [round(x, 3) for x in vel],
                            "Pos": [round(x, 3) for x in pos],
                            "Grace Period": grace_time
                        }
                        if self.set_color:
                            self.set_color([255, 255, 0])
                        self._log_event("User Disengage", log_data)

                        if command_mode == 'position':
                            self._log_event("Hover Calculated", {
                                "stopping_distance": 0.0,
                                "Target": [round(x, 3) for x in hover_pos],
                                "Grace Period": grace_time,
                                "source": "virtual_position_target_at_disengage",
                            })

                        continue

                common_inertia_log = {
                    "mass_class": mass_class,
                    "inertia_command": command_mode,
                    "raw_energy_gain": round(raw_energy_gain, 4),
                    "applied_energy_gain": round(applied_energy_gain, 4),
                    "energy_gain_saturated": bool(energy_gain_saturated),
                    "virtual_velocity": [round(x, 3) for x in v_virtual],
                }

                if command_mode == 'position':
                    log_data = {
                        **common_inertia_log,
                        "speed": round(speed, 3),
                        "acceleration": round(acceleration, 3),
                        "vel": [round(x, 3) for x in vel],
                        "heading": [round(x, 3) for x in interaction_heading],
                        "Interaction Origin": [
                            round(x, 3) for x in interaction_origin
                        ],
                        "Pos": [round(x, 3) for x in pos],
                        "Target": [round(x, 3) for x in target_pos]
                    }
                    self._log_event("User Pushing", log_data)
                    self.lo_commander.send_position_setpoint(target_pos[0], target_pos[1], target_pos[2], 0)
                elif command_mode == 'velocity':
                    velocity_xy = v_virtual[:2].copy()
                    velocity_norm = float(np.linalg.norm(velocity_xy))
                    velocity_saturated = velocity_norm > max_velocity_command
                    if velocity_saturated:
                        velocity_xy *= max_velocity_command / velocity_norm
                    body_velocity = world_to_body_xy(velocity_xy, current_yaw)
                    yaw_rate_cmd = max(min(-5.0 * current_yaw, 50.0), -50.0)
                    log_data = {
                        **common_inertia_log,
                        "speed": round(speed, 3),
                        "acceleration": round(acceleration, 3),
                        "vel": [round(x, 3) for x in vel],
                        "Pos": [round(x, 3) for x in pos],
                        "body_velocity_command": [
                            round(float(x), 3) for x in body_velocity
                        ],
                        "velocity_command_saturated": velocity_saturated,
                    }
                    self._log_event("User Pushing", log_data)
                    self.lo_commander.send_hover_setpoint(
                        body_velocity[0], body_velocity[1],
                        yaw_rate_cmd, target_pos[2]
                    )
                else:
                    is_decelerating = np.dot(dv_lb, interact_vel) < 0
                    if is_decelerating:
                        # When decelerating, output a given value (defaulting to 0.0)
                        given_decel_value = 0.0
                        target_pitch, target_roll = given_decel_value, given_decel_value
                    elif base_attitude != 0:
                        target_pitch, target_roll = heavy_inertia_attitude(
                            dv_lb[:2],
                            dt,
                            current_yaw,
                            current_mass,
                            virtual_mass,
                            max_attitude_deg,
                        )
                    else:
                        target_pitch, target_roll = 0, 0

                    log_data = {
                        **common_inertia_log,
                        "speed": round(speed, 3),
                        "acceleration": round(acceleration, 3),
                        "vel": [round(x, 3) for x in vel],
                        "heading": [round(x, 3) for x in interaction_heading],
                        "Pos": [round(x, 3) for x in pos],
                        "Target_Attitude": [round(target_pitch, 3), round(target_roll, 3)],
                    }
                    self._log_event("User Pushing", log_data)
                    yaw_rate_cmd = max(min(-5.0 * current_yaw, 50.0), -50.0)
                    self.lo_commander.send_zdistance_setpoint(
                        target_roll, target_pitch, yaw_rate_cmd, target_pos[2]
                    )
            elif status == 2:  # coasting
                if blender_state is not None:
                    blender_state['status'] = 2
                end_pos, coast_t = self.calculate_coasting(pos, vel, fric_coe)

                self.lo_commander.send_notify_setpoint_stop()
                self.hl_commander.go_to(end_pos[0], end_pos[1], end_pos[2], 0, coast_t, relative=False)
                self._safe_sleep(coast_t)
                logger.info(f"Switching to Hover From {status}.")
                hover_pos = end_pos
                if self.set_color:
                    self.set_color([255, 157, 0])
                status = 0
                continue

            elif status == 3:  # grace period
                if blender_state is not None:
                    blender_state['status'] = 3

                grace_start = time.time()
                
                if getattr(self, 'virtual_trajectory', None) is not None:
                    for wp in self.virtual_trajectory:
                        self.lo_commander.send_position_setpoint(wp['pos'][0], wp['pos'][1], wp['pos'][2], 0)
                        self._safe_sleep(wp['dt'])
                    self.virtual_trajectory = None
                    self._log_event("Hovering")

                while time.time() < grace_time + grace_start:
                    self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                    self._safe_sleep(dt)

                if self.set_color:
                    self.set_color([255, 157, 0])
                status = 0
                if blender_state is not None:
                    blender_state['status'] = 0

                interaction_heading = np.zeros(3)
                continue

            if blender_state is not None and blender_state['sending_positions']:
                now_t = time.time()
                if now_t - last_blender_send_time >= 0.1:
                    send_pos = hover_pos if status == 0 else list(pos)
                    send_blender_position(send_pos)
                    last_blender_send_time = now_t
                    if status == 0 and blender_state['finish_requested']:
                        blender_state['sending_positions'] = False
                        blender_state['finish_requested'] = False
                        blender_state['stop_at_next_zero'] = False
                        blender_state['edit_active'] = False
                        logger.info("Sent final hover position, stopped streaming.")

            elif status == 4:
                if not detect_speed_threshold(speed) or use_virtual_stopping_model:
                    logger.info("Calculate Hover.")
                    if not use_virtual_stopping_model:
                        self.lo_commander.send_zdistance_setpoint(-np.sign(current_roll), -np.sign(current_pitch), 0, hover_pos[2])
                        self._safe_sleep(dt)
                    hover_pos, stopping_distance, trajectory = calculate_virtual_hover_pos(pos, interaction_heading, speed)
                    log_data = {
                        "stopping_distance": round(stopping_distance, 4),
                        "Target": [round(x, 3) for x in hover_pos],
                        "Grace Period": grace_time
                    }

                    if use_virtual_stopping_model:
                        self.virtual_trajectory = trajectory

                        logger.info(f"Waypoint:{len(trajectory)}")
                    else:
                        self.virtual_trajectory = None

                    self._log_event("Hover Calculated", log_data)
                    status = 3
                    continue

                h_norm = np.linalg.norm(interaction_heading)
                if h_norm > 0:
                    h = interaction_heading / h_norm
                    roll_cmd = np.sign(h[1]) * abs(current_roll)
                    pitch_cmd = np.sign(h[0]) * abs(current_pitch)
                else:
                    roll_cmd = -current_roll
                    pitch_cmd = -current_pitch
                self.lo_commander.send_zdistance_setpoint(roll_cmd, pitch_cmd, 0, hover_pos[2])
                self._safe_sleep(1 / 500)
                continue
            self._safe_sleep(dt)

        if blender_state is not None:
            worker_done.set()

        self.lo_commander.send_notify_setpoint_stop()

    def interaction_peer_translation_vel(
            self,
            drone_id,
            vel_threshold=0.01,
            z=1,
            fric_coe=-1.0,
            base_attitude=1,
            duration=60,
            grace_time=1,
            v_scalar=None,
            pub_socket=None,
            sub_socket=None,
    ):
        """Symmetric peer interaction: every drone can be pushed and mirrors others.

        When this drone detects a user push it broadcasts per-step offsets to peers.
        When it receives a push message from a peer it applies the offset to its own
        hover position. If both happen simultaneously the push with the latest
        push_start_time wins; the loser reverts to its pre-push hover position.
        """
        if v_scalar is None:
            v_scalar = np.array([10, 10, 2])
        else:
            v_scalar = np.array(v_scalar)

        dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01

        def drain_sub(duration):
            if sub_socket is None:
                return None
            else:
                # monitoring stays active, while still reacting immediately to peer messages
                start_time = time.time()
                while time.time() - start_time < duration:
                    self._safe_sleep(0)
                    msg = sub_socket.recv_latest()
                    if msg is not None:
                        return msg
                return None

        def detect_speed_threshold(s):
            return s > vel_threshold

        def calculate_braking_angles(v_x, v_y, yaw_deg=0.0, base_att=base_attitude):
            yaw_rad = np.radians(yaw_deg)
            cos_y = np.cos(yaw_rad)
            sin_y = np.sin(yaw_rad)
            body_v_x = v_x * cos_y + v_y * sin_y
            body_v_y = -v_x * sin_y + v_y * cos_y

            pitch = np.sign(body_v_x) * base_att
            roll = -np.sign(body_v_y) * base_att
            pitch = max(min(pitch, 20), -20)
            roll = max(min(roll, 20), -20)
            return pitch, roll

        while True:
            try:
                last_pos = self._get_latest_pos()
                break
            except Exception:
                time.sleep(0.001)

        if z is not None:
            hover_pos = np.array([last_pos[0], last_pos[1], z], dtype=float)
        else:
            hover_pos = last_pos.copy().astype(float)

        self.hl_commander.go_to(hover_pos[0], hover_pos[1], hover_pos[2], 0, 2)
        self._safe_sleep(2)

        logger.info("Starting Peer Interaction mode...")
        self._log_event('Waiting For User Interaction')

        self.log_manager.add_log_entry(
            group_name="configs",
            entry={'delta_v': vel_threshold, 'Delta': dt, 'Grace Period': grace_time},
            name='Peer Config',
        )

        status = 0
        push_start_time = None
        hover_pos_before_push = None
        interaction_heading = np.zeros(3)
        # Accumulated displacement (target_pos - hover_pos_before_push) sent to peers
        accumulated_offset = np.zeros(3)
        # Receiver side: own hover position at the moment the peer's push began
        peer_hover_start = None
        peer_push_start_time = None
        # Suppress local interaction detection while a peer is actively pushing
        receiving_peer_push = False
        # Time of last received peer push msg (for timeout detection)
        last_peer_push_time = None
        # Time when follower entered grace (after peer user_disengage), None if not in grace
        peer_grace_start = None

        start_time = time.time()
        while time.time() - start_time < duration:

            peer_msg = drain_sub(dt)

            state = self._get_latest_drone_state()
            if not state:
                state = {}

            current_pitch = state.get('stateEstimate.pitch', 0.0)
            current_roll = state.get('stateEstimate.roll', 0.0)
            current_yaw = state.get('stateEstimate.yaw', 0.0)

            pos, vel = self._get_latest_pos(vel=True)
            self.check_interaction_boundary(pos)
            if z is not None:
                pos[2] = z
                vel[2] = 0.0

            speed = np.linalg.norm(vel)
            if status == 0:
                if receiving_peer_push:
                    if peer_grace_start is not None:
                        # Follower grace: hold position until leader signals done or timer expires
                        if peer_msg and peer_msg.get('type') == 'grace_done':
                            leader_id = peer_msg.get('drone_id')
                            self._log_event("Peer Grace Done Received", {
                                "leader_id": leader_id,
                                "Pos": [round(x, 3) for x in pos],
                            })
                            receiving_peer_push = False
                            peer_grace_start = None
                            last_peer_push_time = None
                            peer_push_start_time = None
                            peer_hover_start = None
                        elif time.time() - peer_grace_start > grace_time:
                            logger.info("Peer mode: follower grace timeout — resuming detection.")
                            receiving_peer_push = False
                            peer_grace_start = None
                            last_peer_push_time = None
                            peer_push_start_time = None
                            peer_hover_start = None
                        else:
                            self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                    else:
                        # Active following
                        if peer_msg and peer_msg.get('type') == 'push':
                            leader_id = peer_msg.get('drone_id')
                            if peer_msg['push_start_time'] != peer_push_start_time:
                                peer_push_start_time = peer_msg['push_start_time']
                                peer_hover_start = hover_pos.copy()
                                self._log_event("Peer Push Received", {
                                    "leader_id": leader_id,
                                    "push_start_time": peer_push_start_time,
                                    "Pos": [round(x, 3) for x in pos],
                                })
                                if self.set_color:
                                    self.set_color([0, 255, 0])
                            last_peer_push_time = time.time()
                            accumulated = np.array(peer_msg['accumulated_offset'])
                            if z is not None:
                                accumulated[2] = 0.0
                            hover_pos = peer_hover_start + accumulated
                            if z is not None:
                                hover_pos[2] = z
                            self.check_interaction_boundary(hover_pos)
                            self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                            self._log_event("Peer Pushing", {
                                "leader_id": leader_id,
                                "push_start_time": peer_push_start_time,
                                "accumulated_offset": accumulated.tolist(),
                                "Pos": [round(x, 3) for x in pos],
                                "Target": [round(x, 3) for x in hover_pos],
                            })
                        elif peer_msg and peer_msg.get('type') == 'user_disengage':
                            leader_id = peer_msg.get('drone_id')
                            self._log_event("Peer Disengage Received", {
                                "leader_id": leader_id,
                                "Pos": [round(x, 3) for x in pos],
                            })
                            peer_grace_start = time.time()
                            self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                        elif last_peer_push_time is not None and time.time() - last_peer_push_time > grace_time:
                            # No push for too long — give up following
                            logger.info("Peer mode: no push received — giving up following.")
                            receiving_peer_push = False
                            last_peer_push_time = None
                            peer_push_start_time = None
                            peer_hover_start = None
                        else:
                            self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                else:
                    if peer_msg and peer_msg.get('type') == 'push':
                        leader_id = peer_msg.get('drone_id')
                        peer_push_start_time = peer_msg['push_start_time']
                        peer_hover_start = hover_pos.copy()
                        last_peer_push_time = time.time()
                        peer_grace_start = None
                        receiving_peer_push = True
                        self._log_event("Peer Push Received", {
                            "leader_id": leader_id,
                            "push_start_time": peer_push_start_time,
                            "Pos": [round(x, 3) for x in pos],
                        })
                        if self.set_color:
                            self.set_color([0, 255, 0])
                        accumulated = np.array(peer_msg['accumulated_offset'])
                        if z is not None:
                            accumulated[2] = 0.0
                        hover_pos = peer_hover_start + accumulated
                        if z is not None:
                            hover_pos[2] = z
                        self.check_interaction_boundary(hover_pos)
                        self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                        self._log_event("Peer Pushing", {
                            "leader_id": leader_id,
                            "push_start_time": peer_push_start_time,
                            "accumulated_offset": accumulated.tolist(),
                            "Pos": [round(x, 3) for x in pos],
                            "Target": [round(x, 3) for x in hover_pos],
                        })
                    elif detect_speed_threshold(speed):
                        logger.info("Peer mode: local user push detected.")
                        if self.set_color:
                            self.set_color([0, 255, 0])
                        status = 1
                        push_start_time = time.time()
                        hover_pos_before_push = hover_pos.copy()
                        accumulated_offset = np.zeros(3)
                        interaction_heading = vel.copy()
                        continue
                    else:
                        self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)

            elif status == 1:
                # Peer push is newer → it wins; revert and follow peer
                if peer_msg and peer_msg.get('type') == 'push':
                    peer_start = peer_msg.get('push_start_time', 0.0)
                    if peer_start >= push_start_time:
                        leader_id = peer_msg.get('drone_id')
                        logger.info("Peer push is newer — abandoning local push, reverting position.")
                        send_time = time.time()
                        pub_socket.send_json({"type": "grace_done", "drone_id": drone_id})
                        self._log_event("User Disengage", {
                            "leader_id": drone_id,
                            "reason": "peer_push_won",
                            "Pos": [round(x, 3) for x in pos],
                            "latency_ms": round((time.time() - send_time) * 1000, 3),
                        })
                        if self.set_color:
                            self.set_color([255, 255, 0])
                        hover_pos = hover_pos_before_push.copy()
                        push_start_time = None
                        hover_pos_before_push = None
                        accumulated_offset = np.zeros(3)
                        interaction_heading = np.zeros(3)
                        status = 0
                        receiving_peer_push = True
                        peer_grace_start = None
                        last_peer_push_time = time.time()
                        peer_push_start_time = peer_start
                        peer_hover_start = hover_pos.copy()
                        self._log_event("Peer Push Received", {
                            "leader_id": leader_id,
                            "push_start_time": peer_push_start_time,
                            "Pos": [round(x, 3) for x in pos],
                        })
                        if self.set_color:
                            self.set_color([0, 255, 0])
                        accumulated = np.array(peer_msg['accumulated_offset'])
                        if z is not None:
                            accumulated[2] = 0.0
                        hover_pos = peer_hover_start + accumulated
                        if z is not None:
                            hover_pos[2] = z
                        self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                        continue

                if np.linalg.norm(interaction_heading) > 0 > np.dot(vel, interaction_heading):
                    interact_vel = np.zeros(3)
                    speed = 0.0
                else:
                    interact_vel = vel

                target_pos = pos + interact_vel * dt * v_scalar
                accumulated_offset = target_pos - hover_pos_before_push

                if not detect_speed_threshold(speed):
                    interaction_heading = np.zeros(3)
                    send_time = time.time()
                    pub_socket.send_json({"type": "user_disengage", "drone_id": drone_id})
                    disengage_latency_ms = round((time.time() - send_time) * 1000, 2)
                    accumulated_offset = np.zeros(3)
                    if fric_coe > 0:
                        logger.info("Peer mode: switching to coasting.")
                        self._log_event("User Disengage", {
                            "leader_id": drone_id,
                            "reason": "coasting",
                            "Pos": [round(x, 3) for x in pos],
                            "latency_ms": disengage_latency_ms,
                        })
                        if self.set_color:
                            self.set_color([255, 255, 0])
                        status = 2
                    else:
                        logger.info("Peer mode: switching to grace hover.")
                        hover_pos = pos + interact_vel * dt
                        status = 3
                        tilt_angle = calculate_tilt(current_roll, current_pitch)
                        self._log_event("User Disengage", {
                            "leader_id": drone_id,
                            "speed": round(speed, 3),
                            "vel": [round(x, 3) for x in vel],
                            "Pos": [round(x, 3) for x in pos],
                            "Target": [round(x, 3) for x in hover_pos],
                            "Grace Period": grace_time,
                            "latency_ms": disengage_latency_ms,
                        })
                        if self.set_color:
                            self.set_color([255, 255, 0])
                    continue

                send_time = time.time()
                pub_socket.send_json({
                    "type": "push",
                    "drone_id": drone_id,
                    "accumulated_offset": accumulated_offset.tolist(),
                    "push_start_time": push_start_time,
                })
                push_latency_ms = round((time.time() - send_time) * 1000, 2)

                if base_attitude < 0:
                    self._log_event("User Pushing", {
                        "leader_id": drone_id,
                        "speed": round(speed, 3),
                        "vel": [round(x, 3) for x in vel],
                        "Pos": [round(x, 3) for x in pos],
                        "Target": [round(x, 3) for x in target_pos],
                        "latency_ms": push_latency_ms,
                    })
                    self.lo_commander.send_position_setpoint(target_pos[0], target_pos[1], target_pos[2], 0)
                else:
                    target_pitch, target_roll = base_attitude * np.array(
                        calculate_braking_angles(*interact_vel[:2], yaw_deg=current_yaw))
                    self._log_event("User Pushing", {
                        "leader_id": drone_id,
                        "speed": round(speed, 3),
                        "vel": [round(x, 3) for x in vel],
                        "Pos": [round(x, 3) for x in pos],
                        "latency_ms": push_latency_ms,
                    })
                    yaw_rate_cmd = max(min(-5.0 * current_yaw, 50.0), -50.0)
                    self.lo_commander.send_zdistance_setpoint(target_roll, target_pitch, yaw_rate_cmd, target_pos[2])

            elif status == 2:  # coasting
                end_pos, coast_t = self.calculate_coasting(pos, vel, fric_coe)
                self.lo_commander.send_notify_setpoint_stop()
                self.hl_commander.go_to(end_pos[0], end_pos[1], end_pos[2], 0, coast_t, relative=False)
                self._safe_sleep(coast_t)
                hover_pos = np.array(end_pos, dtype=float)
                send_time = time.time()
                pub_socket.send_json({"type": "grace_done", "drone_id": drone_id})
                self._log_event("Grace Done", {"leader_id": drone_id, "Pos": [round(x, 3) for x in hover_pos.tolist()],
                                               "latency_ms": round((time.time() - send_time) * 1000, 2)})
                if self.set_color:
                    self.set_color([227, 253, 255])
                status = 0
                continue

            elif status == 3:  # grace period
                grace_start = time.time()
                self.lo_commander.send_notify_setpoint_stop()
                self.hl_commander.go_to(hover_pos[0], hover_pos[1], hover_pos[2], 0, grace_time, relative=False)
                while time.time() < grace_time + grace_start:
                    self._safe_sleep(dt)
                send_time = time.time()
                pub_socket.send_json({"type": "grace_done", "drone_id": drone_id})
                self._log_event("Grace Done", {"leader_id": drone_id, "Pos": [round(x, 3) for x in hover_pos.tolist()],
                                               "latency_ms": round((time.time() - send_time) * 1000, 2)})
                if self.set_color:
                    self.set_color([227, 253, 255])
                status = 0

        self.lo_commander.send_notify_setpoint_stop()

    def interaction_follow_network(
            self,
            sub_socket,
            z=1,
            fric_coe=-1.0,
            base_attitude=1,
            duration=60,
            v_scalar=None,
    ):
        """Mirror the interaction drone's push state received over ZMQ.

        alpha_vel is always 1 for followers — velocity comes entirely from the
        network message, not from the drone's own state estimator.
        """
        if v_scalar is None:
            v_scalar = np.array([10, 10, 2])
        else:
            v_scalar = np.array(v_scalar)

        dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01

        def calculate_braking_angles(v_x, v_y, yaw_deg=0.0, base_att=base_attitude):
            yaw_rad = np.radians(yaw_deg)
            cos_y = np.cos(yaw_rad)
            sin_y = np.sin(yaw_rad)
            body_v_x = v_x * cos_y + v_y * sin_y
            body_v_y = -v_x * sin_y + v_y * cos_y

            pitch = np.sign(body_v_x) * base_att
            roll = -np.sign(body_v_y) * base_att
            pitch = max(min(pitch, 20), -20)
            roll = max(min(roll, 20), -20)
            return pitch, roll

        # Wait for first own position fix
        while True:
            try:
                last_pos = self._get_latest_pos()
                break
            except Exception:
                time.sleep(0.001)

        if z is not None:
            hover_pos = np.array([last_pos[0], last_pos[1], z], dtype=float)
        else:
            hover_pos = np.array([last_pos[0], last_pos[1], last_pos[2]], dtype=float)

        self.hl_commander.go_to(hover_pos[0], hover_pos[1], hover_pos[2], 0, 2)
        self._safe_sleep(2)

        logger.info("Starting Network Follow mode...")

        poller = zmq.Poller()
        poller.register(sub_socket, zmq.POLLIN)

        prev_remote_status = 0
        coasting_triggered = False
        start_time = time.time()

        while time.time() - start_time < duration:
            # Drain the socket and keep only the latest message
            msg = None
            while dict(poller.poll(0)).get(sub_socket):
                try:
                    msg = sub_socket.recv_json(flags=zmq.NOBLOCK)
                except zmq.Again:
                    break

            if msg is not None:
                remote_status = msg.get('status', 0)
                remote_vel = np.array(msg.get('vel', [0.0, 0.0, 0.0]))
            else:
                remote_status = prev_remote_status
                remote_vel = np.zeros(3)

            state = self._get_latest_drone_state() or {}
            current_yaw = state.get('stateEstimate.yaw', 0.0)

            pos = self._get_latest_pos()
            self.check_interaction_boundary(pos)
            if z is not None:
                pos[2] = z
                remote_vel[2] = 0.0

            if remote_status in (0, 3):
                # Interaction drone is hovering/in grace — hold own hover position
                if prev_remote_status == 1:
                    # Transition out of push: update hover to current position
                    hover_pos = pos.copy()
                self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                coasting_triggered = False

            elif remote_status == 1:
                target_pos = pos + remote_vel * dt * v_scalar
                if base_attitude < 0:
                    self.lo_commander.send_position_setpoint(target_pos[0], target_pos[1], target_pos[2], 0)
                else:
                    target_pitch, target_roll = base_attitude * np.array(
                        calculate_braking_angles(*remote_vel[:2], yaw_deg=current_yaw))
                    yaw_rate_cmd = max(min(-5.0 * current_yaw, 50.0), -50.0)
                    self.lo_commander.send_zdistance_setpoint(target_roll, target_pitch, yaw_rate_cmd, target_pos[2])

            elif remote_status == 2 and not coasting_triggered:
                # Execute coasting once per coasting phase
                end_pos, coast_t = self.calculate_coasting(pos, remote_vel, fric_coe)
                self.lo_commander.send_notify_setpoint_stop()
                self.hl_commander.go_to(end_pos[0], end_pos[1], end_pos[2], 0, coast_t, relative=False)
                hover_pos = np.array(end_pos, dtype=float)
                coasting_triggered = True
                self._safe_sleep(coast_t)

            prev_remote_status = remote_status
            self._safe_sleep(dt)

        self.lo_commander.send_notify_setpoint_stop()

    @staticmethod
    def calculate_virtual_stopping_distance(
            initial_speed,
            mass,
            friction_coefficient,
            drag_coefficient,
            frontal_area,
            air_density=1.225,
            fallback_distance=0.0,
            max_distance=None,
            dt=0.01,
            cur_pos=None,
            heading=None
    ):
        speed = max(float(initial_speed), 0.0)
        mass = float(mass)
        friction_coefficient = max(float(friction_coefficient), 0.0)
        drag_coefficient = max(float(drag_coefficient), 0.0)
        frontal_area = max(float(frontal_area), 0.0)
        air_density = max(float(air_density), 0.0)
        fallback_distance = max(float(fallback_distance), 0.0)

        if speed <= 0.0:
            return 0.0, []

        g = 9.81
        drag_lumped = 0.5 * air_density * drag_coefficient * frontal_area

        if mass <= 0.0:
            distance = fallback_distance
        else:
            if friction_coefficient > 0.0 and drag_lumped > 0.0:
                friction_force = friction_coefficient * mass * g
                distance = mass / (2.0 * drag_lumped) * np.log1p(
                    drag_lumped * speed ** 2 / friction_force
                )
            elif friction_coefficient > 0.0:
                distance = speed ** 2 / (2.0 * friction_coefficient * g)
            elif drag_lumped > 0.0:
                v_stop = 0.01
                distance = (mass / drag_lumped) * np.log(speed / v_stop) if speed > v_stop else 0.0
            else:
                distance = fallback_distance

        if not np.isfinite(distance):
            distance = fallback_distance
        if max_distance is not None:
            distance = min(distance, max(float(max_distance), 0.0))
        distance = max(float(distance), 0.0)

        trajectory = []
        if cur_pos is not None and heading is not None and mass > 0.0:
            heading_norm = np.linalg.norm(heading)
            if heading_norm > 0:
                h_dir = heading / heading_norm
                v = speed
                p = cur_pos.copy()
                dist_accum = 0.0
                
                while v > 0.06:
                    friction_force = friction_coefficient * mass * g
                    drag_force = drag_lumped * v**2
                    a = (friction_force + drag_force) / mass
                    
                    if a < 1e-5:
                        break
                        
                    v_next = v - a * dt
                    dt_actual = dt
                    if v_next < 0:
                        dt_actual = v / a
                        v_next = 0
                        
                    dp = (v * dt_actual) - 0.5 * a * (dt_actual**2)
                    
                    if max_distance is not None and dist_accum + dp >= max_distance:
                        dp = max_distance - dist_accum
                        p = p + h_dir * dp
                        trajectory.append({'pos': p.copy(), 'dt': dt_actual})
                        dist_accum += dp
                        break
                        
                    dist_accum += dp
                    p = p + h_dir * dp
                    trajectory.append({'pos': p.copy(), 'dt': dt_actual})
                    v = v_next
                
                # Override analytical distance with the actually integrated distance
                distance = dist_accum
                final_pos = cur_pos + h_dir * distance
                if trajectory:
                    trajectory.append({'pos': final_pos.copy(), 'dt': 0.0})

        return distance, trajectory

    def calculate_coasting(self, cur_pos, cur_vel, deceleration, fixZ=True):
        vx, vy, vz = cur_vel

        if fixZ:
            vz = 0
        speed = np.linalg.norm(cur_vel)

        if speed == 0:
            return cur_pos

        stopping_distance = (speed ** 2) / (2 * deceleration)
        time_to_stop = speed / deceleration

        ux = vx / speed
        uy = vy / speed
        uz = vz / speed

        end_x = cur_pos[0] + (ux * stopping_distance)
        end_y = cur_pos[1] + (uy * stopping_distance)
        if fixZ:
            end_z = cur_pos[2]
        else:
            end_z = cur_pos[2] + (uz * stopping_distance)

        return [end_x, end_y, end_z], time_to_stop

    def execute_commands(self, cmds):
        """
        Reads a JSON command log and executes it, routing to either
        the standard Commander or the HighLevelCommander.
        """
        commander_map = {
            "Commander": self.lo_commander,
            "HighLevelCommander": self.hl_commander,
        }

        logger.info(f"Executing log with {len(cmds)} entries...")
        start_real_time = time.time()

        for entry in cmds:
            target_log_time = entry['time']
            command_full_name = entry['command']
            args = entry['args']
            kwargs = entry['kwargs']

            current_elapsed = time.time() - start_real_time
            wait_duration = target_log_time - current_elapsed
            if wait_duration > 0:
                self._safe_sleep(wait_duration)

            parts = command_full_name.split(".")
            if len(parts) != 2:
                logger.info(f"Skipping malformed command: {command_full_name}")
                continue

            prefix, method_name = parts

            if prefix in commander_map:
                target_obj = commander_map[prefix]
                try:
                    method = getattr(target_obj, method_name)
                    method(*args, **kwargs)
                except AttributeError:
                    logger.info(f"Error: Method '{method_name}' not found on {prefix}")
            else:
                logger.info(f"Error: Unknown commander type '{prefix}'")

        logger.info("Execution finished.")

    # @Todo
    # def force_render(self):
    #     mission_setting = self.mission['drones'][self.args.drone_id]
    #     hover_pos = np.array(mission_setting['target'])
    #
    #     untracked_extra_marker = []
    #     for marker_name, marker_frame in self.extra_markers.items():
    #         if len(marker_frame) == 0:
    #             untracked_extra_marker.append(marker_name)
    #
    #     if untracked_extra_marker:
    #         untrack_info = ""
    #         for m_name in untracked_extra_marker:
    #             untrack_info += f"{m_name} "
    #         logger.info(f"Markers Not Captured: {untrack_info}")
    #         return
    #
    #     logger.info("Starting Force Render sequence")
    #
    #     dt = 1.0 / self.ctrl_rate if self.ctrl_rate else 0.01
    #     ZKp = 1
    #
    #     apparatus_pos = self._get_latest_extra_marker_center()
    #
    #     self.lo_commander.send_notify_setpoint_stop()
    #     self.hl_commander.go_to(0, 0, 1, 0, 5, relative=False)
    #     self._safe_sleep(5)
    #
    # def gimbal_test(self, test_time=30):
    #     dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01
    #     start_t = time.time()
    #
    #     self.lo_commander.send_setpoint(0.0, 0.0, 0.0, 0)
    #     while time.time() - start_t < test_time:
    #         # self.lo_commander.send_hover_setpoint(0.0, 0.0, 0.0, 0.9295)
    #         self.lo_commander.send_setpoint(0.0, 0.0, 0.0, 35000)
    #         # self.lo_commander.send_position_setpoint(target_x, target_y, target_z, 0)
    #         self._safe_sleep(dt)
    #
    #     logger.info("Gimbal Test Finished")
    #
    # def pwm_swift(self, wait_time=0.5):
    #     start_time = time.time()
    #
    #     for PWM in np.linspace(10000, 60000, 6, endpoint=True):
    #         self._set_pwm_all(PWM)
    #         self._safe_sleep(wait_time)
    #     self._stop_pwm_override()
    #
    # def motor_test(self, test_time=10):
    #     self._set_pwm_all(20000)
    #     self._safe_sleep(test_time)
    #     self._stop_pwm_override()
    #

    # def test_movement_threshold(self, test_pwm=10000, pwm_step=1000, duration=1.0):
    #     """
    #     Applies a fixed PWM and reports if the marker moved more than 1mm.
    #     """
    #     logger.info(f"Starting movement test: PWM={test_pwm} for {duration}s")
    #
    #     # 1. Record starting position
    #     initial_pos = np.array(self._get_latest_extra_marker_center())
    #
    #     try:
    #         start_time = time.time()
    #         while (time.time() - start_time) < duration:
    #             # Apply the constant test signal
    #
    #             cur_pos = np.array(self._get_latest_extra_marker_center())
    #             if np.linalg.norm(cur_pos - initial_pos) > 0.01:
    #                 break
    #
    #             self._set_pwm_all(test_pwm)
    #             test_pwm += pwm_step
    #             self._safe_sleep(0.1)  # 100Hz update
    #
    #
    #     finally:
    #         # Always stop motors after the test
    #         self._stop_pwm_override()
    #
    #     logger.info(f"Movement Detected by PWM: {test_pwm}")
    #     return
    #
    # def render_stiffness(self, K, duration, sys_friction=0.7164, direction_vec=None, displacement_threshold=0.0004,
    #                      alpha=0.8, ground_test=False):
    #     if direction_vec is None:
    #         direction_vec = [-1, 0, 0]
    #
    #     logger.info(f"Starting stiffness rendering: K={K} for {duration}s")
    #     direction_unit = np.array(direction_vec) / np.linalg.norm(direction_vec)
    #
    #     # 1. Record the initial position of the extra marker center
    #     prev_pos, prev_time = self._get_latest_extra_marker_center(timestamp=True)
    #     init_pos = prev_pos.copy()
    #     start_time = time.time()
    #     # Determine loop rate based on mocap FPS
    #     dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01
    #     v_filtered = 0
    #     try:
    #         while (time.time() - start_time) < duration:
    #             loop_start = time.time()
    #
    #             # Get current drone position from mocap
    #             cur_pos, cur_time = self._get_latest_extra_marker_center(timestamp=True)
    #
    #             # --- Calculate Stiffness Force ---
    #             # F = K * Δx (Linear displacement from the recorded initial center)
    #             displacement_vec = cur_pos - init_pos
    #             proj_dist = np.dot(displacement_vec, direction_unit)
    #
    #             time_step = cur_time - prev_time
    #             if time_step <= 0:
    #                 continue
    #             v_raw = proj_dist / time_step
    #
    #             v_scalar = (alpha * v_raw) + ((1 - alpha) * v_filtered)
    #             v_filtered = v_scalar
    #
    #             # if proj_dist - last_displacement <= displacement_threshold:
    #             #     friction_compensation = sys_friction
    #             # elif proj_dist - last_displacement > displacement_threshold:
    #             #     friction_compensation = -sys_friction
    #             # else:
    #             #     friction_compensation = 0
    #
    #             friction_compensation = sys_friction
    #
    #             if proj_dist > displacement_threshold:
    #                 f_stiffness = K * proj_dist + friction_compensation
    #             else:
    #                 f_stiffness = 0.0
    #
    #             # Convert the calculated force to PWM
    #             pwm_value = self.force_to_pwm(f_stiffness)
    #
    #             # Apply PWM to all motors via the controller
    #
    #             if not ground_test:
    #                 self._set_pwm_all(pwm_value)
    #
    #             now = time.time()
    #             self._log_event("Rendering Force",
    #                             {'type': 'stiffness', 'force': f_stiffness, 'displacement': proj_dist,
    #                              'vel': v_filtered, 'time': now})
    #
    #             # Maintain consistent update frequency
    #             elapsed = now - loop_start
    #             if elapsed < dt:
    #                 self._safe_sleep(dt - elapsed)
    #
    #     finally:
    #         # 2. Stop PWM override at the end of the duration
    #         self._stop_pwm_override()
    #         logger.info("Stiffness rendering complete. PWM override stopped.")
    #
    # def render_Karnopp_friction(self, Cp, Dp, delta_v, duration, sys_friction=0.9, direction_vec=None, wait_time=-1,
    #                             alpha=0.3, min_displacement=0.0006, ground_test=False):
    #     """
    #     Simulates friction following the Karnopp model with a conditional Low Pass Filter.
    #     alpha: Smoothing factor (0.0 to 1.0). Lower is smoother/slower.
    #     """
    #     if direction_vec is None:
    #         direction_vec = [-1, 0, 0]
    #
    #     logger.info(f"Starting Karnopp friction: Cp={Cp}, Dp={Dp}, dv={delta_v}")
    #     direction_unit = np.array(direction_vec) / np.linalg.norm(direction_vec)
    #
    #     prev_pos, prev_time = self._get_latest_extra_marker_center(timestamp=True)
    #     init_pos = prev_pos.copy()
    #     start_time = time.time()
    #
    #     dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01
    #     wait_threshold = self.ctrl_rate * wait_time
    #     wait_count = 0
    #
    #     v_filtered = 0.0
    #     has_broken_static = False
    #
    #     try:
    #         while (time.time() - start_time) < duration:
    #             loop_start = time.time()
    #
    #             cur_pos, cur_time = self._get_latest_extra_marker_center(timestamp=True)
    #             time_step = cur_time - prev_time
    #             if time_step <= 0:
    #                 continue
    #
    #             total_displacement_vec = cur_pos - init_pos
    #             total_displacement = np.dot(total_displacement_vec, direction_unit)
    #
    #             # 1. Calculate raw velocity
    #             displacement_vec = cur_pos - prev_pos
    #             displacement = np.dot(displacement_vec, direction_unit)
    #             v_raw = displacement / time_step
    #
    #             # 2. Check for first-time activation
    #             if not has_broken_static and total_displacement > 0.01:
    #                 has_broken_static = True
    #                 v_filtered = v_raw  # Seed the filter with the current velocity
    #
    #             # 3. Apply Low Pass Filter only if activated
    #             if has_broken_static:
    #                 v_scalar = (alpha * v_raw) + ((1 - alpha) * v_filtered)
    #                 v_filtered = v_scalar
    #             else:
    #                 v_filtered = v_raw
    #
    #             # 4. Apply Karnopp Model logic
    #             if total_displacement > 0.01 and displacement > min_displacement:
    #                 if abs(v_filtered) > delta_v:
    #                     f_friction = Cp
    #                     wait_count = 0
    #                 else:
    #                     f_friction = Dp if abs(v_filtered) > 0 else 0
    #
    #             else:
    #                 v_filtered = 0
    #                 if 0 < wait_threshold <= wait_count:
    #                     if displacement > min_displacement:
    #                         f_friction = 0
    #                         wait_count = 0
    #                     else:
    #                         f_friction = sys_friction
    #                 else:
    #                     f_friction = 0
    #                     if has_broken_static:
    #                         wait_count += 1
    #
    #             # 5. Apply directional mask and render
    #             final_force = max(0, f_friction)
    #             pwm_value = self.force_to_pwm(final_force)
    #
    #             if not ground_test:
    #                 self._set_pwm_all(pwm_value)  # Uncomment to apply
    #             logger.info(
    #                 f"Displacement: {total_displacement:6.4f}, Delta P: {displacement:6.4f}, V_filt: {v_filtered:6.4f}, Force: {final_force:4.3f}")
    #
    #             now = time.time()
    #             self._log_event("Rendering Force",
    #                             {'type': 'friction', 'force': final_force, 'displacement': total_displacement,
    #                              'vel': v_filtered, 'time': now})
    #
    #             prev_pos, prev_time = cur_pos, cur_time
    #             elapsed = now - loop_start
    #             if elapsed < dt:
    #                 self._safe_sleep(dt - elapsed)
    #
    #     finally:
    #         self._stop_pwm_override()
    #         logger.info("Friction rendering complete.")
