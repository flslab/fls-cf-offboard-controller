import logging
import threading
import time
import traceback
from copy import deepcopy

import cflib.crazyflie
import numpy as np
import zmq

from Interaction.command_wrapper import CommandWrapper
from Interaction.braking_response_calibration import (
    PlanarBrakingCalibration,
)
from Interaction.position_capture_calibration import PositionCaptureCalibration
from Interaction.calibration_trial_readiness import CalibrationTrialReadinessGate
from Interaction.flight_behaviors import load_commands
from Interaction.onboard_wrench_interaction_pipeline import OnboardMomentumWrenchPipeline
from Interaction.potentiometer_force_sensor import (
    PotentiometerContactDetector,
    PotentiometerReleaseDetector,
)
from Interaction.wrench_interaction_pipeline import WrenchInteractionPipeline
from Interaction.wrench_model_calibration import (
    DEFAULT_CALIBRATION_PATH,
    apply_drone_calibration,
    identify_planar_braking_response,
    identify_xyz_alignment,
    planar_braking_fit_is_current,
    save_drone_calibration,
)

# from Interaction.collision_avoidance.simulation import apf_velocity

logger = logging.getLogger(__name__)


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
    ):
        self.enabled = bool(enabled)
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

    def reset(self):
        """Require a new stationary dwell before another interaction."""
        self.armed = not self.enabled
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
    """Switch translation through contact, braking, and position-hold modes."""

    POSITION_HOLD = 'position_hold'
    CONTACT_POSITION = 'position_interaction'
    CONTACT_ZDISTANCE = 'attitude_zdistance'
    ATTITUDE_COAST = 'attitude_coast'
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
            coast_command_period_s=0.02,
            coast_command_acceleration_deadband_m_s2=0.02,
            coast_candidate_tail_cancellation_max_acceleration_m_s2=1.0,
            coast_acceleration_filter_time_constant_s=0.08,
            coast_handoff_speed_m_s=0.04,
            coast_handoff_max_lateral_speed_m_s=0.15,
            coast_handoff_max_tilt_deg=3.0,
            coast_handoff_max_acceleration_m_s2=0.35,
            coast_alignment_position_tolerance_m=0.04,
            coast_alignment_velocity_tolerance_m_s=0.08,
            coast_alignment_dwell_s=0.05,
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
        self._coast_previous_velocity_xy = None
        self._coast_previous_timestamp = None
        self._coast_filtered_acceleration_xy = np.zeros(2)
        self._coast_model_acceleration_xy = np.zeros(2)
        self._coast_acceleration_valid = False
        self._coast_command_history = []
        self._last_attitude_send_timestamp = None
        self._attitude_send_intervals_s = []
        self._level_attitude_command_started_at = None
        self._tail_neutralization_deadline = None
        self._tail_neutralization_needs_send_anchor = False
        self.release_candidate_action = None
        self.release_candidate_predicted_level_terminal_speed_m_s = None
        self.release_candidate_tail_cancellation_acceleration_m_s2 = None
        self.release_candidate_tail_cancellation_signed_acceleration_m_s2 = None
        self.release_candidate_target_terminal_speed_m_s = None
        self.release_candidate_predicted_terminal_after_pulse_m_s = None
        self.release_candidate_command_hold_s = None

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

    def _transition_mode(self, new_mode):
        self.mode = new_mode
        logger.info({
            self.CONTACT_POSITION: 'HANDLING INTERACTION',
            self.CONTACT_ZDISTANCE: 'HANDLING INTERACTION',
            self.ATTITUDE_COAST: 'COASTING WITH ATTITUDE',
            self.POSITION_COAST: 'COASTING',
            self.ATTITUDE_BRAKING: 'BRAKING',
            self.POSITION_HOLD: 'HOVER',
        }[new_mode])

    def start_contact(self, render_mode='orientation', current_position=None):
        # Detector residuals during braking are expected controller/model
        # transients. Do not let them chatter the command mode.
        if self.shadow_mode or self.mode != self.POSITION_HOLD:
            return False
        if render_mode not in ('position', 'orientation'):
            raise ValueError('contact render mode must be position or orientation')
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
        self._coast_previous_velocity_xy = None
        self._coast_previous_timestamp = None
        self._coast_filtered_acceleration_xy.fill(0.0)
        self._coast_model_acceleration_xy.fill(0.0)
        self._coast_acceleration_valid = False
        self._last_attitude_send_timestamp = None
        self._attitude_send_intervals_s = []
        self._level_attitude_command_started_at = None
        self._tail_neutralization_deadline = None
        self._tail_neutralization_needs_send_anchor = False
        self.release_candidate_action = None
        self.release_candidate_predicted_level_terminal_speed_m_s = None
        self.release_candidate_tail_cancellation_acceleration_m_s2 = None
        self.release_candidate_tail_cancellation_signed_acceleration_m_s2 = None
        self.release_candidate_target_terminal_speed_m_s = None
        self.release_candidate_predicted_terminal_after_pulse_m_s = None
        self.release_candidate_command_hold_s = None
        self._transition_mode(
            self.CONTACT_POSITION
            if render_mode == 'position' else self.CONTACT_ZDISTANCE
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
                self.CONTACT_POSITION, self.CONTACT_ZDISTANCE):
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
        self._coast_previous_velocity_xy = velocity[:2].copy()
        self._coast_previous_timestamp = timestamp
        self._coast_filtered_acceleration_xy.fill(0.0)
        self._coast_model_acceleration_xy.fill(0.0)
        self._coast_acceleration_valid = False
        self._tail_neutralization_deadline = None
        self._tail_neutralization_needs_send_anchor = False
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
            self.ATTITUDE_COAST if coast else self.ATTITUDE_BRAKING
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
        self._coast_previous_velocity_xy = None
        self._coast_previous_timestamp = None
        self._coast_filtered_acceleration_xy.fill(0.0)
        self._coast_model_acceleration_xy.fill(0.0)
        self._coast_acceleration_valid = False
        self._tail_neutralization_deadline = None
        self._tail_neutralization_needs_send_anchor = False
        self.release_candidate_action = None
        self.release_candidate_predicted_level_terminal_speed_m_s = None
        self.release_candidate_tail_cancellation_acceleration_m_s2 = None
        self.release_candidate_tail_cancellation_signed_acceleration_m_s2 = None
        self.release_candidate_target_terminal_speed_m_s = None
        self.release_candidate_predicted_terminal_after_pulse_m_s = None
        self.release_candidate_command_hold_s = None
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
        self._release_candidate_mode = None

    def update_coast_attitude(
            self,
            current_position,
            current_velocity,
            target_position,
            target_velocity,
            timestamp,
            current_orientation_rpy=None,
            allow_position_handoff=True,
            command_timestamp=None,
    ):
        """Brake with attitude toward a stop target frozen at release.

        Returns true only on the sample that transitions to position control.
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
        xy_speed = float(np.linalg.norm(velocity[:2]))
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
            longitudinal_speed = abs(self.brake_projected_speed_m_s)
            lateral_speed = float(np.linalg.norm(lateral_velocity_xy))
        else:
            # With no trustworthy release axis, retain the original full-XY
            # stop gate and latch the complete measured position at handoff.
            brake_direction_xy = np.zeros(2)
            longitudinal_speed = xy_speed
            lateral_speed = 0.0
        self.coast_lateral_speed_m_s = lateral_speed
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
        measured_acceleration_norm = float(np.linalg.norm(
            self._coast_filtered_acceleration_xy
        ))
        planned_command_level = bool(
            np.linalg.norm(tracking['command_acceleration_m_s2']) <= 1e-9
        )
        history_command_level = bool(
            self._coast_command_history
            and np.linalg.norm(self._coast_command_history[-1][1]) <= 1e-9
        )
        self.coast_response_queue_settle_required_s = float(
            self.coast_attitude_response_delay_s
            + 4.0 * self.coast_attitude_time_constant_s
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
        predicted_terminal_safe = bool(
            tracking['predicted_level_terminal_speed_m_s'] is None
            or abs(tracking['predicted_level_terminal_speed_m_s'])
            <= self.coast_handoff_speed_m_s
        )
        self.coast_response_queue_settled = bool(
            planned_command_level
            and history_command_level
            and self.coast_response_queue_settle_elapsed_s is not None
            and self.coast_response_queue_settle_elapsed_s
            >= self.coast_response_queue_settle_required_s
            and predicted_terminal_safe
        )
        motion_reversed = bool(
            np.linalg.norm(self.brake_direction[:2]) > 1e-9
            and self.brake_projected_speed_m_s <= 0.0
        )
        self.coast_handoff_state_ready = bool(
            self._coast_acceleration_valid
            and longitudinal_speed <= self.coast_handoff_speed_m_s
            and lateral_speed <= self.coast_handoff_max_lateral_speed_m_s
            and self.coast_actual_tilt_deg <= self.coast_handoff_max_tilt_deg
            and measured_acceleration_norm
            <= self.coast_handoff_max_acceleration_m_s2
            and self.coast_response_queue_settled
        )
        if self.coast_handoff_state_ready:
            if self._coast_alignment_since is None:
                self._coast_alignment_since = timestamp
        else:
            self._coast_alignment_since = None
        handoff_ready = bool(
            self._coast_alignment_since is not None
            and timestamp - self._coast_alignment_since
            >= self.coast_alignment_dwell_s
        )
        if not handoff_ready or not bool(allow_position_handoff):
            # A timeout is diagnostic only. Switching a still-moving vehicle
            # to position control would recreate the pullback failure, so keep
            # applying bounded dissipative attitude damping until longitudinal
            # speed and the bounded lateral handoff envelope are acceptable.
            if attitude_timed_out:
                self.coast_handoff_reason = 'waiting_for_actual_stop_after_timeout'
            return False

        if motion_reversed:
            self.coast_handoff_reason = (
                'motion_reversed_state_settled'
            )
        else:
            self.coast_handoff_reason = (
                'actual_state_settled_after_timeout'
                if attitude_timed_out else 'actual_state_settled'
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
        self.coast_target_clamped_to_actual = target_is_behind
        if target_is_behind:
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
    def position_interaction_mode(self):
        return self.mode == self.CONTACT_POSITION

    @property
    def braking_mode(self):
        return self.mode in (
            self.ATTITUDE_COAST,
            self.POSITION_COAST,
            self.ATTITUDE_BRAKING,
        )

    @property
    def uses_position_setpoint(self):
        return self.shadow_mode or self.mode in (
            self.POSITION_HOLD,
            self.CONTACT_POSITION,
            self.POSITION_COAST,
        )

    @property
    def command_mode(self):
        if self.shadow_mode:
            return 'shadow_position_hold'
        return self.mode

    def send(self, commander, command_timestamp=None, yaw_deg=None):
        if self.shadow_mode or self.mode in (
                self.POSITION_HOLD,
                self.CONTACT_POSITION,
                self.POSITION_COAST):
            commander.send_position_setpoint(
                *self.hold_position, self.yaw_deg
            )
        elif self.mode in (
            self.CONTACT_ZDISTANCE,
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
            self._record_attitude_command(
                sent_at,
                self._pending_attitude_yaw_deg
                if yaw_deg is None else yaw_deg,
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

    def _run_translation(self, calibration_mode=False) -> None:
        """Run model-based interaction, with a legacy velocity-mode fallback."""
        try:
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
                    calibration_mode=calibration_mode,
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
                    wrench_config = deepcopy(wrench_config)
                    wrench_config['shadow_mode'] = True
                    wrench_config['startup_bias_calibration_enabled'] = True
                    wrench_config.setdefault('calibration_excitation', {})[
                        'enabled'
                    ] = True
                    excitation = wrench_config['calibration_excitation']
                    excitation_end_s = (
                        float(excitation.get('start_delay_s', 1.0))
                        + float(excitation.get('duration_s', 30.0))
                    )
                    braking_config = wrench_config.setdefault(
                        'planar_braking_calibration', {}
                    )
                    braking_config['enabled'] = True
                    braking_plan = PlanarBrakingCalibration(
                        braking_config,
                        start_after_s=excitation_end_s,
                    )
                    # Retired experiment: old mission settings must not
                    # silently re-enable position-capture trials.
                    wrench_config.pop('position_capture_calibration', None)
                    # Validate readiness timing before entering any maneuver.
                    CalibrationTrialReadinessGate(braking_config)
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
                    interaction_duration = translation_setting['duration']
                    if saved_calibration is None:
                        logger.warning(
                            'No saved wrench model calibration for %s at %s; '
                            'using mission/default alignment parameters.',
                            self.drone_id, calibration_path,
                        )
                    else:
                        logger.info('Loaded wrench calibration: %s', calibration_path)
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
                        'Calibration includes %d open-loop planar trials at '
                        'tilt levels %s deg (maximum %.1f); keep the full '
                        '%.2f m XY safety radius clear and do not touch the '
                        'vehicle.',
                        len(braking_plan.trial_directions),
                        ', '.join(
                            f'{value:g}'
                            for value in braking_plan.tilt_levels_deg
                        ),
                        braking_plan.tilt_deg,
                        braking_plan.max_displacement_m,
                    )
                    logger.warning(
                        'Calibration uses paired acceleration/braking durations '
                        '%s s; total scheduled motion time %.1fs plus startup '
                        'bias collection and readiness holds. No position '
                        'capture trials are run.',
                        braking_plan.trial_accelerate_s.tolist(),
                        interaction_duration,
                    )
                interaction_function = (
                    self.interaction_onboard_wrench_admittance
                    if detection_method == 'momentum_impulse'
                    else self.interaction_wrench_admittance
                )
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
                    calibration_path=calibration_path,
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
            if calibration_mode:
                raise
        finally:
            if calibration_mode:
                try:
                    target_z = float(
                        self.mission['drones'][self.drone_id]['target'][2]
                    )
                    self.lo_commander.send_zdistance_setpoint(
                        0.0, 0.0, 0.0, target_z
                    )
                except Exception:
                    logger.exception(
                        'Could not send level attitude while ending calibration'
                    )
            self.lo_commander.send_notify_setpoint_stop()

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
            calibration_path=DEFAULT_CALIBRATION_PATH,
    ):
        """Run wrench interaction from synchronized onboard state estimates.

        This is intentionally separate from ``interaction_wrench_admittance``.
        The original full-pose mocap/Kalman observer path remains available by
        selecting ``state_source: mocap``.
        """
        pipeline = OnboardMomentumWrenchPipeline(config)
        config = pipeline.config
        force_sensor_available = bool(
            getattr(self, 'force_sensor', None) is not None
            and not calibration_mode
        )
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
        force_max_attitude_deg = _validated_attitude_limit(
            force_max_attitude_deg, 'force rendering'
        )
        configured_release_mode = release_mode
        release_mode = resolve_release_mode(
            configured_release_mode,
            force_sensor_available,
            calibration_mode=calibration_mode,
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
                                'target_aware_no_pullback'
                                if release_mode == 'potentiometer_coast'
                                else None
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
                    and state_skew <= max_state_group_skew_s
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
        excitation_config = config['calibration_excitation']
        excitation_end_s = (
            float(excitation_config['start_delay_s'])
            + float(excitation_config['duration_s'])
        )
        planar_braking_config = config['planar_braking_calibration']
        planar_braking_plan = PlanarBrakingCalibration(
            planar_braking_config,
            start_after_s=excitation_end_s,
        )
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
                if planar_attitude_active:
                    self.lo_commander.send_zdistance_setpoint(
                        0.0, 0.0, 0.0, float(nominal_position[2])
                    )
                raise StaleLocalizationError('Onboard state packet set is incomplete')
            state_time = state['time']
            state_age = now - state_time
            if state_age < -0.5:
                check_trial_wait(now, invalidate=True)
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
                if planar_attitude_active:
                    self.lo_commander.send_zdistance_setpoint(
                        0.0, 0.0, 0.0, float(nominal_position[2])
                    )
                raise StaleLocalizationError(
                    'Onboard state-group skew contains an invalid value'
                )
            state_group_skew = float(np.max(state_group_skew_values))
            if state_group_skew > max_state_group_skew_s:
                check_trial_wait(now, invalidate=True)
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
                if (
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
            last_state_time = state_time

            position = state['position']
            self.check_interaction_boundary(position)
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
                state['motor_skew_s'] is None
                or state['motor_skew_s'] > max_motor_state_skew_s
            )
            if safety['require_motor_data'] and (
                motor_pwm is None
                or battery_voltage is None
                or motor_is_stale
                or motor_is_unsynchronized
            ):
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
                    'velocity_xy': output.estimate.velocity[:2].tolist(),
                    'position_xy': position[:2].tolist(),
                    'battery_voltage_V': battery_voltage,
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
                                initial_contact_gate.armed
                                and translation_control.mode
                                == translation_control.POSITION_HOLD
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
                self._log_event('Waiting For User Interaction', {
                    'contact_detector_armed': initial_contact_gate.armed,
                    'max_xy_speed_m_s': (
                        initial_contact_gate.max_xy_speed_m_s
                    ),
                    'stationary_dwell_s': (
                        initial_contact_gate.stationary_dwell_s
                    ),
                    'state_source': 'crazyflie_state_estimate',
                })
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
            ):
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
                        self._bounded_wrench_reference(position)):
                    pipeline.admittance.reset()
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
                                        self._bounded_wrench_reference(position)):
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
                    potentiometer_release_processed = True
                    potentiometer_release_pending = False
                    candidate_release_force_world = None
                    candidate_release_direction = None
                    candidate_release_attitude_deg = None
                    candidate_release_sensor_stale_logged = False
                    if potentiometer_contact_detector is not None:
                        potentiometer_contact_detector.mark_released()
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
                            'force_memory_s': release_force_memory_s,
                            'unloaded_elapsed_s': (
                                potentiometer_release_decision
                                .unloaded_elapsed_s
                            ),
                            'initial_command_mode': (
                                translation_control.command_mode
                            ),
                            'state_source': 'crazyflie_state_estimate',
                        },
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
            if translation_control.mode in (
                    translation_control.ATTITUDE_COAST,
                    translation_control.POSITION_COAST):
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
                coast_handoff_completed = bool(
                    translation_control.mode
                    == translation_control.ATTITUDE_COAST
                    and translation_control.update_coast_attitude(
                        self._bounded_wrench_reference(position),
                        output.estimate.velocity,
                        predicted_stop_position,
                        braking_kwargs['coast_velocity'],
                        state_time,
                        output.estimate.orientation_rpy,
                        allow_position_handoff=(
                            potentiometer_release_processed
                        ),
                        command_timestamp=attitude_command_planned_at,
                    )
                )
                if coast_handoff_completed:
                    self._log_event(
                        'Coast Position Control Handoff',
                        {
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
                            'target_velocity_m_s': [0.0, 0.0, 0.0],
                            'virtual_target_velocity_m_s': (
                                braking_kwargs['coast_velocity'].tolist()
                            ),
                            'virtual_position_error_m': (
                                translation_control
                                .coast_tracking_position_error_m.tolist()
                            ),
                            'virtual_velocity_error_m_s': (
                                translation_control
                                .coast_tracking_velocity_error_m_s.tolist()
                            ),
                            'position_pullback_disabled': True,
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
                            'command_mode': (
                                translation_control.command_mode
                            ),
                            'state_source': 'crazyflie_state_estimate',
                        },
                    )

            braking_completed = coast_handoff_completed
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
                (
                    contact_detection_source == 'potentiometer'
                    or contacts is None
                    or not contacts.translation.active
                )
                and translation_control.consume_detector_rearm(state_time)
            ):
                pipeline.detector.translation.reset(state_time)
                initial_contact_gate.reset()
                if potentiometer_contact_detector is not None:
                    potentiometer_contact_detector.reset()
                if potentiometer_release_detector is not None:
                    potentiometer_release_detector.disarm()
                self._log_event(
                    'Translation Contact Detector Rearm Started',
                    {
                        'rearm_delay_s': translation_control.rearm_delay_s,
                        'requires_stationary_dwell': (
                            initial_contact_gate.enabled
                        ),
                        'state_source': 'crazyflie_state_estimate',
                    },
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
                translation_control.send(
                    self.lo_commander,
                    yaw_deg=np.degrees(output.estimate.orientation_rpy[2]),
                )
            else:
                command_position = translation_control.hold_position.copy()
                command_yaw = translation_control.yaw_deg
                last_command_position = command_position.copy()
                last_command_yaw = float(command_yaw)
                translation_control.send(self.lo_commander)

            estimate = output.estimate
            raw = output.raw_estimate
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
                    'target_aware_no_pullback'
                    if release_mode == 'potentiometer_coast' else None
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
                    else (translation_control.hover_z
                          if not translation_control.uses_position_setpoint
                          else None)
                ),
                'command_roll_deg': (
                    calibration_attitude_command.roll_deg
                    if calibration_attitude_command is not None
                    else (
                        translation_control.contact_roll_deg
                        if not translation_control.uses_position_setpoint
                        else None
                    )
                ),
                'command_pitch_deg': (
                    calibration_attitude_command.pitch_deg
                    if calibration_attitude_command is not None
                    else (
                        translation_control.contact_pitch_deg
                        if not translation_control.uses_position_setpoint
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
                'command_xy_velocity_m_s': None,
                'command_xy_velocity_world_m_s': None,
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
                'coast_handoff_reason': (
                    translation_control.coast_handoff_reason
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
            self._safe_sleep(max(dt - (time.time() - now), 0.0))
            if (calibration_mode and interaction_start is not None
                    and not calibration_wait_this_cycle):
                # Advance the calibration protocol only after one complete,
                # fresh, synchronized sample/control cycle. A telemetry pause
                # therefore cannot skip excitation or attitude phases, nor
                # make the duration check accept an incomplete fit.
                next_elapsed_s = calibration_elapsed_s + dt
                # Never step past an unadmitted start boundary: otherwise a
                # short level phase could be skipped while waiting for a trial.
                for boundary_s, _label, key, gate, _plan in calibration_trial_boundaries:
                    if (not gate.admitted(key)
                            and calibration_elapsed_s < boundary_s <= next_elapsed_s):
                        next_elapsed_s = min(next_elapsed_s, boundary_s)
                calibration_elapsed_s = next_elapsed_s

        if calibration_mode:
            fit = identify_xyz_alignment(
                model_calibration_samples,
                window_s=float(config['impulse_estimator']['window_s']),
            )
            planar_braking_fit = None
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
                )
                planar_braking_fit['protocol'] = {
                    **planar_braking_plan.timing_protocol(),
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
                braking_battery_samples = np.asarray([
                    sample['battery_voltage_V']
                    for sample in planar_braking_samples
                    if sample.get('battery_voltage_V') is not None
                    and np.isfinite(sample['battery_voltage_V'])
                ], dtype=float)
                if len(braking_battery_samples):
                    planar_braking_fit['battery_voltage_V'] = {
                        'minimum': round(float(np.min(
                            braking_battery_samples
                        )), 4),
                        'mean': round(float(np.mean(
                            braking_battery_samples
                        )), 4),
                        'maximum': round(float(np.max(
                            braking_battery_samples
                        )), 4),
                    }
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
            saved_path, saved_entry = save_drone_calibration(
                self.drone_id,
                fit,
                config['motor_model'],
                calibration_path,
                planar_braking_fit=planar_braking_fit,
                position_capture_fit=position_capture_fit,
            )
            self._log_event('Wrench Model Calibration Saved', {
                'state_source': 'crazyflie_state_estimate',
                'path': str(saved_path),
                'impulse_estimator': saved_entry['impulse_estimator'],
                'fit': fit,
                'control_handoff': saved_entry.get('control_handoff'),
                'planar_braking_fit': planar_braking_fit,
                'position_capture_fit': position_capture_fit,
            })
            logger.info('CALIBRATED %s', saved_path)
        else:
            self._log_event('Wrench Interaction Complete', {
                'state_source': 'crazyflie_state_estimate',
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
