"""Timed, repeated level-attitude interaction, independent of firmware braking."""

from copy import deepcopy
import math
import time

import numpy as np

from Interaction.onboard_wrench_interaction_pipeline import OnboardMomentumWrenchPipeline
from Interaction.potentiometer_force_sensor import (
    PotentiometerContactDetector, PotentiometerReleaseDetector,
)
from Interaction.wrench_model_calibration import (
    DEFAULT_CALIBRATION_PATH, apply_detection_calibration,
)


def _number(value, name, *, positive=False):
    result = float(value)
    if not math.isfinite(result) or (result <= 0 if positive else result < 0):
        raise ValueError(f'{name} must be finite and {"positive" if positive else "non-negative"}')
    return result


def validate_level_coast(config, *, sensor_available):
    """Validate this opt-in behavior before any flight commands are sent."""
    options = dict(config.get('level_coast') or {})
    detector = options.get('detector', 'potentiometer')
    if detector not in ('vel', 'model', 'potentiometer'):
        raise ValueError('level_coast.detector must be vel, model or potentiometer')
    wrench = config.get('wrench_interaction') or {}
    if (config.get('detection_method') != 'momentum_impulse'
            or wrench.get('state_source') != 'onboard'):
        raise ValueError('level_coast requires momentum_impulse / onboard state logging; '
                         'select the contact detector with level_coast.detector')
    if wrench.get('shadow_mode', True):
        raise ValueError('level_coast requires wrench_interaction.shadow_mode: false')
    if (wrench.get('firmware_auto_brake') or {}).get('enabled', False):
        raise ValueError('level_coast requires firmware_auto_brake.enabled: false')
    if (wrench.get('contact_attitude_shadow_enabled', False)
            or (wrench.get('post_release_estimator_control') or {}).get('enabled', False)):
        raise ValueError('level_coast cannot share a Pi release/EKF experiment')
    if detector == 'potentiometer' and not sensor_available:
        raise ValueError('level_coast potentiometer detection requires --sense')
    if detector == 'potentiometer':
        _potentiometer_detectors(config)
    options['duration_s'] = _number(config['duration'], 'duration', positive=True)
    options['grace_s'] = _number(config.get('grace_time', .5), 'grace_time')
    options['stop_speed_m_s'] = _number(
        options.get('stop_speed_m_s', .03), 'stop_speed_m_s', positive=True)
    options['detector'] = detector
    velocity = dict(options.get('velocity') or {})
    for key, default, positive in (
            ('onset_speed_m_s', .10, True), ('release_speed_m_s', .08, True),
            ('onset_dwell_s', .03, False), ('release_dwell_s', .05, False),
            ('max_sample_gap_s', .10, True)):
        velocity[key] = _number(velocity.get(key, default), key, positive=positive)
    if velocity['release_speed_m_s'] >= velocity['onset_speed_m_s']:
        raise ValueError('velocity release speed must be below onset speed')
    options['velocity'] = velocity
    return options


def _potentiometer_detectors(config):
    virtual = config.get('virtual_object') or {}
    contact = virtual.get('contact_detection') or {}
    release = virtual.get('release_behavior') or {}
    contact_force = _number(contact.get('force_threshold_n', .18), 'force_threshold_n', positive=True)
    unloaded_force = _number(release.get('unloaded_force_n', .17), 'unloaded_force_n')
    if unloaded_force >= contact_force:
        raise ValueError('potentiometer unloaded force must be below contact threshold')
    contact_detector = PotentiometerContactDetector(
        contact_force, contact.get('onset_dwell_s', .03),
        release.get('max_sample_gap_s', .15))
    release_keys = ('force_drop_n', 'decrease_rate_n_s', 'unloaded_dwell_s',
                    'max_sample_gap_s', 'candidate_stall_timeout_s', 'candidate_lead_drop_n')
    release_detector = PotentiometerReleaseDetector(
        unloaded_force_n=unloaded_force,
        **{k: release[k] for k in release_keys if k in release})
    return contact_detector, release_detector


class VelocityContactDetector:
    """Legacy speed-threshold meaning, with continuous onset/release dwell."""

    def __init__(self, *, onset_speed_m_s, release_speed_m_s,
                 onset_dwell_s, release_dwell_s, max_sample_gap_s):
        # Reuse unloaded-baseline/onset continuity rather than accepting the
        # residual approach velocity immediately after rearming.
        self.onset = PotentiometerContactDetector(
            onset_speed_m_s, onset_dwell_s, max_sample_gap_s)
        self.release_speed = release_speed_m_s
        self.release_dwell = release_dwell_s
        self.max_gap = max_sample_gap_s
        self.reset()

    def reset(self):
        self.onset.reset()
        self.active = False
        self.release_since = None
        self.last_time = None

    def update(self, speed, timestamp, enabled):
        if not enabled:
            self.reset()
            return False, False
        if self.last_time is not None and (
                timestamp <= self.last_time or timestamp - self.last_time > self.max_gap):
            self.release_since = None
        self.last_time = timestamp
        if not self.active:
            started = self.onset.update(speed, timestamp).started
            self.active = started
            return started, False
        if speed < self.release_speed:
            if self.release_since is None:
                self.release_since = timestamp
            if timestamp - self.release_since >= self.release_dwell:
                self.active = False
                return False, True
        else:
            self.release_since = None
        return False, False


class LevelCoastCycle:
    """Only release followed by low XY speed can start the grace timer."""

    def __init__(self, position, stop_speed_m_s, grace_s):
        self.hold_position = np.asarray(position, dtype=float).copy()
        self.stop_speed = stop_speed_m_s
        self.grace_s = grace_s
        self.phase = 'prepare'
        self.grace_started = None

    @property
    def level(self):
        return self.phase in ('contact', 'coast')

    def update(self, position, velocity, now, *, armed=False, started=False, released=False):
        previous = self.phase
        if self.phase == 'prepare' and armed:
            self.phase = 'ready'
        elif self.phase == 'ready' and started:
            self.phase = 'contact'
        elif self.phase == 'contact' and released:
            self.phase = 'coast'
        # A release already below threshold can capture hold in this sample.
        if self.phase == 'coast' and np.linalg.norm(velocity[:2]) < self.stop_speed:
            self.hold_position[:2] = np.asarray(position)[:2]
            self.grace_started = now
            self.phase = 'grace'
        elif self.phase == 'grace' and now - self.grace_started >= self.grace_s:
            self.phase = 'prepare'
        return previous != self.phase


def _fresh_state(owner, state, now, safety):
    """Use the same state, motor freshness and measured boundary limits."""
    from Interaction.interactions import StaleLocalizationError
    max_age = safety.get('max_state_age_s', safety['max_frame_age_s'])
    if state is None or not -0.5 <= now - state['time'] <= max_age:
        raise StaleLocalizationError('Level coast requires fresh synchronized onboard state')
    enforce_skew = safety['enforce_state_group_skew']
    skews = [state['position_skew_s'], state['angular_rate_skew_s']]
    if state['yaw_control_skew_s'] is not None:
        skews.append(state['yaw_control_skew_s'])
    if enforce_skew and any(
            v is None or not math.isfinite(v) or v > safety.get('max_state_group_skew_s', .03)
            for v in skews):
        raise StaleLocalizationError('Level coast onboard state groups are unsynchronized')
    owner.check_interaction_boundary(state['position'])
    motor = state['motor_state']
    pwm = [motor.get(f'motor.m{i}') for i in range(1, 5)]
    voltage = motor.get('pm.vbat')
    age = now - motor.get('time', -math.inf)
    skew = state['motor_skew_s']
    available = (OnboardMomentumWrenchPipeline.motor_data_available(pwm)
                 and isinstance(voltage, (int, float))
                 and math.isfinite(voltage) and voltage > 0
                 and -.5 <= age <= safety['max_motor_age_s']
                 and (not enforce_skew or (skew is not None and math.isfinite(skew)
                      and 0 <= skew <= safety.get('max_motor_state_skew_s', .03))))
    if safety['require_motor_data'] and not available:
        raise RuntimeError('Fresh, state-synchronized motor PWM and battery data are required')
    return (pwm, voltage) if available else (None, None)


def run_level_coast(owner, config):
    """Run within the existing controller lifecycle and safety-aware sleep."""
    # Import lazily: interactions owns the reusable handoff/arming helpers and
    # dispatches here, while this module's state machine stays independently testable.
    from Interaction.interactions import (
        InitialContactArmingGate, StaleLocalizationError,
        reset_pid_integrators_without_ack,
    )
    options = validate_level_coast(
        config, sensor_available=getattr(owner, 'force_sensor', None) is not None)
    calibrated_config = apply_detection_calibration(
        deepcopy(config['wrench_interaction']), owner.drone_id,
        config.get('wrench_calibration_file', DEFAULT_CALIBRATION_PATH))
    pipeline = OnboardMomentumWrenchPipeline(calibrated_config)
    safety = pipeline.config['safety']
    gate = InitialContactArmingGate(**pipeline.config['initial_contact_arming'])
    target = owner.mission['drones'][owner.drone_id]['target']
    nominal = np.asarray(target[:3], dtype=float)
    yaw = float(target[3] if len(target) > 3 else calibrated_config.get('nominal_yaw_deg', 0))
    owner.check_interaction_boundary(nominal)
    cycle = LevelCoastCycle(nominal, options['stop_speed_m_s'], options['grace_s'])
    velocity_detector = VelocityContactDetector(**options['velocity'])
    pot_contact, pot_release = (_potentiometer_detectors(config)
        if options['detector'] == 'potentiometer' else (None, None))
    rate = _number(owner.ctrl_rate, 'ctrl_rate', positive=True)
    dt = 1 / rate
    last_state_time = None
    last_sensor_time = None
    start = None
    startup_deadline = time.monotonic() + safety['startup_timeout_s']
    owner.log_manager.add_log_entry('configs', {
        'behavior': 'level_coast', **options,
        'wrench_interaction_profile': config.get('wrench_interaction_profile'),
        'wrench_interaction': deepcopy(pipeline.config),
        'wrench_detection_calibration': calibrated_config.get('wrench_detection_calibration'),
        'velocity_source': 'crazyflie_state_estimate',
    }, name='Level Coast Config')

    def send():
        if cycle.level:
            owner.lo_commander.send_zdistance_setpoint(0., 0., 0., float(nominal[2]))
        else:
            owner.lo_commander.send_position_setpoint(*cycle.hold_position, yaw)

    try:
        while start is None or time.monotonic() - start < options['duration_s']:
            owner._safe_sleep(0.)  # Sticky battery and operator abort checks.
            now = time.time()
            state = owner._get_synchronized_onboard_wrench_state()
            try:
                pwm, voltage = _fresh_state(owner, state, now, safety)
            except StaleLocalizationError:
                if last_state_time is not None or time.monotonic() >= startup_deadline:
                    raise
                send()
                owner._safe_sleep(dt)
                continue
            if last_state_time is not None and state['time'] <= last_state_time:
                if state['time'] < last_state_time:
                    raise StaleLocalizationError('Level coast state clock moved backwards')
                send()  # A duplicate can refresh the command, never a detector/timer.
                owner._safe_sleep(dt)
                continue
            last_state_time = state['time']
            pipeline.detector.translation.enabled = (
                options['detector'] == 'model' and cycle.phase in ('ready', 'contact'))
            output = pipeline.update(
                position=state['position'], velocity=state['velocity'],
                attitude_rpy=state['attitude_rpy'], angular_velocity=state['angular_velocity'],
                motor_pwm=pwm, battery_voltage=voltage, timestamp=state['time'],
                yaw_control_command=state['yaw_control_command'])
            if not output.calibrated:
                send()
                owner._safe_sleep(dt)
                continue
            if start is None:
                start = time.monotonic()
                owner._log_event('Level Coast Started', options)
            if cycle.phase == 'prepare':
                gate.update(state['velocity'], state['time'])
            started = released = False
            sensor = {}
            enabled = cycle.phase in ('ready', 'contact')
            if options['detector'] == 'model' and output.contacts is not None:
                started = output.contacts.translation.started
                released = output.contacts.translation.ended
            elif options['detector'] == 'vel':
                started, released = velocity_detector.update(
                    float(np.linalg.norm(state['velocity'][:2])), state['time'], enabled)
            elif options['detector'] == 'potentiometer':
                sensor = owner._force_sensor_log_fields(output.estimate, now)
                if not sensor.get('force_sensor_fresh'):
                    raise RuntimeError('Level coast requires fresh potentiometer samples')
                sensor_time = sensor.get('force_sensor_sample_monotonic_time')
                if sensor_time is None:
                    sensor_time = sensor['force_sensor_sample_time']
                if last_sensor_time is not None and sensor_time < last_sensor_time:
                    raise RuntimeError('Level coast potentiometer clock moved backwards')
                if sensor_time != last_sensor_time:
                    last_sensor_time = sensor_time
                    force = float(sensor['force_sensor_compression_force_N'])
                    if cycle.phase == 'contact':
                        released = pot_release.update(force, sensor_time).released
                    elif cycle.phase == 'ready':
                        decision = pot_contact.update(force, sensor_time)
                        started = decision.started
                        if started:
                            pot_release.arm(force, sensor_time, peak_force_n=decision.peak_force_n)
            previous = cycle.phase
            changed = cycle.update(
                state['position'], state['velocity'], time.monotonic(),
                armed=gate.armed, started=started, released=released)
            if changed:
                if cycle.phase == 'contact':
                    owner._set_contact_pid_attitude_authority(True)
                    owner._translation_high_level_active = False
                elif cycle.phase == 'grace':
                    owner._set_contact_pid_attitude_authority(False)
                    reset_pid_integrators_without_ack(owner.cf, ('posCtlPid.resetI', 'velCtlPid.resetI'))
                elif cycle.phase == 'prepare':
                    gate.reset(after_interaction=True)
                    pipeline.detector.translation.reset(state['time'])
                    velocity_detector.reset()
                    if pot_contact is not None:
                        pot_contact.reset()
                        pot_release.disarm()
                owner._log_event('Level Coast Phase Changed', {
                    'previous': previous, 'phase': cycle.phase, 'released': released,
                    'xy_speed_m_s': float(np.linalg.norm(state['velocity'][:2])),
                    'elapsed_s': time.monotonic() - start,
                    'hold_position_m': cycle.hold_position.tolist(),
                })
            send()
            owner.log_manager.add_log_entry('wrench_observer', {
                'time': now, 'behavior': 'level_coast', 'phase': cycle.phase,
                'state_time': state['time'], 'position_m': state['position'].tolist(),
                'velocity_m_s': state['velocity'].tolist(),
                'external_force_N': output.estimate.external_force.tolist(),
                'contact_started': started, 'release_confirmed': released,
                'command_mode': 'level_zdistance' if cycle.level else 'position_hold',
                **sensor,
            })
            owner._safe_sleep(dt)
        owner._log_event('Level Coast Duration Completed', {
            'phase': cycle.phase, 'duration_s': options['duration_s'],
            'elapsed_s': time.monotonic() - start,
        })
    finally:
        owner._set_contact_pid_attitude_authority(False)
