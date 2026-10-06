"""Timed, repeated level-attitude interaction, independent of firmware braking."""

from copy import deepcopy
import math
import time

import numpy as np

from Interaction.onboard_wrench_interaction_pipeline import OnboardMomentumWrenchPipeline
from Interaction.offboard_yaw_damping import (
    DEFAULT_YAW_RATE_DEADBAND_DEG_S, validate_yaw_deadband,
)
from Interaction.potentiometer_force_sensor import (
    PotentiometerContactDetector, PotentiometerReleaseDetector,
)
from Interaction.position_follow import PositionVelocityFollower, validate_position_follow
from Interaction.wrench_model_calibration import (
    DEFAULT_CALIBRATION_PATH, apply_detection_calibration,
)

# Historical name: delay before the selected contact/coast command policy.
# Keep the fixed position hold during this interval; 0.0 switches immediately.
DETECTION_TO_ORI_DELAY_S = 0.0


def _number(value, name, *, positive=False):
    result = float(value)
    if not math.isfinite(result) or (result <= 0 if positive else result < 0):
        raise ValueError(f'{name} must be finite and {"positive" if positive else "non-negative"}')
    return result


def resolve_command_modes(options):
    """Share phase selection and validation with grounded PID preparation."""
    contact = options.get('command_mode', 'orientation')
    modes = {'command_mode': contact,
             'coast_command_mode': options.get('coast_command_mode', contact)}
    for name, value in modes.items():
        if value not in ('orientation', 'position'):
            raise ValueError(f'level_coast.{name} must be orientation or position')
    return modes


def validate_level_coast(config, *, sensor_available):
    """Validate this opt-in behavior before any flight commands are sent."""
    options = dict(config.get('level_coast') or {})
    options.update(resolve_command_modes(options))
    options['position_control'] = validate_position_follow(options.get('position_control', {}))
    if 'detector' in options:
        raise ValueError('level_coast.detector was removed; use config.detection_method')
    detector = config.get('detection_method', 'potentiometer')
    if detector not in ('vel', 'model', 'potentiometer'):
        raise ValueError('level_coast detection_method must be vel, model or potentiometer')
    wrench = config.get('wrench_interaction') or {}
    if wrench.get('state_source') != 'onboard':
        raise ValueError('level_coast requires onboard state logging')
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
    options['detection_to_ori_delay_s'] = _number(
        DETECTION_TO_ORI_DELAY_S, 'DETECTION_TO_ORI_DELAY_S')
    options['grace_s'] = _number(config.get('grace_time', .5), 'grace_time')
    options['grace_start'] = options.get('grace_start', 'speed_threshold')
    if options['grace_start'] not in ('speed_threshold', 'release'):
        raise ValueError('level_coast.grace_start must be speed_threshold or release')
    options['follow_yaw'] = options.get('follow_yaw', False)
    if type(options['follow_yaw']) is not bool:
        raise ValueError('level_coast.follow_yaw must be boolean')
    options['yaw_rate_damping'] = options.get('yaw_rate_damping', False)
    if type(options['yaw_rate_damping']) is not bool:
        raise ValueError('level_coast.yaw_rate_damping must be boolean')
    if options['yaw_rate_damping'] and options['follow_yaw']:
        raise ValueError('yaw_rate_damping and follow_yaw cannot both be enabled')
    options['yaw_rate_deadband_deg_s'] = validate_yaw_deadband(
        options.get('yaw_rate_deadband_deg_s', DEFAULT_YAW_RATE_DEADBAND_DEG_S))
    options['stop_speed_m_s'] = _number(
        options.get('stop_speed_m_s', .03), 'stop_speed_m_s', positive=True)
    options['detection_method'] = detector
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
    """Selectable grace timing; release-based rearming can interrupt coast."""

    def __init__(self, position, stop_speed_m_s, grace_s, grace_start='speed_threshold',
                 detection_to_ori_delay_s=0., *, coast_command_mode='orientation'):
        self.hold_position = np.asarray(position, dtype=float).copy()
        self.stop_speed = stop_speed_m_s
        self.grace_s = grace_s
        self.phase = 'prepare'
        self.grace_started = None
        self.grace_start = grace_start
        self.detection_to_ori_delay_s = _number(detection_to_ori_delay_s, 'detection_to_ori_delay_s')
        self.detected_at = None
        self.delay_pending = False
        self.coast_command_mode = coast_command_mode
        self.interaction_direction_xy = None
        self.interaction_direction_source = None

    def _lock_interaction_direction(self, direction, source, velocity):
        # Reset on every accepted onset, including a coast preemption. Never
        # redefine the axis from the decaying/lateral velocity during coast.
        self.interaction_direction_xy = None
        self.interaction_direction_source = None
        for candidate, candidate_source in ((direction, source), (velocity, 'onset_velocity')):
            if candidate is None:
                continue
            candidate = np.asarray(candidate, dtype=float)
            if candidate.shape not in ((2,), (3,)) or not np.all(np.isfinite(candidate)):
                continue
            norm = float(np.linalg.norm(candidate[:2]))
            if norm > 1e-9:
                self.interaction_direction_xy = candidate[:2].copy() / norm
                self.interaction_direction_source = candidate_source
                break

    def stop_status(self, velocity):
        xy_speed = float(np.linalg.norm(velocity[:2]))
        projected = (None if self.interaction_direction_xy is None
                     else float(np.asarray(velocity)[:2] @ self.interaction_direction_xy))
        directional = self.coast_command_mode == 'orientation'
        metric = ('interaction_projection' if directional and projected is not None
                  else 'xy_norm_no_direction' if directional else 'xy_norm')
        return {
            'interaction_direction_xy': (None if self.interaction_direction_xy is None
                                         else self.interaction_direction_xy.tolist()),
            'interaction_direction_source': self.interaction_direction_source,
            'interaction_velocity_m_s': projected,
            'stop_speed_metric': metric,
            'stop_speed_value_m_s': projected if metric == 'interaction_projection' else xy_speed,
        }

    @property
    def level(self):
        return self.phase in ('contact', 'coast') and not self.delay_pending

    def grace_expired(self, now):
        return self.grace_started is not None and now - self.grace_started >= self.grace_s

    def detection_enabled(self, now):
        return self.phase in ('ready', 'contact') or (
            self.grace_start == 'release' and self.phase in ('coast', 'grace')
            and not self.delay_pending and self.grace_expired(now))

    def update(self, position, velocity, now, *, armed=False, started=False, released=False,
               interaction_direction=None, interaction_direction_source=None,
               stop_velocity=None):
        previous = self.phase
        handoff_velocity = velocity if stop_velocity is None else stop_velocity
        delay_finished = (self.delay_pending
            and now - self.detected_at >= self.detection_to_ori_delay_s)
        if delay_finished:
            self.delay_pending = False
        if self.phase == 'prepare' and armed:
            self.phase = 'ready'
        elif self.phase != 'contact' and self.detection_enabled(now) and started:
            if self.phase == 'coast' and self.detection_to_ori_delay_s > 0:
                # Coast has no current position target. Hold here for the new
                # delay instead of pulling back toward the previous interaction.
                self.hold_position[:2] = np.asarray(position)[:2]
            self.phase = 'contact'
            self.grace_started = None
            self.detected_at = now
            self.delay_pending = self.detection_to_ori_delay_s > 0
            self._lock_interaction_direction(interaction_direction, interaction_direction_source, velocity)
        elif self.phase == 'contact' and released:
            self.phase = 'coast'
            if self.grace_start == 'release':
                self.grace_started = now
        # A release already below threshold can capture hold in this sample.
        if (self.phase == 'coast' and not self.delay_pending and not delay_finished
                and self.stop_status(handoff_velocity)['stop_speed_value_m_s'] < self.stop_speed):
            self.hold_position[:2] = np.asarray(position)[:2]
            if self.grace_start == 'speed_threshold':
                self.grace_started = now
            self.phase = ('ready' if self.grace_start == 'release'
                          and self.grace_expired(now) else 'grace')
        elif self.phase == 'grace' and self.grace_expired(now):
            self.phase = 'ready' if self.grace_start == 'release' else 'prepare'
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
        potentiometer_release_direction, reset_pid_integrators_without_ack,
    )
    options = validate_level_coast(
        config, sensor_available=getattr(owner, 'force_sensor', None) is not None)
    yaw_guard = getattr(owner.cf, '_offboard_yaw_damping_guard', None)
    if options['yaw_rate_damping'] and not getattr(yaw_guard, 'prepared', False):
        raise RuntimeError('offboard yaw damping was not prepared before takeoff')
    yaw_ready = not options['yaw_rate_damping']
    position_pid = getattr(owner.cf, '_offboard_position_pid', None)
    position_follower = None
    if 'position' in (options['command_mode'], options['coast_command_mode']):
        if not getattr(position_pid, 'prepared', False):
            raise RuntimeError('position-follow PID was not prepared before takeoff')
        position_follower = PositionVelocityFollower(position_pid.parameters, options['position_control'])
    calibrated_config = apply_detection_calibration(
        deepcopy(config['wrench_interaction']), owner.drone_id,
        config.get('wrench_calibration_file', DEFAULT_CALIBRATION_PATH))
    pipeline = OnboardMomentumWrenchPipeline(calibrated_config)
    safety = pipeline.config['safety']
    gate = InitialContactArmingGate(**pipeline.config['initial_contact_arming'])
    target = owner.mission['drones'][owner.drone_id]['target']
    nominal = np.asarray(target[:3], dtype=float)
    # Position packets take an absolute angle in degrees. Level/coast packets
    # instead take a yaw rate, which always remains zero.
    yaw = None if options['follow_yaw'] else 0.
    owner.check_interaction_boundary(nominal)
    cycle = LevelCoastCycle(nominal, options['stop_speed_m_s'], options['grace_s'],
                            options['grace_start'], options['detection_to_ori_delay_s'],
                            coast_command_mode=options['coast_command_mode'])
    position_command = nominal.copy()
    command_mode = 'position_hold'
    velocity_detector = VelocityContactDetector(**options['velocity'])
    pot_contact, pot_release = (_potentiometer_detectors(config)
        if options['detection_method'] == 'potentiometer' else (None, None))
    rate = _number(owner.ctrl_rate, 'ctrl_rate', positive=True)
    dt = 1 / rate
    last_state_time = None
    last_sensor_time = None
    start = None
    detection_was_enabled = False
    startup_deadline = time.monotonic() + safety['startup_timeout_s']
    owner.log_manager.add_log_entry('configs', {
        'behavior': 'level_coast', **options,
        'wrench_interaction_profile': config.get('wrench_interaction_profile'),
        'wrench_interaction': deepcopy(pipeline.config),
        'wrench_detection_calibration': calibrated_config.get('wrench_detection_calibration'),
        'velocity_source': 'crazyflie_state_estimate',
        'handoff_velocity_source': 'vicon_position_kf',
        'position_motion_velocity_source': 'vicon_position_kf',
        'position_pid_velocity_source': 'crazyflie_state_estimate',
    }, name='Level Coast Config')

    def send():
        if command_mode == 'position_follow':
            owner.lo_commander.send_position_setpoint(*position_command, yaw)
        elif command_mode == 'level_zdistance':
            owner.lo_commander.send_zdistance_setpoint(0., 0., 0., float(nominal[2]))
        elif yaw is not None:
            owner.lo_commander.send_position_setpoint(*cycle.hold_position, yaw)

    def reset_detectors(timestamp):
        pipeline.detector.translation.reset(timestamp)
        velocity_detector.reset()
        if pot_contact is not None:
            pot_contact.reset()
            pot_release.disarm()

    try:
        print('[interaction] startup', flush=True)
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
            if options['follow_yaw']:
                yaw = math.degrees(float(state['attitude_rpy'][2]))
                if not math.isfinite(yaw):
                    raise StaleLocalizationError('Level coast requires a finite onboard yaw')
            sample_now = time.monotonic()
            enabled = cycle.detection_enabled(sample_now)
            if enabled and not detection_was_enabled:
                # Clear the projected-release latch and discard all evidence
                # collected before grace expired. Do not count the blind gap.
                if options['grace_start'] == 'release' and cycle.grace_started is not None:
                    reset_detectors(state['time'])
                owner._log_event('Level Coast Detection Rearmed', {
                    'phase': cycle.phase, 'grace_start': options['grace_start'],
                })
                if cycle.phase == 'coast':
                    print('[interaction] coast: detection ready', flush=True)
            detection_was_enabled = enabled
            pipeline.detector.translation.enabled = (
                options['detection_method'] == 'model' and enabled)
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
                print('[interaction] prepare', flush=True)
            if cycle.phase == 'prepare':
                gate.update(state['velocity'], state['time'])
                if gate.armed and not yaw_ready:
                    yaw_ready = yaw_guard.request_enable(state['angular_velocity'][2])
                    if yaw_ready:
                        owner._log_event('Level Coast Yaw Damping Enabled', {})
                        print('[interaction] yaw damping enabled', flush=True)
            yaw_status = (yaw_guard.update(state['angular_velocity'][2])
                          if options['yaw_rate_damping'] and yaw_ready else {})
            started = released = False
            # Optional sensing remains diagnostic for model/velocity detection.
            sensor = (owner._force_sensor_log_fields(output.estimate, now)
                      if getattr(owner, 'force_sensor', None) is not None else {})
            if options['detection_method'] == 'model' and output.contacts is not None:
                started = output.contacts.translation.started
                released = output.contacts.translation.ended
            elif options['detection_method'] == 'vel':
                started, released = velocity_detector.update(
                    float(np.linalg.norm(state['velocity'][:2])), state['time'], enabled)
            elif options['detection_method'] == 'potentiometer':
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
                    elif enabled:
                        decision = pot_contact.update(force, sensor_time)
                        started = decision.started
                        if started:
                            pot_release.arm(force, sensor_time, peak_force_n=decision.peak_force_n)
            previous = cycle.phase
            was_level = cycle.level
            previous_command_mode = command_mode
            direction = direction_source = None
            if started:
                if options['detection_method'] == 'potentiometer':
                    direction, direction_source = potentiometer_release_direction(
                        sensor.get('force_sensor_external_force_N', np.zeros(3)), state['velocity'])
                elif options['detection_method'] == 'model':
                    decision = output.contacts.translation
                    direction = decision.release_direction
                    direction_source = 'model_' + (decision.release_direction_source or 'force')
                    if direction is None:
                        direction = output.estimate.external_force
                # Velocity detection uses the cycle's onset-velocity fallback.
            # Start the command delay after evaluating the detector, not before
            # potentially expensive model/sensor work at the start of the loop.
            # Keep onboard velocity for detection, direction locking and FC PID
            # compensation. Use checked Vicon KF for motion policy and handoff.
            stop_velocity = None
            stop_reference = {'stop_velocity_source': 'crazyflie_state_estimate'}
            if cycle.phase in ('contact', 'coast') or (
                    started and options['command_mode'] == 'position'):
                stop_velocity, vicon_time, vicon_skew = (
                    owner._vicon_velocity_reference_for_onboard_state(state))
                stop_reference = {
                    'stop_velocity_source': 'vicon_position_kf',
                    'stop_velocity_m_s': stop_velocity.tolist(),
                    'stop_velocity_time': vicon_time,
                    'stop_velocity_state_skew_s': vicon_skew,
                }
            sample_now = time.monotonic()
            changed = cycle.update(
                state['position'], state['velocity'], sample_now,
                armed=gate.armed and yaw_ready, started=started, released=released,
                interaction_direction=direction, interaction_direction_source=direction_source,
                stop_velocity=stop_velocity)
            stop_status = cycle.stop_status(
                state['velocity'] if stop_velocity is None else stop_velocity)
            stop_status.update(stop_reference)
            stop_status['onboard_stop_speed_value_m_s'] = (
                cycle.stop_status(state['velocity'])['stop_speed_value_m_s'])
            # Authority follows the transmitted command, not contact detection:
            # contact/release bookkeeping continues while the pos delay runs.
            selected_mode = options['coast_command_mode' if cycle.phase == 'coast' else 'command_mode']
            command_mode = (('position_follow' if selected_mode == 'position' else 'level_zdistance')
                            if cycle.level else 'position_hold')
            position_status = {}
            if position_follower is not None:
                # A release can reach hold in the same sample: capture according
                # to the selected coast policy, even if no coast packet was sent.
                capture_position = (was_level and not cycle.level
                    and cycle.phase in ('grace', 'ready')
                    and options['coast_command_mode'] == 'position')
                if command_mode == 'position_follow' or capture_position:
                    if stop_velocity is None:
                        raise StaleLocalizationError('Position following requires checked Vicon velocity')
                    position_command, position_status = position_follower.target(
                        state['position'], state['velocity'], state['attitude_rpy'][2],
                        state['time'], cycle.phase if cycle.level else 'coast', nominal[2],
                        capture=capture_position, motion_velocity=stop_velocity)
                    position_status['position_motion_velocity_source'] = 'vicon_position_kf'
                    position_status['position_pid_velocity_source'] = 'crazyflie_state_estimate'
                    owner.check_interaction_boundary(position_command)
                    if not cycle.level:
                        # Freeze the Vicon stop projection with the onboard
                        # velocity-PID compensation offset applied.
                        cycle.hold_position[:] = position_command
                if command_mode != 'position_follow':
                    position_follower.reset()
            owner._set_contact_pid_attitude_authority(command_mode == 'level_zdistance')
            if cycle.level:
                owner._translation_high_level_active = False
            if (previous_command_mode == 'level_zdistance' and command_mode != 'level_zdistance'
                    and position_follower is None):
                # Mixed/position runs already have zero XY I gains for the whole
                # flight. Preserve the legacy reset for orientation-only runs.
                reset_pid_integrators_without_ack(owner.cf, ('posCtlPid.resetI', 'velCtlPid.resetI'))
            if changed:
                if cycle.phase == 'prepare':
                    gate.reset(after_interaction=True)
                    reset_detectors(state['time'])
                owner._log_event('Level Coast Phase Changed', {
                    'previous': previous, 'phase': cycle.phase, 'released': released,
                    'xy_speed_m_s': float(np.linalg.norm(state['velocity'][:2])),
                    **stop_status,
                    'elapsed_s': time.monotonic() - start,
                    'hold_position_m': cycle.hold_position.tolist(),
                    'grace_start': options['grace_start'],
                    'command_mode': command_mode,
                    'coast_preempted': previous == 'coast' and cycle.phase == 'contact',
                })
                print(f'[interaction] {previous} -> {cycle.phase}', flush=True)
            if started and cycle.delay_pending:
                print(f'[interaction] pos delay: {cycle.detection_to_ori_delay_s:g}s', flush=True)
            if previous_command_mode != command_mode:
                owner._log_event('Level Coast Command Mode Changed', {
                    'previous_command_mode': previous_command_mode,
                    'command_mode': command_mode,
                    'phase': cycle.phase,
                    'detection_to_ori_delay_s': cycle.detection_to_ori_delay_s,
                    'since_detection_s': (None if cycle.detected_at is None
                                          else sample_now - cycle.detected_at),
                })
                labels = {'position_hold': 'hold', 'position_follow': 'pos', 'level_zdistance': 'ori'}
                print(f'[interaction] cmd: {labels[previous_command_mode]} -> {labels[command_mode]}', flush=True)
            if released:
                detection_was_enabled = False  # Also rearm when grace is zero.
            send()
            owner.log_manager.add_log_entry('wrench_observer', {
                'time': now, 'behavior': 'level_coast', 'phase': cycle.phase,
                'state_time': state['time'], 'position_m': state['position'].tolist(),
                'velocity_m_s': state['velocity'].tolist(),
                **stop_status,
                'external_force_N': output.estimate.external_force.tolist(),
                'contact_started': started, 'release_confirmed': released,
                'detection_enabled': enabled,
                'detection_to_ori_delay_s': cycle.detection_to_ori_delay_s,
                'ori_delay_pending': cycle.delay_pending,
                'grace_start': options['grace_start'],
                'grace_elapsed_s': (None if cycle.grace_started is None
                                    else sample_now - cycle.grace_started),
                'command_mode': command_mode,
                'yaw_target_deg': None if command_mode == 'level_zdistance' else yaw,
                'yaw_rate_target_deg_s': 0. if command_mode == 'level_zdistance' else None,
                **position_status,
                'yaw_rate_damping': options['yaw_rate_damping'],
                'yaw_rate_damping_active': options['yaw_rate_damping'] and yaw_ready,
                **yaw_status,
                'effective_yaw_rate_target_deg_s': (
                    0. if options['yaw_rate_damping'] and yaw_ready else None),
                **sensor,
            })
            owner._safe_sleep(dt)
        owner._log_event('Level Coast Duration Completed', {
            'phase': cycle.phase, 'duration_s': options['duration_s'],
            'elapsed_s': time.monotonic() - start,
        })
        print('[interaction] done', flush=True)
    finally:
        if options['yaw_rate_damping']:
            yaw_guard.finish()
        owner._set_contact_pid_attitude_authority(False)
