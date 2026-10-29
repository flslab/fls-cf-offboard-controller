"""Vicon-referenced, velocity-compensated capture and hover for level_coast.

The physical hover anchor is distinct from the position packet used to invert
the FC's P-only cascade. Never freeze a transient velocity-bias compensation.
This is a bounded offboard controller, not a guarantee about physical motion.
"""
import math
import numpy as np

from Interaction.position_follow import PositionVelocityFollower

DEFAULTS = {
    'enabled': True,
    'transition_s': .25,
    'max_jerk_m_s3': 2.0,
    'settle_speed_m_s': .02,
    'settle_tilt_deg': .5,
    'settle_rate_deg_s': 5.0,
    'settle_acceleration_m_s2': .35,
    'settle_dwell_s': .10,
    'acceleration_filter_s': .08,
    'timeout_s': 3.0,
    'max_sample_gap_s': .10,
    'max_vicon_skew_s': .035,
    'max_command_age_s': .05,
}


def validate_position_handoff(options):
    if not isinstance(options, dict):
        raise ValueError('level_coast.position_handoff must be a mapping')
    unknown = set(options) - set(DEFAULTS)
    if unknown:
        raise ValueError('unknown position_handoff options: ' + ', '.join(sorted(unknown)))
    result = dict(DEFAULTS, **options)
    if type(result['enabled']) is not bool:
        raise ValueError('position_handoff.enabled must be boolean')
    for key in set(DEFAULTS) - {'enabled'}:
        value = result[key]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'position_handoff.{key} must be finite and positive')
        result[key] = float(value)
    if result['timeout_s'] <= result['transition_s'] + result['settle_dwell_s']:
        raise ValueError('position_handoff.timeout_s must exceed transition plus dwell')
    if result['max_vicon_skew_s'] > .035 or result['max_command_age_s'] > .05:
        raise ValueError('position_handoff cannot widen Vicon/command freshness ceilings')
    return result


def handoff_enabled(config):
    options = config.get('level_coast') or {}
    values = validate_position_handoff(options.get('position_handoff', {}))
    return config.get('behavior') == 'level_coast' and options.get('coasting_enabled', True) and values['enabled']


def add_handoff_logs(selected, mission):
    config = (mission.get('Interaction') or {}).get('config') or {}
    if not handoff_enabled(config):
        return selected
    # Two existing FC values, one 8-byte block at 50 Hz. Required only to seed
    # capture from the outgoing commanded attitude, not an assumed level pose.
    return {**selected, 'POSITION_HANDOFF': {'log_period_ms': 20,
        **{key: {'type': 'float', 'data': []} for key in ('controller.roll', 'controller.pitch')}}}


def incoming_attitude_command(log_manager, state_time, now, max_age):
    packet, skew = log_manager.get_nearest_group_log_data('POSITION_HANDOFF', state_time)
    stamp = log_manager.get_latest_group_log_time('POSITION_HANDOFF')
    if (not packet or skew is None or not math.isfinite(skew) or abs(skew) > max_age
            or stamp is None or not math.isfinite(stamp) or not -.02 <= now-stamp <= max_age):
        raise RuntimeError('Position handoff requires fresh controller roll/pitch commands')
    rp = np.array([packet['controller.roll'], packet['controller.pitch']], dtype=float)
    if not np.isfinite(rp).all():
        raise ValueError('Position handoff command is not finite')
    return rp


class PositionHandoff:
    def __init__(self, parameters, position_options, options):
        pid = PositionVelocityFollower(parameters, position_options)
        self.kp, self.kv, self.velocity_limit = pid.kp, pid.kv, pid.velocity_limit
        self.position_options = pid.options
        self.options = validate_position_handoff(options)
        self.reset()

    def reset(self):
        self.started = self.last_time = self.last_motion_time = None
        self.anchor = self.last_motion_velocity = self.initial_acceleration = None
        self.acceleration = np.zeros(2)
        self.measured_acceleration = np.zeros(2)
        self.settled_since = None
        self.restore_settled_since = None
        self.ready_to_restore = False
        self.complete = False

    @property
    def active(self):
        return self.started is not None

    def update(self, *, position, velocity, attitude_rpy, angular_velocity,
               motion_position, motion_velocity, timestamp, motion_time, height,
               incoming_rp_deg=None, allow_hold=True):
        vectors = [np.asarray(x, dtype=float) for x in
                   (position, velocity, attitude_rpy, angular_velocity, motion_position, motion_velocity)]
        if any(x.shape != (3,) or not np.isfinite(x).all() for x in vectors):
            raise ValueError('Position handoff requires finite XYZ state and Vicon motion')
        p, v, rpy, rates, mp, mv = vectors
        if not all(math.isfinite(x) for x in (timestamp, motion_time, height)):
            raise ValueError('Position handoff requires finite timestamps and height')
        if abs(timestamp-motion_time) > self.options['max_vicon_skew_s']:
            raise ValueError('Position handoff Vicon reference is stale')
        c, s = math.cos(rpy[2]), math.sin(rpy[2])
        rotation = np.array([[c, s], [-s, c]])  # World to body-yaw.
        decay = 9.81 * np.radians(self.kv)
        max_accel = self.position_options['max_brake_acceleration_m_s2']
        if not self.active:
            rp = np.asarray(incoming_rp_deg, dtype=float)
            if rp.shape != (2,) or not np.isfinite(rp).all():
                raise ValueError('Position handoff requires the outgoing roll/pitch command')
            self.initial_acceleration = rotation.T @ (-9.81 * np.radians(rp[::-1]))
            if np.linalg.norm(self.initial_acceleration) > max_accel + 1e-9:
                raise ValueError('Outgoing attitude command exceeds position handoff envelope')
            self.acceleration = self.initial_acceleration.copy()
            self.started = timestamp
            self.last_motion_time = motion_time
            self.last_motion_velocity = mv[:2].copy()
            dt = 0.
        else:
            dt = timestamp-self.last_time
            if not 0 < dt <= self.options['max_sample_gap_s']:
                raise ValueError('Position handoff requires fresh continuous state samples')
        elapsed = timestamp-self.started
        if not self.complete and elapsed > self.options['timeout_s']:
            raise RuntimeError('Position handoff did not settle before timeout')
        motion_dt = motion_time-self.last_motion_time
        if motion_dt < 0 or motion_dt > self.options['max_sample_gap_s']:
            raise ValueError('Position handoff Vicon clock is discontinuous')
        if motion_dt > 0:
            acceleration = (mv[:2]-self.last_motion_velocity)/motion_dt
            alpha = 1-math.exp(-motion_dt/self.options['acceleration_filter_s'])
            self.measured_acceleration += alpha*(acceleration-self.measured_acceleration)
            self.last_motion_time = motion_time
            self.last_motion_velocity = mv[:2].copy()
        position_error = np.zeros(2) if self.anchor is None else self.anchor[:2]-mp[:2]
        error_body = self.kp*(rotation @ position_error) - rotation @ mv[:2]
        desired = rotation.T @ (decay*error_body)
        norm = np.linalg.norm(desired)
        if norm > max_accel:
            desired *= max_accel/norm
        fraction = min(1., elapsed/self.options['transition_s'])
        blend = fraction**3*(10-15*fraction+6*fraction*fraction)
        desired = (1-blend)*self.initial_acceleration + blend*desired
        change = desired-self.acceleration
        limit = self.options['max_jerk_m_s3']*dt
        norm = np.linalg.norm(change)
        if norm > limit and norm > 0:
            change *= limit/norm
        self.acceleration += change
        physically_stable = (fraction == 1.
            and np.linalg.norm(mv[:2]) <= self.options['settle_speed_m_s']
            and np.linalg.norm(np.degrees(rpy[:2])) <= self.options['settle_tilt_deg']
            and np.linalg.norm(np.degrees(rates[:2])) <= self.options['settle_rate_deg_s']
            and np.linalg.norm(self.measured_acceleration) <= self.options['settle_acceleration_m_s2']
            and np.linalg.norm(self.acceleration) <= self.options['settle_acceleration_m_s2'])
        # Estimator restoration must not be a prerequisite for measuring that
        # estimator-3 capture has settled. Completion still needs a separate
        # fresh dwell after the restoration acknowledgement.
        if not physically_stable:
            self.restore_settled_since = None
            self.ready_to_restore = False
        elif motion_dt > 0:
            if self.restore_settled_since is None:
                self.restore_settled_since = motion_time
            self.ready_to_restore = motion_time-self.restore_settled_since >= self.options['settle_dwell_s']
        stable = physically_stable and allow_hold
        if not self.complete:
            if stable and motion_dt > 0:
                if self.settled_since is None:
                    self.settled_since = motion_time
                if motion_time-self.settled_since >= self.options['settle_dwell_s']:
                    self.anchor = np.array([*mp[:2], height])
                    self.complete = True
            elif not stable:
                self.settled_since = None
        # Inverting BOTH P loops makes the effective velocity error independent
        # of the FC's velocity bias. A jump in that bias changes the temporary
        # packet offset, not the desired acceleration or the physical anchor.
        correction_body = (rotation @ self.acceleration)/decay
        desired_body_velocity = rotation @ v[:2] + correction_body
        if np.any(np.abs(desired_body_velocity) > self.velocity_limit):
            raise ValueError('Position handoff exceeds confirmed FC velocity limits')
        offset = rotation.T @ (desired_body_velocity/self.kp)
        if np.linalg.norm(offset) > self.position_options['max_offset_m']:
            raise ValueError('Position handoff exceeds maximum position offset')
        target = np.array([*(p[:2]+offset), height])
        self.last_time = timestamp
        status = {'position_handoff_complete': self.complete,
            'position_handoff_ready_to_restore': self.ready_to_restore,
            'position_handoff_elapsed_s': elapsed, 'position_handoff_allow_hold': bool(allow_hold),
            'position_handoff_anchor_m': None if self.anchor is None else self.anchor.tolist(),
            'position_handoff_acceleration_m_s2': self.acceleration.tolist(),
            'position_handoff_measured_acceleration_m_s2': self.measured_acceleration.tolist(),
            'position_handoff_velocity_bias_m_s': (v[:2]-mv[:2]).tolist(),
            'position_handoff_command_m': target.tolist(),
            'position_handoff_nominal_pitch_roll_deg': (-self.kv*correction_body).tolist(),
            'position_handoff_motion_position_m': mp.tolist(),
            'position_handoff_motion_velocity_m_s': mv.tolist(),
            'position_handoff_motion_time': motion_time}
        return target, status
