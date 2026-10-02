"""Velocity-following position packets for the opt-in level_coast behavior.

The FC position PID produces a BODY-YAW-frame velocity target, not a position
increment per packet. Invert its confirmed P gains instead of assuming that
``p + v * loop_dt`` requests velocity v. XY I/D/feedforward are disabled for
this experiment while grounded; the ordinary Z and attitude/rate loops remain.
"""

import json
import math
import os
from pathlib import Path

import numpy as np

from Interaction.firmware_parameter_confirmation import confirm_firmware_mode_parameters


POSITION_KP = ('posCtlPid.xKp', 'posCtlPid.yKp')
VELOCITY_KP = ('velCtlPid.vxKp', 'velCtlPid.vyKp')
VELOCITY_LIMIT = ('posCtlPid.xVelMax', 'posCtlPid.yVelMax')
SUPPRESSED_GAINS = tuple(
    f'posCtlPid.{axis}{term}' for axis in ('x', 'y') for term in ('Ki', 'Kd', 'Kff')
) + tuple(
    f'velCtlPid.v{axis}{term}' for axis in ('x', 'y') for term in ('Ki', 'Kd', 'KFF')
)
DEFAULTS = {
    'contact_velocity_retention': 1.0,
    'coast_velocity_retention': 0.0,
    'coast_transition_s': 0.5,
    'max_brake_acceleration_m_s2': 0.8,
    'max_offset_m': 0.6,
}


def validate_position_follow(options):
    if not isinstance(options, dict):
        raise ValueError('level_coast.position_control must be a mapping')
    unknown = set(options) - set(DEFAULTS)
    if unknown:
        raise ValueError('unknown position_control options: ' + ', '.join(sorted(unknown)))
    values = dict(DEFAULTS, **options)
    for key, value in values.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'position_control.{key} must be finite')
        values[key] = float(value)
    if not 0 < values['contact_velocity_retention'] <= 1:
        raise ValueError('contact_velocity_retention must be in (0, 1]')
    if not 0 <= values['coast_velocity_retention'] < values['contact_velocity_retention']:
        raise ValueError('coast_velocity_retention must be below contact_velocity_retention')
    for key in ('coast_transition_s', 'max_brake_acceleration_m_s2', 'max_offset_m'):
        if values[key] <= 0:
            raise ValueError(f'position_control.{key} must be positive')
    return values


def _finite_parameters(values, names):
    if set(values) != set(names):
        raise ValueError('unexpected position-follow PID parameters')
    result = {key: float(value) for key, value in values.items()}
    if any(not math.isfinite(v) or not 0 <= v <= 10000 for v in result.values()):
        raise ValueError('invalid position-follow PID parameter')
    return result


class PositionFollowPidContext:
    """Grounded-only temporary gains, with recovery after interrupted runs.

    Never store changed values to FC persistent storage. Keep the backup until
    all original gains are freshly confirmed after landing or on next startup.
    No gain writes or parameter round trips occur in the flight-command loop.
    """

    def __init__(self, cf, backup_path):
        self.cf = cf
        self.path = Path(backup_path)
        self.prepared = False
        self.parameters = None

    def restore(self):
        self.prepared = False
        if not self.path.exists():
            return
        data = json.loads(self.path.read_text())
        if data.get('schema') != 1:
            raise ValueError('unsupported position-follow PID backup')
        original = _finite_parameters(data['gains'], SUPPRESSED_GAINS)
        for name, value in original.items():
            self.cf.param.set_value(name, str(value))
        confirm_firmware_mode_parameters(self.cf.param, expected=original)
        self.path.unlink()

    def prepare(self):
        names = POSITION_KP + VELOCITY_KP + VELOCITY_LIMIT + SUPPRESSED_GAINS
        toc = getattr(getattr(self.cf.param, 'toc', None), 'toc', {})
        for name in names:
            group, item = name.split('.')
            if item not in toc.get(group, {}):
                raise RuntimeError(f'position-follow requires existing parameter {name}')
        original = _finite_parameters({k: self.cf.param.get_value(k) for k in names}, names)
        if any(original[k] <= 0 for k in POSITION_KP + VELOCITY_KP + VELOCITY_LIMIT):
            raise ValueError('position-follow requires positive XY P gains and speed limits')
        confirm_firmware_mode_parameters(self.cf.param, expected={
            **original, 'stabilizer.controller': 1,
        })
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open('x') as stream:
            json.dump({'schema': 1, 'gains': {k: original[k] for k in SUPPRESSED_GAINS}}, stream)
            stream.flush()
            os.fsync(stream.fileno())
        for name in SUPPRESSED_GAINS:
            self.cf.param.set_value(name, '0')
        self.parameters = {**original, **dict.fromkeys(SUPPRESSED_GAINS, 0.)}
        confirm_firmware_mode_parameters(self.cf.param, expected={
            **self.parameters, 'stabilizer.controller': 1,
        })
        self.prepared = True

    def verify(self):
        if not self.prepared:
            raise RuntimeError('position-follow PID context is not prepared')
        confirm_firmware_mode_parameters(self.cf.param, expected={
            **self.parameters, 'stabilizer.controller': 1,
        })


class PositionVelocityFollower:
    """Re-anchor every fresh command to measured XY; never integrate a stale path.

    Contact follows measured velocity. Coast smoothly lowers the retained
    fraction; the P-only velocity loop then damps motion on BOTH horizontal
    axes. Limit the nominal braking tilt via its equivalent acceleration.
    These are setpoint bounds, not a guarantee about physical acceleration.
    """

    def __init__(self, parameters, options):
        self.options = validate_position_follow(options)
        self.kp = np.array([parameters[k] for k in POSITION_KP], dtype=float)
        self.kv = np.array([parameters[k] for k in VELOCITY_KP], dtype=float)
        self.velocity_limit = np.array([parameters[k] for k in VELOCITY_LIMIT], dtype=float)
        if any(np.any(~np.isfinite(x)) or np.any(x <= 0) for x in (self.kp, self.kv, self.velocity_limit)):
            raise ValueError('invalid confirmed position-follow gains/limits')
        if any(float(parameters[k]) != 0 for k in SUPPRESSED_GAINS):
            raise ValueError('position-follow requires confirmed zero XY I/D/feedforward')
        self.phase = None
        self.coast_started = None
        self.last_time = None

    def reset(self):
        self.phase = None
        self.coast_started = None
        self.last_time = None

    def target(self, position, velocity, yaw_rad, timestamp, phase, height, *, capture=False):
        p, v = np.asarray(position, dtype=float), np.asarray(velocity, dtype=float)
        if (p.shape != (3,) or v.shape != (3,) or not np.all(np.isfinite(p))
                or not np.all(np.isfinite(v))
                or not all(math.isfinite(x) for x in (yaw_rad, timestamp, height))):
            raise ValueError('position-follow requires finite position, velocity and yaw')
        if phase not in ('contact', 'coast'):
            raise ValueError('position-follow target requires contact or coast')
        if self.last_time is not None and timestamp <= self.last_time:
            raise ValueError('position-follow only advances on fresh state')
        self.last_time = timestamp
        if phase != self.phase:
            self.coast_started = timestamp if phase == 'coast' else None
        self.phase = phase
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        world_to_body = np.array([[c, s], [-s, c]])
        body_velocity = world_to_body @ v[:2]
        retention = self.options['contact_velocity_retention']
        if phase == 'coast':
            fraction = min(1., (timestamp - self.coast_started) / self.options['coast_transition_s'])
            blend = fraction * fraction * (3. - 2. * fraction)
            retention += blend * (self.options['coast_velocity_retention'] - retention)
        delta_v = (retention - 1.) * body_velocity
        nominal_tilt = -self.kv * delta_v
        tilt_limit = math.degrees(math.atan(self.options['max_brake_acceleration_m_s2'] / 9.81))
        magnitude = float(np.linalg.norm(nominal_tilt))
        if magnitude > tilt_limit:
            delta_v *= tilt_limit / magnitude
        desired_body_velocity = body_velocity + delta_v
        if capture:
            # Low-speed hold: a P-only velocity loop has the small-angle decay
            # rate g * radians(Kv). Its free-stop projection v / rate avoids
            # putting a fixed target immediately behind a still-moving drone.
            # This is a nominal capture approximation; attitude lag is not zero.
            stop_projection = body_velocity / (9.81 * np.radians(self.kv))
            capture_velocity = np.minimum(np.abs(body_velocity), np.abs(self.kp * stop_projection))
            desired_body_velocity = np.sign(body_velocity) * np.maximum(
                np.abs(desired_body_velocity), capture_velocity)
            delta_v = desired_body_velocity - body_velocity
        if np.any(np.abs(desired_body_velocity) > self.velocity_limit):
            raise ValueError('position-follow target exceeds confirmed FC velocity limit')
        offset = world_to_body.T @ (desired_body_velocity / self.kp)
        if np.linalg.norm(offset) > self.options['max_offset_m']:
            raise ValueError('position-follow target exceeds maximum lookahead distance')
        target = np.array([*(p[:2] + offset), height])
        return target, {
            'position_command_m': target.tolist(),
            'position_offset_m': offset.tolist(),
            'position_velocity_target_m_s': (world_to_body.T @ desired_body_velocity).tolist(),
            'position_velocity_retention_requested': retention,
            'position_nominal_pitch_roll_deg': (-self.kv * delta_v).tolist(),
            'position_capture_projected': bool(capture),
        }
