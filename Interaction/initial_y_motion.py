"""One bounded world-+Y position reference before the first accepted contact."""
import math
import numpy as np


DEFAULTS = {
    'enabled': False,
    'speed_m_s': .03,
    'ramp_s': 1.0,
    'max_distance_m': .60,
    'boundary_margin_m': .20,
}


def validate_initial_motion(options):
    if not isinstance(options, dict):
        raise ValueError('level_coast.initial_motion must be a mapping')
    unknown = set(options) - set(DEFAULTS)
    if unknown:
        raise ValueError('unknown initial_motion options: ' + ', '.join(sorted(unknown)))
    result = dict(DEFAULTS, **options)
    if type(result['enabled']) is not bool:
        raise ValueError('initial_motion.enabled must be boolean')
    for key in set(DEFAULTS) - {'enabled'}:
        value = result[key]
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value <= 0):
            raise ValueError(f'initial_motion.{key} must be finite and positive')
        result[key] = float(value)
    if result['speed_m_s'] > .10:
        raise ValueError('initial_motion.speed_m_s must not exceed 0.10 m/s')
    return result


class InitialYMotion:
    """An absolute, sample-clock trajectory; it never restarts after contact.

    Quintic velocity ramps provide continuous acceleration at start/stop.
    Bounds constrain the reference; ordinary measured-position boundary and
    state-freshness guards remain responsible for the physical flight.
    """
    def __init__(self, origin, options, *, y_max=None, max_sample_gap_s=.10):
        self.options = validate_initial_motion(options)
        self.origin = np.asarray(origin, dtype=float).copy()
        if self.origin.shape != (3,) or not np.isfinite(self.origin).all():
            raise ValueError('initial_motion requires a finite XYZ origin')
        self.distance = self.options['max_distance_m']
        self.limit_reason = 'max_distance'
        if y_max is not None and self.options['enabled']:
            if not math.isfinite(y_max):
                raise ValueError('initial_motion requires a finite Y boundary')
            room = max(0., y_max-self.options['boundary_margin_m']-self.origin[1])
            if room < self.distance:
                self.distance = room
                self.limit_reason = 'boundary_margin'
        self.ramp = self.options['ramp_s']
        self.speed = min(self.options['speed_m_s'], self.distance/self.ramp)
        self.cruise = max(0., self.distance/self.speed-self.ramp) if self.speed else 0.
        self.duration = 2*self.ramp+self.cruise if self.speed else 0.
        self.max_gap = max_sample_gap_s
        self.started = self.last_time = None
        self.state = 'waiting' if self.options['enabled'] else 'disabled'
        self.finished_reason = None
        self.displacement = self.reference_speed = 0.

    @staticmethod
    def _blend(u):
        return u**3*(10.-15.*u+6.*u*u)

    @staticmethod
    def _integral(u):
        return 2.5*u**4-3.*u**5+u**6

    def status(self):
        if not self.options['enabled']:
            return {}
        return {
            'initial_motion_state': self.state,
            'initial_motion_frame': 'world',
            'initial_motion_direction_xy': [0., 1.],
            'initial_motion_reference_speed_m_s': self.reference_speed,
            'initial_motion_displacement_m': self.displacement,
            'initial_motion_limit_m': self.distance,
            'initial_motion_finished_reason': self.finished_reason,
        }

    def update(self, phase, timestamp):
        if self.state == 'disabled':
            return None, {}
        if phase == 'contact' and self.state in ('waiting', 'moving'):
            self.state = 'finished'
            self.finished_reason = 'first_contact'
            self.reference_speed = 0.
            return None, self.status()
        if self.state == 'finished' or phase != 'ready':
            return None, self.status()
        if not math.isfinite(timestamp):
            raise ValueError('initial_motion requires a finite state timestamp')
        if self.started is None:
            self.started = timestamp
            self.state = 'moving'
        elif not 0 <= timestamp-self.last_time <= self.max_gap:
            raise ValueError('initial_motion requires fresh continuous state samples')
        self.last_time = timestamp
        elapsed = timestamp-self.started
        if elapsed >= self.duration-1e-12:
            self.displacement = self.distance
            self.reference_speed = 0.
            self.state = 'finished'
            self.finished_reason = self.limit_reason
        elif elapsed <= self.ramp:
            u = elapsed/self.ramp
            self.displacement = self.speed*self.ramp*self._integral(u)
            self.reference_speed = self.speed*self._blend(u)
        elif elapsed <= self.ramp+self.cruise:
            self.displacement = self.speed*(self.ramp/2+elapsed-self.ramp)
            self.reference_speed = self.speed
        else:
            u = (elapsed-self.ramp-self.cruise)/self.ramp
            self.displacement = self.speed*(self.ramp/2+self.cruise+self.ramp*(u-self._integral(u)))
            self.reference_speed = self.speed*(1-self._blend(u))
        target = self.origin.copy()
        target[1] += min(self.distance, max(0., self.displacement))
        return target, self.status()
