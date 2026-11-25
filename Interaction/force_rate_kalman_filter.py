"""KID horizontal force/rate filter (paper Eqs. 5--8), in world coordinates."""

from dataclasses import dataclass
import math

import numpy as np


COVARIANCE_EIGENVALUE_FLOOR = 1e-8  # N^2
DEFAULTS = {
    'tau_s': .010,
    'process_noise_scale': .1,
    'minimum_valid_span_s': .020,  # startup/recovery only, never a moving average
    'measurement_covariance': [[.020**2, 0.], [0., .020**2]],
}


def validated_covariance(value):
    covariance = np.asarray(value, dtype=float)
    if (covariance.shape != (2, 2) or not np.isfinite(covariance).all()
            or not np.allclose(covariance, covariance.T, rtol=0., atol=1e-12)
            or np.linalg.eigvalsh(covariance).min() <= 0):
        raise ValueError('force-rate measurement_covariance must be symmetric positive definite 2x2')
    return covariance.copy()


def measurement_covariance_from_hover(force_xy, timestamps):
    """Half the covariance of adjacent observations, 5% shrinkage, eigen floor.

    Use only adjacent fresh pairs (0 < dt <= 30 ms). These are instantaneous
    observations, before filtering; subtracting a fixed bias cancels in a pair.
    """
    force = np.asarray(force_xy, dtype=float)
    times = np.asarray(timestamps, dtype=float)
    if (force.ndim != 2 or force.shape[1] != 2 or times.shape != (len(force),)
            or not np.isfinite(force).all() or not np.isfinite(times).all()):
        raise ValueError('hover covariance requires finite XY observations and matching times')
    intervals = np.diff(times)
    differences = np.diff(force, axis=0)[(intervals > 0.) & (intervals <= .030 + 1e-12)]
    if len(differences) < 2:
        raise ValueError('hover covariance needs at least two adjacent fresh force differences')
    covariance = .5 * np.cov(differences, rowvar=False, ddof=1)
    covariance[0, 1] *= .95
    covariance[1, 0] *= .95
    values, vectors = np.linalg.eigh(covariance)
    return (vectors * np.maximum(values, COVARIANCE_EIGENVALUE_FLOOR)) @ vectors.T


@dataclass(frozen=True)
class ForceRateEstimate:
    force_xy: np.ndarray
    rate_xy: np.ndarray
    covariance: np.ndarray
    ready: bool
    rejected: bool
    valid_span_s: float


class ForceRateKalmanFilter:
    """State [Fx, Fy, tau*dFx/dt, tau*dFy/dt], both blocks in newtons."""

    def __init__(self, tau_s=.010, process_noise_scale=.1,
                 minimum_valid_span_s=.020, measurement_covariance=None, max_dt_s=.05):
        for name, value in (('tau_s', tau_s), ('process_noise_scale', process_noise_scale),
                            ('max_dt_s', max_dt_s), ('minimum_valid_span_s', minimum_valid_span_s)):
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value < 0
                    or (name != 'minimum_valid_span_s' and value == 0)):
                raise ValueError(f'force-rate {name} must be finite and positive (span may be zero)')
        self.tau_s = float(tau_s)
        self.q = float(process_noise_scale)
        self.minimum_valid_span_s = float(minimum_valid_span_s)
        self.max_dt_s = float(max_dt_s)
        self.measurement_covariance = validated_covariance(
            DEFAULTS['measurement_covariance'] if measurement_covariance is None
            else measurement_covariance)
        self.reset()

    def reset(self):
        self.state = np.zeros(4)
        self.covariance = np.zeros((4, 4))
        self.last_timestamp = self.started_at = None

    def set_measurement_covariance(self, covariance):
        self.measurement_covariance = validated_covariance(covariance)
        self.reset()

    def update(self, observation_xy, timestamp):
        try:
            observation = np.asarray(observation_xy, dtype=float)
            timestamp = float(timestamp)
        except (TypeError, ValueError) as error:
            self.reset()
            raise ValueError('force-rate observation and timestamp must be finite') from error
        if observation.shape != (2,) or not np.isfinite(observation).all() or not math.isfinite(timestamp):
            self.reset()
            raise ValueError('force-rate observation and timestamp must be finite')
        dt = None if self.last_timestamp is None else timestamp - self.last_timestamp
        if dt is not None and dt <= 0:
            self.reset()
            return ForceRateEstimate(np.zeros(2), np.zeros(2), np.zeros((2, 2)), False, True, 0.)
        discontinuity = dt is not None and dt > self.max_dt_s + 1e-12
        if discontinuity:
            self.reset()
        r = self.measurement_covariance
        if self.last_timestamp is None:
            self.state[:2] = observation
            self.covariance[:2, :2] = r
            self.covariance[2:, 2:] = r
            self.started_at = timestamp
        else:
            h = dt / self.tau_s
            a = np.eye(4)
            a[:2, 2:] = h * np.eye(2)
            q = self.q * np.block([[h**3 / 3. * r, h**2 / 2. * r],
                                   [h**2 / 2. * r, h * r]])
            prediction = a @ self.state
            p = a @ self.covariance @ a.T + q
            gain = np.linalg.solve(p[:2, :2] + r, p[:2, :]).T
            self.state = prediction + gain @ (observation - prediction[:2])
            # Joseph form maintains covariance symmetry/PSD under rounding.
            residual = np.eye(4)
            residual[:, :2] -= gain
            self.covariance = residual @ p @ residual.T + gain @ r @ gain.T
            self.covariance = .5 * (self.covariance + self.covariance.T)
        self.last_timestamp = timestamp
        span = timestamp - self.started_at
        return ForceRateEstimate(
            self.state[:2].copy(), self.state[2:].copy() / self.tau_s,
            self.covariance[:2, :2].copy(),
            span + 1e-12 >= self.minimum_valid_span_s and not discontinuity,
            discontinuity, span)
