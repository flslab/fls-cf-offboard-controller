"""Causal model-contact comparison with no command authority.

The baseline is a nuisance-residual estimate, NOT a calibrated contact force.
Only controller-idle, stationary windows may update it. Potentiometer values
are deliberately not inputs: they can label an offline evaluation, not make
the model detector's decision for it.
"""

from collections import deque
from dataclasses import asdict

import numpy as np

from Interaction.wrench_contact_detector import ContactChannelDetector
from Interaction.onboard_wrench_interaction_pipeline import (
    FiniteWindowMomentumForceEstimator,
)


class StationaryForceBaseline:
    """Bounded XY baseline; learn slowly at rest and freeze on onset evidence."""

    def __init__(self, *, window_s=0.30, minimum_samples=15,
                 max_gap_s=0.05, max_speed_m_s=0.03,
                 max_rate_rad_s=0.15, max_bias_n=0.12,
                 max_spread_n=0.035, update_gate_n=0.04,
                 time_constant_s=2.0, max_slew_n_s=0.01):
        values = (window_s, max_gap_s, max_speed_m_s, max_rate_rad_s,
                  max_bias_n, max_spread_n, update_gate_n,
                  time_constant_s, max_slew_n_s)
        if not all(np.isfinite(v) and v > 0 for v in values):
            raise ValueError('baseline settings must be finite and positive')
        if type(minimum_samples) is not int or minimum_samples < 2:
            raise ValueError('minimum_samples must be an integer >= 2')
        self.window_s = float(window_s)
        self.minimum_samples = minimum_samples
        self.max_gap_s = float(max_gap_s)
        self.max_speed = float(max_speed_m_s)
        self.max_rate = float(max_rate_rad_s)
        self.max_bias = float(max_bias_n)
        self.max_spread = float(max_spread_n)
        self.update_gate = float(update_gate_n)
        self.tau = float(time_constant_s)
        self.max_slew = float(max_slew_n_s)
        self.configuration = dict(
            window_s=window_s, minimum_samples=minimum_samples,
            max_gap_s=max_gap_s, max_speed_m_s=max_speed_m_s,
            max_rate_rad_s=max_rate_rad_s, max_bias_n=max_bias_n,
            max_spread_n=max_spread_n, update_gate_n=update_gate_n,
            time_constant_s=time_constant_s, max_slew_n_s=max_slew_n_s)
        self.reset()

    def reset(self):
        self.bias = np.zeros(3)
        self.ready = False
        self.last_time = None
        self.samples = deque()
        self.reason = 'warming_up'

    def observe(self, force, velocity, angular_velocity, timestamp, *,
                valid, allow_learning, contact_evidence):
        force = np.asarray(force, dtype=float)
        velocity = np.asarray(velocity, dtype=float)
        angular_velocity = np.asarray(angular_velocity, dtype=float)
        finite = (all(v.shape == (3,) and np.all(np.isfinite(v))
                      for v in (force, velocity, angular_velocity))
                  and np.isfinite(timestamp))
        dt = None if self.last_time is None else timestamp - self.last_time
        if not finite or not valid or (dt is not None and (
                dt <= 0 or dt > self.max_gap_s)):
            self.reset()
            self.reason = 'invalid_or_discontinuous'
            return
        self.last_time = float(timestamp)
        if (not allow_learning or contact_evidence
                or np.linalg.norm(velocity[:2]) > self.max_speed
                or np.linalg.norm(angular_velocity) > self.max_rate):
            self.samples.clear()
            self.reason = 'frozen_motion_or_contact'
            return
        residual = force[:2] - self.bias[:2]
        # A large residual must never be learned away as an idle offset.
        if (np.linalg.norm(force[:2]) > self.max_bias
                or (self.ready and np.linalg.norm(residual) > self.update_gate)):
            self.samples.clear()
            self.reason = 'frozen_force_change'
            return
        self.samples.append((timestamp, force[:2].copy()))
        while (len(self.samples) > 2
               and timestamp - self.samples[1][0] >= self.window_s):
            self.samples.popleft()
        if (len(self.samples) < self.minimum_samples
                or timestamp - self.samples[0][0] < self.window_s - 1e-9):
            self.reason = 'warming_up' if not self.ready else 'collecting_idle'
            return
        samples = np.array([s[1] for s in self.samples])
        center = np.median(samples, axis=0)
        # Peak spread, rather than only variance, stops a new pulse from being
        # folded into the bootstrap window just because it is a minority.
        if np.max(np.linalg.norm(samples - center, axis=1)) > self.max_spread:
            self.reason = 'frozen_unstable_window'
            return
        if not self.ready:
            self.bias[:2] = center
            self.ready = True
        elif dt is not None:
            delta = (1.0 - np.exp(-dt / self.tau)) * (center - self.bias[:2])
            norm = np.linalg.norm(delta)
            if norm > self.max_slew * dt:
                delta *= self.max_slew * dt / norm
            self.bias[:2] += delta
        self.reason = 'learning_idle'


class ModelContactDiagnostics:
    """Two shadow detectors on the same samples: raw and baseline-corrected.

    Neither result feeds the production detector, admittance or commander.
    Uses only past baseline values for the current decision (no look-ahead).
    Invalid samples/gaps are marked unavailable, never scored as no-contact.
    """

    def __init__(self, translation_config, baseline_config=None):
        options = dict(translation_config)
        options['enabled'] = True
        self.raw = ContactChannelDetector(**options)
        self.corrected = ContactChannelDetector(**options)
        self.baseline = StationaryForceBaseline(**(baseline_config or {}))
        self.last_time = None

    def reset(self):
        self.raw.reset()
        self.corrected.reset()
        self.baseline.reset()
        self.last_time = None

    def observe_safely(self, **sample):
        """A diagnostic failure is logged as unavailable, not a flight fault."""
        try:
            return self.observe(**sample)
        except Exception as exc:
            self.reset()
            return {'schema_version': 1, 'valid': False,
                    'reason': 'diagnostic_error', 'error': str(exc)[:200],
                    'command_authority': False}

    def observe(self, *, force, covariance, velocity, angular_velocity,
                timestamp, valid, armed, idle):
        force = np.asarray(force, dtype=float)
        covariance = np.asarray(covariance, dtype=float)
        velocity = np.asarray(velocity, dtype=float)
        angular_velocity = np.asarray(angular_velocity, dtype=float)
        valid = bool(valid and np.isfinite(timestamp)
                     and force.shape == (3,) and np.all(np.isfinite(force))
                     and covariance.shape == (3, 3)
                     and np.all(np.isfinite(covariance))
                     and velocity.shape == (3,) and np.all(np.isfinite(velocity))
                     and angular_velocity.shape == (3,)
                     and np.all(np.isfinite(angular_velocity)))
        gap = (self.last_time is not None and (
            timestamp <= self.last_time
            or timestamp - self.last_time > self.baseline.max_gap_s))
        if not valid or gap:
            self.reset()
            return {'schema_version': 1, 'valid': False,
                    'reason': 'invalid_or_discontinuous',
                    'command_authority': False}
        self.last_time = timestamp
        bias_used = self.baseline.bias.copy()
        ready_used = self.baseline.ready
        corrected_force = force - bias_used
        raw_decision = corrected_decision = None
        if armed:
            raw_decision = self.raw.update(force, covariance, timestamp, velocity)
            corrected_decision = self.corrected.update(
                corrected_force, covariance, timestamp, velocity)
        else:
            self.raw.reset(timestamp)
            self.corrected.reset(timestamp)
        axes = list(self.corrected.onset_axes)
        potential_contact = (self.corrected.active or self.corrected.evidence > 0
                             or np.linalg.norm(corrected_force[axes]
                                               / self.corrected.thresholds[axes]) >= 1)
        self.baseline.observe(
            force, velocity, angular_velocity, timestamp, valid=True,
            allow_learning=idle, contact_evidence=potential_contact)
        return {
            'schema_version': 1, 'valid': True, 'armed': bool(armed),
            'command_authority': False,
            'baseline_ready': ready_used, 'baseline_used_N': bias_used.tolist(),
            'baseline_update': self.baseline.reason,
            'corrected_force_N': corrected_force.tolist(),
            'raw': None if raw_decision is None else asdict(raw_decision),
            'corrected': (None if corrected_decision is None
                          else asdict(corrected_decision)),
        }


class ShortWindowContactDiagnostics:
    """Short force window plus the original long-window shadow reference.

    Force thresholds, CUSUM accumulation and release dwell stay unchanged.
    A short continuous-onset guard prevents large isolated residual pulses
    from bypassing confirmation. The production wrench estimate, selected
    detector and control commands remain untouched.
    """

    def __init__(self, translation_config, *, mass, impulse_config,
                 baseline_config=None, window_s=0.03, minimum_window_s=0.02):
        if (isinstance(window_s, bool) or isinstance(minimum_window_s, bool)
                or not np.isfinite(window_s) or not np.isfinite(minimum_window_s)
                or not 0 < minimum_window_s <= window_s):
            raise ValueError('short force window must be finite and positive; '
                             'minimum_window_s must not exceed window_s')
        self.mass = mass
        self.reference_window_s = float(impulse_config['window_s'])
        if window_s > self.reference_window_s:
            raise ValueError('short force window cannot exceed reference window')
        self.impulse_config = dict(impulse_config)
        self.impulse_config.update(window_s=float(window_s),
                                   minimum_window_s=float(minimum_window_s))
        short_options = dict(translation_config)
        # One isolated velocity step contributes to the moving force residual
        # for less than one nominal window. Require significance for a whole
        # window, independent of amplitude, before declaring contact.
        short_options['minimum_onset_duration_s'] = float(window_s)
        self.short = ModelContactDiagnostics(short_options, baseline_config)
        self.long = ModelContactDiagnostics(translation_config, baseline_config)
        self.configuration = {
            'window_s': float(window_s),
            'minimum_window_s': float(minimum_window_s),
            'reference_window_s': self.reference_window_s,
            'minimum_onset_duration_s': float(window_s),
            'baseline': self.short.baseline.configuration,
        }
        self.reset()

    def reset(self):
        self.estimator = FiniteWindowMomentumForceEstimator(
            self.mass, **self.impulse_config)
        self.short.reset()
        self.long.reset()

    def observe_safely(self, **sample):
        try:
            return self.observe(**sample)
        except Exception as exc:
            self.reset()
            return {'schema_version': 2, 'valid': False,
                    'reason': 'diagnostic_error', 'error': str(exc)[:200],
                    'command_authority': False}

    def observe(self, *, force, covariance, velocity, angular_velocity,
                expected_acceleration, timestamp, state_valid, long_valid,
                armed, idle, force_bias=(0., 0., 0.)):
        # State validity is separate from the long estimator's warmup: the
        # short path must not wait for the long force window to become ready.
        vectors = [np.asarray(value, dtype=float) for value in
                   (velocity, angular_velocity, expected_acceleration, force_bias)]
        if (not state_valid or not np.isfinite(timestamp)
                or not all(value.shape == (3,) and np.all(np.isfinite(value))
                           for value in vectors)):
            self.reset()
            return {'schema_version': 2, 'valid': False,
                    'reason': 'invalid_state', 'command_authority': False}
        short_force = self.estimator.update(
            velocity, expected_acceleration, timestamp)
        shared = dict(covariance=covariance, velocity=velocity,
                      angular_velocity=angular_velocity, timestamp=timestamp,
                      armed=armed, idle=idle)
        short = self.short.observe(
            force=short_force.external_force - np.asarray(force_bias, dtype=float),
            valid=short_force.ready and not short_force.rejected, **shared)
        long = self.long.observe(force=force, valid=long_valid, **shared)
        return {
            **short,
            'schema_version': 2,
            'selected_window': 'short',
            'window_s': self.impulse_config['window_s'],
            'minimum_window_s': self.impulse_config['minimum_window_s'],
            'actual_window_s': short_force.window_s,
            'force_estimate_N': (short_force.external_force
                                 - np.asarray(force_bias, dtype=float)).tolist(),
            'long_window': {**long, 'window_s': self.reference_window_s},
        }
