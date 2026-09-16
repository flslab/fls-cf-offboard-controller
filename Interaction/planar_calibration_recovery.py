"""Bounded return-to-center reference for the opt-in planar-only calibration.

The old recovery sent the nominal center as a step POSITION setpoint as soon
as the attitude pulse ended.  A large position error then let the firmware's
position PID demand substantially more tilt than the calibration pulse.
This helper limits the target's movement and requires a settled return before
another trial can begin; it does not change the flight safety envelope.
"""

from __future__ import annotations

import math

import numpy as np


class BoundedPlanarCalibrationRecovery:
    def __init__(self, nominal_position, config):
        self.nominal = np.asarray(nominal_position, dtype=float).copy()
        if self.nominal.shape != (3,) or not np.all(np.isfinite(self.nominal)):
            raise ValueError('bounded planar recovery needs a finite XYZ center')
        self.target_speed_m_s = float(config.get('recovery_target_speed_m_s', 0.35))
        self.max_target_lead_m = float(config.get('recovery_max_target_lead_m', 0.12))
        self.settle_speed_m_s = float(config.get('recovery_settle_speed_m_s', 0.08))
        self.settle_tilt_deg = float(config.get('recovery_settle_tilt_deg', 5.0))
        self.stable_dwell_s = float(config.get('recovery_stable_dwell_s', 0.20))
        self.max_tilt_deg = float(config.get('recovery_max_tilt_deg', 25.0))
        self.position_tolerance_m = float(config.get('recovery_position_tolerance_m', 0.08))
        self.duration_s = float(config.get('recovery_s', 0.0))
        values = np.asarray([
            self.target_speed_m_s, self.max_target_lead_m,
            self.settle_speed_m_s, self.settle_tilt_deg,
            self.stable_dwell_s, self.max_tilt_deg,
            self.position_tolerance_m, self.duration_s,
        ])
        if (not np.all(np.isfinite(values)) or np.any(values <= 0.0)
                or self.duration_s < 4.0
                or self.max_tilt_deg > 25.0
                or self.max_target_lead_m > 0.15
                or self.target_speed_m_s > 0.40):
            raise ValueError(
                'bounded planar recovery requires recovery_s >= 4.0, '
                'target speed <= 0.40 m/s, lead <= 0.15 m, and '
                'tilt limit <= 25 deg'
            )
        self.segment_id = None
        self.target = None
        self.phase = None
        self.last_time_s = None
        self.stable_since_s = None
        self.complete_since_s = None
        self.complete = False

    def require_complete(self):
        if self.segment_id is not None and not self.complete:
            raise RuntimeError(
                'bounded planar recovery did not settle at the nominal '
                'position before the next trial'
            )

    def update(self, segment_id, position, velocity, tilt_deg, now_s):
        position = np.asarray(position, dtype=float)
        velocity = np.asarray(velocity, dtype=float)
        tilt_deg = float(tilt_deg)
        now_s = float(now_s)
        if (position.shape != (3,) or velocity.shape[0] < 2
                or not np.all(np.isfinite(position))
                or not np.all(np.isfinite(velocity[:2]))
                or not math.isfinite(tilt_deg) or not math.isfinite(now_s)):
            raise ValueError('bounded planar recovery received invalid state')
        if tilt_deg > self.max_tilt_deg:
            raise RuntimeError(
                f'bounded planar recovery exceeded tilt limit '
                f'({tilt_deg:.2f} > {self.max_tilt_deg:.2f} deg)'
            )
        if self.segment_id != segment_id:
            self.require_complete()
            self.segment_id = segment_id
            self.target = position.copy()
            self.target[2] = self.nominal[2]
            self.phase = 'settle'
            self.last_time_s = now_s
            self.stable_since_s = None
            self.complete_since_s = None
            self.complete = False
        if now_s < self.last_time_s:
            raise ValueError('bounded planar recovery clock moved backward')
        dt = min(now_s - self.last_time_s, 0.05)
        self.last_time_s = now_s
        xy_speed = float(np.linalg.norm(velocity[:2]))
        stable = (xy_speed <= self.settle_speed_m_s
                  and tilt_deg <= self.settle_tilt_deg)
        if self.phase == 'complete' and (
                not stable
                or float(np.linalg.norm(position[:2] - self.nominal[:2]))
                > self.position_tolerance_m):
            self.phase = 'return'
            self.complete = False
            self.complete_since_s = None
        if self.phase == 'settle':
            # A still-moving vehicle must never accumulate a large frozen
            # POSITION error while the first recovery hold brakes it.
            lead = self.target[:2] - position[:2]
            lead_distance = float(np.linalg.norm(lead))
            if lead_distance > self.max_target_lead_m:
                self.target[:2] = (
                    position[:2]
                    + lead * (self.max_target_lead_m / lead_distance)
                )
            self.stable_since_s = (
                now_s if stable and self.stable_since_s is None
                else self.stable_since_s if stable else None
            )
            if (self.stable_since_s is not None
                    and now_s - self.stable_since_s >= self.stable_dwell_s):
                self.phase = 'return'
                self.complete_since_s = None
        if self.phase == 'return':
            remaining = self.nominal[:2] - self.target[:2]
            distance = float(np.linalg.norm(remaining))
            if distance > 1e-9:
                step = min(distance, self.target_speed_m_s * dt)
                self.target[:2] += remaining * (step / distance)
            lead = self.target[:2] - position[:2]
            lead_distance = float(np.linalg.norm(lead))
            if lead_distance > self.max_target_lead_m:
                self.target[:2] = (
                    position[:2]
                    + lead * (self.max_target_lead_m / lead_distance)
                )
            at_center = (
                float(np.linalg.norm(self.target[:2] - self.nominal[:2]))
                <= self.position_tolerance_m
                and float(np.linalg.norm(position[:2] - self.nominal[:2]))
                <= self.position_tolerance_m
            )
            if at_center and stable:
                if self.complete_since_s is None:
                    self.complete_since_s = now_s
                if now_s - self.complete_since_s >= self.stable_dwell_s:
                    self.phase = 'complete'
                    self.complete = True
                    self.target = self.nominal.copy()
            else:
                self.complete_since_s = None
        return self.target.copy(), self.phase
