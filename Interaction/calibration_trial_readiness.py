"""Fresh-state readiness for position-held pauses between calibration trials.

The caller must keep issuing the nominal position target while ``waiting``.
This gate never changes an already-running open-loop maneuver or its timing.
"""

from __future__ import annotations

import math


class CalibrationTrialReadinessGate:
    def __init__(self, config):
        config = dict(config or {})
        defaults = {
            "trial_start_max_xy_speed_m_s": 0.05,
            "trial_start_max_tilt_deg": 4.0,
            "trial_start_max_position_error_m": 0.08,
            "trial_start_dwell_s": 0.30,
            "trial_start_timeout_s": 5.0,
            "trial_start_max_sample_gap_s": 0.10,
        }
        for name, default in defaults.items():
            value = float(config.get(name, default))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
            setattr(self, name, value)
        if self.trial_start_dwell_s >= self.trial_start_timeout_s:
            raise ValueError("trial start dwell must be shorter than timeout")
        self.key = None
        self.waiting = False
        self.stable_elapsed_s = 0.0
        self._admitted = set()
        self._wait_started_at = None
        self._completed_wait_s = 0.0
        self._last_now = None
        self._last_sample_time = None
        self._last_sample_now = None
        self._stable_sample_start = None
        self._stable_wall_start = None

    def _time(self, now):
        now = float(now)
        if not math.isfinite(now):
            raise ValueError("trial readiness time must be finite")
        if self._last_now is not None and now < self._last_now - 1e-9:
            raise ValueError("trial readiness clock moved backwards")
        self._last_now = now
        return now

    def admitted(self, key):
        return key in self._admitted

    def begin(self, key, now):
        now = self._time(now)
        if self.admitted(key):
            return
        if self.waiting:
            if key != self.key:
                raise ValueError("cannot replace a pending calibration trial")
            return
        self.key = key
        self.waiting = True
        self._wait_started_at = now
        self._last_sample_time = None
        self._last_sample_now = None
        self._reset_dwell()

    def wait_elapsed_s(self, now):
        now = self._time(now)
        return max(0.0, now - self._wait_started_at) if self.waiting else 0.0

    def total_wait_s(self, now):
        return self._completed_wait_s + self.wait_elapsed_s(now)

    def poll(self, now):
        elapsed = self.wait_elapsed_s(now)
        if self.waiting and elapsed >= self.trial_start_timeout_s - 1e-9:
            raise TimeoutError(
                f"Calibration trial {self.key!r} did not settle within "
                f"{self.trial_start_timeout_s:.2f}s of extra position hold "
                f"(requires XY speed <= {self.trial_start_max_xy_speed_m_s:.3f}m/s, "
                f"tilt <= {self.trial_start_max_tilt_deg:.2f}deg, "
                f"XY position error <= {self.trial_start_max_position_error_m:.3f}m "
                f"for {self.trial_start_dwell_s:.2f}s)"
            )

    def _reset_dwell(self):
        self._stable_sample_start = None
        self._stable_wall_start = None
        self.stable_elapsed_s = 0.0

    def invalidate(self, now):
        self.poll(now)
        self._reset_dwell()

    def no_new_sample(self, now):
        """Do not credit repeated polls, but allow slower fresh telemetry.

        A duplicate between two timely unique samples is not a dropout. Its
        age is bounded by the same maximum inter-sample gap as unique data.
        """
        self.poll(now)
        if (self._last_sample_now is not None
                and now - self._last_sample_now
                > self.trial_start_max_sample_gap_s + 1e-9):
            self._reset_dwell()

    def update(self, key, now, sample_time, xy_speed_m_s, tilt_deg,
               position_error_m):
        self.poll(now)
        if self.admitted(key):
            return True
        if not self.waiting or self.key != key:
            raise ValueError("begin the matching trial before updating readiness")
        values = [float(value) for value in (
            sample_time, xy_speed_m_s, tilt_deg, position_error_m
        )]
        if not all(math.isfinite(value) for value in values):
            self._reset_dwell()
            raise ValueError("trial readiness state must be finite")
        sample_time, speed, tilt, position_error = values
        if min(speed, tilt, position_error) < 0:
            self._reset_dwell()
            raise ValueError("trial readiness magnitudes must be nonnegative")
        if self._last_sample_time is not None:
            sample_gap = sample_time - self._last_sample_time
            wall_gap = now - self._last_sample_now
            if sample_gap < 0:
                self._reset_dwell()
                return False
            if sample_gap == 0:
                self.no_new_sample(now)
                return False
            if max(sample_gap, wall_gap) > self.trial_start_max_sample_gap_s + 1e-9:
                self._reset_dwell()
        self._last_sample_time = sample_time
        self._last_sample_now = now
        qualifies = (
            speed <= self.trial_start_max_xy_speed_m_s
            and tilt <= self.trial_start_max_tilt_deg
            and position_error <= self.trial_start_max_position_error_m
        )
        if not qualifies:
            self._reset_dwell()
            return False
        if self._stable_sample_start is None:
            self._stable_sample_start = sample_time
            self._stable_wall_start = now
        self.stable_elapsed_s = min(
            sample_time - self._stable_sample_start,
            now - self._stable_wall_start,
        )
        if self.stable_elapsed_s + 1e-9 < self.trial_start_dwell_s:
            return False
        self._completed_wait_s += now - self._wait_started_at
        self._admitted.add(key)
        self.waiting = False
        return True
