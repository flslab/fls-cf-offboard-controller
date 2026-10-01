"""Recoverable yaw damping/deadband using existing firmware parameters only.

Prepare while grounded; activate after stable interaction arming. A positive
deadband uses P-only rate damping, switching its gain on threshold crossings.
All parameter writes/readbacks run outside the flight-command thread. No
parameters are stored to firmware persistent storage.
"""

import json
import logging
import math
import os
import threading
from pathlib import Path

from Interaction.firmware_parameter_confirmation import confirm_firmware_mode_parameters

logger = logging.getLogger(__name__)
YAW_ANGLE_GAINS = tuple('pid_attitude.yaw_' + term for term in ('kp', 'ki', 'kd', 'kff'))
YAW_RATE_GAINS = tuple('pid_rate.yaw_' + term for term in ('kp', 'ki', 'kd', 'kff'))
DEFAULT_YAW_RATE_DEADBAND_DEG_S = 10.0
RATE_KP = YAW_RATE_GAINS[0]


def validate_yaw_deadband(value):
    result = float(value)
    if isinstance(value, bool) or not math.isfinite(result) or result < 0:
        raise ValueError('level_coast.yaw_rate_deadband_deg_s must be finite and non-negative')
    return result


def _gains(values, names=YAW_ANGLE_GAINS):
    if set(values) != set(names):
        raise ValueError('yaw gain backup has unexpected parameters')
    result = {key: float(values[key]) for key in names}
    if any(not math.isfinite(value) or not 0 <= value <= 10000 for value in result.values()):
        raise ValueError('invalid yaw-angle gain backup')
    return result


class OffboardYawDamping:
    def __init__(self, cf, backup_path, *, deadband_deg_s=DEFAULT_YAW_RATE_DEADBAND_DEG_S):
        self.cf = cf
        self.path = Path(backup_path)
        self.deadband_deg_s = validate_yaw_deadband(deadband_deg_s)
        self.gain_names = YAW_ANGLE_GAINS + (YAW_RATE_GAINS if self.deadband_deg_s else ())
        self.original = None
        self.prepared = False
        self._worker = None
        self._done = threading.Event()
        self._error = None
        self._condition = threading.Condition()
        self._stop = False
        self._finished = False
        self._requested = False
        self._confirmed = None
        self._switching = False

    def restore(self):
        """Call only after landing/stop, or before arming on the next connection."""
        # Never let a pending enable/switch overwrite restored gains later.
        with self._condition:
            self._stop = True
            self._condition.notify_all()
        if self._worker is not None:
            self._worker.join()
        self.prepared = False
        if not self.path.exists():
            return
        document = json.loads(self.path.read_text())
        schema = document.get('schema')
        if schema not in (1, 2):
            raise ValueError('unsupported yaw-angle gain backup')
        values = _gains(document['gains'], YAW_ANGLE_GAINS + (YAW_RATE_GAINS if schema == 2 else ()))
        for name, value in values.items():
            self.cf.param.set_value(name, str(value))
        confirm_firmware_mode_parameters(self.cf.param, expected=values)
        self.path.unlink()  # Keep recovery data if any write/readback fails.
        self.cf._offboard_yaw_damping_active = False
        logger.info('Yaw PID gains restored')

    def prepare(self):
        """Grounded preflight: confirm originals and save them, without gain writes."""
        self.cf._offboard_yaw_damping_active = False
        toc = getattr(getattr(self.cf.param, 'toc', None), 'toc', {})
        for name in self.gain_names:
            group, item = name.split('.')
            if item not in toc.get(group, {}):
                raise RuntimeError(f'yaw damping requires existing parameter {name}')
        self.original = _gains({name: self.cf.param.get_value(name) for name in self.gain_names},
                               self.gain_names)
        if self.deadband_deg_s and self.original[RATE_KP] <= 0:
            raise ValueError('yaw deadband requires a positive original pid_rate.yaw_kp')
        # Cache supplies the expected values only; explicit fresh reads prove
        # the original gains and active PID controller before any gain write.
        confirm_firmware_mode_parameters(self.cf.param, expected={
            **self.original, 'stabilizer.controller': 1,
        })
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open('x') as stream:
            json.dump({'schema': 2 if self.deadband_deg_s else 1, 'gains': self.original}, stream)
            stream.flush()
            os.fsync(stream.fileno())
        self.prepared = True
        self._stop = False

    def request_enable(self, yaw_rate_rad_s=0.):
        """Start once after stable arming; poll without blocking flight commands.

        A failed/partial activation retains the recovery record. The caller must
        land on error and restore afterwards, as with other controller failures.
        """
        if not self.prepared:
            raise RuntimeError('yaw damping was not prepared before takeoff')
        self._request_rate(yaw_rate_rad_s)
        if self._worker is None:
            self._worker = threading.Thread(target=self._enable, name='yaw-damping-enable', daemon=True)
            self._worker.start()
        if not self._done.is_set():
            return False
        if self._error is not None:
            raise RuntimeError('yaw damping activation failed; landing required') from self._error
        return True

    def _request_rate(self, yaw_rate_rad_s):
        rate_deg_s = math.degrees(float(yaw_rate_rad_s))
        if not math.isfinite(rate_deg_s):
            raise ValueError('yaw damping requires a finite onboard yaw rate')
        with self._condition:
            if not self._finished:
                self._requested = abs(rate_deg_s) >= self.deadband_deg_s
                self._condition.notify_all()
        return rate_deg_s

    def update(self, yaw_rate_rad_s):
        """Submit the latest measurement; never queue obsolete threshold edges."""
        if not self._done.is_set() or self._error is not None:
            raise RuntimeError('yaw damping switch failed or activation incomplete; landing required') from self._error
        rate_deg_s = self._request_rate(yaw_rate_rad_s)
        with self._condition:
            return {
                'yaw_rate_measured_deg_s': rate_deg_s,
                'yaw_rate_damping_requested': self._requested,
                'yaw_rate_damping_output_enabled': self._confirmed,
                'yaw_rate_damping_switch_pending': self._switching or self._requested != self._confirmed,
            }

    def finish(self):
        """Exit deadband for landing, asynchronously retaining P-only damping.

        Do not restore I in flight: its hidden state has accumulated even while
        its gain was zero. Restore every original gain only after landing/stop.
        """
        with self._condition:
            self._finished = True
            self._requested = True
            self._condition.notify_all()

    def _enable(self):
        try:
            values = dict.fromkeys(self.gain_names, 0.0)
            with self._condition:
                requested = self._requested
            if self.deadband_deg_s:
                values[RATE_KP] = self.original[RATE_KP] if requested else 0.0
            for name, value in values.items():
                self.cf.param.set_value(name, str(value))
            confirm_firmware_mode_parameters(self.cf.param, expected=values)
            self.cf._offboard_yaw_damping_active = True
            with self._condition:
                self._confirmed = requested
            self._done.set()
            logger.info('Yaw damping enabled; deadband %.1f deg/s', self.deadband_deg_s)
            if not self.deadband_deg_s:
                return  # Legacy behavior leaves the full rate PID unchanged.
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._stop or self._requested != self._confirmed)
                    if self._stop:
                        return
                    requested = self._requested
                    self._switching = True
                value = self.original[RATE_KP] if requested else 0.0
                self.cf.param.set_value(RATE_KP, str(value))
                confirm_firmware_mode_parameters(self.cf.param, timeout_s=.5,
                                                 expected={RATE_KP: value})
                with self._condition:
                    self._confirmed = requested
                    self._switching = False
                    self._condition.notify_all()
        except Exception as error:
            self._error = error
            logger.error('Yaw damping parameter update failed: %s', error)
        finally:
            self._done.set()
