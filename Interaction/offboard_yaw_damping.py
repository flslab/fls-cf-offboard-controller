"""Temporary, recoverable yaw-angle PID bypass using existing parameters only.

Prepare a recovery record while grounded; enable after stable interaction arming.
Leave the rate PID unchanged, so ordinary
position and attitude commands both request zero yaw rate from that inner loop.
Never store parameters to firmware persistent storage.
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


def _gains(values):
    if set(values) != set(YAW_ANGLE_GAINS):
        raise ValueError('yaw gain backup must contain exactly the four yaw-angle gains')
    result = {key: float(values[key]) for key in YAW_ANGLE_GAINS}
    if any(not math.isfinite(value) or not 0 <= value <= 10000 for value in result.values()):
        raise ValueError('invalid yaw-angle gain backup')
    return result


class OffboardYawDamping:
    def __init__(self, cf, backup_path):
        self.cf = cf
        self.path = Path(backup_path)
        self.original = None
        self.prepared = False
        self._worker = None
        self._done = threading.Event()
        self._error = None

    def restore(self):
        """Call only after landing/stop, or before arming on the next connection."""
        # Never let a pending enable overwrite the restored gains later.
        if self._worker is not None:
            self._worker.join()
        self.prepared = False
        if not self.path.exists():
            return
        document = json.loads(self.path.read_text())
        if document.get('schema') != 1:
            raise ValueError('unsupported yaw-angle gain backup')
        values = _gains(document['gains'])
        for name, value in values.items():
            self.cf.param.set_value(name, str(value))
        confirm_firmware_mode_parameters(self.cf.param, expected=values)
        self.path.unlink()  # Keep recovery data if any write/readback fails.
        self.cf._offboard_yaw_damping_active = False
        logger.info('Yaw-angle PID gains restored')

    def prepare(self):
        """Grounded preflight: confirm originals and save them, without gain writes."""
        self.cf._offboard_yaw_damping_active = False
        toc = getattr(getattr(self.cf.param, 'toc', None), 'toc', {})
        for name in YAW_ANGLE_GAINS:
            group, item = name.split('.')
            if item not in toc.get(group, {}):
                raise RuntimeError(f'yaw damping requires existing parameter {name}')
        self.original = _gains({name: self.cf.param.get_value(name) for name in YAW_ANGLE_GAINS})
        # Cache supplies the expected values only; explicit fresh reads prove
        # the original gains and active PID controller before any gain write.
        confirm_firmware_mode_parameters(self.cf.param, expected={
            **self.original, 'stabilizer.controller': 1,
        })
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open('x') as stream:
            json.dump({'schema': 1, 'gains': self.original}, stream)
            stream.flush()
            os.fsync(stream.fileno())
        self.prepared = True

    def request_enable(self):
        """Start once after stable arming; poll without blocking flight commands.

        A failed/partial activation retains the recovery record. The caller must
        land on error and restore afterwards, as with other controller failures.
        """
        if not self.prepared:
            raise RuntimeError('yaw damping was not prepared before takeoff')
        if self._worker is None:
            self._worker = threading.Thread(target=self._enable, name='yaw-damping-enable', daemon=True)
            self._worker.start()
        if not self._done.is_set():
            return False
        if self._error is not None:
            raise RuntimeError('yaw damping activation failed; landing required') from self._error
        return True

    def _enable(self):
        try:
            for name in YAW_ANGLE_GAINS:
                self.cf.param.set_value(name, '0')
            confirm_firmware_mode_parameters(self.cf.param, expected={
                name: 0.0 for name in YAW_ANGLE_GAINS})
            self.cf._offboard_yaw_damping_active = True
            logger.info('Yaw rate damping enabled: yaw-angle PID bypassed, rate PID unchanged')
        except Exception as error:
            self._error = error
        finally:
            self._done.set()
