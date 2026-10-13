"""Non-blocking contact estimator selection and the existing FC curve transport.

The paired hardware build exposes Kalman=2 and PostReleaseVicon15=3 as two
views of one running Kalman task. Switching between them does not reset it.
Only the firmware's matching terminal notice can complete an accepted curve.
"""

import threading
import time
from uuid import uuid4

from Interaction.firmware_parameter_confirmation import confirm_firmware_mode_parameters
from Interaction.post_release_firmware_control_event import (
    FirmwareBrakeMonitor, FirmwareBrakeMonitorError, FirmwareHoldNotification,
    FirmwareReleaseRejectedError, handoff_pi_release_to_firmware,
)


class ContactEstimatorSelector:
    DEFAULT = 2
    CONTACT = 3
    PARAMETER = 'stabilizer.estimator'

    def __init__(self, cf):
        self.cf = cf
        self.prepared = False
        self._lock = threading.Lock()
        self._requested = self.DEFAULT
        self._confirmed = None
        self._deadline = None
        self._callback = self._updated

    def prepare(self):
        # Confirm the paired hardware capabilities, the active PID, and the
        # ordinary estimator before arming. No experimental estimator pre-arm.
        confirm_firmware_mode_parameters(self.cf.param, expected={
            self.PARAMETER: self.DEFAULT, 'stabilizer.controller': 1,
            'kalmanPRel.feedback': 2, 'hlCommander.pRelEnd': 1,
            'hlCommander.pRelHoldG': 0,
        })
        self.cf.param.add_update_callback(group='stabilizer', name='estimator', cb=self._callback)
        self._confirmed = self.DEFAULT
        self.prepared = True

    def _updated(self, name, value):
        if name != self.PARAMETER:
            return
        try:
            value = int(value)
        except (TypeError, ValueError):
            return
        with self._lock:
            if value == self._requested:
                self._confirmed = value

    def request(self, contact, now=None):
        if not self.prepared:
            raise RuntimeError('S-curve contact estimator was not prepared before takeoff')
        requested = self.CONTACT if contact else self.DEFAULT
        calibration = getattr(self.cf, '_estimator_hover_xy', None)
        if contact and calibration is not None and not calibration.ready:
            raise RuntimeError('estimator-3 XY calibration has not been acknowledged and settled')
        with self._lock:
            if requested == self._requested:
                return
            self._requested = requested
            self._confirmed = None
            self._deadline = (time.monotonic() if now is None else now) + .5
        self.cf.param.set_value(self.PARAMETER, str(requested))
        # Queue an explicit read behind the write, without blocking setpoints.
        self.cf.param.request_param_update(self.PARAMETER)

    def ready(self, now=None):
        with self._lock:
            if self._confirmed == self._requested:
                return True
            now = time.monotonic() if now is None else now
            if now >= self._deadline:
                raise RuntimeError('S-curve estimator switch was not confirmed within 0.5 s')
            return False

    def close(self):
        # No worker can enqueue a late contact write after this restoration.
        try:
            self.cf.param.set_value(self.PARAMETER, str(self.DEFAULT))
        finally:
            if self.prepared:
                self.cf.param.remove_update_callback(group='stabilizer', name='estimator', cb=self._callback)
            self.prepared = False


class FirmwareSCurveCoast:
    def __init__(self, owner, selector):
        self.owner = owner
        self.selector = selector
        self.session = uuid4().int & 0xFFFFFFFF
        self.sequence = 0
        self.completion = None
        self.monitor = None
        self.hold_notice = None
        self.started_at = None
        self.active = False
        brake = owner.mission['Interaction']['config']['wrench_interaction']['firmware_auto_brake']
        self.timeout_s = float(brake.get('stop_max_time_s', 6.)) + 1.

    def begin(self, *, height, release_monotonic_s, sample_id=0):
        owner = self.owner
        owner.lo_commander.send_zdistance_setpoint(0., 0., 0., float(height))
        owner._translation_exit_target = None
        baseline, receipt = owner._firmware_brake_status_snapshot()
        self.started_at = time.monotonic()
        self.monitor = FirmwareBrakeMonitor(
            started_monotonic_s=self.started_at, baseline_receipt_time_s=receipt,
            baseline_timeouts=baseline.get('hlCommander.pRelAutoTime'))
        self.completion = FirmwareHoldNotification(owner.cf, session_id=self.session, sequence=self.sequence)
        self.completion.__enter__()
        try:
            event = handoff_pi_release_to_firmware(
                owner.cf, session_id=self.session, sequence=self.sequence,
                arduino_sample_ms=int(sample_id or 0),
                pi_receive_monotonic_ns=int(round(release_monotonic_s * 1e9)),
                firmware_auto_brake_armed=True)
        except FirmwareReleaseRejectedError as error:
            packet, details = owner._capture_firmware_release_rejection(receipt)
            error.add_firmware_diagnostics(packet)
            owner._log_event('Firmware Release Event Rejected', details)
            self.cancel()
            raise
        except Exception:
            self.cancel()
            raise
        owner._translation_high_level_active = True
        self.active = True
        self.hold_notice = None
        owner._log_event('Firmware Post-Release Brake Handoff', {**event, 'brake_mode': 'scurve'})
        print('[interaction] coast: s-curve', flush=True)

    def poll(self, now):
        if not self.active:
            return False
        if self.hold_notice is None:
            packet, receipt = self.owner._firmware_brake_status_snapshot()
            self.owner._check_firmware_brake_monitor_safety(packet)
            notice = self.completion.wait(0.)
            try:
                result = self.monitor.observe(packet, receipt_time_s=receipt, now_wall_s=time.time(),
                                              now_monotonic_s=now, notice=notice)
            except FirmwareBrakeMonitorError as error:
                from Interaction.interactions import StaleLocalizationError
                if error.code in ('status_expired', 'readiness_lost'):
                    raise StaleLocalizationError(str(error)) from error
                raise
            for name, details in result['events']:
                self.owner._log_event(name, details)
            if result['hold_confirmed']:
                self.owner.check_interaction_boundary(notice['hold_position_m'])
                self.completion.acknowledge()
                self.completion.__exit__(None, None, None)
                self.completion = None
                self.hold_notice = notice
                self.selector.request(False, now)
                self.owner._log_event('Level Coast S-Curve Completed', {
                    **notice, 'completion_condition': 'firmware_curve_endpoint',
                    'velocity_threshold_used': False})
                print('[interaction] s-curve done -> default estimator', flush=True)
            elif now - self.started_at > self.timeout_s:
                raise RuntimeError(f'S-curve did not report its completed endpoint within {self.timeout_s:g} s')
        return self.hold_notice is not None and self.selector.ready(now)

    def cancel(self):
        # The next LL packet takes control and cancels the FC plan. Merely
        # unregistering this listener is not an ownership transfer.
        if self.completion is not None:
            self.completion.__exit__(None, None, None)
            self.completion = None
        self.active = False
        self.hold_notice = None
        self.sequence = (self.sequence + 1) & 0xFFFF
        if not self.sequence:
            self.session = uuid4().int & 0xFFFFFFFF
