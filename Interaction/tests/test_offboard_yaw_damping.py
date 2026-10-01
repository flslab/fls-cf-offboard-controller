import json
import math
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from Interaction.offboard_yaw_damping import (
    OffboardYawDamping, YAW_ANGLE_GAINS, YAW_RATE_GAINS, RATE_KP,
)
from controller import Controller


class Param:
    def __init__(self):
        self.values = dict(zip(YAW_ANGLE_GAINS, (6., 1., .1, .2)))
        self.values.update(zip(YAW_RATE_GAINS, (120., 16.7, .4, .2)))
        self.values['stabilizer.controller'] = 1
        self.toc = SimpleNamespace(toc={group: {
            name.split('.')[1]: object() for name in self.values if name.startswith(group+'.')}
            for group in ('pid_attitude', 'pid_rate')})
        self.callbacks = {}
        self.writes = []
        self.fail_once = None

    def get_value(self, name):
        return self.values[name]

    def add_update_callback(self, group, name, cb):
        self.callbacks[group+'.'+name] = cb

    def remove_update_callback(self, group, name, cb):
        self.callbacks.pop(group+'.'+name)

    def request_param_update(self, name):
        self.callbacks[name](name, str(self.values[name]))

    def set_value(self, name, value):
        if name == self.fail_once:
            self.fail_once = None
            raise OSError('write interrupted')
        self.writes.append((name, float(value)))
        self.values[name] = float(value)


class OffboardYawDampingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)/'gains.json'
        self.cf = SimpleNamespace(param=Param())
        self.original = dict(self.cf.param.values)
        self.guard = OffboardYawDamping(self.cf, self.path, deadband_deg_s=0.)

    def enable(self):
        self.guard.prepare()
        self.guard.request_enable()
        self.assertTrue(self.guard._done.wait(2))
        self.guard.request_enable()

    def test_prepare_keeps_original_gains_until_requested(self):
        self.guard.prepare()
        self.assertTrue(self.guard.prepared)
        self.assertFalse(self.cf._offboard_yaw_damping_active)
        self.assertEqual(self.cf.param.writes, [])
        self.assertEqual(self.cf.param.values, self.original)
        self.assertTrue(self.path.exists())

    def test_pending_confirmation_does_not_block_and_restore_waits_for_enable(self):
        self.guard.prepare()
        entered, release = threading.Event(), threading.Event()
        def confirm(*args, **kwargs):
            entered.set()
            if not release.wait(2):
                raise RuntimeError('test timeout')
        with patch('Interaction.offboard_yaw_damping.confirm_firmware_mode_parameters', side_effect=confirm):
            try:
                self.assertFalse(self.guard.request_enable())
                self.assertTrue(entered.wait(1))
                self.assertFalse(self.guard.request_enable())
                self.assertFalse(self.cf._offboard_yaw_damping_active)
                restored = threading.Event()
                worker = threading.Thread(target=lambda: (self.guard.restore(), restored.set()))
                worker.start()
                self.assertFalse(restored.wait(.02))
            finally:
                release.set()
                worker.join(2)
            self.assertTrue(restored.is_set())
        self.assertEqual(self.cf.param.values, self.original)
        self.assertFalse(self.cf._offboard_yaw_damping_active)
        self.assertFalse(self.path.exists())

    def test_zeroes_only_angle_gains_and_restores_exact_original_values(self):
        self.enable()
        self.assertTrue(self.cf._offboard_yaw_damping_active)
        self.assertEqual({k:self.cf.param.values[k] for k in YAW_ANGLE_GAINS}, dict.fromkeys(YAW_ANGLE_GAINS,0.))
        self.assertEqual(json.loads(self.path.read_text())['gains'], self.guard.original)
        self.guard.restore()
        self.assertFalse(self.cf._offboard_yaw_damping_active)
        self.assertEqual(self.cf.param.values, self.original)
        self.assertFalse(self.path.exists())
        self.assertTrue(all(k.startswith('pid_attitude.yaw_') for k,_ in self.cf.param.writes))

    def test_partial_activation_failure_retains_backup_until_grounded_restore(self):
        self.cf.param.fail_once = YAW_ANGLE_GAINS[2]
        with self.assertRaisesRegex(RuntimeError, 'activation failed'): self.enable()
        self.assertTrue(self.path.exists())
        self.assertNotEqual(self.cf.param.values, self.original)
        self.guard.restore()
        self.assertEqual(self.cf.param.values, self.original)
        self.assertFalse(self.path.exists())
        self.assertFalse(self.cf._offboard_yaw_damping_active)

    def test_failed_restore_keeps_backup_for_next_process(self):
        self.enable()
        self.cf.param.fail_once = YAW_ANGLE_GAINS[1]
        with self.assertRaises(OSError): self.guard.restore()
        self.assertTrue(self.path.exists())
        OffboardYawDamping(self.cf,self.path).restore()
        self.assertEqual(self.cf.param.values,self.original)
        self.assertFalse(self.path.exists())

    def test_unconfirmed_original_gains_or_controller_prevents_all_writes(self):
        with patch('Interaction.offboard_yaw_damping.confirm_firmware_mode_parameters',
                   side_effect=RuntimeError('fresh read mismatch')):
            with self.assertRaises(RuntimeError): self.enable()
        self.assertEqual(self.cf.param.writes, [])
        self.assertFalse(self.path.exists())

    def test_missing_parameter_does_not_change_any_gains(self):
        del self.cf.param.toc.toc['pid_attitude']['yaw_kff']
        with self.assertRaises(RuntimeError): self.enable()
        self.assertEqual(self.cf.param.writes, [])

    def test_backup_is_not_overwritten_by_second_enable(self):
        self.enable()
        document=self.path.read_text()
        with self.assertRaises(FileExistsError): self.enable()
        self.assertEqual(self.path.read_text(),document)

    def test_controller_checks_grounded_startup_and_pid(self):
        controller=Controller.__new__(Controller)
        controller.cf=self.cf
        controller.mission={'Interaction':{'config':{'behavior':'level_coast',
            'level_coast':{'yaw_rate_damping':True}}}}
        controller.args=SimpleNamespace(drone_id='unit-test',controller_type='pid',
            skip_takeoff=False,skip_landing=False,calibrate=False,interaction=True)
        controller.flying=False
        controller.log_manager=Mock()
        with patch('Interaction.offboard_yaw_damping.OffboardYawDamping', return_value=self.guard), \
             patch('pathlib.Path.exists', return_value=False):
            for field in ('skip_takeoff','skip_landing'):
                setattr(controller.args,field,True)
                with self.assertRaises(ValueError): controller._prepare_offboard_yaw_damping()
                setattr(controller.args,field,False)
            controller.args.controller_type='mellinger'
            with self.assertRaises(ValueError): controller._prepare_offboard_yaw_damping()
        self.assertEqual(self.cf.param.writes,[])

    def test_controller_prepares_without_activating(self):
        c = Controller.__new__(Controller)
        c.cf = self.cf
        c.mission = {'Interaction': {'config': {'behavior': 'level_coast',
            'level_coast': {'yaw_rate_damping': True}}}}
        c.args = SimpleNamespace(drone_id='unit-test', controller_type='pid',
            skip_takeoff=False, skip_landing=False, calibrate=False, interaction=True)
        c.flying = False
        c.log_manager = Mock()
        with patch('Interaction.offboard_yaw_damping.OffboardYawDamping', return_value=self.guard):
            c._prepare_offboard_yaw_damping()
        self.assertIs(c.cf._offboard_yaw_damping_guard, self.guard)
        self.assertTrue(self.guard.prepared)
        self.assertEqual(self.cf.param.writes, [])

    def test_cleanup_restores_after_landing_and_never_while_still_flying(self):
        from Interaction.tests.test_controller_logging_cleanup import methods
        for landing_failed in (False, True):
            with self.subTest(landing_failed=landing_failed):
                actions=[]
                guard=SimpleNamespace(restore=Mock(side_effect=lambda: actions.append('restore')))
                c=SimpleNamespace(mission_start_time=0,servo=None,bat_logger=None,mocap=None,
                    force_sensor=None,rpi_power_monitor=None,log_manager=None,tracker_process=None,
                    blinker_process=None,smooth_controller=None,led=None,flying=True,
                    _offboard_yaw_damping=guard,disconnect=Mock(side_effect=lambda: actions.append('disconnect')))
                def land():
                    actions.append('land')
                    if landing_failed: raise ConnectionError('link lost')
                    c.flying=False
                c.land=land
                stop=methods({'stop'},time=SimpleNamespace(time=lambda:10),logger=Mock())['stop']
                if landing_failed:
                    with self.assertRaises(ConnectionError):stop(c)
                    guard.restore.assert_not_called()
                    self.assertEqual(actions,['land','disconnect'])
                else:
                    stop(c)
                    self.assertEqual(actions,['land','restore','disconnect'])


class YawDeadbandTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)/'gains.json'
        self.cf = SimpleNamespace(param=Param())
        self.original = dict(self.cf.param.values)
        self.guard = OffboardYawDamping(self.cf, self.path)
        self.addCleanup(self.guard.restore)

    def enable(self, rate=0.):
        self.guard.prepare()
        self.guard.request_enable(math.radians(rate))
        self.assertTrue(self.guard._done.wait(2))
        self.assertTrue(self.guard.request_enable(math.radians(rate)))

    def settle(self, enabled):
        with self.guard._condition:
            self.assertTrue(self.guard._condition.wait_for(
                lambda: self.guard._confirmed == enabled and not self.guard._switching,
                timeout=2))

    def test_signed_threshold_has_no_output_from_p_i_d_or_ff_below_ten(self):
        self.enable()
        for rate in (0., 9.99, 10., 10.01, 40., 4., -9.99, -10., -10.01, -40., -4., 0.):
            with self.subTest(rate=rate):
                desired = abs(rate) >= 10.
                self.guard.update(math.radians(rate))
                self.settle(desired)
                self.assertEqual(self.cf.param.values[RATE_KP], 120. if desired else 0.)
                self.assertTrue(all(self.cf.param.values[k] == 0.
                    for k in YAW_ANGLE_GAINS + YAW_RATE_GAINS[1:]))
                # Firmware PID form with nonzero hidden integral/derivative:
                # only P is allowed to contribute; its sign opposes rotation.
                p, i, d, ff = (self.cf.param.values[k] for k in YAW_RATE_GAINS)
                output = p * (-rate) + i * 100. + d * 200. + ff * 0.
                self.assertEqual(output, -120.*rate if desired else 0.)
        self.assertFalse(self.guard.update(0.)['yaw_rate_damping_switch_pending'])

    def test_preparation_is_read_only_and_activation_handles_initial_high_rate(self):
        self.guard.prepare()
        self.assertEqual(self.cf.param.writes, [])
        self.assertEqual(json.loads(self.path.read_text())['schema'], 2)
        self.guard.request_enable(math.radians(-30.))
        self.assertTrue(self.guard._done.wait(2))
        self.assertTrue(self.guard.request_enable(math.radians(-30.)))
        self.assertEqual(self.cf.param.values[RATE_KP], 120.)
        self.guard.restore()
        self.assertEqual(self.cf.param.values, self.original)

    def test_runtime_switch_does_not_block_coalesces_latest_and_restores_after_join(self):
        self.enable()
        entered, release = threading.Event(), threading.Event()
        def confirm(*args, **kwargs):
            entered.set()
            if not release.wait(2):
                raise RuntimeError('test timeout')
        with patch('Interaction.offboard_yaw_damping.confirm_firmware_mode_parameters', side_effect=confirm):
            try:
                self.guard.update(math.radians(20.))
                self.assertTrue(entered.wait(1))
                for rate in (5., 30., -25., 0.):
                    status = self.guard.update(math.radians(rate))
                    self.assertTrue(status['yaw_rate_damping_switch_pending'])
                self.assertFalse(status['yaw_rate_damping_requested'])
            finally:
                release.set()
            self.settle(False)
        self.assertEqual(self.cf.param.values[RATE_KP], 0.)
        writes = [v for k,v in self.cf.param.writes if k == RATE_KP]
        self.assertEqual(writes, [0., 120., 0.])
        self.guard.restore()
        self.assertEqual(self.cf.param.values, self.original)
        self.assertFalse(self.guard._worker.is_alive())

    def test_readback_failure_is_raised_to_flight_loop_and_keeps_recovery(self):
        self.enable()
        with patch('Interaction.offboard_yaw_damping.confirm_firmware_mode_parameters',
                   side_effect=RuntimeError('no readback')):
            self.guard.update(math.radians(20.))
            self.guard._worker.join(2)
        self.assertFalse(self.guard._worker.is_alive())
        with self.assertRaisesRegex(RuntimeError, 'landing required'):
            self.guard.update(0.)
        self.assertTrue(self.path.exists())
        self.guard.restore()
        self.assertEqual(self.cf.param.values, self.original)

    def test_grounded_restore_waits_for_pending_switch_and_cannot_be_overwritten(self):
        self.enable()
        entered, release, restored = threading.Event(), threading.Event(), threading.Event()
        def confirm(*args, **kwargs):
            entered.set()
            if not release.wait(2):
                raise RuntimeError('test timeout')
        with patch('Interaction.offboard_yaw_damping.confirm_firmware_mode_parameters', side_effect=confirm):
            try:
                self.guard.update(math.radians(30.))
                self.assertTrue(entered.wait(1))
                worker = threading.Thread(target=lambda: (self.guard.restore(), restored.set()))
                worker.start()
                self.assertFalse(restored.wait(.02))
            finally:
                release.set()
                worker.join(2)
        self.assertTrue(restored.is_set())
        self.assertFalse(self.guard._worker.is_alive())
        self.assertEqual(self.cf.param.values, self.original)
        self.assertFalse(self.path.exists())

    def test_partial_rate_activation_and_restore_failure_keep_all_eight_originals(self):
        self.cf.param.fail_once = YAW_RATE_GAINS[2]
        with self.assertRaisesRegex(RuntimeError, 'activation failed'):
            self.enable()
        document = json.loads(self.path.read_text())
        self.assertEqual(set(document['gains']), set(YAW_ANGLE_GAINS+YAW_RATE_GAINS))
        self.cf.param.fail_once = RATE_KP
        with self.assertRaises(OSError):
            self.guard.restore()
        self.assertTrue(self.path.exists())
        OffboardYawDamping(self.cf, self.path, deadband_deg_s=0.).restore()
        self.assertEqual(self.cf.param.values, self.original)
        self.assertFalse(self.path.exists())

    def test_finish_enables_p_damping_for_landing_without_reintroducing_integral(self):
        self.enable()
        self.guard.finish()
        self.settle(True)
        self.guard.update(0.)  # A late sample cannot undo the landing decision.
        self.assertEqual(self.cf.param.values[RATE_KP], 120.)
        self.assertTrue(all(self.cf.param.values[k] == 0. for k in YAW_RATE_GAINS[1:]))

    def test_old_and_new_recovery_records_restore_even_with_deadband_disabled(self):
        for names, schema in ((YAW_ANGLE_GAINS, 1), (YAW_ANGLE_GAINS+YAW_RATE_GAINS, 2)):
            with self.subTest(schema=schema):
                values = {k:self.original[k] for k in names}
                self.path.write_text(json.dumps({'schema':schema, 'gains':values}))
                for k in names:
                    self.cf.param.values[k] = 0.
                OffboardYawDamping(self.cf, self.path, deadband_deg_s=0.).restore()
                self.assertEqual(self.cf.param.values, self.original)

    def test_invalid_threshold_rate_or_missing_rate_parameter_cannot_silently_disable_control(self):
        for value in (-1., float('nan'), float('inf'), True):
            with self.assertRaises(ValueError):
                OffboardYawDamping(self.cf, self.path, deadband_deg_s=value)
        del self.cf.param.toc.toc['pid_rate']['yaw_ki']
        with self.assertRaises(RuntimeError):
            self.guard.prepare()
        self.assertEqual(self.cf.param.writes, [])
        self.assertFalse(self.path.exists())

    def test_nonfinite_rate_aborts_before_writing(self):
        self.guard.prepare()
        with self.assertRaises(ValueError):
            self.guard.request_enable(float('nan'))
        self.assertEqual(self.cf.param.writes, [])

    def test_custom_threshold(self):
        self.guard = OffboardYawDamping(self.cf, self.path, deadband_deg_s=20.)
        self.addCleanup(self.guard.restore)
        self.enable(15.)
        self.assertEqual(self.cf.param.values[RATE_KP], 0.)
        self.guard.update(math.radians(21.))
        self.settle(True)
