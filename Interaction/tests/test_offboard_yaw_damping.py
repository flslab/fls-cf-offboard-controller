import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from Interaction.offboard_yaw_damping import OffboardYawDamping, YAW_ANGLE_GAINS
from controller import Controller


class Param:
    def __init__(self):
        self.values = dict(zip(YAW_ANGLE_GAINS, (6., 1., .1, .2)))
        self.values['stabilizer.controller'] = 1
        self.toc = SimpleNamespace(toc={'pid_attitude': {
            name.split('.')[1]: object() for name in YAW_ANGLE_GAINS}})
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
        self.guard = OffboardYawDamping(self.cf, self.path)

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
