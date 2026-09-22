"""Opt-in S-curve setup; no real CRTP or flight commands."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from Interaction.tests import test_firmware_auto_brake_preflight as baseline
from Interaction.tests.test_firmware_parameter_confirmation import AsyncParams
from Interaction.firmware_parameter_confirmation import confirm_firmware_mode_parameters

class SCurvePreflightTests(unittest.TestCase):
    def controller(self, mode='scurve'):
        ctrl=baseline.FirmwareAutoBrakePreflightTests().controller({
            'enabled':True,'mode':mode,'response_time_s':.14})
        ctrl.prepare_firmware_auto_brake()
        return ctrl

    def params(self, version=26092201, distance='0.0'):
        p=AsyncParams({'hlCommander.pRelSVer':str(version),
            'hlCommander.pRelMode':'2','hlCommander.pRelJoint':'0',
            'hlCommander.pRelHost':'0','kalmanPRel.scEnable':'1',
            'hlCommander.pRelScD':distance,'hlCommander.pRelScT':'6.0',
            'hlCommander.pRelScB':'0.3'})
        p.toc=SimpleNamespace(toc={
            'kalmanPRel':dict.fromkeys(('enable','scEnable')),
            'hlCommander':dict.fromkeys(('pRelAuto','pRelMode','pRelTau','pRelSVer',
                                         'pRelJoint','pRelHost','pRelJVer',
                                         'pRelScD','pRelScT','pRelScB'))})
        p.get_value=Mock(side_effect=lambda name:p.replies[name])
        return p

    def test_mode_two_and_predictor_enabled_without_pi_worker_or_pid_writes(self):
        ctrl=self.controller();p=self.params();ctrl.cf=SimpleNamespace(param=p)
        with patch('Interaction.post_release_pi_planner.PiEventPlanner') as worker:
            ctrl._setup_firmware_auto_brake_params()
            worker.assert_not_called()
        writes=[call.args for call in p.set_value.call_args_list]
        for item in [('hlCommander.pRelMode','2'),('kalmanPRel.scEnable','1'),
                     ('hlCommander.pRelJoint','0'),('hlCommander.pRelHost','0')]:
            self.assertIn(item,writes)
        self.assertEqual(writes[-1],('hlCommander.pRelAuto','1'))
        self.assertFalse(any('Pid.' in k or 'pid_' in k for k,v in writes))

    def test_old_modes_clear_predictor_but_preserve_existing_mode_codes(self):
        for mode,code in [('two_phase','0'),('zero_velocity','1')]:
            ctrl=self.controller(mode);p=self.params();ctrl.cf=SimpleNamespace(param=p)
            ctrl._setup_firmware_auto_brake_params()
            writes=[call.args for call in p.set_value.call_args_list]
            self.assertIn(('kalmanPRel.scEnable','0'),writes)
            self.assertIn(('hlCommander.pRelMode',code),writes)

    def test_version_and_missing_capability_reject_before_any_write(self):
        for fault in ('version','missing'):
            p=self.params(1 if fault=='version' else 26092201)
            if fault=='missing':del p.toc.toc['kalmanPRel']['scEnable']
            ctrl=self.controller();ctrl.cf=SimpleNamespace(param=p)
            with self.assertRaises(RuntimeError):ctrl._setup_firmware_auto_brake_params()
            p.set_value.assert_not_called()

    def test_fresh_mode_confirmation_and_original_observer_ready_gate(self):
        ctrl=self.controller();p=self.params();ctrl.cf=SimpleNamespace(param=p)
        ctrl.log_manager=SimpleNamespace(get_latest_group_log_data=lambda _:{
            'hlCommander.pRelReady':1,'hlCommander.pRelAutoSt':0,
            'hlCommander.pRelEvtVer':1,'hlCommander.pRelMode':2,'hlCommander.pRelTau':.14})
        ctrl._firmware_vicon_last_send_s=1.;ctrl._firmware_vicon_mirror_error=None
        with patch('controller.time.monotonic',return_value=1.):
            ctrl.verify_firmware_auto_brake_ready()
        self.assertEqual(set(p.requested),set(p.replies));self.assertFalse(p.callbacks)

    def test_distance_mode_writes_and_confirms_the_selected_target(self):
        ctrl=baseline.FirmwareAutoBrakePreflightTests().controller({
            'enabled':True,'mode':'scurve','response_time_s':.14,'stop_distance_m':1.2})
        ctrl.prepare_firmware_auto_brake()
        p=self.params(distance='1.2');ctrl.cf=SimpleNamespace(param=p)
        ctrl._setup_firmware_auto_brake_params()
        writes=[call.args for call in p.set_value.call_args_list]
        self.assertIn(('hlCommander.pRelScD','1.2'),writes)
        ctrl.log_manager=SimpleNamespace(get_latest_group_log_data=lambda _:{
            'hlCommander.pRelReady':1,'hlCommander.pRelAutoSt':0,
            'hlCommander.pRelEvtVer':1,'hlCommander.pRelMode':2,'hlCommander.pRelTau':.14})
        ctrl._firmware_vicon_last_send_s=1.;ctrl._firmware_vicon_mirror_error=None
        with patch('controller.time.monotonic',return_value=1.):
            ctrl.verify_firmware_auto_brake_ready()

    def test_distance_target_requires_scurve_mode(self):
        for mode in ('two_phase','zero_velocity','pi_joint'):
            ctrl=baseline.FirmwareAutoBrakePreflightTests().controller({
                'enabled':True,'mode':mode,'response_time_s':.14,'stop_distance_m':1.0})
            with self.assertRaises(ValueError):ctrl.prepare_firmware_auto_brake()

    def test_cached_mode_two_without_fresh_replies_does_not_authorize(self):
        p=self.params();expected={k:float(v) if '.' in v else int(v)
                                  for k,v in p.replies.items()};p.replies=None
        with self.assertRaisesRegex(RuntimeError,'no fresh reply'):
            confirm_firmware_mode_parameters(p,timeout_s=.001,expected=expected)
        p.get_value.assert_not_called();self.assertFalse(p.callbacks)

    def test_pointcloud_and_rigidbody_supported_full_pose_rejected(self):
        for mode in ('pointcloud','rigidbody'):
            ctrl=self.controller();ctrl.args.vicon_mode=mode;ctrl.prepare_firmware_auto_brake()
            ctrl.args.vicon_full_pose=True
            with self.assertRaises(ValueError):ctrl.prepare_firmware_auto_brake()

if __name__=='__main__':unittest.main()
