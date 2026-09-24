"""Opt-in S-curve setup; no real CRTP or flight commands."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from Interaction.tests import test_firmware_auto_brake_preflight as baseline
from Interaction.tests.test_firmware_parameter_confirmation import AsyncParams
from Interaction.firmware_parameter_confirmation import confirm_firmware_mode_parameters

class SCurvePreflightTests(unittest.TestCase):
    def test_analytic_profile_confirmed_before_enable_and_recorded(self):
        profile=dict(shape='single_position_polynomial', execution='attitude',
            tail_s=.7, single_s=.7, handoff='predicted_bumpless', feedback='unified_vicon15')
        ctrl=baseline.FirmwareAutoBrakePreflightTests().controller(dict(
            enabled=True,mode='scurve',response_time_s=.14,command_mode='attitude',analytic_profile=profile))
        ctrl.prepare_firmware_auto_brake()
        p=self.velocity_params();ctrl.cf=SimpleNamespace(param=p)
        for key, value in ctrl._firmware_analytic_expected.items():
            group, name=key.split('.')
            p.toc.toc.setdefault(group,{})[name]=None;p.replies[key]=str(value)
        ctrl.log_manager=SimpleNamespace(curve_recorder=SimpleNamespace(ids={},events=Mock()))
        ctrl._setup_firmware_auto_brake_params()
        writes=[call.args for call in p.set_value.call_args_list]
        self.assertEqual(writes[0],('hlCommander.pRelAuto','0'))
        self.assertEqual(writes[-1],('hlCommander.pRelAuto','1'))
        for key,value in ctrl._firmware_analytic_expected.items():
            self.assertIn((key,str(value)),writes)
            self.assertIn(key,p.requested)
        metadata=ctrl.log_manager.curve_recorder.events.write.call_args.args[0]
        self.assertEqual(metadata['command_mode'],'attitude')
        self.assertEqual(metadata['velocity_source'],'unified_vicon15')
        self.assertFalse(any('Pid.' in key or 'pid_' in key for key,_ in writes))

    def velocity_controller(self, **overrides):
        ctrl=baseline.FirmwareAutoBrakePreflightTests().controller(dict(
            enabled=True,mode='scurve',response_time_s=.14,command_mode='velocity',**overrides))
        ctrl.prepare_firmware_auto_brake()
        return ctrl

    def velocity_params(self):
        p=self.params()
        p.toc.toc['hlCommander'].update(dict.fromkeys(('pRelVelCmd','pRelAdapt')))
        p.toc.toc['pRelResp']={'commit':None}
        p.replies.update({'hlCommander.pRelVelCmd':'1','hlCommander.pRelAdapt':'0'})
        return p

    def test_velocity_mode_disables_attitude_worker_and_confirms_before_enable(self):
        ctrl=self.velocity_controller();p=self.velocity_params();ctrl.cf=SimpleNamespace(param=p)
        ctrl.log_manager=SimpleNamespace(curve_recorder=SimpleNamespace(ids={},events=Mock()))
        with patch('Interaction.firmware_response_model.upload_model') as upload,patch('Interaction.post_release_pi_planner.PiEventPlanner') as worker:
            ctrl._setup_firmware_auto_brake_params()
            upload.assert_not_called();worker.assert_not_called()
        writes=[call.args for call in p.set_value.call_args_list]
        self.assertEqual(writes[0],('hlCommander.pRelAuto','0'))
        self.assertIn(('hlCommander.pRelAdapt','0'),writes)
        self.assertIn(('pRelResp.commit','0'),writes)
        self.assertIn(('hlCommander.pRelVelCmd','1'),writes)
        self.assertEqual(writes[-1],('hlCommander.pRelAuto','1'))
        self.assertEqual(set(p.requested),{'hlCommander.pRelVelCmd','hlCommander.pRelAdapt'})
        self.assertFalse(any('Pid.' in k or 'pid_' in k for k,v in writes))
        self.assertFalse(ctrl.log_manager.curve_recorder.events.write.call_args.args[0]['replanning'])

    def test_velocity_requires_new_capability_not_a_specific_build_number(self):
        ctrl=self.velocity_controller();p=self.params();ctrl.cf=SimpleNamespace(param=p)
        with self.assertRaisesRegex(RuntimeError,'lacks firmware auto-brake parameters'):
            ctrl._setup_firmware_auto_brake_params()
        p.set_value.assert_not_called()

    def modern_velocity_params(self):
        p = self.velocity_params()
        for name in ('pRelExec', 'pRelShape', 'pRelLite'):
            p.toc.toc['hlCommander'][name] = None
            p.replies['hlCommander.' + name] = '0'
        return p

    def test_plain_velocity_explicitly_selects_execution_on_multimode_firmware(self):
        ctrl = self.velocity_controller()
        p = self.modern_velocity_params()
        ctrl.cf = SimpleNamespace(param=p)
        ctrl.log_manager = SimpleNamespace(curve_recorder=SimpleNamespace(ids={}, events=Mock()))
        ctrl._setup_firmware_auto_brake_params()
        writes = [call.args for call in p.set_value.call_args_list]
        for name in ('pRelExec', 'pRelShape', 'pRelLite'):
            key = 'hlCommander.' + name
            self.assertIn((key, '0'), writes)
            self.assertIn(key, p.requested)
        self.assertEqual(writes[-1], ('hlCommander.pRelAuto', '1'))
        metadata = ctrl.log_manager.curve_recorder.events.write.call_args.args[0]
        self.assertEqual(metadata['command_mode'], 'velocity')
        self.assertEqual(metadata['confirmed_parameters']['hlCommander.pRelExec'], 0)
        self.assertIsNone(metadata['velocity_tail_s'])  # No invented firmware default.

    def test_wrong_execution_readback_cannot_enable_braking(self):
        ctrl = self.velocity_controller()
        p = self.modern_velocity_params()
        p.replies['hlCommander.pRelExec'] = '2'  # Firmware is still in attitude mode.
        ctrl.cf = SimpleNamespace(param=p)
        def quick_confirm(param, *, expected):
            return confirm_firmware_mode_parameters(param, expected=expected, timeout_s=.001)
        with patch('Interaction.firmware_parameter_confirmation.confirm_firmware_mode_parameters',
                   side_effect=quick_confirm):
            with self.assertRaisesRegex(RuntimeError, 'pRelExec=2'):
                ctrl._setup_firmware_auto_brake_params()
        self.assertNotIn(('hlCommander.pRelAuto', '1'),
                         [call.args for call in p.set_value.call_args_list])

    def test_prearm_rechecks_execution_and_negotiated_log_protocol(self):
        ctrl = self.velocity_controller()
        p = self.modern_velocity_params()
        ctrl.cf = SimpleNamespace(param=p)
        ctrl._setup_firmware_auto_brake_params()
        p.replies.update({'hlCommander.curveVer': '2', 'hlCommander.curveLog': '1'})
        ctrl.log_manager = SimpleNamespace(
            curve_recorder=SimpleNamespace(check=Mock(), events_enabled=True, protocol_version=2),
            get_latest_group_log_data=lambda _: {
                'hlCommander.pRelReady': 1, 'hlCommander.pRelAutoSt': 0,
                'hlCommander.pRelEvtVer': 1, 'hlCommander.pRelMode': 2,
                'hlCommander.pRelTau': .14})
        ctrl._firmware_vicon_last_send_s = 1.
        ctrl._firmware_vicon_mirror_error = None
        p.requested.clear()
        with patch('controller.time.monotonic', return_value=1.):
            ctrl.verify_firmware_auto_brake_ready()
        for name in ('pRelExec', 'pRelShape', 'pRelLite', 'curveVer', 'curveLog'):
            self.assertIn('hlCommander.' + name, p.requested)

    def test_velocity_rejects_attitude_response_fit_and_fixed_distance(self):
        with self.assertRaisesRegex(ValueError,'not a velocity-loop model'):
            self.velocity_controller(response_model={'enabled':True})
        ctrl=self.velocity_controller(stop_distance_m=.5)
        p=self.velocity_params();ctrl.cf=SimpleNamespace(param=p)
        with self.assertRaisesRegex(ValueError,'free-stop distance'):
            ctrl._setup_firmware_auto_brake_params()
        p.set_value.assert_not_called()

    def test_prearm_reconfirms_velocity_mode_and_worker_disabled(self):
        ctrl=self.velocity_controller();p=self.velocity_params();ctrl.cf=SimpleNamespace(param=p)
        ctrl.log_manager=SimpleNamespace(get_latest_group_log_data=lambda _:{
            'hlCommander.pRelReady':1,'hlCommander.pRelAutoSt':0,
            'hlCommander.pRelEvtVer':1,'hlCommander.pRelMode':2,'hlCommander.pRelTau':.14})
        ctrl._firmware_vicon_last_send_s=1.;ctrl._firmware_vicon_mirror_error=None
        with patch('controller.time.monotonic',return_value=1.):
            ctrl.verify_firmware_auto_brake_ready()
        self.assertIn('hlCommander.pRelVelCmd',p.requested)
        self.assertIn('hlCommander.pRelAdapt',p.requested)
        self.assertFalse(p.callbacks)

    def test_attitude_selection_clears_velocity_option(self):
        ctrl=self.controller();p=self.velocity_params();ctrl.cf=SimpleNamespace(param=p)
        ctrl._setup_firmware_auto_brake_params()
        self.assertIn(('hlCommander.pRelVelCmd','0'),[c.args for c in p.set_value.call_args_list])

    def controller(self, mode='scurve'):
        ctrl=baseline.FirmwareAutoBrakePreflightTests().controller({
            'enabled':True,'mode':mode,'response_time_s':.14})
        ctrl.prepare_firmware_auto_brake()
        return ctrl

    def params(self, version=26092305, distance='0.0'):
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

    def test_missing_capability_rejects_before_any_write(self):
        p=self.params();del p.toc.toc['kalmanPRel']['scEnable']
        ctrl=self.controller();ctrl.cf=SimpleNamespace(param=p)
        with self.assertRaises(RuntimeError):ctrl._setup_firmware_auto_brake_params()
        p.set_value.assert_not_called()

    def test_missing_build_number_does_not_reject_capable_firmware(self):
        p=self.params();del p.toc.toc['hlCommander']['pRelSVer']
        ctrl=self.controller();ctrl.cf=SimpleNamespace(param=p)
        ctrl._setup_firmware_auto_brake_params()
        self.assertIsNone(ctrl.firmware_auto_brake_scurve_version)
        self.assertEqual(p.set_value.call_args.args,('hlCommander.pRelAuto','1'))

    def test_fresh_mode_confirmation_and_original_observer_ready_gate(self):
        ctrl=self.controller();p=self.params();ctrl.cf=SimpleNamespace(param=p)
        ctrl.log_manager=SimpleNamespace(get_latest_group_log_data=lambda _:{
            'hlCommander.pRelReady':1,'hlCommander.pRelAutoSt':0,
            'hlCommander.pRelEvtVer':1,'hlCommander.pRelMode':2,'hlCommander.pRelTau':.14})
        ctrl._firmware_vicon_last_send_s=1.;ctrl._firmware_vicon_mirror_error=None
        with patch('controller.time.monotonic',return_value=1.):
            ctrl.verify_firmware_auto_brake_ready()
        self.assertEqual(set(p.requested),set(p.replies)-{'hlCommander.pRelSVer'});self.assertFalse(p.callbacks)

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

    def test_build_numbers_are_diagnostic_only_including_unknown_versions(self):
        for version in (1, 26092201, 26092202, 26092203, 26092304, 26092305, 99999999):
            ctrl=self.controller();p=self.params(version);ctrl.cf=SimpleNamespace(param=p)
            ctrl._setup_firmware_auto_brake_params()
            self.assertEqual(ctrl.firmware_auto_brake_scurve_version, version)

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
