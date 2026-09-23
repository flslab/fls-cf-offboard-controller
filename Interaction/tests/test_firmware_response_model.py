import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch, Mock

from Interaction.firmware_response_model import (
    PID_NAMES, VERSION, load_model, model_parameters, save_report, upload_model,
    validate_report, fit_completed_calibration,
)
from Interaction.tests.test_firmware_parameter_confirmation import AsyncParams


def report():
    axis = dict(model='delayed_second_order', usable=True, delay_s=.04,
                wn_rad_s=13., zeta=.6, gain=1., bias_deg=.1,
                r_squared=.98, normalized_rmse=.1, rmse_deg=.5)
    return dict(fit_schema_version=1, usable=True, clock_basis='firmware_timestamp_ms',
                attitude_source='ordinary', axes={'roll': dict(axis), 'pitch': dict(axis)})


def model():
    return dict(report=report(), pid_values=dict.fromkeys(PID_NAMES, 1.),
                source_log='test only', source_sha256='unit-test')


class ResponseModelTests(unittest.TestCase):
    def params(self, *, runtime=1):
        m = model()
        replies = dict(model_parameters(m), **{'pRelResp.ver': VERSION,
            'pRelResp.runtime': runtime, 'pRelResp.ready': 0, 'pRelResp.activeId': 0,
            'pRelResp.commit': 0})
        p = AsyncParams(replies)
        p.toc = SimpleNamespace(toc={'pRelResp': {k.split('.')[1]: None for k in replies}})
        def write(name, value):
            p.replies[name] = value
            if name == 'pRelResp.commit':
                p.replies['pRelResp.activeId'] = int(value)
                p.replies['pRelResp.ready'] = int(int(value) != 0)
        p.set_value.side_effect = write
        return p

    def test_upload_transaction_readback_and_no_pid_or_flight_writes(self):
        p = self.params()
        result = upload_model(p, model(), timeout_s=.01)
        writes = [c.args for c in p.set_value.call_args_list]
        self.assertEqual(writes[0], ('pRelResp.commit', '0'))
        self.assertEqual(writes[-1], ('pRelResp.commit', str(result['pRelResp.id'])))
        self.assertTrue(all(k.startswith('pRelResp.') for k, _ in writes))
        self.assertEqual(result['pRelResp.ready'], 1)
        p.get_value.assert_not_called()
        self.assertFalse(p.callbacks)

    def test_unsupported_runtime_or_missing_toc_has_no_writes(self):
        for kind in ('runtime', 'missing'):
            p = self.params(runtime=0 if kind == 'runtime' else 1)
            if kind == 'missing': del p.toc.toc['pRelResp']['rWn']
            with self.assertRaises(RuntimeError): upload_model(p, model(), timeout_s=.001)
            p.set_value.assert_not_called()

    def test_partial_or_wrong_readback_never_commits(self):
        p = self.params()
        original = p.request_param_update
        p.request_param_update = lambda name: None if name == 'pRelResp.rWn' else original(name)
        with self.assertRaisesRegex(RuntimeError, 'rWn'): upload_model(p, model(), timeout_s=.001)
        commits = [c.args[1] for c in p.set_value.call_args_list if c.args[0] == 'pRelResp.commit']
        self.assertEqual(commits, ['0', '0'])

    def test_quality_invalid_values_and_model_kind_rejected(self):
        for value in (float('nan'), float('inf'), -.1, .15, True):
            r = report(); r['axes']['roll']['delay_s'] = value
            with self.assertRaises(ValueError): validate_report(r)
        for field, value in (('model', 'motor_tau'), ('usable', False), ('r_squared', .8)):
            r = report(); r['axes']['pitch'][field] = value
            with self.assertRaises(ValueError): validate_report(r)

    def test_save_per_drone_failed_attempt_preserves_but_does_not_activate_old_fit(self):
        with TemporaryDirectory() as root:
            path = Path(root)/'models.json'; log = Path(root)/'log.json'; log.write_text('[]')
            pid = model()['pid_values']
            self.assertTrue(save_report('lb11', report(), pid, source_log=log, path=path)['accepted'])
            self.assertEqual(load_model('lb11', path=path, pid_values=pid)['report'], report())
            before = json.loads(path.read_text())['drones']['lb11']['accepted']
            save_report('lb2', report(), pid, source_log=log, path=path)
            failed = report(); failed['usable'] = False
            save_report('lb11', failed, pid, source_log=log, path=path)
            self.assertEqual(json.loads(path.read_text())['drones']['lb11']['accepted'], before)
            with self.assertRaisesRegex(ValueError, 'latest'): load_model('lb11', path=path, pid_values=pid)
            load_model('lb2', path=path, pid_values=pid)

    def test_pid_change_missing_fit_wrong_drone_and_bad_schema_rejected(self):
        with TemporaryDirectory() as root:
            path = Path(root)/'models.json'; log = Path(root)/'log.json'; log.write_text('[]')
            pid = model()['pid_values']; save_report('lb11', report(), pid, source_log=log, path=path)
            changed = dict(pid); changed['pid_rate.roll_kp'] = 2.
            with self.assertRaisesRegex(ValueError, 'PID'): load_model('lb11', path=path, pid_values=changed)
            with self.assertRaises(ValueError): load_model('lb2', path=path, pid_values=pid)
            original = path.read_bytes(); broken = report(); broken['clock_basis'] = 'host_receive_time_fallback'
            with self.assertRaises(ValueError): validate_report(broken)
            self.assertEqual(path.read_bytes(), original)

    def test_fitting_only_after_successful_calibration_and_persists_rejection(self):
        with TemporaryDirectory() as root:
            path = Path(root)/'models.json'; log = Path(root)/'log.json'; log.write_text('[]')
            with self.assertRaisesRegex(ValueError, 'did not finish'):
                fit_completed_calibration('lb11', log, model()['pid_values'], path=path)
            self.assertFalse(path.exists())
            log.write_text(json.dumps([{'name': 'Wrench Model Calibration Saved'}]))
            with patch('Interaction.calibration_attitude_response.identify_attitude_response_from_log_records',
                       return_value=report()) as fit:
                result = fit_completed_calibration('lb11', log, model()['pid_values'], path=path)
            self.assertTrue(result['accepted'])
            self.assertEqual(fit.call_args.kwargs['attitude_source'], 'ordinary')

    def test_controller_upload_is_before_auto_enable_and_arm_rechecks(self):
        from Interaction.tests.test_scurve_firmware_preflight import SCurvePreflightTests
        ctrl = SCurvePreflightTests().controller()
        ctrl.firmware_response_model_config = {'enabled': True}
        ctrl.args.drone_id = 'lb11'; ctrl.use_flowdeck = False
        ctrl.cfg = SimpleNamespace(PID_VALUES={})
        p = SCurvePreflightTests().params(); p.toc.toc['hlCommander']['pRelAdapt'] = None
        p.toc.toc['hlCommander']['pRelAdVer'] = None
        ctrl.cf = SimpleNamespace(param=p)
        def upload(*args):
            self.assertEqual(p.set_value.call_args.args, ('hlCommander.pRelAdapt', '0'))
            self.assertNotIn(('hlCommander.pRelAuto', '1'), [c.args for c in p.set_value.call_args_list])
            return {'pRelResp.ready': 1, 'pRelResp.id': 42}
        with patch('Interaction.firmware_response_model.confirm_pid_context', return_value={'pid_rate.roll_kp':90.}), \
             patch('Interaction.firmware_response_model.load_model', return_value=model()), \
             patch('Interaction.firmware_response_model.upload_model', side_effect=upload), \
             patch('Interaction.firmware_parameter_confirmation.confirm_firmware_mode_parameters') as identity:
            ctrl._setup_firmware_auto_brake_params()
        identity.assert_called_once_with(p, expected={'hlCommander.pRelSVer':26092304,
                                                     'hlCommander.pRelAdVer':26092303})
        self.assertEqual(p.set_value.call_args.args, ('hlCommander.pRelAuto','1'))
        self.assertEqual(ctrl._firmware_response_expected['hlCommander.pRelAdapt'], 1)

    def test_plain_calibrate_does_not_require_previous_response_or_enable_brake(self):
        from Interaction.tests.test_firmware_auto_brake_preflight import FirmwareAutoBrakePreflightTests
        ctrl = FirmwareAutoBrakePreflightTests().controller({'enabled': True, 'mode':'scurve',
            'response_model': {'enabled': True}})
        ctrl.args.calibrate = True;ctrl.args.interaction = False
        ctrl.prepare_firmware_auto_brake()
        self.assertFalse(ctrl.firmware_auto_brake_enabled)

    def test_wrong_adaptive_runtime_never_uploads_or_enables(self):
        from Interaction.tests.test_scurve_firmware_preflight import SCurvePreflightTests
        from Interaction.firmware_parameter_confirmation import confirm_firmware_mode_parameters
        ctrl = SCurvePreflightTests().controller()
        ctrl.firmware_response_model_config = {'enabled': True}
        ctrl.args.drone_id = 'lb11'; ctrl.use_flowdeck = False
        ctrl.cfg = SimpleNamespace(PID_VALUES={})
        p = SCurvePreflightTests().params()
        p.toc.toc['hlCommander'].update(pRelAdapt=None, pRelAdVer=None)
        p.replies['hlCommander.pRelAdVer'] = '26092302'
        ctrl.cf = SimpleNamespace(param=p)
        def fresh(param, *, expected):
            return confirm_firmware_mode_parameters(param, expected=expected, timeout_s=.01)
        with patch('Interaction.firmware_response_model.confirm_pid_context', return_value={}), \
             patch('Interaction.firmware_response_model.load_model', return_value=model()), \
             patch('Interaction.firmware_response_model.upload_model') as upload, \
             patch('Interaction.firmware_parameter_confirmation.confirm_firmware_mode_parameters', side_effect=fresh):
            with self.assertRaisesRegex(RuntimeError, 'pRelAdVer'):
                ctrl._setup_firmware_auto_brake_params()
        upload.assert_not_called()
        writes = [c.args for c in p.set_value.call_args_list]
        self.assertIn(('hlCommander.pRelAuto', '0'), writes)
        self.assertNotIn(('hlCommander.pRelAuto', '1'), writes)
        self.assertNotIn(('hlCommander.pRelAdapt', '1'), writes)

    def test_parameter_mapping_uses_closed_loop_fields_not_motor_tau(self):
        values = model_parameters(model())
        self.assertEqual(values['pRelResp.rDelay'], .04)
        self.assertEqual(values['pRelResp.pWn'], 13.)
        self.assertFalse(any('Tau' in k or 'pRelTau' in k for k in values))

    def test_arm_is_not_sent_when_fresh_model_verification_fails(self):
        from controller import Controller
        ctrl = Controller.__new__(Controller)
        ctrl.args = SimpleNamespace(ground_test=False, skip_arm=False)
        ctrl.firmware_auto_brake_enabled = True
        ctrl.firmware_auto_brake_mode = 'scurve'
        ctrl._firmware_response_expected = {'pRelResp.ready':1}
        ctrl.verify_contact_attitude_final_prearm_ready = Mock()
        ctrl.verify_firmware_auto_brake_ready = Mock(side_effect=RuntimeError('model changed'))
        ctrl.cf = SimpleNamespace(platform=SimpleNamespace(send_arming_request=Mock()))
        with self.assertRaisesRegex(RuntimeError, 'model changed'): ctrl.arm()
        ctrl.cf.platform.send_arming_request.assert_not_called()


if __name__ == '__main__': unittest.main()
