"""Reset-policy integration checks using fresh parameter replies, without radio."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from Interaction.firmware_pid_reset import reset_parameters
from Interaction.firmware_parameter_confirmation import confirm_firmware_mode_parameters
from Interaction.tests import test_scurve_firmware_preflight as scurve
from Interaction.tests import test_firmware_auto_brake_preflight as baseline


class ResetPolicyTests(unittest.TestCase):
    keys = ('pidCtl.resetMode', 'hlCommander.pRelRstXY')

    def fixture(self, **config):
        helper = scurve.SCurvePreflightTests()
        ctrl = helper.velocity_controller(**config)
        param = helper.velocity_params()
        for key in self.keys:
            group, name = key.split('.')
            param.toc.toc.setdefault(group, {})[name] = None
            param.replies[key] = '0'  # An earlier experiment disabled reset.
        param.set_value.side_effect = lambda key, value: param.replies.update({key: value})
        ctrl.cf = SimpleNamespace(param=param)
        return ctrl, param

    def test_defaults_restore_enabled_policy_before_enable(self):
        ctrl, param = self.fixture()
        ctrl._setup_firmware_auto_brake_params()
        writes = [c.args for c in param.set_value.call_args_list]
        self.assertEqual(writes[0], ('hlCommander.pRelAuto', '0'))
        for key in self.keys:
            self.assertEqual(param.replies[key], '1')
            self.assertIn(key, param.requested)
        self.assertEqual(writes[-1], ('hlCommander.pRelAuto', '1'))

    def test_switches_are_independent(self):
        for mode, curve in ((False, False), (False, True), (True, False), (True, True)):
            with self.subTest(mode=mode, curve=curve):
                ctrl, param = self.fixture(reset_on_mode_change=mode,
                    reset_xy_on_scurve_transition=curve)
                ctrl._setup_firmware_auto_brake_params()
                self.assertEqual(ctrl._firmware_reset_expected,
                    dict(zip(self.keys, (int(mode), int(curve)))))
                for key, value in ctrl._firmware_reset_expected.items():
                    self.assertEqual(param.replies[key], str(value))

    def test_disable_on_old_firmware_fails_before_writes(self):
        for option, key in zip(('reset_on_mode_change', 'reset_xy_on_scurve_transition'), self.keys):
            ctrl, param = self.fixture(**{option: False})
            group, name = key.split('.')
            del param.toc.toc[group][name]
            with self.assertRaisesRegex(RuntimeError, key):
                ctrl._setup_firmware_auto_brake_params()
            param.set_value.assert_not_called()

    def test_legacy_firmware_defaults_need_no_new_parameters(self):
        self.assertEqual(reset_parameters({}, {}), {})
        helper = scurve.SCurvePreflightTests()
        ctrl = helper.velocity_controller()
        param = helper.velocity_params()
        ctrl.cf = SimpleNamespace(param=param)
        ctrl._setup_firmware_auto_brake_params()
        self.assertEqual(ctrl._firmware_reset_expected, {})

    def test_invalid_config_rejected_during_prepare(self):
        for option in ('reset_on_mode_change', 'reset_xy_on_scurve_transition'):
            for value in (0, 1, 'false', None):
                with self.subTest(option=option, value=value):
                    with self.assertRaisesRegex(ValueError, option):
                        scurve.SCurvePreflightTests().velocity_controller(**{option: value})

    def test_wrong_readback_cannot_enable_or_pass_prearm(self):
        def quick(param, *, expected):
            return confirm_firmware_mode_parameters(param, expected=expected, timeout_s=.001)
        for key in self.keys:
            ctrl, param = self.fixture(reset_on_mode_change=False,
                reset_xy_on_scurve_transition=False)
            param.replies[key] = '1'
            param.set_value.side_effect = lambda k, v: param.replies.update({k: v}) if k != key else None
            with patch('Interaction.firmware_parameter_confirmation.confirm_firmware_mode_parameters', side_effect=quick):
                with self.assertRaisesRegex(RuntimeError, key):
                    ctrl._setup_firmware_auto_brake_params()
                self.assertNotIn(('hlCommander.pRelAuto', '1'),
                    [c.args for c in param.set_value.call_args_list])
                # Readback is repeated before arm, not trusted from startup.
                param.replies.update(dict.fromkeys(self.keys, '0'))
                param.set_value.side_effect = lambda k, v: param.replies.update({k: v})
                ctrl._setup_firmware_auto_brake_params()
                param.replies[key] = '1'
                with self.assertRaisesRegex(RuntimeError, key):
                    ctrl.verify_firmware_auto_brake_ready()

    def test_disabled_auto_brake_does_not_activate_reset_policy(self):
        ctrl = baseline.FirmwareAutoBrakePreflightTests().controller(dict(
            enabled=False, reset_on_mode_change=False, reset_xy_on_scurve_transition=False))
        ctrl.prepare_firmware_auto_brake()
        self.assertFalse(ctrl.firmware_auto_brake_enabled)
        self.assertFalse(hasattr(ctrl, '_firmware_reset_config'))


if __name__ == '__main__':
    unittest.main()
