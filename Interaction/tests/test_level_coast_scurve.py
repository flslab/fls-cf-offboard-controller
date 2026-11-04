import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

from Interaction.level_coast_scurve import ContactEstimatorSelector, FirmwareSCurveCoast
from Interaction.post_release_firmware_control_event import FirmwareReleaseRejectedError
from Interaction.tests.test_firmware_parameter_confirmation import AsyncParams
from Interaction.tests import test_firmware_auto_brake_preflight as preflight
from Interaction.tests import test_scurve_firmware_preflight as scurve_preflight
from Interaction.mission_profiles import resolve_mission_profiles
from Interaction.tests.test_mission_profiles import mission


class ContactEstimatorTests(unittest.TestCase):
    def selector(self, *, switching_enabled=True):
        params = AsyncParams({'stabilizer.estimator': '2', 'stabilizer.controller': '1',
            'kalmanPRel.feedback': '2', 'hlCommander.pRelEnd': '1', 'hlCommander.pRelHoldG': '0'})
        selector = ContactEstimatorSelector(SimpleNamespace(param=params), switching_enabled=switching_enabled)
        selector.prepare()
        return selector, params

    def test_never_switches_or_restores_estimator_even_without_xy_calibration(self):
        selector, params = self.selector(switching_enabled=False)
        selector.cf._estimator_hover_xy = SimpleNamespace(ready=False)
        for contact in (True, False, True):
            selector.request(contact, 1.)
            self.assertTrue(selector.ready(10.))
        selector.close()
        params.set_value.assert_not_called()
        self.assertEqual(params.callbacks, {})

    def test_switch_is_nonblocking_ignores_old_reply_and_requires_confirmation(self):
        selector, params = self.selector()
        selector.request(True, 10.)
        self.assertFalse(selector.ready(10.1))  # Cached default reply is not contact.
        params.callbacks[selector.PARAMETER](selector.PARAMETER, '3')
        self.assertTrue(selector.ready(10.2))
        selector.request(False, 11.)
        self.assertTrue(selector.ready(11.1))
        writes = [call.args for call in params.set_value.call_args_list]
        self.assertEqual(writes, [('stabilizer.estimator', '3'), ('stabilizer.estimator', '2')])
        params.get_value.assert_not_called()
        selector.close()
        self.assertEqual(params.callbacks, {})

    def test_timeout_and_partial_write_failure_still_restore_default(self):
        selector, params = self.selector()
        selector.request(True, 1.)
        with self.assertRaisesRegex(RuntimeError, 'within 0.5 s'):
            selector.ready(1.5)
        selector.close()
        self.assertEqual(params.set_value.call_args_list[-1].args, ('stabilizer.estimator', '2'))
        self.assertEqual(params.callbacks, {})
        selector, params = self.selector()
        params.set_value.side_effect = RuntimeError('write failed')
        with self.assertRaisesRegex(RuntimeError, 'write failed'):
            selector.request(True, 2.)
        params.set_value.side_effect = None
        selector.close()
        self.assertFalse(selector.prepared)

    def test_coast_preemption_replaces_pending_restore_with_contact(self):
        selector, params = self.selector()
        params.replies = None
        selector.request(True, 1.)
        params.callbacks[selector.PARAMETER](selector.PARAMETER, '3')
        selector.request(False, 2.)
        selector.request(True, 2.1)
        params.callbacks[selector.PARAMETER](selector.PARAMETER, '2')
        self.assertFalse(selector.ready(2.2))
        params.callbacks[selector.PARAMETER](selector.PARAMETER, '3')
        self.assertTrue(selector.ready(2.3))
        selector.close()

    def test_hardware_preflight_requires_and_confirms_disabled_speed_gate_before_enable(self):
        raw = mission()
        raw['Interaction']['config']['level_coast']['coast_command_mode'] = 'scurve'
        resolved = resolve_mission_profiles(raw)
        brake = resolved['Interaction']['config']['wrench_interaction']['firmware_auto_brake']
        brake['response_model']['enabled'] = False  # Calibration upload has its own tests.
        profile = brake['analytic_profile']
        profile.update(response_compensation=False, acceleration_residual=False)
        profile.pop('response_bandwidth')
        profile.pop('position_tracking_bandwidth')
        control = preflight.FirmwareAutoBrakePreflightTests().controller(brake)
        control.args.controller_type = 'pid'
        control.mission = resolved
        control.prepare_firmware_auto_brake()
        params = scurve_preflight.SCurvePreflightTests().velocity_params()
        for key, value in control._firmware_analytic_expected.items():
            group, name = key.split('.')
            params.toc.toc.setdefault(group, {})[name] = None
            params.replies[key] = str(value)
        params.toc.toc['hlCommander']['pRelEnd'] = None
        params.replies['hlCommander.pRelEnd'] = '1'
        control.cf = SimpleNamespace(param=params)
        with self.assertRaisesRegex(RuntimeError, 'pRelHoldG'):
            control._setup_firmware_auto_brake_params()
        params.set_value.assert_not_called()
        params.toc.toc['hlCommander']['pRelHoldG'] = None
        params.replies['hlCommander.pRelHoldG'] = '0'
        control._setup_firmware_auto_brake_params()
        self.assertIn('hlCommander.pRelHoldG', params.requested)
        writes = [call.args for call in params.set_value.call_args_list]
        self.assertIn(('hlCommander.pRelHoldG', '0'), writes)
        self.assertEqual(writes[-1], ('hlCommander.pRelAuto', '1'))


class FirmwareSCurveAdmissionTests(unittest.TestCase):
    def attempt(self, reason, detail, *, ack_errno=34, fresh=True):
        packet = {'hlCommander.pRelRejR': reason, 'hlCommander.pRelRejD': detail}
        owner = SimpleNamespace(
            cf=SimpleNamespace(), mission={'Interaction': {'config': {
                'wrench_interaction': {'firmware_auto_brake': {}}}}},
            lo_commander=SimpleNamespace(send_zdistance_setpoint=Mock()),
            _firmware_brake_status_snapshot=lambda: ({}, 1.),
            _capture_firmware_release_rejection=lambda receipt, **kwargs: (
                packet, {'fresh_post_release_packet': fresh}),
            _log_event=Mock())
        curve = FirmwareSCurveCoast(owner, SimpleNamespace())
        completion = MagicMock()
        with patch('Interaction.level_coast_scurve.FirmwareHoldNotification', return_value=completion), \
                patch('Interaction.level_coast_scurve.handoff_pi_release_to_firmware',
                      side_effect=FirmwareReleaseRejectedError(ack_errno)):
            if fresh and ack_errno == 34 and (reason, detail) in ((7, 4), (4, 1)):
                self.assertIs(curve.begin(height=1., release_monotonic_s=1.), False)
            else:
                with self.assertRaises(FirmwareReleaseRejectedError):
                    curve.begin(height=1., release_monotonic_s=1.)
        self.assertFalse(curve.active)
        self.assertIsNone(curve.completion)
        self.assertIsNone(curve.hold_notice)
        self.assertEqual(curve.sequence, 1)
        completion.__exit__.assert_called_once()
        completion.acknowledge.assert_not_called()

    def test_only_explicit_fresh_low_speed_rejections_skip_curve(self):
        for reason, detail in ((7, 4), (4, 1)):
            with self.subTest(reason=reason, detail=detail):
                self.attempt(reason, detail)

    def test_other_or_combined_failures_remain_errors(self):
        for reason, detail in ((7, 6), (7, 12), (7, 20), (4, 0), (6, 1), (3, 1), (10, 4)):
            with self.subTest(reason=reason, detail=detail):
                self.attempt(reason, detail)
        self.attempt(7, 4, fresh=False)
        self.attempt(7, 4, ack_errno=11)


if __name__ == '__main__':
    unittest.main()
