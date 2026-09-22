import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from Interaction.firmware_parameter_confirmation import confirm_firmware_mode_parameters


class AsyncParams:
    def __init__(self, replies=None):
        self.callbacks = {}
        self.requested = []
        self.replies = replies
        self.requested_both = threading.Event()
        self.get_value = Mock(return_value='1')
        self.set_value = Mock()

    def add_update_callback(self, group, name, cb):
        self.callbacks[group+'.'+name] = cb

    def remove_update_callback(self, group, name, cb):
        del self.callbacks[group+'.'+name]

    def request_param_update(self, name):
        self.requested.append(name)
        if self.replies is not None and name in self.replies:
            self.callbacks[name](name, self.replies[name])
        if len(self.requested) == 2:
            self.requested_both.set()


class ConfirmationTests(unittest.TestCase):
    def test_delayed_write_replies_do_not_fail_on_initial_zero_cache(self):
        param = AsyncParams()
        param.get_value.return_value = '0'
        results, errors, done = [], [], threading.Event()
        def run():
            try:
                results.append(confirm_firmware_mode_parameters(param, timeout_s=1.))
            except Exception as exc:
                errors.append(exc)
            finally:
                done.set()
        worker = threading.Thread(target=run)
        worker.start()
        try:
            self.assertTrue(param.requested_both.wait(.5))
            for name in param.requested:
                param.callbacks[name](name, '0')
            self.assertFalse(done.is_set())
            param.callbacks[param.requested[0]](param.requested[0], '1')
            self.assertFalse(done.is_set())
            param.callbacks[param.requested[1]](param.requested[1], '1')
            self.assertTrue(done.wait(.5))
            self.assertEqual(errors, [])
            self.assertEqual(set(results[0].values()), {1})
        finally:
            worker.join(1.1)
        self.assertEqual(param.callbacks, {})
        param.get_value.assert_not_called()
        param.set_value.assert_not_called()

    def test_cached_one_without_fresh_reply_must_timeout(self):
        param = AsyncParams()
        with self.assertRaisesRegex(RuntimeError, 'no fresh reply'):
            confirm_firmware_mode_parameters(param, timeout_s=.01)
        self.assertEqual(param.callbacks, {})
        param.get_value.assert_not_called()

    def test_wrong_and_invalid_values_do_not_authorize(self):
        for value in ('0', '2', 'nan', None):
            param = AsyncParams({'hlCommander.pRelJoint': value,
                                 'hlCommander.pRelHost': '1'})
            with self.assertRaisesRegex(RuntimeError, 'pRelJoint'):
                confirm_firmware_mode_parameters(param, timeout_s=.001)
            self.assertEqual(param.callbacks, {})

    def test_immediate_callbacks_and_repeated_prearm_checks(self):
        param = AsyncParams({'hlCommander.pRelJoint':'1','hlCommander.pRelHost':'1'})
        for _ in range(2):
            self.assertEqual(confirm_firmware_mode_parameters(param),
                             {'hlCommander.pRelJoint':1,'hlCommander.pRelHost':1})
        self.assertEqual(len(param.requested),4)
        self.assertEqual(param.callbacks,{})

    def test_missing_host_and_request_error_cleanup(self):
        param = AsyncParams({'hlCommander.pRelJoint':'1'})
        with self.assertRaisesRegex(RuntimeError, 'pRelHost'):
            confirm_firmware_mode_parameters(param, timeout_s=.001)
        self.assertEqual(param.callbacks,{})
        param.request_param_update = Mock(side_effect=OSError('link lost'))
        with self.assertRaisesRegex(OSError, 'link lost'):
            confirm_firmware_mode_parameters(param)
        self.assertEqual(param.callbacks,{})

    def test_invalid_timeout_rejected(self):
        for value in (0., -1., float('inf'), float('nan')):
            param = AsyncParams()
            with self.assertRaises(ValueError):
                confirm_firmware_mode_parameters(param, timeout_s=value)
            self.assertEqual(param.callbacks,{})

    def test_prearm_still_requires_observer_and_healthy_worker(self):
        from controller import Controller
        ctrl = Controller.__new__(Controller)
        ctrl.firmware_auto_brake_enabled = True
        ctrl.firmware_auto_brake_mode = 'pi_joint'
        ctrl.firmware_auto_brake_response_time_s = .14
        planner = Mock(); planner.status.return_value = {'ready': True}
        ctrl.cf = SimpleNamespace(param=AsyncParams(
            {'hlCommander.pRelJoint':'1','hlCommander.pRelHost':'1'}),
            _post_release_pi_planner=planner)
        ctrl.log_manager = SimpleNamespace(get_latest_group_log_data=lambda _: {})
        ctrl._firmware_vicon_last_send_s = None
        ctrl._firmware_vicon_mirror_error = None
        with patch('controller.time.monotonic', side_effect=(0., 0., 0., 0., 6.)):
            with patch('controller.time.sleep'):
                with self.assertRaisesRegex(RuntimeError, 'brake not ready'):
                    ctrl.verify_firmware_auto_brake_ready()
        planner.status.side_effect = [{'ready':True},{'ready':False}]
        with self.assertRaisesRegex(RuntimeError, 'stopped during pre-arm'):
            ctrl.verify_firmware_auto_brake_ready()


if __name__ == '__main__':
    unittest.main()
