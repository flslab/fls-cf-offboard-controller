"""Host phase timing without loading a camera SDK or touching flight hardware."""
import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


class Clock:
    def __init__(self):
        self.now = 10.0

    def monotonic(self):
        return self.now

    def time(self):
        return 1000.0+self.now

    def advance(self, duration):
        self.now += duration


def load_mocap():
    spec = importlib.util.spec_from_file_location('mocap_timing_under_test', ROOT/'mocap.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict('sys.modules', {'motioncapture': SimpleNamespace(connect=Mock())}):
        spec.loader.exec_module(module)
    return module


class MocapTimingTests(unittest.TestCase):
    def test_wait_processing_callback_and_unmatched_frames_are_distinct(self):
        module = load_mocap()
        clock, frames, diagnostics = Clock(), [], []
        capture = SimpleNamespace(pointCloud=np.array([[0., 0., 0.]]))
        waits = iter([.01, .08, .002])
        mocap = module.Mocap(mode='pointcloud', timing_callback=diagnostics.append)

        def wait():
            clock.advance(next(waits))
            # Last loop receives a frame but no tracked marker.
            if len(diagnostics) == 2:
                capture.pointCloud = np.empty((0, 3))
                mocap.running = False

        def callback(frame):
            frames.append(frame)
            clock.advance(.005)

        capture.waitForNextFrame = wait
        mocap.subscribe_point([0, 0, 0], callback)
        with patch.object(module, 'time', clock), patch.object(
                module.motioncapture, 'connect', return_value=capture):
            mocap.run()
        self.assertEqual(len(frames), 2)
        self.assertEqual(len(diagnostics), 3)
        np.testing.assert_allclose([d['wait_duration_s'] for d in diagnostics], [.01, .08, .002])
        np.testing.assert_allclose([d['callback_duration_s'] for d in diagnostics], [.005, .005, 0])
        self.assertEqual([d['callback_count'] for d in diagnostics], [1, 1, 0])
        self.assertAlmostEqual(diagnostics[1]['local_loop_interval_s'], .085)
        self.assertEqual([d['frame_id'] for d in diagnostics], [0, 1, 2])
        for frame, diagnostic in zip(frames, diagnostics):
            self.assertEqual(frame['time'], diagnostic['time'])
            self.assertEqual(frame['mocap_timing']['frame_id_scope'], 'local_loop')
            self.assertFalse(frame['mocap_timing']['source_capture_time_available'])
            self.assertEqual(frame['mocap_timing']['frame_time_scope'], 'host_after_wait')
            self.assertAlmostEqual(diagnostic['processing_excluding_callbacks_s'], 0)
        self.assertAlmostEqual(frames[0]['time'], 1010.01)

    def test_diagnostic_callback_cost_is_in_next_between_loop_field(self):
        module, clock = load_mocap(), Clock()
        diagnostics = []
        mocap = module.Mocap(mode='rigidbody')

        def diagnostic(row):
            diagnostics.append(row)
            clock.advance(.003)
            if len(diagnostics) == 2:
                mocap.running = False

        mocap.timing_callback = diagnostic
        capture = SimpleNamespace(rigidBodies={}, waitForNextFrame=lambda: clock.advance(.01))
        with patch.object(module, 'time', clock), patch.object(
                module.motioncapture, 'connect', return_value=capture):
            mocap.run()
        self.assertAlmostEqual(diagnostics[1]['between_processing_and_wait_s'], .003)
        self.assertAlmostEqual(diagnostics[1]['local_loop_interval_s'], .013)

    def test_disabled_diagnostics_preserve_callback_data(self):
        mocap = load_mocap().Mocap()
        callback = Mock()
        frame = {'time': 123, 'tvec': [1, 2, 3]}
        mocap._emit_frame(callback, frame)
        callback.assert_called_once_with(frame)
        self.assertNotIn('mocap_timing', frame)

    def test_callback_exception_is_not_hidden_and_duration_is_counted(self):
        module, clock = load_mocap(), Clock()
        mocap = module.Mocap()
        mocap._frame_timing = dict(wait_return_monotonic_s=10., wait_duration_s=.01,
                                  callback_count=0, callback_duration_s=0., max_callback_duration_s=0.)

        def callback(_):
            clock.advance(.02)
            raise RuntimeError('callback failed')

        with patch.object(module, 'time', clock), self.assertRaisesRegex(RuntimeError, 'callback failed'):
            mocap._emit_frame(callback, {'time': 100})
        self.assertEqual(mocap._frame_timing['callback_count'], 1)
        self.assertAlmostEqual(mocap._frame_timing['callback_duration_s'], .02)


class ForwardTimingTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        source = ast.parse((ROOT/'controller.py').read_text())
        cls = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'Controller')
        names = {'_send_position', '_send_position_orientation', '_prepare_mocap_forward_timing',
                 '_finish_mocap_forward_timing', '_log_mocap', '_log_mocap_timing'}
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
        namespace = {'time': self.clock}
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(ROOT/'controller.py'), 'exec'), namespace)
        stub_class = type('ControllerStub', (), {name: namespace[name] for name in names})
        self.controller = stub_class()
        self.controller.send_vicon_to_cf = True
        self.controller.log_manager = Mock()
        self.controller.log_manager.live_logger = None
        self.controller.cf = SimpleNamespace(extpos=Mock())
        self.controller.cf.extpos.send_extpos.side_effect = lambda *args: self.clock.advance(.003)
        self.controller.cf.extpos.send_extpose.side_effect = lambda *args: self.clock.advance(.002)
        self.frame = dict(time=42., tvec=[1, 2, 3], quat=[0, 0, 0, 1],
                          mocap_timing={'wait_return_monotonic_s': 9.999})

    def last_logged_frame(self):
        return self.controller.log_manager.add_log_entry.call_args.args[1]

    def test_forwarding_and_frame_timestamp_unchanged(self):
        self.controller._send_position(self.frame)
        self.controller.cf.extpos.send_extpos.assert_called_once_with(1, 2, 3)
        logged = self.last_logged_frame()
        self.assertEqual(logged['time'], 42.)
        self.assertEqual(self.frame['mocap_timing'], {'wait_return_monotonic_s': 9.999})
        timing = logged['mocap_timing']
        self.assertTrue(timing['extpos_send_called'])
        self.assertAlmostEqual(timing['extpos_send_duration_s'], .003)
        self.assertAlmostEqual(timing['wait_return_to_send_s'], .001)
        self.assertIsNone(timing['extpos_send_interval_s'])
        self.clock.advance(.007)
        self.controller._send_position(self.frame)
        self.assertAlmostEqual(self.last_logged_frame()['mocap_timing']['extpos_send_interval_s'], .01)

    def test_disabled_position_forward_still_logs_without_send(self):
        self.controller.send_vicon_to_cf = False
        self.controller._send_position(self.frame)
        self.controller.cf.extpos.send_extpos.assert_not_called()
        self.assertFalse(self.last_logged_frame()['mocap_timing']['extpos_send_called'])
        self.assertNotIn('extpos_send_duration_s', self.last_logged_frame()['mocap_timing'])

    def test_full_pose_preserves_existing_forwarding_semantics(self):
        self.controller.send_vicon_to_cf = False  # Full pose has always sent regardless.
        self.controller._send_position_orientation(self.frame)
        self.controller.cf.extpos.send_extpose.assert_called_once_with(1, 2, 3, 0, 0, 0, 1)
        self.assertAlmostEqual(self.last_logged_frame()['mocap_timing']['extpos_send_duration_s'], .002)

    def test_legacy_frame_does_not_claim_capture_age(self):
        del self.frame['mocap_timing']
        self.controller._send_position(self.frame)
        self.assertIsNone(self.last_logged_frame()['mocap_timing']['wait_return_to_send_s'])
        self.controller._log_mocap_timing({'frame_id': 1})
        self.controller.log_manager.add_log_entry.assert_called_with('mocap_timing', {'frame_id': 1})

    def test_queue_health_is_sampled_without_flushing(self):
        live_logger = Mock()
        live_logger.stats_snapshot.return_value = {'queued_records': 5}
        self.controller.log_manager.live_logger = live_logger
        row = {'frame_id': 1}
        self.controller._log_mocap_timing(row)
        self.assertEqual(self.last_logged_frame()['logger_queue'], {'queued_records': 5})
        self.assertEqual(row, {'frame_id': 1})
        self.clock.advance(.5)
        self.controller._log_mocap_timing({'frame_id': 2})
        self.assertNotIn('logger_queue', self.last_logged_frame())
        self.clock.advance(.51)
        self.controller._log_mocap_timing({'frame_id': 3})
        self.assertEqual(live_logger.stats_snapshot.call_count, 2)
        live_logger.flush_to_disk.assert_not_called()


if __name__ == '__main__':
    unittest.main()
