"""Firmware interaction host diagnostics must use actual Vicon frame intervals."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from Interaction.Kalman_Filter import VelocityKalmanFilter
from Interaction.log_manager import InteractionLogger


class FirmwareMocapVelocityTimingTests(unittest.TestCase):
    def make_logger(self, elapsed):
        logger = InteractionLogger.__new__(InteractionLogger)
        logger.args = SimpleNamespace(tracker_camera_rate=120)
        logger.groups = {}
        logger.group_kfs = {}
        logger.live_logger = None
        logger.add_log_group('frames', kf=True,
                             kf_use_mocap_elapsed_dt=elapsed)
        return logger

    def frame(self, timestamp, x):
        return {'time': 1234.0 + x, 'frame_id': 9, 'tvec': [x, 0, 1],
                'mocap_timing': {'wait_return_monotonic_s': timestamp}}

    def test_elapsed_interval_corrects_100hz_data_with_120hz_nominal_setting(self):
        elapsed = self.make_logger(True)
        legacy = self.make_logger(False)
        for i in range(300):
            for logger in (elapsed, legacy):
                logger.add_log_entry('frames', self.frame(10 + i * .01, i * .01))
        self.assertAlmostEqual(elapsed.groups['frames'][-1]['vel'][0], 1.0, places=5)
        self.assertAlmostEqual(legacy.groups['frames'][-1]['vel'][0], 1.2, places=5)
        self.assertNotIn('velocity_kf_timing', legacy.groups['frames'][-1])

    def test_variable_dt_updates_process_covariance_and_keeps_gap_history(self):
        logger = self.make_logger(True)
        logger.add_log_entry('frames', self.frame(10.0, 0.0))
        identities = [id(kf) for kf in logger.group_kfs['frames'].values()]
        for t in (10.01, 10.02, 10.05, 10.35):
            logger.add_log_entry('frames', self.frame(t, t-10.0))
        self.assertEqual(identities, [id(kf) for kf in logger.group_kfs['frames'].values()])
        axis = logger.group_kfs['frames']['x']
        self.assertAlmostEqual(axis.dt, .3)
        np.testing.assert_allclose(axis.Q, np.outer([.045, .3], [.045, .3]))
        self.assertAlmostEqual(axis.x[1, 0], 1.0, places=3)

    def test_invalid_timestamp_does_not_reset_history_or_change_logged_epoch(self):
        logger = self.make_logger(True)
        logger.add_log_entry('frames', self.frame(10, 0))
        logger.add_log_entry('frames', self.frame(10.01, .01))
        before = logger.group_kfs['frames']['x'].x.copy()
        for stamp in (None, float('nan'), 10.01, 9.9):
            entry = self.frame(stamp, 99)
            logger.add_log_entry('frames', entry)
            np.testing.assert_array_equal(logger.group_kfs['frames']['x'].x, before)
            self.assertFalse(entry['velocity_kf_timing']['update_applied'])
            self.assertEqual(entry['time'], 1333.0)
            self.assertEqual(entry['frame_id'], 9)
        logger.add_log_entry('frames', self.frame(10.04, .04))
        self.assertAlmostEqual(logger.group_kfs['frames']['x'].dt, .03)

    def test_default_filter_step_keeps_legacy_matrices_and_rejects_invalid_opt_in_dt(self):
        kf = VelocityKalmanFilter(.01, process_noise=3)
        f, q = kf.F.copy(), kf.Q.copy()
        kf.update(.01)
        np.testing.assert_array_equal(kf.F, f)
        np.testing.assert_array_equal(kf.Q, q)
        for dt in (0, -.01, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                kf.update(.02, dt=dt)

    def test_only_firmware_interaction_setup_opts_in(self):
        # Execute the production setup method without opening devices/logs.
        root = Path(__file__).resolve().parents[2]
        tree = ast.parse((root/'controller.py').read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Controller')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'setup_logging')
        namespace = {'logger': Mock()}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(root/'controller.py'), 'exec'), namespace)
        for enabled in (False, True):
            fake = SimpleNamespace(
                args=SimpleNamespace(log=True, illumination=False, hover=False, droneless=True),
                firmware_auto_brake_enabled=enabled,
                _is_interaction_application=lambda: True,
                _uses_onboard_wrench_state=lambda: True,
                _uses_vicon_velocity_for_free_stop=lambda: True,
            )
            sink = Mock()
            with patch('Interaction.log_manager.InteractionLogger', return_value=sink):
                namespace['setup_logging'](fake)
            frames_call = next(c for c in sink.add_log_group.call_args_list if c.args[0]=='frames')
            self.assertEqual(frames_call.kwargs,
                             {'kf': True, 'kf_use_mocap_elapsed_dt': enabled})


if __name__ == '__main__':
    unittest.main()
