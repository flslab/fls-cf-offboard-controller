"""No-hardware checks for the isolated repeat-test entry point."""

import ast
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from Interaction.braking_repeat_test import (
    calibration_reference, repeat_test_config, repeat_test_result,
    validate_repeat_test_options,
)
from Interaction.braking_response_calibration import PlanarBrakingCalibration
from Interaction.interactions import InteractionsControl, StaleLocalizationError
from Interaction.tests import test_calibration_trial_wait_runtime as wait_fixture


class BrakingRepeatTestTests(unittest.TestCase):
    def test_fixed_protocol_preserves_limits_and_input_config(self):
        original = {
            'calibration_excitation': {'enabled': True, 'duration_s': 30},
            'planar_braking_calibration': {
                'tilt_levels_deg': [8, 14, 20], 'accelerate_durations_s': [.16, .24, .32],
                'max_xy_speed_m_s': 1.6, 'max_displacement_m': 1.0,
                'trial_start_dwell_s': .5,
            },
            'guided_touch_test': {'enabled': True},
            'position_capture_calibration': {'enabled': True},
            'safety': {'max_state_age_s': .1},
        }
        before = deepcopy(original)
        config = repeat_test_config(original)
        self.assertEqual(original, before)
        self.assertFalse(config['calibration_excitation']['enabled'])
        self.assertFalse(config['guided_touch_test']['enabled'])
        self.assertNotIn('position_capture_calibration', config)
        self.assertEqual(config['safety'], original['safety'])
        plan = PlanarBrakingCalibration(config['planar_braking_calibration'])
        np.testing.assert_equal(plan.trial_directions, [[0, -1], [0, 1]] * 3)
        np.testing.assert_equal(plan.trial_accelerate_s, [.24] * 6)
        np.testing.assert_equal(plan.trial_brake_s, [.24] * 6)
        np.testing.assert_equal(plan.trial_tilt_levels_deg, [20] * 6)
        self.assertAlmostEqual(plan.end_s, 22.18)
        self.assertEqual(plan.max_xy_speed_m_s, 1.6)
        self.assertEqual(plan.max_displacement_m, 1.0)
        self.assertEqual(config['planar_braking_calibration']['trial_start_dwell_s'], .5)

    def test_cli_requires_exclusive_logged_high_rate_mode(self):
        args = SimpleNamespace(braking_test=True, log=True, smooth_controller_rate=100, cf_log_period=10)
        validate_repeat_test_options(args)
        for field, value in [('sense', True), ('interaction', True), ('calibrate', True),
                             ('illumination', True), ('trajectory', 'test.json'),
                             ('autotune', True), ('log', False),
                             ('smooth_controller_rate', 30), ('cf_log_period', 50),
                             ('cf_log_period', 0), ('cf_log_period', -10)]:
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate_repeat_test_options(SimpleNamespace(**(vars(args) | {field: value})))

    def test_controller_dispatch_and_logging_include_test_mode(self):
        source = ast.parse(Path('controller.py').read_text())
        node = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'Controller')
        selected = [n for n in node.body if isinstance(n, ast.FunctionDef)
                    and n.name in ('_is_interaction_application', 'run_mission', 'calibration_switch')]
        namespace = {'InteractionsControl': Mock(), 'logging': Mock(), 'traceback': Mock()}
        exec(compile(ast.Module(body=selected, type_ignores=[]), 'controller.py', 'exec'), namespace)
        controller = SimpleNamespace(args=SimpleNamespace(
            braking_test=True, autotune=False, simple_takeoff=False, rotation_test=False,
            xy_tune=False, z_tune=False, trajectory=None, orchestrated=True,
            ground_test=False, smooth_controller_rate=100, drone_id='test', sense_axis='y',
            sense_sign=1, sense_max_age=.25), cf=Mock(), log_manager=Mock(), mission={},
            manifest=None, force_sensor=None, _safe_sleep=Mock(), ll_commander=Mock())
        controller.calibration_switch = lambda: namespace['calibration_switch'](controller)
        self.assertTrue(namespace['_is_interaction_application'](controller))
        namespace['run_mission'](controller)
        control = namespace['InteractionsControl'].return_value
        control.run_braking_test.assert_called_once_with()
        control.run_calibration.assert_not_called()

    def make_runtime(self, behavior=None):
        fixture = wait_fixture.CalibrationTrialWaitRuntimeTests()
        runtime = fixture.make_runtime(wait_behavior=behavior)
        controller, logs, clock, times, config, _duration = runtime
        config = repeat_test_config(config)
        plan = PlanarBrakingCalibration(config['planar_braking_calibration'])
        return controller, logs, clock, times, config, plan.end_s + .5

    def test_complete_six_trials_without_excitation_fitting_or_file_changes(self):
        controller, logs, clock, _times, config, duration = self.make_runtime()
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            path = Path(directory) / 'calibration.json'
            path.write_bytes(b'{"previous": "keep exactly"}\n')
            before = calibration_reference(path)
            stack.enter_context(patch('Interaction.interactions.time.time', lambda: clock[0]))
            forbidden = [stack.enter_context(patch('Interaction.interactions.' + name,
                         side_effect=AssertionError(name + ' must not run')))
                         for name in ('identify_xyz_alignment', 'identify_planar_braking_response',
                                      'save_drone_calibration', 'apply_drone_calibration')]
            controller.interaction_onboard_wrench_admittance(
                duration=duration, nominal_position=[0, 0, 1], config=config,
                calibration_mode=True, braking_test_mode=True, calibration_path=path)
            self.assertEqual(calibration_reference(path), before)
            for call in forbidden:
                call.assert_not_called()
        events = wait_fixture.CalibrationTrialWaitRuntimeTests.events
        completed = events(logs, 'Planar Braking Repeat Test Complete')
        self.assertEqual(len(completed), 1)
        self.assertEqual(completed[0]['maneuver_count'], 6)
        self.assertTrue(completed[0]['previous_calibration_preserved'])
        self.assertFalse(completed[0]['model_fitted'])
        self.assertGreater(completed[0]['sample_count'], 300)
        self.assertFalse(events(logs, 'Wrench Calibration Excitation Started'))
        self.assertFalse(events(logs, 'Wrench Model Calibration Saved'))
        ready = events(logs, 'Planar Braking Calibration Trial Ready')
        self.assertEqual([r['segment_id'] for r in ready], list(range(6)))
        phases = events(logs, 'Planar Braking Calibration Phase')
        expected = ['level_before_acceleration', 'accelerate', 'level_before_brake',
                    'brake', 'level_after_brake', 'recovery']
        for segment in range(6):
            self.assertEqual([r['phase'] for r in phases if r['segment_id'] == segment], expected)
        self.assertLess(ready[0]['time'] - 1000., 3.)  # No old 30-second XYZ wait.

    def test_stale_attitude_aborts_without_completion_or_save(self):
        def stale(state, _time, _logs, controller):
            if controller.lo_commander.calls and controller.lo_commander.calls[-1][0] == 'zdistance':
                state['time'] -= .2

        controller, logs, clock, _times, config, duration = self.make_runtime(stale)
        with patch('Interaction.interactions.time.time', lambda: clock[0]), \
                patch('Interaction.interactions.save_drone_calibration') as save:
            with self.assertRaises(StaleLocalizationError):
                controller.interaction_onboard_wrench_admittance(
                    duration=duration, nominal_position=[0, 0, 1], config=config,
                    calibration_mode=True, braking_test_mode=True,
                    calibration_path='/tmp/nonexistent-braking-test-calibration.json')
            save.assert_not_called()
        self.assertFalse(wait_fixture.CalibrationTrialWaitRuntimeTests.events(logs, 'Planar Braking Repeat Test Complete'))
        self.assertEqual(controller.lo_commander.calls[-1], ('zdistance', (0., 0., 0., 1.), {}))

    def test_missing_samples_cannot_be_reported_complete(self):
        config = repeat_test_config({})['planar_braking_calibration']
        plan = PlanarBrakingCalibration(config)
        with self.assertRaisesRegex(ValueError, 'missing trial samples'):
            repeat_test_result(plan, config, [], {'path': 'none', 'sha256': None})

    def test_entry_point_uses_calibration_target_and_preserves_mission(self):
        controller, _logs, _clock, _times, config, _duration = self.make_runtime()
        config['calibration_nominal_position'] = [0., 0., 1.]
        controller.mission = {
            'drones': {'test': {'target': [0., -1., 1., 0.]}},
            'Interaction': {'action': 'translation', 'config': {
                'detection_method': 'momentum_impulse', 'duration': 60,
                'wrench_interaction': config,
            }},
        }
        before = deepcopy(controller.mission)
        controller.interaction_onboard_wrench_admittance = Mock()
        controller.run_braking_test()
        kwargs = controller.interaction_onboard_wrench_admittance.call_args.kwargs
        self.assertTrue(kwargs['braking_test_mode'])
        self.assertTrue(kwargs['calibration_mode'])
        self.assertEqual(kwargs['nominal_position'], [0., 0., 1.])
        self.assertLess(kwargs['duration'], 24.)
        self.assertEqual(controller.mission, before)


if __name__ == '__main__':
    unittest.main()
