"""Exercise adaptive-default CLI and mission scoping without hardware imports."""
import argparse
import ast
from contextlib import redirect_stderr
from copy import deepcopy
import datetime
import io
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from Interaction.braking_repeat_test import validate_repeat_test_options


SOURCE = Path(__file__).resolve().parents[2] / 'controller.py'


def parse_controller_args(tokens):
    tree = ast.parse(SOURCE.read_text())
    main = next(node for node in tree.body if isinstance(node, ast.If)
                and isinstance(node.test, ast.Compare))
    statements = []
    for node in main.body:
        if isinstance(node, ast.With):
            break  # Never create the hardware-owning Controller.
        statements.append(node)
    namespace = {'argparse': argparse, 'datetime': datetime,
                 'validate_repeat_test_options': validate_repeat_test_options}
    with patch('sys.argv', ['controller.py', *tokens]):
        exec(compile(ast.Module(body=statements, type_ignores=[]), str(SOURCE), 'exec'), namespace)
    return namespace['args']


def calibration_switch():
    tree = ast.parse(SOURCE.read_text())
    controller = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                      and node.name == 'Controller')
    method = next(node for node in controller.body if isinstance(node, ast.FunctionDef)
                  and node.name == 'calibration_switch')
    namespace = {'InteractionsControl': Mock(), 'deepcopy': deepcopy,
                 'logging': Mock(), 'traceback': Mock()}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), 'exec'), namespace)
    return namespace['calibration_switch'], namespace['InteractionsControl']


class AdaptiveCalibrationCliTests(unittest.TestCase):
    def args(self, **changes):
        values = dict(calibrate=True, adaptive_braking_calibration=None,
                      interaction=False, braking_test=False, ground_test=False,
                      smooth_controller_rate=100, drone_id='lb11', sense_axis='y',
                      sense_sign=1, sense_max_age=.25)
        values.update(changes)
        return SimpleNamespace(**values)

    def controller(self, args, mission):
        return SimpleNamespace(args=args, mission=mission, cf=Mock(), log_manager=Mock(),
                               manifest=None, force_sensor=None, _safe_sleep=Mock())

    def test_flag_is_tristate_and_both_explicit_choices_parse(self):
        defaults = parse_controller_args([])
        self.assertIsNone(defaults.adaptive_braking_calibration)
        args = parse_controller_args(['--calibrate', '--adaptive-braking-calibration',
                                      '--log', '--smooth-controller-rate', '100'])
        self.assertTrue(args.calibrate)
        self.assertTrue(args.adaptive_braking_calibration)
        self.assertFalse(args.braking_test)
        self.assertFalse(args.interaction)
        disabled = parse_controller_args([
            '--calibrate', '--no-adaptive-braking-calibration',
            '--log', '--smooth-controller-rate', '100',
        ])
        self.assertFalse(disabled.adaptive_braking_calibration)

    def test_cli_rejects_explicit_selection_without_exclusive_calibration(self):
        for option in ('--adaptive-braking-calibration',
                       '--no-adaptive-braking-calibration'):
            for modes in ([], ['--interaction'], ['--braking-test'],
                          ['--calibrate', '--interaction'],
                          ['--calibrate', '--braking-test']):
                with self.subTest(option=option, modes=modes), redirect_stderr(
                        io.StringIO()), self.assertRaises(SystemExit):
                    parse_controller_args([
                        *modes, option, '--log',
                        '--smooth-controller-rate', '100',
                    ])

    def test_existing_logging_and_sample_rate_requirements_still_apply(self):
        for options in ([], ['--log', '--smooth-controller-rate', '30']):
            with self.subTest(options=options), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_controller_args(['--calibrate', '--adaptive-braking-calibration', *options])

    def test_default_calibration_enables_both_in_independent_copy(self):
        run, factory = calibration_switch()
        mission = {'Interaction': {'config': {'wrench_interaction': {
            'adaptive_braking_calibration': {'enabled': False},
            'online_prediction_calibration': {'enabled': False}}}}}
        before = deepcopy(mission)
        run(self.controller(self.args(), mission))
        configured = factory.call_args.args[3]
        self.assertIsNot(configured, mission)
        self.assertTrue(configured['Interaction']['config']['wrench_interaction'][
            'adaptive_braking_calibration']['enabled'])
        self.assertTrue(configured['Interaction']['config']['wrench_interaction'][
            'online_prediction_calibration']['enabled'])
        self.assertEqual(mission, before)
        factory.return_value.run_calibration.assert_called_once()

    def test_explicit_opt_out_overrides_mission_without_disabling_diagnostic_fit(self):
        run, factory = calibration_switch()
        mission = {'Interaction': {'config': {'wrench_interaction': {
            'adaptive_braking_calibration': {'enabled': True},
            'online_prediction_calibration': {'enabled': True},
        }}}}
        before = deepcopy(mission)
        run(self.controller(self.args(adaptive_braking_calibration=False), mission))
        configured = factory.call_args.args[3]
        wrench = configured['Interaction']['config']['wrench_interaction']
        self.assertFalse(wrench['adaptive_braking_calibration']['enabled'])
        self.assertTrue(wrench['online_prediction_calibration']['enabled'])
        self.assertEqual(mission, before)

    def test_selected_mode_enables_both_in_independent_copy_only(self):
        run, factory = calibration_switch()
        mission = {
            'boundary_limits': {'x': [-1.5, 1.5]},
            'Interaction': {'config': {'wrench_interaction': {
                'adaptive_braking_calibration': {'enabled': False, 'tilt_limit_deg': 20},
                'online_prediction_calibration': {'enabled': False, 'max_sample_gap_s': .06},
                'planar_braking_calibration': {'max_xy_speed_m_s': 1.6},
            }}},
        }
        before = deepcopy(mission)
        run(self.controller(self.args(adaptive_braking_calibration=True), mission))
        configured = factory.call_args.args[3]
        expected = deepcopy(before)
        for name in ('adaptive_braking_calibration', 'online_prediction_calibration'):
            expected['Interaction']['config']['wrench_interaction'][name]['enabled'] = True
        self.assertEqual(configured, expected)
        self.assertIsNot(configured, mission)
        self.assertEqual(mission, before)
        configured['boundary_limits']['x'][0] = -99
        self.assertEqual(mission, before)
        factory.return_value.run_calibration.assert_called_once()
        factory.return_value.run_braking_test.assert_not_called()

    def test_selected_mode_can_create_missing_optional_config(self):
        run, factory = calibration_switch()
        mission = {'drones': {'lb11': {'target': [0, 0, 1]}}}
        run(self.controller(self.args(adaptive_braking_calibration=True), mission))
        config = factory.call_args.args[3]['Interaction']['config']['wrench_interaction']
        self.assertTrue(config['adaptive_braking_calibration']['enabled'])
        self.assertTrue(config['online_prediction_calibration']['enabled'])
        self.assertNotIn('Interaction', mission)

    def test_programmatic_invalid_modes_rejected_before_dispatch(self):
        for changes in ({'calibrate': False}, {'interaction': True}, {'braking_test': True}):
            with self.subTest(changes=changes):
                run, factory = calibration_switch()
                controller = self.controller(self.args(adaptive_braking_calibration=True, **changes), {})
                with self.assertRaisesRegex(ValueError, 'requires --calibrate'):
                    run(controller)
                factory.assert_not_called()

    def test_ground_test_does_not_construct_flight_controller_or_mutate_mission(self):
        run, factory = calibration_switch()
        controller = self.controller(self.args(adaptive_braking_calibration=True, ground_test=True), {})
        run(controller)
        factory.assert_not_called()
        controller._safe_sleep.assert_called_once_with(1)
        self.assertEqual(controller.mission, {})


if __name__ == '__main__':
    unittest.main()
