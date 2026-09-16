"""No-hardware admission checks for the explicit seventh-order flight mode."""

import ast
from copy import deepcopy
from contextlib import redirect_stderr
import io
from pathlib import Path
import unittest
from unittest.mock import patch

from Interaction.active_septic_brake_test import validate_active_septic_brake_test
from Interaction.tests.test_vicon_rigidbody_position_only import parse_args


def mission():
    return {
        'drones': {'lb11': {'target': [0, -1, 1, 90]}},
        'Interaction': {
            'action': 'translation',
            'config': {
                'detection_method': 'momentum_impulse',
                'wrench_calibration_file': '/tmp/nonexistent-active-septic-fit.json',
                'virtual_object': {
                    'contact_detection': {'source': 'potentiometer'},
                    'release_behavior': {'mode': 'potentiometer_coast'},
                },
                'wrench_interaction': {
                    'state_source': 'onboard',
                    'shadow_mode': False,
                    'control_handoff': {
                        'coast_jerk_limited_attitude_enabled': True,
                        'coast_jerk_limited_septic_smoothing_enabled': True,
                        'coast_jerk_limited_free_stop_enabled': True,
                    },
                },
            },
        },
    }


class ActiveSepticBrakeTestTests(unittest.TestCase):
    def test_controller_checks_fit_before_arm_and_takeoff(self):
        source = Path(__file__).resolve().parents[2] / 'controller.py'
        tree = ast.parse(source.read_text())
        controller = next(
            node for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == 'Controller'
        )
        start = next(
            node for node in controller.body
            if isinstance(node, ast.FunctionDef) and node.name == 'start'
        )
        calls = [
            node.func.attr for node in ast.walk(start)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        ]
        self.assertLess(calls.index('prepare_active_septic_brake_test'),
                        calls.index('arm'))
        self.assertLess(calls.index('prepare_active_septic_brake_test'),
                        calls.index('takeoff'))

    def test_controller_cli_requires_exact_one_flag_expansion(self):
        args = parse_args([
            '--orchestrated', '--interaction', '--sense', '--log',
            '--vicon-rigidbody-position-only', 'FLS',
            '--smooth-controller-rate', '100', '--cf-log-period', '10',
            '--active-septic-brake-test',
        ])
        self.assertTrue(args.active_septic_brake_test)
        for extra in ('--radio radio://0/6/1M/E7E7E7E704', '--ground-test',
                      '--skip-takeoff', '--droneless'):
            with self.subTest(extra=extra), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args([
                    '--orchestrated', '--interaction', '--sense', '--log',
                    '--vicon-rigidbody-position-only', 'FLS',
                    '--smooth-controller-rate', '100', '--cf-log-period', '10',
                    '--active-septic-brake-test', *extra.split(),
                ])

    def test_missing_fit_fails_before_flight(self):
        with self.assertRaisesRegex(ValueError, 'pre-arm calibration gate'):
            validate_active_septic_brake_test(
                mission(), drone_id='lb11', sense_axis='y', sense_sign=1,
            )

    def test_verified_fit_uses_same_signed_direction_as_runtime(self):
        with patch(
            'Interaction.active_septic_brake_test.apply_required_jerk_braking_calibration',
            return_value=({}, {'planar_braking_fit': {'usable': True}}),
        ) as gate:
            result = validate_active_septic_brake_test(
                mission(), drone_id='lb11', sense_axis='y', sense_sign=-1,
            )
        self.assertAlmostEqual(result['runtime_direction_xy'][0], 1.0)
        self.assertAlmostEqual(result['runtime_direction_xy'][1], 0.0)
        self.assertTrue(result['planar_braking_fit_verified'])
        self.assertEqual(gate.call_args.args[1], 'lb11')
        self.assertEqual(
            gate.call_args.args[2], '/tmp/nonexistent-active-septic-fit.json',
        )

    def test_shadow_or_unsmoothed_mission_is_refused(self):
        for mutate in (
            lambda d: d['Interaction']['config']['wrench_interaction'].update(
                shadow_mode=True),
            lambda d: d['Interaction']['config']['wrench_interaction'].update(
                _crazysim_model_verified=True),
            lambda d: d['Interaction']['config']['wrench_interaction'][
                'control_handoff'].update(
                    coast_jerk_limited_septic_smoothing_enabled=False),
        ):
            candidate = deepcopy(mission())
            mutate(candidate)
            with self.assertRaises(ValueError):
                validate_active_septic_brake_test(
                    candidate, drone_id='lb11', sense_axis='y', sense_sign=1,
                )


if __name__ == '__main__':
    unittest.main()
