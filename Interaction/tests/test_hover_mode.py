"""No-hardware checks for the indefinite ``--hover`` mode."""

import ast
from contextlib import redirect_stderr
import io
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import argparse
import datetime

from Interaction.braking_repeat_test import validate_repeat_test_options


SOURCE = Path(__file__).resolve().parents[2] / 'controller.py'


def parse_controller_args(tokens):
    tree = ast.parse(SOURCE.read_text())
    main = next(
        node for node in tree.body
        if isinstance(node, ast.If) and isinstance(node.test, ast.Compare)
    )
    statements = []
    for node in main.body:
        if isinstance(node, ast.With):
            break
        statements.append(node)
    namespace = {
        'argparse': argparse,
        'datetime': datetime,
        'math': math,
        'validate_repeat_test_options': validate_repeat_test_options,
    }
    with patch('sys.argv', ['controller.py', *tokens]):
        exec(
            compile(ast.Module(body=statements, type_ignores=[]), str(SOURCE), 'exec'),
            namespace,
        )
    return namespace['args']


def controller_methods(*names):
    tree = ast.parse(SOURCE.read_text())
    controller = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'Controller'
    )
    selected = [
        node for node in controller.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {
        'logger': Mock(),
        'time': SimpleNamespace(time=lambda: 1000.0),
    }
    exec(
        compile(ast.Module(body=selected, type_ignores=[]), str(SOURCE), 'exec'),
        namespace,
    )
    return namespace


class HoverModeTests(unittest.TestCase):
    def test_cli_registers_isolated_airborne_mode(self):
        args = parse_controller_args(['--hover'])
        self.assertTrue(args.hover)
        for tokens in (
            ['--hover', '--interaction'],
            ['--hover', '--calibrate', '--log', '--smooth-controller-rate', '100'],
            ['--hover', '--ground-test'],
            ['--hover', '--droneless'],
            ['--hover', '--skip-takeoff'],
            ['--hover', '--skip-landing'],
            ['--hover', '--sense', '--log'],
        ):
            with self.subTest(tokens=tokens), redirect_stderr(
                io.StringIO()
            ), self.assertRaises(SystemExit):
                parse_controller_args(tokens)

    def test_commands_fixed_target_then_holds(self):
        timeline = []

        class StopTest(Exception):
            pass

        def safe_sleep(duration_s):
            timeline.append(('sleep', duration_s))
            if len([item for item in timeline if item[0] == 'sleep']) == 3:
                raise StopTest

        commander = Mock()
        log_manager = Mock()
        instance = SimpleNamespace(
            hl_commander=commander,
            log_manager=log_manager,
            _safe_sleep=safe_sleep,
        )
        method = controller_methods('run_hover_forever')['run_hover_forever']

        with self.assertRaises(StopTest):
            method(instance)

        commander.go_to.assert_called_once_with(
            0.0, 0.0, 1.0, 0.0, 2.0, relative=False
        )
        self.assertEqual(timeline, [
            ('sleep', 2.0), ('sleep', 1.0), ('sleep', 1.0),
        ])
        self.assertEqual(
            log_manager.add_log_entry.call_args.kwargs['name'],
            'Hover Target Commanded',
        )

    def test_run_mission_dispatches_hover(self):
        method = controller_methods('run_mission')['run_mission']
        args = SimpleNamespace(
            hover=True,
            baseline=False,
            calibrate=False,
            braking_test=False,
            mpc=False,
            autotune=False,
            simple_takeoff=False,
            rotation_test=False,
            xy_tune=False,
            z_tune=False,
            trajectory=None,
            orchestrated=True,
        )
        instance = SimpleNamespace(args=args, run_hover_forever=Mock())
        method(instance)
        instance.run_hover_forever.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
