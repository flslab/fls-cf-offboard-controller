"""No-hardware checks for the opt-in rigid-body position-only Vicon route."""

import argparse
import ast
from contextlib import redirect_stderr
import datetime
import io
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

from Interaction.braking_repeat_test import validate_repeat_test_options


SOURCE = Path(__file__).resolve().parents[2] / 'controller.py'


def parse_args(tokens):
    tree = ast.parse(SOURCE.read_text())
    main = next(node for node in tree.body if isinstance(node, ast.If))
    statements = []
    for node in main.body:
        if isinstance(node, ast.With):
            break
        statements.append(node)
    namespace = dict(
        argparse=argparse, datetime=datetime, math=math,
        validate_repeat_test_options=validate_repeat_test_options,
    )
    with patch('sys.argv', ['controller.py', *tokens]):
        exec(compile(ast.Module(body=statements, type_ignores=[]),
                     str(SOURCE), 'exec'), namespace)
    return namespace['args']


def controller_methods(*names):
    tree = ast.parse(SOURCE.read_text())
    controller = next(node for node in tree.body
                      if isinstance(node, ast.ClassDef)
                      and node.name == 'Controller')
    selected = [node for node in controller.body
                if isinstance(node, ast.FunctionDef) and node.name in names]
    namespace = dict(
        logger=Mock(),
        time=types.SimpleNamespace(sleep=lambda _: None,
                                   monotonic=lambda: 10.0),
    )
    exec(compile(ast.Module(body=selected, type_ignores=[]),
                 str(SOURCE), 'exec'), namespace)
    return namespace


class RigidBodyPositionOnlyTests(unittest.TestCase):
    def test_single_option_selects_vicon_rigidbody_without_full_pose(self):
        args = parse_args([
            '--interaction', '--log', '--vicon-mode', 'pointcloud',
            '--vicon-rigidbody-position-only', 'FLS',
        ])
        self.assertTrue(args.vicon)
        self.assertEqual(args.vicon_mode, 'rigidbody')
        self.assertEqual(args.obj_name, 'FLS')
        self.assertFalse(args.vicon_full_pose)
        self.assertFalse(args.save_vicon)
        self.assertIsNone(args.contact_attitude_run)

    def test_requires_logging_and_refuses_orientation_forwarding(self):
        for tokens in (
            ['--interaction', '--vicon-rigidbody-position-only', 'FLS'],
            ['--interaction', '--log', '--vicon-rigidbody-position-only', 'FLS',
             '--vicon-full-pose'],
            ['--interaction', '--log', '--vicon-rigidbody-position-only', 'FLS',
             '--save-vicon'],
            ['--interaction', '--log', '--vicon-rigidbody-position-only', ' '],
        ):
            with self.subTest(tokens=tokens), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    parse_args(tokens)

    def test_rigidbody_frame_logs_quaternion_but_sends_only_xyz(self):
        methods = controller_methods(
            'setup_motion_capture', '_send_position',
            '_prepare_mocap_forward_timing', '_finish_mocap_forward_timing',
            '_log_mocap',
        )
        instance = types.SimpleNamespace(
            args=parse_args([
                '--interaction', '--log',
                '--vicon-rigidbody-position-only', 'FLS',
            ]),
            mission={}, log_manager=Mock(), cf=Mock(),
            send_vicon_to_cf=True,
            contact_attitude_diagnostics_enabled=False,
            _log_mocap_timing=Mock(),
        )
        for name in (
            'setup_motion_capture', '_send_position',
            '_prepare_mocap_forward_timing', '_finish_mocap_forward_timing',
            '_log_mocap',
        ):
            setattr(instance, name, types.MethodType(methods[name], instance))

        class FakeMocap:
            def __init__(self, **kwargs):
                self.mode = kwargs['mode']

            def subscribe_object(self, name, callback):
                self.name = name
                self.callback = callback

            def start(self):
                pass

        with patch.dict(sys.modules, {'mocap': types.SimpleNamespace(Mocap=FakeMocap)}):
            instance.setup_motion_capture()
        self.assertEqual(instance.mocap.mode, 'rigidbody')
        self.assertEqual(instance.mocap.name, 'FLS')

        frame = dict(tvec=[1.0, 2.0, 3.0], quat=[0.0, 0.0, 0.5, 0.866],
                     time=1.0)
        instance.mocap.callback(frame)
        instance.cf.extpos.send_extpos.assert_called_once_with(1.0, 2.0, 3.0)
        instance.cf.extpos.send_extpose.assert_not_called()
        logged = instance.log_manager.add_log_entry.call_args.args
        self.assertEqual(logged[0], 'frames')
        self.assertEqual(logged[1]['quat'], frame['quat'])


if __name__ == '__main__':
    unittest.main()
