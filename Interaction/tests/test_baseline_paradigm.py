"""No-hardware checks for the controller-side ``--baseline`` flight path."""

import ast
from contextlib import redirect_stderr
import io
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from Interaction.tests.test_mpc_calibration_cli import parse_controller_args


SOURCE = Path(__file__).resolve().parents[2] / 'controller.py'


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
        compile(
            ast.Module(body=selected, type_ignores=[]), str(SOURCE), 'exec'
        ),
        namespace,
    )
    return namespace


class BaselineParadigmTests(unittest.TestCase):
    def test_cli_requires_orchestrated_logged_100hz_isolated_flight(self):
        args = parse_controller_args([
            '--orchestrated', '--baseline', '--log',
            '--smooth-controller-rate', '100', '--cf-log-period', '10',
        ])
        self.assertTrue(args.baseline)
        bad = (
            ['--baseline'],
            ['--orchestrated', '--baseline', '--log',
             '--smooth-controller-rate', '50', '--cf-log-period', '10'],
            ['--orchestrated', '--baseline', '--log',
             '--smooth-controller-rate', '100', '--cf-log-period', '20'],
            ['--orchestrated', '--baseline', '--interaction', '--log',
             '--smooth-controller-rate', '100', '--cf-log-period', '10'],
            ['--orchestrated', '--baseline', '--sense', '--log',
             '--smooth-controller-rate', '100', '--cf-log-period', '10'],
            ['--orchestrated', '--baseline', '--ground-test', '--log',
             '--smooth-controller-rate', '100', '--cf-log-period', '10'],
        )
        for tokens in bad:
            with self.subTest(tokens=tokens), redirect_stderr(
                io.StringIO()
            ), self.assertRaises(SystemExit):
                parse_controller_args(tokens)

    def test_command_ownership_and_timing(self):
        timeline = []

        class HighLevelCommander:
            def go_to(self, *args, **kwargs):
                timeline.append(('hl_go_to', args, kwargs))

        class LowLevelCommander:
            def send_position_setpoint(self, *args):
                timeline.append(('position', args))

        class LogManager:
            def add_log_entry(self, group_name, entry, name=None):
                timeline.append(('event', name, entry))

        clock = [0.0]

        def safe_sleep(duration_s):
            timeline.append(('sleep', duration_s))
            clock[0] += duration_s

        def reset_integrators(_cf, names):
            timeline.append(('integrator_reset', names, clock[0]))
            return 'raw_by_name_no_ack'

        namespace = controller_methods('run_baseline')
        namespace['reset_pid_integrators_without_ack'] = reset_integrators
        instance = SimpleNamespace(
            hl_commander=HighLevelCommander(),
            ll_commander=LowLevelCommander(),
            log_manager=LogManager(),
            cf=object(),
            _safe_sleep=safe_sleep,
        )

        namespace['run_baseline'](instance)

        go_to = next(item for item in timeline if item[0] == 'hl_go_to')
        self.assertEqual(go_to[1], (0.0, 1.0, 1.0, 0.0, 3.0))
        self.assertEqual(go_to[2], {'relative': False})
        sleeps = [item[1] for item in timeline if item[0] == 'sleep']
        self.assertEqual(sleeps[:2], [3.0, 5.0])
        self.assertEqual(len(sleeps[2:]), 300)
        self.assertTrue(all(value == 0.01 for value in sleeps[2:]))

        position_indices = [
            index for index, item in enumerate(timeline)
            if item[0] == 'position'
        ]
        self.assertEqual(len(position_indices), 300)
        self.assertTrue(all(
            timeline[index][1] == (0.0, -1.0, 1.0, 0.0)
            for index in position_indices
        ))
        reset_index = next(
            index for index, item in enumerate(timeline)
            if item[0] == 'integrator_reset'
        )
        self.assertEqual(
            timeline[reset_index][1],
            ('posCtlPid.resetI', 'velCtlPid.resetI'),
        )
        self.assertEqual(timeline[reset_index][2], 8.0)
        self.assertLess(reset_index, position_indices[0])

        event_names = [
            item[1] for item in timeline if item[0] == 'event'
        ]
        self.assertEqual(event_names, [
            'Baseline High Level Outbound Started',
            'Baseline Destination Hold Started',
            'Baseline Position Return Started',
            'Baseline Complete',
        ])

    def test_run_mission_dispatches_only_baseline(self):
        namespace = controller_methods(
            'run_mission', '_is_interaction_application'
        )
        args = SimpleNamespace(
            baseline=True,
            interaction=False,
            calibrate=False,
            braking_test=False,
            mpc=False,
            sense=False,
            autotune=False,
            simple_takeoff=False,
            rotation_test=False,
            xy_tune=False,
            z_tune=False,
            trajectory=None,
            orchestrated=True,
        )
        instance = SimpleNamespace(args=args, run_baseline=Mock())
        self.assertTrue(namespace['_is_interaction_application'](instance))
        namespace['run_mission'](instance)
        instance.run_baseline.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
