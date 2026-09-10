"""No-hardware checks for the controller-side ``--mpc`` entry point."""

import argparse
import ast
from contextlib import redirect_stderr
from copy import deepcopy
import datetime
import io
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from Interaction.braking_repeat_test import validate_repeat_test_options


SOURCE = Path(__file__).resolve().parents[2] / "controller.py"
INTERACTIONS_SOURCE = SOURCE.parent / "Interaction" / "interactions.py"


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
        "argparse": argparse,
        "datetime": datetime,
        "math": math,
        "validate_repeat_test_options": validate_repeat_test_options,
    }
    with patch("sys.argv", ["controller.py", *tokens]):
        exec(
            compile(
                ast.Module(body=statements, type_ignores=[]),
                str(SOURCE),
                "exec",
            ),
            namespace,
        )
    return namespace["args"]


def controller_methods(*names):
    tree = ast.parse(SOURCE.read_text())
    controller = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "Controller"
    )
    selected = [
        node for node in controller.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {
        "InteractionsControl": Mock(),
        "deepcopy": deepcopy,
        "logging": Mock(),
        "traceback": Mock(),
    }
    exec(
        compile(ast.Module(body=selected, type_ignores=[]), str(SOURCE), "exec"),
        namespace,
    )
    return namespace


class MPCBootstrapCliTests(unittest.TestCase):
    def test_start_payload_is_bound_to_directional_model_contract(self):
        source = INTERACTIONS_SOURCE.read_text()
        self.assertIn(
            "mpc_bootstrap_model_contract_for_direction(", source
        )
        for field in (
            "release_dataset_model_label",
            "release_dataset_model_fingerprint",
            "release_dataset_state_dimension",
            "release_dataset_command_delay_s",
        ):
            self.assertIn(f"'{field}'", source)
        self.assertIn(
            "- release_dataset_effective_command_delay_s", source
        )
        self.assertGreaterEqual(
            source.count("- release_dataset_effective_command_delay_s"), 2
        )
        self.assertIn(
            "state_time,\n"
            "                                    "
            "release_dataset_effective_command_delay_s,",
            source,
        )

    def test_every_episode_direction_reset_also_clears_model_contract(self):
        lines = INTERACTIONS_SOURCE.read_text().splitlines()
        direction_resets = [
            index for index, line in enumerate(lines)
            if "release_dataset_direction_xy = None" in line
        ]
        self.assertGreater(len(direction_resets), 1)
        for index in direction_resets:
            with self.subTest(line=index+1):
                self.assertTrue(any(
                    "release_dataset_model_contract = None" in line
                    for line in lines[index:index+3]
                ))

    def test_one_orchestrated_flag_keeps_sensor_reader_disabled(self):
        args = parse_controller_args([
            "--orchestrated", "--mpc", "--log",
            "--smooth-controller-rate", "100",
            "--cf-log-period", "10",
        ])
        self.assertTrue(args.mpc)
        self.assertFalse(args.sense)

    def test_sensor_axis_option_does_not_change_automatic_mpc_entry(self):
        args = parse_controller_args([
            "--orchestrated", "--mpc", "--sense-axis", "x", "--log",
            "--smooth-controller-rate", "100", "--cf-log-period", "10",
        ])

        self.assertTrue(args.mpc)
        self.assertFalse(args.sense)
        self.assertEqual(args.sense_axis, "x")

    def test_requires_logging_fresh_state_rate_and_exclusive_mode(self):
        bad = (
            ["--mpc"],
            ["--mpc", "--log", "--smooth-controller-rate", "100",
             "--cf-log-period", "10"],
            ["--orchestrated", "--mpc", "--log", "--smooth-controller-rate", "50",
             "--cf-log-period", "10"],
            ["--orchestrated", "--mpc", "--log", "--smooth-controller-rate", "100",
             "--cf-log-period", "20"],
            ["--orchestrated", "--mpc", "--interaction", "--log", "--smooth-controller-rate", "100"],
            ["--orchestrated", "--mpc", "--calibrate", "--log", "--smooth-controller-rate", "100"],
            ["--orchestrated", "--mpc", "--braking-test", "--log", "--smooth-controller-rate", "100"],
            ["--orchestrated", "--mpc", "--ground-test", "--log", "--smooth-controller-rate", "100"],
            ["--orchestrated", "--mpc", "--sense", "--log", "--smooth-controller-rate", "100",
             "--cf-log-period", "10"],
            ["--orchestrated", "--mpc", "--simple-takeoff", "--log",
             "--smooth-controller-rate", "100", "--cf-log-period", "10"],
            ["--orchestrated", "--mpc", "--trajectory", "path.json", "--log",
             "--smooth-controller-rate", "100", "--cf-log-period", "10"],
            ["--orchestrated", "--mpc", "--illumination", "--log",
             "--smooth-controller-rate", "100", "--cf-log-period", "10"],
        )
        for tokens in bad:
            with self.subTest(tokens=tokens), redirect_stderr(
                io.StringIO()
            ), self.assertRaises(SystemExit):
                parse_controller_args(tokens)

    def test_dispatch_uses_private_overlay_and_never_old_calibration(self):
        namespace = controller_methods("calibration_switch")
        mission = {
            "Interaction": {"action": "translation", "config": {
                "wrench_interaction": {
                    "learning_velocity_mpc_shadow": {
                        "enabled": True,
                        "command_authority": True,
                    },
                    "control_handoff": {
                        "coast_velocity_braking_enabled": True,
                    },
                },
            }},
        }
        before = deepcopy(mission)
        args = SimpleNamespace(
            mpc=True,
            calibrate=False,
            braking_test=False,
            interaction=False,
            targeted_braking_calibration=False,
            adaptive_braking_calibration=None,
            ground_test=False,
            smooth_controller_rate=100,
            drone_id="lb11",
            sense_axis="y",
            sense_sign=1,
            sense_max_age=.25,
        )
        instance = SimpleNamespace(
            args=args,
            mission=mission,
            cf=Mock(),
            log_manager=Mock(),
            manifest=None,
            force_sensor=None,
            _safe_sleep=Mock(),
        )
        namespace["calibration_switch"](instance)
        factory = namespace["InteractionsControl"]
        self.assertIsNone(factory.call_args.kwargs["force_sensor"])
        configured = factory.call_args.args[3]
        self.assertEqual(mission, before)
        self.assertIsNot(configured, mission)
        wrench = configured["Interaction"]["config"]["wrench_interaction"]
        self.assertTrue(wrench["mpc_bootstrap_calibration"]["enabled"])
        self.assertFalse(
            wrench["learning_velocity_mpc_shadow"]["command_authority"]
        )
        self.assertFalse(
            wrench["control_handoff"]["coast_velocity_braking_enabled"]
        )
        controller = factory.return_value
        controller.run_mpc_calibration.assert_called_once_with()
        controller.run_calibration.assert_not_called()
        controller.run_braking_test.assert_not_called()

    def test_run_mission_and_interaction_identity_include_mpc(self):
        namespace = controller_methods("run_mission", "_is_interaction_application")
        args = SimpleNamespace(
            mpc=True,
            calibrate=False,
            braking_test=False,
            interaction=False,
            sense=False,
            orchestrated=True,
            autotune=False,
            simple_takeoff=False,
            rotation_test=False,
            xy_tune=False,
            z_tune=False,
            trajectory=None,
        )
        instance = SimpleNamespace(args=args, calibration_switch=Mock())
        self.assertTrue(namespace["_is_interaction_application"](instance))
        namespace["run_mission"](instance)
        instance.calibration_switch.assert_called_once_with()

    def test_private_mission_is_prepared_before_flight_dispatch(self):
        namespace = controller_methods("prepare_mpc_mission")
        original = {"name": "downloaded"}
        prepared = {"name": "validated-private-overlay"}
        instance = SimpleNamespace(
            args=SimpleNamespace(
                mpc=True, drone_id="lb11", smooth_controller_rate=100,
            ),
            mission=original,
            missions=[original],
        )
        with patch(
            "Interaction.mpc_bootstrap_calibration.prepare_mpc_bootstrap_mission",
            return_value=prepared,
        ) as prepare:
            namespace["prepare_mpc_mission"](instance)
        self.assertIs(instance.mission, prepared)
        self.assertIs(instance.missions[0], prepared)
        prepare.assert_called_once_with(
            original, drone_id="lb11", controller_rate_hz=100,
        )


if __name__ == "__main__":
    unittest.main()
