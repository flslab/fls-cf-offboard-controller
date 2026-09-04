"""Version/provenance tests and a genuinely tiny optional v2 PPO smoke test."""
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from Interaction.train_rl_coast import (
    ACTION_VERSION, OBSERVATION_VERSION, TASK_VERSION, PPO_SETTINGS,
    CoastTrainingConfig, RESUME_MATCH_FIELDS, assert_source_stable,
    build_parser, canonical_sha256, checkpoint_metadata_path, run_training,
    sha256_file, validate_args, validate_resume_checkpoint, write_json,
)


class CoastTrainingTests(unittest.TestCase):
    def test_defaults_version_actions_and_undiscounted_terminal_loss(self):
        config = CoastTrainingConfig()
        self.assertEqual(config.seed, 41)
        self.assertEqual(config.timesteps, 32768)
        self.assertEqual(config.eval_seed, 19001)
        self.assertEqual(config.eval_count, 64)
        self.assertEqual(config.n_envs, 4)
        self.assertEqual(PPO_SETTINGS["gamma"], 1.0)
        self.assertEqual(PPO_SETTINGS["gae_lambda"], 1.0)
        self.assertIn("coast_v2", TASK_VERSION)
        self.assertIn("position", ACTION_VERSION)

    def test_rejects_inspected_v1_seeds_and_overlapping_streams(self):
        for kwargs in (
            dict(seed=9001), dict(eval_seed=9001), dict(eval_seed=491723),
            dict(seed=491723), dict(seed=8999), dict(eval_seed=8999),
            dict(seed=19000), dict(eval_seed=42), dict(seed=-1),
            dict(timesteps=0), dict(eval_count=0), dict(threads=0),
            dict(learning_rate=float("nan")), dict(n_envs=0),
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                CoastTrainingConfig(**kwargs)

    def test_cli_validation_does_not_create_or_overwrite_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            report = root / "frozen.json"
            write_json(report, {})
            args = build_parser().parse_args([
                "--frozen-report", str(report), "--output", str(root / "new"),
            ])
            self.assertEqual(validate_args(args).eval_seed, 19001)
            self.assertFalse(args.output.exists())
            args.output = root
            with self.assertRaisesRegex(ValueError, "already exists"):
                validate_args(args)
            args.output = root / "new"
            args.resume = root / "missing.zip"
            with self.assertRaisesRegex(ValueError, "existing v2"):
                validate_args(args)
            args.resume.write_bytes(b"checkpoint")
            with self.assertRaisesRegex(ValueError, "sidecar"):
                validate_args(args)

    def test_help_does_not_import_rl_or_flight_modules(self):
        code = '''
import importlib.abc
import sys
class BlockOptional(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'torch', 'stable_baselines3', 'gymnasium', 'cflib', 'motioncapture'}:
            raise AssertionError('unexpected optional or flight import: ' + fullname)
sys.meta_path.insert(0, BlockOptional())
from Interaction.train_rl_coast import main
main(['--help'])
'''
        result = subprocess.run([sys.executable, "-c", code],
                                capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--resume", result.stdout)
        self.assertIn("position-PD surrogate", result.stdout)

    def test_changed_source_invalidates_run(self):
        with patch("Interaction.train_rl_coast.source_hashes", return_value={"source": "original"}):
            assert_source_stable({"source": "original"})
            with self.assertRaisesRegex(RuntimeError, "source changed"):
                assert_source_stable({"source": "modified"})

    @staticmethod
    def resume_fixture(root):
        checkpoint = root / "model_best.zip"
        checkpoint.write_bytes(b"small checkpoint stand-in for metadata-only validation")
        env_config = dict(horizon_s=3.0, max_brake_s=.5)
        sources = {"Interaction/rl_coast_env.py": "a" * 64}
        expected = dict(
            schema_version=2, task_version=TASK_VERSION,
            observation_version=OBSERVATION_VERSION, action_version=ACTION_VERSION,
            env_config=env_config, env_config_sha256=canonical_sha256(env_config),
            frozen_parameters_sha256="b" * 64, development_scenarios_sha256="c" * 64,
            source_hashes=sources, source_tree_sha256=canonical_sha256(sources),
            ppo_settings=PPO_SETTINGS,
        )
        metadata = dict(copy.deepcopy(expected), checkpoint_sha256=sha256_file(checkpoint),
                        training_config=dict(seed=41, timesteps=32768))
        write_json(checkpoint_metadata_path(checkpoint), metadata)
        return checkpoint, expected, metadata

    def test_matching_v2_resume_allows_new_training_rng_and_budget(self):
        with tempfile.TemporaryDirectory() as directory:
            checkpoint, expected, metadata = self.resume_fixture(Path(directory))
            expected["training_config"] = dict(seed=1041, timesteps=65536)
            self.assertEqual(validate_resume_checkpoint(checkpoint, expected), metadata)

    def test_resume_rejects_every_version_environment_source_and_model_mismatch(self):
        with tempfile.TemporaryDirectory() as directory:
            checkpoint, expected, metadata = self.resume_fixture(Path(directory))
            for key in RESUME_MATCH_FIELDS:
                changed = copy.deepcopy(metadata)
                changed[key] = "incompatible"
                write_json(checkpoint_metadata_path(checkpoint), changed)
                with self.subTest(key=key), self.assertRaisesRegex(ValueError, key):
                    validate_resume_checkpoint(checkpoint, expected)
            write_json(checkpoint_metadata_path(checkpoint), {"schema_version": 1})
            with self.assertRaisesRegex(ValueError, "schema_version"):
                validate_resume_checkpoint(checkpoint, expected)

    def test_resume_rejects_changed_checkpoint_and_missing_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            checkpoint, expected, _ = self.resume_fixture(root)
            checkpoint.write_bytes(b"tampered checkpoint")
            with self.assertRaisesRegex(ValueError, "SHA-256"):
                validate_resume_checkpoint(checkpoint, expected)
            with self.assertRaisesRegex(ValueError, "sidecar"):
                validate_resume_checkpoint(root / "missing.zip", expected)


@unittest.skipUnless(importlib.util.find_spec("stable_baselines3") is not None
                     and importlib.util.find_spec("Interaction.rl_coast_env") is not None,
                     "optional RL dependencies or v2 environment are not installed")
class CoastTrainingSmokeTests(unittest.TestCase):
    def test_tiny_training_reload_resume_and_scenario_audit(self):
        from stable_baselines3 import PPO
        from Interaction.evaluate_braking_snapshots import projected_frozen_model
        from Interaction.rl_coast_env import CoastBrakeConfig, CoastBrakingEnv

        report = dict(frozen_tilt_fit=dict(
            model="second_order", delay_s=.025, wn_rad_s=14.5,
            zeta=.98, gain=1.035, bias_world_y_rad=0.0,
        ), frozen_motion_gain=1.09)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            frozen = root / "frozen.json"
            write_json(frozen, report)
            args = build_parser().parse_args([
                "--frozen-report", str(frozen), "--output", str(root / "run"),
                "--timesteps", "16", "--n-envs", "1", "--eval-count", "2",
            ])
            # This patch shrinks only the unit-test rollout. Production CLI
            # retains 512 steps, batch 256 and 10 optimizer epochs unchanged.
            with patch.dict(PPO_SETTINGS, n_steps=16, batch_size=16, n_epochs=1):
                statistics = run_training(args, validate_args(args))
                self.assertEqual(statistics["actual_training_steps"], 16)
                self.assertFalse(statistics["final_test_evaluated"])
                self.assertTrue(np.isfinite(statistics["final_mean_loss"]))
                manifest = json.loads((args.output / "manifest.json").read_text())
                self.assertEqual(manifest["task_version"], TASK_VERSION)
                self.assertEqual(manifest["observation_shape"], [117])
                self.assertIn("Interaction/train_rl_braking.py", manifest["source_hashes"])
                self.assertIn("Interaction/rl_coast_env.py", manifest["source_hashes"])
                self.assertEqual(manifest, json.loads((args.output / "config.json").read_text()))
                scenarios = json.loads((args.output / "development_scenarios.json").read_text())
                self.assertEqual(manifest["development_scenarios_sha256"], canonical_sha256(scenarios))
                for name in ("initial", "best", "final"):
                    checkpoint = args.output / f"model_{name}.zip"
                    metadata = validate_resume_checkpoint(checkpoint, manifest)
                    self.assertEqual(metadata["checkpoint_sha256"], sha256_file(checkpoint))
                    self.assertTrue(metadata["position_controller_is_assumed_surrogate_not_identified"])
                audit = [json.loads(line) for line in
                         (args.output / "training_scenarios.jsonl").read_text().splitlines()]
                self.assertGreater(len(audit), 0)
                self.assertEqual(audit[0]["initial_env_seed"], 41)
                self.assertEqual(audit[0]["reset_seed"], 41)
                self.assertEqual(audit[0]["episode_index"], 0)
                self.assertIn("target_distance_m", audit[0]["scenario"])
                self.assertIn("rng_state_after_reset", audit[0])

                first_policy = PPO.load(args.output / "model_final.zip", device="cpu")
                second_policy = PPO.load(args.output / "model_final.zip", device="cpu")
                env = CoastBrakingEnv(projected_frozen_model(report, [0, 1]),
                                      config=CoastBrakeConfig(), randomize=False)
                try:
                    observation, _ = env.reset(seed=11111)
                    np.testing.assert_array_equal(
                        first_policy.predict(observation, deterministic=True)[0],
                        second_policy.predict(observation, deterministic=True)[0])
                finally:
                    env.close()
                resumed = build_parser().parse_args([
                    "--frozen-report", str(frozen), "--output", str(root / "resumed"),
                    "--resume", str(args.output / "model_final.zip"), "--seed", "1041",
                    "--timesteps", "16", "--n-envs", "1", "--eval-count", "2",
                ])
                resumed_statistics = run_training(resumed, validate_args(resumed))
                self.assertEqual(resumed_statistics["actual_training_steps"], 16)
                self.assertEqual(resumed_statistics["model_timesteps"], 32)
                self.assertAlmostEqual(resumed_statistics["baseline_mean_loss"], statistics["final_mean_loss"])
                self.assertTrue((args.output / "development_history.csv").is_file())
                self.assertTrue((args.output / "training_scalars" / "progress.csv").is_file())


if __name__ == "__main__":
    unittest.main()
