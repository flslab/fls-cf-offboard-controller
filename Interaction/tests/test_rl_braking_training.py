"""Fast provenance/CLI tests, plus a tiny optional CPU PPO save/reload smoke test."""
from dataclasses import dataclass
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from Interaction.train_rl_braking import (
    PPO_SETTINGS, TrainingConfig, assert_source_stable, build_parser, canonical_sha256,
    evaluate_policy, run_training, sha256_file, validate_args, write_json,
)


@dataclass(frozen=True)
class DummyScenario:
    name: str


class DummyPolicy:
    def predict(self, observation, deterministic=False):
        if not deterministic:
            raise AssertionError("validation policy must be deterministic")
        return np.array(1), None


class DummyEnv:
    instances = []

    def __init__(self, nominal_model, config, scenarios, randomize):
        self.nominal_model = nominal_model
        self.config = config
        self.scenarios = scenarios
        self.randomize = randomize
        self.closed = False
        self.instances.append(self)

    def reset(self, seed):
        self.seed = seed
        self.steps = 0
        return np.zeros(3), {}

    def step(self, action):
        if action != 1:
            raise AssertionError("unexpected policy action")
        self.steps += 1
        done = self.steps == 3
        return np.zeros(3), -2.0 if done else 0.0, done, False, (
            dict(loss=2.0, metrics=dict(speed=.03, capture=True, non_scalar=[3])) if done else {}
        )

    def close(self):
        self.closed = True


class RLBrakingTrainingTests(unittest.TestCase):
    def test_config_defaults_and_undiscounted_terminal_cost(self):
        config = TrainingConfig()
        self.assertEqual(config.seed, 42)
        self.assertEqual(config.n_envs, 4)
        self.assertEqual(config.eval_count, 64)
        self.assertEqual(config.eval_seed, 9001)
        self.assertEqual(PPO_SETTINGS["gamma"], 1.0)
        self.assertEqual(PPO_SETTINGS["gae_lambda"], 1.0)

    def test_rejects_invalid_numeric_and_overlapping_seed_config(self):
        for kwargs in (
            dict(seed=-1), dict(seed=2**32), dict(timesteps=0), dict(n_envs=-1),
            dict(threads=0), dict(eval_count=0), dict(timesteps=1.5),
            dict(learning_rate=float("nan")), dict(learning_rate=float("inf")),
            dict(learning_rate=0), dict(learning_rate=2), dict(eval_seed=42),
            dict(eval_seed=45), dict(eval_seed=2**32 - 2),
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                TrainingConfig(**kwargs)

    def test_cli_validates_paths_without_creating_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            report = root / "frozen.json"
            write_json(report, {})
            args = build_parser().parse_args([
                "--frozen-report", str(report), "--output", str(root / "run"),
                "--seed", "123", "--timesteps", "2048", "--n-envs", "1",
            ])
            self.assertEqual(validate_args(args).seed, 123)
            self.assertFalse(args.output.exists())
            args.output.mkdir()
            with self.assertRaisesRegex(ValueError, "already exists"):
                validate_args(args)
            args.output = root / "new_run"
            args.resume = root / "missing.zip"
            with self.assertRaisesRegex(ValueError, "resume checkpoint"):
                validate_args(args)
            args.resume = None
            args.frozen_report = root / "missing.json"
            with self.assertRaisesRegex(ValueError, "frozen report"):
                validate_args(args)

    def test_hashes_detect_content_change_not_dictionary_order(self):
        self.assertEqual(canonical_sha256(dict(a=1, b=2)),
                         canonical_sha256(dict(b=2, a=1)))
        self.assertNotEqual(canonical_sha256(dict(a=1)), canonical_sha256(dict(a=2)))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "artifact.json"
            write_json(path, dict(value=1))
            first = sha256_file(path)
            self.assertEqual(first, sha256_file(path))
            write_json(path, dict(value=2))
            self.assertNotEqual(first, sha256_file(path))

    def test_changed_source_invalidates_run(self):
        with patch("Interaction.train_rl_braking.source_hashes", return_value={"source": "original"}):
            assert_source_stable({"source": "original"})
            with self.assertRaisesRegex(RuntimeError, "source changed"):
                assert_source_stable({"source": "other"})

    def test_evaluation_is_fixed_deterministic_and_scores_terminal_tail(self):
        DummyEnv.instances = []
        scenarios = [DummyScenario("first"), DummyScenario("second")]
        result = evaluate_policy(DummyPolicy(), "model", "config", scenarios,
                                 seed=99, env_factory=DummyEnv)
        self.assertEqual(result["mean_loss"], 2.0)
        self.assertEqual(result["std_loss"], 0.0)
        self.assertEqual(result["mean_episode_reward"], -2.0)
        self.assertEqual(result["mean_decisions"], 3.0)
        self.assertEqual(result["metric_means"], dict(speed=.03, capture=1.0))
        self.assertEqual([row["scenario"] for row in result["episodes"]], ["first", "second"])
        self.assertEqual([env.seed for env in DummyEnv.instances], [99, 100])
        self.assertTrue(all(env.closed and not env.randomize for env in DummyEnv.instances))
        self.assertEqual([env.scenarios for env in DummyEnv.instances], [[scenarios[0]], [scenarios[1]]])
        with self.assertRaisesRegex(ValueError, "at least one"):
            evaluate_policy(DummyPolicy(), "model", "config", [], env_factory=DummyEnv)

    def test_bad_terminal_metrics_do_not_leave_environment_open(self):
        class InvalidEnv(DummyEnv):
            def step(self, action):
                return np.zeros(3), -1, True, False, dict(loss=float("nan"), metrics={})

        DummyEnv.instances = []
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            evaluate_policy(DummyPolicy(), None, None, [DummyScenario("invalid")],
                            env_factory=InvalidEnv)
        self.assertTrue(DummyEnv.instances[-1].closed)


@unittest.skipUnless(importlib.util.find_spec("stable_baselines3") is not None,
                     "optional stable-baselines3 offline training dependency is not installed")
class RLBrakingTrainingSmokeTests(unittest.TestCase):
    def test_tiny_training_checkpoint_provenance_and_reload(self):
        from stable_baselines3 import PPO
        from Interaction.evaluate_braking_snapshots import projected_frozen_model
        from Interaction.rl_braking_env import RLBrakeConfig, RLBrakingEnv

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
            # Production settings are immutable CLI defaults; shrink only this
            # save/reload test's rollout/optimizer cost, never experiment runs.
            with patch.dict(PPO_SETTINGS, n_steps=16, batch_size=16, n_epochs=1):
                statistics = run_training(args, validate_args(args))
            self.assertEqual(statistics["actual_training_steps"], 16)
            self.assertFalse(statistics["final_test_evaluated"])
            self.assertTrue(np.isfinite(statistics["final_mean_loss"]))
            for name in ("initial", "best", "final"):
                checkpoint = args.output / f"model_{name}.zip"
                metadata = json.loads((args.output / f"model_{name}.metadata.json").read_text())
                self.assertEqual(metadata["checkpoint_sha256"], sha256_file(checkpoint))
                self.assertEqual(metadata["frozen_report"]["sha256"], sha256_file(frozen))
                self.assertFalse(metadata["final_test_evaluated"])
            scenarios = json.loads((args.output / "development_scenarios.json").read_text())
            metadata = json.loads((args.output / "config.json").read_text())
            self.assertEqual(metadata["development_scenarios_sha256"], canonical_sha256(scenarios))
            self.assertIn("Interaction/rl_braking_env.py", metadata["source_hashes"])
            loaded = PPO.load(args.output / "model_final.zip", device="cpu")
            reloaded = PPO.load(args.output / "model_final.zip", device="cpu")
            env = RLBrakingEnv(projected_frozen_model(report, np.array([0, 1])),
                               config=RLBrakeConfig(), randomize=False)
            try:
                observation, _ = env.reset(seed=12345)
                first, _ = loaded.predict(observation, deterministic=True)
                second, _ = reloaded.predict(observation, deterministic=True)
                np.testing.assert_array_equal(first, second)
            finally:
                env.close()
            self.assertTrue((args.output / "development_history.csv").is_file())
            self.assertTrue((args.output / "training_scalars" / "progress.csv").is_file())
            audit = [json.loads(line) for line in (args.output / "training_scenarios.jsonl").read_text().splitlines()]
            self.assertGreater(len(audit), 0)
            self.assertEqual(audit[0]["initial_env_seed"], 42)
            self.assertEqual(audit[0]["reset_seed"], 42)
            self.assertEqual(audit[0]["env_index"], 0)
            self.assertEqual(audit[0]["episode_index"], 0)
            self.assertIn("model", audit[0]["scenario"])
            self.assertIn("rng_state_after_reset", audit[0])

            resumed = build_parser().parse_args([
                "--frozen-report", str(frozen), "--output", str(root / "resumed"),
                "--resume", str(args.output / "model_final.zip"),
                "--timesteps", "16", "--n-envs", "1", "--eval-count", "2",
            ])
            with patch.dict(PPO_SETTINGS, n_steps=16, batch_size=16, n_epochs=1):
                resumed_stats = run_training(resumed, validate_args(resumed))
            self.assertEqual(resumed_stats["actual_training_steps"], 16)
            self.assertEqual(resumed_stats["model_timesteps"], 32)
            self.assertAlmostEqual(resumed_stats["baseline_mean_loss"], statistics["final_mean_loss"])


if __name__ == "__main__":
    unittest.main()
