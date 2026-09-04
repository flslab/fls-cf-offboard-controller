"""Reproducible offline PPO training for an irreversible brake-to-level switch.

This module never connects to a vehicle, writes calibration, or evaluates the
independent final test set. The fixed development scenarios here select a
checkpoint only. A simulated policy is not a flight-ready controller.

Run with ``python -m Interaction.train_rl_braking --help`` in the optional RL
environment. Importing configuration/provenance helpers does not require SB3.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import platform
import random
import sys
import time
from typing import Any

import numpy as np


PPO_SETTINGS = dict(
    n_steps=512, batch_size=256, n_epochs=10, gamma=1.0,
    # No reward discount or GAE trace truncation across the short, terminal-only
    # optimal-stopping horizon: waiting must not dilute an identical cost.
    gae_lambda=1.0, ent_coef=0.005,
    policy_kwargs={"net_arch": [64, 64]},
)
VALIDATION_INTERVAL = 32768
SOURCE_FILES = (
    "train_rl_braking.py", "rl_braking_env.py", "offline_braking_selector.py",
    "braking_validation_simulator.py", "predictive_brake_release.py",
    "evaluate_braking_snapshots.py", "braking_split_diagnostic.py",
    "braking_trajectory_fit.py", "braking_replay.py",
    "validate_predictive_brake_release.py",
)


@dataclass(frozen=True)
class TrainingConfig:
    seed: int = 42
    timesteps: int = 65536
    n_envs: int = 4
    learning_rate: float = 0.0003
    threads: int = 1
    eval_count: int = 64
    eval_seed: int = 9001

    def __post_init__(self):
        for key in ("timesteps", "n_envs", "threads", "eval_count"):
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{key} must be a positive integer")
        for key in ("seed", "eval_seed"):
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < 2**32:
                raise ValueError(f"{key} must be an integer in [0, 2**32)")
        if self.seed + self.n_envs > 2**32 or self.eval_seed + self.eval_count > 2**32:
            raise ValueError("consecutive environment seeds exceed the 32-bit seed range")
        if not (self.seed + self.n_envs <= self.eval_seed
                or self.eval_seed + self.eval_count <= self.seed):
            raise ValueError("development episode seeds must not overlap training environment seeds")
        if not math.isfinite(self.learning_rate) or not 0 < self.learning_rate <= 1:
            raise ValueError("learning_rate must be finite and in (0, 1]")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def source_hashes() -> dict[str, str]:
    """Hash the policy/environment plus the offline model/metric dependency chain."""
    directory = Path(__file__).resolve().parent
    return {f"Interaction/{name}": sha256_file(directory / name) for name in SOURCE_FILES}


def assert_source_stable(expected: dict[str, str]) -> None:
    if source_hashes() != expected:
        raise RuntimeError("offline training source changed during this run; discard this run and restart from frozen code")


def package_versions() -> dict[str, str]:
    result = {"python": platform.python_version()}
    for name in ("numpy", "scipy", "torch", "gymnasium", "stable-baselines3"):
        try:
            result[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            result[name] = "not-installed"
    return result


def write_json(path: Path, value: Any) -> None:
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def _scalar_metrics(metrics: dict) -> dict:
    result = {}
    for key, value in metrics.items():
        if isinstance(value, (bool, int, float, np.bool_, np.integer, np.floating)):
            scalar = float(value)
            if not math.isfinite(scalar):
                raise ValueError(f"nonfinite evaluation metric: {key}")
            result[key] = scalar
    return result


def evaluate_policy(
    policy, nominal_model, env_config, scenarios, *, seed=9001,
    env_factory=None, progress=None,
) -> dict:
    """Deterministic, fixed development episodes; never sample training or test data.

    ``env_factory`` permits lightweight tests without torch/Gymnasium. Observed
    episode metrics are scoring outputs only, never inputs to ``predict``.
    """
    if not scenarios:
        raise ValueError("evaluation needs at least one fixed scenario")
    if env_factory is None:
        from Interaction.rl_braking_env import RLBrakingEnv
        env_factory = RLBrakingEnv
    rows = []
    for index, scenario in enumerate(scenarios):
        env = env_factory(nominal_model, config=env_config,
                          scenarios=[scenario], randomize=False)
        try:
            observation, _ = env.reset(seed=seed + index)
            episode_reward = 0.0
            for step in range(1, 10001):
                prediction = policy.predict(observation, deterministic=True)
                action = prediction[0] if isinstance(prediction, tuple) else prediction
                action = int(np.asarray(action).item())
                observation, reward, terminated, truncated, info = env.step(action)
                episode_reward += float(reward)
                if terminated or truncated:
                    if "loss" not in info or "metrics" not in info:
                        raise ValueError("terminal environment info lacks loss/metrics")
                    loss = float(info["loss"])
                    if not math.isfinite(loss) or not math.isfinite(episode_reward):
                        raise ValueError("nonfinite evaluation loss/reward")
                    rows.append(dict(
                        scenario=getattr(scenario, "name", str(index)), seed=seed + index,
                        loss=loss, episode_reward=episode_reward, decisions=step,
                        terminated=bool(terminated), truncated=bool(truncated),
                        metrics=_scalar_metrics(info["metrics"]),
                    ))
                    break
            else:
                raise RuntimeError("evaluation exceeded 10,000 decisions without a terminal tail")
        finally:
            env.close()
        if progress is not None:
            progress(index + 1, len(scenarios))
    losses = np.asarray([row["loss"] for row in rows], dtype=float)
    metric_keys = sorted(set.intersection(*(set(row["metrics"]) for row in rows)))
    return dict(
        count=len(rows), mean_loss=float(losses.mean()), std_loss=float(losses.std()),
        min_loss=float(losses.min()), max_loss=float(losses.max()),
        mean_episode_reward=float(np.mean([row["episode_reward"] for row in rows])),
        mean_decisions=float(np.mean([row["decisions"] for row in rows])),
        metric_means={key: float(np.mean([row["metrics"][key] for row in rows]))
                      for key in metric_keys},
        episodes=rows,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frozen-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True,
                        help="new directory only; existing runs are never overwritten")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--timesteps", type=int, default=65536,
                        help="additional training steps; PPO rounds up to a complete rollout")
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--learning-rate", type=float, default=0.0003)
    parser.add_argument("--resume", type=Path,
                        help="warm-start PPO checkpoint; new RNG/environment stream, not bit-exact continuation")
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--eval-count", type=int, default=64)
    parser.add_argument("--eval-seed", type=int, default=9001)
    return parser


def validate_args(args) -> TrainingConfig:
    config = TrainingConfig(**{key: getattr(args, key) for key in TrainingConfig.__dataclass_fields__})
    if args.output.exists():
        raise ValueError("output already exists; choose a new directory")
    if not args.frozen_report.is_file():
        raise ValueError("frozen report must be an existing file")
    if args.resume is not None and not args.resume.is_file():
        raise ValueError("resume checkpoint must be an existing file (include .zip)")
    return config


def run_training(args, config: TrainingConfig) -> dict:
    """Train and select on development data only, writing complete local provenance."""
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.logger import configure
    from stable_baselines3.common.utils import set_random_seed
    from stable_baselines3.common.vec_env import DummyVecEnv

    from Interaction.evaluate_braking_snapshots import projected_frozen_model
    from Interaction.rl_braking_env import RLBrakeConfig, RLBrakingEnv, sample_scenario

    start = time.monotonic()
    torch.set_num_threads(config.threads)
    torch.use_deterministic_algorithms(True)
    set_random_seed(config.seed, using_cuda=False)
    random.seed(config.seed)
    raw_report = args.frozen_report.read_bytes()
    frozen_report = json.loads(raw_report)
    nominal_model = projected_frozen_model(frozen_report, np.array([0.0, 1.0]))
    env_config = RLBrakeConfig()
    rng = np.random.default_rng(config.eval_seed)
    scenarios = [sample_scenario(rng, nominal_model, randomize=True,
                                name=f"development_{index:04d}")
                 for index in range(config.eval_count)]
    scenario_payload = [asdict(scenario) for scenario in scenarios]
    code_hashes = source_hashes()
    provenance = dict(
        schema_version=1, offline_only=True, flight_commands_generated=False,
        calibration_written=False, final_test_evaluated=False,
        created_utc=datetime.now(timezone.utc).isoformat(),
        training_config=asdict(config), env_config=asdict(env_config),
        ppo_settings=PPO_SETTINGS, policy="MlpPolicy", device="cpu",
        seed_protocol=dict(training_env_seeds=list(range(config.seed, config.seed + config.n_envs)),
                           development_scenario_seed=config.eval_seed,
                           development_episode_seeds=list(range(config.eval_seed, config.eval_seed + config.eval_count))),
        validation_interval_steps=VALIDATION_INTERVAL,
        checkpoint_selection="lowest mean loss on fixed development scenarios only",
        frozen_model=asdict(nominal_model),
        frozen_parameters_sha256=canonical_sha256(asdict(nominal_model)),
        frozen_report=dict(path=str(args.frozen_report.resolve()),
                           sha256=hashlib.sha256(raw_report).hexdigest()),
        source_hashes=code_hashes, source_tree_sha256=canonical_sha256(code_hashes),
        development_scenarios_sha256=canonical_sha256(scenario_payload),
        versions=package_versions(), platform=platform.platform(),
        executable=sys.executable, torch_deterministic_algorithms=True,
        observation_normalization="fixed environment scaling; no VecNormalize",
        training_scenario_audit="training_scenarios.jsonl; one reset record per environment episode, including exact case and post-reset RNG state",
        reproducibility_note="Repeatable seeds/configuration on this software/hardware stack; cross-platform bit identity is not guaranteed.",
        resume=(None if args.resume is None else dict(
            path=str(args.resume.resolve()), sha256=sha256_file(args.resume),
            mode="checkpoint and optimizer warm start; RNG/episode state reset, not exact continuation")),
    )
    provenance["configuration_sha256"] = canonical_sha256(dict(
        training=asdict(config), environment=asdict(env_config),
        ppo=PPO_SETTINGS, frozen_model=asdict(nominal_model),
    ))
    args.output.mkdir(parents=True, exist_ok=False)
    write_json(args.output / "config.json", provenance)
    write_json(args.output / "frozen_report.json", frozen_report)
    write_json(args.output / "development_scenarios.json", scenario_payload)
    audit_path = args.output / "training_scenarios.jsonl"
    audit_stream = audit_path.open("w", buffering=1)
    audit_episode_counts = [0] * config.n_envs

    class AuditedTrainingEnv(RLBrakingEnv):
        def __init__(self, index):
            self.audit_index = index
            super().__init__(nominal_model, config=env_config, randomize=True)

        def reset(self, *, seed=None, options=None):
            result = super().reset(seed=seed, options=options)
            episode = audit_episode_counts[self.audit_index]
            audit_episode_counts[self.audit_index] += 1
            audit_stream.write(json.dumps(dict(
                env_index=self.audit_index, episode_index=episode,
                initial_env_seed=config.seed + self.audit_index,
                reset_seed=seed, scenario=asdict(self.scenario),
                rng_bit_generator=type(self.np_random.bit_generator).__name__,
                rng_state_after_reset=self.np_random.bit_generator.state,
            ), sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n")
            return result

    env = None
    try:
        env = DummyVecEnv([lambda index=index: AuditedTrainingEnv(index)
                           for index in range(config.n_envs)])
        env.seed(config.seed)
        kwargs = dict(PPO_SETTINGS, learning_rate=config.learning_rate,
                      seed=config.seed, device="cpu", verbose=0)
        if args.resume is None:
            model = PPO("MlpPolicy", env, **kwargs)
        else:
            model = PPO.load(args.resume, env=env, **kwargs)
        model.set_logger(configure(str(args.output / "training_scalars"), ["csv", "json"]))
        initial_steps = int(model.num_timesteps)
        provenance.update(
            initial_model_timesteps=initial_steps,
            observation_shape=list(env.observation_space.shape),
            action_space=str(env.action_space),
            rollout_size=config.n_envs * PPO_SETTINGS["n_steps"],
        )
        write_json(args.output / "config.json", provenance)
        history = []
        best = {"loss": math.inf, "timesteps": initial_steps}
        last_print = [0.0]

        def progress(message, force=False):
            now = time.monotonic()
            if force or now - last_print[0] >= 25.0:
                event = dict(elapsed_s=now - start, model_timesteps=int(model.num_timesteps),
                             training_steps=int(model.num_timesteps) - initial_steps, message=message)
                print(json.dumps(event, sort_keys=True), flush=True)
                with (args.output / "progress.jsonl").open("a") as stream:
                    stream.write(json.dumps(event, sort_keys=True) + "\n")
                last_print[0] = now

        def save_checkpoint(name, evaluation):
            assert_source_stable(code_hashes)
            audit_stream.flush()
            path = args.output / f"model_{name}.zip"
            model.save(path)
            write_json(args.output / f"model_{name}.metadata.json", dict(
                **provenance, checkpoint_sha256=sha256_file(path),
                checkpoint_file=path.name, checkpoint_timesteps=int(model.num_timesteps),
                training_scenario_audit_prefix_bytes=audit_path.stat().st_size,
                training_scenario_audit_prefix_sha256=sha256_file(audit_path),
                training_episode_counts=audit_episode_counts.copy(),
                elapsed_s=time.monotonic() - start,
                development_evaluation={key: value for key, value in evaluation.items() if key != "episodes"},
            ))

        def validate(label):
            progress(f"development validation started: {label}", force=True)
            evaluation = evaluate_policy(
                model, nominal_model, env_config, scenarios, seed=config.eval_seed,
                progress=lambda done, total: progress(f"development validation {done}/{total}"),
            )
            step = int(model.num_timesteps)
            evaluation.update(label=label, model_timesteps=step,
                              training_steps=step - initial_steps, elapsed_s=time.monotonic() - start)
            (args.output / "development_evaluations").mkdir(exist_ok=True)
            write_json(args.output / "development_evaluations" / f"{step:012d}_{label}.json", evaluation)
            scalar = {key: value for key, value in evaluation.items() if key not in ("episodes", "metric_means")}
            scalar.update({f"metric/{key}": value for key, value in evaluation["metric_means"].items()})
            history.append(scalar)
            with (args.output / "development_history.jsonl").open("a") as stream:
                stream.write(json.dumps(scalar, sort_keys=True, allow_nan=False) + "\n")
            fields = sorted(set().union(*(row.keys() for row in history)))
            with (args.output / "development_history.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows(history)
            if evaluation["mean_loss"] < best["loss"]:
                best.update(loss=evaluation["mean_loss"], timesteps=step)
                save_checkpoint("best", evaluation)
            progress(f"development validation done: {label}, mean_loss={evaluation['mean_loss']:.6f}, best={best['loss']:.6f}", force=True)
            return evaluation

        baseline = validate("resumed_baseline" if args.resume else "untrained_baseline")
        save_checkpoint("initial", baseline)

        class DevelopmentCallback(BaseCallback):
            def __init__(self):
                super().__init__()
                self.next_validation = initial_steps + VALIDATION_INTERVAL

            def _on_step(self):
                progress("training")
                if self.num_timesteps >= self.next_validation:
                    validate("periodic")
                    self.next_validation = self.num_timesteps + VALIDATION_INTERVAL
                return True

        progress("training started", force=True)
        model.learn(total_timesteps=config.timesteps, callback=DevelopmentCallback(),
                    reset_num_timesteps=False, log_interval=1, progress_bar=False)
        model.logger.record("time/total_timesteps", int(model.num_timesteps))
        model.logger.dump(step=int(model.num_timesteps))
        assert_source_stable(code_hashes)
        final = validate("final_development")
        save_checkpoint("final", final)
        statistics = dict(
            offline_only=True, final_test_evaluated=False,
            requested_training_steps=config.timesteps,
            actual_training_steps=int(model.num_timesteps) - initial_steps,
            model_timesteps=int(model.num_timesteps),
            wall_time_s=time.monotonic() - start,
            baseline_mean_loss=baseline["mean_loss"], final_mean_loss=final["mean_loss"],
            best_mean_loss=best["loss"], best_model_timesteps=best["timesteps"],
            source_stable_through_completion=True,
            training_episode_counts=audit_episode_counts.copy(),
            training_scenario_audit_sha256=sha256_file(audit_path),
            selection_split="fixed development scenarios; not an independent test result",
            evaluations=history,
        )
        write_json(args.output / "statistics.json", statistics)
        progress(f"complete; checkpoints: {args.output.resolve()}", force=True)
        return statistics
    except BaseException as exc:
        write_json(args.output / "failure.json", dict(
            error_type=type(exc).__name__, error=str(exc), elapsed_s=time.monotonic() - start,
            offline_only=True, final_test_evaluated=False,
        ))
        raise
    finally:
        if env is not None:
            env.close()
        audit_stream.close()


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        config = validate_args(args)
    except (TypeError, ValueError) as exc:
        parser.error(str(exc))
    try:
        run_training(args, config)
    except ModuleNotFoundError as exc:
        parser.error(f"optional offline RL dependency missing: {exc.name}; use the RL requirements environment")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
