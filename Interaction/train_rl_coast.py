"""Versioned, simulator-only PPO training for a fixed coast endpoint.

Action 0 retains -20 degree braking; action 1 irreversibly hands control to an
assumed position-PD surrogate. All episodes use the same physical-time horizon.
The surrogate is not an identified flight controller. No aircraft connection,
runtime integration, calibration write or final-test evaluation occurs here.

This separate v2 entry point preserves v1 source and checkpoint semantics. Run
``python -m Interaction.train_rl_coast --help`` without importing optional RL
or flight dependencies. Selection uses fixed development cases only.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
import time

import numpy as np

from Interaction.train_rl_braking import (
    SOURCE_FILES as V1_SOURCE_FILES, TrainingConfig, canonical_sha256,
    evaluate_policy, package_versions, sha256_file, write_json,
)


TASK_VERSION = "coast_v2"
OBSERVATION_VERSION = "coast_v2_v1prefix114_target_time3"
ACTION_VERSION = "brake20_or_latched_position_v2"
PPO_SETTINGS = dict(
    n_steps=512, batch_size=256, n_epochs=10, gamma=1.0, gae_lambda=1.0,
    ent_coef=0.005, policy_kwargs={"net_arch": [64, 64]},
)
VALIDATION_INTERVAL = 32768
SOURCE_FILES = (*V1_SOURCE_FILES, "rl_coast_env.py", "train_rl_coast.py")
PREVIOUSLY_INSPECTED_SEEDS = (9001, 491723)
RESUME_MATCH_FIELDS = (
    "schema_version", "task_version", "observation_version", "action_version",
    "env_config", "env_config_sha256", "frozen_parameters_sha256",
    "development_scenarios_sha256", "source_hashes", "source_tree_sha256",
    "ppo_settings",
)


@dataclass(frozen=True)
class CoastTrainingConfig(TrainingConfig):
    seed: int = 41
    timesteps: int = 32768
    eval_seed: int = 19001

    def __post_init__(self):
        super().__post_init__()
        for old_seed in PREVIOUSLY_INSPECTED_SEEDS:
            if (self.seed <= old_seed < self.seed + self.n_envs
                    or self.eval_seed <= old_seed < self.eval_seed + self.eval_count):
                raise ValueError(f"seed {old_seed} belongs to inspected v1 data; choose a new v2 seed stream")


def source_hashes():
    directory = Path(__file__).resolve().parent
    return {f"Interaction/{name}": sha256_file(directory / name) for name in SOURCE_FILES}


def assert_source_stable(expected):
    if source_hashes() != expected:
        raise RuntimeError("coast training source changed during this run; discard this run and restart from frozen code")


def checkpoint_metadata_path(checkpoint):
    return Path(checkpoint).with_suffix(".metadata.json")


def validate_resume_checkpoint(checkpoint, expected):
    """Reject semantically incompatible or changed-source checkpoint warm starts.

    Training seed, additional budget and learning rate may change between
    predeclared development rounds; environment, task, model, development set
    and network/PPO semantics must not. This is not a bit-exact RNG continuation.
    """
    checkpoint = Path(checkpoint)
    metadata_path = checkpoint_metadata_path(checkpoint)
    if not checkpoint.is_file() or not metadata_path.is_file():
        raise ValueError("resume requires an existing v2 checkpoint and matching .metadata.json sidecar")
    try:
        metadata = json.loads(metadata_path.read_text())
    except (OSError, ValueError) as exc:
        raise ValueError("resume sidecar is not readable JSON") from exc
    if not isinstance(metadata, dict):
        raise ValueError("resume sidecar must be a metadata object")
    for key in RESUME_MATCH_FIELDS:
        if key not in expected or key not in metadata or metadata[key] != expected[key]:
            raise ValueError(f"resume metadata mismatch: {key}; v1 or altered v2 checkpoints cannot be resumed")
    if metadata.get("checkpoint_sha256") != sha256_file(checkpoint):
        raise ValueError("resume checkpoint content does not match its recorded SHA-256")
    return metadata


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frozen-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True,
                        help="new directory only; never overwrite an existing v1/v2 run")
    parser.add_argument("--seed", type=int, default=41)
    parser.add_argument("--timesteps", type=int, default=32768,
                        help="additional transitions, rounded up to a complete PPO rollout")
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--learning-rate", type=float, default=0.0003)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--eval-count", type=int, default=64)
    parser.add_argument("--eval-seed", type=int, default=19001)
    parser.add_argument("--resume", type=Path,
                        help="matching v2 ZIP plus sidecar only; fresh RNG/episodes, not exact continuation")
    return parser


def validate_args(args):
    config = CoastTrainingConfig(**{
        key: getattr(args, key) for key in CoastTrainingConfig.__dataclass_fields__})
    if args.output.exists():
        raise ValueError("output already exists; choose a new directory")
    if not args.frozen_report.is_file():
        raise ValueError("frozen report must be an existing file")
    if args.resume is not None:
        if not args.resume.is_file() or args.resume.suffix != ".zip":
            raise ValueError("resume must be an existing v2 .zip checkpoint")
        if not checkpoint_metadata_path(args.resume).is_file():
            raise ValueError("resume requires its matching v2 .metadata.json sidecar")
    return config


def run_training(args, config):
    """Train a versioned coast policy without touching v1 code or final test cases."""
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.logger import configure
    from stable_baselines3.common.utils import set_random_seed
    from stable_baselines3.common.vec_env import DummyVecEnv

    from Interaction.evaluate_braking_snapshots import projected_frozen_model
    from Interaction.rl_coast_env import (
        CoastBrakeConfig, CoastBrakingEnv, sample_coast_scenario,
        TASK_VERSION as ENV_TASK_VERSION, OBSERVATION_VERSION as ENV_OBSERVATION_VERSION,
        ACTION_VERSION as ENV_ACTION_VERSION,
    )

    started = time.monotonic()
    if (ENV_TASK_VERSION, ENV_OBSERVATION_VERSION, ENV_ACTION_VERSION) != (
            TASK_VERSION, OBSERVATION_VERSION, ACTION_VERSION):
        raise ValueError("coast environment and trainer version contracts disagree")
    torch.set_num_threads(config.threads)
    torch.use_deterministic_algorithms(True)
    set_random_seed(config.seed, using_cuda=False)
    raw = args.frozen_report.read_bytes()
    frozen_report = json.loads(raw)
    nominal_model = projected_frozen_model(frozen_report, np.array([0.0, 1.0]))
    env_config = CoastBrakeConfig()
    rng = np.random.default_rng(config.eval_seed)
    scenarios = [sample_coast_scenario(rng, nominal_model, randomize=True,
                                      name=f"coast_development_{index:04d}")
                 for index in range(config.eval_count)]
    serialized_scenarios = [asdict(scenario) for scenario in scenarios]
    hashes = source_hashes()
    provenance = dict(
        schema_version=2, task_version=TASK_VERSION,
        observation_version=OBSERVATION_VERSION, action_version=ACTION_VERSION,
        offline_only=True, flight_commands_generated=False, flight_validated=False,
        calibration_written=False, final_test_evaluated=False,
        position_controller_is_assumed_surrogate_not_identified=True,
        created_utc=datetime.now(timezone.utc).isoformat(),
        training_config=asdict(config), env_config=asdict(env_config),
        env_config_sha256=canonical_sha256(asdict(env_config)),
        ppo_settings=PPO_SETTINGS, policy="MlpPolicy", device="cpu",
        seed_protocol=dict(
            training_env_seeds=list(range(config.seed, config.seed + config.n_envs)),
            development_scenario_seed=config.eval_seed,
            development_episode_seeds=list(range(config.eval_seed, config.eval_seed + config.eval_count)),
            rejected_inspected_v1_seeds=list(PREVIOUSLY_INSPECTED_SEEDS)),
        validation_interval_steps=VALIDATION_INTERVAL,
        checkpoint_selection="lowest mean loss on fixed v2 development scenarios only",
        frozen_model=asdict(nominal_model),
        frozen_parameters_sha256=canonical_sha256(asdict(nominal_model)),
        frozen_report=dict(path=str(args.frozen_report.resolve()), sha256=hashlib.sha256(raw).hexdigest()),
        source_hashes=hashes, source_tree_sha256=canonical_sha256(hashes),
        development_scenarios_sha256=canonical_sha256(serialized_scenarios),
        versions=package_versions(), platform=platform.platform(), executable=sys.executable,
        torch_deterministic_algorithms=True,
        observation_normalization="fixed v2 environment scaling and exact v1 prefix; no VecNormalize",
        training_scenario_audit="training_scenarios.jsonl: exact scenario, environment/episode index, seed and post-reset RNG state",
        reproducibility_note="Seeded CPU run on recorded software/hardware; cross-platform bit identity is not guaranteed.",
    )
    provenance["configuration_sha256"] = canonical_sha256(dict(
        training=asdict(config), environment=asdict(env_config),
        ppo=PPO_SETTINGS, frozen_model=asdict(nominal_model),
        task_version=TASK_VERSION, action_version=ACTION_VERSION,
        observation_version=OBSERVATION_VERSION))
    resumed_metadata = None
    if args.resume is not None:
        resumed_metadata = validate_resume_checkpoint(args.resume, provenance)
    provenance["resume"] = None if args.resume is None else dict(
        path=str(args.resume.resolve()), sha256=sha256_file(args.resume),
        metadata_sha256=sha256_file(checkpoint_metadata_path(args.resume)),
        parent_checkpoint_timesteps=resumed_metadata.get("checkpoint_timesteps"),
        mode="compatible v2 checkpoint/optimizer warm start; fresh RNG/episode stream")

    args.output.mkdir(parents=True, exist_ok=False)
    for filename in ("config.json", "manifest.json"):
        write_json(args.output / filename, provenance)
    write_json(args.output / "frozen_report.json", frozen_report)
    write_json(args.output / "development_scenarios.json", serialized_scenarios)
    audit_path = args.output / "training_scenarios.jsonl"
    audit_stream = audit_path.open("w", buffering=1)
    episode_counts = [0] * config.n_envs

    class AuditedCoastEnv(CoastBrakingEnv):
        def __init__(self, index):
            self.audit_index = index
            super().__init__(nominal_model, config=env_config, randomize=True)

        def reset(self, *, seed=None, options=None):
            result = super().reset(seed=seed, options=options)
            index = self.audit_index
            episode_index = episode_counts[index]
            episode_counts[index] += 1
            audit_stream.write(json.dumps(dict(
                task_version=TASK_VERSION, env_index=index, episode_index=episode_index,
                initial_env_seed=config.seed + index, reset_seed=seed,
                scenario=asdict(self.scenario),
                rng_bit_generator=type(self.np_random.bit_generator).__name__,
                rng_state_after_reset=self.np_random.bit_generator.state,
            ), sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n")
            return result

    env = None
    try:
        env = DummyVecEnv([lambda index=index: AuditedCoastEnv(index) for index in range(config.n_envs)])
        env.seed(config.seed)
        kwargs = dict(PPO_SETTINGS, learning_rate=config.learning_rate,
                      seed=config.seed, device="cpu", verbose=0)
        model = (PPO("MlpPolicy", env, **kwargs) if args.resume is None
                 else PPO.load(args.resume, env=env, **kwargs))
        model.set_logger(configure(str(args.output / "training_scalars"), ["csv", "json"]))
        initial_steps = int(model.num_timesteps)
        provenance.update(initial_model_timesteps=initial_steps,
                          observation_shape=list(env.observation_space.shape),
                          action_space=str(env.action_space),
                          rollout_size=config.n_envs * PPO_SETTINGS["n_steps"])
        for filename in ("config.json", "manifest.json"):
            write_json(args.output / filename, provenance)
        history = []
        best = dict(loss=math.inf, timesteps=initial_steps)
        last_print = [0.0]

        def progress(message, force=False):
            now = time.monotonic()
            if force or now - last_print[0] >= 25.0:
                event = dict(task_version=TASK_VERSION, elapsed_s=now - started,
                             model_timesteps=int(model.num_timesteps),
                             training_steps=int(model.num_timesteps) - initial_steps, message=message)
                print(json.dumps(event, sort_keys=True), flush=True)
                with (args.output / "progress.jsonl").open("a") as stream:
                    stream.write(json.dumps(event, sort_keys=True) + "\n")
                last_print[0] = now

        def save_checkpoint(name, evaluation):
            assert_source_stable(hashes)
            audit_stream.flush()
            path = args.output / f"model_{name}.zip"
            model.save(path)
            write_json(checkpoint_metadata_path(path), dict(
                **provenance, checkpoint_file=path.name, checkpoint_sha256=sha256_file(path),
                checkpoint_timesteps=int(model.num_timesteps), elapsed_s=time.monotonic() - started,
                training_scenario_audit_prefix_bytes=audit_path.stat().st_size,
                training_scenario_audit_prefix_sha256=sha256_file(audit_path),
                training_episode_counts=episode_counts.copy(),
                development_evaluation={key: value for key, value in evaluation.items() if key != "episodes"}))

        def validate(label):
            progress(f"coast development validation started: {label}", force=True)
            evaluation = evaluate_policy(
                model, nominal_model, env_config, scenarios, seed=config.eval_seed,
                env_factory=CoastBrakingEnv,
                progress=lambda done, total: progress(f"coast development validation {done}/{total}"))
            if any(row["truncated"] for row in evaluation["episodes"]):
                raise ValueError("development episode truncated before the common physical-time horizon")
            step = int(model.num_timesteps)
            evaluation.update(task_version=TASK_VERSION, label=label, model_timesteps=step,
                              training_steps=step - initial_steps, elapsed_s=time.monotonic() - started)
            evaluation_directory = args.output / "development_evaluations"
            evaluation_directory.mkdir(exist_ok=True)
            write_json(evaluation_directory / f"{step:012d}_{label}.json", evaluation)
            scalar = {key: value for key, value in evaluation.items() if key not in ("episodes", "metric_means")}
            scalar.update({f"metric/{key}": value for key, value in evaluation["metric_means"].items()})
            history.append(scalar)
            with (args.output / "development_history.jsonl").open("a") as stream:
                stream.write(json.dumps(scalar, sort_keys=True, allow_nan=False) + "\n")
            with (args.output / "development_history.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=sorted(set().union(*(row.keys() for row in history))))
                writer.writeheader()
                writer.writerows(history)
            if evaluation["mean_loss"] < best["loss"]:
                best.update(loss=evaluation["mean_loss"], timesteps=step)
                save_checkpoint("best", evaluation)
            progress(f"coast development validation done: {label}, mean_loss={evaluation['mean_loss']:.6f}, best={best['loss']:.6f}", force=True)
            return evaluation

        baseline = validate("resumed_baseline" if args.resume else "untrained_baseline")
        save_checkpoint("initial", baseline)

        class DevelopmentCallback(BaseCallback):
            def __init__(self):
                super().__init__()
                self.next_validation = initial_steps + VALIDATION_INTERVAL

            def _on_step(self):
                progress("coast training")
                if self.num_timesteps >= self.next_validation:
                    validate("periodic")
                    self.next_validation = self.num_timesteps + VALIDATION_INTERVAL
                return True

        progress("coast training started", force=True)
        model.learn(total_timesteps=config.timesteps, callback=DevelopmentCallback(),
                    reset_num_timesteps=False, log_interval=1, progress_bar=False)
        model.logger.record("time/total_timesteps", int(model.num_timesteps))
        model.logger.dump(step=int(model.num_timesteps))
        assert_source_stable(hashes)
        final = validate("final_development")
        save_checkpoint("final", final)
        statistics = dict(
            schema_version=2, task_version=TASK_VERSION, observation_version=OBSERVATION_VERSION,
            action_version=ACTION_VERSION, offline_only=True, final_test_evaluated=False,
            position_controller_is_assumed_surrogate_not_identified=True,
            requested_training_steps=config.timesteps,
            actual_training_steps=int(model.num_timesteps) - initial_steps,
            model_timesteps=int(model.num_timesteps), wall_time_s=time.monotonic() - started,
            baseline_mean_loss=baseline["mean_loss"], final_mean_loss=final["mean_loss"],
            best_mean_loss=best["loss"], best_model_timesteps=best["timesteps"],
            source_stable_through_completion=True, training_episode_counts=episode_counts.copy(),
            training_scenario_audit_sha256=sha256_file(audit_path),
            selection_split="fixed v2 development cases only; not independent test evidence",
            evaluations=history)
        write_json(args.output / "statistics.json", statistics)
        progress(f"coast training complete; checkpoints: {args.output.resolve()}", force=True)
        return statistics
    except BaseException as exc:
        write_json(args.output / "failure.json", dict(
            task_version=TASK_VERSION, error_type=type(exc).__name__, error=str(exc),
            elapsed_s=time.monotonic() - started, offline_only=True, final_test_evaluated=False))
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
        run_training(args, config)
    except (TypeError, ValueError) as exc:
        parser.error(str(exc))
    except ModuleNotFoundError as exc:
        parser.error(f"optional offline RL dependency missing: {exc.name}; use the RL requirements environment")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
