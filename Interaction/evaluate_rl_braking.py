"""Independent paired evaluation of simulator-trained braking PPO policies.

No flight connection, calibration write, fitting, or training. Final synthetic
test populations must not be used to select checkpoints or change rewards.
Previously inspected measured-initial-state tests are regression evidence only.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, replace
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from Interaction.evaluate_braking_snapshots import projected_frozen_model
from Interaction.offline_braking_selector import FrozenTiltModel
from Interaction.predictive_brake_release import BrakeReleaseConfig, PredictiveBrakeRelease
from Interaction.rl_braking_env import BrakingScenario, RLBrakeConfig, RLBrakingEnv, sample_scenario
from Interaction.validate_predictive_brake_release import observed_seeds, STRESS_CASES
from Interaction.train_rl_braking import source_hashes, sha256_file


BASELINES = ('fixed_240ms', 'predictive_margin_080', 'predictive_margin_025')


@dataclass(frozen=True)
class EvaluationCase:
    name: str
    population: str
    scenario: BrakingScenario
    nominal_model: FrozenTiltModel


def make_synthetic_cases(nominal_model, count=256, seed=491723, population='held_out_synthetic'):
    if count <= 0 or seed < 0:
        raise ValueError('count must be positive and population seed nonnegative')
    rng = np.random.default_rng(seed)
    return [EvaluationCase(f'{population}:{seed}:{i}', population,
                sample_scenario(rng, nominal_model, randomize=True, name=f'{seed}:{i}'), nominal_model)
            for i in range(count)]


def make_observed_cases(paths, frozen_report, exclusions=()):
    cases, sources = [], []
    applied = []
    for path in paths:
        path = Path(path)
        raw = path.read_bytes()
        excluded = [int(x.rsplit(':', 1)[1]) for x in exclusions if x.rsplit(':', 1)[0] == path.name]
        applied.extend(f'{path.name}:{i}' for i in excluded)
        sources.append(dict(path=str(path.resolve()), sha256=hashlib.sha256(raw).hexdigest()))
        for seed in observed_seeds(json.loads(raw), path.name, frozen_report, excluded):
            nominal = seed['model']
            for stress in STRESS_CASES:
                model = replace(nominal,
                    motion_gain=nominal.motion_gain*stress.motion_gain_multiplier,
                    command_gain=nominal.command_gain*stress.command_gain_multiplier,
                    wn_rad_s=nominal.wn_rad_s*stress.wn_multiplier,
                    zeta=nominal.zeta*stress.zeta_multiplier,
                    delay_s=nominal.delay_s+stress.extra_model_delay_s)
                name = f"{seed['name']}:{stress.name}"
                scenario = BrakingScenario(name=name, model=model, snapshot=seed['snapshot'],
                    command_history=tuple(seed['command_history']),
                    first_decision_s=seed['first_brake_after_snapshot_s'],
                    measurement_delay_s=stress.measurement_delay_s,
                    transport_delay_s=stress.transport_delay_s,
                    velocity_bias_m_s=stress.velocity_bias_m_s,
                    tilt_bias_deg=stress.tilt_bias_deg, rate_bias_deg_s=stress.rate_bias_deg_s)
                population = 'observed_initial_nominal_regression' if stress.name == 'nominal' else 'observed_initial_stress_regression'
                cases.append(EvaluationCase(name, population, scenario, nominal))
    if sorted(applied) != sorted(exclusions):
        raise ValueError('exclusion did not match a supplied observed log')
    return cases, sources


def run_episode(case, method, config=None, policy=None, *, trace=False):
    config = config or RLBrakeConfig()
    if method == 'fixed_240ms' and (config.max_brake_s < .24 or not np.isclose(
            .24/config.control_period_s, round(.24/config.control_period_s), rtol=0, atol=1e-9)):
        raise ValueError('fixed 240 ms baseline needs an exact tick boundary and sufficient maximum duration')
    env = RLBrakingEnv(case.nominal_model, config=config, randomize=False)
    observation, info = env.reset(seed=0, options={'scenario': case.scenario})
    predictor = None
    if method in BASELINES[1:]:
        margin = {'predictive_margin_080': .08, 'predictive_margin_025': .025}[method]
        predictor = PredictiveBrakeRelease(case.nominal_model, BrakeReleaseConfig(
            release_speed_margin_m_s=margin, control_period_s=config.control_period_s,
            max_brake_duration_s=config.max_brake_s))
    elif method != 'fixed_240ms' and policy is None:
        raise ValueError('a learned method requires an explicit policy')
    rewards = 0.
    decisions = []
    while True:
        now = info['decision_time_s']
        if method == 'fixed_240ms':
            action = int(now-env.first_decision_s >= .24-1e-10)
            reason = 'fixed_duration'
        elif predictor is not None:
            decision = predictor.update(info['snapshot'], info['command_history'], decision_time_s=now)
            action = int(decision['released'])
            reason = decision['reason']
        else:
            action, _ = policy.predict(observation, deterministic=True)
            action = int(np.asarray(action).item())
            reason = 'ppo_deterministic'
        if trace:
            decisions.append(dict(time_s=now, action=action, reason=reason))
        observation, reward, terminated, truncated, info = env.step(action)
        rewards += float(reward)
        if terminated or truncated:
            if truncated:
                raise ValueError('evaluation unexpectedly truncated before complete tail')
            break
    row = dict(case=case.name, population=case.population, method=method,
               loss=float(info['loss']), reward=float(rewards), **info['metrics'])
    row['mode'] = method
    row['last_decision_reason'] = reason
    trajectory = None
    if trace:
        trajectory = dict(case=case.name, method=method, first_decision_s=env.first_decision_s,
                          states=[asdict(x) for x in env.plant.snapshot_history[::10]],
                          commands=list(env.plant.command_history), decisions=decisions)
    env.close()
    return row, trajectory


def summarize(rows):
    aggregates = []
    for population, method in sorted({(row['population'], row['method']) for row in rows}):
        group = [row for row in rows if row['population'] == population and row['method'] == method]
        losses = np.array([row['loss'] for row in group])
        aggregates.append(dict(population=population, method=method, count=len(group),
            mean_loss=float(np.mean(losses)), p95_loss=float(np.quantile(losses, .95)),
            reversal_count=sum(not r['no_reverse_in_simulation'] for r in group),
            capture_count=sum(r['low_speed_level_capture_state_in_simulation'] for r in group),
            near_stationary_count=sum(r['near_stationary_in_simulation'] for r in group),
            maximum_rollback_m=max(r['max_rollback_m'] for r in group),
            mean_rollback_m=float(np.mean([r['max_rollback_m'] for r in group])),
            mean_absolute_tail_velocity_m_s=float(np.mean([abs(r['terminal_mean_velocity_m_s']) for r in group])),
            mean_release_after_brake_s=float(np.mean([r['release_after_brake_s'] for r in group])),
            max_forward_displacement_m=max(r['max_forward_displacement_m'] for r in group)))
    return aggregates


def markdown(report):
    lines = ['# Simulator-trained PPO: paired offline evaluation', '',
        '**No flight validation. No position target or position-controller handoff.**', '',
        'All methods receive identical initial conditions, hidden plant parameters, measurement bias/delay and command delay. '
        'Every method is observed for 1.5 s after its level command. Lower scalar loss is not by itself a safety result.', '',
        '| Population | Method | N | Mean loss | Reversals | Near stationary | Low-speed level | Worst rollback cm | Mean abs. tail speed m/s |',
        '|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for row in report['aggregate']:
        lines.append(f"| {row['population']} | {row['method']} | {row['count']} | {row['mean_loss']:.4f} | "
            f"{row['reversal_count']} | {row['near_stationary_count']} | {row['capture_count']} | "
            f"{100*row['maximum_rollback_m']:.2f} | {row['mean_absolute_tail_velocity_m_s']:.4f} |")
    lines += ['', '## Interpretation limits', '',
        '- Held-out synthetic means new random draws from the training distribution, not new measured flight data.',
        '- Measured initial-state regression cases were inspected before training; later motion is independently simulated.',
        '- The 0.08 m/s predictor has a more conservative velocity margin than the PPO loss target of 0.025 m/s; the 0.025 predictor is also reported.',
        '- Loss combines speed error, reverse velocity, rollback and duration. Inspect each component: minimizing loss does not guarantee non-reversal.',
        '- Near stationary uses a finite last-100-ms speed tolerance, not indefinite hover or accurate stopping position.',
        '- The policy cannot accelerate forward or change a target. Some states may not be recoverable with this action set.',
        '- RL is trained by interaction with a simulator, offline from the aircraft. This is not dataset-only offline RL.']
    return '\n'.join(lines)+'\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frozen-report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--policy', action='append', default=[], metavar='LABEL=CHECKPOINT')
    parser.add_argument('--population-seed', type=int, default=491723)
    parser.add_argument('--count', type=int, default=256)
    parser.add_argument('--population', choices=['held_out_synthetic', 'development'], default='held_out_synthetic')
    parser.add_argument('--observed-log', action='append', default=[], type=Path)
    parser.add_argument('--exclude', action='append', default=[])
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('output exists; use a new directory')
    raw = args.frozen_report.read_bytes()
    frozen = json.loads(raw)
    nominal = projected_frozen_model(frozen, [0., 1.])
    cases = make_synthetic_cases(nominal, args.count, args.population_seed, args.population)
    observed, sources = make_observed_cases(args.observed_log, frozen, args.exclude)
    cases.extend(observed)
    policies, checkpoint_info = {}, []
    for specification in args.policy:
        label, path = specification.split('=', 1)
        if not label or label in BASELINES or label in policies:
            parser.error('policy labels must be unique and not baseline names')
        from stable_baselines3 import PPO
        import torch
        torch.set_num_threads(1)
        policies[label] = PPO.load(path, device='cpu')
        checkpoint_info.append(dict(label=label, path=str(Path(path).resolve()),
                                    sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest()))
    manifest = dict(frozen_report=str(args.frozen_report.resolve()),
        frozen_report_sha256=hashlib.sha256(raw).hexdigest(), population_seed=args.population_seed,
        population=args.population, config=asdict(RLBrakeConfig()),
        cases=[asdict(case) for case in cases], source_logs=sources, exclusions=args.exclude,
        checkpoints=checkpoint_info,
        source_sha256={**source_hashes(), 'Interaction/evaluate_rl_braking.py':sha256_file(Path(__file__))})
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    if args.prepare_only:
        print(args.output/'manifest.json')
        return 0
    started = time.perf_counter()
    rows, traces = [], []
    for method in [*BASELINES, *policies]:
        for index, case in enumerate(cases):
            row, trajectory = run_episode(case, method, policy=policies.get(method), trace=index < 2)
            rows.append(row)
            if trajectory:
                traces.append(trajectory)
            if (index+1) % 64 == 0:
                print(f'{method}: {index+1}/{len(cases)}', flush=True)
        print(f'{method}: complete', flush=True)
    report = dict(offline_only=True, flight_validated=False, position_target_validated=False,
                  elapsed_s=time.perf_counter()-started, rows=rows, aggregate=summarize(rows))
    current_hashes = {**source_hashes(), 'Interaction/evaluate_rl_braking.py':sha256_file(Path(__file__))}
    if current_hashes != manifest['source_sha256']:
        raise RuntimeError('evaluation source changed during the run; do not interpret this output as a frozen experiment')
    for filename, data in [('report.json',report), ('trajectories.json',traces)]:
        (args.output/filename).write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')
    (args.output/'README.md').write_text(markdown(report))
    print(args.output/'README.md')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
