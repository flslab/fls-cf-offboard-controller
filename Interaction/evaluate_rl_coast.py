"""Paired OFFLINE evaluation of fixed-target braking and assumed position capture.

V1 LEVEL decisions are explicitly adapted to direct POSITION takeover here.
These are not native-v1 trajectory results or a validated firmware simulation.
No training, flight, calibration writes, target relatching or hidden-model oracle.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np

from Interaction.evaluate_braking_snapshots import projected_frozen_model
from Interaction.offline_braking_selector import BrakingSnapshot, FrozenTiltModel
from Interaction.predictive_brake_release import BrakeReleaseConfig, PredictiveBrakeRelease
from Interaction.rl_braking_env import BrakingScenario, RLBrakeConfig
from Interaction.rl_coast_env import CoastBrakeConfig, CoastScenario, CoastBrakingEnv, sample_coast_scenario


BASELINES = ('immediate_position', 'fixed240ms_position',
             'predictive080_position_adapter', 'nominal_distance_gate')
V1_LABEL = 'v1_ppo_position_adapter'


@dataclass(frozen=True)
class CoastEvaluationCase:
    name: str
    population: str
    scenario: CoastScenario
    nominal_model: FrozenTiltModel


def make_cases(nominal_model, count=256, seed=804271, population='held_out_synthetic'):
    if isinstance(count, bool) or count <= 0 or seed < 0 or seed in (9001, 491723):
        raise ValueError('positive population count and fresh v2 seed required')
    rng = np.random.default_rng(seed)
    return [CoastEvaluationCase(f'{population}:{seed}:{i}', population,
            sample_coast_scenario(rng, nominal_model,
                name=f'coast_development_{i:04d}' if population=='development' else f'{seed}:{i}'), nominal_model)
            for i in range(count)]


def observed_regressions(path, nominal_model):
    """Reuse measured INITIAL states only, with explicitly synthetic target gaps.

The optional input is v1's provenance manifest, NOT its held-out outcomes. Only
the seven previously inspected nominal measured initial states are selected.
Targets are not reconstructed or misrepresented as actual coast endpoints.
"""
    raw = Path(path).read_bytes()
    source = json.loads(raw)
    rows = [r for r in source['cases'] if r['population']=='observed_initial_nominal_regression']
    if not rows:
        raise ValueError('regression manifest contains no observed nominal initial states')
    cases = []
    for record in rows:
        item = record['scenario']
        nominal = FrozenTiltModel(**record['nominal_model'])
        sign = -1 if nominal.projected_bias_rad*nominal_model.projected_bias_rad < 0 else 1
        braking = BrakingScenario(**{**item, 'model':FrozenTiltModel(**item['model']),
            'snapshot':BrakingSnapshot(**item['snapshot']),
            'command_history':tuple(tuple(p) for p in item['command_history'])})
        for gap in (.08,.16,.28):
            name = f"{record['name']}:assigned_gap_{gap:.2f}"
            scenario = CoastScenario(braking=replace(braking,name=name), target_distance_m=gap,
                                     direction_sign=sign)
            cases.append(CoastEvaluationCase(name,'observed_initial_synthetic_target_regression',
                                             scenario, nominal_model))
    return cases, dict(path=str(Path(path).resolve()),sha256=hashlib.sha256(raw).hexdigest(),
                       note='Measured initial states; assigned target gaps and assumed nominal position dynamics, not measured endpoints.')


def nominal_distance_action(info):
    """Declared target-aware heuristic, not a fitted position stopping envelope.

Use only measured state, target and clock. Nominal 0.10 s reaction allowance,
1 m/s^2 position deceleration and 2 cm allowance remain fixed for every plant.
"""
    state = info['snapshot']
    v = max(0., state.velocity_m_s)
    remaining = info['target_position_m'] - state.position_m
    age = max(0.,info['decision_time_s']-state.time_s)
    required = v*(.10+age) + v*v/(2.*1.) + .02
    return int(v <= .04 or remaining >= required)


def run_episode(case, method, config=None, policy=None, *, trace=False):
    config = config or CoastBrakeConfig()
    if method == 'fixed240ms_position' and (config.max_brake_s < .24 or not math.isclose(
            .24/config.control_period_s,round(.24/config.control_period_s),abs_tol=1e-9)):
        raise ValueError('fixed comparator needs an exact 240 ms tick and sufficient brake duration')
    env = CoastBrakingEnv(case.nominal_model,config=config,randomize=False)
    observation,info = env.reset(seed=0,options={'scenario':case.scenario})
    nominal = replace(case.nominal_model,
                      projected_bias_rad=case.scenario.direction_sign*case.nominal_model.projected_bias_rad)
    predictor = (PredictiveBrakeRelease(nominal,BrakeReleaseConfig(
        control_period_s=config.control_period_s,max_brake_duration_s=config.max_brake_s))
        if method=='predictive080_position_adapter' else None)
    if method not in BASELINES and policy is None:
        raise ValueError('learned method needs an explicit policy')
    decisions=[]
    total_reward=0.
    try:
        for _ in range(10001):
            now=info['decision_time_s']
            if method=='immediate_position':
                action=1
            elif method=='fixed240ms_position':
                action=int(now-env.first_decision_s >= .24-1e-10)
            elif method=='nominal_distance_gate':
                action=nominal_distance_action(info)
            elif predictor is not None:
                decision=predictor.update(info['snapshot'],info['command_history'],decision_time_s=now)
                action=int(decision['released'])
            else:
                view=observation[:114] if method==V1_LABEL else observation
                action,_=policy.predict(view,deterministic=True)
                action=int(np.asarray(action).item())
            if trace:
                decisions.append(dict(time_s=now,action=action,
                    measured_position_m=info['snapshot'].position_m,
                    measured_velocity_m_s=info['snapshot'].velocity_m_s,
                    target_position_m=info['target_position_m']))
            observation,reward,terminated,truncated,info=env.step(action)
            total_reward+=float(reward)
            if terminated or truncated:
                if truncated:
                    raise ValueError('full fixed physical horizon must terminate, not truncate')
                break
        else:
            raise RuntimeError('episode did not terminate')
        metrics=info['metrics']
        row=dict(case=case.name,population=case.population,method=method,
                 loss=float(info['loss']),reward=total_reward,**metrics)
        trajectory=None
        if trace:
            trajectory=dict(case=case.name,method=method,
                first_decision_s=env.first_decision_s,target_position_m=info['target_position_m'],
                states=[asdict(s) for s in env.plant.snapshot_history[::10]],
                commands=list(env.plant.command_history),decisions=decisions,
                position_decisions=[{**r,'snapshot':asdict(r['snapshot'])} for r in env.position_decisions])
        return row,trajectory
    finally:
        env.close()


def summarize(rows):
    groups=[]
    for population,method in sorted({(r['population'],r['method']) for r in rows}):
        r=[x for x in rows if (x['population'],x['method'])==(population,method)]
        groups.append(dict(population=population,method=method,count=len(r),
            mean_loss=float(np.mean([x['loss'] for x in r])),
            p95_loss=float(np.quantile([x['loss'] for x in r],.95)),
            joint_settled_count=sum(bool(x['joint_settled_in_simulation']) for x in r),
            no_reverse_settled_count=sum(bool(x['joint_settled_in_simulation']) and bool(x['no_reverse_in_simulation']) for x in r),
            reversal_count=sum(not bool(x['no_reverse_in_simulation']) for x in r),
            reverse_speed_exceeds_004_count=sum(x['min_velocity_m_s'] < -.04 for x in r),
            rollback_exceeds_002_count=sum(x['max_rollback_m'] > .02 for x in r),
            reversal_before_position_count=sum(bool(x['reversal_before_position']) for x in r),
            reversal_after_position_count=sum(bool(x['reversal_after_position']) for x in r),
            mean_terminal_max_abs_position_error_m=float(np.mean([x['terminal_max_abs_position_error_m'] for x in r])),
            mean_terminal_max_abs_velocity_m_s=float(np.mean([x['terminal_max_abs_velocity_m_s'] for x in r])),
            mean_target_overshoot_m=float(np.mean([x['max_target_overshoot_m'] for x in r])),
            maximum_target_overshoot_m=max(x['max_target_overshoot_m'] for x in r),
            mean_rollback_m=float(np.mean([x['max_rollback_m'] for x in r])),
            maximum_rollback_m=max(x['max_rollback_m'] for x in r),
            mean_handoff_after_s=float(np.mean([x['handoff_after_s'] for x in r]))))
    return groups


def source_hashes():
    from Interaction.train_rl_coast import source_hashes as training_hashes
    return {**training_hashes(),'Interaction/evaluate_rl_coast.py':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


def markdown(report):
    lines=['# V2 fixed-target offline comparison','',
        '**Position dynamics are an uncalibrated surrogate. No physical validation or deployment.**','',
        'Common3s horizon; target fixed before action. V1/predictor LEVEL actions are adapted to direct POSITION takeover, not native v1 dynamics.','',
        '|Population|Method|N|Loss|Joint settled|Settled without reversal|Reversals|Tail max error mean cm|Tail max speed mean m/s|Worst overshoot cm|',
        '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for r in report['aggregate']:
        lines.append(f"|{r['population']}|{r['method']}|{r['count']}|{r['mean_loss']:.3f}|{r['joint_settled_count']}|"
                     f"{r['no_reverse_settled_count']}|{r['reversal_count']}|{100*r['mean_terminal_max_abs_position_error_m']:.2f}|"
                     f"{r['mean_terminal_max_abs_velocity_m_s']:.3f}|{100*r['maximum_target_overshoot_m']:.2f}|")
    lines += ['', 'Joint settled means the complete last0.3s satisfies position<=3cm, speed<=0.04m/s, tilt<=3deg and rate<=5deg/s. It does not imply no earlier overshoot/reversal; that conjunction is reported separately.',
              'Regression targets are assigned synthetic gaps, not log-derived release endpoints. Aggregate loss is not a flight-safety probability.']
    return '\n'.join(lines)+'\n'


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frozen-report',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--policy',action='append',default=[],metavar='LABEL=CHECKPOINT')
    parser.add_argument('--v1-policy',type=Path)
    parser.add_argument('--regression-manifest',type=Path)
    parser.add_argument('--count',type=int,default=256)
    parser.add_argument('--population-seed',type=int,default=804271)
    parser.add_argument('--population',choices=['development','held_out_synthetic'],default='held_out_synthetic')
    parser.add_argument('--prepare-only',action='store_true')
    args=parser.parse_args(argv)
    if args.output.exists():
        parser.error('output exists; choose a new directory')
    if args.population=='held_out_synthetic' and args.population_seed==19001:
        parser.error('development seed cannot be presented as held out')
    raw=args.frozen_report.read_bytes()
    nominal=projected_frozen_model(json.loads(raw),[0.,1.])
    cases=make_cases(nominal,args.count,args.population_seed,args.population)
    sources=[]
    if args.regression_manifest:
        extra,source=observed_regressions(args.regression_manifest,nominal)
        cases.extend(extra)
        sources.append(source)
    policies={}
    checkpoints=[]
    specs=[(s.split('=',1)[0],Path(s.split('=',1)[1])) for s in args.policy]
    if args.v1_policy:
        specs.append((V1_LABEL,args.v1_policy))
    for label,path in specs:
        if not label or label in BASELINES or label in policies or (label==V1_LABEL and path!=args.v1_policy):
            parser.error('unique, unreserved policy labels required')
        meta=json.loads(path.with_suffix('.metadata.json').read_text())
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        if digest!=meta['checkpoint_sha256']:
            raise ValueError('checkpoint SHA mismatch')
        if label!=V1_LABEL:
            from Interaction.train_rl_coast import TASK_VERSION, OBSERVATION_VERSION, ACTION_VERSION, canonical_sha256
            for field,expected in [('task_version',TASK_VERSION),('observation_version',OBSERVATION_VERSION),
                                   ('action_version',ACTION_VERSION)]:
                if meta.get(field)!=expected:
                    raise ValueError(f'checkpoint version mismatch: {field}')
            if meta['env_config']!=asdict(CoastBrakeConfig()):
                raise ValueError('checkpoint environment configuration mismatch')
            if meta['frozen_parameters_sha256']!=canonical_sha256(asdict(nominal)):
                raise ValueError('checkpoint nominal model mismatch')
            if any(source_hashes().get(k)!=v for k,v in meta['source_hashes'].items()):
                raise ValueError('checkpoint source differs from evaluator dependency chain')
        else:
            from Interaction.train_rl_braking import source_hashes as legacy_hashes, canonical_sha256
            if (meta.get('env_config')!=asdict(RLBrakeConfig())
                    or meta.get('source_hashes')!=legacy_hashes()
                    or meta.get('frozen_parameters_sha256')!=canonical_sha256(asdict(nominal))):
                raise ValueError('legacy checkpoint configuration/source/model does not match preserved v1')
        from stable_baselines3 import PPO
        import torch
        torch.set_num_threads(1)
        policies[label]=PPO.load(path,device='cpu')
        expected=114 if label==V1_LABEL else 117
        if policies[label].observation_space.shape!=(expected,) or policies[label].action_space.n!=2:
            raise ValueError('policy observation/action space mismatch')
        checkpoints.append(dict(label=label,path=str(path.resolve()),sha256=digest,
            legacy_semantics_adapted=label==V1_LABEL))
    manifest=dict(created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
        task_version='coast_v2',offline_only=True,position_model_identified=False,
        frozen_report=dict(path=str(args.frozen_report.resolve()),sha256=hashlib.sha256(raw).hexdigest()),
        population_seed=args.population_seed,config=asdict(CoastBrakeConfig()),
        cases=[asdict(c) for c in cases],sources=sources,checkpoints=checkpoints,source_sha256=source_hashes())
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    if args.prepare_only:
        print(args.output/'manifest.json')
        return 0
    started=time.perf_counter()
    rows,traces=[],[]
    for method in [*BASELINES,*policies]:
        for index,case in enumerate(cases):
            row,trajectory=run_episode(case,method,policy=policies.get(method),trace=index<2)
            rows.append(row)
            if trajectory:
                traces.append(trajectory)
            if (index+1)%32==0:
                print(f'{method}: {index+1}/{len(cases)}',flush=True)
        print(f'{method}: complete',flush=True)
    if source_hashes()!=manifest['source_sha256']:
        raise RuntimeError('evaluation source changed mid-run')
    report=dict(offline_only=True,flight_validated=False,position_model_identified=False,
                elapsed_s=time.perf_counter()-started,rows=rows,aggregate=summarize(rows))
    for name,payload in [('report.json',report),('trajectories.json',traces)]:
        (args.output/name).write_text(json.dumps(payload,indent=2,allow_nan=False)+'\n')
    (args.output/'README.md').write_text(markdown(report))
    print(args.output/'README.md')
    return 0


if __name__=='__main__':
    raise SystemExit(main())
