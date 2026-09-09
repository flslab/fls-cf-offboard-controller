"""Offline closed-loop simulation of early leveling; never flight approval.

The controller receives only simulated, causally delayed measurements after the
logged pre-brake initial state. It is never reset to later recorded states.
The nominal plant is a regression check. Parameter/sensor perturbations are
predeclared sensitivity tests, not identified uncertainty bounds or flight data.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, replace
import hashlib
import json
from pathlib import Path

import numpy as np

from Interaction.braking_replay import extract
from Interaction.braking_split_diagnostic import build_trials, brake_anchor
from Interaction.braking_validation_simulator import DelayedTiltPlant
from Interaction.evaluate_braking_snapshots import projected_frozen_model
from Interaction.offline_braking_selector import BrakingSnapshot
from Interaction.predictive_brake_release import BrakeReleaseConfig, PredictiveBrakeRelease


@dataclass(frozen=True)
class StressCase:
    name: str
    motion_gain_multiplier: float = 1.0
    command_gain_multiplier: float = 1.0
    wn_multiplier: float = 1.0
    zeta_multiplier: float = 1.0
    extra_model_delay_s: float = 0.0
    transport_delay_s: float = 0.0
    measurement_delay_s: float = 0.0
    velocity_bias_m_s: float = 0.0
    tilt_bias_deg: float = 0.0
    rate_bias_deg_s: float = 0.0


# These are deliberately declared before inspecting simulation results. Do not
# fit/tune bounds to make all rows pass and call the result independent evidence.
STRESS_CASES = (
    StressCase('nominal'),
    StressCase('motion_gain_0.8', motion_gain_multiplier=.8),
    StressCase('motion_gain_1.2', motion_gain_multiplier=1.2),
    StressCase('command_gain_0.8', command_gain_multiplier=.8),
    StressCase('command_gain_1.2', command_gain_multiplier=1.2),
    StressCase('wn_0.7', wn_multiplier=.7),
    StressCase('wn_1.3', wn_multiplier=1.3),
    StressCase('zeta_0.7', zeta_multiplier=.7),
    StressCase('zeta_1.3', zeta_multiplier=1.3),
    StressCase('model_delay_plus_20ms', extra_model_delay_s=.02),
    StressCase('measurement_delay_20ms', measurement_delay_s=.02),
    StressCase('transport_delay_20ms', transport_delay_s=.02),
    StressCase('velocity_bias_minus_0.03', velocity_bias_m_s=-.03),
    StressCase('velocity_bias_plus_0.03', velocity_bias_m_s=.03),
    StressCase('tilt_bias_minus_1deg', tilt_bias_deg=-1.),
    StressCase('tilt_bias_plus_1deg', tilt_bias_deg=1.),
    StressCase('rate_bias_minus_10deg_s', rate_bias_deg_s=-10.),
    StressCase('rate_bias_plus_10deg_s', rate_bias_deg_s=10.),
    StressCase('combined_stronger_fast_delayed', motion_gain_multiplier=1.2,
               wn_multiplier=1.3, measurement_delay_s=.02, transport_delay_s=.02),
    StressCase('combined_weaker_slow_delayed', motion_gain_multiplier=.8,
               wn_multiplier=.7, extra_model_delay_s=.02),
)


def observed_seeds(records, source_name, frozen_report, excluded_segments=()):
    """Keep only the pre-brake state and past commands, not future motion."""
    _, trials, _, metadata = extract(records)
    result = []
    for item in build_trials(records, trials):
        trial = item.trial
        if trial.segment in excluded_segments:
            continue
        index = brake_anchor(item)
        if index < 0:
            raise ValueError('trial lacks a pre-brake snapshot')
        t0 = float(trial.times[index])
        past = trial.command_times <= t0
        result.append(dict(
            name=f'{source_name}:{trial.segment}', kind='observed_initial_state',
            snapshot=BrakingSnapshot(0., 0., float(trial.velocities[index]),
                                    float(item.angle[index]), float(item.rate[index])),
            command_history=list(zip((trial.command_times[past]-t0).tolist(),
                                     np.arctan(trial.commands[past]/9.81).tolist())),
            first_brake_after_snapshot_s=float(trial.phase_times['brake']-t0),
            baseline_brake_duration_s=.24,
            actual_logged_brake_duration_s=float(trial.phase_times['level_after_brake']-trial.phase_times['brake']),
            model=projected_frozen_model(frozen_report, trial.direction),
            direction_xy=trial.direction.tolist(), source_metadata=metadata,
        ))
    return result


def synthetic_seeds(frozen_report):
    """Exploratory +Y states; project the world bias independently of log order."""
    direction = np.array([0., 1.])
    model = projected_frozen_model(frozen_report, direction)
    return [dict(name=f'synthetic_level_{speed:g}m_s', kind='synthetic_extrapolation',
                 snapshot=BrakingSnapshot(0., 0., speed, 0., 0.),
                 command_history=[(-1., 0.)], first_brake_after_snapshot_s=0.,
                 baseline_brake_duration_s=.24, model=model,
                 direction_xy=direction.tolist()) for speed in (.1, .2, .4, .7, 1.)]


def summary(trace, release_time, first_decision, mode, reason):
    t = np.asarray([row['time_s'] for row in trace])
    p = np.asarray([row['position_m'] for row in trace])
    v = np.asarray([row['velocity_m_s'] for row in trace])
    angle = np.asarray([row['tilt_rad'] for row in trace])
    rate = np.asarray([row['tilt_rate_rad_s'] for row in trace])
    active = t >= first_decision-1e-10
    tail = t >= t[-1]-.10
    rollback = np.maximum.accumulate(p[active])-p[active]
    nonreverse = bool(np.min(v[active]) >= -1e-6)
    capture_ready = bool(nonreverse and np.min(v[tail]) >= 0 and np.max(v[tail]) <= .20
                         and np.max(abs(angle[tail])) <= np.radians(3)
                         and np.max(abs(rate[tail])) <= np.radians(5))
    return dict(mode=mode, release_reason=reason,
        release_after_brake_s=float(release_time-first_decision),
        observation_after_level_s=float(t[-1]-release_time),
        min_velocity_m_s=float(np.min(v[active])),
        terminal_mean_velocity_m_s=float(np.mean(v[tail])),
        terminal_max_abs_velocity_m_s=float(np.max(abs(v[tail]))),
        max_rollback_m=float(np.max(rollback)),
        max_forward_displacement_m=float(np.max(p[active])-p[active][0]),
        final_displacement_m=float(p[-1]-p[active][0]),
        terminal_max_abs_tilt_deg=float(np.degrees(np.max(abs(angle[tail])))),
        terminal_max_abs_rate_deg_s=float(np.degrees(np.max(abs(rate[tail])))),
        no_reverse_in_simulation=nonreverse,
        low_speed_level_capture_state_in_simulation=capture_ready,
        near_stationary_in_simulation=bool(capture_ready and np.max(abs(v[tail])) <= .04),
        position_target_validated=False, flight_validated=False)


def compact_decision(row):
    """Preserve scalar decision evidence without thousands of forecast points."""
    result = {}
    for key, value in row.items():
        if isinstance(value, dict):
            result[key] = {k: v for k, v in value.items() if not isinstance(v, (list, dict))}
        elif not isinstance(value, list):
            result[key] = value
    return result


def simulate(seed, case, mode, config=None, *, tail_observation_s=1.5):
    """Pair each controller with exactly the same plant and initial state."""
    if mode not in ('fixed_240ms', 'predictive_level'):
        raise ValueError('unknown simulated controller mode')
    config = config or BrakeReleaseConfig()
    config.validate()
    if not np.isfinite(tail_observation_s) or tail_observation_s < 1.2:
        raise ValueError('observe at least 1.2 seconds after the final level command')
    case_values = [value for key, value in asdict(case).items() if key != 'name']
    if (not np.isfinite(case_values).all()
            or min(case.motion_gain_multiplier, case.command_gain_multiplier,
                   case.wn_multiplier, case.zeta_multiplier) <= 0
            or min(case.extra_model_delay_s, case.transport_delay_s,
                   case.measurement_delay_s) < 0):
        raise ValueError('stress parameters must be finite and within their domains')
    if (not np.isfinite([seed['first_brake_after_snapshot_s'],
                        seed['baseline_brake_duration_s']]).all()
            or seed['first_brake_after_snapshot_s'] < 0
            or seed['baseline_brake_duration_s'] <= 0
            or seed['snapshot'].time_s != 0):
        raise ValueError('seed must start at time zero with valid brake timing')
    model = seed['model']
    if config.control_period_s + model.delay_s >= config.prediction_horizon_s:
        raise ValueError('forecast must extend beyond one control tick plus command delay')
    plant_model = replace(model,
        motion_gain=model.motion_gain*case.motion_gain_multiplier,
        command_gain=model.command_gain*case.command_gain_multiplier,
        wn_rad_s=model.wn_rad_s*case.wn_multiplier,
        zeta=model.zeta*case.zeta_multiplier,
        delay_s=model.delay_s+case.extra_model_delay_s)
    plant = DelayedTiltPlant(plant_model, seed['snapshot'], seed['command_history'],
                             transport_delay_s=case.transport_delay_s)
    # No pre-seed measurements are fabricated. Delay-stress trials first run a
    # known-level warmup; both paired policies use the same shifted start.
    first = max(seed['first_brake_after_snapshot_s'], case.measurement_delay_s)
    fixed_level_time = first+seed['baseline_brake_duration_s']
    policy = PredictiveBrakeRelease(model, config)
    trace, decisions = [], []
    release_time, reason = None, None
    tick, last_command = 0, None
    while True:
        decision_time = first+tick*config.control_period_s
        # Exact baseline switch, even if not on a tick. No extra reverse-tilt
        # time is silently added to make the baseline look worse.
        if mode == 'fixed_240ms' and release_time is None and fixed_level_time < decision_time-1e-10:
            plant.advance_to(fixed_level_time)
            plant.send_command(fixed_level_time, 0.)
            release_time, reason, last_command = fixed_level_time, 'fixed_duration', 0.
        state = plant.advance_to(decision_time)
        if mode == 'fixed_240ms':
            command = -np.radians(config.brake_tilt_deg) if decision_time < fixed_level_time-1e-10 else 0.
            if command == 0 and release_time is None:
                release_time, reason = decision_time, 'fixed_duration'
        else:
            measured = plant.snapshot_at_or_before(decision_time-case.measurement_delay_s)
            if measured is None:
                raise ValueError('delayed measurement unavailable after explicit warmup')
            measured = replace(measured,
                velocity_m_s=measured.velocity_m_s+case.velocity_bias_m_s,
                tilt_rad=measured.tilt_rad+np.radians(case.tilt_bias_deg),
                tilt_rate_rad_s=measured.tilt_rate_rad_s+np.radians(case.rate_bias_deg_s))
            decision = policy.update(measured, plant.command_history, decision_time_s=decision_time)
            command = float(decision['command_tilt_rad'])
            decisions.append(dict(simulation_time_s=decision_time, **compact_decision(decision)))
            if decision['released'] and release_time is None:
                release_time, reason = decision_time, decision['reason']
        if last_command is None or command != last_command:
            plant.send_command(decision_time, command)
            last_command = command
        trace.append(dict(asdict(state), simulated_command_tilt_rad=float(command)))
        if release_time is not None and decision_time-release_time >= tail_observation_s-1e-10:
            break
        if decision_time-first > config.max_brake_duration_s+tail_observation_s+1:
            raise RuntimeError('simulated controller never completed its observation window')
        tick += 1
    # Score every integration boundary, not only 100 Hz control ticks: a short
    # reversal between two decisions must not disappear from the evaluation.
    result = summary([asdict(state) for state in plant.snapshot_history],
                     release_time, first, mode, reason)
    result.update(seed=seed['name'], seed_kind=seed['kind'], stress_case=case.name,
                  stress_parameters=asdict(case), simulated_plant_model=asdict(plant_model),
                  measurement_warmup_extension_s=first-seed['first_brake_after_snapshot_s'],
                  frozen_controller_model=asdict(model), controller_config=asdict(config),
                  first_decision_time_s=first,
                  data_after_initial_state='independent RK4 plant states, never future log rows')
    return result, trace, decisions


def aggregate(rows):
    out = []
    for kind, case in sorted({(r['seed_kind'], r['stress_case']) for r in rows}):
        for mode in ('fixed_240ms', 'predictive_level'):
            chosen = [r for r in rows if r['seed_kind'] == kind and r['stress_case'] == case and r['mode'] == mode]
            out.append(dict(seed_kind=kind, stress_case=case, mode=mode, count=len(chosen),
                reverse_count=sum(not r['no_reverse_in_simulation'] for r in chosen),
                capture_state_count=sum(r['low_speed_level_capture_state_in_simulation'] for r in chosen),
                near_stationary_count=sum(r['near_stationary_in_simulation'] for r in chosen),
                max_rollback_m=max(r['max_rollback_m'] for r in chosen),
                minimum_tail_speed_m_s=min(r['terminal_mean_velocity_m_s'] for r in chosen),
                maximum_tail_speed_m_s=max(r['terminal_mean_velocity_m_s'] for r in chosen),
                release_min_s=min(r['release_after_brake_s'] for r in chosen),
                release_max_s=max(r['release_after_brake_s'] for r in chosen)))
    return out


def format_markdown(report):
    lines = ['# 提前回水平：离线闭环敏感性验证', '',
        '**这不是实机结果，也不是位置控制接管验证。** 名义 plant 与控制器共用模型，只算回归检查；预设扰动是敏感性测试，不是已标定的不确定性范围。', '',
        '每条轨迹只从日志取制动前初始状态及此前已发送命令，之后完全积分自己的模拟状态。每次回水平后继续观察至少 1.5 s。', '',
        '指标“可接管状态”仅指观测末段速度在 0–0.20 m/s、倾角≤3°、角速度≤5°/s 且全程未反向；**不代表已经停稳或位置接管必定成功**。近静止另外要求末段速度≤0.04 m/s。', '',
        '| 初始状态来源 | 扰动 | 方法 | N | 回退数 | 低速水平状态数 | 近静止数 | 最大回退 cm | 末速度范围 m/s | 解除制动范围 s |',
        '|---|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for row in report['aggregate']:
        lines.append(f"| {row['seed_kind']} | {row['stress_case']} | {row['mode']} | {row['count']} | {row['reverse_count']} | {row['capture_state_count']} | {row['near_stationary_count']} | {100*row['max_rollback_m']:.2f} | {row['minimum_tail_speed_m_s']:.3f}–{row['maximum_tail_speed_m_s']:.3f} | {row['release_min_s']:.3f}–{row['release_max_s']:.3f} |")
    lines += ['', '## 限制', '']+[f'- {x}' for x in report['cautions']]
    return '\n'.join(lines)+'\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument('--frozen-report', required=True, type=Path)
    parser.add_argument('--exclude', action='append', default=[], metavar='FILENAME:SEGMENT')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--nominal-only', action='store_true')
    parser.add_argument('--release-margin-m-s', type=float, default=.08)
    parser.add_argument('--synthetic-speed-sweep', action='store_true',
                        help='also explore 0.1/0.2/0.4/0.7/1.0 m/s with level, zero-rate initial states; not measured validation')
    args = parser.parse_args(argv)
    config = BrakeReleaseConfig(release_speed_margin_m_s=args.release_margin_m_s)
    try:
        config.validate()
    except ValueError as exc:
        parser.error(str(exc))
    if args.output.exists():
        parser.error('output directory exists; choose a new directory')
    frozen_raw = args.frozen_report.read_bytes()
    frozen = json.loads(frozen_raw)
    seeds, sources, applied_exclusions = [], [], []
    for path in args.logs:
        raw = path.read_bytes()
        excluded = [int(x.rsplit(':', 1)[1]) for x in args.exclude if x.rsplit(':', 1)[0] == path.name]
        seeds.extend(observed_seeds(json.loads(raw), path.name, frozen, excluded))
        sources.append(dict(path=str(path.resolve()), sha256=hashlib.sha256(raw).hexdigest()))
        applied_exclusions.extend(f'{path.name}:{segment}' for segment in excluded)
    if sorted(applied_exclusions) != sorted(args.exclude):
        parser.error('an exclusion filename did not match an input log')
    if not seeds:
        parser.error('no observed seed remains')
    if args.synthetic_speed_sweep:
        seeds.extend(synthetic_seeds(frozen))
    cases = (STRESS_CASES[0],) if args.nominal_only else STRESS_CASES
    rows, traces, decisions = [], [], []
    for seed in seeds:
        for case in cases:
            for mode in ('fixed_240ms', 'predictive_level'):
                row, trace, choice = simulate(seed, case, mode, config)
                rows.append(row)
                if case.name == 'nominal' or not row['no_reverse_in_simulation']:
                    traces.append(dict(seed=seed['name'], stress_case=case.name, mode=mode, trace=trace))
                if mode == 'predictive_level':
                    decisions.append(dict(seed=seed['name'], stress_case=case.name, decisions=choice))
        print(f"Simulated {seed['name']}", flush=True)
    report = dict(offline_only=True, flight_commands_generated=False, calibration_written=False,
        model_fitted=False, source_logs=sources, excluded=args.exclude,
        implementation_sha256={name: hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
            for name in ('validate_predictive_brake_release.py', 'predictive_brake_release.py',
                         'braking_validation_simulator.py', 'offline_braking_selector.py',
                         'braking_split_diagnostic.py', 'evaluate_braking_snapshots.py', 'braking_replay.py')},
        frozen_report_path=str(args.frozen_report.resolve()),
        frozen_report_sha256=hashlib.sha256(frozen_raw).hexdigest(),
        controller_config=asdict(config), stress_cases=[asdict(c) for c in cases],
        seed_count=len(seeds), seeds=[dict(s, snapshot=asdict(s['snapshot']), model=asdict(s['model'])) for s in seeds],
        rows=rows, aggregate=aggregate(rows),
        cautions=[
            '控制器/plant 共用名义模型的通过不是独立物理验证；扰动范围是探索设定，不是已识别的实机边界。',
            '没有位置目标、位置控制接管、横向耦合、外力、空气阻力或完整估计器模型；不能保证停点。',
            '0.08 m/s 是预先指定的探索性余量，不是从这批仿真调到全通过的保证值。',
            '0.71–0.79 m/s 的真实初始状态只覆盖很窄范围；synthetic 标记的速度扫描是外推。',
            '16:47 segment 1 的严重定位跳变必须显式排除；16:58 本身也有定位空档，种子不等于洁净整段飞行。',
            '传感器延迟使用带原时间戳的历史模拟状态；初始历史不足时两种策略都先执行同样的水平预热，不编造过去观测。',
            '各扰动场景关联、共享种子，计数不是独立飞行样本数或可靠性概率。',
            '回水平锁定后不再施加反向脉冲；仍可能保留正向漂移，故非反向与近静止分别报告。',
        ])
    args.output.mkdir(parents=True, exist_ok=False)
    for name, payload in [('report.json', report), ('trajectories.json', traces), ('decisions.json', decisions)]:
        (args.output/name).write_text(json.dumps(payload, indent=2, allow_nan=False)+'\n')
    (args.output/'README.md').write_text(format_markdown(report))
    print(args.output/'README.md')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
