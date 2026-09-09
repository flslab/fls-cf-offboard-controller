"""Evaluate hypothetical finite braking candidates at causal logged-state snapshots.

No fitting, live calibration write, flight connection or flight command. This is
NOT a closed-loop counterfactual replay: each snapshot belongs to the originally
executed command trajectory. The target is an explicit hypothetical distance
from the measured brake anchor, never the future observed stopping point.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from Interaction.braking_replay import extract
from Interaction.braking_split_diagnostic import build_trials, brake_anchor
from Interaction.offline_braking_selector import (
    BrakingSnapshot, FrozenTiltModel, evaluate_braking_candidates,
)


DEFAULT_OFFSETS_S = (0.0, 0.10, 0.15, 0.20, 0.24)


def projected_frozen_model(report, direction):
    """The identified opposed-Y model does not cover arbitrary lateral axes."""
    direction = np.asarray(direction, dtype=float)
    if (direction.shape != (2,) or not np.isfinite(direction).all()
            or abs(direction[0]) > 1e-6 or not np.isclose(abs(direction[1]), 1)):
        raise ValueError('frozen model is only identified for normalized opposed Y directions')
    fit = report['frozen_tilt_fit']
    if fit['model'] != 'second_order':
        raise ValueError('expected an already-frozen second-order tilt model')
    return FrozenTiltModel(
        delay_s=fit['delay_s'], wn_rad_s=fit['wn_rad_s'], zeta=fit['zeta'],
        command_gain=fit['gain'], motion_gain=report['frozen_motion_gain'],
        projected_bias_rad=fit['bias_world_y_rad']*float(direction[1]),
    )


def evaluate_trial_snapshots(item, frozen_report, target_distance_m,
                             offsets_s=DEFAULT_OFFSETS_S):
    """Select states/commands at or before each snapshot, preserving target origin."""
    if not np.isfinite(target_distance_m) or target_distance_m < 0:
        raise ValueError('an explicit finite nonnegative hypothetical target distance is required')
    offsets = np.asarray(offsets_s, dtype=float)
    if offsets.ndim != 1 or not len(offsets) or not np.isfinite(offsets).all() or np.any(offsets < 0):
        raise ValueError('snapshot offsets must be finite and nonnegative')
    trial = item.trial
    if (len(trial.times) < 2 or np.any(np.diff(trial.times) <= 0)
            or len(item.angle) != len(trial.times) or len(item.rate) != len(trial.times)):
        raise ValueError('invalid measured state timeline')
    model = projected_frozen_model(frozen_report, trial.direction)
    anchor = brake_anchor(item)
    if anchor < 0:
        raise ValueError('missing pre-brake anchor measurement')
    target = float(trial.positions[anchor]+target_distance_m)
    brake_time = float(trial.phase_times['brake'])
    rows = []
    for offset in offsets:
        requested_time = brake_time+float(offset)
        if requested_time > trial.times[-1]:
            raise ValueError('requested snapshot lies beyond the observed trial')
        index = int(np.searchsorted(trial.times, requested_time, side='right')-1)
        time = float(trial.times[index])
        state = BrakingSnapshot(
            time_s=time, position_m=float(trial.positions[index]),
            velocity_m_s=float(trial.velocities[index]),
            tilt_rad=float(item.angle[index]), tilt_rate_rad_s=float(item.rate[index]),
        )
        sent = trial.command_times <= time
        history = list(zip(trial.command_times[sent].tolist(),
                           np.arctan(trial.commands[sent]/9.81).tolist()))
        remaining = float(target-state.position_m)
        diagnostics = evaluate_braking_candidates(
            state, model, history, target_remaining_m=remaining,
        )
        rows.append(dict(
            segment=int(trial.segment), direction_xy=trial.direction.tolist(),
            requested_after_brake_s=float(offset), actual_after_brake_s=time-brake_time,
            measurement_before_requested_s=requested_time-time,
            anchor_time_s=float(trial.times[anchor]),
            anchor_position_m=float(trial.positions[anchor]),
            hypothetical_target_distance_m=float(target_distance_m),
            frozen_hypothetical_target_position_m=target,
            remaining_target_distance_m=remaining,
            diagnostics=diagnostics,
        ))
    return rows


def artifact(path):
    raw = path.read_bytes()
    return json.loads(raw), dict(path=str(path.resolve()), sha256=hashlib.sha256(raw).hexdigest())


def format_report(report):
    lines = [
        '# 离线制动候选快照（不控制飞行）', '',
        f"假设目标：制动前实测位置沿试验运动轴前方 **{report['hypothetical_target_distance_m']*100:.1f} cm**。",
        '这是显式指定的假想目标，不是此次开环测试实际下发的 coast 目标，也不是用实测停止点反推的目标。', '',
        '每个快照重新读取原始飞行产生的实测状态，不能把这些选择串成一条已经验证的闭环轨迹。',
        '冻结旧模型、不重新拟合；只使用该快照以前已发送的命令，保留尚未生效的队列。', '',
        '| 试验 / 制动后请求时刻 | 当前速度 | 剩余距离 | 离线选择 | 预测尾部速度 | 预测停稳 | 原因 |',
        '|---|---:|---:|---|---:|---|---|',
    ]
    for row in report['snapshots']:
        result = row['diagnostics']
        chosen = result['selected']
        state = result.get('snapshot') or {}
        lines.append(
            f"| {row['segment']} / {row['requested_after_brake_s']:.2f} s "
            f"| {state.get('velocity_m_s', float('nan')):.3f} m/s "
            f"| {row['remaining_target_distance_m']*100:.2f} cm "
            f"| {('无可行选择' if chosen is None else str(round(chosen['pulse_duration_s']*1000))+' ms 反向脉冲后水平')} "
            f"| {('—' if chosen is None else format(chosen['terminal_velocity_m_s'], '.3f')+' m/s')} "
            f"| {('—' if chosen is None else str(chosen['settled_stop_predicted']))} "
            f"| {result['reason']} |"
        )
    lines += ['', '## 全部候选', '',
              '末速度不是停止距离；可行选择也不必然预测已经停稳。默认检查整个 0.8 s 窗口内不出现负速度，目标超越容差为 2 cm。', '',
              '| 试验 / 快照 | 反向脉冲 | 最低速度 | 尾部速度 | 最大目标超越 | 拒绝原因 |',
              '|---|---:|---:|---:|---:|---|']
    for row in report['snapshots']:
        for candidate in row['diagnostics']['candidates']:
            lines.append(
                f"| {row['segment']} / {row['requested_after_brake_s']:.2f} s "
                f"| {candidate['pulse_duration_s']*1000:.0f} ms "
                f"| {candidate.get('min_velocity_m_s', float('nan')):.3f} m/s "
                f"| {candidate.get('terminal_velocity_m_s', float('nan')):.3f} m/s "
                f"| {candidate.get('overshoot_m', float('nan'))*100:.2f} cm "
                f"| {', '.join(candidate['rejection_reasons']) or '无（仅模型内）'} |"
            )
    lines += ['', '## 限制', ''] + ['- '+line for line in report['cautions']]
    return '\n'.join(lines)+'\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--frozen-report', type=Path, required=True)
    parser.add_argument('--target-distance-m', type=float, required=True,
                        help='explicit hypothetical forward distance from measured brake anchor')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('output already exists; choose a new directory')
    records, source = artifact(args.log)
    frozen, frozen_source = artifact(args.frozen_report)
    _, trials, _, metadata = extract(records)
    items = build_trials(records, trials)
    snapshots = [row for item in items for row in evaluate_trial_snapshots(
        item, frozen, args.target_distance_m)]
    parameters = dict(tilt=frozen['frozen_tilt_fit'], motion_gain=frozen['frozen_motion_gain'])
    report = dict(
        offline_only=True, fit_calls=0, calibration_written=False,
        flight_commands_generated=False, closed_loop_simulation=False,
        target_is_hypothetical_not_logged_coast_target=True,
        hypothetical_target_distance_m=args.target_distance_m,
        source=source, frozen_model_source=frozen_source,
        frozen_parameters_sha256=hashlib.sha256(json.dumps(
            parameters, sort_keys=True, separators=(',', ':')).encode()).hexdigest(),
        frozen_parameters=parameters, metadata=metadata, snapshots=snapshots,
        cautions=[
            '每个快照属于原始已执行命令产生的轨迹；不能把选出的不同命令串成闭环改善的证据。',
            '本次校准没有 release/coast 目标，所填距离仅是假想测试参数，没有使用未来停止位置。',
            '预测使用近似的命令/host 状态时基；外部定位空档和位置—速度不一致尚未解决。',
            '可行仅指当前模型和有限 0.8 s 窗口内通过阈值；不能保证物理上不会回退或保证停稳。',
            '未识别横向位置接管或电池/扰动下完整动力学；没有生成实机控制指令。',
            '命令历史采用原始提取器验证过的已发送 phase-change 指令；重复同值发送已压缩。',
        ],
    )
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    (args.output/'ANALYSIS_zh.md').write_text(format_report(report))
    print(str((args.output/'ANALYSIS_zh.md').resolve()))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
