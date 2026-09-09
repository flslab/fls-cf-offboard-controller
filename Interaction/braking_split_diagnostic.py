"""Offline separation of command->tilt and tilt->motion errors; no flight writes.

python -m Interaction.braking_split_diagnostic LOG --output NEW_DIRECTORY
Actual/future attitude is used ONLY in the explicitly labeled oracle diagnostic.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.integrate import cumulative_trapezoid, trapezoid
from scipy.optimize import least_squares

from Interaction.braking_replay import extract, first_crossing, holdout_fits, metrics, replay, write_csv
from Interaction.braking_trajectory_fit import relative_trial


@dataclass
class TiltTrial:
    trial: object
    angle: np.ndarray
    rate: np.ndarray


def build_trials(records, trials):
    states = {}
    for record in records:
        if record.get('type') == 'wrench_observer':
            row = record['data']
            states.setdefault(float(row['state_time']), row)
    result = []
    for trial in trials:
        rates = []
        for timestamp in trial.times:
            state = states[float(timestamp)]
            yaw = state['orientation_rpy_rad'][2]
            roll_rate, pitch_rate, _ = state['angular_velocity_rad_s']
            c, s = np.cos(yaw), np.sin(yaw)
            dx, dy = trial.direction
            # Small-planar-angle rate projection; RATE_EST is not an exact
            # derivative of the equivalent tilt angle. Sensitivity is reported.
            rates.append(-(pitch_rate*(c*dx+s*dy) + roll_rate*(-s*dx+c*dy)))
        result.append(TiltTrial(relative_trial(trial),
                               np.arctan(trial.attitude_acceleration/9.81), np.array(rates)))
    return result


def second_homogeneous(t, q0, rate0, wn, zeta):
    alpha = wn*zeta
    if zeta < 1-1e-6:
        omega = wn*np.sqrt(1-zeta*zeta)
        return np.exp(-alpha*t)*(q0*np.cos(omega*t)+(rate0+alpha*q0)*np.sin(omega*t)/omega)
    if zeta > 1+1e-6:
        root = wn*np.sqrt(zeta*zeta-1)
        r1, r2 = -alpha+root, -alpha-root
        a = (rate0-r2*q0)/(r1-r2)
        return a*np.exp(r1*t)+(q0-a)*np.exp(r2*t)
    return np.exp(-wn*t)*(q0+(rate0+wn*q0)*t)


def tilt_prediction(item, fit, anchor=0, use_rate=True):
    """Delayed linear tilt transfer, with pending pre-anchor commands retained."""
    trial = item.trial
    t0 = trial.times[anchor]
    times = trial.times[anchor:]-t0
    delayed = trial.command_times+fit['delay_s']
    commands = fit['gain']*np.arctan(trial.commands/9.81)
    k = np.searchsorted(delayed, t0, side='right')-1
    target0 = (commands[k] if k >= 0 else 0.) + fit['bias_world_y_rad']*trial.direction[1]
    q0 = item.angle[anchor]-target0
    if fit['model']=='first_order':
        prediction = target0+q0*np.exp(-times/fit['tau_s'])
    else:
        prediction = target0+second_homogeneous(times, q0,
            item.rate[anchor] if use_rate else 0., fit['wn_rad_s'], fit['zeta'])
    for i, event in enumerate(delayed):
        if event <= t0:
            continue
        elapsed = times-(event-t0)
        mask = elapsed >= 0
        change = commands[i]-(commands[i-1] if i else 0.)
        if fit['model']=='first_order':
            response = -np.expm1(-elapsed[mask]/fit['tau_s'])
        else:
            response = 1+second_homogeneous(elapsed[mask], -1., 0., fit['wn_rad_s'], fit['zeta'])
        prediction[mask] += change*response
    return prediction


def decode(values, model):
    if model=='first_order':
        delay, tau, gain, bias = values
        return dict(model=model, delay_s=float(delay), tau_s=float(tau),
                    gain=float(gain), bias_world_y_rad=float(bias))
    delay, wn, zeta, gain, bias = values
    return dict(model=model, delay_s=float(delay), wn_rad_s=float(wn), zeta=float(zeta),
                gain=float(gain), bias_world_y_rad=float(bias))


def fit_tilt(training, model):
    if model=='first_order':
        seeds = [[.04, .10, 1., 0.], [.08, .05, 1., 0.]]
        bounds = ([0, .005, .7, -.035], [.15, .3, 1.3, .035])
    else:
        seeds = [[.02, 20., .7, 1., 0.], [.04, 35., 1., 1., 0.], [.005, 15., 1.5, 1., 0.]]
        bounds = ([0, 5, .2, .7, -.035], [.15, 100, 2., 1.3, .035])

    def residual(values):
        fit = decode(values, model)
        residuals = []
        for item in training:
            dt = np.diff(item.trial.times)
            weight = np.r_[dt[0]/2, (dt[:-1]+dt[1:])/2, dt[-1]/2]
            weight /= weight.sum()*len(training)
            residuals.extend((tilt_prediction(item, fit)-item.angle)*np.sqrt(weight))
        return np.asarray(residuals)

    candidates = [least_squares(residual, seed, bounds=bounds, max_nfev=250,
                                ftol=1e-9, xtol=1e-9, gtol=1e-9, x_scale='jac') for seed in seeds]
    valid = [r for r in candidates if r.success and np.isfinite(r.fun).all()]
    if not valid:
        raise ValueError('tilt fit did not converge')
    best = min(valid, key=lambda r: float(r.fun@r.fun))
    return dict(decode(best.x, model), training_segments=[t.trial.segment for t in training],
                train_rmse_deg=float(np.degrees(np.sqrt(best.fun@best.fun))),
                seed_costs=[float(r.fun@r.fun) for r in candidates],
                active_bounds=best.active_mask.tolist(), offline_only=True)


def brake_anchor(item):
    trial = item.trial
    return np.searchsorted(trial.times, trial.phase_times['brake'], side='right')-1


def motion_gain(training):
    """Only training trajectories; the same gain is used for oracle and causal models."""
    x, y = [], []
    for item in training:
        trial, anchor = item.trial, brake_anchor(item)
        integral = cumulative_trapezoid(trial.attitude_acceleration[anchor:], trial.times[anchor:], initial=0)
        x.extend(integral)
        y.extend(trial.velocities[anchor:]-trial.velocities[anchor])
    gain = float(np.linalg.lstsq(np.asarray(x)[:, None], y, rcond=None)[0][0])
    if not .2 < gain < 2.5:
        raise ValueError('oracle motion gain outside diagnostic bounds')
    return gain


def motion_metrics(item, angle, gain, name):
    trial, anchor = item.trial, brake_anchor(item)
    times = trial.times[anchor:]-trial.times[anchor]
    velocity = trial.velocities[anchor:]
    position = trial.positions[anchor:]-trial.positions[anchor]
    predicted_v = velocity[0]+gain*cumulative_trapezoid(9.81*np.tan(angle), times, initial=0)
    predicted_p = cumulative_trapezoid(predicted_v, times, initial=0)
    tail = times >= times[-1]-.1
    actual_cross = first_crossing(times, velocity, position)
    predicted_cross = first_crossing(times, predicted_v, predicted_p)
    row = dict(segment=trial.segment, duration_s=trial.duration, model=name,
        velocity_rmse_m_s=float(np.sqrt(np.mean((predicted_v-velocity)**2))),
        actual_terminal_m_s=float(np.mean(velocity[tail])), predicted_terminal_m_s=float(np.mean(predicted_v[tail])),
        terminal_error_m_s=float(np.mean(predicted_v[tail]-velocity[tail])),
        end_position_error_m=float(predicted_p[-1]-position[-1]),
        actual_reverse=bool(min(velocity)<-.03), predicted_reverse=bool(min(predicted_v)<-.03),
        actual_crossing=actual_cross, predicted_crossing=predicted_cross,
        crossing_error_s=(None if not actual_cross or not predicted_cross else predicted_cross['time_s']-actual_cross['time_s']))
    trace = [dict(segment=trial.segment, model=name, time_s=float(t), actual_v=float(v), predicted_v=float(vp),
                  actual_p=float(p), predicted_p=float(pp))
             for t,v,vp,p,pp in zip(times, velocity, predicted_v, position, predicted_p)]
    return row, trace


def phase_diagnostics(item):
    trial = item.trial
    q = np.degrees(item.angle)
    b, level = trial.phase_times['brake'], trial.phase_times['level_after_brake']
    mask = (trial.times>=b)&(trial.times<level+.4)
    indices = np.flatnonzero(mask)
    peak = indices[np.argmin(q[mask])]
    return dict(segment=trial.segment, angle_at_brake_deg=float(q[brake_anchor(item)]),
                peak_brake_angle_deg=float(q[peak]), peak_after_level_s=float(trial.times[peak]-level),
                final_mean_angle_deg=float(np.mean(q[trial.times>=trial.times[-1]-.1])))


def external_position_audit(records, trials):
    frames = sorted((r['data'] for r in records if r.get('type')=='frames'), key=lambda r:r['time'])
    if not frames:
        return []
    times = np.array([r['time'] for r in frames])
    xy = np.array([r['tvec'][:2] for r in frames])
    rows = []
    for trial in trials:
        if times[0] > trial.times[0] or times[-1] < trial.times[-1]:
            continue
        position = np.stack([np.interp(trial.times, times, xy[:, k]) for k in range(2)], axis=-1) @ trial.direction
        anchor = np.searchsorted(trial.times, trial.phase_times['brake'], side='right')-1
        t = trial.times[anchor:]-trial.times[anchor]
        p = position[anchor:]-position[anchor]
        velocity_integral = float(trapezoid(trial.velocities[anchor:], t))
        tail = t >= t[-1]-.1
        slope = float(np.polyfit(t[tail]-t[tail][0], p[tail], 1)[0])
        rows.append(dict(segment=trial.segment, external_displacement_m=float(p[-1]),
            onboard_displacement_m=float(trial.positions[-1]-trial.positions[anchor]),
            onboard_velocity_integral_m=velocity_integral,
            external_rollback_before_recovery_m=float(max(p)-p[-1]), external_terminal_slope_m_s=slope))
    return rows


def rolling_horizon(item, fit, gain):
    """Each forecast starts from available state; no corrections within a forecast.

    Evaluates the actually executed future schedule, not an unflown controller.
    """
    rows = []
    trial = item.trial
    for anchor in range(brake_anchor(item), len(trial.times)-1, 5):
        t = trial.times[anchor:]-trial.times[anchor]
        q = tilt_prediction(item, fit, anchor)
        pv = trial.velocities[anchor]+gain*cumulative_trapezoid(9.81*np.tan(q), t, initial=0)
        pp = cumulative_trapezoid(pv, t, initial=0)
        for horizon in [.05, .10, .20, .40]:
            if t[-1] < horizon:
                continue
            actual_v = float(np.interp(horizon, t, trial.velocities[anchor:]))
            actual_p = float(np.interp(horizon, t, trial.positions[anchor:]-trial.positions[anchor]))
            rows.append(dict(segment=trial.segment, anchor_s=float(trial.times[anchor]), horizon_s=horizon,
                velocity_error_m_s=float(np.interp(horizon, t, pv)-actual_v),
                position_error_m=float(np.interp(horizon, t, pp)-actual_p)))
    return rows


def oracle_timing_scan(training, held):
    """NONCAUSAL timing diagnostic; positive shift means future actual attitude.

    Trim every candidate's end by 60ms for identical comparison support. This
    must never be installed as a negative delay in an online controller.
    """
    def data(items, shift):
        inputs, targets = [], []
        for item in items:
            trial, anchor = item.trial, brake_anchor(item)
            times = trial.times[anchor:]
            times = times[times <= trial.times[-1]-.06]
            accel = np.interp(times+shift, trial.times, trial.attitude_acceleration)
            inputs.append(cumulative_trapezoid(accel, times, initial=0))
            targets.append(np.interp(times, trial.times, trial.velocities)-trial.velocities[anchor])
        return np.concatenate(inputs), np.concatenate(targets)
    candidates=[]
    for shift in np.linspace(-.06,.06,61):
        x,y=data(training,float(shift))
        gain=float(np.dot(x,y)/np.dot(x,x))
        candidates.append((float(np.mean((gain*x-y)**2)),float(shift),gain))
    loss,shift,gain=min(candidates)
    x,y=data(held,shift)
    return dict(oracle_only=True, training_selected_future_attitude_shift_s=shift,
                gain=gain, training_velocity_rmse_m_s=float(np.sqrt(loss)),
                held_velocity_rmse_m_s=float(np.sqrt(np.mean((gain*x-y)**2))),
                comparison_tail_trim_s=.06,
                warning='Positive lead uses future measured attitude; not a physical or deployable negative delay.')


def analyze_split(records, progress=print):
    saved, trials, samples, metadata = extract(records)
    items = build_trials(records, trials)
    legacy = holdout_fits(trials, samples, saved)
    folds, angle_rows, rows, traces, angle_traces, rolling = [], [], [], [], [], []
    for fold in legacy:
        held_ids = fold['held_segments']
        training = [t for t in items if t.trial.segment not in held_ids]
        held = [t for t in items if t.trial.segment in held_ids]
        progress(f'Split diagnostic: hold out {held_ids}', flush=True)
        fits = {model:fit_tilt(training, model) for model in ['first_order','second_order']}
        gain = motion_gain(training)
        folds.append(dict(held_segments=held_ids, tilt_fits=fits, motion_gain=gain,
                          oracle_timing_diagnostic=oracle_timing_scan(training,held)))
        for item in held:
            rolling.extend(rolling_horizon(item, fits['second_order'], gain))
            anchor = brake_anchor(item)
            predictions = {}
            for model, fit in fits.items():
                predicted = tilt_prediction(item, fit)
                predictions[model+'_history'] = predicted[anchor:]
                error = np.degrees(predicted-item.angle)
                angle_rows.append(dict(segment=item.trial.segment, model=model,
                    rmse_deg=float(np.sqrt(np.mean(error**2))), max_error_deg=float(max(abs(error)))))
                for t,q,qp in zip(item.trial.times, item.angle, predicted):
                    angle_traces.append(dict(segment=item.trial.segment, model=model, time_s=float(t),
                        actual_deg=float(np.degrees(q)), predicted_deg=float(np.degrees(qp))))
            predictions['second_order_brake_state'] = tilt_prediction(item, fits['second_order'], anchor)
            predictions['second_order_brake_state_zero_rate'] = tilt_prediction(item, fits['second_order'], anchor, False)
            predictions['oracle_actual_future_tilt'] = item.angle[anchor:]
            for name, angle in predictions.items():
                row, trace = motion_metrics(item, angle, gain, name)
                rows.append(row)
                traces.extend(trace)
            old, trace = metrics(item.trial, fold['fit'])
            rows.append(dict(segment=item.trial.segment, duration_s=item.trial.duration, model='legacy_command',
                velocity_rmse_m_s=old['velocity_rmse_m_s'], actual_terminal_m_s=old['actual_terminal_mean_m_s'],
                predicted_terminal_m_s=old['predicted_terminal_mean_m_s'], terminal_error_m_s=old['terminal_mean_error_m_s'],
                end_position_error_m=old['end_position_error_m'], actual_reverse=old['actual_reverse_over_0_03'],
                predicted_reverse=old['predicted_reverse_over_0_03'], actual_crossing=old['actual_first_zero_crossing'],
                predicted_crossing=old['predicted_first_zero_crossing'], crossing_error_s=old['crossing_time_error_s']))
            traces.extend(dict(segment=x['segment'], model='legacy_command', time_s=x['elapsed_s'],
                actual_v=x['actual_velocity_m_s'], predicted_v=x['predicted_velocity_m_s'],
                actual_p=x['actual_displacement_m'], predicted_p=x['predicted_displacement_m']) for x in trace)
    summary = {}
    for model in sorted({r['model'] for r in rows}):
        r = [x for x in rows if x['model']==model]
        crossings = [abs(x['crossing_error_s']) for x in r if x['crossing_error_s'] is not None]
        summary[model] = dict(velocity_rmse_m_s=float(np.sqrt(np.mean([x['velocity_rmse_m_s']**2 for x in r]))),
            terminal_mae_m_s=float(np.mean([abs(x['terminal_error_m_s']) for x in r])),
            terminal_max_error_m_s=float(max(abs(x['terminal_error_m_s']) for x in r)),
            end_position_max_error_m=float(max(abs(x['end_position_error_m']) for x in r)),
            missed_reverse=sum(x['actual_reverse'] and not x['predicted_reverse'] for x in r),
            false_reverse=sum(not x['actual_reverse'] and x['predicted_reverse'] for x in r),
            crossing_mae_s=float(np.mean(crossings)) if crossings else None,
            comparable_crossing_count=len(crossings))
    rolling_summary = {}
    for horizon in [.05, .10, .20, .40]:
        selected = [r for r in rolling if r['horizon_s']==horizon]
        rolling_summary[str(horizon)] = dict(count=len(selected),
            velocity_mae_m_s=float(np.mean([abs(r['velocity_error_m_s']) for r in selected])),
            velocity_max_error_m_s=float(max(abs(r['velocity_error_m_s']) for r in selected)),
            position_mae_m=float(np.mean([abs(r['position_error_m']) for r in selected])),
            position_max_error_m=float(max(abs(r['position_error_m']) for r in selected)))
    return dict(offline_only=True, metadata=metadata, holdout_folds=folds, tilt_metrics=angle_rows,
        motion_metrics=rows, summary=summary, phase_diagnostics=[phase_diagnostics(t) for t in items],
        rolling_horizon_summary=rolling_summary, rolling_forecasts=rolling,
        external_position_audit=external_position_audit(records, trials),
        caution='Oracle uses future measured tilt. Host receive timestamps cannot isolate physical lag. No flight approval.'), traces, angle_traces


def render(output, report, traces, angle_traces):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    models = ['legacy_command', 'second_order_brake_state', 'oracle_actual_future_tilt']
    colors = ['#888888', '#257bb7', '#c87919']
    fig, axes = plt.subplots(6, 2, figsize=(13,16))
    for i in range(6):
        ax=axes[i,0]
        for model, color in [('first_order','#888888'), ('second_order','#257bb7')]:
            rows=[r for r in angle_traces if r['segment']==i and r['model']==model]
            if model=='first_order':
                ax.plot([r['time_s'] for r in rows],[r['actual_deg'] for r in rows],color='black',label='Measured tilt')
            ax.plot([r['time_s'] for r in rows],[r['predicted_deg'] for r in rows],color=color,ls='--',label=model)
        ax.set_title(f'Trial {i}: command-to-tilt, held out')
        ax.set_ylabel('Signed tilt (deg)'); ax.set_xlabel('Time from pre-trial sample (s)')
        ax=axes[i,1]
        for model,color in zip(models,colors):
            rows=[r for r in traces if r['segment']==i and r['model']==model]
            if model=='legacy_command':
                ax.plot([r['time_s'] for r in rows],[r['actual_v'] for r in rows],color='black',label='Measured velocity')
            ax.plot([r['time_s'] for r in rows],[r['predicted_v'] for r in rows],color=color,ls='--',label=model)
        ax.axhline(0,color='#aaa',lw=.7); ax.set_title(f'Trial {i}: stopping velocity, held out')
        ax.set_ylabel('Signed velocity (m/s)'); ax.set_xlabel('Time from brake anchor (s)')
        for ax in axes[i]:ax.grid(alpha=.2)
    for ax in axes[0]:ax.legend(fontsize=7)
    fig.suptitle('Split diagnostic: predicted tilt vs future measured-tilt ORACLE\nNo runtime changes; oracle is NOT deployable',fontsize=13)
    fig.tight_layout(rect=(0,0,1,.97)); fig.savefig(output/'split_comparison.png',dpi=150); plt.close(fig)


def markdown(report):
    lines=['# 制动误差分解与最小补测计划','',
        '仅离线分析。没有修改飞行控制、校准文件或启动飞行。所有预测参数按完整时长正反试验对留出拟合。','',
        '## 核心结论','',
        '命令到实际姿态的动态是主要改进方向，但不是唯一误差来源。二阶姿态模型能改善过渡过程，'
        '实际姿态输入的 oracle 仍有残差；固定阻力/偏差参数不能解释全部末段行为。','',
        '| 方法 | 末速度平均绝对误差 m/s | 最大观察结束位置误差 cm | 回飞漏判/误报 |',
        '|---|---:|---:|---:|']
    labels={'legacy_command':'原命令响应模型','first_order_history':'分段一阶姿态模型',
            'second_order_history':'分段二阶姿态模型（命令历史）',
            'second_order_brake_state':'二阶＋刹车前实测角度/角速度',
            'second_order_brake_state_zero_rate':'二阶＋实测角度但假设角速度零',
            'oracle_actual_future_tilt':'未来实测姿态 oracle（不可上线）'}
    for key,label in labels.items():
        row=report['summary'][key]
        lines.append(f"| {label} | {row['terminal_mae_m_s']:.4f} | {100*row['end_position_max_error_m']:.2f} | {row['missed_reverse']}/{row['false_reverse']} |")
    lines+=['','位置误差按回正观察结束、尚未开始返回起点计算，不等同于最终稳定停车距离。',
        'Oracle 使用未来实测姿态，仅用于定位误差来源，不能作为在线预测效果。','',
        '## 已排查的替代解释','',
        '- 阶段事件与实际命令的时钟偏移波动约 0.63ms；没有发现秒级对齐错误。',
        '- 当前日志采用主机接收时间。`log_manager._cf_log_group_callback` 丢弃了设备时间戳；拟合延迟是“命令到收到状态”的有效延迟，不可全部解释为物理执行器延迟。',
        '- 外部 frames 位置记录支持试验 0/4/5 的真实回退；未拟合额外时间偏移，外部采集时钟延迟仍未独立验证。',
        '- 速度积分与机载/外部位置存在厘米级差异，不能把全部位置误差都归因于控制动力学。',
        '- 第二阶模型初始化必须考虑角速度；假设角速度为零的对比结果见上表。',
        '- 拟合使用稳定二阶参数、跨试验共享参数及固定多初始值，不拟合每次试验的独立偏差。','',
        '## 短时滚动预测（尚未接入控制）','',
        '每次从当时可用角度、角速度、位置、速度开始单独预测，预测期间不再修正状态。'
        '使用实际执行过的未来命令，不代表改变制动时机的控制器已得到验证。窗口相互重叠，不能当作独立试验。','',
        '| 预测时长 s | 速度 MAE / 最大误差 m/s | 位置 MAE / 最大误差 cm |','|---|---:|---:|']
    for horizon,row in report['rolling_horizon_summary'].items():
        lines.append(f"| {horizon} | {row['velocity_mae_m_s']:.4f}/{row['velocity_max_error_m_s']:.4f} | "
                     f"{100*row['position_mae_m']:.2f}/{100*row['position_max_error_m']:.2f} |")
    lines+=['','## 当前最小必要补测','',
        '**先只补固定 0.24s 条件的重复性验证：±Y 各三次，共六次。** 不需要为了这次分析重新拟合 XYZ observer。',
        '- 使用原先 20°、加速 0.24s、回正 0.20s、反向制动 0.24s、回正观察 0.65s。',
        '- 建议从 −Y 开始交替方向，与本轮正向先行顺序交叉；记录电量、试验顺序和所有入刹状态。',
        '- 保持既有起步稳定要求、速度/位移边界和定位保护。不扩大角度、速度或净空。',
        '- 先冻结本轮模型，直接预测新一轮；不要先用新数据调参再宣布验证通过。',
        '- 目标：区分同条件重复波动和模型结构误差。目前每种条件只有一次，时长与试验顺序/电量相关，现有日志无法回答重复性。',
        '- 下一轮建议同时记录设备采样时间戳与主机接收时间，不改变控制器当前时间语义。当前日志不足以分离遥测延迟与真实动力学延迟。',
        '', '**只有重复性可接受后，再补改变制动动作的试验：** 固定加速准备段，分别用 0.16/0.20/0.24s 制动脉冲，'
        '比较真实入刹速度、姿态和角速度相近的试验；不能仅凭准备时长假定入刹状态相同。正反向都保留，分批重复，仍遵守现有边界。',
        '这一步才直接验证“何时提前回正”。上述是待执行的补测方案，不是已部署的任务配置或飞行授权；独立姿态试验入口尚未改动。','',
        '## 文件','', '`report.json`：分段模型、全部留出指标、外部位置核对、滚动预测和非因果时间扫描。',
        '`split_comparison.png`：实际/预测姿态与速度。`motion_metrics.csv`、`tilt_metrics.csv`：逐次结果。','']
    return '\n'.join(lines)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log',type=Path); parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--no-plots',action='store_true'); args=parser.parse_args(argv)
    if args.output.exists():parser.error('choose a new output directory')
    records=json.loads(args.log.read_text())
    report,traces,angles=analyze_split(records)
    report['metadata'].update(source_log=str(args.log.resolve()),source_sha256=hashlib.sha256(args.log.read_bytes()).hexdigest())
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    (args.output/'README.md').write_text(markdown(report))
    write_csv(args.output/'motion_metrics.csv',report['motion_metrics'])
    write_csv(args.output/'tilt_metrics.csv',report['tilt_metrics'])
    write_csv(args.output/'motion_trajectories.csv',traces);write_csv(args.output/'tilt_trajectories.csv',angles)
    if not args.no_plots:render(args.output,report,traces,angles)
    print(json.dumps(report['summary'],indent=2))


if __name__=='__main__':main()
