"""Offline bench diagnostics; body acceleration comparisons are not external force."""
from __future__ import annotations

import html
import json
from pathlib import Path

import numpy as np

from Interaction.imu_bench_diagnostic import FIELDS, PLAN, save_json


def causal_average(t, y, window=.08, max_gap=.005):
    """Exact left-held time integral. Never bridge missing data or partial windows."""
    t, y = np.asarray(t), np.asarray(y)
    result = np.full_like(y, np.nan, dtype=float)
    if len(t) < 2:
        return result
    if np.any(np.diff(t) <= 0):
        raise ValueError('average requires strictly increasing source time')
    integral = np.vstack([np.zeros(y.shape[1]), np.cumsum(np.diff(t)[:, None]*y[:-1], axis=0)])
    left = t-window
    j = np.searchsorted(t, left, side='right')-1
    good = j >= 0
    j = np.clip(j, 0, len(t)-1)
    bad = np.r_[0, np.cumsum(np.diff(t) > max_gap + 1e-9)]
    good &= bad - bad[j] == 0
    result[good] = (integral[good] - integral[j[good]] -
                    (left[good]-t[j[good]])[:, None]*y[j[good]]) / window
    return result


def decode(rows):
    """Unwrap producer clock; preserve discontinuities as separate segments."""
    samples, issues = [], {'duplicate_epochs': 0, 'clock_discontinuities': 0,
                          'invalid_payloads': 0, 'truncated_json_lines': 0}
    previous = None
    elapsed, segment = 0., 0
    for row in rows:
        if row.get('kind') != 'imu':
            continue
        try:
            data = row['data']
            epoch, tick = int(data['contactImu.epoch']), int(row['cf_log_tick_ms_mod24'])
            values = np.array([float(data[f]) for f in FIELDS])
            if not np.isfinite(values).all() or not 0 <= epoch < 65536 or not 0 <= tick < 2**24:
                raise ValueError('invalid values')
        except (KeyError, TypeError, ValueError):
            issues['invalid_payloads'] += 1
            previous = None
            segment += 1
            continue
        if previous is not None:
            delta, log_delta = (epoch-previous[0]) % 65536, (tick-previous[1]) % 2**24
            if delta == 0 and 0 <= log_delta < 32768:
                issues['duplicate_epochs'] += 1
                continue
            if not 0 < delta < 32768 or not 0 < log_delta < 2**23 or abs(delta-log_delta) > 5:
                issues['clock_discontinuities'] += 1
                segment += 1
                elapsed += .001
            else:
                elapsed += delta/1000.
        elif samples:
            elapsed += .001
        previous = epoch, tick
        samples.append({'t': elapsed, 'segment': segment, 'stage': row.get('stage', 'unknown'),
                        'acc': values[3:]*9.81, 'gyro': values[:3],
                        'host_ns': row['host_monotonic_ns']})
    return samples, issues


def sampling_comparison(t, acc):
    """10 phases of ideal 100 Hz sampling vs high-rate 80 ms body-axis mean."""
    high = causal_average(t, acc)
    variants = []
    for phase in range(10):
        low_t = np.arange(t[0]+phase/1000., t[-1], .01)
        idx = np.clip(np.searchsorted(t, low_t, side='right')-1, 0, len(t)-1)
        low = causal_average(low_t, acc[idx], max_gap=.011)
        reference = np.array([np.interp(low_t, t, high[:, j]) for j in range(3)]).T
        valid = np.isfinite(low).all(axis=1) & np.isfinite(reference).all(axis=1)
        if valid.any():
            variants.append({'phase_ms': phase, 'samples': int(valid.sum()),
                             'p95_abs_difference_m_s2': np.percentile(abs(low[valid]-reference[valid]), 95, axis=0).tolist(),
                             'max_abs_difference_m_s2': np.max(abs(low[valid]-reference[valid]), axis=0).tolist(),
                             'time': low_t, 'low': low, 'high': reference})
    return variants


def analyze(session):
    session = Path(session)
    manifest = json.loads((session/'session.json').read_text())
    rows, parse_errors = [], 0
    with (session/'packets.jsonl').open() as file:
        for line in file:
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                parse_errors += 1
    samples, issues = decode(rows)
    # Receiver timestamps determine phase boundaries even if the consumer was queued.
    intervals = manifest.get('stages', [])
    if intervals:
        for sample in samples:
            sample['stage'] = 'preparation'
            for interval in intervals:
                if interval['start_host_monotonic_ns'] <= sample['host_ns'] < interval.get('end_host_monotonic_ns', float('inf')):
                    sample['stage'] = interval['name']
                    break
    issues['truncated_json_lines'] = parse_errors
    period = manifest.get('telemetry', {}).get('requested_period_ms', 1)
    report = {'schema': 'imu_bench_report_v1', 'capture_status': manifest['status'],
              'synthetic': manifest.get('synthetic', False), 'issues': issues, 'stages': [],
              'calibration_applied': False,
              'scope': 'Processed body IMU; no external-force reconstruction or mechanical-mode identification.'}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 2, figsize=(12, 11))
    comp_fig, comp_ax = plt.subplots(figsize=(10, 4))
    plotted_comparison = False
    for k, (name, _, _) in enumerate(PLAN):
        selected = [s for s in samples if s['stage'] == name]
        item = {'name': name, 'samples': len(selected), 'high_rate_qualified': False,
                'requested_rate_qualified': False}
        report['stages'].append(item)
        axes[k, 0].set_title(name+' | body acceleration')
        axes[k, 1].set_title(name+' | body angular rate')
        if len(selected) < 2:
            continue
        t = np.array([s['t'] for s in selected]); acc = np.array([s['acc'] for s in selected]); gyro = np.array([s['gyro'] for s in selected])
        segments = np.array([s['segment'] for s in selected]); dt = np.diff(t)
        split = (dt > .005+1e-9) | (dt <= 0) | (np.diff(segments) != 0)
        chunks = np.split(np.arange(len(t)), np.flatnonzero(split)+1)
        longest = max(chunks, key=len)
        duration = float(t[-1]-t[0]); rate = (len(t)-1)/duration if duration > 0 else 0
        delivered = min(1., len(t)/max(1., duration*1000/period+1))
        item.update(duration_source_s=duration, received_source_rate_hz=rate,
                    nominal_sample_coverage=delivered, max_source_gap_ms=float(max(dt)*1000),
                    long_gaps=int(split.sum()), accel_mean_m_s2=np.mean(acc, axis=0).tolist(),
                    accel_std_m_s2=np.std(acc, axis=0).tolist(), gyro_mean_deg_s=np.mean(gyro, axis=0).tolist(),
                    gyro_std_deg_s=np.std(gyro, axis=0).tolist())
        item['high_rate_qualified'] = bool(period == 1 and rate >= 950 and not split.any() and
                                           not any(issues.values()) and duration >= 1)
        item['requested_rate_qualified'] = bool(rate >= .95*1000/period and not split.any() and
                                                not any(issues.values()) and duration >= 1)
        for j, label in enumerate('XYZ'):
            # NaN separators prevent a line implying continuity across a source gap.
            ap, gp = acc[:, j].copy(), gyro[:, j].copy()
            ap[np.flatnonzero(split)+1] = np.nan; gp[np.flatnonzero(split)+1] = np.nan
            axes[k, 0].plot(t-t[0], ap, lw=.6, label=label)
            axes[k, 1].plot(t-t[0], gp, lw=.6, label=label)
        variants = []
        # A partial segment is displayed only as exploratory, with its exact coverage.
        if len(longest) > 1 and t[longest[-1]]-t[longest[0]] >= 1 and period < 10:
            variants = sampling_comparison(t[longest], acc[longest])
            item['comparison_segment_s'] = [float(t[longest[0]]-t[0]), float(t[longest[-1]]-t[0])]
            item['sampling_phase_comparisons'] = [{kk: vv for kk, vv in v.items() if kk not in ('time','low','high')} for v in variants]
            if name == 'gentle_taps' and variants:
                worst = max(variants, key=lambda v: v['p95_abs_difference_m_s2'][1])
                comp_ax.plot(worst['time']-t[0], worst['high'][:, 1], label='High-rate 80 ms body-Y average')
                comp_ax.plot(worst['time']-t[0], worst['low'][:, 1], alpha=.8,
                             label=f'100 Hz sampled average (phase {worst["phase_ms"]} ms)')
                plotted_comparison = True
    for k in range(4):
        for j in range(2):
            axes[k,j].set_xlabel('Stage source time (s)'); axes[k,j].set_ylabel('m/s²' if j == 0 else 'deg/s'); axes[k,j].grid(alpha=.2)
            if axes[k,j].lines:
                axes[k,j].legend(fontsize=8)
    fig.suptitle('PROPS-OFF bench diagnostic | firmware-filtered IMU'+(' | SYNTHETIC' if report['synthetic'] else ''))
    fig.tight_layout(rect=[0,0,1,.97]); fig.savefig(session/'imu_stages.png', dpi=140); plt.close(fig)
    comp_ax.set(xlabel='Tap-stage source time (s)', ylabel='Body-Y acceleration mean (m/s²)',
                title='Sampling sensitivity only — not estimated contact force')
    if plotted_comparison:
        comp_ax.legend()
    else:
        comp_ax.text(.5,.5,'Insufficient contiguous high-rate tap data', ha='center', transform=comp_ax.transAxes)
    comp_ax.grid(alpha=.2); comp_fig.tight_layout(); comp_fig.savefig(session/'sampling_comparison.png', dpi=140); plt.close(comp_fig)
    static = [r for r in report['stages'] if r['name'] in ('static_before','static_after') and 'accel_mean_m_s2' in r]
    if len(static) == 2:
        report['static_mean_change_m_s2'] = (np.array(static[1]['accel_mean_m_s2'])-static[0]['accel_mean_m_s2']).tolist()
    save_json(session/'report.json', report)
    table = ''.join(f'<tr><td>{html.escape(r["name"])}</td><td>{r["samples"]}</td><td>{r.get("received_source_rate_hz",0):.1f}</td><td>{r.get("max_source_gap_ms",0):.1f}</td><td>{"合格" if r["requested_rate_qualified"] else "不足 / 仅供诊断"}</td></tr>' for r in report['stages'])
    body = f'''<!doctype html><meta charset="utf-8"><title>拆桨 IMU 诊断</title>
<style>body{{font:17px/1.7 system-ui;max-width:1100px;margin:35px auto;padding:0 20px;color:#253442}}img{{width:100%}}td,th{{padding:10px;border-bottom:1px solid #ddd;text-align:left}}table{{width:100%;border-collapse:collapse}}code{{word-break:break-all}}</style>
<h1>拆桨 IMU 诊断{' · 模拟数据' if report['synthetic'] else ''}</h1><p>采集状态：{html.escape(manifest['status'])}。仅记录和分析；没有写入校准参数或发送飞行指令。</p>
<p>四段动作：静止、缓慢 roll、轻推接触头后松开、再次静止。加速度包含重力，不能直接当作接触力。慢 roll 中出现的变化不等于噪声。</p>
<table><tr><th>阶段</th><th>有效样本</th><th>接收频率 Hz</th><th>最大源时间间隔 ms</th><th>请求 {1000/period:g} Hz 的数据质量</th></tr>{table}</table>
<p>本次请求 {1000/period:g} Hz。质量合格要求源时间连续、至少一秒、实际收到至少请求频率的 95%；重复、时钟异常、无效数据或超过 5 ms 缺口均会降级。完整采集与数据质量合格是两个独立结果。500 Hz 不会标成 1 kHz 合格；原始数值报告分别保留两项判定。问题计数：<code>{html.escape(json.dumps(issues))}</code></p>
<img src="imu_stages.png"><h2>80 ms 窗口与 100 Hz 采样对比</h2><img src="sampling_comparison.png">
<p>在最长连续轻触片段上，模拟十种 100 Hz 采样相位；图中选择 Y 轴差异较大的相位。两条线均作 80 ms 因果时间平均，比较的是机体系加速度，不是完整外力观测器。超过 5 ms 缺口不参与高频积分；不足整窗处不输出。质量不合格时只作探索性检查。</p>
<p>如果松开后出现连续衰减波动，支持接触激发的运动或局部振动；支撑方式会改变振动。100 Hz 模拟曲线与高频参考不同时，说明该信号对采样相位敏感。现有接口已经过固件滤波，不能分离滤波前的信号，也不能把频谱峰直接当成机架固有频率。电机不转时未复现，不能排除带桨工作时的问题。</p>
<p>前后静止均值差仅在姿态和支撑一致时可用于漂移检查；本工具不拟合或应用加速度校准。</p>
<p>可选 Vicon 保存的是未分配身份的原始点云和主机接收时间；未自动跟踪 lb11 或反推质心加速度，也没有当作 IMU 真值。若终止等待超时或存在错误文件，应单独检查光学记录。</p>
<p><a href="report.json">数值报告</a> · <a href="session.json">会话与阶段时间</a> · 原始 IMU/电机：packets.jsonl；可选光学：vicon.jsonl。</p>'''
    (session/'report.html').write_text(body)
    return report
