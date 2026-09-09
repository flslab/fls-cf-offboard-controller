"""Offline, causal stopping replay. Never writes flight calibration or sends commands.

Run with: python -m Interaction.braking_replay LOG.json --output NEW_DIRECTORY
For fixed-duration repeat tests, add --summary-only (observations, no model fit).
Only numpy is required for analysis; matplotlib is needed unless --no-plots.
"""
from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path
import hashlib

import numpy as np

from Interaction.wrench_model_calibration import identify_planar_braking_response


PHASES = ("level_before_acceleration", "accelerate", "level_before_brake",
          "brake", "level_after_brake", "recovery")


@dataclass
class Trial:
    segment: int
    duration: float
    direction: np.ndarray
    times: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray
    attitude_acceleration: np.ndarray
    command_times: np.ndarray
    commands: np.ndarray
    phase_times: dict
    max_gap: float


def attitude_acceleration(rpy, direction):
    """Small-planar-angle convention used by this experiment, in world axes.

    This is an initialization proxy, not an independently identified attitude
    model. No future attitude measurements enter the prediction.
    """
    roll, pitch, yaw = rpy
    body = -9.81 * np.tan([pitch, roll])
    c, s = np.cos(yaw), np.sin(yaw)
    return float(np.array([c * body[0] - s * body[1],
                           s * body[0] + c * body[1]]) @ direction)


def extract(records, max_gap_s=.10):
    """Align relative command clocks by event/next-command pairs, not 'start'.

    CommandWrapper and LiveLogger have different clock origins. Phase events
    immediately precede their command in log order. The offset spread reports
    uncertainty from logging/computation latency; offsets are never fit to motion.
    """
    saved = [r['data'] for r in records
             if r.get('name') == 'Wrench Model Calibration Saved']
    repeated = [r['data'] for r in records
                if r.get('name') == 'Planar Braking Repeat Test Complete']
    if len(saved) == 1 and not repeated and saved[0].get('planar_braking_fit'):
        fit = saved[0]['planar_braking_fit']
        source_metadata = dict(source_event='Wrench Model Calibration Saved',
                               calibration_saved_path=saved[0]['path'],
                               contains_fitted_model=True)
    elif len(repeated) == 1 and not saved:
        completion = repeated[0]
        if not isinstance(completion.get('protocol'), dict):
            raise ValueError('repeat-test completion event is missing its timing protocol')
        # This retains the extraction API without inventing any fitted dynamics.
        fit = dict(usable=False, offline_only=True, repeat_test=True,
                   maneuver_count=completion.get('maneuver_count'),
                   protocol=completion['protocol'])
        source_metadata = dict(source_event='Planar Braking Repeat Test Complete',
                               contains_fitted_model=False,
                               previous_calibration_preserved=completion.get('previous_calibration_preserved'),
                               calibration_reference=completion.get('calibration_reference'),
                               logged_sample_count=completion.get('sample_count'))
    else:
        raise ValueError('expected exactly one saved planar calibration or completed repeat test in a complete flight log')
    count = fit.get('maneuver_count')
    if isinstance(count, bool) or not isinstance(count, (int, float)) or not np.isfinite(count) or count < 1 or int(count) != count:
        raise ValueError('invalid maneuver count in completion event')
    phase_entries, offsets = {}, []
    for i, record in enumerate(records):
        if record.get('name') != 'Planar Braking Calibration Phase':
            continue
        event = record['data']
        if event['phase'] not in PHASES:
            continue
        following = None
        expected = ('Commander.send_position_setpoint' if event['phase'] == 'recovery'
                    else 'Commander.send_zdistance_setpoint')
        for candidate in records[i + 1:i + 100]:
            if candidate.get('type') == 'wrench_observer':
                break
            if candidate.get('name') == expected:
                following = candidate['data']
                break
        if following is None:
            raise ValueError(f'missing actual command after phase {event}')
        offsets.append(float(event['time']) - float(following['time']))
        phases = phase_entries.setdefault(int(event['segment_id']), {})
        if event['phase'] in phases:
            raise ValueError('duplicate trial phase')
        phases[event['phase']] = (event, following)
    if not offsets or np.ptp(offsets) > .01:
        raise ValueError('command/event clock alignment is missing or inconsistent (>10ms)')
    offset = float(np.median(offsets))
    observer = sorted((r['data'] for r in records if r.get('type') == 'wrench_observer'),
                      key=lambda r: float(r['state_time']))
    # Repeated polls are not new measurements; keep the first occurrence.
    unique = {}
    for row in observer:
        unique.setdefault(float(row['state_time']), row)
    observer = list(unique.values())
    state_times = np.array([r['state_time'] for r in observer])
    protocol = fit['protocol']
    expected_count = int(fit['maneuver_count'])
    if sorted(phase_entries) != list(range(expected_count)):
        raise ValueError('incomplete trial ids')
    trials, samples = [], []
    for segment, entries in sorted(phase_entries.items()):
        if set(entries) != set(PHASES):
            raise ValueError(f'trial {segment} missing phases')
        phase_times = {name: float(entries[name][1]['time']) + offset for name in PHASES}
        if np.any(np.diff(list(phase_times.values())) <= 0):
            raise ValueError('nonmonotonic phase schedule')
        direction = np.array(entries['accelerate'][0]['direction_xy'], dtype=float)
        if direction.shape != (2,) or not np.isfinite(direction).all() or np.linalg.norm(direction) < .9:
            raise ValueError('invalid trial direction')
        direction /= np.linalg.norm(direction)
        start, end = phase_times[PHASES[0]], phase_times['recovery']
        first = np.searchsorted(state_times, start, side='right') - 1
        last = np.searchsorted(state_times, end, side='left')
        if first < 0 or last - first < 20:
            raise ValueError('insufficient pre-command and observation samples')
        rows = observer[first:last]
        times = state_times[first:last]
        gaps = np.r_[np.diff(times), start - times[0], end - times[-1]]
        if max(gaps) > max_gap_s or any(float(r['state_group_skew_s']) > .03 for r in rows):
            raise ValueError(f'trial {segment}: stale/gapped or skewed telemetry')
        pos = np.array([r['position_m'][:2] for r in rows]) @ direction
        vel = np.array([r['velocity_m_s'][:2] for r in rows]) @ direction
        att = np.array([attitude_acceleration(r['orientation_rpy_rad'], direction) for r in rows])
        if not np.isfinite(np.r_[times, pos, vel, att]).all():
            raise ValueError('nonfinite telemetry')
        command_times = np.array([phase_times[name] for name in PHASES[:-1]])
        commands = np.array([np.array(entries[name][0]['command_acceleration_xy_m_s2']) @ direction
                             for name in PHASES[:-1]])
        if not np.isfinite(np.r_[command_times, commands]).all():
            raise ValueError('nonfinite commands')
        # Verify the requested phase really appears in sent roll/pitch commands.
        for name, u in zip(PHASES[:-1], commands):
            sent = entries[name][1]['args']
            sent_tilt = float(np.hypot(*sent[:2]))
            if not np.isclose(9.81 * np.tan(np.radians(sent_tilt)), abs(u), atol=.01):
                raise ValueError('phase acceleration does not match sent tilt')
        durations = protocol.get('trial_accelerate_s')
        if durations is None:
            durations = [protocol['accelerate_s']] * expected_count
        if len(durations) != expected_count or not np.isfinite(durations).all() or min(durations) <= 0:
            raise ValueError('invalid trial acceleration durations in completion protocol')
        duration = durations[segment]
        trial = Trial(segment, float(duration), direction, times, pos, vel, att,
                      command_times, commands, phase_times, float(max(gaps)))
        trials.append(trial)
        for j, t in enumerate(times):
            k = np.searchsorted(command_times, t, side='right') - 1
            if k < 0:
                continue
            samples.append(dict(segment_id=segment, timestamp=float(t),
                                command_started_at=float(command_times[k]), phase=PHASES[k],
                                direction_xy=direction.tolist(),
                                command_acceleration_xy=(commands[k] * direction).tolist(),
                                velocity_xy=(vel[j] * direction).tolist()))
    metadata = dict(command_clock_offset_s=offset,
                    command_clock_offset_spread_s=float(np.ptp(offsets)),
                    command_clock_method='median phase-event minus next-command timestamp; approximate send anchor',
                    flight_git=next((r['data'] for r in records if r.get('type') == 'git'), None))
    metadata.update(source_metadata)
    return fit, trials, samples, metadata


def step(a, v, p, target, tau, dt, bias=0., drag=0.):
    """Exact integration of a'=(target-a)/tau, v'=a+bias-drag*v, p'=v.

    The same function is used by trajectory fitting and out-of-sample replay.
    """
    if drag < 0 or dt < 0 or tau < 0:
        raise ValueError('drag, dt and tau must be nonnegative')
    if drag > 1e-8:
        c = drag
        e = np.exp(-c * dt)
        f = -np.expm1(-c * dt) / c
        # Integral of f, with a cancellation-safe series near zero.
        h = ((dt-f)/c if c*dt > 1e-4 else
             dt**2 * (.5-c*dt/6+(c*dt)**2/24-(c*dt)**3/120))
        v1 = v * e + (target + bias) * f
        p1 = p + v * f + (target + bias) * h
        if tau <= 0:
            return target, v1, p1
        r = 1. / tau
        er = np.exp(-r * dt)
        fr = -np.expm1(-r * dt) / r
        if abs(c-r) < 1e-6 * max(c, r):
            transient_v = dt * e
            transient_p = ((f - dt*e)/c if c*dt > 1e-4 else
                           dt**2 * (.5-c*dt/3+(c*dt)**2/8-(c*dt)**3/30))
        else:
            transient_v = (er-e)/(c-r)
            transient_p = (fr-f)/(c-r)
        delta = a-target
        return target+delta*er, v1+delta*transient_v, p1+delta*transient_p
    if tau <= 0:
        return target, v + (target + bias) * dt, p + v * dt + .5 * (target + bias) * dt**2
    decay = np.exp(-dt / tau)
    impulse = (a - target) * tau * (-np.expm1(-dt / tau))
    distance = (a - target) * tau * (dt - tau * (-np.expm1(-dt / tau)))
    return (target + (a - target) * decay,
            v + (target + bias) * dt + impulse,
            p + v * dt + .5 * (target + bias) * dt**2 + distance)


def replay(trial, fit, initialization='command_history'):
    """Roll forward without future state corrections, from last state before brake.

    History mode warms the effective acceleration state using past commands;
    attitude mode initializes it from measured tilt at the same anchor. Both
    retain delayed pending commands. Saved parameters describe an effective
    command-to-acceleration lag, not a separately fitted attitude transfer model.
    """
    delay = float(fit['command_delay_s'])
    tau = float(fit['command_time_constant_s'])
    gain = float(fit['horizontal_acceleration_scale'])
    bias = (float(fit.get('acceleration_bias_m_s2', 0))
            + float(fit.get('world_y_bias_m_s2', 0)) * float(trial.direction[1]))
    drag = float(fit.get('linear_drag_per_s', 0))
    if not np.isfinite([delay, tau, gain, bias, drag]).all() or min(delay, tau, drag) < 0 or gain <= 0:
        raise ValueError('invalid fitted dynamics')
    anchor = np.searchsorted(trial.times, trial.phase_times['brake'], side='right') - 1
    t0 = trial.times[anchor]
    shifted = trial.command_times + delay
    grid = np.unique(np.r_[trial.times, shifted[(shifted > trial.times[0]) & (shifted < trial.times[-1])]])
    a = gain * trial.attitude_acceleration[0]
    v, p = float(trial.velocities[0]), float(trial.positions[0])
    outputs = {}
    for i, t in enumerate(grid):
        if t == t0:
            v, p = float(trial.velocities[anchor]), float(trial.positions[anchor])
            if initialization == 'measured_attitude':
                a = gain * trial.attitude_acceleration[anchor]
            elif initialization != 'command_history':
                raise ValueError('unknown initialization')
        if t >= t0:
            outputs[t] = (v, p, a + bias - drag*v)
        if i + 1 < len(grid):
            k = np.searchsorted(shifted, t, side='right') - 1
            u = trial.commands[k] if k >= 0 else 0.
            a, v, p = step(a, v, p, gain * u, tau, float(grid[i+1] - t), bias, drag)
    result = np.array([outputs[t] for t in trial.times[anchor:]])
    return anchor, result


def first_crossing(times, velocities, positions):
    """Positive-to-nonpositive crossing, NOT a stable-stop certificate."""
    for i in range(1, len(times)):
        if velocities[i-1] > 0 and velocities[i] <= 0:
            w = velocities[i-1] / (velocities[i-1] - velocities[i])
            return {'time_s': float(times[i-1] + w * (times[i] - times[i-1])),
                    'position_m': float(positions[i-1] + w * (positions[i] - positions[i-1]))}
    return None


def metrics(trial, fit, initialization='command_history'):
    anchor, pred = replay(trial, fit, initialization)
    t = trial.times[anchor:] - trial.times[anchor]
    v, p = trial.velocities[anchor:], trial.positions[anchor:] - trial.positions[anchor]
    pv, pp = pred[:, 0], pred[:, 1] - trial.positions[anchor]
    tail = t >= t[-1] - .10
    actual_cross, predicted_cross = first_crossing(t, v, p), first_crossing(t, pv, pp)
    actual_reverse, predicted_reverse = bool(np.min(v) < -.03), bool(np.min(pv) < -.03)
    measured_integral = float(np.sum(.5 * (v[:-1] + v[1:]) * np.diff(t)))
    row = dict(segment=trial.segment, pulse_duration_s=trial.duration,
               direction_xy=trial.direction.tolist(), initialization=initialization,
               anchor_before_brake_s=float(trial.phase_times['brake'] - trial.times[anchor]),
               observation_s=float(t[-1]), max_sample_gap_s=trial.max_gap,
               initial_speed_m_s=float(v[0]),
               actual_terminal_mean_m_s=float(np.mean(v[tail])),
               predicted_terminal_mean_m_s=float(np.mean(pv[tail])),
               terminal_mean_error_m_s=float(np.mean(pv[tail] - v[tail])),
               velocity_rmse_m_s=float(np.sqrt(np.mean((pv-v)**2))),
               position_rmse_m=float(np.sqrt(np.mean((pp-p)**2))),
               actual_end_displacement_m=float(p[-1]), predicted_end_displacement_m=float(pp[-1]),
               end_position_error_m=float(pp[-1]-p[-1]),
               measured_velocity_integral_minus_position_m=measured_integral-float(p[-1]),
               predicted_end_minus_measured_velocity_integral_m=float(pp[-1])-measured_integral,
               actual_min_speed_m_s=float(min(v)), predicted_min_speed_m_s=float(min(pv)),
               actual_reverse_over_0_03=actual_reverse, predicted_reverse_over_0_03=predicted_reverse,
               reversal_classification_correct=actual_reverse == predicted_reverse,
               actual_first_zero_crossing=actual_cross, predicted_first_zero_crossing=predicted_cross,
               crossing_time_error_s=(None if not actual_cross or not predicted_cross
                                      else predicted_cross['time_s'] - actual_cross['time_s']),
               crossing_position_error_m=(None if not actual_cross or not predicted_cross
                                          else predicted_cross['position_m'] - actual_cross['position_m']))
    trace = [dict(segment=trial.segment, initialization=initialization, elapsed_s=float(ti),
                  actual_velocity_m_s=float(vi), predicted_velocity_m_s=float(vpi),
                  actual_displacement_m=float(pi), predicted_displacement_m=float(ppi))
             for ti, vi, vpi, pi, ppi in zip(t, v, pv, p, pp)]
    return row, trace


def holdout_fits(trials, samples, saved):
    """Leave both signs of one nominal pulse duration out of parameter fitting.

    Minimum training trials/sign is two because there are four training trials.
    This is diagnostic CV, not authorization to save/enable a flight calibration.
    """
    folds = []
    if len({t.duration for t in trials}) < 3:
        raise ValueError('duration holdout needs >=3 durations with opposed directions; '
                         'use --summary-only for fixed-duration repeat tests')
    for duration in sorted({t.duration for t in trials}):
        held = [t.segment for t in trials if t.duration == duration]
        train = [t.segment for t in trials if t.segment not in held]
        training_samples = [s for s in samples if s['segment_id'] in train]
        if len(train) < 4 or len(held) < 2:
            raise ValueError('duration holdout needs >=3 durations with opposed directions')
        fit = identify_planar_braking_response(
            training_samples, expected_maneuver_count=len(train),
            minimum_trials_per_direction=2, minimum_r_squared=.70,
            minimum_validation_r_squared=.50, max_delay_s=.25, max_time_constant_s=.25,
            maximum_acceleration_extrapolation_ratio=saved.get('maximum_acceleration_extrapolation_ratio', 1.25))
        folds.append(dict(held_duration_s=duration, train_segments=train, held_segments=held, fit=fit))
    return folds


def analyze(records, max_gap_s=.10):
    saved, trials, samples, metadata = extract(records, max_gap_s)
    if saved.get('repeat_test'):
        raise ValueError('repeat-test log contains no fitted model; use --summary-only '
                         'for observed repeatability (not model validation)')
    folds = holdout_fits(trials, samples, saved)
    results, traces = [], []
    for trial in trials:
        fold = next(f for f in folds if trial.segment in f['held_segments'])
        for mode, fit in [('saved_in_sample', saved), ('held_duration_out', fold['fit'])]:
            for initialization in ['command_history', 'measured_attitude']:
                row, trace = metrics(trial, fit, initialization)
                row.update(mode=mode, fit_usable=bool(fit['usable']))
                results.append(row)
                traces.extend(dict(x, mode=mode) for x in trace)
    return dict(metadata=metadata, saved_fit=saved, holdout_folds=folds, metrics=results), traces, trials


def summarize_observations(records, max_gap_s=.10):
    """Observed braking and repeat spread, with no fitting or future-flight claims."""
    completion, trials, _, metadata = extract(records, max_gap_s)
    rows, traces = [], []
    for trial in trials:
        anchor = np.searchsorted(trial.times, trial.phase_times['brake'], side='right') - 1
        t = trial.times[anchor:] - trial.times[anchor]
        v = trial.velocities[anchor:]
        p = trial.positions[anchor:] - trial.positions[anchor]
        tail = t >= t[-1] - .10
        rollback = np.maximum.accumulate(p) - p
        rows.append(dict(segment=trial.segment, pulse_duration_s=trial.duration,
                         direction_xy=trial.direction.tolist(),
                         anchor_before_brake_s=float(trial.phase_times['brake'] - trial.times[anchor]),
                         observation_s=float(t[-1]), max_sample_gap_s=trial.max_gap,
                         initial_speed_m_s=float(v[0]),
                         actual_terminal_mean_m_s=float(np.mean(v[tail])),
                         actual_min_speed_m_s=float(np.min(v)),
                         actual_end_displacement_m=float(p[-1]),
                         actual_max_forward_displacement_m=float(np.max(p)),
                         actual_max_rollback_m=float(np.max(rollback)),
                         actual_terminal_rollback_m=float(rollback[-1]),
                         actual_reverse_over_0_03=bool(np.min(v) < -.03),
                         actual_first_zero_crossing=first_crossing(t, v, p),
                         measured_velocity_integral_minus_position_m=float(
                             np.sum(.5 * (v[:-1] + v[1:]) * np.diff(t)) - p[-1])))
        traces.extend(dict(segment=trial.segment, elapsed_s=float(ti),
                           actual_velocity_m_s=float(vi), actual_displacement_m=float(pi))
                      for ti, vi, pi in zip(t, v, p))
    groups = []
    keys = sorted({(r['pulse_duration_s'], tuple(r['direction_xy'])) for r in rows})
    for duration, direction in keys:
        members = [r for r in rows if r['pulse_duration_s'] == duration and tuple(r['direction_xy']) == direction]
        group = dict(pulse_duration_s=duration, direction_xy=list(direction),
                     segments=[r['segment'] for r in members], count=len(members))
        for quantity in ['initial_speed_m_s', 'actual_terminal_mean_m_s', 'actual_max_rollback_m']:
            values = np.array([r[quantity] for r in members])
            group[quantity] = dict(mean=float(np.mean(values)), minimum=float(np.min(values)),
                                   maximum=float(np.max(values)),
                                   sample_std=(float(np.std(values, ddof=1)) if len(values) > 1 else None))
        groups.append(group)
    metadata.update(analysis_mode='observed_repeatability_only', offline_only=True,
                    independent_model_validation=False,
                    observation_end='last telemetry sample before position recovery')
    return dict(metadata=metadata, protocol=completion['protocol'], metrics=rows,
                repeat_groups=groups), traces, trials


def observed_markdown_report(report):
    lines = ['# Observed braking repeat test', '',
             'Offline measurements only: no model fitting, validation, flight command or calibration-file changes.', '',
             'Positive is the trial acceleration direction. Each trace starts at the last state before the brake command and ends before position recovery.',
             'Terminal speed is the final 0.10 s mean. Rollback is the largest decline from a preceding position peak in this window, not a position-recovery command.', '',
             '| Trial | Direction | Pulse (s) | Brake-entry speed (m/s) | Terminal speed (m/s) | Max rollback (cm) |',
             '|---|---|---:|---:|---:|---:|']
    for r in report['metrics']:
        lines.append(f"| {r['segment']} | {r['direction_xy']} | {r['pulse_duration_s']:.2f} | "
                     f"{r['initial_speed_m_s']:.3f} | {r['actual_terminal_mean_m_s']:.3f} | "
                     f"{100*r['actual_max_rollback_m']:.2f} |")
    lines += ['', '## Repeat spread', '',
              'Compare like duration and direction only. Three repeats in one flight are not independent-flight certification.', '',
              '| Direction | Pulse (s) | N | Terminal mean (m/s) | Terminal sample SD | Terminal min / max |',
              '|---|---:|---:|---:|---:|---|']
    for group in report['repeat_groups']:
        stats = group['actual_terminal_mean_m_s']
        sd = 'n/a' if stats['sample_std'] is None else f"{stats['sample_std']:.3f}"
        lines.append(f"| {group['direction_xy']} | {group['pulse_duration_s']:.2f} | {group['count']} | "
                     f"{stats['mean']:.3f} | {sd} | {stats['minimum']:.3f} / {stats['maximum']:.3f} |")
    lines += ['', 'No stopping guarantee follows from these observations. A zero crossing alone is not stable stopping.',
              'Position is onboard telemetry; velocity-integral disagreement is recorded in metrics.csv. Repeat spread can also reflect entry speed, attitude, battery and order.',
              'Use a model frozen from the earlier flight for any later independent prediction test; do not refit on these trials and call them validation.', '',
              '## Data provenance', '', '```json', json.dumps(report['metadata'], indent=2), '```', '',
              '`report.json` contains per-trial measurements, protocol and grouped statistics. `trajectories.csv` contains measured traces only.',
              'Summary-only mode intentionally generates no model-comparison plots.', '']
    return '\n'.join(lines)


def write_csv(path, rows):
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows({k: json.dumps(v) if isinstance(v, (list, dict)) else v
                          for k, v in row.items()} for row in rows)


def render_plots(output, report, traces, trials):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for initialization in ['command_history', 'measured_attitude']:
        fig, axes = plt.subplots(len(trials), 2, figsize=(12, 2.5 * len(trials)), squeeze=False)
        for i, trial in enumerate(trials):
            for mode, label, color in [('saved_in_sample', 'Saved fit (in-sample)', '#2675bc'),
                                       ('held_duration_out', 'Held-duration-out', '#db8030')]:
                rows = [x for x in traces if x['segment'] == trial.segment and x['mode'] == mode
                        and x['initialization'] == initialization]
                for j, quantity in enumerate(['velocity_m_s', 'displacement_m']):
                    ax = axes[i, j]
                    if mode == 'saved_in_sample':
                        ax.plot([x['elapsed_s'] for x in rows], [x['actual_'+quantity] for x in rows],
                                color='black', label='Measured', lw=1.8)
                    ax.plot([x['elapsed_s'] for x in rows], [x['predicted_'+quantity] for x in rows],
                            color=color, label=label, ls='--')
                    if mode == 'saved_in_sample':
                        anchor = trial.times[np.searchsorted(trial.times, trial.phase_times['brake'], side='right')-1]
                        ax.axvline(trial.phase_times['level_after_brake']-anchor, color='#888', ls=':', label='Level command')
                        ax.axhline(0, color='#aaa', lw=.7)
                        ax.set_title(f"Trial {trial.segment}: {trial.duration:.2f}s, direction {trial.direction.tolist()}")
                        ax.set_ylabel('Signed speed (m/s)' if j == 0 else 'Signed displacement (m)')
                        ax.set_xlabel('Time since last measurement before brake (s)')
                        ax.grid(alpha=.2)
        axes[0, 0].legend(fontsize=8)
        fig.suptitle(f'Open-loop stopping replay — {initialization}\nNo future state resets; excludes position recovery', fontsize=13)
        fig.tight_layout(rect=(0, 0, 1, .965))
        fig.savefig(output / f'{initialization}.png', dpi=150)
        plt.close(fig)


def markdown_report(report):
    lines = ['# Braking prediction replay', '',
             'Offline diagnostic only. No controller or calibration files were changed.', '',
             '## Method', '',
             '- Causal continuous first-order acceleration lag with saved/refitted delay, gain and bias; no drag term.',
             '- Integration is analytic between delayed command changes; calibration fitting uses its existing sampled low-pass/window regression. This is a causal continuous realization of those parameters, not bit-identical fitter interpolation.',
             '- Warm up acceleration from preceding commands, then anchor measured position/velocity once, just before brake.',
             '- Alternate measured-attitude initialization is a sensitivity check, not a separately calibrated attitude model.',
             '- Actual command schedule is replayed. This does not validate a counterfactual earlier release of braking.',
             '- Saved-fit replay is in-sample. Held-duration-out fits four entire trials and predicts the other two.',
             '- CV is within one flight, with only two training trials per sign; not independent-flight certification.',
             '- Zero crossing is not stable stopping. Missing crossings are censored at the last sample before position recovery.',
             '- Position uses onboard state, not independently verified ground truth. Timing offset is inferred from adjacent log records.',
             '', '## Command-history initialization', '',
             '| Trial | T (s) | Actual terminal (m/s) | Saved predicted | Held-out predicted | Held-out end-position error (cm) | Reversal detected? |',
             '|---|---:|---:|---:|---:|---:|---|']
    rows = report['metrics']
    for r in rows:
        if r['mode'] != 'held_duration_out' or r['initialization'] != 'command_history':
            continue
        saved = next(x for x in rows if x['mode'] == 'saved_in_sample' and x['segment'] == r['segment']
                     and x['initialization'] == 'command_history')
        lines.append(f"| {r['segment']} | {r['pulse_duration_s']:.2f} | {r['actual_terminal_mean_m_s']:.3f} | "
                     f"{saved['predicted_terminal_mean_m_s']:.3f} | {r['predicted_terminal_mean_m_s']:.3f} | "
                     f"{r['end_position_error_m']*100:+.2f} | actual={r['actual_reverse_over_0_03']}, predicted={r['predicted_reverse_over_0_03']} |")
    lines += ['', 'Terminal speed is mean over the final 0.10s; reverse threshold is -0.03m/s (diagnostic, not a flight limit).',
              'Positive error means predicted farther forward than measured. Full crossing metrics and both initializations are in metrics.csv.',
              'Velocity-integral vs onboard-position disagreement is also reported; endpoint error is not purely dynamics-model error.',
              '', '## Data quality', '', '```json', json.dumps(report['metadata'], indent=2), '```', '',
              '## Output files', '', '`report.json`: fitted parameters, train/holdout ids, all metrics.',
              '`metrics.csv`: per-trial errors. `trajectories.csv`: measured/predicted samples.',
              '`command_history.png`, `measured_attitude.png`: six-trial comparisons (unless --no-plots).', '']
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='new directory; existing paths are refused')
    parser.add_argument('--max-gap-s', type=float, default=.10)
    parser.add_argument('--no-plots', action='store_true')
    parser.add_argument('--summary-only', action='store_true',
                        help='observed terminal speed/rollback and repeat spread only; no fit or model validation')
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('output already exists; choose a new directory')
    if not np.isfinite(args.max_gap_s) or args.max_gap_s <= 0:
        parser.error('--max-gap-s must be positive and finite')
    with args.log.open() as f:
        records = json.load(f)
    try:
        analysis = summarize_observations if args.summary_only else analyze
        report, traces, trials = analysis(records, args.max_gap_s)
    except ValueError as exc:
        parser.error(str(exc))
    report['metadata']['source_log'] = str(args.log.resolve())
    with args.log.open('rb') as f:
        digest = hashlib.sha256()
        for block in iter(lambda: f.read(1024 * 1024), b''):
            digest.update(block)
    report['metadata']['source_sha256'] = digest.hexdigest()
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    write_csv(args.output / 'metrics.csv', report['metrics'])
    write_csv(args.output / 'trajectories.csv', traces)
    renderer = observed_markdown_report if args.summary_only else markdown_report
    (args.output / 'README.md').write_text(renderer(report))
    if not args.no_plots and not args.summary_only:
        render_plots(args.output, report, traces, trials)
    verb = 'Summarized observed' if args.summary_only else 'Replayed'
    print(f'{verb} {len(trials)} trials; report: {args.output / "README.md"}')


if __name__ == '__main__':
    main()
