"""Offline full-trajectory model comparison; never exports flight calibration.

python -m Interaction.braking_trajectory_fit LOG.json --output NEW_DIRECTORY
Requires numpy, scipy, and (unless --no-plots) matplotlib.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from Interaction.braking_replay import extract, holdout_fits, metrics, replay, write_csv


CANDIDATES = ('trajectory_base', 'trajectory_world_bias', 'trajectory_drag_world_bias')
# Declared before evaluating held-out trials. Never initialize from an all-data fit.
STARTS = ((.05, .10, 1., 0.), (.10, .05, 1., 0.),
          (.02, .18, 1., 0.), (.15, .03, 1., 0.))
LOWER = (0., .005, .2, -.5)
UPPER = (.25, .25, 2.5, .5)


def relative_trial(trial):
    """Remove epoch time before finite differences: nanosecond steps would quantize."""
    origin = float(trial.times[0])
    return replace(trial, times=trial.times-origin, command_times=trial.command_times-origin,
                   phase_times={k: v-origin for k, v in trial.phase_times.items()})


def parameters(values, model):
    delay, tau, gain, bias = map(float, values[:4])
    result = dict(command_delay_s=delay, command_time_constant_s=tau,
                  horizontal_acceleration_scale=gain, acceleration_bias_m_s2=0.,
                  world_y_bias_m_s2=0., linear_drag_per_s=0.)
    if model == 'trajectory_base':
        result['acceleration_bias_m_s2'] = bias
    elif model in ('trajectory_world_bias', 'trajectory_drag_world_bias'):
        result['world_y_bias_m_s2'] = bias
    else:
        raise ValueError(f'unknown model {model}')
    if model == 'trajectory_drag_world_bias':
        result['linear_drag_per_s'] = float(values[4])
    return result


def trajectory_residual(values, model, trials, terminal_weight=1.):
    """Equal trial weighting: time-mean speed error plus final 0.1s mean error.

    Positions, held-out trajectories and zero crossings are not training targets.
    Only measured p/v at the brake anchor and preceding state/history initialize
    replay. The complete brake/level tail is predicted open-loop afterwards.
    """
    fit = parameters(values, model)
    residuals = []
    for trial in trials:
        anchor, predicted = replay(trial, fit)
        times = trial.times[anchor:]
        dt = np.diff(times)
        weights = np.r_[dt[0]/2, (dt[:-1]+dt[1:])/2, dt[-1]/2]
        weights /= weights.sum()
        error = predicted[:, 0] - trial.velocities[anchor:]
        tail = times >= times[-1]-.1
        residuals.extend(error*np.sqrt(weights))
        residuals.append(np.sqrt(terminal_weight)*float(np.mean(error[tail])))
    return np.asarray(residuals) / np.sqrt(len(trials))


def fit_trajectories(trials, model, starts=STARTS):
    if len(trials) < 4:
        raise ValueError('at least four training trials are required')
    if any(abs(float(t.direction[1])) < .98 for t in trials):
        raise ValueError('world-Y candidates require opposed Y trials')
    if not any(t.direction[1] > 0 for t in trials) or not any(t.direction[1] < 0 for t in trials):
        raise ValueError('both Y directions must be present in training')
    trials = [relative_trial(t) for t in trials]
    lower, upper = list(LOWER), list(UPPER)
    if model == 'trajectory_drag_world_bias':
        lower.append(0.)
        upper.append(3.)
    optimizations = []
    for start in starts:
        initial = list(start)
        if model == 'trajectory_drag_world_bias':
            initial.append(.3)
        result = least_squares(trajectory_residual, initial, bounds=(lower, upper),
                               args=(model, trials), max_nfev=180,
                               ftol=1e-8, xtol=1e-8, gtol=1e-8,
                               x_scale='jac')
        optimizations.append(result)
    converged = [r for r in optimizations if r.success and np.isfinite(r.fun).all()]
    if not converged:
        raise ValueError(f'no converged fit for {model}')
    best = min(converged, key=lambda r: float(r.fun @ r.fun))
    fit = parameters(best.x, model)
    singular_values = np.linalg.svd(best.jac, compute_uv=False)
    fit.update(model=model, offline_only=True, optimizer_success=bool(best.success),
               train_segments=[t.segment for t in trials],
               objective_rmse_m_s=float(np.sqrt(np.sum(best.fun**2))),
               parameter_order=['delay_s', 'tau_s', 'gain',
                                'parallel_bias_m_s2' if model == 'trajectory_base' else 'world_y_bias_m_s2']
                               + (['drag_per_s'] if model == 'trajectory_drag_world_bias' else []),
               bound_active=best.active_mask.tolist(),
               jacobian_condition_number=(None if singular_values[-1] < 1e-12
                                          else float(singular_values[0]/singular_values[-1])),
               initial_seeds=[list(s) for s in starts],
               seed_objectives=[float(np.sqrt(r.fun@r.fun)) for r in optimizations],
               seed_success=[bool(r.success) for r in optimizations],
               training_objective='equal-trial time-mean velocity squared error + terminal-mean squared error',
               terminal_weight=1., position_in_objective=False)
    return fit


def aggregate(rows):
    result = {}
    for model in sorted({r['model'] for r in rows}):
        selected = [r for r in rows if r['model'] == model and r['evaluation'] == 'held_duration_out']
        if not selected:
            continue
        crossing = [abs(r['crossing_time_error_s']) for r in selected if r['crossing_time_error_s'] is not None]
        result[model] = dict(
            velocity_rmse_m_s=float(np.sqrt(np.mean([r['velocity_rmse_m_s']**2 for r in selected]))),
            terminal_mean_mae_m_s=float(np.mean([abs(r['terminal_mean_error_m_s']) for r in selected])),
            terminal_mean_max_abs_error_m_s=float(max(abs(r['terminal_mean_error_m_s']) for r in selected)),
            end_position_mae_m=float(np.mean([abs(r['end_position_error_m']) for r in selected])),
            end_position_max_abs_error_m=float(max(abs(r['end_position_error_m']) for r in selected)),
            zero_crossing_mean_abs_time_error_s=float(np.mean(crossing)) if crossing else None,
            zero_crossing_comparable_count=len(crossing),
            zero_crossing_missed=sum(r['actual_first_zero_crossing'] is not None and r['predicted_first_zero_crossing'] is None for r in selected),
            zero_crossing_spurious=sum(r['actual_first_zero_crossing'] is None and r['predicted_first_zero_crossing'] is not None for r in selected),
            reversal_missed=sum(r['actual_reverse_over_0_03'] and not r['predicted_reverse_over_0_03'] for r in selected),
            reversal_false_alarm=sum(not r['actual_reverse_over_0_03'] and r['predicted_reverse_over_0_03'] for r in selected),
        )
    return result


def compare(records, progress=print):
    saved, original_trials, samples, metadata = extract(records)
    trials = [relative_trial(t) for t in original_trials]
    legacy_folds = holdout_fits(original_trials, samples, saved)
    folds, rows, traces = [], [], []
    for legacy in legacy_folds:
        held_ids = legacy['held_segments']
        training = [t for t in trials if t.segment not in held_ids]
        held = [t for t in trials if t.segment in held_ids]
        models = {'legacy_window': legacy['fit']}
        for model in CANDIDATES:
            progress(f'Fitting {model}; hold out trials {held_ids}', flush=True)
            models[model] = fit_trajectories(training, model)
        folds.append(dict(held_segments=held_ids, train_segments=legacy['train_segments'], models=models))
        for model, fit in models.items():
            for trial in held:
                row, trace = metrics(trial, fit)
                row.update(model=model, evaluation='held_duration_out')
                rows.append(row)
                traces.extend(dict(x, model=model, evaluation='held_duration_out') for x in trace)
    all_fits = {}
    for model in CANDIDATES:
        progress(f'Fitting {model} on all six trials for diagnostics only', flush=True)
        all_fits[model] = fit_trajectories(trials, model)
    for trial in trials:
        for model, fit in {'saved_window': saved, **all_fits}.items():
            row, _ = metrics(trial, fit)
            rows.append(dict(row, model=model, evaluation='in_sample'))
    attitude_tail = []
    for trial in trials:
        tail = trial.times >= trial.times[-1]-.1
        # Independent diagnostic; none of these future attitude samples enter predictions.
        attitude_tail.append(dict(segment=trial.segment,
            mean_signed_tilt_acceleration_m_s2=float(np.mean(trial.attitude_acceleration[tail])),
            mean_world_y_tilt_acceleration_m_s2=float(np.mean(trial.attitude_acceleration[tail])*trial.direction[1]),
            mean_signed_velocity_m_s=float(np.mean(trial.velocities[tail]))))
    summary = aggregate(rows)
    base = summary['trajectory_base']
    # Descriptive non-inferiority check, not a flight qualification threshold.
    for model in CANDIDATES[1:]:
        value = summary[model]
        value['dominates_base_on_selected_cv_metrics'] = all(
            value[k] <= base[k] for k in ('velocity_rmse_m_s', 'terminal_mean_mae_m_s',
                'terminal_mean_max_abs_error_m_s', 'end_position_mae_m',
                'end_position_max_abs_error_m', 'reversal_missed', 'reversal_false_alarm'))
    return dict(metadata=metadata, offline_only=True,
                note='No model is auto-selected or exported for flight. Same-flight CV is preliminary.',
                candidates=list(CANDIDATES), holdout_folds=folds, all_data_fits=all_fits,
                metrics=rows, summary=summary, measured_attitude_tail=attitude_tail), traces, trials


def report_markdown(report):
    lines = ['# Full-trajectory stopping model comparison', '',
        '**Offline only. No flight controller or calibration file changed. No model is flight-approved.**', '',
        '## Method', '',
        '- `a_dot=(k*u(t-L)-a)/tau`; `v_dot=a-c*v+b`; `p_dot=v`.',
        '- Base: fit L/tau/k and common signed-motion bias (same structure as old fit).',
        '- World bias: replace common signed bias with world-Y bias, projected with opposite sign in +/-Y trials.',
        '- Drag + world bias: additionally fit nonnegative linear drag c. No per-trial parameters.',
        '- New fitting and replay call the SAME exact interval integrator, with delayed command changes as boundaries.',
        '- Loss: equal trial weighting of full velocity-trajectory MSE plus terminal-mean MSE (weight 1, last 0.1s).',
        '- Position and zero crossing are validation outputs, not fitted targets. Velocity/position consistency is reported separately.',
        '- Four fixed multi-start seeds and fixed bounds, independent of held-out data or the saved all-data parameters.',
        '- Each fold fits four complete trials and predicts the two opposite-direction trials at the held-out duration.',
        '- Pre-brake p/v initialized once; response warmed from past command history. Future state never corrects prediction.',
        '- `legacy_window` repeats the previous window fitter on the same training folds; all-data results are labeled in-sample.',
        '- Observed zero crossing is not stable stopping. Missing crossings are censored before position recovery.',
        '- These six trials have already been inspected: model selection here is exploratory and needs a new independent flight.', '',
        '## Held-duration-out results', '',
        '| Model | Velocity RMSE (m/s) | Terminal MAE / max (m/s) | Endpoint MAE / max (cm) | Missed / false reverse |',
        '|---|---:|---:|---:|---:|']
    for model, s in report['summary'].items():
        lines.append(f"| {model} | {s['velocity_rmse_m_s']:.4f} | {s['terminal_mean_mae_m_s']:.4f} / {s['terminal_mean_max_abs_error_m_s']:.4f} | "
                     f"{100*s['end_position_mae_m']:.2f} / {100*s['end_position_max_abs_error_m']:.2f} | {s['reversal_missed']} / {s['reversal_false_alarm']} |")
    lines += ['', 'Reverse classification threshold: -0.03m/s, diagnostic only. Full crossing errors/counts are in report.json.',
              '', '## Per-trial held-out terminal speeds (m/s)', '',
              '| Trial | Actual | Legacy window | Trajectory base | World bias | Drag + world bias |',
              '|---|---:|---:|---:|---:|---:|']
    rows = [r for r in report['metrics'] if r['evaluation']=='held_duration_out']
    for segment in sorted({r['segment'] for r in rows}):
        selected = {r['model']: r for r in rows if r['segment']==segment}
        values = [f"{selected[m]['predicted_terminal_mean_m_s']:+.3f}" for m in ('legacy_window', *CANDIDATES)]
        lines.append(f"| {segment} | {selected['legacy_window']['actual_terminal_mean_m_s']:+.3f} | " + ' | '.join(values) + ' |')
    lines += ['', '## All-data parameters (in-sample, not deployed)', '',
              '| Model | Delay (ms) | Tau (ms) | Gain | Parallel bias | World-Y bias | Drag (1/s) |',
              '|---|---:|---:|---:|---:|---:|---:|']
    for model, fit in report['all_data_fits'].items():
        lines.append(f"| {model} | {1000*fit['command_delay_s']:.1f} | {1000*fit['command_time_constant_s']:.1f} | "
                     f"{fit['horizontal_acceleration_scale']:.4f} | {fit['acceleration_bias_m_s2']:.4f} | "
                     f"{fit['world_y_bias_m_s2']:.4f} | {fit['linear_drag_per_s']:.4f} |")
    lines += ['', 'Individual fold parameters, active bounds and Jacobian condition numbers are in report.json.',
              'A coefficient at its bound, inconsistent folds, or poor conditioning is a warning against interpreting it physically.',
              'A fitted world bias may absorb tilt trim, sensing error or unmodeled dynamics; it does not prove a physical cause.',
              '', '## Source and timing', '', '```json', json.dumps(report['metadata'], indent=2), '```', '']
    return '\n'.join(lines)


def plot_comparison(output, traces, trials):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors = {'legacy_window': '#888888', 'trajectory_base': '#2375b9',
              'trajectory_world_bias': '#309a69', 'trajectory_drag_world_bias': '#d46f19'}
    fig, axes = plt.subplots(len(trials), 2, figsize=(13, 2.65*len(trials)), squeeze=False)
    for i, trial in enumerate(trials):
        for model, color in colors.items():
            rows = [r for r in traces if r['segment']==trial.segment and r['model']==model]
            for j, quantity in enumerate(('velocity_m_s', 'displacement_m')):
                ax = axes[i, j]
                if model == 'legacy_window':
                    ax.plot([r['elapsed_s'] for r in rows], [r['actual_'+quantity] for r in rows],
                            color='black', lw=1.8, label='Measured')
                    anchor = trial.times[np.searchsorted(trial.times, trial.phase_times['brake'], side='right')-1]
                    ax.axvline(trial.phase_times['level_after_brake']-anchor, color='#aaa', ls=':', label='Level command')
                    ax.axhline(0, color='#bbb', lw=.7)
                    ax.set_title(f'Trial {trial.segment} | {trial.duration:.2f}s | Y sign {trial.direction[1]:+.0f}')
                    ax.set_ylabel('Signed speed (m/s)' if j==0 else 'Signed displacement (m)')
                    ax.set_xlabel('Time since pre-brake anchor (s)')
                    ax.grid(alpha=.2)
                ax.plot([r['elapsed_s'] for r in rows], [r['predicted_'+quantity] for r in rows],
                        color=color, ls='--', label=model)
    axes[0, 0].legend(fontsize=7, loc='upper right')
    fig.suptitle('Full-trajectory model comparison | held-duration-out predictions\nNo future state corrections; position recovery excluded', fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, .97))
    fig.savefig(output/'held_out_comparison.png', dpi=150)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--no-plots', action='store_true')
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('output already exists; choose a new directory')
    with args.log.open() as f:
        records = json.load(f)
    report, traces, trials = compare(records)
    with args.log.open('rb') as f:
        digest = hashlib.sha256()
        for block in iter(lambda: f.read(1024*1024), b''):
            digest.update(block)
    report['metadata'].update(source_log=str(args.log.resolve()), source_sha256=digest.hexdigest())
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    (args.output/'README.md').write_text(report_markdown(report))
    write_csv(args.output/'metrics.csv', report['metrics'])
    write_csv(args.output/'trajectories.csv', traces)
    if not args.no_plots:
        plot_comparison(args.output, traces, trials)
    print(f'Offline comparison saved to {args.output}')


if __name__ == '__main__':
    main()
