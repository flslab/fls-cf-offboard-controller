"""Build an offline + recorded CrazySim comparison; never runs a vehicle."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from Interaction.virtual_collision_sim import demo_scenarios, save_results


def load_gazebo_run(run_directory):
    directory = Path(run_directory)
    report = json.loads((directory/'report.json').read_text())
    rows = [json.loads(line) for line in (directory/'trace.jsonl').read_text().splitlines()]
    rows = [r for r in rows if 'reference' in r]
    if report.get('status') != 'collision_complete' or not rows:
        raise ValueError('Gazebo run did not complete an encounter')
    config = report['plan_config']
    trigger = report['trigger_sim_time_s']
    samples = []
    for row in rows:
        ref = row['reference']
        # Include the measurement at outgoing-velocity assessment, before
        # capture takes control, then end the collision replay.
        if row['phase'] == 'capture':
            if not samples:
                raise ValueError('no collision samples precede capture')
            dt = row['elapsed_s'] - samples[-1]['time_s']
            ref = [dict(x=samples[-1][f'ref_x{i+1}_m']
                        + report['target_velocities_m_s'][i]*dt,
                        v=report['target_velocities_m_s'][i]) for i in (0, 1)]
        item = dict(time_s=row['elapsed_s'],
                    phase='separation' if row['phase'] == 'capture' else row['phase'],
                    separation_m=row['separation_m'])
        for i in (0, 1):
            drone = row['drones'][i]
            item.update({f'x{i+1}_m': drone['position_m'][0],
                         f'v{i+1}_m_s': drone['velocity_m_s'][0],
                         f'ref_x{i+1}_m': ref[i]['x'],
                         f'ref_v{i+1}_m_s': ref[i]['v']})
        samples.append(item)
        if row['phase'] == 'capture':
            break
    summary = dict(
        status='passed' if report['collision_passed'] else 'failed',
        checks=report['checks'],
        trigger_time_s=trigger-rows[0]['sim_time_s'],
        trigger_distance_m=report['trigger_distance_actual_m'],
        minimum_separation_m=report['raw_odom_minimum_separation_m'],
        transition_duration_s=report['plan_duration_s'],
        target_velocity1_m_s=report['target_velocities_m_s'][0],
        target_velocity2_m_s=report['target_velocities_m_s'][1],
        terminal_velocity1_m_s=report['outgoing_velocities_m_s'][0],
        terminal_velocity2_m_s=report['outgoing_velocities_m_s'][1],
        max_momentum_error_kg_m_s=report['collision_momentum_peak_error_kg_m_s'],
        measured_restitution=report['measured_restitution'],
        outgoing_window=report.get('outgoing_window'),
        actual_zero_crossing_after_trigger_s=report.get('actual_zero_crossing_after_trigger_s'),
        tracking_scope=report['tracking_scope'],
        simulation_only=True, firmware_sitl=True, flight_validated=False,
        source_directory=str(directory.resolve()))
    return dict(name='Gazebo · 两机实际跟踪', config=config, summary=summary, samples=samples)


def plot_result(scenario, destination):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np

    rows = scenario['samples']
    c, summary = scenario['config'], scenario['summary']
    t = np.array([s['time_s'] for s in rows]) - summary['trigger_time_s']
    figure, axes = plt.subplots(4, 1, figsize=(9.5, 10), sharex=True, constrained_layout=True)
    colors = ['#147d92', '#bc4a32']
    for i, color in enumerate(colors, 1):
        axes[0].plot(t, [s[f'x{i}_m'] for s in rows], color=color, label=f'Drone {i}, simulated')
        axes[0].plot(t, [s[f'ref_x{i}_m'] for s in rows], color=color, linestyle='--', alpha=.55)
        axes[1].plot(t, [s[f'v{i}_m_s'] for s in rows], color=color)
        axes[1].plot(t, [s[f'ref_v{i}_m_s'] for s in rows], color=color, linestyle='--', alpha=.55)
    axes[0].set_ylabel('Position x (m)')
    axes[0].legend(loc='upper center', ncol=2, frameon=False)
    axes[1].set_ylabel('Velocity x (m/s)')
    gap = [s.get('separation_m', s['x2_m']-s['x1_m']) for s in rows]
    axes[2].plot(t, gap, color='#386c38', label='Center separation')
    axes[2].axhline(c['minimum_separation_m'], color='#b7373c', linestyle=':', label='Separation limit')
    axes[2].set_ylabel('Separation (m)')
    axes[2].legend(frameon=False)
    p0 = c['mass1_kg']*c['velocity1_m_s']+c['mass2_kg']*c['velocity2_m_s']
    momentum = [c['mass1_kg']*s['v1_m_s']+c['mass2_kg']*s['v2_m_s']-p0 for s in rows]
    # Collision conservation is evaluated after the trigger; approach motors
    # may intentionally change momentum while both vehicles accelerate.
    axes[3].plot(t[t >= 0], np.array(momentum)[t >= 0], color='#6e50a1')
    axes[3].axhline(0, color='.5', linewidth=.6)
    axes[3].set_ylabel('Momentum error\n(kg m/s)')
    axes[3].set_xlabel('Time from virtual collision trigger (s)')
    for axis in axes:
        axis.axvline(0, color='.4', linestyle=':', linewidth=.8)
        axis.axvline(summary['transition_duration_s'], color='.6', linestyle=':', linewidth=.8)
        axis.spines[['top', 'right']].set_visible(False)
        axis.grid(alpha=.16)
    source = 'Paired Gazebo / frozen SITL' if summary['firmware_sitl'] else 'Delayed scalar model'
    figure.suptitle(source + ' | noncontact collision emulation\n'
                    'Solid: simulated motion. Dashed: existing septic S-curve reference.', fontsize=13)
    figure.savefig(destination, dpi=180)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gazebo-run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--visualization', type=Path, required=True)
    args = parser.parse_args()
    scenarios = demo_scenarios()
    if args.gazebo_run:
        scenarios.insert(0, load_gazebo_run(args.gazebo_run))
    compact = save_results(args.output, scenarios)
    template = Path(__file__).with_name('virtual_collision_viewer.html').read_text()
    args.visualization.parent.mkdir(parents=True, exist_ok=True)
    args.visualization.write_text(template.replace('__SIMULATION_DATA__',
        json.dumps(compact, ensure_ascii=False, separators=(',', ':')).replace('</', '<\\/')))
    plot_result(scenarios[0], args.output/'trajectories.png')
    print(json.dumps(scenarios[0]['summary'], indent=2))


if __name__ == '__main__':
    main()
