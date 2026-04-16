import csv
import json
import math
import sys
import numpy as np
import matplotlib.pyplot as plt

V_MARGINS = [round(x * 0.01, 2) for x in range(11)]  # 0.00 to 0.10, step 0.01


def load_records(log_path):
    records = []
    with open(log_path) as f:
        for line in f:
            line = line.strip().rstrip(',')
            if not line or line in ('[', ']'):
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return records


def analyze(log_path, drift_window_start=2.0, drift_window_end=12.0, decel_search_sec=5.0):
    records = load_records(log_path)

    # Find "Waiting For User Interaction" time
    wait_time = next(
        (r['data']['time'] for r in records
         if r.get('type') == 'events' and r.get('name') == 'Waiting For User Interaction'),
        None
    )
    if wait_time is None:
        raise ValueError("No 'Waiting For User Interaction' event found in log.")
    print(f"Waiting For User Interaction time: {wait_time}")

    # Collect all frames with speed magnitude
    frames = []
    for r in records:
        if r.get('type') == 'frames':
            vel = r['data']['vel']
            speed = math.sqrt(vel[0]**2 + vel[1]**2)
            frames.append({'time': r['data']['time'], 'speed': speed})

    print(f"Total frames: {len(frames)}")

    # --- Drift velocity: drift_window_start to drift_window_end after wait_time ---
    drift_window = [f for f in frames
                    if wait_time + drift_window_start <= f['time'] <= wait_time + drift_window_end]
    drift_speeds = [f['speed'] for f in drift_window]
    drift_min = min(drift_speeds)
    drift_max = max(drift_speeds)
    drift_avg = float(np.mean(drift_speeds))

    print(f"\nDrift velocity ({drift_window_start}s-{drift_window_end}s window after wait):")
    print(f"  min: {drift_min:.6f} m/s")
    print(f"  max: {drift_max:.6f} m/s")
    print(f"  avg: {drift_avg:.6f} m/s")

    # --- User Disengage events ---
    disengage_events = [r['data'] for r in records
                        if r.get('type') == 'events' and r.get('name') == 'User Disengage']
    print(f"\nUser Disengage events: {len(disengage_events)}")

    # --- send_notify_setpoint_stop absolute time ---
    # Command records use relative time; derive absolute time from the nearest preceding frame/state.
    stop_time = None
    last_abs_time = None
    for r in records:
        if r.get('type') in ('frames', 'state'):
            last_abs_time = r['data']['time']
        if r.get('type') == 'commands' and r.get('name') == 'Commander.send_notify_setpoint_stop':
            stop_time = last_abs_time
            break
    if stop_time is None:
        stop_time = disengage_events[-1]['time'] + 3.0
        print("Warning: send_notify_setpoint_stop not found; falling back to last disengage + 3s")
    else:
        print(f"send_notify_setpoint_stop at: {stop_time:.3f}")

    # --- First push time per session ---
    # Each session = from previous boundary (wait_time or last disengage) to next disengage
    push_events = [r['data'] for r in records
                   if r.get('type') == 'events' and r.get('name') == 'User Pushing']

    boundaries = [wait_time] + [ev['time'] for ev in disengage_events]
    first_push_times = []
    for i, (t_from, t_to) in enumerate(zip(boundaries[:-1], boundaries[1:])):
        session_pushes = [ev for ev in push_events if t_from < ev['time'] <= t_to]
        if session_pushes:
            first_push_times.append(session_pushes[0]['time'])

    print(f"Push sessions found: {len(first_push_times)}")

    # --- Sweep v_margin ---
    print(f"\n{'v_margin':>9} | {'threshold':>10} | "
          f"{'grace_min':>9} {'grace_max':>9} {'grace_avg':>9} | "
          f"{'improv_min':>10} {'improv_max':>10} {'improv_avg':>10}")
    print("-" * 95)

    results = []
    for v_margin in V_MARGINS:
        threshold = drift_max + v_margin

        # Grace time: per session, from first drop below threshold after first push
        #             to last rise above threshold; search ends at next session's first push,
        #             or disengage + decel_search_sec for the last session
        grace_times = []
        for i, (t_push, ev) in enumerate(zip(first_push_times, disengage_events)):
            t_disengage = ev['time']
            if i + 1 < len(first_push_times):
                t_end = first_push_times[i + 1] - 1
            else:
                t_end = t_disengage + decel_search_sec
            post_push = [f for f in frames if t_push <= f['time'] <= t_end]

            first_below = next((f['time'] for f in post_push if f['speed'] < threshold), None)
            last_above = None
            for f in post_push:
                if f['speed'] > threshold:
                    last_above = f['time']
            # print(last_above - first_below)
            if first_below is not None and last_above is not None and last_above > first_below:
                grace_times.append(last_above - first_below)

        # Improvement: time from first threshold exceedance to first push, per session
        improvements = []
        for t_push in first_push_times:
            # Search frames up to 5s before the first push
            pre_window = [f for f in frames if t_push - 1.0 <= f['time'] <= t_push]
            first_exceed = next((f['time'] for f in pre_window if f['speed'] > threshold), None)
            if first_exceed is not None:
                improvements.append(t_push - first_exceed)

        g_min = min(grace_times) if grace_times else float('nan')
        g_max = max(grace_times) if grace_times else float('nan')
        g_avg = float(np.mean(grace_times)) if grace_times else float('nan')
        i_min = min(improvements) if improvements else float('nan')
        i_max = max(improvements) if improvements else float('nan')
        i_avg = float(np.mean(improvements)) if improvements else float('nan')

        results.append((v_margin, threshold, g_min, g_max, g_avg, i_min, i_max, i_avg,
                        grace_times, improvements))

        print(f"{v_margin:>9.2f} | {threshold:>10.6f} | "
              f"{g_min:>9.3f} {g_max:>9.3f} {g_avg:>9.3f} | "
              f"{i_min:>10.3f} {i_max:>10.3f} {i_avg:>10.3f}")

    # --- Plot ---
    t_start = wait_time
    rel_times = [f['time'] - t_start for f in frames]
    speeds = [f['speed'] for f in frames]

    fig, ax = plt.subplots(figsize=(14, 5))
    ax.plot(rel_times, speeds, linewidth=0.8, color='steelblue', label='Frame speed')

    # cmap = plt.cm.RdYlGn_r
    # for idx, (v_margin, threshold, *_) in enumerate(results):
    #     color = cmap(idx / (len(results) - 1))
    #     ax.axhline(threshold, color=color, linestyle='--', linewidth=0.9,
    #                label=f'margin={v_margin:.2f} ({threshold:.4f} m/s)')
    _, threshold, *_ = results[0]
    ax.axhline(threshold, linestyle='--', color='gray', linewidth=0.9,
               label=f'Max Drift Vel={threshold:.4f} m/s')


    ax.axhline(0.13, linestyle='--', color='red', linewidth=0.9,
               label=f'Default Δv={0.13} m/s')

    ax.axvline(wait_time - t_start, color='purple', linestyle='-', alpha=0.5,
               linewidth=1.5, label='Waiting for interaction')
    ax.axvspan(wait_time + drift_window_start - t_start, wait_time + drift_window_end - t_start,
               alpha=0.1, color='purple',
               label=f'Drift window ({drift_window_start}s-{drift_window_end}s)')

    for i, ev in enumerate(disengage_events):
        label = 'User Disengage' if i == 0 else None
        ax.axvline(ev['time'] - t_start, color='green', linestyle='--', alpha=0.7,
                   linewidth=1, label=label)

    for i, t_push in enumerate(first_push_times):
        label = 'First push' if i == 0 else None
        ax.axvline(t_push - t_start, color='black', linestyle=':', alpha=0.6,
                   linewidth=1, label=label)

    ax.set_xlim(wait_time - t_start, stop_time - t_start)
    ax.set_xlabel('Time (s, relative)')
    ax.set_ylabel('Speed (m/s)')
    ax.set_title('Frame velocity magnitude vs time')
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    out_path = log_path.replace('.json', '_vel.png')
    plt.savefig(out_path, dpi=150)
    print(f"\nPlot saved to: {out_path}")
    plt.show()

    # --- CSV ---
    csv_path = log_path.replace('.json', '_results.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['v_margin', 'threshold',
                         'grace_min', 'grace_max', 'grace_avg',
                         'improv_min', 'improv_max', 'improv_avg'])
        for v_margin, threshold, g_min, g_max, g_avg, i_min, i_max, i_avg, _, _ in results:
            writer.writerow([v_margin, threshold, g_min, g_max, g_avg, i_min, i_max, i_avg])
    print(f"CSV saved to: {csv_path}")


if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv) > 1 else \
        "../../logs/lb11_translation_2026-04-16_12-05-41.json"
    analyze(path)
