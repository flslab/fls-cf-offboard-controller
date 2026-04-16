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
            frames.append({'time': r['data']['time'], 'speed': speed, 'vy': vel[1]})

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

    # --- Detect push onset via velocity change (within 500ms before each push) ---
    # Strategy: scan backwards from t_push; find the speed local minimum —
    # the last decelerating frame before the push acceleration begins.
    all_speeds = [f['speed'] for f in frames]

    push_onset_times = []
    for t_push in first_push_times:
        pre_indices = [i for i, f in enumerate(frames)
                       if t_push - 0.5 <= f['time'] <= t_push]
        onset = None
        if pre_indices:
            pre_speeds = [all_speeds[i] for i in pre_indices]
            min_idx = pre_indices[int(np.argmin(pre_speeds))]
            onset = frames[min_idx]['time']
        push_onset_times.append(onset)
        print(f"  Push at t={t_push - wait_time:.3f}s  ->  onset at "
              f"t={onset - wait_time:.3f}s  ({(t_push - onset)*1000:.1f} ms early)"
              if onset is not None else
              f"  Push at t={t_push - wait_time:.3f}s  ->  onset not detected")

    # --- Sweep v_margin ---
    print(f"\n{'v_margin (m/s)':>14} | {'ΔV (m/s)':>10} | "
          f"{'Min Grace Time (s)':>18} {'Max Grace Time (s)':>18} {'Avg Grace Time (s)':>18} | "
          f"{'Min TTD (s)':>9} {'Max TTD (s)':>9} {'Avg TTD (s)':>9}")
    print("-" * 115)

    def compute_metrics(threshold):
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
            if first_below is not None and last_above is not None and last_above > first_below:
                grace_times.append(last_above - first_below)

        detect_times = []
        for t_onset, t_push in zip(push_onset_times, first_push_times):
            if t_onset is None:
                continue
            post_onset = [f for f in frames if t_onset <= f['time'] <= t_push + 2.0]
            first_exceed = next((f['time'] for f in post_onset if f['speed'] > threshold), None)
            if first_exceed is not None:
                detect_times.append(first_exceed - t_onset)

        g_min = min(grace_times) if grace_times else float('nan')
        g_max = max(grace_times) if grace_times else float('nan')
        g_avg = float(np.mean(grace_times)) if grace_times else float('nan')
        d_min = min(detect_times) if detect_times else float('nan')
        d_max = max(detect_times) if detect_times else float('nan')
        d_avg = float(np.mean(detect_times)) if detect_times else float('nan')
        return g_min, g_max, g_avg, d_min, d_max, d_avg, grace_times, detect_times

    results = []
    for v_margin in V_MARGINS:
        threshold = drift_max + v_margin
        g_min, g_max, g_avg, d_min, d_max, d_avg, grace_times, detect_times = \
            compute_metrics(threshold)
        results.append((v_margin, threshold, g_min, g_max, g_avg, d_min, d_max, d_avg,
                        grace_times, detect_times))
        print(f"{v_margin:>14.2f} | {threshold:>10.6f} | "
              f"{g_min:>18.3f} {g_max:>18.3f} {g_avg:>18.3f} | "
              f"{d_min:>9.3f} {d_max:>9.3f} {d_avg:>9.3f}")

    BASELINE_THRESHOLD = 0.13
    baseline_metrics = compute_metrics(BASELINE_THRESHOLD)
    print(f"\nBaseline (ΔV=0.13): grace=[{baseline_metrics[0]:.3f}, {baseline_metrics[1]:.3f}, "
          f"avg={baseline_metrics[2]:.3f}]  TTD=[{baseline_metrics[3]:.3f}, "
          f"{baseline_metrics[4]:.3f}, avg={baseline_metrics[5]:.3f}]")

    # --- Plot ---
    t_start = wait_time
    rel_times = [f['time'] - t_start for f in frames]
    speeds = [f['speed'] for f in frames]
    vys = [f['vy'] for f in frames]

    fig, (ax, ax_vy) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
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

    for i, (t_push, t_onset) in enumerate(zip(first_push_times, push_onset_times)):
        if t_onset is not None:
            label = 'Push onset (vel change)' if i == 0 else None
            rel_onset = t_onset - t_start
            onset_speed = next(f['speed'] for f in frames if f['time'] == t_onset)
            ax.axvline(rel_onset, color='orange', linestyle='-', alpha=0.3,
                       linewidth=0.5, label=label)
            ax.scatter([rel_onset], [onset_speed], color='orange', s=10, zorder=6,
                       marker='^', alpha=1)

    ax.set_xlim(wait_time - t_start, stop_time - t_start)
    ax.set_ylabel('Speed (m/s)')
    ax.set_title('Frame velocity magnitude vs time')
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    # --- vy subplot ---
    ax_vy.plot(rel_times, vys, linewidth=0.8, color='tomato', label='vy')
    ax_vy.axhline(0, color='black', linewidth=0.6, linestyle='-')

    for i, ev in enumerate(disengage_events):
        label = 'User Disengage' if i == 0 else None
        ax_vy.axvline(ev['time'] - t_start, color='green', linestyle='--', alpha=0.7,
                      linewidth=1, label=label)
    for i, t_push in enumerate(first_push_times):
        label = 'First push' if i == 0 else None
        ax_vy.axvline(t_push - t_start, color='black', linestyle=':', alpha=0.6,
                      linewidth=1, label=label)
    for i, (t_push, t_onset) in enumerate(zip(first_push_times, push_onset_times)):
        if t_onset is not None:
            label = 'Push onset' if i == 0 else None
            rel_onset = t_onset - t_start
            onset_vy = next(f['vy'] for f in frames if f['time'] == t_onset)
            ax_vy.axvline(rel_onset, color='orange', linestyle='-', alpha=0.3,
                          linewidth=0.5, label=label)
            ax_vy.scatter([rel_onset], [onset_vy], color='orange', s=10, zorder=6, marker='^')

    ax_vy.set_xlim(wait_time - t_start, stop_time - t_start)
    ax_vy.set_xlabel('Time (s, relative)')
    ax_vy.set_ylabel('vy (m/s)')
    ax_vy.set_title('Y-axis velocity vs time (+ / − = opposite directions)')
    ax_vy.legend(fontsize=7, ncol=2)
    ax_vy.grid(True, alpha=0.3)

    plt.tight_layout()

    out_path = log_path.replace('.json', '_vel.png')
    plt.savefig(out_path, dpi=300)
    print(f"\nPlot saved to: {out_path}")
    plt.show()

    # --- CSV ---
    csv_path = log_path.replace('.json', '_results.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['v_margin (m/s)', 'ΔV (m/s)',
                         'Min Grace Time (s)', 'Max Grace Time (s)', 'Avg Grace Time (s)',
                         'Min TTD (s)', 'Max TTD (s)', 'Avg TTD (s)'])
        for v_margin, threshold, g_min, g_max, g_avg, d_min, d_max, d_avg, _, _ in results:
            writer.writerow([v_margin, threshold, g_min, g_max, g_avg, d_min, d_max, d_avg])
        bg_min, bg_max, bg_avg, bd_min, bd_max, bd_avg, _, _ = baseline_metrics
        writer.writerow(['baseline', BASELINE_THRESHOLD,
                         bg_min, bg_max, bg_avg, bd_min, bd_max, bd_avg])
    print(f"CSV saved to: {csv_path}")


if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv) > 1 else \
        "../../logs/lb11_translation_2026-04-16_12-05-41.json"
    analyze(path)
