import csv
import json
import math
import sys
import numpy as np
import matplotlib.pyplot as plt

V_MARGINS = [round(x * 0.01, 2) for x in range(11)]  # 0.00 to 0.10, step 0.01
FONT = 20


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


def analyze(log_path, drift_window_start=2.0, drift_window_end=8.0, decel_search_sec=2.5):
    records = load_records(log_path)

    # Read baseline delta_v from log config
    config_rec = next(
        (r for r in records
         if r.get('type') == 'configs' and r.get('name') == 'Translation Config'),
        None
    )
    if config_rec is None:
        raise ValueError("No 'Translation Config' record found in log.")
    BASELINE_THRESHOLD = config_rec['data']['delta_v']
    STABILIZE_TIME = config_rec['data']['Stabilize Time']
    print(f"Baseline delta_v from config: {BASELINE_THRESHOLD}")
    print(f"Stabilize time from config:   {STABILIZE_TIME} s")

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
            tvec = r['data']['tvec']
            frames.append({'time': r['data']['time'], 'speed': speed,
                           'vy': vel[1], 'vz': vel[2],
                           'px': tvec[0], 'py': tvec[1], 'pz': tvec[2]})

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

    # Drift distance: net displacement and total path length over the drift window
    if len(drift_window) >= 2:
        x0, y0 = drift_window[0]['px'], drift_window[0]['py']
        disp_x = [(f['px'] - x0) * 1000 for f in drift_window]
        disp_y = [(f['py'] - y0) * 1000 for f in drift_window]
        disp_r = [math.sqrt(dx**2 + dy**2) for dx, dy in zip(disp_x, disp_y)]
        idx_r = int(np.argmax(disp_r))
        idx_x = max(range(len(disp_x)), key=lambda i: abs(disp_x[i]))
        idx_y = max(range(len(disp_y)), key=lambda i: abs(disp_y[i]))
        print(f"  max drift (2D):    {disp_r[idx_r]:.3f} mm  "
              f"(x={disp_x[idx_r]:.2f} mm, y={disp_y[idx_r]:.3f} mm)")
        print(f"  max drift along x: {disp_x[idx_x]:.3f} mm  "
              f"(y={disp_y[idx_x]:.2f} mm at that time)")
        print(f"  max drift along y: {disp_y[idx_y]:.3f} mm  "
              f"(x={disp_x[idx_y]:.2f} mm at that time)")

    # Z-axis drift (independent)
    drift_vz = [f['vz'] for f in drift_window]
    z0 = drift_window[0]['pz']
    disp_z = [(f['pz'] - z0) * 1000 for f in drift_window]
    print(f"\nDrift along Z ({drift_window_start}s-{drift_window_end}s window):")
    print(f"  vz min: {min(drift_vz)*1000:.3f} mm/s")
    print(f"  vz max: {max(drift_vz)*1000:.3f} mm/s")
    print(f"  vz avg: {float(np.mean(drift_vz))*1000:.3f} mm/s")
    idx_z = max(range(len(disp_z)), key=lambda i: abs(disp_z[i]))
    print(f"  max drift along z: {disp_z[idx_z]:.3f} mm  "
          f"(x={disp_x[idx_z]:.3f} mm, y={disp_y[idx_z]:.3f} mm at that time)")

    # 3D drift
    drift_speed3d = [math.sqrt(f['speed']**2 + f['vz']**2) for f in drift_window]
    disp_3d = [math.sqrt(disp_x[i]**2 + disp_y[i]**2 + disp_z[i]**2)
               for i in range(len(drift_window))]
    idx_3d = int(np.argmax(disp_3d))
    print(f"\nDrift 3D ({drift_window_start}s-{drift_window_end}s window):")
    print(f"  speed3d min: {min(drift_speed3d)*1000:.3f} mm/s")
    print(f"  speed3d max: {max(drift_speed3d)*1000:.3f} mm/s")
    print(f"  speed3d avg: {float(np.mean(drift_speed3d))*1000:.3f} mm/s")
    print(f"  max drift (3D): {disp_3d[idx_3d]:.3f} mm  "
          f"(x={disp_x[idx_3d]:.3f} mm, y={disp_y[idx_3d]:.3f} mm, z={disp_z[idx_3d]:.3f} mm)")

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

    baseline_metrics = compute_metrics(BASELINE_THRESHOLD)
    print(f"\nBaseline (ΔV=0.13): grace=[{baseline_metrics[0]:.3f}, {baseline_metrics[1]:.3f}, "
          f"avg={baseline_metrics[2]:.3f}]  TTD=[{baseline_metrics[3]:.3f}, "
          f"{baseline_metrics[4]:.3f}, avg={baseline_metrics[5]:.3f}]")

    # --- Plot helpers ---
    t_start = wait_time
    rel_times = [f['time'] - t_start for f in frames]
    speeds_mm = [f['speed'] * 1000 for f in frames]   # m/s -> mm/s
    vys_mm    = [f['vy']    * 1000 for f in frames]
    vzs_mm    = [f['vz']    * 1000 for f in frames]

    _, threshold_ms, *_ = results[0]
    threshold_mm      = threshold_ms * 1000
    baseline_mm       = BASELINE_THRESHOLD * 1000
    xlim = (wait_time - t_start, stop_time - t_start)

    def _add_event_lines(ax, *, speed_values=None):
        """Draw shared event markers; pass speed_values to place onset scatter."""
        ax.axvline(wait_time - t_start, color='tab:purple', linestyle='-', alpha=0.5,
                   linewidth=1.5, label='Waiting for interaction')
        ax.axvspan(wait_time + drift_window_start - t_start,
                   wait_time + drift_window_end - t_start,
                   alpha=0.1, color='tab:purple',
                   label=f'Drift window ({drift_window_start}s-{drift_window_end}s)')
        for i, ev in enumerate(disengage_events):
            ax.axvline(ev['time'] - t_start, color='green', linestyle='--', alpha=0.7,
                       linewidth=1, label='User Disengage' if i == 0 else None)
            ax.axvline(ev['time'] + STABILIZE_TIME - t_start, color='lime', linestyle=':',
                       alpha=0.9, linewidth=1.2, label='Grace time end' if i == 0 else None)
        for i, t_push in enumerate(first_push_times):
            ax.axvline(t_push - t_start, color='black', linestyle=':', alpha=0.6,
                       linewidth=1, label='First push' if i == 0 else None)
        for i, (t_push, t_onset) in enumerate(zip(first_push_times, push_onset_times)):
            if t_onset is not None:
                rel_onset = t_onset - t_start
                ax.axvline(rel_onset, color='orange', linestyle='-', alpha=0.3, linewidth=0.5,
                           label='Push onset' if i == 0 else None)
                if speed_values is not None:
                    onset_val = next(v for f, v in zip(frames, speed_values)
                                     if f['time'] == t_onset)
                    ax.scatter([rel_onset], [onset_val], color='orange', s=10, zorder=6,
                               marker='^')

    def _save(fig, suffix):
        p = log_path.replace('.json', suffix)
        fig.savefig(p, dpi=300)
        print(f"Plot saved to: {p}")

    # ── Figure 1: speed magnitude, linear (mm/s) ─────────────────────────────
    fig1, ax1 = plt.subplots(figsize=(14, 5))
    ax1.plot(rel_times, speeds_mm, linewidth=0.8, color='steelblue', label='Frame speed')
    ax1.axhline(threshold_mm, linestyle='--', color='gray', linewidth=0.9,
                label=f'Max Drift Vel={threshold_mm:.2f} mm/s')
    ax1.axhline(baseline_mm, linestyle='--', color='red', linewidth=0.9,
                label=f'Default Δv={baseline_mm:.1f} mm/s')
    _add_event_lines(ax1, speed_values=speeds_mm)
    ax1.set_xlim(*xlim)
    ax1.set_xlabel('Time (s, relative)')
    ax1.set_ylabel('Speed (mm/s)')
    ax1.set_title('Frame velocity magnitude vs time')
    ax1.legend(fontsize=FONT - 9, ncol=2)
    ax1.grid(True, alpha=0.3)
    fig1.tight_layout()
    _save(fig1, '_speed_linear.png')

    # ── Figure 2: vy, linear (mm/s) ──────────────────────────────────────────
    fig2, ax2 = plt.subplots(figsize=(14, 5))
    ax2.plot(rel_times, vys_mm, linewidth=0.8, color='tomato', label='vy')
    ax2.axhline(0, color='black', linewidth=0.6, linestyle='-')
    _add_event_lines(ax2, speed_values=vys_mm)
    ax2.set_xlim(*xlim)
    ax2.set_xlabel('Time (s, relative)')
    ax2.set_ylabel('vy (mm/s)')
    ax2.set_title('Y-axis velocity vs time (+ / − = opposite directions)')
    ax2.legend(fontsize=FONT - 9, ncol=2)
    ax2.grid(True, alpha=0.3)
    fig2.tight_layout()
    _save(fig2, '_vy_linear.png')

    # ── Figure 2b: vz, linear (mm/s) ─────────────────────────────────────────
    fig2b, ax2b = plt.subplots(figsize=(14, 5))
    ax2b.plot(rel_times, vzs_mm, linewidth=0.8, color='steelblue', label='vz')
    ax2b.axhline(0, color='black', linewidth=0.6, linestyle='-')
    _add_event_lines(ax2b, speed_values=vzs_mm)
    ax2b.set_ylim(-20, 20)
    ax2b.set_xlim(*xlim)
    ax2b.set_xlabel('Time (s, relative)')
    ax2b.set_ylabel('vz (mm/s)')
    ax2b.set_title('Z-axis velocity vs time (+ / − = opposite directions)')
    ax2b.legend(fontsize=FONT - 9, ncol=2)
    ax2b.grid(True, alpha=0.3)
    fig2b.tight_layout()
    _save(fig2b, '_vz_linear.png')

    # ── Figure 3: speed magnitude, log scale (mm/s) ──────────────────────────
    fig3, ax3 = plt.subplots(figsize=(14, 5))
    ax3.plot(rel_times, speeds_mm, linewidth=0.8, color='steelblue', label='Frame speed')
    ax3.axhline(threshold_mm, linestyle='--', color='gray', linewidth=0.9,
                label=f'Max Drift Vel={threshold_mm:.2f} mm/s')
    ax3.axhline(baseline_mm, linestyle='--', color='red', linewidth=0.9,
                label=f'Default Δv={baseline_mm:.1f} mm/s')
    _add_event_lines(ax3, speed_values=speeds_mm)
    ax3.set_yscale('symlog', linthresh=1.0)
    ax3.set_yticks([0, 1, 10, 100, 1000])
    ax3.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f'{int(v)}'))
    ax3.set_xlim(*xlim)
    ax3.set_xlabel('Time (s, relative)')
    ax3.set_ylabel('Speed (mm/s, log)')
    ax3.set_title('Frame velocity magnitude vs time (log scale)')
    ax3.legend(fontsize=FONT - 9, ncol=2)
    ax3.grid(True, alpha=0.3, which='both')
    fig3.tight_layout()
    _save(fig3, '_speed_log.png')

    # ── Figure 4: vy, symlog scale (mm/s) ────────────────────────────────────
    fig4, ax4 = plt.subplots(figsize=(14, 5))
    ax4.plot(rel_times, vys_mm, linewidth=0.8, color='tomato', label='vy')
    ax4.axhline(0, color='black', linewidth=0.6, linestyle='-')
    _add_event_lines(ax4, speed_values=vys_mm)
    ax4.set_yscale('symlog', linthresh=1.0)
    ax4.set_yticks([-1000, -100, -10, -1, 0, 1, 10, 100, 1000])
    ax4.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f'{int(v)}'))
    ax4.set_xlim(*xlim)
    ax4.set_xlabel('Time (s, relative)')
    ax4.set_ylabel('vy (mm/s, symlog)')
    ax4.set_title('Y-axis velocity vs time (symlog scale, + / − = opposite directions)')
    ax4.legend(fontsize=FONT - 9, ncol=2)
    ax4.grid(True, alpha=0.3, which='both')
    fig4.tight_layout()
    _save(fig4, '_vy_log.png')

    # ── Figure 5 & 6: paper-quality plots ────────────────────────────────────
    paper_x0 = first_push_times[0] - t_start - 3.0   # absolute offset from wait_time
    paper_x1 = disengage_events[-1]['time'] - t_start + 3.0 + 1.94

    # Shift so paper_x0 == 0 on the paper plots
    paper_rel_times = [t - paper_x0 for t in rel_times]
    paper_xlim = (0, paper_x1 - paper_x0)

    def _paper_rel(t):
        return t - paper_x0

    # Only 2 drift spans: before first push, and after the last grace-time end
    last_grace_end = disengage_events[-1]['time'] + STABILIZE_TIME - t_start
    drift_spans = [
        (0, _paper_rel(first_push_times[0] - t_start)),
        (_paper_rel(last_grace_end), paper_xlim[1]),
    ]

    def _paper_style(ax):
        ax.tick_params(axis='both', labelsize=FONT)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.grid(True, linestyle='--', alpha=0.7)

    DRIFT_COLOR = 'purple'

    def _add_drift_spans(ax, y_text):
        for x0, x1 in drift_spans:
            ax.axvspan(x0, x1, alpha=0.1, color=DRIFT_COLOR, zorder=0)
            ax.text((x0 + x1) / 2, y_text, 'Drift', ha='center', va='center',
                    fontsize=FONT - 2, color='steelblue', style='italic',
                    transform=ax.get_xaxis_transform())

    # Figure 5: vy paper plot (symlog)
    fig5, ax5 = plt.subplots(figsize=(10, 6))
    ax5.plot(paper_rel_times, vys_mm, linewidth=1.2, color='black')
    # ax5.plot(paper_rel_times, vys_mm, linewidth=1.2, color='tab:brown')
    # ax5.axhline(0, color='black', linewidth=0.8, linestyle='-')
    ax5.set_yscale('symlog', linthresh=1.0)

    ax5.set_ylim(-1500, 1500)
    ax5.set_yticks([-1000, -100, -10, -1, 0, 1, 10, 100, 1000])
    ax5.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f'{int(v)}'))
    _add_drift_spans(ax5, y_text=0.05)
    ax5.set_xlim(*paper_xlim)
    ax5.set_xlabel('Time (s)', fontsize=FONT)
    ax5.set_title('mm/s', loc='left', fontsize=FONT, x=-0.06)
    _paper_style(ax5)
    fig5.tight_layout()
    fig5.savefig('vy_time.png', dpi=300)

    # Figure 6: speed paper plot (symlog)
    fig6, ax6 = plt.subplots(figsize=(10, 6))
    ax6.plot(paper_rel_times, speeds_mm, linewidth=1.2, color='tab:blue')
    ax6.text(x=1, y=98, ha='center', va='bottom', s='92.0', fontsize=FONT-2, color='tab:purple')
    ax6.axhline(baseline_mm, linestyle='--', color='tab:purple', linewidth=1.5,
                label=r'$S_D = S_Q = S_H = 92.0~mm/s$')
    ax6.set_yscale('symlog', linthresh=1.0)
    ax6.set_yticks([0, 1, 10, 100, 1000])
    ax6.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f'{int(v)}'))
    _add_drift_spans(ax6, y_text=0.05)
    ax6.set_xlim(*paper_xlim)
    ax6.set_ylim(0, 1500)
    ax6.set_xlabel('Time (s)', fontsize=FONT)
    ax6.set_title('mm/s', loc='left', fontsize=FONT, x=-0.06)
    ax6.legend(fontsize=FONT)
    _paper_style(ax6)
    fig6.tight_layout()
    fig6.tight_layout()

    fig6.savefig('speed_time.png', dpi=300)
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
    # path = sys.argv[1] if len(sys.argv) > 1 else \
    #     "../../logs/lb11_translation_2026-04-16_12-05-41.json" # delta v = 0.13, grace time = 2


    path = sys.argv[1] if len(sys.argv) > 1 else \
        "../../logs/lb11_translation_2026-04-16_16-02-40.json" # delta v = 0.092, grace time = 1.94
    analyze(path)
