"""
analyze_lf_delay.py

Analyse the leader-follower tracking delay for LFmsgBAFC experiments.
Loads separate leader and follower log files. The start of each interaction
(T1) is read directly from the leader's cmd_positions timestamps so that
both drones are aligned on the same wall-clock time axis.

Per interaction:
  T1  — leader receives position_setpoint command (from cmd_positions)
  T10 — leader first arrives at target_y ± ARRIVE_TOL (from leader frames)
  Interaction window used for plots: [T1 - PAD_BEFORE, T1 + PAD_AFTER]
  Shaded region in plots: [T1, T10]

Generates one set of 3 plots for "before FC" and one set for "after FC":
  1. Y Position  — leader Y and follower Y vs time (zero-referenced to T1)
  2. Distance    — 3-D distance from follower to ideal position (leader + offset)
  3. Delay       — for each leader sample in [T1, T10], time until follower
                   reaches the same Y position (linear interpolation)
"""

import json
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


# ── Log paths ─────────────────────────────────────────────────────────────────
LOG_DIR = Path("../../logs/LFmsgBAFC")

BEFORE_LEADER   = LOG_DIR / "LFmsgbeforeFC_leader_2026-04-08_15-31-27.json"
BEFORE_FOLLOWER = LOG_DIR / "LFmsgbeforeFC_follower_2026-04-08_15-31-29.json"
AFTER_LEADER    = LOG_DIR / "LFmsgafterFC_leader_2026-04-08_15-53-03.json"
AFTER_FOLLOWER  = LOG_DIR / "LFmsgafterFC_follower_2026-04-08_15-53-06.json"

# ── Configuration ─────────────────────────────────────────────────────────────
FOLLOWER_OFFSET = np.array([1.0, 0.0, 0.0])   # follower commanded = leader + offset
ARRIVE_TOL      = 0.03   # m — leader arrival detection tolerance
PAD_BEFORE      = 0.2    # s of context before T1
PAD_AFTER       = 0.2    # s of context after T1 (must cover follower arrival)


# ── Load & parse ──────────────────────────────────────────────────────────────
def _load_raw(path):
    """Load JSON, recovering from truncated files."""
    with open(path) as f:
        content = f.read()
    try:
        return json.loads(content)
    except json.JSONDecodeError:
        data = json.loads(content.rstrip().rstrip(",") + "]")
        print(f"  Warning: truncated JSON in {Path(path).name}, recovered {len(data)} entries.")
        return data


def load_frames(path):
    """Return sorted list of {time, pos, vel} from 'frames' entries."""
    frames = []
    for entry in _load_raw(path):
        if entry.get("type") == "frames" and entry.get("data"):
            d = entry["data"]
            frames.append({
                "time": d["time"],
                "pos":  np.array(d["tvec"]),
                "vel":  np.array(d.get("vel", [0.0, 0.0, 0.0])),
            })
    frames.sort(key=lambda x: x["time"])
    return frames


def load_cmd_steps(path):
    """
    Return list of {time, target_y} for each 'position_setpoint' command
    in the leader log, in chronological order.

    The 'timestamp' field in cmd_positions is an ISO-8601 string recorded
    on the same machine as time.time(), so fromisoformat gives the correct
    wall-clock Unix time when interpreted as local time.
    """
    steps = []
    for entry in _load_raw(path):
        if entry.get("type") == "cmd_positions" and entry.get("data"):
            d = entry["data"]
            if d.get("cmd") == "position_setpoint":
                t = datetime.fromisoformat(d["timestamp"]).timestamp()
                steps.append({"time": t, "target_y": d["y"]})
    steps.sort(key=lambda x: x["time"])
    return steps


def find_leader_arrival(leader_frames, t1, target_y, tol=ARRIVE_TOL):
    """
    Return the timestamp of the first leader frame after t1 where
    |leader_y - target_y| <= tol.  Falls back to the last frame time
    if arrival is never detected.
    """
    for fr in leader_frames:
        if fr["time"] >= t1 and abs(fr["pos"][1] - target_y) <= tol:
            return fr["time"]
    # fallback: last frame in log
    return leader_frames[-1]["time"]


def slice_time(entries, t_start, t_end):
    return [e for e in entries if t_start <= e["time"] <= t_end]


def compute_integral_delay(blocks, frames, offset, t_start, t_end, dt=0.001, return_plot_data=False):
    """
    Calculate time delay using the Area / Integral method.
    Normalizes the step amplitude and integrates the difference.
    """
    t_grid = np.arange(t_start, t_end, dt)

    b_times = np.array([b["time"] for b in blocks])
    # Add the offset to the block Y position to get the true commanded target
    b_y = np.array([b["pos"][1] + offset[1] for b in blocks])

    f_times = np.array([f["time"] for f in frames])
    f_y = np.array([f["pos"][1] for f in frames])

    # Interpolate onto common grid
    b_grid = np.interp(t_grid, b_times, b_y)
    f_grid = np.interp(t_grid, f_times, f_y)

    # Find initial and final steady-state values from the block command
    idx_window = int(0.2 / dt)
    y_init = np.mean(b_grid[:idx_window])
    y_final = np.mean(b_grid[-idx_window:])
    amplitude = y_final - y_init

    if abs(amplitude) < 1e-4:
        if return_plot_data:
            return 0.0, t_grid, b_grid, f_grid
        return 0.0

    # Normalize both signals to go from 0 to 1
    b_norm = (b_grid - y_init) / amplitude
    f_norm = (f_grid - y_init) / amplitude

    # Integrate the difference (Area between curves)
    delay_s = -np.trapz(b_norm - f_norm, dx=dt)

    if return_plot_data:
        return delay_s, t_grid, b_norm, f_norm
    return delay_s



def compute_phase_lag(leader_frames, follower_frames, t_start, t_end,
                      dt=0.0001, max_lag=1.0):
    """
    Normalized cross-correlation of the Y-velocity signals over [t_start, t_end].

    Returns
    -------
    lag_s   : float — lag at peak (s, positive = follower behind leader)
    corr    : ndarray — normalized cross-correlation values
    lags    : ndarray — lag axis in seconds
    """
    t_grid = np.arange(t_start, t_end, dt)

    # 1. Extract Time and Velocity (Y-axis) directly
    l_times = np.array([f["time"] for f in leader_frames])
    l_y = np.array([f["pos"][1] for f in leader_frames])
    l_vy = np.array([f["vel"][1] for f in leader_frames])

    f_times = np.array([f["time"] for f in follower_frames])
    f_y = np.array([f["pos"][1] for f in follower_frames])
    f_vy = np.array([f["vel"][1] for f in follower_frames])

    # 2. Interpolate the Velocity directly onto the common time grid

    # l_grid = np.interp(t_grid, l_times, l_y)
    # f_grid = np.interp(t_grid, f_times, f_y)
    l_grid = np.interp(t_grid, l_times, l_vy)
    f_grid = np.interp(t_grid, f_times, f_vy)

    # 3. Mean-center the velocity pulses
    l_sig = l_grid - l_grid.mean()
    f_sig = f_grid - f_grid.mean()

    # 4. Correlate
    norm = np.linalg.norm(l_sig) * np.linalg.norm(f_sig)
    corr = np.correlate(f_sig, l_sig, mode="full") / (norm if norm > 0 else 1.0)

    n = len(t_grid)
    lags = np.arange(-(n - 1), n) * dt

    # 5. Find peak within ±max_lag
    mask = np.abs(lags) <= max_lag
    lag_s = lags[int(np.argmax(np.where(mask, corr, -np.inf)))]

    return lag_s, corr, lags


def compute_delay_y(leader_frames, follower_frames, offset, t_start, t_end):
    """
    For each leader sample in [t_start, t_end], compute the time delay for
    the follower to reach the target Y position (leader_Y + offset_Y).

    Returns arrays: leader_times, delays (seconds).
    """
    f_times = np.array([f["time"] for f in follower_frames])
    f_y = np.array([f["pos"][1] for f in follower_frames])

    leader_times = []
    delays = []

    for fr in leader_frames:
        t_l = fr["time"]
        if t_l < t_start or t_l > t_end:
            continue

        target_y = fr["pos"][1] + offset[1]

        start_search = np.searchsorted(f_times, t_l)

        for j in range(start_search, len(f_times) - 1):
            y0, y1 = f_y[j], f_y[j + 1]
            t0_f, t1_f = f_times[j], f_times[j + 1]

            if (y0 <= target_y <= y1) or (y1 <= target_y <= y0):
                if abs(y1 - y0) > 1e-9:
                    frac = (target_y - y0) / (y1 - y0)
                    t_cross = t0_f + frac * (t1_f - t0_f)
                else:
                    t_cross = t0_f

                delay = t_cross - t_l
                if delay >= 0:
                    leader_times.append(t_l)
                    delays.append(delay)
                    break

    return np.array(leader_times), np.array(delays)


def top_lag_candidates(corr, lags, top_k=5):
    """Return the top_k lag candidates sorted by descending correlation."""
    top_idx = np.argsort(corr)[-top_k:][::-1]
    return [(lags[i], corr[i]) for i in top_idx]


# ── Styling ───────────────────────────────────────────────────────────────────
def style_ax(ax):
    ax.grid(True, linestyle="--", alpha=0.7)
    ax.tick_params(axis="both", labelsize=16)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


# ── Plot set ──────────────────────────────────────────────────────────────────
def plot_interaction(leader_frames, follower_frames, t1, t10,
                     condition_label, out_prefix):
    """
    Generate and save 3 plots for one interaction.

    Parameters
    ----------
    leader_frames, follower_frames : list of {time, pos, vel}
    t1  : float — Unix time when leader was commanded (T1)
    t10 : float — Unix time when leader arrived at target (T10)
    condition_label : str
    out_prefix : str — prefix for output PNG filenames
    """
    t_vis_start = t1  - PAD_BEFORE
    t_vis_end   = t10  + PAD_AFTER
    t_origin    = t_vis_start          # all relative times are (t - t_origin)

    l_slice = slice_time(leader_frames,   t_vis_start, t_vis_end)
    f_slice = slice_time(follower_frames, t_vis_start, t_vis_end)

    if not l_slice or not f_slice:
        print(f"  [{condition_label}] No data in window — skipping.")
        return

    l_t   = np.array([x["time"] - t_origin for x in l_slice])
    l_pos = np.array([x["pos"]             for x in l_slice])
    f_t   = np.array([x["time"] - t_origin for x in f_slice])
    f_pos = np.array([x["pos"]             for x in f_slice])

    # Shaded region: T1 → T10 (leader movement window)
    t_shade_start = t1  - t_origin
    t_shade_end   = t10 - t_origin

    # Reference Y for Y-position plot: median over 0.5 s before T1 (robust to noise)
    hover_dur = 0.5  # s
    l_hover = [fr["pos"][1] for fr in leader_frames if t1 - hover_dur <= fr["time"] < t1]
    y_ref   = float(np.median(l_hover)) if l_hover else float(np.interp(t_shade_start, l_t, l_pos[:, 1]))

    # ── 1. Y Position ─────────────────────────────────────────────────────
    l_y_mm = (l_pos[:, 1] - y_ref) * 1000
    f_y_mm = (f_pos[:, 1] - y_ref) * 1000

    fig1, ax1 = plt.subplots(figsize=(10, 6))
    ax1.scatter(l_t, l_y_mm, alpha=0.5, s=10, label="Leader Y")
    ax1.scatter(f_t, f_y_mm, alpha=0.5, s=10, label="Follower Y")
    ax1.axvspan(t_shade_start, t_shade_end, alpha=0.08, color="orange",
                label="Leader travel (T1 → T10)")
    ax1.axvline(t_shade_start, color="orange", linewidth=1, linestyle="--")
    ax1.set_title(f"[{condition_label}] Y Position (mm)", loc="left", fontsize=16)
    ax1.set_xlabel("Time (s)", fontsize=16)
    ax1.legend(fontsize=14)
    style_ax(ax1)
    plt.tight_layout()
    p1 = f"{out_prefix}_ypos.png"
    fig1.savefig(p1, dpi=300)
    print(f"  Saved: {p1}")
    plt.show()

    # ── 2. 3-D Distance from ideal formation position ─────────────────────
    # Ideal = leader_pos + FOLLOWER_OFFSET; interpolate follower at leader times
    follower_at_l = np.column_stack([
        np.interp(l_t, f_t, f_pos[:, i]) for i in range(3)
    ])
    ideal_pos = l_pos + FOLLOWER_OFFSET
    dist = np.linalg.norm(follower_at_l - ideal_pos, axis=1) * 1000  # mm

    fig2, ax2 = plt.subplots(figsize=(10, 6))
    ax2.scatter(l_t, dist, alpha=0.5, s=10, color="#c0392b",
                label="Distance to ideal position (mm)")
    ax2.axhline(dist.mean(), color="#2c3e50", linestyle="--", linewidth=1,
                label=f"Mean = {dist.mean():.1f} mm")
    ax2.axvspan(t_shade_start, t_shade_end, alpha=0.08, color="orange",
                label="Leader travel (T1 → T10)")
    ax2.axvline(t_shade_start, color="orange", linewidth=1, linestyle="--")
    ax2.set_title(f"[{condition_label}] 3-D Distance from Ideal Formation Position (mm)",
                  loc="left", fontsize=16)
    ax2.set_xlabel("Time (s)", fontsize=16)
    ax2.legend(fontsize=14)
    style_ax(ax2)
    plt.tight_layout()
    p2 = f"{out_prefix}_distance.png"
    fig2.savefig(p2, dpi=300)
    print(f"  Saved: {p2}")
    plt.show()

    # ── 3. Per-sample following delay ────────────────────────────────────
    delay_l_times, delays = compute_delay_y(
        leader_frames, follower_frames, FOLLOWER_OFFSET, t1, t10
    )

    if len(delays) > 0:
        delay_l_times_rel = delay_l_times - t_origin

        fig3d, ax3d = plt.subplots(figsize=(10, 6))
        ax3d.scatter(delay_l_times_rel, delays * 1000, alpha=0.5, s=10,
                     label="Per-sample delay")
        mean_delay = delays.mean()
        median_delay = np.median(delays)
        ax3d.axhline(mean_delay * 1000, color="#2c3e50", linestyle="--", linewidth=1,
                     label=f"Mean = {mean_delay*1000:.1f} ms")
        ax3d.axhline(median_delay * 1000, color="#e67e22", linestyle=":", linewidth=1,
                     label=f"Median = {median_delay*1000:.1f} ms")
        ax3d.set_ylim(0, 600)
        ax3d.set_xlim(delay_l_times_rel[0], delay_l_times_rel[-1])
        ax3d.set_title(f"[{condition_label}] Following Delay (ms)", loc="left", fontsize=16)
        ax3d.set_xlabel("Time (s)", fontsize=16)
        ax3d.legend(fontsize=16)
        style_ax(ax3d)
        plt.tight_layout()
        p3d = f"{out_prefix}_delay_scatter.png"
        fig3d.savefig(p3d, dpi=300)
        print(f"  Saved: {p3d}")
        plt.show()
    else:
        print(f"  [{condition_label}] No per-sample delay data computed.")
        mean_delay = median_delay = 0.0

    # ── 4. Cross-correlation ──────────────────────────────────────────────
    # Window: 0.5 s before T1 to 2 s after, so the step transient dominates
    # and the flat hover on either side doesn't wash out the lag.
    xcorr_start = t1 - 0.5
    xcorr_end   = t10  + 1.5
    lag_s, corr, lags = compute_phase_lag(
        leader_frames, follower_frames, xcorr_start, xcorr_end
    )
    top_candidates = top_lag_candidates(corr, lags, top_k=5)

    fig3, ax3 = plt.subplots(figsize=(10, 6))
    ax3.plot(lags * 1000, corr, linewidth=1.2, color="#2980b9",
             label="Cross-correlation")
    ax3.axvline(lag_s * 1000, color="#c0392b", linestyle="--", linewidth=1.5,
                label=f"Peak lag = {lag_s*1000:.0f} ms")
    ax3.axvline(0, color="#7f8c8d", linestyle=":", linewidth=1)
    ax3.set_xlim(-1000, 1000)
    ax3.set_title(f"[{condition_label}] Follower Y Phase Lag (cross-correlation)",
                  loc="left", fontsize=16)
    ax3.set_xlabel("Lag (ms)", fontsize=16)
    ax3.set_ylabel("Normalized correlation", fontsize=14)
    ax3.legend(fontsize=14)
    style_ax(ax3)
    plt.tight_layout()
    p3 = f"{out_prefix}_delay.png"
    fig3.savefig(p3, dpi=300)
    print(f"  Saved: {p3}")
    plt.show()

    # ── 4. Area / Integral Delay ──────────────────────────────────────────
    # Window needs to extend well past T10 so the follower finishes settling
    int_start = t1
    int_end = t10+0.2
    integral_delay_s = compute_integral_delay(
        leader_frames, follower_frames, [1, 0, 0], int_start, int_end
    )
    # ── Summary ───────────────────────────────────────────────────────────
    print(f"\n  [{condition_label}] Summary:")
    print(f"    T1  (cmd issued)     : {t1:.3f}  (t+{t_shade_start:.2f} s in plot)")
    print(f"    T10 (leader arrived) : {t10:.3f}  (t+{t_shade_end:.2f} s in plot)")
    print(f"    Leader travel time   : {(t10 - t1) * 1000:.0f} ms")
    print(f"    Phase lag (xcorr)    : {lag_s * 1000:.0f} ms")
    print(f"    Area Delay (integral): {integral_delay_s * 1000:.0f} ms")
    if len(delays) > 0:
        print(f"    Mean delay (sample)  : {mean_delay*1000:.1f} ms")
        print(f"    Median delay (sample): {median_delay*1000:.1f} ms")
        print(f"    Min/Max delay        : {delays.min()*1000:.1f} / {delays.max()*1000:.1f} ms")
        print(f"    Std delay            : {delays.std()*1000:.1f} ms")
    print("    Top 5 lag candidates:")
    for cand_lag_s, cand_corr in top_candidates:
        print(f"      lag={cand_lag_s * 1000:7.3f} ms, corr={cand_corr:.6f}")

    int_start = t1 - 0.2
    int_end = t10 + 0.2

    integral_delay_s, t_grid, l_norm, f_norm = compute_integral_delay(
        leader_frames, follower_frames, [1, 0, 0], int_start, int_end, return_plot_data=True
    )

    # Generate the visual plot for the area
    fig4, ax4 = plt.subplots(figsize=(10, 6))
    plot_t = t_grid - t_origin

    ax4.plot(plot_t, l_norm, label="Leader (Normalized)", color="#2980b9", linewidth=2)
    ax4.plot(plot_t, f_norm, label="Follower (Normalized)", color="#e67e22", linewidth=2)

    # Shade the area between the curves
    ax4.fill_between(plot_t, l_norm, f_norm, where=(l_norm >= f_norm),
                     color="#f1c40f", alpha=0.3, label=f"Area = {integral_delay_s * 1000:.0f} ms")
    ax4.fill_between(plot_t, l_norm, f_norm, where=(l_norm < f_norm),
                     color="#e74c3c", alpha=0.3)  # Highlights any follower overshoot

    ax4.set_title(f"[{condition_label}] Integral Time Delay (Normalized Area)", loc="left", fontsize=16)
    ax4.set_xlabel("Time (s)", fontsize=16)
    ax4.set_ylabel("Normalized Position", fontsize=16)
    ax4.legend(fontsize=14)
    style_ax(ax4)

    plt.tight_layout()
    p4 = f"{out_prefix}_integral.png"
    fig4.savefig(p4, dpi=300)
    print(f"  Saved: {p4}")
    plt.show()
# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    conditions = [
        ("Before FC", BEFORE_LEADER, BEFORE_FOLLOWER,
         str(LOG_DIR / "LFmsgbeforeFC_analysis")),
        ("After FC",  AFTER_LEADER,  AFTER_FOLLOWER,
         str(LOG_DIR / "LFmsgafterFC_analysis")),
    ]

    for label, l_path, f_path, out_prefix in conditions:
        print(f"\n{'='*60}")
        print(f"Condition: {label}")
        print(f"{'='*60}")

        leader_frames   = load_frames(l_path)
        follower_frames = load_frames(f_path)
        cmd_steps       = load_cmd_steps(l_path)

        print(f"  Leader   frames : {len(leader_frames)}")
        print(f"  Follower frames : {len(follower_frames)}")
        print(f"  Leader cmd steps: {len(cmd_steps)}")

        if not cmd_steps:
            print(f"  No cmd_steps found — skipping.")
            continue

        # Use the first step command as the interaction anchor
        step    = cmd_steps[-1]
        t1      = step["time"]
        target_y = step["target_y"]
        t10     = find_leader_arrival(leader_frames, t1, target_y)

        print(f"  Step 0: T1={t1:.3f}, target_y={target_y:.3f} m, T10={t10:.3f}")
        print(f"  Leader travel: {(t10 - t1)*1000:.0f} ms")

        plot_interaction(leader_frames, follower_frames, t1, t10,
                         label, out_prefix)


if __name__ == "__main__":
    main()
