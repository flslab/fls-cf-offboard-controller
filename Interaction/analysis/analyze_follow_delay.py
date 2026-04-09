"""
analyze_follow_delay.py

Analyse the tracking delay of a follower drone during two push interactions.
Focuses on the Y-axis where the block (leader) is pushed in the +Y direction.

For each interaction:
  1. Distance plot: 3D distance from drone to (block + offset) over time.
  2. Delay plot: For each block sample, find the timestamp when the drone
     Y-position reaches (block_Y + offset_Y), using linear interpolation
     between consecutive drone samples.  Delay = drone_arrival_time - block_time.
"""

import json
import os

import numpy as np
import matplotlib.pyplot as plt

from Interaction.analysis.analyze_lf_delay import compute_phase_lag, compute_integral_delay

# ── Configuration ────────────────────────────────────────────────────────────
LOG_FILE = os.path.join(os.path.dirname(__file__), "../..", "logs", "leader_follower_block_2026-04-07_17-17-58.json")

OFFSET = np.array([0.5, 0.0, 0.0])
VEL_THRESHOLD = 0.1   # m/s on Y-axis to define interaction windows
MIN_SEQ_LEN = 10      # ignore sequences shorter than this (noise spikes)
PAD_BEFORE = 0.2       # seconds of context before interaction starts
PAD_AFTER = 0.2      # seconds after interaction ends (to see drone catch up)


# ── Load & parse ─────────────────────────────────────────────────────────────
def load_log(path):
    with open(path) as f:
        data = json.load(f)

    frames, blocks = [], []
    for entry in data:
        if entry.get("type") == "frames" and entry.get("data"):
            d = entry["data"]
            frames.append({
                "time": d["time"],
                "pos": np.array(d["tvec"]),
                "vel": np.array(d.get("vel", [0, 0, 0])),
            })
        elif entry.get("type") == "block" and entry.get("data"):
            d = entry["data"]
            blocks.append({
                "time": d["time"],
                "pos": np.array(d["tvec"]),
                "vel": np.array(d.get("vel", [0, 0, 0])),
            })
    return frames, blocks


def find_interactions(blocks, vel_axis=1, threshold=VEL_THRESHOLD, min_len=MIN_SEQ_LEN):
    """Find consecutive runs where block velocity on `vel_axis` exceeds threshold."""
    sequences = []
    in_seq = False
    start_idx = 0

    for i, b in enumerate(blocks):
        above = b["vel"][vel_axis] > threshold
        if above and not in_seq:
            start_idx = i
            in_seq = True
        elif not above and in_seq:
            if (i - start_idx) >= min_len:
                sequences.append((start_idx, i - 1))
            in_seq = False

    if in_seq and (len(blocks) - start_idx) >= min_len:
        sequences.append((start_idx, len(blocks) - 1))

    return sequences


def compute_delay_y(blocks, frames, offset, t_start, t_end):
    """
    For each block sample in [t_start, t_end], compute the time delay for
    the drone to reach the target Y position (block_Y + offset_Y).

    Method: For a given block sample at time t_b with target position target_y,
    search forward in the drone's (frames) timeline for the moment the drone's
    Y position crosses target_y.  Between consecutive frame samples, assume
    linear motion and interpolate the exact crossing time.

    Returns arrays: block_times, delays (seconds).
    """
    f_times = np.array([f["time"] for f in frames])
    f_y = np.array([f["pos"][1] for f in frames])

    block_times = []
    delays = []

    for b in blocks:
        t_b = b["time"]
        if t_b < t_start or t_b > t_end:
            continue

        target_y = b["pos"][1] + offset[1]

        start_search = np.searchsorted(f_times, t_b)

        for j in range(start_search, len(f_times) - 1):
            y0, y1 = f_y[j], f_y[j + 1]
            t0_f, t1_f = f_times[j], f_times[j + 1]

            if (y0 <= target_y <= y1) or (y1 <= target_y <= y0):
                if abs(y1 - y0) > 1e-9:
                    frac = (target_y - y0) / (y1 - y0)
                    t_cross = t0_f + frac * (t1_f - t0_f)
                else:
                    t_cross = t0_f

                delay = t_cross - t_b
                if delay >= 0:
                    block_times.append(t_b)
                    delays.append(delay)
                    break

    return np.array(block_times), np.array(delays)


def slice_time(entries, t_start, t_end):
    """Filter entries to those within [t_start, t_end]."""
    return [e for e in entries if t_start <= e["time"] <= t_end]

def top_lag_candidates(corr, lags, top_k=5):
    """Return the top_k lag candidates sorted by descending correlation."""
    top_idx = np.argsort(corr)[-top_k:][::-1]
    return [(lags[i], corr[i]) for i in top_idx]


def style_ax(ax):
    """Apply consistent draw_friction.py-style formatting."""
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.tick_params(axis='both', labelsize=16)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

# ── Plotting ─────────────────────────────────────────────────────────────────
def plot_interaction(blocks, frames, offset, interaction_range, interaction_name):
    """Generate Y-position, distance, delay, and cross-correlation plots."""
    idx_start, idx_end = interaction_range
    t_int_start = blocks[idx_start]["time"]
    t_int_end = blocks[idx_end]["time"]

    # Padded window for visualization
    t_vis_start = t_int_start - PAD_BEFORE
    t_vis_end = t_int_end + PAD_AFTER

    b_slice = slice_time(blocks, t_vis_start, t_vis_end)
    f_slice = slice_time(frames, t_vis_start, t_vis_end)

    if not b_slice or not f_slice:
        print(f"  No data in window for {interaction_name}, skipping.")
        return

    # ── Common data (time starts at 0 = vis window start) ────────────────
    t_origin = t_vis_start
    b_t = np.array([b["time"] - t_origin for b in b_slice])
    b_pos = np.array([b["pos"] for b in b_slice])
    target_pos = b_pos + offset

    f_t = np.array([f["time"] - t_origin for f in f_slice])
    f_pos = np.array([f["pos"] for f in f_slice])


    # Interpolate drone position at block timestamps for distance calc
    drone_at_block = np.column_stack([
        np.interp(b_t, f_t, f_pos[:, i]) for i in range(3)
    ])
    dist = np.linalg.norm(drone_at_block - target_pos, axis=1)

    t_shade_start = t_int_start - t_origin
    t_shade_end = t_int_end - t_origin

    # ── 1. Y position plot (1 s before/after detected interaction) ───────
    ypos_t_start = t_int_start - 1.0
    ypos_t_end = t_int_end + 1.0
    ypos_t_origin = ypos_t_start

    b_ypos = slice_time(blocks, ypos_t_start, ypos_t_end)
    f_ypos = slice_time(frames, ypos_t_start, ypos_t_end)

    b_t_yp = np.array([b["time"] - ypos_t_origin for b in b_ypos])
    b_pos_yp = np.array([b["pos"] for b in b_ypos])
    target_pos_yp = b_pos_yp + offset

    f_t_yp = np.array([f["time"] - ypos_t_origin for f in f_ypos])
    f_pos_yp = np.array([f["pos"] for f in f_ypos])

    # Zero-reference Y to the block position at window start
    y_origin_yp = b_pos_yp[0][1] if len(b_pos_yp) > 0 else 0
    target_y_zeroed = (target_pos_yp[:, 1] - y_origin_yp) * 1000
    drone_y_zeroed = (f_pos_yp[:, 1] - y_origin_yp) * 1000

    fig1, ax1 = plt.subplots(1, 1, figsize=(10, 6))

    ax1.scatter(b_t_yp, target_y_zeroed, alpha=0.5, s=10, label="Commanded Position Y (block Y + offset)")
    ax1.scatter(f_t_yp, drone_y_zeroed, alpha=0.5, s=10, label="Drone Y")
    t_shade_start_yp = t_int_start - ypos_t_origin
    t_shade_end_yp = t_int_end - ypos_t_origin
    ax1.axvspan(t_shade_start_yp, t_shade_end_yp, alpha=0.08, color="orange")
    ax1.set_ylim(0, 1200)
    ax1.set_xlim(b_t_yp[0], b_t_yp[-1])
    ax1.set_title("Y Position (mm)", loc="left", fontsize=16)
    ax1.set_xlabel("Time (s)", fontsize=16)
    ax1.legend(fontsize=16)
    style_ax(ax1)

    plt.tight_layout()
    fname1 = LOG_FILE.replace(".json", f"_{interaction_name.lower().replace(' ', '_')}_ypos.png")
    fig1.savefig(fname1, dpi=300)
    print(f"  Saved: {fname1}")
    plt.show()

    # ── 2. Distance to theoretical position ──────────────────────────────
    fig2, ax2 = plt.subplots(1, 1, figsize=(10, 6))

    ax2.scatter(b_t, dist * 1000, alpha=0.5, s=10, color="#c0392b", label="Euclidean Distance")
    ax2.axhline(dist.mean() * 1000, color="#2c3e50", linestyle="--", linewidth=1,
                label=f"Mean = {dist.mean()*1000:.1f} mm")
    ax2.axvspan(t_shade_start, t_shade_end, alpha=0.08, color="orange")
    ax2.set_ylim(0, 600)
    ax2.set_xlim(b_t[0], b_t[-1])
    ax2.set_title("Distance to Theoretical Position (mm)", loc="left", fontsize=16)
    ax2.set_xlabel("Time (s)", fontsize=16)
    ax2.legend(fontsize=16)
    style_ax(ax2)

    plt.tight_layout()
    fname2 = LOG_FILE.replace(".json", f"_{interaction_name.lower().replace(' ', '_')}_distance.png")
    fig2.savefig(fname2, dpi=300)
    print(f"  Saved: {fname2}")
    plt.show()

    # ── 3. Delay plot ────────────────────────────────────────────────────
    delay_b_times, delays = compute_delay_y(
        blocks, frames, offset, t_int_start, t_int_end
    )

    if len(delays) == 0:
        print(f"  No delay data for {interaction_name}")
        return

    delay_b_times_rel = delay_b_times - t_origin

    fig3, ax_d = plt.subplots(1, 1, figsize=(10, 6))

    ax_d.scatter(delay_b_times_rel, delays * 1000, alpha=0.5, s=10, label="Per-sample delay")
    mean_delay = delays.mean()
    median_delay = np.median(delays)
    ax_d.axhline(mean_delay * 1000, color="#2c3e50", linestyle="--", linewidth=1,
                 label=f"Mean = {mean_delay*1000:.1f} ms")
    ax_d.axhline(median_delay * 1000, color="#e67e22", linestyle=":", linewidth=1,
                 label=f"Median = {median_delay*1000:.1f} ms")
    ax_d.set_ylim(0, 600)
    ax_d.set_xlim(delay_b_times_rel[0], delay_b_times_rel[-1])
    ax_d.set_title("Following Delay (ms)", loc="left", fontsize=16)
    ax_d.set_xlabel("Time (s)", fontsize=16)
    ax_d.legend(fontsize=16)
    style_ax(ax_d)

    plt.tight_layout()
    fname3 = LOG_FILE.replace(".json", f"_{interaction_name.lower().replace(' ', '_')}_delay.png")
    fig3.savefig(fname3, dpi=300)
    print(f"  Saved: {fname3}")
    plt.show()

    # ── 4. Cross-correlation plot ───────────────────────────────────────
    xcorr_start = t_int_start
    xcorr_end = t_int_end
    lag_s, corr, lags = compute_phase_lag(
        blocks, frames, xcorr_start, xcorr_end
    )
    top_candidates = top_lag_candidates(corr, lags, top_k=5)

    fig4, ax4 = plt.subplots(1, 1, figsize=(10, 6))
    ax4.plot(lags * 1000, corr, linewidth=1.2, color="#2980b9",
             label="Cross-correlation")
    ax4.axvline(lag_s * 1000, color="#c0392b", linestyle="--", linewidth=1.5,
                label=f"Peak lag = {lag_s*1000:.0f} ms")
    ax4.axvline(0, color="#7f8c8d", linestyle=":", linewidth=1)
    ax4.set_xlim(-1000, 1000)
    ax4.set_title("Follower Y Phase Lag (cross-correlation)", loc="left", fontsize=16)
    ax4.set_xlabel("Lag (ms)", fontsize=16)
    ax4.set_ylabel("Normalized correlation", fontsize=14)
    ax4.legend(fontsize=16)
    style_ax(ax4)

    plt.tight_layout()
    fname4 = LOG_FILE.replace(".json", f"_{interaction_name.lower().replace(' ', '_')}_xcorr.png")
    fig4.savefig(fname4, dpi=300)
    print(f"  Saved: {fname4}")
    plt.show()

    # ── 5. Area / Integral Delay plot ────────────────────────────────────
    # Start 0.5s before the push begins to get a clean baseline, and end
    # 1.5s after the push finishes to ensure the drone has fully settled.
    int_start = t_int_start-0.2
    int_end = t_int_end+0.3

    integral_delay_s, t_grid, b_norm, f_norm = compute_integral_delay(
        blocks, frames, offset, int_start, int_end, return_plot_data=True
    )

    fig5, ax5 = plt.subplots(1, 1, figsize=(10, 6))
    plot_t = t_grid - t_origin

    ax5.plot(plot_t, b_norm, label="Commanded (Normalized)", color="#2980b9", linewidth=2)
    ax5.plot(plot_t, f_norm, label="Follower (Normalized)", color="#e67e22", linewidth=2)

    ax5.fill_between(plot_t, b_norm, f_norm, where=(b_norm >= f_norm),
                     color="#f1c40f", alpha=0.3, label=f"Area = {integral_delay_s * 1000:.0f} ms")
    ax5.fill_between(plot_t, b_norm, f_norm, where=(b_norm < f_norm),
                     color="#e74c3c", alpha=0.3)

    ax5.set_title("Integral Time Delay (Normalized Area)", loc="left", fontsize=16)
    ax5.set_xlabel("Time (s)", fontsize=16)
    ax5.set_ylabel("Normalized Position", fontsize=16)
    ax5.legend(fontsize=16)
    style_ax(ax5)

    plt.tight_layout()
    fname5 = LOG_FILE.replace(".json", f"_{interaction_name.lower().replace(' ', '_')}_integral.png")
    fig5.savefig(fname5, dpi=300)
    print(f"  Saved: {fname5}")
    plt.show()

    # ── Summary ──────────────────────────────────────────────────────────
    print(f"\n  {interaction_name} Summary:")
    print(f"    Duration:     {t_int_end - t_int_start:.2f} s")
    print(f"    Samples used: {len(delays)} / {idx_end - idx_start + 1}")
    print(f"    Mean delay:   {mean_delay*1000:.1f} ms")
    print(f"    Median delay: {median_delay*1000:.1f} ms")
    print(f"    Min delay:    {delays.min()*1000:.1f} ms")
    print(f"    Max delay:    {delays.max()*1000:.1f} ms")
    print(f"    Std delay:    {delays.std()*1000:.1f} ms")
    print(f"    Phase lag:    {lag_s*1000:.0f} ms")
    print("    Top 5 lag candidates:")
    for cand_lag_s, cand_corr in top_candidates:
        print(f"      lag={cand_lag_s*1000:7.3f} ms, corr={cand_corr:.6f}")


# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    frames, blocks = load_log(LOG_FILE)
    print(f"Loaded {len(frames)} frame entries, {len(blocks)} block entries")

    interactions = find_interactions(blocks)
    print(f"Found {len(interactions)} interactions (vy > {VEL_THRESHOLD}, min {MIN_SEQ_LEN} samples)")

    if len(interactions) < 2:
        print("ERROR: Expected 2 interactions, found", len(interactions))
        return

    t0 = blocks[0]["time"]
    labels = ["Interaction 1 — Slow Push", "Interaction 2 — Quick Push"]

    for i, (irange, label) in enumerate(zip(interactions[:2], labels)):
        idx_s, idx_e = irange
        print(f"\n{'='*60}")
        print(f"{label}")
        print(f"  Block indices: {idx_s}–{idx_e}  ({idx_e - idx_s + 1} samples)")
        print(f"  Time window:   {blocks[idx_s]['time']-t0:.2f}–{blocks[idx_e]['time']-t0:.2f} s")
        print(f"{'='*60}")
        plot_interaction(blocks, frames, OFFSET, irange, label)


if __name__ == "__main__":
    main()
