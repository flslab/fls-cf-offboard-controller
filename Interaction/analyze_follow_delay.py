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
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("macosx")


# ── Configuration ────────────────────────────────────────────────────────────
LOG_FILE = "../logs/leader_follower_block_2026-04-07_17-17-58.json"
OFFSET = np.array([0.5, 0.0, 0.0])
VEL_THRESHOLD = 0.1   # m/s on Y-axis to define interaction windows
MIN_SEQ_LEN = 10      # ignore sequences shorter than this (noise spikes)
PAD_BEFORE = 0.5       # seconds of context before interaction starts
PAD_AFTER = 2.0        # seconds after interaction ends (to see drone catch up)


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

    Returns arrays: block_times (relative), delays (seconds).
    """
    # Build sorted arrays for frames
    f_times = np.array([f["time"] for f in frames])
    f_y = np.array([f["pos"][1] for f in frames])

    block_times = []
    delays = []

    for b in blocks:
        t_b = b["time"]
        if t_b < t_start or t_b > t_end:
            continue

        target_y = b["pos"][1] + offset[1]

        # Search frames starting from the block's timestamp
        # (the drone can only react AFTER the block position is observed)
        start_search = np.searchsorted(f_times, t_b)

        found = False
        for j in range(start_search, len(f_times) - 1):
            y0, y1 = f_y[j], f_y[j + 1]
            t0_f, t1_f = f_times[j], f_times[j + 1]

            # Check if target_y is between y0 and y1 (monotonic crossing)
            if (y0 <= target_y <= y1) or (y1 <= target_y <= y0):
                # Linear interpolation for exact crossing time
                if abs(y1 - y0) > 1e-9:
                    frac = (target_y - y0) / (y1 - y0)
                    t_cross = t0_f + frac * (t1_f - t0_f)
                else:
                    t_cross = t0_f

                delay = t_cross - t_b
                if delay >= 0:  # only forward-looking delays make sense
                    block_times.append(t_b)
                    delays.append(delay)
                    found = True
                    break

        # If the drone never reaches target_y, skip this sample

    return np.array(block_times), np.array(delays)


def slice_time(entries, t_start, t_end):
    """Filter entries to those within [t_start, t_end]."""
    return [e for e in entries if t_start <= e["time"] <= t_end]


# ── Plotting ─────────────────────────────────────────────────────────────────
def plot_interaction(blocks, frames, offset, interaction_range, interaction_name, t0):
    """Generate distance and delay plots for one interaction."""
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

    # ── 1. Distance plot ─────────────────────────────────────────────────
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f"{interaction_name}", fontsize=14, fontweight="bold")

    b_t = np.array([b["time"] - t0 for b in b_slice])
    b_pos = np.array([b["pos"] for b in b_slice])
    b_vel_y = np.array([b["vel"][1] for b in b_slice])
    target_pos = b_pos + offset

    f_t = np.array([f["time"] - t0 for f in f_slice])
    f_pos = np.array([f["pos"] for f in f_slice])

    # Interpolate drone position at block timestamps for distance calc
    drone_at_block = np.column_stack([
        np.interp(b_t, f_t, f_pos[:, i]) for i in range(3)
    ])
    dist = np.linalg.norm(drone_at_block - target_pos, axis=1)

    # Interaction time window (shaded)
    t_shade_start = t_int_start - t0
    t_shade_end = t_int_end - t0

    # Plot Y positions
    ax1.plot(b_t, target_pos[:, 1], label="Target (block_Y + offset_Y)",
             color="#e74c3c", linewidth=1.5)
    ax1.plot(f_t, f_pos[:, 1], label="Drone Y", color="#2980b9", linewidth=1.5)
    ax1.axvspan(t_shade_start, t_shade_end, alpha=0.1, color="orange",
                label="Interaction window")
    ax1.set_ylabel("Y position (m)")
    ax1.legend(fontsize=9, loc="upper left")
    ax1.grid(True, alpha=0.3)
    ax1.set_title("Y-axis: Target vs Drone position")

    # Plot 3D distance
    ax2.plot(b_t, dist * 100, color="#c0392b", linewidth=1.2)
    ax2.axhline(dist.mean() * 100, color="#2c3e50", linestyle="--", linewidth=1,
                label=f"Mean = {dist.mean()*100:.1f} cm")
    ax2.axvspan(t_shade_start, t_shade_end, alpha=0.1, color="orange")
    ax2.set_ylabel("3D Distance to target (cm)")
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)
    ax2.set_title("Distance from drone to (block + offset)")

    # Plot block velocity Y
    ax3.plot(b_t, b_vel_y, color="#27ae60", linewidth=1.2)
    ax3.axhline(VEL_THRESHOLD, color="gray", linestyle=":", linewidth=1,
                label=f"Threshold = {VEL_THRESHOLD} m/s")
    ax3.axvspan(t_shade_start, t_shade_end, alpha=0.1, color="orange")
    ax3.set_ylabel("Block vel_Y (m/s)")
    ax3.set_xlabel("Time (s)")
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)
    ax3.set_title("Block Y-velocity")

    plt.tight_layout()
    fname = LOG_FILE.replace(".json", f"_{interaction_name.lower().replace(' ', '_')}_distance.png")
    # plt.savefig(fname, dpi=150)
    print(f"  Saved: {fname}")
    plt.show()

    # ── 2. Delay plot ────────────────────────────────────────────────────
    delay_b_times, delays = compute_delay_y(
        blocks, frames, offset, t_int_start, t_int_end
    )

    if len(delays) == 0:
        print(f"  No delay data for {interaction_name}")
        return

    delay_b_times_rel = delay_b_times - t0

    fig2, (ax_d1, ax_d2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig2.suptitle(f"{interaction_name} — Tracking Delay", fontsize=14, fontweight="bold")

    ax_d1.plot(delay_b_times_rel, delays * 1000, "o-", color="#8e44ad",
               markersize=2, linewidth=1, label="Per-sample delay")
    mean_delay = delays.mean()
    median_delay = np.median(delays)
    ax_d1.axhline(mean_delay * 1000, color="#2c3e50", linestyle="--", linewidth=1,
                  label=f"Mean = {mean_delay*1000:.1f} ms")
    ax_d1.axhline(median_delay * 1000, color="#e67e22", linestyle=":", linewidth=1,
                  label=f"Median = {median_delay*1000:.1f} ms")
    ax_d1.set_ylabel("Delay (ms)")
    ax_d1.legend(fontsize=9)
    ax_d1.grid(True, alpha=0.3)
    ax_d1.set_title("Time for drone to reach each block target Y position")

    # Also show block velocity for context
    b_in_window = [b for b in blocks if t_int_start <= b["time"] <= t_int_end]
    bw_t = np.array([b["time"] - t0 for b in b_in_window])
    bw_vy = np.array([b["vel"][1] for b in b_in_window])
    ax_d2.plot(bw_t, bw_vy, color="#27ae60", linewidth=1.2, label="Block vel_Y")
    ax_d2.axhline(VEL_THRESHOLD, color="gray", linestyle=":", linewidth=1)
    ax_d2.set_ylabel("Block vel_Y (m/s)")
    ax_d2.set_xlabel("Time (s)")
    ax_d2.legend(fontsize=9)
    ax_d2.grid(True, alpha=0.3)

    plt.tight_layout()
    fname2 = LOG_FILE.replace(".json", f"_{interaction_name.lower().replace(' ', '_')}_delay.png")
    # plt.savefig(fname2, dpi=150)
    print(f"  Saved: {fname2}")
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
        plot_interaction(blocks, frames, OFFSET, irange, label, t0)


if __name__ == "__main__":
    main()
