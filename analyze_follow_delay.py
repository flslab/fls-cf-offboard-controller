"""
analyze_follow_delay.py
Analyse the tracking delay of a follower drone by comparing
  - "frames"  : the follower's own mocap position (what it actually is)
  - "block"   : the leader's mocap position (what the follower is commanded to track)

The follower is commanded to go to  leader_pos + follow_offset.
Delay is measured as:
  1. Time lag: cross-correlation of position signals to find the best time shift.
  2. Position error: instantaneous distance between follower and (leader + offset).

Usage:
    python analyze_follow_delay.py <log_file> [--offset 0.5 0 0]
"""

import argparse
import json
import sys

import matplotlib.pyplot as plt
import numpy as np


def load_log(path):
    with open(path) as f:
        data = json.load(f)

    frames = []   # follower drone position
    blocks = []   # leader position (what follower tries to follow)

    for entry in data:
        if entry.get("type") == "frames" and entry.get("data"):
            d = entry["data"]
            frames.append({"time": d["time"], "tvec": np.array(d["tvec"])})
        elif entry.get("type") == "block" and entry.get("data"):
            d = entry["data"]
            blocks.append({"time": d["time"], "tvec": np.array(d["tvec"])})

    return frames, blocks


def to_arrays(entries):
    """Convert list of dicts to time array and Nx3 position array."""
    t = np.array([e["time"] for e in entries])
    pos = np.array([e["tvec"] for e in entries])
    return t, pos


def time_lag_xcorr(t_follower, sig_follower, t_leader, sig_leader, dt=0.01):
    """Estimate time lag via cross-correlation on uniformly resampled signals."""
    t_start = max(t_follower[0], t_leader[0])
    t_end = min(t_follower[-1], t_leader[-1])
    t_uniform = np.arange(t_start, t_end, dt)

    # Resample both signals to uniform grid
    f_interp = np.interp(t_uniform, t_follower, sig_follower)
    l_interp = np.interp(t_uniform, t_leader, sig_leader)

    # Zero-mean
    f_interp -= f_interp.mean()
    l_interp -= l_interp.mean()

    # Cross-correlate
    corr = np.correlate(f_interp, l_interp, mode="full")
    lags = np.arange(-len(l_interp) + 1, len(f_interp)) * dt

    best_idx = np.argmax(corr)
    best_lag = lags[best_idx]
    return best_lag, lags, corr


def main(log_file, offset):
    frames, blocks = load_log(log_file)
    if not blocks:
        print("ERROR: No 'block' entries found in the log file.")
        print("       This log was likely produced before the 'block' logging was added.")
        print("       Re-run the flight with the updated controller_follow.py to get block data.")
        sys.exit(1)

    print(f"Loaded {len(frames)} frame entries, {len(blocks)} block entries")

    t_f, pos_f = to_arrays(frames)
    t_b, pos_b = to_arrays(blocks)

    # Normalise time to start at 0
    t0 = min(t_f[0], t_b[0])
    t_f -= t0
    t_b -= t0

    # The follower is commanded to: leader_pos + offset
    target_pos = pos_b + offset  # what the follower SHOULD be at

    # ── 1. Cross-correlation time lag per axis ───────────────────────────
    axis_names = ["X", "Y", "Z"]
    lags = []
    print("\n── Cross-Correlation Time Lag ──")
    for i, ax in enumerate(axis_names):
        lag, lag_arr, corr = time_lag_xcorr(t_f, pos_f[:, i], t_b, target_pos[:, i])
        lags.append(lag)
        print(f"  {ax}-axis lag: {lag*1000:.1f} ms")

    # ── 2. Instantaneous position error ──────────────────────────────────
    # Interpolate follower position at block timestamps
    follower_at_block_times = np.column_stack([
        np.interp(t_b, t_f, pos_f[:, i]) for i in range(3)
    ])
    error = follower_at_block_times - target_pos
    error_dist = np.linalg.norm(error, axis=1)

    print("\n── Position Error (follower vs leader+offset) ──")
    print(f"  Mean distance error:   {error_dist.mean()*100:.2f} cm")
    print(f"  Median distance error: {np.median(error_dist)*100:.2f} cm")
    print(f"  Max distance error:    {error_dist.max()*100:.2f} cm")
    print(f"  Std distance error:    {error_dist.std()*100:.2f} cm")

    for i, ax in enumerate(axis_names):
        print(f"  {ax} mean error: {error[:, i].mean()*100:.2f} cm  (std {error[:, i].std()*100:.2f} cm)")

    # ── 3. Plots ─────────────────────────────────────────────────────────
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle("Leader-Follower Tracking Delay Analysis", fontsize=14, fontweight="bold")

    colors_target = ["#e74c3c", "#27ae60", "#2980b9"]
    colors_follow = ["#f39c12", "#1abc9c", "#8e44ad"]

    for i, ax_name in enumerate(axis_names):
        axes[i].plot(t_b, target_pos[:, i], label=f"Leader+offset ({ax_name})",
                     color=colors_target[i], linewidth=1, alpha=0.8)
        axes[i].plot(t_f, pos_f[:, i], label=f"Follower ({ax_name})",
                     color=colors_follow[i], linewidth=1, alpha=0.8)
        axes[i].set_ylabel(f"{ax_name} (m)")
        axes[i].legend(loc="upper right", fontsize=9)
        axes[i].grid(True, alpha=0.3)
        axes[i].set_title(f"{ax_name}-axis  |  xcorr lag = {lags[i]*1000:.1f} ms", fontsize=10)

    # Distance error
    axes[3].plot(t_b, error_dist * 100, color="#c0392b", linewidth=0.8, alpha=0.8)
    axes[3].axhline(error_dist.mean() * 100, color="#2c3e50", linestyle="--", linewidth=1,
                    label=f"Mean = {error_dist.mean()*100:.2f} cm")
    axes[3].set_ylabel("Error (cm)")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="upper right", fontsize=9)
    axes[3].grid(True, alpha=0.3)
    axes[3].set_title("3D position error (follower – target)", fontsize=10)

    plt.tight_layout()
    plt.savefig(log_file.replace(".json", "_delay_analysis.png"), dpi=150)
    plt.show()

    # ── 4. Cross-correlation plot ────────────────────────────────────────
    fig2, axes2 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig2.suptitle("Cross-Correlation (follower vs leader+offset)", fontsize=14, fontweight="bold")

    for i, ax_name in enumerate(axis_names):
        lag, lag_arr, corr = time_lag_xcorr(t_f, pos_f[:, i], t_b, target_pos[:, i])
        corr_norm = corr / corr.max() if corr.max() > 0 else corr
        axes2[i].plot(lag_arr * 1000, corr_norm, linewidth=0.8, color=colors_target[i])
        axes2[i].axvline(lag * 1000, color="#2c3e50", linestyle="--", linewidth=1,
                         label=f"Peak = {lag*1000:.1f} ms")
        axes2[i].set_ylabel(f"{ax_name} correlation")
        axes2[i].legend(fontsize=9)
        axes2[i].grid(True, alpha=0.3)
        axes2[i].set_xlim(-500, 500)

    axes2[2].set_xlabel("Lag (ms)")
    plt.tight_layout()
    plt.savefig(log_file.replace(".json", "_xcorr.png"), dpi=150)
    plt.show()


if __name__ == "__main__":
    log_file = "./logs/leader_follower_block_2026-04-07_16-55-30.json"
    main(log_file, [0.5, 0, 0])
