"""
velocity_full_video.py

Renders a 30-fps MP4 of the ``plot_velocity_full`` chart clipped to a
configurable time window (default 2–8 s of relative time).

Each video frame reveals data whose real log-timestamp ≤ current playback
time, so the animation is driven by actual timestamps rather than frame index.
A vertical playhead line and a bottom progress bar both advance with real time.

Usage
─────
    python Interaction/plots/velocity_full_video.py \\
        --leader   logs/.../LFMoCapDelay_leader_20_10ms_*.json \\
        --follower logs/.../LFMoCapDelay_follower_20_10ms_*.json \\
        --output   velocity_video.mp4 \\
        --t-start 2 --t-end 8

The script auto-discovers the most recent leader/follower pair from
LOG_DIR when --leader / --follower are omitted.
"""

import argparse
import glob
import json
import os
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation

# ── Import analysis helpers ────────────────────────────────────────────────────
_repo_root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_repo_root))

from Interaction.analysis.analyze_lfmocap_delay_goto_wait import (
    load_log,
    extract_group,
    extract_vel_y_series,
    build_timing_table,
    get_clock_offset,
    apply_clock_offset,
    find_log_pairs,
    DELTA_V,
)

LOG_DIR = os.path.join(_repo_root, "logs", "LFMoCapDelay", 'sync')

FPS = 30


# ── Data helpers ───────────────────────────────────────────────────────────────

def _mask_upto(times: np.ndarray, vels: np.ndarray,
               t_clip_start: float, t_clip_end: float,
               t_playback: float, t0: float):
    """Data points in [t0+t_clip_start, t0+t_playback] — the 'past' (colored) segment."""
    mask = (times >= t0 + t_clip_start) & (times <= t0 + t_playback)
    return times[mask] - t0, vels[mask]


def _mask_from(times: np.ndarray, vels: np.ndarray,
               t_clip_end: float, t_playback: float, t0: float):
    """Data points in (t0+t_playback, t0+t_clip_end] — the 'future' (gray) segment."""
    mask = (times >= t0 + t_playback) & (times <= t0 + t_clip_end)
    return times[mask] - t0, vels[mask]


# ── Main rendering ─────────────────────────────────────────────────────────────

def render_video(leader_log, follower_log, rows, label,
                 t_clip_start, t_clip_end, output_path):
    """Build and save the animation."""

    # ── Extract series ────────────────────────────────────────────────────
    l_times, l_vel = extract_vel_y_series(leader_log,   "frames")
    f_times, f_vel = extract_vel_y_series(follower_log, "frames")

    if len(l_times) < 2:
        print("Not enough leader frame data — aborting.")
        return

    t0 = min(r["T1"] for r in rows) - 2.0   # same origin as analysis script

    has_follower = len(f_times) >= 2
    n_subplots   = 1 + int(has_follower)

    delta_v = rows[0].get("delta_v", DELTA_V) if rows else DELTA_V

    FS = 28   # global font size

    # ── Figure layout ─────────────────────────────────────────────────────
    fig_h = 6 * n_subplots
    fig, axes = plt.subplots(
        n_subplots, 1,
        figsize=(16, fig_h),
        sharex=True,
    )
    if n_subplots == 1:
        axes = [axes]

    # plt.tight_layout(rect=[0, 0.04, 1, 1])   # reserve bottom margin for xlabel

    # ── Static decorations ────────────────────────────────────────────────
    ax_l = axes[0]
    ax_l.set_title(f"Leader Y velocity (m/s)  —  {label}", loc="left", fontsize=FS)
    ax_l.set_ylim(0, 1.2)
    ax_l.set_xlim(t_clip_start, t_clip_end)
    ax_l.grid(True, linestyle="--", alpha=0.6)
    ax_l.spines["top"].set_visible(False)
    ax_l.spines["right"].set_visible(False)
    ax_l.axhline(delta_v, color="red", linestyle=":", linewidth=1.5,
                 label=f"ΔV = {delta_v} m/s")
    t1_lab = t10_lab = False
    for r in rows:
        t1  = r["T1"]  - t0
        t10 = r["T10"] - t0
        if t_clip_start <= t1 <= t_clip_end or t_clip_start <= t10 <= t_clip_end:
            l1 = "T1 (step start)"    if not t1_lab  else "_nolegend_"
            l2 = "T10 (step arrived)" if not t10_lab else "_nolegend_"
            ax_l.axvline(t1,  color="magenta",   linestyle="--", linewidth=1.8,
                         alpha=0.75, label=l1)
            ax_l.axvline(t10, color="darkorange", linestyle="--", linewidth=1.8,
                         alpha=0.75, label=l2)
            ax_l.text(t1, 0.06, f" {r['step']}", fontsize=FS * 0.6, color="#e74c3c")
            t1_lab = t10_lab = True
    ax_l.legend(fontsize=FS * 0.75, loc="upper left")

    ax_idx = 1
    ax_f = ax_lf = None

    if has_follower:
        ax_f = axes[ax_idx]; ax_idx += 1
        ax_f.set_title(f"Follower Y velocity (m/s)  —  {label}", loc="left", fontsize=FS)
        ax_f.set_ylim(0, 1.2)
        ax_f.grid(True, linestyle="--", alpha=0.6)
        ax_f.spines["top"].set_visible(False)
        ax_f.spines["right"].set_visible(False)
        ax_f.axhline(delta_v, color="red", linestyle=":", linewidth=1.5,
                     label=f"ΔV = {delta_v} m/s")
        t2_lab = t11_lab = False
        for r in rows:
            t2  = r["T2"]  - t0
            t11 = r["T11"] - t0
            if t_clip_start <= t2 <= t_clip_end or t_clip_start <= t11 <= t_clip_end:
                l1 = "T2 (detected)"      if not t2_lab  else "_nolegend_"
                l2 = "T11 (step arrived)" if not t11_lab else "_nolegend_"
                ax_f.axvline(t2,  color="magenta",   linestyle="--", linewidth=1.8,
                             alpha=0.75, label=l1)
                ax_f.axvline(t11, color="darkorange", linestyle="--", linewidth=1.8,
                             alpha=0.75, label=l2)
                ax_f.text(t2, 0.06, f" {r['step']}", fontsize=FS * 0.6, color="#27ae60")
                t2_lab = t11_lab = True
        ax_f.legend(fontsize=FS * 0.75, loc="upper left")

    axes[-1].set_xlabel("Time (s)", fontsize=FS)
    for ax in axes:
        ax.tick_params(axis="both", labelsize=FS)

    # ── Animated lines (past = colored, future = gray) ────────────────────
    (line_l,)        = ax_l.plot([], [], color="#2980b9", linewidth=2,
                                  label="Leader Y velocity")
    (line_l_future,) = ax_l.plot([], [], color="#b0b0b0", linewidth=1.5,
                                  zorder=1)
    line_f = line_f_future = None
    if ax_f is not None:
        (line_f,)        = ax_f.plot([], [], color="#27ae60", linewidth=2,
                                      label="Follower Y velocity")
        (line_f_future,) = ax_f.plot([], [], color="#b0b0b0", linewidth=1.5,
                                      zorder=1)

    # Playhead vertical lines (one per subplot)
    playheads = [ax.axvline(t_clip_start, color="#e74c3c",
                            linewidth=1.8, alpha=0.9, zorder=10)
                 for ax in axes]

    plt.tight_layout(rect=[0,0,1,1])
    # ── Frame generation ─────────────────────────────────────────────────
    total_dur  = t_clip_end - t_clip_start          # seconds of content
    n_frames   = int(total_dur * FPS)
    frame_dt   = total_dur / n_frames               # seconds per frame

    def update(frame_idx):
        t_playback = t_clip_start + frame_idx * frame_dt   # relative time

        # Leader: past (colored) + future (gray)
        x, y = _mask_upto(l_times, l_vel, t_clip_start, t_clip_end, t_playback, t0)
        line_l.set_data(x, y)
        x, y = _mask_from(l_times, l_vel, t_clip_end, t_playback, t0)
        line_l_future.set_data(x, y)

        if line_f is not None:
            x, y = _mask_upto(f_times, f_vel, t_clip_start, t_clip_end, t_playback, t0)
            line_f.set_data(x, y)
            x, y = _mask_from(f_times, f_vel, t_clip_end, t_playback, t0)
            line_f_future.set_data(x, y)

        # Playhead
        for ph in playheads:
            ph.set_xdata([t_playback, t_playback])

        return [line_l, line_l_future, line_f, line_f_future, *playheads]

    anim = FuncAnimation(
        fig, update,
        frames=n_frames,
        interval=1000 / FPS,
        blit=False,
    )

    writer = FFMpegWriter(fps=FPS, bitrate=3000,
                          extra_args=["-vcodec", "libx264", "-pix_fmt", "yuv420p"])
    print(f"Rendering {n_frames} frames → {output_path} ...")
    anim.save(output_path, writer=writer, dpi=150)
    print(f"Saved → {output_path}")
    plt.close(fig)


# ── CLI ────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="Render velocity_full as a timestamp-driven 30-fps video"
    )
    ap.add_argument("--leader",   type=str, default=None,
                    help="Path to leader log JSON (glob OK)")
    ap.add_argument("--follower", type=str, default=None,
                    help="Path to follower log JSON (glob OK)")
    ap.add_argument("--output",   type=str, default=None,
                    help="Output MP4 path (default: velocity_full_video.mp4 next to leader log)")
    ap.add_argument("--t-start",  type=float, default=2.0,
                    help="Clip start in seconds of relative time (default 2)")
    ap.add_argument("--t-end",    type=float, default=8.0,
                    help="Clip end   in seconds of relative time (default 8)")
    ap.add_argument("--log-dir",  type=str, default=LOG_DIR,
                    help="Log directory for auto-discovery")
    args = ap.parse_args()

    # ── Resolve log paths ─────────────────────────────────────────────────
    if args.leader and args.follower:
        leader_matches   = sorted(glob.glob(args.leader))
        follower_matches = sorted(glob.glob(args.follower))
        if not leader_matches:
            sys.exit(f"No files matched: {args.leader}")
        if not follower_matches:
            sys.exit(f"No files matched: {args.follower}")
        leader_path   = leader_matches[-1]
        follower_path = follower_matches[-1]
        m = re.search(r"(\d+)_(\d+)ms", os.path.basename(leader_path))
        label = f"alpha={m.group(1)} duration={m.group(2)}ms" if m else "experiment"
    else:
        pairs = find_log_pairs(args.log_dir)
        if not pairs:
            sys.exit(f"No log pairs found in {args.log_dir}")
        label, leader_path, follower_path = pairs[-1]
        print(f"Auto-selected: {label}")

    output_path = args.output or os.path.join(
        os.path.dirname(leader_path),
        "velocity_full_video.mp4",
    )

    print(f"Leader:   {leader_path}")
    print(f"Follower: {follower_path}")

    # ── Load & pre-process ────────────────────────────────────────────────
    leader_log   = load_log(leader_path)
    follower_log = load_log(follower_path)

    offset = get_clock_offset(follower_log)
    if offset:
        print(f"Clock offset (follower − leader): {offset * 1000:+.2f} ms — correcting")
        apply_clock_offset(follower_log, offset)

    rows = build_timing_table(leader_log, follower_log)
    if not rows:
        sys.exit("No matched timing rows — cannot determine t0.")

    render_video(
        leader_log, follower_log, rows, label,
        t_clip_start=args.t_start,
        t_clip_end=args.t_end,
        output_path=output_path,
    )


if __name__ == "__main__":
    main()
