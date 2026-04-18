#!/usr/bin/env python3
"""
speed_video.py  –  render speed-vs-time for every JSON log in a directory into one video.

Video plays at 1:1 real time (30 fps, each frame = 1/30 s of log time).
All log files are concatenated in sorted order.

Usage:
    python speed_video.py [log_dir] [output.mp4]

Defaults:
    log_dir    = ./logs/SIGGRAPH_Poster
    output.mp4 = speed_over_time.mp4

Dependencies:
    pip install imageio imageio-ffmpeg matplotlib numpy
"""

import glob
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import imageio.v2 as imageio

# LOG_DIR        = sys.argv[1] if len(sys.argv) > 1 else '../../logs/SIGGRAPH_Poster/dark'
# OUT_DIR        = sys.argv[2] if len(sys.argv) > 2 else LOG_DIR
# FPS            = 30
# FONT           = 20
# WAIT_OFFSET_S  = 6.50   # seconds after "Waiting For User Interaction" to use as t=0
# VIDEO_DURATION = 19.0  # seconds of log time to render per video


LOG_DIR        = sys.argv[1] if len(sys.argv) > 1 else '../../logs/SIGGRAPH_Poster/lit'
OUT_DIR        = sys.argv[2] if len(sys.argv) > 2 else LOG_DIR
FPS            = 30
FONT           = 20
WAIT_OFFSET_S  = 6.70   # seconds after "Waiting For User Interaction" to use as t=0
VIDEO_DURATION = 11.0  # seconds of log time to render per video




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


def process_log(args):
    """Render one log and write its .mp4. Runs in a worker process."""
    log_path, out_dir = args
    records = load_records(log_path)

    start_time = next(
        (r['data'] for r in records if r.get('type') == 'start'),
        None
    )
    if start_time is None:
        return f"[skip] no 'start' event: {os.path.basename(log_path)}"

    t0 = start_time + WAIT_OFFSET_S  # video t=0

    raw = []
    for r in records:
        if r.get('type') == 'frames':
            vel = r['data']['vel']
            speed_mm = math.sqrt(vel[0] ** 2 + vel[1] ** 2) * 1000
            raw.append((r['data']['time'] - t0, speed_mm))

    # keep only frames within [0, VIDEO_DURATION]
    raw = [(t, s) for t, s in raw if 0.0 <= t <= VIDEO_DURATION]
    if not raw:
        return f"[skip] no frames in window: {os.path.basename(log_path)}"

    log_name = os.path.splitext(os.path.basename(log_path))[0]
    times    = np.array([t for t, _ in raw])
    speeds   = np.array([s for _, s in raw])
    n_frames = max(1, int(math.ceil(VIDEO_DURATION * FPS)))

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_xlim(0, VIDEO_DURATION)
    ax.set_ylim(0, max(float(speeds.max()) * 1.15, 10))
    # ax.set_yscale('symlog', linthresh=1.0)
    # ax.set_yticks([0, 1, 10, 100, 1000])
    ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f'{int(v)}'))
    ax.set_xlabel('Time (s)', fontsize=FONT)
    ax.set_title('Speed (mm/s)', loc='left', fontsize=FONT, x=0)
    # fig.suptitle(log_name, fontsize=FONT - 4)
    ax.tick_params(axis='both', labelsize=FONT)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.grid(True, linestyle='--', alpha=0.7)

    ax.axhline(100, linestyle='--', color='tab:purple', linewidth=1.5,
               label=r'$S_D = S_Q = S_H = 100.0~mm/s$')
    ax.legend(fontsize=FONT - 4)
    line_done,  = ax.plot([], [], linewidth=1.2, color='tab:blue', animated=True)
    line_ahead, = ax.plot([], [], linewidth=1.2, color='lightgray', animated=True)
    vline = ax.axvline(0, color='black', linestyle='--', linewidth=1.2, animated=True)
    fig.tight_layout()

    # Draw static background once, then blit only the animated artists each frame.
    fig.canvas.draw()
    bg = fig.canvas.copy_from_bbox(fig.bbox)
    w, h = fig.canvas.get_width_height()

    out_path = os.path.join(out_dir, f"{log_name}.mp4")
    with imageio.get_writer(out_path, fps=FPS, macro_block_size=1) as writer:
        ptr = 0
        for k in range(n_frames):
            t_now = k / FPS
            while ptr < len(times) and times[ptr] <= t_now:
                ptr += 1
            line_done.set_data(times[:ptr], speeds[:ptr])
            line_ahead.set_data(times[ptr - 1:], speeds[ptr - 1:])  # connect at boundary
            vline.set_xdata([t_now, t_now])

            fig.canvas.restore_region(bg)
            ax.draw_artist(line_ahead)
            ax.draw_artist(line_done)
            ax.draw_artist(vline)
            fig.canvas.blit(fig.bbox)

            buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
            writer.append_data(buf)

    plt.close(fig)
    return f"{log_name}: {n_frames} frames ({VIDEO_DURATION:.1f} s)  → {out_path}"


def main():
    log_files = sorted(glob.glob(os.path.join(LOG_DIR, '*.json')))
    if not log_files:
        print(f"No JSON files found in {LOG_DIR}")
        sys.exit(1)

    os.makedirs(OUT_DIR, exist_ok=True)
    print(f"Found {len(log_files)} log files in {LOG_DIR}")

    args = [(p, OUT_DIR) for p in log_files]
    with ProcessPoolExecutor() as pool:
        futures = {pool.submit(process_log, a): a[0] for a in args}
        for fut in as_completed(futures):
            print(fut.result())


if __name__ == '__main__':
    main()
