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
import argparse
import subprocess
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import imageio.v2 as imageio
from PIL import Image, ImageDraw, ImageFont

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
WAIT_OFFSET_S  = 6.60   # seconds after "Waiting For User Interaction" to use as t=0
VIDEO_DURATION = 11.0  # seconds of log time to render per video

SCALE_START_S  = 5.21  # start of speed-delta scaling window (seconds after video t=0)
SCALE_END_S    = 8.50  # end of speed-delta scaling window (seconds after video t=0)
SCALE_FACTOR   = 1.0   # divide speed deltas by this within the window



DASH_MODE        = 'light'   # 'dark' | 'light'
DASH_TRANSPARENT = True     # True → .webm with alpha channel
FLIP_LAYOUT      = False    # True → bar on left, content on right
CHROMA_GREEN     = (0, 255, 0)   # chroma-key fallback (unused when DASH_TRANSPARENT=True)

DASH_W, DASH_H = 360, 1040

_PANEL_PAD = 20
_PANEL_R   = 20

# Bar — shifted right to make room for larger ticks
_BAR_W   = 64
_VBAR_X0 = 180
_VBAR_X1 = _VBAR_X0 + _BAR_W

# Circle — centred under bar, overlapping bar bottom
_CIRCLE_R  = 68
_CIRCLE_CX = _VBAR_X0 + _BAR_W // 2
_VBAR_Y0   = _PANEL_PAD + 52
_VBAR_Y1   = DASH_H - _PANEL_PAD - 2 * _CIRCLE_R - 20
_CIRCLE_CY = _VBAR_Y1 + _CIRCLE_R - 10
_VBAR_H    = _VBAR_Y1 - _VBAR_Y0

_TICK_X0 = _VBAR_X0 - 24
_TICK_X1 = _VBAR_X0 - 8
_LABEL_X = _VBAR_X0 - 32

_THRESHOLD_COLOR = (30, 120, 255)

# Navy → purple → magenta → red → yellow  (low speed → high speed)
_COLOR_STOPS = [
    (0.00, ( 22,  22, 108)),
    (0.25, (132,  15, 195)),
    (0.50, (205,  20, 118)),
    (0.75, (235,  35,  20)),
    (1.00, (255, 200,   0)),
]

_THEMES = {
    'dark': dict(
        bg         = (4,   8,  20),
        text_spd   = (220, 240, 255),
        text_unit  = (80,  140, 200),
        text_thr   = (0,   180, 255),
        border_out = (0,   140, 220),
        border_mid = (0,   70,  140),
        trough     = (8,   14,  30),
        unlit_div  = 6,
    ),
    'light': dict(
        bg         = (238, 240, 250),
        text_spd   = (12,  12,  32),
        text_unit  = (98,  98,  125),
        text_thr   = (122, 32,  195),
        border_out = (72,  75,  96),
        border_mid = (148, 150, 170),
        trough     = (192, 195, 214),
        unlit_div  = 4,
    ),
}


def _load_font(size):
    for path in [
        '/System/Library/Fonts/Helvetica.ttc',
        '/System/Library/Fonts/SFNSMono.ttf',
        '/Library/Fonts/Arial.ttf',
        '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
        '/usr/share/fonts/truetype/freefont/FreeSansBold.ttf',
    ]:
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            continue
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


class AlphaMovWriter:
    """Write RGBA frames to a HEVC MOV with transparency via direct ffmpeg pipe."""
    def __init__(self, path, w, h, fps):
        self._proc = subprocess.Popen(
            ['ffmpeg', '-y',
             '-f', 'rawvideo', '-pix_fmt', 'rgba', '-s', f'{w}x{h}', '-r', str(fps),
             '-i', 'pipe:0',
             '-c:v', 'prores_ks', '-profile:v', '4', '-pix_fmt', 'yuva444p10le',
             '-loglevel', 'error', path],
            stdin=subprocess.PIPE,
        )

    def append_data(self, frame_rgba):
        self._proc.stdin.write(frame_rgba.tobytes())

    def close(self):
        self._proc.stdin.close()
        self._proc.wait()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def _lerp_grad(t):
    t = max(0.0, min(1.0, t))
    for j in range(len(_COLOR_STOPS) - 1):
        t0, c0 = _COLOR_STOPS[j]
        t1, c1 = _COLOR_STOPS[j + 1]
        if t <= t1:
            s = (t - t0) / (t1 - t0 + 1e-9)
            return tuple(int(c0[k] + s * (c1[k] - c0[k])) for k in range(3))
    return _COLOR_STOPS[-1][1]


def render_dashboard_frame(cur_speed, max_speed, fonts):
    """Slim futuristic HUD — translucent panel, vertical speed bar, circle readout."""
    THRESHOLD = 100.0
    frac     = min(cur_speed, max_speed) / max_speed
    frac_thr = THRESHOLD / max_speed
    fill_y   = int(_VBAR_Y1 - frac     * _VBAR_H)
    thr_y    = int(_VBAR_Y1 - frac_thr * _VBAR_H)

    tc   = _THRESHOLD_COLOR
    img  = Image.new('RGBA', (DASH_W, DASH_H), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)

    f_label, f_scale, f_thr_f, f_num, f_unit = fonts

    if DASH_MODE == 'dark':
        dim_col  = (210, 225, 250, 230)   # bright — visible on black
        trough_c = (90,  90,  140, 70)    # subtle bright trough on dark
    else:
        dim_col  = (50,  65,  100, 220)   # dark — visible on white/light
        trough_c = (20,  20,   50, 55)    # subtle dark trough on light

    # ── "SPEED" header ───────────────────────────────────────────────────────
    lbl = 'SPEED'
    bb  = draw.textbbox((0, 0), lbl, font=f_label)
    lw  = bb[2] - bb[0]
    draw.text((DASH_W // 2 - lw // 2, _PANEL_PAD + 12), lbl, fill=dim_col, font=f_label)

    # ── Bar fill row-by-row (full gradient, unlit region translucent) ────────
    for y in range(_VBAR_Y0, _VBAR_Y1 + 1):
        fy  = (_VBAR_Y1 - y) / (_VBAR_H + 1e-9)
        col = _lerp_grad(fy) + (255,) if y >= fill_y else trough_c
        draw.line([(_VBAR_X0, y), (_VBAR_X1, y)], fill=col)

    # Fill-edge glow
    if _VBAR_Y0 < fill_y < _VBAR_Y1:
        ec = _lerp_grad((_VBAR_Y1 - fill_y) / (_VBAR_H + 1e-9))
        for w, a in [(10, 55), (4, 130), (2, 240)]:
            draw.line([(_VBAR_X0, fill_y), (_VBAR_X1, fill_y)],
                      fill=tuple(min(255, v + 70) for v in ec) + (a,), width=w)

    # ── Threshold glow line ──────────────────────────────────────────────────
    for w, a in [(10, 40), (6, 110), (2, 240)]:
        draw.line([(_VBAR_X0, thr_y), (_VBAR_X1, thr_y)], fill=tc + (a,), width=w)

    # ── Ticks and labels ─────────────────────────────────────────────────────
    tick_step = 50 if max_speed <= 350 else 100
    v = 0.0
    while v <= max_speed + 0.1:
        if v == 0:
            v += tick_step
            continue
        ty_t = int(_VBAR_Y1 - (v / max_speed) * _VBAR_H)
        is_t = abs(v - THRESHOLD) < 0.1
        t_col = tc + (240,) if is_t else dim_col[:3] + (140,)
        draw.line([(_TICK_X0, ty_t), (_TICK_X1, ty_t)],
                  fill=t_col, width=4 if is_t else 2)
        lbl_s = f'{int(v)}'
        font  = f_thr_f if is_t else f_scale
        col_s = tc + (220,) if is_t else dim_col
        bb2   = draw.textbbox((0, 0), lbl_s, font=font)
        lw2, lh2 = bb2[2] - bb2[0], bb2[3] - bb2[1]
        draw.text((_LABEL_X - lw2, ty_t - lh2 // 2), lbl_s, fill=col_s, font=font)
        v += tick_step

    # ── Circle speed readout — fill = gradient color at current speed ────────
    cx, cy, r  = _CIRCLE_CX, _CIRCLE_CY, _CIRCLE_R
    fill_col   = _lerp_grad(frac)

    for extra, alpha in [(14, 20), (8, 55), (4, 120)]:
        draw.ellipse([cx-r-extra, cy-r-extra, cx+r+extra, cy+r+extra],
                     outline=fill_col + (alpha,), width=4)
    draw.ellipse([cx - r, cy - r, cx + r, cy + r],
                 fill=fill_col + (230,), outline=fill_col + (255,), width=4)

    num_str = f'{cur_speed:.0f}'
    bb_n = draw.textbbox((0, 0), num_str, font=f_num)
    bb_u = draw.textbbox((0, 0), 'mm/s',  font=f_unit)
    nw, nh = bb_n[2] - bb_n[0], bb_n[3] - bb_n[1]
    uw, uh = bb_u[2] - bb_u[0], bb_u[3] - bb_u[1]
    ny = cy - (nh + 4 + uh) // 2 - 4
    uy = ny + nh + 4
    draw.text((cx - nw // 2, ny), num_str, fill=(255, 255, 255, 255), font=f_num)
    draw.text((cx - uw // 2, uy), 'mm/s',  fill=(220, 220, 220, 200), font=f_unit)

    out = np.array(img)
    return out if DASH_TRANSPARENT else out[:, :, :3]


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

    # Scale down speed deltas within [SCALE_START_S, SCALE_END_S] by SCALE_FACTOR.
    # Accumulate divided deltas from the first frame in the window.
    in_window = (times >= SCALE_START_S) & (times <= SCALE_END_S)
    idxs = np.where(in_window)[0]
    if len(idxs) > 1:
        for i in idxs[1:]:
            # speeds[i] = speeds[i - 1] + (speeds[i] - speeds[i - 1]) / SCALE_FACTOR
            speeds[i] = speeds[i] / SCALE_FACTOR

    n_frames = max(1, int(math.ceil(VIDEO_DURATION * FPS)))
    bar_max  = max(float(speeds.max()) * 1.1, 200.0)  # dashboard speed axis max

    fig, ax = plt.subplots(figsize=(10, 6), dpi=300)
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

    out_plot_path = os.path.join(out_dir, f"{log_name}.mp4")
    dash_ext      = 'mov' if DASH_TRANSPARENT else 'mp4'
    out_dash_path = os.path.join(out_dir, f"{log_name}_dashboard.{dash_ext}")
    # f_label, f_scale, f_thr_f, f_num, f_unit
    fonts = (_load_font(40), _load_font(52), _load_font(52), _load_font(68), _load_font(28))

    # Probe dashboard frame dimensions from the first rendered frame
    _probe_frame = render_dashboard_frame(0.0, 200.0, fonts)
    dash_h, dash_w = _probe_frame.shape[:2]

    if DASH_TRANSPARENT:
        dash_writer_ctx = AlphaMovWriter(out_dash_path, dash_w, dash_h, FPS)
    else:
        dash_writer_kw = dict(fps=FPS, macro_block_size=1, quality=10)
        dash_writer_ctx = imageio.get_writer(out_dash_path, **dash_writer_kw)

    with imageio.get_writer(out_plot_path, fps=FPS, macro_block_size=1, quality=10, output_params=['-crf', '14']) as plot_writer, \
         dash_writer_ctx as dash_writer:
        ptr = 0
        for k in range(n_frames):
            t_now = k / FPS
            while ptr < len(times) and times[ptr] <= t_now:
                ptr += 1
            cur_speed = float(speeds[ptr - 1]) if ptr > 0 else 0.0

            # --- line-plot frame (matplotlib blit) ---
            line_done.set_data(times[:ptr], speeds[:ptr])
            line_ahead.set_data(times[ptr - 1:], speeds[ptr - 1:])
            vline.set_xdata([t_now, t_now])
            fig.canvas.restore_region(bg)
            ax.draw_artist(line_ahead)
            ax.draw_artist(line_done)
            ax.draw_artist(vline)
            fig.canvas.blit(fig.bbox)
            plot_buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
            plot_writer.append_data(plot_buf)

            # --- dashboard frame (PIL) ---
            dash_buf = render_dashboard_frame(cur_speed, bar_max, fonts)
            dash_writer.append_data(dash_buf)

    plt.close(fig)
    return f"{log_name}: {n_frames} frames → {out_plot_path}, {out_dash_path}"


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
