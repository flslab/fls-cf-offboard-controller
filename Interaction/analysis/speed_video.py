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

DASH_W, DASH_H = 420, 720

IMG_PATH   = ''   # path to image to display; leave empty for placeholder

_PANEL_PAD = 10
_PANEL_R   = 14

# Bar is a plain rectangle — no rounded corners, 60 px wide
_BAR_W  = 60
_VBAR_Y0 = _PANEL_PAD + 16            # = 26
_VBAR_Y1 = DASH_H - _PANEL_PAD - 16   # = 694
_VBAR_H  = _VBAR_Y1 - _VBAR_Y0        # = 668

if not FLIP_LAYOUT:
    # Bar right, content left — ticks to the LEFT of bar
    _VBAR_X0  = DASH_W - _PANEL_PAD - 18 - _BAR_W  # 332
    _VBAR_X1  = _VBAR_X0 + _BAR_W                   # 392
    _L_X0     = _PANEL_PAD + 10                      # 20
    _L_X1     = _VBAR_X0 - 14                        # 318
    _TICK_X1  = _VBAR_X0 - 4                         # 328
    _TICK_X0  = _VBAR_X0 - 14                        # 318
    _LABEL_X  = _VBAR_X0 - 18                        # 314 (right-align labels here)
else:
    # Bar left, content right — ticks to the RIGHT of bar
    _VBAR_X0  = _PANEL_PAD + 18                      # 28
    _VBAR_X1  = _VBAR_X0 + _BAR_W                   # 88
    _L_X0     = _VBAR_X1 + 90                        # 178
    _L_X1     = DASH_W - _PANEL_PAD - 10             # 400
    _TICK_X0  = _VBAR_X1 + 4                         # 92
    _TICK_X1  = _VBAR_X1 + 14                        # 102
    _LABEL_X  = _VBAR_X1 + 18                        # 106 (left-align labels here)

_IMG_X0   = _L_X0
_IMG_Y0   = 74
_IMG_SIZE = min(_L_X1 - _L_X0, 200)


_THRESHOLD_COLOR = (30, 120, 255)   # blue threshold line and tag

# Orange → red → magenta → purple → navy
_COLOR_STOPS = [
    (0.00, (255, 140,   0)),
    (0.28, (235,  35,  20)),
    (0.52, (205,  20, 118)),
    (0.76, (132,  15, 195)),
    (1.00, ( 22,  22, 108)),
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


def _lerp_color(f, lit=True, div=6):
    f = max(0.0, min(1.0, f))
    for j in range(len(_COLOR_STOPS) - 1):
        f0, c0 = _COLOR_STOPS[j]
        f1, c1 = _COLOR_STOPS[j + 1]
        if f <= f1:
            t   = (f - f0) / (f1 - f0 + 1e-9)
            col = tuple(int(c0[k] + t * (c1[k] - c0[k])) for k in range(3))
            return col if lit else tuple(max(6, c // div) for c in col)
    c = _COLOR_STOPS[-1][1]
    return c if lit else tuple(max(6, c // div) for c in c)


def render_dashboard_frame(cur_speed, max_speed, fonts, panel_img=None):
    """Vertical futuristic dashboard — translucent dark panel, rectangular bar."""
    THRESHOLD = 100.0
    frac     = min(cur_speed, max_speed) / max_speed
    frac_thr = THRESHOLD / max_speed
    fill_y   = int(_VBAR_Y1 - frac     * _VBAR_H)
    thr_y    = int(_VBAR_Y1 - frac_thr * _VBAR_H)

    th  = _THEMES[DASH_MODE]
    tc  = _THRESHOLD_COLOR
    img = Image.new('RGBA', (DASH_W, DASH_H), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)

    f_large, f_unit, f_scale, f_title, f_thr = fonts

    if DASH_MODE == 'dark':
        dim_col    = (70,  120, 170, 200)
        bright_col = (200, 228, 255, 255)
        tick_col   = (50,  100, 160, 190)
    else:
        dim_col    = (50,  65,  100, 220)
        bright_col = (10,  20,   60, 255)
        tick_col   = (30,  60,  110, 200)
    warn_col = (220, 60, 40, 255)
    spd_col  = warn_col if cur_speed > THRESHOLD else bright_col

    # ── Translucent panel ─────────────────────────────────────────────────────
    p0, p1 = _PANEL_PAD, DASH_W - _PANEL_PAD
    q0, q1 = _PANEL_PAD, DASH_H - _PANEL_PAD
    draw.rounded_rectangle([p0, q0, p1, q1], radius=_PANEL_R,
                           fill=th['bg'] + (185,))
    # Neon border
    draw.rounded_rectangle([p0, q0, p1, q1], radius=_PANEL_R,
                           outline=(0, 140, 220, 190), width=1)
    draw.rounded_rectangle([p0+2, q0+2, p1-2, q1-2], radius=max(_PANEL_R-2, 0),
                           outline=(0, 70, 140, 90), width=1)

    # Corner L-brackets
    bs, bw = 14, 2
    for cx, cy, sx, sy in [(p0, q0, 1, 1), (p1, q0, -1, 1),
                            (p0, q1, 1, -1), (p1, q1, -1, -1)]:
        ax, bx = cx, cx+sx*bs; ay, by = cy, cy+sy*bw
        draw.rectangle([min(ax,bx), min(ay,by), max(ax,bx), max(ay,by)], fill=(0, 200, 255, 220))
        ax, bx = cx, cx+sx*bw; ay, by = cy, cy+sy*bs
        draw.rectangle([min(ax,bx), min(ay,by), max(ax,bx), max(ay,by)], fill=(0, 200, 255, 220))

    # ── Title ─────────────────────────────────────────────────────────────────
    ty0 = _VBAR_Y0 + 2
    draw.text((_L_X0, ty0), 'Speed of', fill=dim_col, font=f_title)

    # ── Image / placeholder ───────────────────────────────────────────────────
    box = [_IMG_X0, _IMG_Y0, _IMG_X0+_IMG_SIZE, _IMG_Y0+_IMG_SIZE]
    if panel_img is not None:
        img.paste(panel_img.convert('RGBA'), (_IMG_X0, _IMG_Y0))
    else:
        draw.rectangle(box, fill=(8, 12, 28, 200))
        draw.rectangle(box, outline=(0, 80, 140, 160), width=1)
        for cx, cy, sx, sy in [(box[0], box[1], 1, 1), (box[2], box[1], -1, 1),
                                (box[0], box[3], 1, -1), (box[2], box[3], -1, -1)]:
            s = 10
            ax, bx = cx, cx+sx*s; ay, by = cy, cy+sy*2
            draw.rectangle([min(ax,bx), min(ay,by), max(ax,bx), max(ay,by)], fill=(0, 160, 200, 180))
            ax, bx = cx, cx+sx*2; ay, by = cy, cy+sy*s
            draw.rectangle([min(ax,bx), min(ay,by), max(ax,bx), max(ay,by)], fill=(0, 160, 200, 180))

    # ── Speed number (left of panel, vertically centered) ─────────────────────
    spd_str = f'{cur_speed:.0f}'
    bb_s = draw.textbbox((0, 0), spd_str, font=f_large)
    bb_u = draw.textbbox((0, 0), 'mm/s', font=f_unit)
    sw, sh = bb_s[2]-bb_s[0], bb_s[3]-bb_s[1]
    uw, uh = bb_u[2]-bb_u[0], bb_u[3]-bb_u[1]
    total_spd_h = sh + 6 + uh
    spd_y  = DASH_H // 2 - total_spd_h // 2
    unit_y = spd_y + sh + 6
    mid_x  = _L_X0 + (_L_X1 - _L_X0) // 2
    draw.line([(_L_X0, spd_y - 10), (_L_X1, spd_y - 10)],
              fill=(0, 80, 140, 110), width=1)
    draw.text((mid_x - sw//2, spd_y),   spd_str, fill=spd_col,  font=f_large)
    draw.text((mid_x - uw//2, unit_y), 'mm/s',   fill=dim_col, font=f_unit)

    # ── Rectangular bar — border then row fill ────────────────────────────────
    for exp, col in [
        (4, (0, 60, 120, 255)),
        (2, (0, 35, 85, 255)),
        (0, th['trough'] + (255,)),
    ]:
        draw.rectangle([_VBAR_X0-exp, _VBAR_Y0-exp, _VBAR_X1+exp, _VBAR_Y1+exp],
                       fill=col)

    for y in range(_VBAR_Y0, _VBAR_Y1+1):
        fy  = (_VBAR_Y1 - y) / (_VBAR_H + 1e-9)
        col = _lerp_color(fy, True) + (255,) if y >= fill_y else th['trough'] + (255,)
        draw.line([(_VBAR_X0, y), (_VBAR_X1, y)], fill=col)

    # Fill-edge glow
    if _VBAR_Y0 < fill_y < _VBAR_Y1:
        fy = (_VBAR_Y1 - fill_y) / (_VBAR_H + 1e-9)
        ec = _lerp_color(fy, True)
        for w, a in [(6, 60), (3, 140), (1, 255)]:
            c = tuple(min(255, v + 90) for v in ec) + (a,)
            draw.line([(_VBAR_X0, fill_y), (_VBAR_X1, fill_y)], fill=c, width=w)

    # Right-edge highlight on lit region
    for y in range(max(fill_y, _VBAR_Y0), _VBAR_Y1+1):
        fy  = (_VBAR_Y1 - y) / (_VBAR_H + 1e-9)
        col = _lerp_color(fy, True)
        draw.point((_VBAR_X1, y),
                   fill=tuple(min(255, c+80) for c in col) + (180,))

    # ── Scale ticks & labels (left or right of bar, per FLIP_LAYOUT) ─────────
    tick_step = 50 if max_speed <= 350 else 100
    v = 0.0
    while v <= max_speed + 0.1:
        ty_t = int(_VBAR_Y1 - (v / max_speed) * _VBAR_H)
        is_t = abs(v - THRESHOLD) < 0.1
        t_col = tc + (255,) if is_t else tick_col
        draw.line([(_TICK_X0, ty_t), (_TICK_X1, ty_t)],
                  fill=t_col, width=2 if is_t else 1)

        lbl = f'{int(v)}'
        bb  = draw.textbbox((0, 0), lbl, font=f_scale)
        lw, lh = bb[2]-bb[0], bb[3]-bb[1]
        tx = (_LABEL_X - lw) if not FLIP_LAYOUT else _LABEL_X

        if is_t:
            thr_lbl = 'S_D=S_Q=S_H=100'
            bb2 = draw.textbbox((0, 0), thr_lbl, font=f_thr)
            lw2, lh2 = bb2[2]-bb2[0], bb2[3]-bb2[1]
            tx2 = (_LABEL_X - lw2) if not FLIP_LAYOUT else _LABEL_X
            draw.text((tx2, ty_t - lh2//2), thr_lbl,
                      fill=(80, 160, 255, 200), font=f_thr)
        else:
            draw.text((tx, ty_t - lh//2), lbl, fill=dim_col, font=f_scale)

        v += tick_step

    # ── Threshold glow line (within bar) ──────────────────────────────────────
    for w, a in [(8, 50), (4, 130), (2, 210), (1, 255)]:
        draw.line([(_VBAR_X0, thr_y), (_VBAR_X1, thr_y)],
                  fill=tc + (a,), width=w)

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

    out_plot_path = os.path.join(out_dir, f"{log_name}.mp4")
    dash_ext      = 'webm' if DASH_TRANSPARENT else 'mp4'
    out_dash_path = os.path.join(out_dir, f"{log_name}_dashboard.{dash_ext}")
    panel_img = None
    if IMG_PATH:
        try:
            panel_img = Image.open(IMG_PATH).convert('RGB').resize(
                (_IMG_SIZE, _IMG_SIZE), Image.LANCZOS)
        except Exception:
            pass
    # f_large, f_unit, f_scale, f_title, f_thr
    fonts = (_load_font(80), _load_font(26), _load_font(26), _load_font(26), _load_font(26))

    dash_writer_kw = dict(fps=FPS, macro_block_size=1)
    if DASH_TRANSPARENT:
        dash_writer_kw['codec'] = 'libvpx-vp9'
        dash_writer_kw['output_params'] = ['-pix_fmt', 'yuva420p', '-auto-alt-ref', '0']

    with imageio.get_writer(out_plot_path, fps=FPS, macro_block_size=1) as plot_writer, \
         imageio.get_writer(out_dash_path, **dash_writer_kw) as dash_writer:
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
            dash_buf = render_dashboard_frame(cur_speed, bar_max, fonts, panel_img)
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
