#!/usr/bin/env python3
"""
drone_tracker.py – Track a drone in a video and place a WebM overlay next to it.

Two modes:
  manual   – draw the initial box yourself (default)
  auto     – use background subtraction to detect the moving drone

Keys during preview:
    q / ESC  – quit
    p        – pause / resume
    r        – restart tracking (manual: redraw box; auto: reset background)

Dependencies:
    pip install opencv-python
"""

import subprocess
import sys
import cv2
import numpy as np


INPUT_VIDEO   = '../../logs/SIGGRAPH_Poster/lit/lit_with_lb4.mp4'
OUTPUT_VIDEO  = '../../logs/SIGGRAPH_Poster/Emoji_Speed_Overlay.mp4'   # e.g. "tracked.mp4"
OVERLAY_VIDEO = "../../logs/SIGGRAPH_Poster/lit/lb5_lit.mov"
MODE          = "manual"   # "manual" | "auto"
START_TIME_S  = 19.05       # Time in seconds to start reading the video

OVERLAY_GAP = 10   # pixels between bbox edge and overlay

BOX_COLOR   = (0, 255, 80)
LABEL_COLOR = (0, 255, 80)
FAIL_COLOR  = (0, 80, 255)
BOX_THICK   = 2
FONT        = cv2.FONT_HERSHEY_SIMPLEX


# ── Trackers ──────────────────────────────────────────────────────────────────

def make_tracker():
    for name in ('TrackerCSRT_create', 'TrackerKCF_create', 'TrackerMIL_create'):
        fn = getattr(cv2, name, None)
        if fn:
            return fn()
    raise RuntimeError("No OpenCV tracker found. Install opencv-contrib-python.")


def select_roi(frame):
    print("Draw a box around the drone, then press ENTER or SPACE. Press C to cancel.")
    roi = cv2.selectROI("Select drone", frame, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow("Select drone")
    if roi[2] == 0 or roi[3] == 0:
        return None
    return roi


def detect_drone_auto(fgmask, min_area=200, max_area=40_000):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask   = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask,   cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in contours:
        area = cv2.contourArea(c)
        if min_area < area < max_area:
            if best is None or area > cv2.contourArea(best):
                best = c
    return cv2.boundingRect(best) if best is not None else None


# ── Overlay helpers ───────────────────────────────────────────────────────────

class OverlayVideo:
    """One-shot overlay reader with alpha support via ffmpeg pipe (bgra output).

    OpenCV's VideoCapture cannot reliably decode WebM alpha on macOS.
    Piping through ffmpeg with -pix_fmt bgra is the only robust path.
    Returns None once the clip is exhausted — no looping.
    Requires ffmpeg in PATH (e.g. `brew install ffmpeg`).
    """
    def __init__(self, path):
        # Probe dimensions and pixel format
        probe = subprocess.run(
            ['ffprobe', '-v', 'error', '-select_streams', 'v:0',
             '-show_entries', 'stream=width,height,pix_fmt', '-of', 'csv=s=x:p=0', path],
            capture_output=True, text=True, check=True,
        )
        parts = probe.stdout.strip().split('x')
        w, h, pix_fmt = int(parts[0]), int(parts[1]), parts[2]
        self._w, self._h = w, h
        self._frame_bytes = w * h * 4  # BGRA

        # ffmpeg always outputs straight alpha when decoding to bgra,
        # regardless of how the source was encoded (VP9 yuva420p, etc.)
        self._premultiplied = False
        print(f"Overlay: {w}x{h}  pix_fmt={pix_fmt} (decoded as straight alpha)")

        self._path = path
        self._proc = self._open_pipe()
        self._done = False

        # Read first frame for the size-picker preview, then reopen
        self._first_frame = self._read_one()
        if self._first_frame is not None:
            a = self._first_frame[:, :, 3]
            print(f"Overlay alpha channel — min={a.min()}  max={a.max()}  "
                  f"{'(no transparency detected — check WebM export settings)' if a.min() == 255 else 'OK'}")
        self._proc.stdout.close()
        self._proc.wait()
        self._proc = self._open_pipe()

    def _open_pipe(self):
        return subprocess.Popen(
            ['ffmpeg', '-i', self._path, '-f', 'rawvideo',
             '-pix_fmt', 'bgra', '-loglevel', 'error', 'pipe:1'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        )

    def _read_one(self):
        raw = self._proc.stdout.read(self._frame_bytes)
        if len(raw) < self._frame_bytes:
            return None
        return np.frombuffer(raw, dtype=np.uint8).reshape(self._h, self._w, 4).copy()

    def peek_first(self):
        return self._first_frame

    def next_frame(self):
        if self._done:
            return None
        frame = self._read_one()
        if frame is None:
            self._done = True
        return frame

    def release(self):
        if self._proc.poll() is None:
            self._proc.stdout.close()
            self._proc.terminate()
            self._proc.wait()


SIDES = ['Right', 'Left', 'Above', 'Below']   # index matches trackbar value


def place_overlay(canvas, overlay_frame, bbox, frame_w, frame_h, size, side=0,
                  premultiplied=False):
    """Paste overlay_frame (resized to `size`) next to bbox.

    side: 0=Right, 1=Left, 2=Above, 3=Below
    premultiplied: always False — ffmpeg decodes to straight alpha in bgra
    """
    ow, oh = size
    resized = cv2.resize(overlay_frame, (ow, oh), interpolation=cv2.INTER_AREA)

    bx, by, bw, bh = bbox

    if side == 0:   # Right
        ox = bx + bw + OVERLAY_GAP
        oy = by + bh - oh
    elif side == 1:   # Left
        ox = bx - OVERLAY_GAP - ow
        oy = by + bh - oh
    elif side == 2:   # Above
        ox = bx + (bw - ow) // 2
        oy = by - OVERLAY_GAP - oh
    else:             # Below
        ox = bx + (bw - ow) // 2
        oy = by + bh + OVERLAY_GAP

    ox = max(0, min(ox, frame_w - ow))
    oy = max(0, min(oy, frame_h - oh))

    if resized.ndim == 3 and resized.shape[2] == 4:
        alpha = resized[:, :, 3:4].astype(float) / 255.0
        rgb   = resized[:, :, :3].astype(float)
        roi   = canvas[oy:oy+oh, ox:ox+ow].astype(float)
        if premultiplied:
            # rgb already contains alpha*color; just add the background contribution
            blended = rgb + (1.0 - alpha) * roi
        else:
            blended = alpha * rgb + (1.0 - alpha) * roi
        canvas[oy:oy+oh, ox:ox+ow] = np.clip(blended, 0, 255).astype(np.uint8)
    else:
        canvas[oy:oy+oh, ox:ox+ow] = resized

    return canvas


def pick_overlay_config(frame, bbox, overlay_vid):
    """Interactive size + position picker shown on the first frame.

    Trackbars:
      Height  – overlay height in pixels (width scales to preserve aspect ratio)
      Position – 0 Right | 1 Left | 2 Above | 3 Below

    Press ENTER or SPACE to confirm.
    Returns (w, h, side).
    """
    ov_frame = overlay_vid.peek_first()
    if ov_frame is None:
        bx, by, bw, bh = bbox
        return bh, bh, 0

    aspect = ov_frame.shape[1] / ov_frame.shape[0]
    H, W   = frame.shape[:2]
    bx, by, bw, bh = bbox

    win = "Adjust overlay  –  ENTER to confirm"
    cv2.namedWindow(win)
    cv2.createTrackbar("Height",   win, max(bh, 20), H,        lambda v: None)
    cv2.createTrackbar("Position", win, 0,           len(SIDES) - 1, lambda v: None)

    print(f"Height: size slider  |  Position: {', '.join(f'{i}={s}' for i, s in enumerate(SIDES))}")
    print("Press ENTER or SPACE to confirm.")

    oh, ow, side = bh, bh, 0
    while True:
        oh   = max(cv2.getTrackbarPos("Height",   win), 10)
        side = cv2.getTrackbarPos("Position", win)
        ow   = max(int(oh * aspect), 1)

        preview = frame.copy()
        place_overlay(preview, ov_frame, bbox, W, H, (ow, oh), side,
                      premultiplied=overlay_vid._premultiplied)
        label = f"Size: {ow}x{oh}  |  Position: {SIDES[side]}  |  ENTER to confirm"
        cv2.putText(preview, label, (10, H - 10), FONT, 0.55, (200, 200, 200), 1)
        cv2.imshow(win, preview)

        key = cv2.waitKey(50) & 0xFF
        if key in (13, 32):
            break
        if key in (ord('q'), 27):
            break

    cv2.destroyWindow(win)
    print(f"Overlay config: {ow}x{oh}, {SIDES[side]}")
    return ow, oh, side


# ── Main ──────────────────────────────────────────────────────────────────────

def run(input_path, output_path, mode, overlay_path, start_time_s=0.0):
    cap = cv2.VideoCapture(input_path)
    if not cap.isOpened():
        sys.exit(f"Cannot open video: {input_path}")

    fps   = cap.get(cv2.CAP_PROP_FPS) or 30.0
    W     = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H     = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    start_frame = int(start_time_s * fps) if start_time_s > 0 else 0

    writer = None
    if output_path:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(output_path, fourcc, fps, (W, H))
        print(f"Writing to {output_path}")

    overlay_vid = OverlayVideo(overlay_path) if overlay_path else None

    ret, frame = cap.read()
    if not ret:
        sys.exit("Empty video.")

    tracker   = None
    tracking  = False
    fgbg      = cv2.createBackgroundSubtractorMOG2(history=200, varThreshold=50, detectShadows=False)
    frame_idx = 0
    paused    = False
    overlay_size = None   # set once after first drone selection

    def init_tracker(fr):
        nonlocal tracker, tracking, overlay_size
        roi = select_roi(fr)
        if roi is None:
            return
        tracker = make_tracker()
        tracker.init(fr, roi)
        tracking = True
        print(f"Tracker initialised at {roi}")
        if overlay_vid is not None and overlay_size is None:
            overlay_size = pick_overlay_config(fr, roi, overlay_vid)

    if mode == "manual" and start_frame == 0:
        init_tracker(frame)

    while True:
        if not paused:
            if frame_idx > 0:
                ret, frame = cap.read()
                if not ret:
                    break

            if frame_idx == start_frame and start_frame > 0:
                print(f"Reached start time {start_time_s}s. Initialising tracking...")
                if mode == "manual":
                    init_tracker(frame)
                else:
                    fgbg = cv2.createBackgroundSubtractorMOG2(history=200, varThreshold=50, detectShadows=False)

            frame_idx += 1
            display = frame.copy()
            bbox = None

            if frame_idx >= start_frame:
                if mode == "auto":
                    fgmask = fgbg.apply(frame)
                    roi    = detect_drone_auto(fgmask)
                    if roi:
                        bbox = roi
                else:
                    if tracking:
                        ok, raw_bbox = tracker.update(frame)
                        if ok:
                            x, y, w, h = (int(v) for v in raw_bbox)
                            bbox = (x, y, w, h)
                        else:
                            tracking = False
                            cv2.putText(display, "Tracking lost – press R to retry",
                                        (20, 40), FONT, 0.7, FAIL_COLOR, 2)

            if overlay_vid is not None and bbox is not None and overlay_size is not None:
                ov_frame = overlay_vid.next_frame()
                if ov_frame is not None:
                    ow, oh, side = overlay_size
                    display = place_overlay(display, ov_frame, bbox, W, H, (ow, oh), side,
                                            premultiplied=overlay_vid._premultiplied)

            if writer:
                writer.write(display)

            cv2.imshow("Drone Tracker", display)

        key = cv2.waitKey(1 if not paused else 50) & 0xFF
        if key in (ord('q'), 27):
            break
        if key == ord('p'):
            paused = not paused
        if key == ord('r'):
            if mode == "manual":
                overlay_size = None
                init_tracker(frame)
            else:
                fgbg = cv2.createBackgroundSubtractorMOG2(history=200, varThreshold=50, detectShadows=False)
                print("Background model reset.")

    cap.release()
    if overlay_vid:
        overlay_vid.release()
    if writer:
        writer.release()
        print("Done.")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    run(INPUT_VIDEO, OUTPUT_VIDEO, MODE, OVERLAY_VIDEO, START_TIME_S)
