"""
vicon_noise_tracker.py
======================
Track a single stationary Vicon rigidbody object for a given duration and
characterise its noise.

  * GLOBAL noise  → positional variation across frames.
                    The object is stationary, so every deviation from the
                    mean position is measurement noise.
                    Reported as: per-axis std and 3-D RMSE about the mean.

  * LOCAL noise   → Vicon's per-frame subject quality value.
                    This is the RMS marker-fitting residual in MILLIMETRES:
                    how far each observed marker deviates from its expected
                    position in the stored rigid-body template after the
                    least-squares fit.  LOWER IS BETTER.  Typical range:
                      < 1 mm   → excellent
                      1–3 mm   → good / acceptable
                      > 5 mm   → poor (occlusion, reflections, bad calib.)
                    There is NO upper bound; values > 3 are normal.

Usage
-----
    python vicon_noise_tracker.py --subject CF1 --duration 30
    python vicon_noise_tracker.py --subject CF1 --duration 60 --out noise_log.json
"""

import argparse
import json
import logging
import time
import sys
import numpy as np

from pyvicon_datastream import PyViconDatastream, StreamMode, Direction

# ── logging ──────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("vicon_noise")

VICON_PC_IP   = "192.168.1.39"
VICON_ADDRESS = f"{VICON_PC_IP}:801"


# ── helpers ───────────────────────────────────────────────────────────────────

def rmse_about_mean(data: np.ndarray) -> float:
    """
    3-D positional RMSE relative to the sample mean.
    data : (N, 3) array of (x, y, z) positions in metres.
    Returns scalar RMSE in metres.
    """
    if len(data) < 2:
        return float("nan")
    centroid = data.mean(axis=0)          # shape (3,)
    residuals = data - centroid            # shape (N, 3)
    sq_dist   = np.sum(residuals ** 2, axis=1)   # ||r_i||²
    return float(np.sqrt(sq_dist.mean()))


def per_axis_std(data: np.ndarray):
    """Return (std_x, std_y, std_z) in metres."""
    return tuple(data.std(axis=0).tolist())


# ── callable entry-point for orchestrator ────────────────────────────────────

def run_tracker(subject: str, out_path: str, stop_event=None):
    """
    Connect to Vicon, stream frames for `subject` until `stop_event` is set
    (or KeyboardInterrupt), then compute and save noise stats to `out_path`.

    Parameters
    ----------
    subject    : Vicon rigidbody name (case-sensitive)
    out_path   : full path for the output JSON file
    stop_event : threading.Event — when set the loop exits gracefully.
                 Pass None to run until KeyboardInterrupt only.
    """
    _log = logging.getLogger("vicon_noise")
    client = PyViconDatastream()

    _log.info(f"[NoiseTracker] Connecting to {VICON_ADDRESS} …")
    client.connect(VICON_ADDRESS)
    if not client.is_connected():
        _log.error("[NoiseTracker] Failed to connect — tracker will not run.")
        return

    client.enable_segment_data()
    client.enable_marker_data()
    client.set_stream_mode(StreamMode.ServerPush)
    client.set_axis_mapping(Direction.Forward, Direction.Left, Direction.Up)
    _log.info(f"[NoiseTracker] Tracking '{subject}' until stop signal …")

    records = []
    frame_count = 0
    t_start = time.time()

    try:
        while True:
            if stop_event is not None and stop_event.is_set():
                break
            if not client.get_frame():
                continue
            frame_count += 1

            n_subjects = client.get_subject_count()
            for i in range(n_subjects):
                name = client.get_subject_name(i)
                if name != subject:
                    continue

                root_seg = client.get_subject_root_segment_name(name)
                tvec     = client.get_segment_global_translation(name, root_seg)
                try:
                    quality = float(client.get_subject_quality(name))
                except Exception:
                    quality = float("nan")

                pos_m = [tvec[0] / 1000.0, tvec[1] / 1000.0, tvec[2] / 1000.0]
                records.append({"time": time.time(), "pos": pos_m, "marker_residual_mm": quality})

                _log.debug(
                    f"[NoiseTracker] t={time.time()-t_start:.1f}s  "
                    f"pos=({pos_m[0]:+.4f}, {pos_m[1]:+.4f}, {pos_m[2]:+.4f}) m  "
                    f"residual={quality:.4f} mm"
                )
                break

    except KeyboardInterrupt:
        _log.info("[NoiseTracker] Interrupted.")
    finally:
        client.disconnect()
        _log.info("[NoiseTracker] Disconnected.")

    if not records:
        _log.warning("[NoiseTracker] No data collected — skipping save.")
        return

    # ── analysis ─────────────────────────────────────────────────────────────
    positions = np.array([r["pos"]               for r in records])
    qualities = np.array([r["marker_residual_mm"] for r in records])
    N = len(records)
    duration = records[-1]["time"] - records[0]["time"]

    centroid           = positions.mean(axis=0)
    std_x, std_y, std_z = per_axis_std(positions)
    rmse_3d            = rmse_about_mean(positions)
    valid_q            = qualities[~np.isnan(qualities)]

    # ── Packet inter-arrival time ─────────────────────────────────────────────
    timestamps_s  = np.array([r["time"] for r in records])
    iat_ms        = np.diff(timestamps_s) * 1000.0   # ms
    iat_mean      = float(iat_ms.mean())   if len(iat_ms) > 0 else float("nan")
    iat_max       = float(iat_ms.max())    if len(iat_ms) > 0 else float("nan")
    iat_min       = float(iat_ms.min())    if len(iat_ms) > 0 else float("nan")
    iat_median    = float(np.median(iat_ms)) if len(iat_ms) > 0 else float("nan")

    _log.info(f"[NoiseTracker] {N} frames over {duration:.1f}s  |  "
              f"Global RMSE: {rmse_3d*1000:.3f} mm  |  "
              + (f"Marker residual mean: {valid_q.mean():.3f} mm" if len(valid_q) else ""))
    _log.info(f"[NoiseTracker] Packet inter-arrival time (ms) — "
              f"Mean: {iat_mean:.3f}  Max: {iat_max:.3f}  "
              f"Min: {iat_min:.3f}  Median: {iat_median:.3f}")

    output = {
        "subject":     subject,
        "frame_count": N,
        "duration_s":  duration,
        "global_noise": {
            "centroid_m": centroid.tolist(),
            "std_x_m":    std_x,
            "std_y_m":    std_y,
            "std_z_m":    std_z,
            "rmse_3d_m":  rmse_3d,
            "rmse_3d_mm": rmse_3d * 1000,
        },
        "local_noise_marker_residual_mm": {
            "description": "RMS marker fitting residual in mm (lower=better, no upper bound)",
            "mean":   float(np.nanmean(qualities)),
            "std":    float(np.nanstd(qualities)),
            "median": float(np.nanmedian(qualities)),
            "min":    float(np.nanmin(qualities)),
            "max":    float(np.nanmax(qualities)),
        },
        "packet_inter_arrival_time_ms": {
            "mean":   iat_mean,
            "max":    iat_max,
            "min":    iat_min,
            "median": iat_median,
        },
        "frames": records,
    }

    import os
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(output, f, indent=2)
    _log.info(f"[NoiseTracker] Log saved → {out_path}")



# ── standalone CLI ────────────────────────────────────────────────────────────

def main():
    import threading
    ap = argparse.ArgumentParser(description="Vicon stationary-noise tracker")
    ap.add_argument("--subject",  "-s", required=True,
                    help="Vicon rigidbody name to track (case-sensitive)")
    ap.add_argument("--duration", "-d", type=float, default=30.0,
                    help="Recording duration in seconds (default: 30)")
    ap.add_argument("--out", "-o", default="",
                    help="Output JSON file path")
    args = ap.parse_args()

    stop = threading.Event()

    def _stopper():
        time.sleep(args.duration)
        stop.set()

    threading.Thread(target=_stopper, daemon=True).start()
    out = args.out or f"vicon_noise_{args.subject}_{int(time.time())}.json"
    run_tracker(args.subject, out, stop_event=stop)


if __name__ == "__main__":
    main()

