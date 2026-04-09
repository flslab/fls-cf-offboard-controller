"""
Analyze drone telemetry logs from lateral force rendering user study.
Computes kinetic metrics per interaction instance and outputs a CSV.

Interaction Detection:
  - Start: first "User Pushing" event after a quiet gap
  - End:   corresponding "User Disengage" event
  - Noise: very short duration or very few push events
  - Missing: expected interactions not detected (based on filename)

Metrics (per interaction):
  - Duration (s)
  - Impulse/mass J/m = Δv  (peak_speed - baseline_speed)
  - Peak-to-Average speed ratio
  - Rise Time (s): start → first time speed ≥ 90% of peak
  - Settling Time (s): disengage → last time speed exceeds ±5% band above baseline
"""

import json
import math
import os
import re
import csv
import numpy as np
from collections import Counter

LOG_DIR = os.path.join(os.path.dirname(__file__), "../..", "logs", "User_Studies")
OUTPUT_CSV = os.path.join(os.path.dirname(__file__), "../..", "logs", "User_Studies", "interaction_metrics.csv")

# ─── noise thresholds ────────────────────────────────────────────────────────
MIN_DURATION_S = 0.15   # interactions shorter than this are noise
MIN_PUSH_EVENTS = 5     # interactions with fewer push events are noise
# Combined: if both conditions hold, also treat as noise (faint brief contact)
NOISE_COMBINED_MAX_SPEED = 0.15  # m/s
NOISE_COMBINED_MAX_DURATION = 0.30  # s

# ─── settling window ─────────────────────────────────────────────────────────
SETTLE_SEARCH_WINDOW_S = 10.0   # how long after disengage to look for settling
SETTLE_FRACTION = 0.05          # ±5 % of peak excursion above baseline

# ─── baseline window ─────────────────────────────────────────────────────────
BASELINE_WINDOW_S = 1.0         # seconds before push start to average


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def load_log(path: str) -> list[dict]:
    """
    Load a log file that is either a well-formed JSON array or a line-delimited
    JSON array whose closing ] may be missing / truncated.
    """
    with open(path, encoding="utf-8") as fh:
        raw = fh.read().strip()

    # Try normal JSON first
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        pass

    # Fall back: strip leading "[" and trailing "]" (if present), then parse
    # each comma-separated object line by line.
    if raw.startswith("["):
        raw = raw[1:]
    if raw.endswith("]"):
        raw = raw[:-1]

    records = []
    for line in raw.splitlines():
        line = line.strip().rstrip(",")
        if not line:
            continue
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError:
            continue          # skip genuinely corrupt lines
    return records


def parse_filename(filename: str) -> tuple[int, list[str]]:
    """
    Return (user_id, [ordered interaction types]).
    Handles names like:
        us_5_3poke_2push_3flick.json
        us_9_3_flick.json          ← underscore before type
    """
    base = os.path.splitext(filename)[0]
    parts = base.split("_")
    user_id = int(parts[1])

    sequence: list[str] = []
    # reassemble the tail (in case of "3_flick" split into ["3","flick"])
    tail = "_".join(parts[2:])
    for m in re.finditer(r"(\d+)_?(poke|push|flick)", tail):
        count = int(m.group(1))
        itype = m.group(2)
        sequence.extend([itype] * count)

    return user_id, sequence


def segment_interactions(data: list[dict]) -> list[dict]:
    """
    Build raw interaction segments: groups of consecutive "User Pushing"
    events followed by exactly one "User Disengage" event.
    """
    interactions = []
    current_start = None
    current_events: list[dict] = []

    for entry in data:
        if entry.get("type") != "events":
            continue
        name = entry["name"]
        t = entry["data"]["time"]

        if name == "User Pushing":
            if current_start is None:
                current_start = t
            current_events.append(entry)

        elif name == "User Disengage":
            if current_start is not None:
                interactions.append(
                    {
                        "start": current_start,
                        "end": t,
                        "push_events": current_events,
                        "disengage": entry,
                    }
                )
            current_start = None
            current_events = []

    return interactions


def is_noisy(seg: dict) -> bool:
    duration = seg["end"] - seg["start"]
    n = len(seg["push_events"])
    if duration < MIN_DURATION_S or n < MIN_PUSH_EVENTS:
        return True
    # Faint brief contact: low max speed AND short duration
    max_speed = max(e["data"]["speed"] for e in seg["push_events"])
    if max_speed < NOISE_COMBINED_MAX_SPEED and duration < NOISE_COMBINED_MAX_DURATION:
        return True
    return False


def get_frames(data: list[dict], t_lo: float, t_hi: float) -> list[dict]:
    """Return frames entries within [t_lo, t_hi] with speed pre-computed."""
    result = []
    for entry in data:
        if entry.get("type") != "frames":
            continue
        t = entry["data"]["time"]
        if t_lo <= t <= t_hi:
            vel = entry["data"]["vel"]
            speed = math.sqrt(sum(v * v for v in vel))
            result.append({"time": t, "vel": vel, "speed": speed})
    return result


# ──────────────────────────────────────────────────────────────────────────────
# Metric computation
# ──────────────────────────────────────────────────────────────────────────────

def compute_metrics(seg: dict, data: list[dict]) -> dict | None:
    """
    Compute all kinetic metrics for one interaction segment.
    Returns None if there is insufficient frames data.
    """
    t_start = seg["start"]
    t_end = seg["end"]

    frames_pre = get_frames(data, t_start - BASELINE_WINDOW_S, t_start)
    frames_during = get_frames(data, t_start, t_end)
    frames_post = get_frames(data, t_end, t_end + SETTLE_SEARCH_WINDOW_S)

    if not frames_during:
        return None

    # ── baseline speed ──────────────────────────────────────────────────────
    baseline_speed = (
        float(np.mean([f["speed"] for f in frames_pre])) if frames_pre else 0.0
    )

    # ── speed statistics during interaction ─────────────────────────────────
    speeds_during = [f["speed"] for f in frames_during]
    peak_speed = max(speeds_during)
    min_speed  = min(speeds_during)
    avg_speed  = float(np.mean(speeds_during))
    std_speed  = float(np.std(speeds_during))
    excursion  = peak_speed - baseline_speed  # amplitude above baseline

    # ── J/m: accumulated impulse per unit mass ──────────────────────────────
    # J/m = ∫|a| dt ≈ Σ ||v(t_{i+1}) - v(t_i)||
    # Sums the magnitude of each incremental velocity change over all
    # consecutive frame pairs, accounting for both magnitude and direction.
    impulse_per_mass = 0.0
    for i in range(len(frames_during) - 1):
        dv = [
            frames_during[i + 1]["vel"][k] - frames_during[i]["vel"][k]
            for k in range(3)
        ]
        impulse_per_mass += math.sqrt(sum(c * c for c in dv))

    # ── peak-to-average ratio ───────────────────────────────────────────────
    peak_to_avg = peak_speed / avg_speed if avg_speed > 0 else None

    # ── rise time ───────────────────────────────────────────────────────────
    threshold_90 = baseline_speed + 0.9 * excursion
    rise_time = None
    for f in frames_during:
        if f["speed"] >= threshold_90:
            rise_time = f["time"] - t_start
            break

    # ── settling time ────────────────────────────────────────────────────────
    # Band: baseline ± (SETTLE_FRACTION * excursion)
    settle_band = SETTLE_FRACTION * excursion
    settling_time = None
    if frames_post:
        last_outside_t = None
        for f in frames_post:
            if abs(f["speed"] - baseline_speed) > settle_band:
                last_outside_t = f["time"]
        if last_outside_t is not None:
            settling_time = last_outside_t - t_end
        else:
            settling_time = 0.0  # already settled at disengage

    return {
        "duration_s": round(t_end - t_start, 4),
        "impulse_per_mass": round(impulse_per_mass, 4),
        "peak_speed": round(peak_speed, 4),
        "min_speed": round(min_speed, 4),
        "avg_speed": round(avg_speed, 4),
        "std_speed": round(std_speed, 4),
        "peak_to_avg": round(peak_to_avg, 4) if peak_to_avg is not None else None,
        "rise_time_s": round(rise_time, 4) if rise_time is not None else None,
        "settling_time_s": round(settling_time, 4) if settling_time is not None else None,
        "baseline_speed": round(baseline_speed, 4),
    }


# ──────────────────────────────────────────────────────────────────────────────
# Main processing
# ──────────────────────────────────────────────────────────────────────────────

def process_file(filename: str) -> tuple[list[dict], dict]:
    """
    Process one log file. Returns (rows, summary).
    """
    path = os.path.join(LOG_DIR, filename)
    data = load_log(path)

    user_id, expected_sequence = parse_filename(filename)
    n_expected = len(expected_sequence)

    # ── segment & classify ──────────────────────────────────────────────────
    all_segs = segment_interactions(data)
    noisy_segs = [s for s in all_segs if is_noisy(s)]
    valid_segs = [s for s in all_segs if not is_noisy(s)]

    n_noise = len(noisy_segs)
    n_valid = len(valid_segs)
    n_missing = max(0, n_expected - n_valid)

    rows: list[dict] = []

    for idx, seg in enumerate(valid_segs):
        # Assign interaction type from expected sequence (in order)
        itype = expected_sequence[idx] if idx < len(expected_sequence) else "unknown"
        instance_num = idx + 1  # 1-based within this file

        metrics = compute_metrics(seg, data)
        if metrics is None:
            continue

        row = {
            "user_id": user_id,
            "filename": filename,
            "interaction_type": itype,
            "instance_num": instance_num,
            **metrics,
        }
        rows.append(row)

    summary = {
        "user_id": user_id,
        "filename": filename,
        "n_expected": n_expected,
        "n_detected_total": len(all_segs),
        "n_valid": n_valid,
        "n_noise": n_noise,
        "n_missing": n_missing,
    }

    return rows, summary


def main():
    all_rows: list[dict] = []
    summaries: list[dict] = []

    filenames = sorted(
        f for f in os.listdir(LOG_DIR) if f.endswith(".json")
    )

    print("\n=== Processing files ===")
    for filename in filenames:
        try:
            rows, summary = process_file(filename)
            all_rows.extend(rows)
            summaries.append(summary)
            print(
                f"  {filename}: expected={summary['n_expected']}, "
                f"valid={summary['n_valid']}, noise={summary['n_noise']}, "
                f"missing={summary['n_missing']}"
            )
        except Exception as e:
            print(f"  {filename}: FAILED – {e}")

    # ── write CSV ────────────────────────────────────────────────────────────
    if not all_rows:
        print("No data rows produced.")
        return

    fieldnames = [
        "user_id",
        "filename",
        "interaction_type",
        "instance_num",
        "duration_s",
        "impulse_per_mass",
        "peak_speed",
        "min_speed",
        "avg_speed",
        "std_speed",
        "peak_to_avg",
        "rise_time_s",
        "settling_time_s",
        "baseline_speed",
    ]

    with open(OUTPUT_CSV, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_rows)

    print(f"\nWrote {len(all_rows)} rows → {OUTPUT_CSV}")

    # ── noise / missing report ───────────────────────────────────────────────
    total_noise = sum(s["n_noise"] for s in summaries)
    total_missing = sum(s["n_missing"] for s in summaries)

    print("\n=== Noise & Missing Summary ===")
    print(f"{'File':<40} {'Expected':>8} {'Valid':>6} {'Noise':>6} {'Missing':>8}")
    print("-" * 72)
    for s in summaries:
        print(
            f"{s['filename']:<40} {s['n_expected']:>8} {s['n_valid']:>6} "
            f"{s['n_noise']:>6} {s['n_missing']:>8}"
        )
    print("-" * 72)
    print(f"{'TOTAL':>40} {'':>8} {'':>6} {total_noise:>6} {total_missing:>8}")

    # ── per-user per-type aggregate ──────────────────────────────────────────
    print("\n=== Per-User Per-Type Mean Metrics ===")
    from itertools import groupby

    def mean_or_na(vals):
        vals = [v for v in vals if v is not None]
        return round(float(np.mean(vals)), 4) if vals else "N/A"

    # group by (user_id, interaction_type)
    sorted_rows = sorted(all_rows, key=lambda r: (r["user_id"], r["interaction_type"]))
    print(
        f"{'user_id':>8} {'type':>6} {'N':>4} "
        f"{'J/m':>8} {'PkAvg':>7} {'rise(s)':>8} {'settle(s)':>10}"
    )
    print("-" * 60)
    for (uid, itype), group in groupby(
        sorted_rows, key=lambda r: (r["user_id"], r["interaction_type"])
    ):
        grp = list(group)
        n = len(grp)
        print(
            f"{uid:>8} {itype:>6} {n:>4} "
            f"{mean_or_na([r['impulse_per_mass'] for r in grp]):>8} "
            f"{mean_or_na([r['peak_to_avg'] for r in grp]):>7} "
            f"{mean_or_na([r['rise_time_s'] for r in grp]):>8} "
            f"{mean_or_na([r['settling_time_s'] for r in grp]):>10}"
        )


if __name__ == "__main__":
    main()
