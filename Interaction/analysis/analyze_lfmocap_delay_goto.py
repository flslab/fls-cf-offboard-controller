"""
analyze_lfmocap_delay.py

Offline analysis for the LFMoCapDelay experiment.

Auto-discovers all step-size pairs (e.g. 20 cm, 50 cm) in LOG_DIR, then for
each pair and across all pairs:
  1. Saves a per-step timing CSV.
  2. Plots TTT leader vs follower (comparison across step sizes).
  3. Plots TimeToDetect and TotalDelay (comparison across step sizes).
  4. Plots leader Y velocity during T1→T10 for each step (all sizes overlaid).
  5. Plots leader Y velocity over the full log.
"""

import csv
import glob
import json
import os
import re

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

matplotlib.use("macosx")

LOG_DIR = os.path.join(os.path.dirname(__file__), "../..", "logs", "LFMoCapDelay_goto")

# ── Config ────────────────────────────────────────────────────────────────────
DELTA_V = 0.1          # m/s — KF threshold used during the experiment


# ── Log discovery ─────────────────────────────────────────────────────────────
def find_log_pairs(log_dir: str) -> list[tuple[str, str, str]]:
    """
    Scan log_dir for LFMoCapDelay_leader_{aldo tpha}_{duration}ms*.json files.
    For each found leader file, find the matching follower file with the same
    (alpha, duration) pair.
    Returns list of (label, leader_path, follower_path) sorted by (alpha, duration).
    """
    pattern = os.path.join(log_dir, "LFMoCapDelay_leader_*.json")
    leader_files = sorted(glob.glob(pattern))
    pairs = []
    for lf in leader_files:
        m = re.search(r"LFMoCapDelay_leader_(\d+)_(\d+)ms", os.path.basename(lf))
        if not m:
            continue
        alpha, duration = m.group(1), m.group(2)
        ff_pattern = os.path.join(log_dir, f"LFMoCapDelay_follower_{alpha}_{duration}ms*.json")
        ff_matches = glob.glob(ff_pattern)
        if not ff_matches:
            print(f"  Warning: no follower log found for alpha={alpha} duration={duration}ms, skipping.")
            continue
        label = f"alpha={alpha} duration={duration}ms"
        pairs.append((label, lf, ff_matches[0]))
    pairs.sort(key=lambda x: (int(re.search(r"alpha=(\d+)", x[0]).group(1)),
                               int(re.search(r"duration=(\d+)", x[0]).group(1))))
    return pairs


# ── Parsing ───────────────────────────────────────────────────────────────────
def load_log(path: str) -> list[dict]:
    with open(path) as f:
        return json.load(f)


def extract_group(log: list[dict], group: str) -> list[dict]:
    return [e["data"] for e in log if e.get("type") == group and e.get("data")]


def _events_by_name(log: list[dict], event_name: str) -> list[dict]:
    return [
        e["data"] for e in log
        if e.get("type") == "events" and e.get("data", {}).get("event") == event_name
    ]


def _leader_timing_from_events(leader_log: list[dict]) -> dict:
    starts   = {e["step"]: e for e in _events_by_name(leader_log, "step_start")}
    arrivals = {e["step"]: e for e in _events_by_name(leader_log, "step_arrived") if "T10" in e}
    result = {}
    for step, s in starts.items():
        if step in arrivals:
            a = arrivals[step]
            result[step] = {
                "step":         step,
                "alpha_mm":     s.get("alpha_mm", "?"),
                "T1":           s["T1"],
                "T10":          a["T10"],
                "TTT_leader_s": round(a["T10"] - s["T1"], 4),
            }
    return result


def _follower_timing_from_events(follower_log: list[dict]) -> dict:
    detections = {e["step"]: e for e in _events_by_name(follower_log, "leader_detected")}
    arrivals   = {e["step"]: e for e in _events_by_name(follower_log, "step_arrived") if "T11" in e}
    mission    = next(iter(_events_by_name(follower_log, "mission_start")), {})
    result = {}
    for step, d in detections.items():
        if step in arrivals:
            a = arrivals[step]
            result[step] = {
                "step":             step,
                "follower_step_mm": mission.get("follower_step_mm", "?"),
                "delta_v":          mission.get("delta_v", DELTA_V),
                "T2":               d["T2"],
                "T11":              a["T11"],
                "TTT_follower_s":   round(a["T11"] - d["T2"], 4),
            }
    return result


def build_timing_table(leader_log: list[dict], follower_log: list[dict]) -> list[dict]:
    leader_timing = {d["step"]: d for d in extract_group(leader_log, "timing") if "T1" in d}
    if not leader_timing:
        leader_timing = _leader_timing_from_events(leader_log)

    follower_timing = {d["step"]: d for d in extract_group(follower_log, "timing") if "T2" in d}
    if not follower_timing:
        follower_timing = _follower_timing_from_events(follower_log)

    rows = []
    for step in sorted(set(leader_timing) & set(follower_timing)):
        l = leader_timing[step]
        f = follower_timing[step]
        T1  = l["T1"]
        T10 = l["T10"]
        T2  = f["T2"]
        T11 = f["T11"]
        rows.append({
            "step":                  step,
            "alpha_mm":              l.get("alpha_mm", "?"),
            "follower_step_mm":      f.get("follower_step_mm", "?"),
            "delta_v":               f.get("delta_v", DELTA_V),
            "T1":                    T1,
            "T2":                    T2,
            "T10":                   T10,
            "T11":                   T11,
            "TimeToDetect_ms":       round((T2  - T1)  * 1000, 1),
            "TotalDelay_ms":         round((T11 - T10) * 1000, 1),
            "TTT_leader_ms":         round((T10 - T1)  * 1000, 1),
            "TTT_follower_ms":       round((T11 - T2)  * 1000, 1),
            "TTT_leader_s_logged":   l.get("TTT_leader_s"),
            "TTT_follower_s_logged": f.get("TTT_follower_s"),
        })
    return rows


def extract_y_series(log: list[dict], group: str = "frames"):
    entries = extract_group(log, group)
    if not entries:
        return np.array([]), np.array([])
    times = np.array([e["time"] for e in entries if "time" in e and "tvec" in e])
    ys    = np.array([e["tvec"][1] for e in entries if "time" in e and "tvec" in e])
    return times, ys


def extract_vel_y_series(log: list[dict], group: str = "frames"):
    entries = extract_group(log, group)
    if not entries:
        return np.array([]), np.array([])
    valid = [e for e in entries if "time" in e and "vel" in e]
    times = np.array([e["time"]   for e in valid])
    vels  = np.array([e["vel"][1] for e in valid])
    return times, vels


# ── CSV report ────────────────────────────────────────────────────────────────
def save_csv(rows: list[dict], out_path: str):
    if not rows:
        print("No matched steps found.")
        return

    per_step_fields = [
        "step", "alpha_mm", "follower_step_mm", "delta_v",
        "T1", "T2", "T10", "T11",
        "TimeToDetect_ms", "TotalDelay_ms", "TTT_leader_ms", "TTT_follower_ms",
        "TTT_leader_s_logged", "TTT_follower_s_logged",
    ]
    keys   = ["TimeToDetect_ms", "TotalDelay_ms", "TTT_leader_ms", "TTT_follower_ms"]
    labels = ["TimeToDetect",    "TotalDelay",    "TTT_leader",    "TTT_follower"]
    vals   = {k: np.array([r[k] for r in rows]) for k in keys}

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(per_step_fields)
        for r in rows:
            writer.writerow([r.get(k, "") for k in per_step_fields])
        writer.writerow([])
        writer.writerow(["metric", "mean_ms", "median_ms", "std_ms", "min_ms", "max_ms"])
        for k, lbl in zip(keys, labels):
            v = vals[k]
            writer.writerow([lbl,
                              round(v.mean(), 2), round(float(np.median(v)), 2),
                              round(v.std(), 2),  round(v.min(), 2), round(v.max(), 2)])
    print(f"CSV saved → {out_path}")


# ── Per-dataset plots ─────────────────────────────────────────────────────────
# Each function takes a single (label, rows, leader_log) and a tag for filenames.

def plot_ttt(label: str, rows: list, tag: str, save_dir: str):
    if not rows:
        return
    steps = [r["step"] for r in rows]
    ttt_l = np.array([r["TTT_leader_ms"]   for r in rows])
    ttt_f = np.array([r["TTT_follower_ms"] for r in rows])
    x     = np.arange(len(steps))
    width = 0.35

    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    ax.bar(x - width/2, ttt_l, width, label="TTT leader",   alpha=0.85)
    ax.bar(x + width/2, ttt_f, width, label="TTT follower", alpha=0.85)

    ax.set_xticks(x)
    ax.set_xticklabels([f"Step {s}" for s in steps], fontsize=16)
    ax.set_title(f"Time To Travel (ms)  —  {label}", loc="left", fontsize=16)
    ax.tick_params(axis="both", labelsize=16)
    ax.grid(True, linestyle="--", alpha=0.7)
    ax.set_ylim(0, 1000)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.legend(fontsize=16)
    plt.tight_layout()

    out = os.path.join(save_dir, f"lfmocap_ttt_{tag}.png")
    fig.savefig(out, dpi=300)
    print(f"Plot saved  → {out}")
    # plt.show()


def plot_ttd_totaldelay(label: str, rows: list, tag: str, save_dir: str):
    if not rows:
        return
    steps = [r["step"] for r in rows]
    ttd   = np.array([r["TimeToDetect_ms"] for r in rows])
    td    = np.array([r["TotalDelay_ms"]   for r in rows])
    x     = np.arange(len(steps))
    width = 0.35

    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    ax.bar(x - width/2, ttd, width, label="TimeToDetect", alpha=0.85)
    ax.bar(x + width/2, td,  width, label="TotalDelay",   alpha=0.85)

    ax.set_xticks(x)
    ax.set_xticklabels([f"Step {s}" for s in steps], fontsize=16)
    ax.set_title(f"TTD & Total Delay (ms)  —  {label}", loc="left", fontsize=16)
    ax.tick_params(axis="both", labelsize=16)
    ax.set_ylim(0, 400)
    ax.grid(True, linestyle="--", alpha=0.7)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.legend(fontsize=16)
    plt.tight_layout()

    out = os.path.join(save_dir, f"lfmocap_ttd_totaldelay_{tag}.png")
    fig.savefig(out, dpi=300)
    print(f"Plot saved  → {out}")
    # plt.show()


def plot_velocity_t1_t10(label: str, rows: list, leader_log: list, tag: str, save_dir: str):
    times, vel = extract_vel_y_series(leader_log, "frames")
    if len(times) < 2:
        print(f"Not enough leader frame data for {label}.")
        return

    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    for r in rows:
        T1, T10 = r["T1"], r["T10"] + 0.3
        mask = (times >= T1) & (times <= T10)
        if not mask.any():
            continue
        ax.plot(times[mask] - T1, vel[mask], linewidth=1.5, label=f"Step {r['step']}")

    ax.set_xlabel("Time since T1 (s)", fontsize=16)
    ax.set_title(f"Leader Y velocity (m/s)  —  T1 to T10  —  {label}", loc="left", fontsize=16)
    ax.tick_params(axis="both", labelsize=16)
    ax.grid(True, linestyle="--", alpha=0.7)
    ax.set_ylim(0, 3.5)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.legend(fontsize=14)
    plt.tight_layout()

    out = os.path.join(save_dir, f"lfmocap_velocity_t1_t10_{tag}.png")
    fig.savefig(out, dpi=300)
    print(f"Plot saved  → {out}")
    # plt.show()


def plot_velocity_full(label: str, rows: list, leader_log: list, follower_log: list, tag: str, save_dir: str):
    l_times, l_vel = extract_vel_y_series(leader_log, "frames")
    f_times, f_vel = extract_vel_y_series(follower_log, "frames")
    if len(l_times) < 2:
        print(f"Not enough leader frame data for {label}.")
        return

    t_start = min(r["T1"]  for r in rows) - 2.0
    t_end   = max(r["T10"] for r in rows) + 2.0
    t0      = t_start

    has_follower = len(f_times) >= 2
    nrows = 2 if has_follower else 1
    fig, axes = plt.subplots(nrows, 1, figsize=(20, 6 * nrows), sharex=True)
    if nrows == 1:
        axes = [axes]

    ax_l = axes[0]
    l_mask = (l_times >= t_start) & (l_times <= t_end)
    ax_l.plot(l_times[l_mask] - t0, l_vel[l_mask], color="#2980b9", linewidth=1.2, label="Leader Y velocity")

    t1_labeled = t10_labeled = False
    for r in rows:
        t1_lbl  = "T1 (step start)"    if not t1_labeled  else "_nolegend_"
        t10_lbl = "T10 (step arrived)" if not t10_labeled else "_nolegend_"
        ax_l.axvline(r["T1"]  - t0, color="magenta",    linestyle="--", linewidth=2.0, alpha=0.8, label=t1_lbl)
        ax_l.axvline(r["T10"] - t0, color="darkorange",  linestyle="--", linewidth=2.0, alpha=0.8, label=t10_lbl)
        ax_l.text(r["T1"] - t0, 0.05, f" {r['step']}", fontsize=8, color="#e74c3c", va="bottom")
        t1_labeled = t10_labeled = True

    delta_v = rows[0].get("delta_v", DELTA_V) if rows else DELTA_V
    ax_l.axhline(delta_v, color="red", linestyle=":", linewidth=1.5, label=f"delta_v = {delta_v} m/s")

    ax_l.set_title(f"Leader Y velocity (m/s)  —  full log  —  {label}", loc="left", fontsize=16)
    ax_l.tick_params(axis="both", labelsize=16)
    ax_l.grid(True, linestyle="--", alpha=0.7)
    ax_l.spines["top"].set_visible(False)
    ax_l.spines["right"].set_visible(False)
    ax_l.set_ylim(0, 2)
    ax_l.set_xlim(t_start - t0, t_end - t0)
    ax_l.legend(fontsize=14)

    if has_follower:
        ax_f   = axes[1]
        f_mask = (f_times >= t_start) & (f_times <= t_end)
        ax_f.plot(f_times[f_mask] - t0, f_vel[f_mask], color="#27ae60", linewidth=1.2, label="Follower Y velocity")

        t2_labeled = t11_labeled = False
        for r in rows:
            t2_lbl  = "T2 (detected)"      if not t2_labeled  else "_nolegend_"
            t11_lbl = "T11 (step arrived)" if not t11_labeled else "_nolegend_"
            ax_f.axvline(r["T2"]  - t0, color="magenta",   linestyle="--", linewidth=2.0, alpha=0.8, label=t2_lbl)
            ax_f.axvline(r["T11"] - t0, color="darkorange", linestyle="--", linewidth=2.0, alpha=0.8, label=t11_lbl)
            ax_f.text(r["T2"] - t0, 0.05, f" {r['step']}", fontsize=8, color="#27ae60", va="bottom")
            t2_labeled = t11_labeled = True

        ax_f.axhline(delta_v, color="red", linestyle=":", linewidth=1.5, label=f"delta_v = {delta_v} m/s")

        ax_f.set_xlabel("Time (s)", fontsize=16)
        ax_f.set_title(f"Follower Y velocity (m/s)  —  full log  —  {label}", loc="left", fontsize=16)
        ax_f.tick_params(axis="both", labelsize=16)
        ax_f.grid(True, linestyle="--", alpha=0.7)
        ax_f.spines["top"].set_visible(False)
        ax_f.spines["right"].set_visible(False)
        ax_f.set_ylim(0, 2)
        ax_f.legend(fontsize=14)
    else:
        axes[0].set_xlabel("Time (s)", fontsize=16)

    plt.tight_layout()

    out = os.path.join(save_dir, f"lfmocap_velocity_full_{tag}.png")
    fig.savefig(out, dpi=300)
    print(f"Plot saved  → {out}")
    # plt.show()


# ── Main ──────────────────────────────────────────────────────────────────────
def main(log_dir: str):
    pairs = find_log_pairs(log_dir)
    if not pairs:
        print(f"No log pairs found in {log_dir}")
        return

    for label, leader_path, follower_path in pairs:
        print(f"\n[{label}]  leader:   {os.path.basename(leader_path)}")
        print(f"{'':8}  follower: {os.path.basename(follower_path)}")
        leader_log   = load_log(leader_path)
        follower_log = load_log(follower_path)
        rows = build_timing_table(leader_log, follower_log)
        tag  = label.replace(" ", "_").replace("=", "")   # e.g. "alpha100_duration200ms"

        save_csv(rows, os.path.join(log_dir, f"lfmocap_delay_results_{tag}.csv"))
        plot_ttt(label, rows, tag, log_dir)
        plot_ttd_totaldelay(label, rows, tag, log_dir)
        plot_velocity_t1_t10(label, rows, leader_log, tag, log_dir)
        plot_velocity_full(label, rows, leader_log, follower_log, tag, log_dir)


if __name__ == "__main__":
    main(LOG_DIR)
