"""
analyze_lfmocap_delay.py

Offline analysis for the LFMoCapDelay experiment.

Loads one leader log and one follower log, matches steps by index, then:
  1. Prints a per-step timing table (T1, T2, T10, T11, derived delays).
  2. Plots per-step delay bar charts.
  3. Plots Y-position time series for both drones with T1/T2/T10/T11 markers.
  4. Plots the follower's KF-estimated leader Y-velocity with ΔV threshold
     and T2 detection markers.

Hard-coded file names are at the top of this file; pass them into main().
"""

import json
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

matplotlib.use("macosx")

# ── Hard-coded log paths ──────────────────────────────────────────────────────
LEADER_LOG   = "../../logs/LFMoCapDelay_leader_2026-04-08_12-00-00.json"
FOLLOWER_LOG = "../../logs/LFMoCapDelay_follower_2026-04-08_12-00-00.json"

# ── Config ────────────────────────────────────────────────────────────────────
DELTA_V = 0.1          # m/s — KF threshold used during the experiment


# ── Parsing ───────────────────────────────────────────────────────────────────
def load_log(path: str) -> list[dict]:
    with open(path) as f:
        return json.load(f)


def extract_group(log: list[dict], group: str) -> list[dict]:
    """Return all data-dicts whose 'type' matches group."""
    return [e["data"] for e in log if e.get("type") == group and e.get("data")]


def build_timing_table(leader_log: list[dict], follower_log: list[dict]) -> list[dict]:
    """
    Match leader and follower timing entries by step index and compute all
    derived delay variables.

    Returns a list of dicts, one per matched step, with keys:
      step, alpha_mm, T1, T2, T10, T11,
      TimeToDetect_ms, TotalDelay_ms, TTT_leader_ms, TTT_follower_ms
    """
    leader_timing   = {d["step"]: d for d in extract_group(leader_log,   "timing")}
    follower_timing = {d["step"]: d for d in extract_group(follower_log, "timing")}

    rows = []
    for step in sorted(set(leader_timing) & set(follower_timing)):
        l = leader_timing[step]
        f = follower_timing[step]

        T1  = l["T1"]
        T10 = l["T10"]
        T2  = f["T2"]
        T11 = f["T11"]

        rows.append({
            "step":              step,
            "alpha_mm":          l.get("alpha_mm", "?"),
            "T1":                T1,
            "T2":                T2,
            "T10":               T10,
            "T11":               T11,
            "T1_iso":            l.get("T1_iso", ""),
            "T2_iso":            f.get("T2_iso", ""),
            "T10_iso":           l.get("T10_iso", ""),
            "T11_iso":           f.get("T11_iso", ""),
            "TimeToDetect_ms":   round((T2  - T1)  * 1000, 1),
            "TotalDelay_ms":     round((T11 - T10) * 1000, 1),
            "TTT_leader_ms":     round((T10 - T1)  * 1000, 1),
            "TTT_follower_ms":   round((T11 - T2)  * 1000, 1),
        })
    return rows


def extract_y_series(log: list[dict], group: str = "frames"):
    """Return (times, y_positions) arrays from a Vicon frame group."""
    entries = extract_group(log, group)
    if not entries:
        return np.array([]), np.array([])
    times = np.array([e["time"] for e in entries if "time" in e and "tvec" in e])
    ys    = np.array([e["tvec"][1] for e in entries if "time" in e and "tvec" in e])
    return times, ys


def extract_kf_vel_y(follower_log: list[dict]):
    """Return (times, kf_vel_y) arrays from the follower's leader_frames group."""
    entries = extract_group(follower_log, "leader_frames")
    if not entries:
        return np.array([]), np.array([])
    times = np.array([e["time"] for e in entries if "time" in e and "kf_vel_y" in e])
    vels  = np.array([e["kf_vel_y"] for e in entries if "time" in e and "kf_vel_y" in e])
    return times, vels


# ── Console report ────────────────────────────────────────────────────────────
def print_table(rows: list[dict]):
    if not rows:
        print("No matched steps found.")
        return

    alpha = rows[0]["alpha_mm"]
    print(f"\n{'='*90}")
    print(f"  LFMoCapDelay — α = {alpha} mm")
    print(f"{'='*90}")
    hdr = (f"{'Step':>4}  {'T1 (iso)':>25}  {'T2 (iso)':>25}  "
           f"{'T10 (iso)':>25}  {'T11 (iso)':>25}")
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        print(f"{r['step']:>4}  {r['T1_iso']:>25}  {r['T2_iso']:>25}  "
              f"{r['T10_iso']:>25}  {r['T11_iso']:>25}")

    print()
    hdr2 = (f"{'Step':>4}  {'TimeToDetect':>14}  {'TotalDelay':>12}  "
            f"{'TTT_leader':>12}  {'TTT_follower':>14}")
    print(hdr2)
    print("-" * len(hdr2))
    for r in rows:
        print(f"{r['step']:>4}  {r['TimeToDetect_ms']:>12.1f} ms  "
              f"{r['TotalDelay_ms']:>10.1f} ms  "
              f"{r['TTT_leader_ms']:>10.1f} ms  "
              f"{r['TTT_follower_ms']:>12.1f} ms")

    print()
    keys = ["TimeToDetect_ms", "TotalDelay_ms", "TTT_leader_ms", "TTT_follower_ms"]
    labels = ["TimeToDetect", "TotalDelay", "TTT_leader", "TTT_follower"]
    vals = {k: np.array([r[k] for r in rows]) for k in keys}
    print(f"{'Metric':<16}  {'Mean':>8}  {'Median':>8}  {'Std':>8}  "
          f"{'Min':>8}  {'Max':>8}")
    print("-" * 64)
    for k, lbl in zip(keys, labels):
        v = vals[k]
        print(f"{lbl:<16}  {v.mean():>7.1f}ms  {np.median(v):>7.1f}ms  "
              f"{v.std():>7.1f}ms  {v.min():>7.1f}ms  {v.max():>7.1f}ms")
    print(f"{'='*90}\n")


# ── Plot 1: per-step delay bar chart ──────────────────────────────────────────
def plot_delay_bars(rows: list[dict]):
    if not rows:
        return

    steps  = [r["step"] for r in rows]
    ttd    = [r["TimeToDetect_ms"]  for r in rows]
    td     = [r["TotalDelay_ms"]    for r in rows]
    ttt_l  = [r["TTT_leader_ms"]    for r in rows]
    ttt_f  = [r["TTT_follower_ms"]  for r in rows]
    alpha  = rows[0]["alpha_mm"]

    x      = np.arange(len(steps))
    width  = 0.2

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.bar(x - 1.5*width, ttd,   width, label="TimeToDetect",  color="#e74c3c")
    ax.bar(x - 0.5*width, td,    width, label="TotalDelay",    color="#9b59b6")
    ax.bar(x + 0.5*width, ttt_l, width, label="TTT_leader",    color="#2980b9")
    ax.bar(x + 1.5*width, ttt_f, width, label="TTT_follower",  color="#27ae60")

    ax.set_xticks(x)
    ax.set_xticklabels([f"Step {s}" for s in steps])
    ax.set_ylabel("Time (ms)")
    ax.set_title(f"LFMoCapDelay — Per-step delays  (α = {alpha} mm)")
    ax.legend()
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.grid(axis="y", alpha=0.3)
    ax.grid(axis="y", which="minor", alpha=0.15)
    plt.tight_layout()
    plt.show()


# ── Plot 2: Y-position time series ────────────────────────────────────────────
def plot_y_positions(rows: list[dict],
                     leader_times: np.ndarray, leader_ys: np.ndarray,
                     follower_times: np.ndarray, follower_ys: np.ndarray):
    if not rows:
        return

    alpha = rows[0]["alpha_mm"]
    t0    = min(leader_times[0] if len(leader_times) else 0,
                follower_times[0] if len(follower_times) else 0)

    fig, ax = plt.subplots(figsize=(13, 5))

    if len(leader_times):
        ax.plot(leader_times - t0, leader_ys,
                label="Leader Y (Vicon)", color="#2980b9", linewidth=1.5)
    if len(follower_times):
        ax.plot(follower_times - t0, follower_ys,
                label="Follower Y (Vicon)", color="#27ae60", linewidth=1.5)

    # Per-step event markers
    colors = {"T1": "#e74c3c", "T10": "#c0392b", "T2": "#f39c12", "T11": "#d35400"}
    labels_added = set()
    for r in rows:
        for key, c in colors.items():
            t_val = r[key] - t0
            lbl   = key if key not in labels_added else "_nolegend_"
            ax.axvline(t_val, color=c, linestyle="--", linewidth=0.9,
                       alpha=0.8, label=lbl)
            labels_added.add(key)
            ax.text(t_val, ax.get_ylim()[0] if ax.get_ylim()[0] != 0 else 0,
                    key, fontsize=6, color=c, rotation=90,
                    va="bottom", ha="right")

    ax.set_xlabel("Time (s, relative)")
    ax.set_ylabel("Y position (m)")
    ax.set_title(f"LFMoCapDelay — Y-position time series  (α = {alpha} mm)")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


# ── Plot 3: KF velocity + detection markers ───────────────────────────────────
def plot_kf_velocity(rows: list[dict],
                     kf_times: np.ndarray, kf_vel_y: np.ndarray):
    if not rows or len(kf_times) == 0:
        return

    alpha = rows[0]["alpha_mm"]
    t0    = kf_times[0]

    fig, ax = plt.subplots(figsize=(13, 4))

    ax.plot(kf_times - t0, kf_vel_y,
            label="KF leader vel_Y", color="#8e44ad", linewidth=1.2)
    ax.axhline( DELTA_V, color="gray", linestyle=":", linewidth=1,
                label=f"+ΔV = {DELTA_V} m/s")
    ax.axhline(-DELTA_V, color="gray", linestyle=":", linewidth=1,
                label=f"−ΔV = {DELTA_V} m/s")

    # T1 (leader step start) and T2 (follower detection) per step
    t1_added = t2_added = False
    for r in rows:
        t1_lbl = "T1 (leader starts)" if not t1_added else "_nolegend_"
        t2_lbl = "T2 (follower detects)" if not t2_added else "_nolegend_"
        ax.axvline(r["T1"] - t0, color="#e74c3c", linestyle="--",
                   linewidth=0.9, alpha=0.8, label=t1_lbl)
        ax.axvline(r["T2"] - t0, color="#f39c12", linestyle="--",
                   linewidth=0.9, alpha=0.8, label=t2_lbl)
        ax.text(r["T2"] - t0, DELTA_V,
                f"  {r['TimeToDetect_ms']:.0f} ms", fontsize=7,
                color="#f39c12", va="bottom")
        t1_added = t2_added = True

    ax.set_xlabel("Time (s, relative)")
    ax.set_ylabel("KF vel_Y (m/s)")
    ax.set_title(
        f"LFMoCapDelay — Follower KF leader Y-velocity  (α = {alpha} mm, ΔV = {DELTA_V} m/s)"
    )
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


# ── Plot 4: TTT comparison (leader vs follower) ───────────────────────────────
def plot_ttt_comparison(rows: list[dict]):
    """
    Scatter / line plot comparing TTT_leader and TTT_follower across steps.
    Helps visualise whether the follower is faster or slower than the leader.
    """
    if not rows:
        return

    steps  = [r["step"] for r in rows]
    ttt_l  = np.array([r["TTT_leader_ms"]   for r in rows])
    ttt_f  = np.array([r["TTT_follower_ms"] for r in rows])
    alpha  = rows[0]["alpha_mm"]

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(steps, ttt_l, "o-", label="TTT_leader",   color="#2980b9", linewidth=1.5)
    ax.plot(steps, ttt_f, "s-", label="TTT_follower", color="#27ae60", linewidth=1.5)
    ax.axhline(ttt_l.mean(), color="#2980b9", linestyle="--", linewidth=0.8,
               label=f"mean leader = {ttt_l.mean():.0f} ms")
    ax.axhline(ttt_f.mean(), color="#27ae60", linestyle="--", linewidth=0.8,
               label=f"mean follower = {ttt_f.mean():.0f} ms")

    ax.set_xticks(steps)
    ax.set_xticklabels([f"Step {s}" for s in steps])
    ax.set_ylabel("Travel time (ms)")
    ax.set_title(f"LFMoCapDelay — TTT leader vs follower  (α = {alpha} mm)")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


# ── Main ──────────────────────────────────────────────────────────────────────
def main(leader_log_path: str, follower_log_path: str):
    print(f"Leader log:   {leader_log_path}")
    print(f"Follower log: {follower_log_path}")

    leader_log   = load_log(leader_log_path)
    follower_log = load_log(follower_log_path)

    # ── timing table ────────────────────────────────────────────────────
    rows = build_timing_table(leader_log, follower_log)
    print_table(rows)

    # ── position series ─────────────────────────────────────────────────
    leader_times,   leader_ys   = extract_y_series(leader_log,   "frames")
    follower_times, follower_ys = extract_y_series(follower_log, "frames")

    # ── KF velocity (follower-side view of leader) ───────────────────────
    kf_times, kf_vel_y = extract_kf_vel_y(follower_log)

    # ── plots ────────────────────────────────────────────────────────────
    plot_delay_bars(rows)
    plot_y_positions(rows, leader_times, leader_ys, follower_times, follower_ys)
    plot_kf_velocity(rows, kf_times, kf_vel_y)
    plot_ttt_comparison(rows)


if __name__ == "__main__":
    main(LEADER_LOG, FOLLOWER_LOG)
