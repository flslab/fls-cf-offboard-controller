"""
Block-interaction gesture analysis: flick vs poke vs push
=========================================================
Reads trajectory logs from logs/block_interaction/, computes kinematic
features (peak velocity, peak acceleration, rotational stability), runs
pairwise DTW for within-gesture repeatability, and saves an overlay plot.

Log format (JSON-lines array):
  {"type": "frames", "data": {
      "tvec": [x, y, z],   # position (m)
      "quat": [qx, qy, qz, qw],  # orientation (qw is scalar, largest |value|)
      "vel":  [vx, vy, vz],       # velocity (m/s)
      "time": <unix timestamp>
  }}

File naming: us{id}_{num}{gesture}_translation_{timestamp}.json
"""

import os
import re
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from collections import defaultdict
from itertools import combinations

# ── paths ─────────────────────────────────────────────────────────────────────
_HERE         = os.path.dirname(os.path.abspath(__file__))
LOG_DIR       = os.path.join(_HERE, "../..", "logs", "block_interaction")
OUT_PNG_TRAJ  = os.path.join(_HERE, "trajectories_xy.png")
OUT_PNG_VEL   = os.path.join(_HERE, "velocity_vs_time.png")

FILE_RE  = re.compile(
    r"us(?P<uid>\d+)_(?P<num>\d+)(?P<gesture>poke|push|flick)_translation_"
    r"(?P<ts>[\d\-_]+)\.json$"
)

# ── data loading ──────────────────────────────────────────────────────────────

def load_trial(path: str):
    """
    Parse a block-interaction JSON-lines file.
    Returns: tvec (N,3), quat (N,4), time (N,), vel (N,3)  as float64 arrays.
    """
    tvec, quat, time_, vel = [], [], [], []
    with open(path, encoding="utf-8") as fh:
        raw = fh.read()

    for line in raw.splitlines():
        line = line.strip().rstrip(",")
        if not line or line in ("[", "]"):
            continue
        try:
            rec = json.loads(line)
        except json.JSONDecodeError:
            continue
        if rec.get("type") != "frames":
            continue
        d = rec["data"]
        tvec.append(d["tvec"])
        quat.append(d["quat"])
        time_.append(d["time"])
        vel.append(d.get("vel", [0.0, 0.0, 0.0]))

    return (
        np.asarray(tvec,  dtype=np.float64),
        np.asarray(quat,  dtype=np.float64),
        np.asarray(time_, dtype=np.float64),
        np.asarray(vel,   dtype=np.float64),
    )


def discover_trials(log_dir: str) -> dict:
    """Return {gesture: [(uid, filepath), ...]} sorted by filename."""
    trials = defaultdict(list)
    for fname in sorted(os.listdir(log_dir)):
        m = FILE_RE.match(fname)
        if m:
            trials[m.group("gesture")].append(
                (int(m.group("uid")), os.path.join(log_dir, fname))
            )
    return dict(trials)


# ── kinematics ────────────────────────────────────────────────────────────────

def _qw_index(quat: np.ndarray) -> int:
    """Detect which column holds qw (scalar part ≈ largest magnitude)."""
    return int(np.argmax(np.median(np.abs(quat), axis=0)))


def translation_speed(tvec: np.ndarray, time_: np.ndarray) -> np.ndarray:
    """Instantaneous speed (m/s) derived from position differences."""
    dt    = np.diff(time_)
    dt    = np.where(dt < 1e-9, 1e-9, dt)
    speed = np.linalg.norm(np.diff(tvec, axis=0), axis=1) / dt
    return np.concatenate([[0.0], speed])


def velocity_speed(vel: np.ndarray) -> np.ndarray:
    """Speed from the logged velocity vector."""
    return np.linalg.norm(vel, axis=1)


def angular_displacement_series(quat: np.ndarray) -> np.ndarray:
    """
    Angular displacement (rad) between consecutive frames.
    Uses the identity: angle = 2*arccos(|q1·q2|) for unit quaternions.
    """
    dots = np.einsum("ij,ij->i", quat[:-1], quat[1:])   # dot per row-pair
    dots = np.clip(np.abs(dots), 0.0, 1.0)
    angles = 2.0 * np.arccos(dots)
    return np.concatenate([[0.0], angles])


def finite_diff(signal: np.ndarray, time_: np.ndarray) -> np.ndarray:
    """First derivative of signal w.r.t. time."""
    dt = np.diff(time_)
    dt = np.where(dt < 1e-9, 1e-9, dt)
    d1 = np.diff(signal) / dt
    return np.concatenate([[0.0], d1])


def total_path_length(tvec: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(tvec, axis=0), axis=1)))


def total_rotation(quat: np.ndarray) -> float:
    """Sum of all inter-frame angular displacements (rad)."""
    return float(np.sum(angular_displacement_series(quat)))


# ── DTW ──────────────────────────────────────────────────────────────────────

def dtw_distance(a: np.ndarray, b: np.ndarray) -> float:
    """
    Classic DTW using squared-Euclidean local cost.
    a, b are (N, D) and (M, D) arrays; D columns are the feature dimensions.
    """
    n, m = len(a), len(b)
    INF  = np.inf
    cost = np.full((n, m), INF)

    cost[0, 0] = float(np.dot(a[0] - b[0], a[0] - b[0]))
    for i in range(1, n):
        d = float(np.dot(a[i] - b[0], a[i] - b[0]))
        cost[i, 0] = cost[i - 1, 0] + d
    for j in range(1, m):
        d = float(np.dot(a[0] - b[j], a[0] - b[j]))
        cost[0, j] = cost[0, j - 1] + d
    for i in range(1, n):
        for j in range(1, m):
            d = float(np.dot(a[i] - b[j], a[i] - b[j]))
            cost[i, j] = d + min(cost[i - 1, j],
                                  cost[i, j - 1],
                                  cost[i - 1, j - 1])
    return float(cost[-1, -1])


def pairwise_mean_dtw(seqs: list) -> float:
    """Mean DTW distance over all pairs (within-gesture repeatability)."""
    pairs = list(combinations(range(len(seqs)), 2))
    if not pairs:
        return float("nan")
    return float(np.mean([dtw_distance(seqs[a], seqs[b]) for a, b in pairs]))


# ── analysis ──────────────────────────────────────────────────────────────────

def analyse():
    trials_map = discover_trials(LOG_DIR)
    if not trials_map:
        print(f"[ERROR] No trial files found in:\n  {LOG_DIR}")
        return

    gesture_stats       = {}                                 # gesture -> list of per-trial dicts
    gesture_tvecs       = defaultdict(list)                  # for DTW
    uid_gesture_data    = defaultdict(lambda: defaultdict(list))  # uid -> gesture -> trials

    print("=" * 70)
    print("Per-trial kinematics")
    print("=" * 70)

    for gesture in sorted(trials_map):
        gesture_stats[gesture] = []
        for uid, path in trials_map[gesture]:
            tvec, quat, time_, vel = load_trial(path)
            if len(tvec) < 4:
                print(f"  [SKIP] {os.path.basename(path)} — too few frames")
                continue

            # normalise time to start at 0
            t = time_ - time_[0]

            # — speed & acceleration (use logged velocity for smoothness) —
            speed  = velocity_speed(vel)
            accel  = finite_diff(speed, t)

            # — angular quantities —
            ang_disp = angular_displacement_series(quat)   # (N,) rad per frame
            ang_vel  = ang_disp / np.where(np.diff(t, prepend=t[0]) < 1e-9,
                                            1e-9,
                                            np.diff(t, prepend=t[0]))

            pk_vel    = float(np.max(speed))
            pk_accel  = float(np.max(np.abs(accel)))
            pk_angvel = float(np.max(ang_vel))
            tot_rot   = float(np.sum(ang_disp))
            tot_disp  = total_path_length(tvec)

            gesture_stats[gesture].append({
                "uid":        uid,
                "peak_vel":   pk_vel,
                "peak_accel": pk_accel,
                "pk_angvel":  pk_angvel,
                "total_rot":  tot_rot,
                "total_disp": tot_disp,
                "tvec":       tvec,
                "t":          t,
            })
            gesture_tvecs[gesture].append(tvec)
            uid_gesture_data[uid][gesture].append({"tvec": tvec, "t": t, "speed": speed})

            print(f"  [{gesture:5s}] uid={uid} | pk_vel={pk_vel:.4f} m/s"
                  f" | pk_acc={pk_accel:.4f} m/s²"
                  f" | pk_angvel={pk_angvel:.4f} rad/s"
                  f" | Δrot={tot_rot:.4f} rad"
                  f" | path={tot_disp:.4f} m")

    # — DTW within each gesture —
    dtw_scores = {g: pairwise_mean_dtw(gesture_tvecs[g])
                  for g in gesture_tvecs}

    # ── markdown summary ──────────────────────────────────────────────────────
    print("\n\n## Statistical Summary\n")
    print("| Gesture | N | PkVel mean (m/s) | PkVel std | "
          "PkAccel mean (m/s²) | PkAccel std | "
          "DTW Repeatability | Mean ΔRot (rad) | Std ΔRot |")
    print("|---------|---|------------------|-----------|"
          "--------------------|-------------|"
          "-------------------|----------------|----------|")

    for gesture in sorted(gesture_stats):
        s    = gesture_stats[gesture]
        pv   = [x["peak_vel"]   for x in s]
        pa   = [x["peak_accel"] for x in s]
        tr   = [x["total_rot"]  for x in s]
        dtw  = dtw_scores.get(gesture, float("nan"))
        n    = len(s)

        def fmt(vals, fn=np.mean):
            return f"{fn(vals):.4f}" if vals else "N/A"

        print(f"| {gesture:7s} | {n} "
              f"| {fmt(pv):16s} | {fmt(pv, np.std):9s} "
              f"| {fmt(pa):18s} | {fmt(pa, np.std):11s} "
              f"| {dtw:17.4f} "
              f"| {fmt(tr):14s} | {fmt(tr, np.std):8s} |")

    # — rotational stability: poke vs push —
    print()
    if "poke" in gesture_stats and "push" in gesture_stats:
        print("### Rotational Stability: poke vs push\n")
        print("| Gesture | Mean ΔRotation (rad) | Std ΔRotation (rad) | N |")
        print("|---------|---------------------|---------------------|---|")
        for g in ("poke", "push"):
            tr = [x["total_rot"] for x in gesture_stats[g]]
            n  = len(tr)
            print(f"| {g:7s} | {np.mean(tr):.4f}               "
                  f"| {np.std(tr):.4f}               | {n} |")
        print()
        avg_poke = np.mean([x["total_rot"] for x in gesture_stats["poke"]])
        avg_push = np.mean([x["total_rot"] for x in gesture_stats["push"]])
        more = "poke" if avg_poke > avg_push else "push"
        print(f"**Result:** '{more}' induces more rotational displacement "
              f"({max(avg_poke, avg_push):.4f} rad vs "
              f"{min(avg_poke, avg_push):.4f} rad).")
        print()

    # ── visualisation setup ───────────────────────────────────────────────────
    all_uids  = sorted(uid_gesture_data.keys())
    gestures  = sorted({g for uid_d in uid_gesture_data.values() for g in uid_d})
    n_gest    = len(gestures)

    # one distinct colour per subject
    cmap      = plt.cm.tab10
    uid_color = {uid: cmap(i / max(len(all_uids) - 1, 1))
                 for i, uid in enumerate(all_uids)}

    # ── Figure 1: XY trajectory (3 forms × subject colours) ──────────────────
    fig1, axes1 = plt.subplots(1, n_gest, figsize=(5 * n_gest, 5),
                                squeeze=False)
    axes1 = axes1[0]

    for col, gesture in enumerate(gestures):
        ax = axes1[col]
        for uid in all_uids:
            colour = uid_color[uid]
            trials = uid_gesture_data[uid].get(gesture, [])
            for i, trial in enumerate(trials):
                tn  = trial["tvec"] - trial["tvec"][0]   # origin-align
                lbl = f"Subject {uid}" if i == 0 else None
                ax.plot(tn[:, 0], tn[:, 1],
                        color=colour, lw=1.8, alpha=0.75, label=lbl)
                ax.plot(tn[0, 0],  tn[0, 1],  "o",
                        color=colour, ms=5, alpha=0.9)
                ax.plot(tn[-1, 0], tn[-1, 1], "x",
                        color=colour, ms=7, mew=2, alpha=0.9)
        ax.set_title(gesture, fontsize=12)
        ax.set_xlabel("ΔX (m)", fontsize=10)
        ax.set_ylabel("ΔY (m)", fontsize=10)
        ax.axhline(0, color="k", lw=0.5, ls="--", alpha=0.4)
        ax.axvline(0, color="k", lw=0.5, ls="--", alpha=0.4)
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, alpha=0.3)

    leg_handles1 = [plt.Line2D([0], [0], color=uid_color[u], lw=2,
                                label=f"Subject {u}") for u in all_uids]
    fig1.legend(handles=leg_handles1, loc="upper center",
                ncol=len(all_uids), fontsize=10,
                title="Subject  (○ start  × end)")
    fig1.suptitle("XY Trajectory — origin-aligned per trial",
                  fontsize=13, y=1.02)
    fig1.tight_layout()
    fig1.savefig(OUT_PNG_TRAJ, dpi=150, bbox_inches="tight")
    print(f"[OK] Trajectory plot saved → {OUT_PNG_TRAJ}")

    # ── Figure 2: velocity vs time (line = mean, band = ±1 std) ──────────────
    fig2, axes2 = plt.subplots(1, n_gest, figsize=(5 * n_gest, 4),
                                squeeze=False)
    axes2 = axes2[0]
    N_GRID = 300   # common time grid resolution

    for col, gesture in enumerate(gestures):
        ax = axes2[col]
        for uid in all_uids:
            colour = uid_color[uid]
            trials = uid_gesture_data[uid].get(gesture, [])
            if not trials:
                continue

            # interpolate every trial onto a shared time grid
            t_end  = min(trial["t"][-1] for trial in trials)
            t_grid = np.linspace(0.0, t_end, N_GRID)
            mat    = np.array([np.interp(t_grid, trial["t"], trial["speed"])
                               for trial in trials])   # (n_trials, N_GRID)

            mean_v = mat.mean(axis=0)
            std_v  = mat.std(axis=0)

            ax.plot(t_grid, mean_v,
                    color=colour, lw=2.0, label=f"Subject {uid}")
            ax.fill_between(t_grid,
                            mean_v - std_v,
                            mean_v + std_v,
                            color=colour, alpha=0.20)

        ax.set_title(gesture, fontsize=12)
        ax.set_xlabel("Time (s)", fontsize=10)
        ax.set_ylabel("Speed (m/s)", fontsize=10)
        ax.grid(True, alpha=0.3)

    leg_handles2 = [plt.Line2D([0], [0], color=uid_color[u], lw=2,
                                label=f"Subject {u}") for u in all_uids]
    fig2.legend(handles=leg_handles2, loc="upper center",
                ncol=len(all_uids), fontsize=10,
                title="Subject  (line = mean, band = ±1 std)")
    fig2.suptitle("Velocity vs Time", fontsize=13, y=1.02)
    fig2.tight_layout()
    fig2.savefig(OUT_PNG_VEL, dpi=150, bbox_inches="tight")
    print(f"[OK] Velocity plot saved → {OUT_PNG_VEL}")


if __name__ == "__main__":
    analyse()
