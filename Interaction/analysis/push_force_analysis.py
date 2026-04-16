"""
push_force_analysis.py

Analyses a translation interaction log and plots the force exerted along the
negative-Y axis as a function of time, covering the window from the first
"User Pushing" event until the first "User Disengage" event.

Force model
───────────
  The drone's roll angle encodes the horizontal force along Y:

      F_y  =  m_lb · g · tan(roll)          [Newtons]

  where
    m_lb  = 170 g  = 0.170 kg  (LightBender mass)
    g     = 9.81 m/s²
    roll  = stateEstimate.roll at each timestep  [degrees]

  With Crazyflie sign convention a negative roll tilts the drone toward −Y,
  so F_y is negative when the user pushes in the −Y direction.
  The plot shows  −F_y  (force along **negative** Y) so a real push gives a
  positive curve.

Usage
─────
  Run from the project root:
    python Interaction/analysis/push_force_analysis.py

  Or from any directory (path is resolved relative to this script):
    python /path/to/Interaction/analysis/push_force_analysis.py
"""

import json
import math
import os
import sys
from pathlib import Path

# ── constants ─────────────────────────────────────────────────────────────────
M_LB  = 0.170      # LightBender mass  [kg]
G     = 9.81       # gravitational acceleration  [m/s²]

DEFAULT_BASELINE_ROLL = -3.7721259593963623  # hover roll  [degrees]


# ── helpers ───────────────────────────────────────────────────────────────────
def load_log(path: str):
    """Load the JSONL-style log (one JSON object per line)."""
    entries = []
    with open(path) as fh:
        for line in fh:
            line = line.strip().rstrip(",")
            if not line or line in ("[", "]"):
                continue
            try:
                entries.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return entries


def find_event_time(entries, name: str) -> float | None:
    """Return the `time` field of the first event whose name matches *name*."""
    for e in entries:
        if e.get("type") == "events" and e.get("name") == name:
            return e["data"]["time"]
    return None


def extract_roll_in_window(entries, t_start: float, t_end: float):
    """
    Return (times, rolls) for all VEL_ORI state entries whose timestamp falls
    in [t_start, t_end].
    """
    times = []
    rolls = []
    for e in entries:
        if e.get("type") != "state" or e.get("group") != "VEL_ORI":
            continue
        t    = e["data"]["time"]
        roll = e["data"].get("stateEstimate.roll")
        if roll is None:
            continue
        if t_start <= t <= t_end:
            times.append(t)
            rolls.append(roll)
    return times, rolls


# ── main ──────────────────────────────────────────────────────────────────────
def main(logfile):
    # ── load ─────────────────────────────────────────────────────────────────
    print(f"Loading  {logfile} …")
    entries = load_log(logfile)
    print(f"  {len(entries):,} entries loaded")

    # ── find window ───────────────────────────────────────────────────────────
    t_push     = find_event_time(entries, "User Pushing")
    t_disengage = find_event_time(entries, "User Disengage")

    if t_push is None:
        sys.exit("ERROR: no 'User Pushing' event found in log")
    if t_disengage is None:
        sys.exit("ERROR: no 'User Disengage' event found in log")

    print(f"  First push      : t = {t_push:.3f}  (absolute)")
    print(f"  First disengage : t = {t_disengage:.3f}  (absolute)")
    print(f"  Window duration : {t_disengage - t_push:.3f} s")

    # ── extract roll ──────────────────────────────────────────────────────────
    times_abs, rolls = extract_roll_in_window(entries, t_push, t_disengage)

    if not times_abs:
        sys.exit("ERROR: no VEL_ORI state entries found in the push window")

    print(f"  Roll samples in window: {len(rolls)}")

    # Relative time (0 = first push)
    t0   = times_abs[0]
    times = [t - t0 for t in times_abs]

    baseline_rad = math.radians(0)

    # F_y = m · g · tan(roll - baseline)   [N]
    # Plot −F_y so that pushing in −Y gives a positive value
    forces = [
        M_LB * G * math.tan(math.radians(r) - baseline_rad)
        for r in rolls
    ]

    # ── plot ──────────────────────────────────────────────────────────────────
    try:
        import numpy as np
        import matplotlib
        matplotlib.use("macosx")
        import matplotlib.pyplot as plt
        import matplotlib.ticker as ticker
    except ImportError:
        sys.exit("matplotlib / numpy not installed — run: pip install matplotlib numpy")

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    fig.suptitle(
        f"Interaction push analysis\n"
        f"{os.path.basename(logfile)}",
        fontsize=11,
    )

    # ── top panel: roll ───────────────────────────────────────────────────────
    ax_r = axes[0]
    ax_r.plot(times, rolls, color="steelblue", linewidth=1.2, label="roll (deg)")
    ax_r.axhline(0, color="gray", linestyle="--",
                 linewidth=0.9, label=f"baseline = {0:.4f}°")
    ax_r.set_ylabel("Roll (°)")
    ax_r.set_title("Measured roll — push window")
    ax_r.legend(fontsize=8)
    ax_r.grid(True, alpha=0.35)

    # ── bottom panel: force ───────────────────────────────────────────────────
    ax_f = axes[1]
    ax_f.plot(times, forces, color="crimson", linewidth=1.4,
              label=r"$-F_y = m_{lb}\,g\,\tan(\Delta\phi)$  [N]")
    ax_f.axhline(0, color="black", linewidth=0.7, linestyle="-")

    t_arr = np.array(times)
    f_arr = np.array(forces)

    # Shade positive region (user pushing in −Y)
    ax_f.fill_between(t_arr, f_arr, 0,
                      where=f_arr > 0,
                      alpha=0.18, color="crimson", label="positive push (−Y)")
    ax_f.fill_between(t_arr, f_arr, 0,
                      where=f_arr < 0,
                      alpha=0.18, color="royalblue", label="push (+Y)")

    peak_idx = int(np.argmax(np.abs(f_arr)))
    peak_f   = f_arr[peak_idx]
    peak_t   = t_arr[peak_idx]
    y_range  = f_arr.max() - f_arr.min() if f_arr.max() != f_arr.min() else 1.0
    text_y   = peak_f + 0.15 * y_range * (1 if peak_f >= 0 else -1)
    ax_f.annotate(
        f"peak: {peak_f:+.3f} N",
        xy=(peak_t, peak_f),
        xytext=(peak_t + 0.05, text_y),
        arrowprops=dict(arrowstyle="->", color="black", lw=0.8),
        fontsize=8,
    )

    ax_f.set_xlabel("Time since first push (s)")
    ax_f.set_ylabel("Force along −Y  (N)")
    ax_f.set_title(
        f"Force along −Y  (m = {M_LB*1000:.0f} g, g = {G} m/s²,  "
        f"baseline roll = {0:.4f}°)"
    )
    ax_f.legend(fontsize=8, loc="lower right")
    ax_f.grid(True, alpha=0.35)
    ax_f.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.3f"))

    plt.tight_layout()

    log_p    = Path(logfile)
    out_path = str(log_p.parent / (log_p.stem + "_force.png"))

    fig.savefig(out_path, dpi=150)
    print(f"Plot saved → {out_path}")

    plt.show()


if __name__ == "__main__":
    _project_root = Path(__file__).resolve().parents[2]
    logfile = str(_project_root / 'logs' / 'lb11_translation_2026-04-15_17-11-12.json')

    logfile = str(_project_root / 'logs' / 'lb11_translation_2026-04-15_17-16-09.json')
    main(logfile)


# ── Example ───────────────────────────────────────────────────────────────────
# Run from project root:
#   python Interaction/analysis/push_force_analysis.py
#
# Run from any directory (path resolves via __file__):
#   python /path/to/Interaction/analysis/push_force_analysis.py
