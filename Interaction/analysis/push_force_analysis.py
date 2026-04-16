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
import os
import sys
from pathlib import Path

# ── constants ─────────────────────────────────────────────────────────────────
M_LB  = 0.170      # LightBender mass  [kg]
G     = 9.81       # gravitational acceleration  [m/s²]

DEFAULT_BASELINE_ROLL = 0 # hover roll  [degrees]


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


def extract_window(entries, t_start: float, t_end: float):
    """
    Return (times, rolls, vy_list) for all VEL_ORI state entries whose
    timestamp falls in [t_start, t_end].
    """
    times = []
    rolls = []
    vys   = []
    for e in entries:
        if e.get("type") != "state" or e.get("group") != "VEL_ORI":
            continue
        t    = e["data"]["time"]
        roll = e["data"].get("stateEstimate.roll")
        vy   = e["data"].get("stateEstimate.vy")
        if roll is None or vy is None:
            continue
        if t_start <= t <= t_end:
            times.append(t)
            rolls.append(roll)
            vys.append(vy)
    return times, rolls, vys


# ── main ──────────────────────────────────────────────────────────────────────
def main(logfile, label=None, show_plot=False):
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

    # ── extract roll + vy ────────────────────────────────────────────────────
    times_abs, rolls, vys = extract_window(entries, t_push, t_disengage)

    if not times_abs:
        sys.exit("ERROR: no VEL_ORI state entries found in the push window")

    print(f"  Samples in window: {len(rolls)}")

    try:
        import numpy as np
        import matplotlib
        matplotlib.use("macosx")
        import matplotlib.pyplot as plt
        import matplotlib.ticker as ticker
    except ImportError:
        sys.exit("matplotlib / numpy not installed — run: pip install matplotlib numpy")

    # Relative time (0 = first push)
    t0    = times_abs[0]
    t_arr = np.array([t - t0 for t in times_abs])
    r_arr = np.array(rolls)
    vy_arr = np.array(vys)

    # Force: F_y = m · g · tan(roll)  [N]
    f_arr = M_LB * G * np.tan(np.radians(-r_arr))

    # Acceleration: a_y = d(vy)/dt  [m/s²]  — central differences via np.gradient
    a_arr = np.gradient(vy_arr, t_arr)

    # F / a  [kg] — effective inertia; NaN where |a| is too small to be reliable
    _ACC_MIN = 0.1   # m/s²  — threshold below which ratio is suppressed
    fa_ratio = np.where(np.abs(a_arr) >= _ACC_MIN, f_arr / a_arr, np.nan)

    # ── Stats ─────────────────────────────────────────────────────────────────
    print(f"\n  a_y statistics (m/s²):")
    print(f"    min : {a_arr.min():+.4f}")
    print(f"    max : {a_arr.max():+.4f}")
    print(f"    mean: {a_arr.mean():+.4f}")
    valid = fa_ratio[~np.isnan(fa_ratio)]
    if valid.size:
        print(f"\n  F/a statistics (kg, |a|≥{_ACC_MIN} m/s²):")
        print(f"    min : {valid.min():+.4f}")
        print(f"    max : {valid.max():+.4f}")
        print(f"    mean: {valid.mean():+.4f}")

    # ── Figure 1: time-series (4 panels) ─────────────────────────────────────
    fig1, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    fig1.suptitle(
        f"Interaction push analysis — time series\n{os.path.basename(logfile)}",
        fontsize=11,
    )

    # Panel 1: roll
    ax_r = axes[0]
    ax_r.plot(t_arr, r_arr, color="steelblue", linewidth=1.2, label="roll (°)")
    ax_r.axhline(0, color="gray", linestyle="--", linewidth=0.9, label="0°")
    ax_r.set_ylabel("Roll (°)")
    ax_r.legend(fontsize=8)
    ax_r.grid(True, alpha=0.35)

    # Panel 2: force
    ax_f = axes[1]
    ax_f.plot(t_arr, f_arr, color="crimson", linewidth=1.4,
              label=r"$F_y = m_{lb}\,g\,\tan(\phi)$  [N]")
    ax_f.axhline(0, color="black", linewidth=0.7)
    ax_f.fill_between(t_arr, f_arr, 0, where=f_arr > 0,
                      alpha=0.18, color="crimson", label="+F (−Y push)")
    ax_f.fill_between(t_arr, f_arr, 0, where=f_arr < 0,
                      alpha=0.18, color="royalblue", label="−F (+Y push)")
    peak_idx = int(np.argmax(np.abs(f_arr)))
    peak_f, peak_t = f_arr[peak_idx], t_arr[peak_idx]
    f_range  = float(f_arr.max() - f_arr.min()) or 1.0
    text_y   = peak_f + 0.15 * f_range * (1 if peak_f >= 0 else -1)
    ax_f.annotate(f"peak: {peak_f:+.3f} N",
                  xy=(peak_t, peak_f), xytext=(peak_t + 0.05, text_y),
                  arrowprops=dict(arrowstyle="->", color="black", lw=0.8), fontsize=8)
    ax_f.set_ylabel("Force along Y  (N)")
    ax_f.legend(fontsize=8, loc="lower right")
    ax_f.grid(True, alpha=0.35)
    ax_f.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.3f"))

    # Panel 3: acceleration
    ax_a = axes[2]
    ax_a.plot(t_arr, a_arr, color="darkorange", linewidth=1.2,
              label=r"$a_y = \Delta v_y / \Delta t$  [m/s²]")
    ax_a.axhline(0, color="black", linewidth=0.7)
    ax_a.set_ylabel("Acceleration Y  (m/s²)")
    ax_a.annotate(f"min: {a_arr.min():+.3f}  max: {a_arr.max():+.3f} m/s²",
                  xy=(0.02, 0.05), xycoords="axes fraction", fontsize=8,
                  color="darkorange")
    ax_a.legend(fontsize=8)
    ax_a.grid(True, alpha=0.35)

    # Panel 4: F / a  (effective inertia)
    ax_fa_t = axes[3]
    ax_fa_t.plot(t_arr, fa_ratio, color="purple", linewidth=1.2,
                 label=r"$F_y / a_y$  [kg]")
    ax_fa_t.axhline(0, color="black", linewidth=0.7)
    ax_fa_t.axhline(M_LB, color="gray", linewidth=0.9, linestyle="--",
                    label=f"drone mass = {M_LB*1000:.0f} g")
    if valid.size:
        ax_fa_t.annotate(
            f"mean: {valid.mean():+.3f} kg   min: {valid.min():+.3f}   max: {valid.max():+.3f}",
            xy=(0.02, 0.05), xycoords="axes fraction", fontsize=8, color="purple")
    ax_fa_t.set_xlabel("Time since first push (s)")
    ax_fa_t.set_ylabel("F / a  (kg)")
    ax_fa_t.set_title(f"Effective inertia  (NaN where |a| < {_ACC_MIN} m/s²)")
    ax_fa_t.legend(fontsize=8)
    ax_fa_t.grid(True, alpha=0.35)

    fig1.tight_layout()
    log_p     = Path(logfile)
    suffix    = f"_{label}" if label else ""
    out_ts    = str(log_p.parent / (log_p.stem + suffix + "_force_time.png"))

    if not show_plot:
        fig1.savefig(out_ts, dpi=150)
        print(f"Time-series plot saved → {out_ts}")

    # ── Figure 2: force vs acceleration ──────────────────────────────────────
    fig2, ax_fa = plt.subplots(figsize=(7, 6))
    fig2.suptitle(
        f"Force vs. Acceleration (Y axis)\n{os.path.basename(logfile)}",
        fontsize=11,
    )

    sc = ax_fa.scatter(a_arr, f_arr, c=t_arr, cmap="plasma",
                       s=30, zorder=3, label="samples")
    cbar = fig2.colorbar(sc, ax=ax_fa)
    cbar.set_label("Time since push (s)")

    # Connect dots in time order so the trajectory is visible
    ax_fa.plot(a_arr, f_arr, color="gray", linewidth=0.7, alpha=0.5, zorder=2)

    ax_fa.axhline(0, color="black", linewidth=0.6, linestyle="--")
    ax_fa.axvline(0, color="black", linewidth=0.6, linestyle="--")
    ax_fa.set_xlabel(r"$a_y = \Delta v_y / \Delta t$  (m/s²)")
    ax_fa.set_ylabel(r"$F_y = m_{lb}\,g\,\tan(\phi)$  (N)")
    ax_fa.set_title("Each dot = one telemetry sample, colour = elapsed time")
    ax_fa.grid(True, alpha=0.35)

    fig2.tight_layout()
    out_fa = str(log_p.parent / (log_p.stem + suffix + "_force_vs_acc.png"))

    if show_plot:
        plt.show()
    else:
        fig2.savefig(out_fa, dpi=150)
        print(f"Force-vs-accel plot saved → {out_fa}")


if __name__ == "__main__":
    _project_root = Path(__file__).resolve().parents[2]
    # main(str(_project_root / 'logs' / 'mass_emulation' / 'lb11_translation_2026-04-15_17-54-28.json'), label="300g", show_plot=False)
    # main(str(_project_root / 'logs' / 'mass_emulation' / 'lb11_translation_2026-04-15_17-55-55.json'), label="170g", show_plot=False)


    # main(str(_project_root / 'logs' / 'lb11_translation_2026-04-15_18-23-49.json'), label="170g", show_plot=False)
    # main(str(_project_root / 'logs' / 'lb11_translation_2026-04-15_18-20-48.json'), label="300g", show_plot=False)


    main(str(_project_root / 'logs' / 'lb11_translation_2026-04-15_18-38-30.json'), label="170g", show_plot=False)
    # main(str(_project_root / 'logs' / 'lb11_translation_2026-04-15_18-34-57.json'), label="300g", show_plot=False)

# ── Example ───────────────────────────────────────────────────────────────────
# Run from project root:
#   python Interaction/analysis/push_force_analysis.py
#
# Run from any directory (path resolves via __file__):
#   python /path/to/Interaction/analysis/push_force_analysis.py
