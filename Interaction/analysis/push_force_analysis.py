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
    Return (times, rolls, vy_list) where:
      - times / vys come from 'frames' entries (vel[1] = Y velocity).
      - rolls come from 'state' / VEL_ORI entries, linearly interpolated
        onto each frame timestamp (constant-rate assumption between samples).
    """
    # ── roll series from VEL_ORI state entries ────────────────────────────────
    roll_times: list[float] = []
    roll_vals:  list[float] = []
    for e in entries:
        if e.get("type") != "state" or e.get("group") != "VEL_ORI":
            continue
        t    = e["data"]["time"]
        roll = e["data"].get("stateEstimate.roll")
        if roll is None:
            continue
        if t_start <= t <= t_end:
            roll_times.append(t)
            roll_vals.append(roll)

    # ── vy series from frames entries ─────────────────────────────────────────
    frame_times: list[float] = []
    vys:         list[float] = []
    for e in entries:
        if e.get("type") != "frames":
            continue
        t   = e["data"]["time"]
        vel = e["data"].get("vel")
        if vel is None or len(vel) < 2:
            continue
        if t_start <= t <= t_end:
            frame_times.append(t)
            vys.append(vel[1])

    if not roll_times or not frame_times:
        return [], [], []

    # ── interpolate roll onto frame timestamps (linear / fixed rate) ──────────
    def lerp_roll(t: float) -> float:
        if t <= roll_times[0]:
            return roll_vals[0]
        if t >= roll_times[-1]:
            return roll_vals[-1]
        lo, hi = 0, len(roll_times) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if roll_times[mid] <= t:
                lo = mid
            else:
                hi = mid
        t0, t1 = roll_times[lo], roll_times[hi]
        alpha = (t - t0) / (t1 - t0) if t1 != t0 else 0.0
        return roll_vals[lo] + alpha * (roll_vals[hi] - roll_vals[lo])

    rolls = [lerp_roll(t) for t in frame_times]
    return frame_times, rolls, vys


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
        sys.exit("ERROR: no frames / VEL_ORI entries found in the push window")

    print(f"  Samples in window: {len(rolls)}")

    try:
        import numpy as np
        import matplotlib
        matplotlib.use("macosx")
        import matplotlib.pyplot as plt
        import matplotlib.ticker as ticker
        from scipy import signal as sp_signal
    except ImportError:
        sys.exit("matplotlib / numpy / scipy not installed — run: pip install matplotlib numpy scipy")

    # Relative time (0 = first push)
    t0    = times_abs[0]
    t_arr = np.array([t - t0 for t in times_abs])
    r_arr = np.array(rolls)
    vy_arr = np.array(vys)

    # Force: F_y = m · g · tan(roll)  [N]
    f_arr = M_LB * G * np.tan(np.radians(r_arr))

    # Acceleration: a_y = d(vy)/dt  [m/s²]  — central differences via np.gradient
    a_arr = np.gradient(vy_arr, t_arr)

    # F / a  [kg] — effective inertia; NaN where |a| is too small to be reliable
    _ACC_MIN = 0.1  # m/s²  — threshold below which ratio is suppressed
    fa_ratio = np.where(np.abs(a_arr) >= _ACC_MIN, f_arr / a_arr, np.nan)

    # ── Low-pass filter (zero-phase Butterworth) ──────────────────────────────
    _CUTOFF_HZ = 2.0   # steady-state cutoff  [Hz]
    _FILT_ORDER = 4
    _fs = 1.0 / float(np.mean(np.diff(t_arr)))   # estimated sample rate  [Hz]
    _nyq = _fs / 2.0
    if _CUTOFF_HZ >= _nyq:
        # cutoff above Nyquist — skip filtering (shouldn't happen normally)
        f_filt = f_arr.copy()
        a_filt = a_arr.copy()
    else:
        b_lp, a_lp = sp_signal.butter(_FILT_ORDER, _CUTOFF_HZ / _nyq, btype="low")
        f_filt = sp_signal.filtfilt(b_lp, a_lp, f_arr)
        a_filt = sp_signal.filtfilt(b_lp, a_lp, a_arr)

    # ── OLS regression: Fy = m_eff · ay + bias  (positive quadrant only) ──────
    pos_mask = (a_filt > 0) & (f_filt > 0)
    a_pos = a_filt[pos_mask]
    f_pos = f_filt[pos_mask]
    if a_pos.size < 2:
        m_eff, ols_bias, r_sq = float("nan"), float("nan"), float("nan")
    else:
        A_mat = np.column_stack([a_pos, np.ones_like(a_pos)])
        (m_eff, ols_bias), *_ = np.linalg.lstsq(A_mat, f_pos, rcond=None)
        f_pred = m_eff * a_pos + ols_bias
        ss_res = float(np.sum((f_pos - f_pred) ** 2))
        ss_tot = float(np.sum((f_pos - f_pos.mean()) ** 2))
        r_sq   = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

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
    print(f"\n  OLS regression  Fy = m_eff·ay + bias  (filtered, {_CUTOFF_HZ} Hz LP):")
    print(f"    m_eff (effective mass) : {m_eff:+.4f} kg")
    print(f"    bias (intercept)       : {ols_bias:+.4f} N")
    print(f"    R²                     : {r_sq:.4f}")

    # ── Figure 1: time-series (4 panels) ─────────────────────────────────────
    fig1, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    fig1.suptitle(
        f"Interaction push analysis — time series\n{label} g",
        fontsize=11,
    )

    # Panel 1: roll
    ax_r = axes[0]
    ax_r.plot(t_arr, r_arr, color="steelblue", linewidth=1.2, label="roll (°)")
    ax_r.axhline(0, color="gray", linestyle="--", linewidth=0.9, label="0°")
    ax_r.set_ylabel("Roll (°)")
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

    # ── Figure 2: force vs acceleration  (positive quadrant, filtered + OLS) ──
    fig2, ax_fa = plt.subplots(figsize=(8, 6))
    fig2.suptitle(
        f"Force vs. Acceleration (Y axis, ay > 0 & Fy > 0)\n{label} g",
        fontsize=11,
    )

    # Raw telemetry — positive quadrant only, faint scatter coloured by time
    raw_mask = (a_arr > 0) & (f_arr > 0)
    t_raw_pos = t_arr[raw_mask]
    sc = ax_fa.scatter(a_arr[raw_mask], f_arr[raw_mask], c=t_raw_pos,
                       cmap="plasma", s=18, alpha=0.35, zorder=2,
                       label="raw samples (ay>0, Fy>0)")
    cbar = fig2.colorbar(sc, ax=ax_fa)
    cbar.set_label("Time since push (s)")

    # Filtered steady-state trajectory — positive quadrant only
    ax_fa.scatter(a_pos, f_pos, color="steelblue", s=22, alpha=0.7, zorder=3,
                  label=f"filtered ({_CUTOFF_HZ} Hz LP, order {_FILT_ORDER})")
    ax_fa.plot(a_pos, f_pos, color="steelblue", linewidth=1.2,
               alpha=0.5, zorder=3)

    # OLS regression line spanning the positive a_y range
    if not np.isnan(m_eff) and a_pos.size >= 2:
        _a_span = np.linspace(0.0, float(a_pos.max()), 200)
        _f_span = m_eff * _a_span + ols_bias
        ax_fa.plot(_a_span, _f_span, color="crimson", linewidth=2.0,
                   linestyle="--", zorder=4,
                   label=(rf"OLS: $F_y = {m_eff:+.3f}\,a_y {ols_bias:+.3f}$"
                          f"\n$R^2={r_sq:.3f}$"))

    ax_fa.set_xlim(0, 3.5)
    ax_fa.set_ylim(0, 0.5)
    ax_fa.set_xlabel(r"$a_y$  (m/s²)")
    ax_fa.set_ylabel(r"$F_y = m_{lb}\,g\,\tan(\phi)$  (N)")
    ax_fa.set_title(
        f"Effective mass: {m_eff:+.3f} kg   bias: {ols_bias:+.3f} N   "
        f"$R^2$={r_sq:.3f}"
    )
    ax_fa.legend(fontsize=8, loc="best")
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

    main(str(_project_root / 'logs' / 'lb11_translation_2026-04-15_19-57-39.json'), label="170", show_plot=False)
    main(str(_project_root / 'logs' / 'lb11_translation_2026-04-15_19-54-39.json'), label="300", show_plot=False)


# ── Example ───────────────────────────────────────────────────────────────────
# Run from project root:
#   python Interaction/analysis/push_force_analysis.py
#
# Run from any directory (path resolves via __file__):
#   python /path/to/Interaction/analysis/push_force_analysis.py
