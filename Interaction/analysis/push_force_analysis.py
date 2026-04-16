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
import sys
from pathlib import Path

# ── constants ─────────────────────────────────────────────────────────────────
M_LB  = 0.170      # LightBender mass  [kg]
G     = 9.81       # gravitational acceleration  [m/s²]

DEFAULT_BASELINE_ROLL = 0 # hover roll  [degrees]


# ── helpers ───────────────────────────────────────────────────────────────────
def load_log(path: str):
    """Load the log file as a JSON array."""
    with open(path) as fh:
        return json.load(fh)


def get_mass_ratio(entries) -> float | None:
    """Return the mass_ratio from the Translation Config entry, or None."""
    for e in entries:
        if e.get("type") == "configs" and e.get("name") == "Translation Config":
            return e["data"].get("mass_ratio")
    return None


def find_all_event_times(entries, name: str) -> list[float]:
    """Return all `time` values for events whose name matches *name*, in order."""
    return [
        e["data"]["time"]
        for e in entries
        if e.get("type") == "events" and e.get("name") == name
    ]


def find_longest_push_window(entries) -> tuple[float, float] | tuple[None, None]:
    """
    Pair each 'User Pushing' event with the next 'User Disengage' that follows it,
    then return the (t_push, t_disengage) pair with the longest duration.
    """
    push_times     = find_all_event_times(entries, "User Pushing")
    disengage_times = find_all_event_times(entries, "User Disengage")

    if not push_times or not disengage_times:
        return None, None

    best_push, best_dis, best_dur = None, None, -1.0
    for t_p in push_times:
        # find the first disengage that comes after this push
        for t_d in disengage_times:
            if t_d > t_p:
                dur = t_d - t_p
                if dur > best_dur:
                    best_dur, best_push, best_dis = dur, t_p, t_d
                break  # only pair with the immediately following disengage

    return best_push, best_dis


def extract_window(entries, t_start: float, t_end: float):
    """
    Return (times, rolls, pitches, vxs, vys) all aligned to frame timestamps:
      - rolls / pitches interpolated from VEL_ORI state entries.
      - vxs / vys are world-frame velocities from frames entries.
    """
    # ── roll + pitch from VEL_ORI ─────────────────────────────────────────────
    state_times: list[float] = []
    roll_vals:   list[float] = []
    pitch_vals:  list[float] = []
    for e in entries:
        if e.get("type") != "state" or e.get("group") != "VEL_ORI":
            continue
        t     = e["data"]["time"]
        roll  = e["data"].get("stateEstimate.roll")
        pitch = e["data"].get("stateEstimate.pitch")
        if roll is None or pitch is None:
            continue
        if t_start <= t <= t_end:
            state_times.append(t)
            roll_vals.append(roll)
            pitch_vals.append(pitch)

    # ── vx + vy from frames ───────────────────────────────────────────────────
    frame_times: list[float] = []
    vxs:         list[float] = []
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
            vxs.append(vel[0])
            vys.append(vel[1])

    if not state_times or not frame_times:
        return [], [], [], [], []

    # ── generic linear interpolator ───────────────────────────────────────────
    def lerp(times: list, vals: list, t: float) -> float:
        if t <= times[0]:
            return vals[0]
        if t >= times[-1]:
            return vals[-1]
        lo, hi = 0, len(times) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if times[mid] <= t:
                lo = mid
            else:
                hi = mid
        t0, t1 = times[lo], times[hi]
        alpha = (t - t0) / (t1 - t0) if t1 != t0 else 0.0
        return vals[lo] + alpha * (vals[hi] - vals[lo])

    rolls   = [lerp(state_times, roll_vals,  t) for t in frame_times]
    pitches = [lerp(state_times, pitch_vals, t) for t in frame_times]

    return frame_times, rolls, pitches, vxs, vys


# ── main ──────────────────────────────────────────────────────────────────────
def main(logfile, show_plot=False):
    # ── load ─────────────────────────────────────────────────────────────────
    print(f"Loading  {logfile} …")
    entries = load_log(logfile)
    print(f"  {len(entries):,} entries loaded")

    # ── derive label from mass_ratio ─────────────────────────────────────────
    mass_ratio = get_mass_ratio(entries)
    if mass_ratio is None:
        label = "unknown"
        print("  WARNING: no mass_ratio found in log — label set to 'unknown'")
    else:
        label = str(int(round(170/mass_ratio)))
        print(f"  mass_ratio = {mass_ratio:.4f}  →  label = {label} g")

    # ── find longest push window ──────────────────────────────────────────────
    t_push, t_disengage = find_longest_push_window(entries)

    if t_push is None:
        print("  SKIP: no matching 'User Pushing' / 'User Disengage' pair found")
        return

    _TRIM_END_S = 0.15  # trim this many seconds before disengage to drop release spike
    t_end = t_disengage - _TRIM_END_S

    print(f"  Longest push    : t = {t_push:.3f}  (absolute)")
    print(f"  Disengage       : t = {t_disengage:.3f}  (absolute, trimmed by {_TRIM_END_S*1000:.0f} ms)")
    print(f"  Window duration : {t_end - t_push:.3f} s")

    # ── extract roll, pitch, velocity ────────────────────────────────────────
    times_abs, rolls, pitches, vxs, vys = extract_window(entries, t_push, t_end)

    if not times_abs:
        print("  SKIP: no frames / VEL_ORI entries found in the push window")
        return

    print(f"  Samples in window: {len(rolls)}")

    _MIN_SAMPLES = 20
    if len(rolls) < _MIN_SAMPLES:
        print(f"  SKIP: only {len(rolls)} samples in window (need ≥ {_MIN_SAMPLES})")
        return

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
    p_arr = np.array(pitches)
    vx_arr = np.array(vxs)
    vy_arr = np.array(vys)

    # Force vector from tilt angles  [N]
    #   Fx = m·g·tan(pitch),  Fy = m·g·tan(roll)
    fx_arr = M_LB * G * np.tan(np.radians(p_arr))
    fy_arr = M_LB * G * np.tan(np.radians(r_arr))

    # Acceleration vector from velocity  [m/s²]
    ax_arr = np.gradient(vx_arr, t_arr)
    ay_arr = np.gradient(vy_arr, t_arr)
    a_mag  = np.sqrt(ax_arr**2 + ay_arr**2)

    # Scalar: force opposing the acceleration direction
    #   F_against = F⃗ · (−â) = −(Fx·ax + Fy·ay) / |a|
    _ACC_MIN = 0.0  # m/s² — suppress ratio below this magnitude
    with np.errstate(invalid="ignore", divide="ignore"):
        f_arr  = np.where(a_mag >= _ACC_MIN,
                          -(fx_arr * ax_arr + fy_arr * ay_arr) / a_mag,
                          np.nan)
    a_arr = a_mag  # scalar magnitude for the ratio

    # F/a ratio — NaN suppressed where |a| is too small
    _ACC_MIN = 0.1  # m/s² — suppress ratio below this magnitude
    fa_ratio = np.where(a_mag >= _ACC_MIN, f_arr / a_mag, np.nan)

    # ── Low-pass filter on components (zero-phase Butterworth) ───────────────
    _CUTOFF_HZ  = 5.0
    _FILT_ORDER = 4
    _fs  = 1.0 / float(np.mean(np.diff(t_arr)))
    _nyq = _fs / 2.0
    if _CUTOFF_HZ >= _nyq:
        fx_filt = fx_arr.copy(); fy_filt = fy_arr.copy()
        ax_filt = ax_arr.copy(); ay_filt = ay_arr.copy()
    else:
        b_lp, a_lp = sp_signal.butter(_FILT_ORDER, _CUTOFF_HZ / _nyq, btype="low")
        fx_filt = sp_signal.filtfilt(b_lp, a_lp, fx_arr)
        fy_filt = sp_signal.filtfilt(b_lp, a_lp, fy_arr)
        ax_filt = sp_signal.filtfilt(b_lp, a_lp, ax_arr)
        ay_filt = sp_signal.filtfilt(b_lp, a_lp, ay_arr)

    a_mag_filt  = np.sqrt(ax_filt**2 + ay_filt**2)
    a_safe_filt = np.where(a_mag_filt > 0, a_mag_filt, 1.0)
    f_filt      = -(fx_filt * ax_filt + fy_filt * ay_filt) / a_safe_filt

    # ── OLS: F_against = m_eff · |a| + bias  (F>0 region only) ──────────────
    pos_mask = (a_mag_filt >= _ACC_MIN) & (f_filt > 0)
    a_pos = a_mag_filt[pos_mask]
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
    print(f"\n  |a| statistics (m/s²):")
    print(f"    min : {a_arr.min():+.4f}")
    print(f"    max : {a_arr.max():+.4f}")
    print(f"    mean: {a_arr.mean():+.4f}")
    valid = fa_ratio[~np.isnan(fa_ratio)]
    if valid.size:
        print(f"\n  F_against/|a| statistics (g, |a|≥{_ACC_MIN} m/s²):")
        print(f"    min : {valid.min():+.4f}")
        print(f"    max : {valid.max():+.4f}")
        print(f"    mean: {valid.mean():+.4f}")
    print(f"\n  OLS  F_against = m_eff·|a| + bias  (filtered {_CUTOFF_HZ} Hz LP, F>0):")
    print(f"    m_eff (effective mass) : {m_eff*1000:+.4f} g")
    print(f"    bias (intercept)       : {ols_bias:+.4f} N")
    print(f"    R²                     : {r_sq:.4f}")

    # ── Figure 1: time-series (4 panels) ─────────────────────────────────────
    fig1, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    fig1.suptitle(
        f"Interaction push analysis — time series\n{label} g",
        fontsize=11,
    )

    # Panel 1: pitch + roll
    ax_r = axes[0]
    ax_r.plot(t_arr, p_arr, color="steelblue", linewidth=1.2, label="pitch (°)")
    ax_r.plot(t_arr, r_arr, color="darkcyan",  linewidth=1.2, label="roll (°)")
    ax_r.axhline(0, color="gray", linestyle="--", linewidth=0.9)
    ax_r.set_ylabel("Angle (°)")
    ax_r.legend(fontsize=8)
    ax_r.grid(True, alpha=0.35)

    # Panel 2: force opposing acceleration direction
    ax_f = axes[1]
    ax_f.plot(t_arr, f_arr, color="crimson", linewidth=1.4,
              label=r"$F_{against} = -\vec{F}\cdot\hat{a}$  [N]")
    ax_f.axhline(0, color="black", linewidth=0.7)
    ax_f.fill_between(t_arr, f_arr, 0, where=f_arr > 0,
                      alpha=0.18, color="crimson", label="+F (resisting push)")
    ax_f.fill_between(t_arr, f_arr, 0, where=f_arr < 0,
                      alpha=0.18, color="royalblue", label="−F (aiding push)")
    peak_idx = int(np.argmax(np.abs(f_arr)))
    peak_f, peak_t = f_arr[peak_idx], t_arr[peak_idx]
    f_range  = float(f_arr.max() - f_arr.min()) or 1.0
    text_y   = peak_f + 0.15 * f_range * (1 if peak_f >= 0 else -1)
    ax_f.annotate(f"peak: {peak_f:+.3f} N",
                  xy=(peak_t, peak_f), xytext=(peak_t + 0.05, text_y),
                  arrowprops=dict(arrowstyle="->", color="black", lw=0.8), fontsize=8)
    ax_f.set_ylabel("Force against push direction (N)")
    ax_f.legend(fontsize=8, loc="lower right")
    ax_f.grid(True, alpha=0.35)
    ax_f.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.3f"))

    # Panel 3: acceleration magnitude
    ax_a = axes[2]
    ax_a.plot(t_arr, a_arr, color="darkorange", linewidth=1.2,
              label=r"$|a| = \sqrt{a_x^2 + a_y^2}$  [m/s²]")
    ax_a.axhline(0, color="black", linewidth=0.7)
    ax_a.set_ylabel("|acceleration|  (m/s²)")
    ax_a.annotate(f"min: {a_arr.min():.3f}  max: {a_arr.max():.3f} m/s²",
                  xy=(0.02, 0.05), xycoords="axes fraction", fontsize=8,
                  color="darkorange")
    ax_a.legend(fontsize=8)
    ax_a.grid(True, alpha=0.35)

    # Panel 4: F_against / |a|  (effective inertia)
    ax_fa_t = axes[3]
    ax_fa_t.plot(t_arr, fa_ratio, color="purple", linewidth=1.2,
                 label=r"$F_{against} / |a|$  [g]")
    ax_fa_t.axhline(0, color="black", linewidth=0.7)
    ax_fa_t.axhline(M_LB, color="gray", linewidth=0.9, linestyle="--",
                    label=f"drone mass = {M_LB*1000:.0f} g")
    if valid.size:
        ax_fa_t.annotate(
            f"mean: {valid.mean()*1000:+.3f} g   min: {valid.min()*1000:+.3f}   max: {valid.max()*1000:+.3f}",
            xy=(0.02, 0.05), xycoords="axes fraction", fontsize=8, color="purple")
    ax_fa_t.set_xlabel("Time since first push (s)")
    ax_fa_t.set_ylabel("F / |a|  (g)")
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
        f"Force against push vs. |acceleration|  (F>0 region)\n{label} g",
        fontsize=11,
    )

    # Raw — F_against > 0, coloured by time
    raw_mask  = (a_arr >= _ACC_MIN) & (f_arr > 0)
    t_raw_pos = t_arr[raw_mask]
    sc = ax_fa.scatter(a_arr[raw_mask], f_arr[raw_mask], c=t_raw_pos,
                       cmap="plasma", s=18, alpha=0.35, zorder=2,
                       label=r"raw  ($F_{against}>0$)")
    cbar = fig2.colorbar(sc, ax=ax_fa)
    cbar.set_label("Time since push (s)")

    # Filtered trajectory
    ax_fa.scatter(a_pos, f_pos, color="steelblue", s=22, alpha=0.7, zorder=3,
                  label=f"filtered ({_CUTOFF_HZ} Hz LP, order {_FILT_ORDER})")
    ax_fa.plot(a_pos, f_pos, color="steelblue", linewidth=1.2, alpha=0.5, zorder=3)

    # OLS line
    if not np.isnan(m_eff) and a_pos.size >= 2:
        _a_span = np.linspace(0.0, float(a_pos.max()), 200)
        _f_span = m_eff * _a_span + ols_bias
        ax_fa.plot(_a_span, _f_span, color="crimson", linewidth=2.0,
                   linestyle="--", zorder=4,
                   label=(rf"OLS: $F_{{against}} = {m_eff:+.3f}\,|a| {ols_bias:+.3f}$"
                          f"\n$R^2={r_sq:.3f}$"))

    ax_fa.set_xlim(left=0)
    ax_fa.set_ylim(bottom=0)
    ax_fa.set_xlabel(r"$|a|$  (m/s²)")
    ax_fa.set_ylabel(r"$F_{against}$  (N)")
    ax_fa.set_title(
        f"Effective mass: {m_eff*1000:+.3f} g   bias: {ols_bias:+.3f} N   "
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

    # ── Figure 3: discrete 100 ms windowed time series ────────────────────────
    _WIN_S   = 0.100   # window width  [s]
    _STEP_S  = 0.010   # sliding-window step [s]

    t_max = float(t_arr[-1])
    bin_edges = np.arange(0.0, t_max + _WIN_S, _WIN_S)
    bin_centers, f_disc, a_disc, fa_disc = [], [], [], []
    for lo in bin_edges[:-1]:
        hi   = lo + _WIN_S
        mask = (t_arr >= lo) & (t_arr < hi)
        if not mask.any():
            continue
        f_m = float(np.mean(f_arr[mask]))
        a_m = float(np.mean(a_arr[mask]))
        bin_centers.append(lo + _WIN_S / 2.0)
        f_disc.append(f_m)
        a_disc.append(a_m)
        fa_disc.append(f_m / a_m if abs(a_m) >= _ACC_MIN else float("nan"))

    bc = np.array(bin_centers)
    fd = np.array(f_disc)
    ad = np.array(a_disc)
    fad = np.array(fa_disc)

    fig3, axes3 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig3.suptitle(
        f"Discrete 100 ms window averages\n{label} g",
        fontsize=11,
    )
    axes3[0].bar(bc, fd, width=_WIN_S * 0.85, color="crimson", alpha=0.75,
                 label=r"$\overline{F}_{against}$ per 100 ms bin")
    axes3[0].axhline(0, color="black", linewidth=0.7)
    axes3[0].set_ylabel("Force against push (N)")
    axes3[0].legend(fontsize=8)
    axes3[0].grid(True, alpha=0.35)

    axes3[1].bar(bc, ad, width=_WIN_S * 0.85, color="darkorange", alpha=0.75,
                 label=r"$\overline{|a|}$ per 100 ms bin")
    axes3[1].axhline(0, color="black", linewidth=0.7)
    axes3[1].set_ylabel("|acceleration|  (m/s²)")
    axes3[1].legend(fontsize=8)
    axes3[1].grid(True, alpha=0.35)

    axes3[2].bar(bc, fad, width=_WIN_S * 0.85, color="purple", alpha=0.75,
                 label=r"$\overline{F}_{against} / \overline{|a|}$ per 100 ms bin")
    axes3[2].axhline(0, color="black", linewidth=0.7)
    axes3[2].axhline(M_LB, color="gray", linewidth=0.9, linestyle="--",
                     label=f"drone mass = {M_LB*1000:.0f} g")
    axes3[2].set_xlabel("Time since first push (s)")
    axes3[2].set_ylabel("F / a (g)")
    axes3[2].legend(fontsize=8)
    axes3[2].grid(True, alpha=0.35)

    fig3.tight_layout()
    out_disc = str(log_p.parent / (log_p.stem + f"_{label}_disc_100ms.png"))
    if not show_plot:
        fig3.savefig(out_disc, dpi=150)
        print(f"Discrete-window plot saved → {out_disc}")

    # ── Figure 4: sliding 100 ms window at 10 ms granularity ─────────────────
    slide_centers = np.arange(0.0, t_max + _STEP_S, _STEP_S)
    half = _WIN_S / 2.0
    f_slide, a_slide, fa_slide = [], [], []
    for tc in slide_centers:
        mask = (t_arr >= tc - half) & (t_arr < tc + half)
        if not mask.any():
            f_slide.append(float("nan"))
            a_slide.append(float("nan"))
            fa_slide.append(float("nan"))
            continue
        f_m = float(np.mean(f_arr[mask]))
        a_m = float(np.mean(a_arr[mask]))
        f_slide.append(f_m)
        a_slide.append(a_m)
        fa_slide.append(f_m / a_m if abs(a_m) >= _ACC_MIN else float("nan"))

    fs = np.array(f_slide)
    as_ = np.array(a_slide)
    fas = np.array(fa_slide)

    fig4, axes4 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig4.suptitle(
        f"Sliding 100 ms window averages (10 ms step)\n{label} g",
        fontsize=11,
    )
    axes4[0].plot(slide_centers, fs, color="crimson", linewidth=1.4,
                  label=r"$\overline{F}_{against}$ (100 ms sliding)")
    axes4[0].axhline(0, color="black", linewidth=0.7)
    axes4[0].fill_between(slide_centers, fs, 0,
                          where=np.nan_to_num(fs) > 0,
                          alpha=0.18, color="crimson")
    axes4[0].set_ylabel("Force against push (N)")
    axes4[0].legend(fontsize=8)
    axes4[0].grid(True, alpha=0.35)

    axes4[1].plot(slide_centers, as_, color="darkorange", linewidth=1.4,
                  label=r"$\overline{|a|}$ (100 ms sliding)")
    axes4[1].axhline(0, color="black", linewidth=0.7)
    axes4[1].set_ylabel("|acceleration|  (m/s²)")
    axes4[1].legend(fontsize=8)
    axes4[1].grid(True, alpha=0.35)

    axes4[2].plot(slide_centers, fas, color="purple", linewidth=1.4,
                  label=r"$\overline{F}_{against} / \overline{|a|}$ (100 ms sliding)")
    axes4[2].axhline(0, color="black", linewidth=0.7)
    axes4[2].axhline(M_LB, color="gray", linewidth=0.9, linestyle="--",
                     label=f"drone mass = {M_LB*1000:.0f} g")
    axes4[2].set_xlabel("Time since first push (s)")
    axes4[2].set_ylabel("F / a (g)")
    axes4[2].legend(fontsize=8)
    axes4[2].grid(True, alpha=0.35)

    fig4.tight_layout()
    out_slide = str(log_p.parent / (log_p.stem + f"_{label}_slide_100ms.png"))
    if show_plot:
        plt.show()
    else:
        fig4.savefig(out_slide, dpi=150)
        print(f"Sliding-window plot saved → {out_slide}")


if __name__ == "__main__":
    _project_root = Path(__file__).resolve().parents[2]
    _log_dir = _project_root / 'logs' / 'mass_emulation'

    for _logfile in sorted(_log_dir.glob("*.json")):
        main(str(_logfile), show_plot=False)


# ── Example ───────────────────────────────────────────────────────────────────
# Run from project root:
#   python Interaction/analysis/push_force_analysis.py
#
# Run from any directory (path resolves via __file__):
#   python /path/to/Interaction/analysis/push_force_analysis.py
