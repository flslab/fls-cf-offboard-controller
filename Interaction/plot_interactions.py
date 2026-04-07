
import json, os, re, warnings
import numpy as np
from collections import defaultdict
from scipy import stats
try:
    from statsmodels.stats.multicomp import pairwise_tukeyhsd
    HAS_TUKEY = True
except ImportError:
    HAS_TUKEY = False
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.patches as mpatches

warnings.filterwarnings("ignore")

# ─── paths ─────────────────────────────────────────────────────────────────────
_HERE   = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(_HERE, "..", "logs", "block_interaction")
OUT_DIR = LOG_DIR

# ─── style (matches stats_analysis.py) ────────────────────────────────────────
plt.rcParams.update({
    "font.family":     "serif",
    "font.serif":      ["Times New Roman"],
    "font.size":       20,
    "axes.titlesize":  20,
    "axes.labelsize":  20,
    "xtick.labelsize": 20,
    "ytick.labelsize": 20,
    "legend.fontsize": 20,
})

TC           = mcolors.TABLEAU_COLORS
TYPE_COLORS  = {"flick": TC["tab:blue"],
                "push":  TC["tab:orange"],
                "poke":  TC["tab:purple"]}
TYPE_DISPLAY = {"flick": "Flick", "push": "Slow Poke", "poke": "Poke"}
TYPES        = ["flick", "push", "poke"]

# Per-file/trial color cycle (up to 6 distinct trials per user per gesture)
TRIAL_COLORS = [TC["tab:blue"], TC["tab:orange"], TC["tab:green"],
                TC["tab:red"],  TC["tab:cyan"],   TC["tab:brown"]]

# File regex
FILE_RE = re.compile(
    r"us(?P<uid>\d+)_(?P<num>\d+)(?P<gesture>poke|push|flick)_translation_"
    r"(?P<ts>[\d\-_]+)\.json$"
)

# Velocity interpolation grid
N_GRID   = 300
T_WINDOW = 2.5   # seconds after interaction start

M2MM = 1e3   # metres → millimetres

METRICS = {
    "first_speed": "First Detected Speed (mm/s)",
    "peak_speed":  "Peak Speed (mm/s)",
    "peak_accel":  "Peak Accel. (mm/s²)",
    "rise_time":   "Rise Time (s)",
    "duration":    "Duration (s)",
    "impulse":     "Impulse (mm)",
}


# ══════════════════════════════════════════════════════════════════════════════
# Data loading
# ══════════════════════════════════════════════════════════════════════════════

def _load_records(path):
    with open(path, encoding="utf-8") as fh:
        raw = fh.read()
    out = []
    for line in raw.splitlines():
        line = line.strip().rstrip(",")
        if not line or line in ("[", "]"):
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return out


def _parse_file(path):
    """
    Returns:
      frames : list of {'tvec':(3,), 'vel':(3,), 'time':float}
      segs   : list of (t_start, t_end) for each interaction
    """
    records = _load_records(path)

    frames = []
    for rec in records:
        if rec.get("type") == "frames":
            d = rec["data"]
            frames.append({
                "tvec": np.array(d["tvec"],                    dtype=np.float64),
                "vel":  np.array(d.get("vel", [0., 0., 0.]),  dtype=np.float64),
                "time": float(d["time"]),
            })

    segs, cur, cur_spd = [], None, None
    for rec in records:
        if rec.get("type") != "events":
            continue
        name = rec["name"]
        t    = float(rec["data"]["time"])
        if name == "User Pushing" and cur is None:
            cur     = t
            cur_spd = float(rec["data"].get("speed", 0.0)) * M2MM
        elif name == "User Disengage" and cur is not None:
            segs.append((cur, t, cur_spd))
            cur, cur_spd = None, None

    return frames, segs


def _speed(vel_arr):
    return np.linalg.norm(vel_arr, axis=1)


def _interp_vel(t_rel, speed, t_grid):
    """Interpolate speed onto t_grid; clamp to [0, T_WINDOW]."""
    return np.interp(t_grid, t_rel, speed,
                     left=speed[0], right=speed[-1])


def load_all_data():
    """
    Returns
    -------
    file_data  : {uid: {gesture: [{'tvec','t','speed','fname'}]}}
    inter_data : {uid: {gesture: [{'tvec','t','speed'}]}}
    metric_data: {gesture: {'peak_speed':[], 'rise_time':[], 'duration':[], 'impulse':[]}}
    all_uids   : sorted list of int
    """
    files_by_gesture = defaultdict(list)
    for fname in sorted(os.listdir(LOG_DIR)):
        m = FILE_RE.match(fname)
        if m:
            files_by_gesture[m.group("gesture")].append(
                (int(m.group("uid")), os.path.join(LOG_DIR, fname), fname)
            )

    file_data   = defaultdict(lambda: defaultdict(list))
    inter_data  = defaultdict(lambda: defaultdict(list))
    metric_data = {g: {k: [] for k in METRICS} for g in TYPES}

    for gesture in TYPES:
        for uid, path, fname in files_by_gesture.get(gesture, []):
            frames, segs = _parse_file(path)
            if not frames:
                continue

            # ── file-level trajectory (convert to mm / mm·s⁻¹) ───────────
            all_t    = np.array([f["time"] for f in frames])
            all_tvec = np.array([f["tvec"] for f in frames]) * M2MM
            all_vel  = np.array([f["vel"]  for f in frames]) * M2MM
            t_rel    = all_t - all_t[0]
            # index of the last frame inside each interaction segment
            seg_end_indices = []
            for t_start, t_end, _ in segs:
                idxs = np.where((all_t >= t_start) & (all_t <= t_end))[0]
                if len(idxs):
                    seg_end_indices.append(int(idxs[-1]))
            file_data[uid][gesture].append({
                "tvec":      all_tvec,
                "t":         t_rel,
                "speed":     _speed(all_vel),
                "fname":     fname,
                "seg_ends":  seg_end_indices,
            })

            # ── interaction-level ──────────────────────────────────────────
            for t_start, t_end, first_spd in segs:
                sub = [f for f in frames if t_start <= f["time"] <= t_end]
                if len(sub) < 4:
                    continue
                tvec  = np.array([f["tvec"] for f in sub]) * M2MM
                vel   = np.array([f["vel"]  for f in sub]) * M2MM
                times = np.array([f["time"] for f in sub])
                t_ri  = times - times[0]
                spd   = _speed(vel)          # mm/s

                dur      = float(t_ri[-1])
                pk_spd   = float(np.max(spd))
                pk_idx   = int(np.argmax(spd))
                rise_t   = float(t_ri[pk_idx])
                impulse  = float(np.trapz(spd, t_ri))   # mm

                dt       = np.diff(t_ri)
                dt       = np.where(dt < 1e-9, 1e-9, dt)
                accel    = np.concatenate([[0.0], np.diff(spd) / dt])
                pk_accel = float(np.max(np.abs(accel)))  # mm/s²

                inter_data[uid][gesture].append({
                    "tvec":  tvec,
                    "t":     t_ri,
                    "speed": spd,
                })
                metric_data[gesture]["first_speed"].append(first_spd)
                metric_data[gesture]["peak_speed"].append(pk_spd)
                metric_data[gesture]["peak_accel"].append(pk_accel)
                metric_data[gesture]["rise_time"].append(rise_t)
                metric_data[gesture]["duration"].append(dur)
                metric_data[gesture]["impulse"].append(impulse)

    all_uids = sorted(set(
        uid for g in TYPES for uid, *_ in files_by_gesture.get(g, [])
    ))
    return file_data, inter_data, metric_data, all_uids


# ══════════════════════════════════════════════════════════════════════════════
# Shared helpers
# ══════════════════════════════════════════════════════════════════════════════

def apply_style(ax, xlabel="", ylabel=""):
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, linestyle="--", alpha=0.7)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def _save(fig, name):
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=300, bbox_inches="tight")
    print(f"  [saved] {name}")
    plt.close(fig)


def _origin_align(tvec):
    aligned = tvec - tvec[0]
    aligned[:, 0] += 50.0   # shift start to x=50 mm
    return aligned


def _mark_endpoints(ax, tn, color, ms=8, mew=2):
    """Mark start and end of a trajectory with 'x'."""
    ax.plot(tn[0,  0], tn[0,  1], "x", color=color, ms=ms, mew=mew, zorder=5)
    ax.plot(tn[-1, 0], tn[-1, 1], "x", color=color, ms=ms, mew=mew, zorder=5)


def _style_xy_ax(ax, title=""):
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    if title:
        ax.set_title(title, loc="left")
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 200)
    ax.set_aspect("equal", adjustable="box")
    apply_style(ax)


def _vel_mean_std(trials, t_grid):
    """Compute mean & std speed curves interpolated onto t_grid."""
    mat = np.array([_interp_vel(tr["t"], tr["speed"], t_grid) for tr in trials])
    return mat.mean(axis=0), mat.std(axis=0)


# ══════════════════════════════════════════════════════════════════════════════
# Group 1 — All gestures, all data
# ══════════════════════════════════════════════════════════════════════════════

def plot_all_traj(file_data, inter_data):
    """
    Two XY trajectory plots:
      all_traj_by_file.png        — each file = 1 trajectory
      all_traj_by_interaction.png — each interaction segment = 1 trajectory
    Layout: 1 × 3 subplots, one per gesture.
    All users' data overlaid; colour = gesture type.
    """
    # ── all_traj_by_file: mark end of each interaction segment ────────────
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    for ax, gesture in zip(axes, TYPES):
        color = TYPE_COLORS[gesture]
        for uid_data in file_data.values():
            for tr in uid_data.get(gesture, []):
                tn = _origin_align(tr["tvec"])
                ax.plot(tn[:, 0], tn[:, 1],
                        color=color, lw=1.2, alpha=0.55)
                for idx in tr.get("seg_ends", []):
                    ax.plot(tn[idx, 0], tn[idx, 1],
                            "x", color=color, ms=8, mew=2, zorder=5)
        _style_xy_ax(ax, TYPE_DISPLAY[gesture])
    fig.tight_layout()
    _save(fig, "all_traj_by_file.png")

    # ── all_traj_by_interaction: mark start and end of each segment ───────
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    for ax, gesture in zip(axes, TYPES):
        color = TYPE_COLORS[gesture]
        for uid_data in inter_data.values():
            for tr in uid_data.get(gesture, []):
                tn = _origin_align(tr["tvec"])
                ax.plot(tn[:, 0], tn[:, 1],
                        color=color, lw=1.2, alpha=0.55)
                _mark_endpoints(ax, tn, color)
        _style_xy_ax(ax, TYPE_DISPLAY[gesture])
    fig.tight_layout()
    _save(fig, "all_traj_by_interaction.png")


def plot_all_vel(inter_data):
    """
    all_vel_vs_time.png — mean ± std velocity per gesture across all interactions.
    Matches stats_interaction_velocity.png style.
    """
    t_grid = np.linspace(0, T_WINDOW, N_GRID)
    fig, ax = plt.subplots(figsize=(10, 6))
    handles = []

    for gesture in TYPES:
        all_trials = [tr for uid_d in inter_data.values()
                      for tr in uid_d.get(gesture, [])
]
        if not all_trials:
            continue
        mu, sd = _vel_mean_std(all_trials, t_grid)
        c = TYPE_COLORS[gesture]
        ax.plot(t_grid, mu, color=c, linewidth=2.0)
        ax.fill_between(t_grid, mu - sd, mu + sd, color=c, alpha=0.20)
        handles.append(mpatches.Patch(color=c, label=TYPE_DISPLAY[gesture]))

    ax.axvline(0, color="gray", linestyle="--", linewidth=1.2, alpha=0.8)
    ax.set_title("Average Speed (mm/s)", loc="left")
    ax.legend(handles=handles, frameon=False)
    ax.set_ylim(0, 1100)
    ax.set_xlim(0, 2.5)
    apply_style(ax, xlabel="Time relative to interaction start (s)")
    fig.tight_layout()
    _save(fig, "all_vel_vs_time.png")


# ══════════════════════════════════════════════════════════════════════════════
# Group 2 — Per-user reproducibility
# ══════════════════════════════════════════════════════════════════════════════

def plot_per_user_traj(inter_data, all_uids):
    """
    user_{uid}_traj.png — for each user: 1×3 subplots (one per gesture).
    Each interaction segment is one trajectory; colour = gesture type.
    """
    for uid in all_uids:
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        any_data = False
        for ax, gesture in zip(axes, TYPES):
            color = TYPE_COLORS[gesture]
            for tr in inter_data[uid].get(gesture, []):
                tn = _origin_align(tr["tvec"])
                ax.plot(tn[:, 0], tn[:, 1],
                        color=color, lw=1.5, alpha=0.75)
                _mark_endpoints(ax, tn, color)
                any_data = True
            _style_xy_ax(ax, TYPE_DISPLAY[gesture])

        if not any_data:
            plt.close(fig)
            continue
        fig.tight_layout()
        _save(fig, f"user_{uid}_traj.png")


def plot_per_user_vel(inter_data, all_uids):
    """
    user_{uid}_vel.png — for each user: 1×3 subplots (one per gesture).
    Mean ± std velocity across all interactions for that user (same style as all_vel_vs_time).
    """
    t_grid = np.linspace(0, T_WINDOW, N_GRID)
    for uid in all_uids:
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        any_data = False
        for ax, gesture in zip(axes, TYPES):
            color  = TYPE_COLORS[gesture]
            trials = [tr for tr in inter_data[uid].get(gesture, [])
]
            if trials:
                mu, sd = _vel_mean_std(trials, t_grid)
                ax.plot(t_grid, mu, color=color, lw=2.0)
                ax.fill_between(t_grid, mu - sd, mu + sd, color=color, alpha=0.20)
                ax.axvline(0, color="gray", linestyle="--", linewidth=1.2, alpha=0.8)
                any_data = True
            ax.set_ylim(0, 1100)
            ax.set_xlim(0, 2.5)
            apply_style(ax,
                        xlabel="Time relative to interaction start (s)",
                        ylabel="Speed (mm/s)")

        if not any_data:
            plt.close(fig)
            continue
        fig.tight_layout()
        _save(fig, f"user_{uid}_vel.png")


# ══════════════════════════════════════════════════════════════════════════════
# Group 3 — Cross-user comparison per gesture
# ══════════════════════════════════════════════════════════════════════════════

def _uid_colormap(all_uids):
    cmap = plt.cm.tab10
    return {uid: cmap(i / max(len(all_uids) - 1, 1))
            for i, uid in enumerate(all_uids)}


def plot_gesture_traj(inter_data, all_uids):
    """
    gesture_{name}_traj.png — one figure per gesture.
    All users' interaction trajectories; colour = user.
    """
    uid_color = _uid_colormap(all_uids)
    for gesture in TYPES:
        fig, ax = plt.subplots(figsize=(8, 7))
        handles = []
        for uid in all_uids:
            color  = uid_color[uid]
            trials = inter_data[uid].get(gesture, [])
            first  = True
            for tr in trials:
                tn  = _origin_align(tr["tvec"])
                lbl = f"S{uid}" if first else None
                ax.plot(tn[:, 0], tn[:, 1],
                        color=color, lw=1.4, alpha=0.65, label=lbl)
                _mark_endpoints(ax, tn, color)
                first = False
            if trials:
                handles.append(mpatches.Patch(color=color, label=f"S{uid}"))

        _style_xy_ax(ax, f"{TYPE_DISPLAY[gesture]} — all subjects")
        ax.legend(handles=handles, frameon=False, fontsize=16)
        fig.tight_layout()
        _save(fig, f"gesture_{gesture}_traj.png")


def plot_gesture_vel(inter_data, all_uids):
    """
    gesture_{name}_vel.png — one figure per gesture.
    Mean ± std velocity per user; colour = user.
    """
    uid_color = _uid_colormap(all_uids)
    t_grid    = np.linspace(0, T_WINDOW, N_GRID)

    for gesture in TYPES:
        fig, ax = plt.subplots(figsize=(10, 6))
        handles = []
        for uid in all_uids:
            trials = [tr for tr in inter_data[uid].get(gesture, [])
]
            if not trials:
                continue
            color    = uid_color[uid]
            mu, sd   = _vel_mean_std(trials, t_grid)
            ax.plot(t_grid, mu, color=color, lw=2.0)
            ax.fill_between(t_grid, mu - sd, mu + sd, color=color, alpha=0.20)
            handles.append(mpatches.Patch(color=color, label=f"S{uid}"))

        ax.axvline(0, color="gray", linestyle="--", linewidth=1.2, alpha=0.8)
        ax.set_title(f"{TYPE_DISPLAY[gesture]} — Average Speed (mm/s)", loc="left")
        ax.legend(handles=handles, frameon=False, fontsize=16,
                  title="Subject  (band = ± std)")
        ax.set_ylim(0, 1100)
        ax.set_xlim(0, 2.5)
        apply_style(ax, xlabel="Time relative to interaction start (s)")
        fig.tight_layout()
        _save(fig, f"gesture_{gesture}_vel.png")


# ══════════════════════════════════════════════════════════════════════════════
# Statistics — boxplots with ANOVA + Tukey HSD significance brackets
# ══════════════════════════════════════════════════════════════════════════════

def add_significance_brackets(ax, sig_pairs, x_positions, y_top, step=0.08):
    """Draw bracket + * above significantly different pairs (from stats_analysis.py)."""
    valid = [(g1, g2) for g1, g2 in sig_pairs
             if g1 in x_positions and g2 in x_positions]
    if not valid:
        return
    span         = y_top - ax.get_ylim()[0]
    bracket_step = step * span
    heights = []
    y       = y_top
    for _ in valid:
        h = y + bracket_step
        heights.append((y, h))
        y = h + bracket_step
    needed_top = max(h for _, h in heights) * 1.1
    lo, hi = ax.get_ylim()
    ax.set_ylim(lo, max(hi, needed_top))
    for (g1, g2), (y_base, h) in zip(valid, heights):
        x1, x2 = x_positions[g1], x_positions[g2]
        ax.plot([x1, x1, x2, x2], [y_base, h, h, y_base], lw=1.2, color="black")
        ax.text((x1 + x2) / 2, h, "*", ha="center", va="bottom", fontsize=14)


def run_anova_tukey(metric_data):
    """Returns {metric: {'F', 'p', 'eta2', 'sig_pairs'}}."""
    results = {}
    for key in METRICS:
        groups = [np.array(metric_data[g][key]) for g in TYPES]
        groups = [g for g in groups if len(g) > 0]
        if len(groups) < 2:
            continue
        F, p        = stats.f_oneway(*groups)
        grand_mean  = np.mean(np.concatenate(groups))
        ss_between  = sum(len(g) * (g.mean() - grand_mean) ** 2 for g in groups)
        ss_total    = sum(((g - grand_mean) ** 2).sum() for g in groups)
        eta2        = ss_between / ss_total if ss_total > 0 else float("nan")
        sig_pairs   = []

        if HAS_TUKEY:
            all_vals  = np.concatenate([metric_data[g][key] for g in TYPES])
            all_types = np.concatenate([[g] * len(metric_data[g][key]) for g in TYPES])
            if len(set(all_types)) >= 2:
                tukey = pairwise_tukeyhsd(all_vals, all_types, alpha=0.05)
                for row in tukey.summary().data[1:]:
                    g1, g2, _, _, _, _, reject = row
                    if reject:
                        sig_pairs.append((g1, g2))

        stars = "***" if p < 0.001 else "**" if p < 0.01 else "*" if p < 0.05 else "n.s."
        n_str = ", ".join(f"{TYPE_DISPLAY[g]}(n={len(metric_data[g][key])})" for g in TYPES)
        print(f"  {METRICS[key]:<25s}  F={F:.2f}  p={p:.4e}  {stars}  η²={eta2:.3f}")
        print(f"    {n_str}")
        results[key] = {"F": F, "p": p, "eta2": eta2, "sig_pairs": sig_pairs}

    return results


def plot_stats_boxplots(metric_data):
    """
    stats_boxplots.png — 2×2 boxplot grid with ANOVA + Tukey significance brackets.
    Matches stats_fig1_boxplots.png style from stats_analysis.py.
    """
    print("\n─── One-way ANOVA results ───────────────────────────────────────")
    results = run_anova_tukey(metric_data)

    x_labels = [TYPE_DISPLAY[t] for t in TYPES]
    x_pos    = {t: i for i, t in enumerate(TYPES)}
    colors   = [TYPE_COLORS[t] for t in TYPES]

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    axes = axes.flatten()

    for ax, (key, label) in zip(axes, METRICS.items()):
        groups = [metric_data[g][key] for g in TYPES]
        bp = ax.boxplot(
            groups,
            positions=range(len(TYPES)),
            widths=0.45,
            patch_artist=True,
            medianprops=dict(color="black", linewidth=2),
            flierprops=dict(marker="o", markersize=4, alpha=0.5),
        )
        for patch, c in zip(bp["boxes"], colors):
            patch.set_facecolor(c)
            patch.set_alpha(0.55)
        for element in ("whiskers", "caps"):
            for line, c in zip(bp[element], np.repeat(colors, 2)):
                line.set_color(c)
        for flier, c in zip(bp["fliers"], colors):
            flier.set_markerfacecolor(c)
            flier.set_markeredgecolor(c)

        ax.set_title(label, loc="left")
        ax.set_xticks(range(len(TYPES)))
        ax.set_xticklabels(x_labels)

        all_vals = [v for g in groups for v in g]
        if all_vals:
            ax.set_ylim(0, max(all_vals) * 1.2)
            if key in results:
                r         = results[key]
                sig_pairs = [(g1, g2) for g1, g2 in r["sig_pairs"]
                             if g1 in x_pos and g2 in x_pos]
                add_significance_brackets(ax, sig_pairs, x_pos,
                                          max(all_vals) * 1.05)
                stars = ("***" if r["p"] < 0.001 else "**" if r["p"] < 0.01
                         else "*" if r["p"] < 0.05 else "n.s.")
                ax.text(0.98, 0.98,
                        f"F={r['F']:.1f}, {stars}",
                        transform=ax.transAxes,
                        ha="right", va="top", fontsize=16,
                        color="dimgray")
        apply_style(ax)

    for ax in axes.flatten()[len(METRICS):]:
        ax.set_visible(False)
    plt.tight_layout()
    _save(fig, "stats_boxplots.png")


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("=" * 60)
    print("Loading data from", LOG_DIR)
    print("=" * 60)
    file_data, inter_data, metric_data, all_uids = load_all_data()

    for g in TYPES:
        n_files = sum(len(file_data[u].get(g, [])) for u in all_uids)
        n_inter = sum(len(inter_data[u].get(g, [])) for u in all_uids)
        print(f"  {TYPE_DISPLAY[g]:<10s}: {n_files} files, {n_inter} interactions, "
              f"{len(metric_data[g]['peak_speed'])} metric samples")

    print(f"  Subjects: {all_uids}")

    os.makedirs(OUT_DIR, exist_ok=True)

    print("\n─── Group 1: All gestures, all data ─────────────────────────────")
    plot_all_traj(file_data, inter_data)
    plot_all_vel(inter_data)

    print("\n─── Group 2: Per-user reproducibility ───────────────────────────")
    plot_per_user_traj(inter_data, all_uids)
    plot_per_user_vel(inter_data, all_uids)

    print("\n─── Group 3: Cross-user comparison per gesture ──────────────────")
    plot_gesture_traj(inter_data, all_uids)
    plot_gesture_vel(inter_data, all_uids)

    print("\n─── Statistics ──────────────────────────────────────────────────")
    plot_stats_boxplots(metric_data)

    print("\nDone. All figures saved to:", OUT_DIR)
