"""
Interaction analysis plots.

Figures generated:
  1. fig1_flick_across_subjects.png    — strip+box per subject, flick only
  2. fig2_push_across_subjects.png     — strip+box per subject, push only
  3. fig3_subject7_multi_flick.png     — bar chart per metric, S7 flick instances
  4. fig4_subject7_multi_push.png      — bar chart per metric, S7 push (Slow Poke) instances
  5. fig5_subject7_multi_poke.png      — bar chart per metric, S7 poke instances
  6. fig6_subject7_type_comparison.png — grouped bar: flick / push / poke for S7

Subject aliases (sorted by original ID):
  1→S1  3→S2  5→S3  6→S4  8→S5  9→S6  11→S7
"""

import csv
import math
import os
import warnings

import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np

warnings.filterwarnings("ignore")

# ─── global font style ───────────────────────────────────────────────────────
plt.rcParams.update({
    "font.family":     "serif",
    "font.serif":      ["Times New Roman"],
    "font.size":       16,
    "axes.titlesize":  16,
    "axes.labelsize":  16,
    "xtick.labelsize": 16,
    "ytick.labelsize": 16,
    "legend.fontsize": 16,
})

# ─── paths ────────────────────────────────────────────────────────────────────
CSV_PATH = os.path.join(os.path.dirname(__file__), "..", "logs", "User_Studies",
                        "interaction_metrics.csv")
OUT_DIR = os.path.join(os.path.dirname(__file__))

# ─── subject alias map ────────────────────────────────────────────────────────
SUBJECT_ALIAS = {
    "1": "S1", "3": "S2", "5": "S3", "6": "S4",
    "8": "S5", "9": "S6", "11": "S7",
}
FOCUS_SUBJECT = "11"

# ─── metrics ──────────────────────────────────────────────────────────────────
METRICS = [
    ("impulse_per_mass", "Impulse/mass (m/s)"),
    ("peak_speed",       "Peak Speed (m/s)"),
    ("rise_time_s",      "Rise Time (s)"),
    ("duration_s",       "Duration (s)"),
]
METRIC_KEYS   = [m[0] for m in METRICS]
METRIC_LABELS = [m[1] for m in METRICS]

# ─── academic color palette (TABLEAU, no red/green) ──────────────────────────
TC = mcolors.TABLEAU_COLORS
TYPE_COLORS = {
    "flick": TC["tab:blue"],
    "push":  TC["tab:orange"],
    "poke":  TC["tab:purple"],
}
# Three distinct instance colors — blue, orange, cyan
INSTANCE_COLORS = [TC["tab:blue"], TC["tab:orange"], TC["tab:cyan"]]


# ─── load ─────────────────────────────────────────────────────────────────────
def load_csv(path: str) -> list[dict]:
    extra = ["avg_speed", "std_speed", "peak_to_avg",
             "settling_time_s", "baseline_speed", "min_speed"]
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        for k in METRIC_KEYS + extra:
            try:
                r[k] = float(r[k])
            except (ValueError, KeyError):
                r[k] = float("nan")
        r["instance_num"] = int(r["instance_num"])
    return rows


def filter_rows(rows, itype=None, subject=None):
    out = rows
    if itype:
        out = [r for r in out if r["interaction_type"] == itype]
    if subject:
        out = [r for r in out if r["user_id"] == str(subject)]
    return out


def alias(uid: str) -> str:
    return SUBJECT_ALIAS.get(str(uid), f"S{uid}")


# ─── helpers ──────────────────────────────────────────────────────────────────
def jitter(n, width=0.15):
    rng = np.random.default_rng(42)
    return rng.uniform(-width, width, n)


def subject_order(rows, itype):
    return sorted({r["user_id"] for r in rows if r["interaction_type"] == itype},
                  key=lambda x: int(x))


def apply_style(ax, xlabel=""):
    ax.set_xlabel(xlabel)
    ax.grid(True, linestyle="--", alpha=0.7)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.tick_params(axis="both", labelsize=16)


# ══════════════════════════════════════════════════════════════════════════════
# Fig 1 — line plot: avg metric per subject, one line per interaction type
# ══════════════════════════════════════════════════════════════════════════════
def plot_across_subjects(rows: list[dict], out_path: str):
    types        = ["flick", "push", "poke"]
    type_display = {"flick": "Flick", "push": "Slow Poke", "poke": "Poke"}
    markers      = {"flick": "o", "push": "s", "poke": "^"}

    # All subjects that appear in any of the three types, in order
    all_subjects = sorted(
        {r["user_id"] for r in rows if r["interaction_type"] in types},
        key=lambda x: int(x)
    )
    x_pos    = np.arange(len(all_subjects))
    x_labels = [alias(s) for s in all_subjects]

    fig, axes = plt.subplots(1, 4, figsize=(22, 6))

    for ax, (key, label) in zip(axes, METRICS):
        all_vals = []
        for itype in types:
            avgs = []
            for s in all_subjects:
                vals = [r[key] for r in rows
                        if r["user_id"] == s
                        and r["interaction_type"] == itype
                        and not math.isnan(r[key])]
                avgs.append(float(np.mean(vals)) if vals else float("nan"))

            # only connect non-nan points
            xs = [xi for xi, v in zip(x_pos, avgs) if not math.isnan(v)]
            ys = [v  for v      in avgs              if not math.isnan(v)]

            ax.plot(xs, ys,
                    color=TYPE_COLORS[itype],
                    marker=markers[itype],
                    markersize=8,
                    linewidth=2,
                    label=type_display[itype])
            all_vals.extend(ys)

        ax.set_title(label, loc="left")
        ax.set_xticks(x_pos)
        ax.set_xticklabels(x_labels, rotation=30, ha="right")
        if all_vals:
            ax.set_ylim(0, max(all_vals) * 1.2)
        apply_style(ax, xlabel="Subject")

    # single shared legend on the figure
    handles, lbls = axes[0].get_legend_handles_labels()
    fig.legend(handles, lbls, loc="upper right",
               bbox_to_anchor=(1.0, 1.0), frameon=False)

    plt.tight_layout()
    fig.savefig(out_path, dpi=500, bbox_inches="tight")
    print(f"[fig] across subjects (line) → {out_path}")


# ══════════════════════════════════════════════════════════════════════════════
# Fig 3, 4, 5 — bar chart per metric, multiple instances (one subject + type)
# ══════════════════════════════════════════════════════════════════════════════
def plot_multi_instances(rows: list[dict], subject: str, itype: str,
                         out_path: str, bar_labels: list[str]):
    """
    bar_labels: display names for each instance bar, e.g. ["Flick 1", "Flick 2", ...]
    """
    data  = sorted(filter_rows(rows, itype=itype, subject=subject),
                   key=lambda r: r["instance_num"])
    n_inst = len(data)
    x_pos  = np.arange(n_inst)
    color  = TYPE_COLORS.get(itype, TC["tab:gray"])

    fig, axes = plt.subplots(1, 4, figsize=(22, 6))

    for ax, (key, label) in zip(axes, METRICS):
        vals = [r[key] for r in data]

        bars = ax.bar(x_pos, vals, color=color, alpha=0.85,
                      edgecolor="white", width=0.5)

        for bar, v in zip(bars, vals):
            if not math.isnan(v):
                ax.text(bar.get_x() + bar.get_width() / 2, v * 1.02,
                        f"{v:.3g}", ha="center", va="bottom")

        ax.set_title(label, loc="left")
        ax.set_xticks(x_pos)
        ax.set_xticklabels(bar_labels[:n_inst], rotation=20, ha="right")
        ax.set_ylim(0, max((v for v in vals if not math.isnan(v)), default=1) * 1.2)
        apply_style(ax)

    plt.tight_layout()
    fig.savefig(out_path, dpi=500, bbox_inches="tight")
    print(f"[fig] {alias(subject)} multi-{itype} → {out_path}")


# ══════════════════════════════════════════════════════════════════════════════
# Fig 6 — grouped bar: flick / push / poke for one subject
# ══════════════════════════════════════════════════════════════════════════════
def plot_type_comparison(rows: list[dict], subject: str, out_path: str):
    keys   = METRIC_KEYS
    labels = METRIC_LABELS
    types  = ["flick", "push", "poke"]
    type_display = ["Flick", "Slow Poke", "Poke"]

    def mean_vals(itype):
        d = filter_rows(rows, itype=itype, subject=subject)
        return [float(np.nanmean([r[k] for r in d])) if d else float("nan")
                for k in keys]

    means = {t: mean_vals(t) for t in types}

    fig, axes = plt.subplots(1, 4, figsize=(22, 6))

    for ax, ki, label in zip(axes, range(len(keys)), labels):
        x_pos  = np.arange(len(types))
        vals   = [means[t][ki] for t in types]
        colors = [TYPE_COLORS[t] for t in types]

        bars = ax.bar(x_pos, vals, color=colors, alpha=0.85,
                      edgecolor="white", width=0.5)

        for bar, v in zip(bars, vals):
            if not math.isnan(v):
                ax.text(bar.get_x() + bar.get_width() / 2, v * 1.02,
                        f"{v:.3g}", ha="center", va="bottom")

        ax.set_title(label, loc="left")
        ax.set_xticks(x_pos)
        ax.set_xticklabels(type_display)
        ax.set_ylim(0, max((v for v in vals if not math.isnan(v)), default=1) * 1.2)
        apply_style(ax)

    plt.tight_layout()
    fig.savefig(out_path, dpi=500, bbox_inches="tight")
    print(f"[fig] {alias(subject)} type comparison → {out_path}")


# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    rows       = load_csv(CSV_PATH)
    rows_typed = [r for r in rows if r["interaction_type"] != "unknown"]

    plot_across_subjects(
        rows_typed,
        os.path.join(OUT_DIR, "interaction_across_subjects.png")
    )
    plot_multi_instances(
        rows_typed, FOCUS_SUBJECT, "flick",
        os.path.join(OUT_DIR, "subject7_multi_flick.png"),
        bar_labels=["Flick 1", "Flick 2", "Flick 3"],
    )
    plot_multi_instances(
        rows_typed, FOCUS_SUBJECT, "push",
        os.path.join(OUT_DIR, "subject7_multi_push.png"),
        bar_labels=["Slow Poke 1", "Slow Poke 2", "Slow Poke 3"],
    )
    plot_multi_instances(
        rows_typed, FOCUS_SUBJECT, "poke",
        os.path.join(OUT_DIR, "subject7_multi_poke.png"),
        bar_labels=["Poke 1", "Poke 2", "Poke 3"],
    )
    plot_type_comparison(
        rows_typed, FOCUS_SUBJECT,
        os.path.join(OUT_DIR, "subject7_type_comparison.png")
    )
    print("\nDone.")
