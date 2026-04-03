"""
Publication-quality statistical analysis for UIST paper.
Drone interaction telemetry: flick / poke / push.

Outputs
-------
  stats_fig1_boxplots.png / .pdf     — 2×2 boxplot grid
  stats_fig2_scatter.png  / .pdf     — rise_time vs peak_speed scatter
  stats_fig3_velocity.png / .pdf     — average velocity profiles (from raw logs)
  stats_table_anova.tex              — LaTeX-ready ANOVA table
"""

import csv, json, math, os, re, warnings
import numpy as np
from scipy import stats
from scipy.interpolate import interp1d
from statsmodels.stats.multicomp import pairwise_tukeyhsd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.patches as mpatches

warnings.filterwarnings("ignore")

# ─── paths ─────────────────────────────────────────────────────────────────────
_HERE    = os.path.dirname(__file__)
CSV_PATH = os.path.join(_HERE, "..", "logs", "User_Studies", "interaction_metrics.csv")
LOG_DIR  = os.path.join(_HERE, "..", "logs", "User_Studies")
OUT_DIR  = os.path.join(_HERE, "..", "logs", "User_Studies")

# ─── style (matches existing figures) ─────────────────────────────────────────
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

TC = mcolors.TABLEAU_COLORS
TYPE_COLORS  = {"flick": TC["tab:blue"], "push": TC["tab:orange"], "poke": TC["tab:purple"]}
TYPE_DISPLAY = {"flick": "Flick", "push": "Slow Poke", "poke": "Poke"}
TYPES        = ["flick", "push", "poke"]

# NOTE: CSV column is "impulse_per_mass"; the analysis refers to it as "impulse_mass"
METRICS = {
    "rise_time_s":      "Rise Time (s)",
    "duration_s":       "Duration (s)",
    "peak_speed":       "Peak Speed (m/s)",
    "impulse_per_mass": "Impulse/mass (m/s)",
}
METRIC_KEYS   = list(METRICS.keys())
METRIC_LABELS = list(METRICS.values())

# noise-filter constants (must match analyze_interactions.py)
MIN_DURATION_S           = 0.15
MIN_PUSH_EVENTS          = 5
NOISE_COMBINED_MAX_SPEED = 0.15
NOISE_COMBINED_MAX_DUR   = 0.30


# ══════════════════════════════════════════════════════════════════════════════
# STEP 1 — Load & clean
# ══════════════════════════════════════════════════════════════════════════════
def load_data(path: str) -> dict[str, dict[str, list]]:
    """Returns {itype: {metric_key: [values]}}."""
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))

    data: dict[str, dict[str, list]] = {t: {k: [] for k in METRIC_KEYS} for t in TYPES}
    skipped = 0

    for r in rows:
        itype = r["interaction_type"]
        if itype not in TYPES:
            continue
        try:
            vals = {k: float(r[k]) for k in METRIC_KEYS}
        except (ValueError, KeyError):
            skipped += 1
            continue
        if any(math.isnan(v) for v in vals.values()):
            skipped += 1
            continue
        for k, v in vals.items():
            data[itype][k].append(v)

    print("=" * 60)
    print("STEP 1: Data loading")
    print("=" * 60)
    if skipped:
        print(f"  Dropped {skipped} rows (NaN / unknown type).")

    for t in TYPES:
        n = len(data[t][METRIC_KEYS[0]])
        print(f"\n  {TYPE_DISPLAY[t]}  (n={n})")
        for k, lbl in METRICS.items():
            arr = np.array(data[t][k])
            print(f"    {lbl:<30s}  mean={arr.mean():.4f}  std={arr.std():.4f}")

    return data


# ══════════════════════════════════════════════════════════════════════════════
# STEP 2 — One-way ANOVA + η² + Tukey HSD
# ══════════════════════════════════════════════════════════════════════════════
def run_anova(data: dict) -> dict:
    """Returns {metric_key: {F, p, eta2, tukey_result}}."""
    print("\n" + "=" * 60)
    print("STEP 2: One-way ANOVA + Tukey HSD")
    print("=" * 60)

    results = {}
    for k, lbl in METRICS.items():
        groups = [np.array(data[t][k]) for t in TYPES]
        F, p   = stats.f_oneway(*groups)

        # η² = SS_between / SS_total
        grand_mean  = np.mean(np.concatenate(groups))
        ss_between  = sum(len(g) * (g.mean() - grand_mean) ** 2 for g in groups)
        ss_total    = sum(((v - grand_mean) ** 2).sum() for g in groups for v in g)
        eta2        = ss_between / ss_total if ss_total > 0 else float("nan")

        # Tukey HSD
        all_vals  = np.concatenate(groups)
        all_types = np.concatenate([[t] * len(data[t][k]) for t in TYPES])
        tukey     = pairwise_tukeyhsd(all_vals, all_types, alpha=0.05)

        sig_pairs = []
        for row in tukey.summary().data[1:]:          # skip header
            g1, g2, _, _, _, _, reject = row
            if reject:
                sig_pairs.append((g1, g2))

        results[k] = {"F": F, "p": p, "eta2": eta2,
                      "tukey": tukey, "sig_pairs": sig_pairs}

        stars = "***" if p < 0.001 else "**" if p < 0.01 else "*" if p < 0.05 else "n.s."
        print(f"\n  {lbl}")
        print(f"    F = {F:.4f},  p = {p:.4e}  {stars},  η² = {eta2:.4f}")
        if sig_pairs:
            for g1, g2 in sig_pairs:
                print(f"    Tukey: {TYPE_DISPLAY.get(g1,g1)} ≠ {TYPE_DISPLAY.get(g2,g2)}  (p<0.05)")
        else:
            print("    Tukey: no significant pairwise differences")

    return results


# ══════════════════════════════════════════════════════════════════════════════
# STEP 3 — Expected-results check
# ══════════════════════════════════════════════════════════════════════════════
EXPECTED = {
    "duration_s":       ("significant",     0.05),
    "rise_time_s":      ("highly significant", 0.001),
    "peak_speed":       ("not significant", None),
    "impulse_per_mass": ("not significant", None),
}

def check_expected(results: dict):
    print("\n" + "=" * 60)
    print("STEP 3: Expected-results verification")
    print("=" * 60)
    for k, (expectation, thresh) in EXPECTED.items():
        p = results[k]["p"]
        if thresh is not None:
            ok = p < thresh
        else:
            ok = p >= 0.05
        status = "OK" if ok else "WARNING"
        print(f"  [{status}] {METRICS[k]}: expected={expectation}, p={p:.4e}")


# ══════════════════════════════════════════════════════════════════════════════
# LaTeX table
# ══════════════════════════════════════════════════════════════════════════════
def write_latex_table(results: dict, path: str):
    def sig(p):
        if p < 0.001: return r"$p < 0.001$~***"
        if p < 0.01:  return r"$p < 0.01$~**"
        if p < 0.05:  return r"$p < 0.05$~*"
        return r"$p = {:.3f}$~n.s.".format(p)

    lines = [
        r"\begin{table}[h]",
        r"\centering",
        r"\caption{One-way ANOVA results for interaction metrics.}",
        r"\label{tab:anova}",
        r"\begin{tabular}{lcccc}",
        r"\hline",
        r"Metric & $F$ & $p$ & $\eta^2$ & Sig. \\",
        r"\hline",
    ]
    for k, lbl in METRICS.items():
        r = results[k]
        lbl_tex = lbl.replace("/", "/").replace("²", r"$^2$")
        pairs_str = ", ".join(
            f"{TYPE_DISPLAY.get(a,a)} vs {TYPE_DISPLAY.get(b,b)}"
            for a, b in r["sig_pairs"]
        ) or "—"
        lines.append(
            f"{lbl_tex} & {r['F']:.2f} & {sig(r['p'])} & {r['eta2']:.3f} & {pairs_str} \\\\"
        )
    lines += [r"\hline", r"\end{tabular}", r"\end{table}"]
    with open(path, "w") as fh:
        fh.write("\n".join(lines))
    print(f"\n  LaTeX table → {path}")


# ══════════════════════════════════════════════════════════════════════════════
# Helpers shared by plots
# ══════════════════════════════════════════════════════════════════════════════
def apply_style(ax, xlabel="", ylabel=""):
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, linestyle="--", alpha=0.7)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.tick_params(axis="both", labelsize=20)


def add_significance_brackets(ax, pairs_sig, x_positions, y_top, step=0.07):
    """Draw bracket + * above pairs that are significantly different.
    Pre-computes all bracket heights, then extends ylim to fit them.
    """
    valid = [(g1, g2) for g1, g2 in pairs_sig
             if g1 in x_positions and g2 in x_positions]
    if not valid:
        return

    span = y_top - ax.get_ylim()[0]
    bracket_step = step * span

    # pre-compute heights
    heights = []
    y = y_top
    for _ in valid:
        h = y + bracket_step
        heights.append((y, h))
        y = h + bracket_step

    # extend ylim so nothing is clipped
    needed_top = max(h for _, h in heights) * 1.1
    lo, hi = ax.get_ylim()
    ax.set_ylim(lo, max(hi, needed_top))

    # draw brackets
    for (g1, g2), (y_base, h) in zip(valid, heights):
        x1, x2 = x_positions[g1], x_positions[g2]
        ax.plot([x1, x1, x2, x2], [y_base, h, h, y_base],
                lw=1.2, color="black")
        ax.text((x1 + x2) / 2, h, "*",
                ha="center", va="bottom", fontsize=14)


def save(fig, base: str):
    for ext in ("png", "pdf"):
        fig.savefig(f"{base}.{ext}", dpi=500, bbox_inches="tight")
    print(f"  Saved: {base}.png / .pdf")


# ══════════════════════════════════════════════════════════════════════════════
# PLOT 1 — 2×2 boxplot grid
# ══════════════════════════════════════════════════════════════════════════════
def plot_boxplots(data: dict, results: dict, out_base: str):
    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    axes = axes.flatten()
    x_labels = [TYPE_DISPLAY[t] for t in TYPES]
    x_pos    = {t: i for i, t in enumerate(TYPES)}

    for ax, key, label in zip(axes, METRIC_KEYS, METRIC_LABELS):
        groups = [data[t][key] for t in TYPES]
        colors = [TYPE_COLORS[t] for t in TYPES]

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
        ax.set_ylim(0, max(all_vals) * 1.2)

        # significance brackets (function extends ylim if needed)
        sig_pairs = [(g1, g2) for g1, g2 in results[key]["sig_pairs"]
                     if g1 in x_pos and g2 in x_pos]
        add_significance_brackets(ax, sig_pairs, x_pos, max(all_vals) * 1.05)

        apply_style(ax)

    plt.tight_layout()
    save(fig, out_base)
    plt.close()


# ══════════════════════════════════════════════════════════════════════════════
# PLOT 2 — Scatter: rise_time vs peak_speed
# ══════════════════════════════════════════════════════════════════════════════
def plot_scatter(data: dict, out_base: str):
    fig, ax = plt.subplots(figsize=(10, 6))

    handles = []
    for t in TYPES:
        xs = data[t]["rise_time_s"]
        ys = data[t]["peak_speed"]
        ax.scatter(xs, ys,
                   color=TYPE_COLORS[t],
                   s=70, alpha=0.8,
                   edgecolors="white", linewidths=0.5,
                   zorder=3)
        handles.append(mpatches.Patch(color=TYPE_COLORS[t], label=TYPE_DISPLAY[t]))

    ax.legend(handles=handles, frameon=False)
    ax.set_title("Peak Speed (m/s)", loc="left")
    apply_style(ax, xlabel="Rise Time (s)")

    plt.tight_layout()
    save(fig, out_base)
    plt.close()


# ══════════════════════════════════════════════════════════════════════════════
# PLOT 3 — Average velocity profiles from raw JSON logs
# ══════════════════════════════════════════════════════════════════════════════
def _load_log(path: str) -> list:
    with open(path, encoding="utf-8") as fh:
        raw = fh.read().strip()
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        raw = raw[1:] if raw.startswith("[") else raw
        raw = raw[:-1] if raw.endswith("]") else raw
        records = []
        for line in raw.splitlines():
            line = line.strip().rstrip(",")
            if line:
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
        return records


def _segment_interactions(data: list) -> list:
    segs, cur_start, cur_events = [], None, []
    for entry in data:
        if entry.get("type") != "events":
            continue
        name = entry["name"]
        t    = entry["data"]["time"]
        if name == "User Pushing":
            if cur_start is None:
                cur_start = t
            cur_events.append(entry)
        elif name == "User Disengage" and cur_start is not None:
            segs.append({"start": cur_start, "end": t, "push_events": cur_events})
            cur_start, cur_events = None, []
    return segs


def _is_noisy(seg: dict) -> bool:
    dur = seg["end"] - seg["start"]
    n   = len(seg["push_events"])
    if dur < MIN_DURATION_S or n < MIN_PUSH_EVENTS:
        return True
    max_spd = max(e["data"]["speed"] for e in seg["push_events"])
    return max_spd < NOISE_COMBINED_MAX_SPEED and dur < NOISE_COMBINED_MAX_DUR


def _parse_filename(filename: str):
    base  = os.path.splitext(filename)[0]
    parts = base.split("_")
    uid   = int(parts[1])
    tail  = "_".join(parts[2:])
    seq   = []
    for m in re.finditer(r"(\d+)_?(poke|push|flick)", tail):
        seq.extend([m.group(2)] * int(m.group(1)))
    return uid, seq


T_START_OFFSET = -0.1    # seconds before push start
T_END_OFFSET   =  1.5    # seconds after push start
N_GRID         = 300     # interpolation points
TIME_GRID      = np.linspace(T_START_OFFSET, T_END_OFFSET, N_GRID)


def compute_velocity_profiles() -> dict | None:
    files = [f for f in os.listdir(LOG_DIR) if f.endswith(".json")
             and f != "interaction_metrics.csv"]
    curves: dict[str, list[np.ndarray]] = {t: [] for t in TYPES}

    for fname in sorted(files):
        try:
            _, seq = _parse_filename(fname)
        except Exception:
            continue
        if not seq:
            continue

        try:
            log = _load_log(os.path.join(LOG_DIR, fname))
        except Exception:
            continue

        all_segs   = _segment_interactions(log)
        valid_segs = [s for s in all_segs if not _is_noisy(s)]

        for idx, seg in enumerate(valid_segs):
            itype = seq[idx] if idx < len(seq) else None
            if itype not in TYPES:
                continue

            t0 = seg["start"]
            t_lo = t0 + T_START_OFFSET - 0.05
            t_hi = t0 + T_END_OFFSET   + 0.05

            frames = [
                (e["data"]["time"] - t0,
                 math.sqrt(sum(v**2 for v in e["data"]["vel"])))
                for e in log
                if e.get("type") == "frames"
                and t_lo <= e["data"]["time"] <= t_hi
            ]
            if len(frames) < 5:
                continue
            frames.sort()
            ts, spds = zip(*frames)
            try:
                f_interp = interp1d(ts, spds, kind="linear",
                                    bounds_error=False, fill_value="extrapolate")
                curves[itype].append(f_interp(TIME_GRID))
            except Exception:
                pass

    if not any(curves.values()):
        return None

    profiles = {}
    for t in TYPES:
        arr = np.array(curves[t])
        if arr.size == 0:
            continue
        profiles[t] = {"mean": arr.mean(axis=0), "std": arr.std(axis=0),
                       "n": len(arr)}
    return profiles


def plot_velocity_profiles(out_base: str):
    print("\n  Computing velocity profiles from raw logs…", end=" ", flush=True)
    profiles = compute_velocity_profiles()
    if profiles is None:
        print("\n  Velocity profile plot not generated (no time series data)")
        return
    summary = ", ".join(f"{TYPE_DISPLAY[t]}: n={profiles[t]['n']}" for t in profiles)
    print(f"done ({summary})")

    fig, ax = plt.subplots(figsize=(10, 6))
    handles = []
    for t in TYPES:
        if t not in profiles:
            continue
        mu  = profiles[t]["mean"]
        sd  = profiles[t]["std"]
        ax.plot(TIME_GRID, mu, color=TYPE_COLORS[t], linewidth=2.0)
        ax.fill_between(TIME_GRID, mu - sd, mu + sd,
                        color=TYPE_COLORS[t], alpha=0.20)
        handles.append(mpatches.Patch(color=TYPE_COLORS[t], label=TYPE_DISPLAY[t]))

    ax.axvline(0, color="gray", linestyle="--", linewidth=1.2, label="Interaction start")
    ax.set_title("Average Speed (m/s)", loc="left")
    ax.legend(handles=handles, frameon=False)
    apply_style(ax, xlabel="Time relative to start (s)")
    ax.set_ylim(bottom=0)

    plt.tight_layout()
    save(fig, out_base)
    plt.close()


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    data    = load_data(CSV_PATH)
    results = run_anova(data)
    check_expected(results)

    print("\n" + "=" * 60)
    print("STEP 4: Generating figures")
    print("=" * 60)

    plot_boxplots(
        data, results,
        os.path.join(OUT_DIR, "stats_interaction_boxplots")
    )
    plot_scatter(
        data,
        os.path.join(OUT_DIR, "stats_interaction_scatter")
    )
    plot_velocity_profiles(
        os.path.join(OUT_DIR, "stats_interaction_velocity")
    )
    write_latex_table(
        results,
        os.path.join(OUT_DIR, "stats_table_anova.tex")
    )

    print("\n" + "=" * 60)
    print("STEP 5: Short interpretation")
    print("=" * 60)
    for k, lbl in METRICS.items():
        r = results[k]
        stars = "***" if r["p"] < 0.001 else "**" if r["p"] < 0.01 \
                else "*" if r["p"] < 0.05 else "n.s."
        pairs = ", ".join(
            f"{TYPE_DISPLAY.get(a,a)} vs {TYPE_DISPLAY.get(b,b)}"
            for a, b in r["sig_pairs"]
        ) or "no pairs"
        print(f"  {lbl:<30s}  F={r['F']:.2f}  {stars}  η²={r['eta2']:.3f}  [{pairs}]")

    print("\nDone.")
