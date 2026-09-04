"""Freeze development-only coast PPO selections, then validate and report tests.

The selection path reads only completed training statistics, best-checkpoint
metadata and checkpoint bytes. It never reads final-test outcomes. An existing
selection is immutable. Reporting separately checks all 277 preregistered cases
and the exact case/population/method Cartesian product before drawing figures.
No Interaction source is imported or changed, and no training is performed.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shlex
import statistics


ROOT = Path(__file__).resolve().parent
LINEAGES = (41, 52, 63)
BASELINES = (
    "immediate_position", "fixed240ms_position", "predictive080_position_adapter",
    "nominal_distance_gate", "v1_ppo_position_adapter",
)
PPO_LABELS = tuple(f"ppo_lineage{lineage}" for lineage in LINEAGES)
METHODS = (*BASELINES, *PPO_LABELS)
TASK_VERSION = "coast_v2"
DEVELOPMENT_BASELINE_LOSS = 8.9517578265
EXPECTED_POPULATIONS = {
    "held_out_synthetic": 256,
    "observed_initial_synthetic_target_regression": 21,
}
LABELS = {
    "immediate_position": "立即 POSITION",
    "fixed240ms_position": "固定 240 ms 后 POSITION",
    "predictive080_position_adapter": "预测器 0.08 → POSITION 适配",
    "nominal_distance_gate": "标称距离门限",
    "v1_ppo_position_adapter": "v1 PPO → POSITION 适配",
    "ppo_lineage41": "v2 PPO 谱系 41",
    "ppo_lineage52": "v2 PPO 谱系 52",
    "ppo_lineage63": "v2 PPO 谱系 63",
}
PLOT_LABELS = (
    "Immediate\nposition", "Fixed 240 ms\nposition", "Predictor .08\nadapter",
    "Nominal\ndistance gate", "V1 PPO\nposition adapter",
    "V2 PPO\nlineage 41", "V2 PPO\nlineage 52", "V2 PPO\nlineage 63",
)


def read_json(path):
    return json.loads(Path(path).read_text())


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite(value, label, *, nonnegative=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a numeric scalar")
    value = float(value)
    if not math.isfinite(value) or (nonnegative and value < 0):
        raise ValueError(f"{label} is nonfinite or outside its domain")
    return value


def integer(value, label, *, minimum=0):
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{label} must be an integer >= {minimum}")
    return value


def close(first, second):
    return math.isclose(float(first), float(second), rel_tol=1e-10, abs_tol=1e-10)


def validate_completed_candidate(stats, meta, lineage, stage, checkpoint):
    """Metadata-only completion gate; no final outcomes or training curves."""
    if (stats.get("task_version") != TASK_VERSION or meta.get("task_version") != TASK_VERSION
            or stats.get("schema_version") != 2 or meta.get("schema_version") != 2):
        raise ValueError("selection requires versioned v2 training outputs")
    if (stats.get("offline_only") is not True or stats.get("final_test_evaluated") is not False
            or meta.get("final_test_evaluated") is not False
            or stats.get("source_stable_through_completion") is not True):
        raise ValueError("training run is not complete, source-stable and development-only")
    actual = integer(stats.get("actual_training_steps"), "actual_training_steps", minimum=1)
    requested = integer(stats.get("requested_training_steps"), "requested_training_steps", minimum=1)
    initial = integer(meta.get("initial_model_timesteps"), "initial_model_timesteps")
    total = integer(stats.get("model_timesteps"), "model_timesteps", minimum=1)
    rollout = integer(meta.get("rollout_size"), "rollout_size", minimum=1)
    if total != initial + actual or not requested <= actual < requested + rollout or actual % rollout:
        raise ValueError("training completion or PPO rollout accounting is inconsistent")
    final_rows = [row for row in stats.get("evaluations", []) if row.get("label") == "final_development"]
    if (len(final_rows) != 1 or final_rows[0].get("model_timesteps") != total
            or final_rows[0].get("training_steps") != actual
            or not close(finite(final_rows[0].get("mean_loss"), "final mean loss", nonnegative=True),
                         finite(stats.get("final_mean_loss"), "final_mean_loss", nonnegative=True))):
        raise ValueError("statistics do not contain a complete final development evaluation")
    loss = finite(meta.get("development_evaluation", {}).get("mean_loss"), "best mean loss", nonnegative=True)
    best_step = integer(meta.get("checkpoint_timesteps"), "checkpoint_timesteps")
    if (not initial <= best_step <= total or stats.get("best_model_timesteps") != best_step
            or not close(loss, finite(stats.get("best_mean_loss"), "best_mean_loss", nonnegative=True))):
        raise ValueError("best checkpoint does not agree with completed development statistics")
    expected_seed = lineage + (stage - 1) * 1000
    if (meta.get("training_config", {}).get("seed") != expected_seed
            or meta["training_config"].get("eval_seed") != 19001
            or meta["training_config"].get("eval_count") != 64):
        raise ValueError("training lineage seed or fixed development protocol differs")
    digest = sha256(checkpoint)
    if digest != meta.get("checkpoint_sha256"):
        raise ValueError("checkpoint changed after its development validation")
    if not isinstance(meta.get("source_hashes"), dict) or not meta["source_hashes"]:
        raise ValueError("checkpoint lacks its frozen source chain")
    return dict(
        lineage=lineage, stage=stage, label=f"ppo_lineage{lineage}",
        path=str(Path(checkpoint).resolve()), sha256=digest,
        source_hashes=meta["source_hashes"], task_version=TASK_VERSION,
        observation_version=meta["observation_version"], action_version=meta["action_version"],
        env_config=meta["env_config"], frozen_parameters_sha256=meta["frozen_parameters_sha256"],
        development_scenarios_sha256=meta["development_scenarios_sha256"],
        development_mean_loss=loss, model_timesteps=best_step,
        initial_model_timesteps=initial, actual_round_training_steps=actual,
        requested_round_training_steps=requested,
    )


def choose_candidates(candidates, rounds):
    """Pure development-only ranking with stable, predeclared tie breaks."""
    integer(rounds, "rounds", minimum=1)
    if rounds > 3:
        raise ValueError("the experiment permits at most three development rounds")
    expected = Counter((lineage, stage) for lineage in LINEAGES for stage in range(1, rounds + 1))
    if Counter((row["lineage"], row["stage"]) for row in candidates) != expected:
        raise ValueError("selection requires every completed round for all three lineages exactly once")
    for row in candidates:
        finite(row["development_mean_loss"], "development mean loss", nonnegative=True)
    signature_keys = ("source_hashes", "observation_version", "action_version", "env_config",
                      "frozen_parameters_sha256", "development_scenarios_sha256")
    for key in signature_keys:
        if any(row[key] != candidates[0][key] for row in candidates):
            raise ValueError(f"development candidates do not share a fixed experiment: {key}")
    selected = [min((row for row in candidates if row["lineage"] == lineage),
                    key=lambda row: (row["development_mean_loss"], row["stage"], row["model_timesteps"]))
                for lineage in LINEAGES]
    primary = min(selected, key=lambda row: (row["development_mean_loss"], row["lineage"]))["label"]
    return selected, primary


def select(rounds, root=ROOT):
    root = Path(root).resolve()
    target = root / "selection.json"
    if target.exists():
        raise ValueError("selection already frozen; it is immutable and cannot be overwritten")
    if (root / "final_evaluation" / "report.json").exists():
        raise ValueError("final evaluation already exists; development selection must be frozen first")
    integer(rounds, "rounds", minimum=1)
    if rounds > 3:
        raise ValueError("at most three development rounds are authorized")
    candidates = []
    for lineage in LINEAGES:
        for stage in range(1, rounds + 1):
            directory = root / f"round{stage}_seed{lineage}"
            if (directory / "failure.json").exists():
                raise ValueError(f"failed training run cannot be selected: {directory.name}")
            stats_path = directory / "statistics.json"
            meta_path = directory / "model_best.metadata.json"
            checkpoint = directory / "model_best.zip"
            if not all(path.is_file() for path in (stats_path, meta_path, checkpoint)):
                raise ValueError(f"training round is incomplete: {directory.name}")
            candidate = validate_completed_candidate(
                read_json(stats_path), read_json(meta_path), lineage, stage, checkpoint)
            candidate.update(statistics_sha256=sha256(stats_path), metadata_sha256=sha256(meta_path))
            candidates.append(candidate)
    selected, primary = choose_candidates(candidates, rounds)
    result = dict(
        schema_version=2, task_version=TASK_VERSION, created_utc=datetime.now(timezone.utc).isoformat(),
        selection_split="development_only", rounds=rounds, candidates=candidates, selected=selected,
        primary=primary, final_test_seen=False, flight_approved=False,
        selection_rule="lowest fixed-development mean loss per lineage; ties use earlier round, then checkpoint step; primary ties use lineage number",
    )
    payload = json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n"
    with target.open("x") as stream:
        stream.write(payload)
    for row in selected:
        print("--policy " + shlex.quote(f"{row['label']}={row['path']}"))
    print("Primary (frozen on development only):", primary)
    return result


def verify_selection(selection, root):
    if (selection.get("selection_split") != "development_only"
            or selection.get("final_test_seen") is not False
            or selection.get("task_version") != TASK_VERSION):
        raise ValueError("missing development-only frozen selection")
    expected_selected, expected_primary = choose_candidates(selection["candidates"], selection["rounds"])
    if selection.get("selected") != expected_selected or selection.get("primary") != expected_primary:
        raise ValueError("selection no longer matches its frozen development scores")
    for row in selection["candidates"]:
        expected = root / f"round{row['stage']}_seed{row['lineage']}" / "model_best.zip"
        if Path(row["path"]).resolve() != expected.resolve():
            raise ValueError("selected checkpoint is outside its declared training round")
        for path, digest in (
            (expected, row["sha256"]),
            (expected.with_suffix(".metadata.json"), row["metadata_sha256"]),
            (expected.parent / "statistics.json", row["statistics_sha256"]),
        ):
            if sha256(path) != digest:
                raise ValueError(f"frozen development artifact changed: {path}")


def check_source_files(hashes, repository):
    if not isinstance(hashes, dict) or not hashes:
        raise ValueError("missing source hashes")
    for name, digest in hashes.items():
        relative = Path(name)
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("source hash must identify a repository-relative file")
        if sha256(repository / relative) != digest:
            raise ValueError(f"source changed since the frozen experiment: {name}")


def validate_manifest(selection, preregistered, manifest, baseline_manifest):
    for key in ("task_version", "cases", "config", "frozen_report", "population_seed", "sources", "source_sha256"):
        if manifest.get(key) != preregistered.get(key):
            raise ValueError(f"final evaluation differs from preregistration: {key}")
    if (manifest.get("task_version") != TASK_VERSION or manifest.get("offline_only") is not True
            or manifest.get("position_model_identified") is not False):
        raise ValueError("evaluation incorrectly claims identified or physical position dynamics")
    cases = manifest["cases"]
    if Counter(case["population"] for case in cases) != Counter(EXPECTED_POPULATIONS):
        raise ValueError("final evaluation must contain all 256 held-out and 21 regression cases")
    identities = Counter((case["name"], case["population"]) for case in cases)
    if any(count != 1 for count in identities.values()):
        raise ValueError("preregistered case identities are duplicated")
    if not close(manifest["config"]["horizon_s"], 3.0) or not close(manifest["config"]["terminal_window_s"], .3):
        raise ValueError("expected common 3 s horizon and full final 0.3 s window")
    checkpoint_rows = manifest.get("checkpoints", [])
    expected_labels = (*PPO_LABELS, "v1_ppo_position_adapter")
    if Counter(row["label"] for row in checkpoint_rows) != Counter(expected_labels):
        raise ValueError("final checkpoint labels are missing, duplicated or unexpected")
    checkpoints = {row["label"]: row for row in checkpoint_rows}
    for selected in selection["selected"]:
        actual = checkpoints[selected["label"]]
        if (actual["sha256"] != selected["sha256"]
                or Path(actual["path"]).resolve() != Path(selected["path"]).resolve()):
            raise ValueError("final PPO checkpoint does not match frozen development selection")
        if actual.get("legacy_semantics_adapted") is not False:
            raise ValueError("v2 checkpoint incorrectly marked as a legacy adapter")
        if selected["env_config"] != manifest["config"]:
            raise ValueError("evaluation uses a different environment from selected training")
        if any(manifest["source_sha256"].get(key) != digest for key, digest in selected["source_hashes"].items()):
            raise ValueError("evaluation source differs from selected checkpoint training source")
    legacy_development = [row for row in baseline_manifest.get("checkpoints", [])
                          if row["label"] == "v1_ppo_position_adapter"]
    if len(legacy_development) != 1:
        raise ValueError("development baseline manifest must freeze one v1 adapter checkpoint")
    legacy = checkpoints["v1_ppo_position_adapter"]
    if (legacy.get("legacy_semantics_adapted") is not True
            or any(legacy[key] != legacy_development[0][key] for key in ("path", "sha256"))):
        raise ValueError("v1 adapter changed since the fixed development baseline")
    return checkpoints


def summarize_rows(rows):
    """Recompute all evaluator aggregates to reject incomplete or stale summaries."""
    aggregates = []
    for population, method in sorted({(row["population"], row["method"]) for row in rows}):
        group = [row for row in rows if (row["population"], row["method"]) == (population, method)]
        losses = sorted(row["loss"] for row in group)
        qindex = .95 * (len(losses) - 1)
        low, high = math.floor(qindex), math.ceil(qindex)
        p95 = losses[low] + (qindex - low) * (losses[high] - losses[low])
        aggregates.append(dict(
            population=population, method=method, count=len(group),
            mean_loss=statistics.fmean(losses), p95_loss=p95,
            joint_settled_count=sum(row["joint_settled_in_simulation"] for row in group),
            no_reverse_settled_count=sum(row["joint_settled_in_simulation"] and row["no_reverse_in_simulation"] for row in group),
            reversal_count=sum(not row["no_reverse_in_simulation"] for row in group),
            reverse_speed_exceeds_004_count=sum(row["min_velocity_m_s"] < -.04 for row in group),
            rollback_exceeds_002_count=sum(row["max_rollback_m"] > .02 for row in group),
            reversal_before_position_count=sum(row["reversal_before_position"] for row in group),
            reversal_after_position_count=sum(row["reversal_after_position"] for row in group),
            mean_terminal_max_abs_position_error_m=statistics.fmean(row["terminal_max_abs_position_error_m"] for row in group),
            mean_terminal_max_abs_velocity_m_s=statistics.fmean(row["terminal_max_abs_velocity_m_s"] for row in group),
            mean_target_overshoot_m=statistics.fmean(row["max_target_overshoot_m"] for row in group),
            maximum_target_overshoot_m=max(row["max_target_overshoot_m"] for row in group),
            mean_rollback_m=statistics.fmean(row["max_rollback_m"] for row in group),
            maximum_rollback_m=max(row["max_rollback_m"] for row in group),
            mean_handoff_after_s=statistics.fmean(row["handoff_after_s"] for row in group),
        ))
    return aggregates


def validate_rows(result, manifest):
    expected = Counter((case["name"], case["population"], method)
                       for case in manifest["cases"] for method in METHODS)
    rows = result.get("rows", [])
    observed = Counter((row["case"], row["population"], row["method"]) for row in rows)
    if any(count != 1 for count in expected.values()) or observed != expected:
        raise ValueError("final rows must be the exact unique Cartesian product of all cases and eight methods")
    if (result.get("offline_only") is not True or result.get("flight_validated") is not False
            or result.get("position_model_identified") is not False):
        raise ValueError("report scope flags are inconsistent with an assumed offline surrogate")
    cases = {(case["name"], case["population"]): case for case in manifest["cases"]}
    for row in rows:
        for key in ("loss", "terminal_max_abs_position_error_m", "terminal_max_abs_velocity_m_s",
                    "max_target_overshoot_m", "max_rollback_m", "handoff_after_s", "horizon_s",
                    "terminal_max_abs_tilt_deg", "terminal_max_abs_rate_deg_s",
                    "integrated_abs_position_error_m_s", "final_time_s"):
            finite(row.get(key), key, nonnegative=True)
        for key in ("min_velocity_m_s", "reward", "target_position_m"):
            finite(row.get(key), key)
        for key in ("joint_settled_in_simulation", "no_reverse_in_simulation",
                    "reversal_before_position", "reversal_after_position"):
            if type(row.get(key)) is not bool:
                raise ValueError(f"{key} must be a boolean, not a missing/coerced metric")
        if not close(row["horizon_s"], 3.0) or not close(row["reward"], -row["loss"]):
            raise ValueError("row lacks the full common horizon or terminal-loss reward identity")
        if row["no_reverse_in_simulation"] != (row["min_velocity_m_s"] >= -1e-6):
            raise ValueError("strict reversal indicator disagrees with minimum velocity")
        config = manifest["config"]
        joint = (row["terminal_max_abs_position_error_m"] <= config["position_tolerance_m"]
                 and row["terminal_max_abs_velocity_m_s"] <= config["velocity_tolerance_m_s"]
                 and row["terminal_max_abs_tilt_deg"] <= config["tilt_tolerance_deg"]
                 and row["terminal_max_abs_rate_deg_s"] <= config["rate_tolerance_deg_s"])
        if row["joint_settled_in_simulation"] != joint:
            raise ValueError("joint-settling flag disagrees with complete terminal-window extrema")
        scenario = cases[(row["case"], row["population"])]["scenario"]
        target = scenario["braking"]["snapshot"]["position_m"] + scenario["target_distance_m"]
        if not close(row["target_position_m"], target):
            raise ValueError("row did not retain its original fixed target")
        first_decision = max(scenario["braking"]["first_decision_s"],
                             scenario["braking"]["measurement_delay_s"])
        if not close(row["final_time_s"], first_decision + config["horizon_s"]):
            raise ValueError("actual final timestamp does not reach the common physical-time endpoint")
        reconstructed_loss = (
            row["terminal_max_abs_position_error_m"] / .03
            + row["terminal_max_abs_velocity_m_s"] / .04
            + 2 * row["max_target_overshoot_m"] / .03
            + row["max_rollback_m"] / .03
            + .2 * row["integrated_abs_position_error_m_s"] / (config["horizon_s"] * .1))
        if not close(row["loss"], reconstructed_loss):
            raise ValueError("terminal loss does not reproduce its five frozen metric components")
    recomputed = summarize_rows(rows)
    provided = result.get("aggregate", [])
    keys = Counter((row["population"], row["method"]) for row in provided)
    if keys != Counter((row["population"], row["method"]) for row in recomputed):
        raise ValueError("aggregate populations/methods are missing, duplicated or unexpected")
    lookup = {(row["population"], row["method"]): row for row in provided}
    for expected_row in recomputed:
        actual = lookup[(expected_row["population"], expected_row["method"])]
        for key, value in expected_row.items():
            if isinstance(value, (int, float)):
                if not close(finite(actual.get(key), key), value):
                    raise ValueError(f"aggregate does not reproduce complete paired rows: {key}")
            elif actual.get(key) != value:
                raise ValueError(f"aggregate label mismatch: {key}")
    return recomputed


def table(aggregate, population, primary):
    lookup = {row["method"]: row for row in aggregate if row["population"] == population}
    lines = [
        "| 方法 | N | 损失↓ | 联合稳定↑ | 无反向且稳定↑ | 严格反向↓ | 明显反向↓ |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for method in METHODS:
        row = lookup[method]
        label = LABELS[method] + ("（开发集主候选）" if method == primary else "")
        lines.append(
            f"| {label} | {row['count']} | {row['mean_loss']:.4f} | {row['joint_settled_count']} | "
            f"{row['no_reverse_settled_count']} | {row['reversal_count']} | {row['reverse_speed_exceeds_004_count']} |")
    lines += ["", "停点精度、超调与回退：", "",
              "| 方法 | 尾窗最大位置误差均值 cm↓ | 尾窗最大速度均值 m/s↓ | 超调均值 / 最坏 cm↓ | 回退均值 / 最坏 cm↓ |",
              "|---|---:|---:|---:|---:|"]
    for method in METHODS:
        row = lookup[method]
        label = LABELS[method] + ("（开发集主候选）" if method == primary else "")
        lines.append(
            f"| {label} | "
            f"{100*row['mean_terminal_max_abs_position_error_m']:.2f} | {row['mean_terminal_max_abs_velocity_m_s']:.4f} | "
            f"{100*row['mean_target_overshoot_m']:.2f} / {100*row['maximum_target_overshoot_m']:.2f} | "
            f"{100*row['mean_rollback_m']:.2f} / {100*row['maximum_rollback_m']:.2f} |")
    return lines


def frozen_development_history(root, candidate):
    """Plot the same immutable development evidence used by selection."""
    path = Path(root) / f"round{candidate['stage']}_seed{candidate['lineage']}" / "statistics.json"
    if sha256(path) != candidate["statistics_sha256"]:
        raise ValueError("development curve statistics changed after selection")
    history = read_json(path).get("evaluations")
    if not isinstance(history, list) or not history:
        raise ValueError("frozen statistics lack development curve evaluations")
    return history


def make_figures(root, selection, aggregate):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    colors = ("#087E8B", "#B45F42", "#695AA6")
    fig, ax = plt.subplots(figsize=(10, 5), constrained_layout=True)
    all_losses = [DEVELOPMENT_BASELINE_LOSS]
    for lineage, color in zip(LINEAGES, colors):
        offset = 0
        for stage in range(1, selection["rounds"] + 1):
            candidate = next(row for row in selection["candidates"]
                             if (row["lineage"], row["stage"]) == (lineage, stage))
            # The separately written JSONL is not selection-hashed. Plot only
            # the evaluations inside the already frozen statistics artifact.
            history = frozen_development_history(root, candidate)
            if not history or history[-1].get("label") != "final_development":
                raise ValueError("development curve is incomplete")
            points = []
            for row in history:
                step = integer(row["training_steps"], "development curve training steps")
                loss = finite(row["mean_loss"], "development curve loss", nonnegative=True)
                if step > candidate["actual_round_training_steps"]:
                    raise ValueError("development curve exceeds the complete round budget")
                points.append((offset + step, loss))
            if any(right[0] < left[0] for left, right in zip(points, points[1:])):
                raise ValueError("development curve moves backwards in sampled work")
            all_losses.extend(loss for _, loss in points)
            ax.plot([step for step, _ in points], [loss for _, loss in points],
                    marker=".", linewidth=1.5, color=color,
                    label=f"PPO lineage {lineage}" if stage == 1 else None)
            offset += candidate["actual_round_training_steps"]
    ax.axhline(DEVELOPMENT_BASELINE_LOSS, color="#555555", linestyle="--",
               label=f"Best fixed development baseline ({DEVELOPMENT_BASELINE_LOSS:.4f})")
    if all(loss > 0 for loss in all_losses):
        ax.set_yscale("log")
        ylabel = "Development mean loss (log scale; lower is better)"
    else:
        ylabel = "Development mean loss (lower is better)"
    ax.set(xlabel="Cumulative sampled transitions per lineage (including discarded updates)",
           ylabel=ylabel, title="Development only: each continuation restores its lineage's prior best")
    ax.margins(x=.025, y=.08)
    ax.grid(alpha=.2, which="both")
    ax.legend(fontsize=8, loc="best")
    fig.savefig(root / "development_curves.png", dpi=180)
    plt.close(fig)

    lookup = {row["method"]: row for row in aggregate if row["population"] == "held_out_synthetic"}
    panel_specs = (
        ("mean_loss", "Mean objective loss (lower)", 1.0, False),
        ("joint_settled_count", "Joint settled in final 0.3 s (%)", 100.0, True),
        ("no_reverse_settled_count", "Settled with no earlier reversal (%)", 100.0, True),
        ("reverse_speed_exceeds_004_count", "Reverse speed > 0.04 m/s (%)", 100.0, True),
        ("mean_terminal_max_abs_position_error_m", "Mean terminal max |position error| (cm)", 100.0, False),
        ("mean_terminal_max_abs_velocity_m_s", "Mean terminal max |velocity| (m/s)", 1.0, False),
    )
    bar_colors = ["#9A9D9F"] * 4 + ["#A68469"] + list(colors)
    primary_index = METHODS.index(selection["primary"])
    fig, axes = plt.subplots(2, 3, figsize=(17, 10), constrained_layout=True)
    for ax, (field, title, scale, is_count) in zip(axes.ravel(), panel_specs):
        values = [lookup[method][field] * scale / (lookup[method]["count"] if is_count else 1)
                  for method in METHODS]
        bars = ax.bar(np.arange(len(METHODS)), values, color=bar_colors)
        bars[primary_index].set_edgecolor("#171717")
        bars[primary_index].set_linewidth(2.2)
        ax.set_xticks(np.arange(len(METHODS)), PLOT_LABELS, rotation=35, ha="right", fontsize=8)
        ax.set_title(title, fontsize=10)
        ax.set_axisbelow(True)
        ax.grid(axis="y", alpha=.2)
        ax.margins(y=.12)
    fig.suptitle(
        "SIMULATOR ONLY: all 8 methods on 256 paired held-out cases, common 3 s horizon\n"
        f"Black outline = {selection['primary']}, chosen on development (not on these test results)", fontsize=13)
    fig.savefig(root / "heldout_comparison.png", dpi=180)
    plt.close(fig)


def report(evaluation, root=ROOT):
    root = Path(root).resolve()
    evaluation = Path(evaluation)
    if not evaluation.is_absolute():
        evaluation = root / evaluation
    selection = read_json(root / "selection.json")
    verify_selection(selection, root)
    prereg_path = root / "final_population_preregistered" / "manifest.json"
    preregistered = read_json(prereg_path)
    manifest = read_json(evaluation / "manifest.json")
    baseline_manifest = read_json(root / "development_baselines" / "manifest.json")
    checkpoints = validate_manifest(selection, preregistered, manifest, baseline_manifest)
    repository = root.parent.parent
    check_source_files(manifest["source_sha256"], repository)
    for checkpoint in checkpoints.values():
        if sha256(checkpoint["path"]) != checkpoint["sha256"]:
            raise ValueError("final evaluated checkpoint bytes have changed")
    if sha256(manifest["frozen_report"]["path"]) != manifest["frozen_report"]["sha256"]:
        raise ValueError("frozen model input bytes have changed")
    # Final outcomes are read only after the frozen selection and input checks.
    result = read_json(evaluation / "report.json")
    aggregate = validate_rows(result, manifest)
    development = read_json(root / "development_baselines" / "report.json")
    dev_rows = [row for row in development["aggregate"] if row["population"] == "development"]
    if Counter(row["method"] for row in dev_rows) != Counter(BASELINES):
        raise ValueError("development baseline report must contain all five fixed methods")
    best_baseline = min(dev_rows, key=lambda row: finite(row["mean_loss"], "baseline loss", nonnegative=True))
    if not close(best_baseline["mean_loss"], DEVELOPMENT_BASELINE_LOSS):
        raise ValueError("fixed development baseline differs from the predeclared 8.9517578265 reference")
    primary = selection["primary"]
    heldout = {row["method"]: row for row in aggregate if row["population"] == "held_out_synthetic"}
    p = heldout[primary]
    comparator = heldout[best_baseline["method"]]
    regression_primary = next(row for row in aggregate
                              if row["population"] == "observed_initial_synthetic_target_regression"
                              and row["method"] == primary)
    total_steps = sum(row["actual_round_training_steps"] for row in selection["candidates"])
    lines = [
        "# V2：固定 coast 终点与位置接管的离线 PPO 结果", "",
        "**只在明确假设的模拟器中训练与评估；没有接入飞控、部署、重新标定或获得实机飞行许可。**", "",
        "技术上这是 simulator-trained PPO：策略与模拟器交互学习，并非仅从固定数据集学习的 offline RL。", "",
        f"已完成 {selection['rounds']} 个开发轮次、3 个训练谱系，共采样 {total_steps:,} 个训练决策步。"
        "首轮从头训练；仅后续第 2/3 轮从本谱系既有最佳检查点热启动，并重新开始预先声明的随机流。丢弃的更新仍计入训练预算。", "",
        f"主候选固定为 `{primary}`（{LABELS[primary]}）。它在读取最终测试结果之前，仅按同一 64 场景开发集的平均损失选定。"
        "所有 3 个 PPO 谱系和全部 5 个固定基线均公布；即使其他谱系最终测试更好，也不更换主候选。", "",
        "## 结论：训练完成，但尚未达到“不回飞地停住”", "",
        f"主候选在合成测试中的平均损失、末尾停稳比例及平均回退优于开发集预选基线，但只有 "
        f"{p['no_reverse_settled_count']}/256 个案例同时满足全过程不反向和末尾联合稳定；"
        f"{p['reverse_speed_exceeds_004_count']}/256 个案例出现超过 0.04 m/s 的反向速度。"
        f"最大回退为 {100*p['maximum_rollback_m']:.2f} cm，基线为 {100*comparator['maximum_rollback_m']:.2f} cm。"
        "平均值改善没有消除尾部风险。", "",
        f"实测初态搭配合成目标的回归组中，主候选 {regression_primary['joint_settled_count']}/21 个案例末尾停稳，"
        f"但有 {regression_primary['reversal_count']}/21 个曾经反向，且损失未优于原适配基线。"
        "因此这次交付是可复现的离线实验，不是可直接部署的飞行策略。第三轮没有改进开发最优值，按预定三轮预算停止。", "",
        f"逐案例记录显示，主候选在首次实际发送 POSITION 命令之前出现反向的合成案例数为 "
        f"{p['reversal_before_position_count']}，之后为 {p['reversal_after_position_count']}。"
        "命令发送不等于控制已生效；旧姿态、在途命令、反馈延迟和位置环都可能参与后续运动，因此不能仅凭时序断言位置环是唯一原因。"
        f"详见[独立审计与分组诊断]({root / 'audit_final.md'})。", "",
        "## 核心定义与可解释范围", "",
        "- 所有方法使用完全相同的原始固定目标、初态、隐藏模型、测量偏差/时延、指令时延及位置 PD 替代模型。目标在动作前固定，不在接管时重新锁定。",
        "- 动作 0 继续反向 20° 制动；动作 1 不可逆地请求直接 POSITION 接管。位置环可产生正、反向校正，但 PPO 不直接选择任意推力。",
        "- 每个 episode 从首个有效决策起完整模拟共同的 3 s；不会因反向或暂时稳定而提前终止。联合稳定要求最后完整 0.3 s 的位置误差 ≤3 cm、速度 ≤0.04 m/s、倾角 ≤3°、角速度 ≤5°/s。",
        "- “严格反向”采用数值容差：全过程最小有符号速度 <−10⁻⁶ m/s；“明显反向”要求最小速度 <−0.04 m/s。联合稳定与“从未反向且稳定”分别统计。",
        "- 尾窗位置/速度列先取每个 episode 最后 0.3 s 的最大绝对值，再对 episode 求均值；不是只看最后一个采样点。超调和回退使用全过程最大值。", "",
        "开发场景存在明确外推：64 个开发初态中有 23 个速度超过既有位置捕获实测最大值 0.414944 m/s。随机模型参数范围是探索性假设，不是由物理数据校准的置信范围。", "",
        "## 256 个未用于选择或调参的合成配对测试", "",
        "这是新随机抽样的假设模型组合，不是新的实测飞行数据，也不是已验证的实机误差范围。", "",
        *table(aggregate, "held_out_synthetic", primary), "",
        f"对照开发集预先最优的固定基线 `{best_baseline['method']}`：主候选独立测试平均损失为 {p['mean_loss']:.4f}"
        f"（基线 {comparator['mean_loss']:.4f}）；联合稳定 {p['joint_settled_count']}/256"
        f"（基线 {comparator['joint_settled_count']}/256）；明显反向 {p['reverse_speed_exceeds_004_count']}/256"
        f"（基线 {comparator['reverse_speed_exceeds_004_count']}/256）。这几个指标可能存在取舍，不能把低损失等同于安全或物理稳定。", "",
        "## 21 个回归案例（7 个实测初态 × 3 个合成目标）", "",
        "7 个既有实测初态分别分配 0.08/0.16/0.28 m 的假设目标间距，之后轨迹完全由模拟器生成。"
        "这些间距不是日志中重建的 coast 终点；本组不能称为新的物理独立测试，不能与上表合并为实机成功率。", "",
        *table(aggregate, "observed_initial_synthetic_target_regression", primary), "",
        "## v1 适配与位置模型的证据边界", "",
        "`v1_ppo_position_adapter` 仅复用 v1 策略及其精确 114 维因果观测前缀，"
        "但把原本的 LEVEL 动作改为直接 POSITION 请求。预测器的释放动作也采用相同适配。"
        "因此本报告的 v1 适配结果不是原生 v1 动力学结果，不能与原先“回水平后观察 1.5 s”的 v1 损失或稳定率直接横比。", "",
        "位置 PD 的增益、饱和、反馈时延与激活时延是明确声明的替代模型假设，不是已辨识的机载位置响应。"
        f"详见[来源证据与假设]({root / 'POSITION_MODEL_EVIDENCE.md'})；该审计报告记录真实位置捕获评估只有 4/12 项通过，保存结果被拒绝，不能据此称位置模型已经校准。", "",
        "## 补充开发诊断（不参与选择或公平基线）", "",
        f"[接管时刻粗扫描]({root / 'development_switch_scan.md'})仅在前 16 个开发场景上离散扫描接管时刻，"
        "并在事后查看完整轨迹：扫描中 11/16 个场景存在至少一个联合稳定时刻，2/16 个场景存在至少一个从未反向且联合稳定的时刻。"
        "这是使用完整后续轨迹的粗粒度事后诊断，不是因果控制策略、公平比较基线或不可行性证明；未用于本脚本的检查点选择。", "",
        "## 下一步与使用限制", "",
        "1. 先用独立实测验证位置命令接管、指令/测量时序、饱和与内部控制状态，建立可复核的位置响应模型及有效范围。缓存 PID 参数不等于已辨识的加速度域 PD 系数。",
        "2. 在不改写原始 coast 目标的前提下，分别检查停点误差、速度、超调、回退和持续稳定；补充定位缺口、通信延迟、多轴耦合与载荷变化验证。",
        "3. 当前有限 3 s 模拟及最后 0.3 s 的联合稳定不保证无限期悬停、不反向或实机安全，不支持直接替换飞行控制逻辑。任何真实部署须另获授权并先完成物理模型验证。",
        "4. 最终测试若被后续改进使用，应降为开发资料，下一版另留新独立测试。", "",
        "## 审计与可复现文件", "",
        f"- 冻结选择：`selection.json`，SHA-256 `{sha256(root / 'selection.json')}`。",
        f"- 预注册全部 277 案例：`final_population_preregistered/manifest.json`，SHA-256 `{sha256(prereg_path)}`。",
        f"- 最终配对结果：`{evaluation.name}/report.json`，SHA-256 `{sha256(evaluation / 'report.json')}`。",
        "- 汇总前已核对：全部案例/人群/方法组合恰好各一行、全部选择检查点及源码哈希、冻结模型、环境配置和预注册场景完全一致；实际终止时刻达到共同 3 s 终点，终止损失由五项指标重新计算，表格聚合由逐案例结果重新计算。",
        "- 全部 2,216 行保留逐 episode 指标，但轨迹文件只保留每种方法前两个案例的抽样状态与命令（共 16 条诊断轨迹），不是完整原始轨迹档案。独立审计能复核全部指标聚合及这些诊断轨迹，不能据此重建所有 episode 的原始轨迹极值。",
        "- 各轮 `statistics.json` / `model_best.metadata.json` / `training_scenarios.jsonl` 保存训练预算、开发选择、源代码与实际随机场景审计。", "",
        f"![开发集全范围曲线]({root / 'development_curves.png'})", "",
        f"![全部方法的独立合成测试比较]({root / 'heldout_comparison.png'})", "",
    ]
    make_figures(root, selection, aggregate)
    check_source_files(manifest["source_sha256"], repository)
    (root / "RESULTS_zh.md").write_text("\n".join(lines))
    print(root / "RESULTS_zh.md")
    return aggregate


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--select-rounds", type=int, choices=(1, 2, 3))
    group.add_argument("--report", type=Path, help="evaluation directory, relative to this experiment or absolute")
    args = parser.parse_args(argv)
    if args.select_rounds is not None:
        select(args.select_rounds)
    else:
        report(args.report)


if __name__ == "__main__":
    main()
