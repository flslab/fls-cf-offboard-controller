"""Read-only frozen-simulator hindsight scan of 16 development cases, not a policy."""
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
sys.path.insert(0, str(REPO))

from Interaction.evaluate_rl_coast import CoastEvaluationCase, run_episode, source_hashes
from Interaction.offline_braking_selector import BrakingSnapshot, FrozenTiltModel
from Interaction.rl_braking_env import BrakingScenario
from Interaction.rl_coast_env import CoastBrakeConfig, CoastScenario


class FixedHandoff:
    def __init__(self, seconds):
        self.seconds = seconds

    def predict(self, observation, deterministic=True):
        if not deterministic or observation.shape != (117,):
            raise ValueError("unexpected observation/prediction contract")
        return int(float(observation[113]) * .5 >= self.seconds - 1e-7), None


def reconstruct(record):
    scenario = record["scenario"]
    braking = scenario["braking"]
    braking = BrakingScenario(**{
        **braking,
        "model": FrozenTiltModel(**braking["model"]),
        "snapshot": BrakingSnapshot(**braking["snapshot"]),
        "command_history": tuple(tuple(command) for command in braking["command_history"]),
    })
    return CoastEvaluationCase(
        record["name"], record["population"],
        CoastScenario(**{**scenario, "braking": braking}),
        FrozenTiltModel(**record["nominal_model"]),
    )


def main():
    output_json = ROOT / "development_switch_scan.json"
    output_md = ROOT / "development_switch_scan.md"
    if output_json.exists() or output_md.exists():
        raise ValueError("scan output already exists; this diagnostic must not overwrite it")
    manifest_path = ROOT / "development_baselines" / "manifest.json"
    raw = manifest_path.read_bytes()
    manifest = json.loads(raw)
    if manifest["population_seed"] != 19001:
        raise ValueError("only the authorized development seed may be scanned")
    records = manifest["cases"][:16]
    if len(records) != 16 or any(r["population"] != "development" for r in records):
        raise ValueError("requires exactly the first 16 development records")
    hashes = source_hashes()
    if hashes != manifest["source_sha256"]:
        raise ValueError("current simulator differs from the frozen development manifest")
    config = CoastBrakeConfig(**manifest["config"])
    config.validate()
    grid = [tick / 100 for tick in range(0, 51, 2)]
    started = time.perf_counter()
    cases = []
    for index, record in enumerate(records):
        case = reconstruct(record)
        results = []
        for seconds in grid:
            row, _ = run_episode(
                case, "diagnostic_fixed_handoff_grid", config=config,
                policy=FixedHandoff(seconds),
            )
            if abs(row["handoff_after_s"] - seconds) > 1e-9:
                raise ValueError("applied handoff differs from requested grid time")
            results.append(dict(requested_handoff_s=seconds, **row))
        best = min(results, key=lambda row: (row["loss"], row["requested_handoff_s"]))
        settled_times = [row["requested_handoff_s"] for row in results if row["joint_settled_in_simulation"]]
        safe_settled_times = [row["requested_handoff_s"] for row in results
                              if row["joint_settled_in_simulation"] and row["no_reverse_in_simulation"]]
        cases.append(dict(
            case=case.name, scenario=asdict(case.scenario),
            best_grid_loss=best["loss"], best_grid_handoff_s=best["requested_handoff_s"],
            best_loss_candidate_joint_settled=best["joint_settled_in_simulation"],
            best_loss_candidate_no_reverse=best["no_reverse_in_simulation"],
            immediate_position_loss=results[0]["loss"],
            any_grid_joint_settled=bool(settled_times),
            any_grid_joint_settled_without_reversal=bool(safe_settled_times),
            joint_settled_handoff_times_s=settled_times,
            joint_settled_without_reversal_handoff_times_s=safe_settled_times,
            rows=results,
        ))
        print(f"development switch scan {index + 1}/16: best loss={best['loss']:.4f}, joint={bool(settled_times)}, joint+nonreverse={bool(safe_settled_times)}", flush=True)
    if source_hashes() != hashes:
        raise ValueError("simulator source changed during diagnostic scan")
    aggregate = dict(
        case_count=len(cases), episodes=sum(len(case["rows"]) for case in cases),
        mean_best_grid_loss=sum(case["best_grid_loss"] for case in cases) / len(cases),
        mean_immediate_position_loss=sum(case["immediate_position_loss"] for case in cases) / len(cases),
        any_grid_joint_settled_count=sum(case["any_grid_joint_settled"] for case in cases),
        any_grid_joint_settled_without_reversal_count=sum(case["any_grid_joint_settled_without_reversal"] for case in cases),
        best_loss_candidate_joint_settled_count=sum(case["best_loss_candidate_joint_settled"] for case in cases),
        best_loss_candidate_joint_settled_without_reversal_count=sum(
            case["best_loss_candidate_joint_settled"] and case["best_loss_candidate_no_reverse"] for case in cases),
    )
    cautions = [
        "Hindsight search inspects complete future trajectories for each frozen hidden plant. It is not a causal deployable policy or a fair learned-policy baseline.",
        "Only the first 16 DEVELOPMENT cases from seed 19001 were scanned. No held-out cases or checkpoints were inspected or selected.",
        "No successful timing on this 20 ms grid does not establish impossibility between grid points, outside the scanned action family, or on hardware.",
        "Joint settled means the complete final 0.3 s satisfies all position, speed, tilt, and rate tolerances. Non-reversal is an additional whole-trajectory condition.",
        "The position controller is an explicitly assumed PD surrogate; this diagnostic does not calibrate it or establish a physical stopping envelope.",
        "Do not use this scan to choose checkpoints or change the frozen experiment objective. It is a descriptive action-timing diagnostic only.",
    ]
    report = dict(
        created_utc=datetime.now(timezone.utc).isoformat(),
        diagnostic_kind="hindsight_development_fixed_handoff_grid",
        deployable_policy=False, checkpoint_selection_used=False,
        held_out_evaluated=False, training_performed=False, offline_only=True,
        population_seed=19001, development_record_indices=list(range(16)),
        manifest_path=str(manifest_path), manifest_sha256=hashlib.sha256(raw).hexdigest(),
        diagnostic_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        source_sha256=hashes, source_unchanged=True,
        config=asdict(config), grid_handoff_times_s=grid,
        elapsed_s=time.perf_counter() - started, aggregate=aggregate,
        cases=cases, cautions=cautions,
    )
    lines = ["# Development-only hindsight handoff-time scan", "",
        "**Diagnostic only: not a deployable policy, checkpoint-selection rule, or physical guarantee.**", "",
        f"First 16 development cases, seed 19001; handoff grid 0, 0.02, ..., 0.50 s; {aggregate['episodes']} unchanged-simulator episodes.", "",
        f"Per-case minimum grid loss averages {aggregate['mean_best_grid_loss']:.4f}; immediate-position loss on these same cases averages {aggregate['mean_immediate_position_loss']:.4f}. This hindsight advantage is not a fair learned-policy improvement claim.", "",
        f"At least one timing jointly settles in {aggregate['any_grid_joint_settled_count']}/16 cases; at least one timing jointly settles without any reversal in {aggregate['any_grid_joint_settled_without_reversal_count']}/16 cases.", "",
        f"Selecting the minimum-loss timing specifically yields {aggregate['best_loss_candidate_joint_settled_count']}/16 jointly settled cases and {aggregate['best_loss_candidate_joint_settled_without_reversal_count']}/16 jointly settled without reversal. These counts need not equal the existence counts.", "",
        "| Development case | Initial speed m/s | Target gap m | Best grid time s | Best grid loss | Any joint settled | Any joint + no reversal |",
        "|---|---:|---:|---:|---:|---|---|",
    ]
    for case in cases:
        scenario = case["scenario"]
        lines.append(f"| {case['case']} | {scenario['braking']['snapshot']['velocity_m_s']:.3f} | {scenario['target_distance_m']:.3f} | {case['best_grid_handoff_s']:.2f} | {case['best_grid_loss']:.4f} | {'yes' if case['any_grid_joint_settled'] else 'no'} | {'yes' if case['any_grid_joint_settled_without_reversal'] else 'no'} |")
    lines += ["", "## Interpretation limits", ""] + [f"- {caution}" for caution in cautions]
    lines += ["", "Full per-timing metrics, exact selected scenarios, configuration, and provenance hashes are in `development_switch_scan.json`.", ""]
    output_json.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    output_md.write_text("\n".join(lines))
    print(json.dumps(aggregate, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
