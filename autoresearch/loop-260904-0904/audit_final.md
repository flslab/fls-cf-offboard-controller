# Final V2 independent audit

Audit time: 2026-09-04T16:30:25.577038+00:00. This is a post-evaluation, read-only audit. No evaluation/training was rerun, no checkpoint was selected or replaced, and no frozen Interaction source or selection file was edited.

## Outcome and concrete shortcomings

The development-frozen primary is `ppo_lineage63`, round 2, checkpoint step 98304, SHA-256 `90b67bbf9d11af862aa8b315b15ae2eab2ec0712e800b4d77fae4d62ab0dabf5`. Its held-out synthetic mean loss is 6.5767 vs 8.5195 for the development-preferred fixed comparator `v1_ppo_position_adapter` (22.80% lower). Joint settling is 166/256 vs 125/256.

This is not safe no-reversal capture: primary strict reversal is 228/256 (89.06%), reverse speed exceeds 0.04 m/s in 180/256, and only 24/256 (9.38%) both settle and never reverse. 90/256 do not meet the joint terminal criteria.

Primary maximum rollback is **0.475865 m**, worse than the v1 adapter's 0.402834 m, despite a lower mean rollback (6.69 vs 9.52 cm). Its maximum overshoot is 0.318702 m and worst reverse velocity is -0.920589 m/s. Scalar-loss improvement does not imply improvement in every case or every tail-risk metric.

On the separate 21 regression cases (7 previously seen measured initial states × 3 synthetic target gaps), primary loss 7.6749 is worse than v1 adapter 7.1225 and predictor adapter 7.0940. All eight methods settle in all 21 cases, but all eight also reverse in every case: no method has a no-reversal settled regression case.

## Provenance and numerical checks

- Exactly 2216 unique rows = 277 cases × 8 methods, with 256 synthetic test cases and 21 separately labeled regressions.
- Nine candidate checkpoint, metadata and statistics hashes checked; three development-only lineage minima and the primary recomputed without changing selection. Per-round budgets, seeds and development-scenario hashes checked.
- Selection timestamp 2026-09-04T16:27:16.824641+00:00 precedes evaluation-manifest timestamp 2026-09-04T16:27:28Z. The timestamps and recorded metadata support the selection-before-test sequence; this audit does not independently prove historical non-access.
- Four evaluated checkpoint hashes, all 13 frozen evaluator/training dependency hashes, all 11 preserved V1 source hashes, frozen model input and regression-source input hashes checked. Final cases/config/source/input declarations equal the preregistered manifest.
- All 2,216 row losses independently reconstructed from the five stored components; maximum absolute difference 0. Reward=-loss, fixed target, actual common 3 s endpoint, settling flags and reversal flags checked. All reported aggregate fields independently recomputed.
- Important evidence limit: these are independent recomputations from complete per-row extrema. They are NOT reconstructions of all 2,216 raw trajectories. Only the first two cases per method (16 traces) were stored, and their states are decimated every tenth snapshot. All 16 first actual POSITION command-send times and command records were checked, together with sampled extrema bounds. Exact raw terminal-window coverage and omitted extrema cannot be independently recovered from these artifacts; no reruns were performed.

## Paired synthetic test results

| Method | N | Mean loss | Joint settled | Settled + no reversal | Strict reversal | Reverse >0.04 m/s | Tail max absolute error mean cm | Tail max absolute speed mean m/s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| immediate_position | 256 | 14.1770 | 138 | 17 | 238 | 217 | 2.43 | 0.0684 |
| fixed240ms_position | 256 | 14.1112 | 67 | 0 | 256 | 256 | 3.74 | 0.1125 |
| predictive080_position_adapter | 256 | 9.0873 | 128 | 10 | 246 | 233 | 2.45 | 0.0733 |
| nominal_distance_gate | 256 | 14.0466 | 120 | 17 | 238 | 217 | 3.34 | 0.0949 |
| v1_ppo_position_adapter | 256 | 8.5195 | 125 | 9 | 246 | 236 | 2.31 | 0.0675 |
| ppo_lineage41 | 256 | 6.6780 | 159 | 21 | 230 | 195 | 1.74 | 0.0462 |
| ppo_lineage52 | 256 | 6.5864 | 157 | 19 | 233 | 193 | 1.78 | 0.0475 |
| ppo_lineage63 | 256 | 6.5767 | 166 | 24 | 228 | 180 | 1.72 | 0.0461 |

Tail columns mean: maximum absolute value over each episode's final 0.3 s, then mean across episodes. Joint settling requires all four terminal extrema: position ≤3 cm, speed ≤0.04 m/s, tilt ≤3°, and angular speed ≤5°/s.

### Paired primary changes versus all five baselines

Δloss is primary minus baseline; negative is better. Gains/losses and reversal transitions are paired by exact case, not inferred by subtracting totals.

| Baseline | Mean Δloss | Primary lower / higher loss | Settling gained / lost | Reversal removed / introduced |
|---|---:|---:|---:|---:|
| immediate_position | -7.6003 | 222 / 34 | 38 / 10 | 12 / 2 |
| fixed240ms_position | -7.5345 | 239 / 17 | 100 / 1 | 28 / 0 |
| predictive080_position_adapter | -2.5106 | 224 / 32 | 40 / 2 | 19 / 1 |
| nominal_distance_gate | -7.4700 | 222 / 34 | 56 / 10 | 12 / 2 |
| v1_ppo_position_adapter | -1.9429 | 219 / 37 | 44 / 3 | 19 / 1 |

For example, versus the v1 adapter, primary loss is higher in 37/256 cases, loses settling in 3 cases and introduces reversal in 1 case, despite better overall loss, settling and reversal totals.

## Reversal stage partition: timing, not cause

Strict reversal means stored minimum signed velocity <−10⁻⁶ m/s. Before/after is split at the first actual POSITION PD command **SENT**, not the handoff request or effective actuation. The boundary is included on both sides within 10⁻¹² s. Four categories below are mutually exclusive, so an episode that reverses on both sides is counted only under “both.” Additional transport/actuation delay and residual plant dynamics prevent assigning causation to POSITION control.

| Method | Synthetic neither | Before only | After only | Both | Regression neither / before-only / after-only / both |
|---|---:|---:|---:|---:|---|
| immediate_position | 18 | 0 | 238 | 0 | 0 / 0 / 21 / 0 |
| fixed240ms_position | 0 | 0 | 187 | 69 | 0 / 0 / 21 / 0 |
| predictive080_position_adapter | 10 | 0 | 246 | 0 | 0 / 0 / 21 / 0 |
| nominal_distance_gate | 18 | 0 | 157 | 81 | 0 / 0 / 21 / 0 |
| v1_ppo_position_adapter | 10 | 0 | 246 | 0 | 0 / 0 / 21 / 0 |
| ppo_lineage41 | 26 | 0 | 230 | 0 | 0 / 0 / 21 / 0 |
| ppo_lineage52 | 23 | 0 | 233 | 0 | 0 / 0 / 21 / 0 |
| ppo_lineage63 | 28 | 0 | 228 | 0 | 0 / 0 / 21 / 0 |

The primary has zero pre-POSITION-send reversal but 228 post-send reversals in synthetic testing. That temporal concentration is not proof that the position controller alone caused them.

## Predeclared exploratory primary subgroups

Definitions were recorded before reading outcomes: initial speed = absolute 1D initial snapshot velocity, split at 0.414944 m/s; direction = stored −Y/+Y sign. All four groups are reported. The speed threshold is the maximum from earlier capture-entry observations, not an identified validity envelope, and release-initial velocity is not the same task-stage measurement. These are descriptive, non-randomized subgroup comparisons, not causal evidence, significance tests, new selection criteria or tuning feedback. Speed groups partition the 256 cases; direction groups separately partition the same 256 cases and must not be added to speed-group counts.

| Subgroup | N | Mean loss | Joint settled | Settled + no reversal | Strict reversal | Reverse >0.04 | Mean / maximum rollback cm | Mean handoff ms |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| initial_speed_le_0.414944 | 166 | 5.0066 | 113 | 22 | 140 | 105 | 4.76 / 25.88 | 53.4 |
| initial_speed_gt_0.414944 | 90 | 9.4726 | 53 | 2 | 88 | 75 | 10.25 / 47.59 | 120.7 |
| direction_minus_y | 133 | 6.6101 | 89 | 16 | 115 | 92 | 6.56 / 41.66 | 77.2 |
| direction_plus_y | 123 | 6.5405 | 77 | 8 | 113 | 88 | 6.84 / 47.59 | 76.9 |

The >0.414944 m/s group is descriptively worse on loss, reversals, speed error and rollback, with only 2/90 no-reversal settled cases. Position-error means alone do not show this deterioration. The two direction groups have similar mean losses but different settling/no-reversal counts; their different sampled initial states and hidden parameters prevent interpreting the difference as a direction effect. Full paired subgroup-versus-baseline results are in `audit_final.json`.

## Seen-initial-state / synthetic-target regressions

These are 21 simulated cases, not 21 independent measured flights; target gaps 0.08, 0.16 and 0.28 m are each assigned to the same seven initial states. Position dynamics here use nominal assumed parameters. Do not combine their success counts with synthetic test cases into a physical success rate.

| Method | Mean loss | Joint settled | Settled + no reversal | Strict reversal | Reverse >0.04 | Mean overshoot cm | Mean rollback cm |
|---|---:|---:|---:|---:|---:|---:|---:|
| immediate_position | 13.4483 | 21/21 | 0/21 | 21/21 | 21/21 | 13.03 | 13.44 |
| fixed240ms_position | 7.4335 | 21/21 | 0/21 | 21/21 | 21/21 | 6.04 | 9.49 |
| predictive080_position_adapter | 7.0940 | 21/21 | 0/21 | 21/21 | 17/21 | 6.38 | 7.82 |
| nominal_distance_gate | 9.9258 | 21/21 | 0/21 | 21/21 | 21/21 | 5.91 | 17.00 |
| v1_ppo_position_adapter | 7.1225 | 21/21 | 0/21 | 21/21 | 16/21 | 6.51 | 7.65 |
| ppo_lineage41 | 7.5920 | 21/21 | 0/21 | 21/21 | 15/21 | 7.06 | 7.99 |
| ppo_lineage52 | 7.3061 | 21/21 | 0/21 | 21/21 | 14/21 | 6.84 | 7.58 |
| ppo_lineage63 | 7.6749 | 21/21 | 0/21 | 21/21 | 15/21 | 7.25 | 7.86 |

Primary has higher loss than both legacy adapters on 14/21 regression cases (lower on 7/21), and higher mean loss than fixed 240 ms as well. Its near-zero final-window motion follows earlier overshoot/reversal and is not monotone capture.

## Worst primary synthetic cases

- loss: `held_out_synthetic:804271:166`, value 42.359950; initial speed 1.1216 m/s, assigned gap 0.1303 m, direction 1. Full row retained in JSON.
- max_rollback_m: `held_out_synthetic:804271:166`, value 0.475865; initial speed 1.1216 m/s, assigned gap 0.1303 m, direction 1. Full row retained in JSON.
- max_target_overshoot_m: `held_out_synthetic:804271:59`, value 0.318702; initial speed 1.0973 m/s, assigned gap 0.0548 m, direction -1. Full row retained in JSON.

## Limits and reproducibility

Simulation-trained PPO is not fixed-dataset offline RL. Assumed position gains, delays, saturation and plant parameter ranges were not calibrated into a physical response model. All methods share the same scenarios and fixed targets; v1/predictor LEVEL decisions are adapted to direct POSITION requests and are not native V1 trajectories. A 3 s simulator horizon and final 0.3 s settling cannot establish indefinite hovering or hardware safety. No flight approval follows from this audit.

Precision: tables round means for readability; JSON preserves numerical values. No post-test policy selection, reward change, subgroup-driven tuning, p-values or confidence claims were performed. The now-viewed test set must not be reused as independent evidence for future modifications.

### Input hashes

- `selection.json`: `7ee47bec822aa8f15a08d404e1ea2bb8e304932dcba73e39fcd8d893f5154e7b`.
- `final_population_preregistered/manifest.json`: `350d13f91b23837a760adb43d43dbf11db750bad6cff4836e248141767c589ad`.
- `final_evaluation/manifest.json`: `fde8200177afc8282aaa0c28e13665f3ab0827113a24c9f47af3bf82d7659a38`.
- `final_evaluation/report.json`: `a5f5d292557d432f440cf6696424e902fcd90acf435a53f019cc397000a06e27`.
- `final_evaluation/trajectories.json`: `48ebd37efafd07d00a92806dbf22625391525ad8d0ce945f0b3f592ce92bbd75`.

Companion machine-readable audit: [audit_final.json](/Users/shuqinzhu/Documents/FLS_Research/fls-cf-offboard-controller/autoresearch/loop-260904-0904/audit_final.json).
