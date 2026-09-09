# V2 verification and execution boundary

Verified locally on 2026-09-04. No vehicle connection, SSH, flight, controller/calibration mutation, deployment, commit or push. This is **simulator-trained PPO**, not fixed-dataset-only offline RL.

## Tests and compatibility

- Isolated RL environment: **104 tests passed**. This covers the v2 environment, trainer and evaluator plus the original RL, predictive-release and independent RK4 simulation suites. The trainer tests include actual tiny CPU PPO train/save/load/resume operations in temporary directories.
- Original flight environment: **348 discovered; 299 passed and 49 skipped** because optional Gymnasium/Stable-Baselines dependencies remain absent there. Coverage also includes calibration/readiness, position-capture runtime, timing, integration and logging.
- Independent review exercised all 34 v2 tests and audited causal observations, fixed-target preservation, delayed position activation, full-horizon integration and selection provenance.
- Compilation passed for the six v2 source/test files and experiment summarizer. `git diff --check` passed. Tracked runtime/controller/calibration files have no changes.
- The smoke tests emit nonfatal `ResourceWarning` messages for SB3 training-scalar handles closed on process exit; this did not cause test/training failures. It is not evidence of a flight-runtime regression.
- Existing v1 work and the pre-existing `.gitignore` change were preserved. Experiment keep/discard uses immutable checkpoints rather than automatic commits in the dirty worktree.

Focused verification command, run from the repository root:

```bash
MPLCONFIGDIR="$PWD/.codex_tmp/rl-mpl-cache" \
.codex_tmp/braking-rl-venv/bin/python -m unittest \
  Interaction.tests.test_rl_coast_env \
  Interaction.tests.test_rl_coast_training \
  Interaction.tests.test_rl_coast_evaluation \
  Interaction.tests.test_rl_braking_env \
  Interaction.tests.test_rl_braking_training \
  Interaction.tests.test_rl_braking_evaluation \
  Interaction.tests.test_predictive_brake_release \
  Interaction.tests.test_braking_validation_simulator \
  Interaction.tests.test_predictive_brake_validation
```

## Predeclared protocol

- At most three rounds for three lineages, with 32,768 / 65,536 / 98,304 new transitions per round. The first round starts fresh; continuation rounds restore each lineage's prior development-best checkpoint. Discarded updates still count toward the sampled budget.
- Same fixed 64-case development population (seed 19001) for selection, with no reward/action/observation/domain changes after source freeze.
- Before training, all final cases were preregistered: 256 new synthetic cases (seed 804271), plus 21 separately labeled regressions made from seven previously observed initial states and three assigned synthetic target gaps. The latter are not 21 new measurements or independent flight tests.
- The best of all five development baselines was the v1 position adapter, loss 8.95175782651966. It is the initial incumbent, not an arbitrarily weaker comparator.
- A supplemental scan of the first 16 development cases used full future trajectories to diagnose timing sensitivity. It is not a deployable baseline, checkpoint-selection input, or proof that better actions/timings are impossible. No final-test outcomes were used in that scan.

## Final audit

- Completed all three rounds and **589,824 sampled training transitions**. Round three did not improve any lineage's development-best checkpoint. No fourth round or test-driven reward/model changes.
- Frozen primary: `ppo_lineage63`, round 2, checkpoint timestep 98,304; SHA-256 `90b67bbf9d11af862aa8b315b15ae2eab2ec0712e800b4d77fae4d62ab0dabf5`. Selection was written at 16:27:16.824641 UTC before final evaluation. Lineage 41 retained round 1 and lineage 52 retained round 2.
- Final evaluator ran **once**, completed successfully in **49.12 s**, and scored all eight methods on identical inputs. Report formatting/audit was subsequently rerun without re-evaluating policies.
- Exactly **2,216 unique `(case, population, method)` rows**: 256 held-out synthetic cases and 21 separately labeled observed-initial/assigned-target regressions, each paired across all eight methods. Final manifest cases/config/model/source fields exactly match preregistration.
- The report validator independently reconstructs each terminal loss from five recorded metric components, confirms fixed target and actual final timestamp, checks joint-settling/reversal flags against recorded extrema, and recomputes every aggregate from the complete row set.
- All 13 frozen v2 experiment source files match hashes. All 11 v1 experiment sources and **27 v1 checkpoint files** also match their recorded hashes. Selected v2 checkpoint, metadata/statistics, model input and prior adapter hashes match throughout final reporting.
- The evaluator records per-episode extrema for all rows, but stores **only 16 diagnostic trajectories** (the first two cases per method), with decimated states. This is not a full raw-trajectory archive; independent reconstruction of every episode's raw extrema is not claimed. Full-horizon and terminal-window coverage are enforced by environment assertions and focused tests, while the final report checks recorded endpoint times and metrics.
- Both generated figures were visually inspected: full development loss range (log scale, no clipping), all eight final-test methods, clearly marked development-selected primary, readable labels and simulator-only qualifications.
- The summarizer's metadata-only selection and paired-row validation were independently reviewed. Its synthetic audit checks rejected altered frozen statistics, inconsistent loss/timestamp, missing/duplicate cases, changed preregistration and stale aggregates. Development curves now use hash-verified statistics, not independent mutable history files.
- A separate final audit independently confirmed all pairings, losses, aggregates, frozen provenance and stored reversal-stage flags. The primary's synthetic reversal partition is 28 neither / 0 before-only / 228 after-only / 0 both relative to the first actual POSITION command sent. This is timing, not causal attribution. See [audit_final.md](/Users/shuqinzhu/Documents/FLS_Research/fls-cf-offboard-controller/autoresearch/loop-260904-0904/audit_final.md) and its full-precision JSON for all predeclared exploratory speed/direction subgroups and paired comparisons.

## Performance result, not flight approval

Against the development-preselected v1 POSITION adapter on 256 held-out synthetic cases, the primary reduced mean loss from **8.5195 to 6.5767**, raised final-window joint settling from **125 to 166**, and reduced mean maximum rollback from **9.52 to 6.69 cm**. However, only **24/256** cases both never reversed and settled; **180/256** exceeded 0.04 m/s reverse speed. Worst rollback increased from **40.28 to 47.59 cm**.

The 21 observed-initial/assigned-target regressions all settled eventually but all reversed; primary mean loss **7.6749** was worse than the v1 adapter **7.1225**. Thus the no-pullback stopping objective is **not met**. These are simulated continuations, not successful physical flights. Full tables, definitions and limitations are in `RESULTS_zh.md`.

## Physical-model limitation

The frozen measured model describes attitude-command response. The position loop is an explicitly assumed delayed PD surrogate, not a newly identified onboard position controller. Historical position-capture evaluation passed only 4/12 trials and was rejected; cached firmware PID coefficients do not establish effective acceleration-domain gains.

The simulated domain includes extrapolation: 23/64 development initial speeds exceed the highest initial speed (0.414944 m/s) in those position-capture trials. Randomized gains/delays are assumptions, not validated physical confidence intervals. Finite-horizon settling, no reversal, overshoot, rollback and final position/speed must remain separate claims. None approves flight deployment.
