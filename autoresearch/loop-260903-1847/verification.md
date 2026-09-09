# Verification and scope

Verified locally on 2026-09-03. No vehicle connection, remote command, flight, source commit, push or deployment.

## Tests

- Isolated RL environment: **70 tests passed** across the RL environment, trainer, evaluator, prior predictive controller and independent RK4 simulator. This includes a real 16-step CPU PPO train/save/reload/resume smoke test in a temporary directory.
- Original flight environment: **314 tests discovered, 291 passed, 23 skipped** because optional RL dependencies were intentionally not installed there. Suites cover the offline model pipeline, timing, calibration readiness, runtime integration and logging.
- `py_compile` passed for the four new modules, three new test files and experiment summarizer.
- `git diff --check` passed. Tracked runtime controller/calibration files have no diff. Existing untracked work from the previous task was preserved.

Focused command, from the repository root:

```bash
MPLCONFIGDIR="$PWD/.codex_tmp/rl-mpl-cache" \
.codex_tmp/braking-rl-venv/bin/python -m unittest \
  Interaction.tests.test_rl_braking_env \
  Interaction.tests.test_rl_braking_training \
  Interaction.tests.test_rl_braking_evaluation \
  Interaction.tests.test_predictive_brake_release \
  Interaction.tests.test_braking_validation_simulator \
  Interaction.tests.test_predictive_brake_validation
```

## Frozen experiment audit

- Three development rounds, three lineages, **1,179,648 sampled training decision steps**. No reward, architecture, observation or randomization changes during these rounds.
- Parallel training wall times per round were at most 142.37, 179.92 and 269.42 seconds (about 9.9 minutes in total, excluding setup, implementation and analysis).
- Final evaluation took **48.13 seconds**. It ran once, after `selection.json` was written.
- All **256 held-out synthetic cases** exactly match the preregistered manifest. The extra 140 observed-state regressions are separately labeled.
- Exactly **2,376 unique `(case, population, method)` rows**: six methods paired on 396 cases. No missing or duplicate pairs.
- Every episode retains the complete **1.5-second post-level tail**. Losses/rewards and all 18 aggregate groups were independently recomputed and matched.
- Three selected checkpoints, eleven source files, three source flight logs and the frozen-model input match their recorded SHA-256 hashes. Training also verified its source chain at checkpoint creation and completion.
- The selected primary is `ppo_lineage33` from development only. A different lineage has lower final-test loss, but the primary was not switched after looking at the test.
- Model checkpoints and the isolated dependency environment are ignored by Git. Summary reports, source and reproducibility manifests remain visible for later review.

## Interpretation boundary

Training completed successfully, but **the stopping-performance objective has not been met**. The selected policy trades fewer reversals for greater residual forward speed. The seven nominal measured initial states produce no reversal but also no near-stationary result under PPO. This is simulated motion after measured initial states, not seven new successful flights.

The current action space only controls when to end a fixed-angle braking pulse. It has no coast-end position target, position-controller model or active correction after leveling. Training history also differs from the measured command prehistory. These limitations must be addressed offline before considering controlled physical validation.
