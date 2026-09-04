# Bounded offline PPO braking experiment

[autoresearch] mode: classic

User authorization: train offline. No vehicle connection, live controller change, calibration write, remote sync, push or paid cloud training.

## Scope and stopping budget

Reuse the existing RK4 plant and the frozen pre-existing tilt/motion model. Add a Gymnasium training wrapper, PPO trainer, paired evaluator and tests. Existing uncommitted offline analysis files are preserved. Training dependencies are isolated in `.codex_tmp/braking-rl-venv`.

At most three development rounds, changing training budget or one documented training setting per round. Initial budgets are 65,536 / additional 131,072 / additional 196,608 steps per seed, with three independent training seeds 11, 22 and 33 if runtime permits. A plateau or technical blocker can stop earlier; no promise that RL wins. Policy checkpoints and their hashes identify candidates; unrelated source files are not reverted.

## Task and objective fixed before training

Simulator-trained PPO (not dataset-only offline RL). Binary action: continue -20-degree projected braking, or send level and irrevocably end control. 10 ms decisions, at most 0.5 s braking; every release, including forced release, is followed by 1.5 s of simulated level-command observation. No forward pulse, motor-level policy, target-position control or position handoff.

Reward is minus terminal loss:

`abs(mean_tail_velocity - 0.025) + 4*max(0, -minimum_velocity) + 2*max_rollback + 0.03*braking_duration`

All speed and distance components are in SI units. No reward or early termination for merely crossing zero. Minimum velocity/drawdown include the entire integration history after the common first decision. Gamma is 1.0 so the terminal macrostep does not reward delaying a negative terminal cost. The exact no-reverse, low-speed-level and near-stationary metrics from the previous simulator are reported separately; finite-window near stationary is not sustained position hold.

Training: independently randomized synthetic initial speeds 0.2–1.0 m/s, angles 0–8 deg and rates -80–5 deg/s; gain ratios 0.8–1.2, wn 0.7–1.3, zeta 0.7–1.3; model delay 15–55 ms, transport/measurement delay 0–20 ms; bounded sensor bias. These are exploratory ranges, not experimentally certified uncertainty bounds. States are synthetic perturbations, not reconstructions of an unobserved flight prelude.

Observation exposes causal delayed measured state, original measurement age, past observation/command history with unavailable-history masks, and elapsed time. True model and bias parameters are hidden.

## Selection versus verification

Metric: lower mean loss over 64 fixed development cases, sampler seed 9001. Keep the best development checkpoint, not the best final-test checkpoint. Report all trained seeds, not only the winner. Baselines: fixed 240 ms, original predictor with 0.08 m/s margin, and same predictor with a declared 0.025 m/s comparison margin. Baselines and PPO have identical plant, noise, delay and action range.

Final synthetic verification: 256 cases from seed 491723, never evaluated until checkpoint selection is frozen. These are held-out draws from the training distribution, not an independent physical test or certified worst-case envelope. Existing seven measured initial states and the prior 20-case stress matrix are explicitly labeled previously seen regressions, not new holdout data.

Guard: the existing 40 offline prediction/plant tests plus new environment, trainer and evaluator tests must pass. No changed runtime imports.

Verify command (after baseline preparation): `.codex_tmp/braking-rl-venv/bin/python -m Interaction.rl_experiment_ledger --run autoresearch/loop-260903-1847 --metric`

Safety screen: read local JSON metric only; no deletion, remote writes, credentials, shell interpolation or flight calls. Implementation-stage baseline must be produced before model development rounds.

Outputs: immutable run configs and versions, checkpoints, fixed scenario manifests, development curves, paired final metrics, Chinese interpretation including failures, and `loop-results.tsv`/`handoff.json`. Changing a test after seeing it requires labeling it development, never silently reusing it as independent verification.

## Working-tree adaptation

The skill's automatic commit/revert cycle is not applied to the existing dirty workspace. Keep/discard refers to independently saved model checkpoints with SHA-256 provenance; source remains uncommitted and no existing work is reverted. This bounded training task does not push or deploy. The baseline guard passed 69 tests before training. Projected-bias randomization is centered on the frozen nominal bias (plus/minus 0.002 rad), not centered on zero.

Round 2 continues each lineage's development-best checkpoint for 131,072 additional steps. Use fresh training RNG seeds 1011/1022/1033 (lineages 11/22/33) so restart does not simply repeat the initial scenario stream. Reward, domain distribution, development seed and model architecture stay unchanged. No final test has been evaluated.

Round 3 is the final allowed development round: continue development-best checkpoints for 196,608 additional steps, fresh RNG seeds 2011/2022/2033. It changes only the continuation budget/stream, not the reward, architecture, observation, or evaluation criteria. Freeze per-lineage and primary checkpoint selection after this round, then evaluate the reserved test exactly once. Seventy focused offline tests pass before this round; the base environment passes 291 related tests with 23 expected optional-RL skips.
