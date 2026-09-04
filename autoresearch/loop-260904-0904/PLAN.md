# V2 bounded offline target-and-handoff experiment

[autoresearch] mode: classic

User authorization: continue offline development/training by adding the fixed coast endpoint and position takeover. No flight, SSH, runtime controller or calibration changes, deployment, commit or push.

## Scope and model evidence

V1 sources and checkpoints stay byte-identical. Add a separate Gymnasium environment, PPO trainer and paired evaluator; reuse the existing independent RK4 attitude plant and frozen measured attitude/motion model. Original flight dependencies remain unchanged.

Historical position-capture trials provide empirical outcomes but **not an identified position-controller dynamics model**. The v2 outer-loop PD is explicitly an uncalibrated surrogate. Its gains/delays and randomization bounds are assumptions, not a certified physical uncertainty envelope. See `POSITION_MODEL_EVIDENCE.md` for evidence and rejected calibration outcomes. Completing this experiment is not approval to fly a policy.

## Task frozen before training

- Single projected travel axis; both +Y/-Y signs represented, direction never redefined after reversal.
- Physically integrated acceleration/level preludes replace v1's independently sampled speed/tilt/rate. Previously SENT command history is preserved; unavailable measurement history stays masked.
- Fixed externally supplied target gap sampled in [0.04, 0.40] m at coast entry, independent of hidden plant parameters. This represents the output of an upstream virtual-coast computation, not a newly fitted force-to-coast equation. It is never moved at position takeover or after overshoot.
- Binary actions: keep -20 degree braking for another 10 ms, or irreversibly hand control to the fixed-target position surrogate. No posture/state reset, teleport, cancellation of pending commands or forward-pulse learning.
- Maximum pre-handoff braking 0.5 s. All methods run to the SAME physical horizon, 3.0 s after their common first decision, even after reversal or apparent success.
- Position surrogate: bounded acceleration `kp*(target-measured_position) - kd*measured_velocity`, converted to bounded tilt through the same delayed attitude plant. Nominal kp=4/s^2, kd=3/s, max acceleration=2 m/s^2; random ranges [2,6], [2,4.5], [1,3]. Additional feedback delay [0,.04] s, takeover activation delay [0,.08] s; 20 ms position-loop ticks. These are declared assumptions, not transplanted firmware PID gains.
- Policy observation retains the v1 114-value causal prefix and appends target-relative distance, fixed target gap and remaining time. Hidden true model, delays and surrogate gains are not policy inputs.

Terminal loss, fixed before training:

```text
terminal_max_abs_position_error / .03
+ terminal_max_abs_speed / .04
+ 2 * max_target_overshoot / .03
+ max_rollback / .03
+ .2 * integral(abs(target-position)) / (3.0 * .1)
```

Terminal window is 0.3 s. Additionally report final-window joint settling (position error <=.03 m, speed <=.04 m/s, tilt <=3 degrees, rate <=5 degrees/s), all-history reversal and reversal before/after position activation, maximum overshoot/rollback, terminal errors and dwell. Success is finite-horizon simulated settling only. Do not equate low scalar loss or zero reversal with reaching the coast endpoint.

## Iteration budget and splits

At most THREE development rounds for three independent lineages 41/52/63: 32,768 initial steps, then 65,536 and 98,304 additional steps. Warm restarts use new RNG streams 1041/1052/1063 then 2041/2052/2063. PPO 64x64 MLP, gamma=1, GAE=1, terminal reward only; no test-driven reward changes. Stop earlier if a technical/data blocker or clear plateau makes additional rounds uninformative.

Fixed development population: 64 scenarios, seed19001. Holdout: 256 scenarios, NEW seed804271, never evaluated until per-lineage and primary checkpoint choices are frozen on development loss. V1 development9001 and holdout491723 are not reused as independent v2 tests. Freeze source and manifests before baseline/training. Report every trained lineage, not only the best final-test model.

Baselines on identical v2 scenarios: immediate position takeover; fixed240ms then takeover; predictive v1 margin.08 then takeover; declared target-aware nominal-distance gate; frozen v1 primary PPO with its114-value observation prefix, mapping its LEVEL action to POSITION takeover. The last two legacy adaptations are labeled changed semantics, not native-v1 end-to-end scores. No baseline receives hidden true plant/surrogate parameters.

Observed-state regressions and handpicked stress cases, if included, are separate from the synthetic holdout. Targets assigned to those states must be labeled synthetic rather than falsely inferred from measured log snapshots.

## Guard, bookkeeping and stop condition

Relevant old and new tests plus compilation and whitespace checks must pass. Score and command safety review is local/read-only with no credentials, deletion or outbound writes. Verify command queries the numeric development metric through `Interaction.rl_experiment_ledger`, using this run's `incumbent.json`.

The workspace was already dirty at entry (HEAD e0910b0). As in v1, autoresearch keep/discard refers to immutable checkpoint SHA256s, not automatic git commit/revert. Preserve existing work. Record baseline and each round in `loop-results.tsv`; record final results, limitations, tests and `handoff.json`. Do not promise that RL will beat a well-designed immediate-position or distance-gated controller.

Round0 completed: all104 relevant tests passed; 34 newv2 tests were also independently checked. Of the five baseline methods, the v1 position-adapter had the lowest development loss8.9517578265; it is the initial incumbent. Development and final population manifests were frozen before v2 training. Final synthetic seed804271 remains unevaluated; 21 separately labeled observed-initial-state/assigned-target regressions accompany it. Exact verify command: `.codex_tmp/braking-rl-venv/bin/python -m Interaction.rl_experiment_ledger --run autoresearch/loop-260904-0904 --metric`. It reads a local numeric JSON field only.
