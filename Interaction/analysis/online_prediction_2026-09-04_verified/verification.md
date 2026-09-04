# Verification — 2026-09-04

Scope: background identification during existing attitude calibration; no RL
changes, new maneuvers, live control activation, safety-limit changes, remote
deployment, commit, push or real flight.

## Checks completed

- 216 focused Python unittest cases passed (4.349 s) across the new core,
  worker, runtime, persistence and offline CLI plus existing calibration,
  readiness, interaction, position-capture and braking-repeat tests.
- Core independent synthetic ODE tests cover parameter recovery, delayed
  pending commands, held-out separation and no future measured-state leakage.
- Runtime fake-clock Commander histories are identical with online fitting
  on/off, including rejected submissions and worker failure. Interrupted
  recovery/readiness cannot submit an unfinished trial.
- Spawn worker tests cover real numerical fitting, bounded queues, duplicate
  output protection and bounded shutdown while numerical work is busy.
- Persistence tests preserve older prediction/control fields on rejected
  reports and save only the previous frozen model, never the all-data refit.
- Python compilation and `git diff --check` passed.
- Mission YAML passed duplicate-key validation; the added online block is
  enabled while the existing 20-degree pulse schedule and safety values remain.
- All 12 source hashes in the prior RL audit's frozen dependency graph match.

## Real-data replay

Source: `translation_inertia_2026-09-03_16-07-59`, 919 samples over six trials.
Source SHA-256: `ade0292b40f7df775b4617af523cc3d45c34c4f03713ce7cdebd86de0315b57d`.

The public replay CLI exercised the production spawn worker. Report completed,
data complete, and the v2 model (training 0–3, validation 4–5) passed diagnostic
gates. Direct persistence-validator inspection accepted this same v2 with
runtime disabled; no active calibration file was written by replay.

V1 had one false reversal prediction; the history retains it. V2 predicted
both observed reversal trials. Max terminal velocity error is 0.0124353 m/s,
max endpoint error 0.0568464 m. These are fixed-window errors, not a proven
stopping-position tolerance. See report.json for per-trial and rolling metrics.

V3 is trained on all six trials and is not independently validated. BLAS
RuntimeWarnings are retained in its numerical_diagnostics; parameters and
seed costs are finite and the objective matches an independent elementwise
recomputation. Invalid/nonfinite/inconsistent fit output is rejected.

## Limits

This is same-flight sequential held-out evidence, not across-flight/general
payload/battery validation. Pi timing and radio behavior are not measured.
The position-controller takeover response is not identified. All new model
artifacts remain diagnostic only; no automatic brake-policy changes occur.

## Reproduce

From the offboard-controller repository, in its scientific Python environment:

```bash
python -m Interaction.fit_online_prediction /absolute/path/to/lb11_log.json \
  --output /absolute/path/to/a-new-directory
```

The output must be new; earlier artifacts are not overwritten.
