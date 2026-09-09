# Conditional Learning MPC for release-to-rest braking

This is the paper-structured Learning MPC path for the task:

> after every confirmed release, brake from the measured initial planar speed
> to zero as quickly as possible, then reach the measured terminal set with
> roll = pitch = 0 and roll/pitch rates = 0 (within explicit tolerances).

The implementation is offline-only. It does not own the Crazyflie command path
and does not authorize a flight.

## What is implemented

`velocity_lmpc_safe_set.py` provides the learned part of LMPC:

- only complete successful release-to-rest trajectories enter the sampled safe
  set;
- every stored state has a reverse minimum-time cost-to-go;
- +Y and -Y releases and each model/constraint fingerprint are isolated;
- an unseen initial speed may only interpolate between demonstrated lower and
  upper speeds; speed extrapolation fails closed;
- terminal dwell, full measured XY speed, roll and pitch, roll/pitch rates, reversal,
  workspace margin, telemetry age/skew, actual commands, and safe-tail bounds
  are checked before an episode is admitted;
- the terminal position-controller setpoint must be within 1 cm of the final
  measured XYZ position, so handoff cannot accelerate toward a still-ahead
  stopping target;
- a published JSON artifact is written atomically and is immutable during an
  episode.

`conditional_velocity_lmpc.py` provides the offline optimizer:

\[
\min_{u_{0:N-1},\lambda}
\sum_{k=0}^{N-1} \ell(u_k,\Delta u_k)
+ \sum_i \lambda_i J_i
\]

subject to the identified delayed attitude dynamics, hard path constraints,
and

\[
x_N=\sum_i\lambda_i x_i,\qquad
v_{release}=\sum_i\lambda_i v^i_{release},\qquad
\lambda_i\ge0,\quad\sum_i\lambda_i=1.
\]

That is the sampled-safe-set terminal constraint and learned cost-to-go that
define LMPC. It is not the older maximum-tilt pulse enumeration.

## Adaptation for different initial speeds

The paper repeats a task from the same initial condition. Real releases start
at different speeds, so this implementation is a conditional/multi-start LMPC.
Each episode freezes the release direction and maps measured velocity onto a
positive release-aligned coordinate. The local safe set brackets the release
speed with successful trajectories from the same direction and exact
model/constraint fingerprint. The optimizer enforces the bracket interpolation
in its terminal equality; it cannot silently use only the easier speed.

Monotonic iteration improvement can therefore be evaluated only within the same
speed/context cell, not across every release.

## Dynamics and state

The paper controls thrust and body rates with a nonlinear quadrotor model. This
repository currently sends roll/pitch setpoints through Crazyflie's inner
attitude loop, so the adapted state is

```text
[aligned velocity, projected tilt, projected tilt rate, delayed command memory]
```

with release speed, direction, cross-axis motion, initial attitude/rates,
battery, and workspace margin as task context or hard gates. A fractional
command delay is propagated in two substeps instead of being rounded to a whole
prediction sample. The model/step/constraint fingerprint prevents a trajectory
collected under one contract from being reused under another.

The optimizer models the braking-axis attitude only. Full measured roll and
pitch, both measured rates, cross-axis speed, the effective level input, and
every command still pending in the identified delay window are mandatory
episode-admission and handoff gates; they are not claimed as model-predicted
guarantees.

## Initial safe set (the new calibration procedure)

This is not the earlier open-loop accelerate/brake pulse fitting procedure.
LMPC cannot start from an empty safe set. Bootstrap it with actual conservative
release-to-rest episodes:

1. Keep LMPC command authority off and use the existing bounded legacy braking
   controller.
2. Begin at the lowest planned release-speed cell, separately for +Y and -Y.
3. Record the confirmed-release state, every fresh synchronized state, every
   actually sent roll/pitch command (including duplicate-state send cycles),
   and the real position-control handoff.
4. Admit an episode offline only if it has no safety/localization event, no
   reverse-speed violation, stays inside the workspace/attitude/rate/command
   limits, remains in the full measured terminal set for the configured dwell,
   and hands position control a target within 1 cm of the final measurement.
5. Collect successful lower and upper speed brackets before asking LMPC to
   interpolate an intermediate release. If a new speed or context is outside
   coverage, run the conservative baseline and add it only after post-flight
   validation.

The old fitted closed-loop response is still the prediction model for this
high-level roll/pitch interface. LMPC learns safe trajectories and cost-to-go;
it does not remove the need for dynamics. Replacing that fitted response needs
a separately validated dynamics model with the same delayed-state contract.

The current `lb11` mission has `coast_velocity_braking_enabled: true`, so its
legacy release controller sends `velocity_hover`, not roll/pitch. Those runs
are deliberately rejected by this attitude-input LMPC pipeline. Do not call
that a calibration failure and do not relabel the hover input as attitude. A
compatible initial safe set needs a separately authorized conservative
attitude-controller collection run (or a different LMPC plant/input contract).

## Offline extraction

`velocity_lmpc_replay.py` is the only flight-log-to-safe-set bridge. It ignores
the legacy `Translation Position Hold Resumed` event. A successful segment must
end with `Release Dataset Terminal Dwell Complete`, emitted only after the final
observer row records the measured dwell and the actual position command. A
recontact, early handoff, unsafe post-terminal command, or detector rearm emits
`Release Dataset Episode Closed` with a rejected outcome; one rejected episode
does not contaminate other complete episodes in the same log.

The extractor never overwrites an artifact:

```bash
venv/bin/python -m Interaction.velocity_lmpc_replay \
  --input /absolute/path/to/complete-flight-log.json \
  --output /absolute/path/to/new-safe-set.json \
  --model-fingerprint reduced-v1:<sha256> \
  --state-dimension 9 \
  --prediction-step-s 0.02 \
  --command-delay-s 0.12
```

The fingerprint and dimension must come from the exact frozen model and
`ConditionalVelocityLMPCConfig`; they are not values to copy blindly from this
example. To extend an existing validated artifact, pass `--existing-artifact`
and still choose a new output path.

## Safety boundary

The terminal convex hull is the same computational relaxation used by the
paper, but this reduced plant contains nonlinear `tan(tilt)` dynamics. A convex
combination of successful states is therefore not, by itself, a proof that its
historical suffix is feasible. The optimizer consequently reports
`safety_certified=false`, reserves the largest queried safe-tail forward
distance, verifies tail command/attitude/rate/slew envelopes, and fails closed
on missing coverage or numerical residuals.

Before online command authority, add and validate a robust terminal funnel or
sampled-vertex recoverability filter, then benchmark a bounded-time solver on
lb11 under radio and logging load. SciPy SLSQP in this module is for offline
replay, not the 8 ms flight loop.

## Verification

```bash
venv/bin/python -m unittest \
  Interaction.tests.test_velocity_lmpc_safe_set \
  Interaction.tests.test_conditional_velocity_lmpc \
  Interaction.tests.test_release_lmpc_terminal_gate \
  Interaction.tests.test_velocity_lmpc_replay
```

The interaction loop records a unique release dataset id, release
direction/state, a bounded pre-release delay queue, every actual command send,
the command effective at each measured state, workspace margin, terminal-gate
audit, and explicit success/rejection closure. Those fields are logging only;
the LMPC remains offline and does not change the active controller.

Reference: Guanrui Li, Alex Tunchez, and Giuseppe Loianno, *Learning Model
Predictive Control for Quadrotors*, ICRA 2022,
https://arxiv.org/abs/2202.07716.
