# Conditional Learning MPC for release-to-rest braking

This is the paper-structured Learning MPC path for the task:

> after every confirmed world-`+Y`/`-Y` release, reduce the release-axis speed
> from its measured initial value to zero as quickly as possible, then enter a
> full measured terminal set with XY speed, roll, pitch, and roll/pitch rates
> equal to zero within explicit estimator tolerances.

The implementation is offline-only. It does not own the Crazyflie command path
and does not authorize a flight.

## What is implemented

`velocity_lmpc_safe_set.py` provides the learned part of LMPC:

- only complete successful release-to-rest trajectories enter the sampled safe
  set;
- every stored state has a reverse cost-to-go made from the same normalized
  minimum-time progress, effort, and slew stage cost used by the optimizer;
- +Y and -Y releases and each model/constraint fingerprint are isolated;
- an unseen initial speed may only interpolate between demonstrated lower and
  upper speeds; speed extrapolation fails closed;
- exact configured command delay, prediction step, coincident decision-time
  state/action phase contract, stage-cost definition, state layout, and hard
  limits are immutable artifact identity;
- terminal dwell, full measured XY speed, roll and pitch, roll/pitch rates,
  reversal, cross-speed, workspace margin, telemetry age/skew, the actual
  aligned and orthogonal commands (including both pending queues), and
  safe-tail bounds are checked before an episode is admitted;
- the terminal position-controller setpoint must be within 1 cm of the final
  measured XYZ position, so handoff cannot accelerate toward a still-ahead
  stopping target;
- a published JSON artifact is written atomically and is immutable during an
  episode.

`conditional_velocity_lmpc.py` provides the offline optimizer:

\[
\min_{u_{0:N-1},\lambda}
\sum_{k=0}^{N-1} h_s(x_k,u_k,\Delta t)
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

Following the real-time approximation in the paper's Eq. (24), this task uses
a normalized reduced-state error `z=[v, tilt, tilt_rate, pending commands]`:

\[
h_s=\Delta t\left(
\frac{\lVert z\rVert^2}{\sqrt{\lVert z\rVert^4+1}}
+w_u\bar u^2+w_{\Delta u}\overline{\dot u}^{,2}
\right).
\]

`J_i` is computed by summing this exact helper over the demonstrated suffix,
so the predicted prefix and learned tail use one seconds-valued objective.
This is an approximate minimum-time proxy, not a proof of a globally fastest
trajectory.

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
command delay is retained exactly instead of being inferred from queue length.
Each constant-input segment is numerically propagated and audited at no more
than 1 ms spacing, including the fractional-delay switch. The model, exact
delay, prediction step, stage-cost specification, and constraints are
fingerprinted so a trajectory collected under one contract cannot be reused
under another.

The optimizer models the release-axis velocity and projected braking attitude
only. It does **not** optimize cross-axis velocity or the orthogonal attitude
dynamics. Full measured roll and pitch, both measured rates, cross-axis speed,
the effective level input, and every aligned/orthogonal command still pending
in the identified delay window are mandatory episode-admission and handoff
gates. They are not claimed as model-predicted guarantees. A future full-planar
controller needs a 2-D plant and 2-D command state before it can claim
minimum-time full-XY stopping.

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
4. Preserve that raw log unchanged, then estimate state at the actual command
   decision epochs on a fixed command-time grid. This resampling stage is not
   yet implemented in this repository; raw flight-loop rows are evidence, not
   directly admissible LMPC samples.
5. Choose the LMPC prediction step from the actual command cadence. In the
   validated record, every non-terminal state timestamp must equal its action
   send timestamp, every delayed-command queue must match both the complete raw
   send history and the exact fixed-grid queue successor, and each transition
   must match the prediction step within 1 ms. Record the exact fitted positive
   command delay--do not infer a fractional delay from queue dimension. The
   strict replay currently rejects a zero-delay contract because state-before-
   send ordering at an identical timestamp is ambiguous.
6. Admit an episode offline only if it has no safety/localization event, no
   reverse-speed violation, stays inside the workspace/attitude/rate/command
   limits, remains in the full measured terminal set for the configured dwell,
   and hands position control a target within 1 cm of the final measurement.
7. Collect successful lower and upper speed brackets before asking LMPC to
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
Even such a run produces raw evidence first: because the controller sends each
command after observing state, its rows must pass a separately implemented and
validated command-grid state resampler before this strict replay can admit
them. There is currently no flight-derived LMPC artifact.

## Offline extraction

`velocity_lmpc_replay.py` is the final strict validator from a decision-time-
resampled record into the safe set; it is not itself the missing resampler. It
independently reconstructs the configured delayed input from the actual send
history on every row, compares the entire raw and fixed-grid pending queues,
and rejects any post-observation state/action phase. It ignores the legacy
`Translation Position Hold Resumed` event. A successful segment must end with
`Release Dataset Terminal Dwell Complete`, emitted only after the final
observer row records the measured dwell and the actual position command. A
recontact, early handoff, unsafe post-terminal command, or detector rearm emits
`Release Dataset Episode Closed` with a rejected outcome; one rejected episode
does not contaminate other complete episodes in the same log.

The extractor never overwrites an artifact:

```bash
venv/bin/python -m Interaction.velocity_lmpc_replay \
  --input /absolute/path/to/complete-flight-log.json \
  --output /absolute/path/to/new-safe-set.json \
  --model-fingerprint reduced-v3:<sha256> \
  --state-dimension 9 \
  --prediction-step-s 0.02 \
  --command-delay-s 0.12
```

The v3 schema binds
`state_action_phase_contract=coincident_decision_time_v1` and invalidates the
earlier phase-blind draft. The fingerprint, prediction step, exact configured
delay, and dimension must come from the same frozen model and
`ConditionalVelocityLMPCConfig`; they are not values to copy blindly from this
example. New artifacts require `--command-delay-s`.
To extend an existing validated artifact, pass `--existing-artifact` and still
choose a new output path; its step, delay, cost specification, and limits are
reused exactly.

## Safety boundary

The terminal convex hull is the same computational relaxation used by the
paper, but this reduced plant contains nonlinear `tan(tilt)` dynamics. A convex
combination of successful states is therefore not, by itself, a proof that its
historical suffix is feasible. The optimizer consequently reports
`safety_certified=false`, reserves the largest queried safe-tail forward
distance through the actual position-handoff target, verifies tail forward and
reverse velocity, command/queue and attitude/rate envelopes, verifies aligned-
command slew, and fails closed on missing coverage or numerical residuals.
Orthogonal command slew is not modeled by this one-dimensional optimizer.

The 1 ms internal checks and the accepted `prediction_step +/- 1 ms` resampled
data grid constrain this discretized numerical contract. All state/action
phases themselves must be coincident; the whole raw delayed queue is checked at
every row. Timing jitter within the accepted transition grid is still an
approximation. These checks are not a continuous-time guarantee for the real
vehicle, and the measured-grid tolerance is not the fixed-grid recursive-
feasibility proof from the paper.

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
  Interaction.tests.test_velocity_lmpc_replay \
  Interaction.tests.test_wrench_interactions_integration
```

The interaction loop records raw evidence: a unique release dataset id, release
direction/state, a bounded pre-release delay queue, every actual command send,
the modeled command effective at each measured state (not a hardware ACK),
workspace margin, terminal-gate audit, and explicit success/rejection closure.
Those fields are logging only; raw rows still require state resampling, and the
LMPC remains offline and does not change the active controller.

Reference: Guanrui Li, Alex Tunchez, and Giuseppe Loianno, *Learning Model
Predictive Control for Quadrotors*, ICRA 2022,
https://arxiv.org/abs/2202.07716.
