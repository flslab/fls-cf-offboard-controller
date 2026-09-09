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

1. Start the dedicated flight from the LightBender orchestrator with
   `python3 orchestrator.py --mpc` (`--skip-record` is optional). This is a real
   flight command. The controller validates the mission, world-Y sensor
   geometry, clear-volume margin, current quality-gated baseline braking fit,
   and independently held-out-validated +Y and -Y prediction models before
   arming.
2. `--mpc` creates an in-memory mission overlay. It turns off velocity-hover,
   predictive/online LMPC, and all LMPC command authority; the bounded legacy
   attitude-coast controller remains the only roll/pitch owner. It does not
   rewrite the mission YAML or calibration file.
3. Collect near 0.25, 0.45, and 0.65 m/s, preferably low to high. At each
   speed, make real releases in both world +Y and -Y; two terminal-success
   attempts per direction are required by the current mission. Cells are
   independent, so a higher-speed attempt is not discarded merely because a
   lower cell is incomplete, and useful cells may be accumulated offline over
   multiple battery-bounded flights. The cell is classified from measured
   release velocity, not from an intended speed. More than 0.03 m/s world-X
   release speed, reversal,
   stale/skewed state, path tilt/rate, and boundary failures do not count.
4. Configure the Crazyflie source stream at 100 Hz, then record one latest
   fresh source state and one actual command on each 50 Hz decision epoch, the
   current-position terminal handoff, and one command-free fresh state after
   that send. Intermediate asynchronous callbacks may be superseded before the
   next decision; duplicate callback states never generate extra commands.
5. Preserve that raw log unchanged, then run
   `Interaction.velocity_lmpc_resample`. It interpolates state only between
   fresh measurements that bracket each actual send time, reconstructs the
   physical state at that send and the delayed command queue, recomputes
   terminal dwell, and refuses extrapolation, hazards, timing gaps, model
   contract mismatches, or output-file replacement. This is feasible trajectory
   evidence for LMPC, not a behavior-cloning label claiming that the baseline
   computed its command from the interpolated send-time state. Raw flight-loop
   rows are not directly admissible LMPC samples.
6. Choose the LMPC prediction step from the actual command cadence. Each
   release locks the exact directional frozen model before its START record;
   that record carries its model fingerprint, state dimension, and positive
   direction-specific command delay. The +Y and -Y delays may differ from one
   another and from the planar baseline delay used by the flight controller.
   In the
   validated record, every non-terminal state timestamp must equal its action
   send timestamp, every delayed-command queue must match both the complete raw
   send history and the exact fixed-grid queue successor, and each transition
   must match the prediction step within 1 ms. Record the exact fitted positive
   command delay--do not infer a fractional delay from queue dimension. The
   strict replay currently rejects a zero-delay contract because state-before-
   send ordering at an identical timestamp is ambiguous.
7. Admit an episode offline only if its matching flight-side bootstrap close
   event says it was countable, terminal-successful, and free of every sticky
   path/timing failure, and if it has no safety/localization event, no
   reverse-speed violation, stays inside the workspace/attitude/rate/command
   limits, remains in the full measured terminal set for the configured dwell,
   and hands position control a target within 1 cm of the final measurement.
8. Collect successful lower and upper speed brackets before asking LMPC to
   interpolate an intermediate release. If a new speed or context is outside
   coverage, run the conservative baseline and add it only after post-flight
   validation.

Two saved calibration products have deliberately separate jobs. The current
quality-gated planar braking fit drives the bounded legacy attitude-coast
controller during collection. Independently held-out-validated directional
prediction models define the exact frozen dynamics, delay, state layout, and
fingerprint that the offline LMPC artifact must use. `--mpc` requires both but
does not refit or overwrite either. LMPC learns safe trajectories and
cost-to-go; it does not remove the need for those dynamics.

Normal `lb11` interaction still has `coast_velocity_braking_enabled: true` and
sends `velocity_hover`; those ordinary runs remain deliberately incompatible
with this attitude-input LMPC pipeline. Only the private `--mpc` overlay selects
the attitude baseline. Even a completed in-flight speed-cell report is only
provisional raw collection coverage. There is no flight-derived LMPC artifact
until resampling and strict replay both accept the episodes.

## Offline extraction

`velocity_lmpc_resample.py` converts the raw complete-flight array into a new
decision-time-aligned array. The current `--mpc` contract configures the
Crazyflie source at 100 Hz and paces the single command owner on a separate
0.02 s grid. The raw observer rows retain the latest source sample used around
each decision rather than claiming that every intermediate callback was logged.
The positive command delay must be copied from the selected direction's
`release_dataset_command_delay_s` START field (also reported by the
resampler), not from the legacy planar baseline fit:

```bash
venv/bin/python -m Interaction.velocity_lmpc_resample \
  --input /absolute/path/to/complete-flight-log.json \
  --output /absolute/path/to/resampled-positive-y.json \
  --prediction-step-s 0.02 \
  --command-delay-s 0.12 \
  --direction-sign positive-y
```

`0.12` is only an example. Use the same run and direction's logged delay; never
copy a delay from another drone, direction, or model. Run the command again
with a different output path and `--direction-sign negative-y` for -Y. The
direction argument is
mandatory: the resampler filters out the opposite-direction episodes, reports
their IDs as skipped, and never places +Y and -Y episodes in the same output.
Keep the subsequent replay, model fingerprint, and safe-set artifact separate
for the two directions as well. Successful resampling prints the locked
`model_fingerprint`, `state_dimension`, prediction step, and command delay;
use those exact four values for the replay below.

`velocity_lmpc_replay.py` is the final strict validator from that resampled
record into the safe set. It
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
  --input /absolute/path/to/resampled-positive-y.json \
  --output /absolute/path/to/safe-set-positive-y.json \
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
  Interaction.tests.test_mpc_bootstrap_calibration \
  Interaction.tests.test_velocity_lmpc_resample \
  Interaction.tests.test_velocity_lmpc_replay \
  Interaction.tests.test_wrench_interactions_integration
```

The interaction loop records raw evidence: a unique release dataset id, release
direction/state, a bounded pre-release delay queue, every actual command send,
the modeled command effective at each measured state (not a hardware ACK),
workspace margin, terminal-gate audit, and explicit success/rejection closure.
Those fields are logging only; raw rows still require state resampling and
strict replay, and the LMPC remains offline and does not change the active
controller.

Reference: Guanrui Li, Alex Tunchez, and Giuseppe Loianno, *Learning Model
Predictive Control for Quadrotors*, ICRA 2022,
https://arxiv.org/abs/2202.07716.
