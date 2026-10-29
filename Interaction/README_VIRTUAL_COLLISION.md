# Two-drone virtual collision

This simulation emulates the **velocity outcome** of a one-dimensional,
frictionless collision while the vehicles remain separated. Their motors
produce the impulse; no physical contact or actual inter-vehicle momentum
transfer is required. The first prototype handles a collinear encounter.
It does not model glancing collisions, spin, downwash or multiple simultaneous
contacts.

## Model and reuse

Let A start to the left of B, with closing speed `u = vA - vB > 0`, masses
`mA, mB`, and coefficient of restitution `0 <= e <= 1`. The collision impulse
and outgoing velocities are

```text
J   = (1 + e) * u / (1/mA + 1/mB)
vA* = vA - J/mA
vB* = vB + J/mB
```

Equal masses at `+0.6/-0.6 m/s` exchange velocities for `e=1`; for `e=0.5`
they leave at `-0.3/+0.3 m/s`. Unequal masses need not both reverse. With
`e=0`, both leave at the center-of-mass velocity.

`virtual_collision_sim.py` reuses `make_septic_brake_profile` from the existing
`jerk_limited_braking.py`. One common profile removes `(1+e)*u` of relative
velocity. Its signed mass-weighted transformation gives both vehicles' position,
velocity, acceleration and jerk. It passes smoothly through zero velocity
without stopping and restarting the curve. Ideal pair momentum is conserved
throughout, and the outgoing kinetic-energy loss is
`0.5 * reduced_mass * (1-e²) * u²`.

This Python generator is existing repository code, but is **not claimed to be
the identical trajectory executed by the deployed firmware**. The current
`FirmwareSCurveCoast.begin()` transports a release event and waits for a
zero-velocity stop endpoint. It has no requested outgoing velocity argument.
This experiment does not modify that live path, its no-reverse requirement,
mission YAML, or firmware parameters on hardware.

## Early trigger and response model

The plan finds the maximum inward relative displacement `C` by solving for
zero relative velocity on the continuous reference. The trigger distance is

```text
Dtrigger = Dsafe + C + (1+e)*u*response_budget + reserve
```

For the offline plant, each acceleration response is a pure delay followed by
a first-order lag. The displacement-error bound is `abs(delta_v)*(delay+tau)`;
the response budget must cover both vehicles' `delay+tau`. This is more
conservative than adding pure command latency alone. The default reserve is
0.08 m and safe center separation is 0.40 m (modeled body radius 0.12 m each).
These are **simulation assumptions**, not a physical safety calibration.

The independent plant integrates interval-averaged analytic acceleration with
an exact zero-order-held lag solution. Full-rate results retain reference and
plant states separately. A bounded-curvature correction below the sampled
minimum guards separation between samples for this scalar plant. Angular
dynamics, estimator noise, rotor effects and battery limits are absent. A
different-delay case deliberately fails transient momentum fidelity even
when endpoint velocities converge; this should not be described as success.

## Run the offline model

From the repository root (Python with NumPy):

```bash
python3 -m Interaction.virtual_collision_sim --output outputs/virtual_collision_sim
python3 -m unittest Interaction.tests.test_virtual_collision_sim Interaction.tests.test_jerk_limited_braking
```

The five cases cover elastic, partially elastic, unequal-mass, unequal-delay,
and zero-restitution encounters. `results.json` holds full-rate samples;
`summary.json` holds check results; `replay.json` is a compact animation input.
The optional `--visualization /absolute/path/virtual-collision.html` fills the
adjacent viewer template for inline playback. The offline summary explicitly
sets `firmware_sitl=false` and `flight_validated=false`.

For custom experiments import `CollisionConfig`, `plan_collision` and
`simulate_collision`. `plan.sample(t)` returns the trajectory about trigger-time
positions `±Dtrigger/2`, including constant-velocity approach for negative times
and outgoing cruise after completion. Planning rejects insufficient initial
spacing. Simulation refuses allocations above 250,000 integration intervals.

## Physical simulation boundary

The paired CrazySim experiment uses two FLS170 Gazebo rigid-body models with
separate frozen SITL firmware instances, PID attitude/altitude control and motor
dynamics. A simulation-only outer tracker follows the Python S-curve using
Gazebo truth. That feedback is privileged simulation information, not an
implemented onboard estimator/controller. Success verifies the particular
modeled encounter; it does not validate current firmware S-curve playback,
radio synchronization, physical clearance or flight.

Keep the raw failed and successful runs when tuning the tracker. Measure
incoming velocity at the trigger, compute targets from it, and assess outgoing
velocity, restitution and pair momentum against that same baseline. Simulation
guard failures must abort the encounter, not lower the acceptance threshold.

Run the paired physics simulation with the local Docker image and existing
CrazySim checkout (the output directory must be new):

```bash
bash Interaction/run_virtual_collision_gazebo.sh "$PWD/outputs/virtual_collision_run"
python3 -m Interaction.render_virtual_collision_results \
  --gazebo-run outputs/virtual_collision_run/gazebo \
  --output outputs/virtual_collision_review \
  --visualization "$PWD/outputs/virtual_collision_review/replay.html"
```

The launcher uses `fls-master-sitl-debug:local`, the sibling `CrazySim` checkout,
the local frozen FLS170 SITL build, and CAD assets. Override paths with
`COLLISION_SIM_IMAGE`, `COLLISION_SIM_CRAZYSIM`, `COLLISION_SIM_FIRMWARE`, or
`COLLISION_SIM_ASSETS`. Sources mount read-only; the simulation opens only
container-loopback UDP connections. No host network or USB device is passed.
The generated animation is an inline fragment; `trajectories.png` and the JSON
reports are ordinary standalone outputs.

### Adjustable Gazebo parameters

`--minimum-surface-gap` sets the **minimum modeled surface margin**, in metres,
for `--geometry-close-pass`. If omitted, that mode retains its original
0.03 m margin. The allowed input range is `0 < gap <= 0.50 m`; explicit use
without the CAD mode is rejected. For the saved FLS170 geometry, the center
distance floor is `2 * 0.1146263271 + gap`. Planning, live aborts and saved-data
clearance checks use the same requested margin. The normal mode still uses
its original 0.40 m center floor.

The approximately **12 cm** in the selected video's results is the measured
CAD support-plane surface-distance lower bound, not an input setting. It came
from a **3 cm** configured sphere margin, plus a **4 cm** trigger reserve,
discrete triggering, tracking response and the conservatism of enclosing the
entire body in a sphere. Setting `--minimum-surface-gap 0.12` therefore requests
a 12 cm minimum margin; it does **not** target an observed closest gap of exactly
12 cm. Increasing the input moves the trigger farther apart; reducing it permits
a closer encounter. The actual result must still be measured in each run.

| Option | Launcher default | Selected fast experiment | Meaning |
| --- | ---: | ---: | --- |
| `--minimum-surface-gap` | 0.03 m in CAD mode | 0.03 m | Minimum conservative surface margin, not an exact closest-gap target; `(0, 0.50]` m |
| `--reference-acceleration` | 2 m/s² | 6 m/s² | Nominal S-curve acceleration limit; `(0, 8]`, no greater than the command cap |
| `--reference-jerk` | 8 m/s³ | 50 m/s³ | Nominal S-curve acceleration-change limit; `(0, 80]` |
| `--command-acceleration-cap` | 3 m/s² | 8 m/s² | Collision-phase acceleration-equivalent command cap; `(0, 8]` |
| `--trigger-reserve` | 0.06 m | 0.04 m | Extra distance above predicted inward closure for empirical triggering; `[0.04, 0.30]` m |
| `--jerk-feedforward-lead` | 0.10 s | 0.20 s | Anticipates the reference acceleration change; `[0.05, 0.35]` s |
| `--predictive-unwind` | Off | On | Predicts remaining acceleration impulse to reduce rebound overshoot |
| `--unwind-impulse-time` | 0.262 s | 0.177 s | Remaining-impulse prediction coefficient; `[0.10, 0.35]` s |
| `--unwind-velocity-gain` | 5 | 7 | Terminal velocity correction gain; `[1, 15]` |
| `--unwind-rate-damping` | 0.20 s | 0.083 s | Terminal damping of the measured tilt response; `[0.03, 0.25]` s |
| `--unwind-tilt-jerk-weight` | 0.0137 s² | 0.0037 s² | Tilt-acceleration derivative contribution to predicted velocity; `[0, 0.02]` s² |
| `--attitude-bias-compensation` | Off | On | Simulation-truth-assisted correction of estimated pitch bias |
| `--trigger-reference` | None | Saved `run03_a6_predict_fast/gazebo` | Completed accepted physics run used to estimate inward closure; required in CAD mode |
| `--geometry-close-pass` | Off | On | Derives the separation floor from saved CAD/rotor bounds |

These ranges are input-validation limits for experiments, **not a guarantee
that every combination passes**. Predictive-unwind coefficients take effect
only when that switch is enabled. The reference jerk does not bound the full
switched controller's actual jerk. The fixed fast-comparison benchmark still
requires closest center distance below 0.32 m; a larger requested margin can
intentionally make that benchmark unattainable while the configured collision
checks pass. Its threshold is not changed to accommodate a new margin.

For example, retain the selected fast controller and choose a 2 cm minimum
modeled surface margin (use a new output directory):

```bash
bash Interaction/run_virtual_collision_gazebo.sh "$PWD/outputs/collision_gap_02" \
  --geometry-close-pass --minimum-surface-gap 0.02 \
  --reference-acceleration 6 --reference-jerk 50 --command-acceleration-cap 8 \
  --trigger-reference outputs/virtual_collision_fast_20261026/run03_a6_predict_fast/gazebo \
  --trigger-reserve 0.04 --jerk-feedforward-lead 0.20 \
  --attitude-bias-compensation --predictive-unwind \
  --unwind-impulse-time 0.177 --unwind-velocity-gain 7 \
  --unwind-rate-damping 0.083 --unwind-tilt-jerk-weight 0.0037
```

Approach speed (±0.6 m/s command), vehicle mass (0.170 kg each), restitution
(`e=1`), flight height (0.8 m), starting center spacing (3.2 m) and approach
ramp (1.2 s) are currently fixed in this Gazebo scenario. Mass, restitution,
initial velocity and delay/lag can already be varied through the **offline**
`CollisionConfig` Python interface; they are not exposed as Gazebo CLI options.
Varying a physical-model mass also needs consistent inertia/motor/model changes.

Parameter verification is saved in
`outputs/virtual_collision_parameters_20261026/`. The new option/configuration
checks plus existing collision/S-curve checks total **37 passing tests**.
One physical run (`gap01/gazebo`) used a **0.01 m** margin with the selected
controller and the same empirical trigger source. It independently passed all
eleven configured collision checks and the four fixed fast-comparison checks:
minimum center separation 0.29435 m, sampled CAD surface-distance lower bound
0.10691 m, 90%-to-90% transitions 0.2103/0.2131 s and peak outgoing-speed
excess 7.68%/5.79%. This is one new run, not repeat evidence for every margin.
The final 0.2 s restitution range was 1.00742–1.01490. The original selected
0.03 m runs and their approximately 0.123–0.128 m CAD gap results are retained.

The tracker uses velocity feedback and measured tilt-rate damping during the
collision. It omits position-error correction in that phase because chasing a
position delayed by braking can add unwanted rebound impulse. The final tuned
baseline reference is less aggressive than the offline default; its limits and actual
incoming velocities are recorded in each run's `plan_config`.

The initial endpoint-only assessment was insufficient: an oscillatory run can
cross its target velocity at the reporting instant. Final acceptance therefore
also checks **all common-timestamp raw Gazebo observations in the final 0.2 s**
before capture: per-drone speed error <= 0.10 m/s and restitution error <= 0.10.
The observation window ends one second after the reference finishes. Reports
retain residual acceleration, velocity zero-crossing times, pair timestamp skew,
and command intervals. These are prototype simulation thresholds, not inherited
flight acceptance gates. Frozen firmware/source hashes and source snapshots
are saved with the raw records for reproducibility.

## Recorded result, 2026-10-26

`outputs/virtual_collision_gazebo_20261026/verified_06/gazebo` contains the
accepted paired run and an independent raw-data review. Both models weigh
0.170 kg. Incoming X velocities were +0.63525 and -0.62557 m/s. The target
outgoing velocities were -0.62557 and +0.63525 m/s. With 2 m/s² reference
acceleration and 8 m/s³ jerk limits, the reference lasted 1.09916 s; actual
velocity crossed zero at 0.615 and 0.605 s after the trigger.

The synchronized minimum center separation was 0.759998 m. The last 0.2 s
before capture had maximum speed errors 0.03638/0.03193 m/s, restitution
1.04757–1.05416, and residual acceleration magnitudes below 0.03531/0.02426 m/s².
Peak pair momentum error was 0.002550 kg·m/s. This passes the stated prototype
thresholds and demonstrates stable noncontact rebound over that window. It is
not exact elasticity: outgoing kinetic energy remains about 10–11% high.

Earlier records are retained. In particular `collision_05` and `verified_01`
passed an obsolete single-instant endpoint check but did not establish stable
outgoing velocity; use the strengthened common-time window assessment instead.
The offline reference/plant and existing generator checks total 33 passing tests.

## Hard-contact appearance and timing

Equal masses exchange their incident velocities only for a collinear,
frictionless, perfectly elastic collision. The same endpoint velocities do not
imply the same interaction: a real hard contact produces opposing contact
forces during compression and recovery; these separated vehicles obtain their
horizontal impulses from thrust. The pair's momentum can be tracked closely,
but this does not make the pair an isolated mechanical system or transfer
momentum directly from one vehicle to the other. The motors also need not
recover the kinetic energy removed during braking.

In the equal-mass, equal-and-opposite-speed case, total momentum is zero for
any exactly antisymmetric motion. A pair can therefore conserve its summed
momentum while both vehicles brake too early, rebound too slowly, or overshoot
the correct outgoing speed. Assess separation, transition duration, restitution,
transient speed overshoot and stable endpoint velocity independently. The final
0.2 s acceptance window does not by itself bound earlier overshoot.

The collision duration is not determined by equal mass. It depends on material
stiffness, shape and incident speed. Cross's billiard-impact analysis describes
stiff-ball contact times of about 1 ms or less; that is a scale comparison, not
a measured duration for these vehicles or a universal value for all objects.
See [Cross, Physics Education 55, 065002 (2020)](https://doi.org/10.1088/1361-6552/aba207)
and [OpenStax, elastic collisions in one dimension](https://openstax.org/books/college-physics-ap-courses/pages/8-4-elastic-collisions-in-one-dimension).

For a symmetric encounter at speeds `+v/-v`, each vehicle must change velocity
by `2v`. If its **actual** horizontal acceleration is bounded by `A`, then
`T >= 2v/A`, before imposing jerk or attitude-response limits. The baseline
measured velocity change of 1.26082 m/s would require at least 0.42027 s at
3 m/s²; achieving it in 20 ms would require an average 63.04 m/s². The harness
caps the acceleration-equivalent **command** at 3 m/s²; it does not establish
that exact bound on the realized rigid-body acceleration, which can overshoot.
Actual acceleration is therefore reported independently.

Timing measurements distinguish the command trigger, sustained braking onset,
first zero crossing, +90% incoming to -90% outgoing transition, and stable
outgoing motion. The baseline's roughly 0.23 s onset includes the deliberately
smooth ramp and must not be labeled pure radio or controller latency. At 30 fps,
a theoretical hard impact is effectively a one-frame velocity change, whereas
the recorded drone reversal spans many frames. Replay time is kept at 1× in the
realism comparison; interpolation and rendering do not improve the dynamics.

The original 0.40 m center-distance gate is a prototype assumption, not a
calibrated hardware limit. Its body collision box is only 0.10 × 0.10 × 0.03 m;
the full CAD guard is visual geometry, so absence of Gazebo contact alone does
not establish physical-model clearance. The actual CAD mesh fits inside an
origin-centered sphere of radius 0.1146263271 m. Added rotating propeller visuals
are within that sphere. Geometry-specific close-pass experiments must retain a
positive margin between these spheres and identify their different distance
criterion explicitly. The artificial red orientation annotation is not part of
the physical vehicle envelope. These bounds apply to this model, not real
propeller flex, localization error, downwash, or physical flight clearance.

## Realism experiments, 2026-10-26

The original baseline remains the default. The explicit experimental options
select a faster 3 m/s² / 20 m/s³ reference, a prior-run closure estimate with a
reserved distance, a CAD-derived distance gate, and reference-jerk feedforward
separate from measured tilt-rate damping. The geometry mode derives the full
body/rotor radius from the generated assets and rejects unreviewed geometry;
it removes only the nonphysical orientation-marker annotation. Its 0.259253 m
center floor represents two bounding radii plus a 0.03 m margin. This is a
different, explicitly declared simulation scenario, not a relaxation applied
to turn a failed baseline run into a success.

Four physics encounters were retained under
`outputs/virtual_collision_realism_20261026/`:

| Run | Minimum center gap | Main velocity reversal, 90% to -90% | Peak rebound-speed overshoot | Result |
| --- | ---: | ---: | ---: | --- |
| `run01_fast20_retry` | 0.47258 m | 0.349–0.350 s | 31.9–32.8% | Original 0.40 m distance gate and all 9 checks pass |
| `run02_cad_close_retry` | 0.32447 m | 0.339–0.341 s | 35.2–36.2% | CAD scenario, all 11 checks pass |
| `run03_cad_lead20` | 0.34798 m | 0.343–0.349 s | 14.7–16.7% | Selected compromise; all 11 checks pass |
| `run04_cad_lead25` | 0.32894 m | 0.343–0.349 s | 14.4–15.1% | Rejected: endpoint and window restitution fail |

Run03 uses a 0.20 s reference-jerk feedforward lead while retaining 0.10 s
measured tilt-rate damping. Its measured incident velocities are +0.59873 and
-0.60103 m/s, compared with +0.63525/-0.62557 m/s in the old baseline. The
relative incident speed is therefore about 4.8% lower; the measured timing
improvement is not an exact equal-input controller comparison. The selected
run begins sustained braking at about 0.14 s, crosses zero at 0.402–0.405 s,
and satisfies the finite-horizon joint speed/restitution/acceleration settling
criterion from 1.47 s after the trigger. The 90%-to-90% duration is not the full
settling time.

The sampled support-plane gap between full CAD bodies is at least 0.17395 m;
the more conservative orientation-independent sphere gap is 0.11873 m. The
configured 0.03 m margin is not the observed surface gap. A separate
interval-local speed allowance is diagnostic only: observed samples do not
prove a continuous speed bound between samples.

This selection improves the main transient but does **not** pass the additional
hard-impact peak-fidelity diagnostic. Peak restitution is 1.15685; the final
0.2 s is 1.07835–1.09870 and kinetic energy is still 16–21% above the incident
energy. Thus all 11 prototype checks passing does not establish exact elastic
collision behavior. The more aggressive 0.25 s lead was retained as a failed
trial, with no acceptance threshold changes.

Reproduce the selected experiment with the saved image/source identifiers and
arguments in `outputs/virtual_collision_realism_20261026/reproduce_selected.sh`.
Independently review a completed result without starting a simulator:

```bash
python3 -m Interaction.analyze_virtual_collision_realism \
  outputs/virtual_collision_realism_20261026/run03_cad_lead20/gazebo \
  --output outputs/virtual_collision_realism_20261026/run03_cad_lead20/gazebo/independent_realism_review.json
```

The selected video in `render/final_selected/` is native Gazebo/Ogre2 rendering
of recorded body positions and orientations at 1× speed, ending before the
intentional hover-capture maneuver. CAD geometry and the simulation clock are
not scaled. The separate ideal-sphere pane is an explicitly analytical,
instantaneous-contact reference, not simulated drone dynamics. Original
source hashes, camera settings, video probes, raw observations and unsuccessful
trials remain available. No hardware flight or deployment was performed.

## Raising the simulation acceleration and jerk

The earlier 3 m/s² command cap was an experiment setting, not the modeled
vehicle's maximum capability. Further experiments keep reference acceleration,
reference jerk and actuator-command acceleration caps separate. Increasing a
reference limit alone is ineffective once the tracker clips the resulting
command, and clipping changes the reference's impulse/time relationship.

The static thrust budget must follow the loaded plugin, not only the SDF
`maxRotVelocity`. The frozen strict-simulation PWM-to-rotor-speed conversion
limits full-PWM collective thrust to approximately 2.877 N; the frozen altitude
PID base/output range gives approximately 2.668 N before allowing for attitude
torque. Holding a 0.170 kg model level in altitude while accelerating horizontally
at 4, 6 or 8 m/s² requires approximately 1.801, 1.955 or 2.152 N, with ideal
tilts of 22.2°, 31.5° and 39.2°. These static calculations motivate staged SIM
experiments; they do not establish transient mixer authority or hardware limits.

The new trial records under `outputs/virtual_collision_fast_20261026/` retain
the declared command cap, observed acceleration, actual tilt, altitude range,
firmware attitude estimates and controller diagnostics. Independent comparison
also checks peak outgoing-speed ratio and peak restitution, instead of relying
only on a settled endpoint. A 3 cm minimum modeled surface margin and the
existing terminal velocity/restitution/momentum thresholds are retained.

Firmware `controller.cmd_pitch` is the actuator pitch correction, whereas
`controller.pitch` is the requested attitude. Firmware pitch and standard
Gazebo Euler pitch use opposite signs in this setup. Diagnostic comparisons
must respect those conventions and distinguish receipt-time alignment from an
exact shared sensor timestamp. A mismatch between estimated and true attitude
can leave a residual outgoing-velocity error even when the inner attitude loop
closely tracks its estimated state.

### Selected faster controller and repeat evidence

Five exploratory physical simulations and two unchanged repeats were performed.
Increasing A/J alone made the 90%-to-90% transition faster but increased rebound
overshoot. A very conservative predictive terminal controller reduced overshoot
but made the transition slower. A high-gain version caused alternating saturated
corrections. All those results, including failures, remain in the trial directory.

The selected experiment is `run05_a6_predict_robust`, followed by
`run06_repeat1` and `run07_repeat2` with the exact same options, source hashes
and prior-run trigger input. Its settings are:

- Nominal reused S-curve acceleration/jerk limits: 6 m/s² and 50 m/s³.
- Final horizontal-acceleration-equivalent command cap: 8 m/s², applied after
  attitude-bias correction. Command tilt remains below 41°; observed tilt above
  45° aborts the simulation.
- Reference jerk lead: 0.20 s. A bounded estimator-minus-true-pitch correction
  uses a 0.15 s filter, ±3° bound and no more than 50 ms receipt-time age.
- Remaining-velocity predictor: `v_pred = v + 0.177*q + 0.0037*qdot`, where
  `q` is the acceleration equivalent of the measured true tilt and `qdot` is
  filtered. After the predicted relative terminal velocity reaches the exchanged
  target, a 40 ms seventh-order blend switches to
  `u = 7*(v_target - v_pred) - 0.083*qdot`.

The predictor coefficients were selected using an approximate delayed scalar
plant fitted to observed tilt response, then tested in paired Gazebo physics.
They are tuning coefficients, not a certified inverse dynamics or stopping
guarantee. `surrogate_search.json` is offline candidate-selection evidence only;
the raw paired simulation records provide the acceptance evidence.

The switch/blend changes the host command before the original S-curve finishes.
Consequently **50 m/s³ is a nominal reference limit, not a verified bound on the
complete command or actual jerk**. Likewise the cap on equivalent acceleration
does not enforce that same cap on actual rigid-body acceleration.

| Measurement | Previous selected A3/J20 case | New selected case and two repeats |
| --- | ---: | ---: |
| 90% incoming to first 90% outgoing speed | 0.343–0.349 s | 0.212–0.229 s |
| Trigger to velocity zero | 0.402–0.405 s | 0.2867–0.2874 s |
| Minimum center separation | 0.34798 m | 0.31012–0.31543 m |
| CAD surface-distance lower bound at sampled attitudes | 0.17395 m | 0.12282–0.12824 m |
| Peak outgoing-speed excess over exchanged target | 14.7–16.7% | 5.1–9.0% |
| Restitution throughout the final 0.2 s | 1.078–1.099 | 1.006–1.018 |

The chosen new video run measured incoming velocities +0.60608/-0.60977 m/s;
the previous selected run measured +0.59873/-0.60103 m/s. Thus the faster result
was not obtained by lowering the approach-speed command or slowing playback.
Actual acceleration peaked around 6.8–7.0 m/s² with approximately 34° tilt.

All three runs independently pass the existing eleven checks and four added
criteria: peak outgoing speed <=110% of target, peak restitution <=1.10,
90%-to-90% transition <0.30 s, and minimum center gap <0.32 m. The original
0.03 m modeled surface margin, terminal error and momentum thresholds were not
relaxed. The short transition time is **not** full settling time or contact
duration. A supplementary requirement that residual acceleration stay below
0.1 m/s² throughout the last 0.2 s was not met in every repeat: the last repeat
reached 0.10118 m/s², and its joint velocity/restitution/acceleration-stable
interval lasted 195 ms; the other two lasted 380 and 230 ms. This supplementary
result is retained rather than rounded into a pass.

The host uses **simulation truth** for attitude prediction and bias correction.
Motion is still realized by the frozen SITL estimator/PID, mixer, motors and
Gazebo rigid body; no per-step force, velocity or pose injection creates the
rebound. This is ideal-state-access simulation, not a deployed onboard S-curve
or demonstrated hardware collision illusion. Firmware log timestamps are not
treated as the Gazebo clock. The frozen firmware/cflib logging-period mismatch
is explicitly recorded and compensated when requesting diagnostic samples.

`independent_repeat_audit.json` contains the repeat/source checks;
`comparison.json` and `comparison.csv` retain the complete trial ledger;
`reproduce_selected.sh` launches a new run with the pinned image and exact
selected arguments. `render/final_selected/` shows the first successful run05,
chosen before the repeats, at normal speed, plus an equal-scale comparison
against the previous selected run. Original baselines and failed trials remain
available; production defaults and firmware are unchanged.
