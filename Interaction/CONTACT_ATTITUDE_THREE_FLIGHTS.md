# Contact-attitude three-flight comparison

This is a diagnostic-only, shadow-estimator protocol. It does not give the
custom estimator command authority, deploy code, flash firmware, arm a vehicle,
or authorize a flight. Run the same physical interaction sequence three times
only after the normal site, hardware, battery, boundary, and pilot checks pass.

| Flight | Vicon route to onboard EKF | Shadow estimate | Permitted interpretation |
| --- | --- | --- | --- |
| 1 | Pointcloud `extpos` | Synchronized onboard-EKF telemetry mirror | Onboard-to-mirror plumbing parity only |
| 2 | Rigid-body `extpos` (XYZ only) | Gyro during contact; gyro + measured accelerometer process input and Vicon XYZ updates after release | Vicon orientation is withheld from both estimator updates and used as an evaluation channel |
| 3 | Rigid-body `extpose` (XYZ + quaternion) | Same custom estimator as flight 2; Vicon quaternion is withheld from its update | Vicon orientation is withheld from the shadow update, but is an onboard-EKF input |

The orientation evaluation channel in flights 2 and 3 is not sensor-independent.
It is withheld from the shadow estimator, but the XYZ position channel comes
from the same Vicon rigid-body measurement and is used by the estimators. That
shared-sensor correlation must remain explicit in every report. In flight 3,
the quaternion is also an onboard-EKF input, so it cannot separately validate
the onboard attitude.

## Pre-arm lifecycle contract

The experiment selector rejects the mission before arming unless all of these
conditions are present in the loaded mission itself:

- `Interaction.config.detection_method: momentum_impulse`;
- `wrench_interaction.state_source: onboard`;
- an enabled `wrench_interaction.initial_contact_arming` gate applied after
  every interaction, with XY speed at most `0.03 m/s`, at least `0.50 s`
  stationary dwell, and at most `0.10 s` between credited state samples;
- `virtual_object.contact_detection.source: potentiometer`;
- `virtual_object.release_behavior.mode: potentiometer_coast`;
- finite, strictly positive contact force/onset-dwell, release drop/rate,
  unloaded force/dwell, sample-gap, stall-timeout, and sensor-stale-timeout
  values;
- `unloaded_force_n < force_threshold_n`; and
- when configured, `0 < candidate_lead_drop_n <= force_drop_n`.

`--sense` is necessary but is not treated as proof that the potentiometer owns
the lifecycle. The mission fields above are checked separately.

The selector also refuses:

- flight 1 with rigid-body or full-pose forwarding;
- flight 2 with full-pose forwarding;
- flight 3 without full-pose forwarding;
- a log-only Vicon route; or
- any run without `--interaction`, `--sense`, `--log`, airborne Vicon, and the
  required rigid-body name for flights 2 and 3.

Flights 2 and 3 additionally require firmware built with
`configs/contact_release_shadow.conf`. That opt-in exposes producer-latched
`contactImu` logging. One maximum-size 26-byte packet contains FP16 gyro,
accelerometer, onboard position, and onboard velocity values plus a common
low-16 stabilizer epoch. This replaces three competing 1 kHz radio streams
with one atomic stream; the existing lower-rate `VEL_ORI` stream supplies the
stationary-yaw alignment check. Pre-arm rejects an old firmware image, a
missing packed field, or a packet whose epoch cannot be safely reconstructed
on the Crazyflie clock. Building the image does not enable the firmware-side
shadow EKF at runtime; `kalmanPRel.enable` remains off by default.

The rigid-body axes must already be calibrated to the Crazyflie body axes. Do
not re-zero roll or pitch at release. The shadow alignment yaw comes from the
configured nominal yaw, so flight 3's full-pose correction cannot leak into its
initialization.

At release, flights 2 and 3 latch position from the latest causal Vicon `tvec`
that was actually forwarded to the onboard EKF, velocity from one atomic
onboard-EKF state packet, and attitude plus gyro bias from the selected gyro
history on the Crazyflie clock. The onboard position at the velocity epoch is
kept only as a diagnostic. The host-loop state, past commands, current
setpoint, and pending command history are also diagnostic-only and never
reconstruct release velocity. The EKF nominal state is created at the frozen
release gyro epoch, so the release `tvec` is not accidentally propagated from
the older velocity-sample epoch. After release, measured body-frame specific
force drives velocity through the estimated attitude, while raw Vicon `tvec`
continues to be fused only as a position observation. Vicon quaternion remains
evaluation-only for the shadow filter. A live Vicon observation without a
capture-to-Crazyflie clock map is labelled as a host-after-wait approximation;
an offline or simulator frame is strict only when it carries a genuine
Crazyflie-clock timestamp.

For an explicit three-flight run, the observer and both listeners are
constructed and registered before arming. The callbacks initially record only
which Crazyflie groups have arrived; they do not enqueue data. Before arming,
the controller requires every mode-specific stream to have reached those exact
listeners and checks the first routed Vicon frame. At interaction start, the
same verified handle is activated and begins filling the bounded queue. Thus
handshake and takeoff cannot overflow the 1 kHz queue, and runtime cannot swap
in an unverified observer. Missing streams, a failed registration, a protocol
mismatch, or activation failure aborts the explicit experiment. Ordinary
non-experiment shadow diagnostics retain their legacy fail-open path.

## Data required for a scientific comparison

Every `contact_attitude_shadow` row records the run, route, estimator phase,
Vicon/onboard/shadow roll-pitch values, timing labels, sensor-correlation label,
and diagnostic integrity counters. The analyzer uses a fail-closed gate:

- Flights 2 and 3 need a Vicon **capture** timestamp mapped to the Crazyflie
  device clock. A host timestamp taken after `waitForNextFrame` is availability
  timing only. Even a small host-time skew cannot enter attitude metrics or
  make the report ready.
- Every metric sample must state `comparison_time_aligned: true`, use an
  accepted strict timing basis, explicitly confirm that a Vicon capture time is
  available and mapped to the Crazyflie clock, carry a finite device-clock skew
  within the configured limit, and have strict device-clock position updates.
- Repeated log rows do not count as new evidence. The default gate requires 30
  unique post-release estimator epochs, 30 unique Vicon frames, at least 10
  distinct accepted strict position updates, and at least 0.20 s of device-time
  coverage. Flight 1 similarly requires 30 unique released mirror epochs and
  0.20 s of coverage.
- Any dropped or skipped packet, rejected position update, contained exception,
  drain-budget overrun, fatal shadow reason, route mismatch, or denial of the
  shared Vicon-position correlation makes the run `FAIL`.
- Missing capture/common-clock metadata, integrity counters, route labels,
  unique-sample coverage, or the minimum sample count makes the run
  `UNSUPPORTED`, never ready.

At present, logs that contain only host-after-wait Vicon timing are expected to
be `UNSUPPORTED`. This is intentional: numerical attitude curves may still be
inspected as diagnostics, but they cannot support the scientific comparison.

After all three complete logs are available, produce the offline report with:

```bash
python -m Interaction.analyze_contact_attitude_runs \
  RUN1.json RUN2.json RUN3.json \
  --output contact_attitude_comparison.json
```

The default gate uses `--max-join-skew-s 0.03` and 30 unique comparison samples
per run. The report includes source hashes and roll/pitch bias, MAE, RMSE,
95th-percentile absolute error, and maximum error only for samples that pass the
strict timing and integrity gates. `READY_FOR_COMPARISON` means that the data is
eligible for offline comparison; it is not an estimator-performance pass and
does not grant flight-control authority.
