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

At release, flights 2 and 3 latch position and velocity from one atomic
onboard-EKF state packet, and attitude plus gyro bias from gyro history. The
host-loop state, past commands, current setpoint, and pending command history
are diagnostic-only and never reconstruct release velocity. After release,
measured body-frame specific force drives velocity through the estimated
attitude, while raw Vicon `tvec` is fused only as a position observation. Vicon
quaternion remains evaluation-only for the shadow filter.

Control-grade handoff is deliberately stricter than these shadow comparison
runs. The first-unloaded preview must finish before dwell confirmation; its
complete identity and mapping evidence are immutable. The mapped release,
release gyro, and atomic p/v seed must be exactly the same raw and unwrapped CF
epoch with zero mapping uncertainty. Every producer-to-transport IMU delay must
be causal and within `0..5 ms`; the wider `±100 ms` low-16 reconstruction
window is diagnostic only. A live Vicon observation without an exact-zero
capture-to-Crazyflie clock map remains approximate and ineligible. Absolute yaw
must include a numeric value and certificate bound to the configured alignment,
and body-rate uncertainty must be strictly positive and tied to the same named
joint IMU calibration artifact. The present independent Arduino clock mapping
is always shadow-only, even if YAML claims zero error. The estimator authority
gate accepts only the future `firmware_shared_clock_release_latch_v1` basis;
Arduino and CrazySim clock bases remain diagnostic-only.

For eventual active use, those named values cannot remain self-reported YAML
claims. The release/yaw/IMU/body-rate/EKF-noise evidence must be loaded from an
immutable artifact bound to the vehicle, rig and extrinsics, firmware, boot/run
session, validity interval, and content hash. The yaw artifact must include its
measured uncertainty. A continuous Crazyflie clock/boot heartbeat and a
post-host-stall recovery dwell are also required because callback receive age
alone cannot rule out a cached packet that was delayed before reaching the host.

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

## Local validation status (2026-09-14)

The local shadow implementation and the position-only CrazySim plant gate are
complete. The captured trace used an explicitly armed simulated vehicle, a
powered-hover gate, a 100 ms scheduled wrench interval, 1 kHz IMU, 200 Hz
odometry, and 1.5 s of post-release motion. All records used the Gazebo
simulation clock. Odometry quaternion was retained only for scoring; a
mutation/removal canary confirmed that it did not change the estimator output.
The saved evaluation reports are:

- `autoresearch/loop-260913-1317/crazysim-trace-v2.jsonl` (retained source);
- `autoresearch/loop-260913-1317/crazysim-capture-report-v2.json`;
- `autoresearch/loop-260913-1317/crazysim-nominal-v2.json`;
- `autoresearch/loop-260913-1317/crazysim-bias-stress-v2.json`.

The nominal run passed the absolute error gates: fused attitude RMSE/p95/final
were `0.221/0.550/0.175 deg`. Its ideal simulated gyro-only baseline was better
(`0.030/0.045/0.015 deg`), so no nominal A/B-improvement claim is made. With a
deterministic `[+1.0, -0.6, 0.0] deg/s` post-release gyro-bias stress, the fused
RMSE/p95/final were `0.582/0.867/0.902 deg`, versus
`0.999/1.636/1.732 deg` for gyro-only. The fused-to-gyro RMSE ratio was `0.583`.
Both runs used 300 position-only updates with zero rejection, a 4 ms causal
finite-difference seed, a `0.174 deg` release-seed attitude error, and no command
history as estimator input. That 4 ms trace passes the offline numerical
evaluator but cannot pass the newer production exact-epoch gate; it is attitude
estimator evidence, not authority evidence. The simulator's odometry twist is
ignored: release velocity is calculated in the world frame from a 10 ms causal
position-only finite difference, and the latest causal position is extrapolated
to the release epoch. Mutating every odometry velocity and quaternion leaves the
estimator-output hash unchanged.

The firmware core is also ported behind an opt-in build switch and remains
runtime-disabled by default. It passes 11 targeted lifecycle tests, all 15
`kalman_core` tests, isolated strict/default Docker builds and CTest, and an
isolated strict CrazySim firmware-in-loop input chain. Schema v4 stages only
gyro attitude/bias before release (`preImuN=247`), fuses no pre-release position
(`prePosN=0`) and explicitly ignores 50 such samples. Wrench clear occurred at
`9,145,000 us`; an independent post-physics world-linear-acceleration detector
observed release at `9,146,000 us`. The attitude seed is exact at that epoch
(`qSeedExact=1`, attitude `seedSkew=0`). After release the run records
`imuN=4080`, `posN=450`, `posRej=366`, a `1000 us` maximum IMU gap, zero late
inputs, zero pair skew, and zero unmatched inputs. A separate default-off run
passes without a release-sensor artifact. Both generated `cf2` files are Linux
ARM64 SITL executables, not flashable Crazyflie firmware.

This strict run closes local transport, independent release-response, lifecycle,
and attitude-seed timing gates; it does not prove the translation seed epoch.
Release p/v are copied from an untimestamped current firmware Kalman core, so
`transEpoch=0` and control eligibility is explicitly false. The world applies a
direct wrench and has no contact geometry or load-cell/potentiometer model, and
absolute attitude accuracy is unscored. Real command authority also remains
blocked by the absence of a firmware/shared-clock physical release latch,
exact-zero Vicon timing, resolved session/vehicle calibration artifacts,
calibrated EKF noise/prior and observability thresholds, current planar braking
calibration, real commander/watchdog fault injection, and the on-site three-run
protocol. No local result in this section authorizes deployment, flashing, or
flight-control authority.
