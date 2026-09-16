# Contact-release closed-loop validation scaffold

`contact_release_closed_loop_harness.py` is a deterministic, simulation-only
test scaffold. It connects the production `PotentiometerContactDetector` and
`PotentiometerReleaseDetector`, the post-release inertial EKF, the septic
braking profile, and an XYZ swept-envelope checker to a small delayed
second-order attitude plant. It never opens a radio, arms a vehicle, sends a
real command, or grants controller authority.

The release lifecycle preserves the Arduino-style sample identity. The first
fully unloaded sample is the immutable physical boundary that starts a
provisional EKF; a later sample confirms release only after unloaded dwell.
Braking begins at confirmation, not at candidate onset or the physical
boundary. A rebound cancels and discards the provisional seed and observations;
the next unloaded edge receives a new sample identity.

The EKF path is intentionally restricted to gyro, accelerometer specific
force, and position-only observations. Commanded acceleration, prior
setpoints, plant state, odometry velocity, and external quaternion are not EKF
inputs. The simulated commander uses one EKF snapshot at release confirmation
to create the brake profile, then samples that frozen profile by elapsed time;
this is not a continuously replanning feedback controller. The harness latches
its simulated authority off when an estimator, position, IMU, or maximum-
command-hold gate fails.

The EKF seed is exact simulated plant truth at the first unloaded sample. Thus,
this scaffold does not validate the contact-phase gyro attitude observer or its
handoff uncertainty.

The nominal deterministic case detects the first unloaded sample at `0.220 s`
(sample 220), confirms it at `0.260 s` (sample 260), and sends 140 simulated
commands. Its maximum command hold is `0.009 s`, stop-bound excess is
`5.966e-5 m`, terminal speed is `7.196e-5 m/s`, limiting XYZ margin is
`0.467 m`, peak tilt/rate are `5.615 deg` and `21.318 deg/s`, and peak command
jerk is `3.997 m/s^3`. These numbers are regression evidence for this small
plant, not hardware performance predictions.

The returned result exposes independent machine-readable gates for:

- dwell-confirmed contact and release;
- EKF validity and IMU range checks;
- position age, gap, drop, and reorder containment;
- maximum host command hold;
- reverse velocity, delay-compensated stopping-position overshoot, and terminal
  speed;
- a **post-hoc** swept check of the realized XYZ path with vehicle radius,
  reserve, and state uncertainty;
- tilt, tilt rate, and sent-command jerk.

The focused test campaign injects gyro and accelerometer bias, an IMU gap, an
IMU outlier, position delay/drop/reorder, an excessive position delay, a host
send stall, a tight workspace boundary, and an unmodelled acceleration. Run it
with:

```sh
venv/bin/python -m unittest \
  Interaction.tests.test_contact_release_closed_loop_harness -v
```

## Validation boundary

A pass here is not a CrazySim or flight result. A strict firmware-in-loop run,
together with a separate default-off capture, establishes default-off behavior,
atomic IMU transport, position-only execution, independent model-level release
response, gyro-only pre-release staging, exact attitude-seed timing, and common
Gazebo-capture-time ordering. In schema v4 the strict run records `preImuN=247`, `prePosN=0`,
`prePosIgn=50`, post-release `imuN=4080`, `posN=450`, `posRej=366`, a
`1000 us` maximum IMU gap, no late/unmatched input, and `qSeedExact=1`.
`transEpoch=0` is an explicit failure: release p/v have no producer timestamp,
so the artifact is not control-eligible. It also does not score absolute
attitude accuracy or exercise the production offboard commander. Both built
executables are Linux ARM64 SITL binaries, not hardware firmware.

Before this estimator may influence real braking, the detector-to-commander
path must still run against the real commander/send path and onboard watchdog,
repeat the drop/reorder/stall injections, and pass the same no-reverse,
pre-command swept-boundary, attitude/rate/jerk, terminal, and command-hold
gates. Hardware feasibility and props-on validation remain separate later
stages. The strict FIL world uses direct `ApplyLinkWrench`; its independent
world-linear-acceleration high-to-low detector is a valid local response proxy
for that deterministic scenario, not a substitute for contact geometry or the
real load-cell/potentiometer release chain.

The scaffold rejects stale and reordered localization at arrival; it does not
implement delayed-measurement rewind inside `PostReleaseInertialEkf`. The live
gate must therefore also prove the selected timestamp-alignment policy rather
than treating a small synthetic arrival delay as that proof.

The realized-path swept result is also post-hoc validation only. It is not the
pre-command runtime certificate required before a real braking command may be
authorized.

The production gate is stricter than either scaffold: the mapped release,
release gyro, and atomic p/v seed must be the same Crazyflie epoch with zero
mapping uncertainty; the preview must have completed before confirmation; all
IMU source-to-transport delays must be causal and within `0..5 ms`; Vicon
timing uncertainty must be exactly zero; and certified yaw plus positive
body-rate uncertainty must be tied to named calibration evidence. Current
hardware evidence does not satisfy those requirements, so production control
remains disabled. In particular, every independently clocked Arduino mapping is
unconditionally shadow-only in the current implementation, even if a YAML file
claims zero offset, drift, or uncertainty. A future firmware-side or genuinely
shared-clock release latch must establish the exact Crazyflie producer epoch
before active post-release control can be considered.
