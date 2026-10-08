# Estimator IMU fixture calibration

This is a **separate, motors-off calibration**, independent of the existing
flight `--calibrate` dynamics/attitude-response calibration. It reads the same
driver-processed body accelerometer and gyro channels used by the estimators.
It does not fly, switch estimators, write firmware parameters, or change S-curve
behavior. Stop the controller/orchestrator and remove the props first.

## What it identifies

The accelerometer fit is `measured = A × reference + b`. It estimates a full 3×3
map (alignment, scale, cross-axis terms) and constant bias. The correction is
`solve(A, measured - b)`. Gyro calibration estimates only its stationary
**residual zero after the driver's existing boot bias correction**, in rad/s;
it does not calibrate gyro axis alignment or scale.

The reference is independently known gravity in the **aircraft body frame**.
Neither the ordinary KF nor estimator 3 is attitude ground truth. An unknown
hover cannot separate real tilt from accelerometer bias/alignment.

## Fixture and poses

Use a rigid fixture with six orthogonal faces aligned to the aircraft body axes.
Identify body X/Y/Z from the flight-controller mounting/configuration, not from
the Vicon world axes. A rounded protective cage, a hand-held pose, or an
approximate “looks level” pose is insufficient for this alignment calibration.

The prompts mean the indicated positive or negative **body axis points upward**:

| Prompt | Reference body specific force (m/s²) |
| --- | --- |
| +X up | (9.81, 0, 0) |
| −X up | (−9.81, 0, 0) |
| +Y up | (0, 9.81, 0) |
| −Y up | (0, −9.81, 0) |
| +Z up | (0, 0, 9.81) |
| −Z up | (0, 0, −9.81) |

Collect all six training poses. Then **remove and independently reseat** each
pose for six validation windows. Validation never contributes to the fit.
These repeats detect inconsistent placement and drift; they cannot detect a
shared error in the fixture axes. Verify the fixture independently, ideally
also with a known tilted pose before activating a correction on hardware.

## Run on the Pi

Alternatively, use the local LightBender orchestrator entry after updating both
checkouts. From `lightbender/orchestrator`:

```bash
python orchestrator.py --calibrate-estimator-imu --drone-id lb11
```

It prompts for firmware/fixture/reference provenance, runs this collector over
interactive SSH, and downloads raw data and fit reports to local orchestrator
logs. It branches before any flight setup and does not launch `controller.py`.
The default aircraft connection is USB; add `--radio` for the manifest radio
URI. Omit `--drone-id` only when the manifest/selected mission identifies one
aircraft. Other flight/sensor modes cannot be combined with this flag.

Use a new output directory for every attempt. Supply the identity of the actual
flashed firmware (build tag/hash), not the offboard Git revision.

```bash
cd /home/fls/fls-cf-offboard-controller
/home/fls/env/bin/python -m Interaction.calibrate_estimator_imu collect \
  --uri usb://0 \
  --drone-id lb11 \
  --firmware-id YOUR_FLASHED_FIRMWARE_BUILD \
  --fixture-id YOUR_BODY_ALIGNED_FIXTURE \
  --reference-note "Six independently checked body-aligned fixture faces" \
  --output Interaction/estimator_calibrations/lb11/session_01
```

Use the aircraft's existing `radio://...` URI instead if not using USB. No
orchestrator launch or `--interaction` is involved. After a props-off
confirmation, the tool warms up for 10 seconds, prompts for each pose, waits
2 seconds for settling, and records 4 seconds by default. Set `--duration-s`
(3–30) or `--settle-s` (1–30) if necessary. Pose motion/noise rejects the window
and offers a retry. Missing telemetry, nonzero motor output, disconnection or
buffer overflow stops the session and preserves collected data.

The firmware must expose `acc.x/y/z`, `gyro.x/y/z`, and `motor.m1/m2/m3/m4`.
The tool checks this before logging. IMU uses one 24-byte block at requested
100 Hz; motor output uses a separate block at requested 20 Hz. Acceleration is
converted from g to m/s², gyro from deg/s to rad/s.

## Outputs and acceptance

- `dataset.json`: identities, cached firmware parameter snapshot, all accepted
  pose windows and rejected attempts. An interrupted session is marked incomplete.
- `packets.jsonl`: received raw packets during warmup/collection, including a
  partially failed window. Packets while waiting at human prompts are discarded.
- `fit/report.json`, `fit/report.md`: per-pose error before/after correction,
  fit matrix, bias, quality limits, dataset SHA-256 and acceptance status.
- `fit/calibration.json`: generated **only when fit and validation pass**.

The checks include at least 200 samples and 2 seconds per window; strictly
increasing device log ticks; no gap over 50 ms; static acceleration/gyro noise
and drift bounds; diverse positive/negative axis coverage; plausible matrix
scale, alignment and bias; stable residual gyro zero; and independent
validation acceleration RMSE ≤0.12 m/s², mean-vector error ≤0.15 m/s²,
angle error ≤1°, gyro zero error ≤0.3°/s. These are initial engineering
acceptance limits, not measured hardware accuracy guarantees. A poor reference
can still produce a repeatable but wrong calibration.

Refit saved data without connecting to an aircraft:

```bash
/home/fls/env/bin/python -m Interaction.calibrate_estimator_imu fit \
  Interaction/estimator_calibrations/lb11/session_01/dataset.json \
  --output Interaction/estimator_calibrations/lb11/session_01/refit_01
```

Existing output directories are refused, preserving prior accepted results.
Machine-specific session files are ignored by Git. Back them up separately.

## Applying the result and remaining work

**Saving this file does not change estimator 3.** The current onboard ESKF has
no complete 3×3 IMU correction loader; the flight controller does not load this
new file. Hardware activation requires a compatible firmware interface, checked
readback and a controlled validation flight. Do not put this JSON into
`wrench_calibration.json`, overwrite driver EEPROM trim with it, or subtract its
gyro zero a second time from already-corrected data.

The fit applies to this processed sensor frame, firmware/driver configuration,
mounting and capture session. Record the sensor configuration; changing IMU
mounting or EEPROM gravity trim invalidates it. Residual gyro zero can change
with reboot and temperature; it is not a universally persistent offset.

Static poses do **not** identify Vicon transport delay, clock offset, gyro
scale/axis errors, or attitude behavior under motor vibration/contact. CRTP
timestamps are log ticks, not IMU capture timestamps; the six IMU fields are in
one packet but not guaranteed to be an atomic sensor snapshot. Delay calibration
needs separately synchronized capture timestamps and independent dynamic
excitation. The existing clock-bracket tools can measure clock constraints;
they cannot by themselves identify Vicon delay. Real-flight direction symmetry
and braking acceptance remain separate checks after any onboard correction.
