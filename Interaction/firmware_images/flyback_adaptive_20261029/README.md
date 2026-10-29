# Marker-point compensation and automatic S-curve unwind

Local candidate `bolt-flyback-auto-26102901`, based on the deployed `bolt-scurve-40default-26102903` source pair.
This image was flashed to lb11 on 2026-10-29 through Radio Pi 10.182.71.171.
All 383,592 application bytes matched the packaged image on full readback.
The known CPU identity, new build tag, self-test, 65 required parameters, new log APIs, unarmed state and zero motor outputs were confirmed after reboot.
See `flash-readback.json` and `deployment/`.
At the time of flash verification, the Pi controller code had not yet been synchronized.
Automatic unwind and marker compensation are boot-disabled until the updated mission controller configures them.
No arming, setpoints or flight were performed.
Flight behavior, localization readiness and target control-loop timing remain unvalidated.
Both pre- and post-flash console captures contain ordinary-Kalman out-of-bounds resets while no mission was running; these are retained in the evidence and are not counted as a successful localization test.
The paired source changes, original source fingerprints, native tests, build configuration and linked Bolt image are preserved here.
No PID gains, 40-degree command envelope, battery/boundary gates or no-coasting behavior are changed.

## One translational measurement point

The user reports a single Vicon marker approximately 86 mm above the FC IMU.
The body-frame lever arm is configured as `marker_from_imu_m: [0, 0, 0.086]`.
Horizontal offsets remain an unmeasured zero assumption; this is not a full rigid-body calibration.
The geometry is applied inside the FC to the raw position observation; the RPi continues to transmit raw Vicon marker positions.

With body-to-world rotation R and IMU-to-marker vector r:

```
x_control = x_marker - (R r - r)
v_marker  = v_IMU + R (omega x r)
```

The constant world offset r preserves the historical level-marker hover coordinates, including height.
The translational velocity refers to the IMU point, not the marker or an asserted center of mass.
The ordinary Kalman position measurement includes the attitude Jacobian `-R[r]_cross` for its right/body attitude-error convention.
A fresh ordinary-Kalman quaternion also de-rotates the Vicon frontend positions before differentiation.
The 15-state estimator then fuses this same-point frontend position/velocity with the existing conservative CI weight 0.999.
The corrected frontend is correlated with attitude; it is not an independent position/velocity ground truth.
Its measurement-variance floor increases from 1e-6 to 1e-5 m² to account for geometry/attitude uncertainty.
This floor and the 20 ms attitude freshness gate do not replace an independent uncertainty validation.

`mocapRef` telemetry exports a coherent corrected position/velocity/receipt-age snapshot.
The RPi uses it for physical stopping and position capture instead of differentiating the raw top-marker position again.
A stale packet, invalid source age, source disagreement over 0.10 m, or missing supporting firmware is rejected.
Position-only Vicon input and grounded startup are required.
Supporting firmware clears geometry for an ordinary/no-coast mission; an old image does not need the new API for no-coast.

## Automatic unwind and measured completion

The release retains strong directional braking.
Every control tick recomputes delayed state from current velocity, roll/pitch, Euler angular rates and the already-issued command queue, using the calibrated attitude-response model.
A bounded search selects a feasible seventh-order actual-attitude return with measured initial angle/angular rate and model-derived initial angular acceleration/jerk, ending at level attitude with zero derivatives.
The return duration is selected from the remaining velocity impulse and the angle/command/slew constraints rather than from `tail_s`.
The planner also screens the minimum predicted signed velocity during the return; terminal velocity alone is insufficient.
The response inverse uses measured-state feedback throughout the return.
The existing bounded acceleration-residual observer remains available; physical tilt acceptance uses actual attitude, not the bias-adjusted model angle.

After the reference ends, attitude ownership continues while measured velocity, angle and angular rate converge.
If a smooth return is already predicted to reverse, it is not executed; bounded measured-state damping is used instead.
This corrective path cannot promise to undo stopping impulse already present at a late release.
No fixed historical trajectory endpoint is used as the final position target.

Configuration is under `wrench_interaction.firmware_auto_brake.analytic_profile`:

```yaml
auto_unwind: true
end_velocity_m_s: 0.0
position_tracking_bandwidth: 0.0
marker_from_imu_m: [0.0, 0.0, 0.086]
```

The fixed-position trajectory tracking pole must be zero for this free stop.
The default profile and old firmware path remain unchanged unless this mode is explicitly selected.
`tail_s` remains a valid legacy configuration value but is not used by automatic unwind.
The ordinary reference-deceleration setting remains configurable, currently 3.6 m/s² in the active local mission.
The existing maximum total release duration remains configurable, currently 6 s.
The automatic duration search is bounded to 0.15–1.2 s, and the combined command-slew bound is 160 degrees/s.
A conservative internal reference-return velocity reserve of 0.12 m/s is subsequently removed by measured-state settling; it is not the requested final velocity or a relaxed terminal gate.
These implementation bounds are compile-time constants in the included header.
The final velocity target is configurable from 0 to 0.02 m/s; hover uses zero.

All of the following must hold continuously for 100 ms on fresh feedback before FC position hold:

| Quantity | Acceptance |
| --- | --- |
| XY velocity error from the requested terminal vector | <= 0.02 m/s |
| Norm of measured roll/pitch | <= 0.5 degrees |
| Norm of measured roll/pitch Euler rates | <= 5 degrees/s |

The reference must also have finished.
The delay-compensated remaining stop velocity, command deviation from calibrated equilibrium and command slew must also be small, so a quiet measured state cannot hide a pending braking command.
A stopped translation with nonzero tilt/rate does not qualify.
Loss of freshness resets the dwell.
A failed/expired plan uses the existing reported abort path; expiration never counts as successful stopping.

The firmware declares read-only `hlCommander.pRelUnwVer=1` and `mocapMarker.api=1`.
The host confirms capabilities and selected values from fresh replies before enabling braking and rechecks before arming.
The new selector/target and tilt limit are latched with the accepted release.
The boot default `pRelUnwAuto=0` preserves the legacy path.
`AUTO_UNWIND` logs phase, selected return duration, predicted terminal reference velocity and the reason for corrective damping.
`scUnwWhy` is 0 for the ordinary path, 1 for no feasible return, 2 for predicted terminal reversal and 3 for predicted intermediate reversal.
Curve-event flag 0x02000000 marks body pitch/roll reference coefficients in degrees; an initial brake event has no fixed polynomial.
The exporter does not invent an exact command or velocity trajectory from these coefficients alone, because feedback, command history and residual state are also needed.

## Position capture and estimator restoration

The position capture still inverts the confirmed P-only position/velocity cascade, so FC velocity-estimate bias does not become a new physical stop target.
It now measures settling while estimator 3 remains selected.
Only after this independent dwell does the RPi request estimator 2.
Fresh post-acknowledgement samples and a second dwell are required before freezing the hover anchor.
The existing speed, tilt, angular-rate, acceleration, jerk, timeout and sample-age constraints remain active.
PB and OB retain their existing capture behavior and benefit from the same-point reference when configured with the marker geometry.
With `coasting_enabled: false`, the marker compensation, automatic unwind and position-capture additions are inactive.

## Evidence and limits

`native-validation.json` records 1,944 closed-loop second-order model cases across eight XY headings, release speeds 0.2/0.4/0.8 m/s, initial along-axis tilt -10/0/5 degrees, initial angular rate -10/0/10 degrees/s, a coupled ±10% natural-frequency/inverse-gain perturbation and ±6 ms command-delay error.
All completed the unchanged physical terminal gates within the configured deadline and the stated native reversal bounds (20 mm/s peak reverse speed, 5 mm reverse displacement).
The observed worst reverse displacement was approximately 0.51 mm and peak reverse speed 8.46 mm/s.
Selected reference-return durations ranged from approximately 0.195 to 0.540 s.
These are ideal-state model simulations, not Vicon/IMU flight validation or a full rigid-body/motor simulation.
They do not test every independently varying model parameter, sensor bias, localization delay, thrust limit or the full 40-degree operating envelope.

A separately reported late-braking case (0.1 m/s and an initial 10-degree braking tilt) still reversed approximately 51 mm before recovering.
It is deliberately retained as a limitation rather than reported as solved.
Earlier detection/unwind initiation and accurate state/response estimates are essential; no terminal-velocity-only rule can guarantee no flyback from arbitrary initial angle and angular momentum.

Native ASan/UBSan checks also cover rotation-only marker motion, finite-difference attitude Jacobians, variance preservation through gaps, stale feedback and zero-speed/nonzero-angle/nonzero-rate handoff rejection.
The Bolt image compiles with the preserved configuration and fits its flash/RAM regions.
Target CPU timing/stack margin, the estimator-transition jump and physical flyback must still be measured on the aircraft.
The existing `hlCommander.scUs` reports the maximum control-step time for that check.

## Reproduce locally

```sh
python3 Interaction/firmware_images/flyback_adaptive_20261029/validate.py
```

This compiles the packaged native C sources with sanitizers, checks package fingerprints and verifies the actual linked ELF parameter table against the example override.
It performs no device access.
`patches/candidate.patch` applies with `patch -p1` to either preserved source tree named in `patches/source.json`.
Build the patched Bolt tree with its preserved `bolt.config` copied to `build-unified/.config`:

```sh
make -j4 KBUILD_OUTPUT="$PWD/build-unified" O="$PWD/build-unified"
```
