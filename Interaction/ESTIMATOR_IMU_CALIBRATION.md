# IMU calibration and offline estimator-3 validation

This workflow is separate from the existing wrench/dynamics `--calibrate`.
It fits static IMU corrections, collects a bounded flight with
**only the default estimator 2 running onboard**, then runs estimator 3 offline
on the analysis computer. These calibration/validation commands never upload a
calibration or flash firmware. The separate, opt-in live XY loading path below
does upload a frozen correction to a paired firmware build.

## What is fitted

- Default `gravity_norm`: 18 distinct static training poses and 6 NEW validation
  poses. Cage/airframe support is allowed; exact axis angles are not required.
  Fit three positive axis scales and three biases by requiring
  `norm((measured - b) / scale) = 9.81 m/s²`. The correction matrix is diagonal.
  Sensor/body alignment rotation and cross-axis terms are **not calibrated**;
  the report sets `alignment_rotation_deg: null` and
  `accel_alignment_calibrated: false`. A shared mounting rotation is invisible
  to gravity magnitude, even when every norm check passes.
- Optional `six_face`, requiring an independently body-aligned fixture:
  `measured = A × reference + b`; correction is
  `solve(A, measured - b)`. The 3×3 matrix identifies scale, alignment and
  cross-axis terms; `b` identifies constant bias in the processed body frame.
- Gyro static zero: the mean residual after the driver's existing boot bias
  correction, in rad/s. This is session/temperature dependent.
- Optional full gyro calibration: `measured_rate = G × true_rate + bg`.
  Known fixed-axis turns fit `G`; independent repeated turns validate it.
  Correction is `solve(G, measured_rate - bg)`.

Neither estimator is used as static calibration truth. Default cage calibration uses
gravity magnitude only. In exact six-face mode the fixture supplies known
body-frame gravity; the gyro rotation fixture supplies a known axis and angle.
The default estimator supplies the user-selected **relative flight comparison
and hover initialization reference**. It shares IMU inputs and is not independent
physical ground truth. The relative hover fit is separate from the static fit.

## Cage-supported arbitrary static poses (default)

Use the same entry, without a YAML or extra option:

```bash
python orchestrator.py --calibrate-estimator-imu --drone-id lb11 --imu-auto-flight
```

Remove props. For each prompt, reposition the cage, secure it without wobble,
then press Enter. Recording waits 2 seconds and captures 4 seconds. The first
six poses approximately cover top/bottom/front/back/left/right. Repeat with
20–40 degree tilts in two other directions, then collect six NEW validation
orientations. Directions are coverage guidance, never attitude truth. Tilted
support may use a stable block/wedge; keep the IMU/flight-controller mounting
unchanged. Avoid resting repeatedly on the same cage contact points.

At least 8 degrees between collected gravity directions is required; similar
poses prompt a retry. Full positive/negative axis coverage, fit conditioning,
quiet telemetry, 0.8–1.2 axis scales and bias below 0.5 m/s² are checked.
Every recorded window is used equally; outliers are not silently discarded.
Independent validation poses are excluded from fitting. Corrected gravity
norm RMSE must stay below 0.12 m/s² for every training and validation window.
These are norm checks, not attitude-accuracy checks. Gyro static zero is still
estimated automatically; dynamic gyro scale/axes require the optional fixture.

A rejected fit prints its reasons and does not continue to flight. Old six-face
records have too few independent directions to be promoted to this method;
collect the new 24-pose session. The original datasets remain unchanged.

Reproduce the cage workflow in the reduced simulator (no aircraft connection):

```bash
venv/bin/python -B -m Interaction.simulate_estimator_imu \
  --calibration-method gravity_norm --output /absolute/new/output_directory
```

This runs three independently noisy known scale/bias cases and one nominal
case through the real collector, standard-controller lifecycle, download checks
and frozen native C estimator replay. UI, transport, plant and flight reference
are simulated; passing does not establish real-flight accuracy or fix mounting
rotation.

## Exact six-face fixture (optional legacy method)

Select it explicitly with `--imu-calibration-method six_face` in orchestrator,
or `--calibration-method six_face` in the direct Pi collector.

Remove props and stop other aircraft clients. Use a rigid fixture whose six
orthogonal faces are independently aligned to BODY X/Y/Z. A rounded cage or
hand-held approximate orientation is insufficient. Positive body axis upward
means positive gravity-specific-force reading:

| Pose | Body specific force (m/s²) |
| --- | --- |
| +X / −X up | (+9.81 / −9.81, 0, 0) |
| +Y / −Y up | (0, +9.81 / −9.81, 0) |
| +Z / −Z up | (0, 0, +9.81 / −9.81) |

In this exact-fixture mode the orchestrator records six training faces and saves a **candidate**, not an
accepted independently validated accelerometer calibration. The direct Pi
collector can additionally record six independently reseated validation faces
when neither `--fit-only` nor `--auto-flight` is selected. The flight comparison
does not change this distinction.

Static windows require at least 200 samples and 2 seconds, monotone CF log ticks,
no gap above 50 ms, bounded noise/drift and stable gyro zero. Scale must be
0.8–1.2, alignment below 10°, accelerometer bias below 0.5 m/s². Independent
static validation limits are 0.12 m/s² RMSE, 0.15 m/s² mean-vector error, 1° angle
and 0.3°/s gyro residual. Repeated faces cannot detect a shared fixture error.

## Full gyro rotation stage

Enable `--imu-calibrate-gyro` in the orchestrator (or `--gyro-calibration` in
the Pi collector). Props remain OFF throughout this stage.

Use an **axis-constrained rotation fixture with independently checked ±90°
mechanical stops**. Freehand rotations or angles read from the default estimator
are not a valid reference. Noncommuting multi-axis rotation cannot be identified
from a single integrated gyro vector in this protocol.

After static fitting, press Enter when the axis fixture is ready. Collect +90°/−90° about each of
BODY X, Y, Z: six training turns, then six independently repeated validation
turns. Each prompt records 10 seconds after the configured settling period:

1. Hold still for the first 2 seconds.
2. At `ROTATE`, turn slowly to the specified stop within the next 6 seconds.
3. At `HOLD STILL`, remain at the stop for the last 2 seconds.

The integrals of measured rates, after removing the static residual zero, are
fitted against the known signed rotations. Each axis must have both signs.
At least 600 samples and 9 seconds are required, with no gaps over 50 ms. Endpoint
stationarity and unchanged zero are checked; peak rate is limited to 120°/s.
Training rotation error must be ≤1°, independent validation error ≤2°; plausible
scale/alignment and independent sample checks also apply. A rejected gyro fit
prevents automatic flight startup and is never exported as an accepted result.

## Run from the orchestrator

Update both offboard and LightBender checkouts. **No flight YAML or interaction
mission is required.** The orchestrator takes the selected aircraft's `init_pos`
from the existing manifest as a marker-selection seed. The Pi reads the existing
`Interaction.config` Vicon host and two-cell battery minimum. The fixed flight
hovers 0.6 m above the measured starting position, moves 20 cm in ±X/±Y and lands.
Fresh Vicon position anchors the actual center before arming; the built-in local
abort fence is ±0.6 m XY and from 0.05 m below to 1 m above that measured start.
Keep that area clear. Single-marker pointcloud tracking needs no rigid body.

```bash
python orchestrator.py --calibrate-estimator-imu --drone-id lb11 --imu-auto-flight
```

After six-face fitting, refit props and use the normal local Dispatcher to select
the marker and confirm launch. The Pi runs detached, just like `--calibrate`;
its controller logs go to `drone_lb11.log`, not the Mac terminal. No Pi-side
flight confirmation or separate Vicon client is started by the collector.
The standard-controller protocol records ordinary IMU logs at 100 Hz. It requires
no interaction, release, grace settings or an experiment SFL. Add
`--imu-calibrate-gyro` only when the known-axis ±90° fixture is available.

Collect the default cage calibration without flying:

```bash
python orchestrator.py --calibrate-estimator-imu --drone-id lb11
```

Each IMU entry first synchronizes its required runtime source files from the
local sibling offboard checkout. Changed Pi files are backed up under
`.imu_runtime_backups/`, uploads are compiled and checked by SHA-256 before
replacement, and an active controller blocks synchronization. No git reset,
firmware change or aircraft command occurs during this step. Failed collection
still downloads its raw data and failure report.

Firmware/fixture/reference descriptions are optional and default to `unknown`,
without startup questions. Supply `--imu-firmware-id`, `--imu-fixture-id`, or
`--imu-reference-note` only when known. Unknown descriptions do not bypass any
sampling/fitting check; reports mark their provenance as undocumented. The firmware ID
must identify the actual flashed build, not the offboard revision. USB is the
default; `--radio` uses the selected aircraft URI. Omit `--drone-id` only when
the manifest/selected mission identifies exactly one aircraft.

### Retry flight capture without repeating calibration

From the Mac orchestrator directory, reuse a saved successful static session:

```bash
python orchestrator.py --validate-estimator-imu --drone-id lb11
```

This selects the latest successful fixture candidate for the selected aircraft
from local orchestrator logs (skipping rejected fits and validation-only copies).
An explicit saved-session path remains optional after `--validate-estimator-imu`.
It opens the normal Mac Dispatcher point-selection UI, uploads the unchanged
dataset and candidate into a new Pi session, and launches background
`controller.py --orchestrated --calibrate` with an IMU validation task. The normal controller
owns localization, PID setup, arming, takeoff and landing; only the airborne
hover/±X/±Y task is replaced. It downloads results to a new local directory and
runs offline replay. READY/START and the local launch confirmation use the normal
swarm workflow. Pi stdout/stderr stay in `drone_<id>.log` instead of being
streamed into the Mac terminal. The normal automatic flight-controller reboot
runs before launching the Pi controller, through the configured radio node when
present. Explicit `--skip-reboot` retains its normal meaning. The workflow never
overwrites the original session or applies calibration
to firmware. Install props, select the marker in the Dispatcher UI, and press
Enter at the Mac's normal launch confirmation. A reboot between static collection
and the standard-controller validation flight is supported: the accelerometer
fit stays unchanged, while a new stationary floor window refreshes the current
gyro zero (2 s settling, 3.1 s recording, at least 200 samples). The entire window
must pass static noise/drift checks with zero motor output. The refreshed gyro
zero and its new clock reference are recorded in flight metadata, tied to the
original candidate hash, and used only by corrected offline replay. IMU sensor
configuration must still match the saved fit. The original candidate is preserved.

When reboot is explicitly skipped, low-level commander ownership can remain
from the preceding run. Validation takeoff starts a fresh high-level
plan, requires its firmware success acknowledgement, then releases low-level
priority before waiting for ascent. At task completion or failure, ownership
is retained until the standard landing routine has acknowledged its replacement
plan; the task does not issue a standalone priority-release notification.
The 25 cm trajectory-deviation limit is unchanged. IMU validation does not
enable the ordinary wrench-calibration capture or attitude-response fitting,
despite using `--calibrate` for the shared lifecycle. Before READY, locked/crashed
supervisor states reject the run explicitly. After arming, takeoff waits for a
fresh armed + can-fly supervisor status, instead of assuming a one-second delay
means readiness. A failed readiness wait disarms without sending a climb target.

During airborne sampling, a stale-telemetry snapshot gets at most 100 ms to
recover. No new motion target is sent during that wait; the previous position
target remains active. Acceptance still requires all streams within their
original ages (100 ms, or 300 ms for health). Persistent outages and other
safety failures abort. Recovery/timeout events include the affected group and
wait duration. Airborne snapshots omit static firmware metadata and copy rows
outside the writer lock to reduce callback contention.

The standard-controller path records ordinary IMU/state logs at 100/50 Hz;
it does not enable the RAM recorder or estimator 3. The Dispatcher workflow uses
the built-in sampling mission and does not accept `--imu-flight-config` overrides.
The original source is unchanged; validation captures/replay are saved under
`orchestrator/logs/<mission_tag>/imu_validation/`. Missing output files remain
in the normal pending-download queue for retry.

### Roll/pitch hover initialization (automatic, offline only)

Use the same `--validate-estimator-imu --drone-id lb11` command. The initial
five-second hover consists of two seconds settling and three seconds training.
The Pi journals that exact window, freezes it before the ±X/±Y movements, and
continues ordinary estimator-2 flight. No new YAML, interaction or prompt is needed.

After download, replay fits an additional **X/Y specific-force offset in the
statically corrected body frame**. The expected specific force comes from the
default estimator's quaternion and a quadratic fit to Vicon position over the
training window. The Z offset, static scale/alignment matrix, gyro zero/matrix,
and yaw calibration remain unchanged. This is a relative operating-condition
initialization; mounting rotation, vibration and reference error cannot be
distinguished from physical IMU bias using these data alone.

Training is restricted to the first contact-free hover, with a constant
estimator-2 position target, at least 200 IMU and 200 Vicon samples, causal
attitude references, at least 2.8 seconds of coverage, and no gap over 50 ms.
Position must remain within 8 cm per axis and 10 cm of its target; fitted speed
must be below 8 cm/s, acceleration below 0.15 m/s² and position residual RMSE
below 1 cm. Reference rotation must remain within 3 degrees, X/Y residual drift
between half-windows below 0.10 m/s², and extra bias below 0.5 m/s² per axis.
An invalid window is rejected without changing the static fit or suppressing
the original raw-versus-static replay.

Both comparison arms initialize once at the same epoch **after training ends**.
The subsequent flight is held out; no bias is learned from motion, release or
interaction. `flight/replay/hover_initialization/relative_fit.json` stores the
frozen X/Y fit and source fingerprint; `effective_candidate.json` stores a
separate offline-only candidate; `comparison.json` stores held-out metrics.
The main replay report and attitude CSV retain the original baseline and add
the matched `static_holdout` and `hover_xy_holdout` arms. Segment/sample mismatches
invalidate this comparison. Yaw is reported for transparency: coupled ESKF
updates may change yaw indirectly, but no yaw correction is fitted.

No calibration is applied to firmware and estimator 3 remains offline.
Fit acceptance, held-out relative improvement, and active flight validation
are separate outcomes; none authorizes automatic deployment.

Direct Pi entry is also available:

```bash
/home/fls/env/bin/python -m Interaction.estimator_validation_flight \
  --drone-id lb11 --calibration Interaction/estimator_calibrations/lb11/SAVED_SESSION \
  --output Interaction/estimator_calibrations/lb11/NEW_SESSION/flight \
  --initial-position 0 -1 0.24
```

### Combined collection and flight sequence

1. Remove props, then collect 18 training + 6 validation static poses without
   typed confirmations (2 s settling + 4 s recording per pose by default).
   Optional exact-fixture mode uses six training faces instead.
2. Complete the optional 12 gyro turns. Fitting runs automatically.
3. Keep the aircraft powered to preserve the gyro zero. Install props, place it
   level at a Vicon-visible starting position, select its marker in the local
   Dispatcher UI, then use the normal launch confirmation (Ctrl+C cancels).
   No relaunch command is needed.
4. Only default estimator 2 + PID position control runs onboard. Existing
   `kalmanPRel.enable` is explicitly disabled when present, estimator 3 is never
   selected, and no Vicon mirror or S-curve release event is sent.
5. The aircraft takes off, hovers, moves 20 cm in +X/−X/+Y/−Y with smooth position
   ramps, returns after each, hovers, and lands. Airborne sampling lasts 58 s;
   standard takeoff/landing and readiness settling add their normal duration.
   Do not push during this capture.
6. The orchestrator downloads the raw data, checks dataset/candidate/flight
   fingerprints, then **automatically compiles and runs offline estimator 3 on
   the analysis computer**. A local sibling offboard checkout, NumPy and a C
   compiler (`cc`, clang or gcc) are required. The Pi needs no compiler.

Flight uses Vicon position only. Default-estimator position, velocity and
quaternion, processed IMU, motor/battery, commands and phase events are saved.
Changed IMU configuration or loss of device-clock continuity since static
collection rejects the flight. Missing/stale tracking, low battery, excessive
default-estimator tilt, boundary or target deviation aborts. Airborne faults
attempt bounded descent and stop/disarm, then require fresh zero-motor telemetry;
tracking/link loss can prevent a controlled landing. Runtime parameters are
restored after stopping, but estimator 3 remains disabled and estimator 2 selected.
Ctrl-C, SIGTERM and SSH SIGHUP use the same bounded shutdown path where possible.

## Offline replay and interpretation

Replay runs the frozen firmware's actual 15-state ESKF C kernel plus its Vicon
position/velocity frontend, once with raw IMU and once with calibrated IMU.
The source fingerprints and build command are retained. The frozen source is
not proven identical to the currently flashed build.

Only the first sample in each segment is initialized from causal, fresh default
estimator position/velocity/quaternion. Subsequent default-estimator attitudes
are **evaluation references only**, never ongoing quaternion observations fed
into estimator 3. Vicon position observations remain causal; there is no future
attitude interpolation. Matrices are applied before the C estimator; its bias
states estimate remaining residuals rather than subtracting the fixture zero
again.

This is a **100 Hz log-rate replay**, not bit-exact high-rate firmware replay.
The replay max IMU gap is 20 ms; larger gaps or missing seed/reference data start
explicitly reported segments. Vicon host receipt times are mapped to CF log ticks,
with causal position fusion within 40 ms; camera capture delay is not identified.
Default-estimator agreement may reflect shared errors. The comparison does not
validate absolute attitude accuracy, active estimator-3 control or interaction
behavior. Report raw/corrected segment counts along with errors.

Run replay again into a new directory without any hardware connection:

```bash
python -m Interaction.replay_estimator3 PATH/flight/packets.jsonl \
  --candidate PATH/fit/candidate.json --output PATH/flight/replay_02
```

## Artifacts

Each new session preserves:

- `dataset.json`, `packets.jsonl`: static and gyro raw windows, rejected attempts,
  provenance, driver parameter snapshots and raw device/host timestamps.
- `fit/report.json`, `fit/report.md`: accelerometer and gyro fit/quality results.
- `fit/calibration.json`: only after independent static and requested gyro checks
  pass. Every successful fit also exports `fit/candidate.json` for flight replay.
- `flight/config.json`, `flight/packets.jsonl`, `flight/report.json`,
  `flight/report.md`: flight capture and failure/completion records.
- Local `flight/replay/report.json`, `report.md`, `attitude.csv`, `build.json`:
  raw/corrected ESKF output, per-phase roll/pitch/yaw RMSE relative to default
  estimator, resets/rejections, and exact native source/build provenance.

Existing session directories are never overwritten. Interrupted sessions retain
raw data and are marked incomplete. Session artifacts are ignored by Git and
must be backed up separately. No `wrench_calibration.json` is overwritten and
no driver EEPROM trim, firmware calibration or global estimator input changes.

For optional high-rate RAM capture during the bounded flight or a normal
level-coast interaction, see [ESTIMATOR_RAM_TRACE.md](ESTIMATOR_RAM_TRACE.md).
This requires an opt-in recorder firmware build; estimator 3 still runs offline.

## Live estimator-3 X/Y initialization

In an existing `level_coast` S-curve interaction mission:

```yaml
level_coast:
  command_mode: orientation
  coast_command_mode: scurve
  estimator_switch_at: release  # detect also supported
  estimator_hover_xy: true     # default false; needs eskfXY API 1 firmware
  # estimator_hover_xy_mode: auto  # default: reuse saved fit, collect only if unavailable
  # Set refresh to remeasure; return to auto afterwards. reuse requires a saved fit.
  # Optional; otherwise newest accepted on-Pi static fit for this drone:
  # estimator_imu_calibration_file: Interaction/estimator_calibrations/lb11/<session>/fit/candidate.json
```

Standard controller takeoff/landing and tracking are reused. During initial
position hold, estimator 2 stays in control. Default mode `auto` first looks for
`fit/hover_xy.json` next to the selected static `fit/candidate.json`. It checks
drone identity, source-candidate fingerprint, firmware API and the fitted
coefficients. A matching saved result is loaded without the 2+3 second sampling
window or extra raw-IMU/quaternion log blocks. Each flight still performs a
fresh firmware commit/readback; an old ACK is never reused.

If no matching result exists (or mode is `refresh`), after the existing
stationary arming gate allow 2 seconds to settle and collect 3 seconds of IMU,
ordinary KF quaternion and Vicon position. Keep hands off during this period.
The same motion/clock/coverage checks as offline fitting apply. A successful fit
and matching firmware ACK atomically save the reusable result; a failed refresh
preserves the last saved file. Set mode back to `auto` after a manual refresh.
Mode `reuse` fails before takeoff if no valid saved result exists, instead of
collecting a new one. The flight loop continues sending position targets while
the worker fits and/or loads parameters.
Detection opens only after fresh commit readback and 300 ms of corrected ESKF
propagation. A rejected fit/load propagates through the normal landing path.

The firmware input rule, in m/s², is:

```
fx_corrected = sx * fx_raw - bx
fy_corrected = sy * fy_raw - by
fz_corrected = fz_raw
gyro_corrected = gyro_raw
```

`sx/sy` include the saved diagonal static scales; `bx/by` include static
bias and this flight's hover residual. Full matrices are rejected before
takeoff. Only the estimator-3 propagation input changes: the sensor driver,
ordinary KF and detection thresholds are untouched. The ESKF is reseeded
from the ordinary KF when the commit is accepted, before estimator 3 gains
authority. Yaw is not calibrated; coupled filter outputs can still change.

`eskfXY.req` commits the four staged values as one set. Fresh `ack`, `on` and
`status` readbacks identify success. The firmware accepts only one commit per
armed flight, while estimator 2 is active and no brake is accepted; staging
later values cannot change the active set. Disarming clears the correction.
The next flight reuses both the static calibration and saved hover compensation.
Reboot clears firmware RAM, so offboard reloads the saved file on the next
flight. No correction is written to firmware persistent storage. The saved
file contains X/Y coefficients only; it does not reuse a per-boot gyro zero.
After changing the aircraft/cage/payload, PID tuning or firmware, use `refresh`.
Persistence and reload are tested in simulation; physical cross-boot accuracy
still needs flight evidence, especially when temperature or vibration changes.

Artifacts: `<log_dir>/<tag>_estimator_xy/{source.json,packets.jsonl,fit.json,loaded.json}`.
`packets.jsonl` exists only when collecting a fresh fit; `loaded.json` records
whether this flight reused the saved compensation. Reusable compensation lives
in the selected calibration's `fit/hover_xy.json`, not in a temporary log folder.
The original candidate remains unchanged. Offline replay now also saves
`onboard_xy_comparison.json`, matching this exact raw-Z/raw-gyro input rule.
Default-estimator agreement is a relative comparison, not independent attitude
truth or a guarantee of active-control performance.

The implementation and reproducible firmware installer are saved in
`Interaction/native_estimator3/estimator_xy_calibration.h` and
`Interaction/install_estimator_xy_firmware.py`. Install into the paired classic
firmware tree, then rebuild its Bolt target; the installer records a patch and
before/after fingerprints and never flashes hardware. This feature defaults
off, so existing firmware and baseline missions continue using their old path.

The paired prebuilt Bolt image, build fingerprints, integration patches and
lb11 Radio-Pi flashing commands are retained in
[the corrected 2026-10-09 firmware release](firmware_images/estimator_xy_20261009_v2/README.md).
Compilation and offline checks passed; flashing and active estimator-3 flight
validation remain separate steps.
