# RAM capture for offline estimator-3 diagnosis

Status: local tests and firmware build only. Not flashed or validated on hardware.
This adds a recorder to firmware; it does not enable/run estimator 3. The existing
100 Hz calibration/replay path remains available without this firmware feature.

## Diagnostic firmware

In the sibling crazyflie-firmware checkout, use a separate output directory:

```sh
make KBUILD_OUTPUT="$PWD/build-ram-trace" bolt_defconfig
./scripts/kconfig/merge_config.sh -O build-ram-trace -m build-ram-trace/.config configs/estimator_ram_trace.conf
make KBUILD_OUTPUT="$PWD/build-ram-trace" olddefconfig
make KBUILD_OUTPUT="$PWD/build-ram-trace" -j8
```

Use the actual platform defconfig instead of bolt_defconfig for a different
vehicle. The fragment forces default Kalman and excludes the experimental
post-release shadow filter. This checkout is not verified to be the firmware
currently installed on the aircraft. Preserve a known-good image and verify
board/source identity before any later flashing. No flash command is run by this
feature. First validate with props removed and inspect max_hook_us and the
existing stabilizer timing diagnostics before attempting flight.

Capture is opt-in at build time and runtime. Kconfig defaults off. The buffer
uses 25,600 bytes plus bookkeeping and does not allocate during recording.
The local Bolt link check retained about 11 KiB normal RAM and 10 KiB CCM;
other configurations must be linked and checked separately. This is a static
memory check, not measured CPU scheduling or stack/heap validation.

## Normal level-coast interaction

Add to the existing `Interaction.config.level_coast` mapping in the SFL YAML:

```yaml
command_mode: orientation       # position is also supported
coast_command_mode: orientation # position is also supported; no S-curve in this diagnostic run
ram_trace:
  hz: 1000                     # 1000, 500, or 250
  post_release_ms: 200
```

Do not change the current detector just for recording. The recorder is prepared
with fresh parameter readback before arming. It starts rolling when level-coast
first reaches ready/contact. The first confirmed system release sends a marker;
firmware freezes after post_release_ms measured from reception of that marker.
Subsequent interactions cannot overwrite the frozen buffer. There is no memory
transfer during interaction. On normal shutdown, after landing and fresh zero
motor output are confirmed, files are written under:

```
<log_dir>/<tag>_ram_trace/trace.bin
<log_dir>/<tag>_ram_trace/report.json
<log_dir>/<tag>_ram_trace/packets.jsonl
```

The marker is the firmware reception of the offboard system-release request,
not physical release or the exact detector evaluation time. Retain the normal
Pot/model and phase logs to distinguish those events. Only the first interaction
is captured. Run a new trial to capture another. If no release occurs, shutdown
freezes the most recent window. If landing/link fails, do not reboot: the frozen
RAM may still be recovered, and the existing ordinary logs remain available.

S-curve is rejected by this diagnostic option because the capture requires
estimator 2 throughout. The recorder freezes on any estimator change; it never
changes the estimator, controller, gains, motors or localization input itself.

## Rate and retained duration

The buffer holds 640 records; an IMU pair consumes one 40-byte record. Position
observations and 50 Hz default-state/quaternion references share the same ring.
At 1000 IMU pairs/s and roughly 100 position frames/s this retains about 0.53 s;
at 250 pairs/s it retains roughly 1.42 s. Actual durations are reported and depend
on stream rates. Long recording periods overwrite older records intentionally.

1000 Hz mode retains every matched producer pair, without quantizing floats or
intentional downsampling. This is the acquisition stream's actual rate, not a
promise that the hardware produces exactly 1000 pairs each second. 250/500 Hz
modes retain selected unmodified samples and explicitly report decimation.
Never describe those modes as full-rate replay. For several seconds at 1 kHz,
use SD logging or a larger-memory recording design instead of silently reducing
precision or starving the flight firmware of RAM.

## Calibration flight integration

The calibration validation flight requires **no flight YAML**:

```sh
python orchestrator.py --calibrate-estimator-imu --drone-id lb11 \
  --imu-calibrate-gyro --imu-auto-flight
```

The flight uses a built-in 66-second trajectory: hover 0.6 m above the measured
start, move 20 cm in ±X/±Y and land. Initial marker selection comes from the existing
manifest; the Vicon host and battery threshold come from existing offboard Python
configuration. The measured Vicon starting position anchors the flight center.
No interaction, detection, release, or grace configuration is used.

RAM capability is detected before takeoff. When present, a 1000 Hz window starts
one second into +Y_out and freezes 600 ms later. Without the recorder interface,
the protocol explicitly reports ordinary 100 Hz logging. No readback/download
occurs during flight. After stop/disarm, RAM artifacts (if present) are downloaded,
fingerprint checked, locally re-decoded and replayed automatically. Ordinary
100 Hz full-flight replay is retained in either case. Partial artifacts survive
failures. `--imu-flight-config` is an optional advanced override only.

Omit --imu-calibrate-gyro if no known-axis rotation fixture is available; that
omission permits residual gyro bias correction only, not full gyro scale fitting.

## Bench capture and recovery

Close other aircraft clients. With props removed and default estimator 2 already
selected, capture without sending any motor/flight commands:

```sh
python -m Interaction.estimator_ram_trace record --uri usb://0 \
  --hz 1000 --duration-ms 600 --output Interaction/estimator_calibrations/bench_ram_01
```

The utility asks for PROPS OFF, records, freezes, then downloads. It never selects
an estimator or enables shadow execution. To recover an already frozen recording
without starting or erasing another:

```sh
python -m Interaction.estimator_ram_trace download --uri usb://0 \
  --output Interaction/estimator_calibrations/recovered_ram_01
```

Keep the aircraft powered until download completes. An aircraft reboot destroys
RAM contents. Output directories must be new. A mismatched session or a changing
snapshot is rejected instead of combining captures. Raw binary is saved before
content validation, preserving malformed captures for diagnosis.

## Replay

On the analysis computer, using the successful calibration candidate from the
same vehicle and IMU session:

```sh
python -m Interaction.replay_estimator3 PATH/packets.jsonl \
  --candidate PATH/fit/candidate.json --output PATH/ram_replay_01
```

Report files include raw/corrected quaternion, position, velocity and residual
bias histories, phase RMSE, segment counts and source/build fingerprints. RAM
packets use producer/firmware microsecond clocks; they are not rounded to the
ordinary 1 ms CF log clock. Causal default-state references seed each segment
once and serve as relative comparison references, never ongoing attitude
observations. Gaps larger than three configured sample periods (minimum 2 ms)
start explicitly reported segments. Kernel's independent maximum-gap protection
remains enabled. Position fusion uses firmware receipt time, not camera capture
time. Default-state timestamps mark publication/availability, not exact internal
Kalman propagation time. This remaining reference lag must be considered.

The report lists queue rejections, unmatched IMU halves, intentional decimation,
ring overwrites, accepted sample count, retained duration and maximum gap. A
single pending gyro at a freeze boundary can count as unmatched. max_hook_us is
measured on hardware over active append/service critical sections; it does not
measure all scheduler disturbance or inactive-call overhead. It is not populated
by local simulation. Task-produced IMU and external position/pose observations
are recorded; ISR-produced measurements are not included. Pose quaternion is
intentionally not an estimator-3 observation.

The replay still uses the frozen C kernel in Interaction/native_estimator3.
It is not verified identical to a currently flashed estimator-3 build, does not
recreate scheduler contention, and uses default Kalman as a shared-sensor relative
reference. A matching replay can identify mechanisms; it does not prove absolute
attitude accuracy or safe active estimator-3 control.
