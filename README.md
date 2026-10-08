Flashing firmwares:
Download the firmwares from here
https://github.com/bitcraze/crazyflie-release/releases

Flash stem32-fw and nrf-51-fw. After running each command hold cf power button for 3 seconds until the blue light flashes. if power button is broken use a wire to connect adjacent pins of the power button for 3 seconds.
https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/functional-areas/cfloader/

```commandline
git clone "https://github.com/bitcraze/crazyflie-clients-python"
cd crazyflie-clients-python
bin/cfloader flash ~/Downloads/cf2-2023.11.bin stm32-fw
```
```commandline
bin/cfloader flash ~/Downloads/cf2_nrf-2024.2.bin nrf51-fw
```

Flashing bootloader:
Download:
https://github.com/bitcraze/crazyflie2-stm-bootloader/releases/tag/1.0

Flash:
https://wiki.bitcraze.io/projects:crazyflie2:development:dfu

```commandline
sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D ~/Downloads/cf2loader-1.0.bin 
```

Install dependencies:
```
git clone https://github.com/bitcraze/crazyflie-lib-python.git
cd crazyflie-lib-python
pip install -e .
pip install pyserial
```
```
git clone https://github.com/flslab/fls-cf-offboard-controller.git
```
```commandline
pip install -r requirements.txt
```

Servo:
```commandline
pip install gpiozero lgpio pigpio
```
Add to `/boot/firmware/config.txt`
```
dtoverlay=pwm
```

CrazyRadio:
To run the controller on your Linux PC or a Raspberry Pi and control the flight of an FLS (built upon a CrazyBolt FC) or a Crazyflie drone using a CrazyRadio, please read:

https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/

After installing, run ```lsusb``` in the terminal, check if it shows device with ```ID 1915:7777``` or ```0483:5740```:
```
Bus 00X Device 00X: ID 1915:7777 Nordic Semiconductor ASA Bitcraze Crazyradio (PA) dongle
```

Vicon rigid-body position-only input (opt-in): add
`--vicon-rigidbody-position-only FLS --log` to the existing controller launch,
replacing `FLS` with the exact Vicon rigid-body name. This selects rigid-body
tracking, sends only XYZ through `extpos` to the flight controller, and records
the rigid-body quaternion in the mocap log. It rejects `--vicon-full-pose` and
does not enable the separate `--contact-attitude-run 2` shadow experiment.
Without this option, the existing Vicon route is unchanged.

The high-rate localizer can also send a dedicated yaw-error measurement to a
compatible Crazyflie firmware build. Configure the `yaw_correction` object in
the localizer JSON. The controller accepts only synchronized HyperGrid PnP
solutions that pass the configured reprojection, feature-count, image-span,
innovation, and temporal gates. The packet carries `FC EKF yaw - PnP yaw` in
radians and never forwards PnP roll or pitch.

### Model-contact calibration and parallel diagnostics

Ordinary `--calibrate --log` also records contact-free detector evaluation data
automatically; no extra flag or mission change is needed. Fly the usual XYZ
excitation **without hand contact**. This does not enable the experimental IMU
detector, change flight commands, or add the optional multi-trial braking sweep.
Normal interaction runs keep their existing subscriptions.

The flight JSON includes `FORCE_IMU` (body `acc.x/y/z` in g and `gyro.x/y/z`
in degrees/s), with a requested 10 ms period. State, motor/battery and attitude
target groups are also requested at 10 ms. Packet records retain FC log ticks,
host wall/monotonic receipt times and receipt sequence numbers. These are not
individual IMU sampling timestamps or an atomic cross-group snapshot; actual
rates and gaps must be checked after the flight. Log payloads are checked before
takeoff (24 bytes for IMU, at most 26 per block; one block reserved for battery).

`calibration_contact_capture` in that JSON saves the pre-flight mission and the
previous per-drone calibration entry/hash before the ordinary fit can overwrite
it. Missing previous calibration is marked explicitly. The existing effective
runtime configuration, commands, Vicon frames and wrench records remain logged.
Calibration disables live contact decisions, so zero live onsets is **not** a
false-positive result: replay must enable detector decisions offline using frozen
thresholds/model. A new fit evaluated on the same flight is an in-sample check,
not independent validation. Keep the complete log for the offline comparison.

Normal FC-owned braking now also loads the offboard XYZ wrench alignment
(`model_delay_s`, `model_time_constant_s`, `model_acceleration_scale`) from the
per-drone wrench calibration file. This does not load retired Pi braking fits
or replace the separate FC attitude-response calibration. The effective
configuration records `wrench_detection_calibration` and its source. Missing
files keep mission defaults with a warning; malformed saved vectors are rejected.

Onboard wrench logs include `model_contact_diagnostics` (schema 2): raw and
baseline-corrected **30 ms short-window** model contact decisions, baseline
readiness/offset, and invalid-gap markers. The original 80 ms force path remains
unchanged; its comparison decisions are stored under `long_window`. Saved XYZ
delay/gain/response calibration is also used by the independent short estimator.
The short path requires at least 20 ms of history and 30 ms of continuous
above-threshold evidence, in addition to the existing strength-weighted CUSUM.
This stops a single velocity-estimate step from triggering just because its
short-window residual is large. Force thresholds and release dwell are unchanged.
`window_s` and `minimum_window_s` in the diagnostic configuration override the
short history lengths; continuous confirmation follows `window_s`.
The estimator retains the sample preceding the window boundary: actual duration
can exceed 30 ms by a sampling interval and is logged as `actual_window_s`.
Neither 30 ms nor the minimum history is a promise of total detection latency.
These are **shadow results only** and never change the selected detector,
rendered force, release command or S-curve. Potentiometer detection stays selected.
The bounded XY baseline uses only past idle/stationary samples and freezes on
force changes, motion or contact evidence; it does not consume potentiometer
labels or claim to measure absolute force. Missing windows are not no-contact
evidence. Baseline learning is conservative and may remain unavailable.

Diagnostics are enabled by default for the onboard path. To disable only this
comparison, set `wrench_interaction.model_contact_diagnostics.enabled: false`.
They run on the offboard computer, not the FC. During a blocking firmware-brake
wait, this existing wrench stream still has a gap; do not use it to claim
continuous release/re-contact validation. No active-detector replacement or
physical-flight validation is implied by these logs.

### Potentiometer compression display

With the aircraft grounded and the flight sensor reader / serial monitor stopped,
run from the offboard repository:

```bash
python -m Interaction.monitor_potentiometer --port /dev/serial0
```

This reads the Arduino at 115200 baud and prints raw ADC plus compression in mm
using the flight reader's current `RAW_COMPRESSION_CALIBRATION`, not the Arduino's
already-converted mm column. Default output is five lines per second; use
`--interval 0.5` to slow it down. Out-of-range ADCs display `compression=N/A`,
and a serial-data timeout clears the displayed result with a notice. Stop with
Ctrl+C. This viewer does not modify calibration or start the flight controller.

The 2026-10-05 calibration uses a monotone cubic PCHIP curve through the seven
measured ADC/compression pairs. Both the viewer and flight reader use this same
curve. Its first derivative is continuous and compression stays within each
pair's measured range; there is no high-order polynomial extrapolation. The
existing one-count allowance above the zero-compression ADC remains. Slopes are
precomputed at import using the PCHIP rule without adding a SciPy dependency.
This smooths the static mapping, not the serial signal: ADC noise still changes
the reported compression, and no averaging/filter delay is added. Check accuracy
at additional measured positions; passing through the calibration points is not
independent validation of physical measurement accuracy.

### Potentiometer recording with any detector

Set `Interaction.config.record_potentiometer: true` in the SFL to start the
Arduino reader and record compression/force regardless of `detection_method`.
The default is `false`; detector-required sensing and contact-validation capture
still work as before. The orchestrator automatically supplies `--sense` and
`--log`; direct controller launches need both flags. This option applies to
interaction runs, not calibration, braking-test, baseline, MPC or hover modes.

```yaml
Interaction:
  config:
    behavior: level_coast
    detection_method: model  # potentiometer | model | vel
    record_potentiometer: true
```

The flight JSON contains a `potentiometer_raw` record for **every valid UART
sample**, including while waiting, interacting, coasting and in grace. Fields
include `raw`, `compression_mm`, `force_n`, `arduino_time_ms`, `host_time`,
`host_monotonic_time` and `sample_sequence`. Capture runs independently of the
control loop and the console print rate, using the existing asynchronous writer.
If contact-validation capture is also enabled, both options share one callback
and do not duplicate samples. A `potentiometer_recording` metadata record saves
the detector, calibration method/points and sensor settings, including the spring
constant. Invalid or out-of-calibration-range samples are not converted or saved
as valid force measurements.

`force_n = compression_mm * sense_spring_constant` estimates force along the
spring (default coefficient: `0.16 N/mm`). It is a spring-force estimate, not a
calibration of total user-applied force or damping. With `model` or `vel`, these
extra measurements do not trigger contact/release or change phase transitions.

### Repeated level-attitude interaction

`Interaction.config.behavior: level_coast` selects an independent timed behavior.
Omitting it (or using `standard`) preserves the existing interaction/EKF and
firmware braking path. The separate LightBender mission is
`Interaction/SFL/translation_level_coast.yaml`; select that mission in the swarm
manifest to use it. The original `translation_inertia.yaml` remains available.

The mission selects `wrench_interaction_profile: level_coast`. Its shared settings
live in this offboard repository at `Interaction/profiles/level_coast.yaml`.
The controller expands that profile immediately after downloading the mission,
before preflight and log setup. An optional inline `wrench_interaction` mapping
overrides individual settings recursively; saved XYZ calibration is loaded after
that. The runtime config log records the profile name and full effective settings.
Unknown or missing profiles fail during mission loading. Deploy the offboard code
and profile together when using a mission that references it.

For `behavior: level_coast`, `detection_method` selects the contact/release
detector directly: `potentiometer`, `model`, or `vel` (default: `potentiometer`).
Remove the old `level_coast.detector` field; it is rejected to avoid ambiguous
selection. The shared onboard momentum observer, startup calibration and state
logging remain active for all three methods; the profile supplies
`state_source: onboard`. The LightBender
orchestrator starts the sensor for `potentiometer`, or for any detector when
`record_potentiometer: true`; no `sensing` field or manual CLI flag is needed.
Legacy `sensing` and `--sense` overrides cannot change that selection. Direct
controller launches still require `--sense` for potentiometer hardware setup.
For the separate `standard` behavior, the existing `momentum_impulse`,
`mocap_wrench` and `velocity` method names keep their original meaning.

```yaml
behavior: level_coast
detection_method: potentiometer  # potentiometer | model | vel
wrench_interaction_profile: level_coast
level_coast:
  command_mode: position        # position | orientation; during contact
  coast_command_mode: orientation  # position | orientation | scurve; after confirmed release
```

`level_coast.command_mode` selects the contact policy (default `orientation`).
`coast_command_mode` independently selects the policy after confirmed release;
omitting it inherits `command_mode`, preserving existing missions. All four
combinations work with `potentiometer`, `model`, and `vel`. `position` uses moving
**position packets**; `orientation` sends zero roll/pitch and zero yaw rate at
mission height. The example follows velocity with position commands during
contact, then switches to level attitude after release. Orientation coast does
not actively brake horizontal velocity and may travel farther.

Detector, release/grace timing and coast preemption are shared. A fresh contact
that preempts coast switches back to `command_mode`. The coast-specific speed
threshold below enters position hold: position coast uses the stopping-position
projection below, while orientation coast
captures current XY. This also applies when release is already below the speed
threshold and goes straight to hold. The existing hardcoded
`DETECTION_TO_ORI_DELAY_S` now means the delay before the selected movement
policy (fixed position hold during the delay).

The paper's `p + v * 0.01` is not a velocity command: the FC position loop
turns its small position error into approximately `Kp * v * 0.01`, which can
make the velocity loop brake almost the entire current speed. The new mode
instead computes a desired velocity and inverts the confirmed position P gains
in the FC's body-yaw frame, then rotates the offset back into world XY:
`p_cmd = p_measured + R * diag(1 / Kp_xy) * R.T * v_reference`.
Each fresh state re-anchors the target; duplicate states resend the last packet
without advancing the target or timers. Z stays at mission height; yaw retains
the selected `follow_yaw` / offboard damping behavior.

With position selected, contact follows measured XY velocity. With position
coast selected, a smooth 0.5 s transition from the first coast command
reduces the retained velocity fraction from 1 to 0. The confirmed velocity P
gains bound the nominal braking tilt to an equivalent 0.8 m/s² horizontal
acceleration. This is a setpoint bound, not a guarantee about actual acceleration
or stopping time. At low speed, freeze a short forward stopping projection for
hover using the nominal velocity-P decay (`v_body / (g * radians(Kv_xy))`),
retaining any farther current follow target. This accounts for residual motion
instead of locking directly onto the current point; it does not promise zero
overshoot with real attitude lag. A new contact discards the
coast ramp and follows the new measured direction.

If either phase selects position, setup temporarily sets **only XY
position/velocity I, D and feedforward gains to zero before takeoff**, after
saving and freshly confirming originals.
It leaves P, Z, attitude/rate PID and estimator settings unchanged. This makes
the inversion defined and prevents interaction-induced XY integral windup;
normal XY integral rejection is consequently unavailable during this experiment.
There are no gain writes or blocking parameter reads in the interaction loop.
Original gains are restored only after confirmed landing/stop; an interrupted
run retains a per-drone recovery file for the next grounded startup. Startup
and pre-arm confirmation require the existing PID parameters; no firmware patch
is needed. Missing parameters or unconfirmed gains prevent arming.

Defaults live in `Interaction/position_follow.py`; optional overrides belong in
`level_coast.position_control`: `contact_velocity_retention` (1.0),
`coast_velocity_retention` (0.0), `coast_transition_s` (0.5),
`max_brake_acceleration_m_s2` (0.8), and `max_offset_m` (0.6).
Generated targets must satisfy the existing flight boundaries, maximum offset
and confirmed FC velocity limits; infeasible targets abort through the existing
landing lifecycle instead of clipping to an unmodelled braking command.
Logs include the confirmed PID context, commanded position, velocity reference,
requested retention and nominal pitch/roll. Validate these against FC
`posCtl.targetVX/VY` and actual attitude in flight; network delay, FC filtering
and attitude response are not exactly inverted by the offboard calculation.
The model and potentiometer choices reuse the existing contact/release detectors
and saved XYZ detection calibration. Velocity uses XY speed hysteresis and dwell;
a low-speed release is a heuristic, not a separate measurement of hand contact.

When both phases select orientation, after the existing stationary arming gate,
contact sends
`send_zdistance_setpoint(0, 0, 0, nominal_z)` continuously. Confirmed release
keeps that same command until `dot(velocity_xy, interaction_direction_xy) <
level_coast.stop_speed_m_s` (default `0.03` m/s), or a newly detected interaction
preempts coast as described below. The projection is signed: lateral drift does
not delay capture, and reverse motion also satisfies the threshold. Capture holds
current XY at nominal Z; orientation-only runs reset position/velocity integrators.

The world-XY direction is normalized and locked at each accepted contact onset:
model reuses its detector's force direction (or onset force if projection is
disabled); potentiometer reuses the signed sensor force transformed to world
coordinates; velocity detection uses onset velocity. A missing force direction
falls back to onset velocity. The axis stays fixed through contact and coast,
including detector resets at grace expiry; a new accepted interaction replaces
it. If both direction and onset velocity are degenerate, use the full XY speed
and log `xy_norm_no_direction`. Position coast continues to require
`hypot(vx, vy) < stop_speed_m_s`. Logs include `interaction_direction_xy`, its
source, signed `interaction_velocity_m_s`, `stop_speed_metric`, and the value
compared against the threshold.

`grace_time` specifies seconds (default `0.5`), and `level_coast.grace_start` selects:

- `speed_threshold` (default): start grace at low-speed capture, hold position,
  then reset detectors and repeat stationary arming. Coast/grace ignore onsets.
- `release`: start grace at each **confirmed** release. At expiry, clear detector
  evidence and the release latch, then allow a fresh onset even during coast.
  That onset immediately enters contact; the next confirmed release restarts grace.
  There is no repeated stationary arming in this mode. If low speed arrives before
  expiry, hold the captured position for the remaining grace; if it arrives later,
  hold position and remain ready. Initial startup still requires stationary arming.

For example, `grace_time: 0.3` with `level_coast.grace_start: release` reopens
detection 300 ms after confirmed release. Detection still requires fresh onset
evidence; velocity/potentiometer detectors retain their unloaded baseline rule.
This is a refractory interval, not proof that residual model force has decayed.
Logs record the grace origin, detection enablement, and `coast_preempted` transitions.

`Interaction/level_coast.py` hardcodes `DETECTION_TO_ORI_DELAY_S = 0.0`
(seconds). After a confirmed onset, keep sending the existing position target
until this interval expires, then send the selected phase's command. Set it to
`0.0` for immediate switching. Release detection, safety checks and the mission
duration continue during the delay; an early release keeps its original grace
start and switches to `coast_command_mode` when the delay ends. If a new contact
preempts coast,
capture current XY for the new position-delay interval. Logs keep contact and
release times separate from `Level Coast Command Mode Changed`, and the console
prints phase changes and command changes such as `cmd: pos -> ori` and
`cmd: ori -> hold`. Logs record the previous and current command modes, including
switches at release and coast preemption.

`level_coast.yaw_rate_damping: true` uses the original firmware and standard
position/z-distance packets. Offboard temporarily sets the four existing
`pid_attitude.yaw_kp/ki/kd/kff` parameters to zero once the initial interaction
stability gate is satisfied and confirms them by fresh reads. Preflight only
checks and backs up the original gains; takeoff and stability waiting retain
normal yaw control. Position commands continue during asynchronous confirmation,
and interaction becomes ready only after confirmation succeeds. Both position
and attitude commands then target zero rate without a heading-restoring term.

`level_coast.yaw_rate_deadband_deg_s` defaults to `10.0` when damping is enabled.
The latest synchronized onboard body-Z angular rate is converted from rad/s to
deg/s. Below the threshold in either direction, all four `pid_rate.yaw_*` gains
are zero, giving zero yaw PID output once the write takes effect. At or above
the threshold, restore only the original `pid_rate.yaw_kp` for proportional
rate damping. Keep yaw rate I/D/feedforward gains zero throughout this mode:
in particular, the firmware still accumulates its integral with zero gain, so
restoring I in flight could cause an unwanted turn. Roll/pitch control is unchanged.
Set the deadband to `0` to retain the previous full yaw-rate PID continuously.

Only threshold crossings write the rate P gain. A worker coalesces changes to
the latest requested state and confirms each switch without blocking flight
commands; failed confirmation aborts the interaction. This is an offboard gate,
so measurement/radio/parameter latency still applies, not a hard real-time
firmware deadband. Observer logs include `yaw_rate_measured_deg_s`,
`yaw_rate_damping_requested`, `yaw_rate_damping_output_enabled` (last confirmed
state), and `yaw_rate_damping_switch_pending`. Passive yaw drift below the
threshold is allowed; this is not heading hold.

On interaction exit, asynchronously enable continuous P damping for landing;
do not restore the accumulated integral while airborne. It requires PID control,
grounded startup and normal landing; it is incompatible with `follow_yaw`.
Original gains are restored only after landing/stop. A recovery record in
`cache/yaw-gains-*.json` is retained if restoration fails, and is restored at the
next grounded startup, even when the option is disabled. Both the earlier
angle-only backups and the new angle/rate backups are supported. No firmware
source, packet format, flash or persistent parameter storage is changed.

`level_coast.follow_yaw: true` updates yaw in all position packets from the current
onboard estimate, including position contact/coast. Disabled or omitted sends
absolute `yaw=0` in position packets, regardless of mission target yaw. Orientation
packets always send zero yaw **rate**, with zero roll/pitch and nominal height. No firmware change
or yaw-contact detector is required. Following waits for the first fresh yaw sample;
it does not substitute a zero heading while awaiting startup state.

`duration` covers the entire repeated loop after observer startup, including
preparation and grace. At expiry in any phase, control returns to the ordinary
mission landing lifecycle. Orientation/position coast requires `firmware_auto_brake.enabled:
false`; those options do not submit release events or braking curves. Existing configured
PID attitude-source switching follows entry into and exit from orientation
packets. State/motor freshness, measured boundaries, battery and operator-abort
checks remain active.
The example is an offline-tested configuration, not flight validation.

### S-curve coast with contact estimator switching

With `wrench_interaction_profile: level_coast`, select:

```yaml
level_coast:
  command_mode: orientation  # orientation | position
  coast_command_mode: scurve # orientation | position | scurve
  estimator_switch_at: release # detect | release (default); S-curve mode only
  grace_start: release       # release | speed_threshold
```

This automatically overlays `Interaction/profiles/level_coast_scurve.yaml`;
mission `wrench_interaction` overrides still win. The overlay reuses the existing
`translation_inertia` compensated firmware `velocity_scurve`, attitude execution,
Vicon15 feedback and forward curve-endpoint hold. Keep the existing calibration
file `Interaction/attitude_response.json`. No firmware source change or flashing
is performed by this option; the connected paired build must already expose the
existing runtime and endpoint capabilities, which are checked before arming.

`estimator_switch_at: release` (the default, including when omitted) keeps
`stabilizer.estimator=2` (ordinary Kalman) during interaction and requests `3`
(the existing PostReleaseVicon15 view) at confirmed release. Set
`estimator_switch_at: detect` to retain the earlier behavior: request `3` at
confirmed interaction onset, including during a position-command detection delay.
This option only applies to S-curve coasting and works with either interaction
command mode and all three detectors. Preparation/hover uses `2`. Both views
share the running Kalman task in the paired firmware; neither transition resets it.
Fresh parameter confirmation is polled without blocking the command stream;
an unconfirmed switch fails after 0.5 s. S-curve dispatch waits for estimator-3
confirmation, then submits one acknowledged firmware release event. The firmware
owns the full curve; offboard sends no low-level position/orientation packets
while it runs. All three detection methods work, and only potentiometer detection
requires the sensor (or enable `record_potentiometer` for diagnostic logging).

The matching firmware endpoint notice and fresh stage-4 status complete the
curve. `pRelHoldG=0` is explicitly written/read back pre-arm: neither the offboard
`stop_speed_m_s` nor a measured terminal-speed window gates this transition.
Fresh firmware state and existing abort/admission checks still apply. Request
the default estimator (`2`), wait for its confirmation while retaining firmware
hold, then send low-level position packets at the exact firmware hold target.
The endpoint target preserves the existing forward-only policy; this option
does not recompute a speed-based stopping projection.

`grace_start: release` retains its release-time origin and can preempt a running
curve with a newly detected interaction after grace. In `release` switch mode,
that new contact requests estimator `2` again; its next release requests `3`.
In `detect` mode, contact continues to request `3`. `speed_threshold` retains
the existing spelling, but in S-curve mode starts grace at curve completion and
default-estimator confirmation. Duration expiry/faults restore the default
estimator and use the existing mission landing lifecycle. Offline tests cover
these transitions; physical timing, stopping quality and repeated flight remain
unvalidated.

### Distance and deceleration inputs for onboard braking

Paired firmware 26092803 exposes these `hlCommander` parameters:

| Firmware parameter | Mission key under `firmware_auto_brake` | Meaning |
| --- | --- | --- |
| `pRelScD` | `stop_distance_m` | Requested distance along the fixed release-velocity direction, in metres; default `0` means free stop. |
| `pRelScA` | `stop_deceleration_m_s2` | Positive reference peak-deceleration cap, in m/s^2; omitted means `3.6`. Currently limited to `3.6` by the existing reference envelope. |

For example, add `stop_distance_m: 0.4` and `stop_deceleration_m_s2: 1.5` to
the existing `firmware_auto_brake` mapping. Both values latch at release.
Distance has priority: the FC solves the existing ramp/seventh-order tail
reference so its velocity integral equals the requested distance and its
terminal velocity is zero. It may use a lower peak than requested. The
configured `tail_s` is preferred; distance/time constraints can shorten it
to no less than 0.15 s. The existing maximum duration remains in force.
An infeasible request is reported, not replaced by a different distance or
a stronger deceleration. No distance uses free stop; omitting both inputs
preserves the old 3.6 m/s^2 reference planner.

Distance mode requires `velocity_scurve`, compensated `attitude` execution,
positive `position_tracking_bandwidth` (the current P/V/A/J setting is `3.0`),
`handoff: curve_endpoint_forward`, and `state_matched_start: false`.
Startup checks `pRelReqVer=1` before enabling the mode, not a firmware build
allowlist. Omission explicitly restores `pRelScA=3.6` on capable firmware.
Version-4 curve events include the requested inputs, actual reference peak,
integrated distance, duration and solver failure reason; older logs still read.

Distance is relative to the FC curve-activation position. Exact reference
area does not guarantee exact physical stopping: delay, initial attitude and
tracking error still matter. The cap is for the reference, not a new hard
limit on corrective acceleration. Existing compensation, actuator bounds,
and forward-only handoff protection are unchanged; an overshoot/reversal can
still cause the hold target to use the current point rather than fly backward
to the reference endpoint. No online replanning is enabled.

### Per-interaction friction for onboard braking

The compensated attitude S-curve can use the kinetic friction selected for the
current interaction (including its randomized 2AFC condition). In the existing
`firmware_auto_brake.analytic_profile` mapping, opt in with:

```yaml
friction_from_interaction: true
```

This requires `mode: scurve`, `shape: velocity_scurve`, `execution: attitude`,
`response_compensation: true`, `rate_feedforward: false`, and
`state_matched_start: false`. Keep the existing tail, handoff, feedback and
calibration settings. `position_tracking_bandwidth: 3.0` selects the paired
P/V/A/J tracker; zero or omission retains velocity/acceleration/jerk tracking.

Use paired firmware exposing `hlCommander.pRelMuVer=1` and `pRelFric` (the
26092802 candidate includes these). Startup checks capabilities rather than a
build-number allowlist. Pulling this repository does not flash firmware or
change the separate mission YAML. The friction opt-in is off when omitted;
older firmware continues to use the legacy release packet.

The release packet atomically carries the selected kinetic coefficient with
the interaction identity. The FC latches it for that release and uses `mu*9.81`
as the requested deceleration bound. It preserves the fixed smooth tail and
existing maximum duration/acceleration envelope; it does not enable repeated
replanning. In 26092803 this bound is also capped by `pRelScA`; the firmware
rejects a duration-infeasible request instead of increasing the cap to stop
sooner. Lower friction generally gives a longer free-stop reference at the
same release speed, but the smooth-tail duration can make different
coefficients produce the same curve. With a requested distance, that distance
takes priority and friction is an additional peak cap. This is not exact
Coulomb motion throughout the smooth tail. Static friction is not used for
this post-release distance calculation.

Version-3 curve events record requested friction, applied deceleration bound,
peak reference deceleration, planned time/distance and limitation flags alongside
the coefficients and computation timing. Readers remain compatible with event
versions 1 and 2. Continuous state-log rates are unchanged.

### Independent frozen contact-detector validation (capture only)

Normal `--interaction --sense` can opt into `Interaction.config.contact_validation_capture`
in the mission YAML. It adds a 100 Hz raw accelerometer/gyro block, receipt/device
timestamps, every valid raw UART potentiometer sample, and the frozen experimental
profile to the log. The primary detector, yaw behavior, force rendering, release
behavior, firmware and existing calibration files are unchanged. The experimental
model is **not executed in the live control loop** and has no command authority.

```yaml
contact_validation_capture:
  enabled: true
  command_authority: false
  profile_path: Interaction/profiles/contact_validation_lb11_20260930.json
  profile_sha256: 7078ab9d78fa80543a26c17aff89b794113fc61ef89b1082482379f054f1a4b7
  initial_no_touch_s: 10.0
```

The packaged profile above is a frozen **lb11-only offline-validation sample**,
not a live flight calibration. A complete mission example is provided in
`Interaction/examples/contact_validation_lb11.yaml`; select it from the
orchestrator instead of the normal level-coast mission to enable this capture.
It ships with Git so a normal Pi pull also obtains its pinned parameters.
Private local profiles (`Interaction/contact_diagnostic_profile.json`),
`wrench_calibration.json` and `attitude_response.json` remain ignored and untouched.
Profiles include frozen coefficients, detector/filter settings, source-data hash,
drone identity and mass. Do not reuse lb11 coefficients for another aircraft.
Missing/changed profiles, missing telemetry and incompatible modes fail before
takeoff. Require `--log`; do not use `--calibrate` for this independent validation.

After the initial contact detector arms, do not touch for the first 10 seconds.
Then perform separated light contacts, waiting for normal rearming between them.
This is an operator protocol, not an automated flight or asserted ground truth.
Potentiometer threshold crossings are a reference proxy; they are not exact
physical contact times. The offline verifier reports coverage, misses and
unmatched onsets, and never treats missing data as zero false positives.

## External RGB camera pose from PixyTile

`camera_node.py` can localize the fixed recording camera before a mission by
detecting only the always-on HyperGrid LEDs in an RGB video frame. It does not
decode MyGrid IDs; the orchestrator turns MyGrid off for this preflight. The
user supplies a rough world position to resolve the
integer-cell ambiguity of the unlabeled lattice; keep rough XY within roughly
half a HyperGrid cell (7.75 cm for the 0.155 m grid) and express it in the same
`world_FLU` frame as the grid file.

Enable `camera_node.pose_estimation` in the swarm manifest and provide paths
that exist on the camera computer:

```yaml
camera_node:
  ip: 192.168.1.90
  user: fls
  pose_estimation:
    enabled: true
    grid_file: /home/fls/fls-marker-localization/high_rate_localizer/config/hypergrid-mygrid-4x4.json
    calibration_file: /home/fls/fls-cf-offboard-controller/config/rgb_camera.json
    rough_position_xyz: [2.336, -0.040, 0.85]
    timeout_s: 20.0
```

Because SFL currently stores camera position but not pitch, `look_at_xyz`
defaults to `[0, 0, camera_z]`, representing the level physical recording
camera. A mission can provide `camera_look_at`, or the pose configuration can
set `look_at_xyz` explicitly, when the authored shot intentionally uses pitch.

The calibration must be for the recording camera at exactly 1920x1080 and the
same fixed focus/crop used for recording. Preflight captures a short MJPEG
burst through the same `rpicam-vid`/`libcamera-vid` pipeline as the mission and
uses its final frame, avoiding a still-mode crop change. Its JSON schema is:

```json
{
  "image_size": [1920, 1080],
  "camera_matrix": [["fx", 0, "cx"], [0, "fy", "cy"], [0, 0, 1]],
  "distortion_coefficients": ["k1", "k2", "p1", "p2", "k3"],
  "capture": {
    "lens_position": 0.5,
    "camera_model": "recording camera model/serial"
  }
}
```

Replace every quoted calibration symbol above with the numeric value produced
by calibration; the loader intentionally rejects placeholders and non-finite
values.

The node rejects a calibration without an image size and fixed lens position,
or one whose lens position differs from the camera command. It accepts a pose
only after three independent captures agree within 2.5 cm and 0.03 rad by
default. Because HyperGrid has no IDs, rough XY is part of the correspondence:
an operator value on the wrong side of a half-cell boundary cannot be repaired
from the image alone. The orchestrator also compares the camera's grid-file
SHA-256 with the grid file loaded by the physical marker-grid controller.

By default, detection searches the bottom 35% of an upright frame. Override
`pose_estimation.detector.roi_normalized` with normalized
`[left, top, right, bottom]` values if the floor grid is framed elsewhere.
The camera sends its world XYZ and viewing yaw to the orchestrator. The
orchestrator computes actual-minus-authored XYZ/yaw offsets before booting the
drones; each controller rotates its SFL targets and waypoints about the
authored camera and then translates the whole swarm.

The last accepted pose can seed a fresh RANSAC fit when drones obscure enough LEDs
to break the complete-rectangle detector. The new capture must still pass all
pose quality gates and the usual three-frame consensus. A rejected capture is
saved on the camera computer as `camera_pose_failed.jpeg` for diagnosis.
Each successful pose also saves `camera_pose_overlay.jpeg`, showing projected
tile outlines and matched LED centers; the orchestrator copies it into the
experiment's `logs/<mission tag>/` folder.
Pose failure or timeout
prevents the drones from booting, and recording is confirmed before the drones
receive `START`. This correction intentionally uses XYZ plus world yaw; keep
the recording camera upright and at the pitch assumed by the authored shot.
Preflight rejects pitch or roll differences above the configured residual
attitude limits because an XYZ+yaw-only swarm transform cannot correct them.
SFL yaw values are radians and remain unwrapped across multi-turn trajectories.
The body-fixed light-module offset follows each drone's corrected yaw, while a
legacy per-drone `position_offset` remains an additional world-axis correction.
