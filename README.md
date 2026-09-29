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

Keep `detection_method: momentum_impulse` for synchronized onboard state logging;
the profile supplies `state_source: onboard`. Select the actual contact detector with
`level_coast.detector: vel`, `model`, or `potentiometer` (`--sense` required).
The model and potentiometer choices reuse the existing contact/release detectors
and saved XYZ detection calibration. Velocity uses XY speed hysteresis and dwell;
a low-speed release is a heuristic, not a separate measurement of hand contact.

After the existing stationary arming gate, contact sends
`send_zdistance_setpoint(0, 0, 0, nominal_z)` continuously. Confirmed release
keeps that same command until `hypot(vx, vy) < level_coast.stop_speed_m_s`
(default `0.03`), or a newly detected interaction preempts coast as described below.
Low speed captures current XY at nominal Z and resets position/velocity integrators.
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

`level_coast.follow_yaw: true` updates position-hold yaw from the current onboard
estimate during preparation, ready, and grace. Disabled or omitted sends absolute
`yaw=0` in those phases, regardless of mission target yaw. Contact/coast always
send zero yaw **rate**, with zero roll/pitch and nominal height. No firmware change
or yaw-contact detector is required. Following waits for the first fresh yaw sample;
it does not substitute a zero heading while awaiting startup state.

`duration` covers the entire repeated loop after observer startup, including
preparation and grace. At expiry in any phase, control returns to the ordinary
mission landing lifecycle. The new mode requires `firmware_auto_brake.enabled:
false`; it does not submit release events or braking curves. Existing configured
PID attitude-source switching is reused at contact and hold capture. State/motor
freshness, measured boundaries, battery and operator-abort checks remain active.
The example is an offline-tested configuration, not flight validation.

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
