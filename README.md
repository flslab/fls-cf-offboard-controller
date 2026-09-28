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
