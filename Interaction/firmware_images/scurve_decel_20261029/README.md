# S-curve: 40-degree command limit and configurable PID resets

Candidate image: `bolt-scurve-reset-26102902.bin`.
Base: `bolt-eskfxy-attfix-26101201`, with the same Bolt configuration.
This image was flashed to lb11 via Radio Pi 10.182.71.171 on 2026-10-29.
All 375,416 application bytes matched the binary on full flash readback.
Startup, CPU identity, self-test, reset/limit parameters, disarmed state and zero motor outputs were verified.
See `flash-readback.json` and `deployment/` for evidence.
No flight was performed; this is not flight validation.

The local S-curve profile now selects `analytic_profile.max_tilt_deg: 40.0`.
This limits combined tilt, not each roll/pitch axis independently.
The reference deceleration remains **3.6 m/s²**, and the seventh-order tail remains **1.2 s**.
Both new reset switches default to enabled, preserving the existing reset behavior.
The shared PID gains, state estimator, and terminal handoff checks are unchanged.

## PID reset policy

| Firmware parameter | Default | Effect of setting 0 |
| --- | --- | --- |
| `pidCtl.resetMode` | 1 | Skip automatic PID/filter reset when the control mode changes. Applies to the PID controller, including position/attitude transitions. |
| `hlCommander.pRelRstXY` | 1 | Skip explicit XY position/velocity PID resets at S-curve entry and hold handoff, and skip the compensated hold integral seed. Latched at each accepted release. |

To retain XY integral history across the S-curve transitions, set **both** to 0.
The switches do not set Ki to zero or freeze normal integral updates.
Any nonzero value keeps the corresponding reset enabled.
Zero commanded thrust still unconditionally resets the position, velocity, attitude, and rate controllers and their existing filter state.
The hold target, transition conditions, and bumpless output blending remain active.

For the offboard controller, override the following fields before starting the mission:

```yaml
Interaction:
  config:
    wrench_interaction:
      firmware_auto_brake:
        reset_on_mode_change: false
        reset_xy_on_scurve_transition: false
```

Retain the other mission/profile fields.
The checked-in profile keeps both options `true`.
On supporting firmware, omitted options restore both parameters to 1, so an earlier experiment cannot silently carry its disabled policy into the next configured mission.
Disabling either option requires its parameter in the connected firmware; an old image is rejected before configuration writes.
The controller writes and confirms fresh values before enabling automatic release handling, then rechecks them before arming.
If firmware auto-brake setup is disabled, these YAML options are not applied.
The S-curve flag has no effect without an accepted S-curve release; the mode-change flag is global to the PID controller when explicitly disabled.

For a separate cflib client, after connection and parameter download, while the drone is stopped:

```python
cf.param.set_value('pidCtl.resetMode', '0')
cf.param.set_value('hlCommander.pRelRstXY', '0')
```

Wait for fresh replies confirming both values before takeoff.
Use the YAML options when running this repository's mission controller, because its startup configuration would otherwise restore the defaults.
Configure these switches before flight, not during a control transition.
These parameters are now available on the verified lb11 deployment; other aircraft require the matching image.

## Configuration and units

In `Interaction.config.wrench_interaction.firmware_auto_brake`, configure:

```yaml
stop_deceleration_m_s2: 3.6
analytic_profile:
  max_tilt_deg: 40.0
  tail_s: 1.2
```

These are nested overrides; retain the other fields supplied by the `level_coast` profile.
The supported reference range is `0 < stop_deceleration_m_s2 <= 8.23`.
It must also fit within `9.81*tan(max_tilt_deg)`.
The upper value is rounded down from the ideal level-altitude acceleration associated with a 40-degree tilt; it is not a measured deceleration or a guarantee that the reference can be tracked.
`max_tilt_deg` is supported only for `velocity_scurve` with attitude execution and `state_matched_start: false`.

The calibrated response controller uses linear command coordinates, `g*angle(rad)`.
Its norm limit at 40 degrees is approximately **6.849** in those coordinates, so the final angle command cannot exceed 40 degrees.
The uncompensated acceleration-to-attitude path uses `g*tan(angle)` and a vector norm bound of approximately **8.232 m/s²**.
Both paths preserve command direction and respect the same geometric tilt ceiling.
The calibrated model is otherwise unchanged; its accuracy and actual thrust/attitude tracking at larger angles need separate flight validation.

## Firmware contract

- `hlCommander.pRelTilt`: selected angle limit, latched with the accepted release event.
- `hlCommander.pRelTMax`: read-only capability, `40.0` degrees.
- `hlCommander.pRelAMax`: read-only reference capability, `8.23` m/s².
- `hlCommander.pRelScA`: selected nominal reference deceleration.

The firmware boot default angle remains approximately 22.183 degrees.
The local 40-degree mission explicitly writes and confirms its selected limit before enabling braking.
A mission omitting the angle option resets the limit to the legacy value on firmware exposing the new parameter.
The host rejects the 40-degree configuration on older firmware before enabling braking, checks fresh capability replies, and rechecks the limit before arming.
Changing the parameter after a release does not alter the envelope of that active S-curve.

## Validation and reproduction

`build-manifest.json` records the actual image/configuration/source fingerprints and validation status.
`patches/candidate.patch` includes the angle-limit and reset-policy changes and applies to both paired baseline source trees identified in `patches/source.json`.
Historical image packages and baseline source snapshots are preserved.

Build the patched Bolt tree using its preserved configuration:

```sh
make -j4 KBUILD_OUTPUT="$PWD/build-unified" O="$PWD/build-unified"
```

Run the native planner and angle-bound checks against a patched source tree:

```sh
clang -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -I /path/to/patched/firmware/src/modules/interface/kalman_core \
  test_deceleration.c -o /tmp/test_scurve_limits
/tmp/test_scurve_limits
```

The native checks cover 240 free/distance-constrained reference cases, invalid requests, low-speed tail-limited behavior, and 540 heading/angle combinations through both command-coordinate bounds.
These are algorithm and command-envelope checks, not a physical flight simulation.

Run the reset call-path checks with:

```sh
python3 validate_reset_policy.py /path/to/patched/firmware
```

This compiles the actual mode-change/reset call sites with counter stubs under ASan/UBSan.
It checks XY versus full reset, mode tracking while resets are disabled, feedforward-only mode normalization, per-release latching, entry/hold resets, integral-seed suppression, legacy S-curve handoffs, and unconditional zero-thrust reset under all four switch combinations.
The 222 offboard tests include independent switch configuration, old-firmware rejection, default restoration, wrong readback rejection, and pre-arm rechecks.
