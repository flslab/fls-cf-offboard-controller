# Offboard specific-force model detector

`level_coast` supports two external-force estimators under `detection_method: model`:
`momentum` (the existing default) and `specific_force` (the opt-in IMU candidate).
The contact/release state machine, configured estimator switching, and firmware
S-curve handoff remain shared. This integration has software tests; it is not
hardware flight validation.

## Select it in an existing mission

`Interaction/examples/specific_force_lb11.yaml` is a complete selectable example
based on the latest logged lb11 mission, including detection-time estimator
switching and S-curve. It does not change the currently selected mission.

Add these overrides to the existing `Interaction.config` mapping. Retain the
mission's existing drone, boundaries, command modes and estimator-switch edge.

```yaml
behavior: level_coast
detection_method: model
wrench_interaction_profile: level_coast
wrench_interaction:
  force_estimator: specific_force  # momentum selects the original method
  specific_force:
    window_s: 0.08
    minimum_window_s: 0.05
    max_dt_s: 0.05
    max_pair_skew_s: 0.009
    max_sample_age_s: 0.1
    accel_scale: [1.0, 1.0, 1.0]
    accel_bias_m_s2: [0.0, 0.0, 0.0]
    imu_from_com_m: [0.0, 0.0, 0.0]
    thrust_scale: 1.0
```

The shared profile supplies onboard state, motor model, startup stationary bias
calibration, contact thresholds and release dwell. An inline-only mission must
supply its usual `state_source: onboard` and `shadow_mode: false` settings.

The existing release threshold and dwell remain configuration-driven (the
current level-coast profile uses 0.044 N along the locked direction and 150 ms).
The new selector is supported only by the `level_coast` model path. With
potentiometer or velocity detection, `force_estimator` and `specific_force`
settings are ignored: no `MODEL_IMU` stream or raw-IMU readiness check is added,
and the existing momentum computation remains diagnostic. With model detection,
selecting `specific_force` for another behavior or without onboard state fails
during log configuration.

## Hold position directly after interaction

Set this beside the estimator-switch option in `Interaction.config.level_coast`:

```yaml
estimator_switch_at: detect
coasting_enabled: false  # omitted/true preserves the configured coasting behavior
```

With coasting disabled, `model` and `potentiometer` capture the current onboard
XY position on their **confirmed release** sample and send a fixed position
setpoint immediately, retaining the mission height and existing yaw policy.
`vel` does the same when onboard horizontal speed stays below
`level_coast.velocity.release_speed_m_s` for `release_dwell_s`.
For example, `0.08` and `0.05` mean below 0.08 m/s for 50 ms. The separate
`stop_speed_m_s` coast threshold is not used in this mode.

The contact command mode is unchanged. No coasting command or firmware S-curve
release event is sent. The hold target stays fixed instead of following the
moving vehicle or returning to the original takeoff target. Grace starts at this
hold transition for either grace policy; the existing rearming rules then apply.

For the S-curve estimator configuration, `estimator_switch_at: detect` still
requests estimator 3 at contact and requests estimator 2 when entering hold.
Position setpoints start immediately while that restoration is acknowledged.
With `estimator_switch_at: release`, the run remains on estimator 2 because
there is no S-curve to execute. Keep the existing S-curve profile, firmware setup,
hover-calibration and diagnostic settings; the switch skips execution, without
changing their preflight configuration.

## Measurements and calibration

The mission automatically adds the `MODEL_IMU` block at 10 ms: `acc.x/y/z` and
`gyro.x/y/z`, six floats (24 bytes). These are the firmware sensor log channels,
not `stateEstimateZ.ax/ay/az`. Ground verification checks fields, freshness and
device-time pairing. Logging remains active when the estimator-3 hover
initialization reuses its cache and does not open `ESTIMATOR_XY_IMU`.

Acceleration is converted from g to m/s²; gyro from degrees/s to radians/s.
The correction is `corrected_acc = accel_scale * raw_acc - accel_bias_m_s2` in
body axes. These coefficients are explicit and frozen for the observation;
they are not implicitly copied from a cached estimator-3 calibration.

`imu_from_com_m` is the body-frame vector from center of mass to IMU, in meters.
Zero disables lever-arm correction. With a measured nonzero vector the observer
subtracts `alpha × r + omega × (omega × r)`, using raw body gyro. Do not enter the
Gazebo-fitted offset as a hardware measurement.

Thrust uses the existing mission motor model (PWM, voltage and hover reference),
optionally multiplied by `thrust_scale`. The Gazebo linear PWM curve is not
installed. The old momentum observer's velocity-alignment gain/delay calibration
does not apply to this IMU path. Per-flight stationary external-force bias is
still estimated by the existing calibration stage unless explicitly disabled.

The body-force balance is rotated using the current onboard attitude and
averaged over the causal 80 ms window. It is independent of KF velocity changes,
but still depends on attitude and physical thrust/IMU calibration.

## Missing data and diagnostics

State/motor and IMU pairing uses the firmware's wrapping 24-bit log clock.
The force window and model detection/dwell advance on the unwrapped IMU clock.
Repeated IMU samples never advance detection. An incomplete pair briefly waits
while refreshing the current command; sustained missing/stale data enters the
existing abort/cleanup path. A sample gap exceeding `max_dt_s` restarts the force
window and clears timing evidence while preserving active contact: a zero or
unready estimate cannot signal release.

`wrench_observer` records `force_estimator`, `force_estimator_ready`, `force_bias_N`,
`imu_cf_timestamp_ms`, `imu_sample_time_s`, `imu_state_skew_s` and `imu_age_s`.
Raw `MODEL_IMU` packets and the resolved configuration are saved for a complete
offline replay. `state_time` remains the host timestamp of the paired state;
for specific-force detector timing, replay on `imu_sample_time_s`.

The Oct 12 08:56:33 historical log did not record these raw IMU channels. Its
compressed-acceleration proxy was only a diagnostic; that proxy is not used here.
