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
    filter: force_rate_kf
    force_rate_kf:
      tau_s: 0.010
      process_noise_scale: 0.1
      minimum_valid_span_s: 0.020
    max_dt_s: 0.05
    max_pair_skew_s: 0.009
    max_sample_age_s: 0.1
    accel_scale: [1.0, 1.0, 1.0]
    accel_bias_m_s2: [0.0, 0.0, 0.0]
    imu_from_com_m: [0.0, 0.0, 0.0]
    thrust_scale: 1.0
```

The shared profile supplies onboard state, motor model, startup stationary bias
calibration and contact thresholds. An inline-only mission must
supply its usual `state_source: onboard` and `shadow_mode: false` settings.

The default specific-force filter is the paper's horizontal force-rate KF.
The shared profile uses 0.12 N for each horizontal push threshold. KID uses
`r_F = norm(force_xy / thresholds_xy)` and accumulates `dt * max(r_F - 1, 0.25)`
when `r_F >= 1`; otherwise evidence decreases by `dt`, down to zero. Push starts
at 0.05 s of evidence. There is no confidence gate or additional onset dwell.
At push onset the normalized horizontal force locks the interaction direction.
Release occurs on the first new valid filtered sample whose signed projection
is at or below `0.55 / norm(direction_xy / thresholds_xy)`: 0.066 N for the
default thresholds. There is no release dwell. The previous confidence,
minimum-onset-duration and release-time settings are ignored in KF mode.
Thresholds, onset evidence and release ratio remain configurable.

`filter: window` explicitly selects the archived averaging/detection behavior;
only that mode uses `window_s`, `minimum_window_s`, confidence gating and
release dwell. Old mission files containing window lengths still load, but
those lengths do not add averaging to the default KF path.
The selector is supported only by the `level_coast` model path. With
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

The instantaneous body-force balance is rotated using the current onboard
attitude and the fixed startup bias is subtracted before filtering. The KF
state is `[Fx, Fy, tau_s*dFx/dt, tau_s*dFy/dt]`, with both blocks in newtons.
For `h = dt/tau_s`, it uses `A = [[I, h*I], [0, I]]`, `H = [I, 0]`, and
`Q = q * [[h^3*R/3, h^2*R/2], [h^2*R/2, h*R]]`. The first valid observation
initializes force, zero rate, and covariance `diag(R, R)`.

Startup hovering collects instantaneous force samples (by default at least
30 samples over 1 s); their component-wise median fixes the force bias. Half
the sample covariance of adjacent horizontal force differences gives `R`;
only pairs with positive intervals at most 30 ms are used. Off-diagonal entries
are multiplied by 0.95 and eigenvalues are floored at 1e-8 N². `R` is frozen
for interaction. When startup calibration is explicitly disabled, the supplied
`force_rate_kf.measurement_covariance` is used (default `diag(0.020², 0.020²)` N²).
The 20 ms valid-span requirement applies only at initialization/recovery; it
is not a sliding window or a release confirmation interval. KF history survives
ordinary interaction rearming. Vertical force remains an instantaneous diagnostic;
KID decisions use the horizontal components only. This path is independent of
KF velocity changes but depends on attitude and physical thrust/IMU calibration.

## Missing data and diagnostics

State/motor and IMU pairing uses the firmware's wrapping 24-bit log clock.
The KF and model detection advance on the unwrapped IMU clock.
Repeated IMU samples never advance detection. An incomplete pair briefly waits
while refreshing the current command; sustained missing/stale data enters the
existing abort/cleanup path. Invalid samples or gaps exceeding `max_dt_s`
(default 50 ms) reset KF history and onset evidence while preserving active
contact and its locked direction: a zero or
unready estimate cannot signal release.

`wrench_observer` records `force_estimator`, `force_estimator_ready`, `force_bias_N`,
`imu_cf_timestamp_ms`, `imu_sample_time_s`, `imu_state_skew_s` and `imu_age_s`.
KF records additionally include `force_filter`, bias-corrected `force_observation_N`,
`force_rate_xy_N_s`, `force_filter_covariance_N2`,
`force_measurement_covariance_N2`, and `force_filter_valid_span_s`.
Raw `MODEL_IMU` packets and the resolved configuration are saved for a complete
offline replay. `state_time` remains the host timestamp of the paired state;
for specific-force detector timing, replay on `imu_sample_time_s`.

The Oct 12 08:56:33 historical log did not record these raw IMU channels. Its
compressed-acceleration proxy was only a diagnostic; that proxy is not used here.
