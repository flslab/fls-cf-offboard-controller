# Model-based human interaction

The active `wrench_interaction` path keeps the Crazyflie position PID in charge
of flight. It changes only the position/yaw reference sent to that PID.

## Runtime pipeline

1. Rigid-body Vicon supplies position and quaternion. Motor PWM and battery
   voltage supply the applied-input estimate. Host timestamps pair each pose
   with the nearest motor packet and reject excessive time skew.
2. Two augmented Kalman observers estimate unmodelled linear and angular
   acceleration. The observer converts these to external force and torque with
   the configured vehicle mass and inertia.
3. A stationary startup interval estimates the per-flight residual bias. Do not
   touch the vehicle between the `Wrench Calibration Started` and
   `Wrench Calibration Complete` events.
4. Separate uncertainty-aware detectors require a physical wrench threshold,
   covariance-normalized confidence, and persistent evidence.
5. A bounded virtual mass/damper/spring converts active contact into a reference
   offset. The existing position PID tracks that reference.

The channel routing is fixed in code:

| Estimated channel | Detection | Reference response |
| --- | --- | --- |
| Force X/Y/Z | yes | position X/Y/Z |
| Yaw torque | yes | yaw |
| Roll/pitch torque | yes | none (log only) |

## Position/orientation rendering policy

For onboard momentum interaction, `virtual_object.inertia_command` is the
priority used when the configured virtual object accelerates no faster than
the native vehicle. The comparison projects the native acceleration
`F_external / current_mass` and the virtual acceleration
`(F_external - F_friction - F_drag) / virtual_mass` onto the interaction
direction at contact onset.

- Faster virtual response always selects position rendering for visual motion.
- Slower/equal response selects the configured `position` or `orientation`
  priority.
- Position rendering integrates the virtual dynamics and continues a bounded
  zero-input friction/drag coast after release.
- Position coast and orientation braking use the same stop-speed, timeout,
  position-hold, and detector-rearm conditions.

Static friction may be configured separately. Below
`friction_min_speed_m_s`, it balances the applied force up to
`static_friction_coefficient * mass * g`; after breakaway, the kinetic
coefficient is used.

## Required localization

The pipeline requires `--vicon-mode rigidbody --vicon-full-pose`. The
LightBender orchestrator adds these arguments when the mission contains
`wrench_interaction`; the drone manifest must provide `label` or `obj_name`.
Point-cloud mode has no orientation and is rejected.

## Safe rollout

The active `translation_inertia.yaml` experiment currently uses
`shadow_mode: false` with force rendering enabled, so it can command roll/pitch.
For the first rollout after any calibration or control change, explicitly set
`shadow_mode: true`. In shadow mode the observer, detectors, and proposed
admittance references run and are logged, but the actual command ignores contact
response and remains at the nominal XYZ/yaw reference unless the explicitly
configured calibration excitation is enabled.

## Model calibration

Run a dedicated contact-free calibration before interaction:

```text
python3 orchestrator/orchestrator.py --calibrate
```

The orchestrator always enables controller logging. If you invoke
`controller.py` directly instead, `--calibrate` must be paired with `--log`
and `--smooth-controller-rate 50` or higher; the active orchestrator uses 100
Hz. The mission may provide a calibration-only
`wrench_interaction.calibration_nominal_position`, so the clear-volume
calibration point does not change the apparatus-aligned interaction target.
`--calibrate` runs two contact-free stages. First it forces
shadow mode and commands the bounded XYZ position chirp used by the wrench
observer. It then performs bounded planar attitude trials with the sequence
level -> accelerate -> level -> equal-duration opposite brake -> level ->
position recovery.  The current Y-mounted experiment fixes tilt at 20 degrees and tests paired
acceleration/braking durations of 0.16, 0.24 and 0.32 seconds in both +Y and -Y
(six trials total). Each trial levels for 0.20 seconds, accelerates for T,
levels for 0.20 seconds, brakes at the opposite 20 degrees for T, levels for
0.65 seconds, then recovers with position control for 2 seconds.
Equal opposite command pulses cancel velocity only in an ideal symmetric
model; delay, drag and initial velocity can produce residual motion or reversal.
Zero tilt is not XY position hold. Inspect measured velocity after braking.
The second fit identifies attitude-command delay, first-order response time,
and planar acceleration scale. Strict gates require enough windows in every
trial, acceptable training and validation R-squared and normalized RMSE in
each direction, bounded acceleration gain, consistent gains across pulse
durations, and agreement between +Y/-Y at every individual level. Poor or
incomplete trials fail without overwriting a usable calibration. The fits and
the exact trial protocol are atomically saved per drone in the same
`Interaction/wrench_calibration.json`.

Before each planar trial, the existing position-recovery interval is followed
by a bounded **nominal-position hold**, not an extended zero-tilt command.
The next trial starts only after XY speed is at most 0.05 m/s, actual tilt is
at most 4 degrees and XY error from nominal is at most 0.08 m continuously for
0.30 seconds. Up to 5 seconds of extra hold is allowed per trial. Duplicate polls do not add
evidence; stale samples or gaps over 0.10 seconds reset the dwell. Wall-time
timeout and flight safety limits stay active. Intentional waits pause the
shared protocol clock, so they do not consume later maneuvers. Open-loop phase
durations are never extended to wait for readiness. These waits add to the
nominal flight duration; the log records each wait and admission.

The position-capture experiment is retired and is not enabled even by old
mission settings. Historical capture logs and stored evidence remain readable;
no capture quality gate participates in the new live calibration. Position
recovery between attitude trials remains, and is not a capture experiment.

Active `potentiometer_coast` refuses to run without a current-schema planar fit
that passed those gates. Runtime braking acceleration is limited to the smaller
of `coast_max_acceleration_m_s2` and the largest fitted calibration-step
acceleration times `maximum_acceleration_extrapolation_ratio`, with an absolute
5 m/s^2 ceiling, so runtime control stays near the acceleration envelope
exercised by calibration. `--interaction` does not run the calibration trials and
automatically loads the saved values.

Before setting `shadow_mode: false`:

1. Fly several untouched hover trials.
2. In a clear flight volume, run `--calibrate` and do not touch the drone.
3. Fit all three `motor_model.angular_accel_scale` values from motor-mixer
   differential versus measured angular acceleration. Active mode refuses a
   missing/zero scale on any rotational axis.
4. Set the force/torque covariance floors and physical thresholds above the
   untouched-motion residual distribution, then confirm deliberate contacts
   cross them reliably.
5. Start with small `max_offset`, `max_velocity`, `max_acceleration`,
   `max_yaw_offset`, and `max_yaw_rate`, with a spotter and unobstructed volume.

Analyze each shadow flight from the orchestrator repository with:

```text
python3 orchestrator/analyze_interaction_logs.py orchestrator/logs/SESSION
```

The report includes quiet p95/p99 force and torque residuals for threshold
tuning and an `angular_motor_scale_fit`. Treat that fit as usable only after
each axis has deliberate, contact-free excitation, a non-trivial mixer-input
spread, and strong fit quality; otherwise collect a better shadow trial.

Every full-pose frame produces a `wrench_observer` log record containing raw and
bias-corrected wrench, covariance, innovation/NIS, contact states, proposed
reference, actual command, frame age, and motor-data age. Stale mocap or stale
motor telemetry terminates the interaction loop and sends the normal setpoint
stop notification; there is no attitude-recovery escalation.

## Potentiometer release and configurable force rendering

Use `--sense --log` (or add `--sense` to an `--interaction --log` run) to
record the spring-backed potentiometer as an independent force reference. The Arduino must emit
`time_ms,raw,filtered,voltage,compression_mm,supply_voltage` at 115200 baud on
`/dev/serial0`. Five-column firmware remains readable, but its Arduino supply
voltage is logged as unavailable.
`compression_mm` is the calibrated spring compression. With the default
10.4 mm travel and spring constant of 0.16 N/mm, the current spring length and
force are `max(10.4 - compression_mm, 0)` and `compression_mm * 0.16`.
Firmware carrying the old `distance_mm` header is positionally compatible; its
fifth column is interpreted as compression.

```text
--sense --sense-axis y --sense-sign 1
```

Change the axis to `y` or `z`, or use `--sense-sign -1`, to match the physical
sensor orientation in the drone body frame. Each sample is rotated into the
world frame using the observer attitude before logging/comparison.
`--sense-port`, `--sense-baud`,
`--sense-spring-constant`, `--sense-max-extension`, and `--sense-max-age`
override the hardware, geometry, and freshness defaults.

While `--sense` is active, a background monitor records Raspberry Pi
`vcgencmd get_throttled` flags every 0.5 seconds. Wrench records therefore
contain both `force_sensor_supply_voltage_V` from the Nano and current/latched
RPi undervoltage, frequency-cap, throttling, and soft-temperature-limit flags.
Use `--sense-power-poll-interval` to change the RPi polling interval.

The sensor can own both engage and release detection. The contact detector must
first observe a low-force baseline, then sustained compression above its onset
threshold. During contact, both observer and potentiometer forces are recorded.
Force rendering is disabled by default, so the attitude command remains
`roll=0`, `pitch=0`. When it is enabled and a fast local force decrease starts a
release candidate, the attitude path immediately removes render tilt. It rolls
the calibrated response forward and may issue one finite, bounded pulse opposite
the predicted residual tail in either direction; this sign-symmetric
cancellation prevents an old counter-tilt or its compensating tail from causing
reversal. The candidate cap is lower than confirmed-coasting acceleration
because the decision is reversible. Contact remains active until force stays
below the unloaded threshold for the configured dwell; a force recovery or lack
of continued unloading cancels the candidate and resumes live rendering. A
brief sensor gap keeps the candidate level/tail-aware and pauses confirmation;
recovered samples must start a fresh unloaded dwell rather than receiving credit
for the gap. If the sensor remains stale for
`candidate_sensor_stale_timeout_s`, the candidate sends one safe level or
latched-position command and then enters the normal landing error path, instead
of leaving candidate attitude control active indefinitely.

Confirmed coasting rolls the calibrated command delay, attitude time constant,
actual tilt, and already-sent command queue forward on every fresh state. It
levels early when the queued braking impulse is sufficient and caps each finite
command-period impulse. Its tail cancellation is sign-symmetric: it opposes
either a predicted braking-tail reversal or a queued forward rebound, then
expires and is recomputed from the next state. It never accelerates to chase
position. If the frozen virtual stop is already passed, it damps velocity
without pulling back.
Position control receives ownership only after longitudinal speed, bounded
transverse speed, actual tilt, and measured acceleration are settled *and* the
last non-level command has cleared the delay queue plus four fitted time
constants. It retains the release-time virtual stop only along the interaction
axis and latches the measured perpendicular coordinate. If the stop is already
behind the vehicle, it latches the complete measured position. A timeout is
diagnostic and cannot force an out-of-envelope vehicle into position control.

Configure the two behaviors independently under `virtual_object`:

```yaml
force_rendering:
  enabled: false
contact_detection:
  source: potentiometer
  force_threshold_n: 0.08
  onset_dwell_s: 0.03
release_behavior:
  mode: potentiometer_coast
  force_drop_n: 0.04
  candidate_lead_drop_n: 0.005
  decrease_rate_n_s: 0.05
  unloaded_force_n: 0.05
  unloaded_dwell_s: 0.05
  max_sample_gap_s: 0.15
  candidate_sensor_stale_timeout_s: 0.25
  candidate_stall_timeout_s: 0.15
  force_memory_s: 0.02
```

Set `force_rendering.enabled: true` to restore estimator-force inertia and
resistance damping during contact. Set `contact_detection.source:
wrench_observer` and `release_behavior.mode: observer_brake` to restore the
previous observer-owned contact/release and counter-tilt braking path.
`potentiometer` contact detection and `potentiometer_coast` require `--sense`.
Active `potentiometer_coast` also requires `inertia_command: orientation` and
keeps dynamic render selection on that observable attitude-command path; a
position-rendered PID tail is not covered by the planar braking fit. The active
force-render tilt is capped to the largest saved calibration trial tilt even
when `max_attitude_deg` requests a larger value.

The current fit covers one opposed planar axis. Coasting therefore commands no
uncalibrated transverse attitude. Once longitudinal speed, tilt, acceleration,
and the delayed command queue are settled, bounded lateral drift may hand off
to native position control. Its perpendicular target is latched at the measured
position, while only the still-ahead longitudinal virtual stop is retained; this
prevents a stale lateral target from pulling the vehicle sideways or backward.

Each `wrench_observer` record includes both `external_force_N` (the observer)
and `control_external_force_N` (the observer control force), plus
`release_braking_external_force_N`, force-rendering state, release mode,
potentiometer force rate/drop, release-candidate/unloaded-dwell state, coast
initial velocity, calibrated compression and spring length, comparison-only
virtual-state errors, braking action/applied acceleration and dissipated power,
sample freshness, release force/momentum/position, stopping position, and
observer-minus-sensor error. During the dedicated calibration run, the sensor
remains comparison-only so it cannot alter the excitation trajectory.
