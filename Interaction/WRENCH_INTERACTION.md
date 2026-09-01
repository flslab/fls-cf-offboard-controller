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

The supplied mission uses `shadow_mode: true`. In this mode the observer,
detectors, and proposed admittance references run and are logged, but the actual
command ignores contact response. It remains at the nominal XYZ/yaw reference
unless the explicitly configured calibration excitation is enabled.

## Model calibration

Run a dedicated contact-free calibration before interaction:

```text
python3 orchestrator/orchestrator.py --calibrate
```

`--calibrate` forces shadow mode, commands a bounded XYZ chirp, identifies an
independent actuator-to-velocity delay, first-order time constant, and
acceleration scale for X/Y/Z, then atomically saves the result on each drone as
`Interaction/wrench_calibration.json` under that controller checkout. Entries
are keyed by drone ID. `--interaction` disables calibration excitation and
automatically loads that drone's saved values. The short stationary bias
calibration still runs at the start of every flight because that bias can vary
between flights.

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

The standard sensor behavior uses the wrench/momentum observer only for engage
detection. During contact, both observer and potentiometer forces are recorded.
Force rendering is disabled by default, so the attitude command remains
`roll=0`, `pitch=0`. When spring compression decreases far and fast enough, the
last force before the decrease and the measured velocity initialize bounded
position coasting. Coasting then runs with zero external-force input under the
configured friction/drag model and holds the final position at the stop-speed
threshold or timeout.

Configure the two behaviors independently under `virtual_object`:

```yaml
force_rendering:
  enabled: false
release_behavior:
  mode: potentiometer_coast
  force_drop_n: 0.01
  decrease_rate_n_s: 0.05
  force_memory_s: 0.02
```

Set `force_rendering.enabled: true` to restore estimator-force inertia and
resistance damping during contact. Set `release_behavior.mode:
observer_brake` to restore the previous observer release-candidate and
counter-tilt braking path. Enabling both restores the previous complete
behavior. `potentiometer_coast` requires `--sense`.

Each `wrench_observer` record includes both `external_force_N` (the observer)
and `control_external_force_N` (the observer control force), plus
`release_braking_external_force_N`, force-rendering state, release mode,
potentiometer force rate/drop, coast initial velocity, calibrated compression
and spring length,
sample freshness, release force/momentum/position, stopping position, and
observer-minus-sensor error. During the dedicated calibration run, the sensor
remains comparison-only so it cannot alter the excitation trajectory.
