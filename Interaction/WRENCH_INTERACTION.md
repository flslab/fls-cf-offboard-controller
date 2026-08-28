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

## Potentiometer force control and comparison

Use `--sense --log` (or add `--sense` to an `--interaction --log` run) to
record the spring-backed potentiometer as an independent force reference. The Arduino must emit
`time_ms,raw,filtered,voltage,distance_mm` at 115200 baud on `/dev/serial0`.
The default spring constant is 0.16 N/mm, so the recorded compression force is
`distance_mm * 0.16`.

```text
--sense --sense-axis y --sense-sign 1
```

Change the axis to `y` or `z`, or use `--sense-sign -1`, to match the physical
sensor orientation in the drone body frame. Each sample is rotated into the
world frame using the observer attitude before logging/comparison.
`--sense-port`, `--sense-baud`,
`--sense-spring-constant`, and `--sense-max-age` override the hardware and
freshness defaults.

In a normal onboard momentum-interaction run, the potentiometer controls the
translation interaction while the wrench observer continues running and is
recorded for comparison. Rising spring force starts contact. A sustained force
drop starts bounded attitude braking immediately. The release snapshot records
the measured force, vehicle momentum (`mass * velocity`), and position; the
controller records and holds the actual stopping position only after velocity
along the release direction reaches zero/reverses (or the braking timeout is
reached). Yaw contact continues to use the wrench observer.

Each `wrench_observer` record includes both `external_force_N` (the observer)
and `control_external_force_N` (the potentiometer-derived force), plus the
calibrated distance, force rate, sample freshness, release force/momentum/
position, braking feed-forward acceleration, stopping position, and
observer-minus-sensor error. A stale sensor sample is treated as zero force so
an active contact fails safely into the braking/hold sequence. During the
dedicated calibration run, the sensor remains comparison-only so it cannot
alter the excitation trajectory.
