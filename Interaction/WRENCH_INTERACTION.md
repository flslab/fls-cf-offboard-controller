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
3. A stationary startup interval estimates the residual model bias. Do not
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

Before setting `shadow_mode: false`:

1. Fly several untouched hover trials.
2. In a clear flight volume, enable `calibration_excitation` for a separate
   shadow trial and do not touch the drone. It commands small bounded XYZ/yaw
   sine motions so the applied-input model can be identified and motion-only
   false positives can be measured.
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
