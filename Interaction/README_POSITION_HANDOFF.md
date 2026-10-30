# Position capture after PB, OB and S-curve coasting

`level_coast` uses one velocity-compensated position capture and hover controller
after all three coasting methods.
This is a local implementation with offline tests; it has not been flight validated.
It addresses estimator-dependent braking at the transition to position control.
It does not guarantee elimination of flyback caused by attitude lag or commands
already applied before the handoff.

## Transition and physical anchor

- PB retains its position-follow coast ramp and full-XY speed threshold.
- OB retains level-attitude coasting and the signed interaction-direction speed threshold.
- S-curve retains firmware braking/unwind and the acknowledged endpoint completion condition.

After that condition, all three enter `capture` and send position packets.
The first packet preserves the outgoing nominal roll/pitch command.
PB supplies its previous calculated command; OB supplies its explicitly level
command; S-curve requires fresh `controller.roll/pitch` telemetry in the
`POSITION_HANDOFF` group at 50 Hz.
A smooth blend and a jerk bound bring the nominal acceleration toward damping
the measured Vicon XY velocity.
Capture requires continuous low speed, tilt, angular rate and acceleration.
The physical hover anchor is captured from Vicon only after this settling gate.
During hover, an outer position/velocity correction retains this anchor while
the sent position packet remains responsive to the FC state estimate.
The anchor and the packet target are intentionally different when FC velocity
differs from Vicon velocity.

For S-curve, offboard sends no low-level packets during the firmware curve.
After confirmed completion, the compensated position stream begins while
estimator 3 is still selected; estimator 2 is requested after the first packet.
Capture cannot complete until the acknowledgement and a further fresh-state
interval have been observed.
This is not an atomic firmware change: state telemetry and command transport
delay can still produce a short mismatch during the switch.

`grace_start: release` retains its release-time origin and permits re-contact
during capture once the configured grace expires.
New contact clears the old capture state and anchor.
`grace_start: speed_threshold` starts grace after capture completes and retains
its stationary re-arming rule, using fresh Vicon motion after compensated hover
so FC velocity bias cannot keep a stationary LightBender permanently disarmed.
`coasting_enabled: false` retains direct hold and bypasses this capture controller.
This takes precedence even when `position_handoff.enabled: true`: no capture,
new hover compensation, handoff telemetry, or handoff-only PID preparation is enabled.

## Position packet construction

Let `B` rotate world XY into the current body-yaw frame, `p_fc` and `v_fc` be
the FC state, and `p_m` and `v_m` be the time-aligned Vicon position/KF velocity.
The confirmed position and velocity P gains are `Kp` and `Kv`.
Define `D = g * radians(Kv)` componentwise.

Before the anchor is captured, desired equivalent acceleration is
`a_des = B.T * D * (-B * v_m)`.
After capture, it is
`a_des = B.T * D * (Kp * B * (anchor - p_m) - B * v_m)`.
The acceleration envelope, initial blend and jerk limit produce `a_cmd`.
The sent XY position is then

```text
v_target_body = B * v_fc + (B * a_cmd) / D
p_packet_xy = p_fc_xy + B.T * (v_target_body / Kp)
```

In the nominal P-only cascade this requests `a_cmd` independently of FC velocity
bias; it does not blindly integrate velocity into a distant position target.
Compensation continues after settling so a changing velocity bias cannot leave
a stale offset baked into the final hover location.
The model uses the small-angle acceleration relation and does not invert
radio delay, FC filters, attitude dynamics or estimator attitude discontinuities.

The existing grounded `PositionFollowPidContext` is now required for pure OB
and orientation-push/S-curve missions as well as PB missions.
It saves, confirms and temporarily zeros XY I/D/feedforward gains before takeoff;
P gains, Z and attitude/rate gains are retained.
The original gains are restored through the existing grounded recovery lifecycle.
Normal XY integral disturbance rejection is unavailable during these runs.

## Configuration and guards

All values below are configurable in `Interaction.config.level_coast.position_handoff`.
Omission uses these defaults.

| Key | Default | Purpose |
|---|---:|---|
| `enabled` | `true` | Common capture/compensated hover for all coasting modes |
| `transition_s` | 0.25 s | Quintic blend from outgoing acceleration |
| `max_jerk_m_s3` | 2.0 m/s³ | Nominal acceleration slew limit |
| `settle_speed_m_s` | 0.02 m/s | Full XY Vicon speed gate |
| `settle_tilt_deg` | 0.5° | Roll/pitch vector norm gate |
| `settle_rate_deg_s` | 5°/s | Roll/pitch rate norm gate |
| `settle_acceleration_m_s2` | 0.35 m/s² | Filtered measured and commanded acceleration gates |
| `settle_dwell_s` | 0.10 s | Continuous settling evidence |
| `acceleration_filter_s` | 0.08 s | Causal acceleration filter time constant |
| `timeout_s` | 3.0 s | Legacy compatibility only; ignored for capture timing |
| `max_sample_gap_s` | 0.10 s | State continuity bound; also post-ACK observation interval |
| `max_vicon_skew_s` | 0.035 s | Vicon/state pairing ceiling; cannot be widened |
| `max_command_age_s` | 0.05 s | Command telemetry and capture/hover send freshness ceiling; cannot be widened |

The existing `position_control.max_brake_acceleration_m_s2` (0.8 m/s²) and
`position_control.max_offset_m` (0.6 m) also constrain capture.
These are nominal command limits, not measured acceleration guarantees.
FC velocity limits, boundaries, battery/motor checks and operator abort remain active.
Duplicate Vicon frames cannot advance settling evidence; slower fresh Vicon
streams can still accumulate a continuous dwell.
Duplicate FC states may refresh a packet only within the command freshness bound.
Capture has no settling timeout; it continues until settling or another mission transition.
Missing/stale state or command data and infeasible offsets abort through
the existing mission cleanup/landing path; they do not silently freeze a target.

Set `position_handoff.enabled: false` only to select the explicit legacy
comparison path: projected fixed PB target, current-position OB hold, and
firmware endpoint S-curve hold after estimator restoration.

## Logs and offline verification

`Level Coast Position Capture Started/Completed` mark the handoff.
The per-sample `position_handoff_*` fields record physical anchor, sent target,
FC-minus-Vicon velocity, nominal and measured acceleration, Vicon timestamp,
settling permission and completion.
`capture_preempted` identifies contact during this stage.
Separate `coast`, `capture` and settled hover when analyzing flyback.

The tests reconstruct the FC cascade independently and cover estimator
position/velocity jumps, all detector/push/coast combinations, re-contact,
freshness faults, boundaries, capture beyond the former timeout and pre-arm requirements.
Run them with:

```sh
venv/bin/python -m unittest Interaction.tests.test_position_handoff Interaction.tests.test_level_coast Interaction.tests.test_level_coast_scurve Interaction.tests.test_position_follow Interaction.tests.test_specific_force_runtime
venv/bin/python -m Interaction.simulate_position_handoff
```

The diagnostic simulation uses a first-order acceleration response with finite
command delay and a 55 mm/s FC velocity-bias step.
It is not replay of a repaired flight and does not model estimator attitude
jumps, timestamped telemetry transport, wind, contact, or radio faults.
Its reverse displacement and terminal velocity must both be inspected.
Residual reversal in a delayed case is a limitation, not evidence of a successful
zero-flyback controller or permission to deploy.

The 27-case sweep (40/80/150 ms response constants, 0/20/50 ms command delays,
10/30/100 mm/s initial speeds) completed capture in every case.
Maximum reverse displacement was 7.68 mm; maximum terminal speed at 4 s was
0.36 mm/s.
This demonstrates settling in that model, not zero reverse motion.
