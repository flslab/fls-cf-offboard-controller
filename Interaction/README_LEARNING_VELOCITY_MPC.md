# Legacy learned-model velocity pulse MPC

`learning_velocity_mpc.py` is the earlier learned-model pulse selector for one
planar velocity transition. It is not the Learning MPC method in *Learning
Model Predictive Control for Quadrotors* (Li, Tunchez, and Loianno, 2022): it
does not learn a sampled safe set or a trajectory cost-to-go across releases.

The paper-structured, varying-release-speed implementation is documented in
`README_CONDITIONAL_VELOCITY_LMPC.md`. Keep this older controller in shadow
while that offline LMPC is seeded and validated.

## Objective

Given a fixed world-XY direction and a target projected velocity, select the
fastest model-feasible acceleration pulse and begin leveling early enough that:

- projected velocity reaches the requested value within `0.05 m/s`;
- projected roll/pitch is within `3 deg`;
- projected angular rate is within `20 deg/s`;
- the constraints remain true for an `0.08 s` dwell;
- target-speed overshoot stays within `0.02 m/s`;
- commanded attitude changes no faster than `180 deg/s`.

The optimizer searches maximum-tilt then level switching times. When the
terminal set lies beyond the finite horizon, it takes the maximum hard-safe
progress step and replans from the next measurement. A bounded causal learner
can add repeated measured-minus-modelled acceleration residuals to later
forecasts.

## Safety boundary

The optimizer remains device-independent and has no `cflib` imports or device
I/O. Its no-overshoot result is conditional on the supplied frozen model,
configured uncertainty margin, fresh synchronized state, and available
actuator authority. The interaction adapter is responsible for command
authorization, deadline enforcement, measured terminal-state dwell, and the
legacy-controller fallback.

Only call `record_sent_command()` after the corresponding command was actually
sent. History must cover at least the identified attitude-command delay;
missing history fails closed without manufacturing a fallback attitude command.

Run the synthetic tests with:

```bash
venv/bin/python -m unittest Interaction.tests.test_learning_velocity_mpc
```

## Real-state shadow and online integration

The normal interaction entry starts the adapter only after confirmed
potentiometer release. It loads the saved directional `prediction_model` and
consumes live Crazyflie state plus actually sent attitude commands. With only
`enabled: true`, its roll/pitch remain log-only. Online use additionally needs
`command_authority: true`, a zero-velocity target, independently validated
model evidence, and exclusive ownership (`predictive_braking.enabled: false`).

- `Learning Velocity MPC Shadow Started`
- `Learning Velocity MPC Shadow Decision`
- `Learning Velocity MPC Shadow Stopped` or `... Unavailable`
- `Learning Velocity MPC Online Started`
- `Learning Velocity MPC Online Decision`
- `Learning Velocity MPC Online Fallback` or `... Position Handoff`

An online decision exceeding `max_decision_time_s`, returning no hard-feasible
candidate, or losing valid state/history immediately surrenders authority to
the legacy coast controller. Position handoff occurs only after measured
projected speed, total roll/pitch, total roll/pitch rate, and the level command
remain inside their terminal limits for the configured dwell.

```yaml
predictive_braking:
  enabled: false

learning_velocity_mpc_shadow:
  enabled: true
  command_authority: true  # false keeps real-state shadow behavior
  target_velocity_m_s: 0.0
  direction_xy: null       # infer +/-Y from the release; or [0.0, -1.0]
  log_interval_s: 0.10
  max_decision_time_s: 0.008
  controller:
    include_selected_trace: false
    prediction_horizon_s: 1.0
    pulse_grid_step_s: 0.02
    max_acceleration_tilt_deg: 8.0
    terminal_velocity_tolerance_m_s: 0.05
    terminal_tilt_tolerance_deg: 3.0
    overshoot_tolerance_m_s: 0.02
```

This replaces the old release command selector, not the identified dynamics.
LMPC still needs the fitted
`delay_s`, `wn_rad_s`, `zeta`, command gain, and motion gain. A future model
replacement must first produce the same frozen-model interface and pass
held-out directional validation.
