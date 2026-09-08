# Learning velocity MPC (offline first)

`learning_velocity_mpc.py` is a minimum-time, receding-horizon controller for
one planar velocity transition. It is inspired by *Learning Model Predictive
Control for Quadrotors* (Rosolia et al., 2022), but it is not a reproduction of
that paper's repeated-track sampled-safe-set implementation.

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

The implementation is offline-only and has no `cflib` imports or device I/O.
Every returned dictionary says `flight_command_generated=False`. Its
no-overshoot result is conditional on the supplied frozen model, configured
uncertainty margin, fresh synchronized state, and available actuator authority.
It must pass logged replay, held-out calibration, compute-budget, shadow, and
staged flight gates before any integration with the interaction loop.

Only call `record_sent_command()` after the corresponding command was actually
sent. History must cover at least the identified attitude-command delay;
missing history fails closed without manufacturing a fallback attitude command.

Run the synthetic tests with:

```bash
venv/bin/python -m unittest Interaction.tests.test_learning_velocity_mpc
```

## Real-state shadow integration

The normal interaction entry now has an opt-in shadow adapter. It starts after
a confirmed potentiometer release, loads the saved directional
`prediction_model`, consumes live Crazyflie state plus the actual attitude
commands sent by the existing controller, and emits:

- `Learning Velocity MPC Shadow Started`
- `Learning Velocity MPC Shadow Decision`
- `Learning Velocity MPC Shadow Stopped` or `... Unavailable`

The adapter never applies the hypothetical MPC roll/pitch. Enable it in the
mission interaction configuration only for a later live-state shadow run,
while leaving the active predictive policy off:

```yaml
predictive_braking:
  enabled: false

learning_velocity_mpc_shadow:
  # Keep false for offline log replay. Change to true only after benchmarking
  # the remote compute/logging budget for a live-state shadow run.
  enabled: false
  target_velocity_m_s: 0.0
  direction_xy: null       # infer +/-Y from the release; or [0.0, -1.0]
  log_interval_s: 0.10
  controller:
    max_acceleration_tilt_deg: 8.0
    terminal_velocity_tolerance_m_s: 0.05
    terminal_tilt_tolerance_deg: 3.0
    overshoot_tolerance_m_s: 0.02
```

This replaces the old candidate selector only in the diagnostic comparison;
it does not replace the identified dynamics. LMPC still needs the fitted
`delay_s`, `wn_rad_s`, `zeta`, command gain, and motion gain. A future model
replacement must first produce the same frozen-model interface and pass
held-out directional validation.
