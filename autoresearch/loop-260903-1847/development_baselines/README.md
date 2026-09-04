# Simulator-trained PPO: paired offline evaluation

**No flight validation. No position target or position-controller handoff.**

All methods receive identical initial conditions, hidden plant parameters, measurement bias/delay and command delay. Every method is observed for 1.5 s after its level command. Lower scalar loss is not by itself a safety result.

| Population | Method | N | Mean loss | Reversals | Near stationary | Low-speed level | Worst rollback cm | Mean abs. tail speed m/s |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| development | fixed_240ms | 64 | 2.5181 | 53 | 2 | 7 | 135.60 | 0.3514 |
| development | predictive_margin_025 | 64 | 0.6528 | 42 | 8 | 22 | 52.37 | 0.1096 |
| development | predictive_margin_080 | 64 | 0.3855 | 30 | 11 | 32 | 40.92 | 0.0927 |

## Interpretation limits

- Held-out synthetic means new random draws from the training distribution, not new measured flight data.
- Measured initial-state regression cases were inspected before training; later motion is independently simulated.
- The 0.08 m/s predictor has a more conservative velocity margin than the PPO loss target of 0.025 m/s; the 0.025 predictor is also reported.
- Loss combines speed error, reverse velocity, rollback and duration. Inspect each component: minimizing loss does not guarantee non-reversal.
- Near stationary uses a finite last-100-ms speed tolerance, not indefinite hover or accurate stopping position.
- The policy cannot accelerate forward or change a target. Some states may not be recoverable with this action set.
- RL is trained by interaction with a simulator, offline from the aircraft. This is not dataset-only offline RL.
