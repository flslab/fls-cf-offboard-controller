# Simulator-trained PPO: paired offline evaluation

**No flight validation. No position target or position-controller handoff.**

All methods receive identical initial conditions, hidden plant parameters, measurement bias/delay and command delay. Every method is observed for 1.5 s after its level command. Lower scalar loss is not by itself a safety result.

| Population | Method | N | Mean loss | Reversals | Near stationary | Low-speed level | Worst rollback cm | Mean abs. tail speed m/s |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| held_out_synthetic | fixed_240ms | 256 | 2.4090 | 212 | 10 | 36 | 141.82 | 0.3305 |
| held_out_synthetic | ppo_lineage11 | 256 | 0.1500 | 26 | 23 | 179 | 18.18 | 0.1282 |
| held_out_synthetic | ppo_lineage22 | 256 | 0.1467 | 28 | 23 | 178 | 14.90 | 0.1261 |
| held_out_synthetic | ppo_lineage33 | 256 | 0.1557 | 34 | 21 | 174 | 18.18 | 0.1219 |
| held_out_synthetic | predictive_margin_025 | 256 | 0.6318 | 161 | 30 | 93 | 47.01 | 0.1078 |
| held_out_synthetic | predictive_margin_080 | 256 | 0.4137 | 115 | 32 | 135 | 39.58 | 0.0997 |
| observed_initial_nominal_regression | fixed_240ms | 7 | 0.8521 | 7 | 0 | 0 | 20.62 | 0.1067 |
| observed_initial_nominal_regression | ppo_lineage11 | 7 | 0.1656 | 0 | 0 | 4 | 0.00 | 0.1856 |
| observed_initial_nominal_regression | ppo_lineage22 | 7 | 0.0806 | 0 | 0 | 7 | 0.00 | 0.0999 |
| observed_initial_nominal_regression | ppo_lineage33 | 7 | 0.1203 | 0 | 0 | 7 | 0.00 | 0.1399 |
| observed_initial_nominal_regression | predictive_margin_025 | 7 | 0.0238 | 0 | 2 | 7 | 0.00 | 0.0427 |
| observed_initial_nominal_regression | predictive_margin_080 | 7 | 0.0749 | 0 | 0 | 7 | 0.00 | 0.0942 |
| observed_initial_stress_regression | fixed_240ms | 133 | 0.9655 | 112 | 6 | 21 | 48.39 | 0.1321 |
| observed_initial_stress_regression | ppo_lineage11 | 133 | 0.1493 | 2 | 4 | 87 | 2.19 | 0.1660 |
| observed_initial_stress_regression | ppo_lineage22 | 133 | 0.0962 | 12 | 15 | 112 | 5.53 | 0.0957 |
| observed_initial_stress_regression | ppo_lineage33 | 133 | 0.1070 | 3 | 13 | 114 | 0.80 | 0.1228 |
| observed_initial_stress_regression | predictive_margin_025 | 133 | 0.2357 | 54 | 20 | 79 | 19.92 | 0.0705 |
| observed_initial_stress_regression | predictive_margin_080 | 133 | 0.1243 | 23 | 20 | 97 | 12.11 | 0.0900 |

## Interpretation limits

- Held-out synthetic means new random draws from the training distribution, not new measured flight data.
- Measured initial-state regression cases were inspected before training; later motion is independently simulated.
- The 0.08 m/s predictor has a more conservative velocity margin than the PPO loss target of 0.025 m/s; the 0.025 predictor is also reported.
- Loss combines speed error, reverse velocity, rollback and duration. Inspect each component: minimizing loss does not guarantee non-reversal.
- Near stationary uses a finite last-100-ms speed tolerance, not indefinite hover or accurate stopping position.
- The policy cannot accelerate forward or change a target. Some states may not be recoverable with this action set.
- RL is trained by interaction with a simulator, offline from the aircraft. This is not dataset-only offline RL.
