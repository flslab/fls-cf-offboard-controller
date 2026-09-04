# Sequential online-model replay

Offline replay only; no flight commands or active calibration updates.

Status: completed; complete data: True; held-out diagnostic gates passed: True.

The last all-data refit is NOT independently validated. Only the previous frozen model is scored on the final held-out pair.

| Version | Training segments | Validation segments | Passed |
|---|---|---|---|
| 1 | [0, 1] | [2, 3] | False |
| 2 | [0, 1, 2, 3] | [4, 5] | True |

## Last frozen model

```json
{
  "attitude_fit": {
    "model": "second_order",
    "delay_s": 0.026248713859991796,
    "wn_rad_s": 14.683277353342621,
    "zeta": 0.9922653238113869,
    "gain": 1.0349037938988435,
    "bias_world_y_rad": -0.0018620367305335532,
    "training_segments": [
      0,
      1,
      2,
      3
    ],
    "train_rmse_deg": 0.32232398721120975,
    "seed_costs": [
      3.164754227268505e-05,
      3.164754227167875e-05,
      3.1647542271758994e-05
    ],
    "active_bounds": [
      0,
      0,
      0,
      0,
      0
    ],
    "offline_only": true
  },
  "motion_gain": 1.08309593746612,
  "identifiability": {
    "method": "local finite-difference sensitivity scaled by declared parameter bounds",
    "parameter_order": [
      "delay_s",
      "wn_rad_s",
      "zeta",
      "gain",
      "bias_world_y_rad"
    ],
    "singular_values": [
      0.7938151202676987,
      0.14078604911968,
      0.06535102132297722,
      0.046885818048512856,
      0.03220427561256956
    ],
    "rank": 5,
    "condition_number": 24.64937046923878,
    "identifiable": true,
    "bound_active_parameters": [],
    "uncertainty_interval_available": false
  }
}
```

## Final held-out trials

| Segment | Tilt RMSE (deg) | Velocity RMSE (m/s) | Terminal velocity error (m/s) | Endpoint error (m) | Actual / predicted reverse |
|---|---|---|---|---|---|
| 4 | 0.3044 | 0.0375 | 0.0124 | 0.0568 | True / True |
| 5 | 0.2236 | 0.0457 | -0.0097 | 0.0518 | True / True |

## Interpretation limits

- Only the tested world-Y attitude-command response is identified.
- Delay is effective host-clock delay, including telemetry/scheduling.
- Evaluation is conditional on the executed command schedule; no future measured states initialize forecasts.
- Position-controller capture, X/Z response, payload/battery extrapolation and reliable stopping are not validated.
- Models are diagnostic only: runtime_enabled=false and flight_approved=false.

Full provenance, raw samples, gate failures and all intermediate candidates are in report.json and report.samples.jsonl.
