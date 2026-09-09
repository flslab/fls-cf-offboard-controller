# V2 fixed-target offline comparison

**Position dynamics are an uncalibrated surrogate. No physical validation or deployment.**

Common3s horizon; target fixed before action. V1/predictor LEVEL actions are adapted to direct POSITION takeover, not native v1 dynamics.

|Population|Method|N|Loss|Joint settled|Settled without reversal|Reversals|Tail max error mean cm|Tail max speed mean m/s|Worst overshoot cm|
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
|development|fixed240ms_position|64|13.646|22|0|64|3.71|0.105|30.54|
|development|immediate_position|64|15.008|38|1|63|2.92|0.073|43.57|
|development|nominal_distance_gate|64|14.097|34|1|63|3.61|0.095|24.65|
|development|predictive080_position_adapter|64|9.347|35|0|64|2.98|0.070|23.14|
|development|v1_ppo_position_adapter|64|8.952|36|0|64|2.85|0.065|23.24|

Joint settled means the complete last0.3s satisfies position<=3cm, speed<=0.04m/s, tilt<=3deg and rate<=5deg/s. It does not imply no earlier overshoot/reversal; that conjunction is reported separately.
Regression targets are assigned synthetic gaps, not log-derived release endpoints. Aggregate loss is not a flight-safety probability.
