# V2 fixed-target offline comparison

**Position dynamics are an uncalibrated surrogate. No physical validation or deployment.**

Common3s horizon; target fixed before action. V1/predictor LEVEL actions are adapted to direct POSITION takeover, not native v1 dynamics.

|Population|Method|N|Loss|Joint settled|Settled without reversal|Reversals|Tail max error mean cm|Tail max speed mean m/s|Worst overshoot cm|
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
|held_out_synthetic|fixed240ms_position|256|14.111|67|0|256|3.74|0.112|32.14|
|held_out_synthetic|immediate_position|256|14.177|138|17|238|2.43|0.068|70.71|
|held_out_synthetic|nominal_distance_gate|256|14.047|120|17|238|3.34|0.095|31.87|
|held_out_synthetic|ppo_lineage41|256|6.678|159|21|230|1.74|0.046|33.41|
|held_out_synthetic|ppo_lineage52|256|6.586|157|19|233|1.78|0.047|32.40|
|held_out_synthetic|ppo_lineage63|256|6.577|166|24|228|1.72|0.046|31.87|
|held_out_synthetic|predictive080_position_adapter|256|9.087|128|10|246|2.45|0.073|32.05|
|held_out_synthetic|v1_ppo_position_adapter|256|8.520|125|9|246|2.31|0.067|32.14|
|observed_initial_synthetic_target_regression|fixed240ms_position|21|7.434|21|0|21|0.37|0.002|15.03|
|observed_initial_synthetic_target_regression|immediate_position|21|13.448|21|0|21|0.38|0.003|23.15|
|observed_initial_synthetic_target_regression|nominal_distance_gate|21|9.926|21|0|21|0.39|0.003|14.79|
|observed_initial_synthetic_target_regression|ppo_lineage41|21|7.592|21|0|21|0.37|0.002|14.80|
|observed_initial_synthetic_target_regression|ppo_lineage52|21|7.306|21|0|21|0.37|0.002|15.96|
|observed_initial_synthetic_target_regression|ppo_lineage63|21|7.675|21|0|21|0.37|0.002|15.76|
|observed_initial_synthetic_target_regression|predictive080_position_adapter|21|7.094|21|0|21|0.37|0.002|15.44|
|observed_initial_synthetic_target_regression|v1_ppo_position_adapter|21|7.122|21|0|21|0.37|0.002|15.59|

Joint settled means the complete last0.3s satisfies position<=3cm, speed<=0.04m/s, tilt<=3deg and rate<=5deg/s. It does not imply no earlier overshoot/reversal; that conjunction is reported separately.
Regression targets are assigned synthetic gaps, not log-derived release endpoints. Aggregate loss is not a flight-safety probability.
