# V2 position-handoff model: evidence and assumptions

Audit date: 2026-09-04. Scope: local, offline source/log inspection. No new fit, flight, deployment, remote access, calibration write, or V1 modification was performed for this audit.

## Conclusion

The inspected evidence identifies an attitude-command response and records real fixed-position capture trials. It does **not** provide an identified continuous position-command capture model. The complete saved capture evaluation is explicitly rejected (`usable=false`): 4 of 12 trials passed. A V2 effective PD position loop must therefore be labeled an assumed simulator surrogate, not a calibrated onboard controller or a physical stopping guarantee.

## Real position-capture evidence

The complete result is in [14:25:10 flight log, line 179595](/Users/shuqinzhu/Documents/FLS_Research/lightbender/orchestrator/logs/translation_inertia_2026-09-03_14-25-10/lb11_translation_inertia_2026-09-03_14-25-10.json:179595), event `Position Capture Calibration Evaluated`. Its `data.position_capture_fit` contains the protocol, all 12 trial summaries, and cached controller context. The following line, 179596, records `Position Capture Calibration Rejected`, reason `fixed-target capture quality gates failed`, with the previous calibration preserved.

| Measured trial property | Observed value |
|---|---|
| Trial count / passed | 12 / 4; passing segments 3, 5, 6, 8 |
| Directions | +X, -X, +Y, -Y |
| Prescribed target gaps | 0.08, 0.16, 0.28 m |
| Forward entry speed | 0.109660 to 0.414944 m/s |
| Entry tilt magnitude | 1.45203 to 2.75381 degrees |
| Capture observation | 3.99860 to 4.25306 s; nominal protocol 4 s |
| Peak overshoot, across trials | 0.0110455 to 0.0365347 m |
| Peak reverse speed, across trials | 0.0272821 to 0.0751533 m/s |
| Terminal XY position error | 0.00609915 to 0.0172848 m |
| Terminal XY speed | 0.0119173 to 0.0726834 m/s |
| Quality failures | 1 overshoot, 6 excessive reverse-speed, 5 insufficient terminal settling, 2 sample-gap failures; categories overlap |

The two excessive sample gaps were 0.167149 and 0.253790 s (segments 2 and 9). These are not clean uniformly sampled system-identification records. Small terminal position error alone did not establish settling: the protocol required XY error <= 0.03 m and XY speed <= 0.05 m/s continuously for at least 0.30 s at the end, as well as overshoot, reversal, and telemetry-quality gates.

The summary schema is `empirical_position_capture_trials`, `fit_schema_version=1`, `usable=false`, and `continuous_capture_envelope_certified=false`. The function [summarize_position_capture](/Users/shuqinzhu/Documents/FLS_Research/fls-cf-offboard-controller/Interaction/position_capture_calibration.py:265) calculates empirical metrics and pass/fail; it does not estimate position-loop coefficients. See metric calculations at lines 320-377 and the explicit scope at lines 1-5. [Calibration persistence](/Users/shuqinzhu/Documents/FLS_Research/fls-cf-offboard-controller/Interaction/wrench_model_calibration.py:1720) rejects unusable capture evidence and explicitly forbids interpreting it as attitude gain or a continuous capture envelope.

Other local records do not establish a usable position fit:

- [13:32:55 log, line 145664](/Users/shuqinzhu/Documents/FLS_Research/lightbender/orchestrator/logs/translation_inertia_2026-09-03_13-32-55/lb11_translation_inertia_2026-09-03_13-32-55.json:145664): capture rejected because XY speed exceeded the configured safety limit.
- [15:59:06 log, line 82734](/Users/shuqinzhu/Documents/FLS_Research/lightbender/orchestrator/logs/translation_inertia_2026-09-03_15-59-06/lb11_translation_inertia_2026-09-03_15-59-06.json:82734): same rejection reason.
- [16:07:59 log, line 93758](/Users/shuqinzhu/Documents/FLS_Research/lightbender/orchestrator/logs/translation_inertia_2026-09-03_16-07-59/lb11_translation_inertia_2026-09-03_16-07-59.json:93758): `Wrench Model Calibration Saved` contains `position_capture_fit=null`.

The 14:25:10 log was the only inspected local log containing a non-null `position_capture_fit` object. The local controller checkout does not contain `Interaction/wrench_calibration.json`. This does not establish the current contents of the remote Pi calibration; no remote read was attempted.

## Cached PID settings are not an estimated effective PD plant

The 14:25:10 result records `controller_parameter_source=crazyflie_cached_parameters`, 100 Hz host control, mass 0.17 kg, and `automatic_interaction_handoff_enabled=false`. Relevant cached settings were:

| Parameter group | Recorded settings |
|---|---|
| `posCtlPid` | X/Y Kp approximately 1.9/2.1; X/Y Ki approximately 0.1; X/Y Kd and feedforward 0 |
| `velCtlPid` | VX/VY Kp 30; Ki 4; Kd approximately 0.005; feedforward 0 |
| Position-controller limits | roll/pitch 20 degrees; X/Y maximum velocity 1 m/s |
| `stabilizer` | estimator 2; controller 1 |

These are observed parameter settings copied from the cached parameter groups at evaluation, not regression estimates of a single equation such as `acceleration = Kp * position_error - Kd * velocity`. Firmware units and transforms, the cascaded position/velocity/attitude loops, update rates, filters, integral state, saturation and anti-windup, and mode-transition behavior must be represented before claiming equivalence. In particular, neither velocity Kp=30 nor position Kp=1.9/2.1 can simply be substituted as an acceleration-domain effective Kp/Kd.

The cached-context collection is documented in [interactions.py](/Users/shuqinzhu/Documents/FLS_Research/fls-cf-offboard-controller/Interaction/interactions.py:8567). Existing `coast_position_gain_s2=4.0` and `coast_velocity_gain_s=2.5` are configured control-law defaults, not fitted onboard position-capture dynamics; see [wrench_interaction_pipeline.py](/Users/shuqinzhu/Documents/FLS_Research/fls-cf-offboard-controller/Interaction/wrench_interaction_pipeline.py:180).

## Genuinely estimated attitude/motion parameters

V1's existing [frozen report](/Users/shuqinzhu/Documents/FLS_Research/lightbender/orchestrator/logs/translation_inertia_2026-09-03_16-47-29/repeat_summary/frozen_validation/report.json:40) retains estimates from the 16:07:59 reference log:

- Command delay: 0.0253699167330055 s.
- Natural frequency: 14.563081936060067 rad/s.
- Damping ratio: 0.9866363098697236.
- Command-to-tilt gain: 1.0345673103996114.
- World-Y tilt bias: -0.0014832332913454936 rad.
- Motion gain: 1.0954087248022135 (report line 70).

The stored tilt training RMSE is 0.297665 degrees. The report's six-trial later braking comparison has pooled brake tilt RMSE 0.344358 degrees; the frozen second-order prediction has terminal-speed MAE 0.0334853 m/s and endpoint MAE 0.0376393 m. These metrics concern attitude-command braking before position recovery. They neither identify nor validate a position-command handoff model. The existing report and its original provenance are preserved; this audit did not refit or rerun it.

## Target definition and causal timing

The real capture protocol latches its target **at capture entry**: current measured position plus the prescribed positive gap along the trial direction. It does not use a coast target fixed at an earlier release. See [target construction and latch](/Users/shuqinzhu/Documents/FLS_Research/fls-cf-offboard-controller/Interaction/position_capture_calibration.py:217).

Consequently, these trials support evidence about fixed-position command capture, but their target gaps are not direct measurements of accuracy to a release-time virtual stopping point. V2 must freeze the original coast target before policy actions and preserve it through position handoff and scoring. Replacing it with the handoff position or a newly convenient target would change the task and hide overshoot.

For later offline trace extraction, use the actual already-sent position command and its start timestamp, not a future scheduled phase. [Sample attribution](/Users/shuqinzhu/Documents/FLS_Research/fls-cf-offboard-controller/Interaction/interactions.py:6012) requires `state_time >= active_position_capture_command_since`, associates the sample with the previously latched capture command, and preserves the final capture sample before recovery. Logging field names include `position_capture_calibration_segment_id`, `position_capture_calibration_phase`, `position_capture_fixed_target_m`, `position_capture_command_started_at`, `state_time`, `position_m`, and `velocity_m_s`.

## V2 surrogate assumptions, not experimental estimates

The planned V2 direct-position latch uses a fixed original target and a common 3 s simulation horizon. Its nominal effective position PD settings Kp=4 s^-2, Kd=3 s^-1, and acceleration cap=2 m/s^2 are assumptions. Planned sensitivity ranges Kp=[2,6], Kd=[2,4.5], cap=[1,3], feedback delay=[0,0.04] s, activation delay=[0,0.08] s, and a 20 ms position-loop period are also assumptions, not calibrated uncertainty bounds.

The run manifest is authoritative for the implemented configuration. Delayed activation must preserve the preceding command until the simulated handoff actually becomes active; delayed feedback must never read future state. Report position error, speed, overshoot, reversal and final dwell separately. Any success applies only to this explicitly assumed surrogate and finite observation window. A 3 s V2 horizon is not interchangeable with the real 4 s capture trials or V1's 1.5 s post-level tail.

## Input and source SHA-256 at audit

Full paths are given in the links above. Hashes cover complete file bytes, not selected events:

| File | SHA-256 |
|---|---|
| 13:32:55 flight log | `a679d4cbf2f204fea0804aa62911ba2636b28832f47911b8807d7ac7ed6a2069` |
| 14:25:10 flight log | `e8eedb9cc05a164affdeedf2d38cffbf0731734fbf84e6e2fd2a7671253136d8` |
| 15:59:06 flight log | `9f70ec9edbef5c3fd46ed29a1c75424352b714b59bb08ce1e4deae0dd514f7c6` |
| 16:07:59 flight log | `ade0292b40f7df775b4617af523cc3d45c34c4f03713ce7cdebd86de0315b57d` |
| Frozen attitude report | `cd288fd7a256a3396401144c37a27cade07a8abeefa71efaf9dac9aad30ed0c2` |
| `Interaction/position_capture_calibration.py` | `d85408010f934829e7ba2b6a53db80c133adf39aed6e6aded9e1c8678b559126` |
| `Interaction/wrench_model_calibration.py` | `41af24938e73877479333584467c6da8f5f5c3d1acb2af73c68f995b9680691f` |
| `Interaction/interactions.py` | `92a4aff4ff44ad8844ce6a9f01a94e2074fa9bb0c8a89fd532b5d3dfca2f0767` |
