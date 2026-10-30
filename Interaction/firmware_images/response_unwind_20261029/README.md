# Response-aware command unwind

Local candidate: `bolt-response-unwind-26102901`, automatic-unwind API 2.
This package has not been pushed, pulled onto a Pi, flashed, armed, or flight-tested.
The currently deployed image, accepted attitude-response calibration, and active mission YAML are unchanged.
See `build-manifest.json`, `native-validation.json`, and `log-admission.json` for exact validation evidence.

## Control change

The previous path treated its seventh-order attitude reference as the aircraft response and then applied an inverse tracking correction.
The new seventh-order polynomial is the **issued roll/pitch command**.
Its forward prediction includes measured attitude and Euler angular rate, calibrated axis-specific response delay, the already-issued 150 ms command queue, 100 Hz held commands, and the aircraft response after the polynomial ends.
The predictor uses a nonlinear tilt-to-horizontal-acceleration map and splits integration at command activation boundaries.
The accepted calibration coefficients remain unchanged; the diagnostic fit of two recent interactions is not installed as a calibration.

Strong directional braking remains the first phase, with the configured nominal deceleration and tilt envelope.
The planner evaluates slower/faster response hypotheses and the remaining braking impulse each control update.
It checks minimum signed velocity over the return, rather than accepting a zero final prediction after an intermediate reversal.
A one-command-step viability check initiates the return before another brake update would make a bounded return unavailable.
The search favors a longer feasible return within the remaining velocity reserve.

The return starts from the last issued command and its command derivatives.
Takeover uses the actual held PID target with zero right-side command derivatives; adjacent 2 ms queue differences are not aircraft angular rates.
Replanning preserves command value, rate, acceleration, and jerk and can only shorten an accepted return deadline.
Both the polynomial and the later settling correction pass through the same bounded C3 command shaper.
There is no unbounded inverse-model command reversal at the end of the tail.
After the command polynomial ends, measured-state settling continues until the existing completion gates pass.

The response hypotheses are finite design checks, not a proof for arbitrary model error.
They vary frequency, damping, gain, and delay; every tick starts again from measured motion.
The existing translational acceleration-residual correction remains available.
No new angular-acceleration observer or online calibration replacement is enabled.

## Failure and completion

Before issuing new braking commands, the initial state and queued commands must admit a bounded return under the checked models.
The candidate searches longer returns when a shorter command-bounded return violates predicted physical rate/acceleration constraints.
If no return is found, it reports abort 18 and uses the existing stage-6 level/host-abort path.
It does not claim that fallback can undo momentum or attitude response already present at admission.
A later loss of return feasibility exits strong braking instead of continuing to wait for a return.
Invalid inputs, missing history, and the existing release deadline remain failures, not successful stops.

The measured completion conditions remain simultaneous for 100 ms of fresh feedback:

| Condition | Limit |
| --- | --- |
| XY velocity error from requested terminal vector | 0.02 m/s |
| Measured roll/pitch norm | 0.5 deg |
| Euler roll/pitch rate norm | 5 deg/s |
| Delay-compensated remaining stop-velocity error | 0.02 m/s |
| Command deviation from calibrated equilibrium | 0.5 deg |
| Command slew norm | 5 deg/s |

The command reference must have ended, and no unsafe pending impulse may remain.
The final position target is captured at measured completion, retaining the existing handoff behavior.
PB, OB, and the no-coasting direct-hold path are unchanged.

## Candidate configuration

`mission-overrides.yaml` is a reviewable overlay, not an active mission.
`mission-example.yaml` is a copy of the current local mission with only the new constraint mapping added.
Its marker offset remains the user's current `[0, 0, 0.08]` m.
Apply these only with the paired API-2 firmware after target validation.

Presence of `analytic_profile.unwind_constraints` opts into API 2, including an empty mapping for defaults.
Old saved missions without this mapping retain their API-1 contract and are rejected by the API-2 preflight rather than silently paired with a different controller.
The host confirms the new capability and settings before arming.
No-coasting bypasses the automatic-unwind and marker settings as before.

| `unwind_constraints` setting | Default | Meaning |
| --- | ---: | --- |
| `max_command_rate_deg_s` | 160 | Norm of issued-command rate |
| `max_command_acceleration_deg_s2` | 4000 | Norm of issued-command curvature |
| `max_command_jerk_deg_s3` | 80000 | Norm of issued-command third derivative |
| `min_response_frequency_scale` | 0.65 | Slow-response hypothesis relative to accepted model |
| `response_delay_margin_s` | 0.02 | Additional checked delay uncertainty |
| `settle_gain_s_inv` | 1.5 | Stop-coordinate settling gain |
| `velocity_reserve_m_s` | 0.08 | Internal forward reserve before settling; final target stays zero |
| `max_predicted_rate_deg_s` | 120 | Predicted aircraft rate design constraint |
| `max_predicted_acceleration_deg_s2` | 1000 | Predicted aircraft angular-acceleration design constraint |
| `settle_max_correction_deg` | 6 | Maximum settling-command correction from neutral trim |

The bounds are configurable and validated on both sides; see `UNWIND_CONSTRAINTS` in the host validator.
Command derivatives and aircraft angular derivatives are different quantities.
The physical screening preserves the unavoidable transient caused by already-queued commands, including activation of the existing held target; it cannot retroactively enforce a lower limit on that transient.
Consequently, recorded peak predicted/plant acceleration can exceed the design setting when the initial queue already requires it.
These design settings are not certified hardware acceleration limits.
Polynomial constraint screening uses 33 samples with 5% derivative margin; this is not an analytic extrema proof.

Fixed search details are explicit in the included source: nominal return range 0.15–1.2 s, replanned remaining segment minimum 0.08 s, 15 ms coarse search, eight duration refinements, 40 ms replanning interval, and 100 ms terminal dwell.
The initial shortest-return admission search starts at 0.15 s.
The internal negative-budget recovery gain is 4/s.
The configured maximum release duration and nominal deceleration remain independent settings.
`tail_s` is still accepted for the legacy path; it does not select this automatic return duration.
Command-return duration, physical leveling time, and the complete release-phase duration are distinct.

## Logs and verification

`scUnwT` reports the whole accepted command-return duration, including replans.
New optional log fields expose worst remaining stop velocity, minimum predicted velocity, replan count, and command-shaper continuation count.
Curve flag `0x4000000` distinguishes issued-command coefficients from the older assumed-attitude records.
The host reconstructs only the logged polynomial segment in degrees; it does not fabricate physical attitude, velocity, or the later settling commands from that polynomial.
No extra high-rate telemetry group is enabled by this patch.

Run `python validate.py` for package hashes, linked parameter presence, boundary tests, and the smaller native grid.
Run `python validate.py --extended` for the full independent native plant grid with AddressSanitizer and UndefinedBehaviorSanitizer.
The plant uses separate double-precision closed-form attitude propagation and nonlinear tilt force.
Rejected initial states are reported separately and do not count as successful physical stops.
The saved initial-state replay uses sampled 50 Hz command telemetry and therefore cannot reproduce the exact on-FC command queue or prove a flight counterfactual.

The candidate's stabilizer stack allocation is increased from 3600 to 4800 bytes only for the opt-in joint-unwind build.
ARM compiler stack-frame inspection and successful linking do not establish runtime stack headroom or control-loop timing.
On-target timing, a matching-estimator response validation, and flight validation remain required before deployment.
