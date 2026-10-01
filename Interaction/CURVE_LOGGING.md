# Flight curve/state logging with per-curve timing

The 26092404 candidate adds diagnostic timing to the 26092403 selected bounded
S-curve runtime. It does not change reference generation, execution gains,
estimator selection, handoff targets, or enable replanning. Flashing/deployment
is separate; this file is not a flight acceptance certificate.

## Enable and identify participants

In the mission's `Interaction.config.wrench_interaction.firmware_auto_brake`,
keep the selected analytic profile and add/retain:

```yaml
curve_log:
  enabled: true
  user_id: P01
  trial_id: null
```

Use a pseudonymous `user_id` for each participant. `trial_id: null` uses the
run tag; every recording also gets an independent UUID `run_id`. Firmware
session/sequence and plan/event IDs distinguish repeated contacts in one run.
Missing user IDs generate a warning, not an invented participant identity.
The ordinary flight command must still have logging enabled (`--log`).
`--calibrate` records continuous states only; it does not activate braking or
require a curve-event protocol. Firmware build/date identifiers remain optional
diagnostic metadata, not arming gates. Required capabilities, parameter readback,
estimator readiness and decodable wire formats are still checked.

Curve logging does not itself require a calibration file. In particular,
`command_mode: velocity` uses the velocity PID and intentionally keeps
`response_model.enabled: false`; enabling an attitude-response worker is not a
remedy for a logging startup error. Startup negotiates wire v1/v2 with the
firmware and rechecks the same version before arm. Wire v2 includes timing.
On multi-execution firmware, plain velocity mode explicitly writes and reads
back `pRelExec=0`, `pRelShape=0`, and `pRelLite=0` where exposed. `pRelVelCmd=1`
alone is not sufficient to select velocity execution. An explicit analytic
profile retains its own execution/shape settings. Runtime metadata leaves
unspecified tail duration unset instead of guessing a firmware default; the
actual segment duration and state source remain in each curve event.

## Saved files and units

- `LOG_DIR/TAG.curves.jsonl`: initial/phase/replan and hold/abort/interrupted
  snapshots, coefficients, segment duration, coefficient units, sources,
  firmware/runtime configuration, participant IDs and timing.
- `LOG_DIR/TAG.states.jsonl`: requested 100 Hz velocity XYZ (m/s),
  gravity-removed world acceleration XYZ (m/s²), roll/pitch (degrees),
  Euler roll/pitch rate (degrees/s), raw body rates and commanded roll/pitch.
  Status/estimator diagnostics remain 10 Hz. Actual delivery can have gaps;
  gaps, missing events, dropped firmware events and disk failures remain visible.
- No additional high-rate timing log block is allocated. Timing travels in
  the same acknowledged/CRC-protected event as its coefficients.

The ordinary `TAG.json` event stream also saves
`Firmware Post-Release Hold Target Received` once per received, identity-matched
FC hold notice. Its `hold_position_m` is the actual commanded target; the
`position_m` in a curve `hold` snapshot is instead the measured vehicle state.
The receipt event is written before monitor readiness validation, with
`hold_confirmed: false`. It does not acknowledge completion or bypass any
health check. `Firmware Post-Release Hold Acquired` remains the later, separately
validated success event. If no notice arrives, no target is fabricated.

For the selected S-curve, coefficients describe **velocity references**, not
attitude commands. They use ascending normalized local time:
`v(u) = c0 + c1*u + ... + c7*u^7`, `u = local_time / segment_duration`.
Keep duration, axes, source and units when comparing different participants;
the same numerical coefficients at different durations do not mean the same
physical acceleration. Actual attitude/commands are separate state channels.

Continuous values are the published firmware state; when the configured
unified ESKF is active, this is its selected state, not injected simulator GT.
Cross-block samples are not atomic: the CSV preserves acceleration/command
timestamp skew and leaves unmatched values blank instead of inventing zeros.
The exporter matches independent blocks within 5 ms. `curve_start_relative_s`
starts at curve activation, not the earlier Arduino release/ACK. Recordings
must be shorter than half a 24-bit log-clock wrap (about 2.33 hours).

## Timing fields (microseconds)

Each v2 curve snapshot has a `timing` object:

| Field | Meaning |
|---|---|
| `plan_compute_us` | Measured generation/initialization of this applied reference segment; not the curve's execution duration or release-ACK latency |
| `control_step_max_us` | Largest bounded-runtime elapsed step through this snapshot, including generation or terminal prediction when they occur |
| `hold_compute_us` | One-time terminal stopping-point predictor elapsed time; unset until it has run |
| `control_steps` | Number of measured runtime steps accumulated for this interaction |
| `clock` | `mcu_elapsed` on Bolt, `sitl_host_elapsed` in SITL, or `unavailable` |

The initial event can have no step maximum yet; the terminal event carries
the accumulated maximum. The terminal snapshot cannot include the time to
serialize itself. These are elapsed times (including MCU preemption), not
exclusive CPU cycle counts, whole-stabilizer WCET, or radio transmission time.
The MCU uses its microsecond hardware clock. SITL uses host monotonic time,
not the sensor-driven simulation clock that can stand still during a function.
Do not use SITL timings to claim Bolt timing/watchdog acceptance.

The new timing instrumentation covers the selected bounded analytic runtime.
Other legacy planners produce unavailable fields, not fabricated zero times.
v1 files/firmware remain readable: timing fields are null. The offboard checks
supported wire formats (1/2), not a firmware build-date allowlist.

## Export for comparisons

From the offboard repository:

```sh
venv/bin/python -m Interaction.curve_log_export /absolute/path/TAG.curves.jsonl
```

This creates, without overwriting existing files:

- `TAG.coefficients.csv`: one row per event/axis/segment; `c0`–`c7`, duration,
  participant/run/interaction/plan identifiers, source and timing columns.
- `TAG.comparison.csv`: aligned continuous state and reference, with the
  latest event's timing snapshot. Timing columns are event snapshots, **not**
  a fresh timing measurement for every state row.

An incomplete recording is rejected by default. `--allow-incomplete` is an
explicit diagnostic export; both CSVs retain `recording_complete=False`.
It neither fills missing samples nor qualifies the trial as complete.

To compare **coefficients only** when the acknowledged curve events are
complete but the continuous samples have gaps, use `--coefficients-only`.
This still requires `curve_events_complete=True`; the CSV separately retains
the whole-recording flag. Interrupted snapshots without a valid reference
have blank coefficients and `curve_reference_valid=False`.

ACK means accepted by the asynchronous file writer, not fsync. Writer/drain
failures still prevent a complete summary. No previous JSONL file is rewritten.
The paired LightBender orchestrator needs its sidecar-download support to fetch
both JSONL files; otherwise they remain in the Pi's `logs/` directory. Firmware
snapshots and experiment output remain ignored; runtime code, tests and this
guide are tracked.

## Diagnostic validity

Neither v1 nor v2 includes per-state-field validity for abort/interruption.
Their position/velocity/acceleration/attitude/rates are therefore null, with
original values under `raw_unverified_state`. A raw zero does not prove stop.
`CURVE_STATUS.scAge` is the last successful prediction query's source age,
not a clock that keeps advancing after a failed query. `CURVE_ESTIMATOR`
contains 10 Hz diagnostic Vicon-filter and ESKF channels; inspect event source
labels before comparing these with the published 100 Hz state.

When existing mocap timing logging is enabled, `point_tracking` reports
`matched`, `empty_cloud` or `outside_gate` and the matching distances. This
change neither fabricates a missing pose nor widens the matching gate.

## Validation and deliverables

The isolated paired firmware/source/build artifacts and Gazebo logging runs
are under `autoresearch/curve-log-timing-260924/`. The previous 37-run study
and 26092403 binary are preserved unchanged. Use the new paired offboard files
and new Bolt binary to obtain measured timing on hardware; none has been
flashed/deployed by this change. See the generated delivery validation report
for actual test results and any observed continuous-log gaps.
# Optional calibrated S-curve attitude compensation

`analytic_profile.response_compensation: true` selects bounded response-aware
execution of the existing velocity S-curve. It requires attitude execution,
`response_model.enabled: true`, and `rate_feedforward: false`. Uploading the
model does **not** enable the adaptive replan worker; `pRelAdapt` stays zero.
`response_bandwidth` (6–16 rad/s, default 12) is latched for each curve.

Wire flag bit 21 marks this execution mode without changing the wire version.
Curve coefficients still have m/s units and along/cross-release-velocity axes.
The two response models instead have **roll/pitch** axes; decoded events expose
`response_axes` separately. Runtime configuration records the calibration,
PID identity, model id, compensation bandwidth and disabled replanning.
Coefficients alone reconstruct the reference, **not** the compensated command:
the latter also needs measured state/rates and the causal command history.

`pRelComp.us`, `guardN`, `satN`, and `stopV` are optional diagnostics for maximum
compensation step time, impulse-guard activations, command saturation, and
predicted forward stopping velocity. `us` is MCU elapsed time on Bolt and host
elapsed time in SITL, never a cross-platform timing equivalence. Existing
curve-event computation timing remains available. A compensation model/state/
history failure is abort reason 12, not an automatic fallback to fixed gains.
