# Firmware curve recording

Firmware build/date identifiers (`pRelSVer`, `pRelAdVer`, `pRelJVer`) are optional
diagnostic metadata, not startup or arming gates. There is no build whitelist.
The controller still checks required parameters, executing response-model support,
parameter write/readback, estimator readiness, and the wire formats it decodes
(`curveVer=1`, release-event protocol 1). New build numbers need no host edit.

Enable recording under `Interaction.config.wrench_interaction.firmware_auto_brake`:

```yaml
enabled: true
mode: scurve
response_model:
  enabled: true
  calibration_file: Interaction/attitude_response.json
curve_log:
  enabled: true
  user_id: P01
  trial_id: null
```

With the normal `--log` option, each run writes `TAG.curves.jsonl` and
`TAG.states.jsonl` beside its main log. `user_id` labels a participant;
null `trial_id` defaults to the run tag. `--calibrate` records states only and
does not activate braking or require the curve-event firmware protocol.

Curve events capture **actually activated** initial/replanned seventh-order
curves and hold/abort/interruption. They preserve plan IDs, replacement IDs,
model and state snapshots, along/cross equivalent-tilt coefficients, segment
durations, and activation/plan-origin firmware timestamps. Coefficients are
ascending powers of normalized local segment time, in radians; split plans
have two half-duration segments. They are not separate XYZ position curves.

Continuous logs request 100 Hz kinematics, acceleration and attitude commands.
They include velocity, world acceleration without gravity, roll/pitch, raw body
rates and converted Euler roll/pitch rates. Continuous attitude comes from
`stateEstimateZ`; curve events separately identify the planner's attitude source.
The independently sampled log blocks are not an atomic state snapshot.

From the offboard repository:

```sh
python -m Interaction.curve_log_export /absolute/path/TAG.curves.jsonl
```

The paired states file must be beside it. Export creates `TAG.comparison.csv`
without overwriting existing files, joins by firmware time within 5 ms, and
leaves unmatched fields blank. `curve_start_relative_s` starts at the initial
curve activation, not the earlier release event. Runs must be under 2.33 hours.
Missing fragments, CRC failures, queue drops, missing state samples and writer
errors are explicit. Incomplete runs require `--allow-incomplete` for diagnosis;
CSV rows then retain `recording_complete=False`. ACK means queued to the file
writer, not fsync. Real radio bandwidth still needs flight-environment validation.

The normal LightBender orchestrator needs its paired sidecar-download support
to fetch both files; otherwise they remain safely on the Pi in `logs/`.
Calibration files, experiment output, firmware snapshots and generated CAD stay
local and are excluded by `.gitignore`. Runtime code, tests and this operating
guide remain tracked. Artifact cleanup removes files from the current Git tree,
not from historical commits or local ignored copies.
# Diagnostic validity and tracking gaps

Curve wire protocol v1 does not carry per-field validity for `abort` or
`interrupted` events. The decoder therefore writes their position, velocity,
acceleration, attitude and rates as `null`, with
`state_validity: unknown_terminal_state_wire_v1`. Original wire values remain
under `raw_unverified_state`; zero there is not evidence of a stopped drone.
Previously recorded JSONL files are not rewritten by this change.

`CURVE_STATUS` includes `scQual` and `scAge` at 10 Hz. `scAge` is the last
successful prediction query's source age, not a continuously advancing clock
after a failed query. `CURVE_ESTIMATOR` records Vicon-filter velocity and shadow
ESKF velocity/acceleration at 10 Hz; these are diagnostic sources, not all the
same as the S-curve feedback observer or ordinary PID state. Compare them with
the existing 100 Hz ordinary-state logs and activated curve snapshots, aligned
by firmware timestamps. No estimator ownership is changed.

When mocap timing logging is enabled, `point_tracking` reports `matched`,
`empty_cloud`, or `outside_gate`, including nearest-point and matching-gate
distances. Missing matches do not generate a fake pose or widen the gate.
