# Release diagnostics

Only the `pi_joint` interaction path temporarily requests `ReleaseTargets` while
waiting for firmware hold/fault. It starts after the release handoff reply, so
the first control cycle is not guaranteed to be captured. The block is optional
and deleted on exit; no control gates or PID gains are changed.

The requested physical period is 20 ms (50 Hz), using the existing hardware
period scale of 10. Six FP16 controller attitude/rate targets plus uint8 brake
stage occupy 13 data bytes per packet. These are controller desired body rates
including attitude feedback, not pure polynomial feedforward rates. Values in
the log block are not an atomic control-cycle snapshot. FC log timestamps wrap
at 24 bits; compare them with microsecond event clocks only after wrap/unit
alignment. Pi monotonic receipt times are retained separately from flush time.

Event records:

- `Pi Release Planner Evidence`: exact compact and expanded snapshot bytes,
  cached model bytes/CRC, returned plan bytes/summary and raw firmware result.
  Session, sequence, token and generation identify the transaction. These use
  existing packets, with bounded in-memory queues and no callback-thread disk IO.
- `Firmware Release Controller Targets`: target samples, startup/errors and
  explicit bounded-buffer overflow counts. Missing diagnostics do not abort.

Evidence is flushed by the existing interaction event logger, including fault
exit. Planning/USB callbacks only append bounded in-memory records. This does
not eliminate existing telemetry gaps or guarantee the complete physical
response can be reconstructed.

Validation on 2026-09-21: 93 focused tests passed. Read-only hardware check was
blocked: Pi .144 was reachable but `lsusb` listed only root hubs and `usb://0`
could not open. No arming, release, setpoint or parameter write was performed.
Actual target-log rate and full-load behavior remain unverified. No firmware
change is required for these existing telemetry variables.
