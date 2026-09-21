# Paired bounded-return candidate (not deployed)

Scope: `pi_joint` only. An accepted attitude return may finish through a short
Vicon readiness outage. This does not repair source-frame timing or prove flight
stability. Do not interpret successful command execution as a stable endpoint.

- Pi permits readiness loss past 300 ms only with fresh stage-2 telemetry and a
  confirmed accepted/executing plan. The deadline is the recorded first snapshot
  receipt plus the plan start delay and duration, plus 300 ms recovery; polling
  does not renew it. Plans extending beyond five seconds from monitor start
  cannot grant this exception. This Pi deadline is a watchdog approximation,
  not a cross-clock synchronization estimate.
- FC allows an existing local return to its fixed end, or an accepted active
  return to its fixed end plus 300 ms. It additionally bounds the exception to
  five seconds from release. Missing IMU/attitude, controller mismatch, aborts,
  packet loss and no usable plan keep their independent fault paths.
- Fresh finite state is still required for the existing direct position handoff.
  No stale-state replanning or stale-state position hold is authorized.
- Both stage 1 and stage 2 use `mode.z=modeAbs` and the release-time height;
  roll/pitch use absolute attitude (plus rate feedforward in stage 2). The PID
  height loop computes thrust. No XY PID gains or manual thrust are introduced.
  A fixed height target does not guarantee actual altitude remains fixed when
  localization is stale or thrust is saturated.

Validation: focused Python monitor/planner/preflight tests and production-extracted
C HLC branches. Includes fixed deadline, no indefinite renewal, recovery, rejected
plans, telemetry loss, rapid-stage exclusion, stale attitude and fixed-Z assertions.
These are not full dynamics or hardware tests. Firmware and offboard must be
deployed together; neither was pushed/flashed as part of this change.

Outstanding: source-frame backlog identification and source-based KF time steps,
actual loaded Pi timing, and prop-off/in-flight validation. Dense arrival-time
bursts must not be described as validated fresh physical samples.
