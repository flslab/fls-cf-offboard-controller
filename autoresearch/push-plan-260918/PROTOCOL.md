# Pi event planner protocol v1 (frozen implementation contract)

Scope: opt-in `firmware_auto_brake.mode: pi_joint`, existing modes unchanged.
New firmware params `hlCommander.pRelHost=1`, `pRelJoint=1`, `pRelMode=0`.
Required `hlCommander.pRelJVer >= 26091805`. Default host/joint modes OFF.
No flash, push or flight in this turn. Benchmark/timing and model coefficients
remain experimental, not hardware flight validation.

All messages use CRTP SETPOINT_HL. FC events/results channel 1, Pi inputs
channel 0. Opcodes 19 snapshot, 20 snapshot ACK, 21 plan chunk, 22 plan commit,
23 plan result (confirm no collision before implementation).

Chunk header little endian `<BBHIHBB>`: opcode, version=1, release sequence
u16, session u32, snapshot token u16, part index u8, part count u8. Data at
most 18 bytes (total packet <=30). Exact final length checked. Each body is
followed by CRC32 IEEE (zlib.crc32), little endian u32. Out-of-order and exact
duplicate chunks accepted; conflicting duplicate resets/rejects assembly.
Bodies cannot cross session/sequence/token. Incomplete or stale body cannot
affect actuation. No dynamic allocation on FC.

Snapshot body `<IIHH26f>` (116 bytes before CRC):
  snapshotUs, latestStartUs (both FC modulo32),
  trustedStateAgeUs, attitudeAgeUs (u16, conservative receipt/sample ages),
  velocityXY[2], anglesCFDeg[3], eulerRatesCFDegS[3], directionXY[2],
  holdYawDeg, integralRateBias[3], attitudeKp[3], attitudeKi[3],
  responseTauS[3], rapidBrakeAnglesCFDeg[3].
Angles use CF roll/-pitch convention; yaw angle is release-heading offset.
The FC captures one bundle AFTER rapid braking has actually been issued,
with fresh Vicon15 and IMU. The age fields preserve the bounded mismatch
between their sample epochs; these are NOT Vicon physical-capture timestamps.
Push once (bounded retransmission of same
snapshot, NOT a continuous subscription). Timestamp is measurement/request
epoch, never Pi wall clock. The FC continues rapid braking in the meantime.
If no valid plan before the local latest-unwind deadline, start the existing
bounded level return, mark host-plan failure and do not report successful hold.

Snapshot ACK `<BBHIH>` (10 bytes). ACK means snapshot received, not planned.

Plan body `<If12f>` (56 bytes before CRC):
  startDelayUs (relative to snapshotUs), durationS,
  initialReferenceAngles[3], initialReferenceRates[3], knotAngles[3],
  endReferenceAngles[3]. Initial acceleration/jerk are zero for this single
  first plan. FC rebuilds the two seventh-order Hermite pieces, 40%/60% split,
  with zero derivatives at the knot and end, using only closed-form algebra.
  New kernel API `jointBuildFromParameters(...)` exposes this reconstruction;
  it MUST NOT invoke jointSolve/jointPredict.

Pi predicts continued rapid attitude response from the snapshot to a future
start epoch (default 80 ms, <= latestStartUs), then runs the shared C jointSolve
kernel in a prewarmed independent process. The snapshot provides all gains
and state; no Gazebo truth and no extra log subscription are used.
The predicted physical state is distinct from the initial reference: preserve
the issued rapid angle command with zero reference rates at curve entry.
Propagate integral bias through the rapid prediction too. If the local FC
deadline leaves insufficient transport/compute lead, fail explicitly rather
than extending the large-tilt interval. Mixed sample ages and provisional
response-model coefficients remain experimental uncertainty.

Plan commit `<BBHIHI>` (14 bytes): opcode, version, release seq, session,
snapshot token, plan-body CRC32. Commit must arrive while still in rapid
brake, before start epoch (at least 5 ms lead), <= latestStartUs, and matching
this one release. Validate finite numbers, positive bounded durations and
sampled geometric feasibility; retain invalid-state/timeout protections.
Atomic schedule once, idempotent exact duplicate commit, reject conflicting
second plan. Do NOT shift the start epoch to late arrival. A ready future plan
is evaluated at that FC epoch; heavy jointSolve never runs in FC host mode.

Plan result `<BBHIHHI>` (16 bytes): opcode23, version, release seq, session,
snapshot token, errno u16, acceptedStartUs u32. Repeated matching commit gets
same result without restarting. Result receipt is not terminal success.

Successful execution: stage1 rapid -> stage2 curve -> stage4 one-shot current
position hold; no stage3/5, no ordinary-EKF terminal-speed gate. Existing low
rate fault/hold heartbeat stays; this feature adds NO periodic state logging.
All snapshot retries, host scheduling/compute/send times, FC accepted epoch
are diagnostic evidence. Never subtract Pi epoch from FC epoch.

Host API: PiEventPlanner(cf), start() (prewarm before arm), begin_release(session_id,
sequence) BEFORE sending release event, status() (JSON-safe diagnostic dict),
close(). Listener callback assembles/queues only; compute and send handling
are off the receive callback. One job per release, bounded retries/timeouts.
Store long-lived service at cf._post_release_pi_planner; existing Interaction
release path selects it only for mode pi_joint.
