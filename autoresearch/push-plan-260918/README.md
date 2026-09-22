# Event-driven Pi planning — experimental pairing

Deployment update: firmware was subsequently flashed and read back under
explicit user authorization. See [DEPLOYMENT.md](DEPLOYMENT.md). The original
first-iteration verification record below is retained as historical evidence.

This iteration implements the transport and execution split, not a claim of
flight acceptance. No firmware flashing, remote push, Pi update or flight is
part of this iteration. Existing two_phase / zero_velocity modes stay intact.

## Select explicitly after installing the matching pair

In the existing mission's `Interaction.config.wrench_interaction` mapping:

```yaml
firmware_auto_brake:
  enabled: true
  mode: pi_joint
  response_time_s: 0.14  # retain the existing measured setup value
```

No launch-chain or XY PID change. Both existing position-only rigidbody and
single-marker pointcloud routes are supported. Full Vicon orientation remains
excluded in this pairing. Firmware requires pRelJVer >= 26091805 and the
hardware-brake/joint-unwind build options. pRelHost and pRelJoint default off;
offboard enables them only for pi_joint. Selecting a legacy mode clears them.

The Pi needs a working C compiler for first-time native planner compilation;
this happens with worker startup before arm. Startup/version failures refuse
arming instead of falling through to another controller. The planner uses a
separate prewarmed process, not the USB receive callback. It is closed when
the controller disconnects.

## Per-release flow

1. Pi primes an event listener then sends the existing release event.
2. FC takes command ownership and issues rapid braking immediately. It pushes
   one snapshot in 7 small packets, with bounded retransmissions.
3. Pi predicts continued rapid response to a future FC-relative start epoch,
   solves a two-piece seventh-order return, then sends 4 plan fragments and
   a commit. This adds no continuous high-rate state subscription.
4. FC validates identity, checksum, shape and timing, reconstructs polynomial
   coefficients with closed-form algebra, and starts at that fixed epoch.
   Heavy trajectory solving is absent from the FC host-mode path.
5. Curve completion enters one-shot current-position hold directly, with no
   intermediate zero-velocity cleanup. This deliberately does not certify that
   measured velocity/attitude/rates all met a shared stable terminal window.

Existing low-rate fault/hold monitoring remains. A success result means only
the plan was accepted, not that the vehicle has stabilized. A late/missing
plan triggers FC-local bounded return-to-level and a fault, never an invented
successful hold or indefinite large-tilt wait. Stale/invalid state safeguards
remain.
The fault is reported immediately; existing Pi emergency landing may take
over before the fallback level-return curve finishes.

## Timing and model limitations

- Default planning start allowance is 80 ms, capped by the FC's local latest
  unwind start. Low-speed releases may leave too little time: they must fail
  explicitly to the bounded local return, not extend the braking interval.
- Snapshot fields include state/attitude sample-age bounds. These are not a
  synchronized physical Vicon capture timestamp; data sources may differ in
  age. No Pi wall clock is subtracted from an FC clock.
- PID attitude gains/integral bias and response tau come from FC. The model
  tau parameters remain the preceding experimental firmware defaults until
  measured on the physical vehicle; `response_time_s` is not a replacement
  for those three angular-response parameters.
- This is one event snapshot/one plan, not continual measurement replanning.
- Known infeasible synthetic states are retained in `feasibility.jsonl`:
  2.7 m/s with actual CF pitch 0 or -20 deg, and 0.5 m/s with +20 deg,
  under the stated +22.18 deg rapid command / zero-rate / 80 ms prefix.
  These are not flight traces; they demonstrate that the current planner is
  not feasible over the entire release-state space. Do not describe this
  protocol milestone as a general flight-ready controller.
- Next validation: props-off real Pi compute/transport deadline test, then
  170 g simulation/controlled flight assessment. A successful build alone
  cannot establish no-reverse flight or terminal convergence.

Protocol: [PROTOCOL.md](PROTOCOL.md). First acceptance command:
`python3 autoresearch/push-plan-260918/verify.py`.

## Recorded first implementation acceptance

`verify.log`: 87 Python tests; production C protocol tests with ASan/UBSan;
independent production C -> spawned Pi process -> production C roundtrip;
matching Pi/FC numerical source bytes; Bolt build all passed. The held-out
roundtrip includes modular FC-clock wrap and reference-continuity attacks.
The measured desktop roundtrip is not a physical Pi-to-FC latency result.

Image: `crazyflie-firmware-master-post-release/build-post-release-pi-plan-260918/bolt.bin`

SHA256: `06b92adb014d009754d78ff44086a08127ab07bdde9bf5cbf67ee722c7870c89`

Firmware image was not flashed; offboard was not pushed or synced to Pi.
