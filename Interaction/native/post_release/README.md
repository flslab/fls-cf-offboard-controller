# Event-only Pi joint planner numerical kernel

The four `post_release_*.[ch]` files are vendored, byte-for-byte copies of the
paired Crazyflie firmware numerical kernel as of 2026-09-18 (`pRelJVer=26091805`).
They permit standalone offboard checkout / Pi startup without a firmware
checkout. Keep both copies synchronized when changing the model or polynomial.
The unit test compares them to `FLS_JOINT_FW`, or the local paired firmware
checkout when present; portable installations retain the pinned-hash check.

| File | SHA256 |
| --- | --- |
| post_release_joint_unwind.c | f8856df43f0bac01536fb46f700b4df3001da152891e249a4b879c1e4660f8e1 |
| post_release_joint_unwind.h | 0d1b47f69c5df252df632f30779d4ba1be56f7f32fb37986bff7e16fb1bc74dd |
| post_release_forward_stop.c | 2a2cfec556ec32d0eacab4f7f6b9e45fe21b439fe5002481ab04ccd4efa0303c |
| post_release_forward_stop.h | 6171bf8050d74d1f2f4315d09e2707287b276a1b87caf0a068e2fc95704460f9 |

`pi_joint_bridge.c` predicts the continued rapid-brake response to the future
FC-relative start epoch and invokes `jointSolve`. It advances actual angle,
rate, world velocity, and attitude-PID integral bias. Curve initial reference
is the **issued rapid command**, not the predicted actual angle/rate. No PID
gains are changed. Candidate durations are bounded (0.45–1.6 s).

The receive callback never compiles, plans, or sends. A prewarmed spawn process
loads an `-O2` native library built with `cc`/`gcc`; compilation failure prevents
arming this mode. A service thread sends event ACKs / plan messages, with no
state lock held across transport or process I/O. The existing low-rate fault /
hold heartbeat remains the authority for completion; plan acceptance is not
flight success.

The FC's two age fields describe receipt/observer age, not exact Vicon physical
capture time. The predictor starts from those bounded but not perfectly
simultaneous observations. Its response model is provisional; finite simulation
predictions do not prove actual flight tracking, no reversal, or joint terminal
convergence. No Pi-to-FC clock synchronization is assumed. Late plans are
rejected, never re-epoch'd; the FC's own bounded fallback remains active.

Wire protocol: `autoresearch/push-plan-260918/PROTOCOL.md`.
