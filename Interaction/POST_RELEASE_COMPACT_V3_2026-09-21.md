# Compact post-release transport v3

Local candidate only. No push, Pi update, firmware flash, arming, or hardware
experiment was performed for this change. The preceding physical 8.011 ms
measurement belongs to v2, not this candidate.

## Wire changes

| Direction | v2 | v3 |
|---|---:|---:|
| FC snapshot body + CRC | 124 + 4 bytes / 8 fragments | 92 + 4 bytes / 6 fragments |
| Pi plan body + CRC | 56 + 4 bytes / 4 fragments | 32 + 4 bytes / 2 fragments |
| Total, including 12-byte fragment headers, excluding result | 332 bytes / 12 packets | 228 bytes / 8 packets |

This removes 4 critical-path packets (33%) and 104 application-payload bytes
(31%). CRTP/USB overhead is not included. No proportional latency claim.
All remaining floating-point values are still float32; no quantization.

### Startup model cache

Opcode24 request is `<BBI>` (opcode, v3, random nonce). FC opcode25 replies use
the existing fragmented envelope (session=nonce, sequence=token=0), carrying
`<9f>`: current attitude Kp[3], Ki[3], response tau[3], plus CRC32. Three reply
packets, outside the release path. The read does not enable any mode or write
any PID parameter. Requests retry at most six times within three seconds.
Missing/conflicting/corrupt replies prevent planner readiness.

The Pi worker is prewarmed and model is synchronized during startup; final
pre-arm verification reads it again after configuration. This is a bounded
startup transaction, not a recurring log subscription. Parameter values come
from FC's active model, not a second hardcoded host copy.

### Dynamic snapshot

Wire `<IIHHI19f>`: FC epoch/latest start, state/attitude ages, model CRC32 id,
then velocityXY, actual angles/rates, release directionXY, release yaw,
live integral bias[3], issued initial reference[3], local-prefix elapsed and
duration. The internal numerical snapshot is unchanged (124 bytes, 28 floats).
Pi expands it using the matching cached model before passing it to the native
solver. Unknown/changed model id does not reach the worker. FC also compares
the live model to the captured model when accepting returned plan fragments.
No gain changes, background resynchronization, or stale-model fallback occur
on the release critical path.

Release direction is retained: deriving it from a later snapshot's current
velocity could incorrectly change the no-reverse direction after a prefix
transition. Integral output is dynamic and must not be cached with Kp/Ki.

### Compact plan

Wire `<If6f>`: start delay, duration, knot[3], end-reference[3], plus CRC32.
The redundant initial-reference and rate arrays are omitted. FC reconstructs
all four boundary derivatives from the exact immutable prefix/token and FC
join epoch. Existing duration/angle/rate/acceleration checks still apply.
Session/sequence/token, CRC, deadline, duplicate/conflict checks remain.

No changes to native numerical solver, XY PID gains, control rates, 40 ms join
epoch, local fallback policy, terminal thresholds, or position-hold behavior.

## Validation

- 90 focused Python tests passed, including cache mismatch/missing model,
  stale nonce, bad model CRC, float32 tau boundaries, bounded startup timeout,
  pre-arm refresh failure, old worker results and duplicate handling.
- Production C ASan/UBSan protocol tests passed for compact snapshot/plan,
  CRC/conflicts, reorder/missing parts, wrap, and reference C3 reconstruction.
- Actual FC C serialization/model-reply/reconstruction and real spawned Pi
  process passed the independent cross-language test, including superseded
  local-prefix token. Printed 40.123/40.234 ms are virtual-clock fixtures.
- Extracted production HLC tests passed, including read-only model requests
  with host mode disabled and model-change rejection before scheduling.
- Bolt build passed; both source trees pass `git diff --check`.

## Paired candidate

Firmware build id **26092101**, protocol **3**. Offboard preflight requires
this exact firmware id. Do not mix v2/v3. Existing deployed v2 is untouched.

Image:
`/Users/shuqinzhu/Documents/FLS_Research/crazyflie-firmware-master-post-release/build-post-release-compact-260921/bolt.bin`

SHA256 `620c22752d0780b8c6cf1fb3810405404487121beb0dcd2fa58b777299d6cfd6`.
Image333696 bytes; linker flash333240, RAM106528, CCM55144 bytes.
Build configuration is identical to the v2 candidate configuration.

Next authorized hardware experiment must update both firmware/offboard, then
repeat the same prop-off synthetic-input acceptance benchmark to compare
FC-clock snapshot-to-plan time. It remains a transport test, not flight proof.
