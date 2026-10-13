# Bounded attitude feedback continuity

Image: `bolt-eskfxy-attfix-26101201.bin`  
Tag: `bolt-eskfxy-attfix-26101201`  
Base: the paired `estimator_xy_20261009_v2` Bolt firmware.

The old estimator publisher could be preempted while its generation was odd.
The higher-priority stabilizer then exhausted all three read retries before
the estimator could resume. One such failure latched brake stage 6/reason 3.
The deterministic test reproduces this failure in the original production
functions. This proves the code defect; the October 12 flight did not log
`pRelFault` and therefore does not prove that it caused that particular abort.

The publisher now prepares its snapshot in private static storage, then commits
the publication and last-good attitude in a short critical section containing
only bounded copies and metadata updates. The attitude getter copies a coherent
snapshot under the same protection. No filter calculation runs in that section.

For an invalid publication in the same active estimator epoch, the getter may
reuse the last valid attitude and body rates. Their original acquisition times
remain unchanged: both must still be at most **20 ms old**. Repeated reads never
extend the deadline. New valid feedback takes over immediately. Disable,
deactivation and a changed seed clear the cache. Startup warmup, position-count,
non-finite-value and quaternion-normalization checks remain in place. A current
snapshot marked valid but containing corrupt numerical data is rejected, not
hidden by the cache. Controller mismatch still aborts. No fault latch is cleared
automatically, and no blind continuation or ordinary-EKF substitution is added.

The private staging buffer avoids increasing the estimator task's stack usage.
The ARM link reports 9,728 bytes free RAM and 10,392 bytes free CCM. Critical
section duration and flight behavior have not yet been measured on hardware.

Diagnostics: `pRelVicon.attHeld` indicates successful cached use on the last
getter completion; `pRelVicon.attHoldN` counts successful cached reads (not
distinct dropouts). Bit 5 of attitude diagnostic `flags` marks successful reuse.
Existing `pRelFault` fields retain the detailed reason for any eventual abort.
These are available firmware logs; this patch does not increase offboard radio
logging traffic.

## Validation

- Hardware ARM and paired SITL compilation passed.
- The fault-injection harness compiles the actual production publisher/getter
  with AddressSanitizer and UndefinedBehaviorSanitizer. It covers preemption,
  transient invalidity, the 20 ms boundary, persistent failure, recovery, changed
  epoch, inactive/disabled state, warmup, missing position observations, invalid
  quaternion/body rates, independent IMU age, future timestamps and clock wrap.
- The compiled ARM ELF passes the active S-curve/XY mission parameter contract.
- All 88 selected offboard regression tests passed, including image integrity,
  preflight checks, brake events and reinteraction.
- Flashed to lb11 through Pi `192.168.1.90`; startup tag, CPU identity,
  all 47 mission parameters, new diagnostic fields, self-test and disarmed/zero
  motor outputs verified. No new flight validation has been performed.
- The initial readback script used the wrong diagnostic group name. It was
  corrected to `pRelVicon`; the final readback passed. The startup banner was
  captured on the first connection and the CPU identity checked again later.

Run the native harness against an installed source tree:

```sh
python3 validate_feedback.py /path/to/patched/firmware
```

To reproduce the old race, use the original paired source with
`--expect-preemption-failure`.

## Rebuild and deploy

Apply `patches/feedback.patch` to the frozen v2 hardware and SITL source trees.
`patches/source.json` identifies the exact estimator file before and after.
Set the tag in each tree's `snapshot_version.json` to the tag above, preserving
its base revision. For ARM, keep `bolt.config` and build with `O=build-unified`
and `KBUILD_OUTPUT` set to that build directory's absolute path. The source
fingerprints and packaged image hashes are recorded in `build-manifest.json`.

The offboard YAML and `specific_force` settings need no change. The fix only
takes effect after flashing this image to the aircraft and verifying its tag.
Use the established Bolt flashing procedure from the v2 release; substitute
this binary. Do not run a firmware update during an active flight.
