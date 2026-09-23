# Bolt calibrated response execution — 26092304

This is actual hardware runtime integration, not the earlier parameter-only
26092301 build. No flashing or flight is performed by the verification script.

## Data path

Normal XYZ-only `--calibrate` saves the accepted roll/pitch delayed second-order
fit to `Interaction/attitude_response.json` with its PID/filter context. It does
not add twelve test pulses. A failed/latest incomplete calibration is not reused.
At next startup, `firmware_auto_brake.response_model.enabled: true` validates the
context, disables brake authority, verifies fresh `pRelSVer=26092304` and
`pRelAdVer=26092303`, uploads and commits the model, then enables the runtime.
The checks run again before arming. Calibration itself does not require an old fit.

The flight controller records the actual attitude PID targets at 500 Hz, including
hover/contact. Release latches the committed model and mode. Measured angle/rate,
velocity feedback and 150 ms of causal command history seed prediction. Initial
angular rate is not zeroed. The closed-loop model does not receive an additional
motor/PID inverse feedforward. Missing model/history does not select a simulation
plant or the legacy planner. With calibrated mode explicitly disabled, the old
S-curve path remains unchanged.

The existing strong directional brake and seventh-order unwind are executed in
the HLC. A priority-1 worker resolves the remaining impulse; stabilizer never runs
the numerical search. Results start 100 ms after their snapshot. Prediction first
advances the old plan through that prefix and shifts the delayed command history.
Until activation the old reference continues. Expired results are discarded;
release epochs prevent results from a prior interaction being applied. Replan
requests are at most 25 Hz, with one job/one scheduled result, so actual replanning
frequency is lower. Near the final tail, the accepted reference finishes without
another replan. At its deadline, fresh state permits immediate position hold at
current XY (release Z/yaw); no new velocity-threshold wait is introduced.

## Source and build

`changes.patch` applies to the exact **26092203 frozen paired Bolt snapshot**,
not an arbitrary clean upstream commit. Base directory used here:
`/Users/shuqinzhu/.codex/worktrees/2713/fls-cf-offboard-controller/autoresearch/scurve-distance-260922/firmware`.
Its upstream base is `0f2632ccb897c390c3029fd29761d840279a0dc2` plus preexisting
paired-controller changes. Do not overwrite the dirty physical firmware checkout.

Candidate source and build are under
`autoresearch/fix-260923-bolt-response/firmware`, with artifact
`build-calibrated/bolt.bin`. Build using the paired baseline Bolt `.config`:

```sh
make O=build-calibrated KBUILD_OUTPUT=build-calibrated olddefconfig
make O=build-calibrated KBUILD_OUTPUT=build-calibrated -j4
bash firmware/bolt-response/verify.sh /absolute/path/to/candidate/firmware
```

The verification script is run from the offboard checkout; the two make commands
are run inside the firmware snapshot. Native tests include the production worker
and runtime, not a separately implemented planner. They exercise calibrated
delay/angular-rate effects, repeated releases, timed current-position handoff,
missing-history refusal, late-worker-result rejection, transaction validation,
armed model immutability and timestamp wrap. Sanitizers are enabled.

## Explicit validation limits

- PID gains, PID estimator ownership and ordinary velocity estimation are not
  changed by this patch. Prediction selects the attitude output source used by
  calibration (`ordinary` or explicitly `post_release15`). This is not a port of
  15-state roll/pitch into PID.
- The diagonal body-axis response is projected into along/cross axes using
  squared direction weights; unequal roll/pitch dynamics at oblique directions
  are an approximation. Tilt-to-horizontal acceleration still uses gravity and
  tilt, not a full measured-thrust/6-DOF model.
- The native closed-loop cases use the same model family as prediction, not an
  independent Gazebo or physical plant. They demonstrate execution/consumption,
  not absence of real reverse flight or guaranteed zero terminal tilt.
- This new worker's MCU worst-case runtime and stack margin have **not** been
  measured on Bolt. `hlCommander.scWorkUs`, `scLate`, `scStack` (words), `scUs`
  and `scModel` expose worker duration, missed activations, stack low-water mark,
  control-path duration and the actually used model ID. Inspect them on hardware
  before accepting the new runtime for flight. Prior Bolt validation belongs to
  the prior build, not automatically this new worker.
- No push, Pi sync, flash, arming, Gazebo run or physical flight was done here.
