# Bolt estimator-3 XY firmware — 2026-10-09

**Superseded. Do not flash this image with the current S-curve profile.**
It lacks `hlCommander.pRelCompP` and `hlCommander.pRelFric`, which that profile
requires. Use [the corrected paired image](../estimator_xy_20261009_v2/README.md).
The old binary and build evidence remain here for diagnosis.

Build tag: `bolt-state-matched-26092502-eskfxy-261009`.
The binary adds the `eskfXY` API v1 used by the opt-in live hover correction.
It is built for Bolt/BMI088 from the paired `classic-260925-0959/hardware`
snapshot, with the existing S-curve and estimator-3 implementation retained.

`hardware/estimator_xy.patch` and `firmware/estimator_xy.patch` retain the
hardware and SITL integration changes. Their `source.json` files identify the
before/after source and header fingerprints. The matching shared header and
integration installer are in `Interaction/native_estimator3/estimator_xy_calibration.h`
and `Interaction/install_estimator_xy_firmware.py`. Rebuilding requires the
paired frozen base source; applying these patches to a different firmware tree
is not a verified build route.

`build-manifest.json` is the original build-time snapshot, including source
hashes and binary SHA-256. Its offboard hashes predate the saved-fit reuse
change; `cache-verification.json` records that later offboard-only change.
`verification.json` records the build, tests and offline comparison available
at image creation. Hardware and SITL compilation passed. This image has not
been flashed or validated in a new real flight by this workflow.

## Flash lb11 through the existing Radio Pi

Stop the mission/aircraft clients, remove props and leave lb11 powered.
The manifest maps lb11 to `radio://0/100/2M/E7E7E7E706`.
The Radio Pi at `192.168.1.90` already has cfloader in `/home/fls/env`.

On the Mac, from the offboard repository:

```bash
scp Interaction/firmware_images/estimator_xy_20261009/bolt-eskfxy-261009.bin \
  fls@192.168.1.90:/home/fls/bolt-eskfxy-261009.bin
ssh fls@192.168.1.90
```

On the Radio Pi:

```bash
/home/fls/env/bin/python -m cfloader \
  -w radio://0/100/2M/E7E7E7E706 \
  flash /home/fls/bolt-eskfxy-261009.bin stm32-fw
```

This explicitly targets STM32 firmware. A successful cfloader operation resets
back into firmware. Update the aircraft Pi offboard checkout separately:

```bash
cd /home/fls/fls-cf-offboard-controller
git pull --ff-only
```

Only after flashing the paired firmware, enable in the existing level-coast
S-curve mission:

```yaml
estimator_hover_xy: true
estimator_hover_xy_mode: auto
```

The first accepted hover fit is saved beside `fit/candidate.json` as
`fit/hover_xy.json`; later flights load matching saved coefficients and wait
for a fresh firmware acknowledgement plus 300 ms propagation. A missing or
invalid saved fit in `auto` triggers 2 s settling + 3 s sampling. Use `refresh`
to recollect deliberately, or `reuse` to require a valid saved fit. The static
candidate remains unchanged, and no gyro zero is cached here. The correction
uses default estimator 2 as a relative reference, not independent attitude truth.
