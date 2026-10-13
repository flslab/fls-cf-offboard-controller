# Corrected paired Bolt estimator-3 XY firmware

Tag: `bolt-distance-26092803-eskfxy-26100902`.
Image: `bolt-distance-eskfxy-26100902.bin`.

This supersedes `estimator_xy_20261009`, whose older curve baseline lacked
`hlCommander.pRelCompP` and `hlCommander.pRelFric`. The corrected image uses
the previously deployed `classic-260928-distance` Bolt base, retaining its
P/V/A/J tracker, friction, requested-distance and curve protocol 4 interfaces,
and adds the same frozen `eskfXY` API v1. The estimator kernels are unchanged.
The current mission keeps friction coupling disabled.

Hardware and paired SITL compilation passed. All 165 relevant offboard tests
and native distance/curve regressions passed. Before flashing, the actual ELF
parameter tables were checked against the resolved S-curve mission and XY
loading requirements: all 47 required parameters exist. The older image
reproduces exactly the two missing parameters. `build-manifest.json` binds the
binary, compressed ELF, configuration, changed sources and parameter table.
`patches/` preserves the XY changes relative to this paired base. Rebuilding
requires that frozen base source. `sitl-overlay-files.txt` records the shared
distance-runtime changes added to the simulator while keeping its platform
adapters. No new physical flight validation is claimed by these checks.

Recheck the shipped ELF against a mission before deploying:

```bash
venv/bin/python -B -m Interaction.firmware_image_contract \
  Interaction/firmware_images/estimator_xy_20261009_v2/bolt.elf.gz \
  /absolute/path/to/mission.yaml
```

Flash lb11 through the existing Radio Pi after stopping aircraft clients and
removing props. Keep lb11 powered. From the Mac offboard checkout:

```bash
scp Interaction/firmware_images/estimator_xy_20261009_v2/bolt-distance-eskfxy-26100902.bin \
  fls@192.168.1.90:/home/fls/bolt-distance-eskfxy-26100902.bin
ssh fls@192.168.1.90
```

On Radio Pi:

```bash
/home/fls/env/bin/python -m cfloader -w radio://0/100/2M/E7E7E7E706 \
  flash /home/fls/bolt-distance-eskfxy-26100902.bin stm32-fw
```

After reset, verify the application tag, `eskfXY.api=1`, `pRelCompP`, `pRelFric`
and the complete mission parameter contract before using the ordinary
`orchestrator.py --interaction` launch. Keep `estimator_hover_xy: true`,
`estimator_hover_xy_mode: auto` and `estimator_switch_at: release` in the
existing level-coast S-curve mission. First successful fitting saves the
compensation; later matching flights reload it with a fresh firmware ACK.
