# Authorized deployment — 2026-09-18

User requested flashing while already in bootloader mode, then pushing the
matching offboard update to remote master. No arming, release injection or
flight was requested or performed. Onboard Pi checkout was not updated.

## Firmware

- Image: `build-post-release-pi-plan-260918/bolt.bin`, 327719 bytes.
- SHA256: `06b92adb014d009754d78ff44086a08127ab07bdde9bf5cbf67ee722c7870c89`.
- Local and transferred-image hashes matched.
- Official cfloader on `fls@192.168.1.90` completed STM32 flashing, exit 0.
- nRF softdevice was unchanged.
- Subsequent read-only `usb://0` check from onboard Pi `192.168.1.144`:
  `system.selftestPassed=1`, `hlCommander.pRelJVer=26091805`;
  `pRelHost=0`, `pRelJoint=0`, `pRelAuto=0`, `pRelMode=0`;
  no connection-lost callback. No parameters set, no setpoints, never armed.

## Offboard integration

Based on fetched remote master `0b91722`, preserving its yaw correction,
Vicon diagnostics and pointcloud support. Resolved overlapping interaction
telemetry changes explicitly, preserving the new compact logger, Pi planner
and the prior remote startup behavior. No mission YAML or XY PID gains changed.

Validation after integration: 105 Python tests passed; production C protocol
ASan/UBSan checks and independent FC-C/spawned-Pi-worker/FC-C roundtrip passed.
The numerical feasibility limitations in README.md remain unchanged.

Remote master publication is the requested next operation after this record.
Neither successful flashing nor unit/protocol tests establish flight readiness.
Use a props-off physical Pi/FC timing check before controlled flight.
