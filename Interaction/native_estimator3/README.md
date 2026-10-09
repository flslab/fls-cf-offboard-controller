# Offline estimator 3 C kernel

The six firmware source/header files are unmodified copies from the frozen
`classic-260925-0959` firmware research snapshot. `manifest.json` pins their
SHA-256 hashes; the build refuses modified copies. This snapshot is not proven
identical to the currently flashed aircraft firmware.

`bridge.c` adapts the actual 15-state ESKF and correlated Vicon position/velocity
frontend for serial ctypes replay. It has no radio, commander or flight API.
Native filter/static scratch state is shared: raw and corrected runs must be
serial and are independently reinitialized.

The bridge raises the accepted IMU gap from 5 ms to 20 ms for the 100 Hz log
stream. It does not synthesize 1 kHz samples. Log ticks are not atomic IMU
capture timestamps; this is a log-rate comparison, not bit-exact flight replay.
Seed gyro is the actual first processed sample; ESKF gyro/accel bias states
start at zero and estimate residuals after any offline affine correction.

Upstream Bitcraze headers retain their copyright notices; the included firmware
license is GPLv3. Exact source/bridge hashes and compiler command are copied
into every replay's `build.json`.
