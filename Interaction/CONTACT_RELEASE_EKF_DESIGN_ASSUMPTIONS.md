# EKF Design Assumptions for Contact–Release Flight

## Crazyflie Onboard EKF (9-State)

- **State:** world position, body-frame velocity, and attitude error.
- Assumes normal free flight with calibrated IMU biases.
- Uses a thrust/drag-based flight model and mainly body-Z acceleration during flight.
- Runs continuously at approximately 100 Hz for real-time control.
- External position corrects velocity and attitude through cross-covariance; full pose can directly correct attitude.

## Contact–Release Shadow EKF (15-State)

- **State:** world position, world velocity, attitude error, gyro bias, and accelerometer bias.
- During contact, acceleration is contaminated by unknown contact forces; attitude therefore propagates from gyro only.
- At release, position and velocity are seeded from the onboard EKF, while attitude and gyro bias come from the contact observer.
- After release, measured 3-axis body specific force is rotated into the world frame by the current estimated attitude, gravity is added, and that realized acceleration propagates position and velocity:

  `p_dot = v`, `v_dot = R(q) (f_m - b_a) + g`, `q_dot = q ⊗ (ω_m - b_g)`.

- The process model has no command, setpoint, requested acceleration, prior-command history, thrust reconstruction, or odometry-velocity input.
- Position-only Vicon updates indirectly correct velocity, attitude, and IMU biases through cross-covariance.

## Why It Fits This Scenario

- Contact forces violate the onboard free-flight thrust/drag assumptions.
- Gyro-only propagation prevents contact acceleration from being mistaken for gravity and corrupting roll/pitch.
- After separation, direct IMU propagation represents the realized motion rather than the commanded motion.
- Bias states reduce short-term inertial drift, while withheld Vicon orientation remains an evaluation signal rather than an estimator input. It is not sensor-independent because position and orientation share the same motion-capture system.
- The estimator object is **shadow-owned**: it has no commander and cannot authorize a setpoint. A separate, default-disabled adapter may expose one immutable same-epoch state candidate to the braking controller only after the release, timestamp, covariance, observability, calibration, state-freshness, and swept-envelope gates all pass.

## Observability and control boundary

- Gravity supplies direct accelerometer information about roll and pitch. It does not directly observe yaw; yaw therefore has an absolute covariance limit but is not required to show artificial short-window variance reduction.
- Position innovations can correct attitude only after inertial propagation has created position/attitude cross-covariance. The live eligibility gate therefore requires a minimum post-release device-time span, IMU propagation count, and strict position-update span rather than accepting a few same-instant updates.
- A release candidate is prepared at the first unloaded sample and must finish strictly before a later dwell-confirmation sample. Confirmation cannot replace or omit any frozen release identity or clock evidence.
- Until interval-aware release propagation exists, the control gate requires `mapped release epoch == release gyro epoch == atomic p/v seed epoch` and exactly zero mapping uncertainty. A merely small skew is diagnostic evidence, not control evidence.
- Clock-source names are split by purpose: Arduino and CrazySim mappings may support diagnostics, while the authority gate accepts only the future `firmware_shared_clock_release_latch_v1` basis. No current runtime can produce that basis from the independent Arduino clock.
- The producer-latched IMU source epoch must precede or equal its CRTP transport epoch by `0..5 ms`. The older `±100 ms` reconstruction window is never accepted as authority evidence.
- A certified absolute-yaw value and certificate ID must be bound to the yaw actually used for alignment. Body-rate uncertainty must contain three strictly positive standard deviations and name the same joint IMU calibration artifact as the IMU range/step evidence.
- Raw and unwrapped Crazyflie epochs, host receive age, release dwell identity, full covariance symmetry/PSD, and all uncertainty bounds are checked fail-closed. Position observations require exact-zero capture-to-Crazyflie timestamp uncertainty because motion-dependent measurement-noise inflation is not implemented.
- Producer-to-transport timing and callback age do not by themselves prove CRTP transport-to-host latency. Production enablement therefore also needs a continuously checked Crazyflie clock/boot identity, a host-stall recovery dwell, and a rule that cached pre-stall packets cannot regain authority merely by arriving at a fresh callback time.
- The current process noise, position noise, initial covariance, and minimum variance-reduction thresholds are simulation defaults, not calibrated hardware evidence. A production configuration must load them from an immutable artifact bound to the vehicle, sensor/rig, firmware, boot/session, frames/extrinsics, and a content hash; the yaw certificate must also contribute its measured uncertainty.
- Short-window covariance shrinkage is not proof of physical observability. Hardware shadow data must set meaningful excitation and relative-reduction thresholds for tilt, gyro bias, and accelerometer bias before those quantities can support control.
- An eligible estimator state is still not command authority. The actual send path separately requires a current planar braking fit, synchronized state, a pre-command XYZ swept-envelope certificate, and a one-cycle authorization token.

**Key limitation:** Position-only aiding can weakly observe attitude and biases during short or poorly excited motion, and accurate IMU–Vicon timing remains essential. The current hardware does not provide an exact shared release epoch, exact-zero Vicon capture timing, continuous host-latency proof, or the required named yaw/IMU/body-rate/EKF-noise calibrations. The active estimator-control path is therefore intentionally blocked before flight even though the local shadow/simulation implementation is complete.
