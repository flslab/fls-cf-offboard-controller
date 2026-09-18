#pragma once

#include <stdbool.h>
#include <stdint.h>

// Two-phase post-release stop primitives. Direction is the measured
// world-frame direction of travel, not a user-supplied terminal velocity.
bool postReleaseRapidBrakeAttitude(
  const float directionXY[2], float yawDeg, float decelerationMps2,
  float* rollDeg, float* pitchDeg);

// Switch while the actual braking attitude still has enough residual impulse
// to remove current forward speed during a seventh-order return to level.
// responseTimeS is a measured/calibrated actuator tail, not a safety bypass.
bool postReleaseShouldStartUnwind(
  const float velocityXY[2], const float directionXY[2],
  float rollDeg, float pitchDeg, float yawDeg,
  float unwindDurationS, float responseTimeS,
  float forwardSpeedMarginMps);

// Seventh-order smoothstep with zero first, second and third endpoint
// derivatives; 1 at the start of unwind and 0 at level completion.
float postReleaseSepticUnwindWeight(float normalizedTime);

// Seventh-order attitude reference, in Crazyflie's roll/-pitch convention.
// anglesDeg[2] is an offset from the release/hold yaw, not world yaw. Its
// terminal value is zero, so the vehicle retains the release heading.
// Unlike a smoothstep it preserves nonzero initial angular velocity and, on
// replanning, the previous reference acceleration and jerk as well.
typedef struct {
  float coefficient[3][8];
  float durationS;
} postReleaseUnwindProfile_t;

// Raw gyro input is right-handed body (p,q,r), in deg/s. Euler output is
// (rollDot, pitchDot, yawDot) in the CF convention; pitchDot=-thetaDot.
// The inverse outputs PID-controller body (p,-q,r), matching its gyro sign.
// Both conversions exclude Euler singularities (|roll|, |pitch| > 85 deg).
bool postReleaseEulerRatesFromBody(float rollDeg, float pitchDeg,
  const float rawBodyRatesDegS[3], float eulerRatesDegS[3]);
bool postReleaseBodyRatesFromEuler(float rollDeg, float pitchDeg,
  const float eulerRatesDegS[3], float controllerRatesDegS[3]);
// Plan/evaluate accepts durations in [0.05,1.0] s. Coefficients use normalized
// time s=t/T; derivatives returned by Evaluate are in physical seconds.
// At t>=T all four terminal derivatives/angles are exactly zero. Negative or
// nonfinite elapsed time is invalid. A false return leaves output unchanged.
bool postReleaseUnwindPlan(const float anglesDeg[3],
  const float ratesDegS[3], const float accelerationsDegS2[3],
  const float jerksDegS3[3], float durationS,
  postReleaseUnwindProfile_t* profile);
bool postReleaseUnwindEvaluate(const postReleaseUnwindProfile_t* profile,
  float elapsedS, float anglesDeg[3], float ratesDegS[3],
  float accelerationsDegS2[3], float jerksDegS3[3]);
// Sampled numerical/geometric screen: finite derivatives, |roll|/|pitch|
// <=35 deg and |yaw offset|<=45 deg at 33 times. This is not an actuator,
// inertia, tracking-error, or no-reversal guarantee between/after samples.
bool postReleaseUnwindFeasible(const postReleaseUnwindProfile_t* profile);
// Signed velocity removed along directionXY, integrated using 24 midpoints
// and the nonlinear thrust-direction ratio under approximately held altitude.
// holdYawDeg + profile yaw offset gives world yaw. Negative means the
// reference accelerates along the release direction. No tail/reserve here.
bool postReleaseUnwindRemainingImpulse(
  const postReleaseUnwindProfile_t* profile, float elapsedS,
  float holdYawDeg, const float directionXY[2], float* impulseMps);
// Try shortest-first T={.20,.25,.30,.35,.40}s with initial a=j=0, preserving
// initial angle/rate. Select only when v_forward <= curve+tail impulse+reserve.
// responseTimeS in [0,.25] adds an angle-clipped, constant-current-rate tail;
// it is a bounded approximation, not an identified angular-inertia model.
// predictedImpulseMps includes that tail but excludes speedReserveMps.
// Distinguish no feasible candidate/invalid inputs from an ordinary wait for
// lower speed. Both non-ready outcomes leave profile/impulse unchanged.
typedef enum {
  PostReleaseUnwindInfeasible = -1,
  PostReleaseUnwindWaiting = 0,
  PostReleaseUnwindReady = 1,
} postReleaseUnwindSelection_t;
postReleaseUnwindSelection_t postReleaseRateAwareUnwindSelect(const float velocityXY[2],
  const float directionXY[2], const float anglesDeg[3],
  const float eulerRatesDegS[3], float holdYawDeg, float responseTimeS,
  float speedReserveMps, postReleaseUnwindProfile_t* profile,
  float* predictedImpulseMps);

// Select a bounded unwind horizon from the current forward speed, measured
// braking attitude and calibrated response tail. Returns false when no
// supported horizon has enough predicted impulse; the caller keeps braking.
bool postReleaseSelectUnwindDuration(
  const float velocityXY[2], const float directionXY[2],
  float rollDeg, float pitchDeg, float yawDeg, float responseTimeS,
  float forwardSpeedMarginMps, float* durationS);

// Latest open-loop switch after a trusted speed sample. This is a bound on
// *remaining* rapid-brake time, not permission to call a stale state fresh.
// The speed consumed by the shortest seventh-order return, measured actuator
// tail and existing margin is reserved before scheduling that switch.
bool postReleaseBlindBrakeLatestDelay(
  float forwardSpeedMps, float maxBrakingAccelerationMps2,
  float responseTimeS, float forwardSpeedMarginMps,
  float* delayS);

// A release during a Vicon gap is permitted only with a fresh, finite,
// independently validated state that still meets the minimum forward-speed,
// vertical-motion and height bounds after uncertainty is accounted for.
bool postReleaseBlindReleaseStateSafe(
  const float positionM[3], const float velocityMps[3],
  uint32_t trustedStateAgeUs);

typedef enum {
  PostReleaseBlindContinue = 0,
  PostReleaseBlindStartUnwind,
  PostReleaseBlindAbort,
} postReleaseBlindDecision_t;

// The short-loss policy applies only to an accepted firmware brake. Stage 1
// is rapid attitude; stage 2 is the running seventh-order unwind. Stages 3/5
// receive the same <300ms grace, but the HLC caller must command level during
// that gap, never use stale velocity PID feedback or declare terminal hold.
postReleaseBlindDecision_t postReleaseBlindBrakeDecision(
  uint8_t stage, uint32_t nowUs, uint32_t lastTrustedUs,
  uint32_t latestUnwindUs, uint32_t releaseStartUs,
  uint32_t rapidSetpointCount);

// Simulation-only position reference for a seventh-order, zero-jerk stop.
// The caller must supply a validated, current world-frame velocity. This
// module neither grants authority nor changes the ordinary HLC/PID path.
bool postReleaseForwardStopTarget(
  const float positionM[3], const float velocityMps[3], float durationS,
  float maxDecelerationMps2, float targetM[3], float directionXY[2]);

// Outer position feedback on the reference trajectory. The inner firmware
// velocity PID remains in charge of attitude. No negative velocity is issued
// along the original travel direction while the stop is active.
bool postReleaseForwardStopFeedback(
  const float referencePositionXY[2], const float referenceVelocityXY[2],
  const float measuredPositionXY[2], const float directionXY[2],
  float commandVelocityXY[2]);

// Simulation-only residual-tilt diagnostic: after the polynomial ends, keep a
// bounded *forward* velocity target until the attitude unloads. The same
// projection is used by the low-speed profile guard below. A zero response
// time disables both diagnostics.
bool postReleaseForwardStopTailGuard(
  const float measuredVelocityXY[2], const float directionXY[2],
  float rollDeg, float pitchDeg, float yawDeg, float responseTimeS,
  float commandVelocityXY[2]);

// During the running profile, a verified residual braking tilt may consume
// the remaining forward speed before the velocity loop can unload it. Add a
// bounded forward-only compensation that tapers away at normal speeds.
bool postReleaseForwardStopProfileGuard(
  const float baseCommandVelocityXY[2], const float measuredVelocityXY[2],
  const float directionXY[2], float rollDeg, float pitchDeg, float yawDeg,
  float responseTimeS, float commandVelocityXY[2]);

// The terminal goal is fixed: zero world-frame XY velocity and level
// roll/pitch. A measured tolerance is necessary to decide when the vehicle
// has settled; these are not user-supplied terminal targets.
typedef struct {
  uint32_t previousTimeUs;
  uint32_t stableSinceUs;
  float previousRollDeg;
  float previousPitchDeg;
  bool hasPrevious;
  bool hasStableWindow;
} postReleaseForwardStopTerminalState_t;

void postReleaseForwardStopTerminalReset(
  postReleaseForwardStopTerminalState_t* terminal);

// Require the speed, level attitude, and attitude-rate windows to overlap
// continuously for 200 ms. Samples must arrive at the HLC's 100 Hz cadence.
// The caller must independently validate estimator freshness/authority.
bool postReleaseForwardStopTerminalUpdate(
  postReleaseForwardStopTerminalState_t* terminal, uint32_t nowUs,
  float ordinarySpeedMps, float verifiedSpeedMps,
  float rollDeg, float pitchDeg);

// Hardware path: use a fresh IMU gyro sample, including yaw rate, rather
// than differentiated attitude, while retaining the same stable dwell.
bool postReleaseForwardStopTerminalUpdateWithBodyRates(
  postReleaseForwardStopTerminalState_t* terminal, uint32_t nowUs,
  float ordinarySpeedMps, float verifiedSpeedMps,
  float rollDeg, float pitchDeg, const float bodyRatesDegPerS[3]);
