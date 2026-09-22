#pragma once

#include "post_release_joint_unwind.h"

// Two fixed rollouts and one held-control step: no shooting/optimization and
// no allocation. This is a response-model switch predictor, NOT a measured
// stability test, a freshness check, or authority to hand over to position.
#ifndef POST_RELEASE_LOCAL_UNWIND_STEPS
#define POST_RELEASE_LOCAL_UNWIND_STEPS 63u
#endif
#define POST_RELEASE_LOCAL_UNWIND_MAX_STEPS (2u * POST_RELEASE_LOCAL_UNWIND_STEPS + 1u)

typedef struct {
  bool startUnwind;
  postReleaseUnwindProfile_t profile;
  jointState_t terminalNow;
  jointState_t terminalAfterTick;
  float minForwardNow, endForwardNow;
  float minForwardAfterTick, endForwardAfterTick;
  float accelerationAlong, jerkAlong;
  float predictionHorizonS;
  uint16_t integrationSteps;
} postReleaseLocalUnwindDecision_t;

// Input angle/rate and reference use CF roll/-pitch and yaw OFFSET from
// holdYawDeg. Rates are Euler derivatives, NOT raw body gyro. World XY
// direction must be normalized. The current issued reference and its first
// three derivatives are preserved at the returned profile start (C3).
//
// Compare unwinding now with continuing that reference for one control tick
// and then unwinding. Both predictions integrate actual attitude/rate response
// and evolving PID I, through the unwind and a bounded level tail of
// min(.60 s, 3*max(tau,1/Kp)). Nonzero positive Kp is required. The finite tail
// is a prediction horizon, not a claim that integral bias has settled away.
// accelerationAlong / jerkAlong are the analytic thrust-direction projection
// from measured attitude/rate; they are not noisy Vicon second differences.
//
// A positive reserve (normally .03 m/s) only advances the SWITCH; it is not a
// terminal acceptance tolerance. Max actuator deceleration is intentionally
// NOT an input: the actual state and issued reference determine the impulse.
// Supported duration .05..1 s, tick .001..03 s, reserve 0..3 m/s. Invalid
// inputs/model or predicted geometric divergence return false, leaving output
// untouched. A true result never asserts the vehicle has actually stopped.
bool postReleaseLocalUnwindDecide(const jointState_t* state,
  const jointModel_t* model, const float reference[3],
  const float referenceRate[3], const float referenceAccel[3],
  const float referenceJerk[3], float holdYawDeg, const float direction[2],
  float unwindDurationS, float nextControlTickS, float forwardReserveMps,
  postReleaseLocalUnwindDecision_t* decision);
