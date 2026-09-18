#pragma once
#include "post_release_forward_stop.h"

// Research-only, no authority/freshness/hold decision in this numerical kernel.
typedef struct {
  float velocity[2], angle[3], rate[3];
} jointState_t;
typedef struct {
  float tau[3], attitudeGain[3], accelerationBias[2];
  float integralRateBias[3];
  float integralGain[3];
} jointModel_t;
typedef struct {
  postReleaseUnwindProfile_t piece[2];
  float knot[3], duration, endReference[3];
} jointPlan_t;
typedef struct {
  jointState_t terminal;
  float minForward, maxTilt, maxRate, cost;
  float integralEnd[3];
} jointPrediction_t;
bool jointEvaluate(const jointPlan_t* plan, float t, float angle[3],
  float rate[3], float acceleration[3], float jerk[3]);
// Cheap reconstruction of the event-driven Pi result; closed-form algebra
// only, never calls jointSolve/jointPredict. Initial acceleration/jerk are 0.
bool jointBuildFromParameters(const float reference[3], const float rate[3],
  const float knot[3], const float endReference[3], float duration,
  jointPlan_t* plan);
bool jointPredict(const jointPlan_t* plan, const jointState_t* initial,
  const jointModel_t* model, float yawDeg, const float direction[2],
  jointPrediction_t* prediction);
// Warm-start each replan with the previously issued reference and its first
// three derivatives. World velocity and actual attitude/rate are independent
// measured initial conditions of the predictor, not reference boundary values.
bool jointSolve(const jointState_t* initial, const jointModel_t* model,
  const float reference[3], const float rate[3], const float accel[3],
  const float jerk[3], float duration, float yawDeg, const float direction[2],
  jointPlan_t* plan, jointPrediction_t* prediction);
