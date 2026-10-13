#include "post_release_vicon_velocity_kf.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PROCESS_NOISE_MPS4 1.0f
#define POSITION_VARIANCE_M2 0.000001f
#define INITIAL_VARIANCE 1000.0f
#define MAX_FRAME_INTERVAL_US 25000u
#define MAX_PROPAGATION_GAP_US 250000u
#define MIN_FRAME_INTERVAL_US 3000u
#define WARMUP_SAMPLES 10u

void postReleaseViconVelocityKfReset(postReleaseViconVelocityKf_t* filter) {
  if (filter != NULL) {
    memset(filter, 0, sizeof(*filter));
  }
}

bool postReleaseViconVelocityKfUpdate(
    postReleaseViconVelocityKf_t* filter,
    const float positionM[3], const uint32_t captureUs) {
  if (filter == NULL) {
    return false;
  }
  if (positionM == NULL || captureUs == 0u) {
    return false;
  }
  for (int axis = 0; axis < 3; axis++) {
    if (!isfinite(positionM[axis])) {
      return false;
    }
  }
  if (!filter->initialized) {
    for (int axis = 0; axis < 3; axis++) {
      filter->covariance[axis][0][0] = INITIAL_VARIANCE;
      filter->covariance[axis][1][1] = INITIAL_VARIANCE;
    }
    filter->initialized = true;
  }
  const bool firstSample = filter->sampleCount == 0u;
  const uint32_t deltaUs = firstSample ? 10000u :
    captureUs - filter->lastCaptureUs;
  if (!firstSample &&
      (deltaUs < MIN_FRAME_INTERVAL_US ||
       deltaUs > MAX_PROPAGATION_GAP_US)) {
    return false;
  }
  const float dt = deltaUs * 1.0e-6f;
  const float halfDtSquared = 0.5f * dt * dt;
  for (int axis = 0; axis < 3; axis++) {
    const float p = filter->positionM[axis] + dt * filter->velocityMps[axis];
    const float v = filter->velocityMps[axis];
    const float p00 = filter->covariance[axis][0][0];
    const float p01 = filter->covariance[axis][0][1];
    const float p11 = filter->covariance[axis][1][1];
    const float pred00 = p00 + 2.0f * dt * p01 + dt * dt * p11 +
      PROCESS_NOISE_MPS4 * halfDtSquared * halfDtSquared;
    const float pred01 = p01 + dt * p11 +
      PROCESS_NOISE_MPS4 * halfDtSquared * dt;
    const float pred11 = p11 + PROCESS_NOISE_MPS4 * dt * dt;
    const float innovationVariance = pred00 + POSITION_VARIANCE_M2;
    if (!isfinite(innovationVariance) || innovationVariance <= 0.0f) {
      filter->valid = false;
      return false;
    }
    const float gainPosition = pred00 / innovationVariance;
    const float gainVelocity = pred01 / innovationVariance;
    const float innovation = positionM[axis] - p;
    filter->positionM[axis] = p + gainPosition * innovation;
    filter->velocityMps[axis] = v + gainVelocity * innovation;
    const float retained = POSITION_VARIANCE_M2 / innovationVariance;
    filter->covariance[axis][0][0] = retained * pred00;
    filter->covariance[axis][0][1] = retained * pred01;
    filter->covariance[axis][1][0] = retained * pred01;
    filter->covariance[axis][1][1] = pred11 - gainVelocity * pred01;
    if (!isfinite(filter->positionM[axis]) ||
        !isfinite(filter->velocityMps[axis]) ||
        !isfinite(filter->covariance[axis][1][1]) ||
        filter->covariance[axis][1][1] < 0.0f) {
      filter->valid = false;
      return false;
    }
  }
  filter->lastCaptureUs = captureUs;
  filter->sampleCount++;
  if (filter->gapRecoverySamples < 2u) {
    filter->gapRecoverySamples++;
  }
  filter->valid = filter->sampleCount >= WARMUP_SAMPLES &&
    filter->gapRecoverySamples >= 2u;
  return true;
}

postReleaseViconPiUpdateResult_t postReleaseViconVelocityKfUpdatePiEpoch(
    postReleaseViconVelocityKf_t* filter, const float positionM[3],
    const uint32_t piReceiveUsMod32, const uint32_t firmwareReceiveUs) {
  if (filter == NULL || positionM == NULL || firmwareReceiveUs == 0u) {
    return postReleaseViconPiInvalidInput;
  }
  for (int axis = 0; axis < 3; axis++) {
    if (!isfinite(positionM[axis])) {
      return postReleaseViconPiInvalidInput;
    }
  }
  const bool hadAcceptedFrame = filter->piReceiveSeen;
  const uint32_t intervalUs = hadAcceptedFrame ?
    piReceiveUsMod32 - filter->lastAcceptedPiReceiveUs : 0u;
  if (hadAcceptedFrame && intervalUs < MIN_FRAME_INTERVAL_US) {
    return postReleaseViconPiIgnoredEarly;
  }
  const bool gap = hadAcceptedFrame && intervalUs > MAX_FRAME_INTERVAL_US;
  const bool longGap = gap && intervalUs > MAX_PROPAGATION_GAP_US;
  if (longGap) {
    postReleaseViconVelocityKfReset(filter);
  }
  const uint32_t captureUs = filter->sampleCount == 0u ?
    firmwareReceiveUs : filter->lastCaptureUs + intervalUs;
  if (!postReleaseViconVelocityKfUpdate(filter, positionM, captureUs)) {
    postReleaseViconVelocityKfReset(filter);
    return postReleaseViconPiRejected;
  }
  if (gap && !longGap) {
    filter->gapRecoverySamples = 0u;
    filter->valid = false;
  }
  filter->lastAcceptedPiReceiveUs = piReceiveUsMod32;
  filter->piReceiveSeen = true;
  return longGap ? postReleaseViconPiRearmedAfterGap :
    gap ? postReleaseViconPiBridgedGap : postReleaseViconPiAccepted;
}
