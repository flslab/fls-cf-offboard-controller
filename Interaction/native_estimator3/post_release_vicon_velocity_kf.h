#pragma once

#include <stdbool.h>
#include <stdint.h>

// The Pi Interaction velocity observer is three independent constant-velocity
// position KFs. This firmware port consumes Vicon positions only: neither IMU
// acceleration nor commander setpoints are inputs to its world-frame velocity.
typedef struct {
  float positionM[3];
  float velocityMps[3];
  float covariance[3][2][2];
  uint32_t lastCaptureUs;
  uint32_t lastAcceptedPiReceiveUs;
  uint32_t sampleCount;
  uint8_t gapRecoverySamples;
  bool piReceiveSeen;
  bool initialized;
  bool valid;
} postReleaseViconVelocityKf_t;

typedef enum {
  postReleaseViconPiRejected = 0,
  postReleaseViconPiAccepted,
  postReleaseViconPiIgnoredEarly,
  postReleaseViconPiBridgedGap,
  postReleaseViconPiRearmedAfterGap,
  postReleaseViconPiInvalidInput,
} postReleaseViconPiUpdateResult_t;

void postReleaseViconVelocityKfReset(postReleaseViconVelocityKf_t* filter);

// Capture timestamps share the simulator's monotone 32-bit microsecond clock.
// A malformed measurement is rejected without discarding a good history.
// Velocity authority is separately gated by receipt age and gap recovery.
bool postReleaseViconVelocityKfUpdate(
  postReleaseViconVelocityKf_t* filter,
  const float positionM[3], uint32_t captureUs);

// Use the Pi receipt epoch only to measure intervals between accepted frames.
// A too-early frame is skipped without destroying a warm observer; the next
// frame is measured against the last accepted epoch. A short gap is
// propagated with its actual elapsed time and needs two ordinary frames
// before velocity authority returns. A gap too long to propagate safely
// still requires the ordinary ten-sample warmup.
postReleaseViconPiUpdateResult_t postReleaseViconVelocityKfUpdatePiEpoch(
  postReleaseViconVelocityKf_t* filter, const float positionM[3],
  uint32_t piReceiveUsMod32, uint32_t firmwareReceiveUs);
