/**
 * Experimental post-release inertial error-state Kalman filter.
 *
 * This module is deliberately independent of the commander and controller.
 * Velocity is propagated only from the release seed and measured IMU specific
 * force. World position is the required external observation; an optional
 * external-pose quaternion can also correct attitude when Vicon provides it.
 * Omitting quaternion updates leaves position fusion and inertial propagation
 * unchanged. The caller owns the initial quaternion provenance and must seed
 * it at physical release.
 *
 * Nominal process model:
 *   v_dot_world = R_body_to_world(q) * (f_measured_body - b_accel) + gravity
 *   q_dot        = q * Exp(gyro_measured_body - b_gyro)
 * The attitude-dependent specific-force term is also linearized into the
 * velocity/attitude cross covariance, so later position innovations can
 * correct attitude without treating the accelerometer as a gravity-only
 * measurement during dynamic braking.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "imu_types.h"

#define POST_RELEASE_EKF_STATE_DIM 15

typedef enum {
  PostReleaseEkfReasonInactive = 0,
  PostReleaseEkfReasonInitialized,
  PostReleaseEkfReasonPropagating,
  PostReleaseEkfReasonPositionFused,
  PostReleaseEkfReasonDuplicateTimestamp,
  PostReleaseEkfReasonPositionRejected,
  PostReleaseEkfReasonBackwardTimestamp,
  PostReleaseEkfReasonImuGapExceeded,
  PostReleaseEkfReasonInvalidArgument,
  PostReleaseEkfReasonNumericalFailure,
  PostReleaseEkfReasonImuPairStale,
  PostReleaseEkfReasonTimingOrderViolation,
  PostReleaseEkfReasonReleaseSensorInvalid,
  PostReleaseEkfReasonSeedEpochMismatch,
  PostReleaseEkfReasonGyroStaging,
  PostReleaseEkfReasonPositionIgnoredBeforeRelease,
  PostReleaseEkfReasonPositionIgnoredAtRelease,
  PostReleaseEkfReasonOrientationFused,
  PostReleaseEkfReasonOrientationRejected,
  PostReleaseEkfReasonOrientationIgnoredBeforeRelease,
  PostReleaseEkfReasonVelocitySeeded,
  PostReleaseEkfReasonVelocitySeedRejected,
} postReleaseEkfReason_t;

typedef struct {
  uint32_t maxImuGapUs;
  // Independent of IMU continuity: 5000 us targets 200 Hz covariance work.
  // Zero retains the original per-IMU-step propagation for regression tests.
  uint32_t covariancePeriodUs;
  float gyroNoiseRadPerSPerSqrtHz;
  float accelNoiseMps2PerSqrtHz;
  float gyroBiasWalkRadPerS2SqrtHz;
  float accelBiasWalkMps3SqrtHz;
  float positionStdM;
  float positionInnovationGateSigma;
  float orientationStdRad;
  float orientationInnovationGateSigma;
} postReleaseInertialEkfParams_t;

/** Error state: world position, world velocity, right attitude error,
 * gyro bias, accelerometer bias. Quaternion order is [w, x, y, z]. */
typedef struct {
  float positionM[3];
  float velocityMps[3];
  float quaternionWxyz[4];
  float gyroBiasRadPerS[3];
  float accelBiasMps2[3];
  float worldAccelerationMps2[3];
  float covariance[POST_RELEASE_EKF_STATE_DIM][POST_RELEASE_EKF_STATE_DIM];
  // Exact composition of the existing discrete per-IMU-step F and Q, not
  // a single averaged-IMU linearization. P is at covarianceTimestampUs;
  // nominal output is always at lastTimestampUs. Flush before consuming P.
  float pendingTransition[POST_RELEASE_EKF_STATE_DIM][POST_RELEASE_EKF_STATE_DIM];
  float pendingNoise[POST_RELEASE_EKF_STATE_DIM][POST_RELEASE_EKF_STATE_DIM];
  uint32_t covarianceTimestampUs;
  uint32_t covariancePeriodUs;
  uint32_t nextCovarianceUs;
  uint32_t covarianceCount;
  uint32_t covarianceForcedCount;
  uint32_t covarianceMaxSpanUs;
  uint32_t covarianceMaxComputeUs;
  uint32_t propagationMaxComputeUs;
  uint16_t pendingCovarianceSteps;

  Axis3f previousGyroRadPerS;
  uint32_t lastTimestampUs;
  uint32_t releaseTimestampUs;
  uint32_t propagationCount;
  uint32_t positionUpdateCount;
  uint32_t viconFusionCount;
  uint32_t rejectedViconFusionCount;
  uint32_t viconFusionTimestampUs;
  uint32_t rejectedPositionCount;
  uint32_t orientationUpdateCount;
  uint32_t rejectedOrientationCount;
  uint32_t preReleaseOrientationIgnoredCount;
  uint32_t maxObservedImuGapUs;
  uint32_t gyroStagingCount;
  uint32_t preReleasePositionIgnoredCount;
  uint32_t releaseEpochPositionIgnoredCount;
  postReleaseEkfReason_t reason;
  bool valid;
  bool previousGyroValid;
  bool postReleaseActive;
  bool externalVelocitySeeded;
} postReleaseInertialEkf_t;

void postReleaseInertialEkfDefaultParams(postReleaseInertialEkfParams_t* params);
/** Bring covariance to lastTimestampUs, without moving nominal state or
 * changing the periodic schedule. Measurement updates call this themselves.
 * Duplicate/malformed measurements are rejected before flushing. */
bool postReleaseInertialEkfFlushCovariance(postReleaseInertialEkf_t* ekf);
// Vicon KF joint XYZ/velocity and per-axis 2x2 covariance. Caller aligns to
// lastTimestampUs; sourceUs identifies a unique accepted front-end frame.
// This REPLACES raw-position fusion, it is not an additional independent input.
bool postReleaseInertialEkfFuseVicon(postReleaseInertialEkf_t *ekf,
  const float position[3],const float velocity[3],const float covariance[3][2][2],
  float priorWeight,uint32_t sourceUs);

/**
 * Seed the filter at the physical release epoch.
 *
 * All translational quantities use the world frame. IMU values use SI units.
 * initialGyroRadPerS may be NULL; supplying it enables trapezoidal gyro
 * integration across the first propagation interval.
 */
bool postReleaseInertialEkfInit(
  postReleaseInertialEkf_t* ekf,
  const postReleaseInertialEkfParams_t* params,
  const float positionM[3],
  const float velocityMps[3],
  const float quaternionWxyz[4],
  const float gyroBiasRadPerS[3],
  const Axis3f* initialGyroRadPerS,
  uint32_t timestampUs);

/**
 * Start the pre-release attitude-only staging phase.
 *
 * The staging phase retains only quaternion, gyro bias, previous gyro, and
 * producer timestamp as meaningful state. Its translation and covariance are
 * deliberately disposable and are replaced by a fixed prior at release.
 */
bool postReleaseInertialEkfStageInit(
  postReleaseInertialEkf_t* ekf,
  const postReleaseInertialEkfParams_t* params,
  const float quaternionWxyz[4],
  const float gyroBiasRadPerS[3],
  const Axis3f* initialGyroRadPerS,
  uint32_t timestampUs);

/** Integrate gyro only while waiting for release; acceleration is not input. */
postReleaseEkfReason_t postReleaseInertialEkfStageGyro(
  postReleaseInertialEkf_t* ekf,
  const postReleaseInertialEkfParams_t* params,
  const Axis3f* gyroRadPerS,
  uint32_t timestampUs);

/**
 * Atomically restart the full ESKF at the release epoch.
 *
 * Quaternion, gyro bias, and previous gyro are copied from staging before the
 * filter storage is cleared. Position and world velocity are copied from the
 * caller, accelerometer bias is reset to zero, and covariance/counters receive
 * the fixed release prior from postReleaseInertialEkfInit().
 */
bool postReleaseInertialEkfRestartAtRelease(
  postReleaseInertialEkf_t* ekf,
  const postReleaseInertialEkfParams_t* params,
  const float positionM[3],
  const float velocityMps[3],
  uint32_t timestampUs);

/**
 * Replace the release-epoch velocity with a quality-checked Vicon KF seed.
 * This one-shot operation preserves the contact-period attitude and biases.
 * It is not a second independent position observation: velocity cross terms
 * are cleared and the caller supplies conservative world-frame uncertainty.
 * The caller must establish capture/receive epoch agreement before invoking.
 */
bool postReleaseInertialEkfSeedVelocityAtRelease(
  postReleaseInertialEkf_t* ekf,
  const float velocityMps[3],
  const float velocityStdMps[3],
  uint32_t timestampUs);

/** Propagate active post-release state from measured IMU only. */
postReleaseEkfReason_t postReleaseInertialEkfPropagate(
  postReleaseInertialEkf_t* ekf,
  const postReleaseInertialEkfParams_t* params,
  const Axis3f* gyroRadPerS,
  const Axis3f* specificForceMps2,
  uint32_t timestampUs);

/**
 * Fuse one world-frame position observation at the current IMU epoch.
 *
 * Orientation is a separate optional update. A pre-release observation and an
 * observation exactly at release are safe no-ops with explicit reason/counter
 * values. Any other timestamp must match the most recent post-release IMU
 * propagation.
 */
postReleaseEkfReason_t postReleaseInertialEkfUpdatePosition(
  postReleaseInertialEkf_t* ekf,
  const postReleaseInertialEkfParams_t* params,
  const float positionM[3],
  float positionStdM,
  uint32_t timestampUs);

/**
 * Optionally fuse one body-to-world quaternion at the current IMU epoch.
 *
 * Position fusion is independent: callers simply omit this update when the
 * Vicon frame has no orientation. Quaternion order is [w, x, y, z].
 */
postReleaseEkfReason_t postReleaseInertialEkfUpdateOrientation(
  postReleaseInertialEkf_t* ekf,
  const postReleaseInertialEkfParams_t* params,
  const float quaternionWxyz[4],
  float orientationStdRad,
  uint32_t timestampUs);
