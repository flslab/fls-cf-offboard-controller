/**
 * Experimental post-release inertial error-state Kalman filter.
 *
 * The nominal state is propagated from measured gyro and accelerometer
 * specific force. Position innovations can correct velocity, attitude, and
 * IMU biases through the propagated cross covariance. An external quaternion
 * is an optional, independent observation: omitting it leaves position-aided
 * inertial propagation active. There is no command input in this module.
 */

#include "post_release_inertial_ekf.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "physicalConstants.h"

#if defined(CONFIG_PLATFORM_SITL)
#include <time.h>
#elif defined(CONFIG_PLATFORM_BOLT)
#include "usec_time.h"
#endif

// Profiling clock only; estimator chronology always uses acquisition stamps.
static uint32_t profileNow(void) {
#if defined(CONFIG_PLATFORM_SITL)
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)((uint64_t)ts.tv_sec * 1000000u + ts.tv_nsec / 1000u);
#elif defined(CONFIG_PLATFORM_BOLT)
  return (uint32_t)usecTimestamp();
#else
  return 0; // Native tests use an external benchmark, never invented MCU time.
#endif
}

#define POSITION_INDEX 0
#define VELOCITY_INDEX 3
#define ATTITUDE_INDEX 6
#define GYRO_BIAS_INDEX 9
#define ACCEL_BIAS_INDEX 12

#define MIN_QUATERNION_NORM 1.0e-6f
#define MIN_COVARIANCE_DIAGONAL 1.0e-12f
#define MAX_COVARIANCE_DIAGONAL 1.0e6f

// This experimental filter is instantiated once by estimator_kalman. Keeping
// matrix scratch space out of the Kalman task stack avoids a stack excursion of
// several kilobytes. The module is consequently intentionally non-reentrant.
static float transition[POST_RELEASE_EKF_STATE_DIM][POST_RELEASE_EKF_STATE_DIM];
static float matrixTmp[POST_RELEASE_EKF_STATE_DIM][POST_RELEASE_EKF_STATE_DIM];
static float matrixOut[POST_RELEASE_EKF_STATE_DIM][POST_RELEASE_EKF_STATE_DIM];

static bool finiteVector(const float* values, const size_t count) {
  for (size_t i = 0; i < count; i++) {
    if (!isfinite(values[i])) {
      return false;
    }
  }
  return true;
}

static bool validParams(const postReleaseInertialEkfParams_t* params) {
  return params != NULL &&
    params->maxImuGapUs > 0 &&
    (params->covariancePeriodUs == 0 ||
     (params->covariancePeriodUs >= 1000 && params->covariancePeriodUs <= 10000)) &&
    isfinite(params->gyroNoiseRadPerSPerSqrtHz) && params->gyroNoiseRadPerSPerSqrtHz > 0.0f &&
    isfinite(params->accelNoiseMps2PerSqrtHz) && params->accelNoiseMps2PerSqrtHz > 0.0f &&
    isfinite(params->gyroBiasWalkRadPerS2SqrtHz) && params->gyroBiasWalkRadPerS2SqrtHz > 0.0f &&
    isfinite(params->accelBiasWalkMps3SqrtHz) && params->accelBiasWalkMps3SqrtHz > 0.0f &&
    isfinite(params->positionStdM) && params->positionStdM > 0.0f &&
    isfinite(params->positionInnovationGateSigma) && params->positionInnovationGateSigma > 0.0f &&
    isfinite(params->orientationStdRad) && params->orientationStdRad > 0.0f &&
    isfinite(params->orientationInnovationGateSigma) &&
      params->orientationInnovationGateSigma > 0.0f;
}

static void setIdentity(float matrix[POST_RELEASE_EKF_STATE_DIM][POST_RELEASE_EKF_STATE_DIM]) {
  memset(matrix, 0, sizeof(float) * POST_RELEASE_EKF_STATE_DIM * POST_RELEASE_EKF_STATE_DIM);
  for (int i = 0; i < POST_RELEASE_EKF_STATE_DIM; i++) {
    matrix[i][i] = 1.0f;
  }
}

static void multiply(
  const float left[POST_RELEASE_EKF_STATE_DIM][POST_RELEASE_EKF_STATE_DIM],
  const float right[POST_RELEASE_EKF_STATE_DIM][POST_RELEASE_EKF_STATE_DIM],
  float result[POST_RELEASE_EKF_STATE_DIM][POST_RELEASE_EKF_STATE_DIM]) {
  for (int row = 0; row < POST_RELEASE_EKF_STATE_DIM; row++) {
    for (int column = 0; column < POST_RELEASE_EKF_STATE_DIM; column++) {
      float value = 0.0f;
      for (int inner = 0; inner < POST_RELEASE_EKF_STATE_DIM; inner++) {
        value += left[row][inner] * right[inner][column];
      }
      result[row][column] = value;
    }
  }
}

static void multiplyRightTranspose(
  const float left[POST_RELEASE_EKF_STATE_DIM][POST_RELEASE_EKF_STATE_DIM],
  const float right[POST_RELEASE_EKF_STATE_DIM][POST_RELEASE_EKF_STATE_DIM],
  float result[POST_RELEASE_EKF_STATE_DIM][POST_RELEASE_EKF_STATE_DIM]) {
  for (int row = 0; row < POST_RELEASE_EKF_STATE_DIM; row++) {
    for (int column = 0; column < POST_RELEASE_EKF_STATE_DIM; column++) {
      float value = 0.0f;
      for (int inner = 0; inner < POST_RELEASE_EKF_STATE_DIM; inner++) {
        value += left[row][inner] * right[column][inner];
      }
      result[row][column] = value;
    }
  }
}

// F and composed Phi are sparse. Skip exact zero entries only (no threshold
// approximation) and traverse the right rows contiguously. No stack matrices.
static void multiplySparseLeft(
    const float left[15][15], const float right[15][15], float result[15][15]) {
  memset(result, 0, sizeof(float) * 15 * 15);
  for (int row = 0; row < 15; row++) {
    for (int inner = 0; inner < 15; inner++) {
      const float value = left[row][inner];
      if (value == 0.0f) { continue; }
      for (int column = 0; column < 15; column++) {
        result[row][column] += value * right[inner][column];
      }
    }
  }
}

// For symmetric B, A B A' = A (A B)'. matrixOut is the result.
static void congruenceSparse(const float a[15][15], const float b[15][15]) {
  multiplySparseLeft(a, b, matrixTmp);
  for (int row = 0; row < 15; row++) {
    for (int column = row + 1; column < 15; column++) {
      const float tmp = matrixTmp[row][column];
      matrixTmp[row][column] = matrixTmp[column][row];
      matrixTmp[column][row] = tmp;
    }
  }
  multiplySparseLeft(a, matrixTmp, matrixOut);
}

static bool normalizeQuaternion(float quaternion[4]) {
  const float normSquared =
    quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
    quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3];
  if (!isfinite(normSquared) || normSquared < MIN_QUATERNION_NORM * MIN_QUATERNION_NORM) {
    return false;
  }

  const float inverseNorm = 1.0f / sqrtf(normSquared);
  for (int i = 0; i < 4; i++) {
    quaternion[i] *= inverseNorm;
  }
  // Match the offboard implementation's unique quaternion representative.
  // q and -q encode the same attitude, but keeping w >= 0 avoids artificial
  // sign jumps in logged state and golden-vector comparisons.
  if (quaternion[0] < 0.0f) {
    for (int i = 0; i < 4; i++) {
      quaternion[i] = -quaternion[i];
    }
  }
  return finiteVector(quaternion, 4);
}

static void quaternionToRotation(const float q[4], float rotation[3][3]) {
  const float w = q[0];
  const float x = q[1];
  const float y = q[2];
  const float z = q[3];

  rotation[0][0] = 1.0f - 2.0f * (y * y + z * z);
  rotation[0][1] = 2.0f * (x * y - w * z);
  rotation[0][2] = 2.0f * (x * z + w * y);
  rotation[1][0] = 2.0f * (x * y + w * z);
  rotation[1][1] = 1.0f - 2.0f * (x * x + z * z);
  rotation[1][2] = 2.0f * (y * z - w * x);
  rotation[2][0] = 2.0f * (x * z - w * y);
  rotation[2][1] = 2.0f * (y * z + w * x);
  rotation[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

static bool multiplyQuaternionRight(float quaternion[4], const float delta[4]) {
  const float w = quaternion[0];
  const float x = quaternion[1];
  const float y = quaternion[2];
  const float z = quaternion[3];

  quaternion[0] = w * delta[0] - x * delta[1] - y * delta[2] - z * delta[3];
  quaternion[1] = w * delta[1] + x * delta[0] + y * delta[3] - z * delta[2];
  quaternion[2] = w * delta[2] - x * delta[3] + y * delta[0] + z * delta[1];
  quaternion[3] = w * delta[3] + x * delta[2] - y * delta[1] + z * delta[0];
  return normalizeQuaternion(quaternion);
}

static bool injectAttitude(float quaternion[4], const float attitudeError[3]) {
  const float angleSquared =
    attitudeError[0] * attitudeError[0] + attitudeError[1] * attitudeError[1] +
    attitudeError[2] * attitudeError[2];
  if (!isfinite(angleSquared)) {
    return false;
  }

  float delta[4];
  if (angleSquared < 1.0e-16f) {
    delta[0] = 1.0f;
    delta[1] = 0.5f * attitudeError[0];
    delta[2] = 0.5f * attitudeError[1];
    delta[3] = 0.5f * attitudeError[2];
  } else {
    const float angle = sqrtf(angleSquared);
    const float scale = sinf(0.5f * angle) / angle;
    delta[0] = cosf(0.5f * angle);
    delta[1] = scale * attitudeError[0];
    delta[2] = scale * attitudeError[1];
    delta[3] = scale * attitudeError[2];
  }
  return multiplyQuaternionRight(quaternion, delta);
}

static bool integrateBodyRate(float quaternion[4], const float omega[3], const float dt) {
  const float rotationVector[3] = {
    omega[0] * dt,
    omega[1] * dt,
    omega[2] * dt,
  };
  return injectAttitude(quaternion, rotationVector);
}

static void setSkew(float matrix[3][3], const float vector[3]) {
  matrix[0][0] = 0.0f;
  matrix[0][1] = -vector[2];
  matrix[0][2] = vector[1];
  matrix[1][0] = vector[2];
  matrix[1][1] = 0.0f;
  matrix[1][2] = -vector[0];
  matrix[2][0] = -vector[1];
  matrix[2][1] = vector[0];
  matrix[2][2] = 0.0f;
}

static bool conditionCovariance(postReleaseInertialEkf_t* ekf) {
  for (int row = 0; row < POST_RELEASE_EKF_STATE_DIM; row++) {
    for (int column = row; column < POST_RELEASE_EKF_STATE_DIM; column++) {
      const float symmetric = 0.5f * (ekf->covariance[row][column] + ekf->covariance[column][row]);
      if (!isfinite(symmetric)) {
        return false;
      }
      ekf->covariance[row][column] = symmetric;
      ekf->covariance[column][row] = symmetric;
    }
    if (ekf->covariance[row][row] < MIN_COVARIANCE_DIAGONAL) {
      ekf->covariance[row][row] = MIN_COVARIANCE_DIAGONAL;
    }
    if (ekf->covariance[row][row] > MAX_COVARIANCE_DIAGONAL) {
      return false;
    }
  }
  return true;
}

static bool stateIsFinite(const postReleaseInertialEkf_t* ekf) {
  return finiteVector(ekf->positionM, 3) &&
    finiteVector(ekf->velocityMps, 3) &&
    finiteVector(ekf->quaternionWxyz, 4) &&
    finiteVector(ekf->gyroBiasRadPerS, 3) &&
    finiteVector(ekf->accelBiasMps2, 3) &&
    finiteVector(ekf->worldAccelerationMps2, 3);
}

static postReleaseEkfReason_t invalidate(
  postReleaseInertialEkf_t* ekf, const postReleaseEkfReason_t reason) {
  if (ekf != NULL) {
    ekf->valid = false;
    ekf->reason = reason;
  }
  return reason;
}

void postReleaseInertialEkfDefaultParams(postReleaseInertialEkfParams_t* params) {
  if (params == NULL) {
    return;
  }
  *params = (postReleaseInertialEkfParams_t) {
    .maxImuGapUs = 5000,
    .covariancePeriodUs = 5000,
    .gyroNoiseRadPerSPerSqrtHz = 0.0061086524f,  // 0.35 degrees/s/sqrt(Hz)
    .accelNoiseMps2PerSqrtHz = 0.18f,
    .gyroBiasWalkRadPerS2SqrtHz = 0.00034906585f,
    .accelBiasWalkMps3SqrtHz = 0.02f,
    .positionStdM = 0.003f,
    .positionInnovationGateSigma = 6.0f,
    .orientationStdRad = 0.05f,
    .orientationInnovationGateSigma = 6.0f,
  };
}

bool postReleaseInertialEkfInit(
  postReleaseInertialEkf_t* ekf,
  const postReleaseInertialEkfParams_t* params,
  const float positionM[3],
  const float velocityMps[3],
  const float quaternionWxyz[4],
  const float gyroBiasRadPerS[3],
  const Axis3f* initialGyroRadPerS,
  const uint32_t timestampUs) {
  if (ekf == NULL || !validParams(params) || positionM == NULL || velocityMps == NULL ||
      quaternionWxyz == NULL || gyroBiasRadPerS == NULL ||
      !finiteVector(positionM, 3) || !finiteVector(velocityMps, 3) ||
      !finiteVector(quaternionWxyz, 4) || !finiteVector(gyroBiasRadPerS, 3) ||
      (initialGyroRadPerS != NULL && !finiteVector(initialGyroRadPerS->axis, 3))) {
    if (ekf != NULL) {
      memset(ekf, 0, sizeof(*ekf));
      ekf->reason = PostReleaseEkfReasonInvalidArgument;
    }
    return false;
  }

  // Copy every caller-owned seed before clearing ekf. This also makes the API
  // safe when a lifecycle restart passes fields from the same ekf instance.
  float positionCopy[3];
  float velocityCopy[3];
  float quaternionCopy[4];
  float gyroBiasCopy[3];
  Axis3f initialGyroCopy;
  const bool initialGyroValid = initialGyroRadPerS != NULL;
  memcpy(positionCopy, positionM, sizeof(positionCopy));
  memcpy(velocityCopy, velocityMps, sizeof(velocityCopy));
  memcpy(quaternionCopy, quaternionWxyz, sizeof(quaternionCopy));
  memcpy(gyroBiasCopy, gyroBiasRadPerS, sizeof(gyroBiasCopy));
  if (initialGyroValid) {
    initialGyroCopy = *initialGyroRadPerS;
  }

  memset(ekf, 0, sizeof(*ekf));
  memcpy(ekf->positionM, positionCopy, sizeof(ekf->positionM));
  memcpy(ekf->velocityMps, velocityCopy, sizeof(ekf->velocityMps));
  memcpy(ekf->quaternionWxyz, quaternionCopy, sizeof(ekf->quaternionWxyz));
  memcpy(ekf->gyroBiasRadPerS, gyroBiasCopy, sizeof(ekf->gyroBiasRadPerS));
  if (!normalizeQuaternion(ekf->quaternionWxyz)) {
    ekf->reason = PostReleaseEkfReasonInvalidArgument;
    return false;
  }

  if (initialGyroValid) {
    ekf->previousGyroRadPerS = initialGyroCopy;
    ekf->previousGyroValid = true;
  }
  ekf->lastTimestampUs = timestampUs;
  ekf->releaseTimestampUs = timestampUs;
  ekf->covarianceTimestampUs = timestampUs;
  ekf->covariancePeriodUs = params->covariancePeriodUs;
  ekf->nextCovarianceUs = timestampUs + params->covariancePeriodUs;
  ekf->valid = true;
  ekf->postReleaseActive = true;
  ekf->reason = PostReleaseEkfReasonInitialized;

  const float initialStdDev[POST_RELEASE_EKF_STATE_DIM] = {
    0.004f, 0.004f, 0.004f,
    0.08f, 0.08f, 0.08f,
    0.0698131701f, 0.0698131701f, 0.0698131701f,
    0.0139626340f, 0.0139626340f, 0.0139626340f,
    0.03f, 0.03f, 0.03f,
  };
  for (int i = 0; i < POST_RELEASE_EKF_STATE_DIM; i++) {
    ekf->covariance[i][i] = initialStdDev[i] * initialStdDev[i];
  }
  return true;
}

bool postReleaseInertialEkfStageInit(
  postReleaseInertialEkf_t* ekf,
  const postReleaseInertialEkfParams_t* params,
  const float quaternionWxyz[4],
  const float gyroBiasRadPerS[3],
  const Axis3f* initialGyroRadPerS,
  const uint32_t timestampUs) {
  const float zero[3] = {0.0f, 0.0f, 0.0f};
  if (!postReleaseInertialEkfInit(
      ekf, params, zero, zero, quaternionWxyz, gyroBiasRadPerS,
      initialGyroRadPerS, timestampUs)) {
    return false;
  }
  ekf->postReleaseActive = false;
  ekf->releaseTimestampUs = 0;
  ekf->gyroStagingCount = 1;
  ekf->reason = PostReleaseEkfReasonGyroStaging;
  return true;
}

postReleaseEkfReason_t postReleaseInertialEkfStageGyro(
  postReleaseInertialEkf_t* ekf,
  const postReleaseInertialEkfParams_t* params,
  const Axis3f* gyroRadPerS,
  const uint32_t timestampUs) {
  if (ekf == NULL || !validParams(params) || gyroRadPerS == NULL ||
      !finiteVector(gyroRadPerS->axis, 3)) {
    return invalidate(ekf, PostReleaseEkfReasonInvalidArgument);
  }
  if (!ekf->valid) {
    return ekf->reason;
  }
  if (ekf->postReleaseActive) {
    return invalidate(ekf, PostReleaseEkfReasonTimingOrderViolation);
  }

  const uint32_t deltaUs = timestampUs - ekf->lastTimestampUs;
  if (deltaUs == 0) {
    ekf->reason = PostReleaseEkfReasonDuplicateTimestamp;
    return ekf->reason;
  }
  if (deltaUs >= 0x80000000u) {
    return invalidate(ekf, PostReleaseEkfReasonBackwardTimestamp);
  }
  if (deltaUs > params->maxImuGapUs) {
    return invalidate(ekf, PostReleaseEkfReasonImuGapExceeded);
  }

  const float dt = 0.000001f * (float)deltaUs;
  float omega[3];
  for (int i = 0; i < 3; i++) {
    const float meanGyro = ekf->previousGyroValid ?
      0.5f * (ekf->previousGyroRadPerS.axis[i] + gyroRadPerS->axis[i]) :
      gyroRadPerS->axis[i];
    omega[i] = meanGyro - ekf->gyroBiasRadPerS[i];
  }
  if (!integrateBodyRate(ekf->quaternionWxyz, omega, dt)) {
    return invalidate(ekf, PostReleaseEkfReasonNumericalFailure);
  }
  ekf->previousGyroRadPerS = *gyroRadPerS;
  ekf->previousGyroValid = true;
  ekf->lastTimestampUs = timestampUs;
  ekf->gyroStagingCount++;
  ekf->reason = PostReleaseEkfReasonGyroStaging;
  return ekf->reason;
}

bool postReleaseInertialEkfRestartAtRelease(
  postReleaseInertialEkf_t* ekf,
  const postReleaseInertialEkfParams_t* params,
  const float positionM[3],
  const float velocityMps[3],
  const uint32_t timestampUs) {
  if (ekf == NULL || !validParams(params) || positionM == NULL ||
      velocityMps == NULL || !finiteVector(positionM, 3) ||
      !finiteVector(velocityMps, 3)) {
    invalidate(ekf, PostReleaseEkfReasonInvalidArgument);
    return false;
  }
  if (!ekf->valid || ekf->postReleaseActive ||
      timestampUs != ekf->lastTimestampUs || !ekf->previousGyroValid) {
    if (ekf != NULL) {
      ekf->valid = false;
      ekf->reason = PostReleaseEkfReasonSeedEpochMismatch;
    }
    return false;
  }

  // These locals are intentional: postReleaseInertialEkfInit() clears ekf.
  // Never pass pointers into that storage across the reset boundary.
  float positionCopy[3];
  float velocityCopy[3];
  float quaternionCopy[4];
  float gyroBiasCopy[3];
  const Axis3f previousGyroCopy = ekf->previousGyroRadPerS;
  const uint32_t stagingCount = ekf->gyroStagingCount;
  const uint32_t preReleasePositionIgnoredCount =
    ekf->preReleasePositionIgnoredCount;
  memcpy(positionCopy, positionM, sizeof(positionCopy));
  memcpy(velocityCopy, velocityMps, sizeof(velocityCopy));
  memcpy(quaternionCopy, ekf->quaternionWxyz, sizeof(quaternionCopy));
  memcpy(gyroBiasCopy, ekf->gyroBiasRadPerS, sizeof(gyroBiasCopy));

  if (!postReleaseInertialEkfInit(
      ekf, params, positionCopy, velocityCopy, quaternionCopy, gyroBiasCopy,
      &previousGyroCopy, timestampUs)) {
    return false;
  }
  ekf->gyroStagingCount = stagingCount;
  ekf->preReleasePositionIgnoredCount = preReleasePositionIgnoredCount;
  return true;
}

bool postReleaseInertialEkfSeedVelocityAtRelease(
    postReleaseInertialEkf_t* ekf, const float velocityMps[3],
    const float velocityStdMps[3], const uint32_t timestampUs) {
  if (ekf == NULL) {
    return false;
  }
  if (!ekf->valid || !ekf->postReleaseActive || ekf->externalVelocitySeeded ||
      timestampUs != ekf->lastTimestampUs ||
      (ekf->releaseTimestampUs != 0 &&
       ekf->releaseTimestampUs != timestampUs) ||
      velocityMps == NULL || velocityStdMps == NULL ||
      !finiteVector(velocityMps, 3) ||
      !finiteVector(velocityStdMps, 3)) {
    ekf->reason = PostReleaseEkfReasonVelocitySeedRejected;
    return false;
  }
  for (int axis = 0; axis < 3; axis++) {
    if (velocityStdMps[axis] <= 0.0f ||
        velocityStdMps[axis] > 1.0f ||
        fabsf(velocityMps[axis]) > 3.0f) {
      ekf->reason = PostReleaseEkfReasonVelocitySeedRejected;
      return false;
    }
  }

  // The Vicon velocity is derived from the same position stream. Do not
  // manufacture an independent position update or retain cross-covariance
  // from the superseded velocity. An 0.08 m/s floor is the existing release
  // prior until Pi KF covariance and capture timing are independently proven.
  if (!postReleaseInertialEkfFlushCovariance(ekf)) { return false; }
  for (int axis = 0; axis < 3; axis++) {
    const int index = VELOCITY_INDEX + axis;
    const float conservativeStd = fmaxf(velocityStdMps[axis], 0.08f);
    for (int other = 0; other < POST_RELEASE_EKF_STATE_DIM; other++) {
      ekf->covariance[index][other] = 0.0f;
      ekf->covariance[other][index] = 0.0f;
    }
    ekf->covariance[index][index] = conservativeStd * conservativeStd;
    ekf->velocityMps[axis] = velocityMps[axis];
  }
  ekf->externalVelocitySeeded = true;
  ekf->reason = PostReleaseEkfReasonVelocitySeeded;
  return true;
}

static bool flushCovariance(postReleaseInertialEkf_t* ekf, bool forced) {
  if (ekf == NULL || !ekf->valid) { return false; }
  if (ekf->pendingCovarianceSteps == 0) { return true; }
  const uint32_t begin = profileNow();
  congruenceSparse(ekf->pendingTransition, ekf->covariance);
  for (int row = 0; row < 15; row++) {
    for (int column = 0; column < 15; column++) {
      ekf->covariance[row][column] =
        matrixOut[row][column] + ekf->pendingNoise[row][column];
    }
  }
  if (!conditionCovariance(ekf)) {
    invalidate(ekf, PostReleaseEkfReasonNumericalFailure);
    return false;
  }
  const uint32_t span = ekf->lastTimestampUs - ekf->covarianceTimestampUs;
  if (span > ekf->covarianceMaxSpanUs) { ekf->covarianceMaxSpanUs = span; }
  ekf->covarianceTimestampUs = ekf->lastTimestampUs;
  ekf->pendingCovarianceSteps = 0;
  ekf->covarianceCount++;
  if (forced) { ekf->covarianceForcedCount++; }
  const uint32_t elapsed = profileNow() - begin;
  if (elapsed > ekf->covarianceMaxComputeUs) { ekf->covarianceMaxComputeUs = elapsed; }
  return true;
}

bool postReleaseInertialEkfFlushCovariance(postReleaseInertialEkf_t* ekf) {
  return flushCovariance(ekf, true);
}

postReleaseEkfReason_t postReleaseInertialEkfPropagate(
  postReleaseInertialEkf_t* ekf,
  const postReleaseInertialEkfParams_t* params,
  const Axis3f* gyroRadPerS,
  const Axis3f* specificForceMps2,
  const uint32_t timestampUs) {
  if (ekf == NULL || !validParams(params) || gyroRadPerS == NULL || specificForceMps2 == NULL ||
      !finiteVector(gyroRadPerS->axis, 3) || !finiteVector(specificForceMps2->axis, 3)) {
    return invalidate(ekf, PostReleaseEkfReasonInvalidArgument);
  }
  if (!ekf->valid) {
    return ekf->reason;
  }
  if (!ekf->postReleaseActive) {
    ekf->reason = PostReleaseEkfReasonGyroStaging;
    return ekf->reason;
  }

  const uint32_t deltaUs = timestampUs - ekf->lastTimestampUs;
  if (deltaUs == 0) {
    ekf->reason = PostReleaseEkfReasonDuplicateTimestamp;
    return ekf->reason;
  }
  if (deltaUs >= 0x80000000u) {
    return invalidate(ekf, PostReleaseEkfReasonBackwardTimestamp);
  }
  if (deltaUs > ekf->maxObservedImuGapUs) {
    ekf->maxObservedImuGapUs = deltaUs;
  }
  if (deltaUs > params->maxImuGapUs) {
    return invalidate(ekf, PostReleaseEkfReasonImuGapExceeded);
  }

  const uint32_t profileBegin = profileNow();
  const float dt = 0.000001f * (float)deltaUs;
  float omega[3];
  for (int i = 0; i < 3; i++) {
    const float meanGyro = ekf->previousGyroValid ?
      0.5f * (ekf->previousGyroRadPerS.axis[i] + gyroRadPerS->axis[i]) :
      gyroRadPerS->axis[i];
    omega[i] = meanGyro - ekf->gyroBiasRadPerS[i];
  }
  ekf->previousGyroRadPerS = *gyroRadPerS;
  ekf->previousGyroValid = true;

  float correctedForce[3];
  for (int i = 0; i < 3; i++) {
    correctedForce[i] = specificForceMps2->axis[i] - ekf->accelBiasMps2[i];
  }

  float rotation[3][3];
  quaternionToRotation(ekf->quaternionWxyz, rotation);
  for (int row = 0; row < 3; row++) {
    ekf->worldAccelerationMps2[row] =
      rotation[row][0] * correctedForce[0] +
      rotation[row][1] * correctedForce[1] +
      rotation[row][2] * correctedForce[2];
  }
  ekf->worldAccelerationMps2[2] -= GRAVITY_MAGNITUDE;

  for (int i = 0; i < 3; i++) {
    ekf->positionM[i] += ekf->velocityMps[i] * dt +
      0.5f * ekf->worldAccelerationMps2[i] * dt * dt;
    ekf->velocityMps[i] += ekf->worldAccelerationMps2[i] * dt;
  }
  if (!integrateBodyRate(ekf->quaternionWxyz, omega, dt)) {
    return invalidate(ekf, PostReleaseEkfReasonNumericalFailure);
  }

  setIdentity(transition);
  const float halfDtSquared = 0.5f * dt * dt;
  for (int i = 0; i < 3; i++) {
    transition[POSITION_INDEX + i][VELOCITY_INDEX + i] = dt;
    transition[ATTITUDE_INDEX + i][GYRO_BIAS_INDEX + i] = -dt;
  }

  float forceSkew[3][3];
  float omegaSkew[3][3];
  setSkew(forceSkew, correctedForce);
  setSkew(omegaSkew, omega);
  for (int row = 0; row < 3; row++) {
    for (int column = 0; column < 3; column++) {
      float rotatedSkew = 0.0f;
      for (int inner = 0; inner < 3; inner++) {
        rotatedSkew += rotation[row][inner] * forceSkew[inner][column];
      }
      const float accelerationAttitudeJacobian = -rotatedSkew;
      const float accelerationBiasJacobian = -rotation[row][column];
      transition[POSITION_INDEX + row][ATTITUDE_INDEX + column] =
        accelerationAttitudeJacobian * halfDtSquared;
      transition[POSITION_INDEX + row][ACCEL_BIAS_INDEX + column] =
        accelerationBiasJacobian * halfDtSquared;
      transition[VELOCITY_INDEX + row][ATTITUDE_INDEX + column] =
        accelerationAttitudeJacobian * dt;
      transition[VELOCITY_INDEX + row][ACCEL_BIAS_INDEX + column] =
        accelerationBiasJacobian * dt;
      transition[ATTITUDE_INDEX + row][ATTITUDE_INDEX + column] -= omegaSkew[row][column] * dt;
    }
  }

  if (ekf->covariancePeriodUs == 0) {
    // Original dense algorithm kept as a numerical/CPU reference.
    multiply(transition, ekf->covariance, matrixTmp);
    multiplyRightTranspose(matrixTmp, transition, matrixOut);
    memcpy(ekf->covariance, matrixOut, sizeof(ekf->covariance));
  } else if (ekf->pendingCovarianceSteps == 0) {
    memcpy(ekf->pendingTransition, transition, sizeof(transition));
    memset(ekf->pendingNoise, 0, sizeof(ekf->pendingNoise));
  } else {
    // Phi <- F Phi; Q <- F Q F'. Each F uses that step's actual attitude,
    // angular velocity, specific force and dt, including rapid contact motion.
    multiplySparseLeft(transition, ekf->pendingTransition, matrixTmp);
    memcpy(ekf->pendingTransition, matrixTmp, sizeof(matrixTmp));
    congruenceSparse(transition, ekf->pendingNoise);
    // Maintain the symmetry required by congruenceSparse on subsequent steps.
    for (int row = 0; row < 15; row++) {
      for (int column = row; column < 15; column++) {
        const float value = 0.5f * (matrixOut[row][column] + matrixOut[column][row]);
        if (!isfinite(value)) { return invalidate(ekf, PostReleaseEkfReasonNumericalFailure); }
        ekf->pendingNoise[row][column] = ekf->pendingNoise[column][row] = value;
      }
    }
  }
  float (*noiseTarget)[15] = ekf->covariancePeriodUs == 0 ?
    ekf->covariance : ekf->pendingNoise;

  const float accelNoiseVariance =
    params->accelNoiseMps2PerSqrtHz * params->accelNoiseMps2PerSqrtHz;
  const float accelPositionVariance = accelNoiseVariance * dt * dt * dt / 3.0f;
  const float accelPositionVelocityCovariance = accelNoiseVariance * halfDtSquared;
  const float accelVelocityVariance = accelNoiseVariance * dt;
  const float gyroVariance =
    params->gyroNoiseRadPerSPerSqrtHz * params->gyroNoiseRadPerSPerSqrtHz * dt;
  const float gyroBiasVariance =
    params->gyroBiasWalkRadPerS2SqrtHz * params->gyroBiasWalkRadPerS2SqrtHz * dt;
  const float accelBiasVariance =
    params->accelBiasWalkMps3SqrtHz * params->accelBiasWalkMps3SqrtHz * dt;

  // Accelerometer white noise is isotropic, so rotating it from body to world
  // leaves its covariance unchanged. Integrating that continuous white noise
  // through position and velocity yields the exact constant-acceleration
  // Qpp/Qpv/Qvp/Qvv blocks for this interval.
  for (int i = 0; i < 3; i++) {
    noiseTarget[POSITION_INDEX + i][POSITION_INDEX + i] +=
      accelPositionVariance;
    noiseTarget[POSITION_INDEX + i][VELOCITY_INDEX + i] +=
      accelPositionVelocityCovariance;
    noiseTarget[VELOCITY_INDEX + i][POSITION_INDEX + i] +=
      accelPositionVelocityCovariance;
    noiseTarget[VELOCITY_INDEX + i][VELOCITY_INDEX + i] +=
      accelVelocityVariance;
    noiseTarget[ATTITUDE_INDEX + i][ATTITUDE_INDEX + i] += gyroVariance;
    noiseTarget[GYRO_BIAS_INDEX + i][GYRO_BIAS_INDEX + i] += gyroBiasVariance;
    noiseTarget[ACCEL_BIAS_INDEX + i][ACCEL_BIAS_INDEX + i] += accelBiasVariance;
  }

  ekf->lastTimestampUs = timestampUs;
  if (!stateIsFinite(ekf)) {
    return invalidate(ekf, PostReleaseEkfReasonNumericalFailure);
  }
  if (ekf->covariancePeriodUs == 0) {
    if (!conditionCovariance(ekf)) { return invalidate(ekf, PostReleaseEkfReasonNumericalFailure); }
    ekf->covarianceTimestampUs = timestampUs;
    ekf->covarianceCount++;
  } else {
    ekf->pendingCovarianceSteps++;
    if ((int32_t)(timestampUs - ekf->nextCovarianceUs) >= 0) {
      if (!flushCovariance(ekf, false)) { return ekf->reason; }
      // Preserve the 5 ms phase: 2 ms IMU steps alternate 6/4 ms flushes,
      // rather than inadvertently becoming a 6 ms / 166.7 Hz estimator.
      ekf->nextCovarianceUs += ((timestampUs - ekf->nextCovarianceUs) /
        ekf->covariancePeriodUs + 1u) * ekf->covariancePeriodUs;
    }
  }
  ekf->propagationCount++;
  const uint32_t elapsed = profileNow() - profileBegin;
  if (elapsed > ekf->propagationMaxComputeUs) { ekf->propagationMaxComputeUs = elapsed; }
  ekf->reason = PostReleaseEkfReasonPropagating;
  return ekf->reason;
}

static bool resetAttitudeCovariance(
  postReleaseInertialEkf_t* ekf, const float attitudeError[3]) {
  setIdentity(transition);
  float errorSkew[3][3];
  setSkew(errorSkew, attitudeError);
  for (int row = 0; row < 3; row++) {
    for (int column = 0; column < 3; column++) {
      transition[ATTITUDE_INDEX + row][ATTITUDE_INDEX + column] -=
        0.5f * errorSkew[row][column];
    }
  }
  multiply(transition, ekf->covariance, matrixTmp);
  multiplyRightTranspose(matrixTmp, transition, matrixOut);
  memcpy(ekf->covariance, matrixOut, sizeof(ekf->covariance));
  return conditionCovariance(ekf);
}

postReleaseEkfReason_t postReleaseInertialEkfUpdatePosition(
  postReleaseInertialEkf_t* ekf,
  const postReleaseInertialEkfParams_t* params,
  const float positionM[3],
  const float positionStdM,
  const uint32_t timestampUs) {
  if (ekf == NULL || !validParams(params) || positionM == NULL ||
      !finiteVector(positionM, 3) || !isfinite(positionStdM) || positionStdM <= 0.0f) {
    return invalidate(ekf, PostReleaseEkfReasonInvalidArgument);
  }
  if (!ekf->valid) {
    return ekf->reason;
  }
  if (!ekf->postReleaseActive) {
    ekf->preReleasePositionIgnoredCount++;
    ekf->reason = PostReleaseEkfReasonPositionIgnoredBeforeRelease;
    return ekf->reason;
  }
  if (timestampUs == ekf->releaseTimestampUs) {
    ekf->releaseEpochPositionIgnoredCount++;
    ekf->reason = PostReleaseEkfReasonPositionIgnoredAtRelease;
    return ekf->reason;
  }
  if (timestampUs != ekf->lastTimestampUs) {
    return invalidate(ekf, PostReleaseEkfReasonTimingOrderViolation);
  }

  if (!postReleaseInertialEkfFlushCovariance(ekf)) { return ekf->reason; }
  const float measurementVariance = positionStdM * positionStdM;
  const float gateSquared =
    params->positionInnovationGateSigma * params->positionInnovationGateSigma;

  // Apply the diagonal position observation as three scalar Joseph updates.
  // The algebra is equivalent to the batch update above, but avoids forming a
  // dense 15x3 gain from an explicitly inverted 3x3 matrix in single
  // precision.  Long 1 kHz firmware runs otherwise accumulate enough loss of
  // symmetry/PSD in the cross-covariance to produce a large bias or attitude
  // correction even while the position innovation itself still passes its
  // Mahalanobis gate.
  uint8_t acceptedAxisCount = 0;
  for (int axis = 0; axis < 3; axis++) {
    const float scalarInnovation = positionM[axis] - ekf->positionM[axis];
    const float scalarInnovationVariance =
      ekf->covariance[POSITION_INDEX + axis][POSITION_INDEX + axis] +
      measurementVariance;
    if (!isfinite(scalarInnovationVariance) ||
        scalarInnovationVariance <= MIN_COVARIANCE_DIAGONAL) {
      return invalidate(ekf, PostReleaseEkfReasonNumericalFailure);
    }
    const float normalizedInnovationSquared =
      scalarInnovation * scalarInnovation / scalarInnovationVariance;
    if (!isfinite(normalizedInnovationSquared)) {
      return invalidate(ekf, PostReleaseEkfReasonNumericalFailure);
    }
    // Gate each independent position axis before its scalar update. A single
    // temporarily inconsistent vertical sample must not discard simultaneous
    // horizontal information, since XY position is what observes roll/pitch
    // during contact and free-flight braking.
    if (normalizedInnovationSquared > gateSquared) {
      continue;
    }

    float gain[POST_RELEASE_EKF_STATE_DIM];
    float correction[POST_RELEASE_EKF_STATE_DIM];
    for (int row = 0; row < POST_RELEASE_EKF_STATE_DIM; row++) {
      gain[row] = ekf->covariance[row][POSITION_INDEX + axis] /
        scalarInnovationVariance;
      correction[row] = gain[row] * scalarInnovation;
    }

    setIdentity(transition);
    for (int row = 0; row < POST_RELEASE_EKF_STATE_DIM; row++) {
      transition[row][POSITION_INDEX + axis] -= gain[row];
    }
    multiply(transition, ekf->covariance, matrixTmp);
    multiplyRightTranspose(matrixTmp, transition, matrixOut);
    for (int row = 0; row < POST_RELEASE_EKF_STATE_DIM; row++) {
      for (int column = 0; column < POST_RELEASE_EKF_STATE_DIM; column++) {
        matrixOut[row][column] +=
          measurementVariance * gain[row] * gain[column];
      }
    }
    memcpy(ekf->covariance, matrixOut, sizeof(ekf->covariance));

    for (int i = 0; i < 3; i++) {
      ekf->positionM[i] += correction[POSITION_INDEX + i];
      ekf->velocityMps[i] += correction[VELOCITY_INDEX + i];
      ekf->gyroBiasRadPerS[i] += correction[GYRO_BIAS_INDEX + i];
      ekf->accelBiasMps2[i] += correction[ACCEL_BIAS_INDEX + i];
    }
    const float attitudeError[3] = {
      correction[ATTITUDE_INDEX],
      correction[ATTITUDE_INDEX + 1],
      correction[ATTITUDE_INDEX + 2],
    };
    if (!injectAttitude(ekf->quaternionWxyz, attitudeError) ||
        !resetAttitudeCovariance(ekf, attitudeError) ||
        !stateIsFinite(ekf)) {
      return invalidate(ekf, PostReleaseEkfReasonNumericalFailure);
    }
    acceptedAxisCount++;
  }
  if (acceptedAxisCount == 0) {
    ekf->rejectedPositionCount++;
    ekf->reason = PostReleaseEkfReasonPositionRejected;
    return ekf->reason;
  }

  ekf->positionUpdateCount++;
  ekf->reason = PostReleaseEkfReasonPositionFused;
  return ekf->reason;
}

// Fuse a correlated position/velocity front-end estimate as ONE observation.
// Covariance intersection avoids assuming independence from earlier Vicon
// frames already represented in the ESKF. Fixed weight; no optimization.
// Six whitened scalar updates, O(6*N^2), one attitude injection/reset.
bool postReleaseInertialEkfFuseVicon(
    postReleaseInertialEkf_t *e,const float p[3],const float v[3],
    const float R[3][2][2],float weight,uint32_t sourceUs) {
  if(!e || !e->valid || !e->postReleaseActive || !p || !v || !R ||
      !finiteVector(p,3) || !finiteVector(v,3) || !isfinite(weight) ||
      weight<.5f || weight>=1 || !sourceUs ||
      (e->viconFusionTimestampUs && (int32_t)(sourceUs-e->viconFusionTimestampUs)<=0))return false;
  float alpha[3],rp[3],rv[3];
  for(unsigned k=0;k<3;k++) {
    const float a=R[k][0][0],b=.5f*(R[k][0][1]+R[k][1][0]),c=R[k][1][1];
    if(!isfinite(a)||!isfinite(b)||!isfinite(c)||a<=0||c<=0||a*c-b*b<=0)return false;
    alpha[k]=b/a;rp[k]=a/(1-weight);rv[k]=(c-b*b/a)/(1-weight);
  }
  // Validate the entire observation before touching pending covariance. A
  // valid but gated-out measurement still leaves P propagated to the nominal
  // epoch; it does not apply a measurement correction or reset the schedule.
  if (!postReleaseInertialEkfFlushCovariance(e)) { return false; }
  for(unsigned k=0;k<3;k++) {
    const float c=R[k][1][1];
    const float ep=p[k]-e->positionM[k],ev=v[k]-e->velocityMps[k];
    if(ep*ep>36*(e->covariance[k][k]/weight+rp[k]) ||
       ev*ev>36*(e->covariance[k+3][k+3]/weight+c/(1-weight))) {
      e->rejectedViconFusionCount++;return false;
    }
  }
  for(unsigned i=0;i<15;i++)for(unsigned j=0;j<15;j++)e->covariance[i][j]/=weight;
  float dx[15]={0};
  for(unsigned k=0;k<3;k++)for(unsigned n=0;n<2;n++) {
    const unsigned i0=k,i1=k+3;
    const float h0=n?-alpha[k]:1,h1=n?1:0,variance=n?rv[k]:rp[k];
    float hp[15],gain[15];
    for(unsigned i=0;i<15;i++)hp[i]=h0*e->covariance[i0][i]+h1*e->covariance[i1][i];
    const float S=h0*hp[i0]+h1*hp[i1]+variance;
    if(!isfinite(S)||S<=0){invalidate(e,PostReleaseEkfReasonNumericalFailure);return false;}
    const float residual=h0*(p[k]-e->positionM[k]-dx[i0])+h1*(v[k]-e->velocityMps[k]-dx[i1]);
    for(unsigned i=0;i<15;i++){gain[i]=hp[i]/S;dx[i]+=gain[i]*residual;}
    for(unsigned i=0;i<15;i++)for(unsigned j=i;j<15;j++) {
      const float c=e->covariance[i][j]-gain[i]*hp[j]-hp[i]*gain[j]+gain[i]*S*gain[j];
      e->covariance[i][j]=e->covariance[j][i]=c;
    }
  }
  for(unsigned i=0;i<3;i++) {
    e->positionM[i]+=dx[i];e->velocityMps[i]+=dx[i+3];
    e->gyroBiasRadPerS[i]+=dx[i+9];e->accelBiasMps2[i]+=dx[i+12];
  }
  const float dr[3]={dx[6],dx[7],dx[8]};
  if(!injectAttitude(e->quaternionWxyz,dr)||!resetAttitudeCovariance(e,dr)||!stateIsFinite(e)) {
    invalidate(e,PostReleaseEkfReasonNumericalFailure);return false;
  }
  e->positionUpdateCount++;e->viconFusionCount++;e->viconFusionTimestampUs=sourceUs;
  e->reason=PostReleaseEkfReasonPositionFused;
  return true;
}

static bool quaternionResidualRotationVector(
    const float estimateWxyz[4], const float measurementWxyz[4],
    float residual[3]) {
  float measured[4];
  memcpy(measured, measurementWxyz, sizeof(measured));
  if (!normalizeQuaternion(measured)) {
    return false;
  }
  const float dot = estimateWxyz[0] * measured[0] +
    estimateWxyz[1] * measured[1] + estimateWxyz[2] * measured[2] +
    estimateWxyz[3] * measured[3];
  if (dot < 0.0f) {
    for (int i = 0; i < 4; i++) {
      measured[i] = -measured[i];
    }
  }

  const float ew = estimateWxyz[0];
  const float ex = estimateWxyz[1];
  const float ey = estimateWxyz[2];
  const float ez = estimateWxyz[3];
  const float mw = measured[0];
  const float mx = measured[1];
  const float my = measured[2];
  const float mz = measured[3];
  const float errorW = ew * mw + ex * mx + ey * my + ez * mz;
  const float errorVector[3] = {
    ew * mx - ex * mw - ey * mz + ez * my,
    ew * my + ex * mz - ey * mw - ez * mx,
    ew * mz - ex * my + ey * mx - ez * mw,
  };
  const float vectorNorm = sqrtf(
    errorVector[0] * errorVector[0] +
    errorVector[1] * errorVector[1] +
    errorVector[2] * errorVector[2]);
  if (!isfinite(errorW) || !isfinite(vectorNorm)) {
    return false;
  }
  const float scale = vectorNorm < 1.0e-7f ?
    2.0f : 2.0f * atan2f(vectorNorm, fmaxf(errorW, 0.0f)) / vectorNorm;
  for (int i = 0; i < 3; i++) {
    residual[i] = scale * errorVector[i];
  }
  return finiteVector(residual, 3);
}

postReleaseEkfReason_t postReleaseInertialEkfUpdateOrientation(
    postReleaseInertialEkf_t* ekf,
    const postReleaseInertialEkfParams_t* params,
    const float quaternionWxyz[4],
    const float orientationStdRad,
    const uint32_t timestampUs) {
  if (ekf == NULL || !validParams(params)) {
    return invalidate(ekf, PostReleaseEkfReasonInvalidArgument);
  }
  if (quaternionWxyz == NULL || !finiteVector(quaternionWxyz, 4) ||
      !isfinite(orientationStdRad) || orientationStdRad <= 0.0f) {
    ekf->rejectedOrientationCount++;
    ekf->reason = PostReleaseEkfReasonOrientationRejected;
    return ekf->reason;
  }
  if (!ekf->valid) {
    return ekf->reason;
  }
  if (!ekf->postReleaseActive) {
    ekf->preReleaseOrientationIgnoredCount++;
    ekf->reason = PostReleaseEkfReasonOrientationIgnoredBeforeRelease;
    return ekf->reason;
  }
  if (timestampUs != ekf->lastTimestampUs) {
    return invalidate(ekf, PostReleaseEkfReasonTimingOrderViolation);
  }

  float checkedResidual[3];
  if (!quaternionResidualRotationVector(ekf->quaternionWxyz, quaternionWxyz, checkedResidual)) {
    ekf->rejectedOrientationCount++;
    ekf->reason = PostReleaseEkfReasonOrientationRejected;
    return ekf->reason;
  }
  if (!postReleaseInertialEkfFlushCovariance(ekf)) { return ekf->reason; }
  const float measurementVariance = orientationStdRad * orientationStdRad;
  const float gateSquared = params->orientationInnovationGateSigma *
    params->orientationInnovationGateSigma;
  uint8_t acceptedAxisCount = 0;
  for (int axis = 0; axis < 3; axis++) {
    float residual[3];
    if (!quaternionResidualRotationVector(
        ekf->quaternionWxyz, quaternionWxyz, residual)) {
      ekf->rejectedOrientationCount++;
      ekf->reason = PostReleaseEkfReasonOrientationRejected;
      return ekf->reason;
    }
    const float innovationVariance =
      ekf->covariance[ATTITUDE_INDEX + axis][ATTITUDE_INDEX + axis] +
      measurementVariance;
    if (!isfinite(innovationVariance) ||
        innovationVariance <= MIN_COVARIANCE_DIAGONAL) {
      return invalidate(ekf, PostReleaseEkfReasonNumericalFailure);
    }
    const float normalizedInnovationSquared =
      residual[axis] * residual[axis] / innovationVariance;
    if (!isfinite(normalizedInnovationSquared)) {
      return invalidate(ekf, PostReleaseEkfReasonNumericalFailure);
    }
    if (normalizedInnovationSquared > gateSquared) {
      continue;
    }

    float gain[POST_RELEASE_EKF_STATE_DIM];
    float correction[POST_RELEASE_EKF_STATE_DIM];
    for (int row = 0; row < POST_RELEASE_EKF_STATE_DIM; row++) {
      gain[row] = ekf->covariance[row][ATTITUDE_INDEX + axis] /
        innovationVariance;
      correction[row] = gain[row] * residual[axis];
    }
    setIdentity(transition);
    for (int row = 0; row < POST_RELEASE_EKF_STATE_DIM; row++) {
      transition[row][ATTITUDE_INDEX + axis] -= gain[row];
    }
    multiply(transition, ekf->covariance, matrixTmp);
    multiplyRightTranspose(matrixTmp, transition, matrixOut);
    for (int row = 0; row < POST_RELEASE_EKF_STATE_DIM; row++) {
      for (int column = 0; column < POST_RELEASE_EKF_STATE_DIM; column++) {
        matrixOut[row][column] +=
          measurementVariance * gain[row] * gain[column];
      }
    }
    memcpy(ekf->covariance, matrixOut, sizeof(ekf->covariance));

    for (int i = 0; i < 3; i++) {
      ekf->positionM[i] += correction[POSITION_INDEX + i];
      ekf->velocityMps[i] += correction[VELOCITY_INDEX + i];
      ekf->gyroBiasRadPerS[i] += correction[GYRO_BIAS_INDEX + i];
      ekf->accelBiasMps2[i] += correction[ACCEL_BIAS_INDEX + i];
    }
    const float attitudeError[3] = {
      correction[ATTITUDE_INDEX], correction[ATTITUDE_INDEX + 1],
      correction[ATTITUDE_INDEX + 2],
    };
    if (!injectAttitude(ekf->quaternionWxyz, attitudeError) ||
        !resetAttitudeCovariance(ekf, attitudeError) ||
        !stateIsFinite(ekf)) {
      return invalidate(ekf, PostReleaseEkfReasonNumericalFailure);
    }
    acceptedAxisCount++;
  }
  if (acceptedAxisCount == 0) {
    ekf->rejectedOrientationCount++;
    ekf->reason = PostReleaseEkfReasonOrientationRejected;
    return ekf->reason;
  }
  ekf->orientationUpdateCount++;
  ekf->reason = PostReleaseEkfReasonOrientationFused;
  return ekf->reason;
}
