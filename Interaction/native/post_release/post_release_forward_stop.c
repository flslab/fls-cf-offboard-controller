#include "post_release_forward_stop.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define POST_RELEASE_RAD_PER_DEG 0.017453292519943295f

static bool finiteThree(const float value[3]) {
  return value != NULL && isfinite(value[0]) && isfinite(value[1]) &&
    isfinite(value[2]);
}

static float boundedAngle(const float angle, const float bound) {
  return fmaxf(-bound, fminf(bound, angle));
}

bool postReleaseEulerRatesFromBody(const float rollDeg, const float pitchDeg,
    const float rawBodyRatesDegS[3], float eulerRatesDegS[3]) {
  if (!isfinite(rollDeg) || !isfinite(pitchDeg) ||
      fabsf(rollDeg) > 85.0f || fabsf(pitchDeg) > 85.0f ||
      !finiteThree(rawBodyRatesDegS) || eulerRatesDegS == NULL) {
    return false;
  }
  const float phi = rollDeg * POST_RELEASE_RAD_PER_DEG;
  const float theta = -pitchDeg * POST_RELEASE_RAD_PER_DEG;
  const float sp = sinf(phi), cp = cosf(phi);
  const float coupled = sp * rawBodyRatesDegS[1] + cp * rawBodyRatesDegS[2];
  const float result[3] = {
    rawBodyRatesDegS[0] + tanf(theta) * coupled,
    -cp * rawBodyRatesDegS[1] + sp * rawBodyRatesDegS[2],
    coupled / cosf(theta),
  };
  if (!finiteThree(result)) {
    return false;
  }
  memcpy(eulerRatesDegS, result, sizeof(result));
  return true;
}

bool postReleaseBodyRatesFromEuler(const float rollDeg, const float pitchDeg,
    const float eulerRatesDegS[3], float controllerRatesDegS[3]) {
  if (!isfinite(rollDeg) || !isfinite(pitchDeg) ||
      fabsf(rollDeg) > 85.0f || fabsf(pitchDeg) > 85.0f ||
      !finiteThree(eulerRatesDegS) || controllerRatesDegS == NULL) {
    return false;
  }
  const float phi = rollDeg * POST_RELEASE_RAD_PER_DEG;
  const float theta = -pitchDeg * POST_RELEASE_RAD_PER_DEG;
  const float result[3] = {
    eulerRatesDegS[0] - sinf(theta) * eulerRatesDegS[2],
    cosf(phi) * eulerRatesDegS[1] -
      sinf(phi) * cosf(theta) * eulerRatesDegS[2],
    sinf(phi) * eulerRatesDegS[1] +
      cosf(phi) * cosf(theta) * eulerRatesDegS[2],
  };
  if (!finiteThree(result)) {
    return false;
  }
  // controller_pid compares desired pitch rate against -sensors->gyro.y.
  memcpy(controllerRatesDegS, result, sizeof(result));
  return true;
}

static bool validUnwindProfile(const postReleaseUnwindProfile_t* profile) {
  if (profile == NULL || !isfinite(profile->durationS) ||
      profile->durationS < 0.05f || profile->durationS > 1.0f) {
    return false;
  }
  for (unsigned axis = 0; axis < 3; axis++) {
    for (unsigned power = 0; power < 8; power++) {
      if (!isfinite(profile->coefficient[axis][power])) {
        return false;
      }
    }
  }
  return true;
}

bool postReleaseUnwindPlan(const float anglesDeg[3],
    const float ratesDegS[3], const float accelerationsDegS2[3],
    const float jerksDegS3[3], const float durationS,
    postReleaseUnwindProfile_t* profile) {
  if (profile == NULL || !finiteThree(anglesDeg) || !finiteThree(ratesDegS) ||
      !finiteThree(accelerationsDegS2) || !finiteThree(jerksDegS3) ||
      !isfinite(durationS) || durationS < 0.05f || durationS > 1.0f) {
    return false;
  }
  postReleaseUnwindProfile_t proposed = {.durationS = durationS};
  for (unsigned axis = 0; axis < 3; axis++) {
    float* c = proposed.coefficient[axis];
    // Normalized-time Hermite coefficients: preserve measured angle/rate and
    // the caller's continuous reference acceleration/jerk on replanning.
    c[0] = anglesDeg[axis];
    c[1] = ratesDegS[axis] * durationS;
    c[2] = accelerationsDegS2[axis] * durationS * durationS * 0.5f;
    c[3] = jerksDegS3[axis] * durationS * durationS * durationS / 6.0f;
    c[4] = -35.0f*c[0] - 20.0f*c[1] - 10.0f*c[2] - 4.0f*c[3];
    c[5] = 84.0f*c[0] + 45.0f*c[1] + 20.0f*c[2] + 6.0f*c[3];
    c[6] = -70.0f*c[0] - 36.0f*c[1] - 15.0f*c[2] - 4.0f*c[3];
    c[7] = 20.0f*c[0] + 10.0f*c[1] + 4.0f*c[2] + c[3];
  }
  if (!validUnwindProfile(&proposed)) {
    return false;
  }
  *profile = proposed;
  return true;
}

bool postReleaseUnwindEvaluate(const postReleaseUnwindProfile_t* profile,
    const float elapsedS, float anglesDeg[3], float ratesDegS[3],
    float accelerationsDegS2[3], float jerksDegS3[3]) {
  if (!validUnwindProfile(profile) || !isfinite(elapsedS) || elapsedS < 0.0f ||
      anglesDeg == NULL || ratesDegS == NULL || accelerationsDegS2 == NULL ||
      jerksDegS3 == NULL) {
    return false;
  }
  float result[4][3] = {{0}};
  if (elapsedS < profile->durationS) {
    const float s = elapsedS / profile->durationS;
    const float inverseT = 1.0f / profile->durationS;
    for (unsigned axis = 0; axis < 3; axis++) {
      const float* c = profile->coefficient[axis];
      float p = c[7], v = 7.0f*c[7], a = 42.0f*c[7], j = 210.0f*c[7];
      for (int k = 6; k >= 0; k--) {
        p = p*s + c[k];
        if (k >= 1) v = v*s + k*c[k];
        if (k >= 2) a = a*s + k*(k-1)*c[k];
        if (k >= 3) j = j*s + k*(k-1)*(k-2)*c[k];
      }
      result[0][axis] = p;
      result[1][axis] = v * inverseT;
      result[2][axis] = a * inverseT * inverseT;
      result[3][axis] = j * inverseT * inverseT * inverseT;
    }
  }
  for (unsigned derivative = 0; derivative < 4; derivative++) {
    if (!finiteThree(result[derivative])) {
      return false;
    }
  }
  memcpy(anglesDeg, result[0], sizeof(result[0]));
  memcpy(ratesDegS, result[1], sizeof(result[1]));
  memcpy(accelerationsDegS2, result[2], sizeof(result[2]));
  memcpy(jerksDegS3, result[3], sizeof(result[3]));
  return true;
}

bool postReleaseUnwindFeasible(const postReleaseUnwindProfile_t* profile) {
  if (!validUnwindProfile(profile)) {
    return false;
  }
  // This is a sampled geometric/numerical screen, not a torque, inertia or
  // identified actuator-tracking guarantee. The 35-degree tilt envelope is
  // unchanged; yaw is an offset from the release heading, bounded to 45 deg.
  for (unsigned sample = 0; sample <= 32; sample++) {
    float angle[3], rate[3], acceleration[3], jerk[3];
    if (!postReleaseUnwindEvaluate(profile, profile->durationS * sample / 32.0f,
        angle, rate, acceleration, jerk) || fabsf(angle[0]) > 35.0f ||
        fabsf(angle[1]) > 35.0f || fabsf(angle[2]) > 45.0f) {
      return false;
    }
  }
  return true;
}

static float unwindProjectedBraking(const float anglesDeg[3],
    const float holdYawDeg, const float directionXY[2]) {
  const float phi = anglesDeg[0] * POST_RELEASE_RAD_PER_DEG;
  const float theta = -anglesDeg[1] * POST_RELEASE_RAD_PER_DEG;
  const float psi = (holdYawDeg + anglesDeg[2]) * POST_RELEASE_RAD_PER_DEG;
  // Exact horizontal/vertical thrust-direction ratio for Rz(psi)Ry(theta)
  // Rx(phi), assuming approximately maintained vertical thrust. This still
  // excludes vertical thrust dynamics and aerodynamic/contact forces.
  const float yawFrameAx = 9.81f * tanf(theta);
  const float yawFrameAy = -9.81f * tanf(phi) / cosf(theta);
  const float ax = cosf(psi)*yawFrameAx - sinf(psi)*yawFrameAy;
  const float ay = sinf(psi)*yawFrameAx + cosf(psi)*yawFrameAy;
  return -(ax*directionXY[0] + ay*directionXY[1]);
}

static bool validUnwindDirection(const float directionXY[2]) {
  return directionXY != NULL && isfinite(directionXY[0]) &&
    isfinite(directionXY[1]) &&
    fabsf(hypotf(directionXY[0], directionXY[1]) - 1.0f) <= 0.01f;
}

bool postReleaseUnwindRemainingImpulse(
    const postReleaseUnwindProfile_t* profile, const float elapsedS,
    const float holdYawDeg, const float directionXY[2], float* impulseMps) {
  if (!validUnwindProfile(profile) || !isfinite(elapsedS) || elapsedS < 0.0f ||
      !isfinite(holdYawDeg) || !validUnwindDirection(directionXY) ||
      impulseMps == NULL) {
    return false;
  }
  const float remainingS = fmaxf(0.0f, profile->durationS - elapsedS);
  const float stepS = remainingS / 24.0f;
  float impulse = 0.0f;
  for (unsigned sample = 0; sample < 24; sample++) {
    float angle[3], rate[3], acceleration[3], jerk[3];
    if (!postReleaseUnwindEvaluate(profile, elapsedS + (sample + 0.5f)*stepS,
        angle, rate, acceleration, jerk) || fabsf(angle[0]) > 35.0f ||
        fabsf(angle[1]) > 35.0f || fabsf(angle[2]) > 45.0f) {
      return false;
    }
    impulse += unwindProjectedBraking(angle, holdYawDeg, directionXY) * stepS;
  }
  if (!isfinite(impulse)) {
    return false;
  }
  *impulseMps = impulse;
  return true;
}

postReleaseUnwindSelection_t postReleaseRateAwareUnwindSelect(const float velocityXY[2],
    const float directionXY[2], const float anglesDeg[3],
    const float eulerRatesDegS[3], const float holdYawDeg,
    const float responseTimeS, const float speedReserveMps,
    postReleaseUnwindProfile_t* profile, float* predictedImpulseMps) {
  if (velocityXY == NULL || !isfinite(velocityXY[0]) ||
      !isfinite(velocityXY[1]) || !validUnwindDirection(directionXY) ||
      !finiteThree(anglesDeg) || !finiteThree(eulerRatesDegS) ||
      !isfinite(holdYawDeg) || !isfinite(responseTimeS) ||
      responseTimeS < 0.0f || responseTimeS > 0.25f ||
      !isfinite(speedReserveMps) || speedReserveMps < 0.0f ||
      speedReserveMps > 0.5f || profile == NULL || predictedImpulseMps == NULL) {
    return PostReleaseUnwindInfeasible;
  }
  // Keep the calibrated response tail, but account for outward motion during
  // it. Constant measured rate with clipped angles is a bounded approximation
  // of residual impulse, not identification of inertia or motor dynamics.
  float tailImpulse = 0.0f;
  const float tailStepS = responseTimeS / 8.0f;
  for (unsigned sample = 0; sample < 8; sample++) {
    const float t = (sample + 0.5f)*tailStepS;
    float predicted[3];
    for (unsigned axis = 0; axis < 3; axis++) {
      const float angle = anglesDeg[axis] + eulerRatesDegS[axis]*t;
      if (!isfinite(angle)) {
        return PostReleaseUnwindInfeasible;
      }
      predicted[axis] = boundedAngle(angle, axis < 2 ? 35.0f : 45.0f);
    }
    tailImpulse += fmaxf(0.0f,
      unwindProjectedBraking(predicted, holdYawDeg, directionXY)) * tailStepS;
  }
  const float forwardSpeed = velocityXY[0]*directionXY[0] +
    velocityXY[1]*directionXY[1];
  if (!isfinite(forwardSpeed) || !isfinite(tailImpulse)) {
    return PostReleaseUnwindInfeasible;
  }
  static const float horizons[] = {0.20f, 0.25f, 0.30f, 0.35f, 0.40f};
  const float zero[3] = {0.0f, 0.0f, 0.0f};
  bool feasibleCandidate = false;
  for (unsigned i = 0; i < sizeof(horizons)/sizeof(horizons[0]); i++) {
    postReleaseUnwindProfile_t candidate;
    float impulse;
    if (postReleaseUnwindPlan(anglesDeg, eulerRatesDegS, zero, zero,
          horizons[i], &candidate) && postReleaseUnwindFeasible(&candidate) &&
        postReleaseUnwindRemainingImpulse(&candidate, 0.0f, holdYawDeg,
          directionXY, &impulse) && isfinite(impulse + tailImpulse)) {
      feasibleCandidate = true;
      if (forwardSpeed <= impulse + tailImpulse + speedReserveMps) {
        *profile = candidate;
        *predictedImpulseMps = impulse + tailImpulse;
        return PostReleaseUnwindReady;
      }
    }
  }
  return feasibleCandidate ? PostReleaseUnwindWaiting : PostReleaseUnwindInfeasible;
}

float postReleaseSepticUnwindWeight(const float normalizedTime) {
  const float s = fmaxf(0.0f, fminf(1.0f, normalizedTime));
  const float s2 = s * s;
  const float s4 = s2 * s2;
  const float smooth = s4 * (35.0f + s * (-84.0f + s * (70.0f - 20.0f * s)));
  return 1.0f - smooth;
}

bool postReleaseBlindBrakeLatestDelay(
    const float forwardSpeedMps, const float maxBrakingAccelerationMps2,
    const float responseTimeS, const float forwardSpeedMarginMps,
    float* delayS) {
  if (delayS == NULL || !isfinite(forwardSpeedMps) ||
      forwardSpeedMps < 0.0f ||
      !isfinite(maxBrakingAccelerationMps2) ||
      maxBrakingAccelerationMps2 <= 0.0f ||
      maxBrakingAccelerationMps2 > 4.0f ||
      !isfinite(responseTimeS) || responseTimeS < 0.02f ||
      responseTimeS > 0.20f || !isfinite(forwardSpeedMarginMps) ||
      forwardSpeedMarginMps < 0.0f || forwardSpeedMarginMps > 0.5f) {
    return false;
  }
  const float shortestUnwindS = 0.20f;
  const float reservedSpeedMps = maxBrakingAccelerationMps2 *
    (0.5f * shortestUnwindS + responseTimeS) + forwardSpeedMarginMps;
  *delayS = fmaxf(0.0f,
    (forwardSpeedMps - reservedSpeedMps) /
      maxBrakingAccelerationMps2);
  // The rapid-attitude stage has a one-second timeout. Admit a fast release
  // only when its latest unwind switch fits inside that existing horizon.
  return isfinite(*delayS) && *delayS <= 1.0f;
}

bool postReleaseBlindReleaseStateSafe(
    const float positionM[3], const float velocityMps[3],
    const uint32_t trustedStateAgeUs) {
  if (positionM == NULL || velocityMps == NULL ||
      trustedStateAgeUs > 60000u) {
    return false;
  }
  for (int axis = 0; axis < 3; axis++) {
    if (!isfinite(positionM[axis]) || !isfinite(velocityMps[axis])) {
      return false;
    }
  }
  const float ageS = trustedStateAgeUs * 1.0e-6f;
  const float speedUncertainty = 4.0f * ageS;
  const float speedXY = hypotf(velocityMps[0], velocityMps[1]);
  const float lowestHeight = positionM[2] -
    fabsf(velocityMps[2]) * ageS - 2.0f * ageS * ageS;
  return speedXY - speedUncertainty >= 0.10f &&
    fabsf(velocityMps[2]) + speedUncertainty <= 0.60f &&
    lowestHeight >= 0.50f;
}

postReleaseBlindDecision_t postReleaseBlindBrakeDecision(
    const uint8_t stage, const uint32_t nowUs,
    const uint32_t lastTrustedUs, const uint32_t latestUnwindUs,
    const uint32_t releaseStartUs, const uint32_t rapidSetpointCount) {
  if ((stage != 1u && stage != 2u && stage != 3u && stage != 5u) ||
      (uint32_t)(nowUs - lastTrustedUs) >= 300000u ||
      (stage == 1u && rapidSetpointCount == 0u &&
       (uint32_t)(nowUs - releaseStartUs) >= 50000u)) {
    return PostReleaseBlindAbort;
  }
  if (stage == 1u && rapidSetpointCount > 0u &&
      (int32_t)(nowUs - latestUnwindUs) >= 0) {
    return PostReleaseBlindStartUnwind;
  }
  return PostReleaseBlindContinue;
}

bool postReleaseSelectUnwindDuration(
    const float velocityXY[2], const float directionXY[2],
    const float rollDeg, const float pitchDeg, const float yawDeg,
    const float responseTimeS, const float forwardSpeedMarginMps,
    float* durationS) {
  if (velocityXY == NULL || directionXY == NULL || durationS == NULL ||
      !isfinite(velocityXY[0]) || !isfinite(velocityXY[1]) ||
      !isfinite(directionXY[0]) || !isfinite(directionXY[1]) ||
      !isfinite(rollDeg) || !isfinite(pitchDeg) || !isfinite(yawDeg) ||
      !isfinite(responseTimeS) || responseTimeS < 0.0f ||
      responseTimeS > 0.25f || !isfinite(forwardSpeedMarginMps) ||
      forwardSpeedMarginMps < 0.0f || forwardSpeedMarginMps > 0.5f ||
      fabsf(hypotf(directionXY[0], directionXY[1]) - 1.0f) > 0.01f) {
    return false;
  }
  // Evaluate supported horizons shortest first, so the strong-brake phase
  // continues until one has enough impulse, without locking to 0.25 s.
  // A shorter curve also limits unwanted impulse at low forward speed.
  static const float horizons[] = {0.20f, 0.25f, 0.30f, 0.35f, 0.40f};
  for (unsigned i = 0; i < sizeof(horizons) / sizeof(horizons[0]); i++) {
    if (postReleaseShouldStartUnwind(velocityXY, directionXY, rollDeg,
          pitchDeg, yawDeg, horizons[i], responseTimeS,
          forwardSpeedMarginMps)) {
      *durationS = horizons[i];
      return true;
    }
  }
  return false;
}

bool postReleaseRapidBrakeAttitude(
    const float directionXY[2], const float yawDeg,
    const float decelerationMps2, float* rollDeg, float* pitchDeg) {
  if (directionXY == NULL || rollDeg == NULL || pitchDeg == NULL ||
      !isfinite(directionXY[0]) || !isfinite(directionXY[1]) ||
      fabsf(hypotf(directionXY[0], directionXY[1]) - 1.0f) > 0.01f ||
      !isfinite(yawDeg) || !isfinite(decelerationMps2) ||
      decelerationMps2 <= 0.0f || decelerationMps2 > 4.0f) {
    return false;
  }
  const float yaw = yawDeg * 0.017453292519943295f;
  const float worldAx = -decelerationMps2 * directionXY[0];
  const float worldAy = -decelerationMps2 * directionXY[1];
  const float bodyAx = cosf(yaw) * worldAx + sinf(yaw) * worldAy;
  const float bodyAy = -sinf(yaw) * worldAx + cosf(yaw) * worldAy;
  *pitchDeg = -atanf(bodyAx / 9.81f) * 57.29577951308232f;
  *rollDeg = -atanf(bodyAy / 9.81f) * 57.29577951308232f;
  return isfinite(*rollDeg) && isfinite(*pitchDeg);
}

bool postReleaseShouldStartUnwind(
    const float velocityXY[2], const float directionXY[2],
    const float rollDeg, const float pitchDeg, const float yawDeg,
    const float unwindDurationS, const float responseTimeS,
    const float forwardSpeedMarginMps) {
  if (velocityXY == NULL || directionXY == NULL ||
      !isfinite(velocityXY[0]) || !isfinite(velocityXY[1]) ||
      !isfinite(directionXY[0]) || !isfinite(directionXY[1]) ||
      fabsf(hypotf(directionXY[0], directionXY[1]) - 1.0f) > 0.01f ||
      !isfinite(rollDeg) || !isfinite(pitchDeg) || !isfinite(yawDeg) ||
      fabsf(rollDeg) > 35.0f || fabsf(pitchDeg) > 35.0f ||
      !isfinite(unwindDurationS) || unwindDurationS < 0.1f ||
      unwindDurationS > 1.0f || !isfinite(responseTimeS) ||
      responseTimeS < 0.0f || responseTimeS > 0.25f ||
      !isfinite(forwardSpeedMarginMps) || forwardSpeedMarginMps < 0.0f ||
      forwardSpeedMarginMps > 0.5f) {
    return false;
  }
  const float yaw = yawDeg * 0.017453292519943295f;
  const float bodyAx = -9.81f * tanf(pitchDeg * 0.017453292519943295f);
  const float bodyAy = -9.81f * tanf(rollDeg * 0.017453292519943295f);
  const float worldAx = cosf(yaw) * bodyAx - sinf(yaw) * bodyAy;
  const float worldAy = sinf(yaw) * bodyAx + cosf(yaw) * bodyAy;
  const float brakingAcceleration = fmaxf(0.0f,
    -(worldAx * directionXY[0] + worldAy * directionXY[1]));
  const float forwardSpeed = velocityXY[0] * directionXY[0] +
    velocityXY[1] * directionXY[1];
  // For this symmetric septic attitude interpolation, the attitude-weight
  // integral is T/2. The extra actuator tail and margin make the switch
  // conservative against delayed leveling; truth must still validate it.
  const float remainingBrakeImpulse = brakingAcceleration *
    (0.5f * unwindDurationS + responseTimeS) + forwardSpeedMarginMps;
  return forwardSpeed <= remainingBrakeImpulse;
}

bool postReleaseForwardStopTarget(
    const float positionM[3], const float velocityMps[3],
    const float durationS, const float maxDecelerationMps2,
    float targetM[3], float directionXY[2]) {
  if (positionM == NULL || velocityMps == NULL || targetM == NULL ||
      directionXY == NULL || !isfinite(durationS) || durationS < 0.1f ||
      durationS > 3.0f || !isfinite(maxDecelerationMps2) ||
      maxDecelerationMps2 <= 0.0f || maxDecelerationMps2 > 4.0f) {
    return false;
  }
  for (int axis = 0; axis < 3; axis++) {
    if (!isfinite(positionM[axis]) || !isfinite(velocityMps[axis])) {
      return false;
    }
  }
  const float speed = hypotf(velocityMps[0], velocityMps[1]);
  if (speed < 0.10f || speed > 1.5f || fabsf(velocityMps[2]) > 0.10f ||
      (1.875f * speed / durationS) > maxDecelerationMps2 ||
      0.5f * speed * durationS > 0.80f) {
    return false;
  }
  // v(t) = v0 * (1 - 10s^3 + 15s^4 - 6s^5), s=t/T.
  // Its integral is v0*T/2. Therefore the ordinary seventh-order go_to
  // polynomial with zero initial acceleration and this endpoint is monotone
  // along the measured velocity and has peak deceleration 1.875*|v0|/T.
  const float proposed[3] = {
    positionM[0] + 0.5f * durationS * velocityMps[0],
    positionM[1] + 0.5f * durationS * velocityMps[1],
    positionM[2],
  };
  if (!isfinite(proposed[0]) || !isfinite(proposed[1])) {
    return false;
  }
  for (int axis = 0; axis < 3; axis++) {
    targetM[axis] = proposed[axis];
  }
  directionXY[0] = velocityMps[0] / speed;
  directionXY[1] = velocityMps[1] / speed;
  return true;
}

static float limit(const float value, const float magnitude) {
  return fmaxf(-magnitude, fminf(magnitude, value));
}

bool postReleaseForwardStopFeedback(
    const float referencePositionXY[2], const float referenceVelocityXY[2],
    const float measuredPositionXY[2], const float directionXY[2],
    float commandVelocityXY[2]) {
  if (referencePositionXY == NULL || referenceVelocityXY == NULL ||
      measuredPositionXY == NULL || directionXY == NULL ||
      commandVelocityXY == NULL) {
    return false;
  }
  for (int axis = 0; axis < 2; axis++) {
    if (!isfinite(referencePositionXY[axis]) ||
        !isfinite(referenceVelocityXY[axis]) ||
        !isfinite(measuredPositionXY[axis]) ||
        !isfinite(directionXY[axis])) {
      return false;
    }
  }
  const float directionNorm = hypotf(directionXY[0], directionXY[1]);
  if (fabsf(directionNorm - 1.0f) > 0.01f) {
    return false;
  }
  const float proposedX = referenceVelocityXY[0] + limit(
    1.5f * (referencePositionXY[0] - measuredPositionXY[0]), 0.30f);
  const float proposedY = referenceVelocityXY[1] + limit(
    1.5f * (referencePositionXY[1] - measuredPositionXY[1]), 0.30f);
  // Project and clamp independently. Unlike a fixed terminal XY PID goal,
  // overshooting the reference cannot command a backward velocity.
  const float along = fmaxf(0.0f, fminf(1.5f,
    proposedX * directionXY[0] + proposedY * directionXY[1]));
  // During the free stop, a lateral correction to a *predicted* endpoint
  // introduces a second, sometimes opposing horizontal trajectory. Track
  // the measured release direction only; any lateral position hold begins
  // after the independently verified terminal window has settled.
  commandVelocityXY[0] = along * directionXY[0];
  commandVelocityXY[1] = along * directionXY[1];
  return isfinite(commandVelocityXY[0]) && isfinite(commandVelocityXY[1]);
}

bool postReleaseForwardStopTailGuard(
    const float measuredVelocityXY[2], const float directionXY[2],
    const float rollDeg, const float pitchDeg, const float yawDeg,
    const float responseTimeS, float commandVelocityXY[2]) {
  if (measuredVelocityXY == NULL || directionXY == NULL ||
      commandVelocityXY == NULL || !isfinite(rollDeg) ||
      !isfinite(pitchDeg) || !isfinite(yawDeg) ||
      !isfinite(responseTimeS) || responseTimeS < 0.0f ||
      responseTimeS > 0.25f || fabsf(rollDeg) > 30.0f ||
      fabsf(pitchDeg) > 30.0f) {
    return false;
  }
  for (int axis = 0; axis < 2; axis++) {
    if (!isfinite(measuredVelocityXY[axis]) ||
        !isfinite(directionXY[axis])) {
      return false;
    }
  }
  const float directionNorm = hypotf(directionXY[0], directionXY[1]);
  if (fabsf(directionNorm - 1.0f) > 0.01f) {
    return false;
  }
  const float radiansPerDegree = 0.017453292519943295f;
  const float yawRad = yawDeg * radiansPerDegree;
  const float bodyAx = -9.81f * tanf(pitchDeg * radiansPerDegree);
  const float bodyAy = -9.81f * tanf(rollDeg * radiansPerDegree);
  const float worldAx = cosf(yawRad) * bodyAx - sinf(yawRad) * bodyAy;
  const float worldAy = sinf(yawRad) * bodyAx + cosf(yawRad) * bodyAy;
  const float projectedAccel = worldAx * directionXY[0] +
                               worldAy * directionXY[1];
  if (!isfinite(projectedAccel)) {
    return false;
  }
  // This is not an endpoint offset: it vanishes as the actual braking tilt
  // levels, and it can never ask the velocity PID to fly backward.
  const float forwardSpeed = fminf(0.40f,
    fmaxf(0.0f, -projectedAccel * responseTimeS));
  commandVelocityXY[0] = forwardSpeed * directionXY[0];
  commandVelocityXY[1] = forwardSpeed * directionXY[1];
  return true;
}

bool postReleaseForwardStopProfileGuard(
    const float baseCommandVelocityXY[2],
    const float measuredVelocityXY[2], const float directionXY[2],
    const float rollDeg, const float pitchDeg, const float yawDeg,
    const float responseTimeS, float commandVelocityXY[2]) {
  if (baseCommandVelocityXY == NULL || measuredVelocityXY == NULL ||
      directionXY == NULL || commandVelocityXY == NULL ||
      !isfinite(baseCommandVelocityXY[0]) ||
      !isfinite(baseCommandVelocityXY[1])) {
    return false;
  }
  float guardVelocityXY[2];
  if (!postReleaseForwardStopTailGuard(measuredVelocityXY, directionXY,
        rollDeg, pitchDeg, yawDeg, responseTimeS, guardVelocityXY)) {
    return false;
  }
  const float baseAlong = baseCommandVelocityXY[0] * directionXY[0] +
                          baseCommandVelocityXY[1] * directionXY[1];
  const float measuredAlong = measuredVelocityXY[0] * directionXY[0] +
                              measuredVelocityXY[1] * directionXY[1];
  const float guardAlong = guardVelocityXY[0] * directionXY[0] +
                           guardVelocityXY[1] * directionXY[1];
  if (!isfinite(baseAlong) || !isfinite(measuredAlong) ||
      !isfinite(guardAlong) || baseAlong < -0.001f) {
    return false;
  }
  const float lowSpeedWeight =
    fmaxf(0.0f, 1.0f - fmaxf(0.0f, measuredAlong) / 0.75f);
  const float forwardCommand = fminf(1.5f,
    fmaxf(0.0f, baseAlong + guardAlong * lowSpeedWeight));
  commandVelocityXY[0] = forwardCommand * directionXY[0];
  commandVelocityXY[1] = forwardCommand * directionXY[1];
  return true;
}

void postReleaseForwardStopTerminalReset(
    postReleaseForwardStopTerminalState_t* terminal) {
  if (terminal == NULL) {
    return;
  }
  *terminal = (postReleaseForwardStopTerminalState_t){0};
}

static bool terminalUpdate(
    postReleaseForwardStopTerminalState_t* terminal, const uint32_t nowUs,
    const float ordinarySpeedMps, const float verifiedSpeedMps,
    const float rollDeg, const float pitchDeg,
    const float bodyRatesDegPerS[3]) {
  if (terminal == NULL || !isfinite(ordinarySpeedMps) ||
      !isfinite(verifiedSpeedMps) || !isfinite(rollDeg) ||
      !isfinite(pitchDeg) || (bodyRatesDegPerS != NULL &&
      (!isfinite(bodyRatesDegPerS[0]) ||
       !isfinite(bodyRatesDegPerS[1]) ||
       !isfinite(bodyRatesDegPerS[2])))) {
    postReleaseForwardStopTerminalReset(terminal);
    return false;
  }

  const bool previousValid = terminal->hasPrevious;
  const uint32_t deltaUs = nowUs - terminal->previousTimeUs;
  const float rollRateDegS = bodyRatesDegPerS != NULL ?
    fabsf(bodyRatesDegPerS[0]) : previousValid && deltaUs > 0 ?
    fabsf(rollDeg - terminal->previousRollDeg) * 1e6f / deltaUs : INFINITY;
  const float pitchRateDegS = bodyRatesDegPerS != NULL ?
    fabsf(bodyRatesDegPerS[1]) : previousValid && deltaUs > 0 ?
    fabsf(pitchDeg - terminal->previousPitchDeg) * 1e6f / deltaUs : INFINITY;
  const bool regularSample = previousValid && deltaUs >= 5000u &&
    deltaUs <= 30000u;
  const bool settled = regularSample &&
    ordinarySpeedMps >= 0.0f && ordinarySpeedMps <= 0.03f &&
    verifiedSpeedMps >= 0.0f && verifiedSpeedMps <= 0.03f &&
    fabsf(rollDeg) <= 3.0f && fabsf(pitchDeg) <= 3.0f &&
    rollRateDegS <= 5.0f && pitchRateDegS <= 5.0f &&
    (bodyRatesDegPerS == NULL || fabsf(bodyRatesDegPerS[2]) <= 5.0f);

  terminal->previousTimeUs = nowUs;
  terminal->previousRollDeg = rollDeg;
  terminal->previousPitchDeg = pitchDeg;
  terminal->hasPrevious = true;
  if (!settled) {
    terminal->hasStableWindow = false;
    return false;
  }
  if (!terminal->hasStableWindow) {
    terminal->stableSinceUs = nowUs;
    terminal->hasStableWindow = true;
    return false;
  }
  return nowUs - terminal->stableSinceUs >= 200000u;
}

bool postReleaseForwardStopTerminalUpdate(
    postReleaseForwardStopTerminalState_t* terminal, const uint32_t nowUs,
    const float ordinarySpeedMps, const float verifiedSpeedMps,
    const float rollDeg, const float pitchDeg) {
  return terminalUpdate(terminal, nowUs, ordinarySpeedMps,
    verifiedSpeedMps, rollDeg, pitchDeg, NULL);
}

bool postReleaseForwardStopTerminalUpdateWithBodyRates(
    postReleaseForwardStopTerminalState_t* terminal, const uint32_t nowUs,
    const float ordinarySpeedMps, const float verifiedSpeedMps,
    const float rollDeg, const float pitchDeg,
    const float bodyRatesDegPerS[3]) {
  if (bodyRatesDegPerS == NULL) {
    postReleaseForwardStopTerminalReset(terminal);
    return false;
  }
  return terminalUpdate(terminal, nowUs, ordinarySpeedMps,
    verifiedSpeedMps, rollDeg, pitchDeg, bodyRatesDegPerS);
}
