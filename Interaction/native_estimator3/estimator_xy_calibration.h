/* Estimator-3-only, once-per-flight XY specific-force calibration (SI units).
 * No changes to the sensor driver, ordinary KF, Z acceleration or gyro.
 * The estimator task owns active coefficients; CRTP writes staged fields.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

typedef struct {
  float sx, sy, bx, by;
} estimatorXYCoefficients_t;

typedef struct {
  estimatorXYCoefficients_t staged, active;
  uint32_t request, processed, applied;
  uint8_t enabled, status;
} estimatorXYCalibration_t;

/* status: 0 reset, 1 applied/frozen, 2 invalid, 3 wrong control phase,
 * 4 already frozen. Read applied as well as status to identify the ACK. */
static inline bool estimatorXYValid(const estimatorXYCoefficients_t *c) {
  return isfinite(c->sx) && isfinite(c->sy) && isfinite(c->bx) && isfinite(c->by)
    && c->sx >= .8f && c->sx <= 1.2f && c->sy >= .8f && c->sy <= 1.2f
    && fabsf(c->bx) <= .75f && fabsf(c->by) <= .75f;
}

static inline bool estimatorXYProcess(estimatorXYCalibration_t *c,
    bool armed, bool defaultEstimator, bool brakeAccepted) {
  if (!armed) {
    c->request = c->processed = c->applied = 0;
    c->enabled = c->status = 0;
    return false;
  }
  if (c->request == c->processed) return false;
  c->processed = c->request;
  if (c->enabled) { c->status = 4; return false; }
  if (!c->request || !defaultEstimator || brakeAccepted) {
    c->status = 3; return false;
  }
  if (!estimatorXYValid(&c->staged)) { c->status = 2; return false; }
  c->active = c->staged;
  c->enabled = 1;
  c->applied = c->request;
  c->status = 1;
  return true;
}

static inline void estimatorXYCorrect(const estimatorXYCalibration_t *c,
    float specificForce[3]) {
  if (!c->enabled) return;
  specificForce[0] = c->active.sx * specificForce[0] - c->active.bx;
  specificForce[1] = c->active.sy * specificForce[1] - c->active.by;
}
