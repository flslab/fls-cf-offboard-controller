#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// Entire closed-loop attitude response, in Crazyflie roll/pitch setpoint signs.
// These are NOT motor lag/torque coefficients.
typedef struct { float delay, wn, zeta, gain, bias; } prResponseAxis;
typedef struct { prResponseAxis axis[2]; uint32_t id; uint8_t source; } prResponseModel;
static inline bool prResponseValid(const prResponseModel *m) {
  if (!m->id || (m->source != 1 && m->source != 2)) return false;
  for (unsigned i=0;i<2;i++) {
    const prResponseAxis *a=&m->axis[i];
    if (!isfinite(a->delay) || !isfinite(a->wn) || !isfinite(a->zeta) ||
        !isfinite(a->gain) || !isfinite(a->bias) || a->delay<0 || a->delay>=.15f ||
        a->wn<=5 || a->wn>=60 || a->zeta<=.2f || a->zeta>=2 ||
        a->gain<=.7f || a->gain>=1.3f || fabsf(a->bias)>3) return false;
  }
  return true;
}
bool prResponseGet(prResponseModel *out);
void prResponseRecord(uint32_t nowUs, float rollDeg, float pitchDeg, bool powered);
bool prResponseHistory(uint32_t nowUs, float roll[76], float pitch[76]);
