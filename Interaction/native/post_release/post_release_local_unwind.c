#include "post_release_local_unwind.h"

#include <math.h>
#include <string.h>

#define RAD 0.017453292519943295f

static bool finite3(const float v[3]) {
  return v && isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static bool validState(const jointState_t* s) {
  if (!s || !finite3(s->angle) || !finite3(s->rate) ||
      !isfinite(s->velocity[0]) || !isfinite(s->velocity[1])) return false;
  for (unsigned i=0; i<3; ++i) {
    if (fabsf(s->angle[i]) > (i<2 ? 35.0f : 45.0f) ||
        fabsf(s->rate[i]) > 220.0f) return false;
  }
  return true;
}

// Held-altitude thrust projection, same CF convention as jointPredict.
// Its exact first time derivative includes roll, pitch AND yaw motion.
static bool acceleration(const jointState_t* s, const jointModel_t* m,
    float yaw, float a[2], float j[2]) {
  const float phi=s->angle[0]*RAD, theta=-s->angle[1]*RAD;
  const float psi=(yaw+s->angle[2])*RAD;
  const float pd=s->rate[0]*RAD, td=-s->rate[1]*RAD, yd=s->rate[2]*RAD;
  const float tp=tanf(phi), tt=tanf(theta), ct=cosf(theta);
  const float c=cosf(psi), si=sinf(psi);
  const float bx=9.81f*tt, by=-9.81f*tp/ct;
  const float dx=9.81f*(1.0f+tt*tt)*td;
  const float dy=-9.81f*((1.0f+tp*tp)*pd+tp*tt*td)/ct;
  a[0]=c*bx-si*by+m->accelerationBias[0];
  a[1]=si*bx+c*by+m->accelerationBias[1];
  j[0]=c*dx-si*dy-yd*(si*bx+c*by);
  j[1]=si*dx+c*dy+yd*(c*bx-si*by);
  return isfinite(a[0]) && isfinite(a[1]) && isfinite(j[0]) && isfinite(j[1]);
}

static float along(const jointState_t* state, const float dir[2]) {
  return state->velocity[0]*dir[0]+state->velocity[1]*dir[1];
}

// Midpoint reference/error/I + exponential rate response with its exact angle
// integral. Velocity uses trapezoidal actual thrust projection, not command
// tilt and not a maximum-a constant. No iterative solve is involved.
static bool step(jointState_t* s, float integral[3], const jointModel_t* m,
    const float reference[3], const float referenceRate[3], float yaw,
    float dt, const float alpha[3]) {
  float a0[2],a1[2],j[2];
  if (!acceleration(s,m,yaw,a0,j)) return false;
  for (unsigned i=0; i<3; ++i) {
    if (!isfinite(reference[i]) || !isfinite(referenceRate[i]) ||
        fabsf(reference[i]) > (i<2 ? 35.0f : 45.0f)) return false;
    const float error=reference[i]-(s->angle[i]+.5f*dt*s->rate[i]);
    const float integralMid=integral[i]+.5f*m->integralGain[i]*error*dt;
    const float desired=referenceRate[i]+m->attitudeGain[i]*error+integralMid;
    const float next=s->rate[i]+(desired-s->rate[i])*alpha[i];
    s->angle[i]+=desired*dt+(s->rate[i]-desired)*m->tau[i]*alpha[i];
    integral[i]+=m->integralGain[i]*error*dt;
    s->rate[i]=next;
  }
  if (!validState(s) || !finite3(integral) || !acceleration(s,m,yaw,a1,j)) return false;
  for (unsigned i=0; i<2; ++i) s->velocity[i]+=.5f*(a0[i]+a1[i])*dt;
  return isfinite(s->velocity[0]) && isfinite(s->velocity[1]);
}

static bool predict(const postReleaseUnwindProfile_t* profile,
    jointState_t* s, float integral[3], const jointModel_t* m,
    float yaw, const float dir[2], float horizon, float* minimum) {
  const float dt=horizon/POST_RELEASE_LOCAL_UNWIND_STEPS;
  const float alpha[3]={1-expf(-dt/m->tau[0]),1-expf(-dt/m->tau[1]),
                       1-expf(-dt/m->tau[2])};
  for (unsigned k=0; k<POST_RELEASE_LOCAL_UNWIND_STEPS; ++k) {
    float reference[3],rate[3],accel[3],jerk[3];
    if (!postReleaseUnwindEvaluate(profile,(k+.5f)*dt,reference,rate,accel,jerk) ||
        !step(s,integral,m,reference,rate,yaw,dt,alpha)) return false;
    *minimum=fminf(*minimum,along(s,dir));
  }
  return true;
}

bool postReleaseLocalUnwindDecide(const jointState_t* state,
    const jointModel_t* model, const float reference[3],
    const float referenceRate[3], const float referenceAccel[3],
    const float referenceJerk[3], float holdYawDeg, const float direction[2],
    float unwindDurationS, float nextControlTickS, float forwardReserveMps,
    postReleaseLocalUnwindDecision_t* decision) {
  if (!decision || !model || !validState(state) || !direction ||
      !isfinite(direction[0]) || !isfinite(direction[1]) ||
      fabsf(hypotf(direction[0],direction[1])-1.0f) > .01f ||
      !finite3(reference) || !finite3(referenceRate) ||
      !finite3(referenceAccel) || !finite3(referenceJerk) ||
      !finite3(model->tau) || !finite3(model->attitudeGain) ||
      !finite3(model->integralRateBias) || !finite3(model->integralGain) ||
      !isfinite(holdYawDeg) || !isfinite(unwindDurationS) ||
      unwindDurationS < .05f || unwindDurationS > 1.0f ||
      !isfinite(nextControlTickS) || nextControlTickS < .001f || nextControlTickS > .03f ||
      !isfinite(forwardReserveMps) || forwardReserveMps < 0 || forwardReserveMps > 3.0f)
    return false;
  float responseTime=0;
  for (unsigned i=0; i<3; ++i) {
    if (model->tau[i] < .01f || model->tau[i] > .2f ||
        model->attitudeGain[i] <= 0 || model->attitudeGain[i] > 20.0f ||
        model->integralGain[i] < 0 || model->integralGain[i] > 20.0f ||
        fabsf(reference[i]) > (i<2 ? 35.0f : 45.0f)) return false;
    responseTime=fmaxf(responseTime,fmaxf(model->tau[i],1.0f/model->attitudeGain[i]));
  }
  for (unsigned i=0; i<2; ++i) {
    if (!isfinite(model->accelerationBias[i]) || fabsf(model->accelerationBias[i]) > 1.0f)
      return false;
  }
  postReleaseLocalUnwindDecision_t result={.terminalNow=*state,.terminalAfterTick=*state};
  if (!postReleaseUnwindPlan(reference,referenceRate,referenceAccel,referenceJerk,
        unwindDurationS,&result.profile) || !postReleaseUnwindFeasible(&result.profile))
    return false;
  float accelerationNow[2],jerkNow[2];
  if (!acceleration(state,model,holdYawDeg,accelerationNow,jerkNow)) return false;
  result.accelerationAlong=accelerationNow[0]*direction[0]+accelerationNow[1]*direction[1];
  result.jerkAlong=jerkNow[0]*direction[0]+jerkNow[1]*direction[1];
  result.minForwardNow=result.minForwardAfterTick=along(state,direction);
  result.predictionHorizonS=unwindDurationS+fminf(.60f,3.0f*responseTime);
  float integral[3];
  memcpy(integral,model->integralRateBias,sizeof(integral));
  if (!predict(&result.profile,&result.terminalNow,integral,model,holdYawDeg,direction,
               result.predictionHorizonS,&result.minForwardNow)) return false;
  result.endForwardNow=along(&result.terminalNow,direction);

  // For a constant rapid command these derivatives are all zero. Preserving
  // them also makes this primitive safe to call at a C3-continuous boundary.
  const float t=nextControlTickS, mid=.5f*t;
  float midpoint[3],midRate[3],nextRef[3],nextRate[3],nextAccel[3];
  for (unsigned i=0; i<3; ++i) {
    midpoint[i]=reference[i]+referenceRate[i]*mid+.5f*referenceAccel[i]*mid*mid+
                referenceJerk[i]*mid*mid*mid/6.0f;
    midRate[i]=referenceRate[i]+referenceAccel[i]*mid+.5f*referenceJerk[i]*mid*mid;
    nextRef[i]=reference[i]+referenceRate[i]*t+.5f*referenceAccel[i]*t*t+
               referenceJerk[i]*t*t*t/6.0f;
    nextRate[i]=referenceRate[i]+referenceAccel[i]*t+.5f*referenceJerk[i]*t*t;
    nextAccel[i]=referenceAccel[i]+referenceJerk[i]*t;
  }
  memcpy(integral,model->integralRateBias,sizeof(integral));
  const float alpha[3]={1-expf(-t/model->tau[0]),1-expf(-t/model->tau[1]),
                       1-expf(-t/model->tau[2])};
  postReleaseUnwindProfile_t afterTick;
  if (!step(&result.terminalAfterTick,integral,model,midpoint,midRate,holdYawDeg,t,alpha) ||
      !postReleaseUnwindPlan(nextRef,nextRate,nextAccel,referenceJerk,unwindDurationS,&afterTick) ||
      !postReleaseUnwindFeasible(&afterTick)) return false;
  result.minForwardAfterTick=fminf(result.minForwardAfterTick,along(&result.terminalAfterTick,direction));
  if (!predict(&afterTick,&result.terminalAfterTick,integral,model,holdYawDeg,direction,
               result.predictionHorizonS,&result.minForwardAfterTick)) return false;
  result.endForwardAfterTick=along(&result.terminalAfterTick,direction);
  result.startUnwind=result.minForwardAfterTick <= forwardReserveMps ||
                     result.minForwardNow <= forwardReserveMps;
  result.integrationSteps=POST_RELEASE_LOCAL_UNWIND_MAX_STEPS;
  *decision=result;
  return true;
}
