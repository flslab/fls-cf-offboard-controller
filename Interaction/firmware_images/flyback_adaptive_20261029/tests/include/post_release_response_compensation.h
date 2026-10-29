#pragma once
// Fixed-cost response-aware attitude tracking. No optimization, allocation,
// model fitting or worker. Units: attitude deg, rate deg/s, velocity m/s.
#include "post_release_response.h"
#define PR_COMP_G_DEG .1712168f
typedef struct {float v,angle,rate;} prCompState;

static inline prCompState prCompDerivative(prCompState s,float u,const prResponseAxis *m) {
  return (prCompState){-PR_COMP_G_DEG*s.angle,s.rate,
    m->wn*m->wn*(m->gain*u+m->bias-s.angle)-2*m->zeta*m->wn*s.rate};
}
static inline prCompState prCompStep(prCompState s,float u,const prResponseAxis *m,float dt) {
  // Explicit midpoint, at most 2 ms. Even the allowed fastest calibration
  // stays within the bounded integration range; no sensor time is modified.
  const prCompState d=prCompDerivative(s,u,m);
  const prCompState mid={s.v+.5f*dt*d.v,s.angle+.5f*dt*d.angle,s.rate+.5f*dt*d.rate};
  const prCompState e=prCompDerivative(mid,u,m);
  return (prCompState){s.v+dt*e.v,s.angle+dt*e.angle,s.rate+dt*e.rate};
}
static inline prCompState prCompPredict(prCompState s,const prResponseAxis *m,const float history[76]) {
  // Only already-issued commands act before the new command's delay elapses.
  // History is a causal ZOH, oldest first, with 2 ms cells ending at now.
  float t=0;
  for(unsigned k=0;k<75 && t<m->delay;k++) {
    const float dt=fminf(.002f,m->delay-t);
    int index=(int)floorf(75+(t-m->delay)/.002f);
    if(index<0)index=0;
    if(index>75)index=75;
    s=prCompStep(s,history[index],m,dt);t+=dt;
  }
  return s;
}
static inline prCompState prCompPredictPosition(prCompState s,float *position,
    const prResponseAxis *m,const float history[76]) {
  // Same delayed command cells/midpoint integration as the stable predictor.
  // Position is relative to curve activation, in the current heading frame;
  // it is propagated to the SAME activation time as v/angle/rate/reference.
  float t=0;
  for(unsigned k=0;k<75 && t<m->delay;k++) {
    const float dt=fminf(.002f,m->delay-t);
    int index=(int)floorf(75+(t-m->delay)/.002f);
    if(index<0)index=0;
    if(index>75)index=75;
    const prCompState derivative=prCompDerivative(s,history[index],m);
    *position+=dt*(s.v+.5f*dt*derivative.v);
    s=prCompStep(s,history[index],m,dt);t+=dt;
  }
  return s;
}
static inline float prCompStopVelocity(prCompState s,const prResponseAxis *m) {
  // Integral of the zero-acceleration-input attitude tail, after delay.
  // It includes angular rate; it is not a speed-only stopping-distance rule.
  return s.v-PR_COMP_G_DEG*(s.rate+2*m->zeta*m->wn*s.angle)/(m->wn*m->wn);
}
static inline float prCompAcceleration(prCompState s,const prResponseAxis *m,
    const float ref[4],float pole) {
  // v'=a, a'=j, j'=wn^2*(gain*u+bias-a)-2*zeta*wn*j.
  // Place the three tracking-error poles at -pole. ref = v,a,j,snap at
  // command activation, NOT at receipt. a/j are measured from tilt/rate.
  const float a=-PR_COMP_G_DEG*s.angle,j=-PR_COMP_G_DEG*s.rate;
  const float w2=m->wn*m->wn,d=2*m->zeta*m->wn;
  return (ref[3]+pole*pole*pole*(ref[0]-s.v)+
    3*pole*pole*(ref[1]-a)+3*pole*(ref[2]-j)+w2*a+d*j+
    w2*PR_COMP_G_DEG*m->bias)/(w2*m->gain);
}
static inline float prCompAccelerationPosition(prCompState s,float position,
    const prResponseAxis *m,const float ref[4],float positionRef,float pole,float positionPole) {
  // Characteristic polynomial (s+positionPole)*(s+pole)^3. PositionPole=0
  // delegates bit-for-bit to the old velocity/acceleration/jerk controller.
  // Input ref is v,a,j,snap; no numerical differentiation of measured data.
  if(positionPole<=0)return prCompAcceleration(s,m,ref,pole);
  const float a=-PR_COMP_G_DEG*s.angle,j=-PR_COMP_G_DEG*s.rate;
  const float w2=m->wn*m->wn,d=2*m->zeta*m->wn;
  const float l2=pole*pole,l3=l2*pole;
  return (ref[3]+positionPole*l3*(positionRef-position)+
    (l3+3*positionPole*l2)*(ref[0]-s.v)+
    (3*l2+3*positionPole*pole)*(ref[1]-a)+
    (3*pole+positionPole)*(ref[2]-j)+w2*a+d*j+
    w2*PR_COMP_G_DEG*m->bias)/(w2*m->gain);
}
