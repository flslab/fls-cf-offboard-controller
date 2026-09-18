#include "post_release_joint_unwind.h"
#include <math.h>
#include <string.h>

// Input order is the 26-float wire snapshot, not a native packed C struct.
// Predict the *actual* response while FC keeps the constant rapid reference.
// This bounded prefix and jointPredict use the same CF roll/-pitch convention.
int piJointSolve(const float x[26], float delay, float out[17]) {
  if (!x || !out || !isfinite(delay) || delay < .02f || delay > .5f) return 0;
  for (int i=0;i<26;i++) if(!isfinite(x[i])) return 0;
  jointState_t s;
  memcpy(s.velocity,x,2*sizeof(float));
  memcpy(s.angle,x+2,3*sizeof(float));
  memcpy(s.rate,x+5,3*sizeof(float));
  const float *dir=x+8, yaw=x[10], *ref=x+23;
  jointModel_t m={0};
  memcpy(m.integralRateBias,x+11,3*sizeof(float));
  memcpy(m.attitudeGain,x+14,3*sizeof(float));
  memcpy(m.integralGain,x+17,3*sizeof(float));
  memcpy(m.tau,x+20,3*sizeof(float));
  if(fabsf(hypotf(dir[0],dir[1])-1)>.01f) return 0;
  for(int i=0;i<3;i++) if(m.tau[i]<.01f || m.tau[i]>.2f ||
    m.attitudeGain[i]<=0 || m.attitudeGain[i]>20 ||
    fabsf(m.integralGain[i])>20 || fabsf(ref[i])>(i<2?25:45)) return 0;
  const int steps=(int)ceilf(delay/.001f);
  const float dt=delay/steps, rad=.017453292519943295f;
  float alpha[3];
  for(int i=0;i<3;i++) alpha[i]=1-expf(-dt/m.tau[i]);
  for(int k=0;k<steps;k++) {
    for(int i=0;i<3;i++) {
      const float error=ref[i]-s.angle[i];
      m.integralRateBias[i]+=m.integralGain[i]*error*dt;
      const float desired=m.attitudeGain[i]*error+m.integralRateBias[i];
      const float next=s.rate[i]+(desired-s.rate[i])*alpha[i];
      s.angle[i]+=.5f*(s.rate[i]+next)*dt;
      s.rate[i]=next;
      if(fabsf(s.angle[i])>(i<2?35:60) || fabsf(next)>220) return 0;
    }
    const float phi=s.angle[0]*rad, theta=-s.angle[1]*rad;
    const float psi=(yaw+s.angle[2])*rad;
    const float bx=9.81f*tanf(theta), by=-9.81f*tanf(phi)/cosf(theta);
    s.velocity[0]+=(cosf(psi)*bx-sinf(psi)*by)*dt;
    s.velocity[1]+=(sinf(psi)*bx+cosf(psi)*by)*dt;
    if(s.velocity[0]*dir[0]+s.velocity[1]*dir[1]<-.025f) return 0;
  }
  const float zero[3]={0}, horizons[]={.45f,.60f,.80f,1.f,1.2f,1.4f,1.6f};
  jointPlan_t p;
  jointPrediction_t prediction;
  for(unsigned i=0;i<sizeof(horizons)/sizeof(horizons[0]);i++) {
    // Preserve issued reference; do not replace it with predicted actual pose.
    if(!jointSolve(&s,&m,ref,zero,zero,zero,horizons[i],yaw,dir,&p,&prediction)) continue;
    out[0]=p.duration;
    memcpy(out+1,ref,3*sizeof(float));
    memcpy(out+4,zero,3*sizeof(float));
    memcpy(out+7,p.knot,3*sizeof(float));
    memcpy(out+10,p.endReference,3*sizeof(float));
    out[13]=prediction.cost;
    out[14]=prediction.minForward;
    out[15]=hypotf(s.velocity[0],s.velocity[1]);
    out[16]=horizons[i];
    return 1;
  }
  return 0;
}
