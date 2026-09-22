#include "post_release_joint_unwind.h"
#include <math.h>
#include <string.h>

// Input is the expanded 28-float numerical snapshot, not a native C struct.
// V3 transport restores cached model fields before invoking this kernel.
// Predict actual response to the FC's immutable rapid OR local unwind prefix.
// This bounded prefix and jointPredict use the same CF roll/-pitch convention.
int piJointSolve(const float x[28], float delay, float out[17]) {
  if (!x || !out || !isfinite(delay) || delay < .02f || delay > .5f) return 0;
  for (int i=0;i<28;i++) if(!isfinite(x[i])) return 0;
  jointState_t s;
  memcpy(s.velocity,x,2*sizeof(float));
  memcpy(s.angle,x+2,3*sizeof(float));
  memcpy(s.rate,x+5,3*sizeof(float));
  const float *dir=x+8, yaw=x[10], *initialRef=x+23;
  const float zero[3]={0}, elapsed=x[26], prefixDuration=x[27];
  postReleaseUnwindProfile_t prefix;
  if (elapsed<0 || prefixDuration<0 || prefixDuration>1.6f ||
      (prefixDuration==0 && elapsed!=0) ||
      (prefixDuration>0 && (elapsed>prefixDuration+.5f ||
        !postReleaseUnwindPlan(initialRef,zero,zero,zero,prefixDuration,&prefix)))) return 0;
  float ref[3], refRate[3]={0}, refAccel[3]={0}, refJerk[3]={0};
  memcpy(ref,initialRef,sizeof(ref));
  jointModel_t m={0};
  memcpy(m.integralRateBias,x+11,3*sizeof(float));
  memcpy(m.attitudeGain,x+14,3*sizeof(float));
  memcpy(m.integralGain,x+17,3*sizeof(float));
  memcpy(m.tau,x+20,3*sizeof(float));
  if(fabsf(hypotf(dir[0],dir[1])-1)>.01f) return 0;
  for(int i=0;i<3;i++) if(m.tau[i]<.01f || m.tau[i]>.2f ||
    m.attitudeGain[i]<=0 || m.attitudeGain[i]>20 ||
    fabsf(m.integralGain[i])>20 || fabsf(initialRef[i])>(i<2?25:45)) return 0;
  const int steps=(int)ceilf(delay/.001f);
  const float dt=delay/steps, rad=.017453292519943295f;
  float alpha[3];
  for(int i=0;i<3;i++) alpha[i]=1-expf(-dt/m.tau[i]);
  for(int k=0;k<steps;k++) {
    if(prefixDuration>0 && !postReleaseUnwindEvaluate(&prefix,
        elapsed+(k+.5f)*dt,ref,refRate,refAccel,refJerk)) return 0;
    for(int i=0;i<3;i++) {
      const float error=ref[i]-s.angle[i];
      m.integralRateBias[i]+=m.integralGain[i]*error*dt;
      const float desired=refRate[i]+m.attitudeGain[i]*error+m.integralRateBias[i];
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
  if(prefixDuration>0 && !postReleaseUnwindEvaluate(&prefix,
      elapsed+delay,ref,refRate,refAccel,refJerk)) return 0;
  const float horizons[]={.45f,.60f,.80f,1.f,1.2f,1.4f,1.6f};
  jointPlan_t p;
  jointPrediction_t prediction;
  for(unsigned i=0;i<sizeof(horizons)/sizeof(horizons[0]);i++) {
    // Preserve issued reference; do not replace it with predicted actual pose.
    if(!jointSolve(&s,&m,ref,refRate,refAccel,refJerk,horizons[i],yaw,dir,&p,&prediction)) continue;
    out[0]=p.duration;
    memcpy(out+1,ref,3*sizeof(float));
    memcpy(out+4,refRate,3*sizeof(float));
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
