#include "post_release_joint_unwind.h"
#include <math.h>
#include <string.h>

#define RAD 0.017453292519943295f
static bool finite3(const float x[3]) {
  return x && isfinite(x[0]) && isfinite(x[1]) && isfinite(x[2]);
}
static float clip(float x, float lo, float hi) {
  return fmaxf(lo, fminf(hi, x));
}
static bool construct(const float a[3], const float r[3], const float ac[3],
    const float j[3], const float knot[3], const float end[3], float duration, jointPlan_t* plan) {
  if (!isfinite(duration) || duration < .13f || duration > 1.6f ||
      !plan || !finite3(a) || !finite3(r) || !finite3(ac) || !finite3(j) ||
      !finite3(knot) || !finite3(end)) return false;
  const float zero[3] = {0};
  float offset[3], tail[3];
  for (int i=0;i<3;i++) {offset[i]=a[i]-knot[i]; tail[i]=knot[i]-end[i];}
  jointPlan_t p = {.duration=duration};
  // Each piece ends with zero rate/acceleration/jerk. The common knot is C3.
  if (!postReleaseUnwindPlan(offset,r,ac,j,duration*.40f,&p.piece[0]) ||
      !postReleaseUnwindPlan(tail,zero,zero,zero,duration*.60f,&p.piece[1])) return false;
  memcpy(p.knot,knot,sizeof(p.knot));
  memcpy(p.endReference,end,sizeof(p.endReference));
  *plan=p;
  return true;
}
bool jointBuildFromParameters(const float a[3], const float r[3],
    const float knot[3], const float end[3], float duration, jointPlan_t* plan) {
  const float zero[3]={0};
  return jointBuildFromBoundaryParameters(a,r,zero,zero,knot,end,duration,plan);
}
bool jointBuildFromBoundaryParameters(const float a[3], const float r[3],
    const float ac[3], const float j[3], const float knot[3],
    const float end[3], float duration, jointPlan_t* plan) {
  jointPlan_t candidate;
  if (!plan || !construct(a,r,ac,j,knot,end,duration,&candidate)) return false;
  // Fixed, modest work in the command task. This screen is geometric only;
  // the Pi predictor remains responsible for predicted terminal dynamics.
  for (int k=0;k<=32;k++) {
    float ref[3],rate[3],acc[3],jerk[3];
    if (!jointEvaluate(&candidate,duration*k/32.0f,ref,rate,acc,jerk)) return false;
    for (int i=0;i<3;i++) {
      if (!isfinite(ref[i]) || !isfinite(rate[i]) || !isfinite(acc[i]) ||
          !isfinite(jerk[i]) || fabsf(ref[i])>(i<2?25.0f:45.0f) ||
          fabsf(rate[i])>180.0f || fabsf(acc[i])>4000.0f ||
          fabsf(end[i])>3.0f) return false;
    }
  }
  *plan=candidate;
  return true;
}
bool jointEvaluate(const jointPlan_t* p, float t, float a[3], float r[3],
    float ac[3], float j[3]) {
  if (!p || !isfinite(t) || t<0 || !isfinite(p->duration) ||
      p->duration<.13f || p->duration>1.6f || !finite3(p->knot) || !a || !r || !ac || !j) return false;
  if(t>=p->duration) {
    for(int i=0;i<3;i++) {a[i]=p->endReference[i]; r[i]=ac[i]=j[i]=0;}
    return true;
  }
  const bool first=t<p->piece[0].durationS;
  if (!postReleaseUnwindEvaluate(&p->piece[first?0:1],
      first?t:t-p->piece[0].durationS,a,r,ac,j)) return false;
  for(int i=0;i<3;i++) a[i]+=first?p->knot[i]:p->endReference[i];
  return true;
}
bool jointPredict(const jointPlan_t* p, const jointState_t* s,
    const jointModel_t* m, float yawDeg, const float direction[2], jointPrediction_t* out) {
  if(!p||!s||!m||!out||!direction||!isfinite(yawDeg)||
     !isfinite(direction[0])||!isfinite(direction[1])||
     fabsf(hypotf(direction[0],direction[1])-1)> .01f ||
     !finite3(s->angle)||!finite3(s->rate)||!finite3(m->integralRateBias)||!finite3(m->integralGain)||
     !isfinite(s->velocity[0])||!isfinite(s->velocity[1])) return false;
  for(int i=0;i<3;i++) if(!isfinite(m->tau[i]) || m->tau[i]<.01f || m->tau[i]>.2f ||
    !isfinite(m->attitudeGain[i]) || m->attitudeGain[i]<0 || m->attitudeGain[i]>20) return false;
  for(int i=0;i<2;i++) if(!isfinite(m->accelerationBias[i])||fabsf(m->accelerationBias[i])>1) return false;
  jointPrediction_t v={.terminal=*s};
  memcpy(v.integralEnd,m->integralRateBias,sizeof(v.integralEnd));
  v.minForward=s->velocity[0]*direction[0]+s->velocity[1]*direction[1];
  // Fixed bounded work; dt <= 10 ms over the supported duration.
  const int steps=160;
  const float dt=p->duration/steps;
  const float rateAlpha[3]={1-expf(-dt/m->tau[0]),
    1-expf(-dt/m->tau[1]),1-expf(-dt/m->tau[2])};
  for(int k=0;k<steps;k++) {
    float a[3],r[3],ac[3],j[3];
    if(!jointEvaluate(p,(k+.5f)*dt,a,r,ac,j)) return false;
    for(int i=0;i<3;i++) {
      if(fabsf(a[i])>(i<2?25.0f:45.0f)||fabsf(r[i])>180||fabsf(ac[i])>4000) return false;
      // Approximate closed angular-rate response INCLUDING rate FF. This is
      // not rigid-body inertia identification; validity must be checked in SITL.
      const float error=a[i]-v.terminal.angle[i];
      v.integralEnd[i]+=m->integralGain[i]*error*dt;
      const float desired=r[i]+m->attitudeGain[i]*error+v.integralEnd[i];
      const float next=v.terminal.rate[i]+(desired-v.terminal.rate[i])*rateAlpha[i];
      v.terminal.angle[i]+=.5f*(v.terminal.rate[i]+next)*dt;
      v.terminal.rate[i]=next;
      if(i<2) v.maxTilt=fmaxf(v.maxTilt,fabsf(v.terminal.angle[i]));
      v.maxRate=fmaxf(v.maxRate,fabsf(next));
    }
    if(v.maxTilt>35 || v.maxRate>220) return false;
    const float phi=v.terminal.angle[0]*RAD, theta=-v.terminal.angle[1]*RAD;
    const float psi=(yawDeg+v.terminal.angle[2])*RAD;
    const float bx=9.81f*tanf(theta), by=-9.81f*tanf(phi)/cosf(theta);
    v.terminal.velocity[0]+=(cosf(psi)*bx-sinf(psi)*by+m->accelerationBias[0])*dt;
    v.terminal.velocity[1]+=(sinf(psi)*bx+cosf(psi)*by+m->accelerationBias[1])*dt;
    const float along=v.terminal.velocity[0]*direction[0]+v.terminal.velocity[1]*direction[1];
    v.minForward=fminf(v.minForward,along);
  }
  v.cost=hypotf(v.terminal.velocity[0],v.terminal.velocity[1]);
  if(!isfinite(v.cost)||!finite3(v.terminal.angle)||!finite3(v.terminal.rate)) return false;
  *out=v;
  return true;
}
bool jointSolve(const jointState_t* s, const jointModel_t* model,
    const float ref[3], const float rate[3], const float ac[3],const float jerk[3],
    float duration,float yawDeg,const float dir[2],jointPlan_t* out,jointPrediction_t* prediction) {
  if(!out||!prediction||!s||!finite3(ref)) return false;
  float knot[3]={.5f*ref[0],.5f*ref[1],0};
  if(!model||!finite3(model->integralRateBias)||!finite3(model->attitudeGain)) return false;
  float end[3];
  for(int i=0;i<3;i++) {
    if(model->attitudeGain[i]<=0) return false;
    // Terminal target is ACTUAL level, not zero reference despite a nonzero
    // attitude PID integral. Compensate existing I without clearing/tuning it.
    end[i]=-model->integralRateBias[i]/model->attitudeGain[i];
    if(fabsf(end[i])>3) return false;
  }
  jointPlan_t p;
  jointPrediction_t v;
  // Two-dimensional finite-difference shooting, never an unbounded optimizer.
  for(int iteration=0;iteration<4;iteration++) {
    if(!construct(ref,rate,ac,jerk,knot,end,duration,&p)||!jointPredict(&p,s,model,yawDeg,dir,&v)) return false;
    float error[2]={v.terminal.velocity[0]-.003f*dir[0],v.terminal.velocity[1]-.003f*dir[1]};
    if(hypotf(error[0],error[1])<.002f) break;
    float jac[2][2];
    for(int axis=0;axis<2;axis++) {
      float perturbed[3]={knot[0],knot[1],knot[2]};
      perturbed[axis]+=.2f;
      jointPlan_t q; jointPrediction_t w;
      if(!construct(ref,rate,ac,jerk,perturbed,end,duration,&q)||!jointPredict(&q,s,model,yawDeg,dir,&w)) return false;
      for(int i=0;i<2;i++) jac[i][axis]=(w.terminal.velocity[i]-v.terminal.velocity[i])/.2f;
    }
    const float det=jac[0][0]*jac[1][1]-jac[0][1]*jac[1][0];
    if(!isfinite(det)||fabsf(det)<1e-7f) return false;
    knot[0]-=clip((jac[1][1]*error[0]-jac[0][1]*error[1])/det,-12,12);
    knot[1]-=clip((-jac[1][0]*error[0]+jac[0][0]*error[1])/det,-12,12);
    // Update only after this Jacobian was evaluated at one common endpoint.
    for(int i=0;i<3;i++) {
      end[i]=-v.integralEnd[i]/model->attitudeGain[i];
      if(fabsf(end[i])>3) return false;
    }
  }
  if(!construct(ref,rate,ac,jerk,knot,end,duration,&p)||!jointPredict(&p,s,model,yawDeg,dir,&v)) return false;
  if(v.cost>.012f || v.minForward<-.025f) return false;
  for(int i=0;i<3;i++) if(fabsf(v.terminal.angle[i])>2 || fabsf(v.terminal.rate[i])>4) return false;
  *out=p; *prediction=v;
  return true;
}
