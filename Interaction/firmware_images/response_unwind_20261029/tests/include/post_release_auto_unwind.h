#pragma once
#include "post_release_response_compensation.h"
#include "post_release_adaptive_scurve.h"
#include <string.h>

// Body-yaw X/pitch, Y/roll; degrees, seconds, m/s. The C3 polynomial is an
// ISSUED ATTITUDE COMMAND, not an assumed perfectly tracked physical angle.
// Terminal velocity includes the calibrated response AFTER that command ends.
#define PR_AUTO_API 2u
#define PR_AUTO_MIN_T .15f
#define PR_AUTO_MAX_T 1.2f
#define PR_AUTO_MODELS 4u

typedef struct {
  float maxRate,maxAcceleration,maxJerk,slowScale,extraDelay,settleGain,reserve;
  float maxBodyRate,maxBodyAcceleration,settleCorrection;
} prAutoLimits;
static inline prAutoLimits prAutoDefaultLimits(void) {
  return (prAutoLimits){160,4000,80000,.65f,.02f,1.5f,.08f,120,1000,6};
}
static inline bool prAutoLimitsValid(const prAutoLimits *l) {
  return l && isfinite(l->maxRate) && l->maxRate>=20 && l->maxRate<=160 &&
    isfinite(l->maxAcceleration) && l->maxAcceleration>=100 && l->maxAcceleration<=8000 &&
    isfinite(l->maxJerk) && l->maxJerk>=1000 && l->maxJerk<=200000 &&
    isfinite(l->slowScale) && l->slowScale>=.55f && l->slowScale<=1 &&
    isfinite(l->extraDelay) && l->extraDelay>=0 && l->extraDelay<=.04f &&
    isfinite(l->settleGain) && l->settleGain>=1 && l->settleGain<=4 &&
    isfinite(l->reserve) && l->reserve>=.03f && l->reserve<=.20f &&
    isfinite(l->maxBodyRate) && l->maxBodyRate>=20 && l->maxBodyRate<=180 &&
    isfinite(l->maxBodyAcceleration) && l->maxBodyAcceleration>=100 && l->maxBodyAcceleration<=1600 &&
    isfinite(l->settleCorrection) && l->settleCorrection>=1 && l->settleCorrection<=12;
}
typedef struct {
  scaPoly curve[2],safeCurve[2];
  float command[2],commandRate[2],commandAcceleration[2],commandJerk[2];
  float begin,last,chosenT,predictedEnd,direction[2],target,limit,maxTime,period;
  float unwindBegin,unwindStarted,totalT,stableSince,lastReplan,equilibrium[2],safeAge;
  float worstStop,minimumPredictedVelocity,shortestMinimum;
  prAutoLimits limits;
  unsigned replans,saturations;
  uint8_t phase,recoveryReason;
  bool stable,safeValid,initialChecked,initialFeasible;
} prAutoUnwind;
static inline float prAutoPastCommand(const float h[76],float delay) {
  int index=(int)floorf(75.f-delay/.002f);
  return h[index<0?0:(index>75?75:index)];
}

static inline float prAutoIntegralTo(const scaPoly *p,float t) {
  const float x=scaClip(t/p->T,0,1);float value=p->c[7]/8;
  for(int k=6;k>=0;k--)value=value*x+p->c[k]/(k+1);
  return p->T*x*value;
}
static inline bool prAutoInit(prAutoUnwind *u,float now,const float velocity[2],
    const float command[2],float target,float tiltLimit,float maxTime) {
  if(!u || !isfinite(now) || !isfinite(target) || target<0 || target>.3f ||
     !isfinite(tiltLimit) || tiltLimit<1 || tiltLimit>40 ||
     !isfinite(maxTime) || maxTime<.5f || maxTime>12)return false;
  for(unsigned k=0;k<2;k++)if(!isfinite(velocity[k]) || !isfinite(command[k]))return false;
  if(hypotf(command[0],command[1])>tiltLimit)return false;
  const float speed=hypotf(velocity[0],velocity[1]);
  *u=(prAutoUnwind){.begin=now,.last=now,.phase=1,.target=target,.limit=tiltLimit,
    .maxTime=maxTime,.limits=prAutoDefaultLimits(),.lastReplan=-1};
  u->direction[0]=speed>.001f?velocity[0]/speed:1;
  u->direction[1]=speed>.001f?velocity[1]/speed:0;
  memcpy(u->command,command,sizeof(u->command));return true;
}
static inline bool prAutoConfigure(prAutoUnwind *u,const prAutoLimits *l) {
  if(!u || !prAutoLimitsValid(l))return false;
  u->limits=*l;return true;
}

// Exact stop coordinate for the calibrated second-order response:
// S=v-g*(rate+2*zeta*wn*angle)/wn^2, S'=-g*(gain*command+bias).
// A neutral command ends the input pulse, not the physical response.
static inline prResponseAxis prAutoHypothesis(const prAutoUnwind *u,
    const prResponseAxis *m,unsigned hypothesis) {
  prResponseAxis result=*m;
  if(hypothesis) {
    result.delay=hypothesis==3?fmaxf(0,result.delay-u->limits.extraDelay):fminf(.149f,result.delay+u->limits.extraDelay);
    result.wn*=u->limits.slowScale;
    if(hypothesis==1)result.zeta=fmaxf(.21f,result.zeta*.80f);
    if(hypothesis==2)result.zeta=fminf(1.99f,result.zeta*1.15f);
    if(hypothesis==3){result.wn=m->wn*1.10f;result.zeta=fmaxf(.21f,result.zeta*.80f);}
    result.gain*=1.15f;result.bias*=1.15f; // preserve the same neutral trim
  }
  return result;
}
// Exact held-input second-order transition. Velocity is advanced through the
// same stop-coordinate identity, so no tilt-force impulse is lost numerically.
static inline void prAutoTransition(const prResponseAxis *m,float dt,float a[4]) {
  const float d=m->zeta*m->wn,w2=m->wn*m->wn,disc=d*d-w2;
  float c,b;
  if(fabsf(disc)<1e-5f){c=1;b=dt;}
  else if(disc>0){const float q=sqrtf(disc);c=coshf(q*dt);b=sinhf(q*dt)/q;}
  else {const float q=sqrtf(-disc);c=cosf(q*dt);b=sinf(q*dt)/q;}
  const float e=expf(-d*dt);
  a[0]=e*(c+d*b);a[1]=e*b;a[2]=-w2*a[1];a[3]=e*(c-d*b);
}
static inline float prAutoMinimumChecked(const prAutoUnwind *u,const prCompState initial[2],
    const prResponseAxis model[2],const float history[2][76],const scaPoly plan[2],bool physicalBounds) {
  float worst=INFINITY;
  for(unsigned h=0;h<PR_AUTO_MODELS;h++) {
    prResponseAxis m[2];prCompState s[2]={initial[0],initial[1]};
    float duration=0,pendingRate[2]={0},pendingAcceleration[2]={0};
    for(unsigned k=0;k<2;k++) {
      m[k]=prAutoHypothesis(u,&model[k],h);
      duration=fmaxf(duration,m[k].delay+plan[k].T+fminf(3.f,4.f/(m[k].zeta*m[k].wn)));
      prCompState pending=initial[k];
      for(float t=0;t<m[k].delay;t+=.002f) {
        const float command=prAutoPastCommand(history[k],m[k].delay-t);
        pendingRate[k]=fmaxf(pendingRate[k],fabsf(pending.rate));
        pendingAcceleration[k]=fmaxf(pendingAcceleration[k],fabsf(prCompDerivative(pending,command,&m[k]).rate));
        pending=prCompStep(pending,command,&m[k],fminf(.002f,m[k].delay-t));
      }
      // The first plan value is the already-held PID target, preserved by
      // the C3 splice. Include its activation boundary in the unavoidable
      // queued transient; a new plan cannot change that initial acceleration.
      pendingRate[k]=fmaxf(pendingRate[k],fabsf(pending.rate));
      pendingAcceleration[k]=fmaxf(pendingAcceleration[k],fabsf(prCompDerivative(pending,plan[k].c[0],&m[k]).rate));
    }
    const float allowedRate=fmaxf(.95f*u->limits.maxBodyRate,hypotf(pendingRate[0],pendingRate[1]));
    const float allowedAcceleration=fmaxf(.95f*u->limits.maxBodyAcceleration,hypotf(pendingAcceleration[0],pendingAcceleration[1]));
    const float period=u->period>0?u->period:.01f;
    // Split at actual command-activation/queue boundaries. A midpoint-only
    // rollout misses acceleration jumps from the zero-order-held setpoint.
    float cachedDt[12],cachedA[12][2][4];unsigned cached=0,replacement=0;
    float t=0;
    for(unsigned step=0;t<duration && step<10000;step++) {
      float next=fminf(duration,t+.01f),command[2],acceleration[2];
      for(unsigned k=0;k<2;k++) {
        const float age=t-m[k].delay,spacing=age<-.000001f?.002f:period;
        float edge=m[k].delay+(floorf((age+.000001f)/spacing)+1)*spacing;
        if(edge<=t+.000001f)edge+=spacing;
        next=fminf(next,edge);
        const float epoch=floorf(fmaxf(0,age+.000001f)/period)*period;
        command[k]=age<-.000001f?prAutoPastCommand(history[k],-age):scaEval(&plan[k],epoch).x;
        acceleration[k]=prCompDerivative(s[k],command[k],&m[k]).rate;
      }
      if(fabsf(s[0].angle)>80 || fabsf(s[1].angle)>80)return -INFINITY;
      if(physicalBounds && (hypotf(s[0].rate,s[1].rate)>allowedRate+.01f || hypotf(acceleration[0],acceleration[1])>allowedAcceleration+.1f))return -INFINITY;
      worst=fminf(worst,u->direction[0]*s[0].v+u->direction[1]*s[1].v);
      const float dt=next-t;if(dt<=0)return -INFINITY;
      unsigned ci=0;
      while(ci<cached && fabsf(cachedDt[ci]-dt)>.0000005f)ci++;
      if(ci==cached) {
        if(cached<12)cached++;else {ci=replacement;replacement=(replacement+1)%12;}
        cachedDt[ci]=dt;for(unsigned k=0;k<2;k++)prAutoTransition(&m[k],dt,cachedA[ci][k]);
      }
      const float oldAngle[2]={s[0].angle,s[1].angle};
      for(unsigned k=0;k<2;k++) {
        const float target=m[k].gain*command[k]+m[k].bias,error=s[k].angle-target;
        const float *a=cachedA[ci][k];
        const float angle=target+a[0]*error+a[1]*s[k].rate,rate=a[2]*error+a[3]*s[k].rate;
        s[k].angle=angle;s[k].rate=rate;
      }
      const float radians=.0174532925199433f;
      s[0].v-=.5f*dt*9.81f*(tanf(oldAngle[0]*radians)+tanf(s[0].angle*radians));
      s[1].v-=.5f*dt*9.81f*(tanf(oldAngle[1]*radians)/cosf(oldAngle[0]*radians)+tanf(s[1].angle*radians)/cosf(s[0].angle*radians));
      t=next;
    }
    if(t<duration-.00001f)return -INFINITY;
    worst=fminf(worst,u->direction[0]*s[0].v+u->direction[1]*s[1].v);
  }
  return worst;
}
static inline float prAutoMinimum(const prAutoUnwind *u,const prCompState initial[2],
    const prResponseAxis model[2],const float history[2][76],const scaPoly plan[2]) {
  return prAutoMinimumChecked(u,initial,model,history,plan,true);
}
static inline float prAutoTerminal(const prAutoUnwind *u,const prCompState state[2],
    const prResponseAxis model[2],const float history[2][76],const scaPoly plan[2],float age) {
  float worst=INFINITY;
  for(unsigned h=0;h<PR_AUTO_MODELS;h++) {
    float result=0;
    for(unsigned k=0;k<2;k++) {
      const prResponseAxis m=prAutoHypothesis(u,&model[k],h);
      const prCompState delayed=prCompPredict(state[k],&m,history[k]);
      const float neutral=-m.bias/m.gain;
      const float impulse=prAutoIntegralTo(&plan[k],plan[k].T)-prAutoIntegralTo(&plan[k],age)-
        neutral*fmaxf(0,plan[k].T-age)+.5f*u->period*(scaEval(&plan[k],age).x-neutral);
      result+=u->direction[k]*(prCompStopVelocity(delayed,&m)-PR_COMP_G_DEG*m.gain*impulse);
    }
    worst=fminf(worst,result);
  }
  return worst;
}
static inline float prAutoWorstStop(const prAutoUnwind *u,const prCompState state[2],
    const prResponseAxis model[2],const float history[2][76]) {
  float worst=INFINITY;
  for(unsigned h=0;h<PR_AUTO_MODELS;h++) {
    float sum=0;
    for(unsigned k=0;k<2;k++) {
      const prResponseAxis m=prAutoHypothesis(u,&model[k],h);
      sum+=u->direction[k]*prCompStopVelocity(prCompPredict(state[k],&m,history[k]),&m);
    }
    worst=fminf(worst,sum);
  }
  return worst;
}
static inline bool prAutoPlan(const prAutoUnwind *u,const float neutral[2],float T,scaPoly plan[2]) {
  if(!isfinite(T) || T<.06f || T>PR_AUTO_MAX_T)return false;
  for(unsigned k=0;k<2;k++)scaMake((scaRef){.x=u->command[k],.w=u->commandRate[k],
    .a=u->commandAcceleration[k],.j=u->commandJerk[k]},neutral[k],T,0,&plan[k]);
  for(unsigned i=0;i<=32;i++) {
    const scaRef a=scaEval(&plan[0],T*i/32),b=scaEval(&plan[1],T*i/32);
    if(!isfinite(a.x) || !isfinite(b.x) || hypotf(a.x,b.x)>u->limit+1e-3f ||
       hypotf(a.w,b.w)>.95f*u->limits.maxRate+1e-3f ||
       hypotf(a.a,b.a)>.95f*u->limits.maxAcceleration+1e-2f ||
       hypotf(a.j,b.j)>.95f*u->limits.maxJerk+.1f)return false;
  }
  return true;
}
static inline float prAutoNeutralBudget(const prAutoUnwind *u,const prCompState state[2],
    const prResponseAxis model[2],const float history[2][76]) {
  const float neutral[2]={-model[0].bias/model[0].gain,-model[1].bias/model[1].gain};
  scaPoly p[2];
  for(unsigned i=0;i<=75;i++) {
    const float T=.08f+.015f*i;
    if(T>PR_AUTO_MAX_T)break;
    if(prAutoPlan(u,neutral,T,p))return fminf(prAutoTerminal(u,state,model,history,p,0),prAutoMinimumChecked(u,state,model,history,p,false));
  }
  // No instantaneous neutral command is invented when the issued derivative
  // state cannot yet make a constrained splice. Continue the last safe curve.
  if(u->safeValid)return prAutoTerminal(u,state,model,history,u->safeCurve,u->safeAge);
  return prAutoWorstStop(u,state,model,history);
}

// Only the bounded command proposal, never raw high-gain inverse feedback,
// reaches the actuator. The acceleration and jerk limits apply in every phase.
static inline bool prAutoBound(prAutoUnwind *u,const float desired[2],float dt) {
  if(dt<=0)return true;
  float goal[2]={desired[0],desired[1]},n=hypotf(goal[0],goal[1]);
  if(n>u->limit)for(unsigned k=0;k<2;k++)goal[k]*=u->limit/n;
  // Receding C3 transition, rather than a clipped acceleration integrator
  // whose hidden windup could oscillate around a rapidly changing target.
  scaPoly p[2];float T=.12f;bool found=false;
  for(unsigned i=0;i<18;i++,T=fminf(PR_AUTO_MAX_T,T*1.15f)) {
    if(prAutoPlan(u,goal,T,p)){found=true;break;}
  }
  if(found){memcpy(u->safeCurve,p,sizeof(p));u->safeAge=dt;u->safeValid=true;}
  else if(u->safeValid){memcpy(p,u->safeCurve,sizeof(p));u->safeAge+=dt;u->saturations++;}
  else return false;
  for(unsigned k=0;k<2;k++) {
    const scaRef ref=scaEval(&p[k],u->safeAge);
    u->command[k]=ref.x;u->commandRate[k]=ref.w;
    u->commandAcceleration[k]=ref.a;u->commandJerk[k]=ref.j;
  }
  return true;
}
static inline bool prAutoTryReturn(prAutoUnwind *u,float now,const prCompState state[2],
    const prResponseAxis model[2],const float history[2][76],float minimum,float maximum,unsigned mode) {
  u->minimumPredictedVelocity=u->shortestMinimum=INFINITY;
  const float neutral[2]={-model[0].bias/model[0].gain,-model[1].bias/model[1].gain};
  scaPoly candidate[2];float low=0,minimumVelocity=-INFINITY;
  for(unsigned i=0;i<=70;i++) {
    const float T=minimum+.015f*i;
    if(T>maximum+1e-5f)break;
    if(!prAutoPlan(u,neutral,T,candidate))continue;
    const float predicted=prAutoMinimum(u,state,model,history,candidate);
    // A command-bounded short return may still demand too much physical
    // angular acceleration. Search longer returns before declaring failure.
    if(isfinite(predicted)){low=T;minimumVelocity=predicted;break;}
  }
  if(!low) {
    u->minimumPredictedVelocity=u->shortestMinimum=-INFINITY;
    return false;
  }
  u->minimumPredictedVelocity=u->shortestMinimum=minimumVelocity;
  if(mode==2)return true;
  if(!mode && minimumVelocity>u->target+u->limits.reserve+.04f)return false;
  // Prefer the longest dynamically feasible return within the available
  // stop-velocity budget. Never increase an already accepted deadline.
  float high=maximum;
  scaPoly longest[2];
  if(minimumVelocity>u->target+u->limits.reserve && prAutoPlan(u,neutral,high,longest)) {
    if(prAutoMinimum(u,state,model,history,longest)>=u->target+u->limits.reserve) {
      memcpy(candidate,longest,sizeof(longest));
    } else {
      for(unsigned i=0;i<8;i++) {
        const float T=.5f*(low+high);scaPoly trial[2];
        if(!prAutoPlan(u,neutral,T,trial)){high=T;continue;}
        if(prAutoMinimum(u,state,model,history,trial)>u->target+u->limits.reserve)low=T;else high=T;
      }
      if(!prAutoPlan(u,neutral,low,candidate))return false;
    }
  }
  u->minimumPredictedVelocity=prAutoMinimum(u,state,model,history,candidate);
  if(u->minimumPredictedVelocity<u->target-.005f)return false;
  memcpy(u->curve,candidate,sizeof(u->curve));memcpy(u->equilibrium,neutral,sizeof(u->equilibrium));
  if(u->phase!=2)u->unwindStarted=now;
  u->chosenT=candidate[0].T;u->unwindBegin=now;u->lastReplan=now;
  u->totalT=now-u->unwindStarted+u->chosenT;
  u->predictedEnd=prAutoTerminal(u,state,model,history,candidate,0);
  u->phase=2;return true;
}
static inline bool prAutoStep(prAutoUnwind *u,float now,const prCompState state[2],
    const prResponseAxis calibration[2],const float history[2][76],float decel,
    bool fresh,const float physicalAngle[2],float output[2]) {
  if(!u || !output || !physicalAngle || !prAutoLimitsValid(&u->limits))return false;
  const float dt=now-u->last;
  if(!isfinite(now) || dt<0 || dt>.03f || now-u->begin>u->maxTime ||
     !isfinite(decel) || decel<=0 || decel>8.23f)return false;
  if(dt>0)u->period=dt;
  for(unsigned k=0;k<2;k++) {
    const prResponseAxis *m=&calibration[k];
    if(!isfinite(state[k].v) || !isfinite(state[k].angle) || !isfinite(state[k].rate) ||
       !isfinite(physicalAngle[k]) || !isfinite(m->delay) || m->delay<0 || m->delay>=.15f ||
       !isfinite(m->wn) || m->wn<=5 || m->wn>=60 || !isfinite(m->zeta) || m->zeta<=.2f || m->zeta>=2 ||
       !isfinite(m->gain) || m->gain<=.7f || m->gain>=1.3f || !isfinite(m->bias) || fabsf(m->bias)>8)return false;
    for(unsigned i=0;i<76;i++)if(!isfinite(history[k][i]))return false;
  }
  prResponseAxis model[2];prCompState delayed[2];float along=0;
  for(unsigned k=0;k<2;k++) {
    model[k]=calibration[k];
    delayed[k]=prCompPredict(state[k],&model[k],history[k]);
    along+=u->direction[k]*state[k].v;
  }
  u->worstStop=prAutoWorstStop(u,state,model,history);
  if(!u->initialChecked) {
    prAutoUnwind probe=*u;
    const bool found=prAutoTryReturn(&probe,now,state,model,history,PR_AUTO_MIN_T,PR_AUTO_MAX_T,2);
    u->initialChecked=true;u->initialFeasible=found && probe.shortestMinimum>=u->target-.005f;
    if(!u->initialFeasible) {
      u->minimumPredictedVelocity=probe.shortestMinimum;
      u->recoveryReason=6;return false;
    }
  }
  bool advanced=false;
  if(u->phase==1) {
    float desired[2];
    for(unsigned k=0;k<2;k++)desired[k]=((decel*u->direction[k]+2*(state[k].v-u->direction[k]*along))/PR_COMP_G_DEG-model[k].bias)/model[k].gain;
    if(!prAutoBound(u,desired,dt))return false;
    advanced=true;
    if(along<=u->target+.02f){u->phase=3;u->recoveryReason=1;}
    else if(!prAutoTryReturn(u,now,state,model,history,PR_AUTO_MIN_T,PR_AUTO_MAX_T,0)) {
      if(u->worstStop<=u->target+.03f || u->minimumPredictedVelocity<u->target-.005f){u->phase=3;u->recoveryReason=2;}
      else if(isfinite(u->shortestMinimum) && dt>0) {
        // Do not cross the command-derivative viability boundary merely
        // because today's neutral return is still barely feasible. Probe one
        // more brake update before issuing it. With monotonically increasing
        // brake input, applying that proposal immediately is conservative.
        prAutoUnwind next=*u;
        const bool shaped=prAutoBound(&next,desired,dt);
        const bool possible=shaped && prAutoTryReturn(&next,now+dt,state,model,history,PR_AUTO_MIN_T,PR_AUTO_MAX_T,2);
        if(!possible || next.shortestMinimum<=u->target+u->limits.reserve+.04f)
          (void)prAutoTryReturn(u,now,state,model,history,PR_AUTO_MIN_T,PR_AUTO_MAX_T,1);
      }
    }
  }
  if(u->phase==2) {
    const float age=now-u->unwindBegin;
    if(!advanced)for(unsigned k=0;k<2;k++) {
      const scaRef ref=scaEval(&u->curve[k],age);
      u->command[k]=ref.x;u->commandRate[k]=ref.w;
      u->commandAcceleration[k]=ref.a;u->commandJerk[k]=ref.j;
    }
    const float end=prAutoTerminal(u,state,model,history,u->curve,age);
    u->predictedEnd=end;
    if(now-u->lastReplan>=.04f && age<u->chosenT-.10f &&
       (end<u->target+.02f || fabsf(u->equilibrium[0]+model[0].bias/model[0].gain)>.3f ||
        fabsf(u->equilibrium[1]+model[1].bias/model[1].gain)>.3f)) {
      const float remaining=u->chosenT-age;
      // At this point command/rate/acceleration/jerk are the last issued
      // polynomial derivatives. Replanning preserves all four boundary values.
      if(prAutoTryReturn(u,now,state,model,history,.08f,remaining,true))u->replans++;
      else {u->lastReplan=now;if(end<u->target-.02f){u->phase=3;u->recoveryReason=3;}}
    }
    if(u->phase==2) {
      const float elapsed=now-u->unwindBegin;
      for(unsigned k=0;k<2;k++) {
        const scaRef ref=scaEval(&u->curve[k],elapsed);
        u->command[k]=ref.x;u->commandRate[k]=ref.w;
        u->commandAcceleration[k]=ref.a;u->commandJerk[k]=ref.j;
      }
      memcpy(u->safeCurve,u->curve,sizeof(u->safeCurve));u->safeAge=elapsed;u->safeValid=true;
      advanced=true;
      if(elapsed>=u->chosenT)u->phase=3;
    }
  }
  if(u->phase==3 && !advanced) {
    float desired[2],stopAlong=0;
    for(unsigned k=0;k<2;k++)stopAlong+=u->direction[k]*(prCompStopVelocity(delayed[k],&model[k])-u->target*u->direction[k]);
    const float budget=prAutoNeutralBudget(u,state,model,history)-u->target;
    if(!isfinite(budget))return false;
    for(unsigned k=0;k<2;k++) {
      // The settling controller regulates the full remaining stop coordinate,
      // not instantaneous velocity alone. It avoids a velocity-reference step
      // and the large inverse-model sign reversal at the end of the old tail.
      const float stop=prCompStopVelocity(delayed[k],&model[k])-u->target*u->direction[k]+u->direction[k]*(budget-stopAlong);
      const float acceleration=u->limits.settleGain*stop+u->direction[k]*(4.f-u->limits.settleGain)*fminf(0,budget);
      desired[k]=(acceleration/PR_COMP_G_DEG-model[k].bias)/model[k].gain;
    }
    {
      float correction[2],norm=0;
      for(unsigned k=0;k<2;k++){correction[k]=desired[k]+model[k].bias/model[k].gain;norm+=correction[k]*correction[k];}
      norm=sqrtf(norm);
      if(norm>u->limits.settleCorrection)for(unsigned k=0;k<2;k++)desired[k]=-model[k].bias/model[k].gain+u->limits.settleCorrection*correction[k]/norm;
    }
    if(u->phase==3) {
      float brake=0,norm=0;
      for(unsigned k=0;k<2;k++) {
        brake+=u->direction[k]*(model[k].gain*desired[k]+model[k].bias);
        norm+=u->direction[k]*u->direction[k]*model[k].gain*model[k].gain;
      }
      const float allowed=u->limits.settleGain*fmaxf(0,u->worstStop-u->target)/PR_COMP_G_DEG;
      if(brake>allowed && norm>0)for(unsigned k=0;k<2;k++)desired[k]-=(brake-allowed)*u->direction[k]*model[k].gain/norm;
    }
    if(!prAutoBound(u,desired,dt))return false;
  }
  float stopError[2],commandError[2];
  for(unsigned k=0;k<2;k++) {
    stopError[k]=prCompStopVelocity(delayed[k],&model[k])-u->target*u->direction[k];
    commandError[k]=u->command[k]+model[k].bias/model[k].gain;
  }
  if(!isfinite(u->command[0]) || !isfinite(u->command[1]) || hypotf(u->command[0],u->command[1])>u->limit+.01f)return false;
  const bool terminal=fresh && u->phase>=3 &&
    hypotf(state[0].v-u->target*u->direction[0],state[1].v-u->target*u->direction[1])<=.02f &&
    hypotf(physicalAngle[0],physicalAngle[1])<=.5f &&
    hypotf(state[0].rate,state[1].rate)<=5 && hypotf(stopError[0],stopError[1])<=.02f &&
    hypotf(commandError[0],commandError[1])<=.5f && hypotf(u->commandRate[0],u->commandRate[1])<=5 &&
    u->worstStop>=u->target-.02f;
  if(terminal){if(!u->stable){u->stable=true;u->stableSince=now;}if(now-u->stableSince>=.1f)u->phase=4;}
  else {u->stable=false;if(u->phase==4)u->phase=3;}
  output[0]=u->command[0];output[1]=u->command[1];u->last=now;return true;
}
