#pragma once
#include "post_release_response_compensation.h"
#include "post_release_adaptive_scurve.h"
#include <string.h>

// Body-yaw X/pitch, Y/roll. Angles/rates are degrees, velocity SI.
// Strong directional braking followed by a C3 actual-attitude return.
// The calibrated response inverse tracks the attitude/velocity reference;
// delayed state is recomputed on every tick, never replaced by a time ramp.
#define PR_AUTO_API 1u
#define PR_AUTO_MIN_T .15f
#define PR_AUTO_MAX_T 1.2f
#define PR_AUTO_RATE 160.f
#define PR_AUTO_POLE 10.f
// Forward reserve during reference return, removed by measured settling.
// It is not the requested final velocity or a relaxed completion gate.
#define PR_AUTO_RESERVE .12f
typedef struct {
  scaPoly curve[2];
  float command[2],commandRate[2],begin,last,chosenT,predictedEnd;
  float initialV[2],direction[2],target,limit,maxTime,unwindBegin,stableSince;
  uint8_t phase; // 1 brake, 2 unwind, 3 measured settling, 4 complete
  uint8_t recoveryReason; // 1 no feasible return, 2 terminal reverse, 3 intermediate reverse
  bool stable;
} prAutoUnwind;
static inline float prAutoIntegralTo(const scaPoly *p,float t) {
  const float x=scaClip(t/p->T,0,1);float value=p->c[7]/8;
  for(int k=6;k>=0;k--)value=value*x+p->c[k]/(k+1);
  return p->T*x*value;
}
static inline float prAutoTerminal(const prCompState delayed[2],
    const scaPoly plan[2],const float direction[2]) {
  float v=0;for(unsigned k=0;k<2;k++)
    v+=direction[k]*(delayed[k].v-PR_COMP_G_DEG*prAutoIntegralTo(&plan[k],plan[k].T));
  return v;
}
static inline float prAutoMinVelocity(const prCompState delayed[2],
    const scaPoly plan[2],const float direction[2]) {
  float minimum=INFINITY;
  for(unsigned i=0;i<=24;i++) {
    float v=0;for(unsigned k=0;k<2;k++)
      v+=direction[k]*(delayed[k].v-PR_COMP_G_DEG*prAutoIntegralTo(&plan[k],plan[k].T*i/24));
    minimum=fminf(minimum,v);
  }
  return minimum;
}
static inline bool prAutoPlan(const prCompState delayed[2],
    const prResponseAxis model[2],const float command[2],const float rate[2],
    float T,float limit,scaPoly plan[2]) {
  for(unsigned k=0;k<2;k++) {
    const prCompState d=prCompDerivative(delayed[k],command[k],&model[k]);
    const float jerk=model[k].wn*model[k].wn*(model[k].gain*rate[k]-delayed[k].rate)-
      2*model[k].zeta*model[k].wn*d.rate;
    scaMake((scaRef){.x=delayed[k].angle,.w=delayed[k].rate,.a=d.rate,.j=jerk},0,T,0,&plan[k]);
  }
  for(unsigned i=0;i<=24;i++) {
    float cmd[2],slew[2],tilt[2];
    for(unsigned k=0;k<2;k++) {
      const scaRef r=scaEval(&plan[k],T*i/24);
      const float w2=model[k].wn*model[k].wn,d=2*model[k].zeta*model[k].wn;
      cmd[k]=(r.a+d*r.w+w2*(r.x-model[k].bias))/(w2*model[k].gain);
      slew[k]=(r.j+d*r.a+w2*r.w)/(w2*model[k].gain);tilt[k]=r.x;
    }
    if(!isfinite(cmd[0])||!isfinite(cmd[1])||hypotf(cmd[0],cmd[1])>limit+1e-3f||
       hypotf(tilt[0],tilt[1])>limit+1e-3f||hypotf(slew[0],slew[1])>PR_AUTO_RATE+1e-2f)return false;
  }
  return true;
}
static inline bool prAutoInit(prAutoUnwind *u,float now,const float velocity[2],
    const float command[2],float target,float tiltLimit,float maxTime) {
  if(!u||!isfinite(now)||!isfinite(target)||target<0||target>.3f||
     !isfinite(tiltLimit)||tiltLimit<1||tiltLimit>40||!isfinite(maxTime)||maxTime<.5f||maxTime>12||
     !isfinite(velocity[0])||!isfinite(velocity[1])||!isfinite(command[0])||!isfinite(command[1])||
     hypotf(command[0],command[1])>tiltLimit)return false;
  const float speed=hypotf(velocity[0],velocity[1]);
  *u=(prAutoUnwind){.begin=now,.last=now,.phase=1,.target=target,.limit=tiltLimit,.maxTime=maxTime};
  u->direction[0]=speed>.001f?velocity[0]/speed:1;u->direction[1]=speed>.001f?velocity[1]/speed:0;
  memcpy(u->command,command,sizeof(u->command));return true;
}
static inline void prAutoBound(prAutoUnwind *u,const float desired[2],float dt) {
  float goal[2]={desired[0],desired[1]},n=hypotf(goal[0],goal[1]);
  if(n>u->limit)for(unsigned k=0;k<2;k++)goal[k]*=u->limit/n;
  float step[2]={goal[0]-u->command[0],goal[1]-u->command[1]};n=hypotf(step[0],step[1]);
  if(n>PR_AUTO_RATE*dt && n>0)for(unsigned k=0;k<2;k++)step[k]*=PR_AUTO_RATE*dt/n;
  for(unsigned k=0;k<2;k++){u->command[k]+=step[k];u->commandRate[k]=dt>0?step[k]/dt:0;}
}
static inline bool prAutoStep(prAutoUnwind *u,float now,const prCompState state[2],
    const prResponseAxis model[2],const float history[2][76],float decel,bool fresh,const float physicalAngle[2],float output[2]) {
  if(!u||!output||!physicalAngle)return false;
  const float dt=now-u->last;
  if(!isfinite(now)||dt<0||dt>.03f||now-u->begin>u->maxTime||!isfinite(decel)||decel<=0||decel>8.23f)return false;
  prCompState delayed[2];float vAlong=0;
  for(unsigned k=0;k<2;k++) {
    if(!isfinite(state[k].v)||!isfinite(state[k].angle)||!isfinite(state[k].rate)||
       !isfinite(model[k].delay)||model[k].delay<0||model[k].delay>=.15f||
       !isfinite(model[k].wn)||model[k].wn<=5||model[k].wn>=60||
       !isfinite(model[k].zeta)||model[k].zeta<=.2f||model[k].zeta>=2||
       !isfinite(model[k].gain)||model[k].gain<=.7f||model[k].gain>=1.3f||
       !isfinite(model[k].bias)||fabsf(model[k].bias)>8)return false;
    for(unsigned i=0;i<76;i++)if(!isfinite(history[k][i]))return false;
    delayed[k]=prCompPredict(state[k],&model[k],history[k]);
    vAlong+=u->direction[k]*state[k].v;
  }
  if(u->phase==1 && vAlong<=u->target+.02f)u->phase=3;
  if(u->phase==1) {
    scaPoly plan[2];float Tmin=0;
    // Fixed upper work bound. Reuse the delay prediction for all candidates.
    for(unsigned i=0;i<=70;i++) {
      const float T=PR_AUTO_MIN_T+.015f*i;
      if(prAutoPlan(delayed,model,u->command,u->commandRate,T,u->limit,plan)) {Tmin=T;break;}
    }
    if(!Tmin) {
      float neutralStop=0;
      for(unsigned k=0;k<2;k++)neutralStop+=u->direction[k]*prCompStopVelocity(delayed[k],&model[k]);
      // Initial command slew can temporarily make a C3 splice infeasible.
      // Continue the bounded brake entry only while its neutral-return stop
      // margin remains positive; never wait through a predicted reversal.
      if(neutralStop<=u->target+.02f){u->phase=3;u->recoveryReason=1;}
    }
    if(Tmin) {
      const float end=prAutoTerminal(delayed,plan,u->direction);
      const float returnV=u->target+PR_AUTO_RESERVE;
      // If a smooth return is already too late, do not execute a knowingly
      // reversing reference. Keep attitude authority and damp measured v/q/w.
      if(end<u->target-.005f){u->phase=3;u->recoveryReason=2;}
      if(u->phase==1 && end<=returnV+.025f+decel*dt) {
        // Select a return duration from the required remaining velocity
        // impulse, then track it with measured-state feedback each tick.
        float low=Tmin,high=PR_AUTO_MAX_T;
        scaPoly longest[2];
        const bool bracket=end>returnV && prAutoPlan(delayed,model,u->command,u->commandRate,high,u->limit,longest) &&
          prAutoTerminal(delayed,longest,u->direction)<=returnV;
        if(bracket)for(unsigned i=0;i<9;i++) {
          const float T=.5f*(low+high);scaPoly trial[2];
          if(!prAutoPlan(delayed,model,u->command,u->commandRate,T,u->limit,trial)) {high=T;continue;}
          if(prAutoTerminal(delayed,trial,u->direction)>returnV)low=T;else high=T;
        }
        if(bracket && !prAutoPlan(delayed,model,u->command,u->commandRate,low,u->limit,plan))return false;
        if(prAutoMinVelocity(delayed,plan,u->direction)<u->target-.005f){u->phase=3;u->recoveryReason=3;}
        else {
          memcpy(u->curve,plan,sizeof(u->curve));u->chosenT=plan[0].T;
          u->unwindBegin=now;u->phase=2;u->predictedEnd=prAutoTerminal(delayed,plan,u->direction);
          for(unsigned k=0;k<2;k++)u->initialV[k]=delayed[k].v;
        }
      }
    }
    if(u->phase==1) {
      float desired[2];
      for(unsigned k=0;k<2;k++)desired[k]=(-model[k].bias+
        (decel*u->direction[k]+2*(state[k].v-u->direction[k]*vAlong))/PR_COMP_G_DEG)/model[k].gain;
      prAutoBound(u,desired,dt);
    }
  }
  if(u->phase==2) {
    const float age=now-u->unwindBegin;float desired[2];
    for(unsigned k=0;k<2;k++) {
      const scaRef q=scaEval(&u->curve[k],age);
      const float ref[4]={u->initialV[k]-PR_COMP_G_DEG*prAutoIntegralTo(&u->curve[k],age),
        -PR_COMP_G_DEG*q.x,-PR_COMP_G_DEG*q.w,-PR_COMP_G_DEG*q.a};
      desired[k]=-prCompAcceleration(delayed[k],&model[k],ref,PR_AUTO_POLE)/PR_COMP_G_DEG;
    }
    prAutoBound(u,desired,dt);
    if(age>=u->chosenT)u->phase=3;
  }
  if(u->phase==3) {
    float desired[2];
    for(unsigned k=0;k<2;k++) {
      const float ref[4]={u->target*u->direction[k],0,0,0};
      desired[k]=-prCompAcceleration(delayed[k],&model[k],ref,PR_AUTO_POLE)/PR_COMP_G_DEG;
    }
    prAutoBound(u,desired,dt);
  }
  float stopError[2],commandError[2];
  for(unsigned k=0;k<2;k++) {
    stopError[k]=prCompStopVelocity(delayed[k],&model[k])-u->target*u->direction[k];
    commandError[k]=u->command[k]+model[k].bias/model[k].gain;
  }
  const bool terminal=fresh && u->phase>=3 &&
    hypotf(state[0].v-u->target*u->direction[0],state[1].v-u->target*u->direction[1])<=.02f &&
    hypotf(physicalAngle[0],physicalAngle[1])<=.5f &&
    hypotf(state[0].rate,state[1].rate)<=5 && hypotf(stopError[0],stopError[1])<=.02f &&
    hypotf(commandError[0],commandError[1])<=.5f && hypotf(u->commandRate[0],u->commandRate[1])<=5;
  if(terminal){if(!u->stable){u->stable=true;u->stableSince=now;}if(now-u->stableSince>=.1f)u->phase=4;}
  else {u->stable=false;if(u->phase==4)u->phase=3;}
  output[0]=u->command[0];output[1]=u->command[1];u->last=now;return true;
}
