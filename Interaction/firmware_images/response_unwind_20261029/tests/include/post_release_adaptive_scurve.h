#pragma once
// Experimental response-aware finite-horizon brake. Pure C, no truth input.
// Local rotational plant gain, COM offset, motor lag and real PID memory;
// not a complete 6-DOF rigid-body/motor model. Every prediction includes rate.
#include <math.h>
#include <stdbool.h>
#include <string.h>

typedef struct { float c[8], T, second[8], end; bool split; } scaPoly;
typedef struct { float x, w, a, j; } scaRef;
typedef struct { float v, x, w, a, integral, attIntegral; } scaState;
typedef struct {
  float tau, kp, damping, bias, accBias, gain,rp,ri,rd,aki,akd,lever,leverX;
  bool closed;
  float delay,wn,zeta,cgain,cbias,history[76];
} scaModel;
typedef struct { float v, x, w, a, minV, distance, peakX, peakW, peakA; } scaPrediction;
static inline float scaClip(float x,float a,float b) { return fminf(b,fmaxf(a,x)); }
static inline void scaMake(scaRef s,float end,float T,float bump,scaPoly *p) {
  p->T=T;p->end=end;p->split=false;float *c=p->c;
  c[0]=s.x-end;c[1]=s.w*T;c[2]=s.a*T*T*.5f;c[3]=s.j*T*T*T/6;
  c[4]=-35*c[0]-20*c[1]-10*c[2]-4*c[3];
  c[5]=84*c[0]+45*c[1]+20*c[2]+6*c[3];
  c[6]=-70*c[0]-36*c[1]-15*c[2]-4*c[3];
  c[7]=20*c[0]+10*c[1]+4*c[2]+c[3];c[0]+=end;
  // Degree-7 null-space: preserves start angle/rate/acceleration and
  // terminal angle/rate/acceleration/jerk. Replans are C2, not C3.
  c[3]+=128*bump;c[4]-=512*bump;c[5]+=768*bump;
  c[6]-=512*bump;c[7]+=128*bump;
}
static inline scaRef scaEval(const scaPoly *p,float t) {
  if(t>=p->T)return (scaRef){.x=p->end};
  const float duration=p->split?p->T*.5f:p->T;
  const float *c=p->split && t>=duration?p->second:p->c;
  const float local=p->split && t>=duration?t-duration:t;
  const float u=scaClip(local/duration,0,1);scaRef r={0};
  // M4F has single-precision hardware only. Terminal value is explicit above;
  // keep the bounded interior polynomial on the FPU, not soft double math.
  float x=c[7],w=7*c[7],a=42*c[7],j=210*c[7];
  for(int k=6;k>=0;k--) {
    x=x*u+c[k];
    if(k>=1)w=w*u+k*c[k];
    if(k>=2)a=a*u+k*(k-1)*c[k];
    if(k>=3)j=j*u+k*(k-1)*(k-2)*c[k];
  }
  r.x=x;r.w=w/duration;r.a=a/(duration*duration);r.j=j/(duration*duration*duration);
  if(t>=p->T) {r.w=r.a=r.j=0;}
  return r;
}
static inline void scaBridge(scaRef start,scaRef mid,float T,scaPoly *p) {
  scaMake(start,0,T*.5f,0,p);float *c=p->c;const float h=T*.5f;
  const float e0=mid.x,e1=mid.w*h,e2=mid.a*h*h,e3=mid.j*h*h*h;
  c[4]+=35*e0-15*e1+2.5f*e2-e3/6;
  c[5]+=-84*e0+39*e1-7*e2+e3/2;
  c[6]+=70*e0-34*e1+6.5f*e2-e3/2;
  c[7]+=-20*e0+10*e1-2*e2+e3/6;
  scaPoly second;scaMake(mid,0,h,0,&second);
  memcpy(p->second,second.c,sizeof(second.c));p->T=T;p->split=true;
}
static inline void scaSmoothBump(scaRef start,float T,float bump,scaPoly *p) {
  scaPoly base;scaMake(start,0,T,0,&base);
  scaRef mid=scaEval(&base,T*.5f);mid.x+=bump;
  // Two degree-7 pieces, C3 at the splice and on every replan. Unlike a
  // degree-7 u^3(1-u)^4 correction, this cannot jump the jerk feedforward.
  scaBridge(start,mid,T,p);
}
static inline float scaFeedforward(scaRef r,scaState z,const scaModel *m) {
  // A fitted closed-loop model already contains the PID dynamics. Do not
  // apply the physical motor/PID inverse on top of it.
  if(m->closed)return 0;
  if(m->gain>0)return scaClip(r.w+
    ((r.a+m->tau*r.j-m->bias)/m->gain-m->ri*z.integral+m->rd*z.a)/m->rp-
    m->aki*z.attIntegral+m->akd*z.w+.5f*(r.w-z.w),-2.79f,2.79f);
  return scaClip(r.w+m->tau*r.a+m->damping*(r.w-z.w),-2.79f,2.79f);
}
static inline float scaClosedCommand(float angle,const scaModel *m) {
  return scaClip((angle-m->cbias)/m->cgain,-.395f,.395f);
}
static inline scaPrediction scaClosedRollout(const scaPoly *p,scaState z,
    const scaModel *m,float tail) {
  scaPrediction o={.minV=z.v};
  const unsigned n=(unsigned)ceilf((p->T+tail)/.002f);
  const float h=(p->T+tail)/n;
  // Exact second-order transition for each held target (float-only). It
  // advances measured angle AND angular rate; no zero-rate initialization.
  const float d=m->zeta*m->wn, w2=m->wn*m->wn, disc=d*d-w2;
  float c,b;
  if(fabsf(disc)<1e-4f) {c=1;b=h;}
  else if(disc>0) {const float q=sqrtf(disc);c=coshf(q*h);b=sinhf(q*h)/q;}
  else {const float q=sqrtf(-disc);c=cosf(q*h);b=sinf(q*h)/q;}
  const float e=expf(-d*h),a00=e*(c+d*b),a01=e*b;
  const float a10=-w2*a01,a11=e*(c-d*b);
  float held=0,commandEpoch=-1e9f;
  for(unsigned k=0;k<n;k++) {
    const float t=k*h, delayed=t-m->delay;
    float command;
    if(delayed<0) {
      const int index=(int)floorf((delayed+.15f)/.002f+1e-5f);
      command=m->history[(int)scaClip(index,0,75)];
    } else {
      const float epoch=floorf((delayed+1e-6f)/.01f)*.01f;
      if(epoch!=commandEpoch) {held=scaClosedCommand(scaEval(p,epoch).x,m);commandEpoch=epoch;}
      command=held;
    }
    const float target=m->cgain*command+m->cbias,err=z.x-target;
    const float nextX=target+a00*err+a01*z.w,nextW=a10*err+a11*z.w;
    const float acc=9.81f*tanf(.5f*(z.x+nextX));
    o.distance+=h*z.v+.5f*h*h*acc;z.v+=h*acc;
    z.a=(nextW-z.w)/h;z.x=nextX;z.w=nextW;
    o.minV=fminf(o.minV,z.v);
    const scaRef r=scaEval(p,t);
    o.peakX=fmaxf(o.peakX,fabsf(scaClosedCommand(r.x,m)));
    o.peakW=fmaxf(o.peakW,fabsf(r.w));o.peakA=fmaxf(o.peakA,fabsf(r.a));
  }
  o.v=z.v;o.x=z.x;o.w=z.w;o.a=z.a;return o;
}
static inline scaPrediction scaRollout(const scaPoly *p,scaState z,
    const scaModel *m,float tail) {
  if(m->closed)return scaClosedRollout(p,z,m,tail);
  scaPrediction o={.minV=z.v};float distance=0;
  // Input/output velocity is the tracked frame origin, not the COM. Rollout
  // force integration is at the COM; angular momentum moves the tracked point.
  z.v+=(m->lever*cosf(z.x)-m->leverX*sinf(z.x))*z.w;
  const unsigned n=(unsigned)ceilf((p->T+tail)/.002f);
  const float h=(p->T+tail)/n;
  scaRef r={0};float heldFeedforward=0,nextCommand=0;
  for(unsigned k=0;k<n;k++) {
    // HLC is 100 Hz; inner PID is 500 Hz. Predict the same zero-order hold
    // on attitude/rate feedforward, not an unrealizable continuously updated
    // inverse model using future gyro acceleration and PID integral values.
    if(k*h+1e-6f>=nextCommand) {
      r=scaEval(p,k*h);heldFeedforward=scaFeedforward(r,z,m);nextCommand+=.01f;
    }
    const float command=m->kp*(r.x-z.x)+m->aki*z.attIntegral-m->akd*z.w+
      heldFeedforward;
    float dw=(command-z.w)/m->tau+m->bias;
    if(m->gain>0) {
      const float torqueAccel=m->gain*(m->rp*(command-z.w)+m->ri*z.integral-m->rd*z.a)+m->bias;
      const float nextA=z.a+(1-expf(-h/m->tau))*(torqueAccel-z.a);
      dw=.5f*(z.a+nextA);z.a=nextA;
      z.integral=scaClip(z.integral+h*(command-z.w),-.581195f,.581195f);
      z.attIntegral=scaClip(z.attIntegral+h*(r.x-z.x),-.349066f,.349066f);
    }
    const float acc=9.81f*tanf(z.x+.5f*h*z.w)+m->accBias;
    distance+=h*z.v+.5f*h*h*acc;z.v+=h*acc;
    z.x+=h*z.w+.5f*h*h*dw;z.w+=h*dw;
    const float pointV=z.v-(m->lever*cosf(z.x)-m->leverX*sinf(z.x))*z.w;
    o.minV=fminf(o.minV,pointV);o.peakX=fmaxf(o.peakX,fabsf(r.x));
    o.peakW=fmaxf(o.peakW,fabsf(r.w));o.peakA=fmaxf(o.peakA,fabsf(r.a));
  }
  o.v=z.v-(m->lever*cosf(z.x)-m->leverX*sinf(z.x))*z.w;
  o.x=z.x;o.w=z.w;o.a=z.a;o.distance=distance;return o;
}
static inline bool scaSolve(scaRef start,scaState measured,const scaModel *m,
    float T,float tail,float terminalV,scaPoly *p,scaPrediction *out,float *bump) {
  if(m->gain>0 || m->closed) {
    float b=0;
    for(unsigned k=0;k<4;k++) {
      scaSmoothBump(start,T,b,p);*out=scaRollout(p,measured,m,tail);
      if(fabsf(out->v-terminalV)<.0005f)break;
      scaPoly shifted;scaSmoothBump(start,T,b+.002f,&shifted);
      scaPrediction other=scaRollout(&shifted,measured,m,tail);
      const float slope=(other.v-out->v)/.002f;
      if(!isfinite(slope) || fabsf(slope)<.02f){*bump=b;return false;}
      b+=scaClip((terminalV-out->v)/slope,-.2f,.2f);
    }
    scaSmoothBump(start,T,b,p);*out=scaRollout(p,measured,m,tail);*bump=b;
    return isfinite(out->v) && fabsf(out->v-terminalV)<.008f && out->peakX<.395f &&
      out->peakW<2.79f && out->peakA<34.91f;
  }
  *out=(scaPrediction){0};*bump=0;return false;
}
