// Native tests of the production reference planner; no flight commands.
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "post_release_stop_request.h"

static void checkCurve(float speed, float cross, float cap, float tail, float distance) {
  prVelocityCurve curve;
  prStopRequestLog info;
  assert(prStopCurveInit(&curve, speed, cross, cap, distance, tail, 12, &info)==PR_STOP_OK);
  assert(info.flags & PR_STOP_VALID);
  assert(curve.peakDecel<=cap+1e-5f);
  const unsigned steps=10000;
  const float dt=curve.total/steps;
  float previous=speed, area=0;
  for(unsigned i=0;i<=steps;i++) {
    const float t=i*dt;
    prVelocityCurveAdvance(&curve,t);
    float v[2];prVelocityCurveEval(&curve,t,v);
    assert(isfinite(v[0]) && isfinite(v[1]));
    assert(v[0]>=0 && v[0]<=previous+2e-4f);
    const scaRef ref=prVelocityAlongEval(&curve,t-(curve.inTail?curve.ramp:0));
    assert(ref.w>=-cap-2e-3f && ref.w<=2e-3f);
    if(i)area+=(previous+v[0])*.5f*dt;
    previous=v[0];
  }
  float terminal[2];prVelocityCurveEval(&curve,curve.total,terminal);
  assert(terminal[0]==0 && terminal[1]==0);
  assert(fabsf(area-info.distance)<3e-4f);
  if(distance>0)assert(fabsf(info.distance-distance)<3e-5f);
}

static void checkTiltEnvelope(void) {
  assert(prStopWithinTilt(8.23f,40));
  assert(!prStopWithinTilt(8.23f,39));
  assert(prStopWithinTilt(4,PR_STOP_LEGACY_TILT_DEG));
  assert(!prStopWithinTilt(6,PR_STOP_LEGACY_TILT_DEG));
  const float invalid[]={0,-1,40.001f,NAN,INFINITY};
  for(unsigned i=0;i<5;i++)assert(!prStopTiltValid(invalid[i]));
  const float limits[]={PR_STOP_LEGACY_TILT_DEG,30,40};
  for(unsigned k=0;k<3;k++)for(unsigned i=0;i<360;i+=2) {
    const float limit=limits[k],heading=i*PR_STOP_DEG_TO_RAD;
    float xy[2]={100*cosf(heading),100*sinf(heading)};
    assert(prStopLimitXY(xy,prStopLinearAccelerationLimit(limit)));
    const float roll=-xy[1]/PR_STOP_GRAVITY_M_S2,pitch=-xy[0]/PR_STOP_GRAVITY_M_S2;
    assert(hypotf(roll,pitch)<=limit*PR_STOP_DEG_TO_RAD+2e-6f);
    assert(acosf(cosf(roll)*cosf(pitch))<=limit*PR_STOP_DEG_TO_RAD+2e-6f);
    float nonlinear[2]={100*cosf(heading),100*sinf(heading)};
    assert(prStopLimitXY(nonlinear,prStopAccelerationLimit(limit)));
    const float p=atanf(nonlinear[0]/PR_STOP_GRAVITY_M_S2);
    const float r=atanf(-nonlinear[1]*cosf(p)/PR_STOP_GRAVITY_M_S2);
    assert(fabsf(acosf(cosf(r)*cosf(p))-limit*PR_STOP_DEG_TO_RAD)<2e-6f);
    float small[2]={.01f,.02f};
    assert(!prStopLimitXY(small,prStopLinearAccelerationLimit(limit)));
    assert(small[0]==.01f && small[1]==.02f);
  }
  puts("PASS: 540 directions/tilt limits in compensated and uncompensated command coordinates");
}

int main(void) {
  assert(PR_STOP_DEFAULT_TILT_DEG==40.0f);
  for(unsigned shape=0;shape<2;shape++)for(unsigned exec=0;exec<3;exec++)for(unsigned matched=0;matched<2;matched++) {
    const bool eligible=shape==0 && exec==2 && !matched;
    assert(prStopRuntimeTilt(40,shape,exec,matched)==(eligible?40:PR_STOP_LEGACY_TILT_DEG));
    assert(prStopRuntimeTilt(25,shape,exec,matched)==(eligible?25:PR_STOP_LEGACY_TILT_DEG));
  }
  puts("PASS: boot default 40 degrees; 12 executor/state combinations preserve scope and explicit smaller limits");

  checkTiltEnvelope();
  assert(PR_STOP_MAX_DECEL_M_S2==8.23f);
  const float speeds[]={.1f,.5f,1,2,3};
  const float caps[]={.6f,3.6f,3.7f,4,6,8.23f};
  const float tails[]={.15f,.8f,1.2f,3};
  unsigned checked=0;
  for(unsigned i=0;i<5;i++)for(unsigned j=0;j<6;j++)for(unsigned k=0;k<4;k++) {
    checkCurve(speeds[i],.1f,caps[j],tails[k],0);checked++;
    prVelocityCurve p;prStopRequestLog info;
    assert(prStopCurveInit(&p,speeds[i],0,caps[j],0,tails[k],12,&info)==PR_STOP_OK);
    // Re-request the same integral through the distance-constrained branch.
    checkCurve(speeds[i],-.1f,caps[j],tails[k],info.distance);checked++;
  }
  const float bad[]={0,-1,8.2301f,9,NAN,INFINITY};
  for(unsigned i=0;i<6;i++) {
    assert(!prStopDecelValid(bad[i]));
    prVelocityCurve p, original;memset(&p,0x5a,sizeof(p));original=p;
    prStopRequestLog info;
    assert(prStopCurveInit(&p,1,0,bad[i],0,.8f,6,&info)==PR_STOP_INVALID);
    assert(memcmp(&p,&original,sizeof(p))==0);
  }
  prVelocityCurve old, stronger;prStopRequestLog a,b;
  assert(prStopCurveInit(&old,3,0,3.6f,0,.8f,6,&a)==PR_STOP_OK);
  assert(prStopCurveInit(&stronger,3,0,4,0,.8f,6,&b)==PR_STOP_OK);
  assert(b.duration<a.duration && b.distance<a.distance);
  // With a low release speed, the configured tail, not the cap, sets the peak.
  assert(prStopCurveInit(&old,.5f,0,3.6f,0,1.2f,6,&a)==PR_STOP_OK);
  assert(prStopCurveInit(&stronger,.5f,0,4,0,1.2f,6,&b)==PR_STOP_OK);
  assert(a.duration==b.duration && a.distance==b.distance && a.peak==b.peak);
  puts("PASS: 240 free/distance reference cases; invalid limits; stronger and tail-limited cases");
  assert(checked==240);
  return 0;
}
