#include "post_release_local_unwind.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static const float zero[3]={0};
static const float forward[2]={1,0};
static const jointModel_t model={{.05f,.07f,.08f},{6,7.1f,6},{0,0},{0},{1,1,1}};

static postReleaseLocalUnwindDecision_t decide(jointState_t s, float yaw,
    const float dir[2], const float ref[3]) {
  postReleaseLocalUnwindDecision_t result;
  assert(postReleaseLocalUnwindDecide(&s,&model,ref,zero,zero,zero,
    yaw,dir,.20f,.01f,.03f,&result));
  assert(result.integrationSteps==POST_RELEASE_LOCAL_UNWIND_MAX_STEPS);
  assert(result.integrationSteps<=128);
  return result;
}

int main(void) {
  const float rapid[3]={0,atanf(4.0f/9.81f)*180.0f/3.14159265358979323846f,0};
  jointState_t level={{1.03f,0},{0,0,0},{0,0,0}};
  const postReleaseLocalUnwindDecision_t low=decide(level,0,forward,rapid);
  // The old fixed-max-a reserve forbade this entire release. Actual level
  // dynamics leave braking room: no zero-window/failure just for v=1.03.
  assert(!low.startUnwind);
  assert(fabsf(low.predictionHorizonS-.70f)<1e-6f);
  assert(low.endForwardNow>.03f && low.endForwardAfterTick>.03f);
  assert(fabsf(low.accelerationAlong)<1e-6f && fabsf(low.jerkAlong)<1e-6f);
  assert(low.endForwardAfterTick<low.endForwardNow);

  jointState_t tilted={{.20f,0},{0,20,0},{0,40,0}};
  const postReleaseLocalUnwindDecision_t strong=decide(tilted,0,forward,rapid);
  assert(strong.startUnwind && strong.minForwardNow<0);
  assert(strong.accelerationAlong < -3.5f && strong.jerkAlong < -7.0f);
  // It reports reversal risk, not fictitious terminal convergence.
  assert(strong.endForwardNow < -.025f);
  assert(fabsf(strong.terminalNow.velocity[0])>.03f);

  jointState_t opening=tilted;
  opening.rate[1]=-40;
  const postReleaseLocalUnwindDecision_t releasing=decide(opening,0,forward,rapid);
  assert(releasing.jerkAlong>7.0f);
  assert(releasing.endForwardNow>strong.endForwardNow);

  // Rotate the complete world fixture: internal CF axes remain unchanged.
  const float rotatedDirection[2]={0,1};
  jointState_t rotated=tilted;
  rotated.velocity[0]=0; rotated.velocity[1]=.20f;
  const postReleaseLocalUnwindDecision_t yaw90=decide(rotated,90,rotatedDirection,rapid);
  assert(fabsf(yaw90.endForwardNow-strong.endForwardNow)<1e-5f);
  assert(fabsf(yaw90.jerkAlong-strong.jerkAlong)<1e-5f);
  assert(fabsf(yaw90.accelerationAlong-strong.accelerationAlong)<1e-5f);
  const float reversedDirection[2]={-1,0};
  rotated.velocity[0]=-.20f;rotated.velocity[1]=0;
  const postReleaseLocalUnwindDecision_t yaw180=decide(rotated,180,reversedDirection,rapid);
  assert(fabsf(yaw180.endForwardNow-strong.endForwardNow)<1e-5f);

  // C3 at the issue boundary is independent of the measured body state.
  const float ref[3]={2,5,1}, rate[3]={1,-2,.5f};
  const float acceleration[3]={1,2,3}, jerk[3]={3,2,1};
  postReleaseLocalUnwindDecision_t continuous;
  assert(postReleaseLocalUnwindDecide(&level,&model,ref,rate,acceleration,jerk,
    0,forward,.2f,.01f,.03f,&continuous));
  float out[3],r[3],a[3],j[3];
  assert(postReleaseUnwindEvaluate(&continuous.profile,0,out,r,a,j));
  for (unsigned i=0;i<3;++i) {
    assert(fabsf(out[i]-ref[i])<1e-5f);
    assert(fabsf(r[i]-rate[i])<1e-5f);
    assert(fabsf(a[i]-acceleration[i])<1e-5f);
    assert(fabsf(j[i]-jerk[i])<1e-5f);
  }
  assert(postReleaseUnwindEvaluate(&continuous.profile,.2f,out,r,a,j));
  for(unsigned i=0;i<3;++i) assert(out[i]==0 && r[i]==0 && a[i]==0 && j[i]==0);

  jointModel_t bias=model;
  bias.integralRateBias[1]=10;
  postReleaseLocalUnwindDecision_t withIntegral;
  assert(postReleaseLocalUnwindDecide(&level,&bias,rapid,zero,zero,zero,
    0,forward,.2f,.01f,.03f,&withIntegral));
  assert(withIntegral.endForwardNow<low.endForwardNow-.01f);
  bias=model;bias.integralGain[1]=0;
  postReleaseLocalUnwindDecision_t withoutIUpdate;
  assert(postReleaseLocalUnwindDecide(&level,&bias,rapid,zero,zero,zero,
    0,forward,.2f,.01f,.03f,&withoutIUpdate));
  assert(fabsf(withoutIUpdate.endForwardNow-low.endForwardNow)>1e-5f);

  // Analytic jerk agrees with finite-differencing attitude at fixed rates;
  // yaw motion contributes, and no Vicon differentiation is required.
  jointState_t turning={{2,.1f},{8,12,3},{4,-6,9}};
  const postReleaseLocalUnwindDecision_t base=decide(turning,25,forward,ref);
  const float epsilon=.001f;
  for(unsigned i=0;i<3;++i) turning.angle[i]+=turning.rate[i]*epsilon;
  const postReleaseLocalUnwindDecision_t advanced=decide(turning,25,forward,ref);
  assert(fabsf((advanced.accelerationAlong-base.accelerationAlong)/epsilon-base.jerkAlong)<.01f);

  // Invalid data cannot change a caller's current executable profile.
  postReleaseLocalUnwindDecision_t unchanged=low, saved=low;
  jointState_t bad=level;bad.velocity[0]=NAN;
  assert(!postReleaseLocalUnwindDecide(&bad,&model,rapid,zero,zero,zero,
    0,forward,.2f,.01f,.03f,&unchanged));
  assert(!memcmp(&unchanged,&saved,sizeof(saved)));
  bad=level;bad.angle[1]=INFINITY;
  assert(!postReleaseLocalUnwindDecide(&bad,&model,rapid,zero,zero,zero,
    0,forward,.2f,.01f,.03f,&unchanged));
  bias=model;bias.tau[1]=0;
  assert(!postReleaseLocalUnwindDecide(&level,&bias,rapid,zero,zero,zero,
    0,forward,.2f,.01f,.03f,&unchanged));
  bias=model;bias.integralGain[1]=NAN;
  assert(!postReleaseLocalUnwindDecide(&level,&bias,rapid,zero,zero,zero,
    0,forward,.2f,.01f,.03f,&unchanged));
  bias=model;bias.attitudeGain[1]=0;
  assert(!postReleaseLocalUnwindDecide(&level,&bias,rapid,zero,zero,zero,
    0,forward,.2f,.01f,.03f,&unchanged));
  const float invalidDirection[2]={0,0};
  assert(!postReleaseLocalUnwindDecide(&level,&model,rapid,zero,zero,zero,
    0,invalidDirection,.2f,.01f,.03f,&unchanged));
  assert(!postReleaseLocalUnwindDecide(&level,&model,rapid,zero,zero,zero,
    0,forward,.2f,.031f,.03f,&unchanged));
  assert(!postReleaseLocalUnwindDecide(&level,&model,rapid,zero,zero,zero,
    0,forward,1.01f,.01f,.03f,&unchanged));
  assert(!memcmp(&unchanged,&saved,sizeof(saved)));

  // Supported-horizon endpoints retain the same statically bounded work.
  for(unsigned k=0;k<2;++k) {
    const float duration=k ? 1.0f : .05f;
    assert(postReleaseLocalUnwindDecide(&level,&model,zero,zero,zero,zero,
      0,forward,duration,.01f,.03f,&unchanged));
    assert(unchanged.integrationSteps==127);
  }
  const clock_t started=clock();
  for(unsigned k=0;k<1000;++k) (void)decide(level,0,forward,rapid);
  const double us=(double)(clock()-started)/CLOCKS_PER_SEC*1e3;
  printf("{\"pass\":true,\"steps\":%u,\"level_1_03_start_unwind\":%s,"
    "\"level_1_03_end_now\":%.6f,\"level_1_03_end_after_tick\":%.6f,"
    "\"strong_tilt_end_now\":%.6f,\"host_mean_us\":%.3f}\n",
    low.integrationSteps,low.startUnwind?"true":"false",low.endForwardNow,
    low.endForwardAfterTick,strong.endForwardNow,us);
  return 0;
}
