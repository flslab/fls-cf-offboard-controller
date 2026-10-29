#include "mocap_marker_geometry.h"
#include "post_release_auto_unwind.h"
#include "post_release_vicon_velocity_kf.h"
#include <assert.h>
#include <stdio.h>
#include <time.h>

static void geometry(void) {
  // Rotation-only motion of a marker 86 mm above a stationary IMU must not
  // manufacture translational velocity. Exercise yaw and both tilt signs.
  const float arm[3]={0,0,.086f};postReleaseViconVelocityKf_t corrected={0},raw={0};
  corrected.observationVarianceM2=1.e-5f;
  for(unsigned i=0;i<2000;i++) {
    const float angle=.35f*sinf(i*.018f),yaw=i*.001f;
    const float q[4]={cosf(yaw/2)*cosf(angle/2),cosf(yaw/2)*sinf(angle/2),
      sinf(yaw/2)*sinf(angle/2),sinf(yaw/2)*cosf(angle/2)};
    float d[3],J[3][3];assert(markerGeometry(q,arm,d,J));
    float observed[3]={1+d[0],2+d[1],3+d[2]},point[3];
    for(unsigned k=0;k<3;k++)point[k]=observed[k]-d[k];
    assert(postReleaseViconVelocityKfUpdate(&raw,observed,10000u*(i+1)));
    assert(postReleaseViconVelocityKfUpdate(&corrected,point,10000u*(i+1)));
    if(i>100)for(unsigned k=0;k<3;k++)assert(fabsf(corrected.velocityMps[k])<.0002f);
    // Finite difference against q * small right/body error.
    for(unsigned axis=0;axis<3;axis++) {
      const float e=.0002f;float b[4]={1,0,0,0};b[axis+1]=e*.5f;
      float qq[4]={q[0]*b[0]-q[1]*b[1]-q[2]*b[2]-q[3]*b[3],
        q[0]*b[1]+q[1]*b[0]+q[2]*b[3]-q[3]*b[2],
        q[0]*b[2]-q[1]*b[3]+q[2]*b[0]+q[3]*b[1],
        q[0]*b[3]+q[1]*b[2]-q[2]*b[1]+q[3]*b[0]},dd[3],JJ[3][3];
      assert(markerGeometry(qq,arm,dd,JJ));
      for(unsigned k=0;k<3;k++)assert(fabsf((dd[k]-d[k])/e-J[k][axis])<.0003f);
    }
  }
  assert(corrected.valid);assert(corrected.observationVarianceM2==1.e-5f);
  float p[3]={1,2,3};
  postReleaseViconVelocityKfUpdatePiEpoch(&corrected,p,10000,10000);
  postReleaseViconVelocityKfUpdatePiEpoch(&corrected,p,500000,500000);
  assert(corrected.observationVarianceM2==1.e-5f);
  float badq[4]={0},out[3],J[3][3];assert(!markerGeometry(badq,arm,out,J));
}
static void completion(void) {
  const prResponseAxis m[2]={{.01f,12,1,1,0},{.01f,12,1,1,0}};
  const float v[2]={0},cmd[2]={0};float h[2][76]={{0}},out[2];prAutoUnwind u;
  assert(prAutoInit(&u,0,v,cmd,0,40,6));
  for(unsigned kind=0;kind<6;kind++) {
    assert(prAutoInit(&u,0,v,cmd,0,40,6));
    memset(h,0,sizeof(h));if(kind==5)for(unsigned i=0;i<76;i++)h[0][i]=20;
    for(unsigned i=0;i<100;i++) {
      prCompState state[2]={{0}};
      if(kind==0)state[0].angle=.6f; // zero speed, still tilted
      if(kind==1)state[0].rate=6;    // zero speed, still rotating
      if(kind==2)state[1].v=.03f;    // sideways motion
      assert(prAutoStep(&u,i*.002f,state,m,h,3.6f,kind!=3,(float[2]){kind==4?.6f:state[0].angle,state[1].angle},out));
      assert(u.phase!=4);
    }
  }
  memset(h,0,sizeof(h));
  assert(prAutoInit(&u,0,v,cmd,0,40,6));
  for(unsigned i=0;i<80;i++) {
    const prCompState state[2]={{0}};assert(prAutoStep(&u,i*.002f,state,m,h,3.6f,true,(float[2]){state[0].angle,state[1].angle},out));
    if(i<50)assert(u.phase!=4);
  }
  assert(u.phase==4);
  assert(!prAutoStep(NULL,0,(prCompState[2]){{0}},m,h,3.6,true,(float[2]){0,0},out));
  assert(!prAutoStep(&u,.3f,(prCompState[2]){{0}},m,h,3.6,true,(float[2]){0,0},out));
}
typedef struct {float minimum,back,velocity,angle,rate,time,T;bool done;unsigned corrective;} result;
static result simulate(float speed,float angle,float rate,unsigned axis,float factor,float extraDelay) {
  const prResponseAxis m[2]={{.013257f,11.878785f,.448143f,.923565f,-.018168f},
                           {.044451f,21.922344f,1.114356f,1.006446f,.000689f}};
  prResponseAxis plant[2]={m[0],m[1]};
  for(unsigned k=0;k<2;k++){plant[k].wn*=factor;plant[k].gain/=factor;plant[k].delay+=extraDelay;}
  const float direction[2]={cosf(axis*3.14159265359f/4),sinf(axis*3.14159265359f/4)};
  float v[2],command[2],history[2][76];prCompState state[2];
  for(unsigned k=0;k<2;k++){v[k]=direction[k]*speed;command[k]=direction[k]*angle;
    state[k]=(prCompState){v[k],direction[k]*angle,direction[k]*rate};}
  for(unsigned k=0;k<2;k++)for(unsigned j=0;j<76;j++)history[k][j]=command[k];
  prAutoUnwind u;assert(prAutoInit(&u,0,v,command,0,40,6));
  result r={.minimum=speed};float p=0,furthest=0;
  for(unsigned i=0;i<3000;i++) {
    float out[2];if(!prAutoStep(&u,i*.002f,state,m,history,3.6f,true,(float[2]){state[0].angle,state[1].angle},out))return r;
    assert(hypotf(out[0],out[1])<=40.001f);
    r.time=i*.002f;r.T=u.chosenT;r.velocity=hypotf(state[0].v,state[1].v);
    r.angle=hypotf(state[0].angle,state[1].angle);r.rate=hypotf(state[0].rate,state[1].rate);
    if(u.phase==4){r.done=true;return r;}
    if(u.phase==3 && !u.chosenT)r.corrective=1;
    for(unsigned k=0;k<2;k++) {
      unsigned index=75-(unsigned)lroundf(plant[k].delay/.002f);
      state[k]=prCompStep(state[k],history[k][index],&plant[k],.002f);
      memmove(history[k],history[k]+1,75*sizeof(float));history[k][75]=out[k];
    }
    const float along=direction[0]*state[0].v+direction[1]*state[1].v;
    p+=along*.002f;furthest=fmaxf(furthest,p);r.back=fmaxf(r.back,furthest-p);
    r.minimum=fminf(r.minimum,along);
  }
  return r;
}
int main(void) {
  geometry();completion();
  unsigned n=0,bad=0,corrective=0;float worstV=0,worstBack=0,maxTime=0,minT=10,maxT=0;
  const clock_t start=clock();
  for(unsigned axis=0;axis<8;axis++)for(unsigned vi=0;vi<3;vi++)
    for(unsigned ai=0;ai<3;ai++)for(unsigned ri=0;ri<3;ri++)for(unsigned ei=0;ei<3;ei++)for(int di=-1;di<=1;di++) {
      const float speeds[]={.2f,.4f,.8f},angles[]={-10,0,5},rates[]={-10,0,10},errors[]={.9f,1,1.1f};
      const result r=simulate(speeds[vi],angles[ai],rates[ri],axis,errors[ei],di*.006f);
      n++;if(!r.done || r.velocity>.02f || r.angle>.5f || r.rate>5 || r.back>.005f || r.minimum<-.02f)bad++;
      corrective+=r.corrective;worstV=fminf(worstV,r.minimum);worstBack=fmaxf(worstBack,r.back);maxTime=fmaxf(maxTime,r.time);
      if(r.T>0){minT=fminf(minT,r.T);maxT=fmaxf(maxT,r.T);}
    }
  const result late=simulate(.1f,10,0,0,1,0);
  printf("{\"cases\":%u,\"failed\":%u,\"corrective_only\":%u,\"min_velocity_m_s\":%.6f,\"max_flyback_m\":%.6f,\"max_duration_s\":%.3f,\"unwind_min_s\":%.3f,\"unwind_max_s\":%.3f,\"native_cpu_s\":%.3f,\"late_braking_flyback_m\":%.6f}\n",n,bad,corrective,worstV,worstBack,maxTime,minT,maxT,(double)(clock()-start)/CLOCKS_PER_SEC,late.back);
  // The late-release stress case is intentionally reported, never hidden as
  // proof of universal no-reversal. Completion must still be physically gated.
  assert(late.done && late.velocity<=.02f && late.angle<=.5f && late.rate<=5);
  return bad?2:0;
}
