#include "post_release_auto_unwind.h"
#include <assert.h>
#include <stdio.h>
#include <time.h>

typedef struct {
  double angle,rate,delay,gain,bias,transition[4];
} plantAxis;
typedef struct {
  float minimum,back,speed,angle,rate,time,returnTime,peakCommand,peakRate,peakAcceleration;
  bool done,fault,initialFeasible,rejected;
  unsigned replans;
} result;
static FILE *traceFile;

// Independent double-precision closed-form plant, with nonlinear tilt force.
// It intentionally does not propagate the aircraft with the prediction kernel.
static plantAxis makePlant(prResponseAxis calibrated,float wnScale,float dampingScale,
    float gainScale,float delayError,float angle,float rate) {
  const double dt=.002,wn=calibrated.wn*wnScale,z=calibrated.zeta*dampingScale;
  const double d=z*wn,disc=d*d-wn*wn,e=exp(-d*dt);
  double c,b;
  if(fabs(disc)<1e-8){c=1;b=dt;}
  else if(disc>0){const double q=sqrt(disc);c=cosh(q*dt);b=sinh(q*dt)/q;}
  else {const double q=sqrt(-disc);c=cos(q*dt);b=sin(q*dt)/q;}
  return (plantAxis){.angle=angle,.rate=rate,.delay=fmax(0,calibrated.delay+delayError),
    .gain=calibrated.gain*gainScale,.bias=calibrated.bias,
    .transition={e*(c+d*b),e*b,-wn*wn*e*b,e*(c-d*b)}};
}
static void plantStep(plantAxis *p,float command) {
  const double target=p->gain*command+p->bias,error=p->angle-target;
  const double next=target+p->transition[0]*error+p->transition[1]*p->rate;
  p->rate=p->transition[2]*error+p->transition[3]*p->rate;p->angle=next;
}
static result simulate(float speed,float angle,float rate,unsigned heading,float wnScale,
    float dampingScale,float gainScale,float delayError,unsigned commandEvery) {
  const prResponseAxis m[2]={{.013257f,11.878785f,.448143f,.923565f,-.018168f},
    {.044451f,21.922344f,1.114356f,1.006446f,.000689f}};
  const float direction[2]={cosf(heading*3.14159265359f/4),sinf(heading*3.14159265359f/4)};
  float v[2],command[2],queue[2][76];plantAxis plant[2];
  for(unsigned k=0;k<2;k++) {
    v[k]=speed*direction[k];command[k]=angle*direction[k];
    plant[k]=makePlant(m[k],wnScale,dampingScale,gainScale,delayError,command[k],rate*direction[k]);
    for(unsigned j=0;j<76;j++)queue[k][j]=command[k];
  }
  prAutoUnwind u;assert(prAutoInit(&u,0,v,command,0,40,6));
  result out={.minimum=speed};float position=0,furthest=0;
  for(unsigned i=0;i<3000;i++) {
    prCompState state[2];for(unsigned k=0;k<2;k++)state[k]=(prCompState){v[k],plant[k].angle,plant[k].rate};
    if(i%commandEvery==0) {
      if(!prAutoStep(&u,i*.002f,state,m,queue,3.6f,true,(float[2]){state[0].angle,state[1].angle},command)) {
        out.initialFeasible=u.initialFeasible;
        out.rejected=u.initialChecked && !u.initialFeasible && u.recoveryReason==6;
        out.fault=!out.rejected;return out;
      }
      out.initialFeasible=u.initialFeasible;
      if(traceFile)fprintf(traceFile,"%.3f,%.6f,%.5f,%.5f,%.5f,%u,%.5f,%.5f,%.5f,%.5f,%u\n",i*.002,
        direction[0]*v[0]+direction[1]*v[1],plant[1].angle,plant[1].rate,command[1],u.phase,u.totalT,u.worstStop,u.predictedEnd,u.minimumPredictedVelocity,u.recoveryReason);
      out.peakCommand=fmaxf(out.peakCommand,hypotf(command[0],command[1]));
      assert(hypotf(command[0],command[1])<=40.01f);
      assert(hypotf(u.commandRate[0],u.commandRate[1])<=u.limits.maxRate+.02f);
      assert(hypotf(u.commandAcceleration[0],u.commandAcceleration[1])<=u.limits.maxAcceleration+.2f);
      assert(hypotf(u.commandJerk[0],u.commandJerk[1])<=u.limits.maxJerk+5.f);
    }
    out.time=i*.002f;out.returnTime=u.totalT;out.replans=u.replans;
    out.speed=hypotf(v[0],v[1]);out.angle=hypot(plant[0].angle,plant[1].angle);out.rate=hypot(plant[0].rate,plant[1].rate);
    out.peakRate=fmaxf(out.peakRate,out.rate);
    if(u.phase==4){out.done=true;return out;}
    const double oldPitch=plant[0].angle,oldRoll=plant[1].angle,oldRate=hypot(plant[0].rate,plant[1].rate);
    for(unsigned k=0;k<2;k++) {
      const unsigned age=(unsigned)lround(plant[k].delay/.002);
      assert(age<=75);plantStep(&plant[k],queue[k][75-age]);
      memmove(queue[k],queue[k]+1,75*sizeof(float));queue[k][75]=command[k];
    }
    const double pitch=.5*(oldPitch+plant[0].angle)*.0174532925199433;
    const double roll=.5*(oldRoll+plant[1].angle)*.0174532925199433;
    v[0]-=.002*9.81*tan(pitch);v[1]-=.002*9.81*tan(roll)/cos(pitch);
    out.peakAcceleration=fmaxf(out.peakAcceleration,fabs(hypot(plant[0].rate,plant[1].rate)-oldRate)/.002);
    const float along=direction[0]*v[0]+direction[1]*v[1];
    position+=along*.002f;furthest=fmaxf(furthest,position);out.back=fmaxf(out.back,furthest-position);out.minimum=fminf(out.minimum,along);
  }
  return out;
}
static void boundaryTests(void) {
  const float v[2]={.8f,0},command[2]={10,0},neutral[2]={.7f,-.4f};
  prAutoUnwind u;assert(prAutoInit(&u,0,v,command,0,40,6));
  u.commandRate[0]=5;u.commandAcceleration[0]=-20;u.commandJerk[0]=30;
  scaPoly p[2];assert(prAutoPlan(&u,neutral,.8f,p));
  const scaRef start=scaEval(p,0),end=scaEval(p,.8f);
  assert(fabsf(start.x-10)<1e-5 && fabsf(start.w-5)<1e-5 && fabsf(start.a+20)<1e-4 && fabsf(start.j-30)<1e-3);
  assert(end.x==neutral[0] && end.w==0 && end.a==0 && end.j==0);
  prAutoLimits bad=prAutoDefaultLimits();bad.maxJerk=NAN;assert(!prAutoConfigure(&u,&bad));
  const prResponseAxis m[2]={{.01,12,1,1,0},{.01,12,1,1,0}};float q[2][76]={{0}},out[2];
  for(unsigned test=0;test<4;test++) {
    assert(prAutoInit(&u,0,(float[2]){0,0},(float[2]){0,0},0,40,6));
    for(unsigned i=0;i<100;i++) {
      prCompState s[2]={{0}};if(test==0)s[0].rate=6;if(test==1)s[1].v=.03;
      const bool ok=prAutoStep(&u,i*.002,s,m,q,3.6,test!=2,(float[2]){test==3?.6f:0,0},out);
      if(!ok){assert(u.recoveryReason==6 && !u.initialFeasible);break;}
      assert(u.phase!=4);
    }
  }
}
int main(int argc,char **argv) {
  if(argc==2 && strcmp(argv[1],"probe")==0) {
    const prResponseAxis m[2]={{.013257f,11.878785f,.448143f,.923565f,-.018168f},
      {.044451f,21.922344f,1.114356f,1.006446f,.000689f}};
    float value[164];
    while(scanf("%f",value)==1) {
      for(unsigned i=1;i<164;i++)assert(scanf("%f",value+i)==1);
      prCompState s[2]={{value[0],value[2],value[4]},{value[1],value[3],value[5]}};
      prAutoUnwind u;float out[2],q[2][76];
      memcpy(q,value+12,sizeof(q));
      assert(prAutoInit(&u,0,value,value+6,0,40,6));
      u.direction[0]=value[8];u.direction[1]=value[9];
      const bool ok=prAutoStep(&u,0,s,m,q,3.6f,true,value+10,out);
      printf("%d %u %.6f\n",ok,u.recoveryReason,u.minimumPredictedVelocity);
    }
    return 0;
  }
  if(argc==3){traceFile=fopen(argv[2],"w");assert(traceFile);fprintf(traceFile,"t,v,roll,rate,command,phase,T,worst_stop,end,min,reason\n");
    const result r=strcmp(argv[1],"bad")==0?simulate(.6,5,-20,1,.65,1.15,1.1,.02,5):simulate(.6,-5,0,2,1,.8,1.1,-.02,5);fclose(traceFile);printf("min=%.6f back=%.6f done=%d rejected=%d\n",r.minimum,r.back,r.done,r.rejected);return 0;}
  boundaryTests();unsigned cases=0,failed=0,faults=0,admitted=0,admittedFailed=0,rejected=0;
  float minV=0,maxBack=0,maxTime=0,minReturn=10,maxReturn=0,maxCommand=0,maxRate=0,maxAcceleration=0;
  const clock_t start=clock();
  const float speeds[]={.25f,.6f,1.f},angles[]={-5,0,5},rates[]={-20,0,20};
  const float wn[]={.65f,.8f,1.f,1.1f},damping[]={.8f,1.f,1.15f},gain[]={.9f,1.f,1.1f};
  const unsigned ns=argc>1?4:1;
  for(unsigned h=0;h<8;h++)for(unsigned vi=0;vi<3;vi++)for(unsigned ai=0;ai<3;ai++)for(unsigned ri=0;ri<3;ri++)
    for(unsigned wi=0;wi<ns;wi++)for(unsigned zi=0;zi<(argc>1?3:1);zi++)for(unsigned gi=0;gi<(argc>1?3:1);gi++)for(int di=-1;di<=1;di++) {
      if(argc>1 && argv[1][0]=='a' && (vi!=0 || ai!=2 || ri!=2))continue;
      const result r=simulate(speeds[vi],angles[ai],rates[ri],h,argc>1?wn[wi]:1,
        argc>1?damping[zi]:1,argc>1?gain[gi]:1,di*.02f,5);
      cases++;faults+=r.fault;admitted+=r.initialFeasible;rejected+=r.rejected;
      if(r.rejected)continue;
      if(!r.done || r.speed>.02 || r.angle>.5 || r.rate>5 || r.back>.005 || r.minimum<-.02) {
        failed++;admittedFailed+=r.initialFeasible;
        fprintf(stderr,"case=%u h=%u v=%.2f a=%.1f r=%.1f wn=%.2f z=%.2f gain=%.2f delay=%.3f done=%d fault=%d admitted=%d min=%.4f back=%.4f t=%.3f\n",
          cases,h,speeds[vi],angles[ai],rates[ri],argc>1?wn[wi]:1,argc>1?damping[zi]:1,argc>1?gain[gi]:1,di*.02f,r.done,r.fault,r.initialFeasible,r.minimum,r.back,r.time);
      }
      minV=fminf(minV,r.minimum);maxBack=fmaxf(maxBack,r.back);maxTime=fmaxf(maxTime,r.time);
      if(r.returnTime>0){minReturn=fminf(minReturn,r.returnTime);maxReturn=fmaxf(maxReturn,r.returnTime);}
      maxCommand=fmaxf(maxCommand,r.peakCommand);maxRate=fmaxf(maxRate,r.peakRate);maxAcceleration=fmaxf(maxAcceleration,r.peakAcceleration);
    }
  const result late=simulate(.1,10,0,0,1,1,1,0,5);
  printf("{\"cases\":%u,\"failed\":%u,\"faults\":%u,\"initially_admitted\":%u,\"rejected_before_new_commands\":%u,\"admitted_failed\":%u,\"minimum_velocity_m_s\":%.6f,\"maximum_flyback_m\":%.6f,\"maximum_duration_s\":%.3f,\"return_min_s\":%.3f,\"return_max_s\":%.3f,\"peak_command_deg\":%.3f,\"peak_physical_rate_deg_s\":%.3f,\"peak_physical_acceleration_deg_s2\":%.3f,\"late_release_rejected\":%s,\"cpu_s\":%.3f}\n",
    cases,failed,faults,admitted,rejected,admittedFailed,minV,maxBack,maxTime,minReturn,maxReturn,maxCommand,maxRate,maxAcceleration,late.rejected?"true":"false",(double)(clock()-start)/CLOCKS_PER_SEC);
  return failed?2:0;
}
