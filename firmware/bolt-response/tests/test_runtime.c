#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <setjmp.h>
#include "post_release_response.h"
#include "post_release_calibrated_job.h"
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
typedef struct {float x,y,z;} vec;
typedef struct {float roll,pitch,yaw;} attitude;
typedef struct {attitude attitude;} state_t;
enum {modeAbs=1};
typedef struct {struct {int z,roll,pitch,yaw;} mode;vec position;attitude attitude;} setpoint_t;
struct traj_eval {vec pos,vel,acc,omega;float yaw;};
static vec mkvec(float x,float y,float z){return (vec){x,y,z};}
static vec vzero(void){return (vec){0};}
static float radians(float x){return x*.01745329252f;}
static float degrees(float x){return x*57.29577951f;}
static uint32_t now=1000000;
static uint32_t workerCost,clockCalls;
static uint32_t usecTimestamp(void){return now+((clockCalls++%2)?workerCost:0);}
static float vxy[2],posxy[2],rollHistory[76],pitchHistory[76];
static bool historyOk=true;
static unsigned quality=2;
bool prResponseHistory(uint32_t us,float r[76],float p[76]){(void)us;memcpy(r,rollHistory,sizeof(rollHistory));memcpy(p,pitchHistory,sizeof(pitchHistory));return historyOk;}
static unsigned estimatorKalmanGetSCurvePrediction(float p[2],float v[2],float a[2],uint32_t *age){memcpy(p,posxy,sizeof(posxy));memcpy(v,vxy,sizeof(vxy));a[0]=a[1]=0;*age=0;return quality;}
static void postReleaseEulerRatesFromBody(float r,float p,const float b[3],float e[3]){(void)r;(void)p;memcpy(e,b,3*sizeof(float));}
static int planner,notices;
static int plan_go_to_from(int *p,const struct traj_eval *h,bool a,bool b,vec v,float yaw,float duration,float t){(void)p;(void)a;(void)b;(void)yaw;(void)t;assert(duration==0);assert(h->vel.x==0 && h->vel.y==0);assert(v.x==posxy[0] && v.y==posxy[1]);return 0;}
static void positionControllerResetXY(float x,float y){(void)x;(void)y;}
static void postReleaseAutoQueueHoldNotice(const struct traj_eval *h,uint32_t us){(void)h;(void)us;notices++;}
static float postReleaseAutoDirectionXY[2]={0,1},postReleaseAutoYawRad,postReleaseAutoHeightM=1;
static float postReleaseAutoReferenceDeg[3],scPredV,scAlong,scCross;
static uint32_t scSourceAge,scBeginUs,scLastUs,scMaxUs,postReleaseAutoReplanCount,postReleaseAutoRapidSetpointCount,postReleaseAutoHoldCount;
static unsigned scQuality,postReleaseAutoStage=1,postReleaseAutoAbortReason;
static bool scActive;
static struct {float total;} scProfile;
static jmp_buf taskYield;
#define STATIC_MEM_TASK_ALLOC(name, size)
static unsigned uxTaskGetStackHighWaterMark(void *p){(void)p;return 512;}
static void vTaskDelay(unsigned ticks){assert(ticks==1);longjmp(taskYield,1);}
#include "firmware/src/modules/src/post_release_calibrated_worker.inc"
#include "firmware/src/modules/src/post_release_calibrated_runtime.inc"
static void workerOnce(void){clockCalls=0;if(!setjmp(taskYield))scrWorker(NULL);}

static float run(float delay,float rate,bool missing) {
  scrModel=(prResponseModel){.id=42,.source=1,.axis={{delay,15,.78f,.97f,0},{delay,12,.6f,.9f,0}}};
  scActive=false;postReleaseAutoStage=1;quality=2;historyOk=!missing;
  scrPending=scrReady=scrBusy=false;notices=0;now+=10000000;
  memset(rollHistory,0,sizeof(rollHistory));memset(pitchHistory,0,sizeof(pitchHistory));
  posxy[0]=posxy[1]=0;vxy[0]=0;vxy[1]=.55f;
  scaState plant={.v=.55f,.w=rate};float checksum=0;
  for(unsigned k=0;k<650;k++) {
    state_t state={.attitude={degrees(-plant.x),0,0}};
    float observed[3]={state.attitude.roll,0,0},body[3]={degrees(-plant.w),0,0};
    float position[3]={posxy[0],posxy[1],1},velocity[3]={0,plant.v,0};setpoint_t command;
    bool active=scCalibratedGetSetpoint(&command,&state,0,now,true,position,velocity,observed,body);
    if(missing){assert(postReleaseAutoStage==6 && command.attitude.roll==0 && !scrPending);return 0;}
    if(!active){assert(postReleaseAutoStage==4 && notices==1);printf("handoff delay=%g rate=%g t=%g v=%g tilt=%g replans=%u\n",delay,rate,k*.01,plant.v,degrees(plant.x),postReleaseAutoReplanCount);return checksum;}
    if(scrPending)workerOnce();
    scaModel m={.closed=true,.delay=delay,.wn=15,.zeta=.78f,.cgain=.97f};
    for(unsigned j=0;j<76;j++)m.history[j]=-radians(rollHistory[j]);
    scaPoly p;scaMake((scaRef){.x=-radians(command.attitude.roll)*.97f},-radians(command.attitude.roll)*.97f,.01f,0,&p);
    scaPrediction out=scaClosedRollout(&p,plant,&m,0);
    plant=(scaState){.v=out.v,.x=out.x,.w=out.w};
    memmove(rollHistory,rollHistory+5,71*sizeof(float));for(unsigned j=71;j<76;j++)rollHistory[j]=command.attitude.roll;
    posxy[1]+=plant.v*.01f;vxy[1]=plant.v;now+=10000;checksum+=command.attitude.roll*(k+1);
  }
  fprintf(stderr,"no handoff: stage=%u reason=%u v=%g tilt=%g\n",postReleaseAutoStage,postReleaseAutoAbortReason,plant.v,degrees(plant.x));assert(0);return 0;
}
int main(void) {
  assert(scAdaptiveVersion==26092303u && !scAdaptiveEnable && !scAdaptiveRunning);
  float a=run(.04f,.4f,false),b=run(.08f,.4f,false),c=run(.04f,-.4f,false);
  assert(fabsf(a-b)>1 && fabsf(a-c)>1);run(.04f,0,true);
  scrMailbox.stamp=now;scrPending=scrBusy=true;workerCost=200000;unsigned late=scrLate;
  workerOnce();assert(scrLate==late+1 && scrReady && !scrReply.valid && !scrBusy);
  assert(scrWorkerMaxUs==200000 && scrStackFree==512);workerCost=0;
  puts("PASS production runtime: calibrated commands, delay/rate sensitivity, repeated release, timed position handoff, no missing-history fallback");
  puts("PASS production worker: deadline rejection, nonblocking publication, timing and stack telemetry");
}
