#include "post_release_push_plan.h"
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void u16(uint8_t* p,uint16_t n) {p[0]=n;p[1]=n>>8;}
static void u32(uint8_t* p,uint32_t n) {p[0]=n;p[1]=n>>8;p[2]=n>>16;p[3]=n>>24;}
static uint32_t r32(const uint8_t* p) {return p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void header(uint8_t* p,uint8_t op,uint16_t token) {
  p[0]=op;p[1]=PostReleasePushVersion;u16(p+2,7);u32(p+4,0x12345678);u16(p+8,token);
}
static void startPrefixMagnitude(postReleasePushState_t* s,uint32_t stamp,uint16_t token,
    float elapsed,float duration,float initialAngle) {
  postReleasePushReset(s,0x12345678,7,token);
  const postReleasePushSnapshot_t v={.snapshotUs=stamp,.latestStartUs=stamp+200000,
    .trustedStateAgeUs=2000,.attitudeAgeUs=1000,.velocity={0,-1},
    .direction={0,-1},.attitudeGain={6,7.1,6},.integralGain={1,1,1},
    .tau={.05,.07,.08},.rapidReference={initialAngle,0,0},
    .prefixElapsedS=elapsed,.prefixDurationS=duration};
  assert(postReleasePushCapture(s,&v));
  assert(!postReleasePushCapture(s,&v));
}
static void startPrefix(postReleasePushState_t* s,uint32_t stamp,uint16_t token,
    float elapsed,float duration) {
  startPrefixMagnitude(s,stamp,token,elapsed,duration,-12);
}
static void start(postReleasePushState_t* s,uint32_t stamp) {
  startPrefix(s,stamp,9,0,0);
}
static void seal(uint8_t body[PostReleasePlanWireSize]) {
  u32(body+PostReleasePlanBodySize,postReleasePushCrc(body,PostReleasePlanBodySize));
}
static void makePlan(uint8_t body[PostReleasePlanWireSize],float elapsed,float duration) {
  u32(body,80000);
  float values[13]={.8,-12,0,0,0,0,0,-5,0,0,0,0,0};
  if(duration>0) {
    const float ref[3]={-12,0,0},zero[3]={0};
    float ac[3],j[3];postReleaseUnwindProfile_t prefix;
    assert(postReleaseUnwindPlan(ref,zero,zero,zero,duration,&prefix));
    assert(postReleaseUnwindEvaluate(&prefix,elapsed+.08f,values+1,values+4,ac,j));
    values[7]=.5f*values[1];
  }
  memcpy(body+4,values,4);memcpy(body+8,values+7,24);seal(body);
}
static size_t partPacket(uint8_t p[30],const uint8_t body[PostReleasePlanWireSize],
    uint8_t k,uint16_t token) {
  header(p,PostReleasePlanPart,token);p[10]=k;p[11]=PostReleasePlanPartCount;
  size_t count=PostReleasePlanWireSize-k*PostReleasePushChunkSize;
  if(count>PostReleasePushChunkSize) count=PostReleasePushChunkSize;
  memcpy(p+PostReleasePushHeaderSize,body+k*PostReleasePushChunkSize,count);
  return PostReleasePushHeaderSize+count;
}
static int receive(postReleasePushState_t* s,const uint8_t body[PostReleasePlanWireSize],
    uint8_t k,uint32_t now,bool allowed,jointPlan_t* plan,bool* scheduled,bool* complete) {
  uint8_t p[30];size_t n=partPacket(p,body,k,s->token);
  return postReleasePushReceivePart(s,p,n,now,allowed,plan,scheduled,complete);
}
static int load(postReleasePushState_t* s,const uint8_t body[PostReleasePlanWireSize],
    uint32_t now,bool allowed,jointPlan_t* plan,bool* scheduled) {
  const uint8_t order[]={1,1,0};bool complete;
  for(unsigned k=0;k<sizeof(order)-1;k++) {
    assert(receive(s,body,order[k],now,allowed,plan,scheduled,&complete)==0);
    assert(!*scheduled && !complete);
  }
  int result=receive(s,body,order[sizeof(order)-1],now,allowed,plan,scheduled,&complete);
  assert(complete);return result;
}
int main(void) {
  assert(PostReleasePushVersion==3);
  assert(postReleasePushCrc((const uint8_t*)"123456789",9)==0xcbf43926u);
  postReleasePushState_t state;start(&state,1000000);
  uint8_t rebuilt[PostReleaseSnapshotWireSize],packet[30],body[PostReleasePlanWireSize];
  for(unsigned k=0;k<PostReleaseSnapshotPartCount;k++) {
    size_t n=postReleasePushFragment(&state,k,packet);
    assert(n==(k==5?18:30));assert(packet[0]==19 && packet[11]==6 && packet[1]==3);
    memcpy(rebuilt+k*18,packet+12,n-12);
  }
  assert(memcmp(rebuilt,state.snapshot,12)==0);
  assert(memcmp(rebuilt+16,state.snapshot+12,56)==0);
  assert(memcmp(rebuilt+72,state.snapshot+104,20)==0);
  assert(r32(rebuilt+12)==postReleasePushCrc(state.snapshot+68,36));
  assert(r32(rebuilt+PostReleaseSnapshotWireBodySize)==postReleasePushCrc(rebuilt,PostReleaseSnapshotWireBodySize));
  assert(postReleasePushFragment(&state,PostReleaseSnapshotPartCount,packet)==0);
  makePlan(body,0,0);
  jointPlan_t plan={0};bool scheduled,complete;
  // Last fragment commits once: neither an ACK nor a COMMIT is sent.
  assert(load(&state,body,1001000,true,&plan,&scheduled)==0 && scheduled);
  assert(state.committed && state.acknowledged && state.acceptedStartUs==1080000);
  assert(state.acceptedReceiveUs==1001000 && plan.duration==.8f);
  jointPlan_t old=plan;
  assert(receive(&state,body,0,1999999,false,&plan,&scheduled,&complete)==0);
  assert(complete && !scheduled && memcmp(&old,&plan,sizeof(plan))==0);
  assert(state.acceptedReceiveUs==1001000);
  assert(!postReleasePushMarkExecuting(&state,1079999));
  assert(postReleasePushMarkExecuting(&state,1080123));
  assert(!postReleasePushMarkExecuting(&state,1080999));
  assert(state.executedUs==1080123);
  float a[3],r[3],ac[3],j[3];
  assert(jointEvaluate(&plan,plan.duration,a,r,ac,j));
  assert(a[0]==0 && r[0]==0 && ac[0]==0 && j[0]==0);

  // Conflicting duplicates cannot silently reset/remix an assembly.
  start(&state,1000000);
  assert(receive(&state,body,0,1001000,true,&plan,&scheduled,&complete)==0 && !complete);
  assert(state.acknowledged && !state.committed);
  body[15]^=1;
  assert(receive(&state,body,0,1001000,true,&plan,&scheduled,&complete)==EINVAL);
  body[15]^=1;
  assert(receive(&state,body,1,1001000,true,&plan,&scheduled,&complete)==EBADMSG);
  assert(state.invalidPlan && !state.committed);
  start(&state,1000000);body[15]^=1;
  assert(load(&state,body,1001000,true,&plan,&scheduled)==EBADMSG && !scheduled);
  body[15]^=1;
  assert(receive(&state,body,1,1001000,true,&plan,&scheduled,&complete)==EBADMSG);

  // Old versions/releases/tokens neither authorize nor poison a new token.
  startPrefix(&state,1000000,10,0,0);
  size_t n=partPacket(packet,body,0,9);
  assert(postReleasePushReceivePart(&state,packet,n,1001000,true,&plan,&scheduled,&complete)==ESTALE);
  packet[8]=10;packet[1]=1;
  assert(postReleasePushReceivePart(&state,packet,n,1001000,true,&plan,&scheduled,&complete)==ESTALE);
  packet[1]=3;packet[4]^=1;
  assert(postReleasePushReceivePart(&state,packet,n,1001000,true,&plan,&scheduled,&complete)==ESTALE);
  assert(!state.invalidPlan && state.planMask==0);
  assert(load(&state,body,1001000,true,&plan,&scheduled)==0 && scheduled);
  start(&state,1000000);n=partPacket(packet,body,1,9);
  assert(postReleasePushReceivePart(&state,packet,n+1,1001000,true,&plan,&scheduled,&complete)==EINVAL);
  assert(postReleasePushReceivePart(&state,packet,9,1001000,true,&plan,&scheduled,&complete)==EINVAL);
  assert(!state.committed && state.planMask==0);

  // Wrap-safe absolute epochs and precise existing five-ms scheduling guard.
  start(&state,0xfffff000u);
  assert(load(&state,body,0xfffff000u+75000u,true,&plan,&scheduled)==0 && scheduled);
  assert(postReleasePushMarkExecuting(&state,0xfffff000u+80000u));
  start(&state,1000000);
  assert(load(&state,body,1075001,true,&plan,&scheduled)==ETIMEDOUT && !scheduled);
  start(&state,1000000);
  assert(load(&state,body,1001000,false,&plan,&scheduled)==EBUSY && !scheduled);
  start(&state,1000000);u32(body,200001);seal(body);
  assert(load(&state,body,1001000,true,&plan,&scheduled)==ETIMEDOUT);
  start(&state,1000000);makePlan(body,0,0);float nan=NAN;memcpy(body+24,&nan,4);seal(body);
  assert(load(&state,body,1001000,true,&plan,&scheduled)==EINVAL);
  start(&state,1000000);makePlan(body,0,0);float bad=50;memcpy(body+8,&bad,4);seal(body);
  assert(load(&state,body,1001000,true,&plan,&scheduled)==ERANGE);
  start(&state,1000000);makePlan(body,0,0);float badEnd=-10;memcpy(body+20,&badEnd,4);seal(body);
  assert(load(&state,body,1001000,true,&plan,&scheduled)==ERANGE && !scheduled);
  start(&state,1000000);makePlan(body,0,0);float badDuration=.01f;memcpy(body+4,&badDuration,4);seal(body);
  assert(load(&state,body,1001000,true,&plan,&scheduled)==ERANGE && !scheduled);

  // All four derivatives join a running unwind; a=j=0 would be wrong.
  startPrefix(&state,1000000,11,.05f,.6f);makePlan(body,.05f,.6f);
  assert(load(&state,body,1001000,true,&plan,&scheduled)==0 && scheduled);
  const float prefixRef[3]={-12,0,0},zero[3]={0};
  postReleaseUnwindProfile_t prefix;
  assert(postReleaseUnwindPlan(prefixRef,zero,zero,zero,.6f,&prefix));
  float pa[3],pr[3],pac[3],pj[3];
  assert(postReleaseUnwindEvaluate(&prefix,.13f,pa,pr,pac,pj));
  assert(jointEvaluate(&plan,0,a,r,ac,j));
  for(int k=0;k<3;k++) {
    assert(fabsf(a[k]-pa[k])<.002f && fabsf(r[k]-pr[k])<.02f);
    assert(fabsf(ac[k]-pac[k])<.02f && fabsf(j[k]-pj[k])<.2f);
  }
  // A 22-degree return in .20 s is geometrically level but its 240 deg/s
  // reference is outside the unchanged joint-plan 180 deg/s limit. A .31 s
  // local prefix remains bounded and admits a C3 Pi splice without relaxing it.
  const float realisticRef[3]={-22,0,0};
  postReleaseUnwindProfile_t fastPrefix,realisticPrefix;
  assert(postReleaseUnwindPlan(realisticRef,zero,zero,zero,.20f,&fastPrefix));
  assert(postReleaseUnwindEvaluate(&fastPrefix,.10f,pa,pr,pac,pj));
  assert(fabsf(pr[0])>180);
  assert(postReleaseUnwindPlan(realisticRef,zero,zero,zero,.31f,&realisticPrefix));
  assert(postReleaseUnwindEvaluate(&realisticPrefix,.155f,pa,pr,pac,pj));
  assert(fabsf(pr[0])<160);
  assert(postReleaseUnwindEvaluate(&realisticPrefix,.13f,pa,pr,pac,pj));
  const float horizons[]={.45f,.60f,.80f,1.f,1.2f,1.4f,1.6f};
  bool found=false;
  for(unsigned h=0;h<sizeof(horizons)/sizeof(horizons[0]) && !found;h++) {
    for(int knotIndex=-24;knotIndex<=24 && !found;knotIndex++) {
      float knot[3]={knotIndex,0,0};
      if(!jointBuildFromBoundaryParameters(pa,pr,pac,pj,knot,zero,horizons[h],&plan)) continue;
      float values[13]={horizons[h]};
      memcpy(values+1,pa,3*sizeof(float));memcpy(values+4,pr,3*sizeof(float));
      memcpy(values+7,knot,3*sizeof(float));
      u32(body,80000);memcpy(body+4,values,4);memcpy(body+8,values+7,24);seal(body);
      startPrefixMagnitude(&state,1000000,20,.05f,.31f,-22);
      assert(load(&state,body,1001000,true,&plan,&scheduled)==0 && scheduled);
      found=true;
    }
  }
  assert(found);
  assert(jointEvaluate(&plan,0,a,r,ac,j));
  for(int k=0;k<3;k++) {
    assert(fabsf(a[k]-pa[k])<.002f && fabsf(r[k]-pr[k])<.02f);
    assert(fabsf(ac[k]-pac[k])<.02f && fabsf(j[k]-pj[k])<.2f);
  }
  // Newer fallback token invalidates an accepted rapid plan as well.
  startPrefix(&state,1100000,12,.15f,.6f);
  n=partPacket(packet,body,0,11);
  assert(postReleasePushReceivePart(&state,packet,n,1101000,true,&plan,&scheduled,&complete)==ESTALE);
  assert(!state.committed && !state.hasExecuted);
  const float nans[3]={NAN,0,0};
  assert(!jointBuildFromParameters(zero,zero,zero,nans,.8,&plan));
  assert(!jointBuildFromBoundaryParameters(zero,zero,nans,zero,zero,zero,.8,&plan));
  assert(!jointBuildFromParameters(zero,zero,zero,zero,.01,&plan));

  // Reception and actual execution timestamps share the same FC clock.
  header(packet,21,12);uint8_t reply[PostReleasePlanResultSize];
  assert(postReleasePushResult(packet,14,0,42,30,0,reply)==24);
  assert(reply[0]==23 && reply[1]==3 && r32(reply+12)==42 && r32(reply+16)==30 && r32(reply+20)==0);
  assert(postReleasePushResult(packet,14,0,42,30,45,reply)==24 && r32(reply+20)==45);
  assert(postReleasePushResult(packet,14,EINVAL,42,30,45,reply)==24 && r32(reply+12)==0 && r32(reply+16)==0 && r32(reply+20)==0);
  puts("PASS: v3 compact automatic C protocol, C3 unwind splice, CRC/conflicts, stale generations, reorder, no ACK/commit, wrap, receive/execution timing");
  return 0;
}
