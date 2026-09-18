#include "post_release_push_plan.h"
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void u16(uint8_t* p,uint16_t n) {p[0]=n;p[1]=n>>8;}
static void u32(uint8_t* p,uint32_t n) {p[0]=n;p[1]=n>>8;p[2]=n>>16;p[3]=n>>24;}
static uint32_t r32(const uint8_t* p) {return p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void header(uint8_t* p,uint8_t op) {
  p[0]=op;p[1]=1;u16(p+2,7);u32(p+4,0x12345678);u16(p+8,9);
}
static void start(postReleasePushState_t* s,uint32_t stamp) {
  postReleasePushReset(s,0x12345678,7,9);
  const postReleasePushSnapshot_t v={.snapshotUs=stamp,.latestStartUs=stamp+200000,
    .trustedStateAgeUs=2000,.attitudeAgeUs=1000,.velocity={0,-1},
    .direction={0,-1},.attitudeGain={6,7.1,6},.integralGain={1,1,1},
    .tau={.05,.07,.08},.rapidReference={-22,0,0}};
  assert(postReleasePushCapture(s,&v));
  assert(!postReleasePushCapture(s,&v));
}
static uint32_t makePlan(uint8_t body[60]) {
  u32(body,80000);
  const float values[13]={.8,-22,0,0,0,0,0,-8,0,0,0,0,0};
  memcpy(body+4,values,sizeof(values));
  uint32_t crc=postReleasePushCrc(body,56);u32(body+56,crc);return crc;
}
static int part(postReleasePushState_t* s,const uint8_t body[60],uint8_t k) {
  uint8_t p[30];header(p,21);p[10]=k;p[11]=4;
  size_t count=k==3?6:18;memcpy(p+12,body+k*18,count);
  return postReleasePushAcceptPart(s,p,12+count);
}
static void load(postReleasePushState_t* s,const uint8_t body[60]) {
  const uint8_t order[]={3,1,1,0,2};
  for(unsigned k=0;k<sizeof(order);k++) assert(part(s,body,order[k])==0);
}
static int commit(postReleasePushState_t* s,uint32_t crc,uint32_t now,bool allowed,jointPlan_t* plan,bool* scheduled) {
  uint8_t p[14];header(p,22);u32(p+10,crc);
  return postReleasePushCommit(s,p,sizeof(p),now,allowed,plan,scheduled);
}
int main(void) {
  assert(postReleasePushCrc((const uint8_t*)"123456789",9)==0xcbf43926u);
  postReleasePushState_t state;start(&state,1000000);
  uint8_t rebuilt[120],packet[30],body[60];
  for(unsigned k=0;k<7;k++) {
    size_t n=postReleasePushFragment(&state,k,packet);
    assert(n==(k==6?24:30));assert(packet[0]==19 && packet[11]==7);
    memcpy(rebuilt+k*18,packet+12,n-12);
  }
  assert(memcmp(rebuilt,state.snapshot,120)==0);
  assert(r32(rebuilt+116)==postReleasePushCrc(rebuilt,116));
  assert(postReleasePushFragment(&state,7,packet)==0);
  header(packet,20);assert(postReleasePushAcknowledge(&state,packet,9)==EINVAL);
  packet[8]++;assert(postReleasePushAcknowledge(&state,packet,10)==ESTALE);
  packet[8]--;assert(postReleasePushAcknowledge(&state,packet,10)==0 && state.acknowledged);
  uint32_t crc=makePlan(body);
  jointPlan_t plan={0};bool scheduled;
  assert(commit(&state,crc,1001000,true,&plan,&scheduled)==EAGAIN && !scheduled);
  load(&state,body);
  assert(commit(&state,crc+1,1001000,true,&plan,&scheduled)==EBADMSG && !scheduled);
  assert(commit(&state,crc,1001000,false,&plan,&scheduled)==EBUSY && !scheduled);
  assert(commit(&state,crc,1075001,true,&plan,&scheduled)==ETIMEDOUT && !scheduled);
  assert(commit(&state,crc,1001000,true,&plan,&scheduled)==0 && scheduled);
  assert(state.acceptedStartUs==1080000 && plan.duration==.8f);
  jointPlan_t old=plan;
  assert(commit(&state,crc,1999999,false,&plan,&scheduled)==0 && !scheduled);
  assert(memcmp(&old,&plan,sizeof(plan))==0); // duplicate never restarts
  assert(commit(&state,crc+1,1001000,true,&plan,&scheduled)==EALREADY);
  float a[3],r[3],ac[3],j[3];
  assert(jointEvaluate(&plan,plan.duration,a,r,ac,j));
  assert(a[0]==0 && r[0]==0 && ac[0]==0 && j[0]==0);
  // Conflicting duplicate drops the incomplete assembly, not another release.
  start(&state,1000000);assert(part(&state,body,0)==0);body[15]^=1;
  assert(part(&state,body,0)==EINVAL && state.planMask==0);body[15]^=1;
  assert(part(&state,body,1)==0);assert(commit(&state,crc,1001000,true,&plan,&scheduled)==EAGAIN);
  load(&state,body);header(packet,21);packet[10]=3;packet[11]=4;
  memcpy(packet+12,body+54,6);assert(postReleasePushAcceptPart(&state,packet,19)==EINVAL);
  packet[4]^=1;assert(postReleasePushAcceptPart(&state,packet,18)==ESTALE);
  // Modulo32 clock wrap, precise five-ms scheduling guard, bounded horizon.
  start(&state,0xfffff000u);load(&state,body);
  assert(commit(&state,crc,0xfffff000u+75000u,true,&plan,&scheduled)==0 && scheduled);
  start(&state,1000000);u32(body,200001);crc=postReleasePushCrc(body,56);u32(body+56,crc);load(&state,body);
  assert(commit(&state,crc,1001000,true,&plan,&scheduled)==ETIMEDOUT);
  start(&state,1000000);makePlan(body);float nan=NAN;memcpy(body+44,&nan,4);
  crc=postReleasePushCrc(body,56);u32(body+56,crc);load(&state,body);
  assert(commit(&state,crc,1001000,true,&plan,&scheduled)==EINVAL);
  start(&state,1000000);makePlan(body);float bad=50;memcpy(body+32,&bad,4);
  crc=postReleasePushCrc(body,56);u32(body+56,crc);load(&state,body);
  assert(commit(&state,crc,1001000,true,&plan,&scheduled)==ERANGE);
  // A geometrically valid plan may not jump away from the rapid command or
  // invent a nonzero initial reference rate at the future transition epoch.
  start(&state,1000000);makePlan(body);float discontinuous=-20;memcpy(body+8,&discontinuous,4);
  crc=postReleasePushCrc(body,56);u32(body+56,crc);load(&state,body);
  assert(commit(&state,crc,1001000,true,&plan,&scheduled)==ERANGE && !scheduled);
  start(&state,1000000);makePlan(body);float initialRate=1;memcpy(body+20,&initialRate,4);
  crc=postReleasePushCrc(body,56);u32(body+56,crc);load(&state,body);
  assert(commit(&state,crc,1001000,true,&plan,&scheduled)==ERANGE && !scheduled);
  const float zero[3]={0},nans[3]={NAN,0,0};
  assert(!jointBuildFromParameters(zero,zero,zero,nans,.8,&plan));
  assert(!jointBuildFromParameters(zero,zero,zero,zero,.01,&plan));
  // Result echoes identity, contains errno and only accepted schedules.
  header(packet,22);uint8_t reply[16];
  assert(postReleasePushResult(packet,14,0,42,reply)==16);
  assert(reply[0]==23 && r32(reply+12)==42);
  assert(postReleasePushResult(packet,14,EINVAL,42,reply)==16 && r32(reply+12)==0);
  puts("PASS: production C protocol, CRC, identity, reorder, duplicates, deadlines, wrap, reconstruction");
  return 0;
}
