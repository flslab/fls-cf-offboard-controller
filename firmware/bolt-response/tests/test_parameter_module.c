#include <assert.h>
#include <stdio.h>
#include "firmware/src/modules/src/post_release_response.c"
static bool armed;
bool supervisorIsArmed(void) {return armed;}
int main(void) {
  prResponseModel out={0};
  assert(version==26092301u && runtime==1 && !prResponseGet(&out));
  draft=(prResponseModel){.id=42,.source=1,.axis={{.04f,13,.6f,1,0},{.02f,12,.4f,1,0}}};
  commit=41;apply();assert(error==2 && !ready);
  commit=42;apply();assert(prResponseGet(&out) && out.id==42 && activeId==42);
  armed=true;draft.axis[0].wn=20;dirty();commit=0;apply();
  assert(error==1 && prResponseGet(&out) && out.axis[0].wn==13);
  armed=false;dirty();assert(!prResponseGet(&out));
  draft.axis[0].delay=NAN;commit=42;apply();assert(error==2 && !ready);
  float r[76],p[76];uint32_t start=UINT32_MAX-180000u;
  for(unsigned k=0;k<130;k++)prResponseRecord(start+2000*k,k,-(float)k,true);
  assert(prResponseHistory(start+260000u,r,p));
  assert(r[0]==55 && r[75]==129 && p[0]==-55);
  assert(!prResponseHistory(start+280000u,r,p));
  prResponseRecord(start+260000u,0,0,false);assert(!prResponseHistory(start+260000u,r,p));
  puts("PASS: commit identity, incomplete/invalid upload, armed immutability, reset, causal history and uint32 wrap");
}
