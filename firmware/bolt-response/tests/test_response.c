#include <assert.h>
#include <stdio.h>
#include "post_release_adaptive_scurve.h"
#include "post_release_response.h"
int main(void) {
  prResponseModel config={.id=1,.source=1,.axis={{.04f,13,.6f,1,.1f},{.04f,13,.6f,1,.1f}}};
  assert(prResponseValid(&config));config.axis[0].delay=NAN;assert(!prResponseValid(&config));
  scaModel m={.closed=true,.delay=.06f,.wn=13,.zeta=.6f,.cgain=1};
  scaPoly p;scaMake((scaRef){0},0,.001f,0,&p);
  for(unsigned k=0;k<76;k++)m.history[k]=.2f;
  scaPrediction withHistory=scaRollout(&p,(scaState){0},&m,.05f);
  assert(withHistory.x>.001f); // prior contact command still in flight
  memset(m.history,0,sizeof(m.history));
  scaPrediction withoutHistory=scaRollout(&p,(scaState){0},&m,.05f);
  assert(fabsf(withoutHistory.x)<1e-8);
  scaPrediction rate=scaRollout(&p,(scaState){.w=.87f},&m,.05f);
  assert(rate.x>.001f); // real release rate is not reset
  assert(scaFeedforward((scaRef){.w=1,.a=2},(scaState){0},&m)==0);
  m.cbias=.01f;m.cgain=.9f;
  assert(fabsf(m.cgain*scaClosedCommand(.1f,&m)+m.cbias-.1f)<1e-7);
  for(float zeta=.21f;zeta<2;zeta+=.1f) {
    m.zeta=zeta;scaPrediction o=scaRollout(&p,(scaState){.x=.1f,.w=.87f},&m,.2f);
    assert(isfinite(o.v) && isfinite(o.x) && isfinite(o.w));
  }
  puts("PASS: model validity, release rate, queued command delay, bias/gain, no double PID, stable overdamped/underdamped prediction");
}
