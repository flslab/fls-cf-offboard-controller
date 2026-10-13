"""Compile the actual publisher/getter with a deterministic preemption probe.

Usage: python validate_feedback.py /path/to/patched/firmware
No device access. Uses the firmware's own snapshot and diagnostic headers.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile


def function(source, signature):
    start = source.index(signature)
    return source[start:source.index('\n}', start) + 2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('firmware', type=Path)
    parser.add_argument('--expect-preemption-failure', action='store_true',
                        help='Verify that the unpatched source fails at the forced preemption')
    args = parser.parse_args()
    root = args.firmware.resolve()
    source = (root / 'src/modules/src/estimator/estimator_kalman.c').read_text()
    type_end = source.index('} postReleaseShadowLogSnapshot_t;')
    type_start = source.rfind('typedef struct {', 0, type_end)
    snapshot_type = source[type_start:type_end + len('} postReleaseShadowLogSnapshot_t;')]
    getter = function(source, 'bool estimatorKalmanGetHardwareAttitudeDiagnostic(')
    publisher = function(source, 'static void postReleaseShadowPublishLog(void)')
    program = r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "post_release_attitude_snapshot.h"
#include "post_release_fault_snapshot.h"
''' + snapshot_type + r'''
static postReleaseShadowLogSnapshot_t postReleaseShadowLogSource;
static postReleaseShadowLogSnapshot_t postReleaseLastGoodAttitude;
static postReleaseShadowLogSnapshot_t postReleaseShadow;
#define lastTimestampUs timestampUs
#define releaseTimestampUs seedTimestampUs
#define viconFusionTimestampUs fusedViconUs
static struct {float x,y,z;} postReleaseShadowPairedGyroDegPerS;
static postReleaseImuFailure_t postReleaseImuFailure;
static uint8_t postReleaseShadowEnable=1, postReleaseShadowActive=1;
static uint8_t postReleaseShadowArmPending, postReleaseShadowValid=1;
static uint8_t postReleaseShadowReason, postReleaseHardwareAttitudeReady;
static uint8_t postReleaseHardwareAttitudeReadyReason, postReleaseAttitudeHeld;
static uint32_t postReleaseShadowGyroAgeUs, postReleaseShadowAccelAgeUs;
static uint32_t postReleaseShadowPairSkewUs, postReleaseShadowPairCount=2;
static uint32_t postReleaseShadowUnmatchedImuCount, postReleaseShadowPairTimestampUs;
static volatile uint32_t postReleaseShadowLogGeneration;
static uint32_t postReleaseAttitudeHoldCount;
static uint32_t nowUs;
static int criticalDepth, injectPreemption, deferredRead, successfulReads;
static uint32_t usecTimestamp(void) {return nowUs;}
static void probeRead(void);
static void enterCritical(void) {criticalDepth++;}
static void exitCritical(void) {
  assert(criticalDepth > 0);
  if (--criticalDepth == 0 && deferredRead) {
    deferredRead=0; probeRead();
  }
}
#define taskENTER_CRITICAL() enterCritical()
#define taskEXIT_CRITICAL() exitCritical()
static uint32_t addGeneration(volatile uint32_t* p, uint32_t n, int order) {
  (void)order;
  *p += n;
  if (injectPreemption && (*p & 1u)) {
    if (criticalDepth) deferredRead=1;
    else probeRead();
  }
  return *p;
}
#define __atomic_add_fetch addGeneration
''' + getter + '\n' + publisher + r'''
static void probeRead(void) {
  float q[4], rates[3];
  postReleaseAttitudeDiagnostic_t d;
  assert(estimatorKalmanGetHardwareAttitudeDiagnostic(q,rates,NULL,&d));
  assert(d.reason == 0 && q[0] == 1.0f && rates[0] == 7.0f);
  successfulReads++;
}
static void healthy(uint32_t seed, uint32_t stamp) {
  postReleaseShadow=(postReleaseShadowLogSnapshot_t){0};
  postReleaseShadow.seedTimestampUs=seed;
  postReleaseShadow.timestampUs=stamp;
  postReleaseShadow.quaternionWxyz[0]=1.0f;
  postReleaseShadow.positionUpdateCount=2;
  postReleaseShadowPairedGyroDegPerS.x=7.0f;
  postReleaseShadowPairTimestampUs=stamp;
  postReleaseShadowEnable=postReleaseShadowActive=postReleaseShadowValid=1;
  nowUs=stamp;
  postReleaseShadowPublishLog();
}
static bool readFeedback(postReleaseAttitudeDiagnostic_t* d, uint32_t* stamp) {
  float q[4], rates[3];
  bool ok=estimatorKalmanGetHardwareAttitudeDiagnostic(q,rates,stamp,d);
  if(ok) assert(q[0]==1.0f && rates[0]==7.0f);
  assert(criticalDepth==0);
  return ok;
}
int main(void) {
  postReleaseAttitudeDiagnostic_t d;
  uint32_t stamp=0;
  // Force a high-priority reader exactly at the old odd-generation window.
  injectPreemption=1;
  healthy(1000,100000);
  assert(successfulReads==1 && deferredRead==0);
  injectPreemption=0;
  assert(readFeedback(&d,&stamp) && stamp==100000 && !(d.flags&32));
  // Invalid publication: preserve the valid sample, never refresh its age.
  nowUs=105000; postReleaseShadowValid=0;
  postReleaseShadow.timestampUs=105000;
  postReleaseShadowPublishLog();
  assert(readFeedback(&d,&stamp) && stamp==100000 && (d.flags&32));
  assert(postReleaseAttitudeHeld && postReleaseAttitudeHoldCount==1);
  nowUs=120000;
  assert(readFeedback(&d,&stamp) && stamp==100000 && d.stateAgeUs==20000);
  nowUs=120001;
  assert(!readFeedback(&d,&stamp) && d.reason==3);
  assert(!postReleaseAttitudeHeld);
  // Recovery switches immediately to the new acquisition time.
  healthy(1000,121000);
  assert(readFeedback(&d,&stamp) && stamp==121000 && !(d.flags&32));
  // New epoch, inactive and disabled states must never inherit old feedback.
  postReleaseShadow.seedTimestampUs=120000; postReleaseShadowValid=0;
  postReleaseShadowPublishLog();
  assert(!readFeedback(&d,&stamp) && d.reason==2);
  healthy(1000,130000); postReleaseShadowActive=0;
  postReleaseShadowPublishLog();
  assert(!readFeedback(&d,&stamp) && d.reason==1);
  healthy(1000,140000); postReleaseShadowEnable=0;
  postReleaseShadowPublishLog();
  assert(!readFeedback(&d,&stamp) && d.reason==1);
  // Startup warmup and missing position observations are still enforced.
  healthy(150000,150001);
  assert(!readFeedback(&d,&stamp) && d.reason==2);
  healthy(150000,200000); postReleaseShadow.positionUpdateCount=1;
  postReleaseShadowPublishLog();
  assert(!readFeedback(&d,&stamp) && d.reason==2);
  // Non-finite/un-normalized current data must not become healthy cache data.
  healthy(150000,210000); postReleaseShadow.quaternionWxyz[0]=NAN;
  postReleaseShadowPublishLog();
  assert(!readFeedback(&d,&stamp) && d.reason==4);
  healthy(150000,220000); postReleaseShadowPairedGyroDegPerS.x=INFINITY;
  postReleaseShadowPublishLog();
  assert(!readFeedback(&d,&stamp) && d.reason==4);
  // Separate IMU age, future timestamps and wrapping uptime.
  healthy(150000,230000); postReleaseShadowPairTimestampUs=209999;
  postReleaseShadowPublishLog();
  assert(!readFeedback(&d,&stamp) && d.reason==3);
  healthy(150000,240000); nowUs=239999;
  assert(!readFeedback(&d,&stamp) && d.reason==3);
  healthy(UINT32_MAX-100000,UINT32_MAX-999);
  nowUs=500; postReleaseShadowValid=0; postReleaseShadowPublishLog();
  assert(readFeedback(&d,&stamp) && d.stateAgeUs==1500 && (d.flags&32));
  nowUs=19001;
  assert(!readFeedback(&d,&stamp) && d.reason==3);
  puts("PASS: production publisher/getter preemption, hold, expiry, recovery, epoch, invalid feedback and clock wrap");
  return 0;
}
'''
    compiler_flags = []
    if args.expect_preemption_failure:
        program = program.replace(
            'assert(estimatorKalmanGetHardwareAttitudeDiagnostic(q,rates,NULL,&d));',
            'assert(!estimatorKalmanGetHardwareAttitudeDiagnostic(q,rates,NULL,&d));'
            'assert(d.reason == 2);'
            'puts("PASS: unpatched publisher reproduces reason-2 read failure during preemption");'
            'exit(0);')
        compiler_flags = ['-Wno-unused-function', '-Wno-unused-variable']
    with tempfile.TemporaryDirectory() as work:
        test = Path(work) / 'feedback.c'
        test.write_text(program)
        executable = Path(work) / 'feedback'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', *compiler_flags,
                        '-fsanitize=address,undefined', '-I',
                        str(root / 'src/modules/interface/estimator'), str(test),
                        '-lm', '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    main()
