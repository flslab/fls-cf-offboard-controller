"""Compile the production reset call sites with native counter stubs.

Usage: python3 validate_reset_policy.py /path/to/patched/firmware
Checks reset dispatch/latching, not flight dynamics or the complete controller.
"""
import pathlib
import subprocess
import sys
import tempfile


def block(text, marker):
    start = text.index(marker)
    brace = text.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


root = pathlib.Path(sys.argv[1]) / 'src/modules/src'
pid = (root / 'controller/controller_pid.c').read_text()
commander = (root / 'crtp_commander_high_level.c').read_text()
runtime = (root / 'post_release_velocity_runtime.inc').read_text()
handoff = (root / 'post_release_position_handoff.inc').read_text()
assert 'static uint8_t resetOnModeChange = 1;' in pid
assert 'static uint8_t scResetXYParam=1;' in commander
assert 'static bool scResetXYActive=true;' in commander
assert 'PARAM_ADD(PARAM_UINT8, resetMode, &resetOnModeChange)' in pid
assert 'PARAM_ADD(PARAM_UINT8, pRelRstXY, &scResetXYParam)' in commander
assert commander.index('scResetXYActive=scResetXYParam!=0;') > commander.index('postReleaseAutoStartUs = nowUs;')

start = handoff.index('  if(scResetXYActive)positionControllerResetXYFromState(s);')
end = handoff.index('  postReleaseAutoStage=4;', start)
terminal = handoff[start:end]
entry = next(line for line in runtime.splitlines() if 'positionControllerResetXYFromState(s)' in line)
legacy = []
for name in ('post_release_scurve_runtime.inc', 'post_release_calibrated_runtime.inc'):
    legacy.append(next(line for line in (root/name).read_text().splitlines()
                       if 'positionControllerResetXY(position[0],position[1])' in line))

source = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
enum {modeDisable,modeAbs,modeVelocity,modeAbsRate};
typedef struct {int x,y,z,roll,pitch,yaw,quat;} setpoint_mode_t;
typedef struct {setpoint_mode_t mode;} setpoint_t;
typedef struct {float x,y,z;} point;
typedef struct {float roll,pitch,yaw;} attitude_t;
typedef struct {point position;attitude_t attitude;} state_t;
typedef struct {float thrust,roll,pitch,yaw;} control_t;
static float actuatorThrust,cmd_thrust,cmd_roll,cmd_pitch,cmd_yaw;
static attitude_t attitudeDesired;
static bool preserveInnerOnModeChange,scResetXYActive=true;
static uint8_t resetOnModeChange=1,scResetXYParam=1;
static uint32_t fullModeResetCount,xyModeResetCount;
static unsigned full,xy,inner,filters,seeds,armed;
static float integral=7;
static bool scHoldValid=true,scCurveEndpointPolicy=false;
static float scHoldTarget[2];static int scHoldInput;
static void positionControllerResetAllPID(float x,float y,float z){(void)x;(void)y;(void)z;full++;integral=0;}
static void positionControllerResetAllfilters(void){filters++;}
static void attitudeControllerResetAllPID(float r,float p,float y){(void)r;(void)p;(void)y;inner++;}
static void positionControllerResetXYFromState(const state_t *s){(void)s;xy++;integral=0;}
static void positionControllerResetXY(float x,float y){(void)x;(void)y;xy++;integral=0;}
static void positionControllerArmHold(float *target,int *input){(void)target;(void)input;armed++;}
static void scSeedCompensatedHold(const state_t *s){(void)s;seeds++;integral=17;}
'''
source += block(pid, 'static void controllerPidReinitialize(') + '\n'
source += block(pid, 'static bool setpointModeChanged(') + '\n'
source += 'static void tick(const setpoint_t *setpoint,const state_t *state){\n'
source += block(pid, 'if (setpointModeChanged(setpoint) && resetOnModeChange != 0)') + '}\n'
source += 'static void stopped(control_t *control,const state_t *state){\n'
source += block(pid, 'if (control->thrust == 0)') + '}\n'
source += 'static void accept(void){scResetXYActive=scResetXYParam!=0;}\n'
source += 'static void enter(const state_t *s){' + entry + '}\n'
source += 'static void leave(const state_t *s){' + terminal + '}\n'
for i, line in enumerate(legacy):
    source += f'static void legacy{i}(const float *position){{{line}}}\n'
source += r'''
int main(void){
  state_t s={.position={1,2,3},.attitude={4,5,6}};
  setpoint_t p={.mode={.z=modeAbs,.roll=modeAbs,.pitch=modeAbs}};
  actuatorThrust=100;
  tick(&p,&s);assert(full==1 && inner==1 && filters==1);
  p.mode.x=p.mode.y=modeAbs;integral=7;
  tick(&p,&s);assert(xy==1 && full==1 && integral==0);
  p.mode.roll=modeAbsRate;tick(&p,&s);assert(xy==1); // Feedforward alone is no mode change.
  resetOnModeChange=0;p.mode.x=p.mode.y=modeDisable;integral=7;
  tick(&p,&s);assert(xy==1 && full==1 && integral==7);
  resetOnModeChange=1;tick(&p,&s);assert(xy==1 && full==1); // Observed while off.
  p.mode.roll=modeVelocity;tick(&p,&s);assert(full==2 && inner==2);
  // Disabled policy suppresses the full-reset branch too.
  resetOnModeChange=0;p.mode.roll=modeAbs;integral=7;
  tick(&p,&s);assert(full==2 && integral==7);
  for(unsigned mode=0;mode<2;mode++)for(unsigned curve=0;curve<2;curve++){
    resetOnModeChange=mode;scResetXYParam=curve;accept();
    unsigned beforeXY=xy,beforeSeed=seeds,beforeArm=armed;
    integral=7;enter(&s);assert(xy==beforeXY+curve && integral==(curve?0:7));
    // Later writes never change an accepted curve's reset policy.
    scResetXYParam=!curve;
    leave(&s);assert(xy==beforeXY+2*curve && seeds==beforeSeed+curve);
    assert(armed==beforeArm+1 && integral==(curve?17:7));
    legacy0((float[]){1,2});legacy1((float[]){1,2});
    assert(xy==beforeXY+4*curve);
    unsigned beforeFull=full,beforeInner=inner,beforeFilters=filters;
    control_t c={.thrust=100,.roll=1,.pitch=2,.yaw=3};integral=7;
    stopped(&c,&s);assert(full==beforeFull && integral==7);
    c.thrust=0;stopped(&c,&s);
    assert(full==beforeFull+1 && inner==beforeInner+1 && filters==beforeFilters+1);
    assert(integral==0 && c.roll==0 && c.pitch==0 && c.yaw==0);
    assert(attitudeDesired.yaw==s.attitude.yaw);
    accept();assert(scResetXYActive==!curve);
  }
  // Any nonzero parameter keeps resets enabled.
  scResetXYParam=2;accept();assert(scResetXYActive);
  resetOnModeChange=2;p.mode.quat=modeAbs;unsigned n=full;tick(&p,&s);assert(full==n+1);
  puts("PASS: mode reset, XY/full dispatch, mode history, feedforward normalization, release latch, entry/hold/legacy resets, trim preservation, zero-thrust reset under all four flag combinations");
}
'''
with tempfile.TemporaryDirectory(prefix='pid-reset-') as directory:
    c = pathlib.Path(directory)/'reset.c'
    binary = pathlib.Path(directory)/'reset'
    c.write_text(source)
    subprocess.run(['clang','-std=c11','-Wall','-Wextra','-Werror',
        '-fsanitize=address,undefined',str(c),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
