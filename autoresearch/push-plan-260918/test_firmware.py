#!/usr/bin/env python3
"""Production C protocol smoke tests; not a flight/closed-loop acceptance."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parent
FW = Path(os.environ.get('FLS_JOINT_FW', '/Users/shuqinzhu/Documents/FLS_Research/crazyflie-firmware-master-post-release'))
with tempfile.TemporaryDirectory(prefix='fls-push-plan-') as directory:
    binary = Path(directory) / 'firmware-smoke'
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
        '-fsanitize=address,undefined', '-I'+str(FW/'src/modules/interface/kalman_core'),
        str(ROOT/'firmware_smoke.c'),
        *[str(FW/'src/modules/src/kalman_core'/name) for name in (
            'post_release_forward_stop.c', 'post_release_joint_unwind.c', 'post_release_push_plan.c')],
        '-lm', '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
source = (FW/'src/modules/src/crtp_commander_high_level.c').read_text()
assert 'static uint8_t jointHostEnabled = 0;' in source
assert 'static uint32_t jointBuildVersion = 26092101u;' in source
assert 'if(jointHostEnabled) return; // Host mode never invokes the heavy FC solver.' in source
assert 'if (!jointHostEnabled && fresh && attitudeFresh' in source
assert 'postReleaseAutoStage=6u;postReleaseAutoAbortReason=11u;' in source
assert 'if(packet->channel!=0) return true;' in source
branch = source[source.index('static bool jointPushHandlePacket'):source.index('static void jointHostCapture')]
assert 'crtpSendPacketBlock' not in branch
assert 'postReleasePushReceivePart(' in branch
assert '!jointHostScheduled && !jointActive' in branch
assert 'postReleaseAutoStage==2u && jointLocalActive' in branch
assert 'jointSolve(' not in branch and 'jointPredict(' not in branch
capture = source[source.index('static void jointHostCapture('):source.index('static bool jointHostBeginLocal(')]
assert 'postReleaseAutoRapidSetpointCount' not in capture
assert 'snapshot.prefixDurationS=jointLocalProfile.durationS' in capture
assert 'postReleaseAutoLatestUnwindUs' not in capture
assert 'postReleasePushMarkExecuting(' in source
assert 'postReleasePushReset(&jointPush,postReleaseAutoSession,postReleaseAutoSequence,++jointPushToken)' in source
applied = source[source.index('void crtpCommanderHighLevelPostReleaseApplied('):source.index('static void jointHostDeadlineFault(')]
assert 'xSemaphoreTake(lockTraj,0)' in applied
assert 'postReleasePushMarkExecuting(' in applied
pid = (FW/'src/modules/src/controller/controller_pid.c').read_text()
assert 'crtpCommanderHighLevelPostReleaseApplied(setpoint)' in pid
assert 'RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep) && control->thrust > 0' in pid
print('PASS: host opt-in, automatic v3 accept, separate transport/local deadlines, prefix generations, actual execution notice, nonblocking-send invariants')
