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
assert 'static uint32_t jointBuildVersion = 26091805u;' in source
assert 'if(jointHostEnabled) return; // Host mode never invokes the heavy FC solver.' in source
assert 'if (!jointHostEnabled && fresh && attitudeFresh' in source
assert 'postReleaseAutoStage=6u;postReleaseAutoAbortReason=11u;' in source
assert 'if(packet->channel!=0) return true;' in source
branch = source[source.index('static bool jointPushHandlePacket'):source.index('static void jointHostCapture')]
assert 'crtpSendPacketBlock' not in branch
assert 'postReleaseAutoStage==1u && !jointHostScheduled' in branch
assert 'jointSolve(' not in branch and 'jointPredict(' not in branch
print('PASS: host opt-in, lightweight path, channel, deadline-fault and nonblocking-send invariants')
