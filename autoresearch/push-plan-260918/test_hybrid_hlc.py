#!/usr/bin/env python3
"""Compile actual production HLC branches with host I/O stubs, no hardware."""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parent
fw = Path(os.environ.get('FLS_LOCAL_UNWIND_FW',
    '/Users/shuqinzhu/Documents/FLS_Research/crazyflie-firmware-master-post-release'))
source = (fw / 'src/modules/src/crtp_commander_high_level.c').read_text()


def block_at(marker, start=0):
    begin = source.index(marker, start)
    brace = source.index('{', begin)
    depth = 1
    end = brace + 1
    while depth:
        if source[end] == '{':
            depth += 1
        elif source[end] == '}':
            depth -= 1
        end += 1
    return source[begin:end]


helpers = '\n\n'.join(block_at(marker) for marker in [
    'static void jointCurrentWireModel(', 'static bool jointPushHandlePacket(',
    'static void jointHostCapture(', 'static float jointHostLocalDuration(',
    'static bool jointHostBeginLocal(', 'static bool jointHostCanFinishWithoutVicon(',
    'static void jointHostDeadlineFault(',
    'void crtpCommanderHighLevelPostReleaseApplied('])
getsetpoint = source.index('static bool postReleaseAutoGetSetpoint(')
host = block_at('if(jointHostEnabled && !jointActive)', getsetpoint)
render_start = source.index('  memset(setpoint, 0, sizeof(*setpoint));', getsetpoint)
render_end = source.index('  else if ((postReleaseAutoStage == 3u', render_start)
render = source[render_start:render_end]
template = (root / 'test_hybrid_hlc.c.in').read_text()
for marker, value in [('/*__PRODUCTION_HELPERS__*/', helpers),
                      ('/*__PRODUCTION_HOST_BLOCK__*/', host),
                      ('/*__PRODUCTION_SETPOINT_BRANCHES__*/', render)]:
    assert template.count(marker) == 1
    template = template.replace(marker, value)
assert 'postReleaseAutoLatestUnwindUs' not in host
assert 'postReleaseAutoRapidSetpointCount>0' not in helpers

with tempfile.TemporaryDirectory(prefix='hybrid-hlc-') as tmp:
    binary = Path(tmp) / 'test'
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-O1',
        '-fsanitize=address,undefined', '-I' + str(fw / 'src/modules/interface/kalman_core'),
        '-x', 'c', '-', *[str(fw / 'src/modules/src/kalman_core' / filename) for filename in [
            'post_release_local_unwind.c', 'post_release_forward_stop.c',
            'post_release_joint_unwind.c', 'post_release_push_plan.c']],
        '-lm', '-o', str(binary)], input=template, text=True, check=True)
    subprocess.run([str(binary)], check=True)
print('Scope: production extracted HLC decision and rendering + real protocol/numeric kernels; '
      'host stubs, not complete RTOS scheduling, estimator freshness, USB or flight validation')
