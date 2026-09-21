#!/usr/bin/env python3
"""Pure C response predictor tests; not closed-loop or MCU WCET acceptance."""
import os
from pathlib import Path
import subprocess
import tempfile

repo = Path(__file__).resolve().parents[2]
native = repo / 'Interaction/native/post_release'
firmware = Path(os.environ.get('FLS_LOCAL_UNWIND_FW',
    '/Users/shuqinzhu/Documents/FLS_Research/crazyflie-firmware-master-post-release'))
for name, folder in [('post_release_local_unwind.h', 'interface'),
                     ('post_release_local_unwind.c', 'src')]:
    assert (native / name).read_bytes() == (
        firmware / 'src/modules' / folder / 'kalman_core' / name).read_bytes(), name

with tempfile.TemporaryDirectory(prefix='local-unwind-test-') as tmp:
    for name, flags in [('sanitize', ['-O1', '-fsanitize=address,undefined']),
                        ('optimized', ['-O2'])]:
        binary = Path(tmp) / name
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', *flags,
            '-I' + str(native), str(Path(__file__).with_suffix('.c')),
            str(native / 'post_release_local_unwind.c'),
            str(native / 'post_release_forward_stop.c'), '-lm', '-o', str(binary)],
            check=True)
        subprocess.run([str(binary)], check=True)
print('PASS: native/sanitized, matching firmware sources, bounded 127-step predictor; '
      'host timing is not Cortex-M4 WCET or flight validation')
