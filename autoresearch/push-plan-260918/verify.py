"""First implementation acceptance, not simulation/flight acceptance."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
FW = Path('/Users/shuqinzhu/Documents/FLS_Research/crazyflie-firmware-master-post-release')
BUILD = FW / 'build-post-release-pi-plan-260918'
TESTS = [
    'Interaction/tests/test_post_release_pi_planner.py',
    'Interaction/tests/test_firmware_auto_brake_preflight.py',
    'Interaction/tests/test_post_release_firmware_control_event.py',
    'Interaction/tests/test_interaction_log_period.py',
    'Interaction/tests/test_compressed_state_logs.py',
]


def main():
    commands = [
        ([sys.executable, '-m', 'unittest', '-q',
          *(name[:-3].replace('/', '.') for name in TESTS)], ROOT),
        ([sys.executable, 'autoresearch/push-plan-260918/test_firmware.py'], ROOT),
        ([sys.executable, 'autoresearch/push-plan-260918/test_holdout.py'], ROOT),
        (['make', 'O=' + BUILD.name, 'KBUILD_OUTPUT=' + str(BUILD), '-j4'], FW),
    ]
    for command, cwd in commands:
        result = subprocess.run(command, cwd=cwd)
        if result.returncode:
            print('FAIL: ' + ' '.join(command), flush=True)
            return 1
    if not (BUILD / 'bolt.bin').is_file():
        print('FAIL: missing Bolt image', flush=True)
        return 1
    native = ROOT / 'Interaction/native/post_release'
    for stem in ('post_release_joint_unwind', 'post_release_forward_stop'):
        for suffix, folder in (('.c', 'src'), ('.h', 'interface')):
            name = stem + suffix
            if (native / name).read_bytes() != (FW / 'src/modules' / folder /
                                                'kalman_core' / name).read_bytes():
                print('FAIL: Pi/FC numerical kernel mismatch: ' + name, flush=True)
                return 1
    print('PASS: protocol, regression and Bolt build; no flight validation', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
