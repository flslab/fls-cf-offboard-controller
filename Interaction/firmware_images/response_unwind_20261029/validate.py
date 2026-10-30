"""Offline package checks; no radio, SSH, parameter writes, or flight API."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parents[2]))
from Interaction.firmware_image_contract import check
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--extended', action='store_true')
    args = parser.parse_args()
    manifest = json.loads((ROOT / 'build-manifest.json').read_text())
    for name, expected in manifest['package_sha256'].items():
        if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != expected:
            raise ValueError('Package fingerprint mismatch: ' + name)
    elf = gzip.decompress((ROOT / 'bolt.elf.gz').read_bytes())
    assert hashlib.sha256(elf).hexdigest() == manifest['elf_sha256']
    canonical = ROOT.parents[1] / 'native' / 'post_release'
    for header in (ROOT / 'tests/include').glob('*.h'):
        if canonical.exists():
            assert header.read_bytes() == (canonical / header.name).read_bytes()
    compiler = shutil.which('clang')
    if not compiler:
        raise RuntimeError('Native sanitizer checks require clang')
    with tempfile.TemporaryDirectory(prefix='response-unwind-') as folder:
        exe = Path(folder) / 'test_response_unwind'
        subprocess.run([compiler, '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=address,undefined', '-I' + str(ROOT / 'tests/include'),
                        str(ROOT / 'tests/test_response_unwind.c'), '-lm', '-o', str(exe)],
                       check=True, timeout=60)
        result = subprocess.run([str(exe)] + (['extended'] if args.extended else []),
                                capture_output=True, text=True, check=True,
                                timeout=1800 if args.extended else 120)
        native = json.loads(result.stdout)
    contract = check(ROOT / 'bolt.elf.gz', yaml.safe_load((ROOT / 'mission-example.yaml').read_text()))
    if not contract['passed']:
        raise RuntimeError('Linked firmware contract missing: ' + str(contract['missing']))
    print(json.dumps(dict(passed=True, native=native, compiled_parameters=len(contract['required']),
                          hardware_access=False, flight_validated=False), indent=2))


if __name__ == '__main__':
    main()
