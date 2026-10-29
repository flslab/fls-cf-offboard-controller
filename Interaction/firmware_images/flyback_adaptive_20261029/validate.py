"""Offline package/native checks. No radio, SSH, parameter writes or flight API."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parents[2]))
from Interaction.firmware_image_contract import check
import yaml


def main():
    manifest=json.loads((ROOT/'build-manifest.json').read_text())
    def require_hash(name,expected):
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=expected:
            raise ValueError('Package fingerprint mismatch: '+name)
    require_hash('bolt-flyback-auto-26102901.bin',manifest['binary_sha256'])
    require_hash('bolt.elf.gz',manifest['elf_gzip_sha256'])
    require_hash('bolt.config',manifest['config_sha256'])
    assert hashlib.sha256(gzip.decompress((ROOT/'bolt.elf.gz').read_bytes())).hexdigest()==manifest['elf_sha256']
    for name,value in manifest['native_source_sha256'].items():require_hash(name,value)
    compiler=shutil.which('clang')
    if not compiler:raise RuntimeError('Native sanitizer checks require clang')
    with tempfile.TemporaryDirectory(prefix='flyback-native-') as folder:
        exe=Path(folder)/'test_flyback'
        command=[compiler,'-std=c11','-O2','-Wall','-Wextra','-Werror',
            '-fsanitize=address,undefined','-I'+str(ROOT/'tests/include'),
            str(ROOT/'tests/test_flyback.c'),str(ROOT/'tests/post_release_vicon_velocity_kf.c'),'-lm','-o',str(exe)]
        subprocess.run(command,check=True,timeout=60)
        result=subprocess.run([str(exe)],capture_output=True,text=True,check=True,timeout=60)
        native=json.loads(result.stdout)
    contract=check(ROOT/'bolt.elf.gz',yaml.safe_load((ROOT/'mission-overrides.yaml').read_text()))
    if not contract['passed']:raise RuntimeError('Linked firmware contract missing: '+str(contract['missing']))
    print(json.dumps({'passed':True,'native':native,'compiled_parameters':len(contract['required']),
        'hardware_access':False,'flight_validated':False},indent=2))


if __name__=='__main__':main()
