"""Flash STM32 application only and compare every application byte by readback.

Run only after identify-before-flash.py has verified the known device is unarmed.
No arming, commander packets, PID changes or NRF/bootloader programming.
"""
import hashlib
import json
import math
from pathlib import Path
import time
import cflib.crtp
from cflib.bootloader import Bootloader, Target

binary = Path(__file__).with_name('bolt-scurve-40default-26102903.bin')
expected = binary.read_bytes()
digest = hashlib.sha256(expected).hexdigest()
assert digest == '42c6dc0d4a43f7022c4dd126d1a6c067ac2a1de1746db2ef5e0cdb497eabc69f'
precheck_path = binary.with_name('identify-before-flash.log')
assert time.time() - precheck_path.stat().st_mtime < 600, 'Repeat the pre-flash check'
precheck = next(json.loads(line.split(' ', 1)[1]) for line in precheck_path.read_text().splitlines()
                if line.startswith('PRE_FLASH_VERIFIED '))
assert precheck['identity'] == '0x37383134303347100034002C'
assert precheck['unarmed'] and precheck['motor_outputs_all_zero']
assert precheck['parameters']['hlCommander.pRelAuto'] == '0'
cflib.crtp.init_drivers()
loader = Bootloader('radio://0/100/2M/E7E7E7E706')
progress_bucket = -1
def progress(message, percent):
    global progress_bucket
    if percent // 10 > progress_bucket:
        progress_bucket = percent // 10
        print('FLASH', percent, message, flush=True)
loader.progress_cb = progress
start = time.monotonic()
try:
    assert loader.start_bootloader(warm_boot=True), 'Warm boot failed'
    print('BOOTLOADER_URI', loader._cload.link.uri, flush=True)
    stm = loader._cload.targets[255]
    assert stm.page_size == 1024 and stm.start_page == 16 and stm.flash_pages == 1024, vars(stm)
    print('TARGET', json.dumps(vars(stm)), flush=True)
    loader.flash(str(binary), [Target('cf2', 'stm32', 'fw', [], [])])
    print('WRITE_COMPLETE; reconnect to captured bootloader URI for full readback before booting', flush=True)
finally:
    loader.close()
