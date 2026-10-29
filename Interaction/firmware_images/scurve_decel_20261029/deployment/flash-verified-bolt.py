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

binary = Path(__file__).with_name('bolt-scurve-reset-26102902.bin')
expected = binary.read_bytes()
digest = hashlib.sha256(expected).hexdigest()
assert digest == 'da098726e0ae83343f97267d91c36a3825bdc9b49e812c8ae7f1b0dd49a8f3db'
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
    print('FLASH_DONE', 'Reading all application bytes', flush=True)
    count = math.ceil(len(expected) / stm.page_size)
    actual = bytearray()
    for offset in range(count):
        page = None
        for attempt in range(3):
            page = loader._cload.read_flash(255, stm.start_page + offset)
            if page is not None and len(page) == stm.page_size:
                break
            print('RETRY_PAGE', offset, attempt + 1, flush=True)
            time.sleep(.2)
        assert page is not None and len(page) == stm.page_size, ('Read failure', offset)
        remaining = len(expected) - len(actual)
        block = page[:min(remaining, stm.page_size)]
        assert block == expected[len(actual):len(actual)+len(block)], ('Mismatch', offset)
        actual.extend(block)
        if offset % 32 == 0 or offset == count - 1:
            print('READBACK', offset + 1, '/', count, flush=True)
    assert actual == expected
    print('FULL_READBACK_VERIFIED', json.dumps({'bytes': len(actual),
          'sha256': hashlib.sha256(actual).hexdigest(),
          'elapsed_seconds': time.monotonic() - start, 'target': 'stm32:fw',
          'nrf_or_bootloader_flashed': False, 'flight_commands_sent': False}), flush=True)
    assert loader.reset_to_firmware(), 'Firmware reset failed'
    print('BOOT_APPLICATION_OK', flush=True)
finally:
    loader.close()
