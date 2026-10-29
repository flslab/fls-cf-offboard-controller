"""Read all STM32 application bytes; never erase or program flash.

After an exact match, reboot to the application for a separate unarmed check.
The operator must put lb11 in cold bootloader mode unless a captured warm URI
is supplied. Bounded page retries handle a lost packet without accepting gaps.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import cflib.crtp
from cflib.bootloader import Bootloader

parser = argparse.ArgumentParser()
parser.add_argument('--uri', help='Previously captured bootloader URI only')
args = parser.parse_args()
expected = Path(__file__).with_name('bolt-scurve-40default-26102903.bin').read_bytes()
assert hashlib.sha256(expected).hexdigest() == '42c6dc0d4a43f7022c4dd126d1a6c067ac2a1de1746db2ef5e0cdb497eabc69f'
cflib.crtp.init_drivers()
loader = Bootloader()
start = time.monotonic()
try:
    if args.uri:
        loader._cload.open_bootloader_uri(args.uri.split('?')[0])
        assert loader._cload.check_link_and_get_info(), 'Captured bootloader URI unavailable'
    assert loader.start_bootloader(warm_boot=False), 'No bootloader found'
    print('BOOTLOADER_URI', loader._cload.link.uri, flush=True)
    stm = loader._cload.targets[255]
    assert (stm.page_size, stm.start_page, stm.flash_pages) == (1024, 16, 1024), vars(stm)
    actual = bytearray()
    pages = math.ceil(len(expected) / stm.page_size)
    retries = 0
    for offset in range(pages):
        page = None
        for attempt in range(6):
            page = loader._cload.read_flash(255, stm.start_page + offset)
            if page is not None and len(page) == stm.page_size:
                break
            retries += 1
            print('RETRY_PAGE', offset, attempt + 1, flush=True)
            time.sleep(.5)
        assert page is not None and len(page) == stm.page_size, ('Read failure', offset)
        block = page[:min(len(expected) - len(actual), stm.page_size)]
        assert block == expected[len(actual):len(actual) + len(block)], ('Mismatch', offset)
        actual.extend(block)
        time.sleep(.005)
        if offset % 32 == 0 or offset == pages - 1:
            print('READBACK', offset + 1, '/', pages, flush=True)
    assert actual == expected
    print('FULL_READBACK_VERIFIED', json.dumps(dict(bytes=len(actual),
        sha256=hashlib.sha256(actual).hexdigest(), elapsed_seconds=time.monotonic() - start,
        target='stm32:fw', page_retries=retries, flash_written=False,
        nrf_or_bootloader_flashed=False, flight_commands_sent=False)), flush=True)
    assert loader.reset_to_firmware(), 'Firmware reset failed'
    print('BOOT_APPLICATION_OK', flush=True)
finally:
    loader.close()
