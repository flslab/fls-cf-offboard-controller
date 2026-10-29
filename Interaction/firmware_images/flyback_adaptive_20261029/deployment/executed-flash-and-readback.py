"""Flash only the identified lb11 STM32 application, then verify every byte.
No arming/setpoints, NRF programming, bootloader programming or Pi code update.
"""
import datetime
import hashlib
import json
import math
from pathlib import Path
import struct
import time
import cflib.crtp
from cflib.bootloader import Bootloader,Target

root=Path(__file__).resolve().parent
manifest=json.loads((root/'build-manifest.json').read_text())
binary=root/'bolt-flyback-auto-26102901.bin';expected=binary.read_bytes()
assert hashlib.sha256(expected).hexdigest()==manifest['binary_sha256']=='0e97e81583676eff78eec9f2f5212e4404d624dbc8aa023c2268488f0a4f4f3e'
proof=root/'device-before.json';assert 0<=time.time()-proof.stat().st_mtime<600,'Repeat grounded identity check'
pre=json.loads(proof.read_text());assert pre['identity']=='0x37383134303347100034002C'
assert pre['unarmed'] and pre['motor_outputs_all_zero'] and pre['selftest_passed']
assert pre['battery_v']>=7 and int(pre['parameters']['hlCommander.pRelAuto'])==0
uid=struct.pack('<3I',*[int(pre['parameters']['cpu.id'+str(i)]) for i in range(3)])
def target_check(loader):
    stm=loader._cload.targets[255]
    assert (stm.page_size,stm.start_page,stm.flash_pages)==(1024,16,1024),vars(stm)
    boot_uid=bytes.fromhex(stm.cpuid.replace(':',''))
    if boot_uid!=uid:
        # This lb11 bootloader reports a zero STM32 UID (also recorded in
        # the prior verified deployment). Bind its known warm address to the
        # fresh application CPU-ID proof instead of treating zeros as an ID.
        assert boot_uid==bytes(12),('Unexpected nonzero bootloader CPU ID',stm.cpuid)
        assert 'B15D84D3A3' in loader._cload.link.uri.upper(),loader._cload.link.uri
    assert len(expected)<=(stm.flash_pages-stm.start_page)*stm.page_size
    return stm
cflib.crtp.init_drivers();start=time.monotonic();bucket=-1
loader=Bootloader(pre['uri']);boot_uri=None
try:
    assert loader.start_bootloader(warm_boot=True),'Warm boot failed'
    boot_uri=loader._cload.link.uri
    (root/'captured-boot-uri.json').write_text(json.dumps({'uri':boot_uri,'application_identity':pre['identity']})+'\n')
    stm=target_check(loader)
    info=dict(uri=boot_uri,identity=pre['identity'],identity_source='fresh application CPU-ID and known lb11 warm-reset address',bootloader_uid_available=bytes.fromhex(stm.cpuid.replace(':',''))!=bytes(12),target=vars(stm),image_sha256=manifest['binary_sha256'])
    (root/'bootloader.json').write_text(json.dumps(info,indent=2)+'\n')
    print('BOOTLOADER_VERIFIED '+json.dumps(info),flush=True)
    def progress(message,percent):
        global bucket
        current=percent//10
        if current>bucket:bucket=current;print('FLASH',percent,message,flush=True)
    loader.progress_cb=progress
    loader.flash(str(binary),[Target('cf2','stm32','fw',[],[])])
    (root/'write-complete.json').write_text(json.dumps(dict(image=manifest['tag'],elapsed_s=time.monotonic()-start,
        bytes=len(expected),target='stm32:fw',nrf_or_bootloader_flashed=False),indent=2)+'\n')
    print('APPLICATION_WRITE_COMPLETE',flush=True)
finally:loader.close()
# Reconnect to the captured identity-bound bootloader link before reading.
time.sleep(1)
reader=Bootloader()
try:
    reader._cload.open_bootloader_uri(boot_uri.split('?')[0])
    assert reader._cload.check_link_and_get_info(),'Captured bootloader link unavailable'
    assert reader.start_bootloader(warm_boot=False),'Readback bootloader handshake failed'
    stm=target_check(reader);actual=bytearray();retries=0;read_start=time.monotonic()
    pages=math.ceil(len(expected)/stm.page_size)
    for offset in range(pages):
        page=None
        for attempt in range(6):
            page=reader._cload.read_flash(255,stm.start_page+offset)
            if page is not None and len(page)==stm.page_size:break
            retries+=1;print('RETRY_PAGE',offset,attempt+1,flush=True);time.sleep(.5)
        assert page is not None and len(page)==stm.page_size,('Read failure',offset)
        count=min(len(expected)-len(actual),stm.page_size);block=page[:count]
        assert block==expected[len(actual):len(actual)+count],('Readback mismatch',offset)
        actual.extend(block);time.sleep(.005)
        if offset%32==0 or offset==pages-1:print('READBACK',offset+1,'/',pages,flush=True)
    assert actual==expected
    report=dict(image=manifest['tag'],identity=pre['identity'],bytes=len(actual),
        sha256=hashlib.sha256(actual).hexdigest(),page_retries=retries,readback_elapsed_s=time.monotonic()-read_start,
        total_elapsed_s=time.monotonic()-start,target='stm32:fw',nrf_or_bootloader_flashed=False,
        parameter_writes=False,flight_performed=False,verified_at=datetime.datetime.now(datetime.timezone.utc).isoformat())
    (root/'full-flash-readback.json').write_text(json.dumps(report,indent=2)+'\n')
    print('FULL_READBACK_VERIFIED '+json.dumps(report),flush=True)
    assert reader.reset_to_firmware(),'Application reboot failed'
    print('BOOT_APPLICATION_OK',flush=True)
finally:reader.close()
