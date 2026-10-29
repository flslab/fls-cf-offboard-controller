"""Read-only full application verification on the recorded lb11 warm link."""
import datetime,hashlib,json,math,time
from pathlib import Path
import cflib.crtp
from cflib.bootloader import Bootloader
root=Path(__file__).resolve().parent
manifest=json.loads((root/'build-manifest.json').read_text());expected=(root/'bolt-flyback-auto-26102901.bin').read_bytes()
assert hashlib.sha256(expected).hexdigest()==manifest['binary_sha256']
pre=json.loads((root/'device-before.json').read_text());boot=json.loads((root/'bootloader.json').read_text())
assert pre['identity']==boot['identity']=='0x37383134303347100034002C'
assert pre['unarmed'] and pre['motor_outputs_all_zero']
assert 'B15D84D3A3' in boot['uri'].upper()
uri=boot['uri'].split('?')[0];cflib.crtp.init_drivers();loader=None;reconnections=0

def connect():
 global loader
 if loader:loader.close()
 loader=Bootloader();loader._cload.open_bootloader_uri(uri)
 assert loader._cload.check_link_and_get_info(),'Known bootloader link unavailable'
 stm=loader._cload.targets[255]
 assert (stm.page_size,stm.start_page,stm.flash_pages)==(1024,16,1024),vars(stm)
 # The observed STM32 bootloader cpuid payload is not reliable for identity. Device
 # identity is bound by the fresh pre-flash application CPU ID and warm URI.
 print('READ_ONLY_LINK',loader._cload.link.uri,flush=True)
 return stm
start=time.monotonic();actual=bytearray();retries=0
try:
 stm=connect();pages=math.ceil(len(expected)/stm.page_size)
 for offset in range(pages):
  page=None
  for attempt in range(3):
   page=loader._cload.read_flash(255,stm.start_page+offset)
   if page is not None and len(page)==stm.page_size:break
   retries+=1;reconnections+=1;print('RECONNECT_PAGE',offset,attempt+1,flush=True)
   time.sleep(.3);stm=connect()
  assert page is not None and len(page)==stm.page_size,('Read failure',offset)
  block=page[:min(len(expected)-len(actual),stm.page_size)]
  assert block==expected[len(actual):len(actual)+len(block)],('Readback mismatch',offset)
  actual.extend(block);time.sleep(.005)
  if offset%32==0 or offset==pages-1:print('READBACK',offset+1,'/',pages,flush=True)
 assert actual==expected
 report=dict(image=manifest['tag'],identity=pre['identity'],bytes=len(actual),
  sha256=hashlib.sha256(actual).hexdigest(),page_retries=retries,reconnections=reconnections,
  readback_elapsed_s=time.monotonic()-start,target='stm32:fw',flash_written=False,
  nrf_or_bootloader_flashed=False,parameter_writes=False,flight_performed=False,
  verified_at=datetime.datetime.now(datetime.timezone.utc).isoformat())
 (root/'full-flash-readback.json').write_text(json.dumps(report,indent=2)+'\n')
 print('FULL_READBACK_VERIFIED '+json.dumps(report),flush=True)
 assert loader.reset_to_firmware(),'Application reboot failed'
 print('BOOT_APPLICATION_OK',flush=True)
finally:
 if loader:loader.close()
