"""Ground-only identity/boot checks; no arming or commander messages."""
import argparse
import datetime
import json
from pathlib import Path
import tempfile
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

parser=argparse.ArgumentParser();parser.add_argument('--after',action='store_true');args=parser.parse_args()
root=Path(__file__).resolve().parent
uri='radio://0/100/2M/E7E7E7E706'
identity='0x37383134303347100034002C'
required=['cpu.id0','cpu.id1','cpu.id2','system.selftestPassed','hlCommander.pRelAuto','stabilizer.estimator']
if args.after:required+=json.loads((root/'required-parameters.json').read_text())
cflib.crtp.init_drivers();console=[]
def receive(text):console.append(text);print(text,end='',flush=True)
with tempfile.TemporaryDirectory(prefix='lb11-firmware-check-') as cache:
    cf=Crazyflie(rw_cache=cache);cf.console.receivedChar.add_callback(receive)
    with SyncCrazyflie(uri,cf=cf) as link:
        end=time.monotonic()+25
        while not link.is_params_updated():
            if time.monotonic()>end:raise TimeoutError('Parameter download timed out')
            time.sleep(.05)
        available={g+'.'+n for g,fields in cf.param.toc.toc.items() for n in fields}
        missing=sorted(set(required)-available);assert not missing,missing
        values={k:cf.param.get_value(k) for k in sorted(set(required))}
        ids=[int(values['cpu.id'+str(i)]) for i in range(3)]
        actual='0x'+''.join(f'{x:08X}' for x in reversed(ids));assert actual==identity,(actual,identity)
        assert int(values['system.selftestPassed'])==1,values['system.selftestPassed']
        config=LogConfig(name='lb11FlashCheck',period_in_ms=100)
        for i in range(1,5):config.add_variable('motor.m'+str(i),'uint16_t')
        config.add_variable('supervisor.info','uint16_t');config.add_variable('pm.vbat','float')
        samples=[]
        with SyncLogger(link,config) as logger:
            for tick,data,_ in logger:
                assert all(data['motor.m'+str(i)]==0 for i in range(1,5)),data
                assert data['supervisor.info']&2==0,('Armed; refuse reboot/flash',data)
                assert data['pm.vbat']>=7.0,('Battery below mission threshold',data)
                samples.append(dict(tick_ms=tick,**data))
                if len(samples)==3:break
        writes=[]
        if not args.after and int(values['hlCommander.pRelAuto'])!=0:
            # Disable an earlier mission only after independent unarmed/zero-output proof.
            import threading
            confirmed=threading.Event()
            def updated(name,value):
                if name=='hlCommander.pRelAuto' and int(value)==0:confirmed.set()
            cf.param.add_update_callback(group='hlCommander',name='pRelAuto',cb=updated)
            try:
                cf.param.set_value('hlCommander.pRelAuto','0');cf.param.request_param_update('hlCommander.pRelAuto')
                assert confirmed.wait(3),'Auto-brake disable not confirmed'
            finally:cf.param.remove_update_callback(group='hlCommander',name='pRelAuto',cb=updated)
            values['hlCommander.pRelAuto']='0';writes=['hlCommander.pRelAuto=0']
        assert int(values['hlCommander.pRelAuto'])==0
        if args.after:
            expected={'mocapMarker.api':1,'mocapMarker.enable':0,'hlCommander.pRelUnwVer':1,
                'hlCommander.pRelUnwAuto':0,'hlCommander.pRelEndV':0,'hlCommander.pRelTMax':40,
                'hlCommander.pRelTilt':40,'pidCtl.resetMode':1,'hlCommander.pRelRstXY':1,
                'kalmanPRel.enable':0,'kalmanPRel.scEnable':0,'stabilizer.estimator':2}
            for k,v in expected.items():assert abs(float(values[k])-v)<1.e-4,(k,values[k],v)
            assert 'bolt-flyback-auto-26102901' in ''.join(console),'New build tag not observed'
            for group,names in {'mocapRef':['x','y','z','vx','vy','vz','age'],
                'hlCommander':['scUnwPhase','scUnwWhy','scUnwT','scUnwEndV']}.items():
                assert all(n in cf.log.toc.toc.get(group,{}) for n in names),(group,names)
        report=dict(identity=actual,uri=uri,drone_id='lb11',verified_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            unarmed=True,motor_outputs_all_zero=True,selftest_passed=True,battery_v=min(s['pm.vbat'] for s in samples),
            required_parameter_count=len(set(required)),parameters=values,samples=samples,
            parameter_writes=writes,setpoints_sent=False,flight_performed=False,console=''.join(console))
        name='device-after.json' if args.after else 'device-before.json'
        (root/name).write_text(json.dumps(report,indent=2)+'\n')
        print('\n'+('POST_FLASH_VERIFIED ' if args.after else 'PRE_FLASH_VERIFIED ')+json.dumps(report),flush=True)
