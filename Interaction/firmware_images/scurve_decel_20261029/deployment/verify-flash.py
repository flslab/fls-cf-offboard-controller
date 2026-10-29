import json, tempfile, time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
REQUIRED = ['eskfXY.ack', 'eskfXY.api', 'eskfXY.bx', 'eskfXY.by', 'eskfXY.on', 'eskfXY.req', 'eskfXY.status', 'eskfXY.sx', 'eskfXY.sy', 'hlCommander.pRelAMax', 'hlCommander.pRelAdapt', 'hlCommander.pRelAttFF', 'hlCommander.pRelAttP', 'hlCommander.pRelAttV', 'hlCommander.pRelAuto', 'hlCommander.pRelComp', 'hlCommander.pRelCompB', 'hlCommander.pRelCompP', 'hlCommander.pRelCompW', 'hlCommander.pRelEnd', 'hlCommander.pRelExec', 'hlCommander.pRelFric', 'hlCommander.pRelHold', 'hlCommander.pRelHoldG', 'hlCommander.pRelHost', 'hlCommander.pRelJoint', 'hlCommander.pRelLite', 'hlCommander.pRelMode', 'hlCommander.pRelOneT', 'hlCommander.pRelRstXY', 'hlCommander.pRelScA', 'hlCommander.pRelScB', 'hlCommander.pRelScD', 'hlCommander.pRelScT', 'hlCommander.pRelSeed', 'hlCommander.pRelShape', 'hlCommander.pRelTMax', 'hlCommander.pRelTail', 'hlCommander.pRelTau', 'hlCommander.pRelTilt', 'hlCommander.pRelVelCmd', 'kalmanPRel.enable', 'kalmanPRel.feedback', 'kalmanPRel.fuseVicon', 'kalmanPRel.scEnable', 'kalmanPRel.viconCI', 'pRelResp.activeId', 'pRelResp.commit', 'pRelResp.id', 'pRelResp.ready', 'pRelResp.runtime', 'pidCtl.resetMode', 'stabilizer.estimator']
cflib.crtp.init_drivers()
console=[]
def on_console(text):
    console.append(text)
    print(text,end='',flush=True)
with tempfile.TemporaryDirectory(prefix='paired-xy-v2-') as cache:
    cf=Crazyflie(rw_cache=cache)
    cf.console.receivedChar.add_callback(on_console)
    with SyncCrazyflie('radio://0/100/2M/E7E7E7E706',cf=cf) as link:
        deadline=time.monotonic()+25
        while not link.is_params_updated():
            if time.monotonic()>deadline:raise TimeoutError('Parameters not received')
            time.sleep(.05)
        available={group+'.'+name for group,fields in cf.param.toc.toc.items() for name in fields}
        missing=sorted(set(REQUIRED)-available)
        assert not missing,missing
        values={name:cf.param.get_value(name) for name in REQUIRED}
        expected={'eskfXY.api':1,'eskfXY.on':0,'eskfXY.ack':0,'eskfXY.req':0,
            'stabilizer.estimator':2,'system.selftestPassed':1,'hlCommander.pRelAuto':0,
            'hlCommander.pRelReqVer':1,'hlCommander.pRelMuVer':1,'hlCommander.pRelEnd':1,
            'hlCommander.pRelSVer':26092803,'hlCommander.curveVer':4,
            'pidCtl.resetMode':1,'hlCommander.pRelRstXY':1,'hlCommander.pRelTMax':40,'cpu.id0':3407916,'cpu.id1':808666896,'cpu.id2':926429492}
        for name,value in expected.items():
            values[name]=cf.param.get_value(name)
            assert float(values[name])==float(value),(name,values[name],value)
        for name, expected_value in {'hlCommander.pRelAMax':8.23,'hlCommander.pRelScA':3.6,'hlCommander.pRelTilt':22.183041,'hlCommander.pRelTail':1.2}.items():
            assert abs(float(values[name])-expected_value)<1e-4,(name,values[name])
        from pathlib import Path
        startup = Path(__file__).with_name('startup-first.log').read_text()
        assert 'bolt-scurve-reset-26102902' in startup
        log=LogConfig(name='pairedXYCheck',period_in_ms=100)
        for n in range(1,5):log.add_variable('motor.m'+str(n),'uint16_t')
        log.add_variable('supervisor.info','uint16_t')
        samples=[]
        with SyncLogger(link,log) as logger:
            for index,entry in enumerate(logger):
                tick,data,_=entry
                assert all(data['motor.m'+str(n)]==0 for n in range(1,5)),data
                assert data['supervisor.info']&2==0,data
                samples.append(dict(tick_ms=tick,**data))
                if index==2:break
        report=dict(required_parameter_count=len(REQUIRED),missing=[],parameters=values,
            build='bolt-scurve-reset-26102902',
            build_evidence='first post-flash startup console plus full byte readback; same CPU identity on current connection',selftest_passed=True,
            unarmed=True,motors_zero=True,parameter_writes=False,setpoints_sent=False,
            flight_performed=False,samples=samples)
        print('\nREADBACK_JSON '+json.dumps(report,sort_keys=True),flush=True)
        print('PAIRED_FIRMWARE_VERIFIED',flush=True)
