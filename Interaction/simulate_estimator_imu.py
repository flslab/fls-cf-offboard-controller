"""Hardware-free integration of fixture collection, flight task and real-C replay.

This is a deterministic reduced plant, not Gazebo or a hardware prediction.
The reference channel is ideal simulated attitude (not the ordinary KF).
Transport, Dispatcher selection and arming ACKs are simulated; production
collection, task, standard takeoff/landing methods, artifact checks and replay
are executed. No radio driver is initialized and no hardware URI is accepted.
"""
import argparse
import ast
from contextlib import contextmanager, ExitStack
from copy import deepcopy
import hashlib
import json
import logging
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from Interaction import calibrate_estimator_imu as collector
from Interaction import estimator_validation_controller as task
from Interaction import estimator_validation_flight as flight
from Interaction.estimator_imu_calibration import G, default_plan, write_json
from Interaction.imu_logging import fls_log_config
from Interaction.replay_estimator3 import run_replay

ROOT = Path(__file__).resolve().parents[1]
ORCHESTRATOR = ROOT.parent/'lightbender/orchestrator'


def rotation_xyz(angles):
    r, p, y = np.deg2rad(angles)
    cr, cp, cy = np.cos([r,p,y]); sr, sp, sy = np.sin([r,p,y])
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
        [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def quaternion(rpy):
    r,p,y=np.asarray(rpy)/2
    cr,cp,cy=np.cos([r,p,y]);sr,sp,sy=np.sin([r,p,y])
    return np.array([cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,
        cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy])


class Clock:
    def __init__(self):
        self.t=1000.;self.on_step=None;self.device_offset_ms=0
    def monotonic(self):return self.t
    def monotonic_ns(self):return round(self.t*1e9)
    def time(self):return self.t+1_700_000_000
    def sleep(self,duration):
        if duration <= 0:return
        count=max(1,round(duration/.01))
        for _ in range(count):
            dt=duration/count;self.t+=dt
            if self.on_step:self.on_step(dt)
    def tick(self):return (100_000+round((self.t-1000.)*1000)+self.device_offset_ms)%(1<<24)


class Param:
    def __init__(self):
        self.values={'stabilizer':{'estimator':'2','controller':'1'},
            'imu_sensors':{'imuPhi':'0','imuTheta':'0'},
            'hlCommander':{'pRelAuto':'0'},'kalmanPRel':{'enable':'0','scEnable':'0'}}
        self.toc=SimpleNamespace(toc=self.values);self.callbacks={};self.writes=[]
    def get_value(self,key):
        group,name=key.split('.');return self.values[group][name]
    def set_value(self,key,value):
        group,name=key.split('.');self.values[group][name]=str(value);self.writes.append((key,str(value)))
    def set_value_raw(self,key,kind,value):self.set_value(key,value)
    def add_update_callback(self,group,name,cb):self.callbacks[group+'.'+name]=cb
    def remove_update_callback(self,group,name,cb):self.callbacks.pop(group+'.'+name)
    def request_param_update(self,key):self.callbacks[key](key,self.get_value(key))


def gravity_sim_directions():
    """Unknown exact support angles; ONLY the simulated plant sees these."""
    bases = np.array([[0,0,1],[0,0,-1],[1,0,0],[-1,0,0],[0,1,0],[0,-1,0]],float)
    result = []
    for group in range(4):
        for base in bases:
            axis = int(np.argmax(np.abs(base)))
            first = np.eye(3)[(axis+1)%3]
            second = np.eye(3)[(axis+2)%3]
            angle = np.deg2rad((0,25,30,38)[group])
            tangent = (first if group == 1 else second) if group < 3 else (first-second)/np.sqrt(2)
            result.append(np.cos(angle)*base+np.sin(angle)*tangent)
    return np.array(result)


def collect_fixture(output, clock, A, b, bg, seed, *, calibration_method='six_face'):
    """Actual interactive collector with independent known-pose sensor source."""
    rng=np.random.default_rng(seed);current={'force':np.array([0.,0.,G]),'step':0}
    poses=default_plan()[:6]
    forces = gravity_sim_directions()*G if calibration_method == 'gravity_norm' else np.array(
        [p['reference_force_m_s2'] for p in poses])
    def prompt(message):
        if message.startswith('['):
            current['force']=forces[int(message.split('/')[0][1:])-1]
        return ''
    @contextmanager
    def source(uri):
        assert uri=='sim://fixture'
        metadata={'firmware_parameters_before':{'imu_sensors':{'imuPhi':'0','imuTheta':'0'}},
            'firmware_parameters_after':{'imu_sensors':{'imuPhi':'0','imuTheta':'0'}},
            'simulated':True,'requested_imu_period_ms':10}
        def packet(timeout):
            current['step']+=1
            if current['step']%6==0:
                data={key:0 for key in collector.MOTOR_FIELDS};name='motors'
            else:
                clock.sleep(.01)
                acc=(A@current['force']+b+rng.normal(0,.005,3))/G
                gyro=np.rad2deg(bg+rng.normal(0,1e-5,3))
                data={**dict(zip(collector.IMU_FIELDS[:3],acc)),
                      **dict(zip(collector.IMU_FIELDS[3:],gyro))};name='imu'
            return name,clock.tick(),data,clock.monotonic_ns()
        yield packet,metadata
    args=SimpleNamespace(output=output,uri='sim://fixture',drone_id='lb11',
        firmware_id='simulated-processed-imu',fixture_id='independent-exact-axis-simulation',
        reference_note='Known plant axes; independent fixture noise, not fitted flight data',
        duration_s=4.,settle_s=2.,fit_only=True,auto_flight=False,gyro_calibration=False,
        calibration_method=calibration_method)
    original=collector.collect_window
    with patch.object(collector,'time',clock),patch.object(collector,'collect_window',
            side_effect=lambda *a,**kw:original(*a,clock=clock.monotonic,**kw)):
        if collector.collect(args,prompt=prompt,source=source):
            raise RuntimeError('synthetic fixture was rejected')
    # A malformed pose label must remain a failure rather than silently accepted.
    from Interaction.estimator_imu_calibration import analyze_file
    bad=json.loads((output/'dataset.json').read_text())
    if calibration_method == 'gravity_norm':
        bad['poses'][1]['accel_m_s2'] = deepcopy(bad['poses'][0]['accel_m_s2'])
    else:
        bad['poses'][0]['reference_force_m_s2'],bad['poses'][1]['reference_force_m_s2']=(
            bad['poses'][1]['reference_force_m_s2'],bad['poses'][0]['reference_force_m_s2'])
    write_json(output/'bad_pose_dataset.json',bad)
    rejected=analyze_file(output/'bad_pose_dataset.json',output/'rejected_bad_pose',require_validation=False)
    assert not rejected.get('fit_passed')


class Plant:
    """Position feedback and attitude lag, with independent IMU-specific force.

    The deterministic ordinary-reference channel is ideal truth here. Real
    firmware PID, motors and ordinary KF are deliberately not modeled. The
    commander priority latch starts with LL ownership, as it can after a
    previous flight without reboot; HLC plans wait for priority release.
    """
    def __init__(self,clock,A,b,bg,seed,flight_accel_bias=None):
        self.clock=clock;self.A=A;self.b=b;self.bg=bg;self.rng=np.random.default_rng(seed)
        self.flight_accel_bias=np.zeros(3) if flight_accel_bias is None else np.asarray(flight_accel_bias,float)
        self.p=np.array([0.,-1.,.24]);self.v=np.zeros(3);self.target=self.p.copy()
        self.rpy=np.zeros(3);self.blocks=[];self.owner=None;self.steps=0;self.armed=False
        self.commands=[];self.truth=[]
        self.low_level_priority=True;self.planned_target=None
        self.armed_at=None
    def command(self,kind,position=None):
        self.commands.append({'time_s':self.clock.t,'kind':kind,'position_m':position})
        if kind == 'position':
            self.low_level_priority=True;self.planned_target=None
            self.target=np.array(position)
        elif kind in ('takeoff','go_to','land'):
            self.planned_target=np.array(position)
            if not self.low_level_priority:self.target=self.planned_target.copy()
        elif kind == 'notify_stop':
            self.low_level_priority=False
            if self.planned_target is not None:self.target=self.planned_target.copy()
        elif kind == 'hlc_stop':
            self.planned_target=None;self.target=self.p.copy()
    def block(self,name,period):
        block=fls_log_config(name,period)
        assert block.period==period  # actual wire byte interpreted in ms by FLS
        block.start=lambda:self.blocks.append(block)
        block.stop=lambda:self.blocks.remove(block)
        block.delete=lambda:None
        return block
    def step(self,dt):
        acceleration=np.clip(8*(self.target-self.p)-5*self.v,-2,2)
        if not self.armed:acceleration=np.zeros(3)
        previous=self.rpy.copy()
        target_rpy=np.array([-acceleration[1]/G,acceleration[0]/G,0.])
        self.rpy+=(target_rpy-self.rpy)*min(1,dt/.06)
        # Convert Euler derivatives to body gyro; these differ at nonzero tilt.
        derivative=(self.rpy-previous)/dt;r,p,_=self.rpy
        body_rate=np.array([[1,0,-np.sin(p)],[0,np.cos(r),np.sin(r)*np.cos(p)],
            [0,-np.sin(r),np.cos(r)*np.cos(p)]])@derivative
        self.p+=self.v*dt+.5*acceleration*dt*dt;self.v+=acceleration*dt
        R=flight.rotation(quaternion(self.rpy))
        extra=self.flight_accel_bias if self.armed else np.zeros(3)
        acc=self.A@(R.T@(acceleration+np.array([0,0,G]))+extra)+self.b+self.rng.normal(0,.005,3)
        gyro=body_rate+self.bg+self.rng.normal(0,1e-5,3)
        q=quaternion(self.rpy)
        data={'imu':{**dict(zip(collector.IMU_FIELDS[:3],acc/G)),
                     **dict(zip(collector.IMU_FIELDS[3:],np.rad2deg(gyro)))},
            'kf':{f'kalman.q{i}':float(q[i]) for i in range(4)},
            'state':dict(zip([f'stateEstimate.{a}' for a in ('x','y','z','vx','vy','vz')],np.r_[self.p,self.v])),
            'health':{'pm.vbat':8.,'supervisor.info':
                      (10 if self.armed_at is not None and self.clock.t-self.armed_at >= 1.4 else 2) if self.armed else 1,
                      **{f'motor.m{i}':30000 if self.armed else 0 for i in range(1,5)}}}
        self.steps+=1
        # Causal reference packets precede same-tick IMU, as transport can do.
        for block in sorted(self.blocks,key=lambda b:b.name.endswith('imu')):
            if round(self.clock.t*1000)%block.period==0:
                group=block.name.removeprefix('imuval_')
                block.data_received_cb.call(self.clock.tick(),data[group],block)
        if self.owner and getattr(self.owner,'_imu_validation_capture',None):
            self.owner._imu_validation_capture.on_pose({'tvec':self.p.tolist(),
                'frame_id':self.steps,'time':self.clock.time()})
        self.truth.append({'time_s':self.clock.t,'position_m':self.p.tolist(),
            'velocity_m_s':self.v.tolist(),'rpy_deg':np.rad2deg(self.rpy).tolist()})


def controller_methods(clock,events):
    """Execute production lifecycle methods without importing hardware modules."""
    source=ROOT/'controller.py';tree=ast.parse(source.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Controller')
    names={'start','arm','takeoff','land','handshake','run_mission'}
    functions=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in names]
    class Wrapper:pass
    class HandoffError(Exception):pass
    def handoff(low,high,method,*args,**kwargs):
        kwargs.pop('dry_run',None);getattr(high,method)(*args,**kwargs)
        events.append({'event':'simulated_HLC_ACK','method':method});low.send_notify_setpoint_stop()
    namespace={'time':clock,'logger':logging.getLogger('sim-controller'),'math':__import__('math'),
        'CommandWrapper':Wrapper,'handoff_to_high_level':handoff,'HandoffError':HandoffError,
        'input':lambda _:(_ for _ in ()).throw(AssertionError('Pi must not prompt during orchestrated launch'))}
    exec(compile(ast.Module(body=functions,type_ignores=[]),str(source),'exec'),namespace)
    return {name:namespace[name] for name in names}


def run_case(output,name,seed,nominal=False,*,calibration_method='six_face',reboot_after_fit=False,flight_accel_bias=None):
    sys.path.insert(0,str(ORCHESTRATOR))
    from imu_validation_mission import ImuValidationMission,retrieve_validation,OUTPUTS
    case=output/name;case.mkdir();base=case/'mac';remote=case/'pi';remote.mkdir()
    clock=Clock();A=rotation_xyz([0,0,0] if nominal else [3.,-1.,.7])@np.diag([1.01,.99,1.02] if not nominal else [1,1,1])
    if calibration_method == 'gravity_norm':
        A = np.diag([1.01,.99,1.02] if not nominal else [1,1,1])
    b=np.zeros(3) if nominal else np.array([.03,-.04,.02])
    bg=np.zeros(3) if nominal else np.deg2rad([.12,-.08,.05])
    source=base/'logs'/f'estimator_imu_lb11_20261009T000000_{seed}'
    source.parent.mkdir(parents=True)
    collect_fixture(source,clock,A,b,bg,seed,calibration_method=calibration_method)
    if reboot_after_fit:
        clock.device_offset_ms=2000-100_000-round((clock.t-1000.)*1000)
        bg=bg+np.deg2rad([.18,-.12,.09])
    # Latest selection must ignore a later rejected session.
    failed=base/'logs/estimator_imu_lb11_99999999';(failed/'fit').mkdir(parents=True)
    (failed/'dataset.json').write_text('{}');write_json(failed/'fit/candidate.json',{'fit_passed':False})
    args=SimpleNamespace(validate_estimator_imu='latest',drone_id='lb11',imu_flight_config=None)
    manifest={'drones':[{'id':'lb11','ip':'simulated','user':'simulated','init_pos':[0,-1,.24]}],
        'controller':{},'common':{}}
    plan=ImuValidationMission(args,manifest,base)
    class Connection:
        def run(self,command,**kwargs):
            # Only local simulation directories are ever created; no shell/SSH.
            if command.startswith('mkdir '):(remote/plan.remote_dir(name)/'fit').mkdir(parents=True)
            return SimpleNamespace(exited=0)
        def put(self,stream,remote):Path(remote).write_bytes(stream.read())
    events=[];settings=plan.stage(Connection(),remote,name,
        runtime_sync=lambda *a:{'simulated_transport':True})
    plant=Plant(clock,A,b,bg,seed+1000,flight_accel_bias);param=Param()
    ll=SimpleNamespace(send_position_setpoint=lambda x,y,z,yaw:plant.command('position',[x,y,z]),
        send_notify_setpoint_stop=lambda:plant.command('notify_stop'))
    high=SimpleNamespace(takeoff=lambda z,duration,yaw:plant.command('takeoff',[0,-1,z]),
        go_to=lambda x,y,z,yaw,duration,relative=False:plant.command('go_to',[x,y,z]),
        land=lambda z,duration:plant.command('land',[plant.p[0],plant.p[1],z]),
        stop=lambda:plant.command('hlc_stop'))
    def arming(enabled):
        plant.armed=enabled;plant.armed_at=clock.t if enabled else None
        events.append({'event':'arm','enabled':enabled})
    cf=SimpleNamespace(param=param,link=True,log=SimpleNamespace(add_config=lambda _:None),
        platform=SimpleNamespace(send_arming_request=arming))
    args=SimpleNamespace(imu_validation_session=str(remote/plan.remote_dir(name)),
        imu_validation_output=str(remote/plan.remote_dir(name)/'flight'),drone_id='lb11',
        init_pos=[0,-1,.24],takeoff_altitude=.84,init_yaw=0.,droneless=False,orchestrated=True,
        ground_test=False,skip_arm=False,skip_takeoff=False,skip_landing=False,vicon=True,
        autotune=False,simple_takeoff=False,rotation_test=False,xy_tune=False,z_tune=False,trajectory=None)
    owner=SimpleNamespace(args=args,cf=cf,ll_commander=ll,hl_commander=high,init_coord=[0,-1,.24],
        led=None,tracker=None,voltage=8.,mission=plan.mission,manifest=manifest,use_flowdeck=False,
        flying=False,mocap=True,firmware_auto_brake_enabled=False,_is_interaction_application=lambda:True)
    owner.log_manager=SimpleNamespace(start=lambda:events.append({'event':'log_start'}),
        add_log_entry=lambda *a:None,get_latest_cf_log_data=lambda group,key:float(plant.p['xyz'.index(key[-1])]))
    owner._send_landing_confirmation=lambda voltage:events.append({'event':'LANDED'})
    owner._get_latest_mocap_frame=lambda:{'tvec':plant.p.tolist()}
    def safe_sleep(duration):clock.sleep(duration);return True
    owner._safe_sleep=safe_sleep
    owner.push_socket=SimpleNamespace(send_json=lambda data:events.extend([data,{'cmd':'START'}]))
    methods=controller_methods(clock,events)
    start_tree=ast.parse((ROOT/'controller.py').read_text())
    cls=next(n for n in start_tree.body if isinstance(n,ast.ClassDef) and n.name=='Controller')
    start=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='start')
    for n in ast.walk(start):
        if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and isinstance(n.func.value,ast.Name) and n.func.value.id=='self':
            if not hasattr(owner,n.func.attr):setattr(owner,n.func.attr,lambda:None)
    for key,method in methods.items():setattr(owner,key,method.__get__(owner))
    plant.owner=owner;clock.on_step=plant.step
    with ExitStack() as stack:
        stack.enter_context(patch.object(task,'time',clock))
        stack.enter_context(patch.object(flight,'time',clock))
        stack.enter_context(patch.object(task,'fls_log_config',plant.block))
        try:
            owner.start();owner.land()
        finally:
            capture=getattr(owner,'_imu_validation_capture',None)
            if capture:capture.finish(not owner.flying)
    assert not owner.flying and plant.commands[-1]['kind']=='hlc_stop'
    assert all(k!='stabilizer.estimator' or v=='2' for k,v in param.writes)
    report={}
    def replay(local,_base):
        report.update(run_replay(local/'flight/packets.jsonl',local/'fit/candidate.json',local/'flight/replay'))
    def download(drone,path,local,label,required):
        shutil.copyfile(path,local);return True
    assert retrieve_validation({'imu_validation':settings},{'id':'lb11'},download,logging.getLogger(name),replay=replay)
    local=Path(settings['local_dir'])
    packets=[json.loads(line) for line in (local/'flight/packets.jsonl').read_text().splitlines()]
    assert any(r['group']=='event' and r['data'].get('name')=='capture_ready' for r in packets)
    counts={group:sum(r['group']==group for r in packets) for group in ('imu','kf','state','health','vicon')}
    stats={}
    for mode in ('raw','corrected'):
        samples=[r for r in report[mode]['samples'] if r['phase'] not in ('preflight','takeoff','land')]
        errors=np.asarray([r['error_deg'] for r in samples])
        stats[mode]={'rmse_rpy_deg':np.sqrt(np.mean(errors**2,axis=0)).tolist(),
            'segments':report[mode]['segments'],'samples':len(samples),
            'position_fusions':samples[-1]['position_fusions']}
        assert report[mode]['segments']==1
        assert len(report[mode]['by_phase'])==18
    write_json(case/'lifecycle.json',{'events':events,'commands':plant.commands,'parameter_writes':param.writes,
        'log_counts':counts,'flight_capture_completed':json.loads((local/'flight/report.json').read_text())['capture_completed']})
    write_json(case/'independent_truth.json',plant.truth)
    write_json(case/'injected_fault.json',{'A':A.tolist(),'bias':b.tolist(),'gyro_bias':bg.tolist(),
        'fixture_seed':seed,'flight_seed':seed+1000,'flight_accel_bias_m_s2':plant.flight_accel_bias.tolist()})
    result={'name':name,'nominal':nominal,'calibration_method':calibration_method,**stats,'local_results':str(local),
        'flight_phases':len(report['corrected']['by_phase']), 'capture_download_replay_completed':True}
    write_json(case/'summary.json',result);plan.directory.cleanup()
    return result


def run(output, *, calibration_method='six_face'):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    results=[]
    for name,seed,nominal in [('known_fault_101',101,False),('known_fault_202',202,False),
                              ('known_fault_303',303,False),('nominal_404',404,True)]:
        print('[IMU simulation]',name,flush=True);results.append(run_case(output,name,seed,nominal,
            calibration_method=calibration_method))
    fault_cases=[r for r in results if not r['nominal']]
    improvement=all(np.linalg.norm(r['corrected']['rmse_rpy_deg'][:2])<
        .25*np.linalg.norm(r['raw']['rmse_rpy_deg'][:2]) for r in fault_cases)
    nominal=results[-1]
    no_regression=np.linalg.norm(nominal['corrected']['rmse_rpy_deg'][:2])<.1
    report={'completed':True,'known_fault_improves':bool(improvement),'nominal_no_material_regression':bool(no_regression),
        'hardware_contacted':False,'firmware_calibration_applied':False,'results':results,
        'calibration_method':calibration_method,
        'scope':'Reduced plant + actual static collector, saved-fit selection, production Controller lifecycle methods, task, download integrity checks and frozen native C estimator.',
        'limitations':['Hardware transport, Dispatcher UI selection and ACKs are simulated.',
            'Reference is ideal simulated attitude, not the real ordinary estimator 2.',
            'Plant/PID are reduced; this does not validate active estimator-3 control or hardware flight.',
            'Known imposed fault establishes conditional repair; it does not identify the real drone fault.'],
        'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            [Path(__file__),ROOT/'controller.py',ROOT/'Interaction/estimator_validation_controller.py',
             ROOT/'Interaction/calibrate_estimator_imu.py',ROOT/'Interaction/replay_estimator3.py',
             ROOT/'Interaction/estimator_gravity_calibration.py',ROOT/'Interaction/estimator_imu_calibration.py']}}
    if calibration_method == 'gravity_norm':
        report['limitations'].append('Injected faults are diagonal scale and bias only. Mounting rotation is not calibrated by gravity magnitude.')
    write_json(output/'summary.json',report)
    lines=['# IMU calibration end-to-end simulation','',report['scope'],'',
        '| Case | Raw roll / pitch RMSE (deg) | Corrected roll / pitch RMSE (deg) | Segments raw/corrected |',
        '| --- | --- | --- | --- |']
    for r in results:
        values=[' / '.join(f'{v:.4f}' for v in r[m]['rmse_rpy_deg'][:2]) for m in ('raw','corrected')]
        lines.append(f'| {r["name"]} | {values[0]} | {values[1]} | {r["raw"]["segments"]}/{r["corrected"]["segments"]} |')
    lines+=['',f'Known fault improvement passed: {improvement}',f'Nominal screen passed: {no_regression}','',*report['limitations']]
    (output/'report.md').write_text('\n'.join(lines)+'\n')
    if not improvement or not no_regression:raise RuntimeError('simulation did not pass its calibration efficacy screen')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path,help='new output directory')
    parser.add_argument('--calibration-method',choices=('six_face','gravity_norm'),default='six_face')
    args=parser.parse_args();run(args.output,calibration_method=args.calibration_method)
