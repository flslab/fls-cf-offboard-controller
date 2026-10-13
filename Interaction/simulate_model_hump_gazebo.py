"""Container-only Gazebo contact capture using real SITL PID/default KF.

Runs production motor model, momentum observer, onset and release detector.
Position commands precede detected contact; level orientation commands follow.
No S-curve is issued: post-release residual is isolated from brake actuation.
Only localhost UDP; no hardware, SSH, or production calibration writes.
"""
from pathlib import Path
import argparse,ast,hashlib,json,os,sys,subprocess,threading,time
import numpy as np

LAUNCH=Path('/workspace/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch')
sys.path.insert(0,str(LAUNCH))
import contact_attitude_ekf_capture as gz
from Interaction.external_wrench_observer import MotorWrenchModel,quaternion_to_euler,body_z_world
from Interaction.onboard_wrench_interaction_pipeline import FiniteWindowMomentumForceEstimator
from Interaction.wrench_contact_detector import ContactChannelDetector
from Interaction.imu_logging import fls_log_config
from Interaction.specific_force_contact_observer import SpecificForceContactObserver
from Interaction.prototype_thrust_curve import PrototypeThrustCurve

def main(args):
    if not Path('/.dockerenv').exists():raise RuntimeError('Container-only simulation')
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    cfg=json.loads(Path(args.config).read_text())
    wc=cfg['wrench_interaction'];axis='XY'.index(args.direction[-1]);sign=1 if args.direction[0]=='+' else -1
    force=np.zeros(3);force[axis]=sign*args.force if not args.calibration else 0
    torque=np.zeros(3);torque[1-axis]=(sign if axis==0 else -sign)*args.torque if not args.calibration else 0
    processes=[];streams=[];scf=None;stop=threading.Event();latest={};rows=[];trace=[];errors=[];events=[]
    pose_thread=None;trigger_thread=None;trigger_errors=[];flying=False;detected=False;released=False;blocks=[]
    env=os.environ.copy();env.update(GZ_SIM_SYSTEM_PLUGIN_PATH='/workspace/.codex-fw-shadow-sitl/sitl_make/build-auto-170/build_crazysim_gz',
       GZ_SIM_RESOURCE_PATH=str(gz.GAZEBO_ROOT/'models')+':'+str(gz.GAZEBO_ROOT/'worlds'),LIBGL_ALWAYS_SOFTWARE='1',CF2_SIM_MODEL='gz_crazyflie')
    firmware=Path('/master-sitl/sitl_make/build-selection/cf2');model=out/'model.sdf';world=out/'world.sdf';odom=out/'raw_odom.jsonl';status=out/'raw_status.jsonl'
    try:
        gz.run_checked([sys.executable,str(LAUNCH/'jinja_gen.py'),str(gz.GAZEBO_ROOT/'models/crazyflie/model.sdf.jinja'),str(gz.GAZEBO_ROOT),
           '--cf_id','0','--cffirm_udp_port','19850','--cflib_udp_port','19851','--cf_name','cf','--external-pose-mode','position_only',
           '--airframe-profile','fls170','--fls-asset-root','/fls-assets','--output-file',str(model)],env=env)
        if args.imu_noise_std>0:
            mt=gz.ET.parse(model);sensor=mt.find('.//sensor[@type="imu"]');imu=gz.ET.SubElement(sensor,'imu')
            for channel,std in [('linear_acceleration',args.imu_noise_std),('angular_velocity',.003)]:
                group=gz.ET.SubElement(imu,channel)
                for axis_name in 'xyz':
                    noise=gz.ET.SubElement(gz.ET.SubElement(group,axis_name),'noise',type='gaussian')
                    gz.ET.SubElement(noise,'mean').text='0';gz.ET.SubElement(noise,'stddev').text=str(std)
            mt.write(model,encoding='unicode',xml_declaration=True)
        tree=gz.ET.parse(gz.GAZEBO_ROOT/'worlds/contact_brake_shadow.sdf')
        scheduler=next(p for p in tree.findall('.//plugin') if p.get('filename')=='gz_contact_wrench_scheduler')
        scheduler.find('duration').text=str(args.duration);scheduler.find('force').text=' '.join(map(str,force));scheduler.find('torque').text=' '.join(map(str,torque))
        tree.write(world,encoding='unicode',xml_declaration=True)
        stream=(out/'gazebo.log').open('w');streams.append(stream)
        processes.append(subprocess.Popen(['stdbuf','-oL','-eL','gz','sim','-s',str(world),'-v','3'],env=env,stdout=stream,stderr=subprocess.STDOUT))
        gz.wait_for_service('/world/contact_brake_shadow/create',env)
        gz.run_checked(['gz','service','-s','/world/contact_brake_shadow/create','--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','10000','--req',f'sdf_filename: "{model}", name: "crazyflie_0", pose: {{position: {{z: 0.5}}}}'],env=env)
        gz.wait_for_text(out/'gazebo.log','External localization input mode: position_only',8)
        for topic,path in [('/cf_0/odom',odom),('/world/contact_brake_shadow/contact_wrench/status',status)]:
            process,stream=gz.start_topic_capture(topic,path);processes.append(process);streams.append(stream)
        stream=(out/'firmware.log').open('w');streams.append(stream)
        processes.append(subprocess.Popen(['stdbuf','-oL','-eL',str(firmware),'19850'],env=env,stdout=stream,stderr=subprocess.STDOUT))
        gz.wait_for_text(out/'firmware.log','Connection established with gazebo',8)
        gz.probe_crtp(19851);gz.world_control('contact_brake_shadow','pause: false',env)
        gz.wait_for_text(out/'firmware.log','Starting stabilizer loop',12);gz.wait_for_stationary_ground(odom)
        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
        cflib.crtp.init_drivers();cache=out/'cache';cache.mkdir(exist_ok=True)
        scf=SyncCrazyflie('udp://127.0.0.1:19851',cf=Crazyflie(rw_cache=str(cache)));scf.open_link();cf=scf.cf
        deadline=time.monotonic()+12
        while not cf.param.is_updated:
            if time.monotonic()>deadline:raise TimeoutError('parameter snapshot unavailable')
            time.sleep(.05)
        for key,value in {'commander.enHighLevel':1,'stabilizer.estimator':2,'stabilizer.controller':1,'locSrv.extPosStdDev':.001}.items():cf.param.set_value(key,str(value))
        for key in ('hlCommander.pRelAuto','kalmanPRel.enable','kalmanPRel.scEnable'):
            group,name=key.split('.')
            if name in cf.param.toc.toc.get(group,{}):cf.param.set_value(key,'0')
        ctree=ast.parse(Path('/offboard/config.py').read_text())
        pid=ast.literal_eval(next(n.value for n in ctree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='PID_VALUES' for t in n.targets)))
        for key,value in pid.items():cf.param.set_value(key,str(value))
        def forward():
            previous=None
            while not stop.is_set():
                try:
                    truth=gz.latest_odom(odom);latest['truth']=truth
                    if truth['sim_time_ns']!=previous:cf.extpos.send_extpos(*truth['position_m']);previous=truth['sim_time_ns']
                except (ValueError,OSError):pass
                except BaseException as e:errors.append(str(e));return
                stop.wait(.003)
        pose_thread=threading.Thread(target=forward,daemon=True);pose_thread.start()
        groups={'kin':['stateEstimate.'+a for a in ['x','y','z','vx','vy','vz']],
                'att':['kalman.q'+str(i) for i in range(4)],
                'act':['motor.m'+str(i) for i in range(1,5)]+['pm.vbat'],
                'imu':['acc.'+a for a in 'xyz']+['gyro.'+a for a in 'xyz']}
        for name,variables in groups.items():
            block=fls_log_config('Hump_'+name,args.log_period_ms)
            for v in variables:block.add_variable(v)
            block.add_variable('simClock.ms16')
            def callback(tick,data,_block,name=name):
                row={'group':name,'tick_ms':tick,'received_monotonic':time.monotonic(),'data':dict(data),'truth':latest.get('truth')}
                latest[name]=row;rows.append(row)
            cf.log.add_config(block);block.data_received_cb.add_callback(callback);block.start();blocks.append(block)
        time.sleep(1);ground=gz.latest_odom(odom)['position_m']
        cf.param.set_value('kalman.initialX',str(ground[0]));cf.param.set_value('kalman.initialY',str(ground[1]));cf.param.set_value('kalman.initialZ',str(ground[2]))
        cf.param.set_value('kalman.resetEstimation','1');time.sleep(.1);cf.param.set_value('kalman.resetEstimation','0');time.sleep(1)
        cf.platform.send_arming_request(True);time.sleep(1);cf.high_level_commander.takeoff(.8,2.);flying=True;time.sleep(2)
        hover=gz.wait_for_hover(odom,target_height_m=.8,window_s=1.,height_tolerance_m=.1,max_speed_m_s=.12,timeout_s=15.)
        print('hover passed',flush=True)
        # Independent simulator hover normalization; never write the hardware fit.
        recent=[r['data'] for r in rows if r['group']=='act' and time.monotonic()-r['received_monotonic']<1]
        pwm=np.array([[r['motor.m'+str(i)] for i in range(1,5)] for r in recent]);voltage=float(np.median([r['pm.vbat'] for r in recent]))
        model_cfg={k:v for k,v in wc['motor_model'].items() if k!='yaw_command_model'}
        model_cfg.update(hover_pwm=float(np.sqrt(np.mean(pwm.astype(float)**2))),hover_voltage=voltage)
        motor=MotorWrenchModel(**model_cfg);observer=FiniteWindowMomentumForceEstimator(mass=wc['mass'],**wc['impulse_estimator'])
        specific=SpecificForceContactObserver(wc['mass'],imu_from_com_m=cfg.get('_sim_imu_from_com_m',(0,0,0)))
        thrust_curve=PrototypeThrustCurve(model_cfg['hover_pwm'],voltage,**cfg['_sim_thrust_curve']) if '_sim_thrust_curve' in cfg else None
        last_specific_time=None;specific_estimate=None
        detector=ContactChannelDetector(**wc['detection']['translation']);last_sample_time=None;bias=np.zeros(3);bias_rows=[];enabled=False;triggered=None
        loop_start=time.monotonic();target=[ground[0],ground[1],.8];debug_next=loop_start;updates=0
        while time.monotonic()-loop_start<15:
            if args.calibration and triggered:
                elapsed=time.monotonic()-triggered
                if args.collective_calibration:
                    z=.8+.15*np.sin(2*np.pi*.6*elapsed) if elapsed<4 else .8
                    cf.commander.send_position_setpoint(ground[0],ground[1],z,0)
                else:
                    angle=3*np.sin(2*np.pi*.8*elapsed) if elapsed<4 else 0
                    cf.commander.send_zdistance_setpoint(angle if axis==1 else 0,angle if axis==0 else 0,0,.8)
            elif detected:cf.commander.send_zdistance_setpoint(0,0,0,.8)
            elif enabled:cf.commander.send_position_setpoint(*target,0)
            if all(g in latest for g in ('kin','att','act','imu')):
                k,a,m,im=[latest[g] for g in ('kin','att','act','imu')];tick=k['tick_ms']
                sample_time=k['data']['simClock.ms16']/1000.
                group_times={g:latest[g]['data']['simClock.ms16']/1000. for g in ('kin','att','act','imu')}
                if (last_sample_time is None or sample_time>last_sample_time) and max(abs(sample_time-z) for z in group_times.values())<=.025:
                    # CRTP log ticks can advance while the simulator acquisition
                    # epoch is unchanged. Do not reset observers with duplicate
                    # sensor samples, and retain not-ready samples in the trace.
                    last_sample_time=sample_time;q=np.array([a['data']['kalman.q'+str(i)] for i in range(4)])
                    rpy=quaternion_to_euler(q[[1,2,3,0]])*[1,-1,1]
                    v=np.array([k['data']['stateEstimate.v'+x] for x in 'xyz'])
                    expected,_=motor.expected_accelerations(rpy,[m['data']['motor.m'+str(i)] for i in range(1,5)],m['data']['pm.vbat'])
                    if thrust_curve:
                        T=thrust_curve.update([m['data']['motor.m'+str(i)] for i in range(1,5)],m['data']['pm.vbat'],sample_time)
                        direction=body_z_world(motor.roll_sign_for_world_thrust*rpy[0],motor.pitch_sign_for_world_thrust*rpy[1],rpy[2])
                        expected=T*direction-[0,0,9.81]
                    momentum_estimate=observer.update(v,expected,sample_time)
                    ir=im['data']
                    imu_time=ir['simClock.ms16']/1000.
                    if last_specific_time is None or imu_time>last_specific_time:
                        specific_estimate=specific.update([ir['acc.'+x]*9.81 for x in 'xyz'],q[[1,2,3,0]],float(np.linalg.norm(expected+[0,0,9.81])),imu_time,np.deg2rad([ir['gyro.'+x] for x in 'xyz']))
                        last_specific_time=imu_time
                    updates+=1
                    estimate=specific_estimate if args.observer_method=='specific_force' else momentum_estimate
                    f=estimate.external_force-bias;decision=None
                    if estimate.ready:
                        if not enabled:bias_rows.append(estimate.external_force.tolist())
                        if enabled and not released and (not args.calibration or args.evaluate_calibration_detection):
                            decision=detector.update(f,np.diag([.02**2,.02**2,.03**2]),sample_time)
                            if decision.started:
                                detected=True;events.append({'name':'detected','tick_ms':tick,'sample_sim_time_s':sample_time,'truth_sim_time_ns':latest['truth']['sim_time_ns']});print('detected',flush=True)
                            if decision.ended:
                                released=True;events.append({'name':'release_confirmed','tick_ms':tick,'sample_sim_time_s':sample_time,'truth_sim_time_ns':latest['truth']['sim_time_ns']});print('release confirmed',flush=True)
                    trace.append({'tick_ms':tick,'sample_sim_time_s':sample_time,'received_monotonic':time.monotonic(),'velocity':v.tolist(),'attitude_model_rpy':np.asarray(rpy).tolist(),'expected_acceleration':expected.tolist(),
                                  'force':f.tolist(),'truth':latest.get('truth'),'enabled':enabled,'detected':detected,'released':released,
                                  'ready':bool(estimate.ready),'group_sample_times_s':group_times,'dwell_cancelled':bool(decision and decision.release_candidate_cancelled)})
            if not enabled and len(bias_rows)>=100:
                bias=np.median(bias_rows[-80:],axis=0);enabled=True
                # The gz CLI takes ~200 ms to start/service. It must never
                # interrupt observer sampling or command transmission.
                def trigger():
                    try:gz.trigger_wrench(env)
                    except BaseException as e:trigger_errors.append(str(e))
                triggered=time.monotonic();trigger_thread=threading.Thread(target=trigger,daemon=True);trigger_thread.start()
                print('force request dispatched',flush=True)
            if trigger_errors:raise RuntimeError('force trigger failed: '+trigger_errors[0])
            if time.monotonic()>debug_next:
                print(json.dumps({'updates':updates,'bias_samples':len(bias_rows),'latest_sample_s':latest.get('kin',{}).get('data',{}).get('simClock.ms16'),'groups':{g:latest.get(g,{}).get('tick_ms') for g in ['kin','att','act','imu']}}),flush=True)
                debug_next=time.monotonic()+2
            if triggered and time.monotonic()-triggered>(5.5 if args.calibration else 2.5):break
            time.sleep(.005)
        if triggered is None:raise RuntimeError('model initialization failed')
        scheduler_events=gz.parse_scheduler_events(status)
        if len(scheduler_events)<2:raise RuntimeError('physical force release not recorded')
        cf.commander.send_notify_setpoint_stop();cf.high_level_commander.land(ground[2],2.);time.sleep(2.3);cf.platform.send_arming_request(False);flying=False
        document={'completed':True,'direction':args.direction,'calibration':args.calibration,'force_N':force.tolist(),'torque_Nm':torque.tolist(),'duration_s':args.duration,
            'config':cfg,'observer_method':args.observer_method,'sim_motor_model':model_cfg,'startup_bias_N':bias.tolist(),'hover':hover,'scheduler_events':scheduler_events,'events':events,'errors':errors,
            'imu_noise_std_m_s2':args.imu_noise_std,
            'log_period_ms':args.log_period_ms,
            'firmware_sha256':hashlib.sha256(firmware.read_bytes()).hexdigest(),'detected':detected,'released':released,
            'source_sha256':{name:hashlib.sha256((Path('/offboard/Interaction')/name).read_bytes()).hexdigest() for name in ['simulate_model_hump_gazebo.py','specific_force_contact_observer.py','prototype_thrust_curve.py']},
            'notes':['Gazebo FLS170 real PID/KF, position-only localization. Production observer/detector primitives; reduced interaction harness, not full orchestrator.',
                     'Orientation hold continues after release to isolate the residual; S-curve is deliberately absent.',
                     'Model is normalized using simulator hover; aircraft calibration is not transplanted as a simulator thrust truth.']}
        (out/'summary.json').write_text(json.dumps(document,indent=2));print(json.dumps({k:document[k] for k in ['detected','released','scheduler_events','events']}),flush=True)
    except BaseException as e:
        (out/'failure.json').write_text(json.dumps({'error':str(e),'events':events,'errors':errors},indent=2));raise
    finally:
        (out/'telemetry.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
        (out/'observer.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in trace))
        if scf:
            if flying:
                try:scf.cf.commander.send_notify_setpoint_stop();scf.cf.high_level_commander.land(0,1);time.sleep(1.2)
                except BaseException:pass
            try:scf.cf.commander.send_stop_setpoint();scf.cf.platform.send_arming_request(False)
            except BaseException:pass
            stop.set()
            if pose_thread:pose_thread.join(timeout=2)
            if trigger_thread:trigger_thread.join(timeout=2)
            scf.close_link()
        for p in reversed(processes):gz.stop_process(p)
        for s in streams:
            if not s.closed:s.close()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--config',required=True)
    p.add_argument('--direction',choices=['+X','-X','+Y','-Y'],default='+Y');p.add_argument('--force',type=float,default=.6);p.add_argument('--torque',type=float,default=.008);p.add_argument('--duration',type=float,default=.18);p.add_argument('--calibration',action='store_true')
    p.add_argument('--observer-method',choices=['momentum','specific_force'],default='momentum')
    p.add_argument('--imu-noise-std',type=float,default=0.)
    p.add_argument('--evaluate-calibration-detection',action='store_true')
    p.add_argument('--collective-calibration',action='store_true')
    p.add_argument('--log-period-ms',type=int,choices=[10,20],default=10)
    main(p.parse_args())
