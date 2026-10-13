"""Container-only FLS170 SITL validation flight using the production IMU task.

Only localhost UDP is opened. Gazebo position replaces the Vicon callback;
normal Controller arm/takeoff/landing and real firmware PID/default KF run.
The UI, SSH and peripheral initialization are covered separately by the
transport integration simulation. No contact or onboard estimator 3 runs.
"""
import ast
from copy import deepcopy
import hashlib
import json
import logging
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

LAUNCH=Path('/workspace/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch')
sys.path[:0]=[str(LAUNCH),'/offboard']
import contact_attitude_ekf_capture as gazebo
from Interaction import estimator_validation_controller as task
from Interaction.estimator_imu_calibration import G, SCHEMA, default_plan, analyze_file, write_json
from Interaction.simulate_estimator_imu import controller_methods,rotation_xyz
from Interaction.command_wrapper import CommandWrapper
from Interaction.commander_handoff import handoff_to_high_level,HandoffError
from Interaction.replay_estimator3 import run_replay


def main(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    processes=[];streams=[];scf=None;owner=None;stopped=threading.Event();pose_thread=None
    env=os.environ.copy()
    plugins='/workspace/.codex-fw-shadow-sitl/sitl_make/build-auto-170/build_crazysim_gz'
    env.update(GZ_SIM_SYSTEM_PLUGIN_PATH=plugins,GZ_SIM_RESOURCE_PATH=str(gazebo.GAZEBO_ROOT/'models')+':'+str(gazebo.GAZEBO_ROOT/'worlds'),
        LIBGL_ALWAYS_SOFTWARE='1',CF2_SIM_MODEL='gz_crazyflie')
    firmware=Path('/master-sitl/sitl_make/build-selection/cf2')
    source=output/'fixture';(source/'fit').mkdir(parents=True,exist_ok=True)
    model=output/'model.sdf';world=output/'world.sdf';odom=output/'raw_odom.jsonl'
    failure=None;events=[]
    try:
        gazebo.run_checked([sys.executable,str(LAUNCH/'jinja_gen.py'),
            str(gazebo.GAZEBO_ROOT/'models/crazyflie/model.sdf.jinja'),str(gazebo.GAZEBO_ROOT),
            '--cf_id','0','--cffirm_udp_port','19850','--cflib_udp_port','19851','--cf_name','cf',
            '--external-pose-mode','position_only','--airframe-profile','fls170',
            '--fls-asset-root','/fls-assets','--output-file',str(model)],env=env)
        tree=gazebo.ET.parse(gazebo.GAZEBO_ROOT/'worlds/contact_brake_shadow.sdf')
        tree.find('.//physics/real_time_factor').text='1'
        tree.write(world,encoding='unicode',xml_declaration=True)
        stream=(output/'gazebo.log').open('w');streams.append(stream)
        processes.append(subprocess.Popen(['stdbuf','-oL','-eL','gz','sim','-s',str(world),'-v','3'],
            env=env,stdout=stream,stderr=subprocess.STDOUT))
        gazebo.wait_for_service('/world/contact_brake_shadow/create',env)
        gazebo.run_checked(['gz','service','-s','/world/contact_brake_shadow/create',
            '--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','10000',
            '--req',f'sdf_filename: "{model}", name: "crazyflie_0", pose: {{position: {{z: 0.5}}}}'],env=env)
        gazebo.wait_for_text(output/'gazebo.log','External localization input mode: position_only',8)
        process,stream=gazebo.start_topic_capture('/cf_0/odom',odom);processes.append(process);streams.append(stream)
        stream=(output/'firmware.log').open('w');streams.append(stream)
        processes.append(subprocess.Popen(['stdbuf','-oL','-eL',str(firmware),'19850'],env=env,
            stdout=stream,stderr=subprocess.STDOUT))
        gazebo.wait_for_text(output/'firmware.log','Connection established with gazebo',8)
        protocol=gazebo.probe_crtp(19851)
        gazebo.world_control('contact_brake_shadow','pause: false',env)
        gazebo.wait_for_text(output/'firmware.log','Starting stabilizer loop',12)
        gazebo.wait_for_stationary_ground(odom)
        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
        cflib.crtp.init_drivers();cache=output/'cache';cache.mkdir()
        scf=SyncCrazyflie('udp://127.0.0.1:19851',cf=Crazyflie(rw_cache=str(cache)))
        scf.open_link();cf=scf.cf
        deadline=time.monotonic()+10
        while not cf.param.is_updated:
            if time.monotonic()>deadline:raise TimeoutError('SITL parameter snapshot unavailable')
            time.sleep(.05)
        cf.param.set_value('commander.enHighLevel','1')
        cf.param.set_value('stabilizer.estimator','2');cf.param.set_value('stabilizer.controller','1')
        cf.param.set_value('locSrv.extPosStdDev','.001')
        for key in ('hlCommander.pRelAuto','kalmanPRel.enable','kalmanPRel.scEnable'):
            if key.split('.')[1] in cf.param.toc.toc.get(key.split('.')[0],{}):cf.param.set_value(key,'0')
        tree=ast.parse(Path('/offboard/config.py').read_text())
        pid=ast.literal_eval(next(n.value for n in tree.body if isinstance(n,ast.Assign)
            and any(isinstance(t,ast.Name) and t.id=='PID_VALUES' for t in n.targets)))
        for key,value in pid.items():cf.param.set_value(key,str(value))
        ground=gazebo.latest_odom(odom)['position_m']
        args=SimpleNamespace(imu_validation_session=str(source),imu_validation_output=str(output/'flight'),
            drone_id='lb11',init_pos=list(ground),takeoff_altitude=ground[2]+.6,init_yaw=0.,
            droneless=False,orchestrated=True,ground_test=False,skip_arm=False,skip_takeoff=False,
            skip_landing=False,vicon=True,crazysim=False,autotune=False,simple_takeoff=False,
            rotation_test=False,xy_tune=False,z_tune=False,trajectory=None)
        owner=SimpleNamespace(args=args,cf=cf,ll_commander=cf.commander,hl_commander=cf.high_level_commander,
            init_coord=list(ground),led=None,tracker=None,voltage=4.2,mission={'takeoff_speed':.5},
            manifest={'mission':{'delta_t':0}},use_flowdeck=False,flying=False,mocap=True,
            firmware_auto_brake_enabled=False,_is_interaction_application=lambda:True)
        owner.verify_contact_attitude_final_prearm_ready=lambda:None
        owner.log_manager=SimpleNamespace(start=lambda:None,add_log_entry=lambda *a:None,
            get_latest_cf_log_data=lambda group,key:None)
        owner._safe_sleep=lambda d:time.sleep(d) or True
        owner._get_latest_mocap_frame=lambda:{'tvec':gazebo.latest_odom(odom)['position_m']}
        owner._send_landing_confirmation=lambda v:events.append({'event':'LANDED','time':time.monotonic()})
        owner.push_socket=SimpleNamespace(send_json=lambda data:events.extend([data,{'cmd':'START'}]))
        methods=controller_methods(time,events)
        # Use the actual firmware command/ACK handoff instead of the reduced
        # plant's simulated ACK helper. No CRTP command is directed to hardware.
        for method in methods.values():
            method.__globals__.update(CommandWrapper=CommandWrapper,
                handoff_to_high_level=handoff_to_high_level,HandoffError=HandoffError)
        for key,method in methods.items():setattr(owner,key,method.__get__(owner))
        def forward():
            last=None
            try:
                while not stopped.is_set():
                    row=gazebo.latest_odom(odom)
                    if row['sim_time_ns']!=last:
                        last=row['sim_time_ns'];cf.extpos.send_extpos(*row['position_m'])
                        capture=getattr(owner,'_imu_validation_capture',None)
                        if capture:capture.on_pose({'tvec':row['position_m'],'time':time.time(),'frame_id':last})
                    time.sleep(.003)
            except BaseException as error:events.append({'pose_error':str(error)})
        pose_thread=threading.Thread(target=forward,daemon=True);pose_thread.start()
        # The known-fault fixture is generated independently of the flight.
        # Capture the device clock solely for the simulation continuity anchor.
        from Interaction.imu_logging import fls_log_config
        clock_block=fls_log_config('fixture_clock',20);clock_block.add_variable('gyro.x','float')
        ticks=[];cf.log.add_config(clock_block)
        clock_block.data_received_cb.add_callback(lambda tick,*_:ticks.append((tick,time.monotonic_ns())))
        clock_block.start();time.sleep(1)
        if not ticks:raise RuntimeError('no timestamped SITL clock')
        # Keep this tiny clock block alive until disconnect. The frozen SITL
        # worker can race an early stop/delete; this does not affect production.
        A=rotation_xyz([3,-1,.7])@np.diag([1.01,.99,1.02]);b=np.array([.03,-.04,.02]);bg=np.deg2rad([.12,-.08,.05])
        rng=np.random.default_rng(909);t=np.arange(400)*.01
        document={'schema':SCHEMA,'status':'complete','drone_id':'lb11','firmware_id':'frozen-fls170-sitl',
            'fixture_id':'synthetic-independent-known-fault','reference_note':'SIMULATION ONLY; logged-IMU fault, not injected into controlling KF',
            'reference_source':'independent_fixture','sensor_frame':'driver_processed_body','motors_off_confirmed':True,
            'capture':{'firmware_parameters_before':deepcopy(cf.param.values),'firmware_parameters_after':deepcopy(cf.param.values)},'poses':[]}
        for definition in default_plan()[:6]:
            acc=np.array(definition['reference_force_m_s2'])@A.T+b+rng.normal(0,.005,(400,3))
            document['poses'].append({**definition,'time_s':t.tolist(),'accel_m_s2':acc.tolist(),
                'gyro_rad_s':(bg+rng.normal(0,1e-5,(400,3))).tolist(),
                'cf_log_tick_ms_mod24':[(ticks[-1][0]-3990+i*10)%(1<<24) for i in range(400)],
                'host_receipt_monotonic_ns':[ticks[-1][1]-3_990_000_000+i*10_000_000 for i in range(400)]})
        write_json(source/'dataset.json',document)
        analyze_file(source/'dataset.json',source/'synthetic_fit',require_validation=False)
        for name in ('candidate.json','report.json','report.md'):
            (source/'fit'/name).write_bytes((source/'synthetic_fit'/name).read_bytes())
        original_builtin=task.builtin_flight_config
        def builtin(seed):
            config=original_builtin(seed);config['minimum_voltage']=3.5
            config['simulation_battery_override']='SITL pm uses one-cell 4.2 V; production two-cell minimum remains unchanged'
            return config
        with patch.object(task,'builtin_flight_config',builtin):
            owner._imu_validation_capture=task.ControllerValidationCapture(owner)
        capture=owner._imu_validation_capture
        original_record=capture.link.record;unmodified=(output/'unmodified_packets.jsonl').open('w');streams.append(unmodified)
        def record(group,data,tick=None):
            # Independent input perturbation affects only the recorded replay
            # stream. It is never fed back into controlling estimator 2.
            original={'group':group,'data':deepcopy(data),'cf_log_tick_ms_mod24':tick,
                'received_s':time.monotonic(),'phase':capture.link.phase}
            unmodified.write(json.dumps(original)+'\n');unmodified.flush()
            if group=='imu':
                data=dict(data)
                acc=A@(np.array([data[f'acc.{a}'] for a in 'xyz'])*G)+b
                gyro=np.deg2rad([data[f'gyro.{a}'] for a in 'xyz'])+bg
                data.update(dict(zip([f'acc.{a}' for a in 'xyz'],acc/G)))
                data.update(dict(zip([f'gyro.{a}' for a in 'xyz'],np.rad2deg(gyro))))
            original_record(group,data,tick)
        capture.link.record=record
        capture.prepare();owner.handshake();capture.arm_requested();owner.arm();owner.takeoff();owner.run_mission();owner.land()
        capture.finish(not owner.flying);unmodified.close()
        report=run_replay(output/'flight/packets.jsonl',source/'fit/candidate.json',output/'replay_known_fault')
        nominal=json.loads((source/'fit/candidate.json').read_text())
        nominal.update(measured_from_reference=np.eye(3).tolist(),accel_bias_m_s2=[0,0,0],gyro_residual_bias_rad_s=[0,0,0])
        write_json(output/'nominal_candidate.json',nominal)
        baseline=run_replay(output/'unmodified_packets.jsonl',output/'nominal_candidate.json',output/'replay_unmodified')
        write_json(output/'summary.json',{'completed':True,'protocol':protocol,
            'real_firmware_pid_and_default_kf':True,'standard_controller_arm_takeoff_land':True,
            'onboard_estimator3_enabled':False,'hardware_contacted':False,
            'known_fault_scope':'Offline logged IMU only; actual controlling KF receives unmodified SITL sensors.',
            'capture':capture.report,'events':events,
            'known_fault':{m:{k:v for k,v in report[m].items() if k!='samples'} for m in ('raw','corrected')},
            'unmodified':{k:v for k,v in baseline['raw'].items() if k!='samples'},
            'firmware_sha256':hashlib.sha256(firmware.read_bytes()).hexdigest(),
            'controller_sha256':hashlib.sha256(Path('/offboard/controller.py').read_bytes()).hexdigest()})
    except BaseException as error:
        failure=error;write_json(output/'failure.json',{'error':str(error),'events':events})
        raise
    finally:
        if owner is not None and owner.flying:
            try:owner.land()
            except BaseException:pass
        if owner is not None and getattr(owner,'_imu_validation_capture',None):
            owner._imu_validation_capture.finish(False)
        stopped.set()
        if pose_thread:pose_thread.join(timeout=2)
        if scf:
            try:scf.cf.commander.send_stop_setpoint();scf.cf.platform.send_arming_request(False)
            finally:scf.close_link()
        for process in reversed(processes):gazebo.stop_process(process)
        for stream in streams:
            if not stream.closed:stream.close()


if __name__=='__main__':
    logging.basicConfig(level=logging.INFO)
    main('/artifacts')
