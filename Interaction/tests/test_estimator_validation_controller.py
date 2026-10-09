"""Standard flight lifecycle integration and IMU capture failure gates, without hardware."""
import ast
import hashlib
import json
import logging
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from Interaction.estimator_validation_controller import ControllerValidationCapture


def controller_method(name):
    path=Path(__file__).resolve().parents[2]/'controller.py'
    cls=next(n for n in ast.parse(path.read_text()).body
             if isinstance(n,ast.ClassDef) and n.name=='Controller')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name)
    namespace={'logger':logging.getLogger(__name__), 'time':__import__('time'), '__file__':str(path)}
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),namespace)
    return namespace[name], namespace


class StandardControllerTests(unittest.TestCase):
    def test_prepare_records_replay_ready_and_anchors_takeoff_above_selected_floor(self):
        from Interaction.simulate_estimator_imu import Param
        with TemporaryDirectory() as tmp:
            capture=self.capture(Path(tmp));capture.owner.cf.param=Param()
            snap={name:{} for name in ('imu','kf','state','health')}
            snap['vicon']={'data':{'position_m':[.05,-.88,.32]}}
            with patch.object(capture.link,'snapshot',return_value=snap), \
                    patch('Interaction.estimator_validation_controller.check_snapshot'), \
                    patch('Interaction.estimator_validation_controller.verify_capture_continuity'), \
                    patch.object(capture,'check_supervisor'), \
                    patch.object(capture,'refresh_gyro_zero'), \
                    patch('Interaction.estimator_validation_controller.fls_log_config',side_effect=lambda *a:Mock()):
                capture.prepare()
            self.assertAlmostEqual(capture.owner.args.takeoff_altitude,.92)
            np.testing.assert_allclose(capture.config['center_m'],[.05,-.88,.92])
            capture.finish(False)
            rows=[json.loads(line) for line in (capture.output/'packets.jsonl').read_text().splitlines()]
            self.assertTrue(any(r['group']=='event' and r['data']['name']=='capture_ready' for r in rows))

    def capture(self, root):
        source = root/'source'
        (source/'fit').mkdir(parents=True)
        (source/'dataset.json').write_text('{}')
        (source/'fit/candidate.json').write_text(json.dumps(dict(fit_passed=True,drone_id='lb11',
            firmware_id='test',dataset_sha256=hashlib.sha256(b'{}').hexdigest())))
        owner = SimpleNamespace(args=SimpleNamespace(imu_validation_session=str(source),
            imu_validation_output=str(root/'flight'),drone_id='lb11',init_pos=[0,-1,.24],
            takeoff_altitude=.84),cf=Mock(),ll_commander=Mock(),init_coord=[0,-1,.24],_safe_sleep=Mock())
        return ControllerValidationCapture(owner)

    def test_airborne_task_never_arms_takes_off_or_lands_and_requires_landing_success(self):
        for landing_ok in (False,True):
            with self.subTest(landing_ok=landing_ok),TemporaryDirectory() as tmp:
                capture = self.capture(Path(tmp))
                snap={'vicon':{'data':{'position_m':[0,-1,.84]}},
                    'kf':{'data':{'kalman.q0':1.,'kalman.q1':0.,'kalman.q2':0.,'kalman.q3':0.}}}
                with patch.object(capture.link,'snapshot',return_value=snap), \
                        patch('Interaction.estimator_validation_controller.check_snapshot'), \
                        patch('Interaction.estimator_validation_controller.trajectory',return_value=[None,
                            ('hover_start',np.array([0,-1,.84]),np.array([0,-1,.84]),1.),None]), \
                        patch('Interaction.estimator_validation_controller.time.monotonic',side_effect=[0.,.5,1.,1.5,2.]):
                    capture.run()
                capture.owner.cf.platform.send_arming_request.assert_not_called()
                capture.owner.cf.high_level_commander.takeoff.assert_not_called()
                capture.owner.cf.high_level_commander.land.assert_not_called()
                capture.owner.ll_commander.send_position_setpoint.assert_called_once()
                capture.owner.ll_commander.send_notify_setpoint_stop.assert_not_called()
                capture.finish(landing_ok)
                report=json.loads((capture.output/'report.json').read_text())
                self.assertEqual(report['capture_completed'],landing_ok)
                self.assertEqual(report['flight_lifecycle'],'controller_standard')
                self.assertFalse(report['candidate_applied'])

    def test_midflight_failure_keeps_report_and_ownership_until_standard_landing(self):
        with TemporaryDirectory() as tmp:
            capture = self.capture(Path(tmp))
            snap={'kf':{'data':{'kalman.q0':1.,'kalman.q1':0.,'kalman.q2':0.,'kalman.q3':0.}}}
            with patch.object(capture.link,'snapshot',return_value=snap), \
                    patch('Interaction.estimator_validation_controller.check_snapshot',side_effect=RuntimeError('stale vicon')):
                with self.assertRaisesRegex(RuntimeError,'stale vicon'):
                    capture.run()
            capture.owner.ll_commander.send_notify_setpoint_stop.assert_not_called()
            capture.finish(True)
            report=json.loads((capture.output/'report.json').read_text())
            self.assertFalse(report['capture_completed'])
            self.assertEqual(report['error'],'stale vicon')

    def test_locked_supervisor_is_rejected_before_arm(self):
        with TemporaryDirectory() as tmp:
            capture=self.capture(Path(tmp))
            with self.assertRaisesRegex(RuntimeError,'LOCKED.*576'):
                capture.check_supervisor({'health':{'data':{'supervisor.info':576}}},require_armable=True)
            capture.owner.cf.platform.send_arming_request.assert_not_called()
            capture.finish(False)

    def test_receive_burst_waits_for_fresh_kf_without_sending_a_new_target(self):
        from Interaction.simulate_estimator_imu import Clock
        with TemporaryDirectory() as tmp:
            capture=self.capture(Path(tmp));clock=Clock();capture.owner._safe_sleep=clock.sleep
            def snapshot(**kwargs):
                return self.motion_snapshot(clock, kf_age=.103 if clock.t==1000 else 0.)
            with patch('Interaction.estimator_validation_controller.time',clock), \
                    patch('Interaction.estimator_validation_flight.time',clock), \
                    patch.object(capture.link,'snapshot',side_effect=snapshot) as read:
                snap=capture.fresh_motion_snapshot()
            self.assertLess(clock.t-1000,.1)
            self.assertEqual(snap['now']-snap['kf']['received_s'],0.)
            read.assert_called_with(include_metadata=False)
            capture.owner.ll_commander.send_position_setpoint.assert_not_called()
            event=capture.link.latest['event']['data']
            self.assertEqual(event['name'],'telemetry_wait_recovered')
            self.assertTrue(event['held_previous_position_target'])
            capture.finish(False)

    def motion_snapshot(self, clock, *, kf_age=0., voltage=8.):
        return {'now':clock.t,
            'vicon':{'received_s':clock.t,'data':{'position_m':[0,-1,.84]}},
            'imu':{'received_s':clock.t,'data':{}},
            'kf':{'received_s':clock.t-kf_age,'data':dict(zip(
                ['kalman.q0','kalman.q1','kalman.q2','kalman.q3'],[1.,0.,0.,0.]))},
            'state':{'received_s':clock.t,'data':dict(zip(
                ['stateEstimate.x','stateEstimate.y','stateEstimate.z'],[0.,-1.,.84]))},
            'health':{'received_s':clock.t,'data':{'pm.vbat':voltage,**{
                f'motor.m{i}':30000 for i in range(1,5)}}}}

    def test_persistent_kf_outage_still_aborts_and_low_battery_is_never_retried(self):
        from Interaction.simulate_estimator_imu import Clock
        from Interaction.estimator_validation_flight import TelemetryStaleError
        for fault in ('outage','battery','boundary'):
            with self.subTest(fault=fault),TemporaryDirectory() as tmp:
                capture=self.capture(Path(tmp));clock=Clock()
                capture.owner._safe_sleep=Mock(side_effect=clock.sleep)
                def snapshot(**kwargs):
                    snap=self.motion_snapshot(clock,kf_age=.15 if fault=='outage' else 0.,
                                              voltage=6. if fault=='battery' else 8.)
                    if fault=='boundary':snap['vicon']['data']['position_m']=[3.,-1.,.84]
                    return snap
                with patch('Interaction.estimator_validation_controller.time',clock), \
                        patch('Interaction.estimator_validation_flight.time',clock), \
                        patch.object(capture.link,'snapshot',side_effect=snapshot):
                    with self.assertRaises(TelemetryStaleError if fault=='outage' else RuntimeError):
                        capture.fresh_motion_snapshot()
                if fault=='outage':
                    self.assertGreaterEqual(clock.t-1000,.1)
                    self.assertLess(clock.t-1000,.11)
                else:
                    capture.owner._safe_sleep.assert_not_called()
                capture.owner.ll_commander.send_position_setpoint.assert_not_called()
                capture.finish(False)

    def test_hot_snapshot_omits_large_metadata_and_keeps_telemetry_independent(self):
        from Interaction.estimator_validation_flight import FlightConnection
        import io
        class Metadata:
            def __deepcopy__(self, memo):raise AssertionError('metadata copied in motion loop')
        cf=Mock();cf.param.get_value.return_value='2'
        link=FlightConnection(cf,{},io.StringIO())
        link.latest={'metadata':{'data':Metadata()},'kf':{'data':{'kalman.q0':1.}}}
        snap=link.snapshot(include_metadata=False)
        self.assertNotIn('metadata',snap)
        snap['kf']['data']['kalman.q0']=0.
        self.assertEqual(link.latest['kf']['data']['kalman.q0'],1.)

    def test_waits_for_fresh_can_fly_and_disarms_if_locked_or_timeout(self):
        from Interaction.simulate_estimator_imu import Clock
        for mode in ('delayed','locked','timeout'):
            with self.subTest(mode=mode),TemporaryDirectory() as tmp:
                capture=self.capture(Path(tmp));clock=Clock()
                capture.owner._safe_sleep=clock.sleep
                def snapshot():
                    info=576 if mode=='locked' else (10 if mode=='delayed' and clock.t>=1001.4 else 2)
                    return {'health':{'received_s':clock.t,'data':{'supervisor.info':info}}}
                with patch('Interaction.estimator_validation_controller.time',clock), \
                        patch('Interaction.estimator_validation_controller.check_snapshot'), \
                        patch.object(capture.link,'snapshot',side_effect=snapshot):
                    if mode=='delayed':
                        capture.wait_until_fly_ready()
                        self.assertGreaterEqual(clock.t,1001.4)
                        capture.owner.cf.platform.send_arming_request.assert_not_called()
                    else:
                        with self.assertRaises((RuntimeError,TimeoutError)):
                            capture.wait_until_fly_ready()
                        capture.owner.cf.platform.send_arming_request.assert_called_once_with(False)
                capture.owner.ll_commander.send_position_setpoint.assert_not_called()
                capture.finish(False)

    def test_floor_gyro_refresh_retains_accel_fit_and_establishes_new_boot_reference(self):
        from Interaction.simulate_estimator_imu import Clock
        from Interaction.estimator_validation_flight import verify_capture_continuity
        with TemporaryDirectory() as tmp:
            capture=self.capture(Path(tmp));clock=Clock()
            capture.owner._safe_sleep=clock.sleep
            config={'firmware_parameters_before':{'imu_sensors':{'imuPhi':'0'}}}
            def snapshot():
                return {'imu':{'received_s':clock.t,'cf_log_tick_ms_mod24':clock.tick(),
                    'data':{'acc.x':0.,'acc.y':0.,'acc.z':1.,'gyro.x':.2,'gyro.y':-.1,'gyro.z':.05}},
                    'health':{'data':{'supervisor.info':1}},'metadata':{'data':config}}
            before=json.loads(json.dumps(capture.candidate))
            with patch('Interaction.estimator_validation_controller.time',clock), \
                    patch('Interaction.estimator_validation_flight.time',clock), \
                    patch('Interaction.estimator_validation_controller.check_snapshot'), \
                    patch.object(capture.link,'snapshot',side_effect=snapshot):
                capture.refresh_gyro_zero()
                dataset={'capture':{'firmware_parameters_after':{'imu_sensors':{'imuPhi':'0'}}},
                    'poses':[{'host_receipt_monotonic_ns':[1], 'cf_log_tick_ms_mod24':[999999]}]}
                with self.assertRaisesRegex(RuntimeError,'continuity'):
                    verify_capture_continuity(dataset,snapshot())
                verify_capture_continuity(dataset,snapshot(),clock_reference=capture.clock_reference)
            self.assertEqual(capture.candidate,before)
            refresh=capture.report['preflight_gyro_zero']
            self.assertGreaterEqual(refresh['samples'],200)
            np.testing.assert_allclose(refresh['gyro_residual_bias_rad_s'],np.deg2rad([.2,-.1,.05]))
            capture.finish(False)

    def test_stuck_previous_low_level_owner_blocks_old_takeoff_but_not_validation_takeoff(self):
        from Interaction.simulate_estimator_imu import Clock, Plant, controller_methods
        clock=Clock();plant=Plant(clock,np.eye(3),np.zeros(3),np.zeros(3),1)
        plant.armed=True;clock.on_step=plant.step
        ground=plant.p.copy()
        # Reproduce the missing-release path: the timer passes while height
        # stays on the floor, then a naked notify activates the pending climb.
        plant.command('takeoff',[0,-1,.84]);clock.sleep(2.68)
        np.testing.assert_allclose(plant.p,ground)
        plant.command('notify_stop');clock.sleep(.1)
        self.assertGreater(plant.p[2],ground[2])
        # Production takeoff must transfer ownership before its timed climb.
        clock=Clock();plant=Plant(clock,np.eye(3),np.zeros(3),np.zeros(3),1)
        plant.armed=True;clock.on_step=plant.step
        high=SimpleNamespace(takeoff=lambda z,t,yaw:plant.command('takeoff',[0,-1,z]))
        low=SimpleNamespace(send_notify_setpoint_stop=lambda:plant.command('notify_stop'))
        owner=SimpleNamespace(args=SimpleNamespace(ground_test=False,skip_takeoff=False,
            imu_validation_session='saved-fit',droneless=False,takeoff_altitude=.84,init_yaw=0.),
            mission={},hl_commander=high,ll_commander=low,
            _is_interaction_application=lambda:True,log_manager=Mock(),_safe_sleep=clock.sleep)
        controller_methods(clock,[])['takeoff'](owner)
        self.assertLess(abs(plant.p[2]-.84),.05)
        self.assertEqual([c['kind'] for c in plant.commands],['takeoff','notify_stop'])

    def test_imu_validation_skips_ordinary_pid_calibration_setup(self):
        setup,_=controller_method('setup_params')
        for session in (None,'saved-fit'):
            with self.subTest(session=session):
                owner=SimpleNamespace(args=SimpleNamespace(calibrate=True,controller_type='pid',
                    imu_validation_session=session,vicon=False,tracker=False,ground_test=True,
                    skip_landing=False,skip_takeoff=False),cf=Mock(),use_flowdeck=False,
                    cfg=SimpleNamespace(PID_VALUES={'posCtlPid.xKp':1.}),log_manager=Mock())
                for method in ('_prepare_offboard_yaw_damping','_prepare_offboard_position_control',
                               '_activate_kalman_estimator','_activate_tumble_check',
                               '_activate_pid_controller','_activate_high_level_commander','_set_pid_values'):
                    setattr(owner,method,Mock())
                with patch('Interaction.firmware_response_model.confirm_pid_context',return_value={'x':1}) as confirm, \
                        patch('Interaction.estimator_ram_trace.prepare_interaction_recorder'):
                    setup(owner)
                self.assertEqual(confirm.call_count,0 if session else 1)
                self.assertEqual(hasattr(owner,'_response_calibration_pid'),not bool(session))

    def test_imu_validation_cleanup_does_not_fit_or_overwrite_ordinary_calibration(self):
        stop,_=controller_method('stop')
        for session in (None,'saved-fit'):
            with self.subTest(session=session):
                capture=Mock()
                owner=SimpleNamespace(args=SimpleNamespace(imu_validation_session=session,
                    drone_id='lb11',log_dir='/tmp',tag='capture'),cf=SimpleNamespace(),
                    servo=None,bat_logger=None,mocap=None,force_sensor=None,rpi_power_monitor=None,
                    tracker_process=None,blinker_process=None,smooth_controller=None,led=None,
                    log_manager=None,mission={},mission_start_time=0,land=Mock(),disconnect=Mock(),
                    flying=False,_response_calibration_pid={'x':1},_imu_validation_capture=capture)
                with patch('Interaction.firmware_response_model.fit_completed_calibration',
                        return_value={'accepted':True}) as fit:
                    stop(owner)
                self.assertEqual(fit.call_count,0 if session else 1)
                capture.finish.assert_called_once_with(True)
                owner.disconnect.assert_called_once()

    def test_standard_start_retains_setup_arm_takeoff_before_validation_task(self):
        path=Path(__file__).resolve().parents[2]/'controller.py'
        tree=ast.parse(path.read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Controller')
        start=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='start')
        namespace={'time':Mock(),'input':Mock(return_value='')}
        exec(compile(ast.Module(body=[start],type_ignores=[]),str(path),'exec'),namespace)
        calls=[]
        owner=SimpleNamespace(args=SimpleNamespace(droneless=False,imu_validation_session=None,orchestrated=True),led=None,
            _imu_validation_capture=SimpleNamespace(prepare=lambda:calls.append('prepare_capture'),
                arm_requested=lambda:calls.append('capture_arm')),_is_interaction_application=lambda:True)
        methods=[n.func.attr for n in ast.walk(start) if isinstance(n,ast.Call)
                 and isinstance(n.func,ast.Attribute) and isinstance(n.func.value,ast.Name)
                 and n.func.value.id=='self']
        for name in methods:
            if name=='_is_interaction_application':continue
            setattr(owner,name,lambda name=name:calls.append(name))
        namespace['start'](owner)
        namespace['input'].assert_not_called()
        for a,b in [('setup_motion_capture','setup_params'),('setup_params','prepare_capture'),
                    ('prepare_capture','handshake'),
                    ('prepare_capture','arm'),('arm','takeoff'),('takeoff','run_mission')]:
            self.assertLess(calls.index(a),calls.index(b))


if __name__=='__main__':
    unittest.main()
