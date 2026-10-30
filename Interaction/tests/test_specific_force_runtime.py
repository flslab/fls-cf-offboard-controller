import unittest
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np

from Interaction.specific_force_runtime import (SpecificForceTelemetry, validate_config,
    requested, IMU_GROUP, IMU_FIELDS)
from Interaction.onboard_wrench_interaction_pipeline import OnboardMomentumWrenchPipeline
from Interaction.config import log_vars_for_mission


def config(**extra):
    return dict(force_estimator='specific_force', startup_bias_calibration_enabled=False,
                motor_model={'pitch_sign_for_world_thrust': -1}, **extra)


def step(p, t, force=0., **extra):
    inputs = dict(position=[0,0,1], velocity=[0,0,0], attitude_rpy=[0,0,0],
        angular_velocity=[0,0,0], motor_pwm=[31900]*4, battery_voltage=7.8,
        timestamp=1000+t, specific_force_body_m_s2=[force/.17,0,9.81],
        gyro_body_rad_s=[0,0,0], imu_timestamp=t)
    inputs.update(extra)
    return p.update(**inputs)


class PipelineTests(unittest.TestCase):
    def test_contact_and_release_without_velocity_changes(self):
        p=OnboardMomentumWrenchPipeline(config())
        starts=[]; ends=[]
        for i in range(100):
            t=i*.01
            o=step(p,t,.4 if .2<=t<.5 else 0.)
            if o.contacts:
                if o.contacts.translation.started: starts.append(t)
                if o.contacts.translation.ended: ends.append(t)
        self.assertEqual(len(starts),1);self.assertEqual(len(ends),1)
        self.assertGreaterEqual(ends[0],.5)
        self.assertLess(ends[0],.85)

    def test_velocity_jump_and_momentum_calibration_do_not_create_force(self):
        p=OnboardMomentumWrenchPipeline(config(impulse_estimator={
            'model_delay_s':.04,'model_acceleration_scale':[.5,.5,.5]}))
        for i in range(40):
            o=step(p,i*.01,velocity=[i*.2,-i*.1,0])
            np.testing.assert_allclose(o.raw_estimate.external_force,0,atol=1e-10)
            if o.contacts:self.assertFalse(o.contacts.translation.started)

    def test_legacy_pitch_rotation_and_calibration(self):
        p=OnboardMomentumWrenchPipeline(config(specific_force={
            'accel_scale':[2,1,1], 'accel_bias_m_s2':[.1,0,0]}))
        for i in range(20):
            o=step(p,i*.01,attitude_rpy=[0,-np.pi/2,0],
                   specific_force_body_m_s2=[(.4/.17+.1)/2,0,9.81])
        np.testing.assert_allclose(o.raw_estimate.external_force,[0,0,-.4],atol=1e-10)

    def test_gap_and_duplicate_never_finish_active_release(self):
        p=OnboardMomentumWrenchPipeline(config())
        for i in range(50):step(p,i*.01,.4 if i<30 else 0)
        p.detector.translation.active=True
        p.detector.translation._release_elapsed=.14
        o=step(p,.8)
        self.assertTrue(o.estimate.measurement_rejected)
        self.assertIsNone(o.contacts)
        self.assertTrue(p.detector.translation.active)
        self.assertEqual(p.detector.translation._release_elapsed,0)
        self.assertIsNone(step(p,.8).contacts)
        for i in range(1,6):
            o=step(p,.8+i*.01)
            if o.contacts:self.assertFalse(o.contacts.translation.ended)

    def test_missing_raw_imu_cannot_fallback_to_compressed_acceleration(self):
        for change in ({'specific_force_body_m_s2':None},{'gyro_body_rad_s':None},
                       {'imu_timestamp':None},{'motor_pwm':None},{'battery_voltage':None}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                step(OnboardMomentumWrenchPipeline(config()),0,**change)

    def test_startup_bias_calibration_waits_for_valid_stationary_window(self):
        c=config();c.update(startup_bias_calibration_enabled=True,observer_settle_s=0.,
                            bias_calibration_s=.12,minimum_bias_samples=5)
        p=OnboardMomentumWrenchPipeline(c)
        for i in range(35):o=step(p,i*.01,.03)
        self.assertTrue(o.calibrated)
        np.testing.assert_allclose(p.force_bias,[.03,0,0],atol=1e-10)
        np.testing.assert_allclose(o.estimate.external_force,0,atol=1e-10)


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.reader=SpecificForceTelemetry({})
        self.packet={**{f:0. for f in IMU_FIELDS},'acc.z':1.,'gyro.y':180.,'time':100.}
        self.meta={'time':100.,'cf_timestamp_ms':0xfffffc}
        self.skew=.002
        self.state={'cf_timestamp_ms':0xfffffc,'motor_skew_s':0.}
        self.logs=SimpleNamespace(
            get_nearest_group_log_data_by_cf_timestamp=lambda *a:(self.packet,self.skew),
            get_nearest_group_log_metadata=lambda *a:(self.meta,0.))

    def test_wraparound_units_and_duplicate(self):
        a=self.reader.read(self.logs,self.state,100.01)
        np.testing.assert_allclose(a['specific_force_body_m_s2'],[0,0,9.81])
        np.testing.assert_allclose(a['gyro_body_rad_s'],[0,np.pi,0])
        self.assertIsNone(self.reader.read(self.logs,self.state,100.01))
        self.meta['cf_timestamp_ms']=6
        b=self.reader.read(self.logs,self.state,100.01)
        self.assertAlmostEqual(b['imu_timestamp'],.01)

    def test_missing_stale_unsynchronized_and_nonfinite(self):
        self.assertIsNone(self.reader.read(self.logs,self.state,100.2))
        self.skew=.01
        self.assertIsNone(self.reader.read(self.logs,self.state,100.01))
        self.skew=0.;self.packet['acc.x']=float('nan')
        with self.assertRaises(RuntimeError):self.reader.read(self.logs,self.state,100.01)

    def test_backwards_clock_is_rejected(self):
        self.reader.read(self.logs,self.state,100.01)
        self.meta['cf_timestamp_ms']-=1
        with self.assertRaises(RuntimeError):self.reader.read(self.logs,self.state,100.01)


class ConfigurationTests(unittest.TestCase):
    def mission(self,brake):
        return {'Interaction':{'config':dict(behavior='level_coast',detection_method='model',
            wrench_interaction=dict(force_estimator='specific_force',state_source='onboard',
                                    firmware_auto_brake={'enabled':brake}))}}

    def test_automatic_raw_logging_for_both_transports_and_legacy_default(self):
        for brake in (False,True):
            m=self.mission(brake)
            logs=log_vars_for_mission(m)
            self.assertEqual(set(logs[IMU_GROUP])-{'log_period_ms'},set(IMU_FIELDS))
            self.assertEqual(logs[IMU_GROUP]['log_period_ms'],10)
            m['Interaction']['config']['wrench_interaction'].pop('force_estimator')
            self.assertNotIn(IMU_GROUP,log_vars_for_mission(m))

    def test_unknown_mode_and_invalid_parameters_fail_before_logging(self):
        for key,value in [('window_s',float('nan')),('accel_scale',[0,1,1]),
                          ('max_pair_skew_s',.01),('imu_from_com_m',[0,1]),('unknown',1)]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_config({'specific_force':{key:value}})
        m=self.mission(True);m['Interaction']['config']['behavior']='other'
        with self.assertRaises(ValueError):requested(m)
        m=self.mission(True);m['Interaction']['config']['wrench_interaction']['state_source']='offboard'
        with self.assertRaises(ValueError):requested(m)
        m=self.mission(True);m['Interaction']['config']['wrench_interaction']['force_estimator']='typo'
        with self.assertRaises(ValueError):log_vars_for_mission(m)

    def test_non_model_detection_ignores_force_estimator_and_raw_imu_settings(self):
        for detector in ('potentiometer', 'vel', None):
            for brake in (False, True):
                with self.subTest(detector=detector, brake=brake):
                    m=self.mission(brake)
                    cfg=m['Interaction']['config']
                    if detector is None:
                        cfg.pop('detection_method')
                    else:
                        cfg['detection_method']=detector
                    cfg['wrench_interaction']['specific_force']={'window_s':-1}
                    original=deepcopy(m)
                    self.assertFalse(requested(m))
                    self.assertNotIn(IMU_GROUP,log_vars_for_mission(m))
                    self.assertEqual(m,original)

    def test_prearm_requires_fresh_device_paired_raw_imu(self):
        from controller import Controller
        from Interaction.tests.test_compressed_state_logs import buffered_logs, publish, compressed_packets
        c=Controller.__new__(Controller)
        c._uses_onboard_wrench_state=lambda:True
        c.mission=self.mission(True);c.args=SimpleNamespace(contact_attitude_run=None)
        c.log_manager=logs=buffered_logs()
        k,a=compressed_packets()
        publish(logs,'FIRMWARE_KIN',5000,1000.,k)
        publish(logs,'FIRMWARE_ACT',5000,1000.,a)
        raw={**{f:0. for f in IMU_FIELDS},'acc.z':1.}
        publish(logs,IMU_GROUP,5000,1000.,raw)
        logs.get_latest_group_log_data=lambda group:dict(logs.cf_log_group_packets[group][-1]) if logs.cf_log_group_packets[group] else {}
        with patch('controller.time.time',return_value=1000.01):
            c.verify_onboard_wrench_logging()
        logs.cf_log_group_packet_metadata[IMU_GROUP].clear()
        with patch('controller.time.monotonic',side_effect=[0.,0.,5.1]),patch('controller.time.sleep'):
            with self.assertRaisesRegex(RuntimeError,'MODEL_IMU'):c.verify_onboard_wrench_logging()

    def test_scurve_capture_prearm_requires_command_telemetry(self):
        from controller import Controller
        from Interaction.tests.test_compressed_state_logs import buffered_logs, publish, compressed_packets
        c=Controller.__new__(Controller)
        c._uses_onboard_wrench_state=lambda:True
        c.mission=self.mission(True);c.args=SimpleNamespace(contact_attitude_run=None)
        cfg=c.mission['Interaction']['config']
        cfg['behavior']='level_coast'
        cfg['level_coast']={'coast_command_mode':'scurve'}
        c.log_manager=logs=buffered_logs()
        k,a=compressed_packets()
        publish(logs,'FIRMWARE_KIN',5000,1000.,k)
        publish(logs,'FIRMWARE_ACT',5000,1000.,a)
        publish(logs,IMU_GROUP,5000,1000.,{**{f:0. for f in IMU_FIELDS},'acc.z':1.})
        logs.get_latest_group_log_data=lambda group:dict(logs.cf_log_group_packets[group][-1]) if logs.cf_log_group_packets[group] else {}
        with patch('controller.time.monotonic',side_effect=[0.,0.,5.1]),patch('controller.time.sleep'):
            with self.assertRaisesRegex(RuntimeError,'POSITION_HANDOFF'):c.verify_onboard_wrench_logging()
        publish(logs,'POSITION_HANDOFF',5000,1000.,{'controller.roll':0.,'controller.pitch':0.})
        with patch('controller.time.time',return_value=1000.01):c.verify_onboard_wrench_logging()


class LevelCoastRuntimeTests(unittest.TestCase):
    def run_scenario(self, drop_after=None):
        from Interaction.level_coast import run_level_coast
        from Interaction.tests.test_level_coast import configuration
        from Interaction.tests.test_wrench_interactions_integration import FakeCommander
        from Interaction.tests.test_position_follow import parameters
        clock=[0.]
        cfg=configuration('model');cfg['duration']=1.
        cfg['wrench_interaction']['force_estimator']='specific_force'
        cfg['wrench_interaction']['motor_model']={'hover_pwm':31900,'hover_voltage':7.8}
        entries=[]
        def packet():
            return {**{f:0. for f in IMU_FIELDS},'acc.z':1.,
                'acc.x':(.4/.17/9.81 if .2<=clock[0]<.4 else 0.),'time':1000.+clock[0]}
        def state():
            return dict(time=1000.+clock[0],cf_timestamp_ms=round(clock[0]*1000),
                position=np.array([0.,0.,1.]),velocity=np.zeros(3),attitude_rpy=np.zeros(3),
                angular_velocity=np.zeros(3),position_skew_s=0.,angular_rate_skew_s=0.,
                yaw_control_skew_s=None,yaw_control_command=None,motor_skew_s=0.,
                motor_state={**{f'motor.m{i}':31900 for i in range(1,5)},
                             'pm.vbat':7.8,'time':1000.+clock[0]})
        logs=SimpleNamespace(add_log_entry=lambda kind,data,**kw:entries.append((kind,data)),
            get_nearest_group_log_data_by_cf_timestamp=lambda *a:(
                (None,None) if drop_after is not None and clock[0]>=drop_after else (packet(),0.)),
            get_nearest_group_log_metadata=lambda *a:({'time':1000.+clock[0],
                'cf_timestamp_ms':round(clock[0]*1000)},0.))
        def sleep(dt):
            clock[0]=round(clock[0]+dt,6)
            self.assertLess(clock[0],2.)
        owner=SimpleNamespace(cf=SimpleNamespace(_offboard_position_pid=SimpleNamespace(
                prepared=True,parameters=parameters())),drone_id='lb11',ctrl_rate=100,
            mission={'drones':{'lb11':{'target':[0,0,1]}}},log_manager=logs,
            lo_commander=FakeCommander(),check_interaction_boundary=Mock(),_log_event=Mock(),
            _safe_sleep=sleep,_get_synchronized_onboard_wrench_state=state,
            _set_contact_pid_attitude_authority=Mock(),
            _vicon_motion_reference_for_onboard_state=lambda *a,**kw:(
                np.array([0.,0.,1.]),np.zeros(3),1000.+clock[0],0.),
            _vicon_velocity_reference_for_onboard_state=lambda *a,**kw:(np.zeros(3),1000.+clock[0],0.))
        with patch('Interaction.level_coast.time.time',side_effect=lambda:1000.+clock[0]), \
             patch('Interaction.level_coast.time.monotonic',side_effect=lambda:clock[0]), \
             patch('Interaction.level_coast.apply_detection_calibration',side_effect=lambda c,*a:c), \
             patch('Interaction.interactions.reset_pid_integrators_without_ack'):
            if drop_after is None:
                run_level_coast(owner,cfg)
            else:
                from Interaction.interactions import StaleLocalizationError
                with self.assertRaisesRegex(StaleLocalizationError,'MODEL_IMU'):
                    run_level_coast(owner,cfg)
        wrench=[r for kind,r in entries if kind=='wrench_observer']
        owner._set_contact_pid_attitude_authority.assert_called_with(False)
        return wrench

    def test_real_pipeline_with_raw_telemetry_drives_contact_and_release(self):
        wrench=self.run_scenario()
        self.assertEqual(sum(r['contact_started'] for r in wrench),1)
        self.assertEqual(sum(r['release_confirmed'] for r in wrench),1)
        self.assertTrue(all(r['force_estimator']=='specific_force' for r in wrench))
        self.assertTrue(all('imu_cf_timestamp_ms' in r for r in wrench))

    def test_missing_imu_during_contact_aborts_without_false_release(self):
        wrench=self.run_scenario(drop_after=.33)
        self.assertEqual(sum(r['contact_started'] for r in wrench),1)
        self.assertFalse(any(r['release_confirmed'] for r in wrench))


if __name__=='__main__':unittest.main()
