import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from controller import Controller
from Interaction.position_follow import (
    POSITION_KP, VELOCITY_KP, VELOCITY_LIMIT, SUPPRESSED_GAINS,
    PositionFollowPidContext, PositionVelocityFollower, validate_position_follow,
)
from Interaction.tests.test_offboard_yaw_damping import Param


def parameters():
    return {**dict(zip(POSITION_KP, (1.9, 2.1))),
            **dict.fromkeys(VELOCITY_KP, 30.),
            **dict.fromkeys(VELOCITY_LIMIT, 2.),
            **dict.fromkeys(SUPPRESSED_GAINS, 0.)}


class PositionFollowerTests(unittest.TestCase):
    def test_contact_inverts_rotated_anisotropic_fc_position_loop(self):
        for yaw in (0., .7, math.pi/2, -2.7):
            follower = PositionVelocityFollower(parameters(), {})
            p, v = np.array([.1, -.5, 1.1]), np.array([.6, -.3, .2])
            target, log = follower.target(p, v, yaw, 1., 'contact', 1.)
            c,s = math.cos(yaw), math.sin(yaw)
            rotation = np.array([[c,s],[-s,c]])
            # Independently reproduce the firmware cascade's position P output.
            actual_body_target = np.array([1.9,2.1]) * (rotation @ (target[:2]-p[:2]))
            np.testing.assert_allclose(actual_body_target, rotation @ v[:2], atol=1e-12)
            self.assertEqual(target[2], 1.)
            self.assertGreater(np.dot(target[:2]-p[:2], v[:2]), 0)

    def test_release_ramps_damping_then_bounds_combined_braking_tilt(self):
        f = PositionVelocityFollower(parameters(), {})
        v = [.8,-.5,0]
        _, first = f.target([0,0,1],v,.4,1.,'coast',1.)
        self.assertEqual(first['position_velocity_retention_requested'], 1.)
        last = None
        for t in (1.1,1.25,1.5,2.):
            target, log = f.target([0,0,1],v,.4,t,'coast',1.)
            velocity = np.array(log['position_velocity_target_m_s'])
            self.assertLessEqual(np.dot(velocity-v[:2],v[:2]), 1e-12)
            self.assertLessEqual(np.linalg.norm(log['position_nominal_pitch_roll_deg']),
                                 math.degrees(math.atan(.8/9.81))+1e-10)
            last = log
        self.assertEqual(last['position_velocity_retention_requested'], 0.)
        # At low speed the acceleration bound no longer clips: reference is zero.
        _, low = f.target([0,0,1],[.02,-.01,0],.4,2.1,'coast',1.)
        np.testing.assert_allclose(low['position_velocity_target_m_s'], [0,0],atol=1e-12)

    def test_next_contact_discards_old_direction_and_coast_timer(self):
        f = PositionVelocityFollower(parameters(), {})
        f.target([0,0,1],[.4,0,0],0,1,'coast',1)
        f.target([.1,0,1],[.2,0,0],0,2,'coast',1)
        target, log = f.target([.2,0,1],[-.3,.1,0],0,2.1,'contact',1)
        self.assertLess(target[0],.2)
        np.testing.assert_allclose(log['position_velocity_target_m_s'],[-.3,.1])
        _, log = f.target([.2,0,1],[-.3,.1,0],0,2.2,'coast',1)
        self.assertEqual(log['position_velocity_retention_requested'],1.)

    def test_low_speed_capture_projects_ahead_using_velocity_pid_decay(self):
        f=PositionVelocityFollower(parameters(),{})
        f.target([0,0,1],[.2,0,0],0,0,'coast',1)
        target,log=f.target([.1,0,1],[.02,0,0],0,1,'coast',1,capture=True)
        self.assertAlmostEqual(target[0]-.1,.02/(9.81*math.radians(30.)))
        self.assertTrue(log['position_capture_projected'])
        self.assertLess(log['position_velocity_target_m_s'][0],.02)

    def test_contact_cancels_fc_damping_despite_different_vicon_velocity(self):
        for yaw in (0., .7, math.pi / 2):
            f = PositionVelocityFollower(parameters(), {})
            p = np.array([.1, -.2, 1.])
            onboard, motion = [.12, -.08, 0.], [.8, .3, 0.]
            target, log = f.target(p, onboard, yaw, 1., 'contact', 1.,
                                   motion_velocity=motion)
            c, s = math.cos(yaw), math.sin(yaw)
            rotation = np.array([[c, s], [-s, c]])
            fc_reference = f.kp * (rotation @ (target[:2] - p[:2]))
            np.testing.assert_allclose(fc_reference - rotation @ np.array(onboard[:2]),
                                       [0., 0.], atol=1e-12)
            np.testing.assert_allclose(log['position_motion_velocity_m_s'], motion)

    def test_coast_brakes_vicon_motion_even_when_ekf_has_opposite_sign(self):
        for yaw in (0., .7, math.pi / 2):
            f = PositionVelocityFollower(parameters(), {})
            p, onboard, motion = np.array([.1, -.2, 1.]), [-.08, .06, 0.], [.04, -.03, 0.]
            f.target(p, onboard, yaw, 1., 'coast', 1., motion_velocity=motion)
            target, log = f.target(p, onboard, yaw, 2., 'coast', 1., motion_velocity=motion)
            c, s = math.cos(yaw), math.sin(yaw)
            rotation = np.array([[c, s], [-s, c]])
            # Firmware's actual velocity error must oppose physical Vicon motion,
            # not the oppositely signed onboard estimate.
            fc_error = f.kp * (rotation @ (target[:2] - p[:2])) - rotation @ np.array(onboard[:2])
            np.testing.assert_allclose(fc_error, -rotation @ np.array(motion[:2]), atol=1e-12)
            self.assertLess(np.dot(rotation.T @ fc_error, motion[:2]), 0.)
            self.assertLessEqual(np.linalg.norm(log['position_nominal_pitch_roll_deg']),
                                 math.degrees(math.atan(.8 / 9.81)))

    def test_capture_uses_vicon_stop_projection_and_compensates_fc_velocity_bias(self):
        f = PositionVelocityFollower(parameters(), {})
        p, onboard, motion = np.array([.1, 0., 1.]), [.12, 0., 0.], [-.02, 0., 0.]
        f.target(p, onboard, 0., 0., 'coast', 1., motion_velocity=motion)
        target, log = f.target(p, onboard, 0., 1., 'coast', 1.,
                               capture=True, motion_velocity=motion)
        projection = motion[0] / (9.81 * math.radians(30.))
        self.assertAlmostEqual(log['position_capture_stop_projection_m'][0], projection)
        self.assertAlmostEqual(target[0] - p[0], projection + (onboard[0] - motion[0]) / 1.9)
        fc_error = 1.9 * (target[0] - p[0]) - onboard[0]
        self.assertGreater(fc_error, 0.)  # Brake physical negative motion.

    def test_invalid_motion_reference_is_rejected(self):
        for motion in ([1., 0.], [float('nan'), 0., 0.], [0., float('inf'), 0.]):
            with self.subTest(motion=motion), self.assertRaises(ValueError):
                PositionVelocityFollower(parameters(), {}).target(
                    [0., 0., 1.], [0., 0., 0.], 0., 1., 'contact', 1., motion_velocity=motion)

    def test_large_vicon_motion_clips_braking_correction_not_ekf_compensation(self):
        f = PositionVelocityFollower(parameters(), {'contact_velocity_retention': .5})
        onboard, motion = [.01, .02, 0.], [.9, -.7, 0.]
        _, log = f.target([0., 0., 1.], onboard, .6, 1., 'contact', 1., motion_velocity=motion)
        correction = np.asarray(log['position_velocity_target_m_s']) - onboard[:2]
        self.assertLess(np.dot(correction, motion[:2]), 0.)
        self.assertAlmostEqual(np.linalg.norm(log['position_nominal_pitch_roll_deg']),
                               math.degrees(math.atan(.8 / 9.81)))
        np.testing.assert_allclose(correction, log['position_velocity_correction_m_s'], atol=1e-12)

    def test_invalid_state_history_integrals_and_limits_are_rejected(self):
        bad = parameters(); bad[SUPPRESSED_GAINS[0]] = .1
        with self.assertRaisesRegex(ValueError,'zero XY'): PositionVelocityFollower(bad,{})
        f = PositionVelocityFollower(parameters(), {})
        f.target([0,0,1],[0,0,0],0,1,'contact',1)
        with self.assertRaisesRegex(ValueError,'fresh state'):
            f.target([0,0,1],[0,0,0],0,1,'contact',1)
        for velocity in ([3,0,0],[1.5,0,0],[float('nan'),0,0]):
            with self.subTest(velocity=velocity), self.assertRaises(ValueError):
                PositionVelocityFollower(parameters(),{}).target([0,0,1],velocity,0,1,'contact',1)
        for options in ({'coast_velocity_retention':1}, {'coast_transition_s':0},
                        {'contact_velocity_retention':True}, {'unknown':1}):
            with self.subTest(options=options),self.assertRaises(ValueError): validate_position_follow(options)

    def test_delayed_ideal_cascade_damps_both_axes_without_chasing_old_position(self):
        # A bounded ideal-plant regression, NOT a physical flight prediction.
        f = PositionVelocityFollower(parameters(), {})
        p, v, accel = np.array([0.,0.,1.]), np.array([.6,-.3,0.]), np.zeros(2)
        dt = .005
        history = []
        for k in range(1000):
            target, log = f.target(p,v,.3,k*dt,'coast',1.)
            history.append(np.array(log['position_velocity_target_m_s']))
            vref = history[max(0,len(history)-5)]  # 20 ms command delay
            desired = 9.81*np.radians(30.)*(vref-v[:2])
            accel += (desired-accel)*dt/.08
            v[:2] += accel*dt
            p[:2] += v[:2]*dt
        self.assertLess(np.linalg.norm(v[:2]),.03)
        self.assertGreater(p[0],0)
        self.assertLess(p[1],0)


class PositionPidRecoveryTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory(); self.addCleanup(temp.cleanup)
        self.path=Path(temp.name)/'pid.json'
        self.param=Param()
        self.param.values.update(parameters())
        self.param.values.update({k:.1 for k in SUPPRESSED_GAINS})
        for name in parameters():
            group,item=name.split('.')
            self.param.toc.toc.setdefault(group,{})[item]=object()
        self.original=dict(self.param.values)
        self.cf=SimpleNamespace(param=self.param)
        self.guard=PositionFollowPidContext(self.cf,self.path)

    def test_prepare_confirms_only_xy_changes_and_restore_recovers_exact_values(self):
        self.guard.prepare(); self.guard.verify()
        self.assertTrue(self.guard.prepared)
        self.assertEqual(set(k for k,_ in self.param.writes),set(SUPPRESSED_GAINS))
        self.assertTrue(all(self.param.values[k]==0 for k in SUPPRESSED_GAINS))
        self.guard.restore()
        self.assertEqual(self.param.values,self.original)
        self.assertFalse(self.path.exists())

    def test_partial_prepare_or_restore_retains_recovery_file(self):
        self.param.fail_once=SUPPRESSED_GAINS[2]
        with self.assertRaises(OSError): self.guard.prepare()
        self.assertFalse(self.guard.prepared)
        self.assertTrue(self.path.exists())
        self.param.fail_once=SUPPRESSED_GAINS[1]
        with self.assertRaises(OSError): self.guard.restore()
        self.assertTrue(self.path.exists())
        PositionFollowPidContext(self.cf,self.path).restore()
        self.assertEqual(self.param.values,self.original)

    def test_missing_parameter_or_failed_confirmation_makes_no_changes(self):
        with patch('Interaction.position_follow.confirm_firmware_mode_parameters',side_effect=RuntimeError('readback')):
            with self.assertRaises(RuntimeError): self.guard.prepare()
        self.assertFalse(self.path.exists()); self.assertFalse(self.param.writes)
        del self.param.toc.toc['posCtlPid']['xKff']
        with self.assertRaises(RuntimeError): self.guard.prepare()
        self.assertFalse(self.param.writes)

    def test_controller_requires_grounded_pid_and_prepares_recoverable_context(self):
        c=Controller.__new__(Controller)
        c.cf=self.cf; c.flying=False; c.log_manager=Mock()
        c.args=SimpleNamespace(drone_id='unit-test',controller_type='pid',
            skip_takeoff=False,skip_landing=False,calibrate=False,interaction=True)
        c.mission={'Interaction':{'config':{'behavior':'level_coast','level_coast':{'command_mode':'position'}}}}
        with patch('Interaction.position_follow.PositionFollowPidContext',return_value=self.guard):
            for key in ('skip_takeoff','skip_landing'):
                setattr(c.args,key,True)
                with self.assertRaises(ValueError): c._prepare_offboard_position_control()
                setattr(c.args,key,False)
            c._prepare_offboard_position_control()
        self.assertIs(c.cf._offboard_position_pid,self.guard)
        self.assertTrue(self.guard.prepared)

    def test_prearm_verification_failure_prevents_arming(self):
        c=Controller.__new__(Controller)
        c.args=SimpleNamespace(ground_test=False,skip_arm=False)
        c.verify_contact_attitude_final_prearm_ready=Mock()
        c.cf=Mock()
        c._offboard_position_pid=SimpleNamespace(verify=Mock(side_effect=RuntimeError('PID changed')))
        with self.assertRaisesRegex(RuntimeError,'PID changed'): c.arm()
        c.cf.platform.send_arming_request.assert_not_called()

    def test_controller_prepares_pid_if_either_phase_uses_position(self):
        for contact in ('orientation', 'position'):
            for coast in ('orientation', 'position'):
                with self.subTest(contact=contact, coast=coast):
                    c = Controller.__new__(Controller)
                    c.cf = SimpleNamespace()
                    c.flying = False
                    c.log_manager = Mock()
                    c.args = SimpleNamespace(drone_id='unit-test', controller_type='pid',
                        skip_takeoff=False, skip_landing=False, calibrate=False, interaction=True)
                    c.mission = {'Interaction': {'config': {'behavior': 'level_coast',
                        'level_coast': {'command_mode': contact, 'coast_command_mode': coast}}}}
                    guard = Mock()
                    with patch('Interaction.position_follow.PositionFollowPidContext', return_value=guard), \
                            patch('pathlib.Path.exists', return_value=False):
                        enabled = 'position' in (contact, coast)
                        if enabled:
                            c.args.skip_takeoff = True
                            with self.assertRaisesRegex(ValueError, 'grounded'):
                                c._prepare_offboard_position_control()
                            guard.prepare.assert_not_called()
                            c.args.skip_takeoff = False
                        c._prepare_offboard_position_control()
                    self.assertEqual(guard.prepare.call_count, int(enabled))
                    self.assertEqual(guard.restore.call_count, int(enabled))
                    if enabled:
                        self.assertIs(c.cf._offboard_position_pid, guard)
                        logged = c.log_manager.add_log_entry.call_args.args[1]
                        self.assertEqual(logged['command_mode'], contact)
                        self.assertEqual(logged['coast_command_mode'], coast)

    def test_cleanup_only_restores_after_confirmed_landing(self):
        from Interaction.tests.test_controller_logging_cleanup import methods
        stop=methods({'stop'},time=SimpleNamespace(time=lambda:10),logger=Mock())['stop']
        for fail in (False,True):
            actions=[]
            guard=SimpleNamespace(restore=Mock(side_effect=lambda:actions.append('restore')))
            c=SimpleNamespace(mission_start_time=0,servo=None,bat_logger=None,mocap=None,
                force_sensor=None,rpi_power_monitor=None,log_manager=None,tracker_process=None,
                blinker_process=None,smooth_controller=None,led=None,flying=True,
                _offboard_position_pid=guard,disconnect=Mock(side_effect=lambda:actions.append('disconnect')))
            def land():
                actions.append('land')
                if fail: raise ConnectionError('link lost')
                c.flying=False
            c.land=land
            if fail:
                with self.assertRaises(ConnectionError): stop(c)
                self.assertEqual(actions,['land','disconnect'])
            else:
                stop(c)
                self.assertEqual(actions,['land','restore','disconnect'])


if __name__ == '__main__':
    unittest.main()
