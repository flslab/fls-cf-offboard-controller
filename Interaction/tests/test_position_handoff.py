import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np

from Interaction.position_handoff import (
    PositionHandoff, validate_position_handoff, handoff_enabled,
    add_handoff_logs, incoming_attitude_command,
)
from Interaction.tests.test_position_follow import parameters


class PositionHandoffTests(unittest.TestCase):
    def update(self, h, t, *, p=(0,0,1), v=(0,0,0), mp=(0,0,1), mv=(0,0,0),
               yaw=0., rp=(0.,0.), rates=(0,0,0), allow_hold=True, incoming=(0,0), motion_time=None):
        return h.update(position=p, velocity=v, attitude_rpy=[*rp,yaw], angular_velocity=rates,
            motion_position=mp, motion_velocity=mv, timestamp=t,
            motion_time=t if motion_time is None else motion_time, height=1.,
            incoming_rp_deg=incoming, allow_hold=allow_hold)

    def test_default_applies_to_all_coasts_but_not_direct_hold_or_other_behaviors(self):
        for mode in ('position','orientation','scurve'):
            c={'behavior':'level_coast','level_coast':{'coast_command_mode':mode}}
            self.assertTrue(handoff_enabled(c))
            self.assertIn('POSITION_HANDOFF',add_handoff_logs({}, {'Interaction':{'config':c}}))
            c['level_coast']['coasting_enabled']=False
            c['level_coast']['position_handoff']={'enabled':True}
            self.assertFalse(handoff_enabled(c))
            selected={'existing_group':{}}
            self.assertIs(add_handoff_logs(selected, {'Interaction':{'config':c}}), selected)
        self.assertFalse(handoff_enabled({'behavior':'translation_inertia'}))

    def test_actual_cascade_error_does_not_jump_with_estimator_bias_or_position_jump(self):
        for yaw in (0., .8, -1.7):
            h=PositionHandoff(parameters(), {}, {})
            self.update(h,0,yaw=yaw,mv=(0,.01,0))
            last_acc=None
            for k in range(1,65):
                p=np.array([.003 if k>=20 else 0.,0.,1.])
                v=np.array([-.04,.055,0.]) if k>=20 else np.array([0.,.01,0.])
                cmd,log=self.update(h,k*.01,p=p,v=v,yaw=yaw,mv=(0,.01,0))
                c,s=math.cos(yaw),math.sin(yaw);r=np.array([[c,s],[-s,c]])
                error=h.kp*(r@(cmd[:2]-p[:2]))-r@v[:2]
                implied_acc=r.T@(9.81*np.radians(h.kv)*error)
                np.testing.assert_allclose(implied_acc,log['position_handoff_acceleration_m_s2'],atol=1e-12)
                if last_acc is not None:
                    self.assertLessEqual(np.linalg.norm(implied_acc-last_acc),.020000001)
                last_acc=implied_acc
            self.assertTrue(h.complete)
            np.testing.assert_allclose(h.anchor,[0,0,1])

    def test_capture_preserves_outgoing_command_and_bounds_combined_acceleration(self):
        h=PositionHandoff(parameters(), {}, {})
        cmd,log=self.update(h,0,mv=(.25,-.2,0),v=(.25,-.2,0),incoming=(1.,-2.))
        np.testing.assert_allclose(log['position_handoff_nominal_pitch_roll_deg'],[-2.,1.],atol=1e-12)
        old=np.array(log['position_handoff_acceleration_m_s2'])
        for k in range(1,100):
            _,log=self.update(h,k*.01,mv=(.25,-.2,0),v=(.25,-.2,0),allow_hold=False)
            a=np.array(log['position_handoff_acceleration_m_s2'])
            self.assertLessEqual(np.linalg.norm(a),.800000001)
            self.assertLessEqual(np.linalg.norm(a-old),.020000001)
            old=a
        self.assertFalse(h.complete)
        self.assertLess(np.dot(old,[.25,-.2]),0)

    def test_hold_anchor_is_physical_not_frozen_bias_offset_and_remains_corrected(self):
        h=PositionHandoff(parameters(), {}, {})
        for k in range(60):
            cmd,log=self.update(h,k*.01,p=(.01,.02,1),mp=(.01,.02,1),v=(0,.055,0))
        self.assertTrue(h.complete)
        np.testing.assert_allclose(h.anchor,[.01,.02,1])
        self.assertAlmostEqual(cmd[1],.02+.055/2.1)
        for k in range(60,85):
            cmd,log=self.update(h,k*.01,p=(.01,.02,1),mp=(.01,.02,1),v=(0,0,0))
        np.testing.assert_allclose(cmd,h.anchor,atol=1e-12)
        # A physical displacement after capture is corrected toward this anchor.
        for k in range(85,110):
            cmd,log=self.update(h,k*.01,p=(.01,.04,1),mp=(.01,.04,1),v=(0,.055,0))
        self.assertLess(log['position_handoff_acceleration_m_s2'][1],0)
        np.testing.assert_allclose(h.anchor,[.01,.02,1])

    def test_speed_tilt_rate_acceleration_and_ack_each_prevent_early_hold(self):
        for overrides in ({'mv':(.03,0,0)}, {'rp':(.02,0)}, {'rates':(.2,0,0)}, {'allow_hold':False}):
            with self.subTest(overrides=overrides):
                h=PositionHandoff(parameters(), {}, {})
                for k in range(90):self.update(h,k*.01,**overrides)
                self.assertFalse(h.complete)
        h=PositionHandoff(parameters(), {}, {'settle_acceleration_m_s2':.05})
        for k in range(70):
            self.update(h,k*.01,mv=((.019 if k%2 else -.019),0,0))
        self.assertFalse(h.complete)

    def test_duplicate_vicon_does_not_satisfy_dwell_and_bad_samples_abort(self):
        h=PositionHandoff(parameters(), {}, {})
        for k in range(30):self.update(h,k*.01)
        for k in range(30,33):self.update(h,k*.01,motion_time=.29)
        self.assertFalse(h.complete)
        with self.assertRaisesRegex(ValueError,'stale'):self.update(h,.34,motion_time=.29)
        with self.assertRaisesRegex(ValueError,'fresh'):self.update(h,.32)
        h=PositionHandoff(parameters(), {}, {})
        self.update(h,0)
        with self.assertRaisesRegex(ValueError,'fresh'):self.update(h,.2)
        with self.assertRaises(ValueError):self.update(PositionHandoff(parameters(),{},{}),0,v=(np.nan,0,0))

    def test_timeout_and_infeasible_commands_do_not_silently_freeze(self):
        h=PositionHandoff(parameters(), {}, {'timeout_s':.5})
        for k in range(51):self.update(h,k*.01,mv=(.2,0,0))
        with self.assertRaisesRegex(RuntimeError,'timeout'):self.update(h,.51,mv=(.2,0,0))
        with self.assertRaisesRegex(ValueError,'envelope'):
            self.update(PositionHandoff(parameters(),{},{}),0,incoming=(20,0))
        with self.assertRaisesRegex(ValueError,'velocity limits'):
            self.update(PositionHandoff(parameters(),{},{}),0,v=(3,0,0))
        with self.assertRaisesRegex(ValueError,'offset'):
            self.update(PositionHandoff(parameters(),{},{}),0,v=(1.5,0,0))

    def test_slower_vicon_stream_advances_dwell_only_on_new_samples(self):
        h=PositionHandoff(parameters(), {}, {})
        for k in range(60):
            self.update(h,k*.01,motion_time=(k//2)*.02)
        self.assertTrue(h.complete)

    def test_config_and_stale_command_rejected(self):
        for opts in ({'enabled':1},{'transition_s':0},{'timeout_s':.2},{'max_vicon_skew_s':.1},{'unknown':1}):
            with self.assertRaises(ValueError):validate_position_handoff(opts)
        lm=SimpleNamespace(get_nearest_group_log_data=Mock(return_value=({'controller.roll':1.,'controller.pitch':2.},0.)),
            get_latest_group_log_time=Mock(return_value=1.))
        np.testing.assert_equal(incoming_attitude_command(lm,1.,1.01,.05),[1.,2.])
        with self.assertRaisesRegex(RuntimeError,'fresh'):incoming_attitude_command(lm,1.,1.2,.05)

    def test_delayed_plant_with_bias_switch_stops_without_centimeter_reverse(self):
        # Independent simple lagged planar plant, not flight validation.
        h=PositionHandoff(parameters(), {}, {})
        p=np.array([0.,0.,1.]);v=np.array([.02,0.,0.]);acc=np.zeros(2);queue=[];positions=[]
        dt=.01
        for k in range(250):
            bias=np.array([.055,0.,0.]) if k>=10 else np.zeros(3)
            target,log=self.update(h,k*dt,p=p,v=v+bias,mp=p,mv=v)
            error=h.kp*(target[:2]-p[:2])-(v+bias)[:2]
            queue.append(9.81*np.radians(h.kv)*error)
            desired=queue[max(0,k-2)]
            acc+=(desired-acc)*dt/.08
            v[:2]+=acc*dt;p[:2]+=v[:2]*dt;positions.append(p[0])
        self.assertTrue(h.complete)
        self.assertLess(np.linalg.norm(v[:2]),.005)
        self.assertLess(max(np.maximum.accumulate(positions)-positions),.002)


if __name__=='__main__':unittest.main()

class EstimatorRestorationDwellTests(unittest.TestCase):
    update=PositionHandoffTests.update
    def test_restoration_uses_physical_dwell_independent_of_ack_then_new_dwell_for_hold(self):
        h=PositionHandoff(parameters(),{}, {})
        for k in range(60):self.update(h,k*.01,allow_hold=False)
        self.assertTrue(h.ready_to_restore);self.assertFalse(h.complete)
        self.assertIsNone(h.settled_since)
        # A velocity-estimate jump does not move the physical anchor or veto
        # settling when the same-point Vicon observation remains stationary.
        for k in range(60,69):self.update(h,k*.01,v=(.08,0,0),allow_hold=True)
        self.assertFalse(h.complete)
        for k in range(69,73):self.update(h,k*.01,v=(.08,0,0),allow_hold=True)
        self.assertTrue(h.complete);np.testing.assert_allclose(h.anchor,[0,0,1])

    def test_tilt_rate_or_motion_reset_restoration_eligibility(self):
        for invalid in ({'rp':(.02,0)}, {'rates':(.2,0,0)}, {'mv':(.04,0,0)}):
            h=PositionHandoff(parameters(),{}, {})
            for k in range(60):self.update(h,k*.01,allow_hold=False)
            self.assertTrue(h.ready_to_restore)
            self.update(h,.6,allow_hold=False,**invalid)
            self.assertFalse(h.ready_to_restore);self.assertIsNone(h.restore_settled_since)
