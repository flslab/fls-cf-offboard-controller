"""Geometry, epoch and grounded-setup checks; no radio or flight access."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np

from Interaction import marker_geometry as geometry
from Interaction.tests.test_firmware_parameter_confirmation import AsyncParams
from controller import Controller

class MarkerGeometryTests(unittest.TestCase):
    def config(self,coast=True):
        return {'behavior':'level_coast','level_coast':{'coasting_enabled':coast},
            'wrench_interaction':{'firmware_auto_brake':{'analytic_profile':{'marker_from_imu_m':[0,0,.086]}}}}

    def test_no_coast_and_other_behaviors_do_not_request_logs_or_geometry(self):
        for c in (self.config(False),dict(self.config(),behavior='other')):
            self.assertFalse(geometry.requested(c))
            selected={};self.assertIs(geometry.add_logs(selected,{'Interaction':{'config':c}}),selected)
        c=self.config();logs=geometry.add_logs({}, {'Interaction':{'config':c}})
        self.assertEqual(len(logs['MOCAP_REFERENCE'])-1,7)
        self.assertEqual(logs['MOCAP_REFERENCE']['mocapRef.age']['type'],'FP16')

    def test_invalid_offsets_rejected(self):
        for arm in ([0,0,.301],[.2,.2,.2],[0,0,float('nan')],[0,0,True],[0,0],None,'0,0,.086'):
            with self.subTest(arm=arm),self.assertRaises(ValueError):geometry.parameters(arm)
        self.assertEqual(geometry.parameters([0,0,.086])['mocapMarker.z'],.086)

    def packet(self):
        return {'time':10.,'mocapRef.age':.01,**{'mocapRef.'+a:v for a,v in zip(('x','y','z','vx','vy','vz'),(1,2,3,.1,.2,0))}}

    def test_prediction_preserves_observation_epoch_for_settling_dwell(self):
        logger=SimpleNamespace(get_nearest_group_log_data=lambda *_:(self.packet(),0))
        p,v,t,skew=geometry.motion_reference(logger,10.005,.035)
        np.testing.assert_allclose(p,[1.0015,2.003,3]);np.testing.assert_allclose(v,[.1,.2,0])
        self.assertAlmostEqual(t,9.99);self.assertAlmostEqual(skew,-.015)
        self.assertAlmostEqual(geometry.motion_reference(logger,10.01,.035)[2],t)

    def test_stale_future_and_nonfinite_packets_rejected(self):
        for field,value in [('mocapRef.age',.036),('mocapRef.age',-.001),('mocapRef.vx',float('nan')),
                            ('time',9),('time',11)]:
            packet=dict(self.packet(),**{field:value})
            logger=SimpleNamespace(get_nearest_group_log_data=lambda *_:(packet,0))
            with self.subTest(field=field,value=value),self.assertRaises(ValueError):geometry.motion_reference(logger,10,.035)
        logger=SimpleNamespace(get_nearest_group_log_data=lambda *_:(None,None))
        with self.assertRaises(ValueError):geometry.motion_reference(logger,10,.035)

    def fixture(self,coast=True):
        ctrl=Controller.__new__(Controller);ctrl.mission={'Interaction':{'config':self.config(coast)}}
        ctrl.args=SimpleNamespace(calibrate=False);p=AsyncParams({})
        p.toc=SimpleNamespace(toc={'mocapMarker':dict.fromkeys(('api','enable','x','y','z'))})
        p.replies={'mocapMarker.api':'1'};p.set_value=Mock(side_effect=lambda k,v:p.replies.update({k:v}))
        ctrl.cf=SimpleNamespace(param=p);return ctrl,p

    def test_offset_written_disabled_then_enabled_and_freshly_confirmed(self):
        c,p=self.fixture();c._prepare_marker_geometry()
        writes=[x.args for x in p.set_value.call_args_list]
        self.assertEqual(writes[0],('mocapMarker.enable','0'))
        self.assertEqual(writes[-1],('mocapMarker.enable','1'))
        self.assertNotIn('mocapMarker.api',[x[0] for x in writes]);self.assertTrue(c.marker_geometry_enabled)
        self.assertIn('mocapMarker.api',p.requested)

    def test_old_firmware_and_airborne_configuration_rejected_before_writes(self):
        for mode in ('old','flying','skip_takeoff','vicon_full_pose'):
            c,p=self.fixture()
            if mode=='old':p.toc.toc={}
            elif mode=='flying':c.flying=True
            else:setattr(c.args,mode,True)
            with self.subTest(mode=mode),self.assertRaises((RuntimeError,ValueError)):c._prepare_marker_geometry()
            p.set_value.assert_not_called()

    def test_no_coast_clears_geometry_on_supporting_firmware(self):
        c,p=self.fixture(False);p.replies['mocapMarker.enable']='1';c._prepare_marker_geometry()
        self.assertFalse(c.marker_geometry_enabled)
        self.assertEqual([x.args for x in p.set_value.call_args_list],[('mocapMarker.enable','0')])
        c,p=self.fixture(False);p.toc.toc={};c._prepare_marker_geometry();p.set_value.assert_not_called()
