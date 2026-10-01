"""Distance/deceleration configuration, capability gates, and event ABI; no flight."""
import math
import struct
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import zlib

from Interaction.tests import test_firmware_auto_brake_preflight as base
from Interaction.tests import test_scurve_firmware_preflight as scurve
from Interaction.tests.test_curve_logging import wire, fragments
from Interaction.curve_logging import (
    HEADER, FLOATS, TIMING, FRICTION, STOP_REQUEST, CurveAssembler, decode_event)
from Interaction.post_release_firmware_control_event import firmware_release_rejection_diagnostics


def controller(**changes):
    profile=dict(shape='velocity_scurve', execution='attitude', tail_s=1.2,
        single_s=1.2, handoff='curve_endpoint_forward', feedback='unified_vicon15',
        response_compensation=True, rate_feedforward=False, state_matched_start=False,
        position_tracking_bandwidth=3)
    mode=dict(enabled=True, mode='scurve', response_time_s=.14, command_mode='attitude',
        analytic_profile=profile, response_model={'enabled':True})
    mode.update(changes)
    return base.FirmwareAutoBrakePreflightTests().controller(mode)


class StopRequestTests(unittest.TestCase):
    def params(self, ctrl):
        p=scurve.SCurvePreflightTests().velocity_params()
        for key,value in {**ctrl._firmware_analytic_expected,
                'hlCommander.pRelReqVer':1,'hlCommander.pRelScA':3.6,'hlCommander.pRelEnd':1}.items():
            group,name=key.split('.');p.toc.toc.setdefault(group,{})[name]=None;p.replies[key]=str(value)
        p.replies['pRelResp.runtime']='1'
        p.set_value.side_effect=lambda key,value:p.replies.update({key:value})
        ctrl.cf=SimpleNamespace(param=p);ctrl.cfg=SimpleNamespace(PID_VALUES={});ctrl.use_flowdeck=False
        ctrl.args.drone_id='fixture'
        ctrl.log_manager=SimpleNamespace(curve_recorder=SimpleNamespace(ids={},events=Mock()))
        return p

    def setup(self, ctrl):
        with patch('Interaction.firmware_response_model.confirm_pid_context',return_value={}), \
             patch('Interaction.firmware_response_model.load_model',return_value={'source_log':'fixture'}), \
             patch('Interaction.firmware_response_model.upload_model',return_value={'pRelResp.id':123}):
            ctrl._setup_firmware_auto_brake_params()

    def test_defaults_and_distance_only(self):
        for distance in (0,.2,1.0):
            c=controller(stop_distance_m=distance);c.prepare_firmware_auto_brake()
            self.assertEqual(c.firmware_auto_brake_stop_deceleration_m_s2,3.6)
            self.assertFalse(c._firmware_stop_deceleration_explicit)
            self.assertEqual(c.firmware_auto_brake_stop_distance_m,distance)
            p=self.params(c);self.setup(c)
            self.assertEqual(p.replies['hlCommander.pRelScA'],'3.6')
            self.assertEqual(float(p.replies['hlCommander.pRelScD']),distance)
            self.assertIn('hlCommander.pRelScA',p.requested)
            self.assertEqual(p.set_value.call_args_list[-1].args,('hlCommander.pRelAuto','1'))

    def test_acceleration_only_and_combined_request_read_back(self):
        for distance in (0,.5):
            c=controller(stop_distance_m=distance,stop_deceleration_m_s2=.6);c.prepare_firmware_auto_brake()
            p=self.params(c);self.setup(c)
            self.assertEqual(p.replies['hlCommander.pRelScA'],'0.6')
            for key in ('hlCommander.pRelReqVer','hlCommander.pRelScA','hlCommander.pRelScD','hlCommander.pRelScT'):
                self.assertIn(key,p.requested)
            metadata=c.log_manager.curve_recorder.events.write.call_args.args[0]
            self.assertEqual(metadata['requested_stop_distance_m'],distance)
            self.assertEqual(metadata['nominal_deceleration_m_s2'],.6)
            self.assertFalse(metadata['replanning'])

    def test_missing_capability_cannot_silently_ignore_distance(self):
        c=controller(stop_distance_m=.5);c.prepare_firmware_auto_brake();p=self.params(c)
        del p.toc.toc['hlCommander']['pRelReqVer']
        with self.assertRaisesRegex(RuntimeError,'pRelReqVer'):self.setup(c)
        p.set_value.assert_not_called()

    def test_default_clears_previous_deceleration(self):
        c=controller();c.prepare_firmware_auto_brake();p=self.params(c)
        p.replies['hlCommander.pRelScA']='0.1';self.setup(c)
        self.assertEqual(p.replies['hlCommander.pRelScA'],'3.6')

    def test_bad_acceleration_and_incompatible_profiles_rejected(self):
        for value in (0,-.1,3.61,math.nan,math.inf,True,None,'0.6'):
            with self.subTest(value=value),self.assertRaisesRegex(ValueError,'stop_deceleration'):
                controller(stop_deceleration_m_s2=value).prepare_firmware_auto_brake()
        for key,value in [('position_tracking_bandwidth',0),('state_matched_start',True),
                          ('handoff','current_position')]:
            c=controller(stop_distance_m=.5)
            mode=c.mission['Interaction']['config']['wrench_interaction']['firmware_auto_brake']
            mode['analytic_profile'][key]=value
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'stop_distance'):
                c.prepare_firmware_auto_brake()

    def test_log_roundtrip_and_backward_compatibility(self):
        for version in (1,2,3):self.assertNotIn('stop_request',decode_event(wire(version=version)))
        data=bytearray(wire(version=4,velocity=True))
        offset=HEADER.size+FLOATS.size+TIMING.size+FRICTION.size
        STOP_REQUEST.pack_into(data,offset,3,.2,3.6,.2,.64,2.735)
        struct.pack_into('<I',data,len(data)-4,zlib.crc32(data[:-4]))
        assembler=CurveAssembler()
        for fragment in reversed(fragments(data)):event,_=assembler.feed(fragment)
        r=event['stop_request'];self.assertTrue(r['distance_constrained'] and r['plan_valid'])
        self.assertAlmostEqual(r['planned_distance_m'],.2)
        self.assertEqual(event['timing']['plan_compute_us'],17)
        self.assertIsNone(r['failure_reason'])
        STOP_REQUEST.pack_into(data,offset,3,.2,.5,.2,.64,2.735)
        struct.pack_into('<I',data,len(data)-4,zlib.crc32(data[:-4]))
        with self.assertRaisesRegex(ValueError,'stop request'):decode_event(data)

    def test_rejection_explains_infeasible_cap_and_distance(self):
        for detail in (1,2,3,4,5):
            r=firmware_release_rejection_diagnostics({'hlCommander.pRelRejR':10,'hlCommander.pRelRejD':detail})
            self.assertIn('infeasible',r['firmware_reject_description'])
            self.assertNotIn('unknown',r['firmware_reject_detail_description'])
