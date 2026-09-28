"""No device connection: atomic friction release, protocol and mode gates."""
import math
import struct
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import zlib

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
from Interaction.firmware_analytic_profile import profile_parameters
from Interaction.post_release_firmware_control_event import (
    encode_pi_release_command, parse_pi_release_ack, friction_release_options,
    handoff_pi_release_to_firmware, FRICTION_PACKET)
from Interaction.curve_logging import decode_event, HEADER, FLOATS, TIMING, FRICTION, CurveAssembler
from Interaction.tests.test_curve_logging import wire, fragments
from Interaction.tests.test_firmware_auto_brake_preflight import FirmwareAutoBrakePreflightTests
from Interaction.tests.test_scurve_firmware_preflight import SCurvePreflightTests


def profile(**kwargs):
    return dict(shape='velocity_scurve', execution='attitude', tail_s=1.2,
        single_s=1.2, handoff='curve_endpoint_forward', feedback='unified_vicon15',
        response_compensation=True, state_matched_start=False,
        friction_from_interaction=True, **kwargs)


class FrictionTests(unittest.TestCase):
    def request(self, mu=None):
        return encode_pi_release_command(session_id=123, sequence=7, arduino_sample_ms=42,
            pi_receive_monotonic_ns=1_000_000, send_monotonic_ns=2_000_000,
            kinetic_friction_coefficient=mu)[0]

    def test_legacy_event_remains_byte_identical(self):
        self.assertEqual(self.request(), struct.pack('<BBHIII',15,1,7,123,42,1000))
        self.assertEqual(friction_release_options({},.1),{})

    def test_mu_is_in_same_packet_and_full_echo_identity(self):
        for mu in (0.,.01,.1,1.,10.):
            request=self.request(mu)
            self.assertEqual(len(request),20)
            self.assertAlmostEqual(FRICTION_PACKET.unpack(request)[-1],mu,places=6)
            packet=CRTPPacket();packet.set_header(CRTPPort.SETPOINT_HL,0)
            packet.data=request+b'\0'
            self.assertTrue(parse_pi_release_ack(packet,request_payload_hex=request.hex(),
                ack_receive_monotonic_ns=3)['event_queued_by_firmware'])
            packet.data=self.request(.05)+b'\0'
            self.assertIsNone(parse_pi_release_ack(packet,request_payload_hex=request.hex(),ack_receive_monotonic_ns=3))
        for mu in (-1,11,math.nan,math.inf,True,'0.1'):
            with self.assertRaises(ValueError):self.request(mu)

    def test_alternating_conditions_sent_once_without_parameter_write(self):
        observed=[]
        for index,mu in enumerate((.1,.01,.1,.01)):
            cf=Mock();callbacks=[]
            cf.add_port_callback.side_effect=lambda _,callback:callbacks.append(callback)
            def send(packet):
                observed.append(FRICTION_PACKET.unpack(packet.data)[-1])
                ack=CRTPPacket();ack.set_header(CRTPPort.SETPOINT_HL,0);ack.data=packet.data+b'\0'
                callbacks[0](ack)
            cf.send_packet.side_effect=send
            options=friction_release_options({'mode':'scurve','analytic_profile':profile()},mu)
            result=handoff_pi_release_to_firmware(cf,session_id=123,sequence=index,
                arduino_sample_ms=42,pi_receive_monotonic_ns=0,monotonic_ns=lambda:1000,
                firmware_auto_brake_armed=True,**options)
            self.assertEqual(result['kinetic_friction_coefficient'],mu)
            self.assertEqual(result['release_wire_version'],2)
            cf.send_packet.assert_called_once();cf.param.set_value.assert_not_called()
        for actual,mu in zip(observed,(.1,.01,.1,.01)):self.assertAlmostEqual(actual,mu,places=6)

    def test_only_supported_profile_and_capability_before_write(self):
        self.assertEqual(profile_parameters(profile())['hlCommander.pRelFric'],1)
        for key,value in [('friction_from_interaction',1),('state_matched_start',True),
                           ('response_compensation',False),('shape','single_position_polynomial')]:
            bad=profile();bad[key]=value
            with self.assertRaises(ValueError):profile_parameters(bad)
        ctrl=FirmwareAutoBrakePreflightTests().controller(dict(enabled=True,mode='scurve',
            response_time_s=.14,command_mode='attitude',analytic_profile=profile(),response_model={'enabled':True}))
        ctrl.prepare_firmware_auto_brake();p=SCurvePreflightTests().velocity_params()
        ctrl.cf=SimpleNamespace(param=p)
        for key,value in ctrl._firmware_analytic_expected.items():
            group,name=key.split('.');p.toc.toc.setdefault(group,{})[name]=None;p.replies[key]=str(value)
        p.toc.toc['hlCommander']['pRelEnd']=None;p.replies['hlCommander.pRelEnd']='1'
        with self.assertRaisesRegex(RuntimeError,'pRelMuVer'):ctrl._setup_firmware_auto_brake_params()
        p.set_value.assert_not_called()

    def test_omitted_option_resets_previous_friction_mode(self):
        tests=SCurvePreflightTests();ctrl=tests.velocity_controller();p=tests.modern_velocity_params()
        p.toc.toc['hlCommander']['pRelFric']=None;p.replies['hlCommander.pRelFric']='1'
        p.set_value.side_effect=lambda key,value:p.replies.update({key:value})
        ctrl.cf=SimpleNamespace(param=p);ctrl._setup_firmware_auto_brake_params()
        self.assertIn(('hlCommander.pRelFric','0'),[c.args for c in p.set_value.call_args_list])
        self.assertIn('hlCommander.pRelFric',p.requested)

    def test_v3_friction_metadata_atomic_crc_and_backward_compatibility(self):
        data=bytearray(wire(velocity=True))
        offset=HEADER.size+FLOATS.size+TIMING.size
        FRICTION.pack_into(data,offset,1|2|8,.01,.18,.18,5.999,3.0)
        struct.pack_into('<I',data,len(data)-4,zlib.crc32(data[:-4]))
        assembler=CurveAssembler();event=None
        for part in reversed(fragments(data)):event,_=assembler.feed(part)
        self.assertTrue(event['friction']['time_limit_applied'])
        self.assertAlmostEqual(event['friction']['kinetic_mu'],.01)
        self.assertEqual(event['friction']['planned_distance_m'],3.)
        self.assertEqual(event['timing']['plan_compute_us'],17)
        for version in (1,2):self.assertNotIn('friction',decode_event(wire(version=version)))
        FRICTION.pack_into(data,offset,8,.01,.18,.18,5.999,3.)
        struct.pack_into('<I',data,len(data)-4,zlib.crc32(data[:-4]))
        with self.assertRaisesRegex(ValueError,'enabled'):decode_event(data)
