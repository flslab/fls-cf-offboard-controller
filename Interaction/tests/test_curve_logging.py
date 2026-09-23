import json
from pathlib import Path
import struct
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import zlib

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
from cflib.utils.encoding import compress_quaternion
from Interaction.curve_logging import (HEADER,FLOATS,PART,VERSION,CHUNK,PART_COUNT,
    CurveAssembler,CurveRecorder,decode_event,curve_state_log_vars,normalize_state)
from Interaction.curve_log_export import reconstruct_command,export
from Interaction.post_release_firmware_control_event import FirmwareBrakeMonitor


def wire(event=1,kind=1):
    v=[0.]*60;v[1]=1;v[14:19]=[.04,13,.7,1,0];v[19:24]=[.02,12,.5,1,0]
    v[24:28]=[.5,0,.5,0];v[28]=.1
    payload=HEADER.pack(1,kind,2,event,123,event,event-1,2000000,50000,2000000,42,0,1<<8)+FLOATS.pack(*v)
    return payload+struct.pack('<I',zlib.crc32(payload))


def fragments(data,event=1):
    return [struct.pack('<BBIBB',PART,VERSION,event,i,PART_COUNT)+data[i*CHUNK:(i+1)*CHUNK] for i in range(PART_COUNT)]


def state(group,tick=50000):
    d={}
    if group=='FIRMWARE_KIN':
        d={f'stateEstimateZ.{x}':0 for x in ['x','y','z','vx','vy','vz','rateRoll','ratePitch','rateYaw']}
        d['stateEstimateZ.quat']=compress_quaternion([0,0,0,1]);d['stateEstimateZ.vy']=500
    if group=='FIRMWARE_ACT':
        d={f'stateEstimateZ.a{x}':0 for x in 'xyz'};d['stateEstimateZ.az']=9810
        d.update({f'motor.m{x}':0 for x in range(1,5)});d['pm.vbat']=8
    if group=='ATT_DES':d={'controller.roll':-5.729578,'controller.pitch':0}
    if group=='CURVE_STATUS':d={'hlCommander.curveId':1,'hlCommander.curveDrop':0,'hlCommander.curveQ':0}
    return SimpleNamespace(group=group,cf_timestamp_ms=tick,host_receive_time_s=10.,source_snapshot_atomic=False,data=d)


class CurveLoggingTests(unittest.TestCase):
    def test_out_of_order_duplicate_and_crc(self):
        a=CurveAssembler();parts=fragments(wire())
        result=None
        for part in reversed(parts):
            result,event=a.feed(part)
        self.assertEqual(result['model_id'],42);self.assertEqual(event,1)
        a.saved(event);self.assertEqual(a.feed(parts[0]),(None,1))
        broken=bytearray(wire());broken[90]^=1
        with self.assertRaisesRegex(ValueError,'CRC'):decode_event(broken)

    def test_reject_mixed_id_bad_size_nonfinite(self):
        a=CurveAssembler()
        with self.assertRaises(ValueError):a.feed(b'bad')
        with self.assertRaises(ValueError):a.feed(fragments(wire())[0][:-1])
        with self.assertRaises(ValueError):
            for p in fragments(wire(),event=2):a.feed(p)
        data=bytearray(wire());struct.pack_into('<f',data,HEADER.size,float('nan'))
        struct.pack_into('<I',data,len(data)-4,zlib.crc32(data[:-4]))
        with self.assertRaises(ValueError):decode_event(data)

    def test_fragment_conflict_and_bounded_storage(self):
        a=CurveAssembler();p=fragments(wire())[0];a.feed(p)
        with self.assertRaisesRegex(ValueError,'conflicting'):a.feed(p[:-1]+bytes([p[-1]^1]))
        for n in range(8):a.feed(fragments(wire(n+1),event=n+1)[0])
        with self.assertRaisesRegex(ValueError,'evicted'):a.feed(fragments(wire(9),event=9)[0])
        self.assertLessEqual(len(a.pending),8)

    def test_state_units_gravity_and_commands(self):
        k=normalize_state(state('FIRMWARE_KIN'));self.assertEqual(k['velocity_m_s'],[0,.5,0])
        self.assertAlmostEqual(k['roll_pitch_deg'][0],0,places=2)
        self.assertEqual(normalize_state(state('FIRMWARE_ACT'))['acceleration_world_m_s2'],[0,0,0])
        e=decode_event(wire());c=reconstruct_command(e,50000)
        self.assertAlmostEqual(c[0],-5.729578,places=5)
        self.assertAlmostEqual(reconstruct_command(e,50500)[0],0)
        self.assertIsNone(reconstruct_command(dict(e,event='hold'),50500))

    def test_split_gain_bias_clamp_and_clock_wrap(self):
        e=decode_event(wire());e['cf_timestamp_ms']=0xfffffa
        e['applied_us_mod32']=10;e['plan_origin_us_mod32']=0xfffffff0
        a=e['axes'][0];a['split']=True;a['coefficients'][1][0]=.2
        self.assertAlmostEqual(reconstruct_command(e,250)[0],-math_degrees(.2),places=4)
        a['response']['gain']=.8;a['response']['bias_rad']=.01
        self.assertAlmostEqual(reconstruct_command(e,250)[0],-math_degrees((.2-.01)/.8),places=4)

    def test_recorder_jsonl_ack_export_and_shutdown(self):
        with TemporaryDirectory() as tmp:
            cf=Mock();logger=SimpleNamespace(args=SimpleNamespace(drone_id='lb11'),add_cf_packet_listener=Mock(return_value=Mock()))
            rec=CurveRecorder(cf,logger,directory=tmp,tag='trial',user_id='P01')
            for part in fragments(wire()):
                p=CRTPPacket();p.set_header(CRTPPort.SETPOINT_HL,2);p.data=part;rec._packet(p)
            for g in ('FIRMWARE_KIN','FIRMWARE_ACT','ATT_DES','CURVE_STATUS'):rec._state(state(g))
            rec.close();rec.close();cf.remove_port_callback.assert_called_once()
            self.assertEqual(cf.send_packet.call_count,1)
            events=[json.loads(x) for x in Path(tmp,'trial.curves.jsonl').read_text().splitlines()]
            self.assertTrue(events[-1]['complete']);self.assertEqual(events[1]['user_id'],'P01')
            output=export(Path(tmp,'trial.curves.jsonl'))
            self.assertIn('P01',Path(output).read_text())
            with self.assertRaises(FileExistsError):export(Path(tmp,'trial.curves.jsonl'))

    def test_state_gap_is_explicit_and_export_requires_opt_in(self):
        with TemporaryDirectory() as tmp:
            cf=Mock();logger=SimpleNamespace(args=SimpleNamespace(),add_cf_packet_listener=Mock(return_value=Mock()))
            rec=CurveRecorder(cf,logger,directory=tmp,tag='trial',events_enabled=False)
            for g in ('FIRMWARE_KIN','FIRMWARE_ACT','ATT_DES'):rec._state(state(g))
            rec._state(state('FIRMWARE_KIN',50030));rec.close()
            summary=json.loads(Path(tmp,'trial.curves.jsonl').read_text().splitlines()[-1])
            self.assertTrue(summary['curve_events_complete']);self.assertFalse(summary['continuous_states_complete'])
            self.assertEqual(summary['state_gap_counts'],{'FIRMWARE_KIN':1})
            with self.assertRaises(ValueError):export(Path(tmp,'trial.curves.jsonl'))
            self.assertTrue(Path(export(Path(tmp,'trial.curves.jsonl'),allow_incomplete=True)).exists())

    def test_failed_state_drain_cannot_write_complete_summary(self):
        with TemporaryDirectory() as tmp:
            cf=Mock();logger=SimpleNamespace(args=SimpleNamespace(),add_cf_packet_listener=Mock(return_value=Mock()))
            rec=CurveRecorder(cf,logger,directory=tmp,tag='trial',events_enabled=False)
            for g in ('FIRMWARE_KIN','FIRMWARE_ACT','ATT_DES'):rec._state(state(g))
            close=rec.states.close
            def fail():
                close()
                raise OSError('state writer failed on close')
            with patch.object(rec.states,'close',side_effect=fail):
                with self.assertRaises(OSError):rec.close()
            summary=json.loads(Path(tmp,'trial.curves.jsonl').read_text().splitlines()[-1])
            self.assertFalse(summary['complete'])
            self.assertEqual(summary['errors'],1)

    def test_controller_checks_version_before_enabling_and_calibration_is_states_only(self):
        from Interaction.tests.test_controller_logging_cleanup import methods
        setup=methods({'setup_logging'},logger=Mock())['setup_logging']
        for calibrate,version_present in ((False,True),(False,False),(True,True)):
            ctrl=SimpleNamespace(args=SimpleNamespace(log=True,illumination=False,hover=False,
                    droneless=False,calibrate=calibrate,log_dir='unused',tag='trial',cf_log_period=10),
                cfg=SimpleNamespace(LOG_VARS={}),mission={'Interaction':{'config':{'wrench_interaction':{
                    'firmware_auto_brake':{'curve_log':{'enabled':True}}}}}},
                _is_interaction_application=lambda:True,_uses_onboard_wrench_state=lambda:True,
                _uses_vicon_velocity_for_free_stop=lambda:False,
                firmware_auto_brake_enabled=True,firmware_auto_brake_mode='scurve',
                firmware_response_model_config={'enabled':True},cf=Mock())
            ctrl.cf.param.toc.toc={'hlCommander':dict.fromkeys(('curveVer','curveLog')) if version_present else {}}
            with patch('Interaction.log_manager.InteractionLogger'),patch('Interaction.curve_logging.CurveRecorder') as recorder,patch('Interaction.firmware_parameter_confirmation.confirm_firmware_mode_parameters') as confirm:
                if not version_present:
                    with self.assertRaisesRegex(RuntimeError,'curve event logging protocol'):setup(ctrl)
                    recorder.assert_not_called();ctrl.cf.param.set_value.assert_not_called();continue
                setup(ctrl)
                self.assertEqual(recorder.call_args.kwargs['events_enabled'],not calibrate)
                self.assertEqual(ctrl.cf.param.set_value.call_args_list[0].args,('hlCommander.curveLog','0'))
                if calibrate:
                    confirm.assert_not_called();self.assertEqual(ctrl.cf.param.set_value.call_count,1)
                else:
                    confirm.assert_called_once()
                    self.assertEqual(confirm.call_args.kwargs['expected'],{'hlCommander.curveVer':1})
                    self.assertEqual(ctrl.cf.param.set_value.call_args_list[-1].args,('hlCommander.curveLog','1'))

    def test_incomplete_tail_and_no_flight_packets(self):
        with TemporaryDirectory() as tmp:
            cf=Mock();logger=SimpleNamespace(args=SimpleNamespace(),add_cf_packet_listener=Mock(return_value=Mock()))
            rec=CurveRecorder(cf,logger,directory=tmp,tag='trial');rec.close()
            summary=json.loads(Path(tmp,'trial.curves.jsonl').read_text().splitlines()[-1])
            self.assertFalse(summary['complete']);cf.send_packet.assert_not_called()
            with self.assertRaises(ValueError):export(Path(tmp,'trial.curves.jsonl'))

    def test_config_logging_bandwidth_and_imu_warning(self):
        logs=curve_state_log_vars({});self.assertEqual(logs['ATT_DES']['log_period_ms'],10)
        for g in logs.values():
            sizes={'uint32_t':4,'int16_t':2,'uint16_t':2,'float':4,'uint8_t':1}
            self.assertLessEqual(sum(sizes[v['type']] for k,v in g.items() if k!='log_period_ms'),26)
        m=FirmwareBrakeMonitor(started_monotonic_s=1,baseline_receipt_time_s=0,baseline_timeouts=0)
        log={'hlCommander.pRelMode':2,'hlCommander.pRelStale0':1,'hlCommander.pRelGap0':.213,'hlCommander.pRelAutoSt':1,'hlCommander.pRelReady':0}
        result=m.observe(log,receipt_time_s=1.01,now_wall_s=1.01,now_monotonic_s=1.01)
        events=dict(result['events']);self.assertTrue(events['Firmware Release Using IMU Prediction']['warning_only'])

    def test_curve_status_registers_against_bolt_toc(self):
        from cflib.crazyflie.log import Log, LogConfig
        # Names exported by the paired Bolt firmware, independent of the
        # generated host configuration. readyErr belongs to pRelVicon.
        exported = {
            'hlCommander.curveDrop', 'hlCommander.curveId',
            'hlCommander.curveQ', 'hlCommander.pRelGap0',
            'hlCommander.pRelStale0', 'pRelVicon.readyErr',
        }
        cf = Mock()
        log = Log(cf)
        log.toc = Mock()
        log.toc.get_element_by_complete_name.side_effect = (
            lambda name: SimpleNamespace() if name in exported else None)
        group = curve_state_log_vars({})['CURVE_STATUS']
        config = LogConfig('CURVE_STATUS', group['log_period_ms'] * 10)
        for name, value in group.items():
            if name != 'log_period_ms':
                config.add_variable(name, value['type'])
        log.add_config(config)
        self.assertTrue(config.valid)
        self.assertEqual({v.name for v in config.variables}, exported)


def math_degrees(x):
    import math
    return math.degrees(x)
