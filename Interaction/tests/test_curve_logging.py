import csv
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
from Interaction.curve_logging import (HEADER,FLOATS,TIMING,PART,VERSION,CHUNK,PART_COUNT,
    CurveAssembler,CurveRecorder,decode_event,curve_state_log_vars,normalize_state)
from Interaction.curve_log_export import reconstruct_command,reconstruct_velocity,export
from Interaction.post_release_firmware_control_event import FirmwareBrakeMonitor


def wire(event=1,kind=1,velocity=False,version=VERSION):
    v=[0.]*60;v[1]=1;v[14:19]=[.04,13,.7,1,0];v[19:24]=[.02,12,.5,1,0]
    v[24:28]=[.5,0,.5,0];v[28]=.1
    flags=1<<8
    if velocity:
        flags|=0x10000;v[14:24]=[0.]*10
        v[28:36]=[.4,-.7,0,0,0,2.1,-2.8,1.0]
    payload=HEADER.pack(version,kind,2,event,123,event,event-1,2000000,50000,2000000,42,0,flags)+FLOATS.pack(*v)
    if version==2:payload+=TIMING.pack((1<<8)|7,17,93,240,501)
    return payload+struct.pack('<I',zlib.crc32(payload))


def fragments(data,event=1):
    count=(len(data)+CHUNK-1)//CHUNK
    return [struct.pack('<BBIBB',PART,data[0],event,i,count)+data[i*CHUNK:(i+1)*CHUNK] for i in range(count)]


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
    def test_timing_v2_and_old_v1_have_unambiguous_missing_values(self):
        e=decode_event(wire())
        self.assertEqual(e['timing']['clock'],'mcu_elapsed')
        self.assertEqual(e['timing']['plan_compute_us'],17)
        self.assertEqual(e['timing']['control_step_max_us'],93)
        self.assertEqual(e['timing']['hold_compute_us'],240)
        old=decode_event(wire(version=1))
        self.assertIsNone(old['timing']['plan_compute_us'])
        self.assertEqual(old['timing']['clock'],'unavailable')
        for flags,expected in ((2<<8,'sitl_host_elapsed'),(1<<8,'mcu_elapsed')):
            data=bytearray(wire());struct.pack_into('<I',data,HEADER.size+FLOATS.size,flags)
            struct.pack_into('<I',data,len(data)-4,zlib.crc32(data[:-4]))
            timing=decode_event(data)['timing']
            self.assertEqual(timing['clock'],expected)
            self.assertIsNone(timing['plan_compute_us'])
            self.assertIsNone(timing['control_step_max_us'])

    def test_assembler_rejects_mixed_protocol_and_acks_old_version(self):
        a=CurveAssembler();a.feed(fragments(wire())[0])
        with self.assertRaisesRegex(ValueError,'version'):
            a.feed(fragments(wire(version=1))[1])
        with TemporaryDirectory() as tmp:
            cf=Mock();logger=SimpleNamespace(args=SimpleNamespace(),add_cf_packet_listener=Mock(return_value=Mock()))
            rec=CurveRecorder(cf,logger,directory=tmp,tag='old',protocol_version=1)
            for part in fragments(wire(version=1)):
                p=CRTPPacket();p.set_header(CRTPPort.SETPOINT_HL,2);p.data=part;rec._packet(p)
            self.assertEqual(cf.send_packet.call_args.args[0].data[1],1)
            rec.close()

    def test_explicit_execution_is_independent_of_coefficient_units(self):
        for position in (False, True):
            for execution, name in ((1, 'velocity'), (2, 'position'), (3, 'attitude')):
                data = bytearray(wire(velocity=True))
                flags = (0x20000 if position else 0x10000) | (4 << 8) | (execution << 19)
                struct.pack_into('<I', data, HEADER.size - 4, flags)
                struct.pack_into('<I', data, len(data) - 4, zlib.crc32(data[:-4]))
                event = decode_event(data)
                self.assertEqual(event['command_mode'], name)
                self.assertEqual(event['coefficient_units'], 'm' if position else 'm/s')
                self.assertEqual(event['velocity_source'], 'unified_vicon15')
                self.assertTrue(event['execution_mode_explicit'])
                self.assertIsNone(reconstruct_command(event, 50000))
                self.assertIsNotNone(reconstruct_velocity(event, 50000))

    def test_position_reference_is_not_attitude_or_velocity_coefficients(self):
        data=bytearray(wire(velocity=True))
        for attitude in (False,True):
            struct.pack_into('<I',data,HEADER.size-4,0x20000|(0x40000 if attitude else 0)|(4<<8))
            struct.pack_into('<I',data,len(data)-4,zlib.crc32(data[:-4]))
            event=decode_event(data)
            self.assertEqual(event['command_mode'],'attitude' if attitude else 'position')
            self.assertEqual(event['coefficient_units'],'m')
            self.assertEqual(event['velocity_source'],'unified_vicon15')
            self.assertEqual(event['curve_axes'],['world_x','world_y'])
            self.assertNotIn('end_rad',event['axes'][0])
            self.assertIsNone(reconstruct_command(event,50000))
            self.assertIsNone(reconstruct_velocity(event,49999))
            self.assertAlmostEqual(reconstruct_velocity(event,50000)[0],-1.4,places=6)
            self.assertEqual(reconstruct_velocity(event,50500),[0.,0.])

    def test_velocity_coefficients_units_sources_and_reference(self):
        for kind in (1,2):
            e=decode_event(wire(kind=kind,velocity=True))
            self.assertEqual(e['event'],'initial' if kind==1 else 'phase_transition')
            self.assertEqual(e['command_mode'],'velocity')
            self.assertEqual(e['coefficient_units'],'m/s')
            self.assertEqual(e['attitude_source'],'ordinary')
            self.assertEqual(e['velocity_source'],'ordinary_firmware_state')
            self.assertFalse(e['response_model_used'])
            self.assertNotIn('end_rad',e['axes'][0])
            self.assertEqual(e['axes'][0]['end_value'],0.)
            self.assertIsNone(reconstruct_command(e,50000))
            self.assertIsNone(reconstruct_velocity(e,49999))
            self.assertAlmostEqual(reconstruct_velocity(e,50000)[1],.4)
            last=.4
            for tick in range(50000,50501):
                vx,vy=reconstruct_velocity(e,tick)
                self.assertAlmostEqual(vx,0.)
                self.assertGreaterEqual(vy,0.)
                self.assertLessEqual(vy,last+1e-7)
                last=vy
            self.assertEqual(reconstruct_velocity(e,50500),[0.,0.])
            self.assertIsNone(reconstruct_velocity(dict(e,event='hold'),50500))
        self.assertIsNone(reconstruct_velocity(decode_event(wire()),50000))

    def test_velocity_reference_export_is_not_mislabeled_as_attitude(self):
        with TemporaryDirectory() as tmp:
            cf=Mock();logger=SimpleNamespace(args=SimpleNamespace(),add_cf_packet_listener=Mock(return_value=Mock()))
            rec=CurveRecorder(cf,logger,directory=tmp,tag='velocity')
            for part in fragments(wire(velocity=True)):
                p=CRTPPacket();p.set_header(CRTPPort.SETPOINT_HL,2);p.data=part;rec._packet(p)
            for g in ('FIRMWARE_KIN','FIRMWARE_ACT','ATT_DES','CURVE_STATUS'):rec._state(state(g))
            rec.close()
            output=export(Path(tmp,'velocity.curves.jsonl'))
            with open(output) as stream: rows=list(csv.DictReader(stream))
            self.assertEqual(rows[0]['command_mode'],'velocity')
            self.assertAlmostEqual(float(rows[0]['reference_vy_m_s']),.4)
            self.assertEqual(rows[0]['reconstructed_roll_deg'],'')

    def test_terminal_wire_state_is_not_presented_as_measured_zero(self):
        for kind in (4, 5):
            event = decode_event(wire(kind=kind))
            self.assertIsNone(event['velocity_xy_m_s'])
            self.assertIsNone(event['position_m'])
            self.assertEqual(event['raw_unverified_state']['velocity_xy_m_s'], [0, 0])
            self.assertEqual(event['state_validity'], f'unknown_terminal_state_wire_v{VERSION}')
        self.assertEqual(decode_event(wire())['state_validity'], 'runtime_accepted')

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
            with open(Path(tmp,'trial.coefficients.csv')) as stream: coefficients=list(csv.DictReader(stream))
            self.assertEqual(len(coefficients),2)
            self.assertEqual(coefficients[0]['user_id'],'P01')
            self.assertEqual(coefficients[0]['plan_compute_us'],'17')
            self.assertEqual(coefficients[0]['hold_compute_us'],'240')
            self.assertAlmostEqual(float(coefficients[0]['c0']),.1)
            with open(output) as stream: continuous=list(csv.DictReader(stream))
            self.assertEqual(continuous[0]['control_step_max_us'],'93')
            self.assertTrue(all(k in continuous[0] for k in ('vx','vy','vz','ax','ay','az','roll_deg','pitch_deg','roll_rate_deg_s','pitch_rate_deg_s')))
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

    def test_coefficients_can_be_complete_when_continuous_samples_have_gaps(self):
        with TemporaryDirectory() as tmp:
            cf=Mock();logger=SimpleNamespace(args=SimpleNamespace(),add_cf_packet_listener=Mock(return_value=Mock()))
            rec=CurveRecorder(cf,logger,directory=tmp,tag='gap',user_id='P03')
            for part in fragments(wire()):
                p=CRTPPacket();p.set_header(CRTPPort.SETPOINT_HL,2);p.data=part;rec._packet(p)
            for g in ('FIRMWARE_KIN','FIRMWARE_ACT','ATT_DES','CURVE_STATUS'):rec._state(state(g))
            rec._state(state('FIRMWARE_KIN',50030));rec.close()
            path=Path(tmp,'gap.curves.jsonl')
            with self.assertRaisesRegex(ValueError,'incomplete'):export(path)
            output=export(path,coefficients_only=True)
            with open(output) as stream: rows=list(csv.DictReader(stream))
            self.assertEqual(rows[0]['recording_complete'],'False')
            self.assertEqual(rows[0]['curve_events_complete'],'True')
            self.assertEqual(rows[0]['curve_reference_valid'],'True')
            self.assertFalse(Path(tmp,'gap.comparison.csv').exists())

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
        for calibrate,version_present,command_mode in ((False,True,'attitude'),(False,False,'attitude'),(True,True,'attitude'),(False,True,'velocity')):
            ctrl=SimpleNamespace(args=SimpleNamespace(log=True,illumination=False,hover=False,
                    droneless=False,calibrate=calibrate,log_dir='unused',tag='trial',cf_log_period=10),
                cfg=SimpleNamespace(LOG_VARS={}),mission={'Interaction':{'config':{'wrench_interaction':{
                    'firmware_auto_brake':{'curve_log':{'enabled':True}}}}}},
                _is_interaction_application=lambda:True,_uses_onboard_wrench_state=lambda:True,
                _uses_vicon_velocity_for_free_stop=lambda:False,
                firmware_auto_brake_enabled=True,firmware_auto_brake_mode='scurve',
                firmware_response_model_config={'enabled':command_mode=='attitude'},
                firmware_brake_command_mode=command_mode,cf=Mock())
            ctrl.cf.param.toc.toc={'hlCommander':dict.fromkeys(('curveVer','curveLog')) if version_present else {}}
            ctrl.cf.param.get_value.return_value='1'
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

    def test_actual_logging_setup_accepts_analytic_attitude_and_position_v2(self):
        from Interaction.tests.test_controller_logging_cleanup import methods
        setup=methods({'setup_logging'},logger=Mock())['setup_logging']
        for mode in ('attitude','position','velocity'):
            ctrl=SimpleNamespace(args=SimpleNamespace(log=True,illumination=False,hover=False,
                droneless=False,calibrate=False,log_dir='unused',tag='trial',cf_log_period=10),
                cfg=SimpleNamespace(LOG_VARS={}),mission={'Interaction':{'config':{'wrench_interaction':{
                    'firmware_auto_brake':{'curve_log':{'enabled':True,'user_id':'P02'}}}}}},
                _is_interaction_application=lambda:True,_uses_onboard_wrench_state=lambda:True,
                _uses_vicon_velocity_for_free_stop=lambda:False,firmware_auto_brake_enabled=True,
                firmware_auto_brake_mode='scurve',firmware_response_model_config={'enabled':False},
                firmware_brake_command_mode=mode,_firmware_analytic_expected={'hlCommander.pRelVelCmd':1},cf=Mock())
            ctrl.cf.param.toc.toc={'hlCommander':dict.fromkeys(('curveVer','curveLog'))}
            ctrl.cf.param.get_value.return_value='2'
            with patch('Interaction.log_manager.InteractionLogger'),patch('Interaction.curve_logging.CurveRecorder') as recorder,patch('Interaction.firmware_parameter_confirmation.confirm_firmware_mode_parameters') as confirm:
                setup(ctrl)
                self.assertEqual(recorder.call_args.kwargs['protocol_version'],2)
                self.assertEqual(recorder.call_args.kwargs['user_id'],'P02')
                self.assertEqual(confirm.call_args.kwargs['expected'],{'hlCommander.curveVer':2})

    def test_plain_velocity_logging_needs_no_calibration_model_for_v1_or_v2(self):
        from Interaction.tests.test_controller_logging_cleanup import methods
        setup = methods({'setup_logging'}, logger=Mock())['setup_logging']
        for version in (1, 2, 99):
            with self.subTest(version=version):
                ctrl = SimpleNamespace(
                    args=SimpleNamespace(log=True, illumination=False, hover=False,
                        droneless=False, calibrate=False, log_dir='unused', tag='trial', cf_log_period=10),
                    cfg=SimpleNamespace(LOG_VARS={}),
                    mission={'Interaction': {'config': {'wrench_interaction': {
                        'firmware_auto_brake': {'command_mode': 'velocity',
                            'response_model': {'enabled': False}, 'curve_log': {'enabled': True}}}}}},
                    _is_interaction_application=lambda: True,
                    _uses_onboard_wrench_state=lambda: True,
                    _uses_vicon_velocity_for_free_stop=lambda: False,
                    firmware_auto_brake_enabled=True, firmware_auto_brake_mode='scurve',
                    firmware_response_model_config={'enabled': False},
                    firmware_brake_command_mode='velocity', _firmware_analytic_expected={}, cf=Mock())
                ctrl.cf.param.toc.toc = {'hlCommander': {'curveVer': None, 'curveLog': None}}
                ctrl.cf.param.get_value.return_value = str(version)
                with patch('Interaction.log_manager.InteractionLogger'), \
                     patch('Interaction.curve_logging.CurveRecorder') as recorder, \
                     patch('Interaction.firmware_response_model.load_model') as load_model, \
                     patch('Interaction.firmware_parameter_confirmation.confirm_firmware_mode_parameters') as confirm:
                    if version == 99:
                        with self.assertRaisesRegex(RuntimeError, 'unsupported curve event logging protocol'):
                            setup(ctrl)
                        recorder.assert_not_called()
                        ctrl.cf.param.set_value.assert_not_called()
                    else:
                        setup(ctrl)
                        self.assertEqual(recorder.call_args.kwargs['protocol_version'], version)
                        self.assertTrue(recorder.call_args.kwargs['events_enabled'])
                        self.assertEqual(confirm.call_args.kwargs['expected'], {'hlCommander.curveVer': version})
                        self.assertEqual(ctrl.cf.param.set_value.call_args_list[-1].args,
                                         ('hlCommander.curveLog', '1'))
                    load_model.assert_not_called()

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
            'hlCommander.scQual', 'hlCommander.scAge',
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

    def test_estimator_diagnostics_are_recorded_at_their_own_period(self):
        with TemporaryDirectory() as tmp:
            cf = Mock()
            logger = SimpleNamespace(args=SimpleNamespace(), add_cf_packet_listener=Mock(return_value=Mock()))
            rec = CurveRecorder(cf, logger, directory=tmp, tag='trial', events_enabled=False)
            for tick in (50000, 50100):
                rec._state(state('CURVE_ESTIMATOR', tick))
            rec.close()
            rows = [json.loads(line) for line in Path(tmp, 'trial.states.jsonl').read_text().splitlines()]
            self.assertEqual(len([row for row in rows if row['type'] == 'state']), 2)
            self.assertEqual(rec.state_gaps['CURVE_ESTIMATOR'], 0)


def math_degrees(x):
    import math
    return math.degrees(x)
