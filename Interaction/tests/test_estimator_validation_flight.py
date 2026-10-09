"""No-hardware checks of fixture-to-flight orchestration and bounded commands."""
from contextlib import contextmanager
from copy import deepcopy
import hashlib
import io
import json
from pathlib import Path
import signal
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import yaml

from Interaction.calibrate_estimator_imu import collect, main
from Interaction.estimator_imu_calibration import analyze_file, fit_dataset, write_json
from Interaction.estimator_validation_flight import (
    FlightConnection, LOGS, PARAMETERS, check_snapshot, euler, load_flight_config,
    multiply, open_flight, run_protocol, run_validation_flight, summarize_capture, trajectory,
    verify_capture_continuity,
    flight_signals, builtin_flight_config, configure_builtin_capture, anchor_builtin_flight,
)
from Interaction.tests.test_estimator_imu_calibration import fixture_dataset


def config():
    return {'mocap_host': 'test', 'rigidbody': 'test', 'body_to_rigidbody_wxyz': [1, 0, 0, 0],
            'orientation_reference_note': 'Known body-aligned rigid fixture',
            'center_m': [0, 0, 1], 'boundary_m': [[-1.5, 1.5], [-1.5, 1.5], [.3, 1.5]],
            'minimum_voltage': 7.}


class FakeFlight:
    def __init__(self, *, fault=None):
        self.now = 100.
        self.phase = 'preflight'
        self.position = np.array([0, 0, .24])
        self.commands = []
        self.armed = False
        self.stopped = False
        self.fault = fault

    def sleep(self, duration):
        self.now += duration

    def clock(self):
        return self.now

    def snapshot(self):
        if self.fault == 'interrupt' and self.phase == '+X_out':
            raise KeyboardInterrupt('operator interrupt')
        groups = {name: {'received_s': self.now, 'data': {}} for name in LOGS}
        groups['vicon'] = {'received_s': self.now, 'data': {
            'position_m': self.position.tolist(), 'body_quaternion_wxyz': [1, 0, 0, 0]}}
        groups['state']['data'] = {f'stateEstimate.{a}': self.position[i] for i, a in enumerate('xyz')}
        groups['kf']['data'] = {f'kalman.q{i}': int(i == 0) for i in range(4)}
        groups['health']['data'] = {'pm.vbat': 8, **{f'motor.m{i}': 0 for i in range(1, 5)}}
        groups['now'] = self.now
        groups['eskf_progress_s'] = self.now
        if self.fault == 'preflight' or (self.fault == 'airborne' and self.phase == '+X_out'):
            groups['vicon']['received_s'] -= .3
        return groups

    def command(self, position, yaw):
        self.position = np.asarray(position).copy()
        self.commands.append((self.phase, self.position.copy(), yaw))

    def arm(self):
        self.armed = True

    def stop(self):
        self.stopped = True


class ProtocolTests(unittest.TestCase):
    def test_config_checks_boundary_reference_and_units(self):
        with TemporaryDirectory() as tmp:
            path = Path(tmp)/'flight.yaml'
            path.write_text(yaml.safe_dump(config()))
            self.assertEqual(load_flight_config(path), config())
            for key, value in (('center_m', [1.4, 0, 1]), ('tracking_mode', 'invalid'),
                               ('minimum_voltage', float('nan')), ('attitude_reference', 'vicon')):
                doc = {**config(), key: value}
                path.write_text(yaml.safe_dump(doc))
                with self.subTest(key=key), self.assertRaises(ValueError):
                    load_flight_config(path)

    def test_ram_flight_starts_asynchronously_in_selected_phase(self):
        link=FakeFlight()
        link.ram_started=False
        link.ram_recorder=Mock()
        link.record=Mock()
        doc={**config(),'ram_trace':{'phase':'+Y_out','offset_s':1.,'hz':1000,'duration_ms':600}}
        run_protocol(link,doc,clock=link.clock,sleep=link.sleep)
        link.ram_recorder.start_async.assert_called_once_with(1000,600)
        link.ram_recorder.download.assert_not_called()
        self.assertTrue(link.stopped)
        event=next(call for call in link.record.call_args_list if call.args[1].get('name')=='ram_record_requested')
        self.assertEqual(event.args[1]['phase'],'+Y_out')

    def test_builtin_profile_anchors_to_actual_vicon_and_preserves_tracking_seed(self):
        doc=builtin_flight_config([0,-1,.24])
        np.testing.assert_allclose(doc['center_m'],[0,-1,.84])
        anchor_builtin_flight(doc,[.02,-.99,.25])
        np.testing.assert_allclose(doc['center_m'],[.02,-.99,.85])
        self.assertEqual(doc['initial_position_m'],[0,-1,.24])
        self.assertEqual(doc['measured_ground_m'],[.02,-.99,.25])
        phases=trajectory(doc['center_m'],[.02,-.99,.27])
        self.assertEqual(len(phases),20)
        self.assertAlmostEqual(sum(p[3] for p in phases),66.)
        self.assertNotIn('detection_method',doc)

    def test_builtin_RAM_selection_is_automatic_and_fallback_explicit(self):
        doc=builtin_flight_config([0,0,.24])
        self.assertFalse(configure_builtin_capture(doc,{}))
        self.assertNotIn('ram_trace',doc)
        self.assertTrue(configure_builtin_capture(doc,{'ramTrace':dict.fromkeys(('mode','imuHz','durationMs','status','session'))}))
        self.assertEqual(doc['ram_trace'],{'phase':'+Y_out','offset_s':1.,'hz':1000,'duration_ms':600})

    def test_metadata_defaults_are_optional_and_preserved_as_unknown(self):
        with patch('Interaction.calibrate_estimator_imu.collect',return_value=0) as capture:
            self.assertEqual(main(['collect','--drone-id','test','--output','/unused']),0)
        args=capture.call_args.args[0]
        self.assertEqual((args.firmware_id,args.fixture_id,args.reference_note),('unknown',)*3)
        data,*_=fixture_dataset()
        for key in ('firmware_id','fixture_id','reference_note'):
            data[key]='unknown'
        fit=fit_dataset(data)
        self.assertTrue(fit['fit_passed'])
        self.assertFalse(fit['provenance_documented'])

    def test_collector_cli_accepts_builtin_seed_without_any_yaml(self):
        base=['collect','--drone-id','test','--firmware-id','build','--fixture-id','fixture',
              '--reference-note','reference','--output','/unused','--auto-flight',
              '--flight-initial-position','0','-1','.24']
        with patch('Interaction.calibrate_estimator_imu.collect',return_value=0) as capture:
            self.assertEqual(main(base),0)
        args=capture.call_args.args[0]
        self.assertIsNone(args.flight_config)
        self.assertEqual(args.flight_initial_position,[0,-1,.24])

    def test_trajectory_is_symmetric_bounded_and_returns_home(self):
        phases = trajectory([0, 0, 1], [0, 0, .26])
        endpoints = {name: end for name, start, end, duration in phases}
        np.testing.assert_allclose(endpoints['+X_out'], [.2, 0, 1])
        np.testing.assert_allclose(endpoints['-X_out'], [-.2, 0, 1])
        np.testing.assert_allclose(endpoints['+Y_out'], [0, .2, 1])
        np.testing.assert_allclose(endpoints['-Y_out'], [0, -.2, 1])
        self.assertEqual(sum(p[3] for p in phases), 66.)
        np.testing.assert_allclose(phases[-1][2], [0, 0, .26])

    def test_full_protocol_uses_estimator2_and_lands(self):
        flight = FakeFlight()
        result = run_protocol(flight, config(), clock=flight.clock, sleep=flight.sleep)
        self.assertTrue(result['capture_completed'])
        self.assertFalse(result['candidate_applied'])
        self.assertEqual(result['control_estimator'], 2)
        self.assertTrue(flight.armed and flight.stopped)
        self.assertGreater(len(flight.commands), 3000)
        np.testing.assert_allclose(flight.position, [0, 0, .26])

    def test_stale_preflight_never_arms_and_airborne_failure_descends(self):
        for fault in ('preflight', 'airborne'):
            flight = FakeFlight(fault=fault)
            with self.subTest(fault=fault), self.assertRaises(RuntimeError):
                run_protocol(flight, config(), clock=flight.clock, sleep=flight.sleep)
            self.assertEqual(flight.armed, fault == 'airborne')
            if fault == 'airborne':
                self.assertTrue(flight.stopped)
                self.assertEqual(flight.commands[-1][0], 'abort_land')
                self.assertLess(flight.position[2], .27)

    def test_interrupt_requests_descent_and_disarm_and_restores_signal_handlers(self):
        flight = FakeFlight(fault='interrupt')
        with self.assertRaises(KeyboardInterrupt):
            run_protocol(flight, config(), clock=flight.clock, sleep=flight.sleep)
        self.assertTrue(flight.stopped)
        self.assertEqual(flight.commands[-1][0], 'abort_land')
        previous = signal.getsignal(signal.SIGTERM)
        with self.assertRaises(KeyboardInterrupt):
            with flight_signals():
                signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
        self.assertEqual(signal.getsignal(signal.SIGTERM), previous)

    def test_snapshot_rejects_bad_battery_motor_pose_and_frozen_producer(self):
        for fault in ('battery', 'motor', 'position', 'stale'):
            snap = FakeFlight().snapshot()
            if fault == 'battery':
                snap['health']['data']['pm.vbat'] = 6
            elif fault == 'motor':
                snap['health']['data']['motor.m1'] = 100
            elif fault == 'position':
                snap['state']['data']['stateEstimate.x'] = 1
            else:
                snap['imu']['received_s'] -= .3
            with self.subTest(fault=fault), self.assertRaises(RuntimeError):
                check_snapshot(snap, config(), flying=False, ground_z=.24)

    def test_body_rigidbody_quaternion_composition(self):
        q = [np.cos(np.pi/12), np.sin(np.pi/12), 0, 0]
        np.testing.assert_allclose(euler(multiply([1, 0, 0, 0], q)), [30, 0, 0], atol=1e-12)


class WorkflowTests(unittest.TestCase):
    def test_same_boot_and_sensor_configuration_are_required(self):
        data = {'capture': {'firmware_parameters_after': {'imu_sensors': {'imuPhi': '0'}}},
                'poses': [{'host_receipt_monotonic_ns': [100_000_000_000],
                           'cf_log_tick_ms_mod24': [50000]}]}
        snap = {'metadata': {'data': {'firmware_parameters_before': {'imu_sensors': {'imuPhi': '0'}}}},
                'imu': {'received_s': 130., 'cf_log_tick_ms_mod24': 80000}}
        verify_capture_continuity(data, snap)
        for fault in ('reset', 'changed', 'missing'):
            bad = deepcopy(snap)
            if fault == 'reset':
                bad['imu']['cf_log_tick_ms_mod24'] = 1000
            elif fault == 'changed':
                bad['metadata']['data']['firmware_parameters_before']['imu_sensors']['imuPhi'] = '3'
            else:
                del bad['metadata']
            with self.subTest(fault=fault), self.assertRaises(RuntimeError):
                verify_capture_continuity(data, bad)

    def test_six_only_fit_is_candidate_never_accepted_calibration(self):
        data, _, _, _ = fixture_dataset()
        data['poses'] = data['poses'][:6]
        result = fit_dataset(data, require_validation=False)
        self.assertTrue(result['fit_passed'])
        self.assertFalse(result['accepted'])
        with self.assertRaises(ValueError):
            fit_dataset(data)
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_json(root/'data.json', data)
            analyze_file(root/'data.json', root/'fit', require_validation=False)
            self.assertTrue((root/'fit/candidate.json').exists())
            self.assertFalse((root/'fit/calibration.json').exists())

    def test_six_poses_then_flight_only_after_static_link_closes(self):
        data, _, _, _ = fixture_dataset()
        opened = []

        @contextmanager
        def source(uri):
            opened.append(True)
            try:
                yield Mock(), {}
            finally:
                opened.pop()

        def runner(*args, **kwargs):
            self.assertFalse(opened)
            self.assertTrue(Path(args[1]).exists())
            return {'capture_completed': True}

        with TemporaryDirectory() as tmp:
            args = SimpleNamespace(output=Path(tmp)/'session', uri='usb://0', drone_id='test',
                calibration_method='six_face',
                firmware_id='synthetic', fixture_id='fixture', reference_note='reference',
                duration_s=4, settle_s=2, auto_flight=True, flight_config=Path(tmp)/'flight.yaml')
            with patch('Interaction.calibrate_estimator_imu.collect_window', side_effect=data['poses']) as capture, \
                    patch('Interaction.calibrate_estimator_imu.time.monotonic', side_effect=[0, 11]):
                self.assertEqual(collect(args, prompt=Mock(return_value='PROPS OFF'), source=source,
                                         flight_runner=runner), 0)
            self.assertEqual(capture.call_count, 6)

    def test_failed_fit_does_not_request_flight(self):
        data, _, _, _ = fixture_dataset()
        for pose in data['poses']:
            pose['accel_m_s2'] = (np.asarray(pose['accel_m_s2'])*.1).tolist()
        @contextmanager
        def source(uri):
            yield Mock(), {}
        with TemporaryDirectory() as tmp:
            args = SimpleNamespace(output=Path(tmp)/'session', uri='usb://0', drone_id='test',
                calibration_method='six_face',
                firmware_id='synthetic', fixture_id='fixture', reference_note='reference',
                duration_s=4, settle_s=2, auto_flight=True, flight_config='unused')
            runner = Mock()
            with patch('Interaction.calibrate_estimator_imu.collect_window', side_effect=data['poses']), \
                    patch('Interaction.calibrate_estimator_imu.time.monotonic', side_effect=[0, 11]):
                self.assertEqual(collect(args, prompt=Mock(return_value='PROPS OFF'), source=source,
                                         flight_runner=runner), 1)
            runner.assert_not_called()

    def test_cancelled_flight_keeps_static_candidate_and_never_connects(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            data, _, _, _ = fixture_dataset()
            data['poses'] = data['poses'][:6]
            write_json(root/'dataset.json', data)
            analyze_file(root/'dataset.json', root/'fit', require_validation=False)
            (root/'flight.yaml').write_text(yaml.safe_dump(config()))
            source = Mock()
            result = run_validation_flight('usb://0', root/'fit/candidate.json', root/'dataset.json',
                                          root/'flight.yaml', root/'flight', prompt=Mock(side_effect=KeyboardInterrupt),
                                          connection=source)
            self.assertFalse(result['capture_completed'])
            self.assertFalse(result['flight_started'])
            self.assertTrue((root/'fit/candidate.json').exists())
            source.assert_not_called()

    def test_successful_capture_and_cleanup_failure_have_distinct_results(self):
        for cleanup_failure in (False, True):
            with self.subTest(cleanup_failure=cleanup_failure), TemporaryDirectory() as tmp:
                root = Path(tmp)
                data, _, _, _ = fixture_dataset()
                data['poses'] = data['poses'][:6]
                data['capture'] = {'firmware_parameters_after': {'imu_sensors': {}}}
                data['poses'][-1]['host_receipt_monotonic_ns'] = [100_000_000_000]*400
                data['poses'][-1]['cf_log_tick_ms_mod24'] = [50000]*400
                write_json(root/'dataset.json', data)
                analyze_file(root/'dataset.json', root/'fit', require_validation=False)
                (root/'flight.yaml').write_text(yaml.safe_dump(config()))
                snap = {'metadata': {'data': {'firmware_parameters_before': {'imu_sensors': {}}}},
                        'imu': {'received_s': 130., 'cf_log_tick_ms_mod24': 80000}}
                @contextmanager
                def connection(uri, conf, journal):
                    journal.write(json.dumps({'group': 'event', 'data': {'name': 'arm_requested'}})+'\n')
                    yield SimpleNamespace(snapshot=lambda: snap)
                    if cleanup_failure:
                        raise RuntimeError('readback restore failed')
                with patch('Interaction.estimator_validation_flight.run_protocol', return_value={
                        'capture_completed': True, 'control_estimator': 2, 'candidate_applied': False}):
                    result = run_validation_flight('usb://0', root/'fit/candidate.json', root/'dataset.json',
                        root/'flight.yaml', root/'flight', prompt=lambda _: '', connection=connection)
                self.assertEqual(result['capture_completed'], not cleanup_failure)
                self.assertTrue(result['flight_started'])
                self.assertFalse(result['candidate_applied'])
                self.assertEqual('error' in result, cleanup_failure)

    def test_diagnostics_report_three_degree_mismatch_without_activation_claim(self):
        data, _, _, _ = fixture_dataset()
        candidate = fit_dataset(data)
        rows = []
        q = [np.cos(np.deg2rad(1.5)), np.sin(np.deg2rad(1.5)), 0, 0]
        for i in range(100):
            t = 100+i*.02
            common = {'received_s': t, 'phase': 'hover_start', 'cf_log_tick_ms_mod24': i*20}
            rows.extend([{**common, 'group': 'vicon', 'data': {'position_m': [0, 0, 1], 'body_quaternion_wxyz': [1, 0, 0, 0]}},
                         {**common, 'group': 'imu', 'data': {'acc.x': 0, 'acc.y': 0, 'acc.z': 1, 'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0}},
                         {**common, 'group': 'kf', 'data': {f'kalman.q{j}': int(j == 0) for j in range(4)}},
                         {**common, 'group': 'eskf', 'data': {**{f'kalmanPRel.q{j}': q[j] for j in range(4)},
                             'kalmanPRel.valid': 1, 'kalmanPRel.active': 1}}])
        with TemporaryDirectory() as tmp:
            path = Path(tmp)/'packets.jsonl'
            path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
            result = summarize_capture(path, candidate)
            self.assertEqual(result['attitude_reference'], 'default_estimator')
            self.assertFalse(result['onboard_estimator3_enabled'])
            self.assertFalse(result['independent_ground_truth'])

    def test_cli_requires_auto_flag_for_override_and_seed_for_builtin(self):
        base = ['collect', '--drone-id', 'test', '--firmware-id', 'build', '--fixture-id', 'f',
                '--reference-note', 'note', '--output', '/unused']
        for extra in (['--auto-flight'], ['--flight-config', '/unused']):
            with self.subTest(extra=extra), self.assertRaises(SystemExit):
                main(base+extra)


class AdapterTests(unittest.TestCase):
    def test_real_adapter_writes_then_checks_parameters_restores_without_arming(self):
        from cflib.crazyflie.log import LogConfig
        values = {}
        param_keys = {**PARAMETERS, 'kalman.resetEstimation': 0, 'kalman.initialX': 0,
                      'kalman.initialY': 0, 'kalman.initialZ': 0, 'kalman.initialYaw': 0,
                'kalmanPRel.enable': 1, 'kalmanPRel.scEnable': 0, 'hlCommander.pRelAuto': 0}
        for key, val in param_keys.items():
            group, name = key.split('.')
            values.setdefault(group, {})[name] = str(val)
        toc = {group: dict.fromkeys(rows, object()) for group, rows in values.items()}
        log_toc = {}
        for _, fields in LOGS.values():
            for field in fields:
                group, name = field.split('.')
                log_toc.setdefault(group, {})[name] = object()

        def set_value(key, value):
            group, name = key.split('.')
            values[group][name] = '0' if key == 'kalman.resetEstimation' else value

        cf = SimpleNamespace(param=SimpleNamespace(is_updated=True, values=values,
                toc=SimpleNamespace(toc=toc), set_value=Mock(side_effect=set_value),
                get_value=lambda key: values[key.split('.')[0]][key.split('.')[1]]),
            log=SimpleNamespace(toc=SimpleNamespace(toc=log_toc), add_config=Mock()),
            extpos=SimpleNamespace(send_extpos=Mock()), commander=Mock(), platform=Mock())

        def confirm(param, *, expected):
            for key, value in expected.items():
                self.assertAlmostEqual(float(param.get_value(key)), value, places=4)

        def start(block):
            fields = {v.name: (1. if v.name.endswith('.q0') else 0.) for v in block.variables}
            if block.name == 'health':
                fields['pm.vbat'] = 8
            block.data_received_cb.call(1000, fields, block)

        class FakeMocap:
            def __init__(self, **kwargs):
                self.running = True
            def subscribe_object(self, name, callback):
                raise AssertionError('pointcloud capture must not require a rigid body')
            def subscribe_point(self, initial, callback, name):
                self.callback = callback
            def start(self):
                self.callback({'tvec': [0, 0, .24], 'frame_id': 1, 'time': 100})
            def join(self, timeout):
                pass

        sync = Mock()
        sync.__enter__ = Mock(return_value=SimpleNamespace(cf=cf))
        sync.__exit__ = Mock(return_value=False)
        original = deepcopy(values)
        original['kalmanPRel']['enable'] = '0'
        with patch('cflib.crtp.init_drivers'), patch('cflib.crazyflie.Crazyflie', return_value=cf), \
                patch('cflib.crazyflie.syncCrazyflie.SyncCrazyflie', return_value=sync), \
                patch.object(LogConfig, 'start', start), patch.object(LogConfig, 'stop'), patch.object(LogConfig, 'delete'), \
                patch.dict(sys.modules, {'mocap': SimpleNamespace(Mocap=FakeMocap)}), \
                patch('Interaction.firmware_parameter_confirmation.confirm_firmware_mode_parameters', side_effect=confirm), \
                patch('Interaction.post_release_firmware_control_event.send_vicon_position_mirror') as mirror, \
                patch('Interaction.estimator_validation_flight.time.sleep'):
            with self.assertRaisesRegex(RuntimeError, 'test abort'):
                point_config = {**config(), 'tracking_mode': 'pointcloud', 'initial_position_m': [0, 0, .24]}
                with open_flight('usb://0', point_config, io.StringIO()) as link:
                    self.assertEqual(cf.param.get_value('kalmanPRel.enable'), '0')
                    self.assertEqual(cf.param.get_value('stabilizer.estimator'), '2')
                    mirror.assert_not_called()
                    raise RuntimeError('test abort')
        self.assertFalse(any(c.args == ('kalmanPRel.enable', '1') for c in cf.param.set_value.call_args_list))
        self.assertFalse(any(c.args == ('stabilizer.estimator', '3') for c in cf.param.set_value.call_args_list))
        cf.platform.send_arming_request.assert_not_called()
        cf.commander.send_position_setpoint.assert_not_called()
        for group, entries in original.items():
            for key, value in entries.items():
                self.assertAlmostEqual(float(values[group][key]), float(value))

    def test_stop_always_attempts_disarm(self):
        cf = SimpleNamespace(commander=Mock(), platform=Mock())
        cf.commander.send_stop_setpoint.side_effect = OSError('link failure')
        link = FlightConnection(cf, config(), io.StringIO())
        with self.assertRaises(OSError):
            link.stop()
        cf.platform.send_arming_request.assert_called_once_with(False)

    def test_stop_requires_fresh_zero_motor_readback(self):
        cf = SimpleNamespace(commander=Mock(), platform=Mock())
        for fresh in (False, True):
            link = FlightConnection(cf, config(), io.StringIO())
            now = [100.]
            def sleep(duration):
                now[0] += duration
                if fresh:
                    link.record('health', {f'motor.m{i}': 0 for i in range(1, 5)})
            with patch('Interaction.estimator_validation_flight.time.monotonic', side_effect=lambda: now[0]), \
                    patch('Interaction.estimator_validation_flight.time.sleep', side_effect=sleep):
                if fresh:
                    link.stop()
                else:
                    with self.assertRaisesRegex(RuntimeError, 'zero-motor'):
                        link.stop()


class StandaloneFlightTests(unittest.TestCase):
    def test_standalone_entry_reuses_saved_candidate_without_fixture_collection(self):
        from Interaction.estimator_validation_flight import main as flight_main
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'fit').mkdir()
            (root/'dataset.json').write_text('{}')
            (root/'fit/candidate.json').write_text(json.dumps({'drone_id':'lb11', 'fit_passed':True,
                'dataset_sha256':hashlib.sha256(b'{}').hexdigest()}))
            def execute(command, **kwargs):
                (root/'retry').mkdir()
                (root/'retry/report.json').write_text(json.dumps({'capture_completed':True}))
                return SimpleNamespace(returncode=0)
            with patch('subprocess.run', side_effect=execute) as runner:
                self.assertEqual(flight_main(['--drone-id','lb11','--calibration',str(root),
                    '--output',str(root/'retry'),'--initial-position','0','-1','.24'],prompt=lambda _:''),0)
            command = runner.call_args.args[0]
            self.assertIn('controller.py', command)
            self.assertIn('--calibrate', command)
            self.assertIn('--vicon', command)
            self.assertIn('--imu-validation-session', command)
            self.assertNotIn('--skip-takeoff', command)
            self.assertNotIn('--skip-landing', command)
            with patch('subprocess.run') as runner:
                self.assertEqual(flight_main(['--drone-id','other','--calibration',str(root),
                    '--output',str(root/'retry')]),1)
                runner.assert_not_called()


if __name__ == '__main__':
    unittest.main()
