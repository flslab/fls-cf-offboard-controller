"""Known-error recovery, independent rejection, persistence and capture guards."""

from copy import deepcopy
from contextlib import contextmanager
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from Interaction.estimator_imu_calibration import (
    G, SCHEMA, analyze_file, default_plan, fit_dataset, validate_pose, write_json,
)
from Interaction.calibrate_estimator_imu import (
    IMU_FIELDS, MOTOR_FIELDS, TickUnwrapper, check_motor_sample, collect, collect_window,
    main, radio_packets,
)


def fixture_dataset():
    rng = np.random.default_rng(1008)
    theta = np.deg2rad(3.)
    matrix = np.array([[1.01, 0, 0], [0, np.cos(theta)*.99, -np.sin(theta)*1.02],
                       [0, np.sin(theta)*.99, np.cos(theta)*1.02]])
    ba = np.array([.03, -.04, .02])
    bg = np.array([.001, -.002, .0005])
    document = {'schema': SCHEMA, 'status': 'complete', 'drone_id': 'test',
                'firmware_id': 'synthetic', 'fixture_id': 'known-six-face-fixture',
                'reference_note': 'Synthetic independent reference',
                'reference_source': 'independent_fixture',
                'sensor_frame': 'driver_processed_body', 'motors_off_confirmed': True,
                'poses': []}
    for definition in default_plan():
        reference = np.asarray(definition['reference_force_m_s2'])
        document['poses'].append({**definition, 'time_s': (np.arange(400)*.01).tolist(),
            'accel_m_s2': (matrix @ reference + ba + rng.normal(0, .015, (400, 3))).tolist(),
            'gyro_rad_s': (bg + rng.normal(0, .0003, (400, 3))).tolist()})
    return document, matrix, ba, bg


class FitTests(unittest.TestCase):
    def setUp(self):
        self.data, self.matrix, self.ba, self.bg = fixture_dataset()

    def test_recovers_three_degree_error_and_bias_on_unseen_orientation(self):
        result = fit_dataset(self.data)
        self.assertTrue(result['accepted'], result['failures'])
        np.testing.assert_allclose(result['measured_from_reference'], self.matrix, atol=.0003)
        np.testing.assert_allclose(result['accel_bias_m_s2'], self.ba, atol=.001)
        np.testing.assert_allclose(result['gyro_residual_bias_rad_s'], self.bg, atol=.00003)
        reference = np.array([1., 2., 3.]) * G / np.sqrt(14)
        corrected = np.linalg.solve(result['measured_from_reference'],
                                    self.matrix @ reference + self.ba - result['accel_bias_m_s2'])
        np.testing.assert_allclose(corrected, reference, atol=.003)
        self.assertAlmostEqual(result['alignment_rotation_deg'], 3., delta=.02)
        self.assertFalse(result['firmware_applied'])
        self.assertFalse(result['vicon_delay_calibrated'])

    def test_validation_is_not_used_to_fit_and_can_reject(self):
        baseline = fit_dataset(self.data)
        self.data['poses'][6]['accel_m_s2'] = (
            np.asarray(self.data['poses'][6]['accel_m_s2']) + [0, .4, 0]).tolist()
        result = fit_dataset(self.data)
        self.assertEqual(result['measured_from_reference'], baseline['measured_from_reference'])
        self.assertFalse(result['accepted'])
        self.assertTrue(any('validation' in reason for reason in result['failures']))

    def test_changed_gyro_bias_rejected_in_training_or_holdout(self):
        for index in (0, 6):
            with self.subTest(index=index):
                data = deepcopy(self.data)
                data['poses'][index]['gyro_rad_s'] = (
                    np.asarray(data['poses'][index]['gyro_rad_s']) + [np.deg2rad(.5), 0, 0]).tolist()
                if index == 0:
                    with self.assertRaisesRegex(ValueError, 'gyro zero changes'):
                        fit_dataset(data)
                else:
                    self.assertFalse(fit_dataset(data)['accepted'])

    def test_unknown_hover_and_missing_axis_are_unobservable(self):
        for mode in ('hover', 'missing'):
            data = deepcopy(self.data)
            for pose in data['poses']:
                pose['reference_force_m_s2'] = [0, 0, G] if mode == 'hover' else [G, 0, 0]
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError, 'diverse'):
                fit_dataset(data)

    def test_reflection_or_wrong_units_rejected(self):
        for factor in ([-1, 1, 1], [.1, .1, .1]):
            data = deepcopy(self.data)
            for pose in data['poses']:
                pose['accel_m_s2'] = (np.asarray(pose['accel_m_s2']) * factor).tolist()
            with self.subTest(factor=factor), self.assertRaisesRegex(ValueError, 'scale/reflection'):
                fit_dataset(data)

    def test_no_estimator_truth_or_incomplete_provenance(self):
        for key, value in (('reference_source', 'ordinary_kf'), ('status', 'incomplete'),
                           ('firmware_id', ''), ('motors_off_confirmed', False),
                           ('sensor_frame', 'raw_sensor')):
            data = deepcopy(self.data)
            data[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                fit_dataset(data)

    def test_duplicate_ids_and_missing_validation_rejected(self):
        self.data['poses'][6]['id'] = self.data['poses'][0]['id']
        with self.assertRaisesRegex(ValueError, 'unique'):
            fit_dataset(self.data)
        self.data['poses'] = self.data['poses'][:6]
        with self.assertRaises(ValueError):
            fit_dataset(self.data)

    def test_copying_training_samples_into_validation_is_rejected(self):
        self.data['poses'][6]['accel_m_s2'] = deepcopy(self.data['poses'][0]['accel_m_s2'])
        self.data['poses'][6]['gyro_rad_s'] = deepcopy(self.data['poses'][0]['gyro_rad_s'])
        with self.assertRaisesRegex(ValueError, 'independent collection'):
            fit_dataset(self.data)

    def test_sensor_configuration_changed_during_capture_is_rejected(self):
        self.data['capture'] = {'firmware_parameters_before': {'imu_sensors': {'imuPhi': '0'}},
                                'firmware_parameters_after': {'imu_sensors': {'imuPhi': '3'}}}
        with self.assertRaisesRegex(ValueError, 'parameters changed'):
            fit_dataset(self.data)

    def test_zero_validation_measurement_rejects_without_nan_report(self):
        self.data['poses'][6]['accel_m_s2'] = np.zeros((400, 3)).tolist()
        result = fit_dataset(self.data)
        self.assertFalse(result['accepted'])
        json.dumps(result, allow_nan=False)

    def test_motion_gap_duplicate_nan_and_short_window_rejected(self):
        for fault in ('motion', 'gap', 'duplicate', 'nan', 'short', 'drift'):
            pose = deepcopy(self.data['poses'][0])
            if fault == 'motion':
                pose['gyro_rad_s'][30][0] = 1.
            elif fault == 'gap':
                pose['time_s'][200:] = [t + .1 for t in pose['time_s'][200:]]
            elif fault == 'duplicate':
                pose['time_s'][20] = pose['time_s'][19]
            elif fault == 'nan':
                pose['accel_m_s2'][10][1] = float('nan')
            elif fault == 'short':
                pose['time_s'] = [t/3 for t in pose['time_s']]
            else:
                pose['accel_m_s2'][200:] = (np.asarray(pose['accel_m_s2'][200:]) + [.2, 0, 0]).tolist()
            with self.subTest(fault=fault), self.assertRaises(ValueError):
                validate_pose(pose)

    def test_file_cli_acceptance_rejection_hash_and_no_overwrite(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            raw = root / 'dataset.json'
            write_json(raw, self.data)
            output = root / 'good'
            self.assertEqual(main(['fit', str(raw), '--output', str(output)]), 0)
            accepted = json.loads((output / 'calibration.json').read_text())
            self.assertEqual(len(accepted['dataset_sha256']), 64)
            original = (output / 'calibration.json').read_bytes()
            self.assertEqual(main(['fit', str(raw), '--output', str(output)]), 1)
            self.assertEqual(original, (output / 'calibration.json').read_bytes())
            self.data['status'] = 'incomplete'
            write_json(raw, self.data)
            result = analyze_file(raw, root / 'bad')
            self.assertFalse(result['accepted'])
            self.assertFalse((root / 'bad' / 'calibration.json').exists())
            self.assertTrue((root / 'bad' / 'report.json').exists())


class FakeStream:
    def __init__(self, *, motor=0, gap=False, no_motor=False):
        self.now = 100.
        self.count = 0
        self.motor = motor
        self.gap = gap
        self.no_motor = no_motor

    def clock(self):
        return self.now

    def __call__(self, timeout):
        self.count += 1
        self.now += .01
        tick = round((self.now - 100)*1000)
        if self.gap and self.now > 102:
            tick += 100
        if self.count % 5 == 1 and not self.no_motor:
            return 'motors', tick, dict.fromkeys(MOTOR_FIELDS, self.motor), round(self.now*1e9)
        data = dict.fromkeys(IMU_FIELDS, 0.)
        data['acc.z'] = 1.
        return 'imu', tick, data, round(self.now*1e9)


class CaptureTests(unittest.TestCase):
    def test_tick_rollover_and_reset_duplicates(self):
        tick = TickUnwrapper()
        self.assertEqual(tick.update((1 << 24)-5), 0)
        self.assertEqual(tick.update(5), .01)
        for value in (5, 4, -1, float('nan')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                tick.update(value)

    def test_zero_motor_required(self):
        check_motor_sample(dict.fromkeys(MOTOR_FIELDS, 0))
        for value in (1, float('nan'), -1):
            with self.subTest(value=value), self.assertRaises(RuntimeError):
                check_motor_sample(dict.fromkeys(MOTOR_FIELDS, value))

    def test_captured_si_units_and_device_clock(self):
        stream = FakeStream()
        pose = collect_window(stream, default_plan()[4], 4, 1, clock=stream.clock)
        validate_pose(pose)
        self.assertEqual(pose['accel_m_s2'][0], [0, 0, G])
        self.assertEqual(pose['gyro_rad_s'][0], [0, 0, 0])
        self.assertGreater(len(pose['motor_checks']), 60)

    def test_nonzero_missing_motor_and_device_gap_rejected(self):
        for kwargs in ({'motor': 1}, {'no_motor': True}, {'gap': True}):
            stream = FakeStream(**kwargs)
            with self.subTest(kwargs=kwargs), self.assertRaises((RuntimeError, ValueError)):
                pose = collect_window(stream, default_plan()[4], 4, 1, clock=stream.clock)
                validate_pose(pose)

    def test_radio_lifecycle_reads_logs_without_commander_or_parameter_writes(self):
        from cflib.crazyflie.log import LogConfig
        fields = {field.split('.')[0]: {} for field in IMU_FIELDS + MOTOR_FIELDS}
        for field in IMU_FIELDS + MOTOR_FIELDS:
            group, name = field.split('.')
            fields[group][name] = object()
        cf = SimpleNamespace(param=SimpleNamespace(is_updated=True, values={'imu_sensors': {'imuPhi': '0'}}),
                             log=SimpleNamespace(toc=SimpleNamespace(toc=fields), add_config=Mock()),
                             disconnected=Mock())
        connection = Mock()
        connection.__enter__ = Mock(return_value=SimpleNamespace(cf=cf))
        connection.__exit__ = Mock(return_value=False)
        with patch('cflib.crtp.init_drivers'), patch('cflib.crazyflie.Crazyflie', return_value=cf), \
                patch('cflib.crazyflie.syncCrazyflie.SyncCrazyflie', return_value=connection), \
                patch.object(LogConfig, 'start'), patch.object(LogConfig, 'stop'), patch.object(LogConfig, 'delete'):
            with radio_packets('usb://0') as (next_packet, metadata):
                self.assertEqual(metadata['firmware_parameters_before'], cf.param.values)
                blocks = [call.args[0] for call in cf.log.add_config.call_args_list]
                # FLS consumes the wire byte in ms; cflib divides its argument
                # by ten. Check actual wire periods, not just display labels.
                self.assertEqual([block.period for block in blocks], [10, 50])
                blocks[0].data_received_cb.call(10, dict.fromkeys(IMU_FIELDS, 0), blocks[0])
                self.assertEqual(next_packet(timeout=.1)[0], 'imu')
                next_packet.reset()
                with self.assertRaises(TimeoutError):
                    next_packet(timeout=.001)
            self.assertEqual(cf.log.add_config.call_count, 2)

    def test_collection_finishes_or_preserves_incomplete_attempt_without_activation(self):
        data, _, _, _ = fixture_dataset()

        @contextmanager
        def source(uri):
            yield Mock(), {'firmware_parameters_before': {'imu_sensors': {}}}

        for failure in (False, True):
            with self.subTest(failure=failure), TemporaryDirectory() as tmp:
                output = Path(tmp) / 'session'
                args = SimpleNamespace(output=output, uri='usb://0', drone_id='test',
                    calibration_method='six_face',
                    firmware_id='synthetic', fixture_id='test-fixture', reference_note='synthetic',
                    duration_s=4, settle_s=2)
                windows = list(data['poses'])
                pose_prompt = Mock(return_value='')
                if failure:
                    windows[2] = RuntimeError('link failed')
                with patch('Interaction.calibrate_estimator_imu.collect_window', side_effect=windows), \
                        patch('Interaction.calibrate_estimator_imu.time.monotonic', side_effect=[0, 11]):
                    if failure:
                        with self.assertRaisesRegex(RuntimeError, 'link failed'):
                            collect(args, prompt=pose_prompt, source=source)
                    else:
                        self.assertEqual(collect(args, prompt=pose_prompt, source=source), 0)
                self.assertFalse(any('PROPS OFF' in call.args[0] for call in pose_prompt.call_args_list))
                captured = json.loads((output / 'dataset.json').read_text())
                self.assertEqual(captured['status'], 'incomplete' if failure else 'complete')
                self.assertEqual(captured['motors_off_confirmation_source'], 'fixture_collection_assumption')
                self.assertEqual(len(captured['poses']), 2 if failure else 12)
                self.assertEqual((output / 'fit' / 'calibration.json').exists(), not failure)


if __name__ == '__main__':
    unittest.main()
