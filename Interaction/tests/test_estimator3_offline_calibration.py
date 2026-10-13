"""Known gyro-matrix recovery and replay through the actual frozen C estimator."""
from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from contextlib import contextmanager

import numpy as np

from Interaction.estimator_gyro_calibration import fit_gyro, rotation_integral, rotation_plan
from Interaction.estimator_imu_calibration import G, analyze_file, fit_dataset, write_json
from Interaction.calibrate_estimator_imu import collect
from Interaction.replay_estimator3 import candidate_for_capture, run_replay


class PreflightGyroZeroTests(unittest.TestCase):
    def test_only_boot_gyro_bias_changes_and_source_fit_stays_unchanged(self):
        candidate={'gyro_residual_bias_rad_s':[0.,0.,0.],
                   'measured_from_reference':np.eye(3).tolist(),'accel_bias_m_s2':[.01,0.,0.]}
        refresh=dict(source_candidate_sha256='fit-hash',stationary_validated=True,samples=300,
            duration_s=3.,accelerometer_fit_changed=False,firmware_calibration_applied=False,
            gyro_residual_bias_rad_s=[.001,-.002,.003])
        rows=[{'group':'metadata','data':{'preflight_gyro_zero':refresh}}]
        effective,used=candidate_for_capture(rows,candidate,'fit-hash')
        self.assertEqual(effective['gyro_residual_bias_rad_s'],refresh['gyro_residual_bias_rad_s'])
        self.assertEqual(candidate['gyro_residual_bias_rad_s'],[0.,0.,0.])
        self.assertEqual(effective['measured_from_reference'],candidate['measured_from_reference'])
        for field,value in (('source_candidate_sha256','wrong'),('stationary_validated',False),
                            ('samples',10),('gyro_residual_bias_rad_s',[float('nan'),0,0])):
            with self.subTest(field=field):
                bad=deepcopy(rows);bad[0]['data']['preflight_gyro_zero'][field]=value
                with self.assertRaisesRegex(ValueError,'preflight gyro-zero'):
                    candidate_for_capture(bad,candidate,'fit-hash')
from Interaction.tests.test_estimator_imu_calibration import fixture_dataset


def gyro_windows():
    rng = np.random.default_rng(1009)
    theta = np.deg2rad(2.)
    matrix = np.array([[np.cos(theta)*1.02, -np.sin(theta)*.99, 0],
                       [np.sin(theta)*1.02, np.cos(theta)*.99, 0], [0, 0, 1.01]])
    bias = np.array([.001, -.002, .0005])
    t = np.arange(1000)*.01
    profile = ((t >= 2) & (t < 8)).astype(float)
    profile /= np.sum((profile[1:]+profile[:-1])*.5*np.diff(t))
    windows = []
    for definition in rotation_plan():
        actual = profile[:, None]*definition['reference_rotation_rad']
        measured = actual@matrix.T+bias+rng.normal(0, 1e-5, actual.shape)
        windows.append({**definition, 'time_s': t.tolist(), 'gyro_rad_s': measured.tolist()})
    return windows, matrix, bias


class GyroTests(unittest.TestCase):
    def test_recovers_scale_axis_and_bias_on_unseen_rate(self):
        windows, matrix, bias = gyro_windows()
        result = fit_gyro(windows, bias)
        self.assertTrue(result['accepted'])
        np.testing.assert_allclose(result['measured_from_reference'], matrix, atol=5e-5)
        true = np.array([.1, -.2, .3])
        corrected = np.linalg.solve(result['measured_from_reference'], matrix@true+bias-result['bias_rad_s'])
        np.testing.assert_allclose(corrected, true, atol=3e-5)

    def test_independent_turns_not_used_to_fit(self):
        windows, _, bias = gyro_windows()
        original = fit_gyro(windows, bias)
        w = np.asarray(windows[6]['gyro_rad_s'])
        w[250:750, 0] += np.deg2rad(1)
        windows[6]['gyro_rad_s'] = w.tolist()
        result = fit_gyro(windows, bias)
        self.assertEqual(result['measured_from_reference'], original['measured_from_reference'])
        self.assertFalse(result['accepted'])

    def test_missing_axis_freehand_gap_and_moving_endpoint_rejected(self):
        windows, _, bias = gyro_windows()
        for fault in ('freehand', 'gap', 'endpoint'):
            window = deepcopy(windows[0])
            if fault == 'freehand':
                window['reference_source'] = 'default_estimator'
            elif fault == 'gap':
                window['time_s'][500:] = [v+.1 for v in window['time_s'][500:]]
            else:
                window['gyro_rad_s'][0:150] = [[.1, 0, 0]]*150
            with self.subTest(fault=fault), self.assertRaises(ValueError):
                rotation_integral(window, bias)
        with self.assertRaises(ValueError):
            fit_gyro(windows[:4]+windows[6:], bias)

    def test_combined_fit_saves_gyro_matrix_and_rejects_failed_gyro(self):
        data, _, _, _ = fixture_dataset()
        data['rotations'], _, _ = gyro_windows()
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_json(root/'dataset.json', data)
            result = analyze_file(root/'dataset.json', root/'good')
            self.assertTrue(result['accepted'])
            self.assertTrue(result['gyro_axis_scale_calibrated'])
            data['rotations'] = data['rotations'][:6]
            write_json(root/'bad.json', data)
            result = analyze_file(root/'bad.json', root/'bad')
            self.assertFalse(result['fit_passed'])
            self.assertFalse((root/'bad/calibration.json').exists())

    def test_interactive_workflow_collects_static_faces_before_gyro(self):
        data, _, _, _ = fixture_dataset()
        windows, _, _ = gyro_windows()
        @contextmanager
        def source(uri):
            yield Mock(), {}
        def prompt(message):
            self.assertNotIn('Type ', message)
            return ''
        with TemporaryDirectory() as tmp:
            args = SimpleNamespace(output=Path(tmp)/'session', uri='usb://0', drone_id='test',
                calibration_method='six_face',
                firmware_id='synthetic', fixture_id='fixture', reference_note='reference',
                duration_s=4, settle_s=2, auto_flight=True, flight_config='unused', gyro_calibration=True)
            runner = Mock(return_value={'capture_completed': True})
            with patch('Interaction.calibrate_estimator_imu.collect_window', side_effect=data['poses'][:6]+windows) as capture, \
                    patch('Interaction.calibrate_estimator_imu.time.monotonic', side_effect=[0, 11]):
                code = collect(args, prompt=prompt, source=source, flight_runner=runner)
            self.assertEqual(code, 0)
            self.assertEqual(capture.call_count, 18)
            self.assertEqual(capture.call_args_list[0].args[1]['id'], 'train_x+')
            self.assertEqual(capture.call_args_list[6].args[1]['id'], 'gyro_train_x+')
            fit = json.loads((args.output/'fit/candidate.json').read_text())
            self.assertTrue(fit['gyro_axis_scale_calibrated'])
            runner.assert_called_once()


def flight_rows(matrix, bias, gyro_bias, n=800):
    rows = [{'group': 'event', 'received_s': 99.99, 'data': {'name': 'capture_ready'}, 'phase': 'preflight'}]
    acceleration = (matrix@np.array([0, 0, G])+bias)/G
    for i in range(n):
        common = {'received_s': 100+i*.01, 'phase': 'hover_start', 'cf_log_tick_ms_mod24': 50000+i*10}
        rows += [{**common, 'group': 'kf', 'data': {f'kalman.q{j}': int(j == 0) for j in range(4)}},
                 {**common, 'group': 'state', 'data': {**{f'stateEstimate.{a}': float(a == 'z') for a in 'xyz'},
                     **{f'stateEstimate.v{a}': 0. for a in 'xyz'}}},
                 {**common, 'group': 'vicon', 'data': {'position_m': [0, 0, 1]}},
                 {**common, 'group': 'imu', 'data': {**{f'acc.{a}': acceleration[j] for j, a in enumerate('xyz')},
                     **{f'gyro.{a}': np.rad2deg(gyro_bias[j]) for j, a in enumerate('xyz')}}}]
    return rows


class NativeReplayTests(unittest.TestCase):
    def test_identical_duplicate_is_reported_but_conflicting_device_tick_rejected(self):
        data,matrix,bias,bg=fixture_dataset()
        with TemporaryDirectory() as tmp:
            root=Path(tmp);write_json(root/'dataset.json',data)
            analyze_file(root/'dataset.json',root/'fit')
            rows=flight_rows(matrix,bias,bg,40)
            index=next(i for i,r in enumerate(rows) if r['group']=='imu' and r['received_s']>100.1)
            rows.insert(index+1,deepcopy(rows[index]))
            path=root/'packets.jsonl'
            path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
            report=run_replay(path,root/'fit/calibration.json',root/'duplicate')
            self.assertEqual(len(report['corrected']['samples']),40)
            self.assertEqual(report['corrected']['segments'],1)
            self.assertEqual(sum(r['reason']=='identical duplicate IMU packet skipped'
                for r in report['corrected']['rejected']),1)
            rows[index+1]['data']['gyro.x']+=1
            path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
            with self.assertRaisesRegex(ValueError,'duplicate/reordered device tick'):
                run_replay(path,root/'fit/calibration.json',root/'conflicting')

    def test_actual_c_replay_reduces_known_sensor_error_and_reports_reference_limits(self):
        data, matrix, bias, bg = fixture_dataset()
        data['rotations'], _, _ = gyro_windows()
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_json(root/'dataset.json', data)
            analyze_file(root/'dataset.json', root/'fit')
            rows = flight_rows(matrix, bias, bg)
            (root/'packets.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
            result = run_replay(root/'packets.jsonl', root/'fit/calibration.json', root/'replay')
            raw = result['raw']['by_phase']['hover_start']['rmse_rpy_deg'][0]
            corrected = result['corrected']['by_phase']['hover_start']['rmse_rpy_deg'][0]
            self.assertLess(corrected, raw*.2)
            self.assertGreater(result['raw']['samples'][-1]['position_fusions'], 20)
            self.assertTrue(result['gyro_matrix_applied'])
            self.assertFalse(result['independent_ground_truth'])
            self.assertFalse(result['onboard_estimator3_enabled'])
            self.assertEqual(result['corrected']['segments'], 1)
            self.assertTrue((root/'replay/build.json').exists())
            with self.assertRaises(FileExistsError):
                run_replay(root/'packets.jsonl', root/'fit/calibration.json', root/'replay')

    def test_gap_starts_reported_segment_instead_of_fabricated_imu(self):
        data, matrix, bias, bg = fixture_dataset()
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_json(root/'dataset.json', data)
            analyze_file(root/'dataset.json', root/'fit')
            rows = flight_rows(matrix, bias, bg, 120)
            for row in rows:
                if row['received_s'] >= 100.5:
                    row['received_s'] += .1
                    row['cf_log_tick_ms_mod24'] += 100
            (root/'packets.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
            result = run_replay(root/'packets.jsonl', root/'fit/calibration.json', root/'replay')
            self.assertEqual(result['raw']['segments'], 2)
            self.assertEqual(result['corrected']['segments'], 2)

    def test_reference_packet_from_future_device_tick_is_not_used(self):
        data, matrix, bias, bg = fixture_dataset()
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_json(root/'dataset.json', data)
            analyze_file(root/'dataset.json', root/'fit')
            rows = flight_rows(matrix, bias, bg, 10)
            future = deepcopy(rows[-4])
            future['cf_log_tick_ms_mod24'] += 20
            q = [np.cos(np.deg2rad(15)), np.sin(np.deg2rad(15)), 0, 0]
            future['data'] = {f'kalman.q{i}': q[i] for i in range(4)}
            rows.insert(len(rows)-1, future)
            (root/'packets.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
            result = run_replay(root/'packets.jsonl', root/'fit/calibration.json', root/'replay')
            self.assertAlmostEqual(result['raw']['samples'][-1]['reference_rpy_deg'][0], 0)


if __name__ == '__main__':
    unittest.main()
