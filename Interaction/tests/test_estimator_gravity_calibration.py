"""Cage pose ambiguity, independent norm validation and real collector/replay."""
from contextlib import contextmanager, redirect_stdout
from copy import deepcopy
import io
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from Interaction.estimator_gravity_calibration import gravity_plan
from Interaction.estimator_imu_calibration import G, SCHEMA, analyze_file, fit_dataset, write_json
from Interaction.simulate_estimator_imu import gravity_sim_directions, rotation_xyz, run_case
from Interaction.calibrate_estimator_imu import collect, main


def gravity_dataset(*, rotation=None):
    rng = np.random.default_rng(1009)
    scale, bias, bg = np.array([1.018,.988,1.01]), np.array([.08,-.1,-.085]), np.array([.001,-.001,.0004])
    rotation = np.eye(3) if rotation is None else rotation
    doc = {'schema':SCHEMA,'status':'complete','drone_id':'lb11',
           'firmware_id':'synthetic','fixture_id':'rounded-cage',
           'reference_note':'unknown exact orientation','reference_source':'gravity_magnitude',
           'sensor_frame':'driver_processed_body','motors_off_confirmed':True,'poses':[]}
    for definition, direction in zip(gravity_plan(),gravity_sim_directions()):
        doc['poses'].append({**definition,'time_s':(np.arange(400)*.01).tolist(),
            'accel_m_s2':(scale*(rotation@direction)*G+bias+rng.normal(0,.01,(400,3))).tolist(),
            'gyro_rad_s':(bg+rng.normal(0,.0001,(400,3))).tolist()})
    return doc, scale, bias


class GravityCalibrationTests(unittest.TestCase):
    def test_cli_default_is_cage_calibration(self):
        with patch('Interaction.calibrate_estimator_imu.collect',return_value=0) as capture:
            self.assertEqual(main(['collect','--drone-id','lb11','--output','/unused']),0)
        self.assertEqual(capture.call_args.args[0].calibration_method,'gravity_norm')

    def test_unknown_support_angles_recover_bias_scale_without_attitude_claim(self):
        data,scale,bias = gravity_dataset()
        result=fit_dataset(data)
        self.assertTrue(result['accepted'],result['failures'])
        np.testing.assert_allclose(result['axis_scales'],scale,atol=.0002)
        np.testing.assert_allclose(result['accel_bias_m_s2'],bias,atol=.002)
        self.assertIsNone(result['alignment_rotation_deg'])
        self.assertFalse(result['accel_alignment_calibrated'])
        self.assertFalse(result['accel_cross_axis_calibrated'])
        self.assertFalse(result['firmware_applied'])
        self.assertTrue(all('reference_force_m_s2' not in p for p in data['poses']))

    def test_common_mounting_rotation_remains_unidentified_and_uncorrected(self):
        rotation=rotation_xyz([5,-3,1])
        data,scale,bias=gravity_dataset(rotation=rotation)
        result=fit_dataset(data)
        self.assertTrue(result['accepted'],result['failures'])
        np.testing.assert_allclose(result['measured_from_reference'],np.diag(scale),atol=.0003)
        ref=np.array([0,0,G])
        corrected=np.linalg.solve(result['measured_from_reference'],scale*(rotation@ref)+bias-result['accel_bias_m_s2'])
        self.assertGreater(np.linalg.norm(corrected-ref),.8)
        self.assertFalse(result['accel_alignment_calibrated'])

    def test_holdouts_do_not_change_fit_and_can_reject(self):
        data,_,_=gravity_dataset();before=fit_dataset(data)
        p=data['poses'][18]
        x=np.array(p['accel_m_s2']);p['accel_m_s2']=(x+.3*x.mean(axis=0)/np.linalg.norm(x.mean(axis=0))).tolist()
        after=fit_dataset(data)
        self.assertEqual(after['measured_from_reference'],before['measured_from_reference'])
        self.assertEqual(after['accel_bias_m_s2'],before['accel_bias_m_s2'])
        self.assertFalse(after['fit_passed'])
        self.assertIn('validation_pose_01',after['failures'][0])

    def test_new_diagonal_holdouts_validate_without_needing_refit_rank(self):
        data,scale,bias=gravity_dataset()
        corners=np.array([[1,1,1],[-1,-1,-1],[1,-1,1],[-1,1,-1],[1,1,-1],[-1,-1,1]])/np.sqrt(3)
        before=fit_dataset(data)
        for pose,direction in zip(data['poses'][18:],corners):
            pose['accel_m_s2']=np.tile(scale*direction*G+bias,(400,1)).tolist()
        after=fit_dataset(data)
        self.assertTrue(after['accepted'],after['failures'])
        self.assertEqual(after['measured_from_reference'],before['measured_from_reference'])

    def test_bad_pose_count_coverage_motion_and_repeated_directions_rejected(self):
        original,_,_=gravity_dataset()
        for mode in ('six_poses','missing_holdout','repeat','hemisphere','moving','sensor_change'):
            data=deepcopy(original)
            if mode=='six_poses':data['poses']=data['poses'][:6]
            elif mode=='missing_holdout':data['poses']=data['poses'][:18]
            elif mode=='repeat':data['poses'][1]['accel_m_s2']=deepcopy(data['poses'][0]['accel_m_s2'])
            elif mode=='hemisphere':
                for p in data['poses']:
                    x=np.array(p['accel_m_s2']);x[:,2]=np.abs(x[:,2]);p['accel_m_s2']=x.tolist()
            elif mode=='moving':data['poses'][0]['gyro_rad_s'][20][0]=1.
            else:data['capture']={'firmware_parameters_before':{'imu_sensors':{'imuPhi':'0'}},'firmware_parameters_after':{'imu_sensors':{'imuPhi':'1'}}}
            with self.subTest(mode=mode),self.assertRaises(ValueError):fit_dataset(data)

    def test_no_relaxation_of_norm_limit_and_rejected_report_has_no_candidate(self):
        data,_,_=gravity_dataset()
        p=data['poses'][-1];x=np.array(p['accel_m_s2']);p['accel_m_s2']=(1.04*x).tolist()
        with TemporaryDirectory() as directory:
            root=Path(directory);write_json(root/'dataset.json',data)
            result=analyze_file(root/'dataset.json',root/'fit')
            self.assertFalse(result['fit_passed'])
            self.assertFalse((root/'fit/candidate.json').exists())
            self.assertIn('NOT calibrated',(root/'fit/report.md').read_text())

    def test_rejected_collection_prints_reason_without_inviting_flight(self):
        data,_,_=gravity_dataset()
        data['poses'][-1]['accel_m_s2']=(1.04*np.array(data['poses'][-1]['accel_m_s2'])).tolist()
        @contextmanager
        def source(uri):yield Mock(),{}
        with TemporaryDirectory() as directory:
            args=SimpleNamespace(output=Path(directory)/'session',uri='usb://0',drone_id='lb11',
                firmware_id='synthetic',fixture_id='cage',reference_note='unknown',
                duration_s=4.,settle_s=2.,fit_only=True)
            output=io.StringIO()
            with redirect_stdout(output),patch('Interaction.calibrate_estimator_imu.collect_window',side_effect=data['poses']), \
                    patch('Interaction.calibrate_estimator_imu.time.monotonic',side_effect=[0,11]):
                self.assertEqual(collect(args,prompt=lambda _: '',source=source),1)
            self.assertIn('corrected gravity norm RMSE',output.getvalue())
            self.assertIn('will NOT start',output.getvalue())
            self.assertNotIn('Continue with orchestrator',output.getvalue())

    def test_real_collector_standard_flight_and_native_replay(self):
        with TemporaryDirectory() as directory,redirect_stdout(io.StringIO()):
            result=run_case(Path(directory),'cage',1919,calibration_method='gravity_norm')
            self.assertTrue(result['capture_download_replay_completed'])
            self.assertEqual(result['flight_phases'],18)
            self.assertEqual(result['corrected']['segments'],1)
            local=Path(result['local_results'])
            data=json.loads((local/'dataset.json').read_text())
            fit=json.loads((local/'fit/candidate.json').read_text())
            self.assertEqual(len(data['poses']),24)
            self.assertTrue(fit['independent_static_validation_passed'])
            self.assertFalse(fit['accel_alignment_calibrated'])
            self.assertLess(np.linalg.norm(result['corrected']['rmse_rpy_deg'][:2]),
                            .25*np.linalg.norm(result['raw']['rmse_rpy_deg'][:2]))


if __name__=='__main__':unittest.main()
