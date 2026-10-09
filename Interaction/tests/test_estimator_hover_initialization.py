"""Frozen, causal X/Y hover initialization and actual-C held-out replay."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from Interaction.estimator_hover_initialization import (
    fit_hover_roll_pitch, apply_hover_roll_pitch, held_out_hover_rows, hover_window)
from Interaction.replay_estimator3 import run_replay


def hover_capture():
    A=np.array([[1.02,.01,0.],[0.,.99,.005],[0.,0.,1.01]])
    bias=np.array([.02,-.04,.03])
    extra=np.array([.12,-.24,.04])
    fit=dict(fit_passed=True, measured_from_reference=A.tolist(),accel_bias_m_s2=bias.tolist(),
             gyro_residual_bias_rad_s=[0.,0.,0.],calibration_method='gravity_norm')
    rows=[dict(group='event',received_s=99.99,phase='preflight',data={'name':'capture_ready'},cf_log_tick_ms_mod24=None),
          dict(group='event',received_s=100.,phase='hover_start',data=hover_window(100.),cf_log_tick_ms_mod24=None)]
    for i in range(1100):
        t=100.+i*.01
        phase='hover_start' if i<501 else ('+X_out' if i<800 else '-Y_out')
        base=dict(received_s=t,phase=phase,cf_log_tick_ms_mod24=50000+10*i)
        acc=(A@(np.array([0.,0.,9.81])+extra)+bias)/9.81
        rows += [{**base,'group':'kf','data':dict(zip([f'kalman.q{k}' for k in range(4)],[1.,0.,0.,0.]))},
                 {**base,'group':'state','data':{**dict(zip(['stateEstimate.'+k for k in 'xyz'],[0.,0.,.8])),
                    **dict(zip(['stateEstimate.v'+k for k in 'xyz'],[0.,0.,0.]))}},
                 {**base,'group':'vicon','data':{'position_m':[0.,0.,.8]}},
                 {**base,'group':'command','data':{'position_m':[0.,0.,.8],'estimator':2,'yaw_deg':0.}},
                 {**base,'group':'imu','data':{**dict(zip(['acc.'+k for k in 'xyz'],acc)),
                    **dict(zip(['gyro.'+k for k in 'xyz'],[0.,0.,0.]))}}]
    return rows,fit,extra


class HoverFitTests(unittest.TestCase):
    def test_xy_only_preserves_corrected_z_gyro_and_source_candidate(self):
        rows,candidate,extra=hover_capture();before=deepcopy(candidate)
        fit=fit_hover_roll_pitch(rows,candidate,'identity')
        self.assertTrue(fit['accepted'],fit['failures'])
        np.testing.assert_allclose(fit['additional_body_accel_bias_m_s2'],[extra[0],extra[1],0.],atol=1e-10)
        effective=apply_hover_roll_pitch(candidate,fit,'identity')
        self.assertEqual(candidate,before)
        self.assertEqual(effective['gyro_residual_bias_rad_s'],before['gyro_residual_bias_rad_s'])
        A=np.asarray(candidate['measured_from_reference'])
        for measured in [np.array([.1,.2,9.7]),np.array([-1.,2.,8.])]:
            old=np.linalg.solve(A,measured-candidate['accel_bias_m_s2'])
            new=np.linalg.solve(A,measured-effective['accel_bias_m_s2'])
            self.assertAlmostEqual(old[2],new[2],12)
        self.assertFalse(fit['yaw_calibrated'])

    def test_holdout_cannot_change_the_fit_and_training_is_excluded(self):
        rows,candidate,_=hover_capture()
        original=fit_hover_roll_pitch(rows,candidate,'identity')
        changed=deepcopy(rows)
        for row in changed:
            if row['received_s']>=105. and row['group']=='imu':row['data']['acc.y']+=5.
        fit=fit_hover_roll_pitch(changed,candidate,'identity')
        self.assertEqual(original,fit)
        selected=held_out_hover_rows(rows,fit)
        ready=next(r for r in selected if r['group']=='event')
        self.assertEqual(ready['received_s'],105.)
        # Earlier rows supply causal seed history, but no replay output precedes capture_ready.
        self.assertTrue(all(r['received_s']>=104.94 for r in selected))

    def test_motion_dropout_future_reference_and_bad_metadata_are_rejected(self):
        for fault in ('motion','target','dropout','future','drift','wrong_phase','nonfinite'):
            with self.subTest(fault=fault):
                rows,candidate,_=hover_capture()
                if fault=='dropout':rows=[r for r in rows if not(r['group']=='imu' and 103.<r['received_s']<103.2)]
                for row in rows:
                    if not 102.<=row['received_s']<105.:continue
                    if fault=='motion' and row['group']=='vicon':row['data']['position_m'][0]=.1*(row['received_s']-102.)
                    if fault=='target' and row['group']=='command':row['data']['position_m'][0]=.01*(row['received_s']-102.)
                    if fault=='future' and row['group']=='kf':row['cf_log_tick_ms_mod24']+=1000
                    if fault=='drift' and row['group']=='imu':row['data']['acc.y']+=(row['received_s']-102.)*.02
                    if fault=='wrong_phase':row['phase']='translation'
                    if fault=='nonfinite' and row['group']=='imu':row['data']['acc.x']=float('nan')
                fit=fit_hover_roll_pitch(rows,candidate,'identity')
                self.assertFalse(fit['accepted'])
                self.assertTrue(fit['failures'])
                with self.assertRaises(ValueError):apply_hover_roll_pitch(candidate,fit,'identity')

    def test_cannot_apply_to_another_fit_or_change_z_or_yaw_flags(self):
        rows,candidate,_=hover_capture();fit=fit_hover_roll_pitch(rows,candidate,'identity')
        for fault in ('identity','z','yaw','gyro'):
            altered=deepcopy(fit)
            if fault=='identity':altered['source_candidate_sha256']='other'
            if fault=='z':altered['additional_body_accel_bias_m_s2'][2]=.01
            if fault=='yaw':altered['yaw_calibrated']=True
            if fault=='gyro':altered['gyro_changed']=True
            with self.assertRaises(ValueError):apply_hover_roll_pitch(candidate,altered,'identity')

    def test_missing_initial_hover_does_not_fall_back_to_interaction(self):
        rows,candidate,_=hover_capture()
        rows=[r for r in rows if r['group']!='event']
        for r in rows:r['phase']='translation'
        self.assertFalse(fit_hover_roll_pitch(rows,candidate,'identity')['accepted'])

    def test_real_c_kernel_improves_unseen_data_without_reset_or_overwriting_fit(self):
        rows,candidate,_=hover_capture()
        with TemporaryDirectory() as directory:
            root=Path(directory);path=root/'candidate.json';path.write_text(json.dumps(candidate))
            fingerprint=hashlib.sha256(path.read_bytes()).hexdigest()
            packets=root/'packets.jsonl';packets.write_text(''.join(json.dumps(r)+'\n' for r in rows))
            result=run_replay(packets,path,root/'replay')
            hover=result['hover_roll_pitch']
            self.assertTrue(hover['completed'],hover)
            self.assertTrue(hover['training_excluded'])
            self.assertTrue(hover['roll_pitch_improved_relative_to_default'])
            self.assertEqual(hover['static_summary']['segments'],1)
            self.assertEqual(hover['initialized_summary']['segments'],1)
            self.assertGreater(hover['initialized_summary']['samples'],500)
            self.assertTrue(all(r['phase']!='hover_start' for r in hover['initialized']['samples'] if r['time_s']>.02))
            self.assertLess(max(hover['initialized_summary']['rmse_rpy_deg'][:2]),.1)
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),fingerprint)
            self.assertFalse(result['firmware_calibration_applied'])
            self.assertFalse(hover['yaw_calibrated'])


if __name__=='__main__':unittest.main()
