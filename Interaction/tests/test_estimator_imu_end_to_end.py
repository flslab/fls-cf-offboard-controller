"""Real collector/task/download/native-kernel integration without hardware."""
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from Interaction.simulate_estimator_imu import run_case


class EndToEndTests(unittest.TestCase):
    def test_hover_xy_frozen_before_motion_and_repaired_after_download(self):
        with TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            result=run_case(Path(directory),'hover_xy',1929,calibration_method='gravity_norm',
                            flight_accel_bias=[.12,-.24,0.])
            local=Path(result['local_results'])
            report=json.loads((local/'flight/replay/report.json').read_text())
            capture=json.loads((local/'flight/report.json').read_text())
            hover=report['hover_roll_pitch']
            self.assertTrue(capture['capture_completed'])
            self.assertTrue(hover['fit']['accepted'],hover['fit']['failures'])
            self.assertTrue(hover['completed'])
            np.testing.assert_allclose(hover['fit']['additional_body_accel_bias_m_s2'],[.12,-.24,0.],atol=.015)
            for mode in ('static_summary','initialized_summary'):
                self.assertEqual(hover[mode]['segments'],1)
            self.assertLess(np.linalg.norm(hover['initialized_summary']['rmse_rpy_deg'][:2]),
                            .2*np.linalg.norm(hover['static_summary']['rmse_rpy_deg'][:2]))
            rows=[json.loads(r) for r in (local/'flight/packets.jsonl').read_text().splitlines()]
            freeze=next(r['received_s'] for r in rows if r['group']=='event' and r['data']['name']=='hover_roll_pitch_window_frozen')
            first_motion=min(r['received_s'] for r in rows if r['phase']=='+X_out')
            self.assertLessEqual(freeze,first_motion)
            self.assertFalse(hover['fit']['gyro_changed'])
            self.assertFalse(report['firmware_calibration_applied'])

    def test_saved_accel_fit_survives_reboot_with_fresh_floor_gyro_zero(self):
        with TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            result=run_case(Path(directory),'reboot',1828,
                            calibration_method='gravity_norm',reboot_after_fit=True)
            local=Path(result['local_results'])
            original=json.loads((local/'fit/candidate.json').read_text())
            replay=json.loads((local/'flight/replay/report.json').read_text())
            refresh=replay['preflight_gyro_zero']
            self.assertTrue(result['capture_download_replay_completed'])
            self.assertGreater(np.linalg.norm(np.array(refresh['gyro_residual_bias_rad_s'])-
                original['gyro_residual_bias_rad_s']),np.deg2rad(.1))
            self.assertFalse(refresh['accelerometer_fit_changed'])
            self.assertFalse(refresh['firmware_calibration_applied'])
            self.assertEqual(result['flight_phases'],18)

    def test_six_faces_standard_flight_download_and_native_replay(self):
        with TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            root=Path(directory)
            result=run_case(root,'regression',1717)
            self.assertTrue(result['capture_download_replay_completed'])
            self.assertEqual(result['flight_phases'],18)
            self.assertEqual(result['raw']['segments'],1)
            self.assertEqual(result['corrected']['segments'],1)
            self.assertLess(np.linalg.norm(result['corrected']['rmse_rpy_deg'][:2]),
                .25*np.linalg.norm(result['raw']['rmse_rpy_deg'][:2]))
            lifecycle=json.loads((root/'regression/lifecycle.json').read_text())
            self.assertTrue(lifecycle['flight_capture_completed'])
            ready=next(i for i,e in enumerate(lifecycle['events']) if e.get('status')=='READY')
            arm=next(i for i,e in enumerate(lifecycle['events']) if e.get('event')=='arm')
            self.assertLess(ready,arm)
            self.assertEqual(lifecycle['events'][-1]['event'],'LANDED')
            local=Path(result['local_results'])
            fit=json.loads((local/'fit/candidate.json').read_text())
            self.assertFalse(fit['independent_static_validation_passed'])
            self.assertFalse(fit['firmware_applied'])
            self.assertEqual(len(json.loads((local/'dataset.json').read_text())['poses']),6)
            rows=[json.loads(line) for line in (local/'flight/packets.jsonl').read_text().splitlines()]
            ticks=[r['cf_log_tick_ms_mod24'] for r in rows if r['group']=='imu']
            self.assertTrue(all(b-a==10 for a,b in zip(ticks,ticks[1:])))


if __name__=='__main__':unittest.main()
