"""Validate the XYZ-only calibration loader used by level coast."""

import copy
import json
from pathlib import Path
import tempfile
import unittest

from Interaction.wrench_model_calibration import apply_detection_calibration


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'calibration.json'
        self.entry = {'updated_at': 'test', 'impulse_estimator': {
            'model_delay_s': [0, 0, .04],
            'model_time_constant_s': [0, 0, 0],
            'model_acceleration_scale': [.8, .796, .706]},
            'planar_braking_fit': {'deliberately': 'invalid retired fit'},
            'control_handoff': {'coast_max_acceleration_m_s2': 999},
            'motor_model': {'hover_pwm': 1}}

    def save(self):
        self.path.write_text(json.dumps({'schema_version': 1,
                                        'drones': {'lb11': self.entry}}))

    def test_only_xyz_alignment_is_loaded(self):
        self.save()
        original = {'firmware_auto_brake': {'enabled': True},
                    'control_handoff': {'coast_max_acceleration_m_s2': 5},
                    'motor_model': {'hover_pwm': 31900},
                    'impulse_estimator': {'window_s': .08}}
        before = copy.deepcopy(original)
        result = apply_detection_calibration(original, 'lb11', self.path)
        self.assertEqual(original, before)
        for key in ['firmware_auto_brake', 'control_handoff', 'motor_model']:
            self.assertEqual(result[key], original[key])
        self.assertEqual(result['impulse_estimator']['window_s'], .08)
        self.assertEqual(result['impulse_estimator']['model_acceleration_scale'],
                         [.8, .796, .706])
        self.assertEqual(result['wrench_detection_calibration']['status'], 'loaded')

    def test_missing_file_or_drone_preserves_defaults(self):
        config = {'impulse_estimator': {'model_acceleration_scale': [1, 1, 1]}}
        for name in ['before_file_exists', 'lb12']:
            result = apply_detection_calibration(config, name, self.path)
            self.assertEqual(result['impulse_estimator'], config['impulse_estimator'])
            self.assertEqual(result['wrench_detection_calibration']['status'],
                             'not_found')
            self.save()

    def test_malformed_saved_vectors_are_rejected(self):
        for value in [[0, 1, 1], [1, float('nan'), 1], [1, 1], [True, 1, 1]]:
            self.entry['impulse_estimator']['model_acceleration_scale'] = value
            self.save()
            with self.assertRaisesRegex(ValueError, 'invalid saved wrench XYZ'):
                apply_detection_calibration({}, 'lb11', self.path)


if __name__ == '__main__':
    unittest.main()
