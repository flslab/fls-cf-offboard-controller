import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from Interaction.wrench_model_calibration import (
    _low_pass,
    apply_drone_calibration,
    identify_axis_alignment,
    save_drone_calibration,
)


class WrenchModelCalibrationTests(unittest.TestCase):
    def test_identifies_axis_delay_time_constant_and_gain(self):
        dt = 0.01
        times = np.arange(0.0, 24.0, dt)
        phase = 2.0 * np.pi * (
            0.2 * times + 0.5 * (1.1 - 0.2) / 24.0 * times ** 2
        )
        model_acceleration = np.sin(phase)
        filtered = _low_pass(model_acceleration, times, 0.06)
        actual_acceleration = 1.5 * np.interp(
            times - 0.04,
            times,
            filtered,
            left=filtered[0],
        ) + 0.03
        velocity = np.zeros_like(times)
        velocity[1:] = np.cumsum(actual_acceleration[:-1] * dt)
        velocity += np.random.default_rng(4).normal(0.0, 0.001, len(times))

        fit = identify_axis_alignment(
            times, model_acceleration, velocity, window_s=0.08
        )

        self.assertAlmostEqual(fit['model_delay_s'], 0.04, delta=0.005)
        self.assertAlmostEqual(
            fit['model_time_constant_s'], 0.06, delta=0.01
        )
        self.assertAlmostEqual(
            fit['model_acceleration_scale'], 1.5, delta=0.03
        )
        self.assertGreater(fit['r_squared'], 0.99)

    def test_saved_calibration_is_loaded_per_drone(self):
        fit = {
            'model_delay_s': [0.01, 0.02, 0.03],
            'model_time_constant_s': [0.04, 0.05, 0.06],
            'model_acceleration_scale': [1.1, 1.2, 1.3],
            'axes': {},
            'sample_count': 100,
            'duration_s': 10.0,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'wrench_calibration.json'
            save_drone_calibration('lb11', fit, {'hover_pwm': 31900}, path)
            resolved, entry = apply_drone_calibration(
                {'impulse_estimator': {'window_s': 0.08}}, 'lb11', path
            )

            self.assertEqual(
                resolved['impulse_estimator']['model_delay_s'],
                [0.01, 0.02, 0.03],
            )
            self.assertEqual(entry['fit']['sample_count'], 100)
            with path.open() as stream:
                document = json.load(stream)
            self.assertIn('lb11', document['drones'])


if __name__ == '__main__':
    unittest.main()
