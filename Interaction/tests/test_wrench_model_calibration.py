import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from Interaction.wrench_model_calibration import (
    _low_pass,
    apply_drone_calibration,
    identify_axis_alignment,
    identify_planar_braking_response,
    planar_braking_fit_is_current,
    save_drone_calibration,
)


class WrenchModelCalibrationTests(unittest.TestCase):
    @staticmethod
    def _current_planar_fit():
        return {
            'fit_schema_version': 2,
            'usable': True,
            'command_delay_s': 0.11,
            'command_time_constant_s': 0.07,
            'horizontal_acceleration_scale': 1.25,
            'fitted_step_acceleration_m_s2': 1.723382,
            'validated_max_acceleration_m_s2': 2.154228,
            'maximum_acceleration_extrapolation_ratio': 1.25,
            'direction_quality': {
                'gain_ratio': 1.02,
                'axis_xy': [0.0, 1.0],
                'minimum_trials_per_direction': 2,
                'minimum_windows_per_trial': 12,
                'minimum_direction_r_squared': 0.70,
                'minimum_direction_validation_r_squared': 0.50,
                'maximum_direction_nrmse': 0.20,
                'maximum_direction_gain_ratio': 1.25,
                'maximum_repeat_gain_deviation': 0.20,
                'directions': {
                    'positive': {
                        'direction_xy': [0.0, 1.0],
                        'trial_ids': [0, 2],
                        'trial_count': 2,
                        'trial_gains': [1.2375, 1.2375],
                        'gain': 1.2375,
                        'train_window_count': 24,
                        'train_r_squared': 0.95,
                        'train_nrmse': 0.05,
                        'validation_window_count': 24,
                        'validation_r_squared': 0.94,
                        'maximum_repeat_gain_deviation': 0.0,
                    },
                    'negative': {
                        'direction_xy': [0.0, -1.0],
                        'trial_ids': [1, 3],
                        'trial_count': 2,
                        'trial_gains': [1.26225, 1.26225],
                        'gain': 1.26225,
                        'train_window_count': 24,
                        'train_r_squared': 0.95,
                        'train_nrmse': 0.05,
                        'validation_window_count': 24,
                        'validation_r_squared': 0.94,
                        'maximum_repeat_gain_deviation': 0.0,
                    },
                },
            },
            'protocol': {
                'calibrated_axes': ['y'],
                'directions_xy': [[0.0, 1.0], [0.0, -1.0]],
                'repetitions': 2,
                'tilt_deg': 8.0,
            },
            'maneuver_count': 4,
        }

    @staticmethod
    def _synthetic_planar_samples(
            segment_gains=None, segment_command_amplitudes=None):
        dt = 0.02
        delay_s = 0.06
        tau_s = 0.08
        bias = 0.02
        samples = []
        command_amplitudes = (
            [1.8] * 4
            if segment_command_amplitudes is None
            else list(segment_command_amplitudes)
        )
        directions = tuple(
            np.array([0.0, 1.0 if index % 2 == 0 else -1.0])
            for index in range(len(command_amplitudes))
        )
        gains = (
            [1.35] * len(directions)
            if segment_gains is None else list(segment_gains)
        )
        if len(gains) != len(directions):
            raise ValueError("segment_gains must match the synthetic trials")
        if len(command_amplitudes) != len(directions):
            raise ValueError(
                "segment_command_amplitudes must match the synthetic trials"
            )

        for segment_id, (direction, gain, command_amplitude) in enumerate(zip(
                directions, gains, command_amplitudes)):
            base = segment_id * 2.0
            local_times = np.arange(0.007, 1.507, dt)
            times = base + local_times
            phase_starts_local = np.array([0.0, 0.2, 0.55, 0.75, 1.2])
            phase_starts = base + phase_starts_local
            phase_commands = np.array([
                0.0, command_amplitude, 0.0, -command_amplitude, 0.0,
            ])
            phase_names = [
                "level_before_acceleration",
                "accelerate",
                "level_before_brake",
                "brake",
                "level_after_brake",
            ]

            grid = np.unique(np.concatenate((times, phase_starts)))
            indices = np.searchsorted(
                phase_starts, grid, side="right"
            ) - 1
            command_grid = phase_commands[
                np.clip(indices, 0, len(phase_commands) - 1)
            ]
            filtered = _low_pass(command_grid, grid, tau_s)
            acceleration = (
                gain * np.interp(
                    times - delay_s,
                    grid,
                    filtered,
                    left=filtered[0],
                    right=filtered[-1],
                )
                + bias
            )
            velocity = np.zeros_like(times)
            velocity[1:] = np.cumsum(
                acceleration[:-1] * np.diff(times)
            )

            for timestamp, local_time, measured_velocity in zip(
                times, local_times, velocity
            ):
                phase_index = (
                    np.searchsorted(
                        phase_starts_local, local_time, side="right"
                    ) - 1
                )
                command = phase_commands[phase_index]
                samples.append({
                    "segment_id": segment_id,
                    "timestamp": float(timestamp),
                    "command_started_at": float(
                        phase_starts[phase_index]
                    ),
                    "phase": phase_names[phase_index],
                    "direction_xy": direction.tolist(),
                    "command_acceleration_xy": (
                        command * direction
                    ).tolist(),
                    "velocity_xy": (
                        measured_velocity * direction
                    ).tolist(),
                })
        return samples

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

    def test_identifies_delayed_planar_braking_response(self):
        fit = identify_planar_braking_response(
            self._synthetic_planar_samples(),
            expected_maneuver_count=4,
        )

        self.assertTrue(fit["usable"])
        self.assertAlmostEqual(fit["command_delay_s"], 0.06, delta=0.005)
        self.assertAlmostEqual(
            fit["command_time_constant_s"], 0.08, delta=0.01
        )
        self.assertAlmostEqual(
            fit["horizontal_acceleration_scale"], 1.35, delta=0.02
        )
        self.assertAlmostEqual(
            fit["acceleration_bias_m_s2"], 0.02, delta=0.01
        )
        self.assertGreater(fit["r_squared"], 0.99)
        self.assertGreater(fit["acceleration_validation_r_squared"], 0.99)
        self.assertEqual(fit["maneuver_count"], 4)
        self.assertLess(
            fit["direction_quality"]["gain_ratio"], 1.01
        )
        fit['protocol'] = {
            'calibrated_axes': ['y'],
            'directions_xy': [[0.0, 1.0], [0.0, -1.0]],
            'repetitions': 2,
            'tilt_deg': float(np.degrees(np.arctan2(1.8, 9.81))),
        }
        self.assertTrue(planar_braking_fit_is_current(fit))

    def test_multi_level_fit_uses_largest_exercised_step_and_signed_pairs(self):
        amplitudes = [1.38, 1.38, 2.45, 2.45, 3.57, 3.57]
        fit = identify_planar_braking_response(
            self._synthetic_planar_samples(
                segment_command_amplitudes=amplitudes
            ),
            expected_maneuver_count=6,
            minimum_trials_per_direction=3,
        )
        tilt_levels = [
            float(np.degrees(np.arctan2(value, 9.81)))
            for value in amplitudes[::2]
        ]
        fit['protocol'] = {
            'calibrated_axes': ['y'],
            'directions_xy': [[0.0, 1.0], [0.0, -1.0]],
            'repetitions': 3,
            'repetitions_per_tilt': 1,
            'tilt_deg': tilt_levels[-1],
            'tilt_levels_deg': tilt_levels,
        }

        self.assertAlmostEqual(
            fit['fitted_step_acceleration_m_s2'],
            1.35 * amplitudes[-1],
            delta=0.03,
        )
        self.assertEqual(fit['validated_max_acceleration_m_s2'], 5.0)
        self.assertEqual(
            len(fit['direction_quality']['per_tilt_gain_ratios']), 3
        )
        self.assertTrue(planar_braking_fit_is_current(fit))

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'wrench_calibration.json'
            save_drone_calibration(
                'lb11',
                {
                    'model_delay_s': [0.01, 0.02, 0.03],
                    'model_time_constant_s': [0.04, 0.05, 0.06],
                    'model_acceleration_scale': [1.0, 1.0, 1.0],
                    'axes': {},
                    'sample_count': 100,
                    'duration_s': 10.0,
                },
                {'hover_pwm': 31900},
                path,
                planar_braking_fit=fit,
            )
            resolved, _ = apply_drone_calibration(
                {
                    'impulse_estimator': {'window_s': 0.08},
                    'planar_braking_calibration': {
                        'directions_xy': [[0.0, 1.0], [0.0, -1.0]],
                        'tilt_levels_deg': tilt_levels,
                        'repetitions_per_tilt': 1,
                        'max_fit_delay_s': 0.25,
                        'max_fit_time_constant_s': 0.25,
                        'minimum_fit_r_squared': 0.70,
                        'minimum_validation_r_squared': 0.50,
                        'minimum_acceleration_scale': 0.20,
                        'maximum_acceleration_scale': 2.50,
                        'minimum_trials_per_direction': 3,
                        'minimum_windows_per_trial': 12,
                        'minimum_direction_r_squared': 0.70,
                        'minimum_direction_validation_r_squared': 0.50,
                        'maximum_direction_nrmse': 0.20,
                        'maximum_direction_gain_ratio': 1.25,
                        'maximum_repeat_gain_deviation': 0.20,
                        'maximum_acceleration_extrapolation_ratio': 1.25,
                    },
                    'control_handoff': {
                        'coast_max_acceleration_m_s2': 5.0,
                    },
                },
                'lb11',
                path,
            )
            self.assertEqual(
                resolved['control_handoff']['coast_max_acceleration_m_s2'],
                5.0,
            )

        corrupted = json.loads(json.dumps(fit))
        corrupted['direction_quality']['per_tilt_gain_ratios'][-1][
            'gain_ratio'
        ] = 1.24
        self.assertFalse(planar_braking_fit_is_current(corrupted))

        missing_level = json.loads(json.dumps(fit))
        missing_level['direction_quality']['directions']['negative'][
            'trial_tilt_levels_deg'
        ][-1] = missing_level['protocol']['tilt_levels_deg'][0]
        self.assertFalse(planar_braking_fit_is_current(missing_level))

    def test_multi_level_fit_rejects_crossed_direction_asymmetry(self):
        amplitudes = [1.38, 1.38, 2.45, 2.45, 3.57, 3.57]
        with self.assertRaisesRegex(ValueError, "opposed-direction gain ratio at"):
            identify_planar_braking_response(
                self._synthetic_planar_samples(
                    segment_gains=[0.8, 1.2, 1.0, 1.0, 1.2, 0.8],
                    segment_command_amplitudes=amplitudes,
                ),
                expected_maneuver_count=6,
                minimum_r_squared=-10.0,
                minimum_validation_r_squared=-10.0,
                minimum_trials_per_direction=3,
                minimum_direction_r_squared=-10.0,
                minimum_direction_validation_r_squared=-10.0,
                maximum_direction_nrmse=10.0,
                maximum_repeat_gain_deviation=0.30,
            )

    def test_rejects_incomplete_planar_protocol(self):
        with self.assertRaisesRegex(ValueError, "incomplete"):
            identify_planar_braking_response(
                self._synthetic_planar_samples(),
                expected_maneuver_count=3,
            )

    def test_rejects_planar_fit_that_does_not_explain_motion(self):
        rng = np.random.default_rng(3)
        bad_samples = [
            dict(
                sample,
                velocity_xy=rng.normal(0.0, 0.1, 2).tolist(),
            )
            for sample in self._synthetic_planar_samples()
        ]
        with self.assertRaisesRegex(
                ValueError, r"could not fit|quality gates"):
            identify_planar_braking_response(
                bad_samples,
                expected_maneuver_count=4,
            )

    def test_rejects_opposed_direction_gain_asymmetry(self):
        asymmetric_samples = self._synthetic_planar_samples(
            segment_gains=[0.50, 1.50, 0.50, 1.50]
        )

        with self.assertRaisesRegex(ValueError, "quality gates"):
            identify_planar_braking_response(
                asymmetric_samples,
                expected_maneuver_count=4,
            )

    def test_rejects_unrepeatable_direction_gain(self):
        unrepeatable_samples = self._synthetic_planar_samples(
            segment_gains=[1.35, 1.35, 0.70, 1.35]
        )

        with self.assertRaisesRegex(ValueError, "quality gates"):
            identify_planar_braking_response(
                unrepeatable_samples,
                expected_maneuver_count=4,
            )

    def test_planar_fit_is_applied_and_survives_impulse_only_resave(self):
        fit = {
            'model_delay_s': [0.01, 0.02, 0.03],
            'model_time_constant_s': [0.04, 0.05, 0.06],
            'model_acceleration_scale': [1.1, 1.2, 1.3],
            'axes': {},
            'sample_count': 100,
            'duration_s': 10.0,
        }
        braking = self._current_planar_fit()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'wrench_calibration.json'
            save_drone_calibration(
                'lb11', fit, {'hover_pwm': 31900}, path,
                planar_braking_fit=braking,
            )
            save_drone_calibration(
                'lb11', fit, {'hover_pwm': 31900}, path,
            )
            resolved, entry = apply_drone_calibration(
                {
                    'impulse_estimator': {'window_s': 0.08},
                    'planar_braking_calibration': {
                        'directions_xy': [[0.0, 1.0], [0.0, -1.0]],
                    },
                    'control_handoff': {
                        'coast_max_acceleration_m_s2': 5.0,
                    },
                },
                'lb11',
                path,
            )

            self.assertEqual(entry['planar_braking_fit'], braking)
            self.assertEqual(
                resolved['control_handoff'][
                    'coast_attitude_response_delay_s'
                ],
                0.11,
            )
            self.assertEqual(
                resolved['control_handoff'][
                    'coast_attitude_time_constant_s'
                ],
                0.07,
            )
            self.assertEqual(
                resolved['control_handoff'][
                    'coast_attitude_acceleration_scale'
                ],
                1.25,
            )
            self.assertEqual(
                resolved['control_handoff']['coast_max_acceleration_m_s2'],
                2.154228,
            )
            self.assertEqual(
                resolved['control_handoff']['coast_calibrated_direction_xy'],
                [0.0, 1.0],
            )

            with self.assertRaisesRegex(ValueError, 'quality gates.*rerun'):
                apply_drone_calibration(
                    {
                        'impulse_estimator': {'window_s': 0.08},
                        'planar_braking_calibration': {
                            'directions_xy': [
                                [0.0, 1.0], [0.0, -1.0]
                            ],
                            'minimum_direction_r_squared': 0.99,
                        },
                        'control_handoff': {},
                    },
                    'lb11',
                    path,
                )

            tighter, _ = apply_drone_calibration(
                {
                    'impulse_estimator': {'window_s': 0.08},
                    'planar_braking_calibration': {
                        'directions_xy': [
                            [0.0, 1.0], [0.0, -1.0]
                        ],
                        'maximum_acceleration_extrapolation_ratio': 1.10,
                    },
                    'control_handoff': {
                        'coast_max_acceleration_m_s2': 5.0,
                    },
                },
                'lb11',
                path,
            )
            self.assertAlmostEqual(
                tighter['control_handoff']['coast_max_acceleration_m_s2'],
                braking['fitted_step_acceleration_m_s2'] * 1.10,
                places=6,
            )

            with self.assertRaisesRegex(ValueError, 'tilt sweep.*rerun'):
                apply_drone_calibration(
                    {
                        'impulse_estimator': {'window_s': 0.08},
                        'planar_braking_calibration': {
                            'directions_xy': [
                                [0.0, 1.0], [0.0, -1.0]
                            ],
                            'tilt_levels_deg': [8.0, 14.0, 20.0],
                            'repetitions_per_tilt': 1,
                        },
                        'control_handoff': {},
                    },
                    'lb11',
                    path,
                )

            with self.assertRaisesRegex(ValueError, 'rerun --calibrate'):
                apply_drone_calibration(
                    {
                        'impulse_estimator': {'window_s': 0.08},
                        'planar_braking_calibration': {
                            'directions_xy': [[1.0, 0.0], [-1.0, 0.0]],
                        },
                        'control_handoff': {},
                    },
                    'lb11',
                    path,
                )

            with self.assertRaisesRegex(ValueError, 'rerun --calibrate'):
                apply_drone_calibration(
                    {
                        'impulse_estimator': {'window_s': 0.08},
                        'planar_braking_calibration': {
                            'directions_xy': [[0.0, 1.0], [0.0, -1.0]],
                        },
                        'control_handoff': {},
                    },
                    'lb11',
                    path,
                    runtime_interaction_axis='x',
                )

            with self.assertRaisesRegex(ValueError, 'direction.*runtime'):
                apply_drone_calibration(
                    {
                        'impulse_estimator': {'window_s': 0.08},
                        'planar_braking_calibration': {
                            'directions_xy': [[0.0, 1.0], [0.0, -1.0]],
                        },
                        'control_handoff': {},
                    },
                    'lb11',
                    path,
                    runtime_interaction_direction_xy=[1.0, 0.0],
                )

            diagonal_braking = json.loads(json.dumps(braking))
            positive_diagonal = [2 ** -0.5, 2 ** -0.5]
            negative_diagonal = [-2 ** -0.5, -2 ** -0.5]
            diagonal_braking['protocol']['calibrated_axes'] = ['x', 'y']
            diagonal_braking['protocol']['directions_xy'] = [
                positive_diagonal, negative_diagonal,
            ]
            diagonal_quality = diagonal_braking['direction_quality']
            diagonal_quality['axis_xy'] = positive_diagonal
            diagonal_quality['directions']['positive'][
                'direction_xy'
            ] = positive_diagonal
            diagonal_quality['directions']['negative'][
                'direction_xy'
            ] = negative_diagonal
            save_drone_calibration(
                'diagonal',
                fit,
                {'hover_pwm': 31900},
                path,
                planar_braking_fit=diagonal_braking,
            )
            with self.assertRaisesRegex(ValueError, 'runtime.*y'):
                apply_drone_calibration(
                    {
                        'impulse_estimator': {'window_s': 0.08},
                        'planar_braking_calibration': {
                            'directions_xy': [
                                [2 ** -0.5, 2 ** -0.5],
                                [-2 ** -0.5, -2 ** -0.5],
                            ],
                        },
                        'control_handoff': {},
                    },
                    'diagonal',
                    path,
                    runtime_interaction_axis='y',
                )

            nonfinite_pooled_quality = json.loads(json.dumps(braking))
            nonfinite_pooled_quality['r_squared'] = float('nan')
            nonfinite_pooled_quality[
                'acceleration_validation_r_squared'
            ] = float('nan')
            save_drone_calibration(
                'nonfinite',
                fit,
                {'hover_pwm': 31900},
                path,
                planar_braking_fit=nonfinite_pooled_quality,
            )
            with self.assertRaisesRegex(ValueError, 'quality gates.*rerun'):
                apply_drone_calibration(
                    {
                        'impulse_estimator': {'window_s': 0.08},
                        'planar_braking_calibration': {
                            'directions_xy': [
                                [0.0, 1.0], [0.0, -1.0]
                            ],
                            'minimum_fit_r_squared': 0.70,
                            'minimum_validation_r_squared': 0.50,
                        },
                        'control_handoff': {},
                    },
                    'nonfinite',
                    path,
                )

    def test_legacy_planar_fit_is_not_applied_without_current_quality_metadata(self):
        fit = {
            'model_delay_s': [0.01, 0.02, 0.03],
            'model_time_constant_s': [0.04, 0.05, 0.06],
            'model_acceleration_scale': [1.1, 1.2, 1.3],
            'axes': {},
            'sample_count': 100,
            'duration_s': 10.0,
        }
        legacy_braking = {
            'usable': True,
            'command_delay_s': 0.20,
            'command_time_constant_s': 0.20,
            'horizontal_acceleration_scale': 2.0,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'wrench_calibration.json'
            save_drone_calibration(
                'lb11', fit, {'hover_pwm': 31900}, path,
            )
            with path.open() as stream:
                document = json.load(stream)
            document['drones']['lb11']['planar_braking_fit'] = legacy_braking
            path.write_text(json.dumps(document))
            resolved, _ = apply_drone_calibration(
                {
                    'impulse_estimator': {'window_s': 0.08},
                    'control_handoff': {
                        'coast_attitude_response_delay_s': 0.12,
                    },
                },
                'lb11',
                path,
            )

            self.assertEqual(
                resolved['control_handoff'][
                    'coast_attitude_response_delay_s'
                ],
                0.12,
            )

    def test_current_fit_rejects_semantically_unsafe_finite_values(self):
        fit = self._current_planar_fit()
        self.assertTrue(planar_braking_fit_is_current(fit))
        corruptions = (
            ('command_delay_s', 1e6),
            ('command_time_constant_s', 1e6),
            ('horizontal_acceleration_scale', 1e-9),
            ('horizontal_acceleration_scale', 1e6),
            ('validated_max_acceleration_m_s2', 1e6),
        )
        for field, value in corruptions:
            with self.subTest(field=field, value=value):
                corrupted = json.loads(json.dumps(fit))
                corrupted[field] = value
                self.assertFalse(planar_braking_fit_is_current(corrupted))

        one_sided = json.loads(json.dumps(fit))
        one_sided['protocol']['directions_xy'] = [[0.0, 1.0]]
        self.assertFalse(planar_braking_fit_is_current(one_sided))

        missing_signed_evidence = json.loads(json.dumps(fit))
        del missing_signed_evidence['direction_quality']['directions'][
            'negative'
        ]
        self.assertFalse(
            planar_braking_fit_is_current(missing_signed_evidence)
        )

        insufficient_trials = json.loads(json.dumps(fit))
        positive = insufficient_trials['direction_quality']['directions'][
            'positive'
        ]
        positive['trial_ids'] = [0]
        positive['trial_count'] = 1
        positive['trial_gains'] = [positive['gain']]
        self.assertFalse(planar_braking_fit_is_current(insufficient_trials))

        failed_direction_quality = json.loads(json.dumps(fit))
        failed_direction_quality['direction_quality']['directions'][
            'positive'
        ]['train_r_squared'] = 0.1
        self.assertFalse(
            planar_braking_fit_is_current(failed_direction_quality)
        )

        forged_repeatability = json.loads(json.dumps(fit))
        positive = forged_repeatability['direction_quality']['directions'][
            'positive'
        ]
        positive['trial_gains'] = [0.9, 1.575]
        positive['maximum_repeat_gain_deviation'] = 0.0
        self.assertFalse(planar_braking_fit_is_current(forged_repeatability))

        nan_fit = json.loads(json.dumps(fit))
        nan_fit['command_delay_s'] = float('nan')
        self.assertFalse(planar_braking_fit_is_current(nan_fit))

    def test_save_rejects_bad_schema_without_overwriting_and_preserves_metadata(self):
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
            unsupported = {
                'schema_version': 999,
                'drones': {'lb11': {'keep': 'unchanged'}},
            }
            original = json.dumps(unsupported, sort_keys=True)
            path.write_text(original)
            with self.assertRaisesRegex(ValueError, 'unsupported'):
                save_drone_calibration(
                    'lb11', fit, {'hover_pwm': 31900}, path,
                    planar_braking_fit=self._current_planar_fit(),
                )
            self.assertEqual(path.read_text(), original)

            path.unlink()
            save_drone_calibration(
                'lb11', fit, {'hover_pwm': 31900}, path,
                planar_braking_fit=self._current_planar_fit(),
            )
            with path.open() as stream:
                document = json.load(stream)
            document['drones']['lb11']['operator_note'] = 'preserve me'
            document['drones']['lb11']['control_handoff'][
                'future_metadata'
            ] = {'keep': True}
            path.write_text(json.dumps(document))

            save_drone_calibration(
                'lb11', fit, {'hover_pwm': 32000}, path,
                planar_braking_fit=self._current_planar_fit(),
            )
            with path.open() as stream:
                updated = json.load(stream)['drones']['lb11']
            self.assertEqual(updated['operator_note'], 'preserve me')
            self.assertEqual(
                updated['control_handoff']['future_metadata'], {'keep': True}
            )

    def test_default_runtime_acceleration_ceiling_is_not_bypassed(self):
        fit = {
            'model_delay_s': [0.01, 0.02, 0.03],
            'model_time_constant_s': [0.04, 0.05, 0.06],
            'model_acceleration_scale': [1.1, 1.2, 1.3],
            'axes': {},
            'sample_count': 100,
            'duration_s': 10.0,
        }
        braking = self._current_planar_fit()
        braking['protocol']['tilt_deg'] = 25.0
        braking['horizontal_acceleration_scale'] = 0.874417
        braking['fitted_step_acceleration_m_s2'] = 4.0
        braking['validated_max_acceleration_m_s2'] = 5.0
        lower_gain = 0.874417 / 1.01
        upper_gain = lower_gain * 1.02
        for label, gain in (
            ('positive', lower_gain), ('negative', upper_gain)
        ):
            braking['direction_quality']['directions'][label]['gain'] = gain
            braking['direction_quality']['directions'][label][
                'trial_gains'
            ] = [gain, gain]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'wrench_calibration.json'
            save_drone_calibration(
                'lb11', fit, {'hover_pwm': 31900}, path,
                planar_braking_fit=braking,
            )
            resolved, _ = apply_drone_calibration(
                {
                    'impulse_estimator': {'window_s': 0.08},
                    'planar_braking_calibration': {
                        'directions_xy': [[0.0, 1.0], [0.0, -1.0]],
                    },
                    'control_handoff': {},
                },
                'lb11',
                path,
            )
            self.assertEqual(
                resolved['control_handoff']['coast_max_acceleration_m_s2'],
                5.0,
            )


if __name__ == '__main__':
    unittest.main()
