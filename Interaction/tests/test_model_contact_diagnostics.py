import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from Interaction.model_contact_diagnostics import (
    ModelContactDiagnostics, ShortWindowContactDiagnostics,
    StationaryForceBaseline,
)
from Interaction.wrench_contact_detector import ContactChannelDetector
from Interaction.wrench_model_calibration import apply_detection_calibration


DETECTOR = dict(component_thresholds=[.08, .08, .12],
                covariance_floor=[.015, .015, .025], confidence_sigma=2.5,
                onset_evidence_s=.05, release_time_s=.15, release_ratio=.55,
                onset_axes=[0, 1], release_projection_axes=[0, 1])


class BaselineTests(unittest.TestCase):
    def learn(self, baseline, force=(0, -.06, -.4)):
        for i in range(60):
            baseline.observe(force, [0, 0, 0], [0, 0, 0], i * .01,
                             valid=True, allow_learning=True,
                             contact_evidence=False)

    def test_learns_xy_only_from_stationary_history(self):
        baseline = StationaryForceBaseline()
        self.learn(baseline)
        self.assertTrue(baseline.ready)
        np.testing.assert_allclose(baseline.bias, [0, -.06, 0])

    def test_freezes_persistent_contact_instead_of_learning_it(self):
        baseline = StationaryForceBaseline()
        self.learn(baseline)
        for i in range(60, 260):
            baseline.observe([0, .20, 0], [0, 0, 0], [0, 0, 0], i * .01,
                             valid=True, allow_learning=True,
                             contact_evidence=True)
        np.testing.assert_allclose(baseline.bias, [0, -.06, 0])

    def test_subthreshold_force_step_is_not_silently_absorbed(self):
        baseline = StationaryForceBaseline()
        self.learn(baseline, (0, 0, 0))
        for i in range(60, 260):
            baseline.observe([0, .06, 0], [0, 0, 0], [0, 0, 0], i * .01,
                             valid=True, allow_learning=True,
                             contact_evidence=False)
        np.testing.assert_array_equal(baseline.bias, np.zeros(3))

    def test_nonidle_or_moving_samples_cannot_bootstrap(self):
        for kwargs in [dict(allow_learning=False, velocity=[0, 0, 0]),
                       dict(allow_learning=True, velocity=[0, .1, 0])]:
            baseline = StationaryForceBaseline()
            for i in range(80):
                baseline.observe([0, -.06, 0], kwargs['velocity'], [0, 0, 0],
                                 i * .01, valid=True,
                                 allow_learning=kwargs['allow_learning'],
                                 contact_evidence=False)
            self.assertFalse(baseline.ready)

    def test_gap_duplicate_and_invalid_input_discard_old_baseline(self):
        for timestamp, valid in [(2.0, True), (.59, True), (.60, False)]:
            baseline = StationaryForceBaseline()
            self.learn(baseline)
            baseline.observe([0, -.06, 0], [0, 0, 0], [0, 0, 0], timestamp,
                             valid=valid, allow_learning=True,
                             contact_evidence=False)
            self.assertFalse(baseline.ready)
            np.testing.assert_array_equal(baseline.bias, [0, 0, 0])

    def test_unstable_bootstrap_is_rejected(self):
        baseline = StationaryForceBaseline()
        for i in range(80):
            baseline.observe([0, .06 * (-1)**i, 0], [0, 0, 0], [0, 0, 0],
                             i * .01, valid=True, allow_learning=True,
                             contact_evidence=False)
        self.assertFalse(baseline.ready)


class DiagnosticsTests(unittest.TestCase):
    def test_diagnostic_failure_cannot_abort_the_controller(self):
        diagnostic = ModelContactDiagnostics(DETECTOR)
        with patch.object(diagnostic, 'observe', side_effect=ValueError('test')):
            result = diagnostic.observe_safely()
        self.assertFalse(result['valid'])
        self.assertFalse(result['command_authority'])
        self.assertEqual(result['reason'], 'diagnostic_error')

    def observe(self, diagnostic, i, force, *, armed=True, valid=True):
        return diagnostic.observe(
            force=force, covariance=np.diag([.0004, .0004, .0009]),
            velocity=[0, 0, 0], angular_velocity=[0, 0, 0],
            timestamp=i * .01, valid=valid, armed=armed, idle=True)

    def test_bias_correction_separates_background_from_onset(self):
        diagnostic = ModelContactDiagnostics(DETECTOR)
        force = np.array([0., -.06, -.4])
        original = force.copy()
        for i in range(60):
            row = self.observe(diagnostic, i, force)
        np.testing.assert_array_equal(force, original)
        self.assertTrue(row['baseline_ready'])
        raw_started = corrected_started = False
        for i in range(60, 100):
            row = self.observe(diagnostic, i, [0, -.10, -.4])
            raw_started |= row['raw']['started']
            corrected_started |= row['corrected']['started']
        self.assertTrue(raw_started)
        self.assertFalse(corrected_started)
        for i in range(100, 125):
            row = self.observe(diagnostic, i, [0, .25, -.4])
            corrected_started |= row['corrected']['started']
        self.assertTrue(corrected_started)
        self.assertFalse(row['command_authority'])

    def test_disarmed_detector_can_collect_idle_baseline_but_never_detect(self):
        diagnostic = ModelContactDiagnostics(DETECTOR)
        for i in range(60):
            row = self.observe(diagnostic, i, [0, -.06, 0], armed=False)
            self.assertIsNone(row['raw'])
            self.assertIsNone(row['corrected'])
        self.assertTrue(row['baseline_ready'])

    def test_baseline_for_current_decision_does_not_use_current_sample(self):
        diagnostic = ModelContactDiagnostics(DETECTOR)
        for i in range(31):
            row = self.observe(diagnostic, i, [0, -.06, 0])
        self.assertFalse(row['baseline_ready'])
        self.assertTrue(diagnostic.baseline.ready)
        row = self.observe(diagnostic, 31, [0, -.06, 0])
        self.assertTrue(row['baseline_ready'])
        np.testing.assert_allclose(row['corrected_force_N'], [0, 0, 0])

    def test_invalid_interval_is_unavailable_not_a_negative_label(self):
        diagnostic = ModelContactDiagnostics(DETECTOR)
        for i in range(60):
            self.observe(diagnostic, i, [0, -.06, 0])
        row = self.observe(diagnostic, 160, [0, .5, 0])
        self.assertFalse(row['valid'])
        self.assertNotIn('corrected', row)
        self.assertFalse(diagnostic.baseline.ready)


class ShortWindowTests(unittest.TestCase):
    def make(self, **kwargs):
        return ShortWindowContactDiagnostics(
            DETECTOR, mass=.17,
            impulse_config=dict(window_s=.08, minimum_window_s=.05,
                                max_dt_s=.05), **kwargs)

    def sample(self, monitor, timestamp, velocity=(0, 0, 0), **kwargs):
        values = dict(force=[0, 0, 0], covariance=np.eye(3) * .0004,
                      velocity=velocity, angular_velocity=[0, 0, 0],
                      expected_acceleration=[0, 0, 0], timestamp=timestamp,
                      state_valid=True, long_valid=True, armed=True, idle=True)
        values.update(kwargs)
        return monitor.observe_safely(**values)

    def test_short_window_ready_before_long_without_command_authority(self):
        monitor = self.make()
        for i in range(4):
            row = self.sample(monitor, i * .01, long_valid=False)
        self.assertTrue(row['valid'])
        self.assertFalse(row['long_window']['valid'])
        self.assertEqual(row['schema_version'], 2)
        self.assertEqual(row['window_s'], .03)
        self.assertEqual(row['long_window']['window_s'], .08)
        self.assertFalse(row['command_authority'])

    def test_config_is_copied_and_calibration_is_preserved(self):
        config = dict(window_s=.08, minimum_window_s=.05, max_dt_s=.05,
                      model_delay_s=[0, 0, .04],
                      model_time_constant_s=[.02, .03, .04],
                      model_acceleration_scale=[.8, .79, .7])
        original = copy.deepcopy(config)
        monitor = ShortWindowContactDiagnostics(DETECTOR, mass=.17,
                                                 impulse_config=config)
        self.assertEqual(config, original)
        for key in ('model_delay_s', 'model_time_constant_s',
                    'model_acceleration_scale'):
            self.assertEqual(monitor.impulse_config[key], original[key])
        self.assertEqual(monitor.short.corrected.minimum_onset_duration_s, .03)
        self.assertEqual(monitor.long.corrected.minimum_onset_duration_s, 0)

    def test_calibrated_acceleration_produces_no_residual(self):
        monitor = ShortWindowContactDiagnostics(
            DETECTOR, mass=.17, impulse_config=dict(
                window_s=.08, minimum_window_s=.05,
                model_acceleration_scale=[.8, .8, .8]))
        for i in range(20):
            row = self.sample(monitor, i*.01, [i*.008, 0, 0],
                              expected_acceleration=[1, 0, 0])
        np.testing.assert_allclose(row['force_estimate_N'], [0, 0, 0], atol=1e-12)

    def test_isolated_velocity_step_does_not_trigger_either_axis(self):
        # A repeated overlapping residual is not independent contact evidence.
        for dt in (.01, .0113, .015):
            for axis in (0, 1):
                monitor = self.make()
                starts = []
                for i in range(100):
                    velocity = np.zeros(3)
                    if i >= 50:
                        velocity[axis] = .2
                    row = self.sample(monitor, i * dt, velocity)
                    starts.append((row.get('corrected') or {}).get('started'))
                self.assertFalse(any(starts), (dt, axis))

    def test_sustained_force_starts_and_releases_on_both_axes(self):
        for axis in (0, 1):
            monitor = self.make()
            starts, ends = [], []
            velocity = np.zeros(3)
            for i in range(130):
                if 60 <= i < 90:
                    velocity[axis] += .01 * .3 / .17
                row = self.sample(monitor, i * .01, velocity)
                decision = row.get('corrected') or {}
                if decision.get('started'):
                    starts.append(i)
                if decision.get('ended'):
                    ends.append(i)
            self.assertEqual(len(starts), 1)
            self.assertLessEqual(starts[0], 69)
            self.assertEqual(len(ends), 1)

    def test_gap_duplicate_invalid_and_nan_discard_force_history(self):
        for time, extra in [(2., {}), (.59, {}),
                            (.60, {'state_valid': False}),
                            (.60, {'expected_acceleration': [float('nan'), 0, 0]})]:
            monitor = self.make()
            for i in range(60):
                self.sample(monitor, i * .01)
            row = self.sample(monitor, time, [.2, 0, 0], **extra)
            self.assertFalse(row['valid'])
            self.assertFalse(row['command_authority'])
            self.assertFalse(monitor.short.baseline.ready)
            self.assertFalse(monitor.short.corrected.active)

    def test_rearm_resets_force_window_and_dwell(self):
        monitor = self.make()
        for i in range(80):
            self.sample(monitor, i*.01, [max(0, i-60)*.02, 0, 0])
        self.assertTrue(monitor.short.corrected.active)
        monitor.reset()
        row = self.sample(monitor, .81, [.4, 0, 0])
        self.assertFalse(row['valid'])
        self.assertFalse(monitor.short.corrected.active)

    def test_invalid_windows_rejected(self):
        for kwargs in [dict(window_s=True), dict(window_s=float('nan')),
                       dict(window_s=.09), dict(minimum_window_s=0),
                       dict(minimum_window_s=.04)]:
            with self.assertRaises(ValueError):
                self.make(**kwargs)

    def test_diagnostic_failure_is_isolated(self):
        monitor = self.make()
        with patch.object(monitor.estimator, 'update', side_effect=RuntimeError('test')):
            row = self.sample(monitor, 0)
        self.assertEqual(row['reason'], 'diagnostic_error')
        self.assertFalse(row['command_authority'])
        self.assertFalse(row['valid'])

    def test_optional_guard_defaults_to_original_fast_detection(self):
        original = ContactChannelDetector(**DETECTOR)
        guarded = ContactChannelDetector(**DETECTOR, minimum_onset_duration_s=.03)
        for detector in (original, guarded):
            detector.update([0, 0, 0], np.eye(3)*.0004, 0)
        fast = original.update([1, 0, 0], np.eye(3)*.0004, .01)
        slow = guarded.update([1, 0, 0], np.eye(3)*.0004, .01)
        self.assertTrue(fast.started)
        self.assertFalse(slow.started)
        guarded.update([1, 0, 0], np.eye(3)*.0004, .02)
        guarded.update([0, 0, 0], np.eye(3)*.0004, .03)
        self.assertFalse(guarded.update([1, 0, 0], np.eye(3)*.0004, .04).started)
        self.assertTrue(guarded.update([1, 0, 0], np.eye(3)*.0004, .07).started)

    def test_invalid_duration_rejected(self):
        for value in (-1, float('nan'), float('inf'), True):
            with self.assertRaises(ValueError):
                ContactChannelDetector(**DETECTOR, minimum_onset_duration_s=value)


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
