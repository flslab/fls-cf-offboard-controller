"""Offline replay contract: causal state use, delayed impulses, isolated folds."""
import copy
from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from Interaction.braking_replay import (
    PHASES, Trial, analyze, extract, first_crossing, holdout_fits, main, metrics,
    replay, step, summarize_observations,
)


def synthetic_trial(segment=0, duration=.16):
    times = np.arange(0., 1.501, .01)
    return Trial(segment, duration, np.array([0., 1.]), times,
                 np.zeros(len(times)), np.full(len(times), .5),
                 np.zeros(len(times)), np.array([0., .2, .4, .6, .8]),
                 np.array([0., 3., 0., -3., 0.]),
                 dict(zip(PHASES, [0., .2, .4, .6, .8, 1.51])), .01)


FIT = dict(command_delay_s=.055, command_time_constant_s=.1,
           horizontal_acceleration_scale=1.07, acceleration_bias_m_s2=.001,
           usable=True)


def synthetic_records(repetitions=1, duration=.16):
    timeline = []
    for segment, direction in enumerate(([0., 1.], [0., -1.]) * repetitions):
        start = 1000. + 3 * segment
        boundaries = [start, start+.2, start+.2+duration, start+.4+duration,
                      start+.4+2*duration, start+1.05+2*duration]
        for phase, time, sign in zip(PHASES, boundaries, [0, 1, 0, -1, 0, 0]):
            u = sign * 9.81 * np.tan(np.radians(20))
            timeline.append((time, 0, dict(type='events', name='Planar Braking Calibration Phase',
                data=dict(segment_id=segment, phase=phase, time=time,
                          direction_xy=direction,
                          command_acceleration_xy_m_s2=(u*np.array(direction)).tolist()))))
            timeline.append((time, 1, dict(type='commands',
                name='Commander.send_position_setpoint' if phase=='recovery' else 'Commander.send_zdistance_setpoint',
                data=dict(time=time-990., args=[-sign*20, 0, 0, 1]))))
        for time in np.arange(start-.02, boundaries[-1]+.03, .01):
            timeline.append((time, 2, dict(type='wrench_observer', data=dict(
                state_time=float(time), position_m=[0., 0., 1.], velocity_m_s=[0., .2, 0.],
                orientation_rpy_rad=[0., 0., 0.], state_group_skew_s=0.))))
    records = [r for _, _, r in sorted(timeline, key=lambda x: x[:2])]
    records.append(dict(type='events', name='Wrench Model Calibration Saved', data=dict(
        path='unchanged.json', planar_braking_fit=dict(FIT, maneuver_count=2*repetitions,
        protocol=dict(accelerate_s=duration, trial_accelerate_s=[duration] * (2*repetitions))))))
    return records


def synthetic_repeat_records():
    records = synthetic_records(repetitions=3, duration=.24)
    records[-1] = dict(type='events', name='Planar Braking Repeat Test Complete', data=dict(
        maneuver_count=6, sample_count=1000, previous_calibration_preserved=True,
        calibration_reference=dict(path='unchanged.json', sha256='a'*64),
        protocol=dict(trial_accelerate_s=[.24] * 6), offline_only=True))
    return records


class BrakingReplayTests(unittest.TestCase):
    def test_constant_acceleration_and_zero_tau_are_exact(self):
        a, v, p = step(2., 3., 4., 2., .1, .5)
        self.assertAlmostEqual(a, 2.)
        self.assertAlmostEqual(v, 4.)
        self.assertAlmostEqual(p, 5.75)
        self.assertEqual(step(100., 3., 4., 2., 0., .5), (2., 4., 5.75))

    def test_decay_preserves_residual_braking_impulse(self):
        a, v, p = step(-2., .5, 0., 0., .1, 1.)
        self.assertAlmostEqual(v, .5 - .2*(1-np.exp(-10)))
        self.assertLess(a, 0)
        self.assertGreater(p, 0)

    def test_prediction_does_not_use_future_positions_velocities_or_attitudes(self):
        trial = synthetic_trial()
        for initialization in ['command_history', 'measured_attitude']:
            anchor, original = replay(trial, FIT, initialization)
            altered = copy.deepcopy(trial)
            altered.positions[anchor+1:] = 900
            altered.velocities[anchor+1:] = -800
            altered.attitude_acceleration[anchor+1:] = 700
            _, predicted = replay(altered, FIT, initialization)
            np.testing.assert_array_equal(original, predicted)

    def test_pending_pre_anchor_command_survives_delay(self):
        trial = synthetic_trial()
        trial.command_times = np.array([0., .55, .60, .8])
        trial.commands = np.array([0., 3., 0., 0.])
        delayed = dict(FIT, command_delay_s=.1, command_time_constant_s=0.,
                       horizontal_acceleration_scale=1., acceleration_bias_m_s2=0.)
        _, prediction = replay(trial, delayed)
        self.assertAlmostEqual(prediction[-1, 0], .65, places=8)
        trial.commands[:] = 0
        _, prediction = replay(trial, delayed)
        self.assertAlmostEqual(prediction[-1, 0], .5)

    def test_crossing_interpolates_and_missing_crossing_is_not_fabricated(self):
        cross = first_crossing([0, 1], [.2, -.2], [0, .1])
        self.assertEqual(cross, dict(time_s=.5, position_m=.05))
        self.assertIsNone(first_crossing([0, 1], [.2, .1], [0, .1]))

    def test_extract_uses_command_clock_not_log_start_and_excludes_recovery(self):
        records = synthetic_records()
        records.insert(0, dict(type='start', data=1234567.))
        _, trials, samples, metadata = extract(records)
        self.assertEqual(len(trials), 2)
        self.assertEqual(metadata['command_clock_offset_s'], 990.)
        self.assertTrue(all(t.times[-1] < t.phase_times['recovery'] for t in trials))
        self.assertNotIn('recovery', {s['phase'] for s in samples})

    def test_incomplete_trial_is_rejected(self):
        records = [r for r in synthetic_records() if not (
            r.get('name') == 'Planar Braking Calibration Phase' and r['data']['phase']=='brake')]
        with self.assertRaisesRegex(ValueError, 'missing phases'):
            extract(records)

    def test_gap_and_skew_are_rejected(self):
        records = synthetic_records()
        bad = [r for r in records if not (r['type']=='wrench_observer'
               and 1000.8 < r['data']['state_time'] < 1001.1)]
        with self.assertRaisesRegex(ValueError, 'stale/gapped'):
            extract(bad)
        for r in records:
            if r['type']=='wrench_observer':
                r['data']['state_group_skew_s'] = .08
        with self.assertRaisesRegex(ValueError, 'skewed'):
            extract(records)

    def test_duplicate_poll_does_not_duplicate_fit_samples(self):
        records = synthetic_records()
        _, _, original, _ = extract(records)
        row = next(r for r in records if r['type']=='wrench_observer')
        records.append(copy.deepcopy(row))
        _, _, after, _ = extract(records)
        self.assertEqual(len(original), len(after))

    def test_duration_pairs_are_never_in_their_training_fold(self):
        trials = [synthetic_trial(i, [.16, .24, .32][i//2]) for i in range(6)]
        samples = [dict(segment_id=i) for i in range(6)]
        with patch('Interaction.braking_replay.identify_planar_braking_response', return_value=FIT) as identify:
            folds = holdout_fits(trials, samples, FIT)
        for fold, call in zip(folds, identify.call_args_list):
            self.assertEqual(len(fold['held_segments']), 2)
            self.assertEqual({s['segment_id'] for s in call.args[0]}, set(fold['train_segments']))
            self.assertFalse(set(fold['held_segments']) & set(fold['train_segments']))

    def test_metrics_include_velocity_position_consistency(self):
        row, _ = metrics(synthetic_trial(), FIT)
        self.assertGreater(row['measured_velocity_integral_minus_position_m'], 0)
        self.assertEqual(row['initial_speed_m_s'], .5)

    def test_repeat_extraction_does_not_invent_fitted_parameters_or_saved_path(self):
        completion, trials, _, metadata = extract(synthetic_repeat_records())
        self.assertEqual(len(trials), 6)
        self.assertTrue(all(t.duration == .24 for t in trials))
        self.assertFalse(completion['usable'])
        self.assertTrue(completion['offline_only'])
        self.assertNotIn('command_delay_s', completion)
        self.assertNotIn('calibration_saved_path', metadata)
        self.assertEqual(metadata['source_event'], 'Planar Braking Repeat Test Complete')
        self.assertFalse(metadata['contains_fitted_model'])
        self.assertEqual(metadata['calibration_reference']['sha256'], 'a'*64)

    def test_repeat_default_analysis_refuses_fitting_and_explains_summary_flag(self):
        with patch('Interaction.braking_replay.identify_planar_braking_response') as fit:
            with self.assertRaisesRegex(ValueError, '--summary-only'):
                analyze(synthetic_repeat_records())
            fit.assert_not_called()
        with self.assertRaisesRegex(ValueError, '--summary-only'):
            holdout_fits([synthetic_trial()], [], FIT)

    def test_ambiguous_or_invalid_repeat_completion_is_rejected(self):
        repeated = synthetic_repeat_records()
        with self.assertRaisesRegex(ValueError, 'exactly one'):
            extract(repeated + [synthetic_records()[-1]])
        repeated[-1]['data']['maneuver_count'] = float('nan')
        with self.assertRaisesRegex(ValueError, 'maneuver count'):
            extract(repeated)

    def test_repeat_summary_groups_opposed_directions_separately_without_fitting(self):
        with patch('Interaction.braking_replay.identify_planar_braking_response') as fit:
            report, traces, trials = summarize_observations(synthetic_repeat_records())
            fit.assert_not_called()
        self.assertEqual(len(report['metrics']), 6)
        self.assertEqual(len(report['repeat_groups']), 2)
        self.assertEqual([g['count'] for g in report['repeat_groups']], [3, 3])
        self.assertAlmostEqual(report['repeat_groups'][0]['actual_terminal_mean_m_s']['mean'], -.2)
        self.assertAlmostEqual(report['repeat_groups'][1]['actual_terminal_mean_m_s']['mean'], .2)
        self.assertFalse(report['metadata']['independent_model_validation'])
        self.assertNotIn('saved_fit', report)
        self.assertTrue(traces)
        self.assertTrue(all('predicted_velocity_m_s' not in r for r in traces))

    def test_rollback_is_decline_from_preceding_peak_before_recovery(self):
        trial = synthetic_trial()
        trial.positions = np.interp(trial.times, [0, .6, .9, 1.5], [0, 0, .2, .12])
        completion = dict(protocol={})
        with patch('Interaction.braking_replay.extract', return_value=(completion, [trial], [], {})):
            report, _, _ = summarize_observations([])
        row = report['metrics'][0]
        self.assertAlmostEqual(row['actual_max_rollback_m'], .08)
        self.assertAlmostEqual(row['actual_end_displacement_m'], .12)
        self.assertAlmostEqual(row['actual_terminal_rollback_m'], .08)
        self.assertIsNone(report['repeat_groups'][0]['actual_max_rollback_m']['sample_std'])

    def test_summary_cli_writes_measurements_only_and_preserves_reference(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            source = directory / 'flight.json'
            reference = directory / 'wrench_calibration.json'
            reference.write_text('{"preserved": true}')
            original = reference.read_bytes()
            records = synthetic_repeat_records()
            records[-1]['data']['calibration_reference']['path'] = str(reference)
            source.write_text(json.dumps(records))
            output = directory / 'observed'
            with redirect_stdout(io.StringIO()):
                main([str(source), '--output', str(output), '--summary-only'])
            report = json.loads((output / 'report.json').read_text())
            self.assertEqual(report['metadata']['analysis_mode'], 'observed_repeatability_only')
            self.assertTrue((output / 'metrics.csv').exists())
            self.assertFalse(list(output.glob('*.png')))
            self.assertEqual(reference.read_bytes(), original)
            refused = directory / 'default-refused'
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as caught:
                main([str(source), '--output', str(refused)])
            self.assertEqual(caught.exception.code, 2)
            self.assertFalse(refused.exists())


if __name__ == '__main__':
    unittest.main()
