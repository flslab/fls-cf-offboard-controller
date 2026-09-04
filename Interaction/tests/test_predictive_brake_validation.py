"""No-hardware checks that simulated validation is causal and honestly scored."""
from copy import deepcopy
from dataclasses import asdict, replace
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from Interaction.braking_split_diagnostic import TiltTrial
from Interaction.offline_braking_selector import BrakingSnapshot, FrozenTiltModel
from Interaction.predictive_brake_release import BrakeReleaseConfig
from Interaction.validate_predictive_brake_release import (
    StressCase, STRESS_CASES, aggregate, observed_seeds, simulate, summary, synthetic_seeds,
)


MODEL = FrozenTiltModel(.025, 14.56, .987, 1.035, 1.095, -.00148)


def seed():
    return dict(name='test:0', kind='observed_initial_state',
        snapshot=BrakingSnapshot(0., 0., .718, np.radians(5.75), np.radians(-56.6)),
        command_history=[(-.6, 0.), (-.4, np.radians(20)), (-.19, 0.)],
        first_brake_after_snapshot_s=.009, baseline_brake_duration_s=.24,
        model=MODEL, direction_xy=[0., 1.])


class PredictiveValidationTests(unittest.TestCase):
    def test_nominal_adaptive_is_early_nonreverse_but_not_stationary(self):
        fixed, _, _ = simulate(seed(), StressCase('nominal'), 'fixed_240ms')
        adaptive, trace, decisions = simulate(seed(), StressCase('nominal'), 'predictive_level')
        self.assertLess(adaptive['release_after_brake_s'], fixed['release_after_brake_s'])
        self.assertTrue(adaptive['no_reverse_in_simulation'])
        self.assertFalse(fixed['no_reverse_in_simulation'])
        self.assertTrue(adaptive['low_speed_level_capture_state_in_simulation'])
        self.assertFalse(adaptive['near_stationary_in_simulation'])
        self.assertGreater(adaptive['terminal_mean_velocity_m_s'], .04)
        self.assertGreaterEqual(adaptive['observation_after_level_s'], 1.5-1e-10)
        released = False
        for row in trace:
            self.assertLessEqual(row['simulated_command_tilt_rad'], 0.)
            if row['simulated_command_tilt_rad'] == 0:
                released = True
            if released:
                self.assertEqual(row['simulated_command_tilt_rad'], 0.)
        self.assertTrue(all(not row['flight_command_generated'] for row in decisions))

    def test_fractional_baseline_switch_is_not_rounded_up_to_tick(self):
        modified = seed()
        modified['baseline_brake_duration_s'] = .235
        row, _, _ = simulate(modified, StressCase('nominal'), 'fixed_240ms')
        self.assertAlmostEqual(row['release_after_brake_s'], .235, places=10)
        self.assertGreaterEqual(row['observation_after_level_s'], 1.5)

    def test_measured_delay_uses_history_time_and_common_warmup(self):
        case = StressCase('delayed', measurement_delay_s=.02)
        fixed, _, _ = simulate(seed(), case, 'fixed_240ms')
        adaptive, _, decisions = simulate(seed(), case, 'predictive_level')
        self.assertEqual(fixed['first_decision_time_s'], adaptive['first_decision_time_s'])
        self.assertAlmostEqual(adaptive['measurement_warmup_extension_s'], .011)
        valid = [r for r in decisions if r['forecast_status'] == 'complete']
        self.assertTrue(valid)
        for row in valid:
            self.assertGreaterEqual(row['state_age_s'], .02-1e-10)
            self.assertLessEqual(row['state_age_s'], .022)
            self.assertLessEqual(row['measurement_time_s'], row['simulation_time_s']-.02+1e-10)

    def test_simulation_does_not_mutate_seed_or_controller_model(self):
        original = seed()
        before = deepcopy(original)
        simulate(original, StressCase('changed', motion_gain_multiplier=1.2), 'predictive_level')
        self.assertEqual(original, before)
        self.assertEqual(asdict(original['model']), asdict(MODEL))

    def test_recorded_future_states_are_not_in_seed(self):
        times = np.linspace(0., 1., 101)
        trial = SimpleNamespace(times=times, velocities=np.full(101, .72), positions=times,
            command_times=np.array([-.1, .1, .44]), commands=np.array([0., -3.57, 0.]),
            phase_times={'brake': .1, 'level_after_brake': .44}, segment=0,
            direction=np.array([0., 1.]))
        item = TiltTrial(trial, np.zeros(101), np.zeros(101))
        frozen = dict(frozen_tilt_fit=dict(model='second_order', delay_s=.025,
            wn_rad_s=14.56, zeta=.987, gain=1.035, bias_world_y_rad=-.00148),
            frozen_motion_gain=1.095)
        with patch('Interaction.validate_predictive_brake_release.extract', return_value=({}, [], [], {})), patch(
                'Interaction.validate_predictive_brake_release.build_trials', return_value=[item]):
            before = observed_seeds([], 'source', frozen)
            item.trial.positions[11:] = 1000
            item.trial.velocities[11:] = -1000
            item.angle[11:] = 100
            item.rate[11:] = 100
            item.trial.commands[2] = 1000
            after = observed_seeds([], 'source', frozen)
        self.assertEqual(before, after)
        self.assertTrue(all(t <= 0 for t, _ in before[0]['command_history']))

    def test_positive_drift_not_counted_as_stopped(self):
        trace = [asdict(BrakingSnapshot(t, .1*t, .1, 0., 0.)) for t in np.linspace(0, 2, 201)]
        result = summary(trace, .2, 0., 'test', 'test')
        self.assertTrue(result['no_reverse_in_simulation'])
        self.assertTrue(result['low_speed_level_capture_state_in_simulation'])
        self.assertFalse(result['near_stationary_in_simulation'])
        self.assertFalse(result['position_target_validated'])

    def test_synthetic_and_observed_aggregates_are_separate(self):
        row, _, _ = simulate(seed(), StressCase('nominal'), 'fixed_240ms')
        adaptive, _, _ = simulate(seed(), StressCase('nominal'), 'predictive_level')
        rows = [row, adaptive, dict(row, seed_kind='synthetic_extrapolation'),
                dict(adaptive, seed_kind='synthetic_extrapolation')]
        groups = aggregate(rows)
        self.assertEqual(len(groups), 4)
        self.assertTrue(all(g['count'] == 1 for g in groups))

    def test_short_observation_is_rejected_and_stress_matrix_is_predeclared(self):
        with self.assertRaises(ValueError):
            simulate(seed(), StressCase('nominal'), 'predictive_level', tail_observation_s=.2)
        self.assertEqual(len(STRESS_CASES), 20)
        self.assertEqual(len({case.name for case in STRESS_CASES}), 20)
        self.assertEqual(BrakeReleaseConfig().release_speed_margin_m_s, .08)

    def test_synthetic_bias_is_projected_from_world_not_first_observed_direction(self):
        frozen = dict(frozen_tilt_fit=dict(model='second_order', delay_s=.025,
            wn_rad_s=14.56, zeta=.987, gain=1.035, bias_world_y_rad=-.00148),
            frozen_motion_gain=1.095)
        negative_y_seed = dict(seed(), model=replace(MODEL, projected_bias_rad=.00148),
                              direction_xy=[0., -1.])
        all_seeds = [negative_y_seed] + synthetic_seeds(frozen)
        self.assertEqual(len(all_seeds), 6)
        for item in all_seeds[1:]:
            self.assertEqual(item['direction_xy'], [0., 1.])
            self.assertAlmostEqual(item['model'].projected_bias_rad, -.00148)

    def test_invalid_config_rejected_before_simulation_loop(self):
        for change in (dict(control_period_s=0), dict(control_period_s=-.01),
                       dict(control_period_s=np.nan), dict(release_speed_margin_m_s=-.01),
                       dict(release_speed_margin_m_s=np.inf), dict(max_brake_duration_s=0)):
            with self.subTest(change=change), self.assertRaises(ValueError), patch(
                    'Interaction.validate_predictive_brake_release.DelayedTiltPlant') as plant:
                simulate(seed(), StressCase('nominal'), 'predictive_level',
                         replace(BrakeReleaseConfig(), **change))
            plant.assert_not_called()

    def test_invalid_observation_stress_and_seed_are_rejected(self):
        for duration in (np.nan, np.inf, -.1):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                simulate(seed(), StressCase('nominal'), 'predictive_level',
                         tail_observation_s=duration)
        for case in (StressCase('bad', measurement_delay_s=-.01),
                     StressCase('bad', velocity_bias_m_s=np.nan),
                     StressCase('bad', motion_gain_multiplier=0)):
            with self.subTest(case=case), self.assertRaises(ValueError):
                simulate(seed(), case, 'predictive_level')
        for change in (dict(first_brake_after_snapshot_s=-.1),
                       dict(baseline_brake_duration_s=np.inf),
                       dict(snapshot=replace(seed()['snapshot'], time_s=1))):
            with self.subTest(change=change), self.assertRaises(ValueError):
                simulate(dict(seed(), **change), StressCase('nominal'), 'fixed_240ms')


if __name__ == '__main__':
    unittest.main()
