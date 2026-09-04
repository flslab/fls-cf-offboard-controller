import dataclasses
import unittest

import numpy as np

from Interaction.predictive_brake_release import (
    BrakeReleaseConfig, BrakingSnapshot, FrozenTiltModel, PredictiveBrakeRelease,
)


class BrakeReleaseConfigTests(unittest.TestCase):
    def test_validate_accepts_defaults_and_valid_domain_boundaries(self):
        configs = [BrakeReleaseConfig(),
                   BrakeReleaseConfig(control_period_s=.1, integration_step_s=.01,
                                      release_speed_margin_m_s=0, max_state_age_s=0),
                   BrakeReleaseConfig(control_period_s=.001, integration_step_s=.001,
                                      prediction_horizon_s=1.2, brake_tilt_deg=1,
                                      max_brake_duration_s=.01)]
        for config in configs:
            with self.subTest(config=config):
                self.assertIsNone(config.validate())

    def test_validate_rejects_nonfinite_fields(self):
        for field in dataclasses.fields(BrakeReleaseConfig):
            for value in (np.nan, np.inf, -np.inf):
                with self.subTest(field=field.name, value=value):
                    config = dataclasses.replace(BrakeReleaseConfig(), **{field.name: value})
                    with self.assertRaises(ValueError):
                        config.validate()

    def test_validate_rejects_nonadvancing_and_out_of_domain_values(self):
        changes = [dict(control_period_s=0), dict(control_period_s=-.01),
                   dict(control_period_s=.101), dict(prediction_horizon_s=.799),
                   dict(integration_step_s=0), dict(integration_step_s=-.001),
                   dict(integration_step_s=.011),
                   dict(control_period_s=.001, integration_step_s=.002),
                   dict(release_speed_margin_m_s=-.001), dict(brake_tilt_deg=0),
                   dict(brake_tilt_deg=-1), dict(brake_tilt_deg=20.01),
                   dict(max_brake_duration_s=0), dict(max_brake_duration_s=-.1),
                   dict(max_state_age_s=-.001), dict(control_period_s=None),
                   dict(control_period_s='invalid')]
        for change in changes:
            with self.subTest(change=change):
                with self.assertRaises(ValueError):
                    dataclasses.replace(BrakeReleaseConfig(), **change).validate()


class PredictiveBrakeReleaseTests(unittest.TestCase):
    def setUp(self):
        self.model = FrozenTiltModel(.025, 14.5, .98, 1.035, 1.09)
        self.snapshot = BrakingSnapshot(10.0, 2.0, .3, 0.0, 0.0)
        self.history = [(9.0, 0.0), (9.8, -np.radians(20))]

    def decide(self, snapshot=None, history=None, config=None, **kwargs):
        controller = PredictiveBrakeRelease(self.model, config)
        return controller.update(snapshot or self.snapshot,
                                 self.history if history is None else history, **kwargs)

    def test_only_simulated_braking_or_level_and_explicit_exploratory_margin(self):
        decision = self.decide()
        self.assertEqual(decision['reason'], 'continue_braking_one_tick')
        self.assertAlmostEqual(decision['command_tilt_rad'], -np.radians(20))
        self.assertFalse(decision['released'])
        self.assertTrue(decision['offline_only'])
        self.assertFalse(decision['flight_command_generated'])
        self.assertFalse(decision['physical_non_reversal_guaranteed'])
        self.assertFalse(decision['position_target_used'])
        self.assertEqual(decision['nominal_uncertainty_margin'], .08)
        self.assertFalse(decision['margin_is_calibrated_confidence_bound'])

    def test_release_at_positive_speed_with_measured_lagged_braking_tilt(self):
        state = dataclasses.replace(self.snapshot, velocity_m_s=.5,
                                    tilt_rad=-.28, tilt_rate_rad_s=-.4)
        decision = self.decide(state)
        self.assertGreater(state.velocity_m_s, 0)
        level = decision['level_now']
        early = np.array(level['velocity_m_s'])[np.array(level['time_s']) <= .1]
        self.assertGreater(np.min(early), 0)
        self.assertLess(level['min_velocity_m_s'], 0)
        self.assertEqual(decision['reason'], 'level_now_already_predicts_reverse')
        self.assertTrue(decision['released'])
        self.assertEqual(decision['command_tilt_rad'], 0)

    def test_release_before_next_tick_uses_up_positive_velocity_margin(self):
        decision = self.decide(dataclasses.replace(self.snapshot, velocity_m_s=.2))
        self.assertGreater(decision['level_now_min_velocity_m_s'], .08)
        self.assertLess(decision['continue_then_level_min_velocity_m_s'], .08)
        self.assertGreater(decision['continue_then_level_min_velocity_m_s'], 0)
        self.assertEqual(decision['reason'], 'next_tick_tail_below_margin')
        self.assertTrue(decision['released'])

    def test_minimum_includes_current_low_speed_even_if_later_acceleration(self):
        state = dataclasses.replace(self.snapshot, velocity_m_s=.02, tilt_rad=.15,
                                    tilt_rate_rad_s=.2)
        decision = self.decide(state, history=[])
        self.assertEqual(decision['continue_then_level_min_velocity_m_s'], .02)
        self.assertEqual(decision['reason'], 'next_tick_tail_below_margin')

    def test_angular_rate_and_delayed_pending_sent_history_change_prediction(self):
        no_queue = self.decide(history=[])
        pending = self.decide(history=[(9, 0), (9.99, -.35)])
        self.assertLess(pending['level_now_final_velocity_m_s'],
                        no_queue['level_now_final_velocity_m_s'] - .02)
        state = dataclasses.replace(self.snapshot, tilt_rate_rad_s=-1)
        rotating = self.decide(state, history=[])
        self.assertLess(rotating['level_now_final_velocity_m_s'],
                        no_queue['level_now_final_velocity_m_s'] - .04)
        times = np.array(pending['level_now']['time_s'])
        angles = np.array(pending['level_now']['tilt_rad'])
        np.testing.assert_allclose(angles[times < .015 - 1e-12], 0, atol=1e-12)

    def test_timestamped_old_state_propagates_before_decision(self):
        state = dataclasses.replace(self.snapshot, time_s=9.98, tilt_rad=-.1,
                                    tilt_rate_rad_s=-.2)
        decision = self.decide(state, decision_time_s=10.0)
        self.assertAlmostEqual(decision['state_age_s'], .02)
        self.assertLess(decision['predicted_decision_velocity_m_s'], state.velocity_m_s)
        self.assertNotEqual(decision['level_now']['predicted_decision_tilt_rad'], state.tilt_rad)
        self.assertEqual(decision['level_now']['time_s'][0], 0.0)
        self.assertAlmostEqual(decision['level_now']['time_s'][-1], .8)

    def test_sent_after_measurement_but_before_decision_is_known_history(self):
        state = dataclasses.replace(self.snapshot, time_s=9.98)
        decision = self.decide(state, history=[(9, 0), (9.99, -.35)], decision_time_s=10)
        self.assertEqual(decision['forecast_status'], 'complete')
        self.assertAlmostEqual(decision['predicted_decision_velocity_m_s'], .3)
        times = np.array(decision['level_now']['time_s'])
        angle = np.array(decision['level_now']['tilt_rad'])
        np.testing.assert_allclose(angle[times < .015 - 1e-12], 0, atol=1e-12)
        self.assertLess(decision['level_now_final_velocity_m_s'], .28)

    def test_no_future_unordered_or_malformed_sent_history(self):
        for history in [[(10.001, 0)], [(9.9, 0), (9.8, 0)], [(9.9, np.nan)],
                        [(9.9,)], [[]]]:
            with self.subTest(history=history):
                decision = self.decide(history=history)
                self.assertEqual(decision['reason'], 'invalid_input')
                self.assertEqual(decision['command_tilt_rad'], 0)
                self.assertTrue(decision['released'])
                self.assertIsNone(decision['level_now'])

    def test_stale_or_future_state_releases_without_prediction(self):
        for now, reason in [(10.031, 'stale_state'), (9.999, 'future_measurement')]:
            decision = self.decide(history=[], decision_time_s=now)
            self.assertEqual(decision['reason'], reason)
            self.assertTrue(decision['released'])
            self.assertEqual(decision['command_tilt_rad'], 0)
            self.assertIsNone(decision['level_now'])

    def test_invalid_gate_and_negative_measured_speed_release(self):
        decision = self.decide(state_valid=False)
        self.assertEqual(decision['reason'], 'invalid_state')
        negative = self.decide(dataclasses.replace(self.snapshot, velocity_m_s=-.001))
        self.assertEqual(negative['reason'], 'measured_reverse')
        self.assertEqual(negative['command_tilt_rad'], 0)
        self.assertIsNone(negative['level_now'])

    def test_timeout_uses_decision_clock_and_latches(self):
        controller = PredictiveBrakeRelease(self.model)
        fast = dataclasses.replace(self.snapshot, velocity_m_s=2)
        first = controller.update(fast, decision_time_s=10.02)
        self.assertFalse(first['released'])
        at_timeout = dataclasses.replace(fast, time_s=10.50)
        decision = controller.update(at_timeout, decision_time_s=10.52)
        self.assertEqual(decision['reason'], 'maximum_brake_duration')
        self.assertAlmostEqual(decision['elapsed_brake_s'], .5)
        self.assertEqual(decision['release_time_s'], 10.52)
        after = controller.update(dataclasses.replace(fast, time_s=11))
        self.assertTrue(after['released'])
        self.assertEqual(after['reason'], 'maximum_brake_duration')
        self.assertEqual(after['command_tilt_rad'], 0)

    def test_release_remains_latched_even_when_later_input_would_brake(self):
        controller = PredictiveBrakeRelease(self.model)
        first = controller.update(self.snapshot, state_valid=False)
        after = controller.update(dataclasses.replace(self.snapshot, time_s=10.01,
                                                     velocity_m_s=10))
        self.assertEqual(after['reason'], first['reason'])
        self.assertEqual(after['release_time_s'], first['release_time_s'])
        self.assertEqual(after['forecast_status'], 'skipped_latched_release')
        self.assertEqual(after['command_tilt_rad'], 0)

    def test_backwards_decision_time_invalid_and_latched(self):
        controller = PredictiveBrakeRelease(self.model)
        self.assertFalse(controller.update(self.snapshot)['released'])
        decision = controller.update(dataclasses.replace(self.snapshot, time_s=9.99))
        self.assertEqual(decision['reason'], 'invalid_input')
        self.assertIn('backwards', decision['detail'])
        self.assertTrue(decision['released'])

    def test_invalid_state_model_or_config_returns_simulated_level(self):
        bad_states = [dataclasses.replace(self.snapshot, tilt_rad=np.nan),
                      dataclasses.replace(self.snapshot, time_s=np.inf)]
        for state in bad_states:
            self.assertEqual(self.decide(state)['reason'], 'invalid_input')
        for change in [dict(control_period_s=0), dict(prediction_horizon_s=.1),
                       dict(integration_step_s=.02),
                       dict(brake_tilt_deg=21), dict(release_speed_margin_m_s=-.1),
                       dict(max_state_age_s=-.1), dict(control_period_s=np.nan)]:
            decision = self.decide(config=dataclasses.replace(BrakeReleaseConfig(), **change))
            self.assertEqual(decision['reason'], 'invalid_input')
            self.assertEqual(decision['command_tilt_rad'], 0)
        bad_model = dataclasses.replace(self.model, motion_gain=-1)
        decision = PredictiveBrakeRelease(bad_model).update(self.snapshot)
        self.assertEqual(decision['reason'], 'invalid_input')
        self.assertTrue(decision['released'])

    def test_longer_horizon_and_finer_grid_and_time_shift_consistent(self):
        original = self.decide()
        fine = self.decide(config=BrakeReleaseConfig(integration_step_s=.0005))
        for key in ('level_now_final_velocity_m_s', 'continue_then_level_final_velocity_m_s'):
            self.assertAlmostEqual(original[key], fine[key], delta=1e-5)
        shifted = self.decide(dataclasses.replace(self.snapshot, time_s=110),
                              history=[(t + 100, q) for t, q in self.history])
        self.assertAlmostEqual(original['level_now_final_velocity_m_s'],
                               shifted['level_now_final_velocity_m_s'], places=10)
        longer = self.decide(config=BrakeReleaseConfig(prediction_horizon_s=1.2))
        self.assertEqual(longer['level_now']['time_s'][-1], 1.2)

    def test_no_positive_action_over_broad_state_grid(self):
        for velocity in [-.1, 0, .1, .5, 1.0]:
            for tilt in [-.35, 0, .35]:
                for rate in [-1, 0, 1]:
                    decision = self.decide(dataclasses.replace(self.snapshot,
                        velocity_m_s=velocity, tilt_rad=tilt, tilt_rate_rad_s=rate))
                    self.assertLessEqual(decision['command_tilt_rad'], 0)
                    if decision['released']:
                        self.assertEqual(decision['command_tilt_rad'], 0)


if __name__ == '__main__':
    unittest.main()
