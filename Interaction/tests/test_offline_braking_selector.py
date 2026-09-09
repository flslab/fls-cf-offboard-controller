import dataclasses
import unittest

import numpy as np

from Interaction.offline_braking_selector import (
    BrakingSnapshot, FrozenTiltModel, evaluate_braking_candidates,
)


class OfflineBrakingSelectorTests(unittest.TestCase):
    def setUp(self):
        self.model = FrozenTiltModel(.025, 14.5, .98, 1.035, 1.09)
        self.state = BrakingSnapshot(10.0, 2.0, .30, 0.0, 0.0)

    def evaluate(self, state=None, **kwargs):
        return evaluate_braking_candidates(state or self.state, self.model,
            target_remaining_m=kwargs.pop('target_remaining_m', 1.0), **kwargs)

    def test_deterministic_offline_only_and_caps_pulse(self):
        first = self.evaluate(brake_tilt_deg=40)
        self.assertEqual(first, self.evaluate(brake_tilt_deg=40))
        self.assertTrue(first['offline_only'])
        self.assertFalse(first['flight_command_generated'])
        self.assertEqual(len(first['candidates']), 4)
        self.assertIsNotNone(first['selected'])
        self.assertEqual(first['candidates'][-1]['hypothetical_pulse_tilt_deg'], -20)

    def test_pending_past_commands_preserved(self):
        without = self.evaluate()['candidates'][0]
        with_queue = self.evaluate(command_history=[(9, 0), (9.99, -.35)])['candidates'][0]
        self.assertLess(with_queue['terminal_velocity_m_s'], without['terminal_velocity_m_s']-.02)
        times = np.array(with_queue['time_s'])
        np.testing.assert_allclose(np.array(with_queue['tilt_rad'])[times < .015], 0, atol=1e-12)

    def test_rate_is_measured_initial_state(self):
        with_rate = self.evaluate(dataclasses.replace(self.state, tilt_rate_rad_s=-1))['candidates'][0]
        without = self.evaluate()['candidates'][0]
        self.assertLess(with_rate['terminal_velocity_m_s'], without['terminal_velocity_m_s']-.04)

    def test_long_tail_catches_reverse_not_visible_at_100ms(self):
        result = self.evaluate(dataclasses.replace(self.state, velocity_m_s=.30, tilt_rad=-.28))
        level = result['candidates'][0]
        early = np.array(level['velocity_m_s'])[np.array(level['time_s']) <= .1]
        self.assertGreater(np.min(early), 0)
        self.assertTrue(level['predicted_reverse'])
        self.assertIsNone(result['selected'])
        self.assertEqual(result['reason'], 'level_now_already_predicts_reverse')

    def test_infeasible_level_does_not_add_forward_cancellation(self):
        result = self.evaluate(dataclasses.replace(self.state, velocity_m_s=.05, tilt_rad=-.3))
        self.assertIsNone(result['selected'])
        self.assertTrue(all(row['hypothetical_pulse_tilt_deg'] <= 0 for row in result['candidates']))

    def test_target_overshoot_checked_through_tail(self):
        result = self.evaluate(target_remaining_m=.005)
        self.assertIsNone(result['selected'])
        self.assertTrue(all(not row['feasible'] for row in result['candidates']))
        self.assertIn('predicted_target_overshoot', result['candidates'][0]['rejection_reasons'])

    def test_negative_or_nonfinite_state_rejected(self):
        for change in [dict(velocity_m_s=-.01), dict(velocity_m_s=float('nan')),
                       dict(time_s=float('inf')), dict(tilt_rate_rad_s=float('nan'))]:
            result = self.evaluate(dataclasses.replace(self.state, **change))
            self.assertIsNone(result['selected'])
            self.assertEqual(result['reason'], 'invalid_input')
            self.assertEqual(result['candidates'], [])

    def test_future_unordered_or_nonfinite_history_rejected(self):
        for history in [[(10.01, 0)], [(9.9, 0), (9.8, 0)], [(9.9, float('nan'))]]:
            self.assertEqual(self.evaluate(command_history=history)['reason'], 'invalid_input')

    def test_short_horizon_missing_level_or_behind_target_rejected(self):
        for kwargs in [dict(forecast_s=.1), dict(target_remaining_m=-.01),
                       dict(pulse_durations_s=[.02]), dict(integration_step_s=.02)]:
            self.assertEqual(self.evaluate(**kwargs)['reason'], 'invalid_input')

    def test_integration_converges_and_preserves_short_pulse(self):
        coarse = self.evaluate(integration_step_s=.002)['candidates'][1]
        fine = self.evaluate(integration_step_s=.0005)['candidates'][1]
        self.assertAlmostEqual(coarse['terminal_velocity_m_s'], fine['terminal_velocity_m_s'], delta=1e-5)
        self.assertAlmostEqual(coarse['end_displacement_m'], fine['end_displacement_m'], delta=1e-5)
        self.assertLess(coarse['terminal_velocity_m_s'], .30)


if __name__ == '__main__':
    unittest.main()
