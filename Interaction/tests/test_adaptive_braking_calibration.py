"""Calibration adapter tests: synthetic state/fake controller, never device I/O."""
import copy
import math
import unittest
from unittest.mock import patch

import numpy as np

from Interaction.adaptive_braking_calibration import AdaptiveBrakingCalibration
from Interaction.braking_response_calibration import PlanarBrakingCalibration
from Interaction.tests.test_model_based_braking import model, state


def plan():
    return PlanarBrakingCalibration(dict(enabled=True, tilt_levels_deg=[20.],
        accelerate_durations_s=[.16, .24, .32, .32], start_delay_s=0.,
        max_xy_speed_m_s=1.6, max_displacement_m=1.))


def report(version=1, validation_passed=True):
    candidate_model = model()
    ids = list(range(version*2))
    candidate_model['train_segment_ids'] = ids
    candidate_model['directional_models'] = {
        'positive_y': dict(terminal_velocity_error_margin_m_s=.01),
        'negative_y': dict(terminal_velocity_error_margin_m_s=.01),
    }
    return dict(validated_control_candidate=dict(
        version=version, model=candidate_model, training_segment_ids=ids,
        validation_segment_ids=[version*2, version*2+1],
        independent_validation=True, validation_passed=validation_passed,
        control_eligible=validation_passed,
        control_eligibility_reason=(
            'own_held_out_validation_passed' if validation_passed
            else 'own_held_out_validation_failed'),
    ))


class FakePredictor:
    def __init__(self, model, *, target_position_xy, direction_xy, brake_deadline_s, config):
        self.model = copy.deepcopy(model)
        self.target_position_xy = np.asarray(target_position_xy).copy()
        self.direction_xy = np.asarray(direction_xy).copy()
        self.brake_deadline_s = brake_deadline_s
        self.config = config
        self.history = []
        self.decisions = []
        self.observations = []
        self.action = 'level'

    def record_command(self, stamp, tilt):
        self.history.append((stamp, tilt))

    def decide(self, now, state):
        self.decisions.append((now, copy.deepcopy(state)))
        return dict(action=self.action, reason='fake_'+self.action,
                    level_latched=self.action != 'brake')

    def observe_state(self, now, state):
        self.observations.append((now, copy.deepcopy(state)))
        return dict(ready=len(self.observations) >= 4, status='fake')


class AdaptiveBrakingCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.plan = plan()
        self.events = []

    def adapter(self, config=None):
        settings = dict(enabled=True, model_based_braking=dict(max_compute_s=.008))
        settings.update(config or {})
        return AdaptiveBrakingCalibration(settings, self.plan, 100.,
                                           lambda name, data: self.events.append((name, data)))

    def command(self, segment, phase, offset=.001):
        start = self.plan.trial_start_s[segment]
        times = dict(level_before_acceleration=0.,
            accelerate=self.plan.level_before_acceleration_s,
            level_before_brake=self.plan.level_before_acceleration_s+self.plan.trial_accelerate_s[segment],
            brake=self.plan.level_before_acceleration_s+self.plan.trial_accelerate_s[segment]+self.plan.level_before_brake_s,
            level_after_brake=self.plan.trial_attitude_durations_s[segment]-self.plan.level_after_brake_s,
            recovery=self.plan.trial_attitude_durations_s[segment])
        return self.plan.command(start+times[phase]+offset, 0.)

    def prepare_brake(self, adapter, segment=4, source=None):
        source = report() if source is None else source
        for t, phase in [(10., 'level_before_acceleration'), (10.2, 'accelerate'),
                         (10.44, 'level_before_brake')]:
            proposed = self.command(segment, phase)
            actual = adapter.modify(proposed, t, state(t), source)
            adapter.record_sent(actual, t+.001)
        for t in [10.64, 10.65]:
            actual = adapter.modify(self.command(segment, 'brake'), t, state(t-.001), source)
            self.assertEqual(actual.phase, 'brake')
            adapter.record_sent(actual, t+.001)
        return source

    def test_default_disabled_is_identity_without_worker_or_model(self):
        adapter = AdaptiveBrakingCalibration({}, self.plan, None, None)
        command = self.command(2, 'brake')
        self.assertIs(adapter.modify(command, None, None, None), command)
        adapter.record_sent(command, None)

    def test_preflight_rejects_invalid_configs_and_worker(self):
        for config in [dict(enabled='true'), dict(target_distance_m=0),
                       dict(target_distance_m=float('nan')), dict(target_distance_m=True),
                       dict(target_distance_m=1.1), dict(model_based_braking=[]),
                       dict(model_based_braking=dict(max_xy_speed_m_s=2.)),
                       dict(model_based_braking=dict(
                           terminal_velocity_tolerance_m_s=.11)),
                       dict(model_based_braking=dict(
                           terminal_tilt_tolerance_deg=5.1)),
                       dict(model_based_braking=dict(
                           candidate_refinement_step_s=.1)),
                       dict(model_based_braking=dict(
                           candidate_refinement_step_s=.005)),
                       dict(model_based_braking=dict(prediction_step_s=.1))]:
            with self.subTest(config=config), self.assertRaises((ValueError, TypeError)):
                self.adapter(config)
        for config in [None, {}, dict(enabled=False), dict(enabled=1), dict(enabled='true')]:
            with self.assertRaises(ValueError):
                AdaptiveBrakingCalibration.validate_online_worker_config(config)
        AdaptiveBrakingCalibration.validate_online_worker_config(dict(enabled=True))

    def test_preflight_requires_paired_y_and_enough_phase_time(self):
        for updates in [dict(trial_directions=np.array([[1., 0.], [-1., 0.]]*3)),
                        dict(trial_directions=np.array([[0., 1.]]*6)),
                        dict(trial_brake_s=np.array([.01]*6)), dict(enabled=False)]:
            original = self.plan
            self.plan = copy.deepcopy(original)
            for key, value in updates.items():
                setattr(self.plan, key, value)
            with self.subTest(updates=updates), self.assertRaises(ValueError):
                self.adapter()
            self.plan = original

    def test_initial_pair_is_always_baseline(self):
        adapter = self.adapter()
        for segment in [0, 1]:
            for index, phase in enumerate(['level_before_acceleration', 'accelerate',
                                          'level_before_brake', 'brake', 'level_after_brake', 'recovery']):
                command = self.command(segment, phase)
                self.assertIs(adapter.modify(command, segment*10+index, {}, report()), command)
        self.assertEqual(len(self.events), 1)
        self.assertFalse(self.events[0][1]['adaptive'])

    def test_missing_candidate_locks_whole_pair_to_baseline(self):
        adapter = self.adapter()
        for segment, source in [(2, None), (3, report())]:
            command = self.command(segment, 'level_before_acceleration')
            self.assertIs(adapter.modify(command, segment, {}, source), command)
        self.assertIsNone(adapter._pair_models[1])
        self.assertEqual(
            self.events[0][1]['reason'],
            'validated_control_candidate_unavailable_at_pair_start',
        )

    def test_failed_prior_validation_candidate_locks_pair_to_baseline(self):
        adapter = self.adapter()
        source = report(validation_passed=False)
        command = self.command(4, 'level_before_acceleration')
        self.assertIs(adapter.modify(command, 1., {}, source), command)
        self.assertIsNone(adapter._pair_models[2])
        self.assertEqual(
            self.events[-1][1]['reason'],
            'own_held_out_validation_failed',
        )

    def test_direction_margin_only_disables_that_direction(self):
        adapter = self.adapter()
        source = report()
        source['validated_control_candidate']['model']['directional_models'] = {
            'positive_y': dict(terminal_velocity_error_margin_m_s=.05),
            'negative_y': dict(terminal_velocity_error_margin_m_s=.01),
        }
        command = self.command(4, 'level_before_acceleration')
        self.assertIs(adapter.modify(command, 1., {}, source), command)
        self.assertIsNotNone(adapter._pair_models[2])
        self.assertEqual(
            adapter._pair_models[2]['direction_control_eligible'],
            {'positive_y': False, 'negative_y': True},
        )
        self.assertEqual(
            self.events[-1][1]['reason'],
            'validated_candidate_frozen',
        )
        self.assertEqual(
            self.events[-1][1]['adaptive_directions'],
            {'positive_y': False, 'negative_y': True},
        )

    @patch('Interaction.adaptive_braking_calibration.ModelBasedBrakingController', FakePredictor)
    def test_direction_gate_keeps_positive_fixed_but_adapts_negative(self):
        adapter = self.adapter()
        source = report()
        source['validated_control_candidate']['model']['directional_models'] = {
            'positive_y': dict(terminal_velocity_error_margin_m_s=.05),
            'negative_y': dict(terminal_velocity_error_margin_m_s=.01),
        }
        positive = self.command(4, 'level_before_acceleration')
        adapter.modify(positive, 1., state(1.), source)
        positive_brake = self.command(4, 'brake')
        self.assertIs(
            adapter.modify(positive_brake, 2., state(1.999), source),
            positive_brake,
        )
        self.assertNotIn(4, adapter._episodes)

        self.prepare_brake(adapter, segment=5, source=source)
        self.assertIn(5, adapter._episodes)

    def test_candidate_cannot_train_on_current_or_future_pair(self):
        adapter = self.adapter()
        command = self.command(4, 'level_before_acceleration')
        adapter.modify(command, 1., {}, report(2))
        self.assertIsNone(adapter._pair_models[2])
        self.assertEqual(self.events[0][1]['reason'], 'candidate_provenance_invalid_or_not_causal')

    def test_stale_candidate_cannot_skip_a_failed_intermediate_fit(self):
        adapter = self.adapter()
        command = self.command(6, 'level_before_acceleration')
        adapter.modify(command, 1., {}, report(1))
        self.assertIsNone(adapter._pair_models[3])
        self.assertEqual(
            self.events[0][1]['reason'],
            'candidate_provenance_invalid_or_not_causal',
        )

    @patch('Interaction.adaptive_braking_calibration.ModelBasedBrakingController', FakePredictor)
    def test_freezes_pair_target_and_shortens_once_without_rebraking(self):
        adapter = self.adapter()
        source = self.prepare_brake(adapter)
        frozen_delay = source['validated_control_candidate']['model']['attitude_fit']['delay_s']
        source['validated_control_candidate']['model']['attitude_fit']['delay_s'] = .15
        actual = adapter.modify(self.command(4, 'brake'), 10.66,
                                state(10.659, p=.1), report(2))
        self.assertEqual(actual.phase, 'level_after_brake')
        self.assertEqual(actual.roll_deg, 0.)
        np.testing.assert_array_equal(actual.command_acceleration_xy, [0., 0.])
        adapter.record_sent(actual, 10.661)
        episode = adapter._episodes[4]
        self.assertAlmostEqual(episode.brake_deadline_s, 10.96)
        np.testing.assert_allclose(episode.target_position_xy, [0., .30])
        self.assertEqual(episode.model['attitude_fit']['delay_s'], frozen_delay)
        self.assertEqual(len(episode.decisions), 1)
        episode.action = 'brake'
        actual = adapter.modify(self.command(4, 'brake'), 10.67, state(10.669, p=.2), report(2))
        self.assertEqual(actual.phase, 'level_after_brake')
        self.assertEqual(len(episode.decisions), 1)
        recovery = self.command(4, 'recovery')
        self.assertIs(adapter.modify(recovery, 11.5, state(11.499), report(2)), recovery)
        # Second trial keeps the same snapshot despite an incoming replacement.
        second = self.command(5, 'level_before_acceleration')
        adapter.modify(second, 12., state(12.), report(2))
        self.assertEqual(adapter._pair_models[2]['version'], 1)

    @patch('Interaction.adaptive_braking_calibration.ModelBasedBrakingController', FakePredictor)
    def test_actual_history_contains_prior_levels_accel_and_sent_brake(self):
        adapter = self.adapter()
        self.prepare_brake(adapter)
        episode = adapter._episodes[4]
        np.testing.assert_allclose([x[1] for x in episode.history],
                                   [0., math.radians(20.), 0., -math.radians(20.), -math.radians(20.)])
        self.assertEqual(len(episode.decisions), 0)
        proposed = self.command(4, 'brake')
        actual = adapter.modify(proposed, 10.66, state(10.659), report())
        self.assertEqual(actual.phase, 'level_after_brake')
        # modify is not a successful send; history has no invented level yet.
        self.assertNotEqual(episode.history[-1][1], 0.)
        adapter.record_sent(actual, 10.661)
        self.assertEqual(episode.history[-1][1], 0.)

    @patch('Interaction.adaptive_braking_calibration.ModelBasedBrakingController', FakePredictor)
    def test_fallback_always_latches_level(self):
        adapter = self.adapter()
        self.prepare_brake(adapter)
        adapter._episodes[4].action = 'fallback'
        result = adapter.modify(self.command(4, 'brake'), 10.66, state(10.659), report())
        self.assertEqual(result.phase, 'level_after_brake')
        self.assertIn(4, adapter._latched)

    @patch('Interaction.adaptive_braking_calibration.ModelBasedBrakingController', FakePredictor)
    def test_brake_decision_continues_only_original_command(self):
        adapter = self.adapter()
        self.prepare_brake(adapter)
        adapter._episodes[4].action = 'brake'
        proposed = self.command(4, 'brake')
        result = adapter.modify(proposed, 10.66, state(10.659), report())
        self.assertIs(result, proposed)
        self.assertEqual(result.phase, 'brake')
        self.assertNotIn(4, adapter._latched)

    @patch('Interaction.adaptive_braking_calibration.ModelBasedBrakingController', FakePredictor)
    def test_residual_observer_gets_causal_warmup_before_first_decision(self):
        adapter = self.adapter(dict(model_based_braking=dict(
            max_compute_s=.008,
            motion_residual_observer_enabled=True,
        )))
        self.prepare_brake(adapter)
        episode = adapter._episodes[4]
        for now in (10.66, 10.68):
            proposed = self.command(4, 'brake')
            actual = adapter.modify(proposed, now, state(now-.001), report())
            self.assertIs(actual, proposed)
            adapter.record_sent(actual, now+.001)
            self.assertEqual(episode.decisions, [])
        result = adapter.modify(
            self.command(4, 'brake'), 10.70, state(10.699), report()
        )
        self.assertEqual(result.phase, 'level_after_brake')
        self.assertEqual(len(episode.decisions), 1)
        self.assertGreaterEqual(len(episode.observations), 5)
        self.assertTrue(all(
            observation[1]['time_s'] <= observation[0]
            for observation in episode.observations
        ))

    @patch('Interaction.adaptive_braking_calibration.ModelBasedBrakingController', FakePredictor)
    def test_missing_measurements_do_not_hold_brake_past_warmup(self):
        adapter = self.adapter()
        self.prepare_brake(adapter)
        result = adapter.modify(self.command(4, 'brake'), 10.66, {}, report())
        self.assertEqual(result.phase, 'level_after_brake')
        self.assertEqual(self.events[-1][1]['reason'], 'insufficient_actual_brake_observations')

    @patch('Interaction.adaptive_braking_calibration.ModelBasedBrakingController', FakePredictor)
    def test_hard_deadline_is_enforced_even_if_predictor_wants_more_brake(self):
        adapter = self.adapter()
        self.prepare_brake(adapter)
        episode = adapter._episodes[4]
        episode.action = 'brake'
        result = adapter.modify(self.command(4, 'brake'), 10.961, state(10.96), report())
        self.assertEqual(result.phase, 'level_after_brake')
        self.assertEqual(self.events[-1][1]['reason'], 'original_brake_deadline')
        self.assertEqual(episode.decisions, [])

    @patch('Interaction.adaptive_braking_calibration.ModelBasedBrakingController', FakePredictor)
    def test_next_pair_refreezes_a_new_model_without_changing_previous_snapshot(self):
        adapter = self.adapter()
        adapter.modify(self.command(4, 'level_before_acceleration'), 1., {}, report())
        adapter.modify(self.command(6, 'level_before_acceleration'), 2., {}, report(2))
        self.assertEqual(adapter._pair_models[2]['version'], 1)
        self.assertEqual(adapter._pair_models[3]['version'], 2)

    def test_real_predictor_and_negative_y_target_are_supported(self):
        adapter = self.adapter(dict(model_based_braking=dict(max_compute_s=.1)))
        # Observe pair start even if this test skips its first trial's data.
        adapter.modify(self.command(4, 'level_before_acceleration'), 1., state(1.), report())
        self.prepare_brake(adapter, segment=5)
        result = adapter.modify(self.command(5, 'brake'), 10.66,
                                state(10.659, direction=-1), report())
        np.testing.assert_allclose(adapter._episodes[5].target_position_xy, [0., -.30])
        self.assertIn(result.phase, ('brake', 'level_after_brake'))
        self.assertNotEqual(self.events[-1][1]['reason'], 'insufficient_effective_command_history')

    def test_sent_command_time_must_increase(self):
        adapter = self.adapter()
        command = self.command(2, 'accelerate')
        adapter.record_sent(command, 1.)
        with self.assertRaises(ValueError):
            adapter.record_sent(command, 1.)


if __name__ == '__main__':
    unittest.main()
