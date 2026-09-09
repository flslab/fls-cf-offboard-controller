"""Causal, no-hardware checks for binary RL stopping and shared baseline scoring."""
from dataclasses import asdict, replace
import math
import unittest
from unittest.mock import patch

import numpy as np

try:
    import gymnasium
except ModuleNotFoundError:
    gymnasium = None

from Interaction.offline_braking_selector import BrakingSnapshot, FrozenTiltModel
from Interaction.validate_predictive_brake_release import StressCase, simulate, summary

if gymnasium is not None:
    from Interaction.rl_braking_env import BrakingScenario, RLBrakeConfig, RLBrakingEnv, sample_scenario, score_episode


MODEL = FrozenTiltModel(.025, 14.56, .987, 1.035, 1.095, -.00148)


def scenario(**changes):
    return replace(BrakingScenario('test_synthetic', MODEL,
        BrakingSnapshot(0., 0., .718, math.radians(5.75), math.radians(-56.6)),
        ((-.6, 0.), (-.4, math.radians(20)), (-.19, 0.)), .009), **changes)


@unittest.skipUnless(gymnasium is not None, 'optional offline RL dependencies are not installed')
class RLBrakingEnvTests(unittest.TestCase):
    def test_observation_schema_and_causal_delayed_measurements(self):
        case = scenario(measurement_delay_s=.02, velocity_bias_m_s=.01,
                        tilt_bias_deg=.3, rate_bias_deg_s=-2.)
        env = RLBrakingEnv(MODEL)
        obs, info = env.reset(seed=42, options={'scenario': case})
        self.assertEqual(obs.shape, (114,))
        self.assertEqual(obs.dtype, np.float32)
        self.assertTrue(env.observation_space.contains(obs))
        self.assertEqual(env.first_decision_s, .02)
        for _ in range(5):
            available = env.plant.snapshot_at_or_before(info['decision_time_s'] - .02)
            measured = info['snapshot']
            self.assertEqual(measured.time_s, available.time_s)
            self.assertLessEqual(measured.time_s, info['decision_time_s'] - .02)
            self.assertAlmostEqual(measured.velocity_m_s, available.velocity_m_s + .01)
            self.assertAlmostEqual(measured.tilt_rad, available.tilt_rad + math.radians(.3))
            self.assertAlmostEqual(measured.tilt_rate_rad_s, available.tilt_rate_rad_s - math.radians(2.))
            self.assertTrue(all(t <= info['decision_time_s'] for t, _ in info['command_history']))
            np.testing.assert_allclose(obs[:5], [measured.velocity_m_s,
                measured.tilt_rad / math.radians(20), measured.tilt_rate_rad_s / math.radians(200),
                measured.position_m, (info['decision_time_s']-measured.time_s)/.03], rtol=1e-6)
            before = info['decision_time_s']
            obs, reward, terminated, truncated, info = env.step(0)
            self.assertAlmostEqual(info['decision_time_s'] - before, .01)
            self.assertEqual((reward, terminated, truncated), (0., False, False))

    def test_seeded_episodes_are_deterministic(self):
        a, b = RLBrakingEnv(MODEL), RLBrakingEnv(MODEL)
        oa, ia = a.reset(seed=981)
        ob, ib = b.reset(seed=981)
        self.assertEqual(a.scenario, b.scenario)
        np.testing.assert_array_equal(oa, ob)
        self.assertEqual(ia, ib)
        for action in [0] * 8 + [1]:
            ra, rb = a.step(action), b.step(action)
            np.testing.assert_array_equal(ra[0], rb[0])
            self.assertEqual(ra[1:], rb[1:])
        # Reusing the same seed replays the sampler, not only its first state.
        np.testing.assert_array_equal(a.reset(seed=981)[0], oa)

    def test_hidden_parameters_are_not_observation_fields(self):
        initial = scenario(first_decision_s=0.)
        changed = replace(initial, name='different_hidden_model',
            model=replace(MODEL, motion_gain=1.3, wn_rad_s=20., delay_s=.06),
            transport_delay_s=.03)
        env_a, env_b = RLBrakingEnv(MODEL), RLBrakingEnv(MODEL)
        oa, ia = env_a.reset(options={'scenario': initial})
        ob, ib = env_b.reset(options={'scenario': changed})
        np.testing.assert_array_equal(oa, ob)
        self.assertEqual(ia, ib)
        self.assertFalse({'model', 'scenario', 'noise', 'true_state', 'future_state'} & set(ia))

    def test_history_contains_only_past_measured_frames_and_sent_commands(self):
        env = RLBrakingEnv(MODEL)
        obs, info = env.reset(options={'scenario': scenario(first_decision_s=0.)})
        first = obs[:5].copy()
        initial_frames = obs[5:77].reshape(12, 6)
        np.testing.assert_array_equal(initial_frames[:-1], np.zeros((11, 6)))
        np.testing.assert_array_equal(initial_frames[-1], np.r_[first, 1.])
        obs, _, _, _, info = env.step(0)
        frames = obs[5:77].reshape(12, 6)
        np.testing.assert_array_equal(frames[:-2], np.zeros((10, 6)))
        np.testing.assert_array_equal(frames[-2], np.r_[first, 1.])
        np.testing.assert_array_equal(frames[-1], np.r_[obs[:5], 1.])
        sent = obs[77:113].reshape(12, 3)
        self.assertAlmostEqual(sent[-1, 0], .01/.5, places=6)
        self.assertEqual(sent[-1, 1], -1.)
        self.assertEqual(sent[-1, 2], 1.)
        np.testing.assert_array_equal(sent[:-4], np.zeros((8, 3)))
        self.assertTrue(np.all(sent[:, 0] >= 0))
        self.assertTrue(all(t <= .01 for t, _ in info['command_history']))

    def test_fixed_240ms_matches_existing_independent_simulation(self):
        case = scenario()
        env = RLBrakingEnv(MODEL, scenarios=[case], randomize=False)
        _, info = env.reset(seed=1)
        for _ in range(24):
            _, reward, terminated, truncated, info = env.step(0)
            self.assertEqual((reward, terminated, truncated), (0., False, False))
        _, reward, terminated, truncated, info = env.step(1)
        old_seed = dict(name=case.name, kind='synthetic_extrapolation', model=MODEL,
                       snapshot=case.snapshot, command_history=case.command_history,
                       first_brake_after_snapshot_s=case.first_decision_s,
                       baseline_brake_duration_s=.24)
        baseline, _, _ = simulate(old_seed, StressCase('nominal'), 'fixed_240ms')
        for key in ('release_after_brake_s', 'observation_after_level_s', 'min_velocity_m_s',
                    'terminal_mean_velocity_m_s', 'max_rollback_m', 'final_displacement_m'):
            self.assertAlmostEqual(info['metrics'][key], baseline[key], places=10, msg=key)
        self.assertEqual((terminated, truncated), (True, False))
        self.assertEqual(reward, -score_episode(info['metrics'], env.config))

    def test_release_latches_and_observes_complete_tail_even_if_reversed(self):
        env = RLBrakingEnv(MODEL)
        env.reset(options={'scenario': scenario(snapshot=BrakingSnapshot(0., 0., -.1, 0., 0.))})
        _, reward, done, truncated, info = env.step(1)
        self.assertTrue(done)
        self.assertFalse(truncated)
        self.assertEqual(info['release_reason'], 'policy_level')
        self.assertAlmostEqual(env.plant.current_snapshot.time_s, env.first_decision_s + 1.5)
        self.assertAlmostEqual(info['metrics']['observation_after_level_s'], 1.5)
        self.assertLess(info['metrics']['min_velocity_m_s'], 0)
        self.assertEqual(info['episode_reward'], reward)
        self.assertTrue(all(a == 0 for t, a in env.plant.command_history if t >= env.first_decision_s))
        state_count = len(env.plant.snapshot_history)
        self.assertGreater(state_count, 1500)
        with self.assertRaises(RuntimeError):
            env.step(0)
        self.assertEqual(len(env.plant.snapshot_history), state_count)

    def test_maximum_brake_duration_forces_level_without_extra_brake_tick(self):
        config = replace(RLBrakeConfig(), max_brake_s=.03)
        env = RLBrakingEnv(MODEL, config)
        env.reset(options={'scenario': scenario(first_decision_s=0.)})
        for _ in range(3):
            self.assertFalse(env.step(0)[2])
        _, reward, done, truncated, info = env.step(0)
        self.assertEqual((done, truncated), (True, False))
        self.assertAlmostEqual(info['metrics']['release_after_brake_s'], .03)
        self.assertAlmostEqual(info['release_time_s'], .03)
        self.assertAlmostEqual(env.plant.current_snapshot.time_s, 1.53)
        self.assertEqual(info['release_reason'], 'maximum_brake_duration')
        self.assertEqual(env.plant.command_history[-1], (.03, 0.))
        self.assertLessEqual(reward, 0)

    def test_terminal_score_uses_every_integration_boundary(self):
        env = RLBrakingEnv(MODEL)
        env.reset(options={'scenario': scenario()})
        for _ in range(24):
            env.step(0)
        with patch('Interaction.rl_braking_env.summary', wraps=summary) as scoring:
            _, _, _, _, info = env.step(1)
        trace = scoring.call_args.args[0]
        self.assertEqual(trace, [asdict(s) for s in env.plant.snapshot_history])
        self.assertGreater(len(trace), 1700)
        self.assertFalse(info['metrics']['flight_validated'])
        self.assertFalse(info['metrics']['position_target_validated'])

    def test_score_has_exact_predeclared_terms(self):
        metrics = dict(terminal_mean_velocity_m_s=.1, min_velocity_m_s=-.2,
                       max_rollback_m=.04, release_after_brake_s=.3)
        self.assertAlmostEqual(score_episode(metrics, RLBrakeConfig()), .075 + .8 + .08 + .009)
        for key in metrics:
            for bad in (float('nan'), float('inf')):
                with self.subTest(key=key, bad=bad), self.assertRaises(ValueError):
                    score_episode(dict(metrics, **{key: bad}), RLBrakeConfig())
        for key in ('max_rollback_m', 'release_after_brake_s'):
            with self.assertRaises(ValueError):
                score_episode(dict(metrics, **{key: -.1}), RLBrakeConfig())

    def test_sampler_ranges_and_nominal_mode(self):
        rng = np.random.default_rng(87)
        for _ in range(100):
            sampled = sample_scenario(rng, MODEL)
            self.assertIn('synthetic', sampled.name)
            self.assertEqual(sampled.command_history, ((-1., 0.),))
            self.assertEqual(sampled.snapshot.time_s, 0)
            self.assertTrue(.2 <= sampled.snapshot.velocity_m_s <= 1)
            self.assertTrue(0 <= math.degrees(sampled.snapshot.tilt_rad) <= 8)
            self.assertTrue(-80 <= math.degrees(sampled.snapshot.tilt_rate_rad_s) <= 5)
            for key in ('motion_gain', 'command_gain'):
                self.assertTrue(.8 <= getattr(sampled.model, key) / getattr(MODEL, key) <= 1.2)
            for key in ('wn_rad_s', 'zeta'):
                self.assertTrue(.7 <= getattr(sampled.model, key) / getattr(MODEL, key) <= 1.3)
            self.assertTrue(.015 <= sampled.model.delay_s <= .055)
            self.assertLessEqual(abs(sampled.model.projected_bias_rad - MODEL.projected_bias_rad), .002)
            self.assertTrue(0 <= sampled.measurement_delay_s <= .02)
            self.assertTrue(0 <= sampled.transport_delay_s <= .02)
            self.assertLessEqual(abs(sampled.velocity_bias_m_s), .015)
            self.assertLessEqual(abs(sampled.tilt_bias_deg), .5)
            self.assertLessEqual(abs(sampled.rate_bias_deg_s), 5)
        nominal = sample_scenario(rng, MODEL, randomize=False)
        self.assertEqual(nominal.model, MODEL)
        self.assertEqual(nominal.measurement_delay_s, 0)
        self.assertEqual(nominal.transport_delay_s, 0)
        self.assertEqual(nominal.velocity_bias_m_s, 0)

    def test_invalid_configuration_rejected(self):
        for changes in (dict(control_period_s=0), dict(control_period_s=np.nan),
                        dict(max_brake_s=-1), dict(max_brake_s=.023),
                        dict(tail_observation_s=.1), dict(history_length=0),
                        dict(history_length=2.5), dict(history_length=True),
                        dict(reverse_weight=-1), dict(rollback_weight=np.inf),
                        dict(duration_weight=-1), dict(target_tail_speed_m_s=np.nan)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                RLBrakingEnv(MODEL, replace(RLBrakeConfig(), **changes))

    def test_invalid_scenarios_models_options_and_actions_rejected(self):
        env = RLBrakingEnv(MODEL)
        for changes in (dict(name=''), dict(snapshot=BrakingSnapshot(1., 0., .3, 0., 0.)),
                        dict(snapshot=BrakingSnapshot(0., 0., np.nan, 0., 0.)),
                        dict(command_history=((.01, 0.),)),
                        dict(command_history=((0., 0.), (-1., 0.))),
                        dict(command_history=((0., np.nan),)), dict(command_history=('bad',)),
                        dict(first_decision_s=-1), dict(measurement_delay_s=-.1),
                        dict(transport_delay_s=np.inf), dict(velocity_bias_m_s=np.nan),
                        dict(model=replace(MODEL, wn_rad_s=0))):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                env.reset(options={'scenario': scenario(**changes)})
        for fixed in ([], [None]):
            with self.assertRaises(ValueError):
                RLBrakingEnv(MODEL, scenarios=fixed)
        with self.assertRaises(ValueError):
            RLBrakingEnv(replace(MODEL, delay_s=-1))
        with self.assertRaises(ValueError):
            env.reset(options={'future_state': 3})
        env.reset(seed=42)
        for action in (-1, 2, .5, [1], np.nan):
            with self.subTest(action=action), self.assertRaises(ValueError):
                env.step(action)

    def test_fixed_list_and_explicit_scenario_override(self):
        cases = [scenario(name='a'), scenario(name='b')]
        env = RLBrakingEnv(MODEL, scenarios=cases, randomize=False)
        env.reset(seed=721)
        selected = env.scenario
        env.reset(seed=721)
        self.assertEqual(env.scenario, selected)
        override = scenario(name='override')
        env.reset(options={'scenario': override})
        self.assertEqual(env.scenario, override)


if __name__ == '__main__':
    unittest.main()
