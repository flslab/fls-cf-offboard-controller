"""Optional, fully offline v2 target/causality/physical-continuity regression tests."""
from dataclasses import asdict, replace
import math
import unittest
from unittest.mock import patch

import numpy as np

try:
    import gymnasium
except ModuleNotFoundError as exc:
    if exc.name != 'gymnasium':
        raise
    gymnasium = None

from Interaction.braking_validation_simulator import DelayedTiltPlant
from Interaction.offline_braking_selector import BrakingSnapshot, FrozenTiltModel

if gymnasium is not None:
    from Interaction.rl_braking_env import BrakingScenario, RLBrakingEnv
    from Interaction.rl_coast_env import (
        CoastBrakeConfig, CoastScenario, CoastBrakingEnv, sample_coast_scenario, score_coast_episode,
    )


MODEL = FrozenTiltModel(.025, 14.56, .987, 1.035, 1.095, -.00148)


def scenario(**changes):
    braking = BrakingScenario('test_initial_synthetic_target', MODEL,
        BrakingSnapshot(0., 0., .718, math.radians(5.75), math.radians(-56.6)),
        ((-.6, 0.), (-.4, math.radians(20)), (-.19, 0.)), first_decision_s=.009)
    return replace(CoastScenario(braking, .16), **changes)


@unittest.skipUnless(gymnasium is not None, 'optional offline RL dependencies are not installed')
class CoastBrakingEnvTests(unittest.TestCase):
    def test_exact_legacy_prefix_before_takeover(self):
        case = scenario(braking=replace(scenario().braking, measurement_delay_s=.02,
                                        velocity_bias_m_s=.01, tilt_bias_deg=.3, rate_bias_deg_s=-2.))
        v1, v2 = RLBrakingEnv(MODEL), CoastBrakingEnv(MODEL)
        old, old_info = v1.reset(seed=3, options={'scenario': case.braking})
        new, new_info = v2.reset(seed=3, options={'scenario': case})
        self.assertEqual(new.shape, (117,))
        self.assertEqual(new.dtype, np.float32)
        self.assertTrue(v2.observation_space.contains(new))
        for _ in range(10):
            np.testing.assert_array_equal(old, new[:114])
            np.testing.assert_array_equal(v2.legacy_observation, old)
            self.assertEqual(old_info['snapshot'], new_info['snapshot'])
            self.assertEqual(old_info['command_history'], new_info['command_history'])
            self.assertAlmostEqual(new[114], (v2.target_position_m-new_info['snapshot'].position_m)/.1, places=6)
            self.assertAlmostEqual(new[115], .16/.1, places=6)
            self.assertAlmostEqual(new[116], (3-new_info['elapsed_brake_s'])/3, places=6)
            old, _, _, _, old_info = v1.step(0)
            new, reward, terminated, truncated, new_info = v2.step(0)
            self.assertEqual((reward, terminated, truncated), (0., False, False))

    def test_target_is_fixed_and_never_clamped_to_overshooting_position(self):
        case = scenario(target_distance_m=.04)
        env = CoastBrakingEnv(MODEL, scenarios=[case])
        _, info = env.reset(seed=4)
        target = info['target_position_m']
        for _ in range(24):
            self.assertEqual(env.step(0)[4]['target_position_m'], target)
        self.assertGreater(env.plant.current_snapshot.position_m, target)
        _, _, _, _, info = env.step(1)
        self.assertEqual(env.target_position_m, target)
        self.assertEqual(info['metrics']['target_position_m'], target)
        self.assertTrue(all(row['target_position_m'] == target for row in env.position_decisions))
        self.assertGreater(info['metrics']['max_target_overshoot_m'], 0)
        with self.assertRaises(AttributeError):
            env.target_position_m = 0

    def test_direct_position_command_uses_delayed_biased_measurement_not_true_model(self):
        braking = replace(scenario().braking, measurement_delay_s=.01,
                          velocity_bias_m_s=.013, tilt_bias_deg=.4, rate_bias_deg_s=3.)
        case = scenario(braking=braking, position_extra_measurement_delay_s=.02)
        env = CoastBrakingEnv(MODEL)
        env.reset(options={'scenario': case})
        for _ in range(5):
            env.step(0)
        handoff = env.plant.current_snapshot.time_s
        measured = env.plant.snapshot_at_or_before(handoff-.03)
        expected_accel = np.clip(4*(env.target_position_m-measured.position_m)
                                - 3*(measured.velocity_m_s+.013), -2, 2)
        env.step(1)
        first = env.position_decisions[0]
        self.assertAlmostEqual(first['decision_time_s'], handoff)
        self.assertEqual(first['snapshot'].time_s, measured.time_s)
        self.assertAlmostEqual(first['snapshot'].velocity_m_s, measured.velocity_m_s+.013)
        self.assertAlmostEqual(first['command_tilt_rad'], math.atan(expected_accel/9.81))
        self.assertNotEqual(first['command_tilt_rad'], 0.)
        for row in env.position_decisions:
            self.assertLessEqual(row['snapshot'].time_s, row['decision_time_s']-.03+1e-12)
            self.assertLessEqual(abs(row['command_tilt_rad']), math.radians(20))

    def test_activation_wait_preserves_last_sent_brake_without_level_or_state_reset(self):
        case = scenario(takeover_activation_delay_s=.053)
        env = CoastBrakingEnv(MODEL)
        env.reset(options={'scenario': case})
        for _ in range(10):
            env.step(0)
        state = env.plant.current_snapshot
        commands = env.plant.command_history
        reference = DelayedTiltPlant(case.braking.model, state, commands,
                                     transport_delay_s=case.braking.transport_delay_s)
        expected = reference.advance_to(state.time_s+.053)
        old_history = env.plant.snapshot_history
        env.step(1)
        activation = env.position_decisions[0]['decision_time_s']
        self.assertAlmostEqual(activation, state.time_s+.053)
        self.assertEqual(env.plant.command_history[:len(commands)], commands)
        self.assertEqual(env.plant.snapshot_history[:len(old_history)], old_history)
        self.assertFalse(any(state.time_s <= t < activation for t, _ in env.plant.command_history[len(commands):]))
        actual = env.plant.snapshot_at_or_before(activation)
        for key in asdict(expected):
            self.assertAlmostEqual(getattr(actual, key), getattr(expected, key), places=12)

    def test_extra_feedback_delay_waits_after_latch_without_changing_first_decision(self):
        base = replace(scenario().braking, first_decision_s=0., measurement_delay_s=.02)
        case = scenario(braking=base, position_extra_measurement_delay_s=.04)
        env = CoastBrakingEnv(MODEL)
        obs, info = env.reset(options={'scenario': case})
        self.assertEqual(env.first_decision_s, .02)
        self.assertEqual(info['snapshot'].time_s, 0.)
        # Only one real initial measured frame; pre-initial frames stay invalid.
        self.assertEqual(np.sum(obs[5:77].reshape(12, 6)[:, 5]), 1.)
        old_commands = env.plant.command_history
        _, _, _, _, terminal = env.step(1)
        activation = terminal['position_activation_time_s']
        self.assertAlmostEqual(activation, .06)
        self.assertEqual(env.position_decisions[0]['snapshot'].time_s, 0.)
        self.assertEqual(env.plant.command_history[:len(old_commands)], old_commands)
        self.assertEqual(env.plant.command_history[len(old_commands)][0], activation)

    def test_hidden_position_parameters_do_not_change_initial_policy_view(self):
        a = scenario()
        b = replace(a, position_kp_s2=6., position_kd_s=4.5,
                    position_extra_measurement_delay_s=.04, takeover_activation_delay_s=.08)
        first, second = CoastBrakingEnv(MODEL), CoastBrakingEnv(MODEL)
        oa, ia = first.reset(options={'scenario': a})
        ob, ib = second.reset(options={'scenario': b})
        np.testing.assert_array_equal(oa, ob)
        self.assertEqual(ia, ib)
        self.assertFalse({'model', 'scenario', 'position_kp_s2', 'position_extra_measurement_delay_s'} & set(ia))

    def test_common_horizon_exact_timestamp_for_immediate_late_and_forced_handoff(self):
        case = scenario()
        ends = []
        for brake_ticks in (0, 24, 50):
            env = CoastBrakingEnv(MODEL)
            env.reset(options={'scenario': case})
            for _ in range(brake_ticks):
                self.assertFalse(env.step(0)[2])
            _, reward, done, truncated, info = env.step(0 if brake_ticks == 50 else 1)
            self.assertEqual((done, truncated), (True, False))
            self.assertEqual(reward, -info['loss'])
            self.assertEqual(info['episode_reward'], reward)
            self.assertAlmostEqual(info['metrics']['handoff_after_s'], brake_ticks*.01)
            self.assertEqual(env.plant.current_snapshot.time_s, env.first_decision_s+3.)
            self.assertEqual(info['decision_time_s'], env.first_decision_s+3.)
            self.assertEqual(env.legacy_observation.shape, (114,))
            ends.append(env.plant.current_snapshot.time_s)
            with self.assertRaises(RuntimeError):
                env.step(0)
            if brake_ticks == 50:
                self.assertEqual(info['handoff_reason'], 'maximum_brake_duration')
        self.assertEqual(len(set(ends)), 1)

    def test_early_settling_and_reversal_do_not_shortcut_horizon(self):
        nominal = replace(MODEL, projected_bias_rad=0.)
        for velocity in (0., -.2):
            braking = BrakingScenario('quiet_or_reverse', nominal,
                                      BrakingSnapshot(0., 0., velocity, 0., 0.), ((-1., 0.),))
            env = CoastBrakingEnv(nominal)
            env.reset(options={'scenario': CoastScenario(braking, .001)})
            _, _, _, _, info = env.step(1)
            self.assertEqual(env.plant.current_snapshot.time_s, 3.)
            if velocity < 0:
                self.assertFalse(info['metrics']['no_reverse_in_simulation'])
                self.assertTrue(info['metrics']['reversal_after_position'])

    def test_full_boundary_metrics_and_exact_terminal_window(self):
        case = scenario(takeover_activation_delay_s=.053)
        env = CoastBrakingEnv(MODEL)
        env.reset(options={'scenario': case})
        for _ in range(24):
            env.step(0)
        _, _, _, _, info = env.step(1)
        states = [s for s in env.plant.snapshot_history if s.time_s >= env.first_decision_s-1e-12]
        position = np.asarray([s.position_m for s in states])
        velocity = np.asarray([s.velocity_m_s for s in states])
        metrics = info['metrics']
        self.assertGreater(len(states), 3000)
        self.assertEqual(metrics['max_rollback_m'], float(np.max(np.maximum.accumulate(position)-position)))
        self.assertEqual(metrics['max_target_overshoot_m'], max(0., float(np.max(position-env.target_position_m))))
        self.assertEqual(metrics['min_velocity_m_s'], float(np.min(velocity)))
        terminal_start = env.first_decision_s+3.-.3
        self.assertTrue(any(s.time_s == terminal_start for s in states))
        self.assertFalse(metrics['position_model_calibrated'])
        self.assertFalse(metrics['position_target_validated'])
        self.assertFalse(metrics['flight_validated'])

    def test_joint_settling_and_dwell_require_all_posture_and_motion_conditions(self):
        env = CoastBrakingEnv(MODEL)
        env.reset(options={'scenario': scenario()})
        target, first = env.target_position_m, env.first_decision_s
        # A synthetic trace fixture tests the metric itself, not a plant claim.
        good = [BrakingSnapshot(first+t, target, 0., 0., 0.) for t in np.linspace(0, 3, 3001)]
        with patch.object(type(env.plant), 'snapshot_history', new_callable=lambda: property(lambda _: tuple(good))):
            row = env._metrics(first, first)
        self.assertTrue(row['joint_settled_in_simulation'])
        self.assertAlmostEqual(row['first_stable_dwell_after_s'], .3)
        for change in (dict(position_m=target+.031), dict(velocity_m_s=.041),
                       dict(tilt_rad=math.radians(3.1)), dict(tilt_rate_rad_s=math.radians(5.1))):
            bad = [replace(s, **change) for s in good]
            with self.subTest(change=change), patch.object(type(env.plant), 'snapshot_history',
                    new_callable=lambda: property(lambda _: tuple(bad))):
                row = env._metrics(first, first)
            self.assertFalse(row['joint_settled_in_simulation'])
            self.assertIsNone(row['first_stable_dwell_after_s'])

    def test_reversal_only_during_activation_wait_is_before_position_not_after(self):
        env = CoastBrakingEnv(MODEL)
        env.reset(options={'scenario': scenario()})
        first = env.first_decision_s
        request, activation = first+.1, first+.18
        # Construct a metric fixture that reverses after latch but recovers
        # before the first PD command, isolating the ownership-boundary test.
        states = [BrakingSnapshot(first+t, .16,
                  -.1 if .12 <= t <= .15 else .01, 0., 0.) for t in np.linspace(0, 3, 3001)]
        with patch.object(type(env.plant), 'snapshot_history',
                          new_callable=lambda: property(lambda _: tuple(states))):
            row = env._metrics(request, activation)
        self.assertFalse(row['reversal_before_handoff_request'])
        self.assertTrue(row['reversal_after_handoff_request'])
        self.assertTrue(row['reversal_before_position'])
        self.assertFalse(row['reversal_after_position'])
        self.assertFalse(row['no_reverse_in_simulation'])

    def test_sampler_endpoint_is_integrated_prelude_and_history_is_rebased(self):
        rng = np.random.default_rng(441)
        directions = set()
        for _ in range(40):
            case = sample_coast_scenario(rng, MODEL)
            directions.add(case.direction_sign)
            self.assertTrue(.04 <= case.target_distance_m <= .4)
            self.assertTrue(2 <= case.position_kp_s2 <= 6)
            self.assertTrue(2 <= case.position_kd_s <= 4.5)
            self.assertTrue(1 <= case.position_accel_limit_m_s2 <= 3)
            self.assertTrue(0 <= case.position_extra_measurement_delay_s <= .04)
            self.assertTrue(0 <= case.takeover_activation_delay_s <= .08)
            self.assertFalse(case.prelude['measured_trajectory'])
            accel, level = case.prelude['acceleration_duration_s'], case.prelude['level_duration_s']
            self.assertTrue(.08 <= accel <= .25)
            self.assertTrue(.04 <= level <= .20)
            plant = DelayedTiltPlant(case.braking.model, BrakingSnapshot(**case.prelude['initial_snapshot']),
                                    ((-1., 0.),), transport_delay_s=case.braking.transport_delay_s)
            plant.send_command(0., math.radians(20.))
            plant.advance_to(accel)
            plant.send_command(accel, 0.)
            state = plant.advance_to(accel+level)
            self.assertEqual(case.braking.snapshot, replace(state, time_s=0., position_m=0.))
            self.assertEqual(case.braking.command_history,
                             tuple((t-state.time_s, a) for t, a in plant.command_history))
            self.assertTrue(all(t < 0 for t, _ in case.braking.command_history))
        self.assertEqual(directions, {-1, 1})

    def test_direction_mirrors_projected_nominal_bias_only_once(self):
        rng = np.random.default_rng(311)
        for _ in range(20):
            case = sample_coast_scenario(rng, MODEL, randomize=False)
            self.assertEqual(case.braking.model.projected_bias_rad,
                             case.direction_sign*MODEL.projected_bias_rad)
            self.assertEqual(case.braking.model.motion_gain, MODEL.motion_gain)
            self.assertEqual(case.position_kp_s2, 4.)
            self.assertEqual(case.position_kd_s, 3.)
            self.assertEqual(case.position_extra_measurement_delay_s, 0.)

    def test_seed_replays_complete_episode_and_fixed_list(self):
        a, b = CoastBrakingEnv(MODEL), CoastBrakingEnv(MODEL)
        oa, ia = a.reset(seed=1267)
        ob, ib = b.reset(seed=1267)
        self.assertEqual(a.scenario, b.scenario)
        np.testing.assert_array_equal(oa, ob)
        self.assertEqual(ia, ib)
        for action in [0]*12+[1]:
            ra, rb = a.step(action), b.step(action)
            np.testing.assert_array_equal(ra[0], rb[0])
            self.assertEqual(ra[1:], rb[1:])
        fixed = [scenario(target_distance_m=.08), scenario(target_distance_m=.28)]
        env = CoastBrakingEnv(MODEL, scenarios=fixed, randomize=False)
        env.reset(seed=77)
        selected = env.scenario
        env.reset(seed=77)
        self.assertEqual(env.scenario, selected)

    def test_loss_exact_weights_and_nonfinite_rejection(self):
        metrics = dict(terminal_max_abs_position_error_m=.03,
                       terminal_max_abs_velocity_m_s=.04, max_target_overshoot_m=.06,
                       max_rollback_m=.09, integrated_abs_position_error_m_s=.3)
        self.assertAlmostEqual(score_coast_episode(metrics, CoastBrakeConfig()), 1+1+4+3+.2)
        for key in metrics:
            for bad in (-1., float('nan'), float('inf')):
                with self.subTest(key=key, bad=bad), self.assertRaises(ValueError):
                    score_coast_episode(dict(metrics, **{key: bad}), CoastBrakeConfig())

    def test_invalid_inputs_rejected_before_running(self):
        for change in (dict(control_period_s=.02), dict(max_brake_s=0), dict(max_brake_s=.503),
                       dict(history_length=13), dict(history_length=True), dict(horizon_s=np.nan),
                       dict(horizon_s=.7), dict(terminal_window_s=.2), dict(stable_dwell_s=0),
                       dict(position_control_period_s=0), dict(position_tolerance_m=-1)):
            with self.subTest(change=change), self.assertRaises(ValueError):
                CoastBrakingEnv(MODEL, replace(CoastBrakeConfig(), **change))
        for change in (dict(target_distance_m=0), dict(target_distance_m=np.nan),
                       dict(direction_sign=0), dict(direction_sign=True), dict(direction_sign=1.),
                       dict(position_kp_s2=0), dict(position_kd_s=np.inf),
                       dict(position_accel_limit_m_s2=-1), dict(position_extra_measurement_delay_s=.05),
                       dict(takeover_activation_delay_s=-.01), dict(prelude=[]),
                       dict(braking=replace(scenario().braking, command_history=((.1, 0.),)))):
            with self.subTest(change=change), self.assertRaises(ValueError):
                CoastBrakingEnv(MODEL, scenarios=[scenario(**change)])
        for cases in ([], [None], 1):
            with self.assertRaises(ValueError):
                CoastBrakingEnv(MODEL, scenarios=cases)
        env = CoastBrakingEnv(MODEL)
        with self.assertRaises(ValueError):
            env.reset(options={'future': 1})
        env.reset(seed=1)
        for action in (-1, 2, .5, [0]):
            with self.assertRaises(ValueError):
                env.step(action)


if __name__ == '__main__':
    unittest.main()
