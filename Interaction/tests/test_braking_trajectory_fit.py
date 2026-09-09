"""Full-trajectory fits share the causal replay integrator, including drag."""
import copy
import unittest
from unittest.mock import patch

import numpy as np
from scipy.integrate import solve_ivp

from Interaction.braking_replay import Trial, PHASES, replay, step
from Interaction.braking_trajectory_fit import (
    CANDIDATES, compare, fit_trajectories, parameters, relative_trial, trajectory_residual,
)


TRUTH = dict(command_delay_s=.055, command_time_constant_s=.085,
             horizontal_acceleration_scale=1.1, acceleration_bias_m_s2=0.,
             world_y_bias_m_s2=-.08, linear_drag_per_s=.4)


def synthetic_trials(truth=TRUTH):
    trials = []
    for i, duration in enumerate([.16, .16, .32, .32]):
        start, accel, level, brake = 0., .2, .2+duration, .4+duration
        after, end = brake+duration, brake+duration+.65
        times = np.arange(0, end, .02)
        phases = dict(zip(PHASES, [start, accel, level, brake, after, end]))
        trial = Trial(i, duration, np.array([0., 1. if i%2==0 else -1.]),
                      times, np.zeros(len(times)), np.full(len(times), 3*duration),
                      np.zeros(len(times)), np.array([start, accel, level, brake, after]),
                      np.array([0., 3.57, 0., -3.57, 0.]), phases, .02)
        anchor, prediction = replay(trial, truth)
        trial.velocities[anchor:] = prediction[:, 0]
        trial.positions[anchor:] = prediction[:, 1]
        trials.append(trial)
    return trials


class BrakingTrajectoryFitTests(unittest.TestCase):
    def test_exact_step_matches_independent_ode_solution(self):
        for tau, drag in [(.1, 0), (.1, .4), (.1, 10.), (.1, 10.0000001),
                          (.1, 1e-7), (.1, 1e-9), (.005, 3.)]:
            with self.subTest(tau=tau, drag=drag):
                a, v, p = .7, .9, -.2
                target, bias, dt = -3., -.08, .12
                actual = step(a, v, p, target, tau, dt, bias, drag)
                expected = solve_ivp(lambda _, y: [(target-y[0])/tau, y[0]+bias-drag*y[1], y[1]],
                                     [0, dt], [a, v, p], rtol=1e-11, atol=1e-12).y[:, -1]
                np.testing.assert_allclose(actual, expected, atol=2e-9, rtol=2e-9)

    def test_zero_tau_drag_solution_and_tiny_intervals(self):
        for dt in [1e-8, .01, 1.]:
            result = step(20., .5, .2, -2., 0., dt, .1, .4)
            expected = solve_ivp(lambda _, y: [-1.9-.4*y[0], y[0]],
                                 [0, dt], [.5, .2], rtol=1e-11, atol=1e-12).y[:, -1]
            np.testing.assert_allclose(result[1:], expected, atol=2e-10)

    def test_world_bias_flips_projection_but_not_physical_direction(self):
        trial = synthetic_trials()[0]
        fit = dict(TRUTH, horizontal_acceleration_scale=1., linear_drag_per_s=0., world_y_bias_m_s2=.1)
        trial.commands[:] = 0
        trial.attitude_acceleration[:] = 0
        anchor, positive = replay(trial, fit)
        trial.direction[1] = -1
        _, negative = replay(trial, fit)
        duration = trial.times[-1]-trial.times[anchor]
        self.assertAlmostEqual(positive[-1, 0]-negative[-1, 0], .2*duration)

    def test_fit_uses_identical_integrator_as_prediction(self):
        trials = synthetic_trials()
        values = [.055, .085, 1.1, -.08, .4]
        residual = trajectory_residual(values, 'trajectory_drag_world_bias', trials)
        np.testing.assert_allclose(residual, 0., atol=1e-12)

    def test_known_parameters_are_recovered_without_per_trial_adjustments(self):
        fit = fit_trajectories(synthetic_trials(), 'trajectory_drag_world_bias',
                              starts=((.05, .09, 1.05, -.05),))
        for name in ('command_delay_s', 'command_time_constant_s', 'horizontal_acceleration_scale',
                     'world_y_bias_m_s2', 'linear_drag_per_s'):
            self.assertAlmostEqual(fit[name], TRUTH[name], delta=2e-4)
        self.assertTrue(fit['offline_only'])
        self.assertNotIn('fit_schema_version', fit)

    def test_future_attitude_and_position_are_not_training_inputs(self):
        trials = synthetic_trials()
        values = [.055, .085, 1.1, -.08, .4]
        before = trajectory_residual(values, 'trajectory_drag_world_bias', trials)
        for t in trials:
            anchor = np.searchsorted(t.times, t.phase_times['brake'], side='right')-1
            t.positions[anchor+1:] = 1e6
            t.attitude_acceleration[anchor+1:] = -1e6
        np.testing.assert_array_equal(before, trajectory_residual(values, 'trajectory_drag_world_bias', trials))

    def test_relative_clock_translation_invariance(self):
        trial = synthetic_trials()[0]
        original = replay(trial, TRUTH)[1]
        trial.times += 100.
        trial.command_times += 100.
        trial.phase_times = {k: v+100 for k, v in trial.phase_times.items()}
        shifted = replay(relative_trial(trial), TRUTH)[1]
        np.testing.assert_allclose(original, shifted, atol=1e-10)

    def test_model_parameters_are_shared_and_drag_is_not_negative(self):
        for candidate in CANDIDATES:
            p = parameters([.05, .1, 1., -.08, .4], candidate)
            self.assertEqual(p['linear_drag_per_s'], .4 if candidate.endswith('drag_world_bias') else 0.)
        with self.assertRaises(ValueError):
            step(0, 0, 0, 0, .1, .1, drag=-.1)

    def test_new_candidate_fits_never_receive_the_held_out_trials(self):
        trials = synthetic_trials()
        for i in [4, 5]:
            extra = copy.deepcopy(trials[i-2])
            extra.segment, extra.duration = i, .24
            trials.append(extra)
        folds = [dict(held_segments=[i, i+1], train_segments=[j for j in range(6) if j not in [i, i+1]],
                      fit=TRUTH) for i in [0, 2, 4]]
        with (
            patch('Interaction.braking_trajectory_fit.extract', return_value=(TRUTH, trials, [], {})),
            patch('Interaction.braking_trajectory_fit.holdout_fits', return_value=folds),
            patch('Interaction.braking_trajectory_fit.fit_trajectories', return_value=TRUTH) as fit,
        ):
            report, _, _ = compare([], progress=lambda *args, **kwargs: None)
        self.assertEqual(fit.call_count, 12)
        for i, fold in enumerate(folds):
            for call in fit.call_args_list[3*i:3*i+3]:
                self.assertEqual({t.segment for t in call.args[0]}, set(fold['train_segments']))
        self.assertTrue(report['offline_only'])


if __name__ == '__main__':
    unittest.main()
