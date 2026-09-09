import copy
import unittest
from types import SimpleNamespace

import numpy as np

from Interaction.braking_split_diagnostic import TiltTrial
from Interaction.evaluate_braking_snapshots import (
    evaluate_trial_snapshots, projected_frozen_model,
)


FROZEN = dict(frozen_tilt_fit=dict(model='second_order', delay_s=.025,
    wn_rad_s=14.5, zeta=.98, gain=1.035, bias_world_y_rad=.003),
    frozen_motion_gain=1.09)


def fixture():
    times = np.linspace(0, 1, 101)
    trial = SimpleNamespace(times=times, positions=.2*times,
        velocities=np.full(101, .2), direction=np.array([0., 1.]), segment=0,
        command_times=np.array([-.5, .05, .1, .34]),
        commands=np.array([0., 3.57, -3.57, 0.]), phase_times={'brake': .1})
    return TiltTrial(trial, np.zeros(101), np.zeros(101))


class BrakingSnapshotTests(unittest.TestCase):
    def test_y_projection_reverses_only_world_bias(self):
        positive = projected_frozen_model(FROZEN, [0, 1])
        negative = projected_frozen_model(FROZEN, [0, -1])
        self.assertEqual(positive.projected_bias_rad, -negative.projected_bias_rad)
        self.assertEqual(positive.motion_gain, negative.motion_gain)
        with self.assertRaises(ValueError):
            projected_frozen_model(FROZEN, [1, 0])

    def test_fixed_target_is_not_recomputed_from_each_snapshot(self):
        rows = evaluate_trial_snapshots(fixture(), FROZEN, .16)
        self.assertEqual(len(rows), 5)
        targets = [row['frozen_hypothetical_target_position_m'] for row in rows]
        np.testing.assert_allclose(targets, .18)
        self.assertGreater(rows[0]['remaining_target_distance_m'], rows[-1]['remaining_target_distance_m'])
        for row in rows:
            state = row['diagnostics']['snapshot']
            self.assertAlmostEqual(state['position_m']+row['remaining_target_distance_m'], .18)

    def test_future_state_and_command_do_not_leak_into_snapshot(self):
        original = fixture()
        baseline = evaluate_trial_snapshots(original, FROZEN, 1, [0])[0]
        modified = copy.deepcopy(original)
        future = modified.trial.times > .1
        modified.trial.positions[future] = 100
        modified.trial.velocities[future] = -100
        modified.angle[future] = 10
        modified.rate[future] = 10
        modified.trial.commands[modified.trial.command_times > .1] = 100
        self.assertEqual(baseline, evaluate_trial_snapshots(modified, FROZEN, 1, [0])[0])
        self.assertTrue(all(t <= baseline['diagnostics']['snapshot']['time_s']
            for t, _ in baseline['diagnostics']['sent_command_history']))

    def test_causal_sampling_not_nearest_future_measurement(self):
        row = evaluate_trial_snapshots(fixture(), FROZEN, 1, [.005])[0]
        self.assertAlmostEqual(row['diagnostics']['snapshot']['time_s'], .1)
        self.assertAlmostEqual(row['measurement_before_requested_s'], .005)

    def test_requires_explicit_finite_forward_target(self):
        for distance in [float('nan'), float('inf'), -.01]:
            with self.assertRaises(ValueError):
                evaluate_trial_snapshots(fixture(), FROZEN, distance)

    def test_no_fitting_or_mutation_of_frozen_model(self):
        frozen = copy.deepcopy(FROZEN)
        evaluate_trial_snapshots(fixture(), frozen, 1)
        self.assertEqual(frozen, FROZEN)


if __name__ == '__main__':
    unittest.main()
