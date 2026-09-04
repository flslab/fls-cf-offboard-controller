import dataclasses
import math
from types import SimpleNamespace
import unittest

import numpy as np

from Interaction.braking_split_diagnostic import TiltTrial, tilt_prediction
from Interaction.braking_validation_simulator import DelayedTiltPlant
from Interaction.offline_braking_selector import BrakingSnapshot, FrozenTiltModel


class DelayedTiltPlantTests(unittest.TestCase):
    def setUp(self):
        self.model = FrozenTiltModel(.025, 14.5, .98, 1.035, 1.09)
        self.state = BrakingSnapshot(10.0, 2.0, .7, 0.0, 0.0)

    def test_level_keeps_constant_velocity_and_linear_position(self):
        plant = DelayedTiltPlant(self.model, self.state)
        end = plant.advance_to(11.0)
        self.assertAlmostEqual(end.position_m, 2.7, places=11)
        self.assertEqual(end.velocity_m_s, .7)
        self.assertEqual(end.tilt_rad, 0)
        self.assertTrue(plant.offline_only)
        self.assertFalse(plant.flight_command_generated)

    def test_constant_equilibrium_tilt_matches_exact_acceleration(self):
        model = dataclasses.replace(self.model, projected_bias_rad=.003)
        angle = .10
        command = (angle - model.projected_bias_rad) / model.command_gain
        plant = DelayedTiltPlant(model, dataclasses.replace(self.state, tilt_rad=angle),
                                 [(9.0, command)], disturbance_acceleration_m_s2=-.03)
        end = plant.advance_to(10.5)
        accel = model.motion_gain * 9.81 * math.tan(angle) - .03
        self.assertAlmostEqual(end.velocity_m_s, .7 + accel * .5, places=11)
        self.assertAlmostEqual(end.position_m, 2 + .7 * .5 + .5 * accel * .5 ** 2, places=11)
        self.assertAlmostEqual(end.tilt_rad, angle, places=13)

    def test_command_never_applies_before_full_delay(self):
        plant = DelayedTiltPlant(self.model, self.state, transport_delay_s=.013)
        plant.send_command(10.0, -.35)
        at_event = plant.advance_to(10.038)
        self.assertEqual(at_event.tilt_rad, 0)
        self.assertEqual(at_event.tilt_rate_rad_s, 0)
        self.assertLess(plant.advance_to(10.04).tilt_rad, 0)
        self.assertIn(10.038, [row.time_s for row in plant.snapshot_history])

    def test_pending_commands_and_exact_short_pulse_boundaries(self):
        plant = DelayedTiltPlant(self.model, self.state,
                                 [(9.9, 0), (9.9901, -.35), (9.9907, 0)],
                                 integration_step_s=.002)
        self.assertEqual(plant.advance_to(10.0151).tilt_rad, 0)
        self.assertLess(plant.advance_to(10.1).tilt_rad, 0)
        times = [row.time_s for row in plant.snapshot_history]
        self.assertIn(9.9901 + self.model.delay_s, times)
        self.assertIn(9.9907 + self.model.delay_s, times)

    def test_causal_measurement_history_and_startup_unavailable(self):
        plant = DelayedTiltPlant(self.model, self.state, integration_step_s=.002)
        self.assertIsNone(plant.get_delayed_snapshot(.01))
        plant.advance_to(10.01)
        query = 10.0051
        snapshot = plant.snapshot_at_or_before(query)
        self.assertLessEqual(snapshot.time_s, query)
        self.assertLess(query - snapshot.time_s, .0021)
        self.assertEqual(snapshot, plant.get_delayed_snapshot(10.01 - query))
        self.assertLess(snapshot.position_m, plant.current_snapshot.position_m)
        self.assertEqual(plant.get_delayed_snapshot(0), plant.current_snapshot)
        self.assertIsNone(plant.snapshot_at_or_before(9.9))
        with self.assertRaises(ValueError):
            plant.snapshot_at_or_before(10.011)

    def test_no_future_commands_or_retroactive_sends(self):
        plant = DelayedTiltPlant(self.model, self.state)
        for timestamp in [9.99, 10.01]:
            with self.assertRaises(ValueError):
                plant.send_command(timestamp, -.35)
        plant.send_command(10.0, -.35)
        plant.advance_to(10.02)
        plant.send_command(10.02, 0)
        self.assertEqual(plant.command_history, ((10.0, -.35), (10.02, 0)))
        self.assertLess(plant.advance_to(10.04).tilt_rad, 0)

    def test_same_instant_last_command_wins(self):
        plant = DelayedTiltPlant(self.model, self.state)
        plant.send_command(10, -.35)
        plant.send_command(10, 0)
        self.assertEqual(plant.advance_to(10.1).tilt_rad, 0)

    def test_rk4_matches_independent_analytic_tilt_with_pending_queue(self):
        for zeta in [.6, 1.0, 1.5]:
            with self.subTest(zeta=zeta):
                model = dataclasses.replace(self.model, zeta=zeta, projected_bias_rad=.002)
                state = dataclasses.replace(self.state, tilt_rad=.12, tilt_rate_rad_s=-.45)
                history = [(9.8, .2), (9.99, -.3), (10.0, -.35)]
                plant = DelayedTiltPlant(model, state, history)
                plant.advance_to(10.0837)
                plant.send_command(10.0837, 0)
                plant.advance_to(10.8)
                samples = plant.snapshot_history
                times = np.array([s.time_s for s in samples])
                command_times, commands = map(np.array, zip(*plant.command_history))
                trial = SimpleNamespace(times=times, command_times=command_times,
                                        commands=9.81 * np.tan(commands),
                                        direction=np.array([0., 1.]))
                item = TiltTrial(trial, np.array([state.tilt_rad]),
                                 np.array([state.tilt_rate_rad_s]))
                fit = dict(model='second_order', delay_s=model.delay_s,
                           wn_rad_s=model.wn_rad_s, zeta=model.zeta,
                           gain=model.command_gain, bias_world_y_rad=model.projected_bias_rad)
                expected = tilt_prediction(item, fit)
                np.testing.assert_allclose([s.tilt_rad for s in samples], expected,
                                           atol=3e-9, rtol=0)

    def test_advance_chunking_and_step_convergence(self):
        def run(step, chunked):
            plant = DelayedTiltPlant(self.model, self.state,
                                     [(9.99, -.35)], integration_step_s=step)
            if chunked:
                for time in [10.0111, 10.0621, 10.071]:
                    plant.advance_to(time)
            plant.advance_to(10.0837)
            plant.send_command(10.0837, 0)
            return plant.advance_to(10.8)
        coarse, fine = run(.002, False), run(.0005, True)
        for name in ['position_m', 'velocity_m_s', 'tilt_rad', 'tilt_rate_rad_s']:
            self.assertAlmostEqual(getattr(coarse, name), getattr(fine, name), delta=2e-8)

    def test_bad_model_history_settings_and_queries_rejected(self):
        for kwargs in [dict(integration_step_s=.003), dict(integration_step_s=0),
                       dict(transport_delay_s=-.1), dict(disturbance_acceleration_m_s2=float('nan'))]:
            with self.assertRaises(ValueError):
                DelayedTiltPlant(self.model, self.state, **kwargs)
        for history in [[(10.01, 0)], [(9.9, 0), (9.8, 0)], [(9.9, float('inf'))], [(9.9,)]]:
            with self.assertRaises(ValueError):
                DelayedTiltPlant(self.model, self.state, history)
        for model in [dataclasses.replace(self.model, delay_s=-1),
                      dataclasses.replace(self.model, wn_rad_s=0),
                      dataclasses.replace(self.model, motion_gain=float('nan'))]:
            with self.assertRaises(ValueError):
                DelayedTiltPlant(model, self.state)
        plant = DelayedTiltPlant(self.model, self.state)
        for time in [9.9, float('nan'), float('inf')]:
            with self.assertRaises(ValueError):
                plant.advance_to(time)
        for delay in [-.01, float('nan')]:
            with self.assertRaises(ValueError):
                plant.get_delayed_snapshot(delay)


if __name__ == '__main__':
    unittest.main()
