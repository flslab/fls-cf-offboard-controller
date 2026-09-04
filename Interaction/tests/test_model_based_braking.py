"""Pure synthetic tests; no cflib, firmware, hardware or frozen RL imports."""
import math
import unittest

import numpy as np
from scipy.integrate import solve_ivp

from Interaction.model_based_braking import ModelBasedBrakingController


def model():
    return dict(schema_version=1, kind="delayed_second_order_planar_prediction",
                prediction_scope="attitude_command_only", train_segment_ids=[0, 1],
                deployment_approved=False, independent_validation_complete=False,
                identifiability=dict(identifiable=True, bound_active_parameters=[]),
                attitude_fit=dict(model="second_order", delay_s=.03, wn_rad_s=14.,
                                  zeta=.8, gain=1., bias_world_y_rad=0.),
                motion_gain=1., data_ranges=[
                    dict(direction_y=sign, command_acceleration_m_s2=[-3.57055, 3.57055],
                         velocity_m_s=[-.2, .8], theta_rad=[-.4, .4],
                         battery_voltage_V=[7.3, 7.5]) for sign in (-1, 1)])


def state(t=0., *, p=0., v=.55, theta=.15, omega=-1., direction=1):
    return dict(time_s=t, position_xy=[0., direction*p], velocity_xy=[0., direction*v],
                orientation_rpy_rad=[-direction*theta, 0., 0.],
                angular_velocity_rad_s=[-direction*omega, 0., 0.],
                state_group_skew_s=.004, battery_voltage_V=7.4)


def controller(*, direction=1, target=.15, deadline=.32, **config):
    config = dict(dict(enabled=True, experimental_calibration=True,
                       max_compute_s=1.), **config)
    result = ModelBasedBrakingController(model(), target_position_xy=[0., direction*target],
        direction_xy=[0., direction], brake_deadline_s=deadline, config=config)
    result.record_command(-.2, .35)
    result.record_command(-.05, 0.)
    return result


class ModelBasedBrakingTests(unittest.TestCase):
    def test_default_disabled_and_candidate_requires_explicit_experiment(self):
        for config, reason in [({}, "disabled"),
                               ({"enabled": True}, "model_not_approved_for_nonexperimental_control")]:
            item = ModelBasedBrakingController(model(), target_position_xy=[0., .15],
                direction_xy=[0., 1.], brake_deadline_s=.32, config=config)
            result = item.decide(0., state())
            self.assertEqual(result['action'], 'fallback')
            self.assertEqual(result['reason'], reason)
            self.assertIsNone(result['roll_deg'])

    def test_direction_and_actual_sent_history(self):
        item = controller()
        item.history = []
        self.assertEqual(item.decide(0., state())['reason'], 'insufficient_effective_command_history')
        with self.assertRaises(ValueError):
            ModelBasedBrakingController(model(), target_position_xy=[0., .15],
                direction_xy=[1., 0.], brake_deadline_s=.32)
        item = controller()
        item.record_command(.01, -.2)
        self.assertEqual(item.decide(0., state())['reason'], 'future_command_in_sent_history')
        with self.assertRaises(ValueError):
            item.record_command(.005, -.3)
        item = controller()
        item.record_command(.02, 0.)  # repeated value still certifies a send time
        with self.assertRaises(ValueError):
            item.record_command(.01, -.3)
        self.assertEqual(item.decide(0., state())['reason'], 'future_command_in_sent_history')

    def test_invalid_measurement_never_returns_stale_command(self):
        for updates in [dict(time_s=-.1), dict(state_group_skew_s=.04),
                        dict(velocity_xy=[float('nan'), .5]),
                        dict(velocity_xy=[.2, .5]), dict(battery_voltage_V=6.5),
                        dict(angular_velocity_rad_s=[8., 0., 0.])]:
            item = controller()
            result = item.decide(0., dict(state(), **updates))
            self.assertEqual(result['action'], 'fallback', result)
            self.assertIsNone(result['projected_tilt_rad'])

    def test_fixed_target_deadline_and_level_latch(self):
        item = controller(deadline=.10)
        original = item.target_position_xy.copy()
        result = item.decide(.11, state(.11))
        self.assertEqual(result['action'], 'level')
        self.assertEqual(result['reason'], 'original_brake_deadline')
        self.assertTrue(result['level_latched'])
        item.record_command(.11, 0.)
        later = item.decide(.12, state(.12, v=.7))
        self.assertEqual(later['action'], 'level')
        np.testing.assert_array_equal(original, item.target_position_xy)

    def test_no_acceleration_or_target_pulling_after_reverse(self):
        item = controller(target=-.02)
        result = item.decide(0., state(v=-.03))
        self.assertEqual(result['action'], 'level')
        self.assertEqual(result['projected_tilt_rad'], 0.)
        self.assertFalse(result['position_controller_identified'])

    def test_opposed_directions_are_symmetric(self):
        a, b = controller(direction=1), controller(direction=-1)
        da, db = a.decide(0., state()), b.decide(0., state(direction=-1))
        self.assertEqual(da['action'], db['action'])
        self.assertAlmostEqual(da['roll_deg'], -db['roll_deg'])
        self.assertAlmostEqual(da['predicted_terminal_velocity_m_s'], db['predicted_terminal_velocity_m_s'])

    def test_pending_command_changes_prediction(self):
        a, b = controller(), controller()
        b.record_command(-.01, -.35)  # not yet effective at t=0; cannot discard
        da, db = a.decide(0., state(v=.2)), b.decide(0., state(v=.2))
        self.assertGreater(abs(da['predicted_terminal_velocity_m_s']-db['predicted_terminal_velocity_m_s']), .02)

    def test_budget_is_visible_fallback(self):
        item = controller()
        ticks = iter([0., 2., 2.])
        item._clock = lambda: next(ticks)
        result = item.decide(0., state())
        self.assertEqual(result['reason'], 'prediction_compute_budget_exceeded')
        self.assertIsNone(result['roll_deg'])

    def test_model_copy_frozen_during_episode(self):
        source = model()
        item = ModelBasedBrakingController(source, target_position_xy=[0., .15],
            direction_xy=[0., 1.], brake_deadline_s=.32,
            config=dict(enabled=True, experimental_calibration=True))
        source['attitude_fit']['delay_s'] = .15
        self.assertEqual(item.params['delay_s'], .03)
        self.assertFalse(item.model['deployment_approved'])

    def test_workload_bound_and_unidentified_tilt(self):
        item = controller()
        result = item.decide(0., state())
        self.assertLessEqual(result['candidate_count'], 9)
        self.assertLessEqual(result['integration_steps'], 370)
        invalid = controller(brake_tilt_deg=25.)
        self.assertEqual(invalid.decide(0., state())['reason'], 'brake_tilt_exceeds_observed_command')
        with self.assertRaises(ValueError):
            controller(max_state_age_s=1e6)

    def test_closed_loop_shortens_brake_and_reduces_synthetic_reverse(self):
        adaptive, control = self.simulate(True), self.simulate(False)
        self.assertGreater(control['reverse'], .15)
        self.assertLess(adaptive['level_at'], .32)
        self.assertLess(adaptive['reverse'], control['reverse']*.25)
        self.assertLess(adaptive['reverse'], .03)
        self.assertEqual(adaptive['fallbacks'], 0)
        self.assertLessEqual(adaptive['transitions'], 1)

    @staticmethod
    def simulate(adaptive):
        """Independent solve_ivp plant, not the forecast implementation."""
        item = controller()
        y = np.array([.15, -1., .55, 0.])  # theta, omega, v, p
        history = [(-.2, .35), (-.05, 0.)]
        reverse, level_at, fallbacks, transitions, previous = 0., None, 0, 0, None
        for index in range(100):
            now = index*.01
            if adaptive:
                decision = item.decide(now, state(now, p=y[3], v=y[2], theta=y[0], omega=y[1]))
                if decision['action'] == 'fallback':
                    fallbacks += 1
                    tilt = 0.
                else:
                    tilt = decision['projected_tilt_rad']
                item.record_command(now, tilt)
            else:
                tilt = -math.radians(20.) if now < .32 else 0.
            if tilt == 0 and level_at is None:
                level_at = now
            if previous is not None and tilt != previous:
                transitions += 1
            previous = tilt
            history.append((now, tilt))
            def rhs(t, values):
                command = next(u for sent, u in reversed(history) if sent <= t-.03+1e-12)
                theta, omega, velocity, _ = values
                return [omega, 14.**2*(command-theta)-2*.8*14.*omega,
                        9.81*math.tan(theta), velocity]
            y = solve_ivp(rhs, [now, now+.01], y, max_step=.001,
                          rtol=1e-8, atol=1e-10).y[:, -1]
            reverse = max(reverse, -y[2])
        return dict(reverse=reverse, level_at=level_at, fallbacks=fallbacks,
                    transitions=transitions, final_position=float(y[3]), final_velocity=float(y[2]))


if __name__ == '__main__':
    unittest.main()
