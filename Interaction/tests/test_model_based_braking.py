"""Pure synthetic tests; no cflib, firmware, hardware or frozen RL imports."""
import copy
import math
import unittest

import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import expm

from Interaction.model_based_braking import (
    ModelBasedBrakingController,
    RollingMotionResidualObserver,
    _second_order_transition,
)


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
    def test_causal_motion_residual_observer_recovers_constant_bias(self):
        observer = RollingMotionResidualObserver(
            sigma_floor_m_s2=.01,
        )
        bias = .6
        snapshot = None
        for stamp in (0., .02, .04, .06, .08):
            snapshot = observer.update(stamp, bias*stamp, 0., 1.)
        self.assertTrue(snapshot['ready'])
        self.assertEqual(snapshot['status'], 'ready')
        self.assertAlmostEqual(
            snapshot['filtered_acceleration_m_s2'], bias, places=9
        )
        self.assertGreaterEqual(snapshot['window_span_s'], .06)
        count = snapshot['sample_count']
        duplicate = observer.update(.08, bias*.08, 0., 1.)
        self.assertEqual(duplicate['sample_count'], count)

    def test_motion_residual_observer_uses_irregular_real_samples(self):
        observer = RollingMotionResidualObserver(
            sigma_floor_m_s2=.01,
        )
        theta = .1
        model_acceleration = 9.81*math.tan(theta)
        bias = -.35
        snapshot = None
        for stamp in (0., .015, .037, .065, .08):
            velocity = (model_acceleration+bias)*stamp
            snapshot = observer.update(stamp, velocity, theta, 1.)
        self.assertTrue(snapshot['ready'])
        self.assertAlmostEqual(snapshot['raw_acceleration_m_s2'], bias, places=9)

    def test_motion_residual_gap_and_outlier_never_reuse_stale_bias(self):
        observer = RollingMotionResidualObserver(
            sigma_floor_m_s2=.01, max_abs_accel_m_s2=1.,
        )
        for stamp in (0., .02, .04, .06):
            ready = observer.update(stamp, .5*stamp, 0., 1.)
        self.assertTrue(ready['ready'])
        reset = observer.update(.12, .06, 0., 1.)
        self.assertFalse(reset['ready'])
        self.assertEqual(reset['status'], 'sample_gap_reset')
        self.assertEqual(reset['filtered_acceleration_m_s2'], 0.)
        for stamp in (.14, .16, .18):
            rejected = observer.update(stamp, 20.*stamp, 0., 1.)
        self.assertFalse(rejected['ready'])
        self.assertEqual(rejected['status'], 'residual_outlier_rejected')
        self.assertTrue(rejected['rejected'])
        self.assertFalse(rejected['clipped'])
        self.assertEqual(rejected['filtered_acceleration_m_s2'], 0.)

    def test_motion_residual_requires_full_causal_window(self):
        observer = RollingMotionResidualObserver()
        for stamp in (0., .02, .04):
            snapshot = observer.update(stamp, .4*stamp, 0., 1.)
        self.assertFalse(snapshot['ready'])
        self.assertEqual(snapshot['status'], 'warming_up')
        reset = observer.update(.03, .012, 0., 1.)
        self.assertFalse(reset['ready'])
        self.assertEqual(reset['status'], 'nonincreasing_timestamp_reset')

    def test_enabled_motion_residual_changes_forecast_and_reports_uncertainty(self):
        corrected = controller(
            deadline=.38,
            motion_residual_observer_enabled=True,
            motion_residual_sigma_floor_m_s2=.05,
        )
        baseline = controller(deadline=.38)
        bias = .6
        for stamp in (0., .02, .04, .06, .08):
            measured = state(
                stamp, v=.20+bias*stamp, theta=0., omega=0.
            )
            corrected.observe_state(stamp, measured)
        current = state(.08, v=.20+bias*.08, theta=0., omega=0.)
        corrected_result = corrected.decide(.08, current)
        baseline_result = baseline.decide(.08, current)
        self.assertTrue(corrected_result['motion_residual_ready'])
        self.assertTrue(
            corrected_result['motion_residual_corrected_prediction']
        )
        self.assertAlmostEqual(
            corrected_result['motion_residual_acceleration_m_s2'], bias,
            places=9,
        )
        self.assertGreater(
            corrected_result['motion_residual_dynamic_velocity_margin_m_s'],
            0.,
        )
        self.assertNotAlmostEqual(
            corrected_result['predicted_terminal_velocity_m_s'],
            baseline_result['predicted_terminal_velocity_m_s'],
        )

    def test_warming_residual_path_preserves_base_forecast(self):
        baseline = controller()
        warming = controller(motion_residual_observer_enabled=True)
        current = state()
        first = baseline.decide(0., current)
        second = warming.decide(0., current)
        self.assertEqual(second['motion_residual_status'], 'warming_up')
        for key in (
                'action', 'reason', 'selected_pulse_s',
                'predicted_terminal_velocity_m_s',
                'predicted_terminal_tilt_deg',
                'predicted_endpoint_position_m',
                'hard_feasible_candidate_count'):
            self.assertEqual(first[key], second[key], key)

    def test_residual_uncertainty_participates_in_hard_velocity_interval(self):
        item = controller(
            deadline=.38,
            motion_residual_observer_enabled=True,
            motion_residual_sigma_floor_m_s2=.5,
        )
        for stamp in (0., .02, .04, .06, .08):
            item.observe_state(stamp, state(
                stamp, v=.2, theta=0., omega=0.
            ))
        result = item.decide(.08, state(.08, v=.2, theta=0., omega=0.))
        self.assertTrue(result['motion_residual_ready'])
        self.assertGreaterEqual(
            result['motion_residual_dynamic_velocity_margin_m_s'], .079
        )
        self.assertEqual(result['hard_feasible_candidate_count'], 0)
        self.assertFalse(result['terminal_velocity_constraint_satisfied'])

    def test_repeated_stale_packet_does_not_renew_residual_horizon(self):
        item = controller(
            deadline=.38,
            motion_residual_observer_enabled=True,
            motion_residual_sigma_floor_m_s2=.05,
        )
        bias = .4
        for stamp in (0., .02, .04, .06, .08):
            item.observe_state(
                stamp,
                state(stamp, v=.2+bias*stamp, theta=0., omega=0.),
            )
        packet = state(.08, v=.2+bias*.08, theta=0., omega=0.)
        observed = item._state(packet, .08)
        residual = item._observe_motion_residual(observed)
        first = item._forecast(observed, .08, np.array([0.]), residual)
        duplicate = item._observe_motion_residual(item._state(packet, .11))
        later = item._forecast(observed, .11, np.array([0.]), duplicate)
        self.assertEqual(residual['sample_count'], duplicate['sample_count'])
        self.assertAlmostEqual(
            first['motion_residual_dynamic_margin_m_s'],
            later['motion_residual_dynamic_margin_m_s'],
            places=12,
        )
        self.assertAlmostEqual(
            first['motion_residual_dynamic_margin_m_s'], 2.*.05*.08,
            places=12,
        )

    def test_motion_residual_configuration_is_bounded(self):
        for config in (
            dict(motion_residual_window_s=.11),
            dict(motion_residual_min_window_s=.03),
            dict(motion_residual_max_sample_gap_s=.05),
            dict(motion_residual_filter_tau_s=.21),
            dict(motion_residual_apply_horizon_s=.11),
            dict(motion_residual_min_samples=2),
            dict(motion_residual_max_abs_accel_m_s2=5.1),
            dict(motion_residual_sigma_multiplier=4.1),
        ):
            with self.subTest(config=config), self.assertRaises(ValueError):
                controller(**config)

    def test_closed_form_transition_matches_matrix_exponential(self):
        for wn in (5., 14., 100.):
            for zeta in (.2, .8, .999999, 1., 1.000001, 1.5, 2.):
                for dt in (1e-6, .003, .01, .02):
                    matrix = np.array([
                        [0., 1.],
                        [-wn*wn, -2.*zeta*wn],
                    ])
                    np.testing.assert_allclose(
                        _second_order_transition(wn, zeta, dt),
                        expm(matrix*dt),
                        rtol=2e-10,
                        atol=2e-12,
                    )

    def test_default_disabled_and_candidate_requires_explicit_experiment(self):
        for config, reason in [({}, "disabled"),
                               ({"enabled": True}, "model_not_approved_for_nonexperimental_control")]:
            item = ModelBasedBrakingController(model(), target_position_xy=[0., .15],
                direction_xy=[0., 1.], brake_deadline_s=.32, config=config)
            result = item.decide(0., state())
            self.assertEqual(result['action'], 'fallback')
            self.assertEqual(result['reason'], reason)
            self.assertIsNone(result['roll_deg'])

    def test_persisted_control_eligibility_flag_is_enforced(self):
        source = model()
        source['control_eligible'] = False
        item = ModelBasedBrakingController(
            source, target_position_xy=[0., .15], direction_xy=[0., 1.],
            brake_deadline_s=.32,
            config=dict(enabled=True, experimental_calibration=True),
        )
        self.assertEqual(
            item.decide(0., state())['reason'],
            'model_marked_control_ineligible',
        )

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
        # The hard terminal constraint makes both selected outcomes converge
        # near zero velocity; the pending command is reflected by selecting a
        # different remaining pulse instead of by leaving a terminal error.
        self.assertNotEqual(da['selected_pulse_s'], db['selected_pulse_s'])
        self.assertTrue(da['hard_terminal_constraints_satisfied'])
        self.assertTrue(db['hard_terminal_constraints_satisfied'])

    def test_budget_is_visible_fallback(self):
        item = controller()
        ticks = iter([0., 2., 2.])
        item._clock = lambda: next(ticks)
        result = item.decide(0., state())
        self.assertEqual(result['action'], 'brake')
        self.assertEqual(
            result['reason'],
            'prediction_compute_budget_exceeded_continue_bounded_original_brake',
        )
        self.assertAlmostEqual(result['selected_pulse_s'], .32)
        self.assertTrue(result['fallback_to_original_brake'])
        self.assertFalse(result['terminal_constraints_evaluated'])
        self.assertIsNone(result['hard_terminal_constraints_satisfied'])
        self.assertAlmostEqual(result['roll_deg'], 20.)

    def test_budget_overrun_never_brakes_after_measured_reverse(self):
        item = controller()
        ticks = iter([0., 2., 2.])
        item._clock = lambda: next(ticks)
        result = item.decide(0., state(v=-.03))
        self.assertEqual(result['action'], 'level')
        self.assertEqual(
            result['reason'],
            'prediction_compute_budget_exceeded_level_for_nonpositive_velocity',
        )
        self.assertFalse(result['fallback_to_original_brake'])
        self.assertEqual(result['projected_tilt_rad'], 0.)
        self.assertTrue(result['level_latched'])

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
        self.assertLessEqual(result['candidate_count'], 64)
        self.assertLessEqual(result['integration_steps'], 370)
        invalid = controller(brake_tilt_deg=25.)
        self.assertEqual(invalid.decide(0., state())['reason'], 'brake_tilt_exceeds_observed_command')
        with self.assertRaises(ValueError):
            controller(max_state_age_s=1e6)
        for config in (
            dict(terminal_velocity_tolerance_m_s=.11),
            dict(terminal_tilt_tolerance_deg=5.1),
        ):
            with self.subTest(config=config), self.assertRaises(ValueError):
                controller(**config)

    def test_terminal_velocity_and_tilt_are_hard_constraints(self):
        velocity_limited = controller(
            terminal_velocity_tolerance_m_s=.001,
        ).decide(0., state())
        self.assertEqual(velocity_limited['action'], 'level')
        self.assertEqual(
            velocity_limited['reason'],
            'no_candidate_satisfies_terminal_state_constraints_level_to_remove_brake',
        )
        self.assertEqual(
            velocity_limited['hard_feasible_candidate_count'], 0
        )
        self.assertFalse(
            velocity_limited['hard_terminal_constraints_satisfied']
        )

        tilt_limited = controller(
            prediction_horizon_s=.1,
            terminal_velocity_tolerance_m_s=.10,
            terminal_tilt_tolerance_deg=.01,
        ).decide(0., state(v=.05))
        self.assertEqual(tilt_limited['action'], 'level')
        self.assertEqual(tilt_limited['hard_feasible_candidate_count'], 0)
        self.assertFalse(tilt_limited['terminal_tilt_constraint_satisfied'])

    def test_selected_brake_candidate_satisfies_all_hard_constraints(self):
        result = controller().decide(0., state())
        self.assertGreater(result['hard_feasible_candidate_count'], 0)
        self.assertTrue(result['hard_terminal_constraints_satisfied'])
        self.assertTrue(result['terminal_velocity_constraint_satisfied'])
        self.assertTrue(result['terminal_tilt_constraint_satisfied'])
        self.assertTrue(result['terminal_candidate_grid_refined'])

    def test_directional_component_and_velocity_error_margin_are_enforced(self):
        source = model()
        quality = copy.deepcopy(source['identifiability'])
        source['directional_models'] = {
            'positive_y': dict(
                direction_y=1, attitude_fit=copy.deepcopy(source['attitude_fit']),
                motion_gain=.8, identifiability=copy.deepcopy(quality),
                terminal_velocity_error_margin_m_s=.049,
            ),
            'negative_y': dict(
                direction_y=-1, attitude_fit=copy.deepcopy(source['attitude_fit']),
                motion_gain=1.2, identifiability=copy.deepcopy(quality),
                terminal_velocity_error_margin_m_s=0.,
            ),
        }
        positive = ModelBasedBrakingController(
            source, target_position_xy=[0., .15], direction_xy=[0., 1.],
            brake_deadline_s=.32,
            config=dict(enabled=True, experimental_calibration=True,
                        max_compute_s=1.),
        )
        positive.record_command(-.2, .35)
        positive.record_command(-.05, 0.)
        result = positive.decide(0., state())
        self.assertEqual(positive.params['motion_gain'], .8)
        self.assertEqual(result['selected_directional_model'], 'positive_y')
        self.assertEqual(result['terminal_velocity_error_margin_m_s'], .049)
        self.assertEqual(result['hard_feasible_candidate_count'], 0)

        negative = ModelBasedBrakingController(
            source, target_position_xy=[0., -.15], direction_xy=[0., -1.],
            brake_deadline_s=.32,
            config=dict(enabled=True, experimental_calibration=True,
                        max_compute_s=1.),
        )
        self.assertEqual(negative.params['motion_gain'], 1.2)
        self.assertEqual(negative.selected_directional_model, 'negative_y')

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
