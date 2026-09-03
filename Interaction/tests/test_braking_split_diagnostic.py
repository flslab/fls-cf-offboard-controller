import copy
import unittest

import numpy as np
from scipy.integrate import solve_ivp

from Interaction.braking_split_diagnostic import (
    TiltTrial, brake_anchor, fit_tilt, motion_gain, motion_metrics,
    oracle_timing_scan, rolling_horizon, second_homogeneous, tilt_prediction,
)
from Interaction.tests.test_braking_trajectory_fit import synthetic_trials


FIT = dict(model='second_order', delay_s=.025, wn_rad_s=14.5, zeta=.95,
           gain=1.035, bias_world_y_rad=.001)


def items():
    result = []
    for trial in synthetic_trials():
        item = TiltTrial(trial, np.zeros(len(trial.times)), np.zeros(len(trial.times)))
        item.angle = tilt_prediction(item, FIT)
        result.append(item)
    return result


class SplitDiagnosticTests(unittest.TestCase):
    def test_second_order_homogeneous_matches_independent_ode(self):
        t = np.linspace(0, 1, 81)
        for zeta in [.3, .95, 1., 1.01, 1.8]:
            q, rate, wn = .12, -.8, 14.5
            expected = solve_ivp(lambda _, y:[y[1], -2*zeta*wn*y[1]-wn**2*y[0]],
                                 [0, 1], [q, rate], t_eval=t, rtol=1e-11, atol=1e-12).y[0]
            np.testing.assert_allclose(second_homogeneous(t,q,rate,wn,zeta), expected, atol=1e-10)

    def test_predictor_has_no_future_state_access(self):
        item = items()[0]
        anchor = brake_anchor(item)
        expected = tilt_prediction(item, FIT, anchor)
        changed = copy.deepcopy(item)
        changed.angle[anchor+1:] = -1000
        changed.rate[anchor+1:] = 1000
        changed.trial.velocities[anchor+1:] = 1000
        np.testing.assert_array_equal(tilt_prediction(changed, FIT, anchor), expected)

    def test_angular_rate_initialization_changes_prediction(self):
        item=items()[0]; anchor=brake_anchor(item)
        item.rate[anchor]=-1.
        with_rate=tilt_prediction(item,FIT,anchor)
        no_rate=tilt_prediction(item,FIT,anchor,False)
        self.assertAlmostEqual(with_rate[0],item.angle[anchor])
        self.assertGreater(max(abs(with_rate-no_rate)), .02)

    def test_pending_commands_before_anchor_are_retained(self):
        item=items()[0]; anchor=brake_anchor(item)
        t0=item.trial.times[anchor]
        item.angle[:]=0; item.rate[:]=0
        item.trial.command_times=np.array([0.,t0-.03,t0+.2])
        item.trial.commands=np.array([0.,3.,0.])
        fit=dict(FIT,model='first_order',delay_s=.08,tau_s=.1,bias_world_y_rad=0.)
        q=tilt_prediction(item,fit,anchor)
        t=item.trial.times[anchor:]-t0
        np.testing.assert_allclose(q[t<.05],0)
        self.assertGreater(max(q), .1)

    def test_known_tilt_parameters_recovered(self):
        fitted=fit_tilt(items(),'second_order')
        for key in ('delay_s','wn_rad_s','zeta','gain','bias_world_y_rad'):
            self.assertAlmostEqual(fitted[key], FIT[key], delta=1e-4)
        self.assertTrue(fitted['offline_only'])
        self.assertEqual(fitted['training_segments'], [0,1,2,3])

    def test_oracle_gain_uses_training_data_and_not_held_out(self):
        training=items()
        for item in training:
            a=brake_anchor(item)
            # Replace motion with exact trapezoidal integration of actual tilt proxy.
            from scipy.integrate import cumulative_trapezoid
            item.trial.attitude_acceleration=9.81*np.tan(item.angle)
            item.trial.velocities[a:]=item.trial.velocities[a]+1.08*cumulative_trapezoid(
                item.trial.attitude_acceleration[a:],item.trial.times[a:],initial=0)
        self.assertAlmostEqual(motion_gain(training),1.08)
        row,_=motion_metrics(training[0],training[0].angle[brake_anchor(training[0]):],1.08,'oracle')
        self.assertAlmostEqual(row['terminal_error_m_s'],0)

    def test_timing_scan_is_explicitly_oracle_and_zero_shift_recovers_known_data(self):
        training=items()
        from scipy.integrate import cumulative_trapezoid
        for item in training:
            a=brake_anchor(item)
            item.trial.attitude_acceleration=9.81*np.tan(item.angle)
            item.trial.velocities[a:]=item.trial.velocities[a]+1.08*cumulative_trapezoid(
                item.trial.attitude_acceleration[a:],item.trial.times[a:],initial=0)
        result=oracle_timing_scan(training[:2],training[2:])
        self.assertTrue(result['oracle_only'])
        self.assertAlmostEqual(result['training_selected_future_attitude_shift_s'],0)
        self.assertAlmostEqual(result['gain'],1.08)

    def test_rolling_forecasts_do_not_extend_into_recovery(self):
        item=items()[0]
        rows=rolling_horizon(item,FIT,1.08)
        self.assertTrue(rows)
        self.assertTrue(all(r['anchor_s']+r['horizon_s'] <= item.trial.times[-1] for r in rows))


if __name__=='__main__':unittest.main()
