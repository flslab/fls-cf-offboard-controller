"""Numerical tests use an independent, piecewise-integrated second-order ODE."""
import copy
import json
import unittest
import warnings
from unittest import mock

import numpy as np
from scipy.integrate import solve_ivp

from Interaction.online_dynamics_model import (
    PHASES, build_tilt_trials, evaluate_predictive_model, fit_predictive_model,
    predict_trajectory,
)


FIT = dict(model="second_order", delay_s=.037, wn_rad_s=17.4, zeta=.76,
           gain=1.07, bias_world_y_rad=.003)
MOTION_GAIN = 1.13


def known_model():
    return dict(schema_version=1, kind="delayed_second_order_planar_prediction",
                attitude_fit=dict(FIT), motion_gain=MOTION_GAIN, train_segment_ids=[0, 1])


def independent_plant(times, onsets, tilts, initial, direction, fit=FIT):
    """Integrate [theta, omega, velocity, position], splitting at delayed steps."""
    times, onsets = np.array(times), np.array(onsets)
    boundaries = np.unique(np.r_[times[0], times[-1],
        (onsets+fit["delay_s"])[(onsets+fit["delay_s"] > times[0]) &
                                  (onsets+fit["delay_s"] < times[-1])]])
    state = np.array(initial, dtype=float)
    result = np.zeros((len(times), 4))
    result[0] = state
    for start, end in zip(boundaries[:-1], boundaries[1:]):
        index = np.searchsorted(onsets+fit["delay_s"], (start+end)/2, side="right")-1
        target = fit["gain"]*(tilts[index] if index >= 0 else 0.)+fit["bias_world_y_rad"]*direction
        def derivative(_, value):
            theta, omega, velocity, _position = value
            return [omega, fit["wn_rad_s"]**2*(target-theta)-
                    2*fit["zeta"]*fit["wn_rad_s"]*omega,
                    MOTION_GAIN*9.81*np.tan(theta), velocity]
        solution = solve_ivp(derivative, (start, end), state, dense_output=True,
                             rtol=2e-11, atol=1e-12, max_step=.005)
        selected = (times > start) & (times <= end)
        result[selected] = solution.sol(times[selected]).T
        state = solution.y[:, -1]
    return result


def synthetic_samples(segment_ids=(0, 1, 2, 3), epoch=1_788_000_000.):
    samples = []
    for segment in segment_ids:
        direction = 1 if segment % 2 == 0 else -1
        duration = .32 + .025*(segment//2)
        onsets = [0., .22, .22+duration, .43+duration, .76+duration]
        tilts = [0., .11, 0., -.15, 0.]
        times = np.r_[0., np.arange(.007, onsets[-1]+.51, .01)]
        plant = independent_plant(times, onsets, tilts,
                                  [FIT["bias_world_y_rad"]*direction, 0., .01, .12], direction)
        clock_origin = epoch+segment*10.
        for t, (theta, omega, velocity, position) in zip(times[1:], plant[1:]):
            phase_index = np.searchsorted(onsets, t, side="right")-1
            samples.append(dict(
                segment_id=segment, timestamp=clock_origin+float(t),
                command_started_at=clock_origin+onsets[phase_index], phase=PHASES[phase_index],
                direction_xy=[0., float(direction)],
                command_acceleration_xy=[0., float(direction*9.81*np.tan(tilts[phase_index]))],
                actual_attitude_rpy_rad=[float(-theta*direction), 0., 0.],
                angular_velocity_rad_s=[float(-omega*direction), 0., 0.],
                velocity_xy=[0., float(velocity*direction)],
                position_xy=[0., float(position*direction)],
                battery_voltage_V=3.9, state_group_skew_s=.005,
            ))
    return samples


class OnlineDynamicsModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.training = synthetic_samples((0, 1))
        cls.validation = synthetic_samples((2, 3))
        cls.model = fit_predictive_model(cls.training)

    def test_independent_second_order_parameters_recovered(self):
        for key in FIT:
            if key == "model":
                self.assertEqual(self.model["attitude_fit"][key], FIT[key])
            else:
                self.assertAlmostEqual(self.model["attitude_fit"][key], FIT[key], delta=2e-4)
        self.assertAlmostEqual(self.model["motion_gain"], MOTION_GAIN, delta=.003)
        self.assertEqual(self.model["train_segment_ids"], [0, 1])
        self.assertTrue(self.model["identifiability"]["identifiable"])
        self.assertFalse(self.model["runtime_enabled"])
        self.assertFalse(self.model["deployment_approved"])
        self.assertFalse(self.model["independent_validation_complete"])
        self.assertEqual(self.model["candidate_status"], "requires_held_out_validation")
        self.assertEqual(self.model["clock_scope"], "host_receive_effective_delay")
        self.assertEqual(self.model["prediction_scope"], "attitude_command_only")
        self.assertEqual(len(self.model["source_provenance"]["sample_sha256"]), 64)
        json.dumps(self.model, allow_nan=False)

    def test_epoch_relative_clocks_retain_first_delayed_command(self):
        trial = build_tilt_trials(self.training)[0].trial
        self.assertEqual(trial.times[0], 0.)
        self.assertAlmostEqual(trial.command_times[0], -.007, delta=2e-7)
        self.assertAlmostEqual(trial.phase_times["accelerate"], .213, delta=2e-7)
        self.assertLess(trial.times[-1], 2.)
        self.assertEqual(self.model["data_ranges"][0]["phase_sample_counts"][PHASES[0]], 22)
        self.assertIn("observed_phase_last_sample_s", self.model["data_ranges"][0])

    def test_direction_projection_and_world_bias_are_not_unsigned(self):
        plus, minus = build_tilt_trials(self.training)
        self.assertGreater(plus.angle[0], 0)
        self.assertLess(minus.angle[0], 0)
        np.testing.assert_allclose(plus.trial.commands, minus.trial.commands)
        self.assertEqual(plus.trial.direction.tolist(), [0., 1.])
        self.assertEqual(minus.trial.direction.tolist(), [0., -1.])
        self.assertAlmostEqual(plus.rate[0], 0., delta=1e-12)

    def test_pending_preanchor_command_matches_independent_ode(self):
        model = known_model()
        model["attitude_fit"] = dict(FIT, delay_s=.08, bias_world_y_rad=0.)
        command_history = [dict(time_s=.5, tilt_rad=0.), dict(time_s=.97, tilt_rad=.12),
                           dict(time_s=1.2, tilt_rad=0.)]
        state = dict(time_s=1., position_m=.2, velocity_m_s=.4,
                     theta_rad=0., omega_rad_s=0., direction_y=-1.)
        times = np.linspace(1., 1.6, 121)
        expected = independent_plant(times, [.5, .97, 1.2], [0., .12, 0.],
                                     [0., 0., .4, .2], -1., model["attitude_fit"])
        actual = predict_trajectory(model, state, command_history, times)
        np.testing.assert_allclose(actual["theta"], expected[:, 0], atol=2e-9)
        np.testing.assert_allclose(actual["v"], expected[:, 2], atol=3e-5)
        np.testing.assert_allclose(actual["x"], expected[:, 3], atol=3e-5)
        np.testing.assert_allclose(np.array(actual["theta"])[times < 1.05], 0., atol=1e-14)
        self.assertGreater(max(actual["theta"]), .1)
        self.assertTrue(actual["conditional_on_declared_future_commands"])

    def test_prediction_is_not_sensitive_to_output_sampling_density(self):
        state = dict(time_s=0., position_m=.2, velocity_m_s=.3,
                     theta_rad=.02, omega_rad_s=.2, direction_y=1.)
        history = [dict(time_s=-.01, tilt_rad=-.1)]
        dense = predict_trajectory(known_model(), state, history, np.linspace(0., .4, 101))
        sparse = predict_trajectory(known_model(), state, history, [.4])
        for key in ("theta", "v", "x"):
            self.assertAlmostEqual(dense[key][-1], sparse[key][-1], delta=3e-6)
        self.assertFalse(sparse["conditional_on_declared_future_commands"])

    def test_zero_horizon_and_empty_command_history(self):
        state = dict(time_s=100., position_m=.2, velocity_m_s=.3,
                     theta_rad=.02, omega_rad_s=.2, direction_y=1.)
        prediction = predict_trajectory(known_model(), state, [], [100.])
        self.assertEqual(prediction["x"], [.2])
        self.assertEqual(prediction["v"], [.3])
        self.assertEqual(prediction["theta"], [.02])

    def test_predictor_clock_origin_shift_preserves_pending_history(self):
        initial = dict(time_s=1., position_m=.2, velocity_m_s=.3,
                       theta_rad=.02, omega_rad_s=.2, direction_y=-1.)
        history = [dict(time_s=.98, tilt_rad=-.1), dict(time_s=1.2, tilt_rad=0.)]
        times = np.linspace(1., 1.5, 51)
        expected = predict_trajectory(known_model(), initial, history, times)
        epoch = 1_788_000_000.
        translated = predict_trajectory(known_model(), dict(initial, time_s=initial["time_s"]+epoch),
            [dict(command, time_s=command["time_s"]+epoch) for command in history], times+epoch)
        for key in ("theta", "v", "x"):
            np.testing.assert_allclose(translated[key], expected[key], atol=3e-7)

    def test_no_future_state_leakage_in_prediction(self):
        from Interaction.online_dynamics_model import _predict_item
        from Interaction.braking_split_diagnostic import brake_anchor
        item = build_tilt_trials(self.validation)[0]
        anchor = brake_anchor(item)
        expected = _predict_item(self.model, item, anchor)
        changed = copy.deepcopy(item)
        changed.angle[anchor+1:] = 1.5
        changed.rate[anchor+1:] = -999.
        changed.trial.positions[anchor+1:] = -888.
        changed.trial.velocities[anchor+1:] = 777.
        changed.trial.attitude_acceleration[anchor+1:] = -666.
        self.assertEqual(_predict_item(self.model, changed, anchor), expected)

    def test_heldout_evaluation_reports_all_horizons_and_signed_errors(self):
        before = json.dumps(self.model, sort_keys=True, allow_nan=False)
        report = evaluate_predictive_model(self.model, self.validation)
        self.assertEqual(before, json.dumps(self.model, sort_keys=True, allow_nan=False))
        self.assertEqual(report["validation_segment_ids"], [2, 3])
        self.assertEqual(report["train_segment_ids"], [0, 1])
        self.assertTrue(report["validation_contains_opposed_pair"])
        self.assertFalse(report["future_measurements_used_for_prediction"])
        self.assertFalse(report["deployment_approved"])
        self.assertEqual(report["evaluation_scope"], "conditional_on_executed_command_schedule")
        for row in report["per_trial"]:
            self.assertLess(row["theta_rmse_deg"], 1e-4)
            self.assertLess(row["velocity_rmse_m_s"], .003)
            self.assertLess(abs(row["terminal_error_m_s"]), .003)
            self.assertLess(abs(row["end_position_error_m"]), .003)
            self.assertEqual(row["actual_reverse"], row["predicted_reverse"])
            self.assertIsNotNone(row["actual_crossing"])
            self.assertIsNotNone(row["predicted_crossing"])
        self.assertEqual([r["horizon_s"] for r in report["rolling_horizon_aggregates"]], [.05, .1, .2, .4])
        self.assertTrue(all(r["forecast_count"] > 0 for r in report["rolling_horizon_aggregates"]))
        json.dumps(report, allow_nan=False)

    def test_train_validation_overlap_rejected_before_evaluation(self):
        with self.assertRaisesRegex(ValueError, "overlap"):
            evaluate_predictive_model(self.model, self.training+self.validation)

    def test_motion_gain_fit_never_uses_heldout_data(self):
        with mock.patch("Interaction.online_dynamics_model.motion_gain", wraps=__import__(
                "Interaction.braking_split_diagnostic", fromlist=["motion_gain"]).motion_gain) as fitted:
            fit_predictive_model(self.training)
        self.assertEqual([item.trial.segment for item in fitted.call_args.args[0]], [0, 1])

    def test_training_requires_opposed_complete_pair(self):
        for ids in ((0,), (0, 2)):
            with self.assertRaisesRegex(ValueError, "opposed"):
                fit_predictive_model(synthetic_samples(ids))

    def test_incomplete_phases_rejected(self):
        with self.assertRaisesRegex(ValueError, "phases"):
            build_tilt_trials([row for row in self.training if row["phase"] != "level_after_brake"])
        with self.assertRaisesRegex(ValueError, "phase"):
            build_tilt_trials([row for row in self.training if row["phase"] != "brake"])

    def test_missing_rate_position_attitude_battery_rejected(self):
        for field in ("angular_velocity_rad_s", "position_xy", "actual_attitude_rpy_rad", "battery_voltage_V"):
            broken = copy.deepcopy(self.training)
            del broken[20][field]
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, field):
                build_tilt_trials(broken)

    def test_nonfinite_and_invalid_direction_rejected(self):
        cases = (("timestamp", float("nan")), ("battery_voltage_V", float("inf")),
                 ("angular_velocity_rad_s", [0., float("nan"), 0.]),
                 ("velocity_xy", [0., float("inf")]), ("position_xy", [0., float("nan")]),
                 ("actual_attitude_rpy_rad", [float("nan"), 0., 0.]),
                 ("direction_xy", [1., 0.]), ("state_group_skew_s", .031))
        for field, value in cases:
            broken = copy.deepcopy(self.training)
            broken[20][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                build_tilt_trials(broken)

    def test_large_gap_and_duplicate_nonmonotone_clocks_rejected(self):
        with self.assertRaisesRegex(ValueError, "gap"):
            build_tilt_trials(self.training[:3]+self.training[12:])
        broken = copy.deepcopy(self.training)
        broken[10]["timestamp"] = broken[9]["timestamp"]
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            build_tilt_trials(broken)
        broken[10]["timestamp"] -= .1
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            build_tilt_trials(broken)

    def test_scheduled_times_cannot_replace_actual_send_times(self):
        for mode in ("future", "changed", "previous", "old_origin"):
            broken = copy.deepcopy(self.training)
            if mode == "future":
                broken[0]["command_started_at"] = broken[0]["timestamp"]+.02
            elif mode == "changed":
                broken[10]["command_started_at"] += .001
            elif mode == "previous":
                for row in broken:
                    if row["phase"] == "accelerate":
                        row["command_started_at"] -= .10
            else:
                for row in broken:
                    row["command_started_at"] -= 1_788_000_000.
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                build_tilt_trials(broken)

    def test_command_value_change_and_wrong_brake_sign_rejected(self):
        for command in ([0., 1.], [1., 0.]):
            broken = copy.deepcopy(self.training)
            index = next(i for i, row in enumerate(broken) if row["phase"] == "brake")
            broken[index]["command_acceleration_xy"] = command
            with self.subTest(command=command), self.assertRaises(ValueError):
                build_tilt_trials(broken)

    def test_recovery_rows_are_not_used_as_prediction_targets(self):
        samples = copy.deepcopy([row for row in self.training if row["segment_id"] == 0])
        samples.append(dict(segment_id=0, phase="recovery", timestamp=samples[-1]["timestamp"]+.02))
        item = build_tilt_trials(samples)[0]
        self.assertEqual(len(item.trial.times), len(samples)-1)
        samples.append(copy.deepcopy(samples[-2]))
        samples[-1]["timestamp"] = samples[-2]["timestamp"]+.01
        with self.assertRaisesRegex(ValueError, "following recovery"):
            build_tilt_trials(samples)

    def test_bound_active_is_a_review_candidate_not_flight_approval(self):
        from Interaction.braking_split_diagnostic import tilt_prediction
        bounded_fit = dict(self.model["attitude_fit"], delay_s=.15)
        cost = 0.
        items = build_tilt_trials(self.training)
        for item in items:
            dt = np.diff(item.trial.times)
            weight = np.r_[dt[0]/2, (dt[:-1]+dt[1:])/2, dt[-1]/2]
            weight /= weight.sum()*len(items)
            cost += float(np.sum((tilt_prediction(item, bounded_fit)-item.angle)**2*weight))
        bounded_fit.update(train_rmse_deg=float(np.degrees(np.sqrt(cost))), seed_costs=[cost]*3)
        with mock.patch("Interaction.online_dynamics_model.fit_tilt",
                        return_value=bounded_fit):
            result = fit_predictive_model(self.training)
        self.assertIn("delay_s", result["identifiability"]["bound_active_parameters"])
        self.assertEqual(result["candidate_status"], "requires_identifiability_review")
        self.assertFalse(result["deployment_approved"])

    def test_nonfinite_fit_metadata_cannot_be_sanitized_into_a_candidate(self):
        for key, value in (("train_rmse_deg", float("nan")), ("seed_costs", [0., float("inf")]),
                           ("gain", float("nan")), ("train_rmse_deg", 100.)):
            broken = dict(self.model["attitude_fit"], **{key: value})
            with self.subTest(key=key, value=value), mock.patch(
                    "Interaction.online_dynamics_model.fit_tilt", return_value=broken), self.assertRaises(ValueError):
                fit_predictive_model(self.training)

    def test_numeric_warnings_remain_visible_after_finite_objective_check(self):
        def warning_fit(*args):
            warnings.warn("overflow encountered in matmul", RuntimeWarning)
            return self.model["attitude_fit"]
        with mock.patch("Interaction.online_dynamics_model.fit_tilt", side_effect=warning_fit):
            fitted = fit_predictive_model(self.training)
        diagnostic = fitted["numerical_diagnostics"]
        self.assertTrue(diagnostic["finite_fit_metadata"])
        self.assertTrue(diagnostic["residual_objective_independently_checked"])
        self.assertEqual(diagnostic["optimizer_runtime_warnings"], ["overflow encountered in matmul"])

    def test_insufficient_excitation_is_unidentifiable_not_flight_approval(self):
        samples = copy.deepcopy(self.training)
        for row in samples:
            for key in ("command_acceleration_xy", "actual_attitude_rpy_rad", "angular_velocity_rad_s",
                        "velocity_xy", "position_xy"):
                row[key] = [value*1e-7 for value in row[key]]
        result = fit_predictive_model(samples)
        self.assertFalse(result["identifiability"]["identifiable"])
        self.assertEqual(result["candidate_status"], "requires_identifiability_review")
        self.assertFalse(result["runtime_enabled"])
        self.assertFalse(result["deployment_approved"])

    def test_prediction_rejects_nonfinite_invalid_schedule_and_model(self):
        state = dict(time_s=1., position_m=.2, velocity_m_s=.3,
                     theta_rad=.02, omega_rad_s=.2, direction_y=1.)
        cases = ((known_model(), dict(state, omega_rad_s=float("nan")), [], [1.]),
                 (known_model(), state, [], [.9, 1.]),
                 (known_model(), state, [], [1., 1.]),
                 (known_model(), state, [dict(time_s=1., tilt_rad=0.), dict(time_s=.9, tilt_rad=0.)], [1.]),
                 (dict(known_model(), motion_gain=float("inf")), state, [], [1.]),
                 (dict(known_model(), attitude_fit=dict(FIT, delay_s=-.01)), state, [], [1.]))
        for model, initial, history, times in cases:
            with self.subTest(initial=initial, times=times), self.assertRaises(ValueError):
                predict_trajectory(model, initial, history, times)


if __name__ == "__main__":
    unittest.main()
