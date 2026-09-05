"""Causality and fold-isolation tests for offline motion-residual evaluation."""
import copy
import unittest
from unittest.mock import patch

import numpy as np
from scipy.integrate import cumulative_trapezoid

from Interaction.braking_replay import PHASES, Trial
from Interaction.braking_split_diagnostic import TiltTrial
from Interaction.evaluate_motion_residual import (
    ResidualConfig,
    analyze,
    causal_tilt_forecast,
    forecast_at_anchor,
    leave_pair_out_fits,
    opposed_trial_pairs,
    residual_snapshots,
    rolling_motion_residual,
    summarize,
)


TILT_FIT = dict(
    model="second_order",
    delay_s=0.03,
    wn_rad_s=16.0,
    zeta=0.9,
    gain=1.0,
    bias_world_y_rad=0.0,
)


def zero_tilt_item(segment=0, direction=1.0, duration=0.2, residual=0.6):
    times = np.arange(0.0, 1.001, 0.01)
    velocity = 0.2 + residual * times
    position = cumulative_trapezoid(velocity, times, initial=0.0)
    phase_times = dict(zip(PHASES, [0.0, 0.1, 0.3, 0.45, 0.65, 1.01]))
    trial = Trial(
        segment=segment,
        duration=duration,
        direction=np.array([0.0, direction]),
        times=times,
        positions=position,
        velocities=velocity,
        attitude_acceleration=np.zeros(len(times)),
        command_times=np.array([0.0, 0.1, 0.3, 0.45, 0.65]),
        commands=np.zeros(5),
        phase_times=phase_times,
        max_gap=0.01,
    )
    return TiltTrial(trial=trial, angle=np.zeros(len(times)), rate=np.zeros(len(times)))


def paired_items(pair_count=4):
    result = []
    for pair in range(pair_count):
        duration = 0.16 + 0.04 * pair
        result.extend([
            zero_tilt_item(2 * pair, 1.0, duration),
            zero_tilt_item(2 * pair + 1, -1.0, duration),
        ])
    return result


class MotionResidualEvaluationTests(unittest.TestCase):
    def test_pair_folds_never_train_on_either_held_direction(self):
        items = paired_items()
        with (
            patch("Interaction.evaluate_motion_residual.fit_tilt", return_value=TILT_FIT) as fit,
            patch("Interaction.evaluate_motion_residual.motion_gain", return_value=1.0) as gain,
        ):
            folds = leave_pair_out_fits(items)
        self.assertEqual(len(folds), 4)
        for index, fold in enumerate(folds):
            held = {2 * index, 2 * index + 1}
            self.assertEqual(set(fold["held_segments"]), held)
            self.assertFalse(held & set(fold["training_segments"]))
            self.assertEqual(fold["model_scope"], "direction_specific_components")
            for offset, (label, sign) in enumerate(
                    (("positive_y", 1), ("negative_y", -1))):
                expected = {
                    item.trial.segment for item in items
                    if item.trial.segment not in held
                    and int(item.trial.direction[1]) == sign
                }
                component = fold["directional_models"][label]
                self.assertEqual(set(component["training_segments"]), expected)
                call_index = 2 * index + offset
                self.assertEqual(
                    {item.trial.segment for item in fit.call_args_list[call_index].args[0]},
                    expected,
                )
                self.assertEqual(
                    {item.trial.segment for item in gain.call_args_list[call_index].args[0]},
                    expected,
                )

    def test_pairing_requires_opposed_equal_duration_trials(self):
        items = paired_items(3)
        self.assertEqual(len(opposed_trial_pairs(items)), 3)
        items[1].trial.direction[:] = items[0].trial.direction
        with self.assertRaisesRegex(ValueError, "not opposed"):
            opposed_trial_pairs(items)
        items = paired_items(3)
        items[1].trial.duration += 0.01
        with self.assertRaisesRegex(ValueError, "same pulse duration"):
            opposed_trial_pairs(items)

    def test_prediction_and_residual_do_not_read_future_state_or_commands(self):
        item = zero_tilt_item()
        config = ResidualConfig()
        # The final logged command is still in the future at this anchor.
        anchor = 50
        original = forecast_at_anchor(item, TILT_FIT, 1.0, anchor, 0.2, config)

        changed = copy.deepcopy(item)
        changed.angle[anchor + 1:] = 1000.0
        changed.rate[anchor + 1:] = -1000.0
        changed.trial.velocities[anchor + 1:] = -900.0
        changed.trial.positions[anchor + 1:] = 800.0
        future_commands = changed.trial.command_times > changed.trial.times[anchor]
        changed.trial.commands[future_commands] = 500.0
        after = forecast_at_anchor(changed, TILT_FIT, 1.0, anchor, 0.2, config)

        for key in (
            "predicted_tilt_rad",
            "baseline_velocity_m_s",
            "baseline_position_m",
            "corrected_velocity_m_s",
            "corrected_position_m",
        ):
            np.testing.assert_array_equal(original[key], after[key])
        self.assertEqual(original["residual"], after["residual"])

    def test_already_sent_but_delayed_command_is_not_discarded(self):
        item = zero_tilt_item()
        item.trial.command_times = np.array([0.0, 0.49, 0.8])
        item.trial.commands = np.array([0.0, 3.0, 0.0])
        anchor = 50
        prediction = causal_tilt_forecast(item, TILT_FIT, anchor, np.array([0.0, 0.02, 0.10]))
        self.assertAlmostEqual(prediction[0], 0.0)
        self.assertAlmostEqual(prediction[1], 0.0)
        self.assertGreater(prediction[-1], 0.01)

    def test_bounded_residual_recovers_constant_unmodeled_acceleration(self):
        item = zero_tilt_item(residual=0.6)
        config = ResidualConfig(history_window_s=0.08, maximum_residual_m_s2=1.5)
        anchor = 70
        estimate = rolling_motion_residual(item, 1.0, anchor, config)
        self.assertTrue(estimate["ready"])
        self.assertAlmostEqual(estimate["raw_acceleration_m_s2"], 0.6, places=10)
        self.assertAlmostEqual(estimate["filtered_acceleration_m_s2"], 0.6, places=10)
        self.assertEqual(estimate["sigma_acceleration_m_s2"], 0.15)
        self.assertFalse(estimate["rejected"])
        self.assertFalse(estimate["clipped"])
        forecast = forecast_at_anchor(item, TILT_FIT, 1.0, anchor, 0.2, config)
        velocity_change = (forecast["corrected_velocity_m_s"][-1]
                           - forecast["baseline_velocity_m_s"][-1])
        position_change = (forecast["corrected_position_m"][-1]
                           - forecast["baseline_position_m"][-1])
        self.assertAlmostEqual(velocity_change, 0.6 * 0.08, places=10)
        self.assertAlmostEqual(
            position_change, 0.6 * (0.08 * 0.2 - 0.5 * 0.08 ** 2), places=10
        )
        self.assertAlmostEqual(
            forecast["residual_velocity_uncertainty_m_s"][-1],
            2.0 * 0.15 * 0.08,
            places=10,
        )
        self.assertEqual(forecast["residual_horizon_anchor"],
                         "measured_state_timestamp")
        self.assertAlmostEqual(
            forecast["residual_valid_until_s"],
            item.trial.times[anchor] + 0.08,
        )
        # Replaying the same packet cannot move the absolute validity deadline.
        repeated = forecast_at_anchor(item, TILT_FIT, 1.0, anchor, 0.05, config)
        self.assertEqual(repeated["residual_valid_until_s"],
                         forecast["residual_valid_until_s"])

    def test_residual_outlier_is_rejected_instead_of_clipped(self):
        item = zero_tilt_item(residual=5.0)
        config = ResidualConfig(maximum_residual_m_s2=1.5)
        estimate = rolling_motion_residual(item, 1.0, 51, config)
        self.assertEqual(estimate["status"], "residual_outlier_rejected")
        self.assertAlmostEqual(estimate["raw_acceleration_m_s2"], 5.0, places=10)
        self.assertTrue(estimate["rejected"])
        self.assertFalse(estimate["clipped"])
        self.assertFalse(estimate["ready"])
        self.assertEqual(estimate["filtered_acceleration_m_s2"], 0.0)

    def test_observer_requires_real_60ms_history_and_four_samples(self):
        item = zero_tilt_item()
        snapshots = residual_snapshots(item, 1.0, ResidualConfig())
        self.assertEqual(snapshots[50]["status"], "warming_up")
        self.assertLess(snapshots[50]["window_span_s"], 0.06)
        self.assertTrue(snapshots[51]["ready"])
        self.assertGreaterEqual(snapshots[51]["sample_count"], 4)
        self.assertGreaterEqual(snapshots[51]["window_span_s"], 0.06 - 1e-12)

    def test_observer_resets_on_real_sample_gap_over_40ms(self):
        item = zero_tilt_item()
        keep = ~((item.trial.times > 0.46) & (item.trial.times < 0.52))
        item.trial.times = item.trial.times[keep]
        item.trial.positions = item.trial.positions[keep]
        item.trial.velocities = item.trial.velocities[keep]
        item.trial.attitude_acceleration = item.trial.attitude_acceleration[keep]
        item.angle = item.angle[keep]
        item.rate = item.rate[keep]
        gap_index = int(np.flatnonzero(np.isclose(item.trial.times, 0.52))[0])
        snapshot = residual_snapshots(item, 1.0, ResidualConfig())[gap_index]
        self.assertEqual(snapshot["status"], "sample_gap_reset")
        self.assertFalse(snapshot["ready"])

    def test_summary_reports_velocity_position_and_reversal_errors(self):
        rows = []
        for model, velocity_error, position_error, predicted_reverse in (
            ("baseline", 0.2, 0.03, False),
            ("bounded_residual", 0.05, 0.01, True),
        ):
            rows.append(dict(
                horizon_s=0.05,
                model=model,
                velocity_error_m_s=velocity_error,
                position_error_m=position_error,
                actual_reverse=True,
                predicted_reverse=predicted_reverse,
                conservative_predicted_reverse=predicted_reverse,
                reversal_classification_correct=predicted_reverse,
                terminal_velocity_inside_uncertainty=(model == "bounded_residual"),
                residual_ready=(model == "bounded_residual"),
                residual_rejected=False,
                residual_clipped=False,
            ))
        result = summarize(rows, (0.05,))['0.05']
        self.assertEqual(result["baseline"]["missed_reversal_count"], 1)
        self.assertEqual(result["bounded_residual"]["missed_reversal_count"], 0)
        self.assertGreater(
            result["baseline"]["velocity_rmse_m_s"],
            result["bounded_residual"]["velocity_rmse_m_s"],
        )
        self.assertGreater(
            result["baseline"]["position_rmse_m"],
            result["bounded_residual"]["position_rmse_m"],
        )

    def test_report_is_exploratory_and_selects_directional_components(self):
        items = paired_items(3)
        component = lambda sign: dict(
            direction_y=sign,
            training_segments=[2, 3, 4, 5],
            tilt_fit=TILT_FIT,
            motion_gain=1.0,
        )
        fold = dict(
            held_segments=[0, 1],
            training_segments=[2, 3, 4, 5],
            model_scope="direction_specific_components",
            directional_models={
                "positive_y": component(1),
                "negative_y": component(-1),
            },
        )
        with (
            patch("Interaction.evaluate_motion_residual.extract",
                  return_value=({}, [item.trial for item in items], [], {})),
            patch("Interaction.evaluate_motion_residual.build_trials", return_value=items),
            patch("Interaction.evaluate_motion_residual.leave_pair_out_fits",
                  return_value=[fold]),
        ):
            report = analyze([], progress=lambda *args, **kwargs: None)
        self.assertEqual(report["evaluation_status"],
                         "exploratory_only_not_runtime_eligible")
        self.assertFalse(report["control_eligibility_granted"])
        self.assertFalse(report["eligibility_evaluated"])
        self.assertFalse(report["calibration_writes"])
        self.assertFalse(report["flight_commands"])
        self.assertTrue(report["runtime_semantics"]["real_samples_only"])
        self.assertIn("ready_only_summary", report)
        self.assertIsNone(
            report["summary"]["0.05"]["baseline"][
                "terminal_velocity_uncertainty_coverage"
            ]
        )
        selected = {row["segment"]: row["selected_model_component"]
                    for row in report["forecasts"]}
        self.assertEqual(selected[0], "positive_y")
        self.assertEqual(selected[1], "negative_y")


if __name__ == "__main__":
    unittest.main()
