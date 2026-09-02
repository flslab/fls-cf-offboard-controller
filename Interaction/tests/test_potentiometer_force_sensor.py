import unittest
from types import SimpleNamespace
import math

import numpy as np

from Interaction.interactions import InteractionsControl
from Interaction.potentiometer_force_sensor import (
    PotentiometerContactDetector,
    PotentiometerForceSensor,
    PotentiometerReleaseDetector,
    parse_potentiometer_line,
)
from Interaction.rpi_power_monitor import parse_get_throttled_output


class PotentiometerForceSensorParsingTest(unittest.TestCase):
    def test_parses_arduino_csv_and_computes_hooke_force(self):
        sample = parse_potentiometer_line(
            b"1234,925,925.25,4.522,7.100\n",
            spring_constant_n_per_mm=0.16,
            host_time=10.0,
        )

        self.assertIsNotNone(sample)
        self.assertEqual(sample.host_time, 10.0)
        self.assertEqual(sample.arduino_time_ms, 1234)
        self.assertEqual(sample.raw, 925)
        self.assertAlmostEqual(sample.filtered_raw, 925.25)
        self.assertIsNone(sample.supply_voltage_v)
        self.assertAlmostEqual(sample.compression_mm, 7.1)
        self.assertAlmostEqual(sample.length_mm, 3.3)
        self.assertAlmostEqual(sample.force_n, 1.136)

    def test_parses_optional_arduino_supply_voltage(self):
        sample = parse_potentiometer_line(
            "1234,925,925.25,4.522,7.100,4.873",
            spring_constant_n_per_mm=0.16,
        )

        self.assertIsNotNone(sample)
        self.assertAlmostEqual(sample.supply_voltage_v, 4.873)
        self.assertAlmostEqual(sample.compression_mm, 7.1)
        self.assertAlmostEqual(sample.length_mm, 3.3)
        self.assertAlmostEqual(sample.force_n, 1.136)

    def test_zero_compression_has_full_length_and_zero_force(self):
        sample = parse_potentiometer_line(
            "1234,1023,1023.0,4.000,0.000",
            spring_constant_n_per_mm=0.16,
            max_extension_mm=10.4,
        )

        self.assertIsNotNone(sample)
        self.assertAlmostEqual(sample.compression_mm, 0.0)
        self.assertAlmostEqual(sample.length_mm, 10.4)
        self.assertAlmostEqual(sample.force_n, 0.0)

    def test_full_compression_has_zero_length_and_maximum_force(self):
        sample = parse_potentiometer_line(
            "1234,2,2.0,0.010,10.400",
            spring_constant_n_per_mm=0.16,
            max_extension_mm=10.4,
        )

        self.assertIsNotNone(sample)
        self.assertAlmostEqual(sample.compression_mm, 10.4)
        self.assertAlmostEqual(sample.length_mm, 0.0)
        self.assertAlmostEqual(sample.force_n, 1.664)

    def test_firmware_fifth_column_is_compression_not_length(self):
        sample = parse_potentiometer_line(
            "63900,1022,1022.0,3.960,0.036,3.962",
            spring_constant_n_per_mm=0.16,
            max_extension_mm=10.4,
        )

        self.assertIsNotNone(sample)
        self.assertAlmostEqual(sample.compression_mm, 0.036)
        self.assertAlmostEqual(sample.length_mm, 10.364)
        self.assertAlmostEqual(sample.force_n, 0.00576)

    def test_live_force_info_log_is_rate_limited(self):
        sample = parse_potentiometer_line(
            "1234,925,925.25,4.522,7.100,4.873",
            spring_constant_n_per_mm=0.16,
            host_time=10.0,
        )
        sensor = PotentiometerForceSensor(info_log_interval_s=0.1)

        with self.assertLogs(
                'Interaction.potentiometer_force_sensor', level='INFO') as logs:
            self.assertTrue(sensor._log_sample_info(sample, monotonic_time=1.0))
            self.assertFalse(sensor._log_sample_info(sample, monotonic_time=1.05))
            self.assertTrue(sensor._log_sample_info(sample, monotonic_time=1.10))

        self.assertEqual(len(logs.output), 2)
        self.assertIn('Potentiometer force=1.136 N', logs.output[0])
        self.assertIn('compression=7.100 mm', logs.output[0])
        self.assertIn('length=3.300 mm', logs.output[0])
        self.assertIn('Vcc=4.873 V', logs.output[0])

    def test_ignores_header_and_invalid_rows(self):
        self.assertIsNone(parse_potentiometer_line(
            "time_ms,raw,filtered,voltage,compression_mm"
        ))
        self.assertIsNone(parse_potentiometer_line("1,2,3"))
        self.assertIsNone(parse_potentiometer_line("1,1024,3,4,5"))
        self.assertIsNone(parse_potentiometer_line("1,500,500,2.4,-1"))

    def test_builds_axis_aligned_fresh_comparison_fields(self):
        sample = parse_potentiometer_line(
            "1234,925,925.25,4.522,7.100",
            spring_constant_n_per_mm=0.16,
            host_time=10.0,
        )
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.force_sensor = SimpleNamespace(
            latest=lambda: sample,
            spring_constant_n_per_mm=0.16,
        )
        controller.sense_axis = 'y'
        controller.sense_axis_index = 1
        controller.sense_sign = -1
        controller.sense_max_age_s = 0.25
        estimate = SimpleNamespace(
            external_force=[0.5, -0.2, 0.0],
            orientation_rpy=[0.0, 0.0, 0.0],
        )

        fields = controller._force_sensor_log_fields(estimate, now=10.05)

        self.assertTrue(fields['force_sensor_fresh'])
        np.testing.assert_allclose(
            fields['force_sensor_external_force_body_N'],
            [0.0, -1.136, 0.0],
        )
        np.testing.assert_allclose(
            fields['force_sensor_external_force_N'], [0.0, -1.136, 0.0]
        )
        self.assertAlmostEqual(
            fields['estimated_external_force_along_sensor_N'], -0.2
        )
        self.assertAlmostEqual(fields['force_sensor_estimate_error_N'], 0.936)
        self.assertAlmostEqual(fields['force_sensor_compression_mm'], 7.1)
        self.assertAlmostEqual(fields['force_sensor_length_mm'], 3.3)
        self.assertNotIn('force_sensor_distance_mm', fields)

        stale = controller._force_sensor_log_fields(estimate, now=10.5)
        self.assertFalse(stale['force_sensor_fresh'])
        self.assertIsNone(stale['force_sensor_estimate_error_N'])

    def test_rotates_body_sensor_axis_into_world_frame(self):
        sample = parse_potentiometer_line(
            "1234,925,925.25,4.522,7.100",
            spring_constant_n_per_mm=0.16,
            host_time=10.0,
        )
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.force_sensor = SimpleNamespace(latest=lambda: sample)
        controller.sense_axis = 'x'
        controller.sense_axis_index = 0
        controller.sense_sign = 1
        controller.sense_max_age_s = 0.25
        estimate = SimpleNamespace(
            external_force=[0.0, 1.0, 0.0],
            orientation_rpy=[0.0, 0.0, math.pi / 2.0],
        )

        fields = controller._force_sensor_log_fields(estimate, now=10.05)

        for actual, expected in zip(
                fields['force_sensor_external_force_N'], [0.0, 1.136, 0.0]):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(
            fields['estimated_external_force_along_sensor_N'], 1.0
        )

    def test_logs_arduino_and_rpi_supply_health(self):
        sample = parse_potentiometer_line(
            "1234,925,925.25,4.400,7.100,4.880",
            host_time=10.0,
        )
        power_sample = parse_get_throttled_output(
            "throttled=0x10001", host_time=9.9
        )
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.force_sensor = SimpleNamespace(
            latest=lambda: sample,
            rpi_power_monitor=SimpleNamespace(latest=lambda: power_sample),
        )
        controller.sense_axis = 'y'
        controller.sense_axis_index = 1
        controller.sense_sign = 1
        controller.sense_max_age_s = 0.25
        estimate = SimpleNamespace(
            external_force=[0.0, 0.0, 0.0],
            orientation_rpy=[0.0, 0.0, 0.0],
        )

        fields = controller._force_sensor_log_fields(estimate, now=10.05)

        self.assertAlmostEqual(fields['force_sensor_supply_voltage_V'], 4.88)
        self.assertEqual(fields['rpi_power_flags_hex'], '0x10001')
        self.assertTrue(fields['rpi_under_voltage_now'])
        self.assertTrue(fields['rpi_under_voltage_occurred'])
        self.assertAlmostEqual(fields['rpi_power_sample_age_s'], 0.15)

    def test_fresh_sensor_force_is_selected_only_for_release_braking(self):
        estimate = SimpleNamespace(external_force=[0.3, 0.4, 0.0])
        sensor_fields = {
            'force_sensor_fresh': True,
            'force_sensor_external_force_N': [0.0, 0.8, 0.0],
        }

        force, source = InteractionsControl._release_braking_force(
            estimate, sensor_fields, sensor_enabled=True
        )

        np.testing.assert_allclose(force, [0.0, 0.8, 0.0])
        self.assertEqual(source, 'potentiometer_force_sensor')

    def test_stale_sensor_falls_back_to_observer_for_release_braking(self):
        estimate = SimpleNamespace(external_force=[0.3, 0.4, 0.0])
        sensor_fields = {
            'force_sensor_fresh': False,
            'force_sensor_external_force_N': [0.0, 0.8, 0.0],
        }

        force, source = InteractionsControl._release_braking_force(
            estimate, sensor_fields, sensor_enabled=True
        )

        np.testing.assert_allclose(force, estimate.external_force)
        self.assertEqual(source, 'wrench_observer')

    def test_release_detector_uses_last_force_before_compression_drop(self):
        detector = PotentiometerReleaseDetector(
            force_drop_n=0.01,
            decrease_rate_n_s=0.05,
            unloaded_force_n=0.05,
            unloaded_dwell_s=0.04,
        )
        detector.arm(0.80, 1.00)
        self.assertFalse(detector.update(0.81, 1.02).released)

        decision = detector.update(0.79, 1.04)

        self.assertFalse(decision.released)
        self.assertTrue(decision.candidate_started)
        self.assertTrue(decision.candidate_active)
        self.assertAlmostEqual(decision.last_force_n, 0.81)
        self.assertAlmostEqual(decision.force_drop_n, 0.02)
        self.assertLess(decision.force_rate_n_s, 0.0)
        self.assertAlmostEqual(decision.pre_release_force_n, 0.81)

        self.assertFalse(detector.update(0.04, 1.06).released)
        decision = detector.update(0.03, 1.11)

        self.assertTrue(decision.released)
        self.assertFalse(decision.candidate_active)
        self.assertGreaterEqual(decision.unloaded_elapsed_s, 0.04)

    def test_contact_detector_requires_baseline_then_sustained_compression(self):
        detector = PotentiometerContactDetector(
            force_threshold_n=0.08,
            onset_dwell_s=0.03,
        )

        self.assertFalse(detector.update(0.20, 1.00).started)
        self.assertFalse(detector.ready)
        self.assertFalse(detector.update(0.01, 1.02).started)
        self.assertTrue(detector.ready)
        self.assertFalse(detector.update(0.10, 1.04).started)
        decision = detector.update(0.40, 1.07)

        self.assertTrue(decision.started)
        self.assertTrue(decision.active)
        self.assertAlmostEqual(decision.peak_force_n, 0.40)

        detector.mark_released()

        self.assertFalse(detector.active)
        self.assertFalse(detector.update(0.40, 1.09).started)

    def test_release_detector_can_inherit_precontact_peak(self):
        detector = PotentiometerReleaseDetector(
            force_drop_n=0.04,
            decrease_rate_n_s=0.05,
            unloaded_force_n=0.05,
            unloaded_dwell_s=0.02,
        )
        detector.arm(0.80, 1.00, peak_force_n=1.60)

        # The inherited 1.60 N contact peak is diagnostic only. A tiny local
        # decrease must not trigger release.
        decision = detector.update(0.79, 1.02)
        self.assertFalse(decision.candidate_active)

        decision = detector.update(0.70, 1.04)
        self.assertTrue(decision.candidate_active)
        self.assertFalse(decision.released)
        self.assertAlmostEqual(decision.pre_release_force_n, 0.80)
        self.assertAlmostEqual(decision.peak_force_n, 1.60)

        self.assertFalse(detector.update(0.04, 1.06).released)
        decision = detector.update(0.03, 1.09)
        self.assertTrue(decision.released)
        self.assertAlmostEqual(decision.peak_force_n, 1.60)
        # Release drop is relative to the local onset edge (0.80 N), while
        # peak_force_n retains the inherited contact peak for diagnostics.
        self.assertAlmostEqual(decision.force_drop_n, 0.77)

    def test_release_detector_ignores_small_force_noise(self):
        detector = PotentiometerReleaseDetector(force_drop_n=0.01)
        detector.arm(0.80, 1.00)

        self.assertFalse(detector.update(0.795, 1.02).released)
        self.assertFalse(detector.update(0.802, 1.04).released)

    def test_release_candidate_cancels_when_force_recovers(self):
        detector = PotentiometerReleaseDetector(
            force_drop_n=0.04,
            unloaded_force_n=0.05,
            unloaded_dwell_s=0.05,
        )
        detector.arm(0.80, 1.00)

        decision = detector.update(0.74, 1.02)
        self.assertTrue(decision.candidate_started)
        self.assertTrue(decision.candidate_active)

        decision = detector.update(0.79, 1.04)
        self.assertTrue(decision.candidate_cancelled)
        self.assertFalse(decision.candidate_active)
        self.assertFalse(decision.released)
        self.assertEqual(decision.candidate_cancel_reason, 'force_rebound')

    def test_release_onset_ignores_old_peak_after_slow_unload(self):
        detector = PotentiometerReleaseDetector(
            force_drop_n=0.04,
            decrease_rate_n_s=0.05,
        )
        detector.arm(1.0, 0.0)
        force_n = 1.0
        timestamp = 0.0
        for _ in range(100):
            force_n -= 0.001
            timestamp += 0.10
            self.assertFalse(
                detector.update(force_n, timestamp).candidate_active
            )

        # This final edge is fast, but only 0.005 N deep. It must not inherit
        # the 1.0 N arm peak and masquerade as a 0.04 N release edge.
        decision = detector.update(force_n - 0.005, timestamp + 0.01)
        self.assertFalse(decision.candidate_started)
        self.assertFalse(decision.candidate_active)
        self.assertAlmostEqual(decision.force_drop_n, 0.005)

    def test_release_candidate_stall_cancels_and_can_resume_contact(self):
        detector = PotentiometerReleaseDetector(
            force_drop_n=0.04,
            unloaded_force_n=0.05,
            candidate_stall_timeout_s=0.50,
        )
        detector.arm(0.80, 1.00)
        self.assertTrue(detector.update(0.70, 1.02).candidate_active)

        decision = None
        for timestamp in (1.12, 1.22, 1.32, 1.42, 1.52):
            decision = detector.update(0.70, timestamp)

        self.assertTrue(decision.candidate_cancelled)
        self.assertEqual(
            decision.candidate_cancel_reason, 'no_downward_progress'
        )
        self.assertFalse(decision.candidate_active)
        self.assertGreaterEqual(decision.candidate_elapsed_s, 0.50)

    def test_release_candidate_rebound_is_relative_to_candidate_minimum(self):
        detector = PotentiometerReleaseDetector(
            force_drop_n=0.04,
            unloaded_force_n=0.05,
        )
        detector.arm(0.80, 1.00)
        self.assertTrue(detector.update(0.70, 1.02).candidate_active)
        self.assertTrue(detector.update(0.40, 1.04).candidate_active)

        decision = detector.update(0.45, 1.06)

        self.assertTrue(decision.candidate_cancelled)
        self.assertEqual(decision.candidate_cancel_reason, 'force_rebound')

    def test_release_unloaded_dwell_must_be_continuous(self):
        detector = PotentiometerReleaseDetector(
            force_drop_n=0.04,
            unloaded_force_n=0.05,
            unloaded_dwell_s=0.05,
        )
        detector.arm(0.80, 1.00)
        self.assertFalse(detector.update(0.70, 1.02).released)
        self.assertFalse(detector.update(0.04, 1.04).released)
        self.assertFalse(detector.update(0.06, 1.07).released)
        self.assertFalse(detector.update(0.04, 1.09).released)
        decision = detector.update(0.03, 1.15)

        self.assertTrue(decision.released)
        self.assertGreaterEqual(decision.unloaded_elapsed_s, 0.05)

    def test_noise_inside_unloaded_band_does_not_cancel_dwell(self):
        detector = PotentiometerReleaseDetector(
            force_drop_n=0.01,
            unloaded_force_n=0.05,
            unloaded_dwell_s=0.05,
        )
        detector.arm(0.80, 1.00)
        self.assertTrue(detector.update(0.70, 1.02).candidate_active)
        self.assertFalse(detector.update(0.00, 1.04).released)

        decision = detector.update(0.02, 1.10)

        self.assertTrue(decision.released)
        self.assertFalse(decision.candidate_cancelled)

    def test_gap_release_keeps_global_peak_diagnostic_only(self):
        detector = PotentiometerReleaseDetector(
            force_drop_n=0.04,
            unloaded_force_n=0.05,
            max_sample_gap_s=0.10,
        )
        detector.arm(0.80, 1.00, peak_force_n=1.60)

        decision = detector.update(0.03, 1.30)

        self.assertTrue(decision.candidate_started)
        self.assertAlmostEqual(decision.pre_release_force_n, 0.80)
        self.assertAlmostEqual(decision.peak_force_n, 1.60)
        self.assertAlmostEqual(decision.force_drop_n, 0.77)

    def test_repeated_timestamp_and_sample_gap_do_not_advance_unload_dwell(self):
        detector = PotentiometerReleaseDetector(
            force_drop_n=0.04,
            unloaded_force_n=0.05,
            unloaded_dwell_s=0.05,
            max_sample_gap_s=0.10,
        )
        detector.arm(0.80, 1.00)
        self.assertFalse(detector.update(0.70, 1.02).released)
        self.assertFalse(detector.update(0.04, 1.04).released)
        self.assertFalse(detector.update(0.03, 1.04).released)

        # The 0.26 s data gap resets continuous dwell evidence.
        self.assertFalse(detector.update(0.03, 1.30).released)
        self.assertFalse(detector.update(0.03, 1.34).released)
        decision = detector.update(0.03, 1.40)

        self.assertTrue(decision.released)
        self.assertGreaterEqual(decision.unloaded_elapsed_s, 0.05)

    def test_release_during_sample_gap_restarts_dwell_on_recovery(self):
        detector = PotentiometerReleaseDetector(
            force_drop_n=0.04,
            unloaded_force_n=0.05,
            unloaded_dwell_s=0.05,
            max_sample_gap_s=0.10,
        )
        detector.arm(0.80, 1.00)

        decision = detector.update(0.03, 1.30)

        self.assertTrue(decision.candidate_started)
        self.assertTrue(decision.candidate_active)
        self.assertFalse(decision.released)
        self.assertEqual(decision.unloaded_elapsed_s, 0.0)
        self.assertFalse(detector.update(0.03, 1.34).released)
        decision = detector.update(0.03, 1.36)
        self.assertTrue(decision.released)

    def test_non_unloaded_sample_gap_cancels_candidate(self):
        detector = PotentiometerReleaseDetector(
            force_drop_n=0.04,
            unloaded_force_n=0.05,
            max_sample_gap_s=0.10,
        )
        detector.arm(0.80, 1.00)
        self.assertTrue(detector.update(0.70, 1.02).candidate_active)

        decision = detector.update(0.60, 1.30)

        self.assertTrue(decision.candidate_cancelled)
        self.assertEqual(decision.candidate_cancel_reason, 'sample_gap')
        self.assertFalse(decision.candidate_active)

    def test_external_stale_cancel_can_detect_release_after_gap(self):
        detector = PotentiometerReleaseDetector(
            force_drop_n=0.04,
            unloaded_force_n=0.05,
            unloaded_dwell_s=0.05,
            max_sample_gap_s=0.10,
        )
        detector.arm(0.80, 1.00)
        self.assertTrue(detector.update(0.70, 1.02).candidate_active)
        self.assertTrue(detector.cancel_candidate(
            preserve_loaded_evidence=True
        ))
        self.assertFalse(detector.candidate_active)

        recovered = detector.update(0.03, 1.30)

        self.assertTrue(recovered.candidate_started)
        self.assertTrue(recovered.candidate_active)
        self.assertFalse(recovered.released)
        self.assertTrue(detector.update(0.03, 1.36).released)

    def test_timestamp_rollback_cancels_release_candidate(self):
        detector = PotentiometerReleaseDetector(
            force_drop_n=0.04,
            unloaded_force_n=0.05,
        )
        detector.arm(0.80, 1.00)
        self.assertTrue(detector.update(0.70, 1.02).candidate_active)

        decision = detector.update(0.60, 0.50)

        self.assertTrue(decision.candidate_cancelled)
        self.assertEqual(
            decision.candidate_cancel_reason, 'timestamp_rollback'
        )
        self.assertFalse(decision.candidate_active)
        self.assertFalse(decision.released)

if __name__ == "__main__":
    unittest.main()
