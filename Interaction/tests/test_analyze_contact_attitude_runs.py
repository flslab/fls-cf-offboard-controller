from copy import deepcopy
import unittest

from Interaction.analyze_contact_attitude_runs import (
    DEFAULT_MIN_COMPARISON_SAMPLES,
    analyze_three_runs,
)


def row(
        run, mode, *, onboard_shadow=(0.0, 0.0),
        vicon_onboard=None, vicon_shadow=None, forwarded=None,
        strict_time=True, sample_index=0):
    vicon = {
        'position_forwarded_to_onboard_ekf': True,
        'frame_sequence': sample_index,
    }
    if forwarded is not None:
        vicon['orientation_forwarded_to_onboard_ekf'] = forwarded
    comparison = {
        'onboard_minus_shadow_roll_pitch_deg': list(onboard_shadow),
        'vicon_minus_onboard_roll_pitch_deg': vicon_onboard,
        'vicon_minus_shadow_roll_pitch_deg': vicon_shadow,
        'onboard_shadow_mirror_exact': run == 1,
        'vicon_to_onboard_host_availability_skew_s': (
            None if run == 1 else 0.005
        ),
        'vicon_orientation_withheld_from_shadow_estimator': True,
        'vicon_orientation_withheld_from_onboard_estimator': (
            None if forwarded is None else not forwarded
        ),
        'shared_mocap_position_sensor_correlation_remains': True,
        'comparison_time_aligned': True,
        'vicon_capture_timestamp_available': run != 1 and strict_time,
        'vicon_capture_mapped_to_cf_clock': run != 1 and strict_time,
        'comparison_time_basis': (
            None if run == 1 else
            'cf_device_timestamp_exact' if strict_time else
            'host_after_wait_availability_approximation'
        ),
        'comparison_scientifically_valid': run != 1 and strict_time,
    }
    if run != 1 and strict_time:
        comparison['comparison_time_skew_s'] = 0.005
    return {
        'type': 'contact_attitude_shadow',
        'data': {
            'experiment_run': run,
            'mode': mode,
            'shadow_only': True,
            'command_authority': False,
            'command_history_used_for_state_reconstruction': False,
            'valid': True,
            'fatal_reason': None,
            'dropped_packets': 0,
            'skipped_position_packets': 0,
            'contained_failure_count': 0,
            'drain_budget_exceeded_count': 0,
            'release_budget_exceeded_count': 0,
            'strict_position_time_update_count': (
                0 if run == 1 else sample_index + 1
            ),
            'last_strict_position_update_cf_timestamp_ms': (
                None if run == 1 else 1000 + 10 * sample_index
            ),
            'approximate_position_time_update_count': 0,
            'vicon_orientation_used_by_shadow_ekf': False,
            'position_timing_scientifically_valid': (
                run == 1 or strict_time
            ),
            'last_position_timing_basis': (
                None if run == 1 else
                'cf_device_timestamp_exact' if strict_time else
                'host_after_wait_availability_approximation'
            ),
            'shadow_estimate': {
                'phase': 'released' if run == 1 else 'post_release',
                'unwrapped_timestamp_ms': 1000 + 10 * sample_index,
            },
            'release_snapshot': (
                {
                    'position_source': 'onboard_ekf_mirror',
                    'velocity_source': 'onboard_ekf_mirror',
                    'position_m': [0.0, 0.0, 1.0],
                    'onboard_ekf_velocity_m_s': [0.0, 0.2, 0.0],
                    'state_time_s': 9.95,
                    'state_sequence': 123,
                    'release_event_monotonic_s': 100.0,
                    'release_event_time_source': (
                        'force_sensor_candidate_onset_monotonic'
                    ),
                    'release_event_to_state_skew_s': -0.005,
                }
                if run == 1 else {
                    'position_source': (
                        'raw_vicon_tvec_forwarded_to_onboard_ekf'
                    ),
                    'velocity_source': (
                        'onboard_ekf_velocity_common_cf_epoch'
                    ),
                    'position_seed_scientifically_time_aligned': strict_time,
                    'position_seed_timing_basis': (
                        'cf_device_timestamp_exact' if strict_time else
                        'host_after_wait_availability_approximation'
                    ),
                    'position_seed_skew_ms': 0.0,
                    'velocity_seed_skew_ms': 0.0,
                    'cf_timestamp_ms': 1000,
                    'position_seed_cf_timestamp_ms': (
                        1000 if strict_time else None
                    ),
                    'gyro_quaternion_wxyz': [1.0, 0.0, 0.0, 0.0],
                    'initial_position_seed_m': [0.0, 0.0, 1.0],
                    'onboard_ekf_position_at_velocity_epoch_m': [
                        0.0, 0.0, 1.0
                    ],
                    'onboard_ekf_velocity_m_s': [0.0, 0.2, 0.0],
                    'position_m': [0.0, 0.0, 1.0],
                    'release_velocity_m_s': [0.0, 0.2, 0.0],
                    'external_position_seed_m': [0.0, 0.0, 1.0],
                    'post_release_position_observation_source': (
                        'raw_vicon_tvec_position_only'
                    ),
                    'state_seed_packet_sequence': 123,
                    'state_seed_cf_timestamp_ms': 1000,
                    'position_seed_packet_sequence': 122,
                    'release_gyro_packet_sequence': 124,
                    'release_event_monotonic_s': 100.0,
                    'release_event_time_source': (
                        'force_sensor_candidate_onset_monotonic'
                    ),
                    'release_event_to_gyro_skew_s': -0.005,
                }
            ),
            'onboard_ekf': {
                'sequence': sample_index,
                'state_time_s': 10.0 + 0.01 * sample_index,
            },
            'post_release_ekf': (
                None if run == 1 else {
                    'position_update_count': sample_index + 1,
                    'rejected_position_count': 0,
                }
            ),
            'vicon': vicon,
            'comparison': comparison,
        },
    }


def rows(run, *, count=DEFAULT_MIN_COMPARISON_SAMPLES, **overrides):
    mode = 'onboard_mirror' if run == 1 else 'inertial_position'
    defaults = {'forwarded': False} if run == 1 else {}
    if run in (2, 3):
        defaults.update({
            'vicon_onboard': [1.0, -2.0],
            'vicon_shadow': [0.5, -1.0],
            'forwarded': run == 3,
        })
    defaults.update(overrides)
    return [
        row(run, mode, sample_index=index, **defaults)
        for index in range(count)
    ]


class AnalyzeContactAttitudeRunsTests(unittest.TestCase):
    def test_strict_protocol_reports_each_comparison_role(self):
        report = analyze_three_runs(rows(1), rows(2), rows(3))
        self.assertEqual(report['status'], 'READY_FOR_COMPARISON')
        self.assertTrue(report['scientific_gate_passed'])
        self.assertEqual(
            report['runs'][0]['onboard_shadow_mirror_exact_fraction'], 1.0
        )
        self.assertEqual(
            report['runs'][1]['vicon_minus_shadow']['sample_count'],
            DEFAULT_MIN_COMPARISON_SAMPLES,
        )
        self.assertIn(
            'withheld from both estimator updates',
            report['runs'][1]['interpretation'],
        )
        self.assertIn(
            'shared-sensor correlation remains',
            report['runs'][1]['interpretation'],
        )
        self.assertIn(
            'withheld from the shadow estimator update',
            report['runs'][2]['interpretation'],
        )

    def test_run_two_rejects_accidental_full_pose_forwarding(self):
        run2 = rows(2)
        run2[0]['data']['vicon'][
            'orientation_forwarded_to_onboard_ekf'
        ] = True
        report = analyze_three_runs(rows(1), run2, rows(3))
        self.assertEqual(report['status'], 'FAIL')
        self.assertIn(
            'vicon_orientation_route_mismatch',
            report['runs'][1]['failures'],
        )

    def test_command_history_state_reconstruction_is_forbidden(self):
        run2 = rows(2)
        run2[0]['data'][
            'command_history_used_for_state_reconstruction'
        ] = True
        report = analyze_three_runs(rows(1), run2, rows(3))
        self.assertEqual(report['runs'][1]['status'], 'FAIL')
        self.assertIn(
            'command_history_must_not_reconstruct_state',
            report['runs'][1]['failures'],
        )

    def test_missing_command_history_provenance_is_unsupported(self):
        run2 = rows(2)
        for record in run2:
            record['data'].pop(
                'command_history_used_for_state_reconstruction'
            )
        report = analyze_three_runs(rows(1), run2, rows(3))
        self.assertEqual(report['runs'][1]['status'], 'UNSUPPORTED')
        self.assertIn(
            'command_history_state_reconstruction_label_missing',
            report['runs'][1]['unsupported_reasons'],
        )

    def test_release_position_seed_must_match_external_position_sample(self):
        run2 = rows(2)
        for record in run2:
            record['data']['release_snapshot'][
                'external_position_seed_m'
            ] = [0.01, 0.0, 1.0]
        report = analyze_three_runs(rows(1), run2, rows(3))
        self.assertEqual(report['runs'][1]['status'], 'FAIL')
        self.assertIn(
            'release_extpos_position_seed_mismatch',
            report['runs'][1]['failures'],
        )

    def test_run_one_rejects_nonzero_or_nonexact_mirror(self):
        run1 = rows(1)
        run1[0]['data']['comparison'][
            'onboard_shadow_mirror_exact'
        ] = False
        run1[0]['data']['comparison'][
            'onboard_minus_shadow_roll_pitch_deg'
        ] = [0.01, 0.0]
        report = analyze_three_runs(run1, rows(2), rows(3))
        self.assertEqual(report['runs'][0]['status'], 'FAIL')
        self.assertIn(
            'onboard_shadow_mirror_not_exact',
            report['runs'][0]['failures'],
        )

    def test_out_of_window_device_clock_pairs_fail_and_do_not_enter_metrics(self):
        run2 = rows(2)
        for record in run2:
            record['data']['comparison']['comparison_time_skew_s'] = 0.04
        report = analyze_three_runs(
            rows(1), run2, rows(3), max_join_skew_s=0.03
        )
        self.assertEqual(report['runs'][1]['status'], 'FAIL')
        self.assertIsNone(report['runs'][1]['vicon_minus_onboard'])
        self.assertIn(
            'comparison_time_not_aligned', report['runs'][1]['failures']
        )

    def test_small_host_skew_without_capture_clock_is_unsupported(self):
        run2 = rows(2, strict_time=False)
        report = analyze_three_runs(rows(1), run2, rows(3))
        second = report['runs'][1]
        self.assertEqual(second['status'], 'UNSUPPORTED')
        self.assertFalse(second['scientific_gate_passed'])
        self.assertIsNone(second['vicon_minus_onboard'])
        self.assertIn(
            'strict_common_clock_sample_minimum_not_met',
            second['unsupported_reasons'],
        )
        self.assertIn(
            'release_position_seed_not_on_common_cf_clock',
            second['unsupported_reasons'],
        )
        self.assertEqual(
            second['vicon_onboard_join_timing'][
                'host_availability_diagnostic_only'
            ]['sample_count'],
            DEFAULT_MIN_COMPARISON_SAMPLES,
        )
        self.assertTrue(
            second['vicon_onboard_join_timing'][
                'host_availability_diagnostic_only'
            ]['never_used_for_scientific_gate']
        )

    def test_missing_comparison_timing_labels_is_unsupported(self):
        run2 = rows(2)
        for record in run2:
            comparison = record['data']['comparison']
            comparison.pop('comparison_time_aligned')
            comparison.pop('comparison_scientifically_valid')
        report = analyze_three_runs(rows(1), run2, rows(3))
        self.assertEqual(report['runs'][1]['status'], 'UNSUPPORTED')
        self.assertIn(
            'comparison_time_alignment_label_missing',
            report['runs'][1]['unsupported_reasons'],
        )
        self.assertIn(
            'comparison_scientific_validity_label_missing',
            report['runs'][1]['unsupported_reasons'],
        )

    def test_missing_capture_common_clock_labels_is_unsupported(self):
        run2 = rows(2)
        for record in run2:
            comparison = record['data']['comparison']
            comparison.pop('vicon_capture_timestamp_available')
            comparison.pop('vicon_capture_mapped_to_cf_clock')
        report = analyze_three_runs(rows(1), run2, rows(3))
        self.assertEqual(report['runs'][1]['status'], 'UNSUPPORTED')
        self.assertIn(
            'capture_common_clock_labels_missing',
            report['runs'][1]['unsupported_reasons'],
        )

    def test_minimum_post_release_sample_count_is_a_hard_gate(self):
        report = analyze_three_runs(
            rows(1, count=DEFAULT_MIN_COMPARISON_SAMPLES - 1),
            rows(2),
            rows(3),
        )
        first = report['runs'][0]
        self.assertEqual(first['status'], 'UNSUPPORTED')
        self.assertIn(
            'minimum_exact_mirror_samples_not_met',
            first['unsupported_reasons'],
        )

    def test_repeated_frames_and_epochs_do_not_satisfy_minimum(self):
        run2 = rows(2)
        for record in run2:
            record['data']['vicon']['frame_sequence'] = 7
            record['data']['shadow_estimate'][
                'unwrapped_timestamp_ms'
            ] = 1000
        report = analyze_three_runs(rows(1), run2, rows(3))
        second = report['runs'][1]
        self.assertEqual(second['status'], 'UNSUPPORTED')
        self.assertIn(
            'minimum_unique_vicon_frames_not_met',
            second['unsupported_reasons'],
        )
        self.assertIn(
            'minimum_unique_shadow_epochs_not_met',
            second['unsupported_reasons'],
        )

    def test_single_timestamp_outlier_does_not_fake_time_coverage(self):
        run2 = rows(2)
        for index, record in enumerate(run2):
            record['data']['shadow_estimate'][
                'unwrapped_timestamp_ms'
            ] = 1000 + index
        run2[-1]['data']['shadow_estimate'][
            'unwrapped_timestamp_ms'
        ] = 10000
        report = analyze_three_runs(rows(1), run2, rows(3))
        second = report['runs'][1]
        self.assertEqual(second['status'], 'UNSUPPORTED')
        self.assertIn(
            'strict_sample_gap_exceeded', second['unsupported_reasons']
        )

    def test_reversed_strict_rows_fail_source_order_gate(self):
        report = analyze_three_runs(rows(1), rows(2), list(reversed(rows(3))))
        third = report['runs'][2]
        self.assertEqual(third['status'], 'FAIL')
        self.assertIn(
            'strict_samples_not_strictly_monotonic', third['failures']
        )

    def test_reversed_mirror_rows_fail_source_order_gate(self):
        report = analyze_three_runs(list(reversed(rows(1))), rows(2), rows(3))
        first = report['runs'][0]
        self.assertEqual(first['status'], 'FAIL')
        self.assertIn(
            'mirror_samples_not_strictly_monotonic', first['failures']
        )

    def test_repeated_strict_position_epoch_cannot_fake_updates(self):
        run2 = rows(2)
        for record in run2:
            record['data'][
                'last_strict_position_update_cf_timestamp_ms'
            ] = 1001
        report = analyze_three_runs(rows(1), run2, rows(3))
        second = report['runs'][1]
        self.assertEqual(second['status'], 'UNSUPPORTED')
        self.assertIn(
            'minimum_unique_strict_position_epochs_not_met',
            second['unsupported_reasons'],
        )

    def test_run_one_rejects_full_pose_orientation_route(self):
        run1 = rows(1)
        for record in run1:
            record['data']['vicon'][
                'orientation_forwarded_to_onboard_ekf'
            ] = True
        report = analyze_three_runs(run1, rows(2), rows(3))
        first = report['runs'][0]
        self.assertEqual(first['status'], 'FAIL')
        self.assertIn('vicon_orientation_route_mismatch', first['failures'])

    def test_missing_release_snapshot_cannot_be_scientifically_ready(self):
        run2 = rows(2)
        for record in run2:
            record['data'].pop('release_snapshot')
        report = analyze_three_runs(rows(1), run2, rows(3))
        self.assertEqual(report['runs'][1]['status'], 'UNSUPPORTED')
        self.assertIn(
            'release_snapshot_missing',
            report['runs'][1]['unsupported_reasons'],
        )

    def test_mixed_release_state_epochs_fail(self):
        run2 = rows(2)
        for record in run2:
            record['data']['release_snapshot'][
                'position_seed_cf_timestamp_ms'
            ] = 999
        report = analyze_three_runs(rows(1), run2, rows(3))
        self.assertEqual(report['runs'][1]['status'], 'FAIL')
        self.assertIn(
            'release_common_epoch_mismatch', report['runs'][1]['failures']
        )

    def test_release_quaternion_change_within_episode_fails(self):
        run2 = rows(2)
        run2[1]['data']['release_snapshot']['gyro_quaternion_wxyz'] = [
            0.999847695156, 0.017452406437, 0.0, 0.0,
        ]

        report = analyze_three_runs(rows(1), run2, rows(3))

        self.assertEqual(report['runs'][1]['status'], 'FAIL')
        self.assertIn(
            'release_snapshot_changed_within_episode',
            report['runs'][1]['failures'],
        )

    def test_release_initial_state_change_within_episode_fails(self):
        run2 = rows(2)
        run2[1]['data']['release_snapshot'][
            'onboard_ekf_velocity_m_s'
        ] = [0.0, 0.21, 0.0]

        report = analyze_three_runs(rows(1), run2, rows(3))

        self.assertEqual(report['runs'][1]['status'], 'FAIL')
        self.assertIn(
            'release_snapshot_changed_within_episode',
            report['runs'][1]['failures'],
        )

    def test_strict_position_update_count_and_time_coverage_are_hard_gates(self):
        run2 = rows(2)
        for index, record in enumerate(run2):
            record['data']['strict_position_time_update_count'] = 1
            record['data']['post_release_ekf']['position_update_count'] = 1
            record['data']['shadow_estimate'][
                'unwrapped_timestamp_ms'
            ] = 1000 + index
        report = analyze_three_runs(rows(1), run2, rows(3))
        second = report['runs'][1]
        self.assertEqual(second['status'], 'UNSUPPORTED')
        self.assertIn(
            'minimum_strict_position_updates_not_met',
            second['unsupported_reasons'],
        )
        self.assertIn(
            'minimum_strict_time_coverage_not_met',
            second['unsupported_reasons'],
        )

    def test_any_rejected_position_update_fails(self):
        run2 = rows(2)
        run2[0]['data']['post_release_ekf']['rejected_position_count'] = 1
        report = analyze_three_runs(rows(1), run2, rows(3))
        self.assertEqual(report['runs'][1]['status'], 'FAIL')
        self.assertIn(
            'post_release_position_update_rejected',
            report['runs'][1]['failures'],
        )

    def test_any_drop_or_contained_failure_fails(self):
        for field in (
                'dropped_packets', 'skipped_position_packets',
                'contained_failure_count', 'drain_budget_exceeded_count',
                'release_budget_exceeded_count'):
            with self.subTest(field=field):
                run2 = rows(2)
                run2[0]['data'][field] = 1
                report = analyze_three_runs(rows(1), run2, rows(3))
                self.assertEqual(report['runs'][1]['status'], 'FAIL')
                self.assertIn(
                    f'{field}_nonzero', report['runs'][1]['failures']
                )

    def test_any_fatal_reason_fails(self):
        run2 = rows(2)
        run2[0]['data']['fatal_reason'] = 'shadow_queue_overflow'
        report = analyze_three_runs(rows(1), run2, rows(3))
        self.assertEqual(report['runs'][1]['status'], 'FAIL')
        self.assertIn(
            'shadow_fatal_reason_present', report['runs'][1]['failures']
        )

    def test_missing_integrity_counters_is_unsupported(self):
        run2 = rows(2)
        for record in run2:
            record['data'].pop('contained_failure_count')
        report = analyze_three_runs(rows(1), run2, rows(3))
        self.assertEqual(report['runs'][1]['status'], 'UNSUPPORTED')
        self.assertTrue(any(
            reason.startswith('integrity_fields_missing:')
            for reason in report['runs'][1]['unsupported_reasons']
        ))

    def test_shared_sensor_correlation_must_be_disclosed(self):
        run2 = rows(2)
        run2[0]['data']['comparison'][
            'shared_mocap_position_sensor_correlation_remains'
        ] = False
        report = analyze_three_runs(rows(1), run2, rows(3))
        self.assertEqual(report['runs'][1]['status'], 'FAIL')
        self.assertIn(
            'shared_mocap_position_correlation_denied',
            report['runs'][1]['failures'],
        )

    def test_orientation_entering_shadow_is_a_failure(self):
        run3 = deepcopy(rows(3))
        run3[0]['data']['vicon_orientation_used_by_shadow_ekf'] = True
        report = analyze_three_runs(rows(1), rows(2), run3)
        self.assertEqual(report['runs'][2]['status'], 'FAIL')
        self.assertIn(
            'vicon_orientation_entered_shadow_estimator',
            report['runs'][2]['failures'],
        )


if __name__ == '__main__':
    unittest.main()
