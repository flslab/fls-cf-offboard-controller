import copy
import math
import unittest

from Interaction.contact_attitude_observer import (
    CF_TIMESTAMP_MODULUS_MS,
    ContactAttitudeConfig,
    quaternion_from_native_rpy,
)
from Interaction.contact_attitude_shadow import ContactAttitudeShadowConfig
from Interaction.contact_attitude_experiment import (
    CONTACT_ATTITUDE_PROTOCOL_VERSION,
    CRAZYSIM_RELEASE_CLOCK_MAPPING_BASIS,
    RELEASE_EVENT_TIME_SOURCE,
)
from Interaction.log_manager import (
    CONTACT_SOURCE_TIMESTAMP_BASIS,
    CRAZYSIM_CF_TIMESTAMP_BASIS,
)
from Interaction.replay_contact_attitude_shadow import (
    ReplayGateConfig,
    analyze_records,
    quaternion_distance_deg,
    tilt_error_deg,
)


def _xyzw(quaternion_wxyz):
    w, x, y, z = quaternion_wxyz
    return [x, y, z, w]


def synthetic_records(*, include_truth=True, bad_release_truth_deg=0.0):
    """Position-only estimator input plus independent full-pose truth."""
    timeline = []
    base_time = 1000.0

    def add(ms, priority, record):
        timeline.append((ms, priority, record))

    protocol_data = {
        "protocol_version": CONTACT_ATTITUDE_PROTOCOL_VERSION,
        "experiment_run": 2,
        "shadow_mode": "inertial_position",
        "vicon_mode": "rigidbody",
        "vicon_orientation_forwarded_to_onboard_ekf": False,
        "alignment_nominal_yaw_deg": 0.0,
    }
    for ms, name in (
        (-2, "Contact Attitude Shadow Prepared Pre-Arm"),
        (-1, "Contact Attitude Shadow Started"),
    ):
        add(ms, -1, {
            "type": "events", "name": name,
            "data": {**protocol_data, "time": base_time + ms / 1000.0},
        })

    for timestamp_ms in range(0, 861):
        host_time = base_time + timestamp_ms / 1000.0
        if timestamp_ms % 10 == 0:
            add(timestamp_ms, 1, {
                "type": "state", "group": "POS_ACC", "data": {
                    "cf_timestamp_ms": timestamp_ms,
                    "host_receive_time_s": host_time,
                    "stateEstimate.x": 0.0,
                    "stateEstimate.y": 0.0,
                    "stateEstimate.z": 1.0,
                    "stateEstimate.ax": 0.0,
                    "stateEstimate.ay": 0.0,
                    "stateEstimate.az": 0.0,
                },
            })
            # These attitude fields are deliberately present. They are onboard
            # EKF diagnostics, not independent quaternion truth.
            add(timestamp_ms, 2, {
                "type": "state", "group": "VEL_ORI", "data": {
                    "cf_timestamp_ms": timestamp_ms,
                    "host_receive_time_s": host_time,
                    "stateEstimate.vx": 0.0,
                    "stateEstimate.vy": 0.0,
                    "stateEstimate.vz": 0.0,
                    "stateEstimate.roll": 0.0,
                    "stateEstimate.pitch": 0.0,
                    "stateEstimate.yaw": 0.0,
                },
            })
            frame_data = {
                "time": host_time,
                "cf_timestamp_ms": timestamp_ms,
                "cf_timestamp_basis": CRAZYSIM_CF_TIMESTAMP_BASIS,
                "cf_timestamp_uncertainty_ms": 0.0,
                "tvec": [0.0, 0.0, 1.0],
                "position_forwarded_to_onboard_ekf": True,
                "orientation_forwarded_to_onboard_ekf": False,
            }
            if include_truth:
                truth_roll = (
                    bad_release_truth_deg if timestamp_ms >= 50 else 0.0
                )
                truth = quaternion_from_native_rpy(
                    math.radians(truth_roll), 0.0, 0.0
                )
                frame_data["quat"] = _xyzw(truth)
            add(timestamp_ms, 4, {
                "type": "frames", "data": frame_data,
            })
        add(timestamp_ms, 5, {
            "type": "state", "group": "GYRO_1KHZ", "data": {
                "cf_timestamp_ms": timestamp_ms,
                "host_receive_time_s": host_time,
                "contactImu.gx": 0.0,
                "contactImu.gy": 0.0,
                "contactImu.gz": 0.0,
                "contactImu.ax": 0.0,
                "contactImu.ay": 0.0,
                "contactImu.az": 1.0,
                "contactImu.px": 0.0,
                "contactImu.py": 0.0,
                "contactImu.pz": 1.0,
                "contactImu.vx": 0.0,
                "contactImu.vy": 0.0,
                "contactImu.vz": 0.0,
                "contactImu.epoch": timestamp_ms & 0xFFFF,
                "transport_cf_timestamp_ms": timestamp_ms,
                "source_cf_timestamp_ms": timestamp_ms,
                "source_cf_timestamp_basis": (
                    CONTACT_SOURCE_TIMESTAMP_BASIS
                ),
                "source_snapshot_atomic": True,
                "source_cf_transport_skew_ms": 0,
            },
        })

    def event(ms, priority, name, **event_data):
        add(ms, priority, {
            "type": "events", "name": name,
            "data": {
                "time": base_time + ms / 1000.0,
                **event_data,
            },
        })

    # Contact begins after stable-hover alignment. Release-candidate cancellation
    # must not reset the physical-contact observer.
    event(21, 9, "Contact Attitude Shadow Contact Candidate Started")
    event(22, 9, "Contact Attitude Shadow Contact Candidate Cancelled")
    event(43, 9, "Contact Attitude Shadow Contact Candidate Started")
    event(44, 9, "Contact Attitude Shadow Contact Confirmed")
    event(45, 9, "Potentiometer Release Candidate Started")
    event(47, 9, "Potentiometer Release Candidate Cancelled")
    event(50, 9, "Potentiometer Release Candidate Started")
    event(
        60, 9, "Contact Attitude Shadow Released",
        cf_timestamp_ms=50,
        release_snapshot={
            "cf_timestamp_ms": 50,
            "release_event_monotonic_s": base_time + 0.050,
            "release_event_time_source": RELEASE_EVENT_TIME_SOURCE,
            "release_event_arduino_time_ms": 50,
            "release_confirmation_monotonic_s": base_time + 0.060,
            "release_confirmation_arduino_time_ms": 60,
            "release_clock_mapping_basis": (
                CRAZYSIM_RELEASE_CLOCK_MAPPING_BASIS
            ),
        },
    )

    records = [record for _ms, _priority, record in sorted(
        timeline, key=lambda item: item[:2]
    )]
    sequence = 0
    for record in records:
        if record.get("type") == "state":
            record["data"]["sequence"] = sequence
            sequence += 1
    return records


def analyze(records, *, experiment_run=None):
    return analyze_records(
        records,
        config=ReplayGateConfig(min_post_release_truth_samples=20),
        observer_config=ContactAttitudeConfig(
            alignment_window_ms=20.0,
            alignment_min_samples=3,
        ),
        shadow_config=ContactAttitudeShadowConfig(
            queue_capacity=8192,
            alignment_stationary_window_ms=20.0,
            alignment_min_state_samples=3,
            experiment_run=experiment_run,
        ),
    )


class ContactAttitudeReplayTests(unittest.TestCase):
    def test_quaternion_metrics_separate_yaw_from_tilt(self):
        identity = quaternion_from_native_rpy(0.0, 0.0, 0.0)
        yaw_only = quaternion_from_native_rpy(0.0, 0.0, math.radians(30.0))
        self.assertAlmostEqual(tilt_error_deg(identity, yaw_only), 0.0)
        self.assertAlmostEqual(
            quaternion_distance_deg(identity, yaw_only), 30.0, places=10
        )

    def test_full_replay_passes_and_candidate_cancel_keeps_contact(self):
        report = analyze(synthetic_records())
        self.assertEqual(report["gate_verdict"], "PASS")
        self.assertTrue(report["passed"])
        self.assertFalse(report["truth_fed_to_filter"])
        self.assertFalse(report["command_history_used_for_state_reconstruction"])
        self.assertEqual(report["lifecycle_counts"]["contact_start"], 2)
        self.assertEqual(report["lifecycle_counts"]["contact_cancel"], 1)
        self.assertEqual(report["lifecycle_counts"]["contact_confirm"], 1)
        self.assertEqual(report["lifecycle_counts"]["candidate_start"], 2)
        self.assertEqual(report["lifecycle_counts"]["candidate_cancel"], 1)
        episode = report["episodes"][0]
        self.assertEqual(episode["gate_verdict"], "PASS")
        self.assertAlmostEqual(episode["release_tilt_error_deg"], 0.0)
        self.assertAlmostEqual(
            episode["post_release_400ms_p95_tilt_error_deg"], 0.0
        )
        self.assertAlmostEqual(
            episode["relative_tilt_drift_at_800ms_deg"], 0.0
        )
        self.assertAlmostEqual(
            report["aggregate_metrics"]["release_tilt_error_max_deg"], 0.0
        )
        self.assertGreaterEqual(
            episode["post_release_400ms_joined_truth_count"], 40
        )
        self.assertEqual(
            episode["release_state_sources"]["velocity"],
            "onboard_ekf_velocity_common_cf_epoch",
        )
        self.assertEqual(
            episode["release_state_sources"]["position"],
            "onboard_ekf_position_common_cf_epoch",
        )
        self.assertEqual(
            episode["final_filter_snapshot"]["position_update_count"], 81
        )
        self.assertEqual(
            episode["gates"]["release_time_provenance"]["status"],
            "PASS",
        )

    def test_firmware_latched_field_names_and_provenance_replay(self):
        records = synthetic_records()
        report = analyze(records, experiment_run=2)

        self.assertEqual(report['gate_verdict'], 'PASS')
        self.assertEqual(
            report['data_quality']['missing_required_packet_field_count'], 0
        )
        self.assertIsNone(report['data_quality']['replay_fatal_reason'])
        self.assertEqual(
            report['data_quality'][
                'invalid_contact_source_provenance_groups'
            ],
            [],
        )

    def test_independent_truth_failure_fails_release_gate(self):
        report = analyze(synthetic_records(bad_release_truth_deg=3.0))
        self.assertEqual(report["gate_verdict"], "FAIL")
        release_gate = report["episodes"][0]["gates"]["release_tilt_error"]
        self.assertEqual(release_gate["status"], "FAIL")
        self.assertAlmostEqual(release_gate["value"], 3.0, places=8)

    def test_onboard_euler_is_never_promoted_to_truth(self):
        report = analyze(synthetic_records(include_truth=False))
        self.assertEqual(report["gate_verdict"], "UNSUPPORTED")
        self.assertIsNone(report["passed"])
        self.assertEqual(report["data_quality"]["truth_sample_count"], 0)
        self.assertIn(
            "withheld_full_pose_orientation_missing",
            report["unsupported_reasons"],
        )
        self.assertEqual(
            report["episodes"][0]["gates"]["release_tilt_error"]["status"],
            "UNSUPPORTED",
        )

    def test_host_only_orientation_truth_cannot_produce_scientific_pass(self):
        records = synthetic_records()
        for record in records:
            if record.get("type") == "frames":
                record["data"].pop("cf_timestamp_ms", None)

        report = analyze(records)

        self.assertEqual(report["gate_verdict"], "UNSUPPORTED")
        self.assertIn(
            "orientation_truth_common_cf_clock_missing",
            report["unsupported_reasons"],
        )
        self.assertEqual(
            report["data_quality"]["common_cf_clock_truth_sample_count"], 0
        )

    def test_numeric_truth_timestamp_without_provenance_is_unsupported(self):
        records = synthetic_records()
        for record in records:
            if record.get("type") == "frames":
                record["data"].pop("cf_timestamp_basis")
                record["data"].pop("cf_timestamp_uncertainty_ms")

        report = analyze(records)

        self.assertEqual(report["gate_verdict"], "UNSUPPORTED")
        self.assertIn(
            "orientation_truth_common_cf_clock_missing",
            report["unsupported_reasons"],
        )

    def test_truth_clock_uncertainty_above_join_limit_is_unsupported(self):
        records = synthetic_records()
        for record in records:
            if record.get("type") == "frames":
                record["data"]["cf_timestamp_uncertainty_ms"] = 1000.0

        report = analyze(records)

        self.assertEqual(report["gate_verdict"], "UNSUPPORTED")
        self.assertEqual(
            report["data_quality"]["common_cf_clock_truth_sample_count"], 0
        )

    def test_missing_protocol_metadata_is_unsupported(self):
        records = [
            record for record in synthetic_records()
            if record.get("name") not in (
                "Contact Attitude Shadow Prepared Pre-Arm",
                "Contact Attitude Shadow Started",
            )
        ]

        report = analyze(records)

        self.assertEqual(report["gate_verdict"], "UNSUPPORTED")
        self.assertIn(
            "contact_attitude_protocol_metadata_missing",
            report["unsupported_reasons"],
        )

    def test_missing_producer_latched_timestamp_is_unsupported(self):
        records = synthetic_records()
        packet = next(
            record for record in records
            if record.get("group") == "GYRO_1KHZ"
        )
        packet["data"]["source_snapshot_atomic"] = False

        report = analyze(records)

        self.assertEqual(report["gate_verdict"], "UNSUPPORTED")
        self.assertIn(
            "producer_latched_contact_timestamp_provenance_missing_or_invalid",
            report["unsupported_reasons"],
        )

    def test_excessive_transport_to_source_skew_is_unsupported(self):
        records = synthetic_records()
        packet = next(
            record for record in records
            if record.get("group") == "GYRO_1KHZ"
        )
        packet["data"]["transport_cf_timestamp_ms"] = 1000
        packet["data"]["source_cf_transport_skew_ms"] = 1000

        report = analyze(records)

        self.assertEqual(report["gate_verdict"], "UNSUPPORTED")
        self.assertIn(
            "producer_latched_contact_timestamp_provenance_missing_or_invalid",
            report["unsupported_reasons"],
        )

    def test_untrusted_release_clock_mapping_is_unsupported(self):
        records = synthetic_records()
        release = next(
            record for record in records
            if record.get("name") == "Contact Attitude Shadow Released"
        )["data"]["release_snapshot"]
        release["release_clock_mapping_basis"] = (
            "arduino_uart_receive_without_calibrated_cf_mapping"
        )

        report = analyze(records)

        self.assertEqual(report["gate_verdict"], "UNSUPPORTED")
        self.assertIn(
            "release_clock_mapping_untrusted",
            report["unsupported_reasons"],
        )

    def test_replay_keeps_wall_and_monotonic_packet_clocks_separate(self):
        records = synthetic_records()
        for record in records:
            data = record.get("data", {})
            if record.get("type") == "state":
                timestamp_ms = data["cf_timestamp_ms"]
                data["host_receive_time_s"] = (
                    1_700_000_000.0 + timestamp_ms / 1000.0
                )
                data["host_receive_monotonic_s"] = (
                    1000.0 + timestamp_ms / 1000.0
                )
            elif record.get("type") == "frames":
                timestamp_ms = data.pop("cf_timestamp_ms")
                data["time"] = 1_700_000_000.0 + timestamp_ms / 1000.0
                data["host_receive_time_s"] = data["time"]
                data["mocap_timing"] = {
                    "wait_return_monotonic_s": (
                        1000.0 + timestamp_ms / 1000.0
                    ),
                }
            elif (
                    record.get("type") == "events"
                    and record.get("name")
                    == "Contact Attitude Shadow Released"):
                data["release_snapshot"][
                    "release_event_monotonic_s"
                ] = 1000.050

        report = analyze(records)

        self.assertEqual(report["gate_verdict"], "UNSUPPORTED")
        self.assertEqual(
            report["data_quality"]["replay_skipped_position_packets"], 0
        )
        episode = report["episodes"][0]
        self.assertEqual(episode["approximate_position_update_count"], 81)
        self.assertEqual(episode["rejected_position_update_count"], 0)
        self.assertEqual(
            episode["gates"]["approximate_position_updates"]["status"],
            "UNSUPPORTED",
        )

    def test_legacy_log_without_raw_cf_timestamps_is_unsupported(self):
        records = synthetic_records()
        for record in records:
            if record.get("group") in ("GYRO_1KHZ", "ACC_ALIGN"):
                record["data"].pop("cf_timestamp_ms")
        report = analyze(records)
        self.assertEqual(report["gate_verdict"], "UNSUPPORTED")
        self.assertIn(
            "raw_Crazyflie_timestamp_missing", report["unsupported_reasons"]
        )

    def test_device_clock_only_replay_handles_24_bit_wrap(self):
        records = synthetic_records()
        offset = CF_TIMESTAMP_MODULUS_MS - 100
        for record in records:
            data = record["data"]
            if record.get("type") == "state":
                raw = data["cf_timestamp_ms"]
                shifted = (offset + raw) % CF_TIMESTAMP_MODULUS_MS
                data["cf_timestamp_ms"] = shifted
                if record.get("group") == "GYRO_1KHZ":
                    data["source_cf_timestamp_ms"] = shifted
                    data["transport_cf_timestamp_ms"] = shifted
                    data["contactImu.epoch"] = shifted & 0xFFFF
                data.pop("host_receive_time_s")
            elif record.get("type") == "frames":
                raw = round((data["time"] - 1000.0) * 1000.0)
                data["cf_timestamp_ms"] = (offset + raw) % CF_TIMESTAMP_MODULUS_MS
                data.pop("time")
            elif record.get("type") == "events":
                data.pop("time")
                if record.get("name") == "Contact Attitude Shadow Released":
                    shifted_release = (
                        offset + 50
                    ) % CF_TIMESTAMP_MODULUS_MS
                    data["cf_timestamp_ms"] = shifted_release
                    release = data["release_snapshot"]
                    release["cf_timestamp_ms"] = shifted_release
                    release["release_event_monotonic_s"] = (
                        offset + 50
                    ) / 1000.0
                    release["release_confirmation_monotonic_s"] = (
                        offset + 60
                    ) / 1000.0
        report = analyze(records)
        self.assertEqual(report["gate_verdict"], "PASS")
        self.assertEqual(
            report["data_quality"]["gyro_stream"]["backward_timestamp_count"],
            0,
        )

    def test_contact_gyro_gap_is_reported_and_fails_filter_gate(self):
        records = synthetic_records()
        records = [
            record for record in records
            if not (
                record.get("group") == "GYRO_1KHZ"
                and 50 <= record["data"]["cf_timestamp_ms"] <= 55
            )
        ]
        report = analyze(records)
        self.assertEqual(report["gate_verdict"], "FAIL")
        self.assertEqual(
            report["data_quality"]["gyro_stream"]["max_cf_gap_ms"], 7.0
        )
        self.assertEqual(
            report["data_quality"]["gyro_stream"][
                "inferred_missing_sample_count"
            ],
            6,
        )
        self.assertEqual(
            report["episodes"][0]["gates"][
                "filter_valid_through_800ms"
            ]["status"],
            "FAIL",
        )

    def test_logged_queue_drop_is_a_failure_even_with_good_truth(self):
        records = synthetic_records()
        records.append({
            "type": "contact_attitude_shadow",
            "data": {"dropped_packets": 1},
        })
        report = analyze(records)
        self.assertEqual(report["gate_verdict"], "FAIL")
        self.assertEqual(
            report["data_quality"]["logged_shadow_dropped_packets"], 1
        )

    def test_any_logged_shadow_integrity_failure_is_not_hidden_by_replay(self):
        for field in (
                "skipped_position_packets",
                "contained_failure_count",
                "drain_budget_exceeded_count",
                "release_budget_exceeded_count"):
            with self.subTest(field=field):
                records = synthetic_records()
                records.append({
                    "type": "contact_attitude_shadow",
                    "data": {field: 1, "fatal_reason": None},
                })
                report = analyze(records)
                self.assertEqual(report["gate_verdict"], "FAIL")
                self.assertEqual(
                    report["data_quality"][f"logged_shadow_{field}"], 1
                )

        records = synthetic_records()
        records.append({
            "type": "contact_attitude_shadow",
            "data": {"fatal_reason": "release_seed_timeout"},
        })
        report = analyze(records)
        self.assertEqual(report["gate_verdict"], "FAIL")
        self.assertEqual(
            report["data_quality"]["logged_shadow_fatal_reasons"],
            ["release_seed_timeout"],
        )

    def test_packet_sequence_gap_is_a_confirmed_failure(self):
        records = synthetic_records()
        state_records = [
            record for record in records if record.get("type") == "state"
        ]
        pivot = len(state_records) // 2
        for record in state_records[pivot:]:
            record["data"]["sequence"] += 1
        report = analyze(records)
        self.assertEqual(report["gate_verdict"], "FAIL")
        self.assertEqual(
            report["data_quality"]["packet_sequence"]["gap_count"], 1
        )
        self.assertEqual(
            report["aggregate_metrics"]["confirmed_packet_drop_count"], 1
        )

    def test_packet_sequence_metadata_is_required_for_a_pass(self):
        records = synthetic_records()
        for record in records:
            if record.get("type") == "state":
                record["data"].pop("sequence")
        report = analyze(records)
        self.assertEqual(report["gate_verdict"], "UNSUPPORTED")
        self.assertIn(
            "packet_sequence_metadata_missing", report["unsupported_reasons"]
        )

    def test_rejected_position_updates_are_a_hard_failure(self):
        records = synthetic_records()
        for record in records:
            if (
                    record.get("type") == "frames"
                    and record["data"]["cf_timestamp_ms"] > 60):
                record["data"]["tvec"] = [100.0, 0.0, 1.0]

        report = analyze(records)

        self.assertEqual(report["gate_verdict"], "FAIL")
        episode = report["episodes"][0]
        self.assertEqual(episode["rejected_position_update_count"], 80)
        self.assertEqual(
            episode["gates"]["rejected_position_updates"]["status"],
            "FAIL",
        )
        self.assertEqual(
            episode["gates"]["strict_position_update_count"]["status"],
            "UNSUPPORTED",
        )

    def test_confirmed_release_replay_keeps_the_first_unloaded_epoch(self):
        records = synthetic_records()

        report = analyze(records)

        self.assertEqual(report["gate_verdict"], "PASS")
        self.assertEqual(report["lifecycle_counts"]["release_request"], 0)
        self.assertEqual(report["lifecycle_counts"]["release"], 1)
        episode = report["episodes"][0]
        self.assertEqual(episode["release_cf_timestamp_ms"], 50)
        self.assertEqual(
            episode["release_time_provenance"][
                "release_confirmation_monotonic_s"
            ],
            1000.060,
        )
        self.assertEqual(
            episode["final_filter_snapshot"]["cf_timestamp_ms"], 860
        )

    def test_input_records_are_not_modified(self):
        records = synthetic_records()
        before = copy.deepcopy(records)
        analyze(records)
        self.assertEqual(records, before)


if __name__ == "__main__":
    unittest.main()
