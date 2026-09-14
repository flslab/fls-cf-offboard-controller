import copy
import math
import unittest

from Interaction.contact_attitude_observer import (
    CF_TIMESTAMP_MODULUS_MS,
    ContactAttitudeConfig,
    quaternion_from_native_rpy,
)
from Interaction.contact_attitude_shadow import ContactAttitudeShadowConfig
from Interaction.log_manager import CONTACT_SOURCE_TIMESTAMP_BASIS
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

    for timestamp_ms in range(0, 861):
        host_time = base_time + timestamp_ms / 1000.0
        if timestamp_ms % 10 == 0:
            add(timestamp_ms, 0, {
                "type": "state", "group": "ACC_ALIGN", "data": {
                    "cf_timestamp_ms": timestamp_ms,
                    "host_receive_time_s": host_time,
                    "acc.x": 0.0, "acc.y": 0.0, "acc.z": 1.0,
                    "stateEstimate.yaw": 0.0,
                },
            })
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
            add(timestamp_ms, 3, {
                "type": "state", "group": "CONTACT_STATE_SEED", "data": {
                    "cf_timestamp_ms": timestamp_ms,
                    "host_receive_time_s": host_time,
                    "stateEstimate.x": 0.0,
                    "stateEstimate.y": 0.0,
                    "stateEstimate.z": 1.0,
                    "stateEstimate.vx": 0.0,
                    "stateEstimate.vy": 0.0,
                    "stateEstimate.vz": 0.0,
                },
            })
            frame_data = {
                "time": host_time,
                "cf_timestamp_ms": timestamp_ms,
                "tvec": [0.0, 0.0, 1.0],
                "position_forwarded_to_onboard_ekf": True,
                "orientation_forwarded_to_onboard_ekf": False,
            }
            if include_truth:
                truth_roll = (
                    bad_release_truth_deg if timestamp_ms >= 60 else 0.0
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
                "gyro.x": 0.0, "gyro.y": 0.0, "gyro.z": 0.0,
            },
        })

    def event(ms, priority, name):
        add(ms, priority, {
            "type": "events", "name": name,
            "data": {"time": base_time + ms / 1000.0},
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
    event(60, 9, "Contact Attitude Shadow Released")

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
            "raw_vicon_tvec_forwarded_to_onboard_ekf",
        )
        self.assertEqual(
            episode["final_filter_snapshot"]["position_update_count"], 80
        )

    def test_firmware_latched_field_names_and_provenance_replay(self):
        records = synthetic_records()
        renames = {
            'GYRO_1KHZ': {
                'gyro.x': 'contactGyro.x',
                'gyro.y': 'contactGyro.y',
                'gyro.z': 'contactGyro.z',
            },
            'ACC_ALIGN': {
                'acc.x': 'contactAccel.x',
                'acc.y': 'contactAccel.y',
                'acc.z': 'contactAccel.z',
                'stateEstimate.yaw': 'contactAccel.yaw',
            },
            'CONTACT_STATE_SEED': {
                'stateEstimate.x': 'contactSeed.x',
                'stateEstimate.y': 'contactSeed.y',
                'stateEstimate.z': 'contactSeed.z',
                'stateEstimate.vx': 'contactSeed.vx',
                'stateEstimate.vy': 'contactSeed.vy',
                'stateEstimate.vz': 'contactSeed.vz',
            },
        }
        epoch_keys = {
            'GYRO_1KHZ': 'contactGyro.epoch',
            'ACC_ALIGN': 'contactAccel.epoch',
            'CONTACT_STATE_SEED': 'contactSeed.epoch',
        }
        for record in records:
            group = record.get('group')
            if record.get('type') != 'state' or group not in renames:
                continue
            data = record['data']
            for old, new in renames[group].items():
                data[new] = data.pop(old)
            timestamp = data['cf_timestamp_ms']
            data[epoch_keys[group]] = timestamp & 0xFFFF
            data['transport_cf_timestamp_ms'] = timestamp
            data['source_cf_timestamp_ms'] = timestamp
            data['source_cf_timestamp_basis'] = (
                CONTACT_SOURCE_TIMESTAMP_BASIS
            )
            data['source_snapshot_atomic'] = True

        report = analyze(records, experiment_run=2)

        self.assertEqual(report['gate_verdict'], 'PASS')
        self.assertEqual(
            report['data_quality']['missing_required_packet_field_count'], 0
        )
        self.assertIsNone(report['data_quality']['replay_fatal_reason'])

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
                data["release_snapshot"] = {
                    "release_event_monotonic_s": 1000.060,
                }

        report = analyze(records)

        self.assertEqual(report["gate_verdict"], "UNSUPPORTED")
        self.assertEqual(
            report["data_quality"]["replay_skipped_position_packets"], 0
        )
        episode = report["episodes"][0]
        self.assertEqual(episode["approximate_position_update_count"], 80)
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
                data["cf_timestamp_ms"] = (offset + raw) % CF_TIMESTAMP_MODULUS_MS
                data.pop("host_receive_time_s")
            elif record.get("type") == "frames":
                raw = round((data["time"] - 1000.0) * 1000.0)
                data["cf_timestamp_ms"] = (offset + raw) % CF_TIMESTAMP_MODULUS_MS
                data.pop("time")
            elif record.get("type") == "events":
                data.pop("time")
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

    def test_deferred_release_replay_keeps_the_first_release_epoch(self):
        records = synthetic_records()
        late_seed_records = []
        retained = []
        for record in records:
            if (
                record.get("group") == "CONTACT_STATE_SEED"
                and record["data"]["cf_timestamp_ms"] in (50, 60)
            ):
                late_seed_records.append(record)
            else:
                retained.append(record)
        records = retained
        release_record = next(
            record for record in records
            if record.get("name") == "Contact Attitude Shadow Released"
        )
        release_record["name"] = "Contact Attitude Shadow Release Deferred"
        release_record["data"].update({
            "cf_timestamp_ms": 60,
            "release_request": {
                "host_loop_position": [0, 0, 1],
                "host_loop_velocity": [0.2, 0, 0],
                "active_setpoint": {"kind": "attitude_zdistance"},
            },
        })
        insertion = next(
            index + 1 for index, record in enumerate(records)
            if (
                record.get("group") == "GYRO_1KHZ"
                and record["data"]["cf_timestamp_ms"] == 62
            )
        )
        records[insertion:insertion] = late_seed_records + [{
            "type": "events",
            "name": "Contact Attitude Shadow Released",
            "data": {
                "time": 1000.062,
                "cf_timestamp_ms": 60,
            },
        }]
        sequence = 0
        for record in records:
            if record.get("type") == "state":
                record["data"]["sequence"] = sequence
                sequence += 1

        report = analyze(records)

        self.assertEqual(report["gate_verdict"], "PASS")
        self.assertEqual(report["lifecycle_counts"]["release_request"], 1)
        self.assertEqual(report["lifecycle_counts"]["release"], 1)
        request_result = next(
            result for result in report["lifecycle_results"]
            if result["action"] == "release_request"
        )
        self.assertTrue(request_result["valid"])
        episode = report["episodes"][0]
        self.assertEqual(episode["release_cf_timestamp_ms"], 60)
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
