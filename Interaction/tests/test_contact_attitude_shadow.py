import math
import time
import unittest

from Interaction.contact_attitude_observer import (
    ContactAttitudeConfig,
    quaternion_from_native_rpy,
)
from Interaction.contact_attitude_shadow import (
    ContactAttitudeShadow,
    ContactAttitudeShadowConfig,
)
from Interaction.log_manager import (
    CONTACT_SOURCE_TIMESTAMP_BASIS,
    CfLogPacket,
    MocapFramePacket,
)


def packet(sequence, group, timestamp, **data):
    return CfLogPacket(
        sequence, group, timestamp, 1000 + timestamp / 1000, data,
        transport_cf_timestamp_ms=timestamp,
        source_cf_timestamp_basis=CONTACT_SOURCE_TIMESTAMP_BASIS,
        source_snapshot_atomic=True,
    )


def mocap_packet(
        sequence, timestamp, position=(0.0, 0.0, 1.0), quaternion=None,
        orientation_forwarded=False, position_forwarded=True,
        cf_timestamp_ms=None,
):
    data = {
        'frame_id': sequence,
        'time': 1000 + timestamp / 1000,
        'tvec': list(position),
        'position_forwarded_to_onboard_ekf': position_forwarded,
        'orientation_forwarded_to_onboard_ekf': orientation_forwarded,
    }
    if quaternion is not None:
        w, x, y, z = quaternion
        data['quat'] = [x, y, z, w]
    return MocapFramePacket(
        sequence, 'frames', 1000 + timestamp / 1000, data,
        cf_timestamp_ms=cf_timestamp_ms,
    )


class ContactAttitudeShadowTests(unittest.TestCase):
    def test_config_rejects_boolean_numeric_limits(self):
        for field in (
            'queue_capacity', 'max_drain_packets', 'history_capacity',
            'alignment_min_state_samples', 'max_release_replay_samples',
            'max_position_replay_samples', 'max_drain_time_s',
            'max_release_processing_time_s',
        ):
            with self.subTest(field=field):
                with self.assertRaises(ValueError):
                    ContactAttitudeShadowConfig(**{field: True})

    def make_shadow(self, **kwargs):
        config = kwargs.pop(
            'config', ContactAttitudeShadowConfig(
                queue_capacity=64,
                alignment_stationary_window_ms=20,
                alignment_min_state_samples=3,
            )
        )
        return ContactAttitudeShadow(
            config=config,
            observer_config=ContactAttitudeConfig(
                alignment_window_ms=20, alignment_min_samples=3,
                alignment_max_sample_gap_ms=10,
            ),
            clock=lambda: 1000.05,
            **kwargs,
        )

    @staticmethod
    def align(shadow):
        sequence = 0
        for timestamp in (0, 10, 20):
            shadow.enqueue_packet(packet(
                sequence, 'ACC_ALIGN', timestamp,
                **{'acc.x': 0, 'acc.y': 0, 'acc.z': 1,
                   'stateEstimate.yaw': 90},
            ))
            sequence += 1
            shadow.enqueue_packet(packet(
                sequence, 'GYRO_1KHZ', timestamp,
                **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
            ))
            sequence += 1
            shadow.enqueue_packet(packet(
                sequence, 'CONTACT_STATE_SEED', timestamp,
                **{
                    'stateEstimate.x': 0,
                    'stateEstimate.y': 0,
                    'stateEstimate.z': 1,
                    'stateEstimate.vx': 0,
                    'stateEstimate.vy': 0,
                    'stateEstimate.vz': 0,
                },
            ))
            sequence += 1
        return shadow.drain(), sequence

    def test_one_packed_packet_supplies_atomic_imu_and_release_seed(self):
        shadow = self.make_shadow(config=ContactAttitudeShadowConfig(
            queue_capacity=64,
            alignment_stationary_window_ms=20,
            alignment_min_state_samples=3,
            alignment_legacy_yaw_deg=0.0,
        ))
        sequence = 0
        for timestamp in (0, 10, 20):
            shadow.enqueue_packet(packet(
                sequence, 'VEL_ORI', timestamp,
                **{
                    'stateEstimate.vx': 0.0,
                    'stateEstimate.vy': 0.0,
                    'stateEstimate.vz': 0.0,
                    'stateEstimate.yaw': 0.0,
                },
            ))
            sequence += 1
            shadow.enqueue_packet(packet(
                sequence, 'GYRO_1KHZ', timestamp,
                **{
                    'contactImu.gx': 0.0,
                    'contactImu.gy': 0.0,
                    'contactImu.gz': 0.0,
                    'contactImu.ax': 0.0,
                    'contactImu.ay': 0.0,
                    'contactImu.az': 1.0,
                    'contactImu.px': 0.0,
                    'contactImu.py': 0.0,
                    'contactImu.pz': 1.0,
                    'contactImu.vx': 0.0,
                    'contactImu.vy': 0.0,
                    'contactImu.vz': 0.0,
                    'contactImu.epoch': timestamp,
                },
            ))
            sequence += 1

        result = shadow.drain()

        self.assertTrue(result['valid'])
        self.assertEqual(result['observer']['phase'], 'ready')
        self.assertEqual(len(shadow._accel_packets), 3)
        self.assertEqual(len(shadow._state_seed_packets), 3)
        self.assertIs(
            shadow._accel_packets[-1], shadow._state_seed_packets[-1]
        )

    @staticmethod
    def add_release_state(
            shadow, sequence, timestamp, position=(0.0, 0.0, 1.0),
            velocity=(0.2, 0.0, 0.0), mocap_position=None,
    ):
        shadow.enqueue_packet(packet(
            sequence, 'POS_ACC', timestamp,
            **{
                'stateEstimate.x': position[0],
                'stateEstimate.y': position[1],
                'stateEstimate.z': position[2],
            },
        ))
        shadow.enqueue_packet(packet(
            sequence + 1, 'VEL_ORI', timestamp,
            **{
                'stateEstimate.vx': velocity[0],
                'stateEstimate.vy': velocity[1],
                'stateEstimate.vz': velocity[2],
            },
        ))
        shadow.enqueue_packet(packet(
            sequence + 2, 'CONTACT_STATE_SEED', timestamp,
            **{
                'stateEstimate.x': position[0],
                'stateEstimate.y': position[1],
                'stateEstimate.z': position[2],
                'stateEstimate.vx': velocity[0],
                'stateEstimate.vy': velocity[1],
                'stateEstimate.vz': velocity[2],
            },
        ))
        shadow.enqueue_mocap_frame(mocap_packet(
            sequence + 3, timestamp,
            position=position if mocap_position is None else mocap_position,
        ))
        return sequence + 4

    def test_alignment_contact_release_and_position_only_fusion(self):
        reports = []
        shadow = self.make_shadow(report=reports.append)
        aligned, sequence = self.align(shadow)
        self.assertTrue(aligned['valid'])
        self.assertEqual(aligned['observer']['phase'], 'ready')
        self.assertTrue(shadow.begin_contact_candidate()['valid'])
        self.assertTrue(shadow.confirm_contact()['valid'])
        shadow.enqueue_packet(packet(
            sequence, 'GYRO_1KHZ', 21,
            **{'gyro.x': 10, 'gyro.y': -20, 'gyro.z': 5},
        ))
        shadow.drain()
        sequence = self.add_release_state(
            shadow, sequence + 1, 21,
            position=(0.4, -0.3, 0.9),
            mocap_position=(0.0, 0.0, 1.0),
        )
        active_setpoint = {'kind': 'attitude_zdistance', 'roll_deg': 4.0}
        effective_command = {
            'kind': 'attitude_zdistance', 'roll_deg': 2.0,
            'effective_query_time': 9.9,
        }
        pending = [{'sequence': 8, 'sent_at': 10.0}]
        released = shadow.release(
            [0, 0, 1], [0.2, 0, 0],
            interaction_direction=[0, 1, 0],
            interaction_direction_source='potentiometer_force_world',
            active_setpoint=active_setpoint,
            effective_command_at_state=effective_command,
            pending_transport_commands=pending,
            inner_loop_tail={'attitude_time_constant_s': 0.08},
        )
        self.assertIsNotNone(released['post_release_ekf'])
        snapshot = released['release_snapshot']
        self.assertEqual(
            snapshot['position_source'],
            'raw_vicon_tvec_forwarded_to_onboard_ekf',
        )
        self.assertEqual(snapshot['position_forward_route'], 'extpos_position_only')
        self.assertEqual(snapshot['position_m'], [0.0, 0.0, 1.0])
        self.assertEqual(snapshot['external_position_seed_m'], [0.0, 0.0, 1.0])
        self.assertEqual(
            snapshot['initial_position_seed_m'], [0.0, 0.0, 1.0]
        )
        self.assertEqual(
            snapshot['onboard_ekf_position_at_velocity_epoch_m'],
            [0.4, -0.3, 0.9],
        )
        self.assertEqual(snapshot['position_seed_skew_ms'], 0.0)
        self.assertEqual(
            snapshot['position_seed_timing_basis'],
            'host_after_wait_availability_approximation',
        )
        self.assertFalse(snapshot['position_seed_scientifically_time_aligned'])
        self.assertEqual(snapshot['interaction_direction'], [0.0, 1.0, 0.0])
        self.assertEqual(snapshot['interaction_direction_source'],
                         'potentiometer_force_world')
        self.assertEqual(snapshot['active_setpoint'], active_setpoint)
        self.assertEqual(snapshot['pending_transport_commands'], pending)
        active_setpoint['roll_deg'] = 99
        effective_command['roll_deg'] = 99
        pending[0]['sequence'] = 99
        self.assertEqual(shadow.snapshot()['release_snapshot'][
            'active_setpoint']['roll_deg'], 4.0)
        self.assertEqual(shadow.snapshot()['release_snapshot'][
            'effective_command_at_state']['roll_deg'], 2.0)
        self.assertEqual(shadow.snapshot()['release_snapshot'][
            'pending_transport_commands'][0]['sequence'], 8)
        shadow.enqueue_packet(packet(
            sequence, 'ACC_ALIGN', 22,
            **{'acc.x': 0.1, 'acc.y': 0, 'acc.z': 1,
               'stateEstimate.yaw': -99},
        ))
        shadow.enqueue_packet(packet(
            sequence + 1, 'GYRO_1KHZ', 22,
            **{'gyro.x': 10, 'gyro.y': -20, 'gyro.z': 5},
        ))
        propagated = shadow.drain()
        self.assertTrue(propagated['shadow_only'])
        self.assertGreater(
            propagated['post_release_ekf']['world_acceleration_m_s2'][0], 0
        )
        fused = shadow.update_extpos([0.0002, 0.0, 1.0])
        self.assertEqual(
            fused['post_release_ekf']['position_update_count'], 1
        )

    def test_release_prefers_exact_cf_timestamped_forwarded_vicon_seed(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow.enqueue_packet(packet(
            sequence, 'GYRO_1KHZ', 21,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        shadow.drain()
        sequence = self.add_release_state(
            shadow, sequence + 1, 21,
            position=(0.4, -0.3, 0.9),
            mocap_position=(0.1, 0.1, 1.0),
        )
        shadow.enqueue_mocap_frame(mocap_packet(
            sequence, 21, position=(0.0, 0.0, 1.0),
            cf_timestamp_ms=21,
        ))

        released = shadow.release([0, 0, 1], [0.2, 0, 0])
        snapshot = released['release_snapshot']

        self.assertEqual(snapshot['external_position_seed_m'], [0.0, 0.0, 1.0])
        self.assertEqual(snapshot['initial_position_seed_m'], [0.0, 0.0, 1.0])
        self.assertEqual(
            snapshot['position_source'],
            'raw_vicon_tvec_forwarded_to_onboard_ekf',
        )
        self.assertEqual(snapshot['position_seed_cf_timestamp_ms'], 21)
        self.assertEqual(
            snapshot['position_seed_timing_basis'],
            'cf_device_timestamp_exact',
        )
        self.assertTrue(snapshot['position_seed_scientifically_time_aligned'])

    def test_release_extpos_is_not_propagated_from_older_velocity_epoch(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow.enqueue_packet(packet(
            sequence, 'CONTACT_STATE_SEED', 20,
            **{
                'stateEstimate.x': -1.0,
                'stateEstimate.y': 0.0,
                'stateEstimate.z': 1.0,
                'stateEstimate.vx': 1.0,
                'stateEstimate.vy': 0.0,
                'stateEstimate.vz': 0.0,
            },
        ))
        shadow.enqueue_packet(packet(
            sequence + 1, 'ACC_ALIGN', 25,
            **{
                'acc.x': 0.0, 'acc.y': 0.0, 'acc.z': 1.0,
                'stateEstimate.yaw': 0.0,
            },
        ))
        shadow.enqueue_packet(packet(
            sequence + 2, 'GYRO_1KHZ', 25,
            **{'gyro.x': 0.0, 'gyro.y': 0.0, 'gyro.z': 0.0},
        ))
        shadow.enqueue_mocap_frame(mocap_packet(
            sequence + 3, 25, position=(0.5, 0.0, 1.0),
            cf_timestamp_ms=25,
        ))

        released = shadow.release([9, 9, 9], [9, 9, 9])
        snapshot = released['release_snapshot']

        self.assertEqual(snapshot['initial_position_seed_m'], [0.5, 0.0, 1.0])
        self.assertEqual(snapshot['position_m'], [0.5, 0.0, 1.0])
        self.assertEqual(snapshot['release_velocity_m_s'], [1.0, 0.0, 0.0])
        self.assertEqual(snapshot['position_seed_skew_ms'], 0.0)
        self.assertEqual(snapshot['velocity_seed_skew_ms'], -5.0)
        self.assertEqual(snapshot['release_to_commit_replay_ms'], 0.0)

    def test_release_rejects_position_never_forwarded_to_onboard(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow.enqueue_packet(packet(
            sequence, 'GYRO_1KHZ', 21,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        shadow.enqueue_packet(packet(
            sequence + 1, 'CONTACT_STATE_SEED', 21,
            **{
                'stateEstimate.x': 0.4,
                'stateEstimate.y': -0.3,
                'stateEstimate.z': 0.9,
                'stateEstimate.vx': 0.2,
                'stateEstimate.vy': 0.0,
                'stateEstimate.vz': 0.0,
            },
        ))
        shadow.enqueue_mocap_frame(mocap_packet(
            sequence + 2, 21, position=(0.0, 0.0, 1.0),
            position_forwarded=False,
        ))

        released = shadow.release([0, 0, 1], [0.2, 0, 0])

        self.assertIsNone(released['release_snapshot'])
        self.assertEqual(
            released['invalid_reason'],
            'release_extpos_position_missing_or_noncausal',
        )

    def test_dynamic_accelerometer_does_not_modify_contact_quaternion(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow.begin_contact_candidate()
        before = shadow.snapshot()['observer']['quaternion_wxyz']
        shadow.enqueue_packet(packet(
            sequence, 'ACC_ALIGN', 21,
            **{'acc.x': 0.8, 'acc.y': 0, 'acc.z': 0.6,
               'stateEstimate.yaw': 10},
        ))
        shadow.enqueue_packet(packet(
            sequence + 1, 'GYRO_1KHZ', 21,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        after = shadow.drain()['observer']['quaternion_wxyz']
        self.assertEqual(before, after)

    def test_contact_gyro_without_causal_acceleration_fails_shadow(self):
        shadow = self.make_shadow(config=ContactAttitudeShadowConfig(
            queue_capacity=64,
            alignment_stationary_window_ms=20,
            alignment_min_state_samples=3,
            gyro_reorder_wait_s=0.001,
        ))
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow._accel_packets.clear()
        shadow.enqueue_packet(packet(
            sequence, 'GYRO_1KHZ', 21,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))

        result = shadow.drain()

        self.assertFalse(result['valid'])
        self.assertEqual(result['fatal_reason'], 'contact_accel_join_missing')
        self.assertEqual(result['observer']['phase'], 'contact')

    def test_initial_contact_requires_stationary_atomic_state_window(self):
        shadow = self.make_shadow()
        sequence = 0
        for timestamp in (0, 10, 20):
            shadow.enqueue_packet(packet(
                sequence, 'ACC_ALIGN', timestamp,
                **{'acc.x': 0, 'acc.y': 0, 'acc.z': 1,
                   'stateEstimate.yaw': 0},
            ))
            sequence += 1
            shadow.enqueue_packet(packet(
                sequence, 'GYRO_1KHZ', timestamp,
                **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
            ))
            sequence += 1
        shadow.drain()

        refused = shadow.begin_contact()

        self.assertFalse(refused['valid'])
        self.assertEqual(
            refused['invalid_reason'], 'alignment_stationary_state_missing'
        )
        self.assertEqual(refused['observer']['phase'], 'ready')

    def test_moving_alignment_state_is_retryable_after_stationary_window(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow._state_seed_packets.clear()
        for offset, timestamp in enumerate((0, 10, 20)):
            shadow.enqueue_packet(packet(
                sequence + offset, 'CONTACT_STATE_SEED', timestamp,
                **{
                    'stateEstimate.x': 0,
                    'stateEstimate.y': 0,
                    'stateEstimate.z': 1,
                    'stateEstimate.vx': 0.2,
                    'stateEstimate.vy': 0,
                    'stateEstimate.vz': 0,
                },
            ))
        shadow.drain()

        refused = shadow.begin_contact()
        self.assertEqual(
            refused['invalid_reason'], 'alignment_stationary_speed_exceeded'
        )
        self.assertEqual(refused['observer']['phase'], 'ready')

        for timestamp in (30, 40, 50):
            shadow.enqueue_packet(packet(
                sequence + timestamp, 'ACC_ALIGN', timestamp,
                **{'acc.x': 0, 'acc.y': 0, 'acc.z': 1,
                   'stateEstimate.yaw': 90},
            ))
            shadow.enqueue_packet(packet(
                sequence + timestamp + 1, 'GYRO_1KHZ', timestamp,
                **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
            ))
            shadow.enqueue_packet(packet(
                sequence + timestamp + 2, 'CONTACT_STATE_SEED', timestamp,
                **{
                    'stateEstimate.x': 0,
                    'stateEstimate.y': 0,
                    'stateEstimate.z': 1,
                    'stateEstimate.vx': 0,
                    'stateEstimate.vy': 0,
                    'stateEstimate.vz': 0,
                },
            ))
        shadow.drain()
        accepted = shadow.begin_contact()
        self.assertTrue(accepted['valid'])
        self.assertEqual(accepted['observer']['phase'], 'contact')

    def test_duplicate_state_timestamp_cannot_certify_stationarity(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow.enqueue_packet(packet(
            sequence, 'CONTACT_STATE_SEED', 20,
            **{
                'stateEstimate.x': 0, 'stateEstimate.y': 0,
                'stateEstimate.z': 1, 'stateEstimate.vx': 0,
                'stateEstimate.vy': 0, 'stateEstimate.vz': 0,
            },
        ))
        refused = shadow.begin_contact()
        self.assertEqual(
            refused['invalid_reason'], 'alignment_state_timestamp_duplicate'
        )
        self.assertFalse(refused['alignment_gate_metrics']['passed'])

    def test_state_sample_gap_cannot_certify_stationarity(self):
        shadow = self.make_shadow(config=ContactAttitudeShadowConfig(
            queue_capacity=64,
            alignment_stationary_window_ms=40,
            alignment_min_state_samples=3,
            alignment_max_state_gap_ms=15,
        ))
        _, sequence = self.align(shadow)
        for timestamp in (30, 40):
            shadow.enqueue_packet(packet(
                sequence + timestamp, 'ACC_ALIGN', timestamp,
                **{'acc.x': 0, 'acc.y': 0, 'acc.z': 1,
                   'stateEstimate.yaw': 90},
            ))
            shadow.enqueue_packet(packet(
                sequence + timestamp + 1, 'GYRO_1KHZ', timestamp,
                **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
            ))
        shadow.enqueue_packet(packet(
            sequence + 99, 'CONTACT_STATE_SEED', 40,
            **{
                'stateEstimate.x': 0, 'stateEstimate.y': 0,
                'stateEstimate.z': 1, 'stateEstimate.vx': 0,
                'stateEstimate.vy': 0, 'stateEstimate.vz': 0,
            },
        ))
        refused = shadow.begin_contact()
        self.assertEqual(
            refused['invalid_reason'], 'alignment_state_sample_gap'
        )

    def test_configured_nominal_yaw_must_match_observed_onboard_yaw(self):
        shadow = self.make_shadow(config=ContactAttitudeShadowConfig(
            queue_capacity=64,
            alignment_stationary_window_ms=20,
            alignment_min_state_samples=3,
            alignment_legacy_yaw_deg=0.0,
        ))
        aligned, _ = self.align(shadow)
        self.assertTrue(aligned['observer']['valid'])

        refused = shadow.begin_contact()

        self.assertEqual(
            refused['invalid_reason'], 'alignment_nominal_yaw_mismatch'
        )
        self.assertAlmostEqual(
            refused['alignment_gate_metrics']['nominal_yaw_error_deg'], 90.0
        )

    def test_false_recontact_keeps_post_release_ekf_timeline(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow.enqueue_packet(packet(
            sequence, 'GYRO_1KHZ', 21,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        shadow.drain()
        sequence = self.add_release_state(shadow, sequence + 1, 21)
        shadow.release([0, 0, 1], [0.2, 0, 0])
        shadow.enqueue_packet(packet(
            sequence, 'ACC_ALIGN', 22,
            **{'acc.x': 0, 'acc.y': 0, 'acc.z': 1,
               'stateEstimate.yaw': 0},
        ))
        shadow.enqueue_packet(packet(
            sequence + 1, 'GYRO_1KHZ', 22,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        shadow.drain()

        candidate = shadow.begin_contact()
        self.assertEqual(candidate['contact_candidate_origin'], 'post_release')
        self.assertIsNotNone(candidate['post_release_ekf'])
        shadow.enqueue_packet(packet(
            sequence + 2, 'ACC_ALIGN', 23,
            **{'acc.x': 0, 'acc.y': 0, 'acc.z': 1,
               'stateEstimate.yaw': 0},
        ))
        shadow.enqueue_packet(packet(
            sequence + 3, 'GYRO_1KHZ', 23,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        during_candidate = shadow.drain()
        self.assertEqual(
            during_candidate['post_release_ekf']['cf_timestamp_ms'], 23
        )

        cancelled = shadow.cancel_contact_candidate()
        self.assertEqual(cancelled['observer']['phase'], 'released')
        self.assertIsNotNone(cancelled['post_release_ekf'])
        self.assertIsNone(cancelled['contact_candidate_origin'])

    def test_confirmed_recontact_discards_parallel_accelerometer_ekf(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow.enqueue_packet(packet(
            sequence, 'GYRO_1KHZ', 21,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        shadow.drain()
        sequence = self.add_release_state(shadow, sequence + 1, 21)
        shadow.release([0, 0, 1], [0.2, 0, 0])
        shadow.enqueue_packet(packet(
            sequence, 'ACC_ALIGN', 22,
            **{'acc.x': 0, 'acc.y': 0, 'acc.z': 1,
               'stateEstimate.yaw': 0},
        ))
        shadow.enqueue_packet(packet(
            sequence + 1, 'GYRO_1KHZ', 22,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        shadow.drain()

        shadow.begin_contact()
        confirmed = shadow.confirm_contact()

        self.assertEqual(confirmed['observer']['phase'], 'contact')
        self.assertIsNone(confirmed['post_release_ekf'])
        self.assertIsNone(confirmed['contact_candidate_origin'])

    def test_irrelevant_packets_are_not_queued(self):
        shadow = self.make_shadow()
        shadow.enqueue_packet(packet(0, 'MOT_BAT', 0, value=1))
        self.assertEqual(shadow.snapshot()['queued_packets'], 0)

    def test_numeric_shadow_failure_is_contained_from_main_loop(self):
        shadow = self.make_shadow()
        self.align(shadow)
        shadow.begin_contact()
        shadow.enqueue_packet(packet(
            10, 'GYRO_1KHZ', 21,
            **{'gyro.x': float('nan'), 'gyro.y': 0, 'gyro.z': 0},
        ))

        result = shadow.drain()

        self.assertFalse(result['valid'])
        self.assertEqual(result['invalid_reason'], 'drain_failed:ValueError')
        self.assertEqual(result['contained_failure_count'], 1)

    def test_release_without_cf_timestamped_state_fails_observationally(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow._state_seed_packets.clear()
        shadow.enqueue_packet(packet(
            sequence, 'GYRO_1KHZ', 21,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))

        active = {'kind': 'attitude_zdistance', 'roll_deg': 4.0}
        result = shadow.release(
            [0, 0, 1], [0.2, 0, 0], active_setpoint=active
        )

        self.assertFalse(result['valid'])
        self.assertEqual(
            result['invalid_reason'],
            'release_state_pair_missing_or_noncausal',
        )
        self.assertIsNone(result['post_release_ekf'])
        self.assertEqual(result['observer']['phase'], 'contact')
        self.assertEqual(result['pending_release_cf_timestamp_ms'], 21)

        active['roll_deg'] = 99.0
        sequence = self.add_release_state(shadow, sequence + 1, 21)
        shadow.enqueue_packet(packet(
            sequence, 'ACC_ALIGN', 22,
            **{'acc.x': 0.1, 'acc.y': 0, 'acc.z': 1,
               'stateEstimate.yaw': 0},
        ))
        shadow.enqueue_packet(packet(
            sequence + 1, 'GYRO_1KHZ', 22,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        retried = shadow.release(
            [9, 9, 9], [9, 9, 9],
            active_setpoint={'kind': 'wrong_retry_context'},
        )
        self.assertTrue(retried['valid'])
        self.assertIsNotNone(retried['post_release_ekf'])
        self.assertEqual(retried['observer']['phase'], 'released')
        snapshot = retried['release_snapshot']
        self.assertEqual(snapshot['cf_timestamp_ms'], 21)
        self.assertEqual(snapshot['ekf_commit_cf_timestamp_ms'], 22)
        self.assertEqual(snapshot['release_to_commit_replay_ms'], 1.0)
        self.assertEqual(snapshot['active_setpoint']['roll_deg'], 4.0)
        self.assertEqual(snapshot['host_loop_position_m'], [0.0, 0.0, 1.0])
        self.assertIsNone(retried['pending_release_cf_timestamp_ms'])

        duplicate = shadow.release([0, 0, 1], [0.2, 0, 0])
        self.assertFalse(duplicate['valid'])
        self.assertEqual(
            duplicate['invalid_reason'], 'release_outside_valid_contact'
        )
        self.assertEqual(
            duplicate['release_snapshot']['cf_timestamp_ms'], 21
        )

    def test_release_event_selects_causal_historical_gyro_epoch(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        for timestamp in range(21, 31):
            shadow.enqueue_packet(packet(
                sequence, 'ACC_ALIGN', timestamp,
                **{'acc.x': 0, 'acc.y': 0, 'acc.z': 1,
                   'stateEstimate.yaw': 0},
            ))
            sequence += 1
            shadow.enqueue_packet(packet(
                sequence, 'GYRO_1KHZ', timestamp,
                **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
            ))
            sequence += 1
            if timestamp == 21:
                shadow.enqueue_packet(packet(
                    sequence, 'CONTACT_STATE_SEED', timestamp,
                    **{
                        'stateEstimate.x': 0,
                        'stateEstimate.y': 0,
                        'stateEstimate.z': 1,
                        'stateEstimate.vx': 0.2,
                        'stateEstimate.vy': 0,
                        'stateEstimate.vz': 0,
                    },
                ))
                sequence += 1
        shadow.drain()
        shadow.enqueue_mocap_frame(mocap_packet(
            sequence, 21, position=(0.0, 0.0, 1.0),
        ))

        released = shadow.release(
            [0, 0, 1], [0.2, 0, 0],
            release_event_monotonic_s=1000.021,
        )

        self.assertTrue(released['valid'])
        snapshot = released['release_snapshot']
        self.assertEqual(snapshot['cf_timestamp_ms'], 21)
        self.assertEqual(snapshot['ekf_commit_cf_timestamp_ms'], 30)
        self.assertAlmostEqual(
            snapshot['release_event_to_gyro_skew_s'], 0.0
        )
        self.assertEqual(
            snapshot['release_event_time_source'],
            'force_sensor_candidate_onset_monotonic',
        )

    def test_release_event_without_nearby_causal_gyro_is_terminal(self):
        now = [1000.05]
        shadow = ContactAttitudeShadow(
            config=ContactAttitudeShadowConfig(
                queue_capacity=64,
                alignment_stationary_window_ms=20,
                alignment_min_state_samples=3,
            ),
            observer_config=ContactAttitudeConfig(
                alignment_window_ms=20,
                alignment_min_samples=3,
                alignment_max_sample_gap_ms=10,
            ),
            clock=lambda: now[0],
        )
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow.enqueue_packet(packet(
            sequence, 'GYRO_1KHZ', 21,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        shadow.drain()
        now[0] = 1000.24

        result = shadow.release(
            [0, 0, 1], [0, 0, 0],
            release_event_monotonic_s=1000.24,
        )

        self.assertFalse(result['valid'])
        self.assertEqual(
            result['fatal_reason'], 'release_event_gyro_skew_exceeded'
        )
        self.assertIsNone(result['release_snapshot'])

    def test_release_replay_sample_budget_bounds_synchronous_work(self):
        shadow = self.make_shadow(config=ContactAttitudeShadowConfig(
            queue_capacity=128,
            alignment_stationary_window_ms=20,
            alignment_min_state_samples=3,
            max_release_replay_samples=8,
        ))
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        for timestamp in range(21, 41):
            shadow.enqueue_packet(packet(
                sequence, 'ACC_ALIGN', timestamp,
                **{'acc.x': 0, 'acc.y': 0, 'acc.z': 1,
                   'stateEstimate.yaw': 0},
            ))
            sequence += 1
            shadow.enqueue_packet(packet(
                sequence, 'GYRO_1KHZ', timestamp,
                **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
            ))
            sequence += 1
            if timestamp == 21:
                shadow.enqueue_packet(packet(
                    sequence, 'CONTACT_STATE_SEED', timestamp,
                    **{
                        'stateEstimate.x': 0,
                        'stateEstimate.y': 0,
                        'stateEstimate.z': 1,
                        'stateEstimate.vx': 0,
                        'stateEstimate.vy': 0,
                        'stateEstimate.vz': 0,
                    },
                ))
                sequence += 1
        shadow.drain()
        shadow.enqueue_mocap_frame(mocap_packet(
            sequence, 21, position=(0.0, 0.0, 1.0),
        ))

        started = time.perf_counter()
        result = shadow.release(
            [0, 0, 1], [0, 0, 0],
            release_event_monotonic_s=1000.021,
        )
        elapsed = time.perf_counter() - started

        self.assertLess(elapsed, 0.1)
        self.assertFalse(result['valid'])
        self.assertEqual(
            result['fatal_reason'], 'release_replay_sample_budget_exceeded'
        )
        self.assertEqual(result['release_budget_exceeded_count'], 1)
        self.assertIsNone(result['post_release_ekf'])

    def test_abandoned_release_transaction_is_terminal_and_clears_anchor(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow._state_seed_packets.clear()
        shadow.enqueue_packet(packet(
            sequence, 'GYRO_1KHZ', 21,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        deferred = shadow.release([0, 0, 1], [0, 0, 0])
        self.assertEqual(deferred['pending_release_cf_timestamp_ms'], 21)

        abandoned = shadow.abandon_release_transaction('retry_timeout')

        self.assertIsNone(abandoned['pending_release_cf_timestamp_ms'])
        self.assertFalse(abandoned['valid'])
        self.assertEqual(
            abandoned['fatal_reason'],
            'release_transaction_abandoned:retry_timeout',
        )
        retried = shadow.release([0, 0, 1], [0, 0, 0])
        self.assertEqual(
            retried['fatal_reason'],
            'release_transaction_abandoned:retry_timeout',
        )

    def test_raw_vicon_position_fuses_causally_without_using_quaternion(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow.enqueue_packet(packet(
            sequence, 'GYRO_1KHZ', 21,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        shadow.drain()
        sequence = self.add_release_state(shadow, sequence + 1, 21)
        shadow.release([0, 0, 1], [0.2, 0, 0])
        shadow.enqueue_packet(packet(
            sequence, 'ACC_ALIGN', 22,
            **{'acc.x': 0, 'acc.y': 0, 'acc.z': 1,
               'stateEstimate.yaw': 0},
        ))
        shadow.enqueue_packet(packet(
            sequence + 1, 'GYRO_1KHZ', 22,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        shadow.enqueue_mocap_frame(mocap_packet(
            sequence + 2,
            22,
            position=(0.0002, 0, 1),
            quaternion=quaternion_from_native_rpy(0.2, -0.1, 0.3),
            orientation_forwarded=True,
        ))

        result = shadow.drain()

        self.assertEqual(result['post_release_ekf']['position_update_count'], 1)
        self.assertEqual(result['last_position_measurement_skew_ms'], 0.0)
        self.assertFalse(result['vicon_orientation_used_by_shadow_ekf'])
        self.assertTrue(result['vicon'][
            'orientation_forwarded_to_onboard_ekf'
        ])

    def test_out_of_order_position_update_fails_before_erasing_newer_update(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow.enqueue_packet(packet(
            sequence, 'GYRO_1KHZ', 21,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        shadow.drain()
        sequence = self.add_release_state(shadow, sequence + 1, 21)
        shadow.release([0, 0, 1], [0.2, 0, 0])
        for timestamp in (22, 23, 24, 25):
            shadow.enqueue_packet(packet(
                sequence, 'ACC_ALIGN', timestamp,
                **{'acc.x': 0, 'acc.y': 0, 'acc.z': 1,
                   'stateEstimate.yaw': 0},
            ))
            sequence += 1
            shadow.enqueue_packet(packet(
                sequence, 'GYRO_1KHZ', timestamp,
                **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
            ))
            sequence += 1
        shadow.drain()
        shadow.enqueue_mocap_frame(mocap_packet(
            sequence, 24, position=(0.002, 0, 1)
        ))
        first = shadow.drain()
        self.assertEqual(first['post_release_ekf']['position_update_count'], 1)

        shadow.enqueue_mocap_frame(mocap_packet(
            sequence + 1, 23, position=(0.001, 0, 1)
        ))
        result = shadow.drain()

        self.assertFalse(result['valid'])
        self.assertEqual(
            result['fatal_reason'],
            'position_measurement_epoch_nonmonotonic',
        )
        self.assertEqual(result['post_release_ekf']['position_update_count'], 1)

    def test_vicon_frame_not_forwarded_to_onboard_is_not_fused(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow.enqueue_packet(packet(
            sequence, 'GYRO_1KHZ', 21,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        shadow.drain()
        sequence = self.add_release_state(shadow, sequence + 1, 21)
        shadow.release([0, 0, 1], [0.2, 0, 0])
        shadow.enqueue_packet(packet(
            sequence, 'ACC_ALIGN', 22,
            **{'acc.x': 0, 'acc.y': 0, 'acc.z': 1,
               'stateEstimate.yaw': 0},
        ))
        shadow.enqueue_packet(packet(
            sequence + 1, 'GYRO_1KHZ', 22,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        shadow.enqueue_mocap_frame(mocap_packet(
            sequence + 2, 22, position=(0.5, 0, 1),
            position_forwarded=False,
        ))
        result = shadow.drain()
        self.assertEqual(result['post_release_ekf']['position_update_count'], 0)
        self.assertFalse(result['vicon'][
            'position_forwarded_to_onboard_ekf'
        ])

    def test_onboard_mirror_is_an_exact_copy_without_inertial_reestimation(self):
        shadow = self.make_shadow(config=ContactAttitudeShadowConfig(
            queue_capacity=64,
            mode='onboard_mirror',
            experiment_run=1,
            vicon_orientation_forwarded=False,
        ))
        shadow.update_onboard_state(
            [0.1, -0.2, 1.0],
            [0.3, 0.0, -0.1],
            [0.02, -0.03, 0.4],
            angular_velocity_rad_s=[0.1, 0.2, 0.3],
            state_time_s=1000.0,
        )
        snapshot = shadow.drain()
        self.assertTrue(snapshot['valid'])
        self.assertEqual(snapshot['mode'], 'onboard_mirror')
        self.assertEqual(
            snapshot['shadow_estimate']['position_m'],
            snapshot['onboard_ekf']['position_m'],
        )
        self.assertEqual(
            snapshot['comparison']['onboard_minus_shadow_roll_pitch_deg'],
            [0.0, 0.0],
        )
        self.assertTrue(
            snapshot['comparison']['onboard_shadow_mirror_exact']
        )
        shadow.begin_contact()
        shadow.confirm_contact()
        released = shadow.release([9, 9, 9], [9, 9, 9])
        self.assertIsNone(released['post_release_ekf'])
        self.assertEqual(released['shadow_estimate']['phase'], 'released')

    def test_onboard_mirror_recontact_cancel_restores_released_phase(self):
        shadow = self.make_shadow(config=ContactAttitudeShadowConfig(
            queue_capacity=64,
            mode='onboard_mirror',
            experiment_run=1,
            vicon_orientation_forwarded=False,
        ))
        shadow.update_onboard_state(
            [0, 0, 1], [0, 0, 0], [0, 0, 0],
            state_time_s=1000.0,
        )
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow.release([0, 0, 1], [0, 0, 0])

        candidate = shadow.begin_contact()
        cancelled = shadow.cancel_contact_candidate()

        self.assertEqual(
            candidate['shadow_estimate']['phase'], 'contact_candidate'
        )
        self.assertEqual(cancelled['shadow_estimate']['phase'], 'released')

    def test_onboard_mirror_confirm_rechecks_state_freshness(self):
        now = [100.0]
        shadow = ContactAttitudeShadow(
            config=ContactAttitudeShadowConfig(
                queue_capacity=64,
                mode='onboard_mirror',
                experiment_run=1,
                vicon_orientation_forwarded=False,
            ),
            clock=lambda: now[0],
        )
        shadow.update_onboard_state(
            [0, 0, 1], [0, 0, 0], [0, 0, 0],
            state_time_s=1_700_000_000.0,
            host_receive_monotonic_s=100.0,
        )
        shadow.begin_contact()
        now[0] = 100.3

        stale = shadow.confirm_contact()

        self.assertFalse(stale['valid'])
        self.assertEqual(stale['invalid_reason'], 'onboard_mirror_state_stale')
        self.assertEqual(
            stale['shadow_estimate']['phase'], 'contact_candidate'
        )
        shadow.update_onboard_state(
            [0, 0, 1], [0, 0, 0], [0, 0, 0],
            state_time_s=1_700_000_000.3,
            host_receive_monotonic_s=100.3,
        )
        confirmed = shadow.confirm_contact()
        self.assertTrue(confirmed['valid'])
        self.assertEqual(confirmed['shadow_estimate']['phase'], 'contact')

    def test_onboard_mirror_stale_release_epoch_is_immediately_terminal(self):
        now = [100.0]
        shadow = ContactAttitudeShadow(
            config=ContactAttitudeShadowConfig(
                queue_capacity=64,
                mode='onboard_mirror',
                experiment_run=1,
                vicon_orientation_forwarded=False,
            ),
            clock=lambda: now[0],
        )
        shadow.update_onboard_state(
            [0, 0, 1], [0, 0, 0], [0, 0, 0],
            state_time_s=1_700_000_000.0,
            host_receive_monotonic_s=100.0,
        )
        shadow.begin_contact()
        shadow.confirm_contact()
        now[0] = 100.3
        deferred = shadow.release([0, 0, 1], [0, 0, 0])
        self.assertFalse(deferred['valid'])
        self.assertEqual(
            deferred['invalid_reason'],
            'onboard_mirror_release_epoch_unavailable',
        )
        self.assertIsNone(deferred['release_snapshot'])

        shadow.update_onboard_state(
            [1, 0, 1], [2, 0, 0], [0, 0, 0],
            state_time_s=1_700_000_000.3,
            host_receive_monotonic_s=100.3,
        )
        retried = shadow.release([0, 0, 1], [0, 0, 0])
        self.assertFalse(retried['valid'])
        self.assertIsNone(retried['release_snapshot'])
        self.assertEqual(
            retried['shadow_estimate']['phase'], 'contact'
        )

        abandoned = shadow.abandon_release_transaction('retry_timeout')

        self.assertFalse(abandoned['valid'])
        self.assertEqual(
            abandoned['fatal_reason'],
            'onboard_mirror_release_epoch_unavailable',
        )

    def test_vicon_onboard_availability_join_uses_one_monotonic_clock(self):
        now = [100.02]
        shadow = ContactAttitudeShadow(
            config=ContactAttitudeShadowConfig(
                queue_capacity=64,
                mode='onboard_mirror',
                experiment_run=1,
                vicon_orientation_forwarded=False,
            ),
            clock=lambda: now[0],
        )
        shadow.update_onboard_state(
            [0, 0, 1], [0, 0, 0], [0, 0, 0],
            state_time_s=1_700_000_000.0,
            host_receive_monotonic_s=100.0,
        )
        truth = quaternion_from_native_rpy(0, 0, 0)
        frame = mocap_packet(1, 20, quaternion=truth)
        frame = MocapFramePacket(
            sequence=frame.sequence,
            group=frame.group,
            host_receive_time_s=1_700_000_000.02,
            data=frame.data,
            host_receive_monotonic_s=100.02,
        )
        shadow.enqueue_mocap_frame(frame)

        comparison = shadow.drain()['comparison']

        self.assertAlmostEqual(
            comparison['vicon_to_onboard_host_availability_skew_s'],
            0.02,
        )
        self.assertTrue(comparison['comparison_time_aligned'])
        self.assertFalse(comparison['comparison_scientifically_valid'])

    def test_three_way_roll_pitch_labels_run_two_orientation_withheld(self):
        shadow = self.make_shadow(config=ContactAttitudeShadowConfig(
            queue_capacity=64,
            mode='inertial_position',
            experiment_run=2,
            vicon_orientation_forwarded=False,
        ))
        self.align(shadow)
        shadow.update_onboard_state(
            [0, 0, 1], [0, 0, 0], [0.1, -0.2, 0.3],
            state_time_s=1000.02,
        )
        truth = quaternion_from_native_rpy(0.12, 0.18, 0.3)
        shadow.enqueue_mocap_frame(mocap_packet(
            20, 20, quaternion=truth, orientation_forwarded=False,
        ))
        snapshot = shadow.drain()
        self.assertIsNotNone(
            snapshot['comparison']['vicon_minus_onboard_roll_pitch_deg']
        )
        self.assertIsNotNone(
            snapshot['comparison']['vicon_minus_shadow_roll_pitch_deg']
        )
        self.assertTrue(snapshot['comparison'][
            'vicon_orientation_withheld_from_onboard_estimator'
        ])
        self.assertTrue(snapshot['comparison'][
            'vicon_orientation_withheld_from_shadow_estimator'
        ])
        self.assertTrue(snapshot['comparison'][
            'shared_mocap_position_sensor_correlation_remains'
        ])

    def test_full_pose_run_uses_nominal_yaw_not_vicon_corrected_onboard_yaw(self):
        shadow = self.make_shadow(config=ContactAttitudeShadowConfig(
            queue_capacity=64,
            mode='inertial_position',
            experiment_run=3,
            vicon_orientation_forwarded=True,
            alignment_legacy_yaw_deg=30.0,
        ))
        aligned, _ = self.align(shadow)
        self.assertAlmostEqual(
            aligned['observer']['legacy_rpy_rad'][2],
            math.radians(30.0),
        )
        self.assertEqual(
            aligned['alignment_yaw_source'], 'configured_nominal_yaw'
        )

    def test_queue_overflow_invalidates_shadow_only(self):
        shadow = self.make_shadow()
        for sequence in range(70):
            shadow.enqueue_packet(packet(
                sequence, 'GYRO_1KHZ', sequence,
                **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
            ))
        result = shadow.snapshot()
        self.assertEqual(result['invalid_reason'], 'shadow_queue_overflow')
        self.assertEqual(result['dropped_packets'], 6)

    def test_drain_packet_budget_fails_closed_and_is_reported(self):
        shadow = self.make_shadow(config=ContactAttitudeShadowConfig(
            queue_capacity=64,
            max_drain_packets=2,
            alignment_stationary_window_ms=20,
            alignment_min_state_samples=3,
        ))
        for sequence in range(3):
            shadow.enqueue_packet(packet(
                sequence, 'GYRO_1KHZ', sequence,
                **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
            ))

        result = shadow.drain()

        self.assertFalse(result['valid'])
        self.assertEqual(
            result['fatal_reason'], 'shadow_drain_packet_budget_exceeded'
        )
        self.assertEqual(result['drain_budget_exceeded_count'], 1)
        self.assertEqual(result['dropped_packets'], 3)

    def test_drain_time_budget_fails_closed_and_is_reported(self):
        ticks = iter((0.0, 0.005, 0.006))
        shadow = ContactAttitudeShadow(
            config=ContactAttitudeShadowConfig(
                queue_capacity=64,
                max_drain_time_s=0.004,
                alignment_stationary_window_ms=20,
                alignment_min_state_samples=3,
            ),
            observer_config=ContactAttitudeConfig(
                alignment_window_ms=20,
                alignment_min_samples=3,
                alignment_max_sample_gap_ms=10,
            ),
            clock=lambda: 1000.05,
            perf_clock=lambda: next(ticks),
        )
        shadow.enqueue_packet(packet(
            0, 'ACC_ALIGN', 0,
            **{'acc.x': 0, 'acc.y': 0, 'acc.z': 1,
               'stateEstimate.yaw': 0},
        ))

        result = shadow.drain()

        self.assertFalse(result['valid'])
        self.assertEqual(
            result['fatal_reason'], 'shadow_drain_time_budget_exceeded'
        )
        self.assertEqual(result['drain_budget_exceeded_count'], 1)
        self.assertGreaterEqual(result['last_drain_duration_s'], 0.006)

    def test_report_sink_failure_is_isolated(self):
        def fail(_record):
            raise RuntimeError('closing logger')

        shadow = self.make_shadow(report=fail)
        result = shadow.drain()
        self.assertFalse(result['valid'])
        self.assertEqual(shadow.snapshot()['invalid_reason'],
                         'shadow_report_failed')


if __name__ == '__main__':
    unittest.main()
