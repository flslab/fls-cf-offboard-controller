import math
import time
import unittest
from unittest import mock

import numpy as np

from Interaction.contact_attitude_observer import (
    ContactAttitudeConfig,
    quaternion_from_native_rpy,
)
from Interaction.contact_attitude_shadow import (
    ContactAttitudeShadow,
    ContactAttitudeShadowConfig,
)
from Interaction.contact_attitude_experiment import (
    CRAZYSIM_RELEASE_CLOCK_MAPPING_BASIS,
    FIRMWARE_SHARED_CLOCK_RELEASE_LATCH_BASIS,
    RELEASE_EVENT_TIME_SOURCE,
)
from Interaction.log_manager import (
    CF_LOG_TRANSPORT_TIMESTAMP_BASIS,
    CONTACT_SOURCE_TIMESTAMP_BASIS,
    CRAZYSIM_CF_TIMESTAMP_BASIS,
    CfLogPacket,
    MocapFramePacket,
)
from Interaction.post_release_estimator_gate import (
    PostReleaseEstimatorGateConfig,
    evaluate_post_release_estimator,
)
from Interaction.post_release_inertial_ekf import PostReleaseInertialEkf


def packet(sequence, group, timestamp, **data):
    return CfLogPacket(
        sequence, group, timestamp, 1000 + timestamp / 1000, data,
        transport_cf_timestamp_ms=timestamp,
        source_cf_timestamp_basis=CONTACT_SOURCE_TIMESTAMP_BASIS,
        source_snapshot_atomic=True,
    )


def packed_packet(sequence, timestamp, *, transport_timestamp=None):
    return CfLogPacket(
        sequence=sequence,
        group='GYRO_1KHZ',
        cf_timestamp_ms=timestamp,
        host_receive_time_s=1000 + timestamp / 1000,
        data={
            'contactImu.gx': 0.0,
            'contactImu.gy': 0.0,
            'contactImu.gz': 0.0,
            'contactImu.ax': 0.0,
            'contactImu.ay': 0.0,
            'contactImu.az': 1.0,
            'contactImu.px': 0.0,
            'contactImu.py': 0.0,
            'contactImu.pz': 1.0,
            'contactImu.vx': 0.2,
            'contactImu.vy': 0.0,
            'contactImu.vz': 0.0,
            'contactImu.epoch': timestamp & 0xFFFF,
        },
        transport_cf_timestamp_ms=(
            timestamp
            if transport_timestamp is None else transport_timestamp
        ),
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
    if cf_timestamp_ms is not None:
        data['mocap_timing'] = {
            'frame_time_scope': 'source_capture',
            'source_capture_time_available': True,
            'source_capture_time_s': 1000 + timestamp / 1000,
            'source_capture_time_basis': 'synthetic_crazysim_clock_v1',
        }
    return MocapFramePacket(
        sequence, 'frames', 1000 + timestamp / 1000, data,
        cf_timestamp_ms=cf_timestamp_ms,
        cf_timestamp_basis=(
            CRAZYSIM_CF_TIMESTAMP_BASIS
            if cf_timestamp_ms is not None else None
        ),
        cf_timestamp_uncertainty_ms=(
            0.0 if cf_timestamp_ms is not None else None
        ),
    )


class ContactAttitudeShadowTests(unittest.TestCase):
    def test_config_rejects_boolean_numeric_limits(self):
        for field in (
            'queue_capacity', 'max_drain_packets', 'history_capacity',
            'alignment_min_state_samples', 'max_release_replay_samples',
            'max_position_replay_samples', 'max_drain_time_s',
            'max_release_processing_time_s',
            'release_event_history_s',
            'post_release_imu_max_abs_gyro_deg_s',
            'post_release_imu_max_abs_accel_g',
            'post_release_imu_max_gyro_step_deg_s',
            'post_release_imu_max_accel_step_g',
        ):
            with self.subTest(field=field):
                with self.assertRaises(ValueError):
                    ContactAttitudeShadowConfig(**{field: True})

        with self.assertRaises(ValueError):
            ContactAttitudeShadowConfig(
                post_release_imu_quality_calibrated=True
            )
        with self.assertRaises(ValueError):
            ContactAttitudeShadowConfig(
                absolute_yaw_reference_certified=True
            )
        default = ContactAttitudeShadowConfig()
        self.assertFalse(default.post_release_imu_quality_calibrated)
        self.assertFalse(default.absolute_yaw_reference_certified)

    def test_default_capacities_cover_measured_release_candidate_lead(self):
        config = ContactAttitudeShadowConfig()

        self.assertGreaterEqual(config.state_seed_capacity, 101)
        self.assertGreaterEqual(config.max_release_replay_samples, 351)
        self.assertGreater(
            config.state_seed_capacity, config.max_release_replay_samples
        )
        self.assertGreater(
            config.history_capacity, config.max_release_replay_samples
        )

    def test_authority_atomic_packet_requires_causal_five_ms_transport(self):
        self.assertTrue(ContactAttitudeShadow._is_strict_atomic_imu_packet(
            packed_packet(1, 21, transport_timestamp=26)
        ))
        self.assertFalse(ContactAttitudeShadow._is_strict_atomic_imu_packet(
            packed_packet(1, 21, transport_timestamp=20)
        ))
        self.assertFalse(ContactAttitudeShadow._is_strict_atomic_imu_packet(
            packed_packet(1, 21, transport_timestamp=27)
        ))

    def test_default_state_history_passes_100ms_gate_at_1khz(self):
        shadow = ContactAttitudeShadow(
            config=ContactAttitudeShadowConfig(
                queue_capacity=512,
                max_drain_packets=512,
                max_drain_time_s=1.0,
                alignment_legacy_yaw_deg=0.0,
            ),
            clock=lambda: 1000.40,
        )
        sequence = 0
        for timestamp in range(301):
            if timestamp % 10 == 0:
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

        aligned = shadow.drain()
        started = shadow.begin_contact()

        self.assertEqual(aligned['observer']['phase'], 'ready')
        self.assertEqual(started['observer']['phase'], 'contact')
        self.assertIsNone(started['invalid_reason'])
        self.assertGreaterEqual(
            started['alignment_gate_metrics']['state_window_covered_ms'],
            100.0,
        )

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

    def test_post_release_imu_step_rejected_before_ekf_propagation(self):
        shadow = self.make_shadow(config=ContactAttitudeShadowConfig(
            queue_capacity=64,
            alignment_stationary_window_ms=20,
            alignment_min_state_samples=3,
            post_release_imu_max_gyro_step_deg_s=5.0,
        ))
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()

        def packed(timestamp, gyro_x):
            nonlocal sequence
            result = packet(
                sequence, 'GYRO_1KHZ', timestamp,
                **{
                    'contactImu.gx': gyro_x,
                    'contactImu.gy': 0.0,
                    'contactImu.gz': 0.0,
                    'contactImu.ax': 0.0,
                    'contactImu.ay': 0.0,
                    'contactImu.az': 1.0,
                    'contactImu.px': 0.0,
                    'contactImu.py': 0.0,
                    'contactImu.pz': 1.0,
                    'contactImu.vx': 0.2,
                    'contactImu.vy': 0.0,
                    'contactImu.vz': 0.0,
                    'contactImu.epoch': timestamp,
                },
            )
            sequence += 1
            return result

        shadow.enqueue_packet(packed(21, 0.0))
        shadow.drain()
        released = shadow.release(
            [0.0, 0.0, 1.0], [0.2, 0.0, 0.0],
            release_event_monotonic_s=1000.021,
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
        )
        self.assertEqual(released['post_release_ekf']['cf_timestamp_ms'], 21)

        shadow.enqueue_packet(packed(22, 10.0))
        rejected = shadow.drain()

        self.assertFalse(rejected['valid'])
        self.assertEqual(
            rejected['fatal_reason'], 'post_release_gyro_step_implausible'
        )
        self.assertEqual(rejected['post_release_ekf']['cf_timestamp_ms'], 21)
        self.assertEqual(
            rejected['post_release_imu_quality_accepted_count'], 0
        )
        self.assertEqual(
            rejected['post_release_imu_quality_rejected_count'], 1
        )
        self.assertEqual(
            rejected['post_release_imu_quality_tainted_count'], 0
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
            'onboard_ekf_position_common_cf_epoch',
        )
        self.assertEqual(snapshot['position_forward_route'], 'extpos_position_only')
        self.assertEqual(snapshot['position_m'], [0.4, -0.3, 0.9])
        self.assertIsNone(snapshot['external_position_seed_m'])
        self.assertEqual(
            snapshot['initial_position_seed_m'], [0.4, -0.3, 0.9]
        )
        self.assertEqual(
            snapshot['onboard_ekf_position_at_velocity_epoch_m'],
            [0.4, -0.3, 0.9],
        )
        self.assertEqual(snapshot['position_seed_skew_ms'], 0.0)
        self.assertEqual(
            snapshot['position_seed_timing_basis'],
            'firmware_latched_stabilizer_source_timestamp_exact',
        )
        self.assertTrue(snapshot['position_seed_scientifically_time_aligned'])
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
        fused = shadow.update_extpos([0.4002, -0.3, 0.9])
        self.assertEqual(
            fused['post_release_ekf']['position_update_count'], 1
        )

    def test_direct_release_without_detector_provenance_never_passes_gate(self):
        shadow = self.make_shadow(config=ContactAttitudeShadowConfig(
            queue_capacity=64,
            alignment_stationary_window_ms=20,
            alignment_min_state_samples=3,
            alignment_legacy_yaw_deg=90.0,
            absolute_yaw_reference_certified=True,
            absolute_yaw_reference_certificate_id='test_yaw_reference_v1',
            absolute_yaw_reference_yaw_deg=90.0,
        ))
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow.enqueue_packet(packet(
            sequence, 'GYRO_1KHZ', 21,
            **{'gyro.x': 0.0, 'gyro.y': 0.0, 'gyro.z': 0.0},
        ))
        shadow.drain()
        sequence = self.add_release_state(shadow, sequence + 1, 21)
        released = shadow.release([0.0, 0.0, 1.0], [0.2, 0.0, 0.0])
        gate_config = PostReleaseEstimatorGateConfig(enabled=True)
        before_updates = evaluate_post_release_estimator(
            released,
            now_cf_timestamp_ms=21,
            now_unwrapped_cf_timestamp_ms=21,
            now_timestamp_basis=CONTACT_SOURCE_TIMESTAMP_BASIS,
            now_host_receive_age_s=(
                released['post_release_control_epoch']['host_receive_age_s']
            ),
            config=gate_config,
        )
        self.assertFalse(before_updates.estimator_control_eligible)
        self.assertEqual(
            before_updates.reason, 'release_preview_provenance_untrusted'
        )

        for timestamp in (22, 23, 24):
            shadow.enqueue_packet(packet(
                sequence, 'ACC_ALIGN', timestamp,
                **{'acc.x': 0.0, 'acc.y': 0.0, 'acc.z': 1.0},
            ))
            sequence += 1
            shadow.enqueue_packet(packet(
                sequence, 'GYRO_1KHZ', timestamp,
                **{'gyro.x': 0.0, 'gyro.y': 0.0, 'gyro.z': 0.0},
            ))
            sequence += 1
            shadow.enqueue_mocap_frame(mocap_packet(
                sequence, timestamp,
                position=(0.0002 * (timestamp - 21), 0.0, 1.0),
                cf_timestamp_ms=timestamp,
            ))
            sequence += 1
            snapshot = shadow.drain()

        decision = evaluate_post_release_estimator(
            snapshot,
            now_cf_timestamp_ms=24,
            now_unwrapped_cf_timestamp_ms=24,
            now_timestamp_basis=CONTACT_SOURCE_TIMESTAMP_BASIS,
            now_host_receive_age_s=(
                snapshot['post_release_control_epoch']['host_receive_age_s']
            ),
            config=gate_config,
        )
        self.assertFalse(decision.estimator_control_eligible, decision)
        self.assertEqual(
            decision.reason, 'release_preview_provenance_untrusted'
        )
        self.assertEqual(snapshot['post_release_ekf']['position_update_count'], 3)

    def test_mapped_release_selects_exact_device_epoch_not_host_latest(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow.enqueue_packet(packed_packet(sequence, 21))
        shadow.enqueue_packet(packed_packet(sequence + 1, 22))
        shadow.drain()

        preview = shadow.begin_release_candidate(
            [0.0, 0.0, 1.0], [0.2, 0.0, 0.0],
            release_event_monotonic_s=1000.023,
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
            release_event_arduino_time_ms=21,
            release_event_cf_timestamp_ms=21,
            release_event_unwrapped_cf_timestamp_ms=21,
            release_clock_mapping_basis=(
                CRAZYSIM_RELEASE_CLOCK_MAPPING_BASIS
            ),
            release_clock_mapping_uncertainty_ms=0.0,
            release_clock_mapping_calibration_id='shared-clock-v1',
        )

        self.assertTrue(preview['release_candidate_active'], preview)
        transaction = shadow._release_preview['transaction']
        self.assertEqual(transaction['estimate'].cf_timestamp_ms, 21)
        self.assertEqual(transaction['gyro_packet'].cf_timestamp_ms, 21)

    def test_exact_mapped_release_packet_may_arrive_during_dwell(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow.enqueue_packet(packed_packet(sequence, 21))
        shadow.drain()
        kwargs = {
            'release_event_monotonic_s': 1000.022,
            'release_event_time_source': RELEASE_EVENT_TIME_SOURCE,
            'release_event_arduino_time_ms': 22,
            'release_event_cf_timestamp_ms': 22,
            'release_event_unwrapped_cf_timestamp_ms': 22,
            'release_clock_mapping_basis': (
                CRAZYSIM_RELEASE_CLOCK_MAPPING_BASIS
            ),
            'release_clock_mapping_uncertainty_ms': 0.0,
            'release_clock_mapping_calibration_id': 'shared-clock-v1',
        }

        deferred = shadow.begin_release_candidate(
            [0.0, 0.0, 1.0], [0.2, 0.0, 0.0], **kwargs
        )
        self.assertEqual(
            deferred['invalid_reason'],
            'release_event_mapped_imu_epoch_unavailable',
        )
        self.assertIsNone(deferred['fatal_reason'])
        self.assertFalse(deferred['release_candidate_active'])

        shadow.enqueue_packet(packed_packet(sequence + 1, 22))
        ready = shadow.begin_release_candidate(
            [0.0, 0.0, 1.0], [0.2, 0.0, 0.0], **kwargs
        )
        self.assertTrue(ready['release_candidate_active'], ready)
        self.assertIsNone(ready['fatal_reason'])

    def test_confirmation_cannot_omit_frozen_release_mapping(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow.enqueue_packet(packed_packet(sequence, 21))
        shadow.drain()
        preview_kwargs = {
            'release_event_monotonic_s': 1000.021,
            'release_event_time_source': RELEASE_EVENT_TIME_SOURCE,
            'release_event_arduino_time_ms': 21,
            'release_event_cf_timestamp_ms': 21,
            'release_event_unwrapped_cf_timestamp_ms': 21,
            'release_clock_mapping_basis': (
                CRAZYSIM_RELEASE_CLOCK_MAPPING_BASIS
            ),
            'release_clock_mapping_uncertainty_ms': 0.0,
            'release_clock_mapping_calibration_id': 'shared-clock-v1',
        }
        preview = shadow.begin_release_candidate(
            [0.0, 0.0, 1.0], [0.2, 0.0, 0.0], **preview_kwargs
        )
        self.assertTrue(preview['release_candidate_active'], preview)

        released = shadow.release(
            [0.0, 0.0, 1.0], [0.2, 0.0, 0.0],
            release_event_monotonic_s=1000.021,
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
            release_event_arduino_time_ms=21,
            release_confirmation_monotonic_s=1000.071,
            release_confirmation_arduino_time_ms=71,
        )
        self.assertEqual(
            released['fatal_reason'], 'release_preview_event_identity_mismatch'
        )

    def test_release_seed_remains_atomic_onboard_state_with_trusted_vicon(self):
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

        self.assertIsNone(snapshot['external_position_seed_m'])
        self.assertEqual(snapshot['initial_position_seed_m'], [0.4, -0.3, 0.9])
        self.assertEqual(
            snapshot['position_source'],
            'onboard_ekf_position_common_cf_epoch',
        )
        self.assertEqual(snapshot['position_seed_cf_timestamp_ms'], 21)
        self.assertEqual(
            snapshot['position_seed_timing_basis'],
            'firmware_latched_stabilizer_source_timestamp_exact',
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

        self.assertEqual(snapshot['initial_position_seed_m'], [-1.0, 0.0, 1.0])
        self.assertAlmostEqual(snapshot['position_m'][0], -0.995)
        self.assertEqual(snapshot['position_m'][1:], [0.0, 1.0])
        self.assertEqual(snapshot['release_velocity_m_s'], [1.0, 0.0, 0.0])
        self.assertEqual(snapshot['position_seed_skew_ms'], -5.0)
        self.assertEqual(snapshot['velocity_seed_skew_ms'], -5.0)
        self.assertEqual(snapshot['release_to_commit_replay_ms'], 0.0)
        self.assertEqual(snapshot['release_replayed_imu_count'], 1)

    def test_unforwarded_vicon_does_not_replace_onboard_release_seed(self):
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

        self.assertTrue(released['valid'])
        snapshot = released['release_snapshot']
        self.assertEqual(snapshot['initial_position_seed_m'], [0.4, -0.3, 0.9])
        self.assertIsNone(snapshot['external_position_seed_m'])
        self.assertEqual(released['post_release_ekf']['position_update_count'], 0)
        self.assertFalse(released['vicon'][
            'position_forwarded_to_onboard_ekf'
        ])

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
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
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
            RELEASE_EVENT_TIME_SOURCE,
        )

    def test_release_event_without_nearby_causal_gyro_is_terminal(self):
        now = [1000.05]
        shadow = ContactAttitudeShadow(
            config=ContactAttitudeShadowConfig(
                queue_capacity=64,
                alignment_stationary_window_ms=20,
                alignment_min_state_samples=3,
                alignment_legacy_yaw_deg=90.0,
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

    def test_release_preview_replay_resumes_without_duplicate_propagation(self):
        class OneShotBudgetClock:
            def __init__(self):
                self.armed = False
                self.calls = 0

            def arm(self):
                self.armed = True
                self.calls = 0

            def __call__(self):
                if not self.armed:
                    return 0.0
                self.calls += 1
                # release() checks the clock five times before replay and once
                # before each sample. Expire before the second propagation.
                return 1.0 if self.calls == 7 else 0.0

        budget_clock = OneShotBudgetClock()
        shadow = ContactAttitudeShadow(
            config=ContactAttitudeShadowConfig(
                queue_capacity=128,
                max_drain_packets=128,
                max_drain_time_s=1.0,
                max_release_processing_time_s=0.01,
                alignment_stationary_window_ms=20,
                alignment_min_state_samples=3,
            ),
            observer_config=ContactAttitudeConfig(
                alignment_window_ms=20,
                alignment_min_samples=3,
                alignment_max_sample_gap_ms=10,
            ),
            clock=lambda: 1000.05,
            perf_clock=budget_clock,
        )
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        sequence = self.add_release_state(shadow, sequence, 21)
        for timestamp in range(21, 26):
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
                    'contactImu.vx': 0.2,
                    'contactImu.vy': 0.0,
                    'contactImu.vz': 0.0,
                    'contactImu.epoch': timestamp,
                },
            ))
            sequence += 1
        shadow.drain()

        budget_clock.arm()
        pending = shadow.begin_release_candidate(
            [0.0, 0.0, 1.0],
            [0.2, 0.0, 0.0],
            release_event_monotonic_s=1000.021,
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
            release_event_arduino_time_ms=21,
        )

        self.assertFalse(pending['release_candidate_active'])
        self.assertEqual(pending['invalid_reason'], 'release_replay_pending')
        replay_work = shadow._pending_release['replay_work']
        self.assertEqual(replay_work['replayed_imu_count'], 1)
        self.assertEqual(
            replay_work['ekf'].snapshot().unwrapped_timestamp_ms, 22
        )

        prepared = shadow.begin_release_candidate(
            [0.0, 0.0, 1.0],
            [0.2, 0.0, 0.0],
            release_event_monotonic_s=1000.021,
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
            release_event_arduino_time_ms=21,
        )

        self.assertTrue(prepared['release_candidate_active'], prepared)
        timeline = shadow._release_preview
        self.assertEqual(timeline['replayed_imu_count'], 4)
        self.assertEqual(
            [entry['unwrapped_timestamp_ms'] for entry in timeline['history']],
            [21, 22, 23, 24, 25],
        )

    def test_release_confirmation_fails_if_preview_replay_is_not_ready(self):
        calls = [0]
        armed = [False]

        def budget_clock():
            if not armed[0]:
                return 0.0
            calls[0] += 1
            return 1.0 if calls[0] == 7 else 0.0

        shadow = ContactAttitudeShadow(
            config=ContactAttitudeShadowConfig(
                queue_capacity=128,
                max_drain_packets=128,
                max_drain_time_s=1.0,
                max_release_processing_time_s=0.01,
                alignment_stationary_window_ms=20,
                alignment_min_state_samples=3,
            ),
            observer_config=ContactAttitudeConfig(
                alignment_window_ms=20,
                alignment_min_samples=3,
                alignment_max_sample_gap_ms=10,
            ),
            clock=lambda: 1000.05,
            perf_clock=budget_clock,
        )
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        sequence = self.add_release_state(shadow, sequence, 21)
        for timestamp in range(21, 26):
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
                    'contactImu.vx': 0.2,
                    'contactImu.vy': 0.0,
                    'contactImu.vz': 0.0,
                    'contactImu.epoch': timestamp,
                },
            ))
            sequence += 1
        shadow.drain()
        armed[0] = True
        pending = shadow.begin_release_candidate(
            [0.0, 0.0, 1.0],
            [0.2, 0.0, 0.0],
            release_event_monotonic_s=1000.021,
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
            release_event_arduino_time_ms=21,
        )
        self.assertEqual(pending['invalid_reason'], 'release_replay_pending')

        confirmed = shadow.release(
            [0.0, 0.0, 1.0],
            [0.2, 0.0, 0.0],
            release_event_monotonic_s=1000.021,
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
            release_event_arduino_time_ms=21,
            release_confirmation_monotonic_s=1000.071,
            release_confirmation_arduino_time_ms=71,
        )

        self.assertFalse(confirmed['valid'])
        self.assertEqual(
            confirmed['fatal_reason'],
            'release_preview_not_ready_at_confirmation',
        )
        self.assertIsNone(confirmed['post_release_ekf'])

    def test_drain_deadline_cannot_bypass_required_release_preview(self):
        class DrainExpiryClock:
            def __init__(self):
                self.armed = False
                self.calls = 0

            def arm(self):
                self.armed = True
                self.calls = 0

            def __call__(self):
                if not self.armed:
                    return 0.0
                self.calls += 1
                # With an empty queue, the fifth call is release()'s check
                # immediately after drain(). Expire only that first budget;
                # later calls share the new 1.0-second origin.
                return 0.0 if self.calls < 5 else 1.0

        budget_clock = DrainExpiryClock()
        shadow = ContactAttitudeShadow(
            config=ContactAttitudeShadowConfig(
                queue_capacity=128,
                max_drain_packets=128,
                max_drain_time_s=1.0,
                max_release_processing_time_s=0.01,
                alignment_stationary_window_ms=20,
                alignment_min_state_samples=3,
            ),
            observer_config=ContactAttitudeConfig(
                alignment_window_ms=20,
                alignment_min_samples=3,
                alignment_max_sample_gap_ms=10,
            ),
            clock=lambda: 1000.05,
            perf_clock=budget_clock,
        )
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        sequence = self.add_release_state(shadow, sequence, 21)
        shadow.enqueue_packet(packet(
            sequence, 'GYRO_1KHZ', 21,
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
                'contactImu.vx': 0.2,
                'contactImu.vy': 0.0,
                'contactImu.vz': 0.0,
                'contactImu.epoch': 21,
            },
        ))
        shadow.drain()

        budget_clock.arm()
        deferred = shadow.begin_release_candidate(
            [0.0, 0.0, 1.0], [0.2, 0.0, 0.0],
            release_event_monotonic_s=1000.021,
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
            release_event_arduino_time_ms=21,
        )
        self.assertEqual(
            deferred['invalid_reason'], 'release_processing_deferred'
        )
        self.assertIsNone(shadow._pending_release)
        self.assertIsNone(shadow._release_preview)

        confirmed = shadow.release(
            [0.0, 0.0, 1.0], [0.2, 0.0, 0.0],
            release_event_monotonic_s=1000.021,
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
            release_event_arduino_time_ms=21,
            release_confirmation_monotonic_s=1000.071,
            release_confirmation_arduino_time_ms=71,
        )

        self.assertFalse(confirmed['valid'])
        self.assertEqual(
            confirmed['fatal_reason'],
            'release_preview_not_ready_at_confirmation',
        )
        self.assertIsNone(confirmed['post_release_ekf'])

    def test_default_replay_budget_covers_50ms_confirmation_dwell(self):
        shadow = ContactAttitudeShadow(
            config=ContactAttitudeShadowConfig(
                queue_capacity=512,
                max_drain_packets=512,
                max_drain_time_s=1.0,
                alignment_stationary_window_ms=20,
                alignment_min_state_samples=3,
            ),
            observer_config=ContactAttitudeConfig(
                alignment_window_ms=20,
                alignment_min_samples=3,
                alignment_max_sample_gap_ms=10,
            ),
            clock=lambda: 1000.20,
            # This assertion covers the derived sample budget. Dedicated
            # synthetic-clock tests above cover wall-time deferral/fail-close
            # behavior without depending on workstation scheduling jitter.
            perf_clock=lambda: 0.0,
        )
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        for timestamp in range(21, 72):
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
                    'contactImu.vx': 0.2,
                    'contactImu.vy': 0.0,
                    'contactImu.vz': 0.0,
                    'contactImu.epoch': timestamp,
                },
            ))
            sequence += 1
        shadow.drain()

        released = shadow.release(
            [0.0, 0.0, 1.0], [0.2, 0.0, 0.0],
            release_event_monotonic_s=1000.021,
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
            release_confirmation_monotonic_s=1000.071,
        )

        self.assertTrue(released['valid'], released)
        self.assertEqual(
            released['release_snapshot']['release_replayed_imu_count'], 50
        )
        self.assertTrue(
            released['release_snapshot'][
                'state_seed_same_atomic_packed_epoch'
            ]
        )
        self.assertEqual(
            released['release_snapshot'][
                'release_replay_imu_quality_accepted_count'
            ],
            50,
        )
        self.assertIsNone(released['fatal_reason'])

    def test_default_history_retains_seed_across_250ms_candidate_lead(self):
        shadow = ContactAttitudeShadow(
            config=ContactAttitudeShadowConfig(
                queue_capacity=512,
                max_drain_packets=512,
                max_drain_time_s=1.0,
                alignment_stationary_window_ms=20,
                alignment_min_state_samples=3,
            ),
            observer_config=ContactAttitudeConfig(
                alignment_window_ms=20,
                alignment_min_samples=3,
                alignment_max_sample_gap_ms=10,
            ),
            clock=lambda: 1000.27,
            perf_clock=lambda: 0.0,
        )
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        for timestamp in range(21, 272):
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
                    'contactImu.vx': 0.2,
                    'contactImu.vy': 0.0,
                    'contactImu.vz': 0.0,
                    'contactImu.epoch': timestamp,
                },
            ))
            sequence += 1
        shadow.drain()

        released = shadow.release(
            [0.0, 0.0, 1.0], [0.2, 0.0, 0.0],
            release_event_monotonic_s=1000.021,
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
            release_confirmation_monotonic_s=1000.271,
        )

        self.assertTrue(released['valid'])
        self.assertEqual(
            released['release_snapshot']['release_replayed_imu_count'], 250
        )
        self.assertEqual(
            released['release_snapshot']['cf_timestamp_ms'], 21
        )
        self.assertIsNone(released['fatal_reason'])

    def test_first_unloaded_preview_runs_ekf_during_confirmation_dwell(self):
        shadow = ContactAttitudeShadow(
            config=ContactAttitudeShadowConfig(
                queue_capacity=512,
                max_drain_packets=512,
                max_drain_time_s=1.0,
                alignment_stationary_window_ms=20,
                alignment_min_state_samples=3,
                alignment_legacy_yaw_deg=90.0,
                post_release_imu_quality_calibrated=True,
                post_release_imu_quality_provenance_id=(
                    'crazysim_exact_imu_quality_v1'
                ),
                absolute_yaw_reference_certified=True,
                absolute_yaw_reference_certificate_id=(
                    'crazysim_yaw_reference_v1'
                ),
                absolute_yaw_reference_yaw_deg=90.0,
            ),
            observer_config=ContactAttitudeConfig(
                alignment_window_ms=20,
                alignment_min_samples=3,
                alignment_max_sample_gap_ms=10,
            ),
            clock=lambda: 1000.075,
        )
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()

        def enqueue_packed(timestamp):
            nonlocal sequence
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
                    'contactImu.vx': 0.2,
                    'contactImu.vy': 0.0,
                    'contactImu.vz': 0.0,
                    'contactImu.epoch': timestamp,
                },
            ))
            sequence += 1

        enqueue_packed(21)
        shadow.drain()
        preview = shadow.begin_release_candidate(
            [0.0, 0.0, 1.0], [0.2, 0.0, 0.0],
            active_setpoint={'roll_deg': 99.0},
            release_event_monotonic_s=1000.021,
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
            release_event_arduino_time_ms=21,
            release_event_cf_timestamp_ms=21,
            release_event_unwrapped_cf_timestamp_ms=21,
            release_clock_mapping_basis=(
                FIRMWARE_SHARED_CLOCK_RELEASE_LATCH_BASIS
            ),
            release_clock_mapping_uncertainty_ms=0.0,
            release_clock_mapping_calibration_id=(
                'firmware_shared_clock_release_latch_calibration_v1'
            ),
        )
        self.assertTrue(preview['release_candidate_active'])
        self.assertIsNotNone(preview['release_candidate_ekf'])
        self.assertIsNone(preview['post_release_ekf'])
        self.assertEqual(preview['observer']['phase'], 'contact')

        for timestamp in range(22, 72):
            enqueue_packed(timestamp)
            if timestamp % 10 == 1:
                elapsed_s = (timestamp - 21) / 1000.0
                shadow.enqueue_mocap_frame(mocap_packet(
                    sequence, timestamp,
                    position=(0.2 * elapsed_s, 0.0, 1.0),
                    cf_timestamp_ms=timestamp,
                ))
                sequence += 1
            shadow.drain()

        before_commit = shadow.snapshot()['release_candidate_ekf']
        preview_ekf = shadow._release_preview['ekf']
        with (
            mock.patch.object(
                preview_ekf, 'propagate',
                side_effect=AssertionError(
                    'confirmation must not replay IMU'
                ),
            ),
            mock.patch.object(
                preview_ekf, 'update_extpos',
                side_effect=AssertionError(
                    'confirmation must not replay position'
                ),
            ),
        ):
            released = shadow.release(
                [9.0, 9.0, 9.0], [9.0, 9.0, 9.0],
                active_setpoint={'roll_deg': -99.0},
                release_event_monotonic_s=1000.021,
                release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
                release_event_arduino_time_ms=21,
                release_event_cf_timestamp_ms=21,
                release_event_unwrapped_cf_timestamp_ms=21,
                release_confirmation_monotonic_s=1000.080,
                release_confirmation_arduino_time_ms=71,
                release_clock_mapping_basis=(
                    FIRMWARE_SHARED_CLOCK_RELEASE_LATCH_BASIS
                ),
                release_clock_mapping_uncertainty_ms=0.0,
                release_clock_mapping_calibration_id=(
                    'firmware_shared_clock_release_latch_calibration_v1'
                ),
            )

        self.assertTrue(released['valid'])
        self.assertFalse(released['release_candidate_active'])
        self.assertEqual(released['observer']['phase'], 'released')
        self.assertEqual(
            released['post_release_ekf']['cf_timestamp_ms'],
            before_commit['cf_timestamp_ms'],
        )
        self.assertEqual(
            released['post_release_ekf']['position_update_count'],
            before_commit['position_update_count'],
        )
        self.assertEqual(
            released['release_snapshot']['release_replayed_imu_count'], 50
        )
        self.assertEqual(
            released['release_snapshot']['release_event_cf_timestamp_ms'], 21
        )
        self.assertEqual(
            released['release_snapshot'][
                'release_event_unwrapped_cf_timestamp_ms'
            ],
            21,
        )
        self.assertEqual(
            released['release_snapshot'][
                'release_clock_mapping_uncertainty_ms'
            ],
            0.0,
        )
        self.assertEqual(
            released['release_snapshot']['active_setpoint']['roll_deg'], 99.0
        )
        epoch = released['post_release_control_epoch']
        self.assertTrue(epoch['strict_atomic_imu'])
        self.assertEqual(
            released['post_release_strict_atomic_imu_count'], 50
        )
        self.assertEqual(released['post_release_nonatomic_imu_count'], 0)
        self.assertEqual(
            released['post_release_imu_quality_accepted_count'], 50
        )
        self.assertEqual(
            released['strict_position_timestamp_uncertainties_ms'],
            [0.0] * 5,
        )
        self.assertEqual(
            released['max_strict_position_timestamp_uncertainty_ms'], 0.0
        )
        decision = evaluate_post_release_estimator(
            released,
            now_cf_timestamp_ms=epoch['cf_timestamp_ms'],
            now_unwrapped_cf_timestamp_ms=epoch['unwrapped_timestamp_ms'],
            now_timestamp_basis=epoch['timestamp_basis'],
            now_host_receive_age_s=epoch['host_receive_age_s'],
            config=PostReleaseEstimatorGateConfig(enabled=True),
        )
        self.assertTrue(decision.estimator_control_eligible, decision)
        self.assertFalse(decision.gate_grants_command_authority)
        self.assertEqual(
            decision.post_release_device_time_coverage_ms, 50.0
        )
        self.assertEqual(decision.position_update_span_ms, 40.0)
        self.assertEqual(decision.inertial_propagation_samples, 50)

    def test_duplicate_imu_epoch_is_idempotent_across_release_lifecycle(self):
        shadow = self.make_shadow(config=ContactAttitudeShadowConfig(
            queue_capacity=128,
            max_drain_packets=128,
            max_drain_time_s=1.0,
            alignment_stationary_window_ms=20,
            alignment_min_state_samples=3,
        ))
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        sequence = self.add_release_state(shadow, sequence, 21)

        def packed_imu(timestamp, host_time_s):
            nonlocal sequence
            result = CfLogPacket(
                sequence,
                'GYRO_1KHZ',
                timestamp,
                host_time_s,
                {
                    'contactImu.gx': 0.0,
                    'contactImu.gy': 0.0,
                    'contactImu.gz': 0.0,
                    'contactImu.ax': 0.0,
                    'contactImu.ay': 0.0,
                    'contactImu.az': 1.0,
                    'contactImu.px': 0.0,
                    'contactImu.py': 0.0,
                    'contactImu.pz': 1.0,
                    'contactImu.vx': 0.2,
                    'contactImu.vy': 0.0,
                    'contactImu.vz': 0.0,
                    'contactImu.epoch': timestamp,
                },
                host_receive_monotonic_s=host_time_s,
                transport_cf_timestamp_ms=timestamp,
                source_cf_timestamp_basis=CONTACT_SOURCE_TIMESTAMP_BASIS,
                source_snapshot_atomic=True,
            )
            sequence += 1
            return result

        shadow.enqueue_packet(packed_imu(21, 1000.021))
        shadow.drain()
        contact_history_length = len(shadow._contact_imu_history)
        shadow.enqueue_packet(packed_imu(21, 1000.900))
        shadow.drain()
        self.assertEqual(
            len(shadow._contact_imu_history), contact_history_length
        )
        self.assertAlmostEqual(
            shadow._packet_monotonic_time(shadow._latest_gyro), 1000.021
        )

        started = shadow.begin_release_candidate(
            [0.0, 0.0, 1.0],
            [0.2, 0.0, 0.0],
            release_event_monotonic_s=1000.021,
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
            release_event_arduino_time_ms=21,
        )
        self.assertTrue(started['release_candidate_active'], started)
        shadow.enqueue_packet(packed_imu(22, 1000.022))
        shadow.drain()
        preview_history_length = len(shadow._release_preview['history'])
        preview_replayed_count = shadow._release_preview['replayed_imu_count']
        shadow.enqueue_packet(packed_imu(22, 1000.901))
        shadow.drain()
        self.assertEqual(
            len(shadow._release_preview['history']), preview_history_length
        )
        self.assertEqual(
            shadow._release_preview['replayed_imu_count'],
            preview_replayed_count,
        )
        self.assertAlmostEqual(
            shadow._release_preview['last_ekf_host_receive_monotonic_s'],
            1000.022,
        )

        released = shadow.release(
            [0.0, 0.0, 1.0],
            [0.2, 0.0, 0.0],
            release_event_monotonic_s=1000.021,
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
            release_event_arduino_time_ms=21,
            release_confirmation_monotonic_s=1000.071,
            release_confirmation_arduino_time_ms=71,
        )
        self.assertTrue(released['valid'], released)
        shadow.enqueue_packet(packed_imu(23, 1000.023))
        shadow.drain()
        released_history_length = len(shadow._ekf_history)
        shadow.enqueue_packet(packed_imu(23, 1000.902))
        result = shadow.drain()
        self.assertTrue(result['valid'], result)
        self.assertEqual(len(shadow._ekf_history), released_history_length)
        self.assertAlmostEqual(
            shadow._last_ekf_host_receive_monotonic_s, 1000.023
        )

    def test_unloaded_preview_cancel_keeps_contact_gyro_timeline(self):
        shadow = self.make_shadow()
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        sequence = self.add_release_state(shadow, sequence, 21)
        shadow.enqueue_packet(packet(
            sequence, 'ACC_ALIGN', 21,
            **{'acc.x': 0.0, 'acc.y': 0.0, 'acc.z': 1.0},
        ))
        shadow.enqueue_packet(packet(
            sequence + 1, 'GYRO_1KHZ', 21,
            **{'gyro.x': 0.0, 'gyro.y': 0.0, 'gyro.z': 0.0},
        ))
        shadow.drain()
        started = shadow.begin_release_candidate(
            [0.0, 0.0, 1.0], [0.0, 0.0, 0.0],
            release_event_arduino_time_ms=21,
        )
        self.assertTrue(started['release_candidate_active'])

        cancelled = shadow.cancel_release_candidate()

        self.assertFalse(cancelled['release_candidate_active'])
        self.assertIsNone(cancelled['pending_release_cf_timestamp_ms'])
        self.assertEqual(cancelled['observer']['phase'], 'contact')
        self.assertEqual(cancelled['observer']['cf_timestamp_ms'], 21)

    def test_cancelled_preview_does_not_contaminate_second_release_candidate(self):
        shadow = self.make_shadow(config=ContactAttitudeShadowConfig(
            queue_capacity=128,
            max_drain_packets=128,
            max_drain_time_s=1.0,
            alignment_stationary_window_ms=20,
            alignment_min_state_samples=3,
        ))
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()

        def enqueue_packed(timestamp):
            nonlocal sequence
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
                    'contactImu.vx': 0.2,
                    'contactImu.vy': 0.0,
                    'contactImu.vz': 0.0,
                    'contactImu.epoch': timestamp,
                },
            ))
            sequence += 1

        enqueue_packed(21)
        shadow.drain()
        first = shadow.begin_release_candidate(
            [0.0, 0.0, 1.0], [0.2, 0.0, 0.0],
            release_event_monotonic_s=1000.021,
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
            release_event_arduino_time_ms=21,
        )
        self.assertTrue(first['release_candidate_active'], first)
        enqueue_packed(22)
        shadow.enqueue_mocap_frame(mocap_packet(
            sequence, 22, position=(0.0002, 0.0, 1.0),
            cf_timestamp_ms=22,
        ))
        sequence += 1
        first_updated = shadow.drain()
        self.assertEqual(
            first_updated['release_candidate_ekf']['position_update_count'], 1
        )

        cancelled = shadow.cancel_release_candidate()
        self.assertFalse(cancelled['release_candidate_active'])
        self.assertEqual(cancelled['observer']['cf_timestamp_ms'], 22)
        enqueue_packed(23)
        shadow.drain()
        second = shadow.begin_release_candidate(
            [0.0, 0.0, 1.0], [0.2, 0.0, 0.0],
            release_event_monotonic_s=1000.023,
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
            release_event_arduino_time_ms=23,
        )
        self.assertTrue(second['release_candidate_active'], second)
        self.assertEqual(
            second['release_candidate_ekf']['position_update_count'], 0
        )

        enqueue_packed(24)
        shadow.enqueue_mocap_frame(mocap_packet(
            sequence, 24, position=(0.0002, 0.0, 1.0),
            cf_timestamp_ms=24,
        ))
        sequence += 1
        shadow.drain()
        released = shadow.release(
            [0.0, 0.0, 1.0], [0.2, 0.0, 0.0],
            release_event_monotonic_s=1000.023,
            release_event_time_source=RELEASE_EVENT_TIME_SOURCE,
            release_event_arduino_time_ms=23,
            release_confirmation_monotonic_s=1000.073,
            release_confirmation_arduino_time_ms=73,
        )

        self.assertTrue(released['valid'], released)
        self.assertEqual(released['release_snapshot']['cf_timestamp_ms'], 23)
        self.assertEqual(released['post_release_ekf']['position_update_count'], 1)

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

    def test_pure_inertial_release_keeps_vicon_evaluation_only(self):
        shadow = self.make_shadow(config=ContactAttitudeShadowConfig(
            queue_capacity=64,
            alignment_stationary_window_ms=20,
            alignment_min_state_samples=3,
            fuse_vicon_position_after_release=False,
        ))
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        sequence = self.add_release_state(shadow, sequence, 21)
        shadow.enqueue_packet(packet(
            sequence, 'GYRO_1KHZ', 21,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        sequence += 1
        shadow.drain()
        released = shadow.release([0, 0, 1], [0.2, 0, 0])
        shadow.enqueue_packet(packet(
            sequence, 'ACC_ALIGN', 22,
            **{'acc.x': 0, 'acc.y': 0, 'acc.z': 1},
        ))
        shadow.enqueue_packet(packet(
            sequence + 1, 'GYRO_1KHZ', 22,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        shadow.enqueue_mocap_frame(mocap_packet(
            sequence + 2, 22, position=(0.5, 0, 1),
            quaternion=quaternion_from_native_rpy(0.2, -0.1, 0.3),
        ))

        result = shadow.drain()
        explicit = shadow.update_extpos([0.5, 0, 1])

        self.assertTrue(released['valid'])
        self.assertFalse(released['release_snapshot'][
            'shadow_position_fusion_enabled'
        ])
        self.assertEqual(
            released['release_snapshot'][
                'post_release_position_observation_source'
            ],
            'none_pure_inertial',
        )
        self.assertEqual(result['post_release_ekf']['position_update_count'], 0)
        self.assertEqual(explicit['post_release_ekf']['position_update_count'], 0)
        self.assertIsNotNone(result['vicon'])
        self.assertFalse(result['vicon_orientation_used_by_shadow_ekf'])

    def test_position_older_than_fixed_lag_window_is_skipped_nonfatally(self):
        shadow = self.make_shadow(config=ContactAttitudeShadowConfig(
            queue_capacity=128,
            max_drain_packets=128,
            max_drain_time_s=1.0,
            alignment_stationary_window_ms=20,
            alignment_min_state_samples=3,
            max_position_replay_samples=2,
        ))
        _, sequence = self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        sequence = self.add_release_state(shadow, sequence, 21)
        shadow.enqueue_packet(packet(
            sequence, 'GYRO_1KHZ', 21,
            **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
        ))
        sequence += 1
        shadow.drain()
        self.assertTrue(shadow.release([0, 0, 1], [0.2, 0, 0])['valid'])
        for timestamp in range(22, 31):
            shadow.enqueue_packet(packet(
                sequence, 'ACC_ALIGN', timestamp,
                **{'acc.x': 0, 'acc.y': 0, 'acc.z': 1},
            ))
            sequence += 1
            shadow.enqueue_packet(packet(
                sequence, 'GYRO_1KHZ', timestamp,
                **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
            ))
            sequence += 1
        shadow.drain()
        shadow.enqueue_mocap_frame(mocap_packet(
            sequence, 22, position=(0.0, 0.0, 1.0), cf_timestamp_ms=22,
        ))

        result = shadow.drain()

        self.assertTrue(result['valid'])
        self.assertIsNone(result['fatal_reason'])
        self.assertEqual(result['skipped_position_packets'], 1)
        self.assertTrue(result['post_release_ekf']['valid'])

    def test_position_budget_resume_does_not_replay_committed_frame(self):
        for preview_mode in (False, True):
            with self.subTest(preview_mode=preview_mode):
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
                    shadow, sequence + 1, 21
                )
                release_method = (
                    shadow.begin_release_candidate
                    if preview_mode else shadow.release
                )
                release_method([0, 0, 1], [0.2, 0, 0])
                for timestamp in (22, 23):
                    shadow.enqueue_packet(packet(
                        sequence, 'ACC_ALIGN', timestamp,
                        **{'acc.x': 0, 'acc.y': 0, 'acc.z': 1},
                    ))
                    sequence += 1
                    shadow.enqueue_packet(packet(
                        sequence, 'GYRO_1KHZ', timestamp,
                        **{'gyro.x': 0, 'gyro.y': 0, 'gyro.z': 0},
                    ))
                    sequence += 1
                shadow.drain()
                for timestamp, position_x in ((22, 0.0002), (23, 0.0004)):
                    shadow._process_mocap_frame(mocap_packet(
                        sequence, timestamp,
                        position=(position_x, 0, 1),
                        cf_timestamp_ms=timestamp,
                    ))
                    sequence += 1

                ticks = iter((0.0, 0.0, 1.0))
                shadow._perf_clock = lambda: next(ticks)
                fuse = (
                    shadow._fuse_release_preview_positions
                    if preview_mode else shadow._fuse_pending_positions
                )
                self.assertFalse(fuse(deadline=0.5))
                snapshot = shadow.snapshot()
                estimate = (
                    snapshot['release_candidate_ekf']
                    if preview_mode else snapshot['post_release_ekf']
                )
                self.assertEqual(estimate['position_update_count'], 1)
                pending = (
                    shadow._release_preview['pending_positions']
                    if preview_mode else shadow._pending_positions
                )
                self.assertEqual(len(pending), 1)
                self.assertIsNone(snapshot['fatal_reason'])

                shadow._perf_clock = lambda: 0.0
                self.assertTrue(fuse(deadline=0.5))
                snapshot = shadow.snapshot()
                estimate = (
                    snapshot['release_candidate_ekf']
                    if preview_mode else snapshot['post_release_ekf']
                )
                self.assertEqual(estimate['position_update_count'], 2)
                pending = (
                    shadow._release_preview['pending_positions']
                    if preview_mode else shadow._pending_positions
                )
                self.assertEqual(len(pending), 0)
                self.assertIsNone(snapshot['fatal_reason'])

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
        self.assertTrue(comparison['host_availability_time_aligned'])
        self.assertFalse(comparison['comparison_time_aligned'])
        self.assertFalse(comparison['comparison_scientifically_valid'])

    def test_host_after_wait_is_not_promoted_to_vicon_capture_time(self):
        shadow = self.make_shadow()
        self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow.update_onboard_state(
            [0, 0, 1], [0, 0, 0], [0, 0, 0],
            state_time_s=1000.02,
            host_receive_monotonic_s=1000.02,
            cf_timestamp_ms=20,
            cf_timestamp_basis=CF_LOG_TRANSPORT_TIMESTAMP_BASIS,
        )
        frame = mocap_packet(
            20, 20, quaternion=quaternion_from_native_rpy(0, 0, 0),
            cf_timestamp_ms=20,
        )
        frame.data['mocap_timing'] = {
            'frame_time_scope': 'host_after_wait',
            'source_capture_time_available': False,
            'wait_return_monotonic_s': 1000.02,
        }
        shadow.enqueue_mocap_frame(frame)

        comparison = shadow.drain()['comparison']

        self.assertTrue(comparison['vicon_capture_mapped_to_cf_clock'])
        self.assertFalse(comparison['vicon_capture_timestamp_available'])
        self.assertFalse(comparison['comparison_time_aligned'])
        self.assertFalse(comparison['comparison_scientifically_valid'])
        mocap = shadow._mocap_snapshot()
        self.assertEqual(
            mocap['position_event_time_basis'],
            'pi_mocap_wait_return_monotonic',
        )
        self.assertEqual(
            mocap['position_event_pi_receive_monotonic_s'], 1000.02,
        )

    def test_comparison_rejects_onboard_state_from_untrusted_clock_basis(self):
        shadow = self.make_shadow()
        self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow.update_onboard_state(
            [0, 0, 1], [0, 0, 0], [0, 0, 0],
            state_time_s=1000.02,
            host_receive_monotonic_s=1000.02,
            cf_timestamp_ms=20,
            cf_timestamp_basis='host_callback_time_v1',
        )
        shadow.enqueue_mocap_frame(mocap_packet(
            20, 20, quaternion=quaternion_from_native_rpy(0, 0, 0),
            cf_timestamp_ms=20,
        ))

        comparison = shadow.drain()['comparison']

        self.assertTrue(comparison['vicon_capture_timestamp_available'])
        self.assertTrue(comparison['vicon_capture_mapped_to_cf_clock'])
        self.assertIsNone(comparison['comparison_onboard_cf_timestamp_ms'])
        self.assertFalse(comparison['comparison_time_aligned'])
        self.assertFalse(comparison['comparison_scientifically_valid'])

    def test_trusted_mocap_comparison_uses_historical_common_cf_epoch(self):
        shadow = self.make_shadow()
        self.align(shadow)
        shadow.begin_contact()
        shadow.confirm_contact()
        shadow.update_onboard_state(
            [0, 0, 1], [0, 0, 0], [0, 0, math.pi / 2],
            state_time_s=1000.02,
            host_receive_monotonic_s=1000.02,
            cf_timestamp_ms=20,
            cf_timestamp_basis=CF_LOG_TRANSPORT_TIMESTAMP_BASIS,
        )
        truth = quaternion_from_native_rpy(0, 0, math.pi / 2)
        shadow.enqueue_mocap_frame(mocap_packet(
            20, 20, quaternion=truth, cf_timestamp_ms=20,
        ))
        shadow.enqueue_packet(packet(
            21, 'ACC_ALIGN', 21,
            **{'acc.x': 0, 'acc.y': 0, 'acc.z': 1},
        ))
        shadow.enqueue_packet(packet(
            22, 'GYRO_1KHZ', 21,
            **{'gyro.x': 1000, 'gyro.y': 0, 'gyro.z': 0},
        ))

        snapshot = shadow.drain()
        comparison = snapshot['comparison']

        self.assertGreater(
            abs(snapshot['shadow_estimate']['legacy_rpy_deg'][0]), 0.5
        )
        self.assertTrue(comparison['vicon_capture_timestamp_available'])
        self.assertTrue(comparison['vicon_capture_mapped_to_cf_clock'])
        self.assertTrue(comparison['comparison_time_aligned'])
        self.assertTrue(comparison['comparison_scientifically_valid'])
        self.assertEqual(comparison['comparison_reference_cf_timestamp_ms'], 20)
        self.assertEqual(
            comparison['comparison_reference_unwrapped_timestamp_ms'], 20
        )
        self.assertEqual(comparison['comparison_shadow_cf_timestamp_ms'], 20)
        self.assertEqual(comparison['comparison_onboard_cf_timestamp_ms'], 20)
        self.assertEqual(
            comparison['vicon_minus_shadow_roll_pitch_deg'], [0.0, 0.0]
        )

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

    def test_snapshot_covariance_eigendecomposition_fails_closed(self):
        cases = (
            (
                'post_release_covariance_eigendecomposition_failed',
                {'side_effect': np.linalg.LinAlgError('did not converge')},
            ),
            (
                'post_release_covariance_eigendecomposition_invalid',
                {'return_value': np.r_[np.zeros(14), np.nan]},
            ),
        )
        for expected_reason, patch_kwargs in cases:
            with self.subTest(reason=expected_reason):
                shadow = ContactAttitudeShadow()
                shadow._ekf = PostReleaseInertialEkf(
                    position_m=[0.0, 0.0, 1.0],
                    velocity_m_s=[0.0, 0.0, 0.0],
                    quaternion_wxyz=[1.0, 0.0, 0.0, 0.0],
                    gyro_bias_rad_s=[0.0, 0.0, 0.0],
                    cf_timestamp_ms=0,
                )
                with mock.patch(
                    'Interaction.contact_attitude_shadow.'
                    'np.linalg.eigvalsh',
                    **patch_kwargs,
                ):
                    snapshot = shadow.snapshot()

                self.assertFalse(snapshot['valid'])
                self.assertEqual(
                    snapshot['invalid_reason'], expected_reason
                )
                self.assertFalse(snapshot['post_release_ekf']['valid'])
                self.assertEqual(
                    snapshot['post_release_ekf']['reason'], expected_reason
                )
                self.assertFalse(snapshot['post_release_ekf'][
                    'state_covariance_same_epoch'
                ])
                self.assertIsNone(snapshot['post_release_ekf'][
                    'covariance_min_eigenvalue'
                ])

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

    def test_drain_time_budget_defers_without_dropping_or_fatal_error(self):
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
        self.assertIsNone(result['fatal_reason'])
        self.assertEqual(
            result['invalid_reason'], 'shadow_drain_time_budget_exceeded'
        )
        self.assertEqual(result['drain_budget_exceeded_count'], 1)
        self.assertEqual(result['dropped_packets'], 0)
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
