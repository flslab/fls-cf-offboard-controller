"""Packet clock metadata is additive logging, never a control-time change."""

import collections
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock, patch

from Interaction.log_manager import (
    CONTACT_SOURCE_TIMESTAMP_BASIS,
    CfLogPacket,
    InteractionLogger,
    MocapFramePacket,
    reconstruct_contact_source_timestamp_ms,
)


class LogPacketTimestampTests(unittest.TestCase):
    def make_logger(self, *, enabled=True):
        logger = InteractionLogger.__new__(InteractionLogger)
        logger.cf_log_group_times = {}
        logger.cf_log_group_packets = collections.defaultdict(
            lambda: collections.deque(maxlen=1000)
        )
        logger.cf_log_packet_lock = threading.Lock()
        logger.cf_log_callback_lock = threading.Lock()
        logger._accepting_cf_log_callbacks = True
        logger._cf_log_packet_sequence = 0
        logger._cf_log_packet_listeners = []
        logger._mocap_frame_sequence = 0
        logger._mocap_frame_listeners = []
        logger.cf_log_data = {'VEL_ORI': {'stateEstimate.vy': {'data': []}}}
        logger.groups = {'frames': []}
        logger.group_kfs = {}
        logger.live_logger = Mock() if enabled else None
        return logger

    def test_callback_arriving_after_shutdown_is_ignored(self):
        logger = self.make_logger()
        logger.cf_var_logger = [Mock()]

        logger.stop()
        logger._cf_log_group_callback(
            1234, {'stateEstimate.vy': 0.2},
            SimpleNamespace(name='VEL_ORI'),
        )

        logger.cf_var_logger[0].stop.assert_called_once_with()
        logger.live_logger.close.assert_called_once_with()
        logger.live_logger.write.assert_not_called()
        self.assertEqual(logger.cf_log_group_times, {})
        self.assertEqual(logger.cf_log_data['VEL_ORI'][
            'stateEstimate.vy']['data'], [])

    def test_shutdown_waits_for_in_flight_callback_before_close(self):
        logger = self.make_logger()
        logger.cf_var_logger = [Mock()]
        write_started = threading.Event()
        release_write = threading.Event()
        close_called = threading.Event()

        def blocking_write(_record):
            write_started.set()
            self.assertTrue(release_write.wait(1.0))

        logger.live_logger.write.side_effect = blocking_write
        logger.live_logger.close.side_effect = close_called.set
        callback = threading.Thread(
            target=logger._cf_log_group_callback,
            args=(
                1234,
                {'stateEstimate.vy': 0.2},
                SimpleNamespace(name='VEL_ORI'),
            ),
        )
        callback.start()
        self.assertTrue(write_started.wait(1.0))

        shutdown = threading.Thread(target=logger.stop)
        shutdown.start()
        self.assertFalse(close_called.wait(0.03))

        release_write.set()
        callback.join(1.0)
        shutdown.join(1.0)
        self.assertFalse(callback.is_alive())
        self.assertFalse(shutdown.is_alive())
        logger.live_logger.close.assert_called_once_with()

    def test_shadow_shutdown_waits_for_in_flight_mocap_write_before_close(self):
        logger = self.make_logger()
        logger.cf_var_logger = []
        logger.add_mocap_frame_listener(lambda _packet: None)
        write_started = threading.Event()
        release_write = threading.Event()
        close_called = threading.Event()

        def blocking_write(_record):
            write_started.set()
            self.assertTrue(release_write.wait(1.0))

        logger.live_logger.write.side_effect = blocking_write
        logger.live_logger.close.side_effect = close_called.set
        callback = threading.Thread(
            target=logger.add_log_entry,
            args=('frames', {'tvec': [0.0, 0.0, 1.0]}),
        )
        callback.start()
        self.assertTrue(write_started.wait(1.0))

        shutdown = threading.Thread(target=logger.stop)
        shutdown.start()
        self.assertFalse(close_called.wait(0.03))

        release_write.set()
        callback.join(1.0)
        shutdown.join(1.0)
        self.assertFalse(callback.is_alive())
        self.assertFalse(shutdown.is_alive())
        logger.live_logger.close.assert_called_once_with()

    def test_saved_metadata_preserves_host_time_and_raw_wrapping_counter(self):
        logger = self.make_logger()
        with patch('Interaction.log_manager.time.time',
                   side_effect=[1000.0, 1000.010]) as host_clock:
            logger._cf_log_group_callback(
                0xFFFFFE, {'stateEstimate.vy': 0.2},
                SimpleNamespace(name='VEL_ORI'),
            )
            logger._cf_log_group_callback(
                8, {'stateEstimate.vy': 0.1},
                SimpleNamespace(name='VEL_ORI'),
            )
        self.assertEqual(host_clock.call_count, 2)
        entries = [call.args[0] for call in logger.live_logger.write.call_args_list]
        self.assertEqual([entry['data']['cf_timestamp_ms'] for entry in entries],
                         [0xFFFFFE, 8])
        for entry, expected_time in zip(entries, (1000.0, 1000.010)):
            self.assertEqual(entry['type'], 'state')
            self.assertEqual(entry['group'], 'VEL_ORI')
            self.assertEqual(entry['data']['time'], expected_time)
            self.assertEqual(entry['data']['host_receive_time_s'], expected_time)

    def test_live_metadata_never_enters_runtime_packets_or_input(self):
        logger = self.make_logger()
        packet = {'stateEstimate.vy': 0.2}
        with patch('Interaction.log_manager.time.time', return_value=1000.0):
            logger._cf_log_group_callback(
                1234, packet, SimpleNamespace(name='VEL_ORI'),
            )
        expected = {'stateEstimate.vy': 0.2, 'time': 1000.0}
        self.assertEqual(packet, expected)
        nearest, skew = logger.get_nearest_group_log_data('VEL_ORI', 1000.02)
        self.assertEqual(nearest, expected)
        self.assertAlmostEqual(skew, 0.02)
        self.assertEqual(logger.get_latest_group_log_time('VEL_ORI'), 1000.0)
        self.assertEqual(logger.get_latest_group_log_data('VEL_ORI'),
                         {'stateEstimate.vy': 0.2})
        saved = logger.live_logger.write.call_args.args[0]['data']
        self.assertIsNot(saved, packet)
        self.assertNotIn('cf_packet_sequence', saved)
        packet['stateEstimate.vy'] = 99.0
        self.assertEqual(saved['stateEstimate.vy'], 0.2)

    def test_disabled_live_logger_preserves_callback_behavior(self):
        logger = self.make_logger(enabled=False)
        with patch('Interaction.log_manager.time.time', return_value=1000.0):
            logger._cf_log_group_callback(
                1234, {'stateEstimate.vy': 0.2},
                SimpleNamespace(name='VEL_ORI'),
            )
        self.assertEqual(logger.cf_log_group_times, {'VEL_ORI': 1000.0})
        self.assertEqual(list(logger.cf_log_group_packets['VEL_ORI']),
                         [{'stateEstimate.vy': 0.2, 'time': 1000.0}])

    def test_read_only_listener_receives_sequence_and_both_clocks(self):
        logger = self.make_logger()
        packets = []
        unsubscribe = logger.add_cf_packet_listener(packets.append)
        with patch('Interaction.log_manager.time.time', side_effect=[10.0, 10.1]):
            logger._cf_log_group_callback(
                50, {'stateEstimate.vy': 0.2}, SimpleNamespace(name='VEL_ORI')
            )
            logger._cf_log_group_callback(
                60, {'stateEstimate.vy': 0.3}, SimpleNamespace(name='VEL_ORI')
            )
        self.assertEqual([packet.sequence for packet in packets], [0, 1])
        self.assertIsInstance(packets[0], CfLogPacket)
        self.assertEqual(packets[0].cf_timestamp_ms, 50)
        self.assertEqual(packets[0].host_receive_time_s, 10.0)
        self.assertNotIn('time', packets[0].data)
        with self.assertRaises(TypeError):
            packets[0].data['stateEstimate.vy'] = 99.0
        entries = [call.args[0] for call in logger.live_logger.write.call_args_list]
        self.assertEqual(
            [entry['data']['cf_packet_sequence'] for entry in entries],
            [0, 1],
        )
        unsubscribe()
        with patch('Interaction.log_manager.time.time', return_value=10.2):
            logger._cf_log_group_callback(
                70, {'stateEstimate.vy': 0.4}, SimpleNamespace(name='VEL_ORI')
            )
        self.assertEqual(len(packets), 2)

    def test_contact_source_timestamp_reconstructs_across_wrap(self):
        source, skew = reconstruct_contact_source_timestamp_ms(
            3, 0xFFFE
        )
        self.assertEqual(source, 0xFFFFFE)
        self.assertEqual(skew, 5)

        source, skew = reconstruct_contact_source_timestamp_ms(1000, 700)
        self.assertIsNone(source)
        self.assertEqual(skew, 300)

    def test_contact_listener_uses_producer_latched_source_timestamp(self):
        logger = self.make_logger()
        packets = []
        logger.add_cf_packet_listener(packets.append)
        data = {
            'contactGyro.x': 1.0,
            'contactGyro.y': 2.0,
            'contactGyro.z': 3.0,
            'contactGyro.epoch': 995,
        }

        with patch('Interaction.log_manager.time.time', return_value=10.0):
            logger._cf_log_group_callback(
                1000, data, SimpleNamespace(name='GYRO_1KHZ')
            )

        self.assertEqual(len(packets), 1)
        packet = packets[0]
        self.assertEqual(packet.cf_timestamp_ms, 995)
        self.assertEqual(packet.transport_cf_timestamp_ms, 1000)
        self.assertEqual(
            packet.source_cf_timestamp_basis,
            CONTACT_SOURCE_TIMESTAMP_BASIS,
        )
        self.assertTrue(packet.source_snapshot_atomic)
        saved = logger.live_logger.write.call_args.args[0]['data']
        self.assertEqual(saved['source_cf_timestamp_ms'], 995)
        self.assertEqual(saved['source_cf_transport_skew_ms'], 5)

    def test_packed_contact_imu_uses_its_single_producer_epoch(self):
        logger = self.make_logger()
        packets = []
        logger.add_cf_packet_listener(packets.append)
        data = {
            'contactImu.gx': 1.0,
            'contactImu.ax': 0.1,
            'contactImu.vx': 0.2,
            'contactImu.epoch': 995,
        }

        with patch('Interaction.log_manager.time.time', return_value=10.0):
            logger._cf_log_group_callback(
                1000, data, SimpleNamespace(name='GYRO_1KHZ')
            )

        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0].cf_timestamp_ms, 995)
        self.assertEqual(packets[0].transport_cf_timestamp_ms, 1000)
        self.assertTrue(packets[0].source_snapshot_atomic)
        self.assertEqual(
            packets[0].source_cf_timestamp_basis,
            CONTACT_SOURCE_TIMESTAMP_BASIS,
        )

    def test_contact_listener_marks_missing_source_epoch_unproven(self):
        logger = self.make_logger()
        packets = []
        logger.add_cf_packet_listener(packets.append)

        with patch('Interaction.log_manager.time.time', return_value=10.0):
            logger._cf_log_group_callback(
                1000,
                {'contactGyro.x': 1.0},
                SimpleNamespace(name='GYRO_1KHZ'),
            )

        packet = packets[0]
        self.assertEqual(packet.cf_timestamp_ms, 1000)
        self.assertIsNone(packet.source_cf_timestamp_basis)
        self.assertFalse(packet.source_snapshot_atomic)
        saved = logger.live_logger.write.call_args.args[0]['data']
        self.assertEqual(
            saved['source_cf_timestamp_error'],
            'missing_invalid_or_ambiguous_firmware_source_epoch',
        )

    def test_listener_failure_cannot_break_logging_or_other_listeners(self):
        logger = self.make_logger()
        received = []
        logger.add_cf_packet_listener(Mock(side_effect=RuntimeError('boom')))
        logger.add_cf_packet_listener(received.append)
        with self.assertLogs('Interaction.log_manager', level='ERROR'):
            with patch('Interaction.log_manager.time.time', return_value=10.0):
                logger._cf_log_group_callback(
                    50, {'stateEstimate.vy': 0.2},
                    SimpleNamespace(name='VEL_ORI'),
                )
        self.assertEqual(len(received), 1)
        self.assertEqual(logger.get_latest_group_log_data('VEL_ORI'),
                         {'stateEstimate.vy': 0.2})

    def test_mocap_listener_receives_post_forward_frame_without_mutating_input(self):
        logger = self.make_logger()
        frames = []
        logger.add_mocap_frame_listener(frames.append)
        original = {
            'frame_id': 7,
            'time': 9.9,
            'tvec': [1.0, 2.0, 3.0],
            'quat': [0.0, 0.0, 0.0, 1.0],
            'orientation_forwarded_to_onboard_ekf': False,
        }
        with patch('Interaction.log_manager.time.time', return_value=10.0):
            logger.add_log_entry('frames', original)
        self.assertEqual(len(frames), 1)
        self.assertIsInstance(frames[0], MocapFramePacket)
        self.assertEqual(frames[0].sequence, 0)
        self.assertEqual(frames[0].host_receive_time_s, 10.0)
        self.assertEqual(frames[0].data['tvec'], (1.0, 2.0, 3.0))
        self.assertNotIn('mocap_frame_sequence', original)
        saved = logger.live_logger.write.call_args.args[0]['data']
        self.assertEqual(saved['mocap_frame_sequence'], 0)
        self.assertEqual(saved['host_receive_time_s'], 10.0)
        with self.assertRaises(TypeError):
            frames[0].data['tvec'][0] = 99.0
        self.assertEqual(saved['tvec'][0], 1.0)

    def test_mocap_listener_ignores_non_primary_groups_and_unsubscribes(self):
        logger = self.make_logger()
        frames = []
        unsubscribe = logger.add_mocap_frame_listener(frames.append)
        logger.add_log_entry('anchor_frames', {'tvec': [0, 0, 0]})
        unsubscribe()
        logger.add_log_entry('frames', {'tvec': [0, 0, 1]})
        self.assertEqual(frames, [])


if __name__ == '__main__':
    unittest.main()
