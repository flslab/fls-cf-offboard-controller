"""Packet clock metadata is additive logging, never a control-time change."""

import collections
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock, patch

from Interaction.log_manager import InteractionLogger


class LogPacketTimestampTests(unittest.TestCase):
    def make_logger(self, *, enabled=True):
        logger = InteractionLogger.__new__(InteractionLogger)
        logger.cf_log_group_times = {}
        logger.cf_log_group_packets = collections.defaultdict(
            lambda: collections.deque(maxlen=1000)
        )
        logger.cf_log_packet_lock = threading.Lock()
        logger.cf_log_data = {'VEL_ORI': {'stateEstimate.vy': {'data': []}}}
        logger.live_logger = Mock() if enabled else None
        return logger

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


if __name__ == '__main__':
    unittest.main()
