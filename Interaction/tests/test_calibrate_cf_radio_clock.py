import struct
import unittest

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

from Interaction.calibrate_cf_radio_clock import (
    causal_offset_bounds_us,
    parse_probe_reply,
)


def packet(sequence, receive_us=1000, enqueue_us=1100):
    result = CRTPPacket()
    result.set_header(CRTPPort.LOCALIZATION, 1)
    result.data = struct.pack('<BHII', 13, sequence, receive_us, enqueue_us)
    return result


class RadioClockProbeTests(unittest.TestCase):
    def test_exact_sequence_and_processing_order_required(self):
        self.assertEqual(parse_probe_reply(packet(7), 7), (1000, 1100, 100))
        self.assertIsNone(parse_probe_reply(packet(7), 8))
        self.assertIsNone(parse_probe_reply(packet(7, 1100, 1000), 7))
        wrong = packet(7)
        wrong.channel = 0
        self.assertIsNone(parse_probe_reply(wrong, 7))

    def test_offset_bounds_use_causality_not_symmetric_delay(self):
        self.assertEqual(
            causal_offset_bounds_us(1000000, 1600000, 1200, 1300),
            (-300.0, 200.0),
        )
        with self.assertRaisesRegex(ValueError, 'causality'):
            causal_offset_bounds_us(1000000, 1050000, 1200, 1300)
