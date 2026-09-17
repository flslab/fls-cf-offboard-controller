import unittest
from unittest.mock import patch

from Interaction.calibrate_arduino_serial_clock import (
    collect_receive_only,
    parse_sync_reply,
)
from Interaction.potentiometer_force_sensor import parse_potentiometer_line


class SerialClockReplyTests(unittest.TestCase):
    def test_matching_reply_extracts_capture_stamp(self):
        self.assertEqual(parse_sync_reply(b"#SYNC,7,4294967295\r\n", 7),
                         4294967295)
        self.assertIsNone(parse_potentiometer_line(b"#SYNC,7,123456\n"))

    def test_force_samples_headers_stale_and_malformed_replies_are_ignored(self):
        for line in (
            b"time_ms,raw,filtered,voltage,compression_mm,supply_voltage\n",
            b"100,900,900,4.0,2.0,5.0\n",
            b"#SYNC,6,123\n",
            b"#SYNC,7,-1\n",
            b"#SYNC,7,4294967296\n",
            b"#SYNC,7,not-a-time\n",
            b"#SYNC,7,123,extra\n",
        ):
            with self.subTest(line=line):
                self.assertIsNone(parse_sync_reply(line, 7))


class ReceiveOnlyTests(unittest.TestCase):
    def test_contiguous_csv_is_diagnostic_without_clock_authority(self):
        class FakeSerial:
            def __init__(self, *args, **kwargs):
                self.lines = iter([
                    b"100,500,500.0,2.0,10.0,5.0\n",
                    b"120,500,500.0,2.0,10.0,5.0\n",
                    b"140,500,500.0,2.0,10.0,5.0\n",
                ])

            def __enter__(self):
                return self

            def __exit__(self, *args):
                pass

            def reset_input_buffer(self):
                pass

            def readline(self):
                return next(self.lines)

        with patch("Interaction.calibrate_arduino_serial_clock.serial.Serial",
                   FakeSerial):
            result = collect_receive_only("/dev/serial0", samples=3)
        self.assertEqual(result["arduino_step_ms_median"], 20)
        self.assertFalse(result["clock_offset_calibrated"])
        self.assertFalse(result["command_authority"])
