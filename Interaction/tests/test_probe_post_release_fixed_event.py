import struct
import unittest

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

from Interaction.post_release_fixed_event_diagnostic import (
    calibrated_fixed_delay_us,
    encode_fixed_release_event,
    parse_fixed_release_ack,
    send_fixed_release_event_diagnostic,
)


class FixedReleaseEventTests(unittest.TestCase):
    def test_v2_request_has_identity_only(self):
        payload = encode_fixed_release_event(123, 7, 456)
        self.assertEqual(len(payload), 12)
        self.assertEqual(struct.unpack("<BBHII", payload),
                         (14, 2, 7, 123, 456))

    def test_runtime_sender_uses_no_per_event_time(self):
        class FakeCf:
            def __init__(self):
                self.packets = []

            def send_packet(self, packet):
                self.packets.append(packet)

        cf = FakeCf()
        result = send_fixed_release_event_diagnostic(
            cf, session_id=123, sequence=7, arduino_sample_ms=456,
            calibration={"fixed_delay_firmware_us": 60188,
                         "calibration_sha256": "abc"},
        )
        self.assertEqual(bytes(cf.packets[0].data),
                         encode_fixed_release_event(123, 7, 456))
        self.assertFalse(result["per_event_elapsed_sent"])
        self.assertFalse(result["pi_timestamp_sent"])
        self.assertFalse(result["command_authority"])

    def test_matching_ack_echoes_compiled_delay_without_authority(self):
        packet = CRTPPacket()
        packet.set_header(CRTPPort.LOCALIZATION, 1)
        packet.data = struct.pack("<BBHIIII", 14, 2, 7, 123, 456, 60188, 100000)
        result = parse_fixed_release_ack(
            packet, session_id=123, sequence=7,
            arduino_sample_ms=456, fixed_delay_us=60188,
        )
        self.assertEqual(result["firmware_release_receive_proxy_us_mod32"], 39812)
        self.assertFalse(result["command_authority"])
        self.assertIsNone(parse_fixed_release_ack(
            packet, session_id=123, sequence=7,
            arduino_sample_ms=456, fixed_delay_us=60187,
        ))

    def test_calibration_holdouts_required(self):
        batch = dict(schema="bolt_clock_rate_prop_off_batch_v1",
                     clock_fit=dict(holdout_compatible=True),
                     release_fit=dict(holdout_covered=True),
                     firmware_delay_shadow=dict(
                         fixed_delay_firmware_ms=60.187894,
                         command_authority=False))
        self.assertEqual(calibrated_fixed_delay_us(batch), 60188)
        batch["clock_fit"]["holdout_compatible"] = False
        with self.assertRaisesRegex(ValueError, "not valid"):
            calibrated_fixed_delay_us(batch)


if __name__ == "__main__":
    unittest.main()
