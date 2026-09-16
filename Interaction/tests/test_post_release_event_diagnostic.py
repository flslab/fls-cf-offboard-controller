import struct
import unittest
from unittest.mock import patch

from Interaction.post_release_event_diagnostic import (
    ACK_STRUCT,
    EVENT_TYPE,
    EVENT_VERSION,
    encode_release_event,
    parse_release_event_ack,
    send_release_event_diagnostic,
)
from Interaction.probe_post_release_event import collect_manual_release


class FakePacket:
    port = 6
    channel = 1

    def __init__(self, data):
        self.data = data


class PostReleaseEventDiagnosticTests(unittest.TestCase):
    def test_send_freezes_first_unloaded_receive_time_without_authority(self):
        class FakeCf:
            def __init__(self):
                self.packets = []

            def send_packet(self, packet):
                self.packets.append(packet)

        cf = FakeCf()
        result = send_release_event_diagnostic(
            cf, session_id=99, sequence=7, arduino_sample_ms=12345,
            pi_receive_monotonic_s=100.0,
            monotonic_ns=lambda: 100_080_000_000,
        )
        self.assertEqual(len(cf.packets), 1)
        self.assertEqual(bytes(cf.packets[0].data),
                         struct.pack("<BBHIII", 14, 1, 7, 99, 12345, 80000))
        self.assertEqual(result["pi_receive_to_send_elapsed_us"], 80000)
        self.assertFalse(result["command_authority"])

    def test_rejects_stale_future_and_invalid_sample_identity(self):
        for sequence, sample, receive, send in (
                (1, 5, 1.0, 2_000_001_000),
                (1, 5, 2.0, 1_000_000_000),
                (1, -1, 1.0, 1_001_000_000),
                (65536, 5, 1.0, 1_001_000_000)):
            with self.subTest(sequence=sequence, sample=sample):
                with self.assertRaises(ValueError):
                    encode_release_event(
                        99, sequence, sample, receive,
                        send_monotonic_ns=send
                    )

    def test_ack_is_identity_checked_and_only_a_delayed_proxy(self):
        packet = FakePacket(ACK_STRUCT.pack(
            EVENT_TYPE, EVENT_VERSION, 9, 99, 42, 50000, 1000000
        ))
        result = parse_release_event_ack(
            packet, session_id=99, sequence=9, arduino_sample_ms=42
        )
        self.assertEqual(result["delayed_release_proxy_us_mod32"], 950000)
        self.assertFalse(result["release_epoch_calibrated"])
        self.assertIsNone(parse_release_event_ack(
            packet, session_id=99, sequence=10, arduino_sample_ms=42
        ))
        self.assertIsNone(parse_release_event_ack(
            packet, session_id=99, sequence=9, arduino_sample_ms=43
        ))
        self.assertIsNone(parse_release_event_ack(
            packet, session_id=100, sequence=9, arduino_sample_ms=42
        ))

    def test_manual_prop_off_probe_freezes_first_unloaded_sample(self):
        forces = [0.12] * 3 + [0.25] * 3 + [0.28, 0.23] + [0.15] * 5
        lines = iter(
            f"{index * 20},50,50,0.24,{force / 0.16:.3f},5.0\n".encode()
            for index, force in enumerate(forces)
        )

        class FakeConnection:
            def readline(self):
                return next(lines)

        clock = [0.0]

        def monotonic():
            clock[0] += 0.02
            return clock[0]

        with patch("Interaction.probe_post_release_event.time.monotonic",
                   side_effect=monotonic):
            decision = collect_manual_release(
                FakeConnection(), timeout_s=10.0,
                contact_force_n=0.18, unloaded_force_n=0.17,
                force_drop_n=0.04, decrease_rate_n_s=0.05,
                unloaded_dwell_s=0.05,
            )
        self.assertTrue(decision.released)
        self.assertEqual(decision.unloaded_started_sample_id, 160)
        self.assertGreater(decision.release_confirmed_sample_id, 160)
