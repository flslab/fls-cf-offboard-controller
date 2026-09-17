import struct
import unittest

from Interaction.post_release_firmware_control_event import (
    FirmwareHoldNotification,
    encode_pi_release_command,
    handoff_pi_release_to_firmware,
    parse_pi_release_ack,
    parse_post_release_hold_notice,
    send_pi_release_command_once,
    send_vicon_position_mirror,
)


class PostReleaseFirmwareControlEventTests(unittest.TestCase):
    def test_vicon_mirror_carries_pi_receive_epoch_without_cross_clock_math(self):
        class FakeCf:
            def __init__(self):
                self.packets = []

            def send_packet(self, packet):
                self.packets.append(packet)

        cf = FakeCf()
        first = send_vicon_position_mirror(
            cf, (1.0, -2.0, 0.8),
            pi_receive_monotonic_s=10.0,
            monotonic_s=lambda: 10.002,
        )
        second = send_vicon_position_mirror(
            cf, (1.01, -1.99, 0.8),
            pi_receive_monotonic_s=10.01,
            monotonic_s=lambda: 10.012,
        )
        self.assertEqual((first['pi_receive_us_mod32'],
                          second['pi_receive_us_mod32']),
                         (10_000_000, 10_010_000))
        self.assertEqual([(packet.port, packet.channel)
                          for packet in cf.packets], [(6, 1), (6, 1)])
        unpacked = struct.unpack('<BIIfff', bytes(cf.packets[-1].data))
        self.assertEqual(unpacked[:3], (16, 10_010_000, 2000))
        self.assertAlmostEqual(unpacked[4], -1.99, places=5)

    def test_stale_vicon_frame_is_rejected_before_send(self):
        class FakeCf:
            def send_packet(self, packet):
                raise AssertionError('bad frame must not be sent')

        with self.assertRaisesRegex(ValueError, 'stale'):
            send_vicon_position_mirror(
                FakeCf(), (0.0, 0.0, 1.0),
                pi_receive_monotonic_s=11.0,
                monotonic_s=lambda: 11.020,
            )

    def test_acknowledged_release_is_already_claimed_by_firmware(self):
        class FakeCf:
            def __init__(self):
                self.callback = None
                self.calls = []

            def add_port_callback(self, port, callback):
                self.callback = callback
                self.calls.append('register')

            def remove_port_callback(self, port, callback):
                self.assert_same(callback)
                self.callback = None
                self.calls.append('remove')

            def assert_same(self, callback):
                assert self.callback is callback

            def send_packet(self, packet):
                self.calls.append('send')
                ack = type('Ack', (), {
                    'port': 8, 'channel': 0,
                    'data': bytes(packet.data) + b'\x00',
                })()
                self.callback(ack)

        cf = FakeCf()
        ticks = iter((100_080_000_000, 100_080_500_000))
        result = handoff_pi_release_to_firmware(
            cf, session_id=99, sequence=7,
            arduino_sample_ms=12345,
            pi_receive_monotonic_ns=100_000_000_000,
            firmware_auto_brake_armed=True,
            monotonic_ns=lambda: next(ticks),
        )
        self.assertEqual(cf.calls, ['register', 'send', 'remove'])
        self.assertTrue(result['low_level_priority_released'])
        self.assertTrue(result['firmware_priority_claimed'])

    def test_rejected_release_does_not_relax_low_level_priority(self):
        class FakeCf:
            def add_port_callback(self, port, callback):
                self.callback = callback

            def remove_port_callback(self, port, callback):
                self.callback = None

            def send_packet(self, packet):
                self.callback(type('Ack', (), {
                    'port': 8, 'channel': 0,
                    'data': bytes(packet.data) + b'\x16',
                })())

        cf = FakeCf()
        ticks = iter((100_080_000_000, 100_080_500_000))
        with self.assertRaisesRegex(RuntimeError, 'rejected'):
            handoff_pi_release_to_firmware(
                cf, session_id=99, sequence=7,
                arduino_sample_ms=12345,
                pi_receive_monotonic_ns=100_000_000_000,
                firmware_auto_brake_armed=True,
                monotonic_ns=lambda: next(ticks),
            )

    def test_hold_notice_matches_release_and_is_acknowledged_without_go_to(self):
        class FakeCf:
            def __init__(self):
                self.callback = None
                self.sent = []

            def add_port_callback(self, port, callback):
                self.callback = callback

            def remove_port_callback(self, port, callback):
                self.assert_callback(callback)
                self.callback = None

            def assert_callback(self, callback):
                assert self.callback is callback

            def send_packet(self, packet):
                self.sent.append(packet)

        def notice(session, sequence, x=1.25):
            return type('Notice', (), {
                'port': 8, 'channel': 1,
                'data': struct.pack('<BBHIffffI', 17, 1, sequence,
                                    session, x, -0.3, 0.8, 0.2, 123456),
            })()

        cf = FakeCf()
        with FirmwareHoldNotification(cf, session_id=99, sequence=7) as waiter:
            cf.callback(notice(98, 7))
            cf.callback(notice(99, 6))
            self.assertIsNone(waiter.wait(0))
            cf.callback(notice(99, 7))
            for actual, expected in zip(
                    waiter.wait(0)['hold_position_m'], (1.25, -0.3, 0.8)):
                self.assertAlmostEqual(actual, expected)
            waiter.acknowledge()
        self.assertEqual(len(cf.sent), 1)
        self.assertEqual((cf.sent[0].port, cf.sent[0].channel), (8, 0))
        self.assertEqual(bytes(cf.sent[0].data),
                         struct.pack('<BBHI', 18, 1, 7, 99))
        self.assertIsNone(parse_post_release_hold_notice(
            notice(99, 7, float('nan')), session_id=99, sequence=7))

    def test_listener_is_armed_before_release_ack_and_measured_hold(self):
        class FakeCf:
            def __init__(self):
                self.callbacks = []
                self.sent_types = []

            def add_port_callback(self, port, callback):
                self.callbacks.append(callback)

            def remove_port_callback(self, port, callback):
                self.callbacks.remove(callback)

            def deliver(self, packet):
                for callback in tuple(self.callbacks):
                    callback(packet)

            def send_packet(self, packet):
                self.sent_types.append(bytes(packet.data)[0])
                if self.sent_types[-1] == 15:
                    self.deliver(type('Ack', (), {
                        'port': 8, 'channel': 0,
                        'data': bytes(packet.data) + b'\x00',
                    })())

        cf = FakeCf()
        with FirmwareHoldNotification(cf, session_id=99, sequence=7) as waiter:
            ticks = iter((100_080_000_000, 100_080_500_000))
            handoff_pi_release_to_firmware(
                cf, session_id=99, sequence=7, arduino_sample_ms=12345,
                pi_receive_monotonic_ns=100_000_000_000,
                firmware_auto_brake_armed=True,
                monotonic_ns=lambda: next(ticks),
            )
            self.assertEqual(len(cf.callbacks), 1)
            cf.deliver(type('Notice', (), {
                'port': 8, 'channel': 1,
                'data': struct.pack('<BBHIffffI', 17, 1, 7, 99,
                                    1.0, 2.0, 0.8, 0.1, 123456),
            })())
            self.assertIsNotNone(waiter.wait(0))
            waiter.acknowledge()
        self.assertEqual(cf.sent_types, [15, 18])
        self.assertEqual(cf.callbacks, [])

    def test_pi_event_is_one_shot_hlc_packet_and_not_a_clock_mapping(self):
        class FakeCf:
            def __init__(self):
                self.packets = []

            def send_packet(self, packet):
                self.packets.append(packet)

        cf = FakeCf()
        ticks = iter((100_080_000_000, 100_080_100_000))
        result = send_pi_release_command_once(
            cf, session_id=99, sequence=7, arduino_sample_ms=12345,
            pi_receive_monotonic_ns=100_000_000_000,
            firmware_auto_brake_armed=True,
            monotonic_ns=lambda: next(ticks),
        )
        self.assertEqual(len(cf.packets), 1)
        self.assertEqual(cf.packets[0].port, 8)
        self.assertEqual(bytes(cf.packets[0].data),
                         struct.pack('<BBHIII', 15, 1, 7, 99, 12345, 80000))
        self.assertEqual(result['pi_receive_to_send_elapsed_us'], 80000)
        self.assertEqual(result['pi_send_return_monotonic_ns'], 100_080_100_000)
        self.assertFalse(result['radio_delivery_confirmed'])
        self.assertFalse(result['firmware_command_started'])
        class AckPacket:
            port = 8
            channel = 0
            data = bytes(cf.packets[0].data) + b'\x00'

        ack = parse_pi_release_ack(
            AckPacket(), request_payload_hex=result['request_payload_hex'],
            ack_receive_monotonic_ns=100_082_000_000,
        )
        self.assertTrue(ack['event_queued_by_firmware'])
        self.assertFalse(ack['firmware_command_started'])
        self.assertIsNone(parse_pi_release_ack(
            AckPacket(), request_payload_hex='00' * 16,
            ack_receive_monotonic_ns=100_082_000_000,
        ))

    def test_requires_explicit_arm_and_fresh_valid_identity(self):
        with self.assertRaises(ValueError):
            send_pi_release_command_once(
                object(), session_id=1, sequence=0, arduino_sample_ms=0,
                pi_receive_monotonic_ns=1000,
                monotonic_ns=lambda: 1100,
            )
        for session, seq, sample, received, sent in (
            (-1, 0, 1, 1000, 1100),
            (1, 65536, 1, 1000, 1100),
            (1, 0, -1, 1000, 1100),
            (1, 0, 1, 2000, 1100),
            (1, 0, 1, 1000, 250_001_001),
        ):
            with self.subTest(session=session, seq=seq, sample=sample):
                with self.assertRaises(ValueError):
                    encode_pi_release_command(
                        session_id=session, sequence=seq,
                        arduino_sample_ms=sample,
                        pi_receive_monotonic_ns=received,
                        send_monotonic_ns=sent,
                    )
