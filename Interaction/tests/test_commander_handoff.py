"""HLC ownership must not be relaxed until the firmware accepted a plan."""
import struct
from types import SimpleNamespace
import unittest
from unittest import mock

from cflib.crazyflie.high_level_commander import HighLevelCommander
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

from Interaction.command_wrapper import CommandWrapper
from Interaction.commander_handoff import HandoffError, handoff_to_high_level
from Interaction.live_logger import LiveLoggerError


class FakeCF:
    def __init__(self, version=8):
        self.platform = SimpleNamespace(get_protocol_version=lambda: version)
        self.callbacks = []
        self.events = []
        self.on_send = self.accept
        self.incoming_thread = False

    def is_called_by_incoming_handler_thread(self):
        return self.incoming_thread

    def add_port_callback(self, port, callback):
        self.events.append(("register", port))
        self.callbacks.append(callback)

    def remove_port_callback(self, port, callback):
        self.events.append(("remove", port))
        if callback in self.callbacks:
            self.callbacks.remove(callback)

    def send_packet(self, packet):
        self.events.append(("high_send", bytes(packet.data)))
        self.on_send(packet)

    def emit(self, payload, *, port=CRTPPort.SETPOINT_HL, channel=0):
        packet = CRTPPacket()
        packet.port, packet.channel, packet.data = port, channel, payload
        self.events.append(("ack", bytes(payload)))
        for callback in list(self.callbacks):
            callback(packet)

    def accept(self, packet):
        self.emit(bytes(packet.data[:3])+b"\x00")


class LowCommander:
    def __init__(self, cf):
        self._cf = cf

    def send_notify_setpoint_stop(self):
        self._cf.events.append(("notify",))


class CommanderHandoffTests(unittest.TestCase):
    def pair(self, version=8):
        cf = FakeCF(version)
        return cf, LowCommander(cf), HighLevelCommander(cf)

    def run_go(self, low, high, **kwargs):
        return handoff_to_high_level(low, high, "go_to", .1, .2, 1., 0., .5,
                                     relative=False, **kwargs)

    def test_modern_success_ack_before_priority_release_and_cleanup(self):
        cf, low, high = self.pair()
        result = self.run_go(low, high)
        self.assertEqual([event[0] for event in cf.events],
                         ["register", "high_send", "ack", "remove", "notify"])
        self.assertEqual(cf.events[1][1][:3], bytes((12, 0, 0)))
        self.assertEqual(cf.callbacks, [])
        self.assertTrue(result["acknowledged"])
        self.assertTrue(result["priority_released"])

    def test_legacy_go_to_matches_command_four_not_stop_three(self):
        cf, low, high = self.pair(version=7)
        self.run_go(low, high)
        self.assertEqual(cf.events[1][1][:3], bytes((4, 0, 0)))

    def test_landing_ack_prefix_includes_height_float_byte(self):
        cf, low, high = self.pair()
        result = handoff_to_high_level(low, high, "land", .1, 2., yaw=None)
        self.assertEqual(cf.events[1][1][:3], struct.pack("<BBf", 8, 0, .1)[:3])
        self.assertEqual(result["command"], "land")

    def test_stop_is_never_a_handoff_plan(self):
        cf, low, high = self.pair()
        with self.assertRaisesRegex(HandoffError, "never stop"):
            handoff_to_high_level(low, high, "stop")
        self.assertEqual(cf.events, [])

    def test_unrelated_or_malformed_replies_are_ignored(self):
        cf, low, high = self.pair()
        def send(packet):
            prefix = bytes(packet.data[:3])
            cf.emit(bytes((3, 0, 0, 0)))  # STOP is not GO_TO.
            cf.emit(prefix+b"\x00", channel=1)
            cf.emit(prefix+b"\x00", port=CRTPPort.COMMANDER_GENERIC)
            cf.emit(prefix[:2]+bytes((1, 0)))  # Wrong relative flag.
            cf.emit(prefix)
            self.assertNotIn(("notify",), cf.events)
            cf.accept(packet)
        cf.on_send = send
        self.run_go(low, high)
        self.assertEqual(cf.events[-1], ("notify",))
        self.assertEqual(cf.callbacks, [])

    def test_matching_rejection_does_not_release_priority(self):
        cf, low, high = self.pair()
        cf.on_send = lambda packet: cf.emit(bytes(packet.data[:3])+bytes((16,)))
        with self.assertRaisesRegex(HandoffError, "result 16"):
            self.run_go(low, high)
        self.assertNotIn(("notify",), cf.events)
        self.assertEqual(cf.callbacks, [])

    def test_timeout_does_not_release_or_retry_and_late_ack_is_detached(self):
        cf, low, high = self.pair()
        cf.on_send = lambda packet: None
        with self.assertRaisesRegex(HandoffError, "timed out"):
            self.run_go(low, high, ack_timeout_s=.001)
        self.assertEqual(sum(event[0] == "high_send" for event in cf.events), 1)
        self.assertNotIn(("notify",), cf.events)
        self.assertEqual(cf.callbacks, [])
        cf.emit(bytes((12, 0, 0, 0)))
        self.assertNotIn(("notify",), cf.events)

    def test_timeout_latches_same_connection_before_a_late_reply_can_match_retry(self):
        cf, low, high = self.pair()
        cf.on_send = lambda packet: None
        with self.assertRaisesRegex(HandoffError, "timed out"):
            self.run_go(low, high, ack_timeout_s=.001)
        events_after_failure = list(cf.events)
        # If a second command were allowed, this would produce the exact ACK
        # prefix an old delayed reply could falsely satisfy.
        cf.on_send = cf.accept
        with self.assertRaisesRegex(HandoffError, "untrusted"):
            self.run_go(low, high)
        self.assertEqual(cf.events, events_after_failure)
        cf.emit(bytes((12, 0, 0, 0)))
        self.assertNotIn(("notify",), cf.events)
        self.assertEqual(cf.callbacks, [])
        # A different command and rewrapping the same CF do not reset trust.
        with self.assertRaisesRegex(HandoffError, "untrusted"):
            handoff_to_high_level(low, HighLevelCommander(cf), "land", .1, 2.)
        replacement_cf, replacement_low, replacement_high = self.pair()
        self.run_go(replacement_low, replacement_high)
        self.assertEqual(replacement_cf.events[-1], ("notify",))

    def test_rejection_and_notify_failure_also_latch_connection(self):
        for mode in ("rejected", "notify_failed"):
            with self.subTest(mode=mode):
                cf, low, high = self.pair()
                if mode == "rejected":
                    cf.on_send = lambda packet: cf.emit(bytes(packet.data[:3])+bytes((16,)))
                else:
                    low.send_notify_setpoint_stop = mock.Mock(side_effect=OSError("notify failed"))
                with self.assertRaises(HandoffError):
                    self.run_go(low, high)
                event_count = len(cf.events)
                with self.assertRaisesRegex(HandoffError, "untrusted"):
                    self.run_go(low, high)
                self.assertEqual(len(cf.events), event_count)
                self.assertEqual(cf.callbacks, [])

    def test_pre_send_reply_during_registration_is_not_accepted(self):
        cf, low, high = self.pair()
        original = cf.add_port_callback
        def register(port, callback):
            original(port, callback)
            cf.emit(bytes((12, 0, 0, 0)))
        cf.add_port_callback = register
        cf.on_send = lambda packet: None
        with self.assertRaisesRegex(HandoffError, "timed out"):
            self.run_go(low, high, ack_timeout_s=.001)
        self.assertNotIn(("notify",), cf.events)
        self.assertEqual(cf.callbacks, [])

    def test_send_error_cleans_callback_without_notifying(self):
        cf, low, high = self.pair()
        def failed(_packet):
            raise OSError("radio disconnected")
        cf.on_send = failed
        with self.assertRaises(HandoffError) as context:
            self.run_go(low, high)
        self.assertIsInstance(context.exception.__cause__, OSError)
        self.assertEqual(cf.callbacks, [])
        self.assertNotIn(("notify",), cf.events)

    def test_logging_failure_before_send_does_not_release(self):
        cf, low, high = self.pair()
        high = CommandWrapper(high, mock.Mock(side_effect=LiveLoggerError("disk failed")))
        with self.assertRaises(HandoffError) as context:
            self.run_go(low, high)
        self.assertIsInstance(context.exception.__cause__, LiveLoggerError)
        self.assertFalse(any(event[0] in ("high_send", "notify") for event in cf.events))
        self.assertEqual(cf.callbacks, [])

    def test_explicit_safety_wrappers_preserve_send_with_failed_log(self):
        cf, low, high = self.pair()
        log = mock.Mock(side_effect=LiveLoggerError("disk failed"))
        low = CommandWrapper(low, log).for_safety_cleanup()
        high = CommandWrapper(high, log).for_safety_cleanup()
        with self.assertLogs("Interaction.command_wrapper", level="ERROR"):
            result = self.run_go(low, high)
        self.assertTrue(result["priority_released"])
        self.assertEqual(cf.events[-1], ("notify",))

    def test_notify_failure_is_unconfirmed_but_callback_is_removed(self):
        cf, low, high = self.pair()
        low.send_notify_setpoint_stop = mock.Mock(side_effect=OSError("notify failed"))
        with self.assertRaisesRegex(HandoffError, "unconfirmed"):
            self.run_go(low, high)
        self.assertEqual(cf.callbacks, [])
        self.assertEqual(low.send_notify_setpoint_stop.call_count, 1)

    def test_missing_callback_interface_and_other_drone_are_rejected(self):
        cf, low, high = self.pair()
        cf.add_port_callback = None
        with self.assertRaisesRegex(HandoffError, "callback interface"):
            self.run_go(low, high)
        other, other_low, _ = self.pair()
        with self.assertRaisesRegex(HandoffError, "same"):
            self.run_go(other_low, high)
        self.assertEqual(cf.events, [])
        self.assertEqual(other.events, [])

    def test_callback_thread_and_concurrent_same_cf_handoff_rejected(self):
        cf, low, high = self.pair()
        cf.incoming_thread = True
        with self.assertRaisesRegex(HandoffError, "incoming-handler"):
            self.run_go(low, high)
        cf.incoming_thread = False
        def send(packet):
            with self.assertRaisesRegex(HandoffError, "already active"):
                self.run_go(low, high)
            cf.accept(packet)
        cf.on_send = send
        self.run_go(low, high)
        self.assertEqual(sum(event[0] == "high_send" for event in cf.events), 1)

    def test_dry_run_is_explicit_log_only_and_cannot_bypass_real_execution(self):
        cf, low, high = self.pair()
        with self.assertRaisesRegex(HandoffError, "execution-disabled"):
            self.run_go(low, high, dry_run=True)
        logs = []
        record = lambda **kwargs: logs.append(kwargs)
        low = CommandWrapper(low, record, execute=False)
        high = CommandWrapper(high, record, execute=False)
        with self.assertRaisesRegex(HandoffError, "explicit dry_run"):
            self.run_go(low, high)
        result = self.run_go(low, high, dry_run=True)
        self.assertEqual(len(logs), 2)
        self.assertEqual(cf.events, [])
        self.assertEqual(result["status"], "dry_run")
        self.assertFalse(result["acknowledged"])
        self.assertFalse(result["priority_released"])

    def test_invalid_duration_group_or_timeout_cannot_send(self):
        cf, low, high = self.pair()
        for duration in (0., -1., float("nan"), float("inf")):
            with self.subTest(duration=duration), self.assertRaises(HandoffError):
                handoff_to_high_level(low, high, "land", .1, duration)
        with self.assertRaisesRegex(HandoffError, "ALL_GROUPS"):
            self.run_go(low, high, group_mask=1)
        for timeout in (0, -.1, float("nan"), .151, True):
            with self.subTest(timeout=timeout), self.assertRaises(HandoffError):
                self.run_go(low, high, ack_timeout_s=timeout)
        self.assertEqual(cf.events, [])

    def test_partial_registration_failure_attempts_callback_cleanup(self):
        cf, low, high = self.pair()
        original = cf.add_port_callback
        def failed(port, callback):
            original(port, callback)
            raise RuntimeError("registration failed")
        cf.add_port_callback = failed
        with self.assertRaises(HandoffError):
            self.run_go(low, high)
        self.assertEqual(cf.callbacks, [])
        self.assertNotIn(("notify",), cf.events)


if __name__ == "__main__":
    unittest.main()
