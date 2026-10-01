"""A received FC hold target is evidence, not a successful takeover gate."""
import unittest
from unittest.mock import Mock, patch

from Interaction.interactions import InteractionsControl, StaleLocalizationError


class FirmwareHoldTargetLoggingTests(unittest.TestCase):
    def run_monitor(self, *, ready_at=None, notice_at=0.0):
        clock = {'now': 1000.0, 'step': 0}
        control = InteractionsControl.__new__(InteractionsControl)
        control.cf = Mock()
        control._flush_release_diagnostics = Mock()
        control._check_firmware_brake_monitor_safety = Mock()
        control._log_event = Mock()
        control._firmware_brake_status_snapshot = lambda: ({
            'hlCommander.pRelAutoSt': 4,
            'hlCommander.pRelReady': int(
                ready_at is not None and clock['now'] >= 1000 + ready_at),
            'hlCommander.pRelAutoTime': 0,
        }, clock['now'])

        def sleep(seconds):
            self.assertEqual(seconds, .05)
            clock['step'] += 1
            clock['now'] = 1000.0 + clock['step'] * .05
        control._safe_sleep = sleep
        notice = dict(session_id=12, sequence=3,
                      hold_position_m=[.01153, -.41515, .9985],
                      hold_yaw_rad=.046, firmware_hold_us_mod32=28623775)
        completion = Mock()
        completion.wait.side_effect = lambda _: (
            notice if notice_at is not None and clock['now'] >= 1000 + notice_at
            else None)
        fault = None
        result = None
        with patch('Interaction.interactions.time.monotonic',
                   side_effect=lambda: clock['now']), \
             patch('Interaction.interactions.time.time',
                   side_effect=lambda: clock['now']):
            try:
                result = control._wait_for_firmware_brake_hold_impl(
                    completion, brake_mode='scurve', baseline_receipt_s=999,
                    baseline_timeouts=0)
            except StaleLocalizationError as exc:
                fault = exc
        events = [(c.args[0], c.args[1]) for c in control._log_event.call_args_list]
        return result, fault, completion, events, notice

    def test_target_logged_once_before_later_confirmation(self):
        result, fault, completion, events, notice = self.run_monitor(ready_at=.2)
        self.assertIsNone(fault)
        self.assertEqual(result, notice)
        completion.acknowledge.assert_called_once_with()
        targets = [e for name, e in events if name.endswith('Target Received')]
        self.assertEqual(len(targets), 1)
        self.assertEqual(targets[0]['hold_position_m'], notice['hold_position_m'])
        self.assertFalse(targets[0]['hold_confirmed'])
        self.assertEqual(targets[0]['firmware_ready'], 0)
        names = [name for name, _ in events]
        self.assertLess(names.index('Firmware Post-Release Hold Target Received'),
                        names.index('Firmware Post-Release Hold Acquired'))

    def test_target_survives_fault_on_same_monitor_iteration(self):
        _, fault, completion, events, notice = self.run_monitor(notice_at=.35)
        self.assertIsInstance(fault, StaleLocalizationError)
        completion.acknowledge.assert_not_called()
        targets = [e for name, e in events if name.endswith('Target Received')]
        self.assertEqual(len(targets), 1)
        self.assertEqual(targets[0]['hold_position_m'], notice['hold_position_m'])
        self.assertFalse(targets[0]['hold_confirmed'])
        self.assertFalse(any(name.endswith('Hold Acquired') for name, _ in events))

    def test_missing_notice_does_not_invent_target(self):
        _, fault, completion, events, _ = self.run_monitor(notice_at=None)
        self.assertIsInstance(fault, StaleLocalizationError)
        completion.acknowledge.assert_not_called()
        self.assertFalse(any(name.endswith(('Target Received', 'Hold Acquired'))
                             for name, _ in events))


if __name__ == '__main__':
    unittest.main()
