"""Focused tests for the simulation-only periodic HLC hold owner."""

import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from Interaction.interactions import InteractionsControl


class HlcHoldRefreshTests(unittest.TestCase):
    def test_verified_velocity_remains_enabled_for_hold_refreshes(self):
        events = []
        high = SimpleNamespace(go_to=Mock())
        low = SimpleNamespace(send_position_setpoint=Mock())
        param = SimpleNamespace(set_value=Mock())
        loop = InteractionsControl.__new__(InteractionsControl)
        loop.cf = SimpleNamespace(param=param)
        loop.lo_commander = low
        loop.hl_commander = high
        loop.pid_attitude_source = 'post-release-15state'
        loop._translation_high_level_active = False
        loop._translation_handoff_origin = None
        loop._safe_sleep = time.sleep
        loop._log_event = lambda name, data: events.append((name, data))
        loop.mission = {'Interaction': {'config': {'wrench_interaction': {
            'control_handoff': {},
            'coast_jerk_limited_position_handoff_duration_s': 0.02,
            'post_handoff_observation_s': 0.22,
            'post_handoff_hlc_refresh_s': 0.03,
        }}}}
        with patch('Interaction.interactions.handoff_to_high_level', return_value={}):
            loop._handoff_translation_hold((0.1, 0.2, 1.0), 0.0)

        param.set_value.assert_called_once_with('hlCommander.pRelVel', '1')
        self.assertGreaterEqual(high.go_to.call_count, 1)
        for call in high.go_to.call_args_list:
            self.assertEqual(call.args[:3], (0.1, 0.2, 1.0))
            self.assertFalse(call.kwargs['relative'])
        refresh = [data for name, data in events
                   if name == 'Translation HLC Hold Refreshed']
        self.assertEqual(len(refresh), 1)
        self.assertEqual(refresh[0]['count'], high.go_to.call_count)

    def test_hlc_segment_uses_same_nested_duration_as_brake_planner(self):
        high = SimpleNamespace(go_to=Mock())
        loop = InteractionsControl.__new__(InteractionsControl)
        loop.cf = SimpleNamespace(param=SimpleNamespace(set_value=Mock()))
        loop.lo_commander = SimpleNamespace(send_position_setpoint=Mock())
        loop.hl_commander = high
        loop.pid_attitude_source = 'standard'
        loop._translation_high_level_active = False
        loop._translation_handoff_origin = None
        loop._log_event = Mock()
        loop.mission = {'Interaction': {'config': {'wrench_interaction': {
            'control_handoff': {
                'coast_jerk_limited_position_handoff_duration_s': 2.0,
            },
            'coast_jerk_limited_position_handoff_duration_s': 1.0,
        }}}}
        with patch('Interaction.interactions.handoff_to_high_level') as handoff:
            loop._handoff_translation_hold((0.1, 0.2, 1.0), 0.0)

        self.assertEqual(handoff.call_args.args[7], 2.0)
        acquired = [call.args[1] for call in loop._log_event.call_args_list
                    if call.args[0] == 'Translation High Level Hold Acquired']
        self.assertEqual(acquired[0]['first_segment_duration_s'], 2.0)


if __name__ == '__main__':
    unittest.main()
