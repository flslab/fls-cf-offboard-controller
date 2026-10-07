"""Diagnostic capture must fit CRTP packets and leave control telemetry intact."""

from copy import deepcopy
import unittest

from Interaction.config import log_vars_for_mission
from Interaction.roll_diagnostics import ROLL_DIAGNOSTIC_LOGS


class RollDiagnosticTests(unittest.TestCase):
    def mission(self, enabled=False):
        return {'Interaction': {'config': {
            'behavior': 'level_coast',
            'level_coast': {'coast_command_mode': 'scurve', 'roll_diagnostics': enabled},
            'wrench_interaction': {'firmware_auto_brake': {'enabled': True}},
        }}}

    def test_opt_in_preserves_existing_control_groups_and_mission(self):
        mission = self.mission(True)
        before = deepcopy(mission)
        baseline = log_vars_for_mission(self.mission())
        selected = log_vars_for_mission(mission)
        self.assertEqual(mission, before)
        self.assertEqual(set(selected) - set(baseline), set(ROLL_DIAGNOSTIC_LOGS))
        for name, block in baseline.items():
            self.assertEqual(selected[name], block)
        # The CF log data payload is bounded to 26 bytes per block.
        sizes = {'float': 4, 'uint8_t': 1, 'uint32_t': 4}
        for name in ROLL_DIAGNOSTIC_LOGS:
            block = selected[name]
            self.assertLessEqual(sum(sizes[value['type']] for key, value in block.items()
                                     if key != 'log_period_ms'), 26)
            self.assertEqual(block['log_period_ms'], 20)

    def test_invalid_flag_and_wrong_control_mode_fail(self):
        for enabled, mode in [('true', 'scurve'), (True, 'orientation')]:
            mission = self.mission(enabled)
            mission['Interaction']['config']['level_coast']['coast_command_mode'] = mode
            with self.assertRaises(ValueError):
                log_vars_for_mission(mission)


if __name__ == '__main__':
    unittest.main()
