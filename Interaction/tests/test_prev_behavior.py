"""Exercise the interaction entry's ordinary SFL hold without flying."""

from copy import deepcopy
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

from controller import Controller, EmergencyStopException, LowBatteryException


class PrevBehaviorTests(unittest.TestCase):
    def download(self, mission, *, intractable=False):
        owner = self.controller(intractable=intractable)
        owner.args.orchestrated = True
        owner.args.interaction = True
        owner.manifest['controller'] = {
            'ip': '192.0.2.1', 'http_port': 8000,
            'mission_file': 'prev_behavior.yaml',
        }
        payload = io.BytesIO(yaml.safe_dump(mission).encode())
        with patch('controller.urllib.request.urlopen', return_value=payload):
            owner.download_mission_config()
        return owner

    def test_old_downloaded_yaml_is_rejected_before_arming(self):
        mission = self.controller().mission
        del mission['Interaction']
        with patch.object(Controller, 'arm') as arm, patch.object(Controller, 'takeoff') as takeoff:
            with self.assertRaisesRegex(ValueError, 'requires an Interaction block'):
                self.download(mission)
        arm.assert_not_called()
        takeoff.assert_not_called()

    def test_downloaded_prev_yaml_selects_hold_without_detection(self):
        owner = self.download(self.controller().mission)
        with patch('controller.InteractionsControl') as detector:
            owner.interation_switch()
        detector.assert_not_called()
        self.assertEqual(owner._safe_sleep.call_args.args, (60.,))

    def test_legacy_recap_and_intractable_missions_still_download(self):
        mission = self.controller().mission
        del mission['Interaction']
        recap = {**mission, 'Recap': {'files': ['recorded.json']}}
        self.assertIn('Recap', self.download(recap).mission)
        self.assertNotIn('Interaction', self.download(mission, intractable=True).mission)

    def controller(self, *, behavior='prev', intractable=False):
        owner = Controller.__new__(Controller)
        owner.args = SimpleNamespace(
            drone_id='lb11', ground_test=False,
            intractable_illumination=intractable, crazysim=False,
            pid_attitude_source='standard',
        )
        owner.mission = {
            'drones': {'lb11': {
                'target': [.7, 1., 1., 0.], 'waypoints': [],
                'delta_t': 60., 'iterations': 1,
            }},
            'Interaction': {'action': 'translation', 'config': {
                'behavior': behavior, 'sensing': False,
                'record_potentiometer': False,
            }},
        }
        owner.missions = [owner.mission]
        owner.manifest = {'mission': {'require_handshake': False}}
        owner.init_coord = [.7, 1., .24]
        owner.mocap = None
        owner.use_flowdeck = False
        owner.led = None
        owner.hl_commander = Mock()
        owner.ll_commander = Mock()
        owner._safe_sleep = Mock()
        owner._compute_viewpoint_transform = Mock(return_value=None)
        owner._compute_reference_offset = Mock(return_value=[0., 0., 0.])
        owner.viewpoint_offsets = []
        owner.reference_offsets = []
        owner.animation_start_times = []
        owner.animation_stop_times = []
        return owner

    def test_interaction_entry_moves_to_sfl_target_and_holds_without_detector(self):
        for intractable in (False, True):
            with self.subTest(intractable=intractable):
                owner = self.controller(intractable=intractable)
                with patch('controller.InteractionsControl') as detector:
                    owner.interation_switch()
                detector.assert_not_called()
                command = owner.hl_commander.go_to.call_args
                self.assertEqual(command.args[:4], (.7, 1., 1., 0.))
                self.assertEqual(command.kwargs, {'relative': False})
                self.assertEqual(owner._safe_sleep.call_args.args, (60.,))
                owner.ll_commander.send_position_setpoint.assert_not_called()
                self.assertEqual(len(owner.animation_start_times), 1)
                self.assertEqual(len(owner.animation_stop_times), 1)

    def test_battery_and_stop_requests_propagate_to_normal_landing(self):
        for error in (LowBatteryException('battery'), EmergencyStopException('stop')):
            with self.subTest(error=type(error).__name__):
                owner = self.controller()
                def sleep(duration):
                    if duration == 60.:
                        raise error
                owner._safe_sleep.side_effect = sleep
                with self.assertRaises(type(error)):
                    owner.interation_switch()
                owner.ll_commander.send_notify_setpoint_stop.assert_called_once()

    def test_existing_intractable_interaction_dispatch_is_unchanged(self):
        owner = self.controller(behavior='standard', intractable=True)
        original = deepcopy(owner.mission)
        owner.run_multiple_orchestrated_missions = Mock()
        owner.orchestrated_mission_interaction = Mock()
        owner.interation_switch()
        owner.orchestrated_mission_interaction.assert_called_once_with()
        owner.run_multiple_orchestrated_missions.assert_not_called()
        self.assertEqual(owner.mission, original)


if __name__ == '__main__':
    unittest.main()
