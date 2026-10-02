from copy import deepcopy
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

from controller import Controller
from Interaction.config import log_vars_for_mission
from Interaction.level_coast import validate_level_coast
from Interaction.mission_profiles import PROFILES, resolve_mission_profiles


def mission():
    return {'drones': {'lb11': {'target': [0, -1, 1]}}, 'Interaction': {
        'action': 'translation', 'config': {
            'behavior': 'level_coast', 'detection_method': 'potentiometer',
            'duration': 60, 'grace_time': .5,
            'level_coast': {'stop_speed_m_s': .03},
            'wrench_interaction_profile': 'level_coast',
        }}}


class MissionProfileTests(unittest.TestCase):
    def test_all_detection_methods_keep_onboard_logging_and_both_command_modes(self):
        for method in ('potentiometer', 'model', 'vel'):
            for command in ('position', 'orientation'):
                with self.subTest(method=method, command=command):
                    resolved = resolve_mission_profiles(mission())
                    config = resolved['Interaction']['config']
                    config['detection_method'] = method
                    config['level_coast']['command_mode'] = command
                    options = validate_level_coast(config, sensor_available=method == 'potentiometer')
                    self.assertEqual(options['detection_method'], method)
                    self.assertEqual(options['command_mode'], command)
                    control = Controller.__new__(Controller)
                    control.args = SimpleNamespace(interaction=True)
                    control.mission = resolved
                    self.assertTrue(control._uses_onboard_wrench_state())
                    self.assertTrue({'VEL_ORI', 'POS_ACC', 'RATE_EST', 'MOT_BAT'}
                                    .issubset(log_vars_for_mission(resolved)))

    def test_level_coast_calibration_keeps_onboard_pipeline_for_all_methods(self):
        from Interaction.interactions import InteractionsControl
        from Interaction.tests.test_wrench_interactions_integration import FakeCommander
        for method in ('potentiometer', 'model', 'vel'):
            with self.subTest(method=method):
                control = InteractionsControl.__new__(InteractionsControl)
                control.drone_id = 'lb11'
                control.lo_commander = FakeCommander()
                control.mission = resolve_mission_profiles(mission())
                control.mission['Interaction']['config']['detection_method'] = method
                control.interaction_onboard_wrench_admittance = Mock()
                control._run_translation(calibration_mode=True)
                control.interaction_onboard_wrench_admittance.assert_called_once()
                self.assertTrue(control.interaction_onboard_wrench_admittance.call_args.kwargs['calibration_mode'])

    def test_profile_is_portable_and_retains_baseline_values(self):
        raw = mission()
        before = deepcopy(raw)
        resolved = resolve_mission_profiles(raw)
        config = resolved['Interaction']['config']
        wrench = config['wrench_interaction']
        self.assertEqual(raw, before)
        self.assertTrue(PROFILES['level_coast'].is_absolute())
        self.assertEqual(wrench['motor_model']['pitch_sign_for_world_thrust'], -1)
        self.assertEqual(wrench['mass'], .17)
        self.assertEqual(wrench['initial_contact_arming']['stationary_dwell_s'], .5)
        self.assertEqual(wrench['safety']['max_state_age_s'], .1)
        self.assertFalse(wrench['firmware_auto_brake']['enabled'])
        self.assertEqual(set(wrench['impulse_estimator']),
                         {'window_s', 'minimum_window_s', 'max_dt_s'})
        validate_level_coast(config, sensor_available=True)
        self.assertTrue({'VEL_ORI', 'POS_ACC', 'RATE_EST', 'MOT_BAT'}
                        .issubset(log_vars_for_mission(resolved)))

    def test_nested_overrides_preserve_siblings_and_do_not_mutate_profile(self):
        raw = mission()
        raw['Interaction']['config']['wrench_interaction'] = {
            'safety': {'max_state_age_s': .08},
            'detection': {'translation': {'component_thresholds': [.1, .1, .15]}},
        }
        config = resolve_mission_profiles(raw)['Interaction']['config']['wrench_interaction']
        self.assertEqual(config['safety']['max_state_age_s'], .08)
        self.assertEqual(config['safety']['max_motor_age_s'], .15)
        self.assertEqual(config['detection']['translation']['component_thresholds'], [.1, .1, .15])
        self.assertEqual(config['detection']['translation']['release_time_s'], .15)
        untouched = resolve_mission_profiles(mission())['Interaction']['config']['wrench_interaction']
        self.assertEqual(untouched['safety']['max_state_age_s'], .1)

    def test_inline_only_missions_are_unchanged(self):
        raw = mission()
        config = raw['Interaction']['config']
        config.pop('wrench_interaction_profile')
        config['wrench_interaction'] = {'firmware_auto_brake': {'enabled': True, 'mode': 'scurve'}}
        self.assertEqual(resolve_mission_profiles(raw), raw)
        self.assertEqual(resolve_mission_profiles({'drones': {}}), {'drones': {}})

    def test_unknown_missing_and_malformed_profiles_fail(self):
        for name in ('missing', '../level_coast', None, ['level_coast']):
            with self.subTest(name=name):
                raw = mission()
                raw['Interaction']['config']['wrench_interaction_profile'] = name
                with self.assertRaisesRegex(ValueError, 'Unknown'):
                    resolve_mission_profiles(raw)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'profile.yaml'
            with patch.dict(PROFILES, {'level_coast': path}):
                with self.assertRaises(FileNotFoundError):
                    resolve_mission_profiles(mission())
                path.write_text('- not a mapping\n')
                with self.assertRaisesRegex(ValueError, 'mapping'):
                    resolve_mission_profiles(mission())

    def test_download_expands_profile_before_preflight_for_each_matching_mission(self):
        control = Controller.__new__(Controller)
        control.args = SimpleNamespace(orchestrated=True, drone_id='lb11',
                                       sense=True, calibrate=False, crazysim=False)
        control.manifest = {'controller': {'ip': '127.0.0.1', 'http_port': 8000,
                                          'mission_files': ['one.yaml', 'two.yaml']}}
        data = yaml.safe_dump(mission()).encode()
        with patch('controller.urllib.request.urlopen', side_effect=lambda _: io.BytesIO(data)):
            control.download_mission_config()
        self.assertEqual(len(control.missions), 2)
        self.assertIs(control.mission, control.missions[0])
        control.prepare_firmware_auto_brake()
        self.assertFalse(control.firmware_auto_brake_enabled)
        for resolved in control.missions:
            validate_level_coast(resolved['Interaction']['config'], sensor_available=True)
        control.missions[0]['Interaction']['config']['wrench_interaction']['mass'] = .25
        self.assertEqual(control.missions[1]['Interaction']['config']['wrench_interaction']['mass'], .17)


if __name__ == '__main__':
    unittest.main()
