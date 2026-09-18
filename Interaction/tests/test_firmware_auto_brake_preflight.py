import unittest
import time
from types import SimpleNamespace
from unittest.mock import patch

from controller import Controller
from Interaction.interactions import (
    InteractionsControl,
    TranslationControlHandoff,
    configure_firmware_owned_brake,
)
from Interaction.onboard_wrench_interaction_pipeline import (
    OnboardMomentumWrenchPipeline,
)


class FirmwareAutoBrakePreflightTests(unittest.TestCase):
    def test_yaw_log_is_optional_only_for_disabled_firmware_yaw_model(self):
        core = {
            'VEL_ORI': ('vx', 'vy', 'vz', 'roll', 'pitch', 'yaw'),
            'POS_ACC': ('x', 'y', 'z'),
            'RATE_EST': ('rateRoll', 'ratePitch', 'rateYaw'),
            'MOT_BAT': ('m1', 'm2', 'm3', 'm4', 'vbat'),
        }

        def values(group):
            if group == 'YAW_CTL':
                return {}
            prefix = {
                'VEL_ORI': 'stateEstimate',
                'POS_ACC': 'stateEstimate',
                'RATE_EST': 'stateEstimateZ',
                'MOT_BAT': 'motor',
            }[group]
            result = {f'{prefix}.{field}': 0.0 for field in core[group]}
            if group == 'MOT_BAT':
                result['pm.vbat'] = 8.0
                result.pop('motor.vbat')
            return result

        controller = self.controller({
            'enabled': True, 'response_time_s': 0.08,
        })
        wrench = controller.mission['Interaction']['config']['wrench_interaction']
        wrench['state_source'] = 'onboard'
        controller._is_interaction_application = lambda: True
        controller.log_manager = SimpleNamespace(
            get_latest_group_log_data=values,
        )
        controller.verify_onboard_wrench_logging()

        wrench['motor_model'] = {
            'yaw_command_model': {'enabled': True},
        }
        with patch('controller.time.monotonic', side_effect=(0.0, 0.0, 5.1)):
            with patch('controller.time.sleep'):
                with self.assertRaisesRegex(RuntimeError, 'YAW_CTL'):
                    controller.verify_onboard_wrench_logging()

    def test_firmware_state_assembly_does_not_fabricate_yaw_pid_logs(self):
        packets = {
            'VEL_ORI': {
                'stateEstimate.vx': 0.0, 'stateEstimate.vy': 0.0,
                'stateEstimate.vz': 0.0, 'stateEstimate.roll': 0.0,
                'stateEstimate.pitch': 0.0, 'stateEstimate.yaw': 0.0,
            },
            'POS_ACC': {
                'stateEstimate.x': 0.0, 'stateEstimate.y': 0.0,
                'stateEstimate.z': 1.0,
            },
            'RATE_EST': {
                'stateEstimateZ.rateRoll': 0,
                'stateEstimateZ.ratePitch': 0,
                'stateEstimateZ.rateYaw': 0,
            },
            'MOT_BAT': {
                'motor.m1': 30000, 'motor.m2': 30000,
                'motor.m3': 30000, 'motor.m4': 30000,
                'pm.vbat': 8.0,
            },
        }

        def nearest(group, timestamp):
            packet = packets.get(group)
            return (({**packet, 'time': timestamp}, 0.0)
                    if packet is not None else (None, None))

        control = InteractionsControl.__new__(InteractionsControl)
        control.mission = {'Interaction': {'config': {'wrench_interaction': {
            'firmware_auto_brake': {'enabled': True},
        }}}}
        control.log_manager = SimpleNamespace(
            get_latest_group_log_time=lambda _: 100.0,
            get_nearest_group_log_data=nearest,
        )
        state = control._get_synchronized_onboard_wrench_state()
        self.assertIsNotNone(state)
        self.assertIsNone(state['yaw_control_command'])
        self.assertIsNone(state['yaw_control_skew_s'])

        control.mission['Interaction']['config']['wrench_interaction'][
            'motor_model'] = {'yaw_command_model': {'enabled': True}}
        self.assertIsNone(control._get_synchronized_onboard_wrench_state())

    def test_firmware_owner_disables_only_pi_release_profile(self):
        original = {
            'control_handoff': {
                'coast_jerk_limited_attitude_enabled': True,
                'coast_jerk_limited_septic_smoothing_enabled': True,
                'coast_jerk_limited_free_stop_enabled': True,
                'coast_jerk_limited_use_vicon_velocity_reference': True,
                'coast_max_tilt_predictive_brake_enabled': True,
                'coast_velocity_braking_enabled': True,
            },
            'startup_bias_calibration_enabled': True,
            'calibration_excitation': {'enabled': True},
        }
        resolved = configure_firmware_owned_brake(original)
        self.assertTrue(original['control_handoff'][
            'coast_jerk_limited_septic_smoothing_enabled'])
        for key in (
            'coast_jerk_limited_attitude_enabled',
            'coast_jerk_limited_septic_smoothing_enabled',
            'coast_jerk_limited_free_stop_enabled',
            'coast_jerk_limited_use_vicon_velocity_reference',
            'coast_max_tilt_predictive_brake_enabled',
        ):
            self.assertFalse(resolved['control_handoff'][key])
        self.assertTrue(resolved['control_handoff'][
            'coast_velocity_braking_enabled'])
        self.assertFalse(resolved['startup_bias_calibration_enabled'])
        self.assertFalse(resolved['calibration_excitation']['enabled'])
        pipeline = OnboardMomentumWrenchPipeline(resolved)
        control = TranslationControlHandoff(
            [0.0, 0.0, 1.0], 0.0, False,
            **pipeline.config['control_handoff'],
        )
        self.assertFalse(control.coast_jerk_limited_free_stop_enabled)

    def controller(self, mode):
        instance = Controller.__new__(Controller)
        instance.args = SimpleNamespace(
            interaction=True, sense=True, vicon=True,
            vicon_mode='rigidbody', vicon_full_pose=False,
            log=True, crazysim=False, ground_test=False,
        )
        instance.mission = {'Interaction': {'config': {
            'wrench_interaction': {'firmware_auto_brake': mode},
        }}}
        return instance

    def test_explicit_measured_response_is_required(self):
        controller = self.controller({'enabled': True})
        with self.assertRaisesRegex(ValueError, 'response_time_s'):
            controller.prepare_firmware_auto_brake()
        controller = self.controller({
            'enabled': True, 'response_time_s': 0.08,
        })
        controller.prepare_firmware_auto_brake()
        self.assertTrue(controller.firmware_auto_brake_enabled)
        self.assertEqual(controller.firmware_auto_brake_response_time_s, 0.08)

    def test_other_interaction_modes_remain_default_off(self):
        controller = self.controller({})
        controller.prepare_firmware_auto_brake()
        self.assertFalse(controller.firmware_auto_brake_enabled)

    def test_mode_defaults_to_two_phase_and_zero_velocity_is_explicit(self):
        controller = self.controller({
            'enabled': True, 'response_time_s': 0.08,
        })
        controller.prepare_firmware_auto_brake()
        self.assertEqual(controller.firmware_auto_brake_mode, 'two_phase')
        controller = self.controller({
            'enabled': True, 'response_time_s': 0.08,
            'mode': 'zero_velocity',
        })
        controller.prepare_firmware_auto_brake()
        self.assertEqual(controller.firmware_auto_brake_mode, 'zero_velocity')
        controller = self.controller({
            'enabled': True, 'response_time_s': 0.08,
            'mode': 'unknown',
        })
        with self.assertRaisesRegex(ValueError, 'firmware_auto_brake.mode'):
            controller.prepare_firmware_auto_brake()

    def test_full_pose_and_offboard_shadow_are_rejected(self):
        controller = self.controller({
            'enabled': True, 'response_time_s': 0.08,
        })
        controller.args.vicon_full_pose = True
        with self.assertRaisesRegex(ValueError, 'position-only'):
            controller.prepare_firmware_auto_brake()
        controller.args.vicon_full_pose = False
        controller.mission['Interaction']['config']['wrench_interaction'][
            'contact_attitude_shadow_enabled'] = True
        with self.assertRaisesRegex(ValueError, 'offboard EKF'):
            controller.prepare_firmware_auto_brake()

    def test_readiness_requires_selected_firmware_mode(self):
        for name, code in [('two_phase', 0), ('zero_velocity', 1)]:
            controller = self.controller({
                'enabled': True, 'response_time_s': 0.08,
                'mode': name,
            })
            controller.prepare_firmware_auto_brake()
            controller.log_manager = SimpleNamespace(
                get_latest_group_log_data=lambda _: {
                    'hlCommander.pRelReady': 1,
                    'hlCommander.pRelAutoSt': 0,
                    'hlCommander.pRelAutoEn': 1,
                    'hlCommander.pRelEvtVer': 1,
                    'hlCommander.pRelMode': code,
                    'hlCommander.pRelTau': 0.08,
                }
            )
            controller._firmware_vicon_last_send_s = time.monotonic()
            controller._firmware_vicon_mirror_error = None
            controller.verify_firmware_auto_brake_ready()

    def test_old_firmware_without_atomic_handoff_protocol_refuses_arm(self):
        controller = self.controller({
            'enabled': True, 'response_time_s': 0.08,
        })
        controller.prepare_firmware_auto_brake()
        controller.log_manager = SimpleNamespace(
            get_latest_group_log_data=lambda _: {
                'hlCommander.pRelReady': 1,
                'hlCommander.pRelAutoSt': 0,
                'hlCommander.pRelAutoEn': 1,
                'hlCommander.pRelMode': 0,
                'hlCommander.pRelTau': 0.08,
            },
        )
        controller._firmware_vicon_last_send_s = 0.0
        controller._firmware_vicon_mirror_error = None
        with patch('controller.time.monotonic', side_effect=(0.0, 0.0, 5.1)):
            with patch('controller.time.sleep'):
                with self.assertRaisesRegex(RuntimeError, 'not ready'):
                    controller.verify_firmware_auto_brake_ready()


if __name__ == '__main__':
    unittest.main()
