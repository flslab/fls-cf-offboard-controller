import unittest
import time
from types import SimpleNamespace

from controller import Controller


class FirmwareAutoBrakePreflightTests(unittest.TestCase):
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
                    'hlCommander.pRelMode': code,
                    'hlCommander.pRelTau': 0.08,
                }
            )
            controller._firmware_vicon_last_send_s = time.monotonic()
            controller._firmware_vicon_mirror_error = None
            controller.verify_firmware_auto_brake_ready()


if __name__ == '__main__':
    unittest.main()
