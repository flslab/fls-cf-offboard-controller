import time
import unittest

import numpy as np

from Interaction.interactions import GuidedTouchProtocol, InteractionsControl


class FakeCommander:
    def __init__(self):
        self.calls = []

    def go_to(self, *args, **kwargs):
        self.calls.append(('go_to', args, kwargs))

    def send_position_setpoint(self, *args):
        self.calls.append(('position', args, {}))


class FakeLogManager:
    def __init__(self, base_time):
        self.groups = {
            'frames': [{
                'frame_id': 0,
                'time': base_time,
                'tvec': [0, 0, 1],
                'quat': [0, 0, 0, 1],
            }],
        }
        self.records = []

    def add_log_entry(self, group_name, entry, *args, **kwargs):
        self.groups.setdefault(group_name, []).append(entry)
        self.records.append((group_name, kwargs.get('name'), entry))

    def get_latest_group_log_data(self, group_name=None):
        if group_name == 'MOT_BAT':
            return {
                'motor.m1': 30000, 'motor.m2': 30000,
                'motor.m3': 30000, 'motor.m4': 30000,
                'pm.vbat': 8.0,
            }
        return {}

    def get_nearest_group_log_data(self, group_name, timestamp):
        packet = self.get_latest_group_log_data(group_name)
        packet['time'] = timestamp
        return packet, 0.0

    @staticmethod
    def get_latest_group_log_time(group_name):
        return time.time()


class FakeOnboardLogManager:
    def __init__(self, base_time):
        self.groups = {}
        self.records = []
        self.packet_time = base_time

    def advance(self, dt=0.02):
        self.packet_time += dt

    def add_log_entry(self, group_name, entry, *args, **kwargs):
        self.groups.setdefault(group_name, []).append(entry)
        self.records.append((group_name, kwargs.get('name'), entry))

    def get_latest_group_log_time(self, group_name):
        return self.packet_time if group_name == 'VEL_ORI' else None

    def get_nearest_group_log_data(self, group_name, timestamp):
        packets = {
            'VEL_ORI': {
                'stateEstimate.vx': 0.0,
                'stateEstimate.vy': 0.0,
                'stateEstimate.vz': 0.0,
                'stateEstimate.roll': 0.0,
                'stateEstimate.pitch': 0.0,
                'stateEstimate.yaw': 0.0,
            },
            'POS_ACC': {
                'stateEstimate.x': 0.0,
                'stateEstimate.y': 0.0,
                'stateEstimate.z': 1.0,
            },
            'RATE_EST': {
                'stateEstimateZ.rateRoll': 0,
                'stateEstimateZ.ratePitch': 0,
                'stateEstimateZ.rateYaw': 0,
            },
            'YAW_CTL': {
                'controller.cmd_yaw': 0.0,
                'controller.r_yaw': 0.0,
            },
            'MOT_BAT': {
                'motor.m1': 30000,
                'motor.m2': 30000,
                'motor.m3': 30000,
                'motor.m4': 30000,
                'pm.vbat': 8.0,
            },
        }
        packet = packets.get(group_name)
        if packet is None:
            return None, None
        return {**packet, 'time': self.packet_time}, abs(self.packet_time - timestamp)


class WrenchInteractionLoopTests(unittest.TestCase):
    def test_guided_touch_protocol_emits_countdown_touch_and_release_once(self):
        protocol = GuidedTouchProtocol({
            'enabled': True,
            'countdown_s': 3,
            'touch_s': 2.0,
            'rest_s': 1.0,
            'trials': ['X', 'Z'],
        })
        emitted = []
        for elapsed_s in (0.0, 0.9, 1.0, 2.0, 3.0, 5.0, 6.0, 9.0, 11.0, 12.0):
            emitted.extend(protocol.due(elapsed_s))
        names = [event[1] for event in emitted]
        self.assertEqual(names.count('Guided Touch Countdown'), 6)
        self.assertEqual(names.count('Guided Touch Start Expected'), 2)
        self.assertEqual(names.count('Guided Touch Release Expected'), 2)
        self.assertEqual(names.count('Guided Touch Test Complete'), 1)
        self.assertEqual(protocol.due(100.0), [])
        self.assertEqual(protocol.required_duration_s, 12.0)

    def test_yaw_only_chirp_keeps_position_fixed_and_ramps_to_nominal(self):
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.bounds = None
        config = {
            'duration_s': 24.0,
            'translation_amplitude_m': [0.0, 0.0, 0.0],
            'translation_frequency_hz': [0.2, 0.27, 0.33],
            'yaw_amplitude_deg': 12.0,
            'yaw_profile': 'chirp',
            'yaw_chirp_start_hz': 0.08,
            'yaw_chirp_end_hz': 0.45,
            'yaw_ramp_s': 2.0,
        }
        yaws = []
        for elapsed_s in np.linspace(0.0, 24.0, 241):
            position, yaw = controller._calibration_excitation_reference(
                [0.0, 0.0, 1.0], 5.0, config, elapsed_s,
            )
            np.testing.assert_allclose(position, [0.0, 0.0, 1.0])
            self.assertLessEqual(abs(yaw - 5.0), 12.0 + 1e-9)
            yaws.append(yaw)
        self.assertAlmostEqual(yaws[0], 5.0)
        self.assertAlmostEqual(yaws[-1], 5.0)
        self.assertGreater(max(yaws) - min(yaws), 20.0)

    def test_shadow_loop_uses_full_pose_and_never_applies_proposed_response(self):
        base_time = time.time()
        logs = FakeLogManager(base_time)
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.log_manager = logs
        controller.pos_group_name = 'frames'
        controller.ctrl_rate = 100
        controller.bounds = {
            'x_min': -1, 'x_max': 1,
            'y_min': -1, 'y_max': 1,
            'z_min': 0.3, 'z_max': 2,
        }
        controller.hl_commander = FakeCommander()
        controller.lo_commander = FakeCommander()

        def advance_frame(_duration):
            previous = logs.groups['frames'][-1]
            logs.groups['frames'].append({
                'frame_id': previous['frame_id'] + 1,
                'time': previous['time'] + 0.02,
                'tvec': [0, 0, 1],
                'quat': [0, 0, 0, 1],
            })

        controller._safe_sleep = advance_frame
        controller.interaction_wrench_admittance(
            duration=0,
            nominal_position=[0, 0, 1],
            nominal_yaw_deg=5,
            config={
                'shadow_mode': True,
                'observer_settle_s': 0,
                'bias_calibration_s': 0.01,
                'minimum_bias_samples': 1,
                'motor_model': {'hover_pwm': 30000, 'hover_voltage': 8.0},
                'safety': {
                    'max_frame_age_s': 10,
                    'max_motor_age_s': 10,
                    'max_motor_pose_skew_s': 1,
                    'startup_timeout_s': 1,
                    'require_motor_data': True,
                },
            },
        )

        position_calls = [call for call in controller.lo_commander.calls if call[0] == 'position']
        self.assertGreaterEqual(len(position_calls), 2)
        for _, args, _ in position_calls:
            self.assertEqual(args, (0.0, 0.0, 1.0, 5.0))
        names = [name for group, name, _entry in logs.records if group == 'events']
        self.assertIn('Wrench Calibration Complete', names)
        observer_rows = [entry for group, _name, entry in logs.records if group == 'wrench_observer']
        self.assertTrue(observer_rows[-1]['shadow_mode'])
        self.assertNotIn('roll_pitch_detect_only', observer_rows[-1])

    def test_onboard_shadow_loop_does_not_require_full_pose_mocap(self):
        logs = FakeOnboardLogManager(time.time())
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.log_manager = logs
        controller.ctrl_rate = 100
        controller.bounds = {
            'x_min': -1, 'x_max': 1,
            'y_min': -1, 'y_max': 1,
            'z_min': 0.3, 'z_max': 2,
        }
        controller.hl_commander = FakeCommander()
        controller.lo_commander = FakeCommander()
        controller._safe_sleep = lambda _duration: logs.advance()

        controller.interaction_onboard_wrench_admittance(
            duration=0,
            nominal_position=[0, 0, 1],
            nominal_yaw_deg=5,
            config={
                'state_source': 'onboard',
                'shadow_mode': True,
                'observer_settle_s': 0,
                'bias_calibration_s': 0.01,
                'minimum_bias_samples': 1,
                'motor_model': {'hover_pwm': 30000, 'hover_voltage': 8.0},
                'safety': {
                    'max_frame_age_s': 10,
                    'max_state_age_s': 10,
                    'max_motor_age_s': 10,
                    'max_motor_pose_skew_s': 1,
                    'max_state_group_skew_s': 1,
                    'max_motor_state_skew_s': 1,
                    'startup_timeout_s': 1,
                    'require_motor_data': True,
                },
            },
        )

        position_calls = [
            call for call in controller.lo_commander.calls
            if call[0] == 'position'
        ]
        self.assertGreaterEqual(len(position_calls), 2)
        for _, args, _ in position_calls:
            self.assertEqual(args, (0.0, 0.0, 1.0, 5.0))
        observer_rows = [
            entry for group, _name, entry in logs.records
            if group == 'wrench_observer'
        ]
        self.assertEqual(
            observer_rows[-1]['state_source'],
            'crazyflie_state_estimate',
        )
        self.assertTrue(observer_rows[-1]['shadow_mode'])


if __name__ == '__main__':
    unittest.main()
