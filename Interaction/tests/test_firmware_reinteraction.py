"""Exercise repeated physical contact edges in the production interaction loop."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

from Interaction.interactions import InteractionsControl, TranslationControlHandoff
from Interaction.post_release_firmware_control_event import (
    HOLD_NOTICE_PACKET, HOLD_NOTICE_TYPE, VERSION,
)
from Interaction.tests.test_wrench_interactions_integration import (
    FakeCommander, FakeHandoffCF, FakeOnboardLogManager,
)


class FirmwareReinteractionTests(unittest.TestCase):
    def test_firmware_hold_does_not_emit_low_level_commands_until_new_contact(self):
        control = TranslationControlHandoff([0, 0, 1], 0, False, rearm_delay_s=.02)
        control.start_contact('orientation', [0, 0, 1])
        control.set_contact_attitude(10, 0)
        control.accept_firmware_hold([.1, .2, 1.1], 15, 2.)
        low, high = Mock(), Mock()
        self.assertIsNone(control.send(low, high_level_commander=high))
        self.assertIsNone(control.send(low, high_level_commander=high))
        self.assertFalse(low.mock_calls)
        self.assertFalse(high.mock_calls)
        np.testing.assert_allclose(control.hold_position, [.1, .2, 1.1])
        self.assertEqual(control.mode, control.POSITION_HOLD)
        self.assertFalse(control.consume_detector_rearm(2.01))
        self.assertTrue(control.consume_detector_rearm(2.03))
        self.assertTrue(control.start_contact('orientation', [.1, .2, 1.1]))
        self.assertFalse(control.firmware_hold_active)
        control.send(low)
        self.assertTrue(low.mock_calls)

    def run_contacts(self, old_flag=None, *, duration=1., second_contact=True):
        clock = {'now': 1000., 'sample': 0, 'release': 0, 'stage': 0}
        logs = FakeOnboardLogManager(clock['now'])
        control = InteractionsControl.__new__(InteractionsControl)
        control.drone_id = 'lb11'
        control.log_manager = logs
        control.ctrl_rate = 100
        control.bounds = dict(x_min=-1, x_max=1, y_min=-1, y_max=1, z_min=.3, z_max=2)
        control.hl_commander = FakeCommander()
        control.lo_commander = FakeCommander()
        control.cf = FakeHandoffCF()
        control.cf.send_packet = Mock()
        control.cf.param = SimpleNamespace(set_value_raw=Mock())
        control.force_sensor = SimpleNamespace(
            spring_constant_n_per_mm=.16, max_extension_mm=10.4, rpi_power_monitor=None,
            latest=lambda: None)
        control.sense_axis = 'y'
        control.sense_axis_index = 1
        control.sense_sign = 1
        control.sense_max_age_s = .25
        commands_during_hold = []
        def track_sender(original):
            def send(*args):
                if getattr(control, '_translation_high_level_active', False):
                    commands_during_hold.append(args)
                original(*args)
            return send
        for name in ('send_position_setpoint', 'send_zdistance_setpoint'):
            setattr(control.lo_commander, name, track_sender(getattr(control.lo_commander, name)))

        def sleep(seconds):
            clock['now'] += max(seconds, 0)
            logs.packet_time = clock['now']
            if clock['stage'] == 2 and clock['now'] >= clock['hold_due']:
                clock['stage'] = 4
                packet = CRTPPacket()
                packet.set_header(CRTPPort.SETPOINT_HL, 1)
                packet.data = HOLD_NOTICE_PACKET.pack(
                    HOLD_NOTICE_TYPE, VERSION, identities[-1][1], identities[-1][0],
                    0., .2 * clock['release'], 1., 0., clock['release'] * 100000)
                for callback in list(control.cf.callbacks):
                    callback(packet)
            if clock['now'] > 1005:
                self.fail('interaction duration was not honored')

        control._safe_sleep = sleep
        control._firmware_brake_status_snapshot = lambda: ({
            'hlCommander.pRelAutoSt': clock['stage'], 'hlCommander.pRelReady': 1,
            'hlCommander.pRelAutoTime': 0,
        }, clock['now'])

        def sensor(_estimate, _now):
            i = clock['sample']; clock['sample'] += 1
            pressed = 5 <= i < 12 or (second_contact and 35 <= i < 42)
            force = .15 if pressed else 0.
            return dict(force_sensor_fresh=True, force_sensor_sample_time=clock['now'],
                        force_sensor_sample_monotonic_time=clock['now'],
                        force_sensor_arduino_time_ms=i * 10,
                        force_sensor_compression_force_N=force,
                        force_sensor_compression_mm=force/.16,
                        force_sensor_external_force_N=[0., force, 0.])

        control._force_sensor_log_fields = sensor
        identities = []

        def release(_cf, **event):
            identity = (event['session_id'], event['sequence'])
            self.assertNotIn(identity, identities, 'firmware would reject a duplicate release')
            identities.append(identity)
            clock['release'] += 1
            clock['stage'] = 2
            clock['hold_due'] = clock['now'] + .08
            return event
        firmware = dict(enabled=True, mode='scurve')
        if old_flag is not None:
            firmware['hold_until_duration'] = old_flag
        config = dict(
            state_source='onboard', shadow_mode=False,
            startup_bias_calibration_enabled=False,
            initial_contact_arming={'enabled': False},
            detection={'translation': {'enabled': False}, 'yaw': {'enabled': False}},
            predictive_braking={'enabled': False},
            learning_velocity_mpc_shadow={'enabled': False, 'command_authority': False},
            firmware_auto_brake=firmware,
            control_handoff={'coast_velocity_braking_enabled': False,
                             'coast_velocity_predictive_unwind_enabled': False},
            safety={'max_frame_age_s': 10, 'max_state_age_s': 10, 'max_motor_age_s': 10,
                    'max_motor_pose_skew_s': 1, 'max_state_group_skew_s': 1,
                    'max_motor_state_skew_s': 1, 'startup_timeout_s': 1,
                    'require_motor_data': True},
        )
        virtual = dict(
            inertia_command='orientation', force_rendering={'enabled': False},
            contact_detection={'source': 'potentiometer', 'force_threshold_n': .08,
                               'onset_dwell_s': 0.},
            release_behavior={'mode': 'potentiometer_coast', 'force_drop_n': .01,
                              'candidate_lead_drop_n': .005, 'decrease_rate_n_s': .01,
                              'unloaded_force_n': .05, 'unloaded_dwell_s': .02,
                              'max_sample_gap_s': .15},
        )
        with patch('Interaction.interactions.time.time', side_effect=lambda: clock['now']), \
             patch('Interaction.interactions.time.monotonic', side_effect=lambda: clock['now']), \
             patch('Interaction.interactions.handoff_pi_release_to_firmware', side_effect=release):
            control.interaction_onboard_wrench_admittance(
                duration=duration, nominal_position=[0., 0., 1.], nominal_yaw_deg=0.,
                rearm_delay_s=.02, config=config, virtual_object_config=virtual)
        starts = [r for group, name, r in logs.records if name == 'Waiting For User Interaction']
        self.assertTrue(starts)
        self.assertGreaterEqual(clock['now'] - starts[0]['time'], duration)
        self.assertLess(clock['now'] - starts[0]['time'], duration + .011)
        self.assertEqual(commands_during_hold, [])
        self.assertEqual(len(control.hl_commander.calls), 1, 'no extra goto at either handoff')
        self.assertEqual(control.cf.send_packet.call_count, len(identities), 'one hold ACK per release')
        self.assertEqual(control.cf.callbacks, [], 'release listeners must be cleaned up')
        return identities, logs.records, control

    def test_two_contacts_run_by_default_until_original_deadline(self):
        identities, records, control = self.run_contacts()
        self.assertEqual([seq for _, seq in identities], [0, 1])
        self.assertEqual(len({session for session, _ in identities}), 1)
        self.assertEqual(sum(name == 'Translation Contact Start' for _, name, _ in records), 2)
        self.assertTrue(control._translation_high_level_active)

    def test_legacy_flag_neither_locks_hold_nor_ends_mission(self):
        for value in (False, True):
            with self.subTest(old_flag=value):
                identities, _, _ = self.run_contacts(value)
                self.assertEqual(len(identities), 2)

    def test_no_second_contact_keeps_hold_until_duration(self):
        identities, _, control = self.run_contacts(second_contact=False)
        self.assertEqual(len(identities), 1)
        self.assertTrue(control._translation_high_level_active)

    def test_duration_during_second_contact_does_not_land_via_old_hlc_hold(self):
        identities, records, control = self.run_contacts(duration=.46)
        self.assertEqual(len(identities), 1)
        self.assertEqual(sum(name == 'Translation Contact Start' for _, name, _ in records), 2)
        self.assertFalse(control._translation_high_level_active)
        self.assertIsNone(control._translation_exit_target)

    def test_no_new_contact_after_duration(self):
        identities, records, control = self.run_contacts(duration=.3)
        self.assertEqual(len(identities), 1)
        self.assertEqual(sum(name == 'Translation Contact Start' for _, name, _ in records), 1)
        self.assertTrue(control._translation_high_level_active)


if __name__ == '__main__':
    unittest.main()
