import unittest
import time
from threading import Event
from types import SimpleNamespace
from unittest.mock import Mock, patch

from controller import Controller, LowBatteryException
from Interaction.interactions import (
    InteractionsControl,
    StaleLocalizationError,
    TranslationControlHandoff,
    configure_firmware_owned_brake,
)
from Interaction.onboard_wrench_interaction_pipeline import (
    OnboardMomentumWrenchPipeline,
)


class FirmwareAutoBrakePreflightTests(unittest.TestCase):
    def test_firmware_monitor_uses_sticky_battery_abort(self):
        controller = Controller.__new__(Controller)
        controller.battery_critical = Event()
        controller.battery_critical.set()
        controller.voltage = 6.94
        controller._prepare_for_emergency_landing = Mock()
        control = InteractionsControl.__new__(InteractionsControl)
        control._safe_sleep = controller._safe_sleep_standalone

        with self.assertRaisesRegex(LowBatteryException, '6.94V'):
            control._check_firmware_brake_monitor_safety()
        controller._prepare_for_emergency_landing.assert_called_once_with()

    def test_firmware_abort_is_not_treated_as_velocity_phase(self):
        control = InteractionsControl.__new__(InteractionsControl)
        control._safe_sleep = Mock()

        with self.assertRaisesRegex(RuntimeError,
                                    'reason not reported'):
            control._check_firmware_brake_monitor_safety(
                {'hlCommander.pRelAutoSt': 6})
        control._safe_sleep.assert_called_once_with(0.0)

    def run_firmware_wait(self, snapshot, *, notice_at=0.0,
                          abort_at=None, expected_error=None):
        clock = {'now': 0.0}
        control = InteractionsControl.__new__(InteractionsControl)
        control._log_event = Mock()
        checked_at = []

        def safe_sleep(duration):
            checked_at.append(clock['now'])
            if abort_at is not None and clock['now'] >= abort_at:
                raise LowBatteryException('critical battery during firmware brake')
            clock['now'] += duration

        control._safe_sleep = safe_sleep
        control._firmware_brake_status_snapshot = lambda: snapshot(clock['now'])
        completion = SimpleNamespace(
            wait=lambda _: ({'session_id': 99, 'sequence': 7}
                            if clock['now'] >= notice_at else None),
            acknowledge=Mock(),
        )
        with patch('Interaction.interactions.time.monotonic',
                   side_effect=lambda: clock['now']), patch(
                       'Interaction.interactions.time.time',
                       side_effect=lambda: 1000.0 + clock['now']):
            if expected_error is None:
                control._wait_for_firmware_brake_hold(
                    completion, brake_mode='two_stage',
                    baseline_receipt_s=999.9, baseline_timeouts=7)
            else:
                with self.assertRaisesRegex(*expected_error):
                    control._wait_for_firmware_brake_hold(
                        completion, brake_mode='two_stage',
                        baseline_receipt_s=999.9, baseline_timeouts=7)
        return clock['now'], completion, control, checked_at

    @staticmethod
    def brake_packet(now, *, stage=2, ready=1, reason=0):
        return ({'hlCommander.pRelAutoSt': stage,
                 'hlCommander.pRelReady': ready,
                 'hlCommander.pRelAbort': reason,
                 'hlCommander.pRelAutoTime': 7}, 1000.0 + now)

    def test_wait_loop_reports_stage_six_on_first_observed_fault(self):
        def snapshot(now):
            return self.brake_packet(now, stage=6 if now >= 0.1 else 2,
                                     ready=0, reason=2)

        elapsed, completion, control, _ = self.run_firmware_wait(
            snapshot, expected_error=(RuntimeError, 'unwind plan invalid'))
        self.assertAlmostEqual(elapsed, 0.1)
        completion.acknowledge.assert_not_called()
        self.assertEqual(control._log_event.call_args.args[1]['code'],
                         'firmware_abort')

    def test_monitor_snapshot_keeps_values_with_their_actual_receipt_time(self):
        control = InteractionsControl.__new__(InteractionsControl)
        packet = {'time': 1000.0, 'hlCommander.pRelAutoSt': 2}
        control.log_manager = SimpleNamespace(
            # Simulate receipt metadata advancing before the callback stores
            # the new data packet. The preceding packet must retain its age.
            get_latest_group_log_time=lambda _: 1000.1,
            get_nearest_group_log_data=Mock(return_value=(packet, 0.1)),
        )
        self.assertEqual(control._firmware_brake_status_snapshot(),
                         (packet, 1000.0))

    def test_release_rejection_capture_uses_new_status_packet(self):
        control = InteractionsControl.__new__(InteractionsControl)
        control._firmware_brake_status_snapshot = lambda: ({
            'hlCommander.pRelRejR': 3,
            'hlCommander.pRelRejD': 1,
        }, 1000.02)
        control._safe_sleep = Mock()

        brake_log, diagnostics = (
            control._capture_firmware_release_rejection(1000.0))

        self.assertEqual(brake_log['hlCommander.pRelRejR'], 3)
        self.assertTrue(diagnostics['fresh_post_release_packet'])
        self.assertIn('predictor not ready',
                      diagnostics['firmware_reject_detail_description'])
        control._safe_sleep.assert_not_called()

    def test_release_rejection_capture_does_not_reuse_sticky_old_reason(self):
        control = InteractionsControl.__new__(InteractionsControl)
        clock = {'now': 0.0}
        control._firmware_brake_status_snapshot = lambda: ({
            'hlCommander.pRelRejR': 5,
            'hlCommander.pRelRejD': 0,
        }, 1000.0)

        def safe_sleep(duration):
            clock['now'] += duration

        control._safe_sleep = safe_sleep
        with patch('Interaction.interactions.time.monotonic',
                   side_effect=lambda: clock['now']):
            brake_log, diagnostics = (
                control._capture_firmware_release_rejection(1000.0))

        self.assertEqual(brake_log, {})
        self.assertFalse(diagnostics['fresh_post_release_packet'])
        self.assertIsNone(diagnostics['firmware_reject_reason'])

    def test_firmware_hold_observation_waits_for_remaining_duration(self):
        control = InteractionsControl.__new__(InteractionsControl)
        clock = {'elapsed': 2.0}
        control._log_event = Mock()
        control._firmware_brake_status_snapshot = lambda: ({
            'hlCommander.pRelAutoSt': 4,
        }, 100.0 + clock['elapsed'])

        def safe_sleep(duration):
            clock['elapsed'] += duration

        control._safe_sleep = safe_sleep
        with patch('Interaction.interactions.time.monotonic',
                   side_effect=lambda: clock['elapsed']), patch(
                       'Interaction.interactions.time.time',
                       side_effect=lambda: 100.0 + clock['elapsed']):
            waited = control._hold_firmware_brake_until_interaction_duration(
                enabled=True, interaction_start_s=100.0,
                duration_s=5.0, brake_mode='scurve')

        self.assertTrue(waited)
        self.assertAlmostEqual(clock['elapsed'], 5.0)
        self.assertEqual(
            [call.args[0] for call in control._log_event.call_args_list],
            ['Post-Release Hold Observation',
             'Post-Release Hold Observation Complete'])

    def test_firmware_hold_observation_is_opt_in(self):
        control = InteractionsControl.__new__(InteractionsControl)
        control._safe_sleep = Mock()
        control._log_event = Mock()

        waited = control._hold_firmware_brake_until_interaction_duration(
            enabled=False, interaction_start_s=100.0,
            duration_s=60.0, brake_mode='scurve')

        self.assertFalse(waited)
        control._safe_sleep.assert_not_called()
        control._log_event.assert_not_called()

    def test_firmware_hold_observation_fails_if_firmware_leaves_hold(self):
        control = InteractionsControl.__new__(InteractionsControl)
        control._safe_sleep = Mock()
        control._log_event = Mock()
        control._firmware_brake_status_snapshot = lambda: ({
            'hlCommander.pRelAutoSt': 0,
        }, 102.0)
        with patch('Interaction.interactions.time.monotonic',
                   return_value=2.0), patch(
                       'Interaction.interactions.time.time',
                       return_value=102.0):
            with self.assertRaisesRegex(RuntimeError, 'left post-release hold'):
                control._hold_firmware_brake_until_interaction_duration(
                    enabled=True, interaction_start_s=100.0,
                    duration_s=5.0, brake_mode='scurve')

    def test_wait_loop_checks_battery_during_telemetry_gap_and_pending_notice(self):
        elapsed, completion, _, checked_at = self.run_firmware_wait(
            lambda _: self.brake_packet(0.0), abort_at=0.15,
            expected_error=(LowBatteryException, 'critical battery'))
        self.assertLessEqual(elapsed, 0.15 + 1e-9)
        self.assertGreaterEqual(len(checked_at), 4)
        completion.acknowledge.assert_not_called()

    def test_wait_loop_never_acks_cached_hold_or_notice_without_new_hold_status(self):
        def snapshot(now):
            if now < 0.1:
                return self.brake_packet(-0.1, stage=4)
            return self.brake_packet(now, stage=4 if now >= 0.3 else 2)

        elapsed, completion, control, _ = self.run_firmware_wait(snapshot)
        self.assertAlmostEqual(elapsed, 0.3)
        completion.acknowledge.assert_called_once_with()
        self.assertEqual(control._log_event.call_args.args[0],
                         'Firmware Post-Release Hold Acquired')

    def test_wait_loop_preserves_terminal_timeout_with_fresh_logs(self):
        elapsed, completion, _, _ = self.run_firmware_wait(
            lambda now: self.brake_packet(now),
            expected_error=(RuntimeError, 'within 6 s'))
        self.assertGreaterEqual(elapsed, 6.0)
        self.assertLess(elapsed, 6.1)
        completion.acknowledge.assert_not_called()

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

    def firmware_params(self, version=26092101, *, extended=True):
        names = ['pRelAuto', 'pRelMode', 'pRelTau']
        if extended:
            names += ['pRelJoint', 'pRelHost', 'pRelJVer']
        values = {'hlCommander.pRelJVer': str(version),
                  'hlCommander.pRelJoint': '1', 'hlCommander.pRelHost': '1'}
        return SimpleNamespace(
            toc=SimpleNamespace(toc={'kalmanPRel': {'enable': object()},
                                    'hlCommander': dict.fromkeys(names)}),
            set_value=Mock(), get_value=lambda key: values[key],
        )

    def test_pi_joint_requires_new_firmware_and_prewarms_before_enable(self):
        controller = self.controller({
            'enabled': True, 'response_time_s': 0.14, 'mode': 'pi_joint'})
        controller.prepare_firmware_auto_brake()
        controller.cf = SimpleNamespace(param=self.firmware_params(26091804))
        with self.assertRaisesRegex(RuntimeError, '26092101'):
            controller._setup_firmware_auto_brake_params()
        controller.cf.param.set_value.assert_not_called()

        controller.cf.param = self.firmware_params()
        planner = Mock()
        planner.status.return_value = {'ready': True}
        def assert_not_enabled():
            self.assertNotIn(('hlCommander.pRelAuto', '1'),
                             [call.args for call in
                              controller.cf.param.set_value.call_args_list])
        planner.start.side_effect = assert_not_enabled
        with patch('Interaction.post_release_pi_planner.PiEventPlanner',
                   return_value=planner):
            controller._setup_firmware_auto_brake_params()
        self.assertIs(controller.cf._post_release_pi_planner, planner)
        planner.start.assert_called_once_with()
        calls = [call.args for call in controller.cf.param.set_value.call_args_list]
        self.assertIn(('hlCommander.pRelHost', '1'), calls)
        self.assertIn(('hlCommander.pRelJoint', '1'), calls)
        self.assertEqual(calls[-1], ('hlCommander.pRelAuto', '1'))
        self.assertFalse(any('pid' in name.lower() for name, _ in calls))

    def test_pi_worker_start_failure_never_enables_host_mode(self):
        controller = self.controller({
            'enabled': True, 'response_time_s': 0.14, 'mode': 'pi_joint'})
        controller.prepare_firmware_auto_brake()
        controller.cf = SimpleNamespace(param=self.firmware_params())
        planner = Mock()
        planner.start.side_effect = RuntimeError('prewarm failed')
        with patch('Interaction.post_release_pi_planner.PiEventPlanner',
                   return_value=planner):
            with self.assertRaisesRegex(RuntimeError, 'prewarm failed'):
                controller._setup_firmware_auto_brake_params()
        planner.close.assert_called_once_with()
        controller.cf.param.set_value.assert_not_called()

    def test_legacy_modes_clear_stale_experiment_without_requiring_new_params(self):
        for mode in ('two_phase', 'zero_velocity'):
            for extended in (True, False):
                controller = self.controller({
                    'enabled': True, 'response_time_s': 0.14, 'mode': mode})
                controller.prepare_firmware_auto_brake()
                controller.cf = SimpleNamespace(
                    param=self.firmware_params(extended=extended))
                controller._setup_firmware_auto_brake_params()
                calls = [call.args for call in
                         controller.cf.param.set_value.call_args_list]
                self.assertEqual(('hlCommander.pRelHost', '0') in calls, extended)
                self.assertEqual(('hlCommander.pRelJoint', '0') in calls, extended)
                self.assertIn(('hlCommander.pRelMode',
                               '1' if mode == 'zero_velocity' else '0'), calls)

    def test_pi_joint_worker_is_required_at_final_prearm_check(self):
        controller = self.controller({
            'enabled': True, 'response_time_s': 0.14, 'mode': 'pi_joint'})
        controller.prepare_firmware_auto_brake()
        controller.cf = SimpleNamespace()
        with self.assertRaisesRegex(RuntimeError, 'not prewarmed'):
            controller.verify_firmware_auto_brake_ready()

    def test_final_prearm_refreshes_model_and_propagates_sync_failure(self):
        controller = self.controller({
            'enabled': True, 'response_time_s': .14, 'mode': 'pi_joint'})
        controller.prepare_firmware_auto_brake()
        planner = Mock()
        planner.status.return_value = {'ready': True}
        planner.sync_model.side_effect = RuntimeError('startup model sync failed')
        controller.cf = SimpleNamespace(param=Mock(), _post_release_pi_planner=planner)
        with patch('Interaction.firmware_parameter_confirmation.confirm_firmware_mode_parameters',
                   return_value={'hlCommander.pRelJoint': 1, 'hlCommander.pRelHost': 1}):
            with self.assertRaisesRegex(RuntimeError, 'model sync failed'):
                controller.verify_firmware_auto_brake_ready()
        planner.sync_model.assert_called_once_with()

    def test_single_marker_pointcloud_is_allowed_but_mixed_is_not(self):
        controller = self.controller({
            'enabled': True, 'response_time_s': 0.08,
        })
        controller.args.vicon_mode = 'pointcloud'
        controller.prepare_firmware_auto_brake()
        self.assertTrue(controller.firmware_auto_brake_enabled)

        controller.args.vicon_full_pose = True
        with self.assertRaisesRegex(ValueError, 'position-only'):
            controller.prepare_firmware_auto_brake()

        controller.args.vicon_full_pose = False
        controller.args.vicon_mode = 'mixed'
        with self.assertRaisesRegex(ValueError, 'position-only'):
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
