import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from Interaction.braking_response_calibration import PlanarBrakingCalibration
from Interaction.position_capture_calibration import PositionCaptureCalibration
from Interaction.interactions import (
    attitude_to_world_acceleration,
    calibrated_force_render_attitude_limit,
    calibration_state_dropout_tolerated,
    calibration_state_group_skew_tolerated,
    constrain_predictive_coast_render_mode,
    GuidedTouchProtocol,
    InitialContactArmingGate,
    InteractionsControl,
    StaleLocalizationError,
    TranslationControlHandoff,
    VirtualObjectPlanarMotion,
    coast_braking_attitude,
    coast_target_braking_attitude,
    force_inertia_attitude,
    heavy_inertia_attitude,
    inertia_command_mode,
    inertia_position_target,
    integrate_rate_limited_leveling_velocity_delta,
    kinetic_energy_velocity,
    potentiometer_release_direction,
    predict_delayed_zero_crossing,
    reconstruct_delayed_first_order_acceleration,
    release_dataset_world_y_directions,
    release_candidate_sensor_stale_watchdog,
    release_tail_neutralization_attitude,
    release_coast_initial_velocity,
    reset_pid_integrators_without_ack,
    resolve_release_mode,
    resolve_wrench_nominal_target,
    select_inertia_render_mode,
    virtual_resistance_force,
    velocity_inertia_mass_class,
    world_acceleration_to_attitude,
    world_to_body_xy,
)


class InitialContactArmingGateTests(unittest.TestCase):
    def test_requires_continuous_low_xy_speed_for_full_dwell(self):
        gate = InitialContactArmingGate(
            max_xy_speed_m_s=0.03,
            stationary_dwell_s=0.5,
        )

        self.assertFalse(gate.update([0.02, 0.0, 0.0], 1.0))
        self.assertFalse(gate.update([0.02, 0.0, 0.0], 1.3))
        self.assertFalse(gate.update([0.03, 0.0, 0.0], 1.4))
        self.assertEqual(gate.stationary_elapsed_s, 0.0)
        self.assertFalse(gate.update([0.0, 0.02, 0.0], 2.0))
        self.assertFalse(gate.update([0.0, 0.02, 0.0], 2.49))
        self.assertTrue(gate.update([0.0, 0.02, 0.0], 2.5))
        self.assertTrue(gate.armed)

    def test_disabled_gate_is_immediately_armed(self):
        gate = InitialContactArmingGate(enabled=False)

        self.assertTrue(gate.armed)
        self.assertFalse(gate.update([1.0, 0.0, 0.0], 0.0))

    def test_reset_requires_a_new_stationary_dwell(self):
        gate = InitialContactArmingGate(
            max_xy_speed_m_s=0.03,
            stationary_dwell_s=0.5,
        )
        gate.update([0.0, 0.0, 0.0], 1.0)
        self.assertTrue(gate.update([0.0, 0.0, 0.0], 1.5))

        gate.reset(after_interaction=True)

        self.assertFalse(gate.armed)
        self.assertFalse(gate.update([0.0, 0.0, 0.0], 2.0))
        self.assertTrue(gate.update([0.0, 0.0, 0.0], 2.5))

    def test_post_interaction_stationary_dwell_can_be_skipped(self):
        gate = InitialContactArmingGate(
            max_xy_speed_m_s=0.03,
            stationary_dwell_s=0.5,
            apply_after_each_interaction=False,
        )
        gate.update([0.0, 0.0, 0.0], 1.0)
        self.assertTrue(gate.update([0.0, 0.0, 0.0], 1.5))

        gate.reset(after_interaction=True)

        self.assertTrue(gate.armed)
        self.assertFalse(gate.update([1.0, 0.0, 0.0], 2.0))


class ReleaseModeTests(unittest.TestCase):
    def test_release_dataset_snaps_task_axis_but_preserves_real_sensor_bias(self):
        measured = np.array([0.10, np.sqrt(1.0-0.10**2)])

        task_direction, measured_direction = (
            release_dataset_world_y_directions(
                [0.02, np.sqrt(1.0-0.02**2)], measured,
            )
        )

        np.testing.assert_allclose(task_direction, [0.0, 1.0])
        np.testing.assert_allclose(measured_direction, measured)

    def test_release_dataset_rejects_diagonal_task_or_measured_axis(self):
        diagonal = np.sqrt(0.5)
        for task_axis, measured_axis in (
            ([diagonal, diagonal], [0.0, 1.0]),
            ([0.0, 1.0], [diagonal, diagonal]),
            ([0.0, 1.0], [0.0, -1.0]),
        ):
            with self.subTest(
                    task_axis=task_axis, measured_axis=measured_axis):
                task_direction, measured_direction = (
                    release_dataset_world_y_directions(
                        task_axis, measured_axis,
                    )
                )
                self.assertIsNone(task_direction)
                self.assertIsNotNone(measured_direction)

    def test_mpc_bootstrap_treats_sensor_as_axis_line_for_opposite_release(self):
        task_direction, measured_direction = (
            release_dataset_world_y_directions(
                [0.0, -1.0],
                [0.0, 1.0],
                sensor_axis_is_unsigned=True,
            )
        )

        np.testing.assert_allclose(task_direction, [0.0, -1.0])
        np.testing.assert_allclose(measured_direction, [0.0, -1.0])

    def test_calibration_target_override_does_not_move_interaction_target(self):
        mission_target = [0.0, -1.0, 1.0, 0.0]
        config = {'calibration_nominal_position': [0.0, 0.0, 1.0]}

        self.assertEqual(
            resolve_wrench_nominal_target(
                mission_target, config, calibration_mode=True
            )[:3],
            [0.0, 0.0, 1.0],
        )
        self.assertEqual(
            resolve_wrench_nominal_target(
                mission_target, config, calibration_mode=False
            )[:3],
            [0.0, -1.0, 1.0],
        )
        self.assertEqual(mission_target, [0.0, -1.0, 1.0, 0.0])

    def test_calibration_target_override_requires_finite_xyz(self):
        with self.assertRaisesRegex(ValueError, 'finite XYZ'):
            resolve_wrench_nominal_target(
                [0.0, -1.0, 1.0],
                {'calibration_nominal_position': [0.0, np.nan, 1.0]},
                calibration_mode=True,
            )

    def test_calibration_ignores_potentiometer_release_dependency(self):
        self.assertEqual(
            resolve_release_mode(
                'potentiometer_coast',
                force_sensor_available=False,
                calibration_mode=True,
            ),
            'observer_brake',
        )

    def test_normal_potentiometer_release_still_requires_sensor(self):
        with self.assertRaisesRegex(ValueError, 'requires --sense'):
            resolve_release_mode(
                'potentiometer_coast',
                force_sensor_available=False,
                calibration_mode=False,
            )

    def test_release_candidate_sensor_stale_watchdog_is_bounded(self):
        stale_since, timed_out = release_candidate_sensor_stale_watchdog(
            True, False, 10.0, None, 0.25
        )
        self.assertEqual(stale_since, 10.0)
        self.assertFalse(timed_out)

        stale_since, timed_out = release_candidate_sensor_stale_watchdog(
            True, False, 10.249, stale_since, 0.25
        )
        self.assertFalse(timed_out)
        stale_since, timed_out = release_candidate_sensor_stale_watchdog(
            True, False, 10.25, stale_since, 0.25
        )
        self.assertTrue(timed_out)

        self.assertEqual(
            release_candidate_sensor_stale_watchdog(
                True, True, 10.30, stale_since, 0.25
            ),
            (None, False),
        )

    def test_release_candidate_sensor_watchdog_rejects_invalid_timeout(self):
        with self.assertRaisesRegex(ValueError, 'timeout must be positive'):
            release_candidate_sensor_stale_watchdog(
                True, False, 1.0, None, 0.0
            )


class PotentiometerReleaseDirectionTests(unittest.TestCase):
    def test_force_axis_wins_over_transient_velocity(self):
        direction, source = potentiometer_release_direction(
            [0.0, 1.2, 0.1], [0.04, 0.04, 0.0]
        )

        np.testing.assert_allclose(direction, [0.0, 1.0, 0.0])
        self.assertEqual(source, 'potentiometer_force_world')

    def test_velocity_is_used_only_when_force_has_no_planar_direction(self):
        direction, source = potentiometer_release_direction(
            [0.0, 0.0, 0.2], [-3.0, 4.0, 0.0]
        )

        np.testing.assert_allclose(direction, [-0.6, 0.8, 0.0])
        self.assertEqual(source, 'measured_velocity_fallback')

    def test_release_candidate_cancels_predicted_counter_tilt_tail(self):
        tracking = release_tail_neutralization_attitude(
            current_velocity_xy=[0.0, 0.10],
            brake_direction_xy=[0.0, 1.0],
            yaw_deg=0.0,
            response_delay_s=0.12,
            response_time_constant_s=0.08,
            acceleration_scale=1.0,
            terminal_speed_margin_m_s=0.03,
            command_hold_s=0.02,
            measured_acceleration_xy=[0.0, -1.0],
            command_history=[(-1.0, np.array([0.0, -1.0]))],
            timestamp=0.0,
            future_command_started_at=0.0,
        )

        self.assertEqual(
            tracking['action'], 'canceling_predicted_reverse_tail'
        )
        self.assertLess(
            tracking['predicted_level_terminal_speed_m_s'], 0.0
        )
        self.assertGreater(
            tracking['tail_cancellation_acceleration_m_s2'], 0.0
        )
        self.assertGreater(tracking['applied_acceleration_m_s2'][1], 0.0)

    def test_release_candidate_levels_when_tail_cannot_reverse_motion(self):
        tracking = release_tail_neutralization_attitude(
            current_velocity_xy=[0.0, 0.30],
            brake_direction_xy=[0.0, 1.0],
            yaw_deg=0.0,
            measured_acceleration_xy=[0.0, 0.0],
            command_history=[(-1.0, np.zeros(2))],
            timestamp=0.0,
        )

        self.assertEqual(tracking['action'], 'leveling_release_candidate')
        self.assertEqual(
            tracking['tail_cancellation_acceleration_m_s2'], 0.0
        )
        self.assertEqual(tracking['roll_deg'], 0.0)
        self.assertEqual(tracking['pitch_deg'], 0.0)

    def test_release_candidate_tail_cancellation_is_sign_symmetric(self):
        results = []
        for tail_sign in (-1.0, 1.0):
            results.append(release_tail_neutralization_attitude(
                current_velocity_xy=[0.0, 0.0],
                brake_direction_xy=[0.0, 1.0],
                yaw_deg=0.0,
                response_delay_s=0.12,
                response_time_constant_s=0.08,
                command_hold_s=0.02,
                measured_acceleration_xy=[0.0, tail_sign],
                command_history=[(
                    -1.0, np.array([0.0, tail_sign])
                )],
                timestamp=0.0,
                future_command_started_at=0.0,
            ))

        reverse_tail, forward_tail = results
        self.assertEqual(
            reverse_tail['action'], 'canceling_predicted_reverse_tail'
        )
        self.assertEqual(
            forward_tail['action'], 'canceling_predicted_forward_tail'
        )
        np.testing.assert_allclose(
            reverse_tail['applied_acceleration_m_s2'],
            -forward_tail['applied_acceleration_m_s2'],
        )
        self.assertAlmostEqual(
            reverse_tail['tail_cancellation_acceleration_m_s2'],
            forward_tail['tail_cancellation_acceleration_m_s2'],
        )

    def test_release_candidate_cancels_forward_tail_after_reversal(self):
        tracking = release_tail_neutralization_attitude(
            current_velocity_xy=[0.0, -0.01],
            brake_direction_xy=[0.0, 1.0],
            yaw_deg=0.0,
            response_delay_s=0.12,
            response_time_constant_s=0.08,
            command_hold_s=0.02,
            measured_acceleration_xy=[0.0, 1.0],
            command_history=[(-1.0, np.array([0.0, 1.0]))],
            timestamp=0.0,
            future_command_started_at=0.0,
        )

        self.assertGreater(
            tracking['predicted_level_terminal_speed_m_s'], 0.0
        )
        self.assertEqual(
            tracking['action'], 'canceling_predicted_forward_tail'
        )
        self.assertLess(tracking['applied_acceleration_m_s2'][1], 0.0)

    def test_release_candidate_still_only_cancels_forward_growing_tail(self):
        for world_y_sign in (-1.0, 1.0):
            with self.subTest(world_y_sign=world_y_sign):
                direction = np.array([0.0, world_y_sign])
                tracking = release_tail_neutralization_attitude(
                    current_velocity_xy=0.60 * direction,
                    brake_direction_xy=direction,
                    yaw_deg=0.0,
                    response_delay_s=0.12,
                    response_time_constant_s=0.08,
                    command_hold_s=0.02,
                    measured_acceleration_xy=direction,
                    command_history=[(-1.0, direction.copy())],
                    timestamp=0.0,
                    future_command_started_at=0.0,
                )

                self.assertGreater(
                    tracking['predicted_level_terminal_speed_m_s'],
                    tracking['forward_speed_m_s'],
                )
                self.assertEqual(
                    tracking['action'], 'canceling_predicted_forward_tail'
                )
                # A candidate does not yet own the stop target: its pulse
                # removes only extra queued speed, not the current motion.
                self.assertAlmostEqual(
                    tracking['target_terminal_speed_m_s'], 0.60
                )
                self.assertGreater(
                    tracking['tail_cancellation_acceleration_m_s2'], 0.0
                )
                self.assertLess(
                    float(tracking['applied_acceleration_m_s2'] @ direction),
                    0.0,
                )

    def test_delayed_rollout_is_invariant_at_command_boundary(self):
        history = [
            (-1.0, np.zeros(2)),
            (0.0, np.array([0.0, -4.545454545454545])),
        ]
        final_speeds = []
        for step_s in (0.02, 0.01, 0.005, 0.001):
            rollout = predict_delayed_zero_crossing(
                current_velocity_xy=[0.0, 0.15],
                measured_acceleration_xy=[0.0, 0.0],
                motion_direction_xy=[0.0, 1.0],
                command_history=history,
                timestamp=0.02,
                response_delay_s=0.12,
                response_time_constant_s=0.08,
                acceleration_scale=1.10,
                future_command_started_at=0.02,
                continue_after_crossing=True,
                step_s=step_s,
            )
            final_speeds.append(rollout['final_speed_m_s'])

        for final_speed in final_speeds:
            self.assertAlmostEqual(final_speed, 0.05, places=8)

    def test_zero_tau_rollout_has_no_half_step_command_impulse(self):
        final_speeds = []
        for step_s in (0.02, 0.01, 0.005):
            rollout = predict_delayed_zero_crossing(
                current_velocity_xy=[0.0, 0.63],
                measured_acceleration_xy=[0.0, -5.0],
                motion_direction_xy=[0.0, 1.0],
                command_history=[(-1.0, np.array([0.0, -5.0]))],
                timestamp=0.0,
                response_delay_s=0.12,
                response_time_constant_s=0.0,
                acceleration_scale=1.0,
                future_command_acceleration_xy=[0.0, 0.0],
                future_command_started_at=0.0,
                continue_after_crossing=True,
                step_s=step_s,
            )
            final_speeds.append(rollout['final_speed_m_s'])

        for final_speed in final_speeds:
            self.assertAlmostEqual(final_speed, 0.03, places=8)

    def test_rollout_detects_zero_crossing_between_positive_endpoints(self):
        rollout = predict_delayed_zero_crossing(
            current_velocity_xy=[0.0, 0.014],
            measured_acceleration_xy=[0.0, -5.0],
            motion_direction_xy=[0.0, 1.0],
            command_history=[],
            timestamp=0.0,
            response_delay_s=0.0,
            response_time_constant_s=0.01,
            acceleration_scale=1.0,
            future_command_acceleration_xy=[0.0, 5.0],
            future_command_started_at=0.0,
            step_s=0.01,
            horizon_s=0.02,
        )

        self.assertTrue(rollout['crossed'])
        self.assertGreater(rollout['time_s'], 0.0)
        self.assertLess(rollout['time_s'], 0.01 * np.log(2.0))

    def test_release_candidate_rolls_out_tail_after_velocity_reversed(self):
        tracking = release_tail_neutralization_attitude(
            current_velocity_xy=[0.0, -0.01],
            brake_direction_xy=[0.0, 1.0],
            yaw_deg=0.0,
            response_delay_s=0.12,
            response_time_constant_s=0.08,
            acceleration_scale=1.0,
            terminal_speed_margin_m_s=0.03,
            command_hold_s=0.02,
            measured_acceleration_xy=[0.0, -1.0],
            command_history=[(-1.0, np.array([0.0, -1.0]))],
            timestamp=0.0,
        )

        self.assertLess(
            tracking['predicted_level_terminal_speed_m_s'], -0.01
        )
        self.assertGreater(
            tracking['tail_cancellation_acceleration_m_s2'], 0.0
        )

    def test_quick_candidate_does_not_extend_first_command_backward(self):
        tracking = release_tail_neutralization_attitude(
            current_velocity_xy=[0.0, 0.10],
            brake_direction_xy=[0.0, 1.0],
            yaw_deg=0.0,
            response_delay_s=0.12,
            response_time_constant_s=0.08,
            acceleration_scale=1.0,
            terminal_speed_margin_m_s=0.03,
            command_hold_s=0.02,
            measured_acceleration_xy=[0.0, 0.0],
            command_history=[(0.0, np.array([0.0, -1.0]))],
            timestamp=0.03,
            future_command_started_at=0.03,
        )

        self.assertAlmostEqual(
            tracking['predicted_level_terminal_speed_m_s'], 0.07, places=8
        )
        self.assertEqual(tracking['action'], 'leveling_release_candidate')


class CalibrationStateDropoutTests(unittest.TestCase):
    def test_calibration_tolerates_brief_stale_state(self):
        self.assertTrue(calibration_state_dropout_tolerated(
            0.109, 0.100, 0.250, calibration_mode=True
        ))

    def test_active_interaction_keeps_strict_state_age_limit(self):
        self.assertFalse(calibration_state_dropout_tolerated(
            0.109, 0.100, 0.250, calibration_mode=False
        ))

    def test_calibration_rejects_sustained_state_dropout(self):
        self.assertFalse(calibration_state_dropout_tolerated(
            0.251, 0.100, 0.250, calibration_mode=True
        ))

    def test_calibration_tolerates_brief_state_group_skew(self):
        self.assertTrue(calibration_state_group_skew_tolerated(
            0.080, 0.030, 0.004, 0.250, calibration_mode=True
        ))

    def test_active_interaction_keeps_strict_group_skew_limit(self):
        self.assertFalse(calibration_state_group_skew_tolerated(
            0.080, 0.030, 0.004, 0.250, calibration_mode=False
        ))

    def test_planar_attitude_trial_never_resumes_after_group_skew(self):
        self.assertFalse(calibration_state_group_skew_tolerated(
            0.080, 0.030, 0.004, 0.250,
            calibration_mode=True,
            planar_attitude_active=True,
        ))

    def test_calibration_rejects_persistent_or_excessive_group_skew(self):
        self.assertFalse(calibration_state_group_skew_tolerated(
            0.080, 0.030, 0.251, 0.250, calibration_mode=True
        ))
        self.assertFalse(calibration_state_group_skew_tolerated(
            0.251, 0.030, 0.004, 0.250, calibration_mode=True
        ))

    def test_group_skew_policy_rejects_nonfinite_values(self):
        with self.assertRaises(ValueError):
            calibration_state_group_skew_tolerated(
                np.nan, 0.030, 0.004, 0.250,
                calibration_mode=True,
            )

    def test_force_render_tilt_stays_inside_calibration_envelope(self):
        self.assertEqual(
            calibrated_force_render_attitude_limit(20.0, 8.0), 8.0
        )
        self.assertEqual(
            calibrated_force_render_attitude_limit(5.0, 8.0), 5.0
        )


class FakeHandoffCF:
    """Single simulated aircraft with the firmware HLC reply interface."""
    def __init__(self):
        self.callbacks = []
        self.platform = SimpleNamespace(get_protocol_version=lambda: 8)

    def add_port_callback(self, port, callback):
        self.callbacks.append(callback)

    def remove_port_callback(self, port, callback):
        self.callbacks.remove(callback)

    def acknowledge_go_to(self):
        for callback in list(self.callbacks):
            callback(SimpleNamespace(port=8, channel=0, data=bytes([12, 0, 0, 0])))


class FakeCommander:
    _default_cf = FakeHandoffCF()

    def __init__(self):
        self.calls = []
        self._cf = self._default_cf

    def go_to(self, *args, **kwargs):
        self.calls.append(('go_to', args, kwargs))
        self._cf.acknowledge_go_to()

    def send_position_setpoint(self, *args):
        self.calls.append(('position', args, {}))

    def send_zdistance_setpoint(self, *args):
        self.calls.append(('zdistance', args, {}))

    def send_hover_setpoint(self, *args):
        self.calls.append(('hover', args, {}))

    def send_velocity_world_setpoint(self, *args):
        self.calls.append(('velocity_world', args, {}))

    def send_notify_setpoint_stop(self, *args):
        self.calls.append(('stop', args, {}))


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


class VelocityInertiaRenderingTests(unittest.TestCase):
    def test_mass_classification_and_render_mode_aliases(self):
        self.assertEqual(velocity_inertia_mass_class(0.17, 0.05), 'light')
        self.assertEqual(velocity_inertia_mass_class(0.17, 0.17), 'matched')
        self.assertEqual(velocity_inertia_mass_class(0.17, 0.50), 'heavy')

        self.assertEqual(inertia_command_mode('light', 'position'), 'position')
        self.assertEqual(inertia_command_mode('light', 'velocity'), 'velocity')
        self.assertEqual(inertia_command_mode('heavy', 'position'), 'position')
        self.assertEqual(inertia_command_mode('heavy', 'velocity'), 'velocity')
        self.assertEqual(
            inertia_command_mode('heavy', 'orientation'), 'orientation'
        )
        self.assertEqual(
            inertia_command_mode('matched', 'orientation'), 'orientation'
        )
        self.assertEqual(
            inertia_command_mode('light', 'orientation'), 'orientation'
        )

    def test_equal_energy_mapping_uses_square_root_mass_ratio(self):
        velocity, raw_gain, applied_gain, saturated = kinetic_energy_velocity(
            [0.20, 0.0, 0.0], current_mass=0.17, virtual_mass=0.085,
        )
        expected_gain = np.sqrt(2.0)
        self.assertAlmostEqual(raw_gain, expected_gain)
        self.assertAlmostEqual(applied_gain, expected_gain)
        self.assertFalse(saturated)
        np.testing.assert_allclose(velocity, [0.20 * expected_gain, 0.0, 0.0])

        heavy_velocity, raw_gain, applied_gain, saturated = (
            kinetic_energy_velocity(
                [0.20, 0.0, 0.0], current_mass=0.17, virtual_mass=0.68,
            )
        )
        self.assertEqual(raw_gain, 0.5)
        self.assertEqual(applied_gain, 0.5)
        self.assertFalse(saturated)
        np.testing.assert_allclose(heavy_velocity, [0.10, 0.0, 0.0])

    def test_energy_gain_is_capped_for_extreme_light_objects(self):
        velocity, raw_gain, applied_gain, saturated = kinetic_energy_velocity(
            [0.20, 0.0, 0.0], current_mass=0.17, virtual_mass=0.001,
            max_energy_gain=4.0,
        )
        self.assertAlmostEqual(raw_gain, np.sqrt(170.0))
        self.assertEqual(applied_gain, 4.0)
        self.assertTrue(saturated)
        np.testing.assert_allclose(velocity, [0.80, 0.0, 0.0])

    def test_position_target_is_anchored_at_contact_origin(self):
        origin = [1.0, 2.0, 1.0]
        measured = [1.10, 1.80, 1.0]

        heavy_target = inertia_position_target(origin, measured, 0.25)
        np.testing.assert_allclose(heavy_target, [1.025, 1.95, 1.0])

        light_target = inertia_position_target(origin, measured, 2.0)
        np.testing.assert_allclose(light_target, [1.20, 1.60, 1.0])

    def test_velocity_command_is_rotated_from_world_to_body(self):
        np.testing.assert_allclose(world_to_body_xy([1.0, 0.0], 0.0), [1, 0])
        np.testing.assert_allclose(
            world_to_body_xy([1.0, 0.0], 90.0), [0, -1], atol=1e-12
        )

    def test_pid_integrator_reset_prefers_no_ack_raw_writes(self):
        class RawParameters:
            def __init__(self):
                self.calls = []

            def set_value_raw(self, name, parameter_type, value):
                self.calls.append((name, parameter_type, value))

        parameters = RawParameters()
        method = reset_pid_integrators_without_ack(
            SimpleNamespace(param=parameters),
            ('posCtlPid.resetI', 'velCtlPid.resetI'),
        )

        self.assertEqual(method, 'raw_by_name_no_ack')
        self.assertEqual(parameters.calls, [
            ('posCtlPid.resetI', 0x08, 1),
            ('velCtlPid.resetI', 0x08, 1),
        ])

    def test_pid_integrator_reset_keeps_older_cflib_fallback(self):
        class LegacyParameters:
            def __init__(self):
                self.calls = []

            def set_value(self, name, value):
                self.calls.append((name, value))

        parameters = LegacyParameters()
        method = reset_pid_integrators_without_ack(
            SimpleNamespace(param=parameters), ('velCtlPid.resetI',)
        )

        self.assertEqual(method, 'acknowledged_fallback')
        self.assertEqual(parameters.calls, [('velCtlPid.resetI', '1')])

    def test_heavy_mass_generates_and_limits_attitude_feedback(self):
        pitch, roll = heavy_inertia_attitude(
            [0.02, 0.0], dt=0.01, yaw_deg=0.0,
            current_mass=0.17, virtual_mass=0.34, max_attitude_deg=20.0,
        )
        self.assertGreater(pitch, 0.0)
        self.assertEqual(roll, 0.0)

        pitch, roll = heavy_inertia_attitude(
            [1.0, 1.0], dt=0.01, yaw_deg=0.0,
            current_mass=0.17, virtual_mass=2.0, max_attitude_deg=8.0,
        )
        self.assertEqual(pitch, 8.0)
        self.assertEqual(roll, 8.0)

    def test_estimated_force_generates_mass_scaled_counter_tilt(self):
        pitch, roll, raw_tilt, saturated = force_inertia_attitude(
            [0.5, 0.0], yaw_deg=0.0,
            current_mass=0.17, virtual_mass=2.0,
            max_attitude_deg=5.0,
        )
        self.assertEqual(pitch, 5.0)
        self.assertEqual(roll, 0.0)
        self.assertGreater(raw_tilt, 5.0)
        self.assertTrue(saturated)

    def test_attitude_limits_reject_tangent_sign_flip_angles(self):
        calls = [
            lambda angle: coast_braking_attitude(
                [0.0, 0.1], [0.0, 1.0], 0.0,
                max_attitude_deg=angle,
            ),
            lambda angle: release_tail_neutralization_attitude(
                [0.0, 0.1], [0.0, 1.0], 0.0,
                max_attitude_deg=angle,
            ),
            lambda angle: coast_target_braking_attitude(
                [0.0, 0.0], [0.0, 0.1], [0.0, 0.1],
                [0.0, 1.0], 0.0, max_attitude_deg=angle,
            ),
            lambda angle: heavy_inertia_attitude(
                [0.01, 0.0], 0.01, 0.0, 0.17, 0.34,
                max_attitude_deg=angle,
            ),
            lambda angle: force_inertia_attitude(
                [0.1, 0.0], 0.0, 0.17, 0.34,
                max_attitude_deg=angle,
            ),
            lambda angle: TranslationControlHandoff(
                [0.0, 0.0, 1.0], 0.0, False,
                brake_max_attitude_deg=angle,
            ),
        ]
        for angle in (90.0, 100.0):
            for call in calls:
                with self.subTest(angle=angle, call=call):
                    with self.assertRaisesRegex(ValueError, 'between 0 and 90'):
                        call(angle)

        pitch, roll, _raw_tilt, saturated = force_inertia_attitude(
            [0.0, 0.05], yaw_deg=0.0,
            current_mass=0.17, virtual_mass=2.0,
            max_attitude_deg=5.0,
        )
        self.assertEqual(pitch, 0.0)
        self.assertGreater(roll, 0.0)
        self.assertFalse(saturated)

    def test_virtual_friction_and_air_drag_are_independently_configurable(self):
        resistance, friction, drag = virtual_resistance_force(
            [2.0, 0.0],
            virtual_mass=2.0,
            kinetic_friction_coefficient=0.10,
            drag_coefficient=1.0,
            frontal_area=0.020,
            air_density=1.20,
            friction_min_speed_m_s=0.02,
        )
        self.assertAlmostEqual(friction, 0.10 * 2.0 * 9.81)
        self.assertAlmostEqual(drag, 0.5 * 1.20 * 1.0 * 0.020 * 4.0)
        np.testing.assert_allclose(resistance, [friction + drag, 0.0])

        no_friction, friction, drag = virtual_resistance_force(
            [0.01, 0.0],
            virtual_mass=2.0,
            kinetic_friction_coefficient=0.10,
            drag_coefficient=0.0,
            friction_min_speed_m_s=0.02,
        )
        np.testing.assert_allclose(no_friction, [0.0, 0.0])
        self.assertEqual(friction, 0.0)
        self.assertEqual(drag, 0.0)

        static_resistance, friction, drag = virtual_resistance_force(
            [0.0, 0.0],
            virtual_mass=0.17,
            kinetic_friction_coefficient=0.30,
            static_friction_coefficient=0.50,
            external_force_xy=[0.20, 0.0],
        )
        np.testing.assert_allclose(static_resistance, [0.20, 0.0])
        self.assertEqual(friction, 0.20)
        self.assertEqual(drag, 0.0)

    def test_virtual_resistance_adds_mass_scaled_counter_tilt(self):
        virtual_friction, _, _ = virtual_resistance_force(
            [0.20, 0.0],
            virtual_mass=2.0,
            kinetic_friction_coefficient=0.10,
            drag_coefficient=0.0,
        )
        pitch, roll, raw_tilt, saturated = force_inertia_attitude(
            [0.0, 0.0],
            yaw_deg=0.0,
            current_mass=0.17,
            virtual_mass=2.0,
            max_attitude_deg=20.0,
            virtual_resistance_force_xy=virtual_friction,
        )
        self.assertAlmostEqual(raw_tilt, np.degrees(np.arctan(0.10)))
        self.assertAlmostEqual(pitch, raw_tilt)
        self.assertEqual(roll, 0.0)
        self.assertFalse(saturated)

    def test_matched_mass_renders_kinetic_friction_without_inertia_feedback(self):
        virtual_friction, friction, drag = virtual_resistance_force(
            [0.20, 0.0],
            virtual_mass=0.17,
            kinetic_friction_coefficient=0.30,
            drag_coefficient=0.0,
        )
        pitch, roll, raw_tilt, saturated = force_inertia_attitude(
            [0.50, 0.0],
            yaw_deg=0.0,
            current_mass=0.17,
            virtual_mass=0.17,
            max_attitude_deg=20.0,
            virtual_resistance_force_xy=virtual_friction,
        )
        self.assertAlmostEqual(friction, 0.30 * 0.17 * 9.81)
        self.assertEqual(drag, 0.0)
        self.assertAlmostEqual(raw_tilt, np.degrees(np.arctan(0.30)))
        self.assertAlmostEqual(pitch, raw_tilt)
        self.assertEqual(roll, 0.0)
        self.assertFalse(saturated)

    def test_faster_virtual_motion_forces_position_rendering(self):
        selection = select_inertia_render_mode(
            [0.20, 0.0], [0.10, 0.0],
            current_mass=0.17, virtual_mass=0.05,
            preferred_mode='orientation',
        )
        self.assertEqual(selection['relation'], 'faster')
        self.assertEqual(selection['mode'], 'position')

    def test_active_predictive_coast_overrides_opposite_drift_to_orientation(self):
        force = np.array([0.0, 0.08])
        velocity = np.array([0.0, -0.025])
        resistance, _, _ = virtual_resistance_force(
            velocity,
            virtual_mass=0.17,
            kinetic_friction_coefficient=0.10,
        )
        selection = select_inertia_render_mode(
            force,
            velocity,
            current_mass=0.17,
            virtual_mass=0.17,
            preferred_mode='orientation',
            virtual_resistance_force_xy=resistance,
        )
        self.assertEqual(selection['mode'], 'position')

        constrained = constrain_predictive_coast_render_mode(
            selection,
            release_mode='potentiometer_coast',
            shadow_mode=False,
        )

        self.assertEqual(constrained['mode'], 'orientation')
        self.assertEqual(
            constrained['relation'],
            'calibrated_orientation_override:faster',
        )
        self.assertEqual(selection['mode'], 'position')

    def test_slower_virtual_motion_honors_render_priority(self):
        friction, _, _ = virtual_resistance_force(
            [0.10, 0.0], virtual_mass=0.17,
            kinetic_friction_coefficient=0.30,
        )
        orientation = select_inertia_render_mode(
            [0.20, 0.0], [0.10, 0.0],
            current_mass=0.17, virtual_mass=0.17,
            preferred_mode='orientation',
            virtual_resistance_force_xy=friction,
        )
        position = select_inertia_render_mode(
            [0.20, 0.0], [0.10, 0.0],
            current_mass=0.17, virtual_mass=0.17,
            preferred_mode='position',
            virtual_resistance_force_xy=friction,
        )
        self.assertEqual(orientation['relation'], 'slower_or_equal')
        self.assertEqual(orientation['mode'], 'orientation')
        self.assertEqual(position['mode'], 'position')

    def test_position_motion_coasts_under_friction_without_reversing(self):
        motion = VirtualObjectPlanarMotion(
            mass=0.17,
            max_velocity_m_s=0.60,
            max_offset_xy=[0.5, 0.5],
            kinetic_friction_coefficient=0.30,
        )
        motion.reset([0.0, 0.0], [0.20, 0.0])
        positions = []
        for _ in range(20):
            state = motion.step([0.0, 0.0], 0.01)
            positions.append(state['position'][0])
        self.assertGreater(positions[-1], 0.0)
        self.assertTrue(all(
            later >= earlier
            for earlier, later in zip(positions, positions[1:])
        ))
        self.assertEqual(state['velocity'][0], 0.0)

    def test_virtual_stop_prediction_does_not_mutate_live_state(self):
        motion = VirtualObjectPlanarMotion(
            mass=0.17,
            max_velocity_m_s=0.60,
            max_offset_xy=[0.5, 0.5],
            kinetic_friction_coefficient=0.10,
        )
        motion.reset([0.0, 0.0], [0.60, 0.0])

        prediction = motion.predict_stop()

        self.assertTrue(prediction['stopped'])
        self.assertAlmostEqual(prediction['position'][0], 0.183, delta=0.01)
        np.testing.assert_allclose(motion.position, [0.0, 0.0])
        np.testing.assert_allclose(motion.velocity, [0.60, 0.0])

    def test_release_coast_combines_measured_velocity_and_last_force(self):
        velocity = release_coast_initial_velocity(
            measured_velocity=[0.10, 0.20, 0.05],
            last_force=[0.0, 0.17, 0.0],
            mass=0.17,
            force_memory_s=0.02,
            max_velocity_m_s=0.60,
        )

        np.testing.assert_allclose(velocity, [0.10, 0.22, 0.0])

    def test_coast_attitude_decelerates_when_actual_velocity_is_too_high(self):
        tracking = coast_braking_attitude(
            current_velocity_xy=[0.0, 0.30],
            brake_direction_xy=[0.0, 1.0],
            yaw_deg=0.0,
        )

        self.assertEqual(tracking['action'], 'decelerating')
        self.assertLess(tracking['applied_acceleration_m_s2'][1], 0.0)
        self.assertGreater(tracking['roll_deg'], 0.0)
        self.assertLessEqual(tracking['power_w_per_kg'], 0.0)

    def test_coast_attitude_brakes_reverse_motion_without_position_pull(self):
        tracking = coast_braking_attitude(
            current_velocity_xy=[0.0, -0.10],
            brake_direction_xy=[0.0, 1.0],
            yaw_deg=0.0,
        )

        self.assertEqual(tracking['action'], 'decelerating')
        self.assertGreater(tracking['applied_acceleration_m_s2'][1], 0.0)
        self.assertLess(tracking['roll_deg'], 0.0)
        self.assertLessEqual(tracking['power_w_per_kg'], 0.0)

    def test_target_braking_reserves_distance_for_attitude_delay(self):
        without_delay = coast_target_braking_attitude(
            current_position_xy=[0.0, 0.0],
            current_velocity_xy=[0.0, 0.60],
            target_position_xy=[0.0, 0.20],
            brake_direction_xy=[0.0, 1.0],
            yaw_deg=0.0,
            response_delay_s=0.0,
        )
        with_delay = coast_target_braking_attitude(
            current_position_xy=[0.0, 0.0],
            current_velocity_xy=[0.0, 0.60],
            target_position_xy=[0.0, 0.20],
            brake_direction_xy=[0.0, 1.0],
            yaw_deg=0.0,
            response_delay_s=0.12,
        )

        self.assertAlmostEqual(with_delay['delay_reserved_distance_m'], 0.072)
        self.assertGreater(
            with_delay['required_deceleration_m_s2'],
            without_delay['required_deceleration_m_s2'],
        )
        self.assertLessEqual(with_delay['power_w_per_kg'], 0.0)

    def test_target_braking_levels_early_when_measured_deceleration_will_stop(self):
        tracking = coast_target_braking_attitude(
            current_position_xy=[0.0, 0.10],
            current_velocity_xy=[0.0, 0.40],
            target_position_xy=[0.0, 0.18],
            brake_direction_xy=[0.0, 1.0],
            yaw_deg=0.0,
            response_delay_s=0.12,
            measured_acceleration_xy=[0.0, -4.0],
            max_acceleration_m_s2=5.0,
            max_attitude_deg=30.0,
        )

        self.assertEqual(tracking['required_deceleration_m_s2'], 0.0)
        self.assertEqual(
            tracking['predicted_forward_speed_after_delay_m_s'], 0.0
        )
        self.assertAlmostEqual(tracking['roll_deg'], 0.0)
        self.assertAlmostEqual(
            tracking['effective_acceleration_limit_m_s2'], 5.0
        )

    def test_target_braking_prioritizes_stopping_over_forward_tail(self):
        for world_y_sign in (-1.0, 1.0):
            with self.subTest(world_y_sign=world_y_sign):
                direction = np.array([0.0, world_y_sign])
                tracking = coast_target_braking_attitude(
                    current_position_xy=[0.0, 0.0],
                    current_velocity_xy=0.60 * direction,
                    target_position_xy=0.20 * direction,
                    brake_direction_xy=direction,
                    yaw_deg=0.0,
                    response_delay_s=0.12,
                    response_time_constant_s=0.08,
                    command_hold_s=0.02,
                    measured_acceleration_xy=direction,
                    command_history=[(-1.0, direction.copy())],
                    timestamp=0.0,
                    future_command_started_at=0.0,
                )

                self.assertGreater(
                    tracking['predicted_level_terminal_speed_m_s'],
                    tracking['forward_speed_m_s'],
                )
                self.assertEqual(tracking['action'], 'decelerating')
                self.assertFalse(tracking['target_passed'])
                self.assertEqual(
                    tracking['tail_cancellation_acceleration_m_s2'], 0.0
                )
                self.assertGreater(
                    tracking['required_deceleration_m_s2'], 0.0
                )
                self.assertLessEqual(
                    tracking['required_deceleration_m_s2'],
                    tracking['impulse_safe_deceleration_m_s2'],
                )
                self.assertLess(tracking['power_w_per_kg'], 0.0)
                self.assertLess(
                    float(tracking['applied_acceleration_m_s2'] @ direction),
                    0.0,
                )

    def test_target_passed_damps_forward_motion_despite_forward_tail(self):
        for world_y_sign in (-1.0, 1.0):
            for distance_past_target in (0.01, 0.50):
                with self.subTest(
                        world_y_sign=world_y_sign,
                        distance_past_target=distance_past_target):
                    direction = np.array([0.0, world_y_sign])
                    tracking = coast_target_braking_attitude(
                        current_position_xy=[0.0, 0.0],
                        current_velocity_xy=0.60 * direction,
                        target_position_xy=-distance_past_target * direction,
                        brake_direction_xy=direction,
                        yaw_deg=0.0,
                        response_delay_s=0.12,
                        response_time_constant_s=0.08,
                        command_hold_s=0.02,
                        measured_acceleration_xy=direction,
                        command_history=[(-1.0, direction.copy())],
                        timestamp=0.0,
                        future_command_started_at=0.0,
                    )

                    self.assertGreater(
                        tracking['predicted_level_terminal_speed_m_s'],
                        tracking['forward_speed_m_s'],
                    )
                    self.assertEqual(
                        tracking['action'],
                        'damping_forward_motion_after_target',
                    )
                    self.assertTrue(tracking['target_passed'])
                    self.assertEqual(
                        tracking['tail_cancellation_acceleration_m_s2'], 0.0
                    )
                    # Passing farther beyond the target must not add a
                    # position-pull term to the forward-velocity damping.
                    self.assertAlmostEqual(
                        tracking['required_deceleration_m_s2'], 2.5 * 0.60
                    )
                    np.testing.assert_allclose(
                        tracking['applied_acceleration_m_s2'],
                        -1.50 * direction,
                    )
                    self.assertLess(tracking['power_w_per_kg'], 0.0)

    def test_forward_tail_stop_controller_retains_one_frame_impulse_limit(self):
        for world_y_sign in (-1.0, 1.0):
            with self.subTest(world_y_sign=world_y_sign):
                direction = np.array([0.0, world_y_sign])
                tracking = coast_target_braking_attitude(
                    current_position_xy=[0.0, 0.0],
                    current_velocity_xy=0.04 * direction,
                    target_position_xy=0.001 * direction,
                    brake_direction_xy=direction,
                    yaw_deg=0.0,
                    response_delay_s=0.12,
                    response_time_constant_s=0.08,
                    terminal_speed_margin_m_s=0.03,
                    command_hold_s=0.02,
                    measured_acceleration_xy=0.05 * direction,
                    command_history=[(-1.0, 0.05 * direction)],
                    timestamp=0.0,
                    future_command_started_at=0.0,
                    max_acceleration_m_s2=5.0,
                    max_attitude_deg=30.0,
                )

                terminal_speed = tracking['predicted_level_terminal_speed_m_s']
                self.assertGreater(terminal_speed, 0.04)
                self.assertEqual(tracking['action'], 'decelerating')
                self.assertLess(
                    tracking['impulse_safe_deceleration_m_s2'],
                    tracking['effective_acceleration_limit_m_s2'],
                )
                self.assertAlmostEqual(
                    tracking['required_deceleration_m_s2'],
                    tracking['impulse_safe_deceleration_m_s2'],
                )
                acceleration = float(
                    tracking['applied_acceleration_m_s2'] @ direction
                )
                self.assertLess(acceleration, 0.0)
                self.assertAlmostEqual(
                    terminal_speed + acceleration * 0.02, 0.03
                )

    def test_target_braking_cancels_forward_rebound_after_small_reversal(self):
        tracking = coast_target_braking_attitude(
            current_position_xy=[0.0, 0.0],
            current_velocity_xy=[0.0, -0.01],
            target_position_xy=[0.0, 0.2],
            brake_direction_xy=[0.0, 1.0],
            yaw_deg=0.0,
            response_delay_s=0.12,
            response_time_constant_s=0.08,
            acceleration_scale=1.0,
            command_hold_s=0.02,
            measured_acceleration_xy=[0.0, 1.0],
            command_history=[(-1.0, np.array([0.0, 1.0]))],
            timestamp=0.0,
            future_command_started_at=0.0,
            max_acceleration_m_s2=5.0,
            max_attitude_deg=30.0,
        )

        self.assertGreater(
            tracking['predicted_level_terminal_speed_m_s'], 0.03
        )
        self.assertEqual(
            tracking['action'],
            'canceling_predicted_forward_tail',
        )
        self.assertLess(tracking['command_acceleration_m_s2'][1], 0.0)

    def test_target_braking_neutralizes_residual_tail_at_zero_speed(self):
        tracking = coast_target_braking_attitude(
            current_position_xy=[0.0, 0.0],
            current_velocity_xy=[0.0, 0.0],
            target_position_xy=[0.0, 0.2],
            brake_direction_xy=[0.0, 1.0],
            yaw_deg=0.0,
            response_delay_s=0.12,
            response_time_constant_s=0.08,
            acceleration_scale=1.0,
            command_hold_s=0.02,
            measured_acceleration_xy=[0.0, -1.0],
            command_history=[(-1.0, np.array([0.0, -1.0]))],
            timestamp=0.0,
            future_command_started_at=0.0,
            max_acceleration_m_s2=5.0,
            max_attitude_deg=30.0,
        )

        self.assertLess(
            tracking['predicted_level_terminal_speed_m_s'], 0.0
        )
        self.assertEqual(
            tracking['action'], 'canceling_predicted_reverse_tail'
        )
        self.assertGreater(tracking['command_acceleration_m_s2'][1], 0.0)

    def test_target_tail_cancellation_is_symmetric_at_zero_speed(self):
        results = []
        for tail_sign in (-1.0, 1.0):
            results.append(coast_target_braking_attitude(
                current_position_xy=[0.0, 0.0],
                current_velocity_xy=[0.0, 0.0],
                target_position_xy=[0.0, 0.2],
                brake_direction_xy=[0.0, 1.0],
                yaw_deg=0.0,
                response_delay_s=0.12,
                response_time_constant_s=0.08,
                command_hold_s=0.02,
                measured_acceleration_xy=[0.0, tail_sign],
                command_history=[(
                    -1.0, np.array([0.0, tail_sign])
                )],
                timestamp=0.0,
                future_command_started_at=0.0,
            ))

        negative_tail, positive_tail = results
        self.assertEqual(
            negative_tail['action'], 'canceling_predicted_reverse_tail'
        )
        self.assertEqual(
            positive_tail['action'], 'canceling_predicted_forward_tail'
        )
        self.assertAlmostEqual(
            negative_tail['tail_cancellation_signed_acceleration_m_s2'],
            -positive_tail['tail_cancellation_signed_acceleration_m_s2'],
        )
        np.testing.assert_allclose(
            negative_tail['command_acceleration_m_s2'],
            -positive_tail['command_acceleration_m_s2'],
        )

    def test_target_tail_only_removes_queue_motion_beyond_current_speed(self):
        excessive_reverse = coast_target_braking_attitude(
            current_position_xy=[0.0, 0.0],
            current_velocity_xy=[0.0, -0.01],
            target_position_xy=[0.0, 0.2],
            brake_direction_xy=[0.0, 1.0],
            yaw_deg=0.0,
            response_delay_s=0.12,
            response_time_constant_s=0.08,
            command_hold_s=0.02,
            measured_acceleration_xy=[0.0, -1.0],
            command_history=[(-1.0, np.array([0.0, -1.0]))],
            timestamp=0.0,
            future_command_started_at=0.0,
        )
        ordinary_reverse = coast_target_braking_attitude(
            current_position_xy=[0.0, 0.0],
            current_velocity_xy=[0.0, -0.01],
            target_position_xy=[0.0, 0.2],
            brake_direction_xy=[0.0, 1.0],
            yaw_deg=0.0,
            response_delay_s=0.0,
            response_time_constant_s=0.0,
            terminal_speed_margin_m_s=0.0,
            measured_acceleration_xy=[0.0, 0.0],
            command_history=[(-1.0, np.zeros(2))],
            timestamp=0.0,
            future_command_started_at=0.0,
        )

        self.assertEqual(
            excessive_reverse['action'], 'canceling_predicted_reverse_tail'
        )
        self.assertAlmostEqual(
            excessive_reverse['tail_terminal_target_speed_m_s'], -0.01
        )
        self.assertGreater(
            excessive_reverse['command_acceleration_m_s2'][1], 0.0
        )
        self.assertEqual(
            ordinary_reverse['action'], 'damping_reverse_motion'
        )
        self.assertEqual(
            ordinary_reverse['tail_cancellation_acceleration_m_s2'], 0.0
        )

    def test_coast_attitude_is_dissipative_for_planar_directions(self):
        for yaw_deg in (-135.0, -20.0, 0.0, 75.0, 170.0):
            for velocity in (
                [0.3, 0.4], [-0.3, 0.4], [0.3, -0.4], [-0.3, -0.4]
            ):
                tracking = coast_braking_attitude(
                    current_velocity_xy=velocity,
                    brake_direction_xy=[0.0, 1.0],
                    yaw_deg=yaw_deg,
                    max_acceleration_m_s2=1.2,
                )
                acceleration = tracking['applied_acceleration_m_s2']
                self.assertLessEqual(float(acceleration @ velocity), 1e-12)
                self.assertLessEqual(float(np.linalg.norm(acceleration)), 1.2)


class WrenchInteractionLoopTests(unittest.TestCase):
    def _run_automatic_mpc_loop_harness(
            self, *, stop_exception, state_mutator=None,
            event_callback=None, record_callback=None,
            bootstrap_overrides=None, handoff_overrides=None):
        class DeterministicClock:
            def __init__(self):
                self.wall_s = 1000.0
                self.monotonic_s = 500.0

            def time(self):
                return self.wall_s

            def monotonic(self):
                return self.monotonic_s

            def advance(self, duration_s):
                duration_s = max(float(duration_s), 0.0)
                self.wall_s += duration_s
                self.monotonic_s += duration_s

        context = SimpleNamespace(
            clock=DeterministicClock(),
            commander=None,
            events=[],
            timeline=[],
        )

        class RecordingCommander(FakeCommander):
            def send_position_setpoint(self, *args):
                super().send_position_setpoint(*args)
                context.timeline.append(('command', 'position', args))
                position_send_stall_s = float(getattr(
                    context, 'position_send_stall_s', 0.0
                ))
                context.position_send_stall_s = 0.0
                if position_send_stall_s > 0.0:
                    context.clock.advance(position_send_stall_s)

            def send_zdistance_setpoint(self, *args):
                super().send_zdistance_setpoint(*args)
                context.timeline.append(('command', 'zdistance', args))

        context.commander = RecordingCommander()

        class AutomaticMPCLogManager(FakeOnboardLogManager):
            def get_nearest_group_log_data(self, group_name, timestamp):
                packet, skew = super().get_nearest_group_log_data(
                    group_name, timestamp
                )
                if state_mutator is not None:
                    state_mutator(context, group_name, packet)
                return packet, skew

            def add_log_entry(self, group_name, entry, *args, **kwargs):
                super().add_log_entry(group_name, entry, *args, **kwargs)
                if record_callback is not None:
                    record_callback(
                        context, group_name, kwargs.get('name'), entry
                    )

        logs = AutomaticMPCLogManager(context.clock.time())
        context.logs = logs
        controller = InteractionsControl.__new__(InteractionsControl)
        context.controller = controller
        controller.drone_id = 'lb11'
        controller.log_manager = logs
        controller.ctrl_rate = 100
        controller.bounds = {
            'x_min': -1.0, 'x_max': 1.0,
            'y_min': -1.0, 'y_max': 1.0,
            'z_min': 0.3, 'z_max': 2.0,
        }
        controller.hl_commander = FakeCommander()
        controller.lo_commander = context.commander
        controller.force_sensor = None

        class RecordingParameters:
            def set_value_raw(self, name, parameter_type, value):
                context.timeline.append((
                    'integrator_reset', name, parameter_type, value,
                ))

        controller.cf = SimpleNamespace(param=RecordingParameters())

        def safe_sleep(duration_s):
            context.clock.advance(duration_s)
            logs.packet_time = context.clock.time()

        controller._safe_sleep = safe_sleep

        def log_event(name, data=None):
            payload = {} if data is None else data
            context.events.append((name, payload))
            context.timeline.append(('event', name, payload))
            logs.add_log_entry('events', payload, name=name)
            if event_callback is not None:
                event_callback(context, name, payload)

        controller._log_event = log_event
        contracts = {
            'positive_y': {
                'direction_label': 'positive_y',
                'direction_xy': [0.0, 1.0],
                'command_delay_s': 0.02,
                'model_fingerprint': 'test-positive-model',
                'state_dimension': 4,
            },
            'negative_y': {
                'direction_label': 'negative_y',
                'direction_xy': [0.0, -1.0],
                'command_delay_s': 0.02,
                'model_fingerprint': 'test-negative-model',
                'state_dimension': 4,
            },
        }

        def contract_for_direction(_contracts, direction_xy):
            return contracts[
                'positive_y' if direction_xy[1] > 0.0 else 'negative_y'
            ]

        bootstrap_config = {
            'enabled': True,
            'initial_speed_targets_m_s': [0.25],
            'speed_tolerance_m_s': 0.04,
            'repetitions_per_cell': 1,
            'max_release_speed_m_s': 0.40,
            'ready_dwell_s': 0.02,
            'level_warmup_min_s': 0.02,
            'prediction_step_s': 0.02,
            'max_acceleration_duration_s': 0.20,
            'max_maneuver_displacement_m': 0.20,
        }
        bootstrap_config.update(bootstrap_overrides or {})
        handoff_config = {
            'coast_attitude_response_delay_s': 0.02,
            'coast_velocity_braking_enabled': False,
            'coast_velocity_predictive_unwind_enabled': False,
        }
        handoff_config.update(handoff_overrides or {})
        config = {
            'state_source': 'onboard',
            'shadow_mode': False,
            'startup_bias_calibration_enabled': False,
            'initial_contact_arming': {'enabled': False},
            'detection': {
                'translation': {'enabled': False},
                'yaw': {'enabled': False},
            },
            'predictive_braking': {'enabled': False},
            'learning_velocity_mpc_shadow': {
                'enabled': False,
                'command_authority': False,
            },
            'mpc_bootstrap_calibration': bootstrap_config,
            'mpc_bootstrap_model_contracts': contracts,
            'control_handoff': handoff_config,
        }
        virtual_object = {
            'inertia_command': 'orientation',
            'force_rendering': {'enabled': False},
            'contact_detection': {'source': 'wrench_observer'},
            'release_behavior': {'mode': 'observer_brake'},
        }

        try:
            with patch(
                'Interaction.interactions.time.time',
                side_effect=context.clock.time,
            ), patch(
                'Interaction.interactions.time.monotonic',
                side_effect=context.clock.monotonic,
            ), patch(
                'Interaction.interactions.validate_mpc_bootstrap_model_contracts',
                return_value=contracts,
            ), patch(
                'Interaction.interactions.mpc_bootstrap_model_contract_for_direction',
                side_effect=contract_for_direction,
            ):
                controller.interaction_onboard_wrench_admittance(
                    duration=10.0,
                    nominal_position=[0.0, 0.0, 1.0],
                    nominal_yaw_deg=0.0,
                    config=config,
                    virtual_object_config=virtual_object,
                    mpc_calibration_mode=True,
                )
        except stop_exception as error:
            context.stop_exception = error
            return context
        self.fail('automatic MPC loop did not reach the requested stop point')

    def test_task_detection_method_selects_legacy_velocity(self):
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.drone_id = 'lb11'
        controller.pub_socket = None
        controller.lo_commander = FakeCommander()
        controller.mission = {
            'drones': {'lb11': {'target': [0.0, 0.0, 1.0]}},
            'Interaction': {'config': {
                'detection_method': 'velocity',
                'duration': 10,
                'delta_v': 0.2,
                'z': -1,
                'friction_coefficient': 0,
                'base_attitude': 1,
                'v_scalar': [10, 10, 5],
                'wrench_interaction': {'state_source': 'onboard'},
            }},
        }
        calls = []
        controller.interaction_translation_vel = lambda **kwargs: calls.append(
            ('velocity', kwargs)
        )
        controller.interaction_onboard_wrench_admittance = (
            lambda **kwargs: calls.append(('momentum_impulse', kwargs))
        )

        controller._run_translation()

        self.assertEqual([name for name, _ in calls], ['velocity'])
        self.assertEqual(calls[0][1]['vel_threshold'], 0.2)

    def test_task_detection_method_selects_momentum_impulse(self):
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.drone_id = 'lb11'
        controller.lo_commander = FakeCommander()
        wrench_config = {'state_source': 'onboard'}
        controller.mission = {
            'drones': {'lb11': {'target': [0.1, 0.2, 1.0, 7.0]}},
            'Interaction': {'config': {
                'detection_method': 'momentum_impulse',
                'duration': 12,
                'grace_time': 2.0,
                'wrench_interaction': wrench_config,
                'virtual_object': {
                    'current_mass': 0.17,
                    'mass': 2.0,
                    'inertia_command': 'orientation',
                },
            }},
        }
        calls = []
        controller.interaction_onboard_wrench_admittance = (
            lambda **kwargs: calls.append(kwargs)
        )

        controller._run_translation()

        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0]['nominal_position'], [0.1, 0.2, 1.0])
        self.assertEqual(calls[0]['nominal_yaw_deg'], 7.0)
        self.assertIsNot(calls[0]['config'], wrench_config)
        self.assertFalse(
            calls[0]['config']['calibration_excitation']['enabled']
        )
        self.assertFalse(
            calls[0]['config']['startup_bias_calibration_enabled']
        )
        self.assertEqual(calls[0]['rearm_delay_s'], 2.0)
        self.assertEqual(
            calls[0]['virtual_object_config']['inertia_command'],
            'orientation',
        )

    def test_sensor_default_release_does_not_require_planar_fit(self):
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.drone_id = 'lb11'
        controller.lo_commander = FakeCommander()
        controller.force_sensor = object()
        controller.sense_axis = 'y'
        calls = []
        controller.interaction_onboard_wrench_admittance = (
            lambda **kwargs: calls.append(kwargs)
        )
        with tempfile.TemporaryDirectory() as directory:
            controller.mission = {
                'drones': {'lb11': {'target': [0.0, 0.0, 1.0]}},
                'Interaction': {'config': {
                    'detection_method': 'momentum_impulse',
                    'duration': 12,
                    'wrench_calibration_file': (
                        f'{directory}/missing-calibration.json'
                    ),
                    'wrench_interaction': {
                        'state_source': 'onboard',
                        'shadow_mode': False,
                    },
                    # Omitting release_behavior with --sense must resolve the
                    # same way here as in the runtime loop: potentiometer coast.
                    'virtual_object': {},
                }},
            }

            with self.assertLogs(level='WARNING') as captured:
                controller._run_translation()

        self.assertEqual(len(calls), 1)
        self.assertTrue(any(
            'No saved wrench model calibration' in message
            for message in captured.output
        ))

    def test_calibration_forces_shadow_excitation_and_uses_its_duration(self):
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.drone_id = 'lb11'
        controller.lo_commander = FakeCommander()
        controller.mission = {
            'drones': {'lb11': {'target': [0.1, 0.2, 1.0, 7.0]}},
            'Interaction': {
                'action': 'translation',
                'config': {
                    'detection_method': 'momentum_impulse',
                    'duration': 60,
                    'wrench_calibration_file': '/tmp/test-calibration.json',
                    'wrench_interaction': {
                        'state_source': 'onboard',
                        'shadow_mode': False,
                        'calibration_excitation': {
                            'enabled': False,
                            'start_delay_s': 2.0,
                            'duration_s': 10.0,
                        },
                    },
                },
            },
        }
        calls = []
        controller.interaction_onboard_wrench_admittance = (
            lambda **kwargs: calls.append(kwargs)
        )

        controller.run_calibration()

        self.assertEqual(len(calls), 1)
        self.assertAlmostEqual(calls[0]['duration'], 12.5)
        self.assertTrue(calls[0]['calibration_mode'])
        self.assertTrue(
            calls[0]['config']['calibration_excitation']['enabled']
        )
        self.assertTrue(
            calls[0]['config']['startup_bias_calibration_enabled']
        )
        self.assertTrue(calls[0]['config']['shadow_mode'])
        self.assertFalse(
            calls[0]['config']['planar_braking_calibration']['enabled']
        )
        self.assertNotIn('position_capture_calibration', calls[0]['config'])
        self.assertEqual(
            calls[0]['calibration_path'], '/tmp/test-calibration.json'
        )

    def test_calibration_ignores_retired_position_capture_clearance(self):
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.drone_id = 'lb11'
        controller.lo_commander = FakeCommander()
        controller.bounds = {
            'x_min': -0.5, 'x_max': 0.5,
            'y_min': -0.5, 'y_max': 0.5,
            'z_min': 0.0, 'z_max': 2.0,
        }
        controller.mission = {
            'drones': {'lb11': {'target': [0.0, 0.0, 1.0, 0.0]}},
            'Interaction': {
                'action': 'translation',
                'config': {
                    'detection_method': 'momentum_impulse',
                    'duration': 60,
                    'wrench_interaction': {
                        'state_source': 'onboard',
                        'planar_braking_calibration': {
                            'max_displacement_m': 0.45,
                        },
                        'position_capture_calibration': {
                            'max_displacement_m': 1.0,
                        },
                    },
                },
            },
        }
        calls = []
        controller.interaction_onboard_wrench_admittance = (
            lambda **kwargs: calls.append(kwargs)
        )

        controller.run_calibration()
        self.assertEqual(len(calls), 1)
        self.assertNotIn('position_capture_calibration', calls[0]['config'])

    def test_mpc_automatic_acceleration_has_explicit_attitude_owner(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_calibrated_direction_xy=[0.0, 1.0],
        )

        self.assertTrue(control.start_contact(
            'mpc_bootstrap_acceleration', [0.0, 0.0, 1.0]
        ))
        self.assertTrue(control.mpc_bootstrap_acceleration_mode)
        self.assertFalse(control.attitude_mode)
        self.assertEqual(
            control.command_mode, control.MPC_BOOTSTRAP_ACCELERATION
        )
        control.set_contact_attitude(-8.0, 0.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=1.0, yaw_deg=0.0)
        self.assertEqual(commander.calls[-1][0], 'zdistance')
        self.assertEqual(
            control.sent_command_snapshot()['kind'],
            'attitude_zdistance',
        )

        self.assertTrue(control.end_contact(
            [0.0, 0.1, 1.0],
            [0.0, 0.25, 0.0],
            1.02,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.radians([-8.0, 0.0, 0.0]),
            coast=True,
        ))
        self.assertEqual(control.command_mode, control.ATTITUDE_COAST)
        self.assertFalse(control.mpc_bootstrap_acceleration_mode)

    def test_mpc_loop_automatically_accelerates_and_logs_sensor_free_start(self):
        class StopAfterBootstrapStart(RuntimeError):
            pass

        class DeterministicClock:
            def __init__(self):
                self.wall_s = 1000.0
                self.monotonic_s = 500.0

            def time(self):
                return self.wall_s

            def monotonic(self):
                return self.monotonic_s

            def advance(self, duration_s):
                duration_s = max(float(duration_s), 0.0)
                self.wall_s += duration_s
                self.monotonic_s += duration_s

        clock = DeterministicClock()
        commander = FakeCommander()

        class AutomaticMPCLogManager(FakeOnboardLogManager):
            def get_nearest_group_log_data(self, group_name, timestamp):
                packet, skew = super().get_nearest_group_log_data(
                    group_name, timestamp
                )
                if group_name == 'VEL_ORI':
                    attitude_calls = [
                        call for call in commander.calls
                        if call[0] == 'zdistance'
                    ]
                    if (
                        attitude_calls
                        and attitude_calls[-1][1][0] <= -7.99
                    ):
                        packet['stateEstimate.vy'] = 0.25
                        packet['stateEstimate.roll'] = -8.0
                return packet, skew

        logs = AutomaticMPCLogManager(clock.time())
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.drone_id = 'lb11'
        controller.log_manager = logs
        controller.ctrl_rate = 100
        controller.bounds = {
            'x_min': -1.0, 'x_max': 1.0,
            'y_min': -1.0, 'y_max': 1.0,
            'z_min': 0.3, 'z_max': 2.0,
        }
        controller.hl_commander = FakeCommander()
        controller.lo_commander = commander
        controller.force_sensor = None

        def safe_sleep(duration_s):
            clock.advance(duration_s)
            logs.packet_time = clock.time()

        controller._safe_sleep = safe_sleep
        start_events = []

        def log_event(name, data=None):
            payload = {} if data is None else data
            logs.add_log_entry('events', payload, name=name)
            if name == 'Learning MPC Bootstrap Braking Started':
                start_events.append(payload)
                raise StopAfterBootstrapStart

        controller._log_event = log_event
        contracts = {
            'positive_y': {
                'direction_label': 'positive_y',
                'direction_xy': [0.0, 1.0],
                'command_delay_s': 0.02,
                'model_fingerprint': 'test-positive-model',
                'state_dimension': 4,
            },
            'negative_y': {
                'direction_label': 'negative_y',
                'direction_xy': [0.0, -1.0],
                'command_delay_s': 0.02,
                'model_fingerprint': 'test-negative-model',
                'state_dimension': 4,
            },
        }

        def contract_for_direction(_contracts, direction_xy):
            return contracts[
                'positive_y' if direction_xy[1] > 0.0 else 'negative_y'
            ]

        config = {
            'state_source': 'onboard',
            'shadow_mode': False,
            'startup_bias_calibration_enabled': False,
            'initial_contact_arming': {'enabled': False},
            'detection': {
                'translation': {'enabled': False},
                'yaw': {'enabled': False},
            },
            'predictive_braking': {'enabled': False},
            'learning_velocity_mpc_shadow': {
                'enabled': False,
                'command_authority': False,
            },
            'mpc_bootstrap_calibration': {
                'enabled': True,
                'initial_speed_targets_m_s': [0.25],
                'speed_tolerance_m_s': 0.04,
                'repetitions_per_cell': 1,
                'max_release_speed_m_s': 0.40,
                'ready_dwell_s': 0.02,
                'level_warmup_min_s': 0.02,
                'prediction_step_s': 0.02,
                'max_acceleration_duration_s': 0.20,
                'max_maneuver_displacement_m': 0.20,
            },
            'mpc_bootstrap_model_contracts': contracts,
            'control_handoff': {
                'coast_attitude_response_delay_s': 0.02,
                'coast_velocity_braking_enabled': False,
                'coast_velocity_predictive_unwind_enabled': False,
            },
        }
        virtual_object = {
            'inertia_command': 'orientation',
            'force_rendering': {'enabled': False},
            'contact_detection': {'source': 'wrench_observer'},
            'release_behavior': {'mode': 'observer_brake'},
        }

        with patch(
            'Interaction.interactions.time.time', side_effect=clock.time
        ), patch(
            'Interaction.interactions.time.monotonic',
            side_effect=clock.monotonic,
        ), patch(
            'Interaction.interactions.validate_mpc_bootstrap_model_contracts',
            return_value=contracts,
        ), patch(
            'Interaction.interactions.mpc_bootstrap_model_contract_for_direction',
            side_effect=contract_for_direction,
        ), self.assertRaises(StopAfterBootstrapStart):
            controller.interaction_onboard_wrench_admittance(
                duration=10.0,
                nominal_position=[0.0, 0.0, 1.0],
                nominal_yaw_deg=0.0,
                config=config,
                virtual_object_config=virtual_object,
                mpc_calibration_mode=True,
            )

        self.assertIsNone(controller.force_sensor)
        attitude_calls = [
            call[1] for call in commander.calls if call[0] == 'zdistance'
        ]
        level_index = next(
            index for index, call in enumerate(attitude_calls)
            if call[:2] == (0.0, 0.0)
        )
        acceleration_index = next(
            index for index, call in enumerate(attitude_calls)
            if call[:2] == (-8.0, 0.0)
        )
        self.assertLess(level_index, acceleration_index)
        self.assertEqual(len(start_events), 1)
        start = start_events[0]
        self.assertEqual(
            start['release_dataset_axis_source'],
            'automatic_world_y_profile',
        )
        self.assertEqual(
            start['measured_velocity_m_s'],
            start['coast_initial_velocity_m_s'],
        )
        self.assertEqual(start['measured_velocity_m_s'], [0.0, 0.25, 0.0])
        self.assertIsNone(
            start['release_dataset_measured_sensor_axis_world_xy']
        )
        self.assertEqual(
            start['initial_command_mode'],
            TranslationControlHandoff.ATTITUDE_COAST,
        )

    def test_mpc_terminal_gate_revalidates_rebound_before_position_handoff(self):
        class StopAfterReboundRow(RuntimeError):
            pass

        scenario = SimpleNamespace(
            phase='prelude',
            terminal_complete_row=None,
            rebound_row=None,
            position_count_at_start=None,
        )

        def state_mutator(context, group_name, packet):
            if group_name == 'VEL_ORI':
                if scenario.phase == 'terminal':
                    packet['stateEstimate.vy'] = 0.0
                    packet['stateEstimate.roll'] = 0.0
                elif scenario.phase == 'rebound':
                    packet['stateEstimate.vy'] = 0.06
                    packet['stateEstimate.roll'] = 4.0
                else:
                    attitude_calls = [
                        call for call in context.commander.calls
                        if call[0] == 'zdistance'
                    ]
                    if (
                        attitude_calls
                        and attitude_calls[-1][1][0] <= -7.99
                    ):
                        packet['stateEstimate.vy'] = 0.25
                        packet['stateEstimate.roll'] = -8.0
            elif (
                group_name == 'POS_ACC'
                and scenario.phase in ('terminal', 'rebound')
            ):
                packet['stateEstimate.y'] = 0.12

        def event_callback(context, name, _payload):
            if name == 'Learning MPC Bootstrap Braking Started':
                scenario.phase = 'terminal'
                scenario.position_count_at_start = sum(
                    call[0] == 'position'
                    for call in context.commander.calls
                )

        def record_callback(_context, group_name, _name, entry):
            if (
                group_name != 'wrench_observer'
                or entry.get('release_dataset_episode_id') is None
            ):
                return
            gate = entry['release_dataset_terminal_gate']
            if scenario.phase == 'terminal' and gate['complete']:
                scenario.terminal_complete_row = entry
                scenario.phase = 'rebound'
            elif scenario.phase == 'rebound':
                scenario.rebound_row = entry
                raise StopAfterReboundRow

        context = self._run_automatic_mpc_loop_harness(
            stop_exception=StopAfterReboundRow,
            state_mutator=state_mutator,
            event_callback=event_callback,
            record_callback=record_callback,
            handoff_overrides={'coast_level_handoff_delay_s': 0.30},
        )

        self.assertIsNotNone(scenario.terminal_complete_row)
        self.assertTrue(
            scenario.terminal_complete_row[
                'release_dataset_terminal_gate'
            ]['complete']
        )
        self.assertEqual(
            scenario.terminal_complete_row['release_dataset_command_owner'],
            TranslationControlHandoff.ATTITUDE_COAST,
        )
        self.assertIsNone(
            scenario.terminal_complete_row['release_dataset_pending_outcome']
        )

        rebound = scenario.rebound_row
        self.assertIsNotNone(rebound)
        self.assertEqual(rebound['velocity_m_s'], [0.0, 0.06, 0.0])
        self.assertAlmostEqual(
            np.degrees(rebound['orientation_rpy_rad'][0]), 4.0
        )
        rebound_gate = rebound['release_dataset_terminal_gate']
        self.assertFalse(rebound_gate['complete'])
        self.assertIn(
            'xy_speed_above_terminal_limit', rebound_gate['violations']
        )
        self.assertIn(
            'attitude_above_terminal_limit', rebound_gate['violations']
        )
        self.assertEqual(
            rebound['release_dataset_command_owner'],
            TranslationControlHandoff.ATTITUDE_COAST,
        )
        self.assertTrue(
            rebound['actual_commands_sent_since_previous_state']
        )
        self.assertTrue(all(
            command['kind'] == 'attitude_zdistance'
            for command in rebound[
                'actual_commands_sent_since_previous_state'
            ]
        ))
        self.assertEqual(
            sum(
                call[0] == 'position'
                for call in context.commander.calls
            ),
            scenario.position_count_at_start,
        )
        self.assertEqual(context.commander.calls[-1][0], 'zdistance')

    def test_mpc_terminal_dwell_waits_for_real_handoff_and_upper_bracket(self):
        class StopAfterTerminalFinalize(RuntimeError):
            pass

        scenario = SimpleNamespace(
            phase='prelude',
            dataset_rows=[],
            prehandoff_complete_rows=[],
            handoff_row=None,
            upper_bracket_row=None,
            terminal_event=None,
        )

        def state_mutator(context, group_name, packet):
            if group_name == 'VEL_ORI':
                if scenario.phase == 'terminal':
                    packet['stateEstimate.vy'] = 0.0
                    packet['stateEstimate.roll'] = 0.0
                else:
                    attitude_calls = [
                        call for call in context.commander.calls
                        if call[0] == 'zdistance'
                    ]
                    if (
                        attitude_calls
                        and attitude_calls[-1][1][0] <= -7.99
                    ):
                        packet['stateEstimate.vy'] = 0.25
                        packet['stateEstimate.roll'] = -8.0
            elif group_name == 'POS_ACC' and scenario.phase == 'terminal':
                packet['stateEstimate.y'] = 0.12

        def event_callback(_context, name, payload):
            if name == 'Learning MPC Bootstrap Braking Started':
                scenario.phase = 'terminal'
            elif name == 'Release Dataset Terminal Dwell Complete':
                scenario.terminal_event = payload
                raise StopAfterTerminalFinalize

        def record_callback(_context, group_name, _name, entry):
            if (
                group_name != 'wrench_observer'
                or entry.get('release_dataset_episode_id') is None
            ):
                return
            scenario.dataset_rows.append(entry)
            if entry.get('release_dataset_resample_upper_bracket'):
                scenario.upper_bracket_row = entry
            elif entry.get('release_dataset_pending_outcome') == (
                'terminal_handoff'
            ):
                scenario.handoff_row = entry
            elif entry['release_dataset_terminal_gate']['complete']:
                scenario.prehandoff_complete_rows.append(entry)

        self._run_automatic_mpc_loop_harness(
            stop_exception=StopAfterTerminalFinalize,
            state_mutator=state_mutator,
            event_callback=event_callback,
            record_callback=record_callback,
            handoff_overrides={'coast_level_handoff_delay_s': 0.30},
        )

        self.assertTrue(scenario.prehandoff_complete_rows)
        first_complete = scenario.prehandoff_complete_rows[0]
        self.assertAlmostEqual(
            first_complete['release_dataset_terminal_gate']['dwell_s'],
            0.08,
            places=8,
        )
        for row in scenario.prehandoff_complete_rows:
            self.assertEqual(
                row['release_dataset_command_owner'],
                TranslationControlHandoff.ATTITUDE_COAST,
            )
            self.assertIsNone(row['release_dataset_pending_outcome'])
            self.assertTrue(row['actual_commands_sent_since_previous_state'])
            self.assertTrue(all(
                command['kind'] == 'attitude_zdistance'
                for command in row[
                    'actual_commands_sent_since_previous_state'
                ]
            ))

        handoff_row = scenario.handoff_row
        self.assertIsNotNone(handoff_row)
        self.assertEqual(
            handoff_row['release_dataset_command_owner'],
            TranslationControlHandoff.POSITION_HOLD,
        )
        self.assertTrue(
            handoff_row['release_dataset_terminal_gate']['complete']
        )
        self.assertEqual(
            handoff_row['release_dataset_pending_outcome'],
            'terminal_handoff',
        )
        handoff_commands = handoff_row[
            'actual_commands_sent_since_previous_state'
        ]
        self.assertEqual(len(handoff_commands), 1)
        self.assertEqual(handoff_commands[0]['kind'], 'position')
        self.assertEqual(
            handoff_commands[0]['position_m'], [0.0, 0.12, 1.0]
        )

        level_commands = [
            command
            for row in scenario.dataset_rows
            for command in (
                row.get('actual_commands_sent_since_previous_state') or []
            )
            if (
                command['kind'] == 'attitude_zdistance'
                and abs(command['roll_deg']) <= 1e-12
                and abs(command['pitch_deg']) <= 1e-12
            )
        ]
        self.assertTrue(level_commands)
        self.assertGreaterEqual(
            handoff_commands[0]['sent_at']-level_commands[0]['sent_at'],
            0.30-1e-12,
        )

        upper = scenario.upper_bracket_row
        self.assertIsNotNone(upper)
        self.assertTrue(upper['release_dataset_resample_upper_bracket'])
        self.assertIsNone(upper['release_dataset_pending_outcome'])
        self.assertEqual(
            upper['release_dataset_final_position_sequence'],
            handoff_commands[0]['sequence'],
        )
        self.assertEqual(
            upper['release_dataset_final_position_sent_at'],
            handoff_commands[0]['sent_at'],
        )
        self.assertGreaterEqual(
            upper['state_time'],
            upper['release_dataset_final_position_sent_at'],
        )
        self.assertEqual(
            upper['actual_commands_sent_since_previous_state'], []
        )
        terminal_event = scenario.terminal_event
        self.assertIsNotNone(terminal_event)
        self.assertEqual(
            terminal_event['release_dataset_outcome'], 'terminal_handoff'
        )
        self.assertEqual(
            terminal_event['final_position_sequence'],
            handoff_commands[0]['sequence'],
        )
        self.assertEqual(
            terminal_event['resample_upper_bracket_state_time'],
            upper['state_time'],
        )

    def test_mpc_coast_predictor_runtime_does_not_drift_command_cadence(self):
        class StopAfterCoastCadenceRows(RuntimeError):
            pass

        scenario = SimpleNamespace(
            phase='prelude',
            context=None,
            start_wall_s=None,
            predictor_calls=[],
            coast_commands={},
        )

        def state_mutator(context, group_name, packet):
            scenario.context = context
            if group_name != 'VEL_ORI':
                return
            if scenario.phase == 'coast':
                packet['stateEstimate.vy'] = 0.20
                packet['stateEstimate.roll'] = 0.0
                return
            attitude_calls = [
                call for call in context.commander.calls
                if call[0] == 'zdistance'
            ]
            if attitude_calls and attitude_calls[-1][1][0] <= -7.99:
                packet['stateEstimate.vy'] = 0.25
                packet['stateEstimate.roll'] = -8.0

        def event_callback(context, name, _payload):
            if name == 'Learning MPC Bootstrap Braking Started':
                scenario.phase = 'coast'
                scenario.start_wall_s = context.clock.time()

        def record_callback(_context, group_name, _name, entry):
            if (
                group_name != 'wrench_observer'
                or entry.get('release_dataset_episode_id') is None
                or scenario.start_wall_s is None
            ):
                return
            for command in (
                entry.get('actual_commands_sent_since_previous_state') or []
            ):
                if (
                    command['kind'] == 'attitude_zdistance'
                    and command['sent_at'] > scenario.start_wall_s+1e-9
                ):
                    scenario.coast_commands[command['sequence']] = command
            if len(scenario.coast_commands) >= 5:
                raise StopAfterCoastCadenceRows

        real_update = TranslationControlHandoff.update_coast_attitude

        def update_with_pi_runtime(control, *args, **kwargs):
            context = scenario.context
            entry_wall_s = context.clock.time()
            entry_monotonic_s = context.clock.monotonic()
            planned_wall_s = kwargs['command_timestamp']
            planned_monotonic_s = (
                entry_monotonic_s+planned_wall_s-entry_wall_s
            )
            # Model a bounded Pi-side predictor calculation without sleeping
            # the test process. The later production pacing wait must absorb
            # this runtime instead of adding it to the 20 ms send interval.
            context.clock.advance(0.0015)
            scenario.predictor_calls.append({
                'entry_wall_s': entry_wall_s,
                'planned_wall_s': planned_wall_s,
                'planned_monotonic_s': planned_monotonic_s,
                'finished_monotonic_s': context.clock.monotonic(),
            })
            return real_update(control, *args, **kwargs)

        with patch.object(
            TranslationControlHandoff,
            'update_coast_attitude',
            autospec=True,
            side_effect=update_with_pi_runtime,
        ):
            context = self._run_automatic_mpc_loop_harness(
                stop_exception=StopAfterCoastCadenceRows,
                state_mutator=state_mutator,
                event_callback=event_callback,
                record_callback=record_callback,
            )

        self.assertGreaterEqual(len(scenario.predictor_calls), 5)
        for call in scenario.predictor_calls:
            self.assertGreater(
                call['planned_wall_s'], call['entry_wall_s']
            )
            self.assertLess(
                call['finished_monotonic_s'], call['planned_monotonic_s']
            )

        commands = [
            scenario.coast_commands[sequence]
            for sequence in sorted(scenario.coast_commands)
        ]
        self.assertGreaterEqual(len(commands), 5)
        send_intervals_s = [
            later['sent_at']-earlier['sent_at']
            for earlier, later in zip(commands, commands[1:])
        ]
        self.assertTrue(send_intervals_s)
        for interval_s in send_intervals_s:
            self.assertAlmostEqual(interval_s, 0.020, delta=0.001)
        planned_times_s = [
            call['planned_wall_s'] for call in scenario.predictor_calls
        ]
        for command in commands:
            self.assertTrue(any(
                abs(command['sent_at']-planned_s) <= 1e-9
                for planned_s in planned_times_s
            ))
        self.assertNotIn(
            'Learning MPC Bootstrap Command Cadence Rejected',
            [name for name, _payload in context.events],
        )

    def test_mpc_recoverable_prelude_sends_current_position_before_reschedule(self):
        class StopAfterReschedule(RuntimeError):
            pass

        scenario = SimpleNamespace(
            phase='prelude',
            scheduled_count=0,
        )

        def state_mutator(context, group_name, packet):
            if group_name == 'VEL_ORI':
                if scenario.phase == 'safe_stop':
                    packet['stateEstimate.vy'] = 0.0
                    packet['stateEstimate.roll'] = 0.0
                else:
                    attitude_calls = [
                        call for call in context.commander.calls
                        if call[0] == 'zdistance'
                    ]
                    if (
                        attitude_calls
                        and attitude_calls[-1][1][0] <= -7.99
                    ):
                        # Skip the 0.25 +/- 0.02 m/s release window without
                        # crossing the recoverable 0.40 m/s absolute limit.
                        packet['stateEstimate.vy'] = 0.30
                        packet['stateEstimate.roll'] = -8.0
            elif group_name == 'POS_ACC' and scenario.phase == 'safe_stop':
                packet['stateEstimate.y'] = 0.12

        def event_callback(_context, name, _payload):
            if name == 'Learning MPC Bootstrap Automatic Attempt Scheduled':
                scenario.scheduled_count += 1
                if scenario.scheduled_count == 2:
                    raise StopAfterReschedule
            elif name == 'Learning MPC Bootstrap Automatic Prelude Rejected':
                scenario.phase = 'safe_stop'

        context = self._run_automatic_mpc_loop_harness(
            stop_exception=StopAfterReschedule,
            state_mutator=state_mutator,
            event_callback=event_callback,
            handoff_overrides={'coast_level_handoff_delay_s': 0.02},
        )

        event_names = [name for name, _payload in context.events]
        self.assertNotIn(
            'Learning MPC Bootstrap Braking Started', event_names
        )
        rejected = next(
            payload for name, payload in context.events
            if name == 'Learning MPC Bootstrap Automatic Prelude Rejected'
        )
        self.assertIn(
            'target_window_skipped_automatic_prelude_violation',
            rejected['prelude_failure_reasons'],
        )
        self.assertFalse(rejected['dataset_started'])
        self.assertEqual(
            rejected['safe_fallback'], 'legacy_attitude_coast_to_rest'
        )

        handoff = next(
            payload for name, payload in context.events
            if name == 'Coast Position Control Handoff'
        )
        self.assertEqual(
            handoff['reason'], 'terminal_current_position_handoff'
        )
        self.assertEqual(handoff['target_position_m'], [0.0, 0.12, 1.0])
        finished = next(
            payload for name, payload in context.events
            if name == 'Learning MPC Bootstrap Automatic Attempt Finished'
        )
        self.assertFalse(finished['counted'])
        self.assertEqual(finished['retry_count_for_cell'], 1)
        self.assertEqual(
            finished['reason'],
            'target_window_skipped_automatic_prelude_violation',
        )
        self.assertEqual(finished['return_target_m'], [0.0, 0.0, 1.0])

        rejection_index = next(
            index for index, item in enumerate(context.timeline)
            if item[:2] == (
                'event',
                'Learning MPC Bootstrap Automatic Prelude Rejected',
            )
        )
        finish_index = next(
            index for index, item in enumerate(context.timeline)
            if item[:2] == (
                'event',
                'Learning MPC Bootstrap Automatic Attempt Finished',
            )
        )
        scheduled_indices = [
            index for index, item in enumerate(context.timeline)
            if item[:2] == (
                'event',
                'Learning MPC Bootstrap Automatic Attempt Scheduled',
            )
        ]
        safe_position_indices = [
            index for index, item in enumerate(context.timeline)
            if item[:2] == ('command', 'position')
            and rejection_index < index < finish_index
        ]
        self.assertEqual(len(safe_position_indices), 1)
        safe_position_index = safe_position_indices[0]
        self.assertEqual(
            context.timeline[safe_position_index][2],
            (0.0, 0.12, 1.0, 0.0),
        )
        reset_indices = {
            item[1]: index
            for index, item in enumerate(context.timeline)
            if item[0] == 'integrator_reset'
            and rejection_index < index < safe_position_index
        }
        self.assertEqual(
            set(reset_indices),
            {'posCtlPid.resetI', 'velCtlPid.resetI'},
        )
        self.assertLess(
            max(reset_indices.values()), safe_position_index
        )
        self.assertLess(rejection_index, safe_position_index)
        self.assertLess(safe_position_index, finish_index)
        self.assertLess(finish_index, scheduled_indices[1])
        scheduled_payloads = [
            payload for name, payload in context.events
            if name == 'Learning MPC Bootstrap Automatic Attempt Scheduled'
        ]
        self.assertEqual(len(scheduled_payloads), 2)
        self.assertEqual(scheduled_payloads[1]['direction_sign'], -1)

    def test_jerk_handoff_rechecks_state_freshness_before_first_position_send(
            self):
        scenario = SimpleNamespace(
            phase='prelude',
            decision_state_age_s=None,
            position_count_before_stall=None,
            handoff_payload=None,
            post_handoff_freshness_checks=0,
        )

        def state_mutator(context, group_name, packet):
            if group_name == 'VEL_ORI':
                if scenario.phase == 'braking':
                    packet['stateEstimate.vy'] = 0.0
                    packet['stateEstimate.roll'] = 0.0
                else:
                    attitude_calls = [
                        call for call in context.commander.calls
                        if call[0] == 'zdistance'
                    ]
                    if (
                        attitude_calls
                        and attitude_calls[-1][1][0] <= -7.99
                    ):
                        packet['stateEstimate.vy'] = 0.25
                        packet['stateEstimate.roll'] = -8.0
            elif group_name == 'POS_ACC' and scenario.phase == 'braking':
                packet['stateEstimate.y'] = 0.12

        def event_callback(context, name, payload):
            if name == 'Learning MPC Bootstrap Braking Started':
                scenario.phase = 'braking'
            elif name == 'Coast Position Control Handoff':
                scenario.handoff_payload = payload
                scenario.position_count_before_stall = sum(
                    call[0] == 'position'
                    for call in context.commander.calls
                )
                # The state was fresh when the handoff decision returned. A
                # slow log sink after both PID resets then consumes more than
                # the normal 100 ms freshness budget before the first send.
                context.clock.advance(0.20)

        real_update = TranslationControlHandoff.update_coast_attitude

        def update_with_jerk_handoff_timestamp(control, *args, **kwargs):
            completed = real_update(control, *args, **kwargs)
            if completed:
                state_timestamp = float(args[4])
                scenario.decision_state_age_s = float(
                    kwargs['command_timestamp'] - state_timestamp
                )
                # This test exercises the common main-loop handoff/send
                # boundary. The legacy coast supplies a natural terminal
                # transition; relabel only its provenance so the jerk-specific
                # freshness guard owns the otherwise identical transition.
                control.coast_handoff_reason = (
                    'jerk_limited_attitude_position_handoff'
                )
                control.brake_completion_reason = control.coast_handoff_reason
                control.coast_jerk_limited_handoff_state_timestamp = (
                    state_timestamp
                )
            return completed

        def freshness_with_calibration_gate_passthrough(
                state_timestamp, decision_timestamp, maximum_age_s):
            age_s = float(decision_timestamp - state_timestamp)
            if scenario.handoff_payload is not None:
                scenario.post_handoff_freshness_checks += 1
                if scenario.post_handoff_freshness_checks == 1:
                    # The automatic-calibration harness has an additional
                    # scheduled-command gate before the common position-send
                    # branch. Let that unrelated gate pass so this regression
                    # reaches the jerk handoff's second, final freshness check.
                    return True
            return bool(0.0 <= age_s <= maximum_age_s)

        with patch(
            'Interaction.interactions.mpc_decision_state_age_is_fresh',
            side_effect=freshness_with_calibration_gate_passthrough,
        ), patch.object(
            TranslationControlHandoff,
            'update_coast_attitude',
            autospec=True,
            side_effect=update_with_jerk_handoff_timestamp,
        ):
            context = self._run_automatic_mpc_loop_harness(
                stop_exception=StaleLocalizationError,
                state_mutator=state_mutator,
                event_callback=event_callback,
                handoff_overrides={'coast_level_handoff_delay_s': 0.02},
            )

        self.assertIsNotNone(scenario.handoff_payload)
        self.assertIsInstance(context.stop_exception, StaleLocalizationError)
        self.assertIsNotNone(scenario.decision_state_age_s)
        self.assertGreaterEqual(scenario.decision_state_age_s, 0.0)
        self.assertLessEqual(scenario.decision_state_age_s, 0.10)
        self.assertGreaterEqual(scenario.post_handoff_freshness_checks, 2)
        self.assertEqual(
            sum(call[0] == 'position' for call in context.commander.calls),
            scenario.position_count_before_stall,
        )
        self.assertEqual(context.commander.calls[-1][0], 'zdistance')
        np.testing.assert_allclose(
            context.commander.calls[-1][1], [0.0, 0.0, 0.0, 1.0]
        )

        event_names = [name for name, _payload in context.events]
        self.assertIn(
            'Coast Position Handoff State Rejected Before Send',
            event_names,
            msg=(event_names, str(context.stop_exception)),
        )
        rejected = next(
            payload for name, payload in context.events
            if name == 'Coast Position Handoff State Rejected Before Send'
        )
        self.assertFalse(rejected['position_command_sent'])
        self.assertEqual(rejected['command_sent'], 'level_attitude_fixed_z')
        self.assertGreater(
            rejected['handoff_state_age_at_send_s'],
            rejected['maximum_state_age_s'],
        )

        handoff_index = next(
            index for index, item in enumerate(context.timeline)
            if item[:2] == ('event', 'Coast Position Control Handoff')
        )
        guard_index = next(
            index for index, item in enumerate(context.timeline)
            if item[:2] == ('command', 'zdistance')
            and index > handoff_index
        )
        rejection_index = next(
            index for index, item in enumerate(context.timeline)
            if item[:2] == (
                'event',
                'Coast Position Handoff State Rejected Before Send',
            )
        )
        reset_indices = [
            index for index, item in enumerate(context.timeline)
            if item[0] == 'integrator_reset' and index < handoff_index
        ]
        self.assertGreaterEqual(len(reset_indices), 2)
        self.assertLess(max(reset_indices), handoff_index)
        self.assertLess(handoff_index, guard_index)
        self.assertLess(guard_index, rejection_index)

    def test_jerk_handoff_neutralizes_position_if_send_call_crosses_age_limit(
            self):
        scenario = SimpleNamespace(
            phase='prelude',
            position_count_before_send=None,
        )

        def state_mutator(context, group_name, packet):
            if group_name == 'VEL_ORI':
                if scenario.phase == 'braking':
                    packet['stateEstimate.vy'] = 0.0
                    packet['stateEstimate.roll'] = 0.0
                else:
                    attitude_calls = [
                        call for call in context.commander.calls
                        if call[0] == 'zdistance'
                    ]
                    if (
                        attitude_calls
                        and attitude_calls[-1][1][0] <= -7.99
                    ):
                        packet['stateEstimate.vy'] = 0.25
                        packet['stateEstimate.roll'] = -8.0
            elif group_name == 'POS_ACC' and scenario.phase == 'braking':
                packet['stateEstimate.y'] = 0.12

        def event_callback(context, name, payload):
            if name == 'Learning MPC Bootstrap Braking Started':
                scenario.phase = 'braking'
            elif name == 'Coast Position Control Handoff':
                scenario.position_count_before_send = sum(
                    call[0] == 'position'
                    for call in context.commander.calls
                )
                # The immediate pre-send check is still fresh. Simulate the
                # synchronous commander call blocking for 200 ms after the
                # position packet is issued but before send() records its time.
                context.position_send_stall_s = 0.20

        real_update = TranslationControlHandoff.update_coast_attitude

        def update_with_jerk_handoff_timestamp(control, *args, **kwargs):
            completed = real_update(control, *args, **kwargs)
            if completed:
                state_timestamp = float(args[4])
                control.coast_handoff_reason = (
                    'jerk_limited_attitude_position_handoff'
                )
                control.brake_completion_reason = control.coast_handoff_reason
                control.coast_jerk_limited_handoff_state_timestamp = (
                    state_timestamp
                )
            return completed

        with patch.object(
            TranslationControlHandoff,
            'update_coast_attitude',
            autospec=True,
            side_effect=update_with_jerk_handoff_timestamp,
        ):
            context = self._run_automatic_mpc_loop_harness(
                stop_exception=StaleLocalizationError,
                state_mutator=state_mutator,
                event_callback=event_callback,
                handoff_overrides={'coast_level_handoff_delay_s': 0.02},
            )

        self.assertIsInstance(context.stop_exception, StaleLocalizationError)
        self.assertIsNotNone(scenario.position_count_before_send)
        self.assertEqual(
            sum(call[0] == 'position' for call in context.commander.calls),
            scenario.position_count_before_send + 1,
        )
        self.assertEqual(context.commander.calls[-1][0], 'zdistance')
        np.testing.assert_allclose(
            context.commander.calls[-1][1], [0.0, 0.0, 0.0, 1.0]
        )
        stale_event = next(
            payload for name, payload in context.events
            if name == 'Coast Position Handoff State Stale During Send'
        )
        self.assertTrue(stale_event['position_command_sent'])
        self.assertEqual(
            stale_event['mitigation_command_sent'],
            'level_attitude_fixed_z',
        )
        self.assertGreater(
            stale_event['handoff_state_age_at_send_s'],
            stale_event['maximum_state_age_s'],
        )
        handoff_index = next(
            index for index, item in enumerate(context.timeline)
            if item[:2] == ('event', 'Coast Position Control Handoff')
        )
        position_index = next(
            index for index, item in enumerate(context.timeline)
            if item[:2] == ('command', 'position') and index > handoff_index
        )
        guard_index = next(
            index for index, item in enumerate(context.timeline)
            if item[:2] == ('command', 'zdistance') and index > position_index
        )
        stale_event_index = next(
            index for index, item in enumerate(context.timeline)
            if item[:2] == (
                'event', 'Coast Position Handoff State Stale During Send',
            )
        )
        self.assertLess(handoff_index, position_index)
        self.assertLess(position_index, guard_index)
        self.assertLess(guard_index, stale_event_index)

    def test_active_translation_aims_release_tilt_along_braking_direction_then_holds(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=5.0,
            shadow_mode=False,
            brake_xy_acceleration_m_s2=1.0,
            brake_xy_speed_m_s=0.04,
            coast_handoff_max_acceleration_m_s2=10.0,
            coast_handoff_max_tilt_deg=30.0,
            brake_min_attitude_taper_speed_m_s=0.15,
            brake_max_attitude_deg=20.0,
            brake_timeout_s=1.0,
        )

        control.send(commander)
        self.assertTrue(control.start_contact())
        control.set_contact_attitude(2.0, -3.0, 1.0)
        control.send(commander)
        self.assertTrue(control.end_contact(
            [0.25, -0.20, 0.95],
            [0.20, 0.0, 0.0],
            1.0,
            interaction_direction=[1.0, 0.0, 0.0],
            current_orientation_rpy=np.radians([4.0, -6.0, 0.0]),
        ))
        self.assertFalse(control.start_contact())
        control.send(commander)
        self.assertFalse(control.update_braking(
            [0.30, -0.2, 0.95], [0.10, 0.20, 0.0], 1.05
        ))
        control.send(commander)
        self.assertTrue(control.update_braking(
            [0.325, -0.18, 0.95], [0.03, 0.20, 0.0], 1.10
        ))
        control.send(commander)

        self.assertEqual([call[0] for call in commander.calls], [
            'position', 'zdistance', 'zdistance', 'zdistance', 'position'
        ])
        expected_release_tilt = 3.0
        np.testing.assert_allclose(
            commander.calls[2][1], [0.0, expected_release_tilt, 0.0, 0.95]
        )
        expected_updated_tilt = 3.0 * (0.10 - 0.04) / (0.15 - 0.04)
        np.testing.assert_allclose(
            commander.calls[3][1], [0.0, expected_updated_tilt, 0.0, 0.95]
        )
        np.testing.assert_allclose(
            commander.calls[4][1], [0.325, -0.18, 0.95, 5.0]
        )
        self.assertEqual(control.command_mode, 'position_hold')
        self.assertEqual(
            control.brake_completion_reason,
            'projected_velocity_zero_or_reversed',
        )

    def test_position_rendering_uses_measured_velocity_attitude_braking(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            brake_xy_speed_m_s=0.04,
            brake_timeout_s=1.0,
        )
        self.assertTrue(control.start_contact(
            'position', [0.1, 0.0, 1.0]
        ))
        control.set_contact_position([0.15, 0.0, 1.0])
        control.send(commander)
        self.assertEqual(commander.calls[-1][0], 'position')
        self.assertTrue(control.end_contact(
            [0.12, 0.0, 1.0], [0.20, 0.0, 0.0], 1.0,
            interaction_direction=[1.0, 0.0, 0.0],
        ))
        self.assertEqual(control.command_mode, 'attitude_braking')
        control.send(commander)
        self.assertEqual(commander.calls[-1][0], 'zdistance')
        self.assertFalse(control.update_braking(
            [0.13, 0.0, 1.0], [0.18, 0.0, 0.0], 1.1,
        ))
        control.send(commander)
        self.assertEqual(commander.calls[-1][0], 'zdistance')
        self.assertTrue(control.update_braking(
            [0.16, 0.0, 1.0], [0.03, 0.0, 0.0], 1.2,
        ))
        self.assertEqual(control.command_mode, 'position_hold')
        np.testing.assert_allclose(control.hold_position, [0.16, 0.0, 1.0])

    def test_passed_virtual_stop_is_clamped_to_actual_position(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            brake_xy_speed_m_s=0.04,
            coast_attitude_response_delay_s=0.0,
            coast_attitude_time_constant_s=0.0,
            coast_handoff_max_acceleration_m_s2=10.0,
            coast_handoff_max_tilt_deg=30.0,
        )
        self.assertTrue(control.start_contact('orientation'))
        control.set_contact_attitude(0.0, 0.0, 0.0)
        self.assertTrue(control.end_contact(
            [0.0, 0.1, 1.0],
            [0.0, 0.22, 0.0],
            1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_force=[0.0, 0.16, 0.0],
            current_mass_kg=0.17,
            coast=True,
        ))
        self.assertEqual(control.command_mode, 'attitude_coast')
        self.assertEqual(control.contact_roll_deg, 0.0)
        self.assertEqual(control.contact_pitch_deg, 0.0)
        control.send(commander, command_timestamp=1.0, yaw_deg=0.0)
        self.assertEqual(commander.calls[-1][0], 'zdistance')

        # The virtual target is deliberately behind the vehicle. It remains
        # comparison-only; attitude control must still dissipate velocity.
        self.assertFalse(control.update_coast_attitude(
            [0.0, 0.14, 1.0], [0.0, 0.20, 0.0],
            [0.0, 0.11, 1.0], [0.0, 0.0, 0.0], 1.01,
            command_timestamp=1.01,
        ))
        control.send(commander, command_timestamp=1.01, yaw_deg=0.0)
        self.assertLessEqual(control.coast_tracking_power_w_per_kg, 0.0)
        self.assertEqual(control.command_mode, 'attitude_coast')

        # The first signed-speed sample below 0.10 m/s latches level attitude.
        # A subsequent rebound does not restart braking or reset the timer.
        self.assertFalse(control.update_coast_attitude(
            [0.0, 0.18, 1.0], [0.0, 0.03, 0.0],
            [0.0, 0.11, 1.0], [0.0, 0.0, 0.0], 1.10,
            command_timestamp=1.10,
        ))
        control.send(commander, command_timestamp=1.10, yaw_deg=0.0)
        self.assertFalse(control.update_coast_attitude(
            [0.0, 0.19, 1.0], [0.0, 0.08, 0.0],
            [0.0, 0.11, 1.0], [0.0, 0.0, 0.0], 1.15,
            command_timestamp=1.15,
        ))
        control.send(commander, command_timestamp=1.15, yaw_deg=0.0)
        self.assertFalse(control.update_coast_attitude(
            [0.0, 0.21, 1.0], [0.0, 0.03, 0.0],
            [0.0, 0.11, 1.0], [0.0, 0.0, 0.0], 1.20,
            command_timestamp=1.20,
        ))
        control.send(commander, command_timestamp=1.20, yaw_deg=0.0)
        self.assertFalse(control.update_coast_attitude(
            [0.0, 0.22, 1.0], [0.0, 0.02, 0.0],
            [0.0, 0.11, 1.0], [0.0, 0.0, 0.0], 1.29,
            command_timestamp=1.29,
        ))
        control.send(commander, command_timestamp=1.29, yaw_deg=0.0)
        self.assertFalse(control.update_coast_attitude(
            [0.0, 0.22, 1.0], [0.0, 0.20, 0.0],
            [0.0, 0.11, 1.0], [0.0, 0.0, 0.0], 1.399,
            command_timestamp=1.399,
        ))
        control.send(commander, command_timestamp=1.399, yaw_deg=0.0)
        self.assertTrue(control.update_coast_attitude(
            [0.0, 0.22, 1.0], [0.0, 0.02, 0.0],
            [0.0, 0.11, 1.0], [0.0, 0.0, 0.0], 1.401,
            command_timestamp=1.401,
        ))
        self.assertEqual(control.command_mode, 'position_hold')
        control.send(commander)
        self.assertEqual(commander.calls[-1][0], 'position')
        np.testing.assert_allclose(control.hold_position, [0.0, 0.22, 1.0])
        np.testing.assert_allclose(
            control.stopping_position_m, [0.0, 0.22, 1.0]
        )
        self.assertTrue(control.coast_target_clamped_to_actual)
        self.assertEqual(
            control.brake_completion_reason,
            'timed_level_to_position_handoff',
        )
        self.assertEqual(
            control.coast_handoff_control_owner,
            'direct_attitude_zdistance',
        )
        self.assertEqual(
            control.coast_handoff_command_mode,
            control.ATTITUDE_COAST,
        )
        np.testing.assert_allclose(
            control.coast_handoff_attitude_target_rp_deg,
            [0.0, 0.0],
        )

    def test_handoff_holds_frozen_virtual_stop_when_it_is_still_ahead(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            brake_xy_speed_m_s=0.20,
            coast_attitude_response_delay_s=0.0,
            coast_attitude_time_constant_s=0.0,
            coast_handoff_speed_m_s=0.20,
            coast_handoff_max_acceleration_m_s2=10.0,
            coast_handoff_max_tilt_deg=30.0,
            coast_alignment_dwell_s=0.0,
            coast_level_handoff_delay_s=0.0,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.10, 1.0], [0.0, 0.30, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(
            [0.0, 0.10, 1.0], [0.0, 0.30, 0.0], timestamp=1.0,
        )
        control.send(commander, command_timestamp=1.0, yaw_deg=0.0)

        self.assertFalse(control.update_coast_attitude(
            [0.0, 0.16, 1.0], [0.0, 0.01, 0.0],
            [0.0, 0.28, 1.0], [0.0, 0.0, 0.0], 1.01,
            command_timestamp=1.01,
        ))
        control.send(commander, command_timestamp=1.01, yaw_deg=0.0)
        self.assertTrue(control.update_coast_attitude(
            [0.0, 0.16, 1.0], [0.0, 0.01, 0.0],
            [0.0, 0.28, 1.0], [0.0, 0.0, 0.0], 1.02,
            command_timestamp=1.02,
        ))
        np.testing.assert_allclose(control.hold_position, [0.0, 0.28, 1.0])
        np.testing.assert_allclose(
            control.coast_handoff_actual_position_m, [0.0, 0.16, 1.0]
        )
        self.assertFalse(control.coast_target_clamped_to_actual)

    def test_mpc_terminal_handoff_latches_measured_position(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_attitude_response_delay_s=0.0,
            coast_attitude_time_constant_s=0.0,
            coast_level_handoff_delay_s=0.0,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.10, 1.0], [0.0, 0.30, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(
            [0.0, 0.10, 1.0], [0.0, 0.30, 0.0], timestamp=1.0,
        )
        control.send(FakeCommander(), command_timestamp=1.0, yaw_deg=0.0)

        self.assertFalse(control.update_coast_attitude(
            [0.0, 0.16, 1.0], [0.0, 0.01, 0.0],
            [0.0, 0.28, 1.0], [0.0, 0.0, 0.0], 1.01,
            allow_position_handoff=False,
            latch_current_position_on_handoff=True,
            command_timestamp=1.01,
        ))
        control.send(FakeCommander(), command_timestamp=1.01, yaw_deg=0.0)
        self.assertTrue(control.update_coast_attitude(
            [0.0, 0.16, 1.0], [0.0, 0.01, 0.0],
            [0.0, 0.28, 1.0], [0.0, 0.0, 0.0], 1.02,
            allow_position_handoff=True,
            latch_current_position_on_handoff=True,
            command_timestamp=1.02,
        ))
        np.testing.assert_allclose(control.hold_position, [0.0, 0.16, 1.0])
        self.assertTrue(control.coast_target_clamped_to_actual)
        self.assertEqual(
            control.coast_handoff_reason,
            'terminal_current_position_handoff',
        )

    def test_calibrated_delayed_braking_handoffs_without_reverse(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_attitude_response_delay_s=0.12,
            coast_attitude_time_constant_s=0.08,
            coast_attitude_acceleration_scale=1.10,
            coast_level_terminal_speed_m_s=0.03,
            coast_command_period_s=0.02,
            coast_handoff_speed_m_s=0.04,
            coast_handoff_max_tilt_deg=3.0,
            coast_handoff_max_acceleration_m_s2=0.35,
            coast_alignment_dwell_s=0.05,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.50, 0.0], 0.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control.confirm_release_candidate(timestamp=0.0)

        dt = 0.02
        position = 0.0
        speed = 0.50
        acceleration = 0.0
        plant_command_history = [(-1.0, 0.0)]
        minimum_speed = speed
        maximum_position = position
        completed = False
        for step in range(150):
            timestamp = step * dt
            actual_roll_deg = -np.degrees(np.arctan2(
                acceleration, 9.81
            ))
            completed = control.update_coast_attitude(
                [0.0, position, 1.0],
                [0.0, speed, 0.0],
                [0.0, 0.25, 1.0],
                [0.0, 0.0, 0.0],
                timestamp,
                [np.radians(actual_roll_deg), 0.0, 0.0],
                command_timestamp=timestamp,
            )
            control.send(
                commander, command_timestamp=timestamp, yaw_deg=0.0
            )
            command = (
                0.0
                if control.mode == control.POSITION_HOLD
                else attitude_to_world_acceleration(
                    control.contact_roll_deg,
                    control.contact_pitch_deg,
                    0.0,
                )[1]
            )
            plant_command_history.append((timestamp, command))
            delayed_time = timestamp + dt - 0.12
            delayed_command = 0.0
            for command_time, historical_command in plant_command_history:
                if command_time > delayed_time:
                    break
                delayed_command = historical_command
            alpha = 1.0 - np.exp(-dt / 0.08)
            next_acceleration = acceleration + alpha * (
                1.10 * delayed_command - acceleration
            )
            next_speed = speed + 0.5 * (
                acceleration + next_acceleration
            ) * dt
            next_position = position + 0.5 * (
                speed + next_speed
            ) * dt
            minimum_speed = min(minimum_speed, next_speed)
            maximum_position = max(maximum_position, next_position)
            position = next_position
            speed = next_speed
            acceleration = next_acceleration
            if completed:
                break

        self.assertTrue(completed)
        self.assertEqual(control.command_mode, 'position_hold')
        self.assertGreaterEqual(minimum_speed, -0.02)
        self.assertLessEqual(maximum_position, 0.27)
        self.assertTrue(control.coast_target_clamped_to_actual)
        self.assertLessEqual(abs(speed), 0.10)
        self.assertEqual(
            control.coast_handoff_reason,
            'timed_level_to_position_handoff',
        )

    def test_coast_handoff_levels_at_point_one_then_waits_point_three(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_level_handoff_speed_m_s=0.10,
            coast_level_handoff_delay_s=0.30,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.30, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)
        control.send(commander, command_timestamp=1.0, yaw_deg=0.0)

        # Above the threshold the existing target-aware attitude brake remains
        # active. The threshold sample itself immediately commands true level.
        self.assertFalse(control.update_coast_attitude(
            [0.0, 0.01, 1.0], [0.0, 0.11, 0.0],
            [0.0, 0.20, 1.0], [0.0, 0.0, 0.0], 1.01,
            command_timestamp=1.01,
        ))
        self.assertFalse(control._coast_level_handoff_latched)
        control.send(commander, command_timestamp=1.01, yaw_deg=0.0)
        self.assertFalse(control.update_coast_attitude(
            [0.0, 0.02, 1.0], [0.0, 0.099, 0.0],
            [0.0, 0.20, 1.0], [0.0, 0.0, 0.0], 1.02,
            current_orientation_rpy=np.zeros(3),
            command_timestamp=1.02,
        ))
        control.send(commander, command_timestamp=1.02, yaw_deg=0.0)
        self.assertTrue(control._coast_level_handoff_latched)
        self.assertEqual(control.contact_roll_deg, 0.0)
        self.assertEqual(control.contact_pitch_deg, 0.0)
        self.assertFalse(control.coast_handoff_state_ready)

        # A speed rebound cannot re-enable attitude braking after the latch.
        self.assertFalse(control.update_coast_attitude(
            [0.0, 0.03, 1.0], [0.0, 0.15, 0.0],
            [0.0, 0.20, 1.0], [0.0, 0.0, 0.0], 1.319,
            current_orientation_rpy=np.radians([8.0, 0.0, 0.0]),
            command_timestamp=1.319,
        ))
        self.assertEqual(control.contact_roll_deg, 0.0)
        self.assertEqual(control.contact_pitch_deg, 0.0)
        control.send(commander, command_timestamp=1.319, yaw_deg=0.0)
        self.assertTrue(control.update_coast_attitude(
            [0.0, 0.04, 1.0], [0.0, 0.15, 0.0],
            [0.0, 0.20, 1.0], [0.0, 0.0, 0.0], 1.321,
            current_orientation_rpy=np.radians([8.0, 0.0, 0.0]),
            command_timestamp=1.321,
        ))
        self.assertEqual(
            control.coast_handoff_reason,
            'timed_level_to_position_handoff',
        )

    def test_coast_can_handoff_directly_to_current_position(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_level_handoff_speed_m_s=0.10,
            coast_level_handoff_delay_s=0.30,
            coast_direct_position_handoff=True,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.30, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)
        control.send(commander, command_timestamp=1.0, yaw_deg=0.0)
        calls_before_handoff = len(commander.calls)

        self.assertTrue(control.update_coast_attitude(
            [0.02, 0.08, 1.01], [0.01, 0.099, 0.0],
            [0.0, 0.20, 1.0], [0.0, 0.0, 0.0], 1.02,
            current_orientation_rpy=np.radians([8.0, 0.0, 0.0]),
            command_timestamp=1.02,
        ))
        self.assertEqual(control.command_mode, 'position_hold')
        self.assertEqual(
            control.coast_handoff_reason,
            'direct_current_position_handoff',
        )
        np.testing.assert_allclose(
            control.hold_position, [0.02, 0.08, 1.01]
        )
        self.assertTrue(control.coast_target_clamped_to_actual)
        self.assertEqual(len(commander.calls), calls_before_handoff)

        control.send(commander)
        self.assertEqual(commander.calls[-1][0], 'position')

    def test_velocity_coast_commands_zero_then_handoffs_below_point_one(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_handoff_speed_m_s=0.10,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.30, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)
        self.assertEqual(control.command_mode, 'velocity_coast')

        control.send(commander, command_timestamp=1.0)
        self.assertEqual(commander.calls[-1][0], 'hover')
        np.testing.assert_allclose(commander.calls[-1][1], [0.0, 0.0, 0.0, 1.0])

        self.assertFalse(control.update_coast_velocity(
            [0.01, 0.10, 1.0], [0.06, 0.09, 0.0], 1.01,
            current_orientation_rpy=np.zeros(3),
        ))
        self.assertEqual(control.command_mode, 'velocity_coast')

        self.assertTrue(control.update_coast_velocity(
            [0.02, 0.12, 1.01], [0.05, 0.08, 0.0], 1.02,
            current_orientation_rpy=np.zeros(3),
        ))
        self.assertEqual(control.command_mode, 'position_hold')
        self.assertEqual(
            control.coast_handoff_reason,
            'velocity_zero_position_handoff',
        )
        np.testing.assert_allclose(
            control.hold_position, [0.02, 0.12, 1.01]
        )
        control.send(commander)
        self.assertEqual(commander.calls[-1][0], 'position')

    def test_predictive_velocity_coast_unwinds_before_level_handoff(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_velocity_handoff_speed_m_s=0.03,
            coast_velocity_unwind_terminal_speed_m_s=0.02,
            coast_velocity_unwind_prediction_margin_s=0.03,
            coast_velocity_handoff_min_projected_speed_m_s=-0.03,
            coast_velocity_handoff_max_rate_deg_s=5.0,
            coast_handoff_max_tilt_deg=0.5,
            coast_alignment_dwell_s=0.08,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)
        control.send(commander, command_timestamp=1.0)
        np.testing.assert_allclose(
            commander.calls[-1][1], [0.0, 0.0, 0.0, 1.0]
        )

        # A strong measured braking attitude predicts that leveling now will
        # consume the remaining forward speed, so the velocity target changes
        # from zero to the measured velocity before the zero crossing.
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.10, 1.0], [0.0, 0.60, 0.0], 1.05,
            current_orientation_rpy=np.radians([20.0, 0.0, 0.0]),
            current_angular_velocity=np.zeros(3),
        ))
        self.assertEqual(control.coast_velocity_phase, 'predictive_unwind')
        self.assertLessEqual(
            control.coast_velocity_predicted_unwind_terminal_speed_m_s,
            control.coast_velocity_unwind_terminal_speed_m_s,
        )
        self.assertTrue(control.consume_velocity_pid_reset_request())
        self.assertFalse(control.consume_velocity_pid_reset_request())
        control.send(commander, command_timestamp=1.05)
        np.testing.assert_allclose(
            commander.calls[-1][1], [0.0, 0.60, 0.0, 1.0]
        )

        # Low speed alone is not sufficient while measured tilt is too large.
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.14, 1.0], [0.0, 0.025, 0.0], 1.10,
            current_orientation_rpy=np.radians([1.0, 0.0, 0.0]),
            current_angular_velocity=np.zeros(3),
        ))
        self.assertFalse(control.coast_velocity_handoff_tilt_ready)

        # A level vehicle still cannot hand off while angular rate is high.
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.15, 1.0], [0.0, 0.025, 0.0], 1.20,
            current_orientation_rpy=np.radians([0.3, 0.0, 0.0]),
            current_angular_velocity=np.radians([6.0, 0.0, 0.0]),
        ))
        self.assertFalse(control.coast_velocity_handoff_rate_ready)

        # Once all three gates are satisfied, require a continuous 80 ms
        # dwell before position control latches the measured pose.
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.155, 1.0], [0.0, 0.025, 0.0], 1.30,
            current_orientation_rpy=np.radians([0.3, 0.0, 0.0]),
            current_angular_velocity=np.radians([3.0, 0.0, 0.0]),
        ))
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.157, 1.0], [0.0, 0.024, 0.0], 1.37,
            current_orientation_rpy=np.radians([0.2, 0.0, 0.0]),
            current_angular_velocity=np.radians([2.0, 0.0, 0.0]),
        ))
        self.assertTrue(control.update_coast_velocity(
            [0.0, 0.158, 1.0], [0.0, 0.023, 0.0], 1.39,
            current_orientation_rpy=np.radians([0.2, 0.0, 0.0]),
            current_angular_velocity=np.radians([2.0, 0.0, 0.0]),
        ))
        self.assertEqual(
            control.coast_handoff_reason,
            'velocity_predictive_unwind_position_handoff',
        )
        np.testing.assert_allclose(
            control.hold_position, [0.0, 0.158, 1.0]
        )
        control.send(commander)
        self.assertEqual(commander.calls[-1][0], 'position')

    def test_integrated_leveling_tail_uses_rate_limited_attitude_ramp(self):
        prediction = integrate_rate_limited_leveling_velocity_delta(
            np.radians([14.4, 0.0, 0.0]),
            np.zeros(3),
            [0.0, 1.0],
            response_delay_s=0.0,
            leveling_rate_deg_s=720.0,
            integration_step_s=0.01,
        )

        # 14.4 degrees takes exactly two 7.2-degree steps to reach level.
        self.assertAlmostEqual(prediction['duration_s'], 0.02)
        expected_delta = 0.005 * (
            attitude_to_world_acceleration(14.4, 0.0, 0.0)[1]
            + 2.0 * attitude_to_world_acceleration(7.2, 0.0, 0.0)[1]
        )
        self.assertAlmostEqual(
            prediction['velocity_delta_m_s'], expected_delta
        )
        self.assertAlmostEqual(
            prediction['final_projected_acceleration_m_s2'], 0.0
        )

    def test_predictive_unwind_adds_live_state_to_decision_latency(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_velocity_unwind_integrated_leveling_enabled=True,
            coast_velocity_unwind_leveling_rate_deg_s=100.0,
            coast_attitude_response_delay_s=0.07,
            coast_velocity_unwind_command_switch_delay_s=0.03,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 1.0, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)

        orientation = np.radians([14.4, 0.0, 0.0])
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.01, 1.0], [0.0, 1.0, 0.0], 1.01,
            current_orientation_rpy=orientation,
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.022,
        ))
        expected = integrate_rate_limited_leveling_velocity_delta(
            orientation,
            np.zeros(3),
            [0.0, 1.0],
            response_delay_s=0.112,
            leveling_rate_deg_s=100.0,
            integration_step_s=0.01,
        )
        self.assertAlmostEqual(
            control.coast_velocity_unwind_observed_decision_latency_s,
            0.012,
        )
        self.assertAlmostEqual(
            control.coast_velocity_unwind_command_switch_delay_s,
            0.03,
        )
        self.assertAlmostEqual(
            control.coast_velocity_unwind_total_response_delay_s,
            0.112,
        )
        self.assertAlmostEqual(
            control.coast_velocity_unwind_integrated_velocity_delta_m_s,
            expected['velocity_delta_m_s'],
        )
        self.assertAlmostEqual(
            control.coast_velocity_unwind_response_horizon_s,
            expected['duration_s'],
        )

    def test_real_tail_calibration_unwinds_before_logged_reverse_case(self):
        direction = np.array([-0.0330945357, 0.9994522258, 0.0])
        orientation = np.array([
            0.1390151197, -0.0224977337, 0.0308844128,
        ])
        angular_velocity = np.array([1.762, -0.071, -0.175])
        velocity = np.array([
            -0.0418995507, 0.9507858157, -0.0472133756,
        ])

        def evaluate(tail_scale):
            control = TranslationControlHandoff(
                initial_position=[0.0, 0.0, 1.0],
                yaw_deg=0.0,
                shadow_mode=False,
                brake_max_attitude_deg=20.0,
                coast_attitude_acceleration_scale=1.038742,
                coast_velocity_braking_enabled=True,
                coast_velocity_predictive_unwind_enabled=True,
                coast_velocity_unwind_terminal_speed_m_s=0.10,
                coast_velocity_unwind_integrated_leveling_enabled=True,
                coast_velocity_unwind_tail_calibration_scale=tail_scale,
                coast_velocity_unwind_leveling_rate_deg_s=100.0,
                coast_velocity_unwind_integration_step_s=0.01,
                coast_velocity_unwind_one_step_lookahead_enabled=True,
                coast_velocity_unwind_one_step_max_dt_s=0.01,
                coast_attitude_response_delay_s=0.07,
                coast_velocity_unwind_command_switch_delay_s=0.03,
            )
            self.assertTrue(control.start_contact('orientation'))
            self.assertTrue(control.end_contact(
                [0.0015076570, -0.6077401042, 1.0182486773],
                [-0.0562, 0.7223, 0.0082],
                1.0,
                interaction_direction=direction,
                coast=True,
            ))
            control.confirm_release_candidate(timestamp=1.0)
            self.assertFalse(control.update_coast_velocity(
                [-0.0110512525, -0.3728695810, 1.0292502642],
                velocity,
                1.01,
                current_orientation_rpy=orientation,
                current_angular_velocity=angular_velocity,
                command_timestamp=1.0184571838,
            ))
            return control

        uncalibrated = evaluate(1.0)
        calibrated = evaluate(1.60)

        self.assertEqual(uncalibrated.coast_velocity_phase, 'fast_brake')
        self.assertEqual(calibrated.coast_velocity_phase, 'predictive_unwind')
        self.assertGreater(
            uncalibrated.coast_velocity_predicted_unwind_terminal_speed_m_s,
            uncalibrated.coast_velocity_dynamic_unwind_threshold_m_s,
        )
        self.assertLessEqual(
            calibrated.coast_velocity_predicted_unwind_terminal_speed_m_s,
            calibrated.coast_velocity_dynamic_unwind_threshold_m_s,
        )
        self.assertAlmostEqual(
            calibrated.coast_velocity_unwind_raw_integrated_velocity_delta_m_s,
            uncalibrated.coast_velocity_unwind_integrated_velocity_delta_m_s,
        )
        self.assertAlmostEqual(
            calibrated.coast_velocity_unwind_integrated_velocity_delta_m_s,
            1.60 * (
                calibrated
                .coast_velocity_unwind_raw_integrated_velocity_delta_m_s
            ),
        )

    def test_virtual_friction_sets_different_unwind_times_from_release_state(self):
        direction = np.array([-0.0487148353, 0.9988127276, 0.0])
        virtual_release_velocity = np.array([
            -0.1709956886, 0.4751973021, 0.0,
        ])
        current_position = np.array([
            -0.2039984614, -0.2242031544, 1.0191417933,
        ])
        current_velocity = np.array([
            -0.2027536631, 0.4741640687, 0.0348397717,
        ])
        current_orientation = np.array([
            -0.0636556545, 0.0458449741, 0.0215193694,
        ])
        current_angular_velocity = np.array([1.02, -0.438, -0.098])

        def evaluate(mu):
            control = TranslationControlHandoff(
                initial_position=[0.0, 0.0, 1.0],
                yaw_deg=0.0,
                shadow_mode=False,
                brake_max_attitude_deg=30.0,
                coast_velocity_braking_enabled=True,
                coast_velocity_predictive_unwind_enabled=True,
                coast_velocity_unwind_virtual_friction_target_enabled=True,
                coast_velocity_unwind_terminal_speed_m_s=0.10,
                coast_velocity_unwind_integrated_leveling_enabled=True,
                coast_velocity_unwind_tail_calibration_scale=1.60,
                coast_velocity_unwind_leveling_rate_deg_s=100.0,
                coast_velocity_unwind_integration_step_s=0.01,
                coast_velocity_unwind_one_step_lookahead_enabled=True,
                coast_velocity_unwind_one_step_max_dt_s=0.20,
                coast_attitude_response_delay_s=0.12,
                coast_velocity_unwind_command_switch_delay_s=0.03,
            )
            self.assertTrue(control.start_contact('orientation'))
            self.assertTrue(control.end_contact(
                [-0.1915758252, -0.2718708515, 1.0121239424],
                [-0.1642066985, 0.3195418417, 0.0728405491],
                1.0,
                interaction_direction=direction,
                virtual_release_velocity=virtual_release_velocity,
                virtual_kinetic_friction_coefficient=mu,
                coast=True,
            ))
            control.confirm_release_candidate(timestamp=1.0)
            self.assertFalse(control.update_coast_velocity(
                current_position,
                current_velocity,
                1.0888841,
                current_orientation_rpy=current_orientation,
                current_angular_velocity=current_angular_velocity,
                command_timestamp=1.0920331,
            ))
            return control

        low_friction = evaluate(0.01)
        high_friction = evaluate(0.10)

        self.assertEqual(
            low_friction.coast_velocity_phase, 'predictive_unwind'
        )
        self.assertEqual(high_friction.coast_velocity_phase, 'fast_brake')
        self.assertEqual(
            low_friction.coast_velocity_unwind_decision_reason,
            'virtual_friction_tail_prediction',
        )
        self.assertGreater(
            low_friction.coast_velocity_effective_unwind_terminal_speed_m_s,
            high_friction.coast_velocity_effective_unwind_terminal_speed_m_s,
        )
        self.assertAlmostEqual(
            low_friction.coast_velocity_virtual_friction_deceleration_m_s2,
            0.0981,
        )
        self.assertAlmostEqual(
            high_friction.coast_velocity_virtual_friction_deceleration_m_s2,
            0.981,
        )

    def test_virtual_friction_unwind_tracks_forward_speed_and_damps_lateral(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_velocity_unwind_virtual_friction_target_enabled=True,
            coast_velocity_unwind_low_speed_fallback_m_s=1.0,
            coast_velocity_handoff_speed_m_s=0.09,
            coast_velocity_rebrake_enabled=False,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.12, 0.40, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            virtual_release_velocity=[0.0, 0.40, 0.0],
            virtual_kinetic_friction_coefficient=0.01,
            coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.01, 1.0], [0.12, 0.40, 0.0], 1.01,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            virtual_target_velocity=[0.0, 0.35, 0.0],
        ))
        self.assertEqual(control.coast_velocity_phase, 'predictive_unwind')
        np.testing.assert_allclose(
            control.coast_velocity_command_xy_m_s,
            [0.115, 0.39],
            atol=1e-12,
        )
        np.testing.assert_allclose(
            control.coast_velocity_unwind_command_correction_xy_m_s,
            [-0.005, -0.01],
            atol=1e-12,
        )
        self.assertLess(
            control.coast_velocity_unwind_terminal_speed_error_m_s, 0.0
        )
        self.assertAlmostEqual(
            control.coast_velocity_virtual_unwind_target_speed_m_s,
            0.364684,
        )
        self.assertEqual(
            control.coast_tracking_action,
            'track_virtual_friction_terminal_speed_continuously',
        )

    def test_virtual_friction_unwind_uses_roll_tail_to_prevent_reversal(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_velocity_unwind_virtual_friction_target_enabled=True,
            coast_velocity_unwind_low_speed_fallback_m_s=1.0,
            coast_velocity_handoff_speed_m_s=0.09,
            coast_velocity_handoff_min_projected_speed_m_s=0.0,
            coast_velocity_rebrake_enabled=False,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.62, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            virtual_release_velocity=[0.0, 0.586, 0.0],
            virtual_kinetic_friction_coefficient=0.10,
            coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)

        # Reproduce the first flight's first zero crossing: measured speed is
        # almost zero but +roll still produces acceleration opposite +Y. The
        # old target<=measured clamp sent another near-zero target and allowed
        # the vehicle to reach about -0.16 m/s. The live roll-tail prediction
        # must instead request a bounded positive velocity error to unload the
        # braking attitude before crossing zero.
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.20, 1.0], [0.0, 0.002, 0.0], 1.70,
            current_orientation_rpy=np.radians([2.13, 0.0, 0.0]),
            current_angular_velocity=np.zeros(3),
            virtual_target_velocity=[0.0, 0.011, 0.0],
        ))
        self.assertEqual(control.coast_velocity_phase, 'predictive_unwind')
        self.assertLess(
            control.coast_velocity_predicted_unwind_terminal_speed_m_s, 0.0
        )
        self.assertEqual(
            control.coast_velocity_unwind_tail_target_speed_m_s, 0.0
        )
        self.assertGreater(
            control.coast_velocity_unwind_tail_compensation_m_s, 0.0
        )
        self.assertGreater(
            control.coast_velocity_unwind_terminal_speed_error_m_s, 0.0
        )
        self.assertGreater(
            float(control.coast_velocity_command_xy_m_s[1]), 0.002
        )
        self.assertLessEqual(
            float(control.coast_velocity_command_xy_m_s[1]), 0.152
        )
        self.assertEqual(
            control.coast_tracking_action,
            'track_virtual_friction_terminal_speed_continuously',
        )
        self.assertFalse(
            control.coast_velocity_handoff_predicted_speed_ready
        )

    def test_virtual_friction_tail_error_is_continuous_across_zero(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_velocity_unwind_virtual_friction_target_enabled=True,
            coast_velocity_unwind_low_speed_fallback_m_s=1.0,
            coast_velocity_handoff_speed_m_s=0.09,
            coast_velocity_rebrake_enabled=False,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.20, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)

        signed_errors = []
        compensations = []
        projected_commands = []
        corrections = []
        for timestamp, roll_deg in (
            (1.01, 1.0),
            (1.02, 0.0),
            (1.03, -1.0),
        ):
            self.assertFalse(control.update_coast_velocity(
                [0.0, 0.20 * (timestamp - 1.0), 1.0],
                [0.0, 0.20, 0.0],
                timestamp,
                current_orientation_rpy=np.radians([roll_deg, 0.0, 0.0]),
                current_angular_velocity=np.zeros(3),
                virtual_target_velocity=[0.0, 0.20, 0.0],
            ))
            signed_errors.append(
                control.coast_velocity_unwind_terminal_speed_error_m_s
            )
            compensations.append(
                control.coast_velocity_unwind_tail_compensation_m_s
            )
            projected_commands.append(
                float(control.coast_velocity_command_xy_m_s[1])
            )
            corrections.append(
                float(
                    control
                    .coast_velocity_unwind_command_correction_xy_m_s[1]
                )
            )

        self.assertGreater(signed_errors[0], 0.0)
        self.assertAlmostEqual(signed_errors[1], 0.0)
        self.assertLess(signed_errors[2], 0.0)
        self.assertGreater(compensations[0], 0.0)
        self.assertEqual(compensations[1:], [0.0, 0.0])
        self.assertLessEqual(corrections[0], 2.0 * 0.01 + 1e-12)
        for previous, current in zip(corrections, corrections[1:]):
            self.assertGreaterEqual(
                current - previous,
                -1.0 * 0.01 - 1e-12,
            )
            self.assertLessEqual(
                current - previous,
                2.0 * 0.01 + 1e-12,
            )
        self.assertLess(
            max(abs(current - previous) for previous, current in zip(
                projected_commands, projected_commands[1:]
            )),
            0.011,
        )

    def test_virtual_friction_longitudinal_saturation_keeps_lateral_damping(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_velocity_unwind_virtual_friction_target_enabled=True,
            coast_velocity_unwind_low_speed_fallback_m_s=1.0,
            coast_velocity_handoff_speed_m_s=0.09,
            coast_velocity_rebrake_enabled=False,
            coast_velocity_unwind_filter_time_constant_s=1e-6,
            coast_velocity_unwind_brake_slew_rate_m_s2=100.0,
            coast_velocity_unwind_lateral_slew_rate_m_s2=100.0,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.12, 0.40, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.01, 1.0], [0.12, 0.40, 0.0], 1.01,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            virtual_target_velocity=[0.0, 0.10, 0.0],
        ))
        np.testing.assert_allclose(
            control.coast_velocity_unwind_command_correction_xy_m_s,
            [-0.05, -0.15],
            atol=1e-12,
        )
        np.testing.assert_allclose(
            control.coast_velocity_command_xy_m_s,
            [0.07, 0.25],
            atol=1e-12,
        )

    def test_virtual_friction_unwind_floors_reverse_target_after_crossing(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_velocity_unwind_virtual_friction_target_enabled=True,
            coast_velocity_unwind_low_speed_fallback_m_s=1.0,
            coast_velocity_handoff_speed_m_s=0.09,
            coast_velocity_rebrake_enabled=False,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.20, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)

        # Build a negative braking correction first. On the next sample the
        # measured velocity crosses zero; the non-reverse floor deliberately
        # takes priority over the normal positive correction slew bound.
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.004, 1.0], [0.0, 0.40, 0.0], 1.01,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            virtual_target_velocity=[0.0, 0.10, 0.0],
        ))
        previous_correction = float(
            control.coast_velocity_unwind_command_correction_xy_m_s[1]
        )
        self.assertLess(previous_correction, 0.0)

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0038, 1.0], [0.0, -0.019, 0.0], 1.02,
            current_orientation_rpy=np.radians([-2.0, 0.0, 0.0]),
            current_angular_velocity=np.zeros(3),
            virtual_target_velocity=[0.0, 0.0, 0.0],
        ))
        self.assertLess(
            control.coast_velocity_unwind_terminal_speed_error_m_s, 0.0
        )
        self.assertGreaterEqual(
            float(control.coast_velocity_command_xy_m_s[1]), 0.0
        )
        self.assertAlmostEqual(
            float(control.coast_velocity_command_xy_m_s[1]), 0.0
        )
        self.assertTrue(
            control.coast_velocity_unwind_nonreverse_floor_applied
        )
        self.assertGreater(
            float(control.coast_velocity_unwind_command_correction_xy_m_s[1])
            - previous_correction,
            control.coast_velocity_unwind_debrake_slew_rate_m_s2 * 0.01,
        )

    def test_virtual_friction_unwind_does_not_add_brake_near_zero(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_velocity_unwind_virtual_friction_target_enabled=True,
            coast_velocity_unwind_low_speed_fallback_m_s=1.0,
            coast_velocity_handoff_speed_m_s=0.09,
            coast_velocity_rebrake_enabled=False,
            coast_attitude_response_delay_s=0.0,
            coast_attitude_time_constant_s=0.0,
            coast_velocity_unwind_prediction_margin_s=0.0,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.08, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.001, 1.0], [0.0, 0.08, 0.0], 1.01,
            current_orientation_rpy=np.radians([2.0, 0.0, 0.0]),
            current_angular_velocity=np.zeros(3),
            virtual_target_velocity=[0.0, 0.0, 0.0],
        ))
        self.assertLess(
            control.coast_velocity_unwind_terminal_speed_error_m_s, 0.0
        )
        self.assertTrue(
            control.coast_velocity_unwind_near_zero_brake_suppressed
        )
        self.assertAlmostEqual(
            float(
                control.coast_velocity_unwind_command_correction_xy_m_s[1]
            ),
            0.0,
        )
        self.assertAlmostEqual(
            float(control.coast_velocity_command_xy_m_s[1]), 0.08
        )

    def test_predictive_unwind_can_send_direct_level_attitude_at_fixed_z(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_velocity_unwind_direct_level_attitude_enabled=True,
            coast_velocity_unwind_low_speed_fallback_m_s=1.0,
            coast_velocity_handoff_speed_m_s=0.03,
            coast_velocity_handoff_position_offset_m=0.12,
            coast_alignment_dwell_s=0.0,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.18], [0.0, 0.40, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)

        self.assertFalse(control.update_coast_velocity(
            [0.08, 0.20, 1.12], [0.0, 0.40, 0.0], 1.01,
            current_orientation_rpy=np.radians([8.0, 0.0, 0.0]),
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.02,
        ))
        self.assertEqual(control.coast_velocity_phase, 'predictive_unwind')
        self.assertTrue(control.direct_level_unwind_active)
        self.assertFalse(control.uses_position_setpoint)
        self.assertEqual(
            control.command_mode, 'predictive_unwind_attitude_zdistance'
        )
        self.assertEqual(
            control.coast_tracking_action, 'direct_level_attitude_unwind'
        )
        self.assertIsNone(control.coast_velocity_unwind_position_target_m)
        self.assertTrue(control.consume_velocity_pid_reset_request())

        control.send(commander, command_timestamp=1.02, yaw_deg=3.0)
        self.assertEqual(commander.calls[-1][0], 'zdistance')
        np.testing.assert_allclose(
            commander.calls[-1][1], [0.0, 0.0, 0.0, 1.0]
        )
        sent = control.sent_command_snapshot()
        self.assertEqual(sent['kind'], 'attitude_zdistance')
        self.assertEqual(sent['roll_deg'], 0.0)
        self.assertEqual(sent['pitch_deg'], 0.0)
        self.assertEqual(sent['zdistance_m'], 1.0)

        # Position control is granted only after the measured state has
        # settled. Its target starts at the measured handoff pose and advances
        # 12 cm along the locked interaction direction, without lateral pull.
        self.assertTrue(control.update_coast_velocity(
            [0.07, 0.24, 1.06], [0.0, 0.02, 0.0], 1.12,
            current_orientation_rpy=np.radians([0.2, 0.0, 0.0]),
            current_angular_velocity=np.radians([2.0, 0.0, 0.0]),
        ))
        self.assertEqual(
            control.coast_handoff_reason,
            'velocity_predictive_unwind_attitude_handoff',
        )
        np.testing.assert_allclose(
            control.coast_handoff_actual_position_m,
            [0.07, 0.24, 1.06],
        )
        np.testing.assert_allclose(control.hold_position, [0.07, 0.36, 1.06])
        self.assertFalse(control.coast_target_clamped_to_actual)
        self.assertTrue(control.coast_lateral_target_latched_to_actual)
        control.send(commander, command_timestamp=1.12)
        self.assertEqual(commander.calls[-1][0], 'position')
        np.testing.assert_allclose(
            commander.calls[-1][1], [0.07, 0.36, 1.06, 0.0]
        )

    def test_world_acceleration_to_attitude_round_trips_at_rotated_yaw(self):
        for yaw_deg in (0.0, 90.0, -135.0):
            for acceleration in (
                np.array([0.6, 0.0]),
                np.array([-0.6, 0.0]),
                np.array([0.0, 0.6]),
                np.array([0.0, -0.6]),
            ):
                attitude = world_acceleration_to_attitude(
                    acceleration, yaw_deg
                )
                recovered = attitude_to_world_acceleration(
                    attitude['roll_deg'], attitude['pitch_deg'], yaw_deg
                )
                np.testing.assert_allclose(
                    recovered, acceleration, atol=1e-12
                )
        y_brake = world_acceleration_to_attitude([0.0, -0.6], 0.0)
        x_brake = world_acceleration_to_attitude([-0.6, 0.0], 0.0)
        self.assertGreater(y_brake['roll_deg'], 0.0)
        self.assertGreater(x_brake['pitch_deg'], 0.0)

    def test_command_history_reconstructs_delayed_acceleration_and_tail(self):
        direction = np.array([0.0, 1.0])
        cases = (
            (
                [(-1.0, [0.0, -0.6]), (-0.05, [0.0, 0.0])],
                -0.6,
                -0.108,
            ),
            (
                [(-1.0, [0.0, 0.0]), (-0.05, [0.0, -0.6])],
                0.0,
                -0.030,
            ),
        )
        for history, expected_acceleration, expected_tail in cases:
            with self.subTest(history=history):
                acceleration = reconstruct_delayed_first_order_acceleration(
                    history,
                    timestamp=0.0,
                    response_delay_s=0.15,
                    response_time_constant_s=0.08,
                )
                self.assertAlmostEqual(
                    float(acceleration @ direction),
                    expected_acceleration,
                    places=6,
                )
                prediction = predict_delayed_zero_crossing(
                    [0.0, 0.60],
                    acceleration,
                    direction,
                    history,
                    timestamp=0.0,
                    response_delay_s=0.15,
                    response_time_constant_s=0.08,
                    acceleration_scale=1.0,
                    future_command_acceleration_xy=np.zeros(2),
                    future_command_started_at=0.0,
                    continue_after_crossing=True,
                    step_s=0.001,
                    horizon_s=4.0,
                )
                self.assertAlmostEqual(
                    prediction['final_speed_m_s'] - 0.60,
                    expected_tail,
                    places=6,
                )

    def test_jerk_profile_uses_modeled_queue_state_not_last_command(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_extra_command_delay_s=0.03,
            coast_velocity_unwind_tail_calibration_scale=1.0,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 0.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        # Level was sent 50 ms ago, but the 150 ms response queue still holds
        # the preceding -0.6 m/s^2 command at the release state.
        control._coast_command_history = [
            (-1.0, np.array([0.0, -0.6])),
            (-0.05, np.zeros(2)),
        ]
        control.confirm_release_candidate(
            timestamp=0.0,
            current_velocity=[0.0, 0.60, 0.0],
        )
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 0.0,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=0.0,
        ))

        self.assertAlmostEqual(
            control.coast_jerk_limited_initial_command_acceleration_m_s2,
            0.0,
            places=6,
        )
        self.assertAlmostEqual(
            control.coast_jerk_limited_modeled_initial_acceleration_m_s2,
            -0.6,
            places=6,
        )
        self.assertAlmostEqual(
            control.coast_jerk_limited_raw_level_tail_delta_m_s,
            -0.108,
            places=6,
        )

    def test_jerk_profile_rejects_tail_uncertainty_above_speed_band(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_extra_command_delay_s=0.03,
            coast_velocity_unwind_tail_calibration_scale=1.6,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 0.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control._coast_command_history = [
            (-1.0, np.array([0.0, -0.6])),
            (-0.05, np.zeros(2)),
        ]
        control.confirm_release_candidate(
            timestamp=0.0,
            current_velocity=[0.0, 0.60, 0.0],
        )
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 0.0,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=0.0,
        ))

        self.assertTrue(
            control.coast_jerk_limited_velocity_fallback_active
        )
        self.assertEqual(control.coast_velocity_phase, 'fast_brake')
        self.assertAlmostEqual(
            control.coast_jerk_limited_nominal_terminal_speed_m_s,
            0.0698,
            places=4,
        )
        self.assertIn(
            'tail uncertainty', control.coast_jerk_limited_fallback_reason
        )

    def test_jerk_profile_rejects_queued_tail_that_crosses_zero(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_extra_command_delay_s=0.03,
            coast_velocity_unwind_tail_calibration_scale=1.6,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.03, 0.0], 0.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control._coast_command_history = [
            (-1.0, np.array([0.0, -1.0])),
            (-0.1097347, np.array([0.0, 1.0])),
        ]
        control.confirm_release_candidate(
            timestamp=0.0,
            current_velocity=[0.0, 0.03, 0.0],
        )
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.03, 0.0], 0.0,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=0.0,
        ))

        self.assertTrue(control.coast_jerk_limited_level_tail_crossed_zero)
        self.assertIsNone(control.coast_jerk_limited_profile)
        self.assertTrue(
            control.coast_jerk_limited_velocity_fallback_active
        )
        self.assertIn('crosses zero', control.coast_jerk_limited_fallback_reason)

    def test_jerk_profile_rejects_mixed_sign_scaled_tail_transient(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_attitude_response_delay_s=0.15,
            coast_attitude_time_constant_s=0.0,
            coast_attitude_acceleration_scale=1.0,
            coast_velocity_unwind_tail_calibration_scale=1.6,
            coast_command_period_s=0.02,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.0875, 0.0], 0.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control._coast_command_history = [
            (-1.00, np.array([0.0, 0.0])),
            (-0.14, np.array([0.0, -1.0])),
            (-0.10, np.array([0.0, -1.0])),
            (-0.06, np.array([0.0, 1.0])),
        ]
        control.confirm_release_candidate(
            timestamp=0.0,
            current_velocity=[0.0, 0.0875, 0.0],
        )

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.0875, 0.0], 0.0,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=0.0,
        ))

        self.assertGreater(
            control.coast_jerk_limited_validation_nominal_min_speed_m_s,
            0.0,
        )
        self.assertLess(
            control.coast_jerk_limited_validation_robust_min_speed_m_s,
            -control.coast_jerk_limited_terminal_speed_margin_m_s,
        )
        self.assertTrue(
            control.coast_jerk_limited_velocity_fallback_active
        )
        self.assertIn(
            'robust terminal speed envelope',
            control.coast_jerk_limited_fallback_reason,
        )

    def test_jerk_profile_rejects_sign_asymmetric_tail_gain(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_attitude_response_delay_s=0.15,
            coast_attitude_time_constant_s=0.0,
            coast_attitude_acceleration_scale=1.0,
            coast_velocity_unwind_tail_calibration_scale=1.6,
            coast_command_period_s=0.02,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.10, 0.0], 0.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        # Uniformly scaling this alternating queue preserves its near-zero
        # net impulse.  The conservative envelope instead allows only the
        # braking lobe to be 1.6x stronger, which must be rejected.
        control._coast_command_history = [
            (-1.00, np.array([0.0, 0.0])),
            (-0.14, np.array([0.0, 1.0])),
            (-0.10, np.array([0.0, -1.0])),
            (-0.06, np.array([0.0, 0.0])),
        ]
        control.confirm_release_candidate(
            timestamp=0.0,
            current_velocity=[0.0, 0.10, 0.0],
        )

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.10, 0.0], 0.0,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=0.0,
        ))

        self.assertGreater(
            control.coast_jerk_limited_validation_nominal_min_speed_m_s,
            0.0,
        )
        self.assertLess(
            control.coast_jerk_limited_validation_robust_min_speed_m_s,
            -control.coast_jerk_limited_terminal_speed_margin_m_s,
        )
        self.assertTrue(
            control.coast_jerk_limited_velocity_fallback_active
        )
        self.assertIn(
            'robust terminal speed envelope',
            control.coast_jerk_limited_fallback_reason,
        )

    def test_jerk_profile_reserves_phase_asymmetric_command_hold_impulse(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_attitude_response_delay_s=0.0,
            coast_attitude_time_constant_s=0.0,
            coast_attitude_acceleration_scale=1.0,
            coast_velocity_unwind_tail_calibration_scale=1.0,
            coast_command_period_s=0.01,
            coast_jerk_limited_activation_guard_s=0.01,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.405, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control._coast_command_history = [(0.0, np.zeros(2))]
        # A 20 ms observed gap plus the 10 ms guard validates a 30 ms hold.
        # The worst admissible phase pattern is 10 ms while acceleration grows
        # and 30 ms while it unwinds, not a uniform 30 ms grid.
        control._attitude_send_intervals_s = [0.02]
        control.confirm_release_candidate(
            timestamp=1.0,
            current_velocity=[0.0, 0.405, 0.0],
        )

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.405, 0.0], 1.0,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.0,
        ))

        self.assertTrue(control.coast_jerk_limited_attitude_active)
        self.assertAlmostEqual(
            control.coast_jerk_limited_validation_command_period_s,
            0.03,
        )
        self.assertGreater(
            control.coast_jerk_limited_cadence_impulse_reserve_m_s,
            0.0,
        )
        self.assertLess(control.coast_jerk_limited_velocity_budget_m_s, 0.4)
        self.assertGreaterEqual(
            control.coast_jerk_limited_validation_robust_min_speed_m_s,
            -control.coast_jerk_limited_terminal_speed_margin_m_s - 1e-9,
        )

    def test_jerk_first_send_replans_for_plan_to_send_gap(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_attitude_response_delay_s=0.0,
            coast_attitude_time_constant_s=0.0,
            coast_attitude_acceleration_scale=1.0,
            coast_velocity_unwind_tail_calibration_scale=1.0,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.40, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control._coast_command_history = [
            (0.0, np.array([0.0, -0.6])),
        ]
        control.confirm_release_candidate(
            timestamp=1.0,
            current_velocity=[0.0, 0.40, 0.0],
        )
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.40, 0.0], 1.0,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.0,
        ))
        provisional_budget = control.coast_jerk_limited_velocity_budget_m_s
        self.assertIsNone(control.consume_jerk_limited_event())

        control.send(commander, command_timestamp=1.01, yaw_deg=0.0)

        self.assertEqual(commander.calls[-1][0], 'zdistance')
        self.assertAlmostEqual(
            control.coast_jerk_limited_profile_started_at, 1.01
        )
        self.assertAlmostEqual(
            control.coast_jerk_limited_first_actual_send_at, 1.01
        )
        self.assertLess(
            control.coast_jerk_limited_velocity_budget_m_s,
            provisional_budget,
        )
        self.assertGreaterEqual(
            control.coast_jerk_limited_validation_robust_min_speed_m_s,
            -control.coast_jerk_limited_terminal_speed_margin_m_s - 1e-9,
        )
        event = control.consume_jerk_limited_event()
        self.assertEqual(
            event['event'], 'Jerk-Limited Attitude Brake Started'
        )
        self.assertAlmostEqual(event['actual_first_send_at'], 1.01)

    def test_jerk_first_send_gap_crossing_zero_uses_level_bridge(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_attitude_response_delay_s=0.0,
            coast_attitude_time_constant_s=0.0,
            coast_attitude_acceleration_scale=1.0,
            coast_velocity_unwind_tail_calibration_scale=1.0,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.40, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control._coast_command_history = [
            (0.0, np.array([0.0, -0.6])),
        ]
        control.confirm_release_candidate(
            timestamp=1.0,
            current_velocity=[0.0, 0.40, 0.0],
        )
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.40, 0.0], 1.0,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.0,
        ))

        control.send(commander, command_timestamp=1.80, yaw_deg=0.0)

        self.assertEqual(commander.calls[-1][0], 'zdistance')
        np.testing.assert_allclose(commander.calls[-1][1][:2], [0.0, 0.0])
        self.assertTrue(control.coast_jerk_limited_fallback_bridge_level_sent)
        self.assertTrue(
            control.coast_jerk_limited_velocity_fallback_active
        )
        self.assertTrue(control.consume_velocity_pid_reset_request())
        self.assertIsNone(control.coast_jerk_limited_first_actual_send_at)
        event = control.consume_jerk_limited_event()
        self.assertEqual(event['event'], 'Jerk-Limited Brake Fallback')
        self.assertIn('crosses zero', event['reason'])

    def test_jerk_first_send_fallback_bridge_slews_from_last_actual_command(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_max_attitude_rate_deg_s=720.0,
            coast_command_period_s=0.01,
        )
        self.assertTrue(control.start_contact('orientation'))
        control.set_contact_attitude(30.0, 0.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=0.99, yaw_deg=0.0)

        # Mimic an unsent provisional profile command.  The actual-send replan
        # then fails because its state cache is absent.  The bridge must slew
        # from +30 deg (the last dispatched command), not from this -20 deg
        # provisional value.
        control._transition_mode(control.VELOCITY_COAST)
        control.coast_velocity_phase = 'jerk_profile_pending'
        control.set_contact_attitude(-20.0, 0.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=1.00, yaw_deg=0.0)

        self.assertEqual(commander.calls[-1][0], 'zdistance')
        bridge_roll, bridge_pitch = commander.calls[-1][1][:2]
        self.assertAlmostEqual(bridge_roll, 22.8, places=9)
        self.assertAlmostEqual(bridge_pitch, 0.0, places=9)
        self.assertTrue(control.coast_jerk_limited_fallback_bridge_sent)
        self.assertFalse(
            control.coast_jerk_limited_fallback_bridge_level_sent
        )
        np.testing.assert_allclose(
            control.coast_jerk_limited_fallback_bridge_rp_deg,
            [22.8, 0.0],
        )
        event = control.consume_jerk_limited_event()
        np.testing.assert_allclose(
            event['fallback_bridge_rp_deg'], [22.8, 0.0]
        )
        self.assertAlmostEqual(event['fallback_bridge_dt_s'], 0.01)
        self.assertFalse(event['fallback_bridge_level'])

    def test_jerk_fallback_bridge_never_reuses_previous_episode_attitude(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
        )
        self.assertTrue(control.start_contact('orientation'))
        control.set_contact_attitude(8.0, -3.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=0.5, yaw_deg=0.0)
        control._transition_mode(control.POSITION_HOLD)

        self.assertTrue(control.start_contact('orientation'))
        control._transition_mode(control.VELOCITY_COAST)
        control.coast_velocity_phase = 'jerk_profile_pending'
        control.set_contact_attitude(-20.0, 0.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=1.0, yaw_deg=0.0)

        self.assertEqual(commander.calls[-1][0], 'zdistance')
        np.testing.assert_allclose(commander.calls[-1][1][:2], [0.0, 0.0])
        self.assertTrue(control.coast_jerk_limited_fallback_bridge_sent)
        self.assertTrue(
            control.coast_jerk_limited_fallback_bridge_level_sent
        )

    def test_jerk_profile_cadence_miss_bridges_before_velocity_fallback(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_attitude_response_delay_s=0.0,
            coast_attitude_time_constant_s=0.0,
            coast_attitude_acceleration_scale=1.0,
            coast_velocity_unwind_tail_calibration_scale=1.0,
            coast_command_period_s=0.01,
            coast_jerk_limited_activation_guard_s=0.01,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.40, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control._coast_command_history = [
            (0.0, np.array([0.0, -0.6])),
        ]
        control.confirm_release_candidate(
            timestamp=1.0,
            current_velocity=[0.0, 0.40, 0.0],
        )
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.40, 0.0], 1.0,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.0,
        ))
        control.send(commander, command_timestamp=1.01, yaw_deg=0.0)
        self.assertEqual(
            control.consume_jerk_limited_event()['event'],
            'Jerk-Limited Attitude Brake Started',
        )
        self.assertAlmostEqual(
            control.coast_jerk_limited_validation_command_period_s,
            0.02,
        )

        # The previous command has now been held 21 ms, one millisecond beyond
        # the exact ZOH period used by validation.  Do not sample a later
        # profile point; bridge toward level and leave the velocity PID reset
        # pending for the next main-loop iteration.
        control.send(commander, command_timestamp=1.031, yaw_deg=0.0)

        self.assertEqual(commander.calls[-1][0], 'zdistance')
        self.assertTrue(control.coast_jerk_limited_fallback_bridge_sent)
        self.assertTrue(
            control.coast_jerk_limited_velocity_fallback_active
        )
        self.assertAlmostEqual(
            control.coast_jerk_limited_cadence_hold_s, 0.021
        )
        self.assertAlmostEqual(
            control.coast_jerk_limited_cadence_deadline_miss_s, 0.001
        )
        self.assertTrue(control.consume_velocity_pid_reset_request())
        event = control.consume_jerk_limited_event()
        self.assertEqual(event['event'], 'Jerk-Limited Brake Fallback')
        self.assertIn('cadence', event['reason'])
        self.assertAlmostEqual(event['cadence_hold_s'], 0.021)
        self.assertAlmostEqual(event['cadence_deadline_miss_s'], 0.001)

    def test_jerk_profile_post_call_cadence_miss_sends_emergency_level(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_attitude_response_delay_s=0.0,
            coast_attitude_time_constant_s=0.0,
            coast_attitude_acceleration_scale=1.0,
            coast_velocity_unwind_tail_calibration_scale=1.0,
            coast_command_period_s=0.01,
            coast_jerk_limited_activation_guard_s=0.01,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.40, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control._coast_command_history = [
            (0.0, np.array([0.0, -0.6])),
        ]
        control.confirm_release_candidate(
            timestamp=1.0,
            current_velocity=[0.0, 0.40, 0.0],
        )
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.40, 0.0], 1.0,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.0,
        ))
        control.send(commander, command_timestamp=1.0, yaw_deg=0.0)
        control.consume_jerk_limited_event()

        # Pre-call time is still inside the validated 20 ms hold.  The mocked
        # commander call returns at 50 ms, so the post-call guard must append
        # an immediate level command and keep velocity ownership blocked.
        with patch(
            'Interaction.interactions.time.time',
            side_effect=[1.01, 1.01, 1.05, 1.051],
        ):
            sent_at = control.send(commander, yaw_deg=0.0)

        self.assertAlmostEqual(sent_at, 1.051)
        self.assertEqual(commander.calls[-2][0], 'zdistance')
        self.assertEqual(commander.calls[-1][0], 'zdistance')
        np.testing.assert_allclose(commander.calls[-1][1][:2], [0.0, 0.0])
        self.assertTrue(control.coast_jerk_limited_fallback_bridge_sent)
        self.assertTrue(
            control.coast_jerk_limited_fallback_bridge_level_sent
        )
        self.assertTrue(
            control.coast_jerk_limited_velocity_fallback_active
        )
        self.assertTrue(control.consume_velocity_pid_reset_request())
        snapshot = control.sent_command_snapshot()
        self.assertEqual(snapshot['kind'], 'attitude_zdistance')
        self.assertTrue(snapshot['emergency_after_late_jerk_send'])
        self.assertAlmostEqual(snapshot['sent_at'], 1.051)
        event = control.consume_jerk_limited_event()
        self.assertEqual(event['miss_origin'], 'profile_send_completion')
        self.assertAlmostEqual(event['cadence_hold_s'], 0.05)
        self.assertAlmostEqual(event['cadence_deadline_miss_s'], 0.03)
        self.assertAlmostEqual(event['fallback_bridge_completed_at'], 1.051)

    def test_jerk_limited_brake_sends_direct_attitude_at_fixed_nominal_z(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_jerk_limited_level_hold_s=0.05,
            coast_jerk_limited_settle_timeout_s=5.0,
            coast_velocity_handoff_speed_m_s=0.09,
            coast_handoff_max_tilt_deg=1.5,
            coast_alignment_dwell_s=0.02,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.14], [0.0, 0.60, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control._coast_command_history = [(0.0, np.zeros(2))]
        control.confirm_release_candidate(timestamp=1.0)

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.14], [0.0, 0.60, 0.0], 1.0,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.0,
        ))
        self.assertEqual(control.coast_velocity_phase, 'jerk_attitude_brake')
        self.assertTrue(control.coast_jerk_limited_attitude_active)
        self.assertEqual(
            control.command_mode, 'jerk_limited_attitude_zdistance'
        )
        control.send(commander, command_timestamp=1.0, yaw_deg=0.0)
        self.assertEqual(commander.calls[-1][0], 'zdistance')
        self.assertEqual(commander.calls[-1][1][3], 1.0)
        self.assertEqual(
            control.coast_jerk_limited_first_actual_send_at, 1.0
        )

        self.assertFalse(control.update_coast_velocity(
            [0.10, 0.05, 1.10], [0.0, 0.55, 0.0], 1.01,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.01,
        ))
        # +Y release with yaw=0 brakes with positive roll. A +X cross-track
        # error independently requests negative world-X acceleration.
        self.assertGreater(control.contact_roll_deg, 0.0)
        self.assertLess(
            control.coast_jerk_limited_command_acceleration_xy_m_s2[0], 0.0
        )
        self.assertLess(
            control.coast_jerk_limited_command_acceleration_xy_m_s2[1], 0.0
        )
        control.send(commander, command_timestamp=1.01, yaw_deg=0.0)
        self.assertEqual(commander.calls[-1][0], 'zdistance')
        self.assertEqual(commander.calls[-1][1][3], 1.0)
        self.assertEqual(control.brake_direction_source, 'locked_interaction_direction')
        np.testing.assert_allclose(control.brake_direction, [0.0, 1.0, 0.0])

    def test_jerk_limited_brake_waits_for_terminal_gates_then_handoffs(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_jerk_limited_level_hold_s=0.05,
            coast_jerk_limited_settle_timeout_s=5.0,
            coast_velocity_handoff_speed_m_s=0.09,
            coast_velocity_handoff_max_rate_deg_s=5.0,
            coast_handoff_max_tilt_deg=1.5,
            coast_handoff_max_acceleration_m_s2=0.35,
            coast_alignment_dwell_s=0.02,
        )
        terminal_quality = {
            'current_state_group_skew_s': 0.0,
            'max_terminal_state_age_s': 0.10,
            'max_terminal_state_group_skew_s': 0.03,
        }
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control._coast_command_history = [(0.0, np.zeros(2))]
        control.confirm_release_candidate(timestamp=1.0)
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 1.0,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.0,
            **terminal_quality,
        ))
        control.send(commander, command_timestamp=1.0, yaw_deg=0.0)
        terminal_time = 1.0 + control.coast_jerk_limited_profile.duration_s
        # This test isolates measured terminal gates and intentionally jumps
        # the analytic clock to profile completion.  Command-cadence failure
        # is exercised separately with actual per-send timestamps.
        control.coast_jerk_limited_validation_command_period_s = (
            control.coast_jerk_limited_profile.duration_s + 0.20
        )

        # A completed mathematical profile cannot bypass the actual-send
        # response window or gyro-rate gate. The EKF tilt is deliberately bad:
        # jerk handoff must log it without giving it control authority.
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.30, 1.0], [0.0, 0.02, 0.0], terminal_time,
            current_orientation_rpy=np.radians([20.0, -10.0, 0.0]),
            current_angular_velocity=np.radians([6.0, 0.0, 0.0]),
            command_timestamp=terminal_time,
            **terminal_quality,
        ))
        self.assertFalse(control.coast_velocity_handoff_tilt_ready)
        self.assertFalse(control.coast_velocity_handoff_rate_ready)
        self.assertIsNone(
            control.coast_jerk_limited_terminal_command_started_at
        )
        control.send(
            commander, command_timestamp=terminal_time, yaw_deg=0.0
        )
        self.assertAlmostEqual(
            control.coast_jerk_limited_terminal_command_started_at,
            terminal_time,
        )
        response_required_s = (
            control.coast_jerk_limited_terminal_response_required_s
        )
        self.assertAlmostEqual(response_required_s, 0.76)
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.301, 1.0], [0.0, 0.02, 0.0], terminal_time + 0.06,
            current_orientation_rpy=np.radians([20.0, -10.0, 0.0]),
            current_angular_velocity=np.zeros(3),
            command_timestamp=terminal_time + 0.06,
            **terminal_quality,
        ))
        self.assertFalse(control.coast_jerk_limited_terminal_command_ready)
        response_settled_at = terminal_time + response_required_s
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.302, 1.0], [0.0, 0.02, 0.0], response_settled_at,
            current_orientation_rpy=np.radians([20.0, -10.0, 0.0]),
            current_angular_velocity=np.zeros(3),
            command_timestamp=response_settled_at,
            **terminal_quality,
        ))
        self.assertTrue(control.coast_jerk_limited_terminal_command_ready)
        self.assertTrue(
            control.coast_jerk_limited_terminal_level_response_ready
        )
        self.assertFalse(control.coast_velocity_handoff_tilt_ready)
        self.assertFalse(control.coast_jerk_limited_estimated_tilt_gate_used)
        self.assertGreater(control.coast_actual_tilt_deg, 20.0)
        self.assertFalse(
            control.coast_jerk_limited_terminal_acceleration_sample_fresh
        )
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.303, 1.0], [0.0, 0.02, 0.0],
            response_settled_at + 0.01,
            current_orientation_rpy=np.radians([20.0, -10.0, 0.0]),
            current_angular_velocity=np.radians([6.0, 0.0, 0.0]),
            command_timestamp=response_settled_at + 0.01,
            **terminal_quality,
        ))
        self.assertTrue(
            control.coast_jerk_limited_terminal_acceleration_sample_fresh
        )
        self.assertTrue(control.coast_jerk_limited_acceleration_ready)
        self.assertFalse(control.coast_velocity_handoff_rate_ready)
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.3035, 1.0], [0.0, 0.02, 0.0],
            response_settled_at + 0.02,
            current_orientation_rpy=np.radians([20.0, -10.0, 0.0]),
            current_angular_velocity=np.zeros(3),
            command_timestamp=response_settled_at + 0.02,
            **terminal_quality,
        ))
        self.assertTrue(control.coast_velocity_handoff_rate_ready)
        # A bad callback-group skew must reset the terminal dwell even when all
        # physical motion gates are ready. Global skew enforcement can remain
        # disabled; the jerk-to-position transfer still fails closed locally.
        bad_terminal_quality = dict(terminal_quality)
        bad_terminal_quality['current_state_group_skew_s'] = 0.031
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.3036, 1.0], [0.0, 0.02, 0.0],
            response_settled_at + 0.03,
            current_orientation_rpy=np.radians([20.0, -10.0, 0.0]),
            current_angular_velocity=np.zeros(3),
            command_timestamp=response_settled_at + 0.03,
            **bad_terminal_quality,
        ))
        self.assertFalse(
            control.coast_jerk_limited_terminal_measurements_fresh
        )
        self.assertIsNone(control.coast_response_queue_settle_elapsed_s)
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.3038, 1.0], [0.0, 0.02, 0.0],
            response_settled_at + 0.04,
            current_orientation_rpy=np.radians([20.0, -10.0, 0.0]),
            current_angular_velocity=np.zeros(3),
            command_timestamp=response_settled_at + 0.04,
            **terminal_quality,
        ))
        self.assertTrue(
            control.coast_jerk_limited_terminal_measurements_fresh
        )
        self.assertAlmostEqual(
            control.coast_response_queue_settle_elapsed_s, 0.0
        )
        self.assertTrue(control.update_coast_velocity(
            [0.0, 0.304, 1.0], [0.0, 0.02, 0.0],
            response_settled_at + 0.07,
            current_orientation_rpy=np.radians([20.0, -10.0, 0.0]),
            current_angular_velocity=np.zeros(3),
            command_timestamp=response_settled_at + 0.07,
            **terminal_quality,
        ))
        self.assertEqual(
            control.coast_handoff_reason,
            'jerk_limited_attitude_position_handoff',
        )
        self.assertEqual(
            control.coast_handoff_control_owner,
            'direct_attitude_zdistance',
        )
        self.assertEqual(
            control.coast_handoff_command_mode,
            'jerk_limited_attitude_zdistance',
        )
        np.testing.assert_allclose(
            control.coast_handoff_attitude_target_rp_deg, [0.0, 0.0]
        )
        np.testing.assert_allclose(control.hold_position, [0.0, 0.304, 1.0])

    def test_jerk_limited_virtual_friction_changes_profile_duration(self):
        def plan(mu):
            control = TranslationControlHandoff(
                initial_position=[0.0, 0.0, 1.0],
                yaw_deg=0.0,
                shadow_mode=False,
                coast_velocity_braking_enabled=True,
                coast_velocity_predictive_unwind_enabled=True,
                coast_jerk_limited_attitude_enabled=True,
                coast_jerk_limited_virtual_friction_enabled=True,
                coast_velocity_rebrake_enabled=False,
            )
            self.assertTrue(control.start_contact('orientation'))
            self.assertTrue(control.end_contact(
                [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 1.0,
                interaction_direction=[0.0, 1.0, 0.0],
                virtual_release_velocity=[0.0, 0.60, 0.0],
                virtual_kinetic_friction_coefficient=mu,
                current_orientation_rpy=np.zeros(3),
                coast=True,
            ))
            control._coast_command_history = [(0.0, np.zeros(2))]
            control.confirm_release_candidate(
                timestamp=1.0,
                current_velocity=[0.0, 0.60, 0.0],
            )
            self.assertFalse(control.update_coast_velocity(
                [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 1.0,
                current_orientation_rpy=np.zeros(3),
                current_angular_velocity=np.zeros(3),
                command_timestamp=1.0,
            ))
            return control

        low_friction = plan(0.01)
        high_friction = plan(0.10)

        self.assertAlmostEqual(
            low_friction.coast_jerk_limited_selected_deceleration_m_s2,
            0.40,
        )
        self.assertAlmostEqual(
            high_friction.coast_jerk_limited_selected_deceleration_m_s2,
            0.981,
        )
        self.assertGreater(
            low_friction.coast_jerk_limited_profile.duration_s,
            high_friction.coast_jerk_limited_profile.duration_s,
        )

    def test_jerk_limited_attitude_change_is_bounded_each_10_ms_send(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            brake_max_attitude_deg=30.0,
            coast_max_acceleration_m_s2=5.0,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_jerk_limited_max_attitude_rate_deg_s=720.0,
            coast_jerk_limited_settle_timeout_s=5.0,
            coast_velocity_rebrake_enabled=False,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control._coast_command_history = [(0.0, np.zeros(2))]
        control.confirm_release_candidate(
            timestamp=1.0,
            current_velocity=[0.0, 0.60, 0.0],
        )

        sent_rp = []
        for index in range(18):
            now = 1.0 + 0.01 * index
            self.assertFalse(control.update_coast_velocity(
                [0.0, 0.60 * (now - 1.0), 1.0],
                [0.0, 0.60, 0.0],
                now,
                current_orientation_rpy=np.zeros(3),
                current_angular_velocity=np.zeros(3),
                command_timestamp=now,
            ))
            control.send(commander, command_timestamp=now, yaw_deg=0.0)
            call = commander.calls[-1]
            self.assertEqual(call[0], 'zdistance')
            sent_rp.append(np.asarray(call[1][:2], dtype=float))

        self.assertLessEqual(
            control.coast_jerk_limited_validation_max_attitude_rate_deg_s,
            23.5,
        )
        for previous, current in zip(sent_rp, sent_rp[1:]):
            self.assertLessEqual(
                float(np.linalg.norm(current - previous)),
                7.2 + 1e-9,
            )

    def test_jerk_limited_infeasible_profile_falls_back_to_velocity(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_jerk_limited_max_duration_s=0.10,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control._coast_command_history = [(0.0, np.zeros(2))]
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 1.0,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.0,
        ))
        self.assertEqual(control.coast_velocity_phase, 'fast_brake')
        self.assertFalse(control.coast_jerk_limited_attitude_active)
        self.assertTrue(control.consume_velocity_pid_reset_request())
        event = control.consume_jerk_limited_event()
        self.assertEqual(event['event'], 'Jerk-Limited Brake Fallback')
        self.assertIn('duration', event['reason'])
        control.send(commander, command_timestamp=1.0, yaw_deg=0.0)
        self.assertEqual(commander.calls[-1][0], 'hover')

    def test_jerk_limited_path_requires_actual_sent_attitude_history(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
        )
        self.assertTrue(control.start_contact('orientation'))
        tracking = control.update_release_candidate_attitude(
            current_velocity=[0.0, 0.60, 0.0],
            current_orientation_rpy=np.zeros(3),
            interaction_direction=[0.0, 1.0, 0.0],
            timestamp=0.90,
            command_timestamp=0.90,
        )
        self.assertEqual(
            tracking['action'],
            'level_without_reconstructable_command_history',
        )
        self.assertEqual(tracking['raw_tilt_deg'], 0.0)
        self.assertEqual(control._coast_command_history, [])
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 1.0,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.0,
        ))

        self.assertTrue(
            control.coast_jerk_limited_velocity_fallback_active
        )
        self.assertIn(
            'actually sent attitude history',
            control.coast_jerk_limited_fallback_reason,
        )

    def test_jerk_release_candidate_ignores_ekf_roll_pitch(self):
        def run_candidate(roll_pitch_rad):
            control = TranslationControlHandoff(
                initial_position=[0.0, 0.0, 1.0],
                yaw_deg=0.0,
                shadow_mode=False,
                coast_velocity_braking_enabled=True,
                coast_velocity_predictive_unwind_enabled=True,
                coast_jerk_limited_attitude_enabled=True,
                coast_jerk_limited_extra_command_delay_s=0.03,
                coast_jerk_limited_virtual_friction_enabled=False,
            )
            self.assertTrue(control.start_contact('orientation'))
            control._coast_command_history = [
                (-1.0, np.array([0.0, -1.0])),
                (-0.05, np.array([0.0, -0.2])),
            ]
            before = [
                (sent_at, command.copy())
                for sent_at, command in control._coast_command_history
            ]
            tracking = control.update_release_candidate_attitude(
                current_velocity=[0.0, 0.20, 0.0],
                current_orientation_rpy=[
                    roll_pitch_rad[0], roll_pitch_rad[1], 0.0,
                ],
                interaction_direction=[0.0, 1.0, 0.0],
                timestamp=0.0,
                command_timestamp=0.0,
            )
            for expected, actual in zip(
                    before, control._coast_command_history):
                self.assertEqual(expected[0], actual[0])
                np.testing.assert_allclose(expected[1], actual[1])
            return control, tracking

        finite_control, finite = run_candidate(np.radians([35.0, -28.0]))
        nan_control, nan_rp = run_candidate([float('nan'), float('nan')])

        for key in (
            'roll_deg',
            'pitch_deg',
            'predicted_level_terminal_speed_m_s',
            'tail_cancellation_signed_acceleration_m_s2',
            'predicted_terminal_after_pulse_m_s',
        ):
            self.assertAlmostEqual(finite[key], nan_rp[key])
        self.assertEqual(
            finite['acceleration_source'],
            'actual_command_history_delayed_first_order_model',
        )
        self.assertEqual(finite['acceleration_source'], nan_rp['acceleration_source'])
        self.assertTrue(finite['command_history_ready'])
        self.assertAlmostEqual(finite['required_command_history_span_s'], 0.79)
        np.testing.assert_allclose(
            finite['modeled_current_acceleration_xy_m_s2'], [0.0, -1.0]
        )
        np.testing.assert_allclose(
            finite['modeled_current_acceleration_xy_m_s2'],
            nan_rp['modeled_current_acceleration_xy_m_s2'],
        )
        self.assertEqual(
            finite_control.release_candidate_acceleration_source,
            'actual_command_history_delayed_first_order_model',
        )
        self.assertEqual(
            finite_control.release_candidate_action,
            nan_control.release_candidate_action,
        )

    def test_jerk_release_candidate_short_history_levels_without_pulse(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
        )
        self.assertTrue(control.start_contact('orientation'))
        control._coast_command_history = [
            (-0.05, np.array([0.0, -5.0])),
        ]
        tracking = control.update_release_candidate_attitude(
            current_velocity=[0.0, 0.20, 0.0],
            current_orientation_rpy=np.radians([-45.0, 35.0, 0.0]),
            interaction_direction=[0.0, 1.0, 0.0],
            timestamp=0.0,
            command_timestamp=0.0,
        )

        self.assertEqual(
            tracking['action'],
            'level_without_reconstructable_command_history',
        )
        self.assertEqual(tracking['acceleration_source'], 'none_level_fail_safe')
        self.assertFalse(tracking['command_history_ready'])
        self.assertAlmostEqual(tracking['command_history_span_s'], 0.05)
        np.testing.assert_allclose(
            [tracking['roll_deg'], tracking['pitch_deg']], [0.0, 0.0]
        )
        self.assertEqual(tracking['tail_cancellation_acceleration_m_s2'], 0.0)
        self.assertIsNone(control._tail_neutralization_deadline)
        self.assertEqual(len(control._coast_command_history), 1)

    def test_jerk_path_uses_yaw_and_command_history_when_ekf_rp_is_invalid(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
        )
        self.assertTrue(control.start_contact('orientation'))
        control._coast_command_history = [(0.0, np.zeros(2))]
        invalid_rp_with_valid_yaw = [float('nan'), float('inf'), 0.0]

        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=invalid_rp_with_valid_yaw,
            coast=True,
        ))
        control.confirm_release_candidate(
            timestamp=1.0,
            current_velocity=[0.0, 0.60, 0.0],
        )
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 1.0,
            current_orientation_rpy=invalid_rp_with_valid_yaw,
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.0,
            current_state_group_skew_s=0.03,
            max_terminal_state_age_s=0.10,
            max_terminal_state_group_skew_s=0.03,
        ))

        self.assertTrue(control.coast_jerk_limited_attitude_active)
        self.assertFalse(control.coast_jerk_limited_velocity_fallback_active)
        self.assertFalse(control.coast_estimated_attitude_telemetry_valid)
        self.assertIsNone(control.coast_actual_tilt_deg)
        self.assertTrue(
            control.coast_jerk_limited_terminal_measurements_fresh
        )

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.006, 1.0], [0.0, 0.59, 0.0], 1.01,
            current_orientation_rpy=invalid_rp_with_valid_yaw,
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.111,
            current_state_group_skew_s=0.03,
            max_terminal_state_age_s=0.10,
            max_terminal_state_group_skew_s=0.03,
        ))
        self.assertFalse(
            control.coast_jerk_limited_terminal_state_age_ready
        )
        self.assertFalse(
            control.coast_jerk_limited_terminal_measurements_fresh
        )

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.012, 1.0], [0.0, 0.58, 0.0], 1.12,
            current_orientation_rpy=invalid_rp_with_valid_yaw,
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.12,
            current_state_group_skew_s=0.031,
            max_terminal_state_age_s=0.10,
            max_terminal_state_group_skew_s=0.03,
        ))
        self.assertTrue(control.coast_jerk_limited_terminal_state_age_ready)
        self.assertFalse(
            control.coast_jerk_limited_terminal_state_group_skew_ready
        )
        self.assertFalse(
            control.coast_jerk_limited_terminal_measurements_fresh
        )

    def test_jerk_history_wait_holds_level_then_plans_from_latest_state(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_history_wait_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_state_kinematic_guard_enabled=True,
            coast_command_period_s=0.02,
        )
        self.assertTrue(control.start_contact('orientation'))
        for index in range(15):
            control.send(
                commander,
                command_timestamp=0.02 * index,
                yaw_deg=0.0,
            )
        control.set_contact_attitude(5.0, 0.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=0.30, yaw_deg=0.0)
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 0.32,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control.confirm_release_candidate(
            timestamp=0.32,
            current_velocity=[0.0, 0.60, 0.0],
        )

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 0.32,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=0.32,
        ))
        self.assertTrue(control.coast_jerk_limited_history_wait_active)
        self.assertFalse(control.coast_jerk_limited_velocity_fallback_active)
        self.assertEqual(
            control.command_mode,
            'jerk_history_level_hold_attitude_zdistance',
        )
        self.assertIsNone(control.coast_jerk_limited_first_actual_send_at)
        wait_event = control.consume_jerk_limited_event()
        self.assertEqual(
            wait_event['event'],
            'Jerk-Limited Attitude History Wait Started',
        )
        self.assertAlmostEqual(wait_event['command_history_span_s'], 0.32)
        self.assertAlmostEqual(wait_event['remaining_history_s'], 0.44)
        np.testing.assert_allclose(
            wait_event['level_command_activation_time_corners'],
            [0.32, 0.36],
        )

        control.send(commander, command_timestamp=0.32, yaw_deg=0.0)
        self.assertEqual(commander.calls[-1][0], 'zdistance')
        np.testing.assert_allclose(commander.calls[-1][1][:2], [0.0, 0.0])
        self.assertEqual(commander.calls[-1][1][3], 1.0)
        self.assertAlmostEqual(
            control.coast_jerk_limited_history_wait_first_actual_send_at,
            0.32,
        )
        self.assertIsNone(control.coast_jerk_limited_first_actual_send_at)

        for index in range(17, 38):
            now = 0.02 * index
            progress = 0.60 * (now - 0.32)
            self.assertFalse(control.update_coast_velocity(
                [0.0, progress, 1.0], [0.0, 0.60, 0.0], now,
                current_orientation_rpy=np.zeros(3),
                current_angular_velocity=np.zeros(3),
                command_timestamp=now,
            ))
            self.assertTrue(control.coast_jerk_limited_history_wait_active)
            control.send(commander, command_timestamp=now, yaw_deg=0.0)
            self.assertEqual(commander.calls[-1][0], 'zdistance')
            np.testing.assert_allclose(
                commander.calls[-1][1][:2], [0.0, 0.0]
            )

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.264, 1.0], [0.0, 0.60, 0.0], 0.76,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=0.76,
        ))
        self.assertFalse(control.coast_jerk_limited_history_wait_active)
        self.assertEqual(control.coast_velocity_phase, 'jerk_attitude_brake')
        self.assertFalse(control.coast_jerk_limited_velocity_fallback_active)
        self.assertAlmostEqual(
            control.coast_jerk_limited_history_wait_elapsed_s, 0.44
        )
        control.send(commander, command_timestamp=0.76, yaw_deg=0.0)
        self.assertAlmostEqual(
            control.coast_jerk_limited_first_actual_send_at, 0.76
        )
        start_event = control.consume_jerk_limited_event()
        self.assertEqual(
            start_event['event'], 'Jerk-Limited Attitude Brake Started'
        )
        self.assertAlmostEqual(start_event['planning_projected_speed_m_s'], 0.60)
        self.assertAlmostEqual(start_event['history_wait_elapsed_s'], 0.44)

    def test_jerk_history_wait_completion_uses_geometry_and_recorded_bound(
            self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_history_wait_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_max_acceleration_m_s2=0.10,
            brake_max_attitude_deg=30.0,
            coast_attitude_acceleration_scale=1.0,
            coast_command_period_s=0.02,
        )
        geometry_bound = 9.81 * np.tan(np.radians(30.0))
        self.assertAlmostEqual(
            control._jerk_history_wait_acceleration_bound_m_s2(),
            geometry_bound,
        )

        self.assertTrue(control.start_contact('orientation'))
        control.set_contact_attitude(40.0, 0.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=0.0, yaw_deg=0.0)
        control.set_contact_attitude(0.0, 0.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=0.10, yaw_deg=0.0)
        recorded_bound = float(np.linalg.norm(
            attitude_to_world_acceleration(40.0, 0.0, 0.0)
        ))
        self.assertGreater(recorded_bound, geometry_bound)
        self.assertAlmostEqual(
            control._jerk_history_wait_acceleration_bound_m_s2(),
            recorded_bound,
        )

        control.release_position_m = np.array([0.0, 0.0, 1.0])
        control.brake_direction = np.array([0.0, 1.0, 0.0])
        control.coast_velocity_phase = 'jerk_history_level_hold'
        control.coast_jerk_limited_history_wait_started_at = 0.50
        control.brake_projected_speed_m_s = 0.20
        horizon_s = 0.10 + max(0.02, 0.03 + 0.01)
        with patch.object(
                control, '_initialize_jerk_limited_profile',
                return_value=False):
            self.assertFalse(control._update_jerk_limited_attitude(
                np.array([0.0, 0.0, 1.0]),
                np.array([0.0, 0.20, 0.0]),
                np.zeros(3),
                np.zeros(3),
                0.80,
                0.90,
                0.02,
                False,
            ))
        self.assertAlmostEqual(
            control.coast_jerk_limited_history_wait_predicted_distance_m,
            0.20 * horizon_s + 0.5 * recorded_bound * horizon_s ** 2,
        )

    def test_jerk_history_wait_uncertainty_uses_recorded_bound(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_history_wait_enabled=True,
            coast_jerk_limited_history_wait_max_s=0.60,
            coast_jerk_limited_history_wait_max_distance_m=5.0,
            coast_jerk_limited_history_wait_max_lateral_distance_m=5.0,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_max_acceleration_m_s2=0.10,
            brake_max_attitude_deg=30.0,
            coast_attitude_acceleration_scale=1.0,
            coast_command_period_s=0.02,
        )
        self.assertTrue(control.start_contact('orientation'))
        control.set_contact_attitude(40.0, 0.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=0.0, yaw_deg=0.0)
        control.set_contact_attitude(0.0, 0.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=0.10, yaw_deg=0.0)
        recorded_bound = float(np.linalg.norm(
            attitude_to_world_acceleration(40.0, 0.0, 0.0)
        ))
        control.release_position_m = np.array([0.0, 0.0, 1.0])
        control.brake_direction = np.array([0.0, 1.0, 0.0])

        timestamp = 0.30
        history_status = control._jerk_command_history_status(timestamp)
        self.assertFalse(control._continue_jerk_history_level_hold(
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 5.0, 0.0]),
            timestamp,
            timestamp,
            history_status,
            0.0,
        ))
        known_response_s = (
            history_status['span_s'] - history_status['response_delay_s']
        )
        expected_uncertainty = float(
            2.0 * recorded_bound * np.exp(
                -known_response_s / control.coast_attitude_time_constant_s
            )
        )
        self.assertAlmostEqual(
            control
            .coast_jerk_limited_history_wait_acceleration_uncertainty_m_s2,
            expected_uncertainty,
        )
        self.assertGreater(
            expected_uncertainty,
            2.0 * control.coast_max_acceleration_m_s2 * np.exp(
                -known_response_s / control.coast_attitude_time_constant_s
            ),
        )
        self.assertEqual(
            control.coast_velocity_phase,
            'jerk_history_level_hold',
        )
        self.assertFalse(
            control.coast_jerk_limited_velocity_fallback_active,
            control.coast_jerk_limited_fallback_reason,
        )

    def test_jerk_history_wait_nonfinite_bound_falls_back(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_history_wait_enabled=True,
            coast_jerk_limited_history_wait_max_s=0.60,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_attitude_time_constant_s=0.0,
        )
        self.assertTrue(control.start_contact('orientation'))
        control.send(commander, command_timestamp=0.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=0.10, yaw_deg=0.0)
        control.release_position_m = np.array([0.0, 0.0, 1.0])
        control.brake_direction = np.array([0.0, 1.0, 0.0])
        timestamp = 0.30
        history_status = control._jerk_command_history_status(timestamp)

        with patch.object(
                control, '_jerk_history_wait_acceleration_bound_m_s2',
                return_value=float('inf')):
            self.assertFalse(control._continue_jerk_history_level_hold(
                np.array([0.0, 0.0, 1.0]),
                np.array([0.0, 1.0, 0.0]),
                timestamp,
                timestamp,
                history_status,
                0.0,
            ))
        self.assertTrue(
            control.coast_jerk_limited_velocity_fallback_active
        )
        self.assertEqual(
            control.coast_jerk_limited_fallback_reason,
            'jerk history-wait acceleration bound is not finite',
        )
        self.assertTrue(np.isinf(
            control
            .coast_jerk_limited_history_wait_acceleration_uncertainty_m_s2
        ))

    def test_jerk_history_wait_send_projection_uses_recorded_bound(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_history_wait_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_max_acceleration_m_s2=0.10,
            brake_max_attitude_deg=30.0,
            coast_attitude_acceleration_scale=1.0,
        )
        self.assertTrue(control.start_contact('orientation'))
        control.set_contact_attitude(40.0, 0.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=0.0, yaw_deg=0.0)
        control.set_contact_attitude(0.0, 0.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=0.10, yaw_deg=0.0)
        recorded_bound = float(np.linalg.norm(
            attitude_to_world_acceleration(40.0, 0.0, 0.0)
        ))

        control.release_position_m = np.array([0.0, 0.0, 1.0])
        control.brake_direction = np.array([0.0, 1.0, 0.0])
        control.coast_jerk_limited_history_wait_completed_at = 0.80
        control._coast_jerk_limited_plan_position_m = np.array([
            0.0, 0.0, 1.0,
        ])
        control._coast_jerk_limited_plan_velocity_m_s = np.array([
            0.0, 0.20, 0.0,
        ])
        control._coast_jerk_limited_plan_state_timestamp = 0.80
        with patch.object(
                control, '_initialize_jerk_limited_profile',
                return_value=False):
            prepared, scheduled_at = control._prepare_jerk_limited_first_send(
                0.94
            )
        self.assertFalse(prepared)
        self.assertAlmostEqual(scheduled_at, 0.94)
        horizon_s = 0.14
        self.assertAlmostEqual(
            control.coast_jerk_limited_history_wait_predicted_distance_m,
            0.20 * horizon_s + 0.5 * recorded_bound * horizon_s ** 2,
        )

    def test_jerk_history_wait_ready_frame_rechecks_lateral_velocity_projection(
            self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_history_wait_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            # Deliberately isolate the history-completion admission gate. The
            # final frame is kinematically abrupt so its lateral velocity is
            # the only evidence that predicts departure from the envelope.
            coast_state_kinematic_guard_enabled=False,
            coast_command_period_s=0.02,
        )
        self.assertTrue(control.start_contact('orientation'))
        for index in range(15):
            control.send(
                commander,
                command_timestamp=0.02 * index,
                yaw_deg=0.0,
            )
        control.set_contact_attitude(5.0, 0.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=0.30, yaw_deg=0.0)
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 0.32,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control.confirm_release_candidate(
            timestamp=0.32,
            current_velocity=[0.0, 0.60, 0.0],
        )
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 0.32,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=0.32,
        ))
        control.send(commander, command_timestamp=0.32, yaw_deg=0.0)

        for index in range(17, 38):
            now = 0.02 * index
            progress = 0.60 * (now - 0.32)
            self.assertFalse(control.update_coast_velocity(
                [0.0, progress, 1.0], [0.0, 0.60, 0.0], now,
                current_orientation_rpy=np.zeros(3),
                current_angular_velocity=np.zeros(3),
                command_timestamp=now,
            ))
            control.send(commander, command_timestamp=now, yaw_deg=0.0)

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.264, 1.0], [5.0, 0.60, 0.0], 0.76,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=0.76,
        ))

        self.assertTrue(control.coast_jerk_limited_velocity_fallback_active)
        self.assertFalse(control.coast_jerk_limited_history_wait_active)
        self.assertIsNone(control.coast_jerk_limited_profile)
        self.assertNotEqual(control.coast_velocity_phase, 'jerk_attitude_brake')
        self.assertGreater(
            control.coast_jerk_limited_history_wait_predicted_lateral_distance_m,
            control.coast_jerk_limited_history_wait_max_lateral_distance_m,
        )
        self.assertIn(
            'lateral travel limit',
            control.coast_jerk_limited_fallback_reason,
        )

    def test_jerk_history_wait_rechecks_travel_at_delayed_first_send(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_history_wait_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_command_period_s=0.02,
        )
        self.assertTrue(control.start_contact('orientation'))
        for index in range(15):
            control.send(
                commander,
                command_timestamp=0.02 * index,
                yaw_deg=0.0,
            )
        control.set_contact_attitude(5.0, 0.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=0.30, yaw_deg=0.0)
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 0.32,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control.confirm_release_candidate(
            timestamp=0.32,
            current_velocity=[0.0, 0.60, 0.0],
        )
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 0.32,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=0.32,
        ))
        control.send(commander, command_timestamp=0.32, yaw_deg=0.0)
        for index in range(17, 38):
            now = 0.02 * index
            progress = 0.60 * (now - 0.32)
            self.assertFalse(control.update_coast_velocity(
                [0.0, progress, 1.0], [0.0, 0.60, 0.0], now,
                current_orientation_rpy=np.zeros(3),
                current_angular_velocity=np.zeros(3),
                command_timestamp=now,
            ))
            control.send(commander, command_timestamp=now, yaw_deg=0.0)

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.264, 1.0], [0.0, 0.60, 0.0], 0.76,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=0.76,
        ))
        self.assertEqual(control.coast_velocity_phase, 'jerk_attitude_brake')

        # The state was admissible at 0.76 s, but an artificial 240 ms
        # decision-to-send delay consumes the remaining bounded travel budget.
        # The send path must re-project that horizon and dispatch level, not the
        # provisional jerk command.
        control.send(commander, command_timestamp=1.00, yaw_deg=0.0)

        self.assertTrue(control.coast_jerk_limited_velocity_fallback_active)
        self.assertFalse(control.coast_jerk_limited_attitude_active)
        self.assertIsNone(control.coast_jerk_limited_first_actual_send_at)
        self.assertEqual(commander.calls[-1][0], 'zdistance')
        np.testing.assert_allclose(commander.calls[-1][1][:2], [0.0, 0.0])
        self.assertGreater(
            control.coast_jerk_limited_history_wait_predicted_distance_m,
            control.coast_jerk_limited_history_wait_max_distance_m,
        )
        self.assertIn(
            'send delay exceeded the history-wait forward travel limit',
            control.coast_jerk_limited_fallback_reason,
        )

    def test_jerk_history_wait_rejects_a_tail_that_can_reach_speed_guard(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_history_wait_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_command_period_s=0.02,
        )
        self.assertTrue(control.start_contact('orientation'))
        for index in range(16):
            control.send(
                commander,
                command_timestamp=0.02 * index,
                yaw_deg=0.0,
            )
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.05, 0.0], 0.32,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control.confirm_release_candidate(
            timestamp=0.32,
            current_velocity=[0.0, 0.05, 0.0],
        )

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.05, 0.0], 0.32,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=0.32,
        ))

        self.assertTrue(control.coast_jerk_limited_velocity_fallback_active)
        self.assertFalse(control.coast_jerk_limited_history_wait_active)
        self.assertIn(
            'positive speed margin',
            control.coast_jerk_limited_fallback_reason,
        )
        event = control.consume_jerk_limited_event()
        self.assertEqual(event['event'], 'Jerk-Limited Brake Fallback')
        self.assertTrue(control.consume_velocity_pid_reset_request())

    def test_jerk_history_wait_bridges_nonlevel_actual_before_fallback(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_history_wait_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_command_period_s=0.02,
        )
        self.assertTrue(control.start_contact('orientation'))
        control.send(commander, command_timestamp=0.0, yaw_deg=0.0)
        control.set_contact_attitude(5.0, 0.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=0.30, yaw_deg=0.0)
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.05, 0.0], 0.32,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control.confirm_release_candidate(
            timestamp=0.32,
            current_velocity=[0.0, 0.05, 0.0],
        )
        history_count = len(control.sent_attitude_acceleration_history())

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.05, 0.0], 0.32,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=0.32,
        ))

        self.assertTrue(control.coast_jerk_limited_velocity_fallback_active)
        self.assertTrue(control.coast_jerk_limited_fallback_bridge_pending)
        self.assertEqual(
            control.command_mode,
            'jerk_fallback_bridge_attitude_zdistance',
        )
        self.assertFalse(control.consume_velocity_pid_reset_request())
        self.assertEqual(
            len(control.sent_attitude_acceleration_history()), history_count
        )

        control.send(commander, command_timestamp=0.32, yaw_deg=0.0)
        self.assertEqual(commander.calls[-1][0], 'zdistance')
        np.testing.assert_allclose(commander.calls[-1][1][:2], [0.0, 0.0])
        self.assertFalse(control.coast_jerk_limited_fallback_bridge_pending)
        self.assertEqual(
            len(control.sent_attitude_acceleration_history()),
            history_count + 1,
        )

        # A duplicate-state retry may call send before the main-loop reset
        # consumer. It must repeat level attitude, never issue velocity first.
        control.send(commander, command_timestamp=0.34, yaw_deg=0.0)
        self.assertEqual(commander.calls[-1][0], 'zdistance')
        np.testing.assert_allclose(commander.calls[-1][1][:2], [0.0, 0.0])
        self.assertTrue(control.consume_velocity_pid_reset_request())
        control.send(commander, command_timestamp=0.36, yaw_deg=0.0)
        self.assertEqual(commander.calls[-1][0], 'hover')

    def test_jerk_history_wait_send_timeout_holds_level_then_falls_back(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_history_wait_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_command_period_s=0.02,
        )
        self.assertTrue(control.start_contact('orientation'))
        for index in range(16):
            control.send(
                commander,
                command_timestamp=0.02 * index,
                yaw_deg=0.0,
            )
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 0.32,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control.confirm_release_candidate(
            timestamp=0.32,
            current_velocity=[0.0, 0.60, 0.0],
        )
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 0.32,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=0.32,
        ))
        control.send(commander, command_timestamp=0.32, yaw_deg=0.0)

        # No fresh state arrives for longer than the hard wait bound. send()
        # must keep fixed-Z level attitude for this cycle and defer velocity
        # ownership until its PID reset is consumed on a later fresh cycle.
        control.send(commander, command_timestamp=0.83, yaw_deg=0.0)

        self.assertEqual(commander.calls[-1][0], 'zdistance')
        np.testing.assert_allclose(commander.calls[-1][1][:2], [0.0, 0.0])
        self.assertTrue(control.coast_jerk_limited_velocity_fallback_active)
        self.assertFalse(control.coast_jerk_limited_history_wait_active)
        self.assertIsNone(control.coast_jerk_limited_profile)
        self.assertTrue(control.consume_velocity_pid_reset_request())
        event = control.consume_jerk_limited_event()
        self.assertEqual(event['event'], 'Jerk-Limited Brake Fallback')
        self.assertIn('expired before a fresh state', event['reason'])

    def test_jerk_history_wait_late_first_send_falls_back_after_level(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_history_wait_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_command_period_s=0.02,
        )
        self.assertTrue(control.start_contact('orientation'))
        for index in range(16):
            control.send(
                commander,
                command_timestamp=0.02 * index,
                yaw_deg=0.0,
            )
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 0.32,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control.confirm_release_candidate(
            timestamp=0.32,
            current_velocity=[0.0, 0.60, 0.0],
        )
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 0.32,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=0.32,
        ))
        self.assertAlmostEqual(
            control.coast_jerk_limited_history_wait_validated_level_at,
            0.36,
        )

        # Runtime uses the real wall clock. A commander return at 0.37 misses
        # the conservatively validated 0.36 activation deadline; that level
        # command is retained as the bridge and jerk authority is denied.
        with patch('Interaction.interactions.time.time', return_value=0.37):
            sent_at = control.send(commander, yaw_deg=0.0)

        self.assertAlmostEqual(sent_at, 0.37)
        self.assertEqual(commander.calls[-1][0], 'zdistance')
        np.testing.assert_allclose(commander.calls[-1][1][:2], [0.0, 0.0])
        self.assertTrue(control.coast_jerk_limited_velocity_fallback_active)
        self.assertTrue(control.coast_jerk_limited_fallback_bridge_level_sent)
        self.assertAlmostEqual(
            control.coast_jerk_limited_history_wait_first_send_deadline_miss_s,
            0.01,
        )
        self.assertTrue(control.consume_velocity_pid_reset_request())
        event = control.consume_jerk_limited_event()
        self.assertEqual(event['event'], 'Jerk-Limited Brake Fallback')
        self.assertIn('activation deadline', event['reason'])

    def test_jerk_history_wait_rejects_large_lateral_drift(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_jerk_limited_history_wait_enabled=True,
            coast_jerk_limited_virtual_friction_enabled=False,
            coast_velocity_rebrake_enabled=False,
            coast_command_period_s=0.02,
        )
        self.assertTrue(control.start_contact('orientation'))
        for index in range(16):
            control.send(
                commander,
                command_timestamp=0.02 * index,
                yaw_deg=0.0,
            )
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [2.0, 0.60, 0.0], 0.32,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control.confirm_release_candidate(
            timestamp=0.32,
            current_velocity=[2.0, 0.60, 0.0],
        )

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.0, 1.0], [2.0, 0.60, 0.0], 0.32,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=0.32,
        ))

        self.assertTrue(control.coast_jerk_limited_velocity_fallback_active)
        self.assertIn(
            'lateral travel limit',
            control.coast_jerk_limited_fallback_reason,
        )

    def test_disabled_jerk_path_does_not_validate_unused_jerk_tuning(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_jerk_limited_attitude_enabled=False,
            coast_jerk_limited_min_deceleration_m_s2=0.0,
            coast_jerk_limited_max_deceleration_m_s2=0.0,
            coast_jerk_limited_max_jerk_m_s3=0.0,
            coast_jerk_limited_max_duration_s=0.0,
            coast_jerk_limited_prepare_lead_s=0.0,
            coast_jerk_limited_activation_guard_s=-1.0,
            coast_jerk_limited_terminal_speed_margin_m_s=1.0,
        )

        self.assertFalse(control.coast_jerk_limited_attitude_enabled)

    def test_jerk_history_wait_requires_jerk_attitude_owner(self):
        with self.assertRaisesRegex(
                ValueError, 'history wait requires jerk-limited attitude'):
            TranslationControlHandoff(
                initial_position=[0.0, 0.0, 1.0],
                yaw_deg=0.0,
                shadow_mode=False,
                coast_jerk_limited_attitude_enabled=False,
                coast_jerk_limited_history_wait_enabled=True,
            )

    def test_jerk_limited_velocity_fallback_overrides_legacy_unwind_owner(self):
        for legacy_owner in ('direct_attitude', 'position'):
            with self.subTest(legacy_owner=legacy_owner):
                commander = FakeCommander()
                control = TranslationControlHandoff(
                    initial_position=[0.0, 0.0, 1.0],
                    yaw_deg=0.0,
                    shadow_mode=False,
                    coast_velocity_braking_enabled=True,
                    coast_velocity_predictive_unwind_enabled=True,
                    coast_jerk_limited_attitude_enabled=True,
                    coast_velocity_unwind_direct_level_attitude_enabled=(
                        legacy_owner == 'direct_attitude'
                    ),
                    coast_velocity_unwind_position_control_enabled=(
                        legacy_owner == 'position'
                    ),
                    coast_velocity_rebrake_enabled=False,
                    coast_alignment_dwell_s=0.0,
                )
                self.assertTrue(control.start_contact('orientation'))
                self.assertTrue(control.end_contact(
                    [0.0, 0.0, 1.0], [0.0, 0.30, 0.0], 1.0,
                    interaction_direction=[0.0, 1.0, 0.0],
                    current_orientation_rpy=np.zeros(3),
                    coast=True,
                ))
                control._activate_jerk_limited_fallback(
                    'test fallback ownership',
                    [0.0, 0.02],
                    prefer_unwind=True,
                )
                # A stale release-line target must not re-enable position
                # ownership during the velocity-only safety fallback.
                control.coast_velocity_unwind_position_target_m = np.array([
                    0.0, 0.20, 1.0,
                ])

                self.assertTrue(
                    control.coast_jerk_limited_velocity_fallback_active
                )
                self.assertEqual(
                    control.coast_velocity_phase, 'predictive_unwind'
                )
                self.assertFalse(control.direct_level_unwind_active)
                self.assertFalse(control.uses_position_setpoint)
                self.assertEqual(
                    control.command_mode,
                    'jerk_fallback_bridge_attitude_zdistance',
                )
                self.assertTrue(control.consume_velocity_pid_reset_request())
                self.assertEqual(control.command_mode, 'velocity_coast')
                control.confirm_release_candidate(
                    timestamp=1.005,
                    current_velocity=[0.0, 0.02, 0.0],
                )
                self.assertTrue(
                    control.coast_jerk_limited_velocity_fallback_active
                )
                self.assertEqual(
                    control.coast_velocity_phase, 'predictive_unwind'
                )
                control.send(
                    commander, command_timestamp=1.01, yaw_deg=0.0
                )
                self.assertEqual(commander.calls[-1][0], 'hover')
                invalid_rp_with_valid_yaw = [
                    float('nan'), float('inf'), 0.0,
                ]
                self.assertFalse(control.update_coast_velocity(
                    [0.0, 0.01, 1.0], [0.0, 0.02, 0.0], 1.02,
                    current_orientation_rpy=invalid_rp_with_valid_yaw,
                    current_angular_velocity=np.zeros(3),
                    command_timestamp=1.02,
                    current_state_group_skew_s=0.0,
                    max_terminal_state_age_s=0.10,
                    max_terminal_state_group_skew_s=0.03,
                ))
                self.assertFalse(
                    control.coast_jerk_limited_fallback_response_ready
                )
                self.assertFalse(
                    control.coast_jerk_limited_estimated_tilt_gate_used
                )
                self.assertIn(
                    control.coast_velocity_unwind_acceleration_source,
                    (
                        'measured_velocity_derivative_jerk_fallback',
                        'unavailable_fail_closed_jerk_fallback',
                    ),
                )
                response_settled_at = (
                    control.coast_jerk_limited_fallback_response_settled_at
                )
                self.assertAlmostEqual(response_settled_at, 1.77)
                # A long jump reaches the response deadline but cannot supply a
                # wholly post-response acceleration interval.
                self.assertFalse(control.update_coast_velocity(
                    [0.0, 0.02, 1.0], [0.0, 0.02, 0.0],
                    response_settled_at,
                    current_orientation_rpy=invalid_rp_with_valid_yaw,
                    current_angular_velocity=np.zeros(3),
                    command_timestamp=response_settled_at,
                    current_state_group_skew_s=0.0,
                    max_terminal_state_age_s=0.10,
                    max_terminal_state_group_skew_s=0.03,
                ))
                self.assertTrue(
                    control.coast_jerk_limited_fallback_response_ready
                )
                self.assertFalse(
                    control
                    .coast_jerk_limited_fallback_acceleration_sample_fresh
                )
                self.assertTrue(control.update_coast_velocity(
                    [0.0, 0.0202, 1.0], [0.0, 0.02, 0.0],
                    response_settled_at + 0.01,
                    current_orientation_rpy=invalid_rp_with_valid_yaw,
                    current_angular_velocity=np.zeros(3),
                    command_timestamp=response_settled_at + 0.01,
                    current_state_group_skew_s=0.0,
                    max_terminal_state_age_s=0.10,
                    max_terminal_state_group_skew_s=0.03,
                ))
                self.assertTrue(
                    control.coast_jerk_limited_fallback_acceleration_ready
                )
                self.assertEqual(
                    control.coast_velocity_unwind_acceleration_source,
                    'measured_velocity_derivative_jerk_fallback',
                )
                self.assertEqual(
                    control.coast_handoff_reason,
                    'jerk_limited_velocity_fallback_position_handoff',
                )
                self.assertEqual(
                    control.coast_handoff_control_owner,
                    'native_velocity_pid',
                )

    def test_jerk_fallback_response_anchor_tracks_velocity_command_changes(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.30, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control._activate_jerk_limited_fallback(
            'test fallback command anchor',
            [0.0, 0.02],
            prefer_unwind=True,
        )
        self.assertTrue(control.consume_velocity_pid_reset_request())

        control.coast_velocity_command_xy_m_s = np.array([0.0, 0.020])
        control.send(commander, command_timestamp=1.01, yaw_deg=0.0)
        self.assertAlmostEqual(
            control
            .coast_jerk_limited_fallback_terminal_velocity_command_started_at,
            1.01,
        )
        # A sub-margin change is treated as the same terminal command, avoiding
        # a noise-driven response timer that can never finish.
        control.coast_velocity_command_xy_m_s = np.array([0.0, 0.024])
        control.send(commander, command_timestamp=1.10, yaw_deg=0.0)
        self.assertAlmostEqual(
            control
            .coast_jerk_limited_fallback_terminal_velocity_command_started_at,
            1.01,
        )
        # Cumulative movement beyond the 0.005 m/s terminal margin is material
        # and must restart the complete plant-response window.
        control.coast_velocity_command_xy_m_s = np.array([0.0, 0.026])
        control.send(commander, command_timestamp=1.20, yaw_deg=0.0)
        self.assertAlmostEqual(
            control
            .coast_jerk_limited_fallback_terminal_velocity_command_started_at,
            1.20,
        )
        np.testing.assert_allclose(
            control
            .coast_jerk_limited_fallback_terminal_velocity_command_xy_m_s,
            [0.0, 0.026],
        )

    def test_jerk_velocity_fallback_does_not_rebrake_from_ekf_tilt(
            self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_jerk_limited_attitude_enabled=True,
            coast_velocity_unwind_position_control_enabled=True,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.30, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=np.zeros(3),
            coast=True,
        ))
        control._activate_jerk_limited_fallback(
            'test fallback rebrake',
            [0.0, 0.02],
            prefer_unwind=True,
        )
        self.assertTrue(control.consume_velocity_pid_reset_request())

        self.assertFalse(control.uses_position_setpoint)
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.01, 1.0], [0.0, 0.06, 0.0], 1.02,
            current_orientation_rpy=[float('nan'), float('inf'), 0.0],
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.02,
        ))
        self.assertEqual(control.coast_velocity_phase, 'predictive_unwind')
        self.assertEqual(control.coast_velocity_rebrake_count, 0)
        self.assertFalse(control.uses_position_setpoint)
        self.assertFalse(control.consume_velocity_rebrake_request())

    def test_direct_level_and_position_unwind_are_mutually_exclusive(self):
        with self.assertRaisesRegex(ValueError, 'mutually exclusive'):
            TranslationControlHandoff(
                initial_position=[0.0, 0.0, 1.0],
                yaw_deg=0.0,
                shadow_mode=False,
                coast_velocity_predictive_unwind_enabled=True,
                coast_velocity_unwind_direct_level_attitude_enabled=True,
                coast_velocity_unwind_position_control_enabled=True,
            )

    def test_velocity_handoff_position_offset_cannot_be_negative(self):
        with self.assertRaisesRegex(ValueError, 'position offset'):
            TranslationControlHandoff(
                initial_position=[0.0, 0.0, 1.0],
                yaw_deg=0.0,
                shadow_mode=False,
                coast_velocity_handoff_position_offset_m=-0.01,
            )

    def test_position_unwind_stays_on_release_line_without_retreat(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_velocity_unwind_position_control_enabled=True,
            coast_velocity_unwind_low_speed_fallback_m_s=1.0,
            coast_velocity_handoff_speed_m_s=0.03,
            coast_attitude_response_delay_s=0.07,
            coast_velocity_unwind_command_switch_delay_s=0.03,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.40, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)

        # Although the measured vehicle has drifted +0.10 m in X, the target
        # remains on the +Y interaction line through the release point. The
        # forward lookahead covers the observed decision and response delay.
        self.assertFalse(control.update_coast_velocity(
            [0.10, 0.20, 1.10], [0.0, 0.40, 0.0], 1.01,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.022,
        ))
        self.assertEqual(control.coast_velocity_phase, 'predictive_unwind')
        self.assertTrue(control.uses_position_setpoint)
        self.assertEqual(control.command_mode, 'predictive_unwind_position')
        np.testing.assert_allclose(
            control.coast_velocity_unwind_position_target_m,
            [0.0, 0.2448, 1.0],
        )
        self.assertAlmostEqual(
            control.coast_velocity_unwind_lateral_error_m, -0.10
        )
        self.assertTrue(control.consume_velocity_pid_reset_request())
        control.send(commander, command_timestamp=1.022)
        self.assertEqual(commander.calls[-1][0], 'position')

        self.assertFalse(control.update_coast_velocity(
            [0.08, 0.26, 1.0], [0.0, 0.20, 0.0], 1.02,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.03,
        ))
        target_before_reversal = (
            control.coast_velocity_unwind_position_target_m.copy()
        )
        np.testing.assert_allclose(target_before_reversal, [0.0, 0.282, 1.0])

        # Once residual attitude creates reverse velocity, the target neither
        # follows the vehicle backward nor re-enters zero-velocity braking.
        self.assertFalse(control.update_coast_velocity(
            [0.06, 0.24, 1.0], [0.0, -0.10, 0.0], 1.03,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
            command_timestamp=1.04,
        ))
        np.testing.assert_allclose(
            control.coast_velocity_unwind_position_target_m,
            target_before_reversal,
        )
        self.assertEqual(control.coast_velocity_phase, 'predictive_unwind')
        self.assertFalse(control.consume_velocity_rebrake_request())

        # The final POSITION_HOLD retains the same release-line target instead
        # of latching the laterally drifted measured pose.
        self.assertFalse(control.update_coast_velocity(
            [0.04, 0.25, 1.0], [0.0, 0.02, 0.0], 1.12,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
        ))
        self.assertTrue(control.update_coast_velocity(
            [0.03, 0.26, 1.0], [0.0, 0.01, 0.0], 1.21,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
        ))
        np.testing.assert_allclose(control.hold_position, target_before_reversal)
        self.assertFalse(control.coast_target_clamped_to_actual)
        self.assertFalse(control.coast_lateral_target_latched_to_actual)

    def test_integrated_leveling_delays_overoptimistic_predictive_unwind(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_velocity_unwind_terminal_speed_m_s=0.10,
            coast_velocity_unwind_integrated_leveling_enabled=True,
            coast_velocity_unwind_leveling_rate_deg_s=720.0,
            coast_velocity_unwind_integration_step_s=0.01,
            coast_attitude_response_delay_s=0.07,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 1.0, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)

        # This reproduces the latest flight's first re-brake state. The legacy
        # constant-tail model predicted 0.088 m/s and unwound here. Integrating
        # the rate-limited attitude return predicts substantial residual speed,
        # so the zero-velocity brake stays active.
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.10, 1.0],
            [-0.0835593641, 1.0033947229, 0.0],
            1.01,
            current_orientation_rpy=[
                0.1939156779, -0.0713523773, 0.0072613615,
            ],
            current_angular_velocity=[1.538, -0.405, 0.022],
        ))
        self.assertEqual(control.coast_velocity_phase, 'fast_brake')
        self.assertGreater(
            control.coast_velocity_predicted_unwind_terminal_speed_m_s,
            control.coast_velocity_unwind_terminal_speed_m_s,
        )
        self.assertLess(
            control.coast_velocity_unwind_integrated_velocity_delta_m_s,
            0.0,
        )
        self.assertGreater(
            control.coast_velocity_unwind_leveling_duration_s, 0.07
        )

    def test_predictive_velocity_coast_rejects_reverse_speed_handoff(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_velocity_handoff_min_projected_speed_m_s=0.0,
            coast_alignment_dwell_s=0.0,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.30, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.05, 1.0], [0.0, -0.05, 0.0], 1.10,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
        ))
        self.assertFalse(control.coast_velocity_handoff_speed_ready)
        self.assertAlmostEqual(
            float(control.coast_velocity_command_xy_m_s[1]), 0.0
        )
        # With the stricter default speed gate, this sample remains in the
        # zero-velocity brake instead of accepting or tracking reverse motion.
        self.assertEqual(control.coast_velocity_phase, 'fast_brake')
        self.assertEqual(
            control.coast_tracking_action,
            'predictive_zero_world_velocity_brake',
        )
        np.testing.assert_allclose(
            control.coast_velocity_command_xy_m_s, [0.0, 0.0]
        )
        self.assertEqual(control.command_mode, 'velocity_coast')

    def test_predictive_velocity_coast_one_step_unwinds_before_fixed_crossing(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_velocity_unwind_terminal_speed_m_s=0.10,
            coast_velocity_unwind_one_step_lookahead_enabled=True,
            coast_velocity_unwind_one_step_max_dt_s=0.01,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.40, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.027, 1.0], [0.0, 1.37, 0.0], 1.02,
            current_orientation_rpy=np.radians([20.0, 0.0, 0.0]),
            current_angular_velocity=np.zeros(3),
        ))
        self.assertGreater(
            control.coast_velocity_predicted_unwind_terminal_speed_m_s,
            control.coast_velocity_unwind_terminal_speed_m_s,
        )
        self.assertLessEqual(
            control.coast_velocity_predicted_unwind_terminal_speed_m_s,
            control.coast_velocity_dynamic_unwind_threshold_m_s,
        )
        self.assertAlmostEqual(
            control.coast_velocity_dynamic_unwind_step_guard_m_s,
            -control.coast_velocity_projected_acceleration_m_s2 * 0.01,
        )
        self.assertEqual(control.coast_velocity_phase, 'predictive_unwind')
        self.assertEqual(
            control.coast_velocity_unwind_decision_reason,
            'one_step_tail_prediction',
        )
        self.assertTrue(control.consume_velocity_pid_reset_request())

    def test_external_mpc_can_acquire_velocity_coast_attitude_commands(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.4, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        self.assertEqual(control.mode, control.VELOCITY_COAST)
        self.assertTrue(control.acquire_external_attitude_coast())
        self.assertEqual(control.mode, control.ATTITUDE_COAST)
        control.set_contact_attitude(-4.0, 0.0, 0.0)
        commander = FakeCommander()
        control.send(commander, command_timestamp=1.01, yaw_deg=0.0)
        self.assertEqual(commander.calls[-1][0], 'zdistance')
        self.assertEqual(commander.calls[-1][1][0], -4.0)
        self.assertTrue(control.set_predictive_position_target(
            [0.0, 0.2, 1.0], 1.2
        ))
        self.assertEqual(control.mode, control.POSITION_HOLD)

    def test_actual_send_snapshot_distinguishes_position_velocity_and_attitude(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
        )
        commander = FakeCommander()

        control.send(commander, command_timestamp=0.90)
        position = control.sent_command_snapshot()
        self.assertEqual(position['kind'], 'position')
        self.assertEqual(position['sent_at'], 0.90)

        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.4, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.send(commander, command_timestamp=1.01, yaw_deg=0.0)
        velocity = control.sent_command_snapshot()
        self.assertEqual(velocity['kind'], 'velocity_hover')
        self.assertEqual(velocity['world_velocity_xy_m_s'], [0.0, 0.0])
        self.assertGreater(velocity['sequence'], position['sequence'])

        self.assertTrue(control.acquire_external_attitude_coast())
        control.set_contact_attitude(-4.0, 1.0, 0.0)
        control.send(commander, command_timestamp=1.02, yaw_deg=0.0)
        attitude = control.sent_command_snapshot()
        self.assertEqual(attitude['kind'], 'attitude_zdistance')
        self.assertEqual(attitude['roll_deg'], -4.0)
        self.assertEqual(attitude['pitch_deg'], 1.0)
        self.assertGreater(attitude['sequence'], velocity['sequence'])

        history = control.sent_commands_after_sequence(position['sequence'])
        self.assertEqual(
            [command['kind'] for command in history],
            ['velocity_hover', 'attitude_zdistance'],
        )
        window = control.sent_commands_in_window(1.00, 1.02)
        self.assertEqual(
            [command['sequence'] for command in window],
            [velocity['sequence'], attitude['sequence']],
        )
        before_attitude_delay = control.sent_command_effective_at(
            1.049, delay_s=0.04
        )
        self.assertEqual(before_attitude_delay['kind'], 'position')
        attitude_effective = control.sent_command_effective_at(
            1.060, delay_s=0.04
        )
        self.assertEqual(attitude_effective['kind'], 'attitude_zdistance')
        self.assertAlmostEqual(
            attitude_effective['effective_query_time'], 1.02
        )

    def test_velocity_coast_rejects_impossible_kinematic_state_jump(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_state_kinematic_guard_enabled=True,
            coast_state_max_kinematic_residual_m=0.03,
            coast_state_max_implied_acceleration_m_s2=20.0,
            coast_state_max_sample_gap_s=0.05,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)

        self.assertFalse(control.update_coast_velocity(
            [0.0, -0.10, 1.0], [0.0, -0.06, 0.0], 1.02,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
        ))
        self.assertFalse(control.coast_state_sample_valid)
        self.assertEqual(
            control.coast_state_rejection_reason,
            'position_velocity_inconsistency',
        )
        self.assertEqual(control.coast_velocity_phase, 'fast_brake')
        rejection = control.consume_coast_state_rejection()
        self.assertEqual(rejection['reason'], 'position_velocity_inconsistency')
        self.assertGreater(rejection['kinematic_residual_m'], 0.03)
        self.assertIsNone(control.consume_coast_state_rejection())

        # The rejected sample becomes the new comparison baseline, so the next
        # coherent sample is accepted instead of causing a rejection cascade.
        self.assertFalse(control.update_coast_velocity(
            [0.0, -0.1012, 1.0], [0.0, -0.06, 0.0], 1.04,
            current_orientation_rpy=np.zeros(3),
            current_angular_velocity=np.zeros(3),
        ))
        self.assertTrue(control.coast_state_sample_valid)

    def test_velocity_coast_recontact_requires_explicit_sensor_authority(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.30, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        self.assertFalse(control.start_contact(
            'orientation', current_position=[0.0, 0.02, 1.0]
        ))
        self.assertTrue(control.start_contact(
            'orientation',
            current_position=[0.0, 0.02, 1.0],
            allow_coast_reentry=True,
        ))
        self.assertEqual(control.command_mode, 'attitude_zdistance')

    def test_predictive_velocity_coast_rebrakes_if_level_but_still_fast(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_velocity_rebrake_speed_m_s=0.15,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.10, 1.0], [0.0, 0.60, 0.0], 1.05,
            current_orientation_rpy=np.radians([20.0, 0.0, 0.0]),
            current_angular_velocity=np.zeros(3),
        ))
        self.assertEqual(control.coast_velocity_phase, 'predictive_unwind')
        self.assertTrue(control.consume_velocity_pid_reset_request())

        # Conservative early unwind may leave forward speed. Once the actual
        # attitude and rate are settled, resume a short zero-velocity pulse.
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.20, 1.0], [0.0, 0.20, 0.0], 1.15,
            current_orientation_rpy=np.radians([0.3, 0.0, 0.0]),
            current_angular_velocity=np.radians([2.0, 0.0, 0.0]),
        ))
        self.assertEqual(control.coast_velocity_phase, 'fast_brake')
        self.assertEqual(control.coast_velocity_rebrake_count, 1)
        self.assertTrue(control.consume_velocity_rebrake_request())
        self.assertFalse(control.consume_velocity_rebrake_request())
        control.send(commander, command_timestamp=1.15)
        np.testing.assert_allclose(
            commander.calls[-1][1], [0.0, 0.0, 0.0, 1.0]
        )

    def test_predictive_velocity_coast_can_disable_rebrake(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_velocity_rebrake_enabled=False,
            coast_velocity_rebrake_speed_m_s=0.02,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)
        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.10, 1.0], [0.0, 0.60, 0.0], 1.05,
            current_orientation_rpy=np.radians([20.0, 0.0, 0.0]),
            current_angular_velocity=np.zeros(3),
        ))
        self.assertEqual(control.coast_velocity_phase, 'predictive_unwind')

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.20, 1.0], [0.0, 0.20, 0.0], 1.15,
            current_orientation_rpy=np.radians([0.3, 0.0, 0.0]),
            current_angular_velocity=np.radians([2.0, 0.0, 0.0]),
        ))
        self.assertEqual(control.coast_velocity_phase, 'predictive_unwind')
        self.assertEqual(control.coast_velocity_rebrake_count, 0)
        self.assertFalse(control.consume_velocity_rebrake_request())

    def test_predictive_velocity_coast_does_not_rebrake_lateral_speed(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
            coast_velocity_predictive_unwind_enabled=True,
            coast_velocity_handoff_speed_m_s=0.03,
            coast_velocity_unwind_terminal_speed_m_s=0.02,
            coast_velocity_rebrake_speed_m_s=0.04,
            coast_velocity_handoff_max_rate_deg_s=5.0,
            coast_handoff_max_tilt_deg=0.5,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.60, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)

        self.assertFalse(control.update_coast_velocity(
            [0.0, 0.10, 1.0], [0.0, 0.60, 0.0], 1.05,
            current_orientation_rpy=np.radians([20.0, 0.0, 0.0]),
            current_angular_velocity=np.zeros(3),
        ))
        self.assertEqual(control.coast_velocity_phase, 'predictive_unwind')

        # Longitudinal speed is below the re-brake threshold. The larger total
        # speed comes from lateral drift, which may block handoff but must not
        # request a full longitudinal re-brake.
        self.assertFalse(control.update_coast_velocity(
            [0.01, 0.12, 1.0], [0.05, 0.01, 0.0], 1.15,
            current_orientation_rpy=np.radians([0.2, 0.0, 0.0]),
            current_angular_velocity=np.radians([2.0, 0.0, 0.0]),
        ))
        self.assertLess(control.brake_projected_speed_m_s, 0.04)
        self.assertGreater(np.linalg.norm([0.05, 0.01]), 0.04)
        self.assertEqual(control.coast_velocity_phase, 'predictive_unwind')
        self.assertFalse(control.consume_velocity_rebrake_request())
        self.assertGreater(
            np.linalg.norm(control.coast_velocity_command_xy_m_s), 0.0
        )

    def test_velocity_coast_hover_uses_fixed_nominal_z(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_velocity_braking_enabled=True,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.2], [1.0, 0.0, 0.0], 1.0,
            interaction_direction=[1.0, 0.0, 0.0], coast=True,
        ))
        self.assertEqual(control.hover_z, 1.2)
        self.assertEqual(control.velocity_coast_fixed_zdistance_m, 1.0)
        control.coast_velocity_command_xy_m_s = np.array([1.0, 0.0])

        control.send(commander, command_timestamp=1.0, yaw_deg=90.0)

        self.assertEqual(commander.calls[-1][0], 'hover')
        np.testing.assert_allclose(
            commander.calls[-1][1], [0.0, -1.0, 0.0, 1.0], atol=1e-12
        )
        self.assertEqual(
            control.sent_command_snapshot()['zdistance_m'], 1.0
        )

    def test_low_speed_direction_reversal_handoffs_after_state_dwell(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            brake_xy_speed_m_s=0.20,
            coast_attitude_response_delay_s=0.0,
            coast_attitude_time_constant_s=0.0,
            coast_handoff_speed_m_s=0.20,
            coast_handoff_max_acceleration_m_s2=10.0,
            coast_handoff_max_tilt_deg=30.0,
            coast_alignment_dwell_s=0.08,
            coast_level_handoff_delay_s=0.08,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.1, 1.0], [0.0, 0.30, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(
            [0.0, 0.1, 1.0], [0.0, 0.30, 0.0], timestamp=1.0,
        )
        control.send(commander, command_timestamp=1.0, yaw_deg=0.0)

        # Reversal immediately enters the same one-way level phase.
        self.assertFalse(control.update_coast_attitude(
            [0.02, 0.12, 1.0], [0.25, -0.01, 0.0],
            [0.0, 0.2, 1.0], [0.0, 0.0, 0.0], 1.01,
            command_timestamp=1.01,
        ))
        control.send(commander, command_timestamp=1.01, yaw_deg=0.0)
        self.assertEqual(control.command_mode, 'attitude_coast')

        # The level phase waits only for its fixed delay; it does not command a
        # second acceleration to eliminate reverse motion.
        self.assertFalse(control.update_coast_attitude(
            [0.021, 0.119, 1.0], [0.002, -0.002, 0.0],
            [0.0, 0.2, 1.0], [0.0, 0.0, 0.0], 1.02,
            command_timestamp=1.02,
        ))
        control.send(commander, command_timestamp=1.02, yaw_deg=0.0)
        self.assertFalse(control.update_coast_attitude(
            [0.022, 0.118, 1.0], [0.002, -0.002, 0.0],
            [0.0, 0.2, 1.0], [0.0, 0.0, 0.0], 1.03,
            command_timestamp=1.03,
        ))
        control.send(commander, command_timestamp=1.03, yaw_deg=0.0)
        self.assertTrue(control.update_coast_attitude(
            [0.023, 0.118, 1.0], [0.002, -0.002, 0.0],
            [0.0, 0.2, 1.0], [0.0, 0.0, 0.0], 1.12,
            command_timestamp=1.12,
        ))
        self.assertEqual(control.command_mode, 'position_hold')
        self.assertEqual(
            control.coast_handoff_reason,
            'timed_level_to_position_handoff',
        )
        self.assertEqual(
            control.brake_completion_reason,
            'timed_level_to_position_handoff',
        )
        np.testing.assert_allclose(control.hold_position, [0.023, 0.2, 1.0])
        self.assertFalse(control.coast_target_clamped_to_actual)
        self.assertTrue(control.coast_lateral_target_latched_to_actual)

    def test_bounded_lateral_drift_handoffs_at_measured_lateral_position(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_attitude_response_delay_s=0.0,
            coast_attitude_time_constant_s=0.0,
            coast_handoff_speed_m_s=0.04,
            coast_handoff_max_lateral_speed_m_s=0.15,
            coast_handoff_max_acceleration_m_s2=100.0,
            coast_handoff_max_tilt_deg=30.0,
            coast_alignment_dwell_s=0.0,
            coast_level_handoff_delay_s=0.0,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.1, 1.0], [0.0, 0.30, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(
            [0.0, 0.1, 1.0], [0.0, 0.30, 0.0], timestamp=1.0,
        )
        control.send(commander, command_timestamp=1.0, yaw_deg=0.0)

        self.assertFalse(control.update_coast_attitude(
            [0.05, 0.20, 1.0], [0.08, 0.0, 0.0],
            [0.0, 0.20, 1.0], [0.0, 0.0, 0.0], 1.01,
            current_orientation_rpy=np.zeros(3),
            command_timestamp=1.01,
        ))
        control.send(commander, command_timestamp=1.01, yaw_deg=0.0)
        self.assertTrue(control.update_coast_attitude(
            [0.05, 0.20, 1.0], [0.08, 0.0, 0.0],
            [0.0, 0.20, 1.0], [0.0, 0.0, 0.0], 1.02,
            current_orientation_rpy=np.zeros(3),
            command_timestamp=1.02,
        ))

        self.assertAlmostEqual(control.coast_lateral_speed_m_s, 0.08)
        np.testing.assert_allclose(control.hold_position, [0.05, 0.20, 1.0])
        self.assertFalse(control.coast_target_clamped_to_actual)
        self.assertTrue(control.coast_lateral_target_latched_to_actual)

    def test_calibrated_coast_rejects_unobservable_position_render_tail(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_calibrated_direction_xy=[0.0, 1.0],
        )
        self.assertTrue(control.start_contact('position'))

        with self.assertRaisesRegex(RuntimeError, 'orientation-rendered'):
            control.end_contact(
                [0.0, 0.1, 1.0], [0.0, 0.2, 0.0], 1.0,
                interaction_direction=[0.0, 1.0, 0.0], coast=True,
            )

    def test_release_records_force_momentum_then_captures_actual_stop_position(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            brake_xy_acceleration_m_s2=0.8,
            brake_xy_speed_m_s=0.04,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.10, 0.20, 1.0],
            [0.30, 0.10, 0.0],
            1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_force=[0.0, 0.12, 0.0],
            current_mass_kg=0.06,
        ))

        np.testing.assert_allclose(control.release_force_N, [0.0, 0.12, 0.0])
        np.testing.assert_allclose(
            control.release_momentum_kg_m_s, [0.018, 0.006, 0.0]
        )
        np.testing.assert_allclose(control.release_position_m, [0.10, 0.20, 1.0])
        self.assertIsNone(control.stopping_position_m)
        self.assertAlmostEqual(
            control.brake_force_feedforward_acceleration_m_s2, 2.0
        )

        self.assertTrue(control.update_braking(
            [0.11, 0.24, 1.0],
            [0.20, 0.03, 0.0],
            1.1,
            current_force=[0.0, 0.02, 0.0],
            current_mass_kg=0.06,
        ))
        np.testing.assert_allclose(
            control.stopping_position_m, [0.11, 0.24, 1.0]
        )
        np.testing.assert_allclose(
            control.hold_position, control.stopping_position_m
        )

    def test_coast_timeout_and_virtual_target_cannot_force_position_handoff(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            brake_xy_speed_m_s=0.04,
            coast_attitude_timeout_s=0.10,
            brake_timeout_s=0.20,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.30, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))

        self.assertFalse(control.update_coast_attitude(
            [0.0, 0.30, 1.0], [0.0, 0.25, 0.0],
            [0.0, -1.0, 1.0], [0.0, -0.50, 0.0], 1.25,
        ))
        self.assertEqual(control.command_mode, 'attitude_coast')
        self.assertEqual(
            control.coast_handoff_reason,
            'waiting_for_timed_level_handoff_after_timeout',
        )
        self.assertLessEqual(control.coast_tracking_power_w_per_kg, 0.0)

    def test_unconfirmed_potentiometer_release_cannot_handoff(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            brake_xy_speed_m_s=0.04,
            coast_attitude_response_delay_s=0.0,
            coast_attitude_time_constant_s=0.0,
            coast_handoff_max_acceleration_m_s2=10.0,
            coast_alignment_dwell_s=0.0,
            coast_level_handoff_delay_s=0.0,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.10, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_mass_kg=0.17,
            coast=True,
        ))
        control.send(commander, command_timestamp=1.0, yaw_deg=0.0)

        self.assertFalse(control.update_coast_attitude(
            [0.0, 0.02, 1.0], [0.0, 0.01, 0.0],
            [0.0, 0.02, 1.0], [0.0, 0.0, 0.0], 1.1,
            allow_position_handoff=False,
            command_timestamp=1.1,
        ))
        control.send(commander, command_timestamp=1.1, yaw_deg=0.0)
        self.assertEqual(control.command_mode, 'attitude_coast')

        control.confirm_release_candidate(
            [0.0, 0.02, 1.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.0], 1.1,
        )
        np.testing.assert_allclose(control.release_position_m, [0.0, 0.02, 1.0])
        np.testing.assert_allclose(
            control.release_momentum_kg_m_s, [0.0, 0.0017, 0.0]
        )
        self.assertFalse(control.update_coast_attitude(
            [0.0, 0.02, 1.0], [0.0, 0.01, 0.0],
            [0.0, -0.50, 1.0], [0.0, 0.0, 0.0], 1.11,
            command_timestamp=1.11,
        ))
        control.send(commander, command_timestamp=1.11, yaw_deg=0.0)
        self.assertTrue(control.update_coast_attitude(
            [0.0, 0.02, 1.0], [0.0, 0.01, 0.0],
            [0.0, -0.50, 1.0], [0.0, 0.0, 0.0], 1.12,
            command_timestamp=1.12,
        ))
        np.testing.assert_allclose(control.hold_position, [0.0, 0.02, 1.0])

    def test_release_confirmation_preserves_candidate_force_vector(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
        )
        self.assertTrue(control.start_contact('orientation'))
        candidate_force = np.array([0.15, 0.80, 0.0])
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.10, 0.20, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_force=candidate_force,
            current_mass_kg=0.17,
            coast=True,
        ))

        control.confirm_release_candidate(
            current_position=[0.01, 0.02, 1.0],
            current_velocity=[0.05, 0.10, 0.0],
            current_force=None,
            timestamp=1.1,
        )

        np.testing.assert_allclose(control.release_force_N, candidate_force)

    def test_cancelled_release_candidate_resumes_original_render_mode(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
        )
        self.assertTrue(control.start_contact(
            'position', [0.1, 0.0, 1.0]
        ))
        self.assertTrue(control.end_contact(
            [0.12, 0.0, 1.0], [0.20, 0.0, 0.0], 1.0,
            interaction_direction=[1.0, 0.0, 0.0],
        ))
        self.assertEqual(control.command_mode, 'attitude_braking')

        self.assertTrue(control.cancel_release_candidate([0.13, 0.0, 1.0]))
        self.assertEqual(control.command_mode, 'position_interaction')
        self.assertFalse(control.braking_mode)

    def test_cancelled_potentiometer_candidate_resumes_orientation_contact(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.1, 1.0], [0.0, 0.2, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        self.assertEqual(control.command_mode, 'attitude_coast')

        self.assertTrue(control.cancel_release_candidate([0.0, 0.12, 1.0]))
        self.assertEqual(control.command_mode, 'attitude_zdistance')
        self.assertEqual(control.contact_roll_deg, 0.0)
        self.assertEqual(control.contact_pitch_deg, 0.0)

    def test_cancelled_potentiometer_candidate_clears_pending_tail_pulse(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_attitude_response_delay_s=0.12,
            coast_attitude_time_constant_s=0.08,
            coast_candidate_tail_cancellation_max_acceleration_m_s2=1.0,
        )
        self.assertTrue(control.start_contact('orientation'))
        control.set_contact_attitude(5.82, 0.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=-1.0, yaw_deg=0.0)

        tracking = control.update_release_candidate_attitude(
            current_velocity=[0.0, 0.10, 0.0],
            current_orientation_rpy=np.radians([5.82, 0.0, 0.0]),
            interaction_direction=[0.0, 1.0, 0.0],
            timestamp=0.0,
            command_timestamp=0.0,
        )
        self.assertEqual(
            tracking['action'], 'canceling_predicted_reverse_tail'
        )
        self.assertTrue(control.cancel_tail_neutralization())

        # Resumed force rendering owns attitude again.  The cancelled pulse's
        # old deadline must not level this newer command later.
        control.set_contact_attitude(3.0, 0.0, yaw_deg=0.0)
        self.assertFalse(control.expire_tail_neutralization(1.0))
        self.assertEqual(control.contact_roll_deg, 3.0)

    def test_tail_pulse_uses_observed_period_and_actual_send_anchor(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_command_period_s=0.02,
        )
        self.assertTrue(control.start_contact('orientation'))
        control.set_contact_attitude(5.82, 0.0, yaw_deg=0.0)
        control.send(commander, command_timestamp=-1.0, yaw_deg=0.0)
        # start_contact intentionally clears timing from the previous contact.
        # Seed the intervals that this contact would have observed before the
        # release candidate begins; the one 90 ms outlier must not dominate.
        control._attitude_send_intervals_s = [0.03] * 7 + [0.09]
        self.assertAlmostEqual(
            control._estimated_attitude_command_hold_s(), 0.03
        )
        self.assertAlmostEqual(
            control._jerk_validation_command_hold_s(), 0.10
        )
        tracking = control.update_release_candidate_attitude(
            current_velocity=[0.0, 0.10, 0.0],
            current_orientation_rpy=np.radians([5.82, 0.0, 0.0]),
            interaction_direction=[0.0, 1.0, 0.0],
            timestamp=0.0,
            command_timestamp=0.0,
        )
        self.assertGreater(
            tracking['tail_cancellation_acceleration_m_s2'], 0.0
        )

        control.send(commander, command_timestamp=0.01, yaw_deg=0.0)
        anchored_deadline = control._tail_neutralization_deadline
        self.assertAlmostEqual(anchored_deadline, 0.04)
        control.send(commander, command_timestamp=0.015, yaw_deg=0.0)
        self.assertEqual(
            control._tail_neutralization_deadline, anchored_deadline
        )
        self.assertFalse(control.expire_tail_neutralization(0.039))
        self.assertTrue(control.expire_tail_neutralization(0.04))

    def test_detector_rearm_waits_for_post_braking_grace_time(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            rearm_delay_s=2.0,
        )
        self.assertTrue(control.start_contact())
        self.assertTrue(control.end_contact(
            [0.1, 0.0, 1.0],
            [0.2, 0.0, 0.0],
            1.0,
            interaction_direction=[1.0, 0.0, 0.0],
        ))
        self.assertTrue(control.update_braking(
            [0.11, 0.0, 1.0], [0.03, 0.0, 0.0], 1.1
        ))
        self.assertFalse(control.consume_detector_rearm(3.099))
        self.assertTrue(control.consume_detector_rearm(3.1))
        self.assertFalse(control.consume_detector_rearm(3.2))

    def test_predictive_position_handoff_uses_requested_target(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            rearm_delay_s=0.2,
        )
        self.assertTrue(control.start_contact())
        control.send(FakeCommander(), command_timestamp=0.9, yaw_deg=0.0)
        self.assertTrue(control.end_contact(
            [0.0, 0.1, 1.0], [0.0, 0.4, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0],
            current_orientation_rpy=[0.0, 0.0, 0.0],
            coast=True,
        ))
        history = control.sent_attitude_acceleration_history()
        self.assertTrue(history)
        self.assertIsNot(history[-1][1], control._coast_command_history[-1][1])
        self.assertTrue(control.set_predictive_position_target(
            [0.0, 0.35, 1.0], 1.2
        ))
        self.assertEqual(control.mode, control.POSITION_HOLD)
        self.assertEqual(
            control.brake_completion_reason,
            'predictive_model_position_handoff',
        )
        np.testing.assert_allclose(control.hold_position, [0.0, 0.35, 1.0])
        self.assertFalse(control.consume_detector_rearm(1.399))
        self.assertTrue(control.consume_detector_rearm(1.4))

    def test_shadow_translation_never_leaves_position_hold(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=True,
        )
        self.assertFalse(control.start_contact())
        control.send(commander)
        self.assertFalse(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], 1.0
        ))
        self.assertEqual(commander.calls, [
            ('position', (0.0, 0.0, 1.0, 0.0), {}),
        ])

    def test_attitude_braking_uses_locked_interaction_direction(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=90.0,
            shadow_mode=False,
        )
        self.assertTrue(control.start_contact())
        self.assertTrue(control.end_contact(
            [0.1, 0.2, 1.0],
            [0.4, -0.4, 0.0],
            1.0,
            interaction_direction=[1.0, 0.0, 0.0],
            current_orientation_rpy=np.radians([30.0, -4.0, 0.0]),
        ))
        control.send(commander)

        command = commander.calls[-1]
        self.assertEqual(command[0], 'zdistance')
        self.assertAlmostEqual(command[1][0], 0.0, places=12)
        self.assertAlmostEqual(
            command[1][1], np.degrees(np.arctan2(2.0 * 0.4, 9.81)),
            places=12,
        )
        np.testing.assert_allclose(control.brake_direction, [1.0, 0.0, 0.0])
        self.assertEqual(
            control.brake_direction_source, 'locked_interaction_direction'
        )
        # Large transverse X speed does not delay handoff after velocity along
        # the locked interaction direction reverses.
        self.assertTrue(control.update_braking(
            [0.3, 0.25, 1.0], [-0.01, -0.8, 0.0], 1.1
        ))
        np.testing.assert_allclose(control.hold_position, [0.3, 0.25, 1.0])

    def test_attitude_braking_timeout_holds_current_position(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            brake_xy_speed_m_s=0.04,
            brake_timeout_s=0.5,
        )
        self.assertTrue(control.start_contact())
        self.assertTrue(control.end_contact(
            [0.1, 0.0, 1.0],
            [0.4, 0.0, 0.0],
            1.0,
            interaction_direction=[1.0, 0.0, 0.0],
            current_orientation_rpy=np.radians([0.0, -5.0, 0.0]),
        ))
        self.assertTrue(control.update_braking(
            [0.4, 0.1, 1.0], [0.2, 0.0, 0.0], 1.5
        ))
        self.assertEqual(control.brake_completion_reason, 'braking_timeout')
        np.testing.assert_allclose(control.hold_position, [0.4, 0.1, 1.0])

    def test_translation_state_transitions_are_logged_at_info(self):
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
        )
        with self.assertLogs('Interaction.interactions', level='INFO') as logs:
            self.assertTrue(control.start_contact())
            self.assertTrue(control.end_contact(
                [0.1, 0.0, 1.0],
                [0.2, 0.0, 0.0],
                1.0,
                interaction_direction=[1.0, 0.0, 0.0],
                current_orientation_rpy=[0.0, 0.0, 0.0],
            ))
            self.assertTrue(control.update_braking(
                [0.11, 0.0, 1.0], [0.03, 0.0, 0.0], 1.1,
                current_orientation_rpy=[0.0, 0.0, 0.0],
            ))

        output = '\n'.join(logs.output)
        self.assertIn('HANDLING INTERACTION', output)
        self.assertIn('BRAKING', output)
        self.assertIn('HOVER', output)

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

    def test_sequential_translation_chirp_excites_only_one_axis_at_a_time(self):
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.bounds = None
        config = {
            'duration_s': 24.0,
            'translation_amplitude_m': [0.10, 0.10, 0.06],
            'translation_frequency_hz': [0.35, 0.35, 0.25],
            'translation_profile': 'sequential_chirp',
            'translation_chirp_end_hz': [1.30, 1.30, 0.85],
            'translation_axis_rest_s': 1.0,
            'translation_ramp_s': 0.6,
            'yaw_amplitude_deg': 0.0,
            'yaw_profile': 'sine',
            'yaw_frequency_hz': 0.2,
        }
        nominal = np.array([0.2, -0.1, 1.0])
        active_axes = []
        for elapsed_s in np.linspace(0.0, 24.0, 481):
            position, _ = controller._calibration_excitation_reference(
                nominal, 0.0, config, elapsed_s,
            )
            offset = position - nominal
            self.assertLessEqual(np.count_nonzero(np.abs(offset) > 1e-12), 1)
            self.assertTrue(np.all(np.abs(offset) <= [0.10, 0.10, 0.06]))
            active_axes.extend(np.flatnonzero(np.abs(offset) > 1e-5).tolist())

        self.assertEqual(set(active_axes), {0, 1, 2})
        rest_position, _ = controller._calibration_excitation_reference(
            nominal, 0.0, config, 7.8,
        )
        np.testing.assert_allclose(rest_position, nominal)

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
                'control_handoff': {
                    'coast_release_goto_takeover_enabled': True,
                    'coast_release_goto_deceleration_m_s2': 1.0,
                    'coast_release_goto_command_delay_s': 0.30,
                },
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
                'control_handoff': {
                    'coast_release_goto_takeover_enabled': True,
                    'coast_release_goto_deceleration_m_s2': 1.0,
                    'coast_release_goto_command_delay_s': 0.30,
                },
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
        self.assertFalse(
            observer_rows[-1]['initial_contact_detector_armed']
        )
        self.assertEqual(
            observer_rows[-1]['initial_contact_stationary_elapsed_s'], 0.0
        )
        config_rows = [
            entry for group, name, entry in logs.records
            if group == 'configs'
            and name == 'Onboard Wrench Interaction Config'
        ]
        self.assertFalse(
            config_rows[-1]['virtual_object']['force_rendering']['enabled']
        )
        self.assertEqual(
            config_rows[-1]['virtual_object']['release_behavior']['mode'],
            'observer_brake',
        )
        self.assertEqual(config_rows[-1]['initial_contact_arming'], {
            'enabled': True,
            'apply_after_each_interaction': True,
            'max_xy_speed_m_s': 0.03,
            'stationary_dwell_s': 0.5,
        })

if __name__ == '__main__':
    unittest.main()
