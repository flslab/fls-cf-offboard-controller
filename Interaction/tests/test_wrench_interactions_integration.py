import tempfile
import time
import unittest
from types import SimpleNamespace

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
    TranslationControlHandoff,
    VirtualObjectPlanarMotion,
    coast_braking_attitude,
    coast_target_braking_attitude,
    force_inertia_attitude,
    heavy_inertia_attitude,
    inertia_command_mode,
    inertia_position_target,
    kinetic_energy_velocity,
    potentiometer_release_direction,
    predict_delayed_zero_crossing,
    release_candidate_sensor_stale_watchdog,
    release_tail_neutralization_attitude,
    release_coast_initial_velocity,
    resolve_release_mode,
    resolve_wrench_nominal_target,
    select_inertia_render_mode,
    virtual_resistance_force,
    velocity_inertia_mass_class,
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

        gate.reset()

        self.assertFalse(gate.armed)
        self.assertFalse(gate.update([0.0, 0.0, 0.0], 2.0))
        self.assertTrue(gate.update([0.0, 0.0, 0.0], 2.5))


class ReleaseModeTests(unittest.TestCase):
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

    def test_sensor_default_release_requires_current_planar_fit(self):
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

            with self.assertLogs(level='ERROR') as captured:
                controller._run_translation()

        self.assertEqual(calls, [])
        self.assertTrue(any(
            'requires a current' in message for message in captured.output
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
        braking_plan = PlanarBrakingCalibration(
            calls[0]['config']['planar_braking_calibration'],
            start_after_s=12.0,
        )
        self.assertAlmostEqual(
            calls[0]['duration'], braking_plan.end_s + 0.5
        )
        self.assertTrue(calls[0]['calibration_mode'])
        self.assertTrue(
            calls[0]['config']['calibration_excitation']['enabled']
        )
        self.assertTrue(
            calls[0]['config']['startup_bias_calibration_enabled']
        )
        self.assertTrue(calls[0]['config']['shadow_mode'])
        self.assertTrue(
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

        # One low-speed sample is not enough, and a rebound resets the dwell.
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
        self.assertTrue(control.update_coast_attitude(
            [0.0, 0.22, 1.0], [0.0, 0.02, 0.0],
            [0.0, 0.11, 1.0], [0.0, 0.0, 0.0], 1.35,
            command_timestamp=1.35,
        ))
        self.assertEqual(control.command_mode, 'position_hold')
        control.send(commander)
        self.assertEqual(commander.calls[-1][0], 'position')
        np.testing.assert_allclose(control.hold_position, [0.0, 0.22, 1.0])
        np.testing.assert_allclose(
            control.stopping_position_m, [0.0, 0.22, 1.0]
        )
        self.assertTrue(control.coast_target_clamped_to_actual)
        self.assertEqual(control.brake_completion_reason, 'actual_state_settled')

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

        self.assertTrue(control.update_coast_attitude(
            [0.0, 0.16, 1.0], [0.0, 0.01, 0.0],
            [0.0, 0.28, 1.0], [0.0, 0.0, 0.0], 1.01,
            command_timestamp=1.01,
        ))
        np.testing.assert_allclose(control.hold_position, [0.0, 0.28, 1.0])
        np.testing.assert_allclose(
            control.coast_handoff_actual_position_m, [0.0, 0.16, 1.0]
        )
        self.assertFalse(control.coast_target_clamped_to_actual)

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
        self.assertLess(abs(speed), 0.04)

    def test_coast_handoff_requires_level_attitude_and_continuous_dwell(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=0.0,
            shadow_mode=False,
            coast_attitude_response_delay_s=0.0,
            coast_attitude_time_constant_s=0.0,
            coast_handoff_speed_m_s=0.04,
            coast_handoff_max_tilt_deg=3.0,
            coast_handoff_max_acceleration_m_s2=0.35,
            coast_alignment_dwell_s=0.05,
        )
        self.assertTrue(control.start_contact('orientation'))
        self.assertTrue(control.end_contact(
            [0.0, 0.0, 1.0], [0.0, 0.03, 0.0], 1.0,
            interaction_direction=[0.0, 1.0, 0.0], coast=True,
        ))
        control.confirm_release_candidate(timestamp=1.0)
        control.send(commander, command_timestamp=1.0, yaw_deg=0.0)

        self.assertFalse(control.update_coast_attitude(
            [0.0, 0.001, 1.0], [0.0, 0.03, 0.0],
            [0.0, 0.20, 1.0], [0.0, 0.0, 0.0], 1.01,
            current_orientation_rpy=np.radians([4.0, 0.0, 0.0]),
            command_timestamp=1.01,
        ))
        control.send(commander, command_timestamp=1.01, yaw_deg=0.0)
        self.assertFalse(control.coast_handoff_state_ready)
        self.assertFalse(control.update_coast_attitude(
            [0.0, 0.001, 1.0], [0.0, 0.03, 0.0],
            [0.0, 0.20, 1.0], [0.0, 0.0, 0.0], 1.02,
            current_orientation_rpy=np.zeros(3),
            command_timestamp=1.02,
        ))
        control.send(commander, command_timestamp=1.02, yaw_deg=0.0)
        self.assertTrue(control.coast_handoff_state_ready)
        self.assertFalse(control.update_coast_attitude(
            [0.0, 0.001, 1.0], [0.0, 0.03, 0.0],
            [0.0, 0.20, 1.0], [0.0, 0.0, 0.0], 1.04,
            current_orientation_rpy=np.zeros(3),
            command_timestamp=1.04,
        ))
        control.send(commander, command_timestamp=1.04, yaw_deg=0.0)
        self.assertTrue(control.update_coast_attitude(
            [0.0, 0.001, 1.0], [0.0, 0.03, 0.0],
            [0.0, 0.20, 1.0], [0.0, 0.0, 0.0], 1.071,
            current_orientation_rpy=np.zeros(3),
            command_timestamp=1.071,
        ))
        self.assertEqual(control.coast_handoff_reason, 'actual_state_settled')

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

        # Reversal alone is insufficient while lateral speed is still high.
        self.assertFalse(control.update_coast_attitude(
            [0.02, 0.12, 1.0], [0.25, -0.01, 0.0],
            [0.0, 0.2, 1.0], [0.0, 0.0, 0.0], 1.01,
            command_timestamp=1.01,
        ))
        control.send(commander, command_timestamp=1.01, yaw_deg=0.0)
        self.assertEqual(control.command_mode, 'attitude_coast')

        # Crossing the original direction is not enough by itself: measured
        # speed/tilt/acceleration must remain settled for the configured dwell.
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
            'motion_reversed_state_settled',
        )
        self.assertEqual(
            control.brake_completion_reason,
            'motion_reversed_state_settled',
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

        self.assertTrue(control.update_coast_attitude(
            [0.05, 0.20, 1.0], [0.08, 0.0, 0.0],
            [0.0, 0.20, 1.0], [0.0, 0.0, 0.0], 1.01,
            current_orientation_rpy=np.zeros(3),
            command_timestamp=1.01,
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
            'waiting_for_actual_stop_after_timeout',
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
        self.assertTrue(control.update_coast_attitude(
            [0.0, 0.02, 1.0], [0.0, 0.01, 0.0],
            [0.0, -0.50, 1.0], [0.0, 0.0, 0.0], 1.11,
            command_timestamp=1.11,
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
            'max_xy_speed_m_s': 0.03,
            'stationary_dwell_s': 0.5,
        })


if __name__ == '__main__':
    unittest.main()
