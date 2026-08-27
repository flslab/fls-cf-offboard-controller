import time
import unittest

import numpy as np

from Interaction.interactions import (
    GuidedTouchProtocol,
    InteractionsControl,
    TranslationControlHandoff,
    VirtualObjectPlanarMotion,
    force_inertia_attitude,
    heavy_inertia_attitude,
    inertia_command_mode,
    inertia_position_target,
    kinetic_energy_velocity,
    select_inertia_render_mode,
    virtual_resistance_force,
    velocity_inertia_mass_class,
    world_to_body_xy,
)


class FakeCommander:
    def __init__(self):
        self.calls = []

    def go_to(self, *args, **kwargs):
        self.calls.append(('go_to', args, kwargs))

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
        self.assertEqual(calls[0]['rearm_delay_s'], 2.0)
        self.assertEqual(
            calls[0]['virtual_object_config']['inertia_command'],
            'orientation',
        )

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
        self.assertEqual(calls[0]['duration'], 13.0)
        self.assertTrue(calls[0]['calibration_mode'])
        self.assertTrue(
            calls[0]['config']['calibration_excitation']['enabled']
        )
        self.assertTrue(calls[0]['config']['shadow_mode'])
        self.assertEqual(
            calls[0]['calibration_path'], '/tmp/test-calibration.json'
        )

    def test_active_translation_aims_release_tilt_along_braking_direction_then_holds(self):
        commander = FakeCommander()
        control = TranslationControlHandoff(
            initial_position=[0.0, 0.0, 1.0],
            yaw_deg=5.0,
            shadow_mode=False,
            brake_xy_acceleration_m_s2=1.0,
            brake_xy_speed_m_s=0.04,
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

    def test_position_rendering_coasts_then_uses_common_hover_handoff(self):
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
        self.assertEqual(control.command_mode, 'position_coast')
        self.assertFalse(control.update_braking(
            [0.13, 0.0, 1.0], [0.18, 0.0, 0.0], 1.1,
            coast_position=[0.17, 0.0, 1.0],
            coast_velocity=[0.10, 0.0, 0.0],
        ))
        control.send(commander)
        np.testing.assert_allclose(
            commander.calls[-1][1], [0.17, 0.0, 1.0, 0.0]
        )
        self.assertTrue(control.update_braking(
            [0.16, 0.0, 1.0], [0.03, 0.0, 0.0], 1.2,
            coast_position=[0.175, 0.0, 1.0],
            coast_velocity=[0.03, 0.0, 0.0],
        ))
        self.assertEqual(control.command_mode, 'position_hold')
        np.testing.assert_allclose(control.hold_position, [0.175, 0.0, 1.0])

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


if __name__ == '__main__':
    unittest.main()
