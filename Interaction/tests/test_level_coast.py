import copy
from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from controller import Controller, LowBatteryException
from Interaction.interactions import InteractionsControl, StaleLocalizationError, BoundaryExceededError
from Interaction.level_coast import (
    LevelCoastCycle, VelocityContactDetector, validate_level_coast,
)
from Interaction.onboard_wrench_interaction_pipeline import OnboardMomentumWrenchPipeline
from Interaction.tests.test_wrench_interactions_integration import FakeCommander, FakeOnboardLogManager


def configuration(detector='potentiometer'):
    return {
        'behavior': 'level_coast', 'detection_method': detector,
        'duration': .85, 'grace_time': .10,
        'level_coast': {'stop_speed_m_s': .03,
            'velocity': {'onset_dwell_s': .01, 'release_dwell_s': .01}},
        'virtual_object': {
            'contact_detection': {'force_threshold_n': .18, 'onset_dwell_s': .01},
            'release_behavior': {'unloaded_force_n': .17, 'unloaded_dwell_s': .01}},
        'wrench_interaction': {
            'state_source': 'onboard', 'shadow_mode': False,
            'startup_bias_calibration_enabled': False,
            'firmware_auto_brake': {'enabled': False},
            'initial_contact_arming': {'stationary_dwell_s': .02},
            'detection': {'yaw': {'enabled': False}, 'translation': {
                'onset_evidence_s': .001, 'release_time_s': .01,
                'release_projection_axes': [0, 1]}},
        },
    }


class LevelCoastStateTests(unittest.TestCase):
    def test_coast_mode_inherits_contact_and_rejects_invalid_values(self):
        config = configuration()
        options = validate_level_coast(config, sensor_available=True)
        self.assertEqual(options['command_mode'], 'orientation')
        self.assertEqual(options['coast_command_mode'], 'orientation')
        config['level_coast']['command_mode'] = 'position'
        self.assertEqual(validate_level_coast(config, sensor_available=True)['coast_command_mode'], 'position')
        for key in ('command_mode', 'coast_command_mode'):
            for bad in ('ori', 'pos', '', None, True, ['position']):
                with self.subTest(key=key, bad=bad):
                    invalid = configuration()
                    invalid['level_coast'][key] = bad
                    with self.assertRaisesRegex(ValueError, key):
                        validate_level_coast(invalid, sensor_available=True)

    def test_delay_keeps_position_and_remembers_early_release(self):
        cycle = LevelCoastCycle([0, 0, 1], .03, .03, 'release', .1)
        cycle.update([0, 0, 1], [0, 0, 0], 0, armed=True)
        cycle.update([.1, 0, 1], [.2, 0, 0], 1., started=True)
        self.assertEqual(cycle.phase, 'contact')
        self.assertFalse(cycle.level)
        self.assertTrue(cycle.detection_enabled(1.01))  # Release remains active.
        np.testing.assert_allclose(cycle.hold_position, [0, 0, 1])
        cycle.update([.2, 0, 1], [.01, 0, 0], 1.02, released=True)
        self.assertEqual(cycle.phase, 'coast')
        self.assertEqual(cycle.grace_started, 1.02)
        self.assertFalse(cycle.detection_enabled(1.09))
        cycle.update([.3, 0, 1], [.01, 0, 0], 1.09)
        self.assertFalse(cycle.level)
        cycle.update([.3, 0, 1], [.01, 0, 0], 1.11)
        self.assertTrue(cycle.level)  # First ori only after the entire delay.
        cycle.update([.3, 0, 1], [.01, 0, 0], 1.12)
        self.assertEqual(cycle.phase, 'ready')
        np.testing.assert_allclose(cycle.hold_position, [.3, 0, 1])

    def test_coast_preemption_restarts_delay_and_holds_current_xy(self):
        cycle = LevelCoastCycle([0, 0, 1], .03, .03, 'release', .1)
        cycle.update([0, 0, 1], [0, 0, 0], 0, armed=True)
        cycle.update([0, 0, 1], [.2, 0, 0], 1., started=True)
        cycle.update([.1, 0, 1], [.2, 0, 0], 1.2, released=True)
        self.assertTrue(cycle.level)
        cycle.update([.5, .2, 1.2], [.2, 0, 0], 1.3, started=True)
        self.assertFalse(cycle.level)
        np.testing.assert_allclose(cycle.hold_position, [.5, .2, 1.])
        cycle.update([.6, .2, 1.], [.2, 0, 0], 1.39)
        self.assertFalse(cycle.level)
        cycle.update([.6, .2, 1.], [.2, 0, 0], 1.41)
        self.assertTrue(cycle.level)

    def test_delay_constant_must_be_finite_and_nonnegative(self):
        for value in (-.1, float('nan'), float('inf')):
            with self.subTest(value=value), patch('Interaction.level_coast.DETECTION_TO_ORI_DELAY_S', value):
                with self.assertRaisesRegex(ValueError, 'DETECTION_TO_ORI_DELAY_S'):
                    validate_level_coast(configuration(), sensor_available=True)

    def test_release_grace_can_preempt_coast_and_restarts_each_release(self):
        cycle = LevelCoastCycle([0, 0, 1], .03, .3, 'release')
        cycle.update([0, 0, 1], [0, 0, 0], 0, armed=True)
        cycle.update([0, 0, 1], [.2, 0, 0], .1, started=True)
        cycle.update([0, 0, 1], [.2, 0, 0], 1., released=True)
        self.assertEqual(cycle.grace_started, 1.)
        self.assertFalse(cycle.detection_enabled(1.29))
        cycle.update([.2, 0, 1], [.2, 0, 0], 1.29, started=True)
        self.assertEqual(cycle.phase, 'coast')
        self.assertTrue(cycle.detection_enabled(1.31))
        # A new onset wins even if speed reaches the stop threshold this sample.
        cycle.update([.2, 0, 1], [.02, 0, 0], 1.31, started=True)
        self.assertEqual(cycle.phase, 'contact')
        self.assertIsNone(cycle.grace_started)
        cycle.update([.4, 0, 1], [.2, 0, 0], 2., released=True)
        self.assertEqual(cycle.grace_started, 2.)
        self.assertFalse(cycle.detection_enabled(2.29))
        self.assertTrue(cycle.detection_enabled(2.31))

    def test_release_grace_low_speed_capture_keeps_remaining_timer(self):
        cycle = LevelCoastCycle([0, 0, 1], .03, .3, 'release')
        cycle.update([0, 0, 1], [0, 0, 0], 0, armed=True)
        cycle.update([0, 0, 1], [.2, 0, 0], .1, started=True)
        cycle.update([.2, 0, 1], [.02, 0, 0], 1., released=True)
        self.assertEqual(cycle.phase, 'grace')
        self.assertFalse(cycle.level)
        np.testing.assert_allclose(cycle.hold_position, [.2, 0, 1])
        cycle.update([.2, 0, 1], [.04, 0, 0], 1.31)
        self.assertEqual(cycle.phase, 'ready')  # No repeated stationary dwell.

    def test_zero_release_grace_and_late_capture(self):
        cycle = LevelCoastCycle([0, 0, 1], .03, 0., 'release')
        cycle.update([0, 0, 1], [0, 0, 0], 0, armed=True)
        cycle.update([0, 0, 1], [.2, 0, 0], .1, started=True)
        cycle.update([0, 0, 1], [.2, 0, 0], 1., released=True)
        self.assertTrue(cycle.detection_enabled(1.))
        cycle.update([.2, 0, 1.2], [.02, 0, 0], 1.1)
        self.assertEqual(cycle.phase, 'ready')
        np.testing.assert_allclose(cycle.hold_position, [.2, 0, 1])

    def test_position_coast_requires_release_and_full_xy_speed_below_threshold(self):
        cycle = LevelCoastCycle([0, 0, 1], .03, .5, coast_command_mode='position')
        cycle.update([0, 0, 1], [0, 0, 0], 0, armed=True)
        cycle.update([0, 0, 1], [0, 0, 0], .1, started=True)
        cycle.update([0, 0, 1], [0, 0, 0], 1.)
        self.assertEqual(cycle.phase, 'contact')
        cycle.update([.2, .3, 1], [-.2, 0, 0], 2., released=True)
        self.assertEqual(cycle.phase, 'coast')
        cycle.update([.2, .3, 1], [.025, .025, 0], 3.)
        self.assertEqual(cycle.phase, 'coast')
        cycle.update([.4, .5, 1.2], [.02, 0, .2], 4.)
        self.assertEqual(cycle.phase, 'grace')
        np.testing.assert_allclose(cycle.hold_position, [.4, .5, 1])
        cycle.update([.7, .8, 1], [0, 0, 0], 4.49, started=True)
        self.assertEqual(cycle.phase, 'grace')
        cycle.update([.7, .8, 1], [0, 0, 0], 4.5, started=True)
        self.assertEqual(cycle.phase, 'prepare')

    def test_orientation_stop_uses_signed_projection_and_ignores_lateral_speed(self):
        for direction in ([1, 0], [0, -1], [3, 4]):
            d = np.asarray(direction, dtype=float) / np.linalg.norm(direction)
            lateral = np.array([-d[1], d[0]]) * .2
            for final_speed in (.02, -.2):
                with self.subTest(direction=direction, final_speed=final_speed):
                    cycle = LevelCoastCycle([0, 0, 1], .03, .5)
                    cycle.update([0, 0, 1], [0, 0, 0], 0, armed=True)
                    cycle.update([0, 0, 1], [0, 0, 0], .1, started=True,
                                 interaction_direction=direction, interaction_direction_source='test_force')
                    cycle.update([0, 0, 1], [*d * .04 + lateral, 0], 1, released=True)
                    self.assertEqual(cycle.phase, 'coast')
                    velocity = [*(d * final_speed + lateral), 0]
                    cycle.update([.2, .3, 1], velocity, 1.1)
                    self.assertEqual(cycle.phase, 'grace')
                    self.assertEqual(cycle.grace_started, 1.1)
                    status = cycle.stop_status(velocity)
                    self.assertAlmostEqual(status['interaction_velocity_m_s'], final_speed)
                    self.assertEqual(status['stop_speed_metric'], 'interaction_projection')
                    np.testing.assert_allclose(cycle.hold_position, [.2, .3, 1])

    def test_direction_stays_fixed_until_accepted_coast_preemption(self):
        cycle = LevelCoastCycle([0, 0, 1], .03, .1, 'release')
        cycle.update([0, 0, 1], [0, 0, 0], 0, armed=True)
        cycle.update([0, 0, 1], [.2, 0, 0], .1, started=True)
        cycle.update([0, 0, 1], [.1, .4, 0], 1, released=True,
                     interaction_direction=[0, 1])
        cycle.update([0, 0, 1], [.1, .4, 0], 1.05, started=True,
                     interaction_direction=[0, -1])  # Grace has not expired.
        np.testing.assert_allclose(cycle.interaction_direction_xy, [1, 0])
        cycle.update([0, 0, 1], [.2, -.2, 0], 1.2, started=True,
                     interaction_direction=[0, -3], interaction_direction_source='new_force')
        self.assertEqual(cycle.phase, 'contact')
        np.testing.assert_allclose(cycle.interaction_direction_xy, [0, -1])
        cycle.update([0, 0, 1], [.2, -.05, 0], 2, released=True)
        self.assertEqual(cycle.phase, 'coast')
        cycle.update([0, 0, 1], [.2, -.02, 0], 2.02)
        self.assertEqual(cycle.phase, 'grace')
        self.assertEqual(cycle.grace_started, 2)

    def test_missing_direction_uses_full_speed_without_inventing_an_axis(self):
        cycle = LevelCoastCycle([0, 0, 1], .03, .1)
        cycle.update([0, 0, 1], [0, 0, 0], 0, armed=True)
        cycle.update([0, 0, 1], [0, 0, 0], .1, started=True, interaction_direction=[0, 0, 1])
        cycle.update([0, 0, 1], [.1, .2, 0], .2, released=True)
        self.assertEqual(cycle.phase, 'coast')
        self.assertIsNone(cycle.interaction_direction_xy)
        self.assertEqual(cycle.stop_status([.1, .2, 0])['stop_speed_metric'], 'xy_norm_no_direction')
        cycle.update([0, 0, 1], [.01, .01, 0], .3)
        self.assertEqual(cycle.phase, 'grace')

    def test_velocity_release_requires_continuous_evidence(self):
        detector = VelocityContactDetector(**validate_level_coast(
            configuration('vel'), sensor_available=False)['velocity'])
        self.assertEqual(detector.update(0, 0, True), (False, False))
        detector.update(.2, .01, True)
        self.assertEqual(detector.update(.2, .03, True), (True, False))
        self.assertEqual(detector.update(.02, .04, True), (False, False))
        self.assertEqual(detector.update(.02, .30, True), (False, False))
        self.assertEqual(detector.update(.02, .32, True), (False, True))

    def test_preflight_rejects_conflicting_authority_missing_sensor_and_bad_thresholds(self):
        variants = [
            ('firmware', lambda c: c['wrench_interaction']['firmware_auto_brake'].update(enabled=True)),
            ('shadow', lambda c: c['wrench_interaction'].update(shadow_mode=True)),
            ('threshold', lambda c: c['level_coast'].update(stop_speed_m_s=float('nan'))),
            ('duration', lambda c: c.update(duration=-1)),
            ('grace', lambda c: c.update(grace_time=-1)),
            ('grace_start', lambda c: c['level_coast'].update(grace_start='unknown')),
            ('follow_yaw', lambda c: c['level_coast'].update(follow_yaw='false')),
            ('yaw_rate_damping', lambda c: c['level_coast'].update(yaw_rate_damping='true')),
            ('yaw_deadband', lambda c: c['level_coast'].update(yaw_rate_deadband_deg_s=-1)),
            ('yaw_deadband_nan', lambda c: c['level_coast'].update(yaw_rate_deadband_deg_s=float('nan'))),
            ('yaw_conflict', lambda c: c['level_coast'].update(yaw_rate_damping=True, follow_yaw=True)),
            ('detection_method', lambda c: c.update(detection_method='unknown')),
            ('old_pipeline_name', lambda c: c.update(detection_method='momentum_impulse')),
            ('old_detector_key', lambda c: c['level_coast'].update(detector='model')),
        ]
        for name, mutate in variants:
            with self.subTest(name=name):
                config = configuration()
                mutate(config)
                with self.assertRaises(ValueError):
                    validate_level_coast(config, sensor_available=True)
        control = Controller.__new__(Controller)
        control.args = SimpleNamespace(calibrate=False, sense=False)
        control.mission = {'Interaction': {'config': configuration()}}
        with self.assertRaisesRegex(ValueError, '--sense'):
            control.prepare_firmware_auto_brake()
        control.args.sense = True
        control.prepare_firmware_auto_brake()
        self.assertFalse(control.firmware_auto_brake_enabled)


class LevelCoastLoopTests(unittest.TestCase):
    def run_scenario(self, detector='potentiometer', *, duration=.85, fault=None,
                     grace_start='speed_threshold', grace_time=.10,
                     pressed_fn=None, speed_fn=None, follow_yaw=False,
                     yaw_fn=None, state_delay=0., target_yaw=0., yaw_rate_damping=False,
                     ori_delay=0., yaw_confirm_delay=0., yaw_rate_fn=None,
                     sensor_present=True, sensor_fresh=True, command_mode='orientation',
                     position_options=None, coast_command_mode=None, duplicate_times=(),
                     velocity_fn=None, force_direction_fn=None, record_potentiometer=False,
                     vicon_velocity_fn=None):
        config = configuration(detector)
        config['record_potentiometer'] = record_potentiometer
        config['duration'] = duration
        config['grace_time'] = grace_time
        config['level_coast']['grace_start'] = grace_start
        config['level_coast']['follow_yaw'] = follow_yaw
        config['level_coast']['yaw_rate_damping'] = yaw_rate_damping
        config['level_coast']['command_mode'] = command_mode
        if coast_command_mode is not None:
            config['level_coast']['coast_command_mode'] = coast_command_mode
        if position_options is not None:
            config['level_coast']['position_control'] = position_options
        original = copy.deepcopy(config)
        clock = {'t': 0.}
        control = InteractionsControl.__new__(InteractionsControl)
        control.drone_id = 'lb11'
        control.ctrl_rate = 100
        control.mission = {'Interaction': {'config': config},
                           'drones': {'lb11': {'target': [0, 0, 1, target_yaw]}}}
        control.bounds = dict(x_min=-1, x_max=1, y_min=-1, y_max=1, z_min=.3, z_max=2)
        control.lo_commander = FakeCommander()
        control.hl_commander = FakeCommander()
        control.cf = SimpleNamespace(param=SimpleNamespace(set_value=Mock(), set_value_raw=Mock()))
        control.cf._offboard_yaw_damping_active = False
        if 'position' in (command_mode, coast_command_mode):
            from Interaction.tests.test_position_follow import parameters
            control.cf._offboard_position_pid = SimpleNamespace(prepared=fault != 'pid', parameters=parameters())
        yaw_requests = []
        yaw_samples = []
        def request_yaw(rate):
            yaw_requests.append(clock['t'])
            if fault == 'yaw':
                raise RuntimeError('yaw activation failed')
            ready = clock['t'] - yaw_requests[0] >= yaw_confirm_delay
            control.cf._offboard_yaw_damping_active = ready
            return ready
        def update_yaw(rate):
            yaw_samples.append((clock['t'], rate))
            if fault == 'yaw_switch' and clock['t'] >= .2:
                raise RuntimeError('yaw switching failed')
            active = abs(np.degrees(rate)) >= 10.
            return dict(yaw_rate_measured_deg_s=np.degrees(rate),
                        yaw_rate_damping_requested=active,
                        yaw_rate_damping_output_enabled=active,
                        yaw_rate_damping_switch_pending=False)
        control.cf._offboard_yaw_damping_guard = SimpleNamespace(
            prepared=True, request_enable=Mock(side_effect=request_yaw),
            update=Mock(side_effect=update_yaw), finish=Mock())
        control.yaw_requests = yaw_requests
        control.yaw_samples = yaw_samples
        control.pid_attitude_source = 'post-release-15state'
        control._pid_15state_control_active = False
        control.force_sensor = object() if sensor_present else None
        control._unsubscribe_contact_attitude_shadow = Mock()
        control.log_manager = FakeOnboardLogManager(1000.)
        control._log_event = Mock()
        control._handoff_translation_hold = Mock()
        commands = []
        control.command_authorities = []
        for name in ('send_position_setpoint', 'send_zdistance_setpoint'):
            original_sender = getattr(control.lo_commander, name)
            def send(*args, name=name, original_sender=original_sender):
                commands.append((clock['t'], name, args))
                control.command_authorities.append(control._pid_15state_control_active)
                original_sender(*args)
            setattr(control.lo_commander, name, send)

        def sleep(seconds):
            if fault == 'battery' and clock['t'] >= .2:
                raise LowBatteryException('test battery abort')
            clock['t'] = round(clock['t'] + seconds, 6)
            self.assertLess(clock['t'], 2., 'duration did not terminate the loop')

        def pressed():
            t = clock['t']
            if pressed_fn is not None:
                return pressed_fn(t)
            return .08 <= t < .16 or .21 <= t < .26 or .36 <= t < .38 or .52 <= t < .60

        def state():
            t = clock['t']
            if t in duplicate_times:
                t = round(t - .01, 6)
            if t < state_delay:
                return None
            speed = .12 if .08 <= t < .3 or .52 <= t < .66 else .02 if t >= .3 else 0.
            if speed_fn is not None:
                speed = speed_fn(t)
            return dict(time=1000.+t-(.2 if fault == 'state' and t >= .2 else 0),
                position=np.array([2. if fault == 'boundary' and t >= .2 else t/10, 0., 1.]),
                velocity=np.asarray(velocity_fn(t) if velocity_fn else [speed, 0., 0.], dtype=float),
                attitude_rpy=np.array([0., 0., np.radians(yaw_fn(t) if yaw_fn else 0.)]),
                angular_velocity=np.array([0., 0., np.radians(yaw_rate_fn(t) if yaw_rate_fn else 0.)]),
                position_skew_s=0., angular_rate_skew_s=0., yaw_control_skew_s=None,
                yaw_control_command=None, motor_skew_s=0., motor_state={
                    'time':1000.+t-(.2 if fault == 'motor' and t >= .2 else 0),
                    'motor.m1':30000, 'motor.m2':30000, 'motor.m3':30000, 'motor.m4':30000, 'pm.vbat':8.})

        def force_world():
            direction = force_direction_fn(clock['t']) if force_direction_fn else [1., 0., 0.]
            return np.asarray(direction, dtype=float) * (.3 if pressed() else 0.)

        def sensor(*_):
            t = clock['t']
            return dict(force_sensor_fresh=sensor_fresh and not (fault == 'sensor' and t >= .2),
                        force_sensor_sample_monotonic_time=t, force_sensor_sample_time=1000.+t,
                        force_sensor_compression_force_N=.3 if pressed() else 0.,
                        force_sensor_external_force_N=force_world().tolist())

        control._get_synchronized_onboard_wrench_state = state
        def vicon_velocity(reference_state):
            if fault == 'vicon' and clock['t'] >= .2:
                raise StaleLocalizationError('test stale Vicon velocity')
            velocity = (vicon_velocity_fn(clock['t']) if vicon_velocity_fn
                        else reference_state['velocity'])
            return np.asarray(velocity, dtype=float), reference_state['time'], 0.
        control._vicon_velocity_reference_for_onboard_state = vicon_velocity
        control._safe_sleep = sleep
        control._force_sensor_log_fields = sensor
        real_update = OnboardMomentumWrenchPipeline.update

        def model_update(pipeline, **kwargs):
            # Use the real model contact detector with controlled wrench input;
            # model fitting itself is covered by the pipeline's existing tests.
            detector_state = pipeline.detector
            pipeline.detector = copy.deepcopy(detector_state)
            pipeline.detector.translation.enabled = False
            result = real_update(pipeline, **kwargs)
            pipeline.detector = detector_state
            estimate = replace(result.estimate, external_force=force_world(), force_covariance=np.eye(3)*.0001,
                measurement_rejected=False)
            return replace(result, estimate=estimate, contacts=pipeline.detector.update(estimate))

        expected = {'battery':LowBatteryException, 'state':StaleLocalizationError,
                    'boundary':BoundaryExceededError, 'motor':RuntimeError, 'sensor':RuntimeError,
                    'yaw':RuntimeError, 'yaw_switch':RuntimeError, 'target':BoundaryExceededError,
                    'pid':RuntimeError, 'vicon':StaleLocalizationError}
        if fault == 'target':
            control.bounds['x_max'] = .05
        with patch('Interaction.level_coast.time.time', side_effect=lambda:1000.+clock['t']), \
                patch('Interaction.level_coast.DETECTION_TO_ORI_DELAY_S', ori_delay), \
                patch('Interaction.level_coast.time.monotonic', side_effect=lambda:clock['t']), \
                patch('Interaction.level_coast.apply_detection_calibration', side_effect=lambda c,*_:c), \
                patch.object(OnboardMomentumWrenchPipeline, 'update',
                             model_update if detector == 'model' else real_update):
            if fault:
                with self.assertRaises(expected[fault]):
                    control._run_translation()
            else:
                control._run_translation()
        self.assertEqual(config, original)
        control._handoff_translation_hold.assert_not_called()
        self.assertEqual(control.hl_commander.calls, [])
        phases = [c.args[1] for c in control._log_event.call_args_list
                  if c.args[0] == 'Level Coast Phase Changed']
        return control, commands, phases, clock['t']

    def test_handoff_uses_vicon_velocity_while_detector_retains_onboard_velocity(self):
        for detector in ('potentiometer', 'model', 'vel'):
            for coast in ('orientation', 'position'):
                with self.subTest(detector=detector, coast=coast):
                    _, _, phases, _ = self.run_scenario(
                        detector, coast_command_mode=coast,
                        pressed_fn=lambda t: .08 <= t < .16,
                        speed_fn=lambda t: .12 if .08 <= t < .25 else .02,
                        vicon_velocity_fn=lambda t: [.2 if t < .4 else .02, 0., 0.])
                    capture = next(p for p in phases if p['previous'] == 'coast')
                    self.assertAlmostEqual(capture['elapsed_s'], .4)
                    self.assertEqual(capture['stop_velocity_source'], 'vicon_position_kf')
                    self.assertAlmostEqual(capture['onboard_stop_speed_value_m_s'], .02)
                    self.assertAlmostEqual(capture['stop_speed_value_m_s'], .02)

    def test_stale_vicon_does_not_fall_back_to_onboard_handoff(self):
        self.run_scenario('potentiometer', fault='vicon')

    def test_vicon_low_speed_can_capture_while_onboard_velocity_is_still_high(self):
        for detector in ('potentiometer', 'model'):
            with self.subTest(detector=detector):
                _, _, phases, _ = self.run_scenario(
                    detector, pressed_fn=lambda t: .08 <= t < .16,
                    speed_fn=lambda t: .12 if t >= .08 else 0.,
                    vicon_velocity_fn=lambda t: [.12 if t < .3 else .02, 0., 0.])
                capture = next(p for p in phases if p['previous'] == 'coast')
                self.assertAlmostEqual(capture['elapsed_s'], .3)
                self.assertAlmostEqual(capture['onboard_stop_speed_value_m_s'], .12)
                self.assertAlmostEqual(capture['stop_speed_value_m_s'], .02)

    def test_pot_and_model_orientation_capture_ignores_lateral_drift_but_position_does_not(self):
        def velocity(t):
            if t < .08:
                return [0, 0, 0]
            if t < .16:
                return [.12, .2, 0]
            if t < .30:
                return [.06, .2, 0]
            return [.02, .2, 0] if t < .6 else [.01, .01, 0]

        for detector in ('potentiometer', 'model'):
            for contact in ('orientation', 'position'):
                for coast in ('orientation', 'position'):
                    with self.subTest(detector=detector, contact=contact, coast=coast):
                        control, commands, phases, _ = self.run_scenario(
                            detector, command_mode=contact, coast_command_mode=coast,
                            pressed_fn=lambda t: .08 <= t < .16, velocity_fn=velocity,
                            force_direction_fn=lambda t: [1, 0, 0] if t < .12 else [0, 1, 0])
                        capture = next(p for p in phases if p['previous'] == 'coast' and p['phase'] == 'grace')
                        self.assertAlmostEqual(capture['elapsed_s'], .3 if coast == 'orientation' else .6)
                        self.assertEqual(capture['interaction_direction_xy'], [1., 0.])
                        expected_source = 'potentiometer_force_world' if detector == 'potentiometer' else 'model_force'
                        self.assertEqual(capture['interaction_direction_source'], expected_source)
                        if coast == 'orientation':
                            self.assertGreater(capture['xy_speed_m_s'], .2)
                            self.assertAlmostEqual(capture['stop_speed_value_m_s'], .02)
                            self.assertTrue(any(t == .3 and n == 'send_position_setpoint' for t, n, _ in commands))
                        else:
                            self.assertEqual(capture['stop_speed_metric'], 'xy_norm')

    def test_velocity_detector_locks_onset_axis_for_orientation_coast(self):
        def velocity(t):
            if t < .08:
                return [0, 0, 0]
            if t < .16:
                return [0, -.15, 0]
            return [.06, -.05, 0] if t < .3 else [.06, -.02, 0]

        _, commands, phases, _ = self.run_scenario('vel', velocity_fn=velocity)
        capture = next(p for p in phases if p['previous'] == 'coast' and p['phase'] == 'grace')
        self.assertAlmostEqual(capture['elapsed_s'], .3)
        self.assertEqual(capture['interaction_direction_xy'], [0., -1.])
        self.assertEqual(capture['interaction_direction_source'], 'onset_velocity')
        self.assertAlmostEqual(capture['stop_speed_value_m_s'], .02)
        self.assertTrue(any(t == .3 and n == 'send_position_setpoint' for t, n, _ in commands))

    def test_all_phase_mode_pairs_send_selected_packets_and_preempt_for_every_detector(self):
        def pressed(t):
            return .08 <= t < .16 or .34 <= t < .42

        def speed(t):
            return .12 if pressed(t) else .06 if .16 <= t < .6 else .02

        for detector in ('model', 'potentiometer', 'vel'):
            for contact_mode in ('orientation', 'position'):
                for coast_mode in ('orientation', 'position'):
                    with self.subTest(detector=detector, contact=contact_mode, coast=coast_mode):
                        control, commands, phases, _ = self.run_scenario(
                            detector, command_mode=contact_mode, coast_command_mode=coast_mode,
                            grace_start='release', pressed_fn=pressed, speed_fn=speed)
                        self.assertEqual([p['phase'] for p in phases],
                                         ['ready', 'contact', 'coast', 'contact', 'coast', 'ready'])
                        self.assertEqual(sum(p['coast_preempted'] for p in phases), 1)
                        sent = {round(t, 6): (name, args) for t, name, args in commands}
                        for row in control.log_manager.groups['wrench_observer']:
                            phase = row['phase']
                            selected = coast_mode if phase == 'coast' else contact_mode
                            name, args = sent[round(row['time'] - 1000., 6)]
                            if phase in ('contact', 'coast') and selected == 'orientation':
                                self.assertEqual(row['command_mode'], 'level_zdistance')
                                self.assertEqual((name, args), ('send_zdistance_setpoint', (0., 0., 0., 1.)))
                                self.assertNotIn('position_velocity_retention_requested', row)
                            else:
                                self.assertEqual(name, 'send_position_setpoint')
                                self.assertEqual(args[2:], (1., 0.))
                                if phase in ('contact', 'coast'):
                                    self.assertEqual(row['command_mode'], 'position_follow')
                                    np.testing.assert_allclose(args[:3], row['position_command_m'])
                        for (_, name, _), authority in zip(commands, control.command_authorities):
                            self.assertEqual(authority, name == 'send_zdistance_setpoint')
                        transitions = [c.args[1] for c in control._log_event.call_args_list
                                       if c.args[0] == 'Level Coast Command Mode Changed']
                        if contact_mode != coast_mode:
                            self.assertEqual(sum(r['phase'] == 'coast' for r in transitions), 2)
                            self.assertEqual(sum(r['phase'] == 'contact' for r in transitions), 2)
                        if coast_mode == 'position':
                            for phase in (p for p in phases if p['phase'] == 'coast'):
                                first = next(r for r in control.log_manager.groups['wrench_observer']
                                             if abs(r['time'] - 1000. - phase['elapsed_s']) < 1e-6)
                                self.assertEqual(first['position_velocity_retention_requested'], 1.)
                        if 'position' in (contact_mode, coast_mode):
                            control.cf.param.set_value_raw.assert_not_called()

    def test_mixed_modes_early_release_and_preemption_preserve_position_delay(self):
        for detector in ('potentiometer', 'model'):
            for contact, coast in (('position', 'orientation'), ('orientation', 'position')):
                with self.subTest(detector=detector, contact=contact):
                    control, commands, phases, _ = self.run_scenario(
                        detector, command_mode=contact, coast_command_mode=coast,
                        grace_start='release', ori_delay=.1,
                        pressed_fn=lambda t: .08 <= t < .16 or .40 <= t < .48,
                        speed_fn=lambda t: .12 if .08 <= t < .7 else .02)
                    self.assertTrue(any(p['coast_preempted'] for p in phases))
                    rows = control.log_manager.groups['wrench_observer']
                    self.assertTrue(any(r['phase'] == 'coast' and r['ori_delay_pending'] for r in rows))
                    sent = {round(t, 6): (name, args) for t, name, args in commands}
                    for row in rows:
                        if row['ori_delay_pending']:
                            self.assertEqual(row['command_mode'], 'position_hold')
                            self.assertEqual(sent[round(row['time']-1000., 6)][0], 'send_position_setpoint')
                    active = [r for r in rows if r['command_mode'] != 'position_hold']
                    self.assertTrue(active)
                    self.assertEqual({r['phase'] for r in active}, {'coast'})
                    self.assertEqual({r['command_mode'] for r in active},
                                     {'position_follow' if coast == 'position' else 'level_zdistance'})

    def test_low_speed_release_captures_using_coast_policy_even_without_coast_packet(self):
        for detector in ('potentiometer', 'model'):
            for contact, coast in (('position', 'orientation'), ('orientation', 'position')):
                with self.subTest(detector=detector, contact=contact):
                    control, _, phases, _ = self.run_scenario(
                        detector, command_mode=contact, coast_command_mode=coast,
                        pressed_fn=lambda t: .08 <= t < .16,
                        speed_fn=lambda t: .12 if .08 <= t < .16 else .01)
                    release = next(r for r in control.log_manager.groups['wrench_observer']
                                   if r['release_confirmed'])
                    self.assertEqual(release['phase'], 'grace')
                    capture = next(p for p in phases if p['released'])
                    if coast == 'position':
                        self.assertTrue(release['position_capture_projected'])
                        self.assertGreater(capture['hold_position_m'][0], release['position_m'][0])
                    else:
                        self.assertNotIn('position_capture_projected', release)
                        np.testing.assert_allclose(capture['hold_position_m'], release['position_m'])

    def test_duplicate_state_resends_selected_coast_packet_without_advancing(self):
        for contact, coast in (('position', 'orientation'), ('orientation', 'position')):
            with self.subTest(contact=contact):
                control, commands, _, _ = self.run_scenario(
                    command_mode=contact, coast_command_mode=coast, duplicate_times=(.24,))
                sent = {t: (name, args) for t, name, args in commands}
                self.assertEqual(sent[.24], sent[.23])
                self.assertEqual(sent[.24][0], 'send_position_setpoint' if coast == 'position'
                                 else 'send_zdistance_setpoint')
                self.assertFalse(any(abs(r['time'] - 1000.24) < 1e-6
                                     for r in control.log_manager.groups['wrench_observer']))

    def test_mixed_mode_faults_abort_and_clear_authority(self):
        for contact, coast in (('position', 'orientation'), ('orientation', 'position')):
            for fault in ('state', 'motor', 'sensor', 'battery', 'boundary', 'target', 'pid'):
                with self.subTest(contact=contact, fault=fault):
                    control, _, _, _ = self.run_scenario(
                        command_mode=contact, coast_command_mode=coast, fault=fault)
                    self.assertFalse(control._pid_15state_control_active)

    def test_mixed_modes_preserve_yaw_packet_semantics(self):
        for contact, coast in (('position', 'orientation'), ('orientation', 'position')):
            for follow in (False, True):
                with self.subTest(contact=contact, follow_yaw=follow):
                    control, commands, _, _ = self.run_scenario(
                        command_mode=contact, coast_command_mode=coast,
                        follow_yaw=follow, yaw_rate_damping=not follow,
                        yaw_fn=lambda t: 30*t)
                    for t, name, args in commands:
                        if name == 'send_zdistance_setpoint':
                            self.assertEqual(args, (0., 0., 0., 1.))
                        else:
                            self.assertAlmostEqual(args[3], 30*t if follow else 0.)
                    if not follow:
                        self.assertTrue(control.yaw_requests)
                        control.cf._offboard_yaw_damping_guard.finish.assert_called_once()

    def test_position_mode_only_sends_position_in_contact_and_coast_for_all_detectors(self):
        def speed(t):
            if .08 <= t < .16 or .52 <= t < .60:
                return .12
            if .16 <= t < .30 or .60 <= t < .66:
                return .05  # Below velocity release, above position-capture gate.
            return .02 if t >= .30 else 0.
        for detector in ('potentiometer','model','vel'):
            with self.subTest(detector=detector):
                control, commands, phases, _ = self.run_scenario(
                    detector, command_mode='position', speed_fn=speed)
                self.assertTrue(all(n=='send_position_setpoint' for _,n,_ in commands))
                rows=control.log_manager.groups['wrench_observer']
                moving=[r for r in rows if r['command_mode']=='position_follow']
                self.assertEqual({r['phase'] for r in moving},{'contact','coast'})
                for row in moving:
                    self.assertGreaterEqual(row['position_command_m'][0],row['position_m'][0])
                    self.assertEqual(row['position_command_m'][2],1.)
                self.assertFalse(control._pid_15state_control_active)
                control.cf.param.set_value_raw.assert_not_called()  # No in-flight PID reset/write.

    def test_position_capture_freezes_forward_target_without_changing_grace_semantics(self):
        control, commands, phases, _=self.run_scenario(command_mode='position',grace_start='release')
        capture=next(r for r in phases if r['previous']=='coast' and r['phase']=='grace')
        t=capture['elapsed_s']
        target=capture['hold_position_m']
        self.assertGreater(target[0],t/10)
        self.assertTrue(any(np.allclose(a[:3],target) for ct,_,a in commands if ct>=t))
        self.assertTrue(any(r['phase']=='ready' for r in phases))

    def test_position_delay_yaw_and_safety_use_existing_lifecycle(self):
        control,commands,phases,_=self.run_scenario(command_mode='position',ori_delay=.1,
                                                  follow_yaw=True,yaw_fn=lambda t:30*t)
        rows=control.log_manager.groups['wrench_observer']
        self.assertTrue(any(r['ori_delay_pending'] for r in rows))
        self.assertTrue(all(r['command_mode']=='position_hold' for r in rows if r['ori_delay_pending']))
        for t,_,a in commands:
            self.assertAlmostEqual(a[3],30*t)
        for fault in ('state','motor','sensor','battery','boundary','target'):
            self.run_scenario(command_mode='position',fault=fault)

    def test_delay_sends_only_pos_until_deadline_for_all_detectors(self):
        for detector in ('potentiometer', 'vel', 'model'):
            with self.subTest(detector=detector):
                control, commands, phases, _ = self.run_scenario(detector, ori_delay=.1)
                onsets = [p['elapsed_s'] for p in phases if p['phase'] == 'contact']
                self.assertEqual(len(onsets), 2)
                for onset in onsets:
                    delayed = [(t,n,a) for t,n,a in commands if onset <= t < onset+.1-1e-8]
                    self.assertTrue(delayed)
                    self.assertTrue(all(n == 'send_position_setpoint' for _,n,_ in delayed))
                    first_ori = next(t for t,n,_ in commands
                                     if t >= onset and n == 'send_zdistance_setpoint')
                    self.assertGreaterEqual(first_ori-onset, .1-1e-8)
                    self.assertLessEqual(first_ori-onset, .111)
                modes = [c.args[1] for c in control._log_event.call_args_list
                         if c.args[0] == 'Level Coast Command Mode Changed'
                         and c.args[1]['command_mode'] == 'level_zdistance']
                self.assertEqual(len(modes), 2)
                self.assertTrue(all(m['since_detection_s'] >= .1-1e-8 for m in modes))

    def test_delay_does_not_block_duration_abort_or_early_release(self):
        control, commands, phases, elapsed = self.run_scenario(duration=.23, ori_delay=.5)
        self.assertTrue(any(p['released'] for p in phases))
        self.assertTrue(all(n == 'send_position_setpoint' for _, n, _ in commands))
        self.assertAlmostEqual(elapsed, .23)
        for fault in ('battery', 'state', 'motor', 'sensor', 'boundary'):
            with self.subTest(fault=fault):
                control, _, _, _ = self.run_scenario(fault=fault, ori_delay=.5)
                self.assertFalse(control._pid_15state_control_active)

    def test_potentiometer_two_contacts_and_ignored_contact_during_coast_and_grace(self):
        control, commands, phases, elapsed = self.run_scenario()
        self.assertEqual([p['phase'] for p in phases].count('contact'), 2)
        first_grace = next(p for p in phases if p['phase'] == 'grace')
        self.assertGreaterEqual(first_grace['elapsed_s'], .3)
        self.assertEqual([p['phase'] for p in phases][:6],
                         ['ready', 'contact', 'coast', 'grace', 'prepare', 'ready'])
        for t, name, args in commands:
            if .1 <= t < .3 or .55 <= t < .66:
                self.assertEqual(name, 'send_zdistance_setpoint')
                self.assertEqual(args, (0., 0., 0., 1.))
            if .3 <= t < .4:
                self.assertEqual(name, 'send_position_setpoint')
                self.assertAlmostEqual(args[0], .03)
        self.assertAlmostEqual(elapsed, .85)
        self.assertEqual([c.args for c in control.cf.param.set_value.call_args_list],
                         [('kalmanPRel.controlRp','1'), ('kalmanPRel.controlRp','0')]*2)

    def test_model_and_velocity_selectors_reach_level_and_grace(self):
        for detector in ('model', 'vel'):
            with self.subTest(detector=detector):
                _, commands, phases, _ = self.run_scenario(detector)
                self.assertEqual([p['phase'] for p in phases].count('contact'), 2)
                self.assertIn('grace', [p['phase'] for p in phases])
                self.assertTrue(any(n == 'send_zdistance_setpoint' for _, n, _ in commands))
                self.assertTrue(all(a == (0.,0.,0.,1.) for _, n, a in commands
                                    if n == 'send_zdistance_setpoint'))

    def test_optional_sensing_is_logged_without_changing_model_or_velocity_decisions(self):
        for detector in ('model', 'vel'):
            baseline, commands, phases, _ = self.run_scenario(detector, sensor_present=False)
            self.assertTrue(all('force_sensor_fresh' not in row
                for row in baseline.log_manager.groups['wrench_observer']))
            for fresh in (True, False):
                with self.subTest(detector=detector, fresh=fresh):
                    control, sensed_commands, sensed_phases, _ = self.run_scenario(
                        detector, sensor_fresh=fresh, record_potentiometer=True)
                    self.assertEqual(commands, sensed_commands)
                    self.assertEqual(phases, sensed_phases)
                    rows = control.log_manager.groups['wrench_observer']
                    self.assertTrue(rows)
                    self.assertTrue(all(row['force_sensor_fresh'] == fresh for row in rows))
                    self.assertTrue(all('force_sensor_compression_force_N' in row for row in rows))

    def test_follow_yaw_updates_all_hold_phases_but_never_becomes_a_yaw_rate(self):
        def yaw(t):
            return 170. + 50*t if t < .2 else -180. + 50*(t-.2)

        for follow in (False, True):
            with self.subTest(follow=follow):
                _, commands, phases, _ = self.run_scenario(
                    follow_yaw=follow, yaw_fn=yaw, target_yaw=90.)
                hold_phases = set()
                for t, name, args in commands:
                    phase = next((p['phase'] for p in reversed(phases)
                                  if p['elapsed_s'] <= t), 'prepare')
                    if name == 'send_position_setpoint':
                        hold_phases.add(phase)
                        self.assertAlmostEqual(args[3], yaw(t) if follow else 0.)
                    else:
                        self.assertEqual(args, (0., 0., 0., 1.))
                self.assertEqual(hold_phases, {'prepare', 'ready', 'grace'})

    def test_follow_yaw_waits_for_first_fresh_state(self):
        _, commands, _, _ = self.run_scenario(
            follow_yaw=True, yaw_fn=lambda t: 45., state_delay=.02)
        self.assertGreaterEqual(commands[0][0], .02)
        self.assertEqual(commands[0][2][3], 45.)

    def test_yaw_damping_waits_for_stability_and_confirmation_before_detection(self):
        control, commands, phases, _ = self.run_scenario(
            yaw_rate_damping=True, yaw_confirm_delay=.06,
            speed_fn=lambda t: .12 if t < .1 else 0.)
        self.assertGreaterEqual(control.yaw_requests[0], .12)
        ready = next(p for p in phases if p['phase'] == 'ready')
        self.assertGreaterEqual(ready['elapsed_s'], control.yaw_requests[0] + .06)
        self.assertTrue(all(name == 'send_position_setpoint'
            for t, name, _ in commands if t < ready['elapsed_s']))
        self.assertEqual(sum(c.args[0] == 'Level Coast Yaw Damping Enabled'
            for c in control._log_event.call_args_list), 1)
        self.assertLessEqual(max(control.yaw_requests), ready['elapsed_s'])

    def test_yaw_damping_never_starts_if_not_stable_or_disabled(self):
        for enabled, speed in ((True, .12), (False, 0.)):
            control, _, _, _ = self.run_scenario(yaw_rate_damping=enabled,
                speed_fn=lambda t: speed)
            self.assertEqual(control.yaw_requests, [])
            self.assertEqual(control.yaw_samples, [])

    def test_yaw_deadband_receives_body_z_rate_in_every_phase_after_activation(self):
        rate = lambda t: -20. if .15 <= t < .35 else 5.
        control, commands, phases, _ = self.run_scenario(
            yaw_rate_damping=True, yaw_rate_fn=rate, yaw_confirm_delay=.01, ori_delay=.02)
        ready = next(p['elapsed_s'] for p in phases if p['phase'] == 'ready')
        self.assertTrue(control.yaw_samples)
        for t, rate_rad_s in control.yaw_samples:
            self.assertGreaterEqual(t, ready)
            self.assertAlmostEqual(np.degrees(rate_rad_s), rate(t))
        # Both position and level commands share the same rate gating, including
        # delay, coast, grace, and the repeated preparation phase.
        self.assertEqual([t for t, _ in control.yaw_samples],
                         [t for t, _, _ in commands if t >= ready])
        control.cf._offboard_yaw_damping_guard.finish.assert_called_once_with()

    def test_yaw_switch_failure_exits_to_landing_and_clears_attitude_authority(self):
        control, commands, _, _ = self.run_scenario(yaw_rate_damping=True, fault='yaw_switch')
        self.assertLess(commands[-1][0], .2)
        self.assertFalse(control._pid_15state_control_active)
        control.cf._offboard_yaw_damping_guard.finish.assert_called_once_with()

    def test_yaw_confirmation_failure_never_arms_detection(self):
        control, commands, phases, _ = self.run_scenario(yaw_rate_damping=True, fault='yaw')
        self.assertEqual(phases, [])
        self.assertTrue(all(name == 'send_position_setpoint' for _, name, _ in commands))

    def test_yaw_damping_keeps_standard_packets_and_ignores_changing_heading(self):
        _, commands, _, _ = self.run_scenario(yaw_rate_damping=True,
            yaw_fn=lambda t: 170. - 340*t, target_yaw=90.)
        for _, name, args in commands:
            if name == 'send_position_setpoint':
                self.assertEqual(args[3], 0.)
            else:
                self.assertEqual(name, 'send_zdistance_setpoint')
                self.assertEqual(args, (0., 0., 0., 1.))

    def test_release_grace_preempts_coast_for_all_detectors(self):
        def pressed(t):
            return .08 <= t < .16 or .34 <= t < .42

        def speed(t):
            return .12 if pressed(t) else .06 if .16 <= t < .6 else .02

        for detector in ('model', 'potentiometer', 'vel'):
            with self.subTest(detector=detector):
                control, commands, phases, _ = self.run_scenario(
                    detector, grace_start='release', pressed_fn=pressed, speed_fn=speed)
                self.assertEqual([p['phase'] for p in phases],
                                 ['ready', 'contact', 'coast', 'contact', 'coast', 'ready'])
                preemption = next(p for p in phases if p['coast_preempted'])
                self.assertGreaterEqual(preemption['elapsed_s'], .34)
                self.assertLess(preemption['elapsed_s'], .42)
                for t, name, args in commands:
                    if .10 <= t < .6:
                        self.assertEqual((name, args), ('send_zdistance_setpoint', (0.,0.,0.,1.)))
                    if t >= .6:
                        self.assertEqual(name, 'send_position_setpoint')
                        self.assertAlmostEqual(args[0], .06)
                self.assertFalse(control._pid_15state_control_active)

    def test_release_grace_discards_early_model_evidence_and_preserves_safety(self):
        _, _, phases, _ = self.run_scenario('model', grace_start='release',
            pressed_fn=lambda t: .08 <= t < .16 or .20 <= t < .23,
            speed_fn=lambda t: .12 if .08 <= t < .6 else .02)
        self.assertEqual(sum(p['phase'] == 'contact' for p in phases), 1)
        for fault in ('battery', 'state', 'motor', 'sensor', 'boundary'):
            with self.subTest(fault=fault):
                self.run_scenario(grace_start='release', fault=fault)

    def test_duration_expires_in_every_phase(self):
        for duration, phase in ((.01,'prepare'), (.06,'ready'), (.13,'contact'),
                                (.23,'coast'), (.34,'grace')):
            with self.subTest(phase=phase):
                control, _, _, elapsed = self.run_scenario(duration=duration)
                completed = [c.args[1] for c in control._log_event.call_args_list
                             if c.args[0] == 'Level Coast Duration Completed']
                self.assertEqual(completed[0]['phase'], phase)
                self.assertAlmostEqual(elapsed, duration)
                self.assertFalse(control._pid_15state_control_active)

    def test_faults_propagate_and_clear_contact_authority(self):
        for fault in ('battery', 'state', 'motor', 'sensor', 'boundary'):
            with self.subTest(fault=fault):
                control, _, _, _ = self.run_scenario(fault=fault)
                self.assertFalse(control._pid_15state_control_active)


if __name__ == '__main__':
    unittest.main()
