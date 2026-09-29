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
        'behavior': 'level_coast', 'detection_method': 'momentum_impulse',
        'duration': .85, 'grace_time': .10,
        'level_coast': {'detector': detector, 'stop_speed_m_s': .03,
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
                'onset_evidence_s': .001, 'release_time_s': .01}},
        },
    }


class LevelCoastStateTests(unittest.TestCase):
    def test_release_is_required_and_full_xy_norm_controls_grace(self):
        cycle = LevelCoastCycle([0, 0, 1], .03, .5)
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
            ('detector', lambda c: c['level_coast'].update(detector='unknown')),
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
    def run_scenario(self, detector='potentiometer', *, duration=.85, fault=None):
        config = configuration(detector)
        config['duration'] = duration
        original = copy.deepcopy(config)
        clock = {'t': 0.}
        control = InteractionsControl.__new__(InteractionsControl)
        control.drone_id = 'lb11'
        control.ctrl_rate = 100
        control.mission = {'Interaction': {'config': config},
                           'drones': {'lb11': {'target': [0, 0, 1]}}}
        control.bounds = dict(x_min=-1, x_max=1, y_min=-1, y_max=1, z_min=.3, z_max=2)
        control.lo_commander = FakeCommander()
        control.hl_commander = FakeCommander()
        control.cf = SimpleNamespace(param=SimpleNamespace(set_value=Mock(), set_value_raw=Mock()))
        control.pid_attitude_source = 'post-release-15state'
        control._pid_15state_control_active = False
        control.force_sensor = object()
        control._unsubscribe_contact_attitude_shadow = Mock()
        control.log_manager = FakeOnboardLogManager(1000.)
        control._log_event = Mock()
        control._handoff_translation_hold = Mock()
        commands = []
        for name in ('send_position_setpoint', 'send_zdistance_setpoint'):
            original_sender = getattr(control.lo_commander, name)
            def send(*args, name=name, original_sender=original_sender):
                commands.append((clock['t'], name, args))
                original_sender(*args)
            setattr(control.lo_commander, name, send)

        def sleep(seconds):
            if fault == 'battery' and clock['t'] >= .2:
                raise LowBatteryException('test battery abort')
            clock['t'] = round(clock['t'] + seconds, 6)
            self.assertLess(clock['t'], 2., 'duration did not terminate the loop')

        def pressed():
            t = clock['t']
            return .08 <= t < .16 or .21 <= t < .26 or .36 <= t < .38 or .52 <= t < .60

        def state():
            t = clock['t']
            speed = .12 if .08 <= t < .3 or .52 <= t < .66 else .02 if t >= .3 else 0.
            return dict(time=1000.+t-(.2 if fault == 'state' and t >= .2 else 0),
                position=np.array([2. if fault == 'boundary' and t >= .2 else t/10, 0., 1.]),
                velocity=np.array([speed, 0., 0.]), attitude_rpy=np.zeros(3), angular_velocity=np.zeros(3),
                position_skew_s=0., angular_rate_skew_s=0., yaw_control_skew_s=None,
                yaw_control_command=None, motor_skew_s=0., motor_state={
                    'time':1000.+t-(.2 if fault == 'motor' and t >= .2 else 0),
                    'motor.m1':30000, 'motor.m2':30000, 'motor.m3':30000, 'motor.m4':30000, 'pm.vbat':8.})

        def sensor(*_):
            t = clock['t']
            return dict(force_sensor_fresh=not (fault == 'sensor' and t >= .2),
                        force_sensor_sample_monotonic_time=t, force_sensor_sample_time=1000.+t,
                        force_sensor_compression_force_N=.3 if pressed() else 0.)

        control._get_synchronized_onboard_wrench_state = state
        control._safe_sleep = sleep
        control._force_sensor_log_fields = sensor
        real_update = OnboardMomentumWrenchPipeline.update

        def model_update(pipeline, **kwargs):
            # Use the real model contact detector with controlled wrench input;
            # model fitting itself is covered by the pipeline's existing tests.
            enabled = pipeline.detector.translation.enabled
            if not hasattr(pipeline, '_level_test_detector'):
                pipeline._level_test_detector = copy.deepcopy(pipeline.detector)
            pipeline.detector.translation.enabled = False
            result = real_update(pipeline, **kwargs)
            pipeline.detector.translation.enabled = enabled
            estimate = replace(result.estimate, external_force=np.array([
                .3 if pressed() else 0., 0., 0.]), force_covariance=np.eye(3)*.0001,
                measurement_rejected=False)
            pipeline._level_test_detector.translation.enabled = enabled
            return replace(result, contacts=pipeline._level_test_detector.update(estimate))

        expected = {'battery':LowBatteryException, 'state':StaleLocalizationError,
                    'boundary':BoundaryExceededError, 'motor':RuntimeError, 'sensor':RuntimeError}
        with patch('Interaction.level_coast.time.time', side_effect=lambda:1000.+clock['t']), \
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
