"""Online identification must not acquire control or change flight commands."""
from copy import deepcopy
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

from Interaction.braking_response_calibration import PlanarBrakingCalibration
from Interaction.interactions import prediction_calibration_report_path
from Interaction.online_dynamics_model import build_tilt_trials
from Interaction.tests import test_calibration_trial_wait_runtime as wait_fixture

ProtocolFinished = wait_fixture.ProtocolFinished


class FakeSession:
    def __init__(self, commander, reject=False, finish_retries=0):
        self.commander = commander
        self.reject = reject
        self.trials = []
        self.last_commands = []
        self.finish_calls = 0
        self.finish_retries = finish_retries
        self.latest_report = None

    def submit_trial(self, samples):
        self.trials.append(deepcopy(samples))
        self.last_commands.append(self.commander.calls[-1])
        return not self.reject

    def request_finish(self):
        self.finish_calls += 1
        return self.finish_calls > self.finish_retries

    def poll(self):
        return []


class OnlinePredictionRuntimeTests(unittest.TestCase):
    def fixture(self):
        runtime = list(wait_fixture.CalibrationTrialWaitRuntimeTests().make_runtime())
        config = runtime[4]['planar_braking_calibration']
        config.update(level_before_acceleration_s=.12, accelerate_s=.16,
                      level_before_brake_s=.12, brake_s=.16,
                      level_after_brake_s=.20, recovery_s=.20)
        runtime[5] = PlanarBrakingCalibration(config, start_after_s=.01).end_s + .04
        return runtime

    def run_fixture(self, runtime, session=None, save=False):
        controller, _logs, clock, _command_times, config, duration = runtime
        with (
            patch('Interaction.interactions.time.time', lambda: clock[0]),
            patch('Interaction.interactions.identify_xyz_alignment', return_value={}),
            patch('Interaction.interactions.identify_planar_braking_response',
                  side_effect=None if save else ProtocolFinished, return_value={}) as fit,
            patch('Interaction.interactions.save_drone_calibration',
                  return_value=(Path('/tmp/cal.json'), {'impulse_estimator': {}})) as writer,
        ):
            kwargs = dict(duration=duration, nominal_position=[0, 0, 1], config=config,
                          calibration_mode=True, prediction_calibration=session)
            if save:
                controller.interaction_onboard_wrench_admittance(**kwargs)
            else:
                with self.assertRaises(ProtocolFinished):
                    controller.interaction_onboard_wrench_admittance(**kwargs)
        return fit, writer

    def test_identical_commands_and_complete_samples_after_recovery_send(self):
        baseline, online = self.fixture(), self.fixture()
        session = FakeSession(online[0].lo_commander)
        self.run_fixture(baseline)
        fit, _writer = self.run_fixture(online, session)
        self.assertEqual(baseline[0].lo_commander.calls, online[0].lo_commander.calls)
        self.assertEqual([rows[0]['segment_id'] for rows in session.trials], [0, 1])
        self.assertTrue(all(call[0] == 'position' for call in session.last_commands))
        self.assertEqual(session.finish_calls, 1)
        self.assertEqual([s for rows in session.trials for s in rows], fit.call_args.args[0])
        for rows in session.trials:
            self.assertTrue(all('angular_velocity_rad_s' in s and 'state_group_skew_s' in s
                                for s in rows))
            self.assertTrue(all(s['command_started_at'] <= s['timestamp'] for s in rows))
            self.assertEqual(len(build_tilt_trials(rows)), 1)

    def test_rejected_background_submissions_do_not_change_commands(self):
        baseline, online = self.fixture(), self.fixture()
        session = FakeSession(online[0].lo_commander, reject=True, finish_retries=2)
        self.run_fixture(baseline)
        self.run_fixture(online, session)
        self.assertEqual(baseline[0].lo_commander.calls, online[0].lo_commander.calls)
        self.assertEqual(len(session.trials), 2)
        self.assertEqual(session.finish_calls, 3)

    def test_cached_report_passed_to_legacy_save_without_waiting(self):
        online = self.fixture()
        session = FakeSession(online[0].lo_commander)
        session.latest_report = {'status': 'collecting', 'validation_passed': False}
        _, writer = self.run_fixture(online, session, save=True)
        self.assertEqual(writer.call_args.kwargs['online_prediction_report'], session.latest_report)

    def test_no_new_report_does_not_change_legacy_save_signature(self):
        online = self.fixture()
        session = FakeSession(online[0].lo_commander)
        _, writer = self.run_fixture(online, session, save=True)
        self.assertNotIn('online_prediction_report', writer.call_args.kwargs)

    def entry_fixture(self, enabled=True):
        controller, logs, _clock, _times, config, _duration = self.fixture()
        config['online_prediction_calibration'] = {'enabled': enabled}
        config['planar_braking_calibration']['accelerate_durations_s'] = [.16, .24, .32]
        controller.mission = {
            'drones': {'test': {'target': [0., 0., 1., 0.]}},
            'Interaction': {'action': 'translation', 'config': {
                'detection_method': 'momentum_impulse', 'duration': 60,
                'wrench_calibration_file': '/tmp/online-test-cal.json',
                'wrench_interaction': config,
            }},
        }
        controller.interaction_onboard_wrench_admittance = Mock()
        return controller

    def test_entry_starts_only_for_enabled_calibration_and_keeps_protocol(self):
        off, on = self.entry_fixture(False), self.entry_fixture(True)
        before = deepcopy(on.mission)
        with patch('Interaction.interactions.OnlinePredictionCalibration') as worker:
            off.run_calibration()
            worker.assert_not_called()
            on.run_calibration()
            worker.assert_called_once()
            worker.return_value.start.assert_called_once()
            worker.return_value.close.assert_called_once()
            init = worker.call_args.kwargs
            self.assertEqual(init['expected_segment_ids'], list(range(6)))
            self.assertFalse(init['metadata']['position_capture_model_identified'])
            kwargs = on.interaction_onboard_wrench_admittance.call_args.kwargs
            self.assertIs(kwargs['prediction_calibration'], worker.return_value)
        baseline = off.interaction_onboard_wrench_admittance.call_args.kwargs
        self.assertEqual(kwargs['duration'], baseline['duration'])
        self.assertEqual(kwargs['nominal_position'], baseline['nominal_position'])
        self.assertEqual(kwargs['config']['planar_braking_calibration'],
                         baseline['config']['planar_braking_calibration'])
        self.assertNotIn('position_capture_calibration', kwargs['config'])
        self.assertEqual(on.mission, before)

    def test_normal_interaction_and_repeat_never_spawn_worker(self):
        for method in ('_run_translation', 'run_braking_test'):
            with self.subTest(method=method):
                controller = self.entry_fixture(True)
                with patch('Interaction.interactions.OnlinePredictionCalibration') as worker:
                    getattr(controller, method)()
                    worker.assert_not_called()
                kwargs = controller.interaction_onboard_wrench_admittance.call_args.kwargs
                self.assertNotIn('prediction_calibration', kwargs)

    def test_worker_closed_on_runtime_error(self):
        controller = self.entry_fixture(True)
        controller.interaction_onboard_wrench_admittance.side_effect = RuntimeError('fake flight error')
        with patch('Interaction.interactions.OnlinePredictionCalibration') as worker:
            with self.assertRaisesRegex(RuntimeError, 'fake flight error'):
                controller.run_calibration()
            worker.return_value.close.assert_called_once()

    def test_worker_closed_even_if_notify_stop_raises(self):
        controller = self.entry_fixture(True)
        controller.lo_commander.send_notify_setpoint_stop = Mock(side_effect=RuntimeError('notify error'))
        with patch('Interaction.interactions.OnlinePredictionCalibration') as worker:
            with self.assertRaisesRegex(RuntimeError, 'notify error'):
                controller.run_calibration()
            worker.return_value.close.assert_called_once()

    def test_failed_recovery_send_cannot_submit_trial(self):
        online = self.fixture()
        commander = online[0].lo_commander
        original = commander.send_position_setpoint

        def fail_recovery(*args, **kwargs):
            if commander.calls and commander.calls[-1][0] == 'zdistance':
                raise RuntimeError('recovery send failed')
            return original(*args, **kwargs)

        commander.send_position_setpoint = fail_recovery
        session = FakeSession(commander)
        with self.assertRaisesRegex(RuntimeError, 'recovery send failed'):
            self.run_fixture(online, session)
        self.assertEqual(session.trials, [])

    def test_readiness_timeout_cannot_submit_unfinished_trial(self):
        def behavior(state, now, logs, controller):
            if wait_fixture.CalibrationTrialWaitRuntimeTests.pending_wait(logs):
                state['velocity'][0] = .059

        online = wait_fixture.CalibrationTrialWaitRuntimeTests().make_runtime(wait_behavior=behavior)
        session = FakeSession(online[0].lo_commander)
        with self.assertRaises(TimeoutError):
            self.run_fixture(online, session)
        self.assertEqual(session.trials, [])

    def test_worker_failure_event_does_not_change_commands(self):
        baseline, online = self.fixture(), self.fixture()
        session = FakeSession(online[0].lo_commander, reject=True)
        session.poll = Mock(side_effect=lambda: [{'event': 'worker_failed', 'data': {'error': 'synthetic'}}]
                            if session.poll.call_count == 1 else [])
        self.run_fixture(baseline)
        self.run_fixture(online, session)
        self.assertEqual(baseline[0].lo_commander.calls, online[0].lo_commander.calls)

    def test_paths_unique_and_never_target_active_calibration(self):
        first = prediction_calibration_report_path('/tmp/calibration.json', '../lb11')
        second = prediction_calibration_report_path('/tmp/calibration.json', '../lb11')
        self.assertNotEqual(first, second)
        self.assertEqual(first.parent.parent, Path('/tmp').resolve() / 'prediction_calibration_runs')
        self.assertEqual(first.name, 'report.json')


if __name__ == '__main__':
    unittest.main()
