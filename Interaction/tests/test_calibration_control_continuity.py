"""Real fake-clock runtime checks for ownership and adaptive command delivery."""
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

from Interaction.online_dynamics_model import build_tilt_trials
from Interaction.tests import test_calibration_trial_wait_runtime as wait_fixture
from Interaction.tests.test_model_based_braking import model
from Interaction.tests.test_online_prediction_runtime import FakeSession


class CalibrationControlContinuityTests(unittest.TestCase):
    def fixture(self, *, adaptive=False):
        def behavior(state, _now, logs, _controller):
            phases = self.events(logs, 'Planar Braking Calibration Phase')
            if phases and phases[-1]['phase'] == 'brake':
                # Small positive velocity makes the real predictor prefer
                # early leveling over the original large remaining pulse.
                state['velocity'][1] = .02*phases[-1]['direction_xy'][1]

        runtime = list(wait_fixture.CalibrationTrialWaitRuntimeTests().make_runtime(wait_behavior=behavior))
        controller, logs, clock, _times, config, _duration = runtime
        config['online_prediction_calibration'] = {'enabled': True}
        config['calibration_excitation'].update(translation_profile='sine',
                                               translation_amplitude_m=[0., 0., 0.],
                                               yaw_amplitude_deg=0.)
        config['adaptive_braking_calibration'] = {
            'enabled': adaptive, 'target_distance_m': .30,
            'model_based_braking': {'max_compute_s': 1.},
        }
        config['planar_braking_calibration'].update(
            tilt_levels_deg=[20], repetitions_per_tilt=2,
            level_before_acceleration_s=.12, accelerate_s=.16,
            level_before_brake_s=.12, brake_s=.16,
            level_after_brake_s=.20, recovery_s=.20)
        controller.mission = {
            'drones': {'test': {'target': [0., 0., 1., 0.]}},
            'Interaction': {'action': 'translation', 'config': {
                'detection_method': 'momentum_impulse', 'duration': 60,
                'wrench_calibration_file': '/tmp/continuity-test-cal.json',
                'wrench_interaction': config,
            }},
        }
        trace = []
        for commander, method, name in (
                (controller.lo_commander, 'send_position_setpoint', 'LL_position'),
                (controller.lo_commander, 'send_zdistance_setpoint', 'LL_attitude'),
                (controller.lo_commander, 'send_notify_setpoint_stop', 'notify'),
                (controller.hl_commander, 'go_to', 'HLC_go_to')):
            original = getattr(commander, method)
            def traced(*args, _original=original, _name=name, **kwargs):
                trace.append((_name, clock[0], args))
                return _original(*args, **kwargs)
            setattr(commander, method, traced)
        cf = controller.hl_commander._cf
        ack = cf.acknowledge_go_to
        def acknowledge():
            if cf.callbacks:
                trace.append(('firmware_ACK', clock[0], ()))
            return ack()
        cf.acknowledge_go_to = acknowledge
        session = FakeSession(controller.lo_commander)
        session.start = Mock()
        session.close = Mock()
        submit = session.submit_trial
        def submit_and_publish(samples):
            result = submit(samples)
            if len(session.trials) == 2:
                candidate_model = model()
                for row in candidate_model['data_ranges']:
                    row['battery_voltage_V'] = [7.9, 8.1]
                session.latest_report = {'candidate': {
                    'version': 1, 'training_segment_ids': [0, 1],
                    'model': candidate_model,
                }}
            return result
        session.submit_trial = submit_and_publish
        return runtime, session, trace

    @staticmethod
    def events(logs, name):
        return [row for group, event, row in logs.records
                if group == 'events' and event == name]

    def run_fixture(self, fixture, *, xyz_fit=None):
        runtime, session, trace = fixture
        controller, _logs, clock, _times, _config, _duration = runtime
        with (
            patch('Interaction.interactions.time.time', lambda: clock[0]),
            patch('Interaction.interactions.OnlinePredictionCalibration', return_value=session),
            patch('Interaction.interactions.identify_xyz_alignment',
                  side_effect=xyz_fit, return_value={}),
            patch('Interaction.interactions.identify_planar_braking_response',
                  return_value={}) as fit,
            patch('Interaction.interactions.save_drone_calibration',
                  return_value=(Path('/tmp/continuity-test-cal.json'), {'impulse_estimator': {}})) as save,
        ):
            controller.run_calibration()
        session.start.assert_called_once()
        session.close.assert_called_once()
        return fit, save

    def test_hlc_ack_and_priority_release_precede_long_final_fit(self):
        fixture = self.fixture()
        runtime, _session, trace = fixture
        controller, logs, clock, *_ = runtime
        def long_fit(*_args, **_kwargs):
            self.assertTrue(controller._translation_high_level_active)
            self.assertEqual([x[0] for x in trace[-3:]], ['HLC_go_to', 'firmware_ACK', 'notify'])
            captured = self.events(logs, 'Translation High Level Hold Acquired')
            self.assertEqual(len(captured), 1)
            self.assertTrue(captured[0]['handoff']['acknowledged'])
            trace.append(('fit_start', clock[0], ()))
            clock[0] += 3.5  # Simulate >2.5 s CPU-only fit, no keepalive thread.
            trace.append(('fit_end', clock[0], ()))
            return {}
        _fit, save = self.run_fixture(fixture, xyz_fit=long_fit)
        save.assert_called_once()
        start = next(i for i, row in enumerate(trace) if row[0] == 'fit_start')
        self.assertEqual([row[0] for row in trace[start:]], ['fit_start', 'fit_end'])
        self.assertEqual(sum(row[0] == 'notify' for row in trace), 1)
        self.assertGreater(trace[-1][1]-trace[start][1], 2.5)

    def test_fit_failure_outer_finally_does_not_reissue_low_level(self):
        fixture = self.fixture()
        runtime, session, trace = fixture
        controller, _logs, clock, *_ = runtime
        def failed_fit(*_args, **_kwargs):
            self.assertTrue(controller._translation_high_level_active)
            self.assertEqual([x[0] for x in trace[-3:]], ['HLC_go_to', 'firmware_ACK', 'notify'])
            clock[0] += 3.5
            trace.append(('fit_failure', clock[0], ()))
            raise RuntimeError('synthetic final fitting failure')
        with self.assertRaisesRegex(RuntimeError, 'synthetic final fitting failure'):
            self.run_fixture(fixture, xyz_fit=failed_fit)
        session.close.assert_called_once()
        self.assertEqual(trace[-1][0], 'fit_failure')
        self.assertEqual(sum(row[0] == 'notify' for row in trace), 1)
        self.assertTrue(controller._translation_high_level_active)

    def test_real_predictor_early_level_changes_sent_commands_and_raw_trials(self):
        baseline, adaptive = self.fixture(), self.fixture(adaptive=True)
        self.run_fixture(baseline)
        fit, _save = self.run_fixture(adaptive)
        runtime, session, trace = adaptive
        logs = runtime[1]
        decisions = self.events(logs, 'Adaptive Braking Decision')
        self.assertTrue(decisions)
        self.assertTrue(all(row['action'] != 'fallback' for row in decisions), decisions)
        self.assertTrue(any(row['action'] == 'level' for row in decisions))
        frozen = self.events(logs, 'Adaptive Braking Pair Frozen')
        self.assertEqual([row['adaptive'] for row in frozen], [False, True])
        self.assertEqual(frozen[1]['training_segment_ids'], [0, 1])
        self.assertEqual(len(session.trials), 4)
        for segment, rows in enumerate(session.trials):
            baseline_rows = baseline[1].trials[segment]
            self.assertEqual(len(build_tilt_trials(rows)), 1)
            levels = [s for s in rows if s['phase'] == 'level_after_brake']
            baseline_levels = [s for s in baseline_rows if s['phase'] == 'level_after_brake']
            phase_time = levels[0]['command_started_at']
            if segment < 2:
                self.assertEqual(rows, baseline_rows)
            else:
                self.assertLess(phase_time, baseline_levels[0]['command_started_at'])
                self.assertGreaterEqual(len([s for s in rows if s['phase'] == 'brake']), 2)
                sent = [r for r in trace if r[0] == 'LL_attitude' and abs(r[1]-phase_time) < 1e-8]
                self.assertEqual(len(sent), 1)
                self.assertEqual(sent[0][2][:2], (0., 0.))
                self.assertTrue(all(s['command_acceleration_xy'] == [0., 0.] for s in levels))
        self.assertEqual(fit.call_args.args[0], [s for rows in session.trials for s in rows])


if __name__ == '__main__':
    unittest.main()
