"""Fake-clock flight-loop checks for bounded position-held trial readiness."""

from contextlib import ExitStack
import unittest
from unittest.mock import patch

from Interaction.braking_response_calibration import PlanarBrakingCalibration
from Interaction.position_capture_calibration import PositionCaptureCalibration
from Interaction.interactions import InteractionsControl, StaleLocalizationError
from Interaction.tests.test_wrench_interactions_integration import (
    FakeCommander, FakeOnboardLogManager,
)


class ProtocolFinished(Exception):
    pass


class CalibrationTrialWaitRuntimeTests(unittest.TestCase):
    def make_runtime(self, *, wait_behavior=None):
        clock = [1000.0]
        logs = FakeOnboardLogManager(clock[0])
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.drone_id = 'test'
        controller.log_manager = logs
        controller.ctrl_rate = 50
        controller.bounds = None
        controller.lo_commander = FakeCommander()
        controller.hl_commander = FakeCommander()
        command_times = []

        def sleep(_seconds):
            command_times.extend(
                [clock[0]] * (len(controller.lo_commander.calls) - len(command_times))
            )
            clock[0] += 0.02
            logs.advance(0.02)

        controller._safe_sleep = sleep
        get_state = controller._get_synchronized_onboard_wrench_state

        def state():
            result = get_state()
            if wait_behavior is not None:
                wait_behavior(result, clock[0], logs, controller)
            return result

        controller._get_synchronized_onboard_wrench_state = state
        config = {
            'state_source': 'onboard', 'shadow_mode': True,
            'observer_settle_s': 0, 'bias_calibration_s': 0.01,
            'minimum_bias_samples': 1,
            'motor_model': {'hover_pwm': 30000, 'hover_voltage': 8},
            'calibration_excitation': {
                'enabled': False, 'start_delay_s': 0, 'duration_s': 0.01,
            },
            'planar_braking_calibration': {
                'enabled': True, 'start_delay_s': 0.01,
                'tilt_levels_deg': [8], 'repetitions_per_tilt': 1,
                'level_before_acceleration_s': 0.04,
                'accelerate_s': 0.04, 'level_before_brake_s': 0.04,
                'brake_s': 0.04, 'level_after_brake_s': 0.04,
                'recovery_s': 0.08, 'trial_start_dwell_s': 0.30,
                'trial_start_timeout_s': 5.0,
            },
            'position_capture_calibration': {'enabled': False},
            'safety': {'calibration_max_protocol_clock_lag_s': 0.10},
        }
        plan = PlanarBrakingCalibration(
            config['planar_braking_calibration'], start_after_s=0.01,
        )
        return controller, logs, clock, command_times, config, plan.end_s + 0.04

    @staticmethod
    def events(logs, name):
        return [entry for group, event, entry in logs.records
                if group == 'events' and event == name]

    @classmethod
    def pending_wait(cls, logs):
        starts = cls.events(logs, 'Planar Braking Calibration Trial Wait Started')
        ready = cls.events(logs, 'Planar Braking Calibration Trial Ready')
        return starts[-1] if len(starts) > len(ready) else None

    def run_planar(self, runtime, exception=ProtocolFinished, pattern=None):
        controller, logs, clock, command_times, config, duration = runtime
        with ExitStack() as stack:
            stack.enter_context(patch('Interaction.interactions.time.time',
                                      lambda: clock[0]))
            stack.enter_context(patch('Interaction.interactions.identify_xyz_alignment',
                                      return_value={}))
            fit = stack.enter_context(patch(
                'Interaction.interactions.identify_planar_braking_response',
                side_effect=ProtocolFinished,
            ))
            save = stack.enter_context(patch('Interaction.interactions.save_drone_calibration'))
            with self.assertRaisesRegex(exception, pattern or ''):
                controller.interaction_onboard_wrench_admittance(
                    duration=duration, nominal_position=[0, 0, 1],
                    config=config, calibration_mode=True,
                )
            save.assert_not_called()
        return fit

    def test_waits_for_stable_state_then_runs_every_phase_without_fit_hold_rows(self):
        def behavior(state, now, logs, _controller):
            pending = self.pending_wait(logs)
            if pending and now - pending['time'] < 0.2:
                state['velocity'][0] = 0.059

        runtime = self.make_runtime(wait_behavior=behavior)
        fit = self.run_planar(runtime)
        controller, logs, _clock, command_times, _config, _duration = runtime
        starts = self.events(logs, 'Planar Braking Calibration Trial Wait Started')
        ready = self.events(logs, 'Planar Braking Calibration Trial Ready')
        self.assertEqual([entry['segment_id'] for entry in starts], [0, 1])
        self.assertEqual(len(ready), 2)
        for start, end in zip(starts, ready):
            self.assertGreaterEqual(end['time'] - start['time'], 0.49)
            held_calls = [call for call, now in zip(controller.lo_commander.calls, command_times)
                          if start['time'] <= now < end['time'] - 1e-6]
            self.assertTrue(held_calls)
            self.assertTrue(all(call[0] == 'position' for call in held_calls))
            self.assertTrue(all(call[1][:3] == (0.0, 0.0, 1.0) for call in held_calls))
        phases = self.events(logs, 'Planar Braking Calibration Phase')
        expected = ['level_before_acceleration', 'accelerate', 'level_before_brake',
                    'brake', 'level_after_brake', 'recovery']
        for segment in [0, 1]:
            self.assertEqual([entry['phase'] for entry in phases
                              if entry['segment_id'] == segment], expected)
        samples = fit.call_args.args[0]
        self.assertTrue(samples)
        for sample in samples:
            self.assertTrue(all(not (start['time'] <= sample['timestamp'] < end['time'])
                                for start, end in zip(starts, ready)))

    def test_never_settled_times_out_on_wall_clock_without_any_attitude_command(self):
        def behavior(state, _now, logs, _controller):
            if self.pending_wait(logs):
                state['velocity'][0] = 0.059

        runtime = self.make_runtime(wait_behavior=behavior)
        fit = self.run_planar(runtime, TimeoutError, 'did not settle within 5.00s')
        controller, logs, _clock, _times, _config, _duration = runtime
        fit.assert_not_called()
        self.assertTrue(all(call[0] == 'position' for call in controller.lo_commander.calls))
        timeout = self.events(logs, 'Planar Braking Calibration Trial Wait Timeout')
        self.assertEqual(len(timeout), 1)
        self.assertAlmostEqual(timeout[0]['wait_elapsed_s'], 5.0, places=5)

    def test_duplicate_and_stale_frames_do_not_credit_readiness_dwell(self):
        duplicate_time = []

        def behavior(state, now, logs, _controller):
            pending = self.pending_wait(logs)
            if (pending and pending['segment_id'] == 0
                    and 0.10 <= now - pending['time'] < 0.36):
                if not duplicate_time:
                    duplicate_time.append(state['time'])
                state['time'] = duplicate_time[0]

        runtime = self.make_runtime(wait_behavior=behavior)
        self.run_planar(runtime)
        ready = self.events(runtime[1], 'Planar Braking Calibration Trial Ready')
        self.assertGreaterEqual(ready[0]['wait_elapsed_s'], 0.64)

    def test_normal_every_other_frame_duplicate_does_not_starve_readiness(self):
        def behavior(state, now, logs, _controller):
            # 50Hz telemetry consumed by a 100Hz control loop: a repeated
            # timestamp is ordinary, provided fresh frames continue arriving.
            if self.pending_wait(logs):
                state['time'] = 1000.0 + int((now - 1000.0 + 1e-8) / 0.02) * 0.02

        runtime = self.make_runtime(wait_behavior=behavior)
        controller, logs, clock, command_times, _config, _duration = runtime
        controller.ctrl_rate = 100

        def sleep(_seconds):
            command_times.extend(
                [clock[0]] * (len(controller.lo_commander.calls) - len(command_times))
            )
            clock[0] += 0.01
            logs.advance(0.01)

        controller._safe_sleep = sleep
        self.run_planar(runtime)
        ready = self.events(logs, 'Planar Braking Calibration Trial Ready')
        self.assertEqual(len(ready), 2)
        self.assertTrue(all(0.29 <= event['wait_elapsed_s'] <= 0.36
                            for event in ready))

    def test_stale_during_actual_attitude_still_levels_and_aborts(self):
        def behavior(state, _now, _logs, controller):
            if controller.lo_commander.calls and controller.lo_commander.calls[-1][0] == 'zdistance':
                state['time'] -= 0.20

        runtime = self.make_runtime(wait_behavior=behavior)
        self.run_planar(runtime, StaleLocalizationError, 'during planar attitude calibration')
        self.assertEqual(runtime[0].lo_commander.calls[-1],
                         ('zdistance', (0.0, 0.0, 0.0, 1.0), {}))

    def test_old_capture_enabled_setting_cannot_run_retired_experiment(self):
        runtime = self.make_runtime()
        controller, logs, clock, _times, config, _duration = runtime
        config['planar_braking_calibration']['enabled'] = False
        config['position_capture_calibration'] = {'enabled': True}
        with (
            patch('Interaction.interactions.time.time', lambda: clock[0]),
            patch('Interaction.interactions.identify_xyz_alignment', return_value={}),
            patch('Interaction.interactions.save_drone_calibration',
                  return_value=('/tmp/test-calibration.json', {'impulse_estimator': {}})) as save,
        ):
            controller.interaction_onboard_wrench_admittance(
                duration=0.3, nominal_position=[0, 0, 1],
                config=config, calibration_mode=True,
            )
            save.assert_called_once()
        self.assertEqual(self.events(logs, 'Position Capture Calibration Phase'), [])

    def test_nonuniform_pulses_wait_at_each_exact_trial_boundary(self):
        runtime = list(self.make_runtime())
        config = runtime[4]['planar_braking_calibration']
        config.update(tilt_levels_deg=[20], accelerate_durations_s=[.04, .06, .08])
        plan = PlanarBrakingCalibration(config, start_after_s=.01)
        runtime[5] = plan.end_s + .04
        self.run_planar(runtime)
        starts = self.events(runtime[1], 'Planar Braking Calibration Trial Wait Started')
        ready = self.events(runtime[1], 'Planar Braking Calibration Trial Ready')
        self.assertEqual(len(starts), 6)
        self.assertEqual(len(ready), 6)
        for start, expected in zip(starts, plan.trial_start_s):
            self.assertAlmostEqual(start['protocol_elapsed_s'], expected)


if __name__ == '__main__':
    unittest.main()
