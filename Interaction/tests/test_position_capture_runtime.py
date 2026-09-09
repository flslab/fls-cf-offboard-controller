"""Hardware-free checks of calibration command ownership and persistence."""

from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from Interaction.interactions import InteractionsControl, StaleLocalizationError
from Interaction.tests.test_wrench_interactions_integration import (
    FakeCommander, FakeOnboardLogManager,
)


class ShortCapturePlan:
    """Exercise the runtime wiring, not the separately tested flight protocol."""

    enabled = True
    end_s = duration_s = 0.14
    max_displacement_m = 1.0
    max_xy_speed_m_s = 0.8
    trial_start_max_position_error_m = 0.08

    def __init__(self, *_args, **_kwargs):
        self.samples = []

    def attitude_phase_due(self, _elapsed):
        return False

    def command(self, elapsed, *_args):
        phase = 'capture' if elapsed < 0.10 else 'recovery'
        return SimpleNamespace(
            active=elapsed < self.end_s, attitude_control=False,
            phase=phase if elapsed < self.end_s else 'complete',
            segment_id=0, direction_xy=np.array([1.0, 0.0]),
            roll_deg=0.0, pitch_deg=0.0,
            position_target=(np.array([0.2, 0.0, 1.0])
                             if phase == 'capture' else None),
        )

    def summarize(self, samples):
        self.samples = list(samples)
        return {'usable': False, 'quality_failures': ['synthetic rejection']}


class PositionCaptureRuntimeTests(unittest.TestCase):
    def test_speed_does_not_abort_but_displacement_remains_guarded(self):
        for phase, displacement, expected in [
            ('recovery', 0.1, 'previous calibration file'),
            ('recovery', 1.1, 'safety limit'),
            ('capture', 0.1, 'previous calibration file'),
            ('capture', 1.1, 'safety limit'),
        ]:
            with self.subTest(phase=phase, displacement=displacement):
                clock = [1000.0]
                logs = FakeOnboardLogManager(clock[0])
                controller = InteractionsControl.__new__(InteractionsControl)
                controller.drone_id = 'test'
                controller.log_manager = logs
                controller.ctrl_rate = 50
                controller.bounds = None
                controller.lo_commander = FakeCommander()
                controller.hl_commander = FakeCommander()

                def sleep(_seconds):
                    clock[0] += 0.02
                    logs.advance(0.02)

                controller._safe_sleep = sleep
                original_state = controller._get_synchronized_onboard_wrench_state
                inject = [False]

                def state():
                    result = original_state()
                    if inject[0]:
                        result['velocity'][0] = .717
                        result['position'][0] = displacement
                    return result

                controller._get_synchronized_onboard_wrench_state = state
                plan = ShortCapturePlan()
                plan.max_xy_speed_m_s = .70
                original_command = plan.command

                def command(elapsed, *args):
                    inject[0] = True
                    result = original_command(elapsed, *args)
                    result.phase = phase
                    result.position_target = (
                        np.array([.2, 0, 1]) if phase == 'capture' else None
                    )
                    return result

                plan.command = command
                with (
                    patch('Interaction.interactions.time.time', lambda: clock[0]),
                    patch('Interaction.interactions.PositionCaptureCalibration', return_value=plan),
                    patch('Interaction.interactions.identify_xyz_alignment', return_value={}),
                    patch('Interaction.interactions.save_drone_calibration') as save,
                ):
                    with self.assertRaisesRegex((RuntimeError, ValueError), expected):
                        controller.interaction_onboard_wrench_admittance(
                            duration=.18, nominal_position=[0, 0, 1],
                            config={
                                'state_source': 'onboard', 'shadow_mode': True,
                                'observer_settle_s': 0, 'bias_calibration_s': .01,
                                'minimum_bias_samples': 1,
                                'motor_model': {'hover_pwm': 30000, 'hover_voltage': 8},
                                'calibration_excitation': {'enabled': False},
                                'planar_braking_calibration': {'enabled': False},
                                'position_capture_calibration': {'enabled': True},
                            }, calibration_mode=True,
                        )
                    save.assert_not_called()

    def test_due_attitude_phase_levels_and_aborts_instead_of_skew_retry(self):
        clock = [1000.0]
        logs = FakeOnboardLogManager(clock[0])
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.drone_id = 'test'
        controller.log_manager = logs
        controller.ctrl_rate = 50
        controller.bounds = None
        controller.lo_commander = FakeCommander()
        controller.hl_commander = FakeCommander()

        def sleep(_seconds):
            clock[0] += 0.02
            logs.advance(0.02)

        controller._safe_sleep = sleep
        original_state = controller._get_synchronized_onboard_wrench_state

        def state():
            result = original_state()
            calls = controller.lo_commander.calls
            if (calls and calls[-1][0] == 'position'
                    and calls[-1][1][0] == 0.2):
                result['position_skew_s'] = 0.08
            return result

        controller._get_synchronized_onboard_wrench_state = state
        plan = ShortCapturePlan()
        # Simulate an attitude phase that is due on the next control cycle,
        # despite the most recent actually-sent command still being position.
        plan.attitude_phase_due = lambda _elapsed: True
        with (
            patch('Interaction.interactions.time.time', lambda: clock[0]),
            patch('Interaction.interactions.PositionCaptureCalibration',
                  return_value=plan),
            patch('Interaction.interactions.save_drone_calibration') as save,
        ):
            with self.assertRaisesRegex(StaleLocalizationError,
                                        'during planar attitude calibration'):
                controller.interaction_onboard_wrench_admittance(
                    duration=0.18,
                    nominal_position=[0.0, 0.0, 1.0],
                    config={
                        'state_source': 'onboard', 'shadow_mode': True,
                        'observer_settle_s': 0.0, 'bias_calibration_s': 0.01,
                        'minimum_bias_samples': 1,
                        'motor_model': {'hover_pwm': 30000, 'hover_voltage': 8},
                        'calibration_excitation': {'enabled': False},
                        'planar_braking_calibration': {'enabled': False},
                        'position_capture_calibration': {'enabled': True},
                    },
                    calibration_mode=True,
                )
            save.assert_not_called()
        self.assertEqual(controller.lo_commander.calls[-1],
                         ('zdistance', (0.0, 0.0, 0.0, 1.0), {}))

    def test_capture_target_survives_skew_retry_and_failed_fit_is_not_saved(self):
        clock = [1000.0]
        logs = FakeOnboardLogManager(clock[0])
        controller = InteractionsControl.__new__(InteractionsControl)
        controller.drone_id = 'test'
        controller.log_manager = logs
        controller.ctrl_rate = 50
        controller.bounds = None
        controller.lo_commander = FakeCommander()
        controller.hl_commander = FakeCommander()

        def sleep(_seconds):
            clock[0] += 0.02
            logs.advance(0.02)

        controller._safe_sleep = sleep
        original_state = controller._get_synchronized_onboard_wrench_state
        injected = []

        def state():
            result = original_state()
            calls = controller.lo_commander.calls
            if (not injected and calls and calls[-1][0] == 'position'
                    and calls[-1][1][0] == 0.2):
                result['position_skew_s'] = 0.08
                injected.append(len(calls))
            return result

        controller._get_synchronized_onboard_wrench_state = state
        plan = ShortCapturePlan()
        with (
            patch('Interaction.interactions.time.time', lambda: clock[0]),
            patch('Interaction.interactions.PositionCaptureCalibration',
                  return_value=plan),
            patch('Interaction.interactions.identify_xyz_alignment',
                  return_value={}),
            patch('Interaction.interactions.save_drone_calibration') as save,
        ):
            with self.assertRaisesRegex(ValueError, 'previous calibration file'):
                controller.interaction_onboard_wrench_admittance(
                    duration=0.18,
                    nominal_position=[0.0, 0.0, 1.0],
                    config={
                        'state_source': 'onboard', 'shadow_mode': True,
                        'observer_settle_s': 0.0, 'bias_calibration_s': 0.01,
                        'minimum_bias_samples': 1,
                        'motor_model': {'hover_pwm': 30000, 'hover_voltage': 8},
                        'calibration_excitation': {'enabled': False},
                        'planar_braking_calibration': {'enabled': False},
                        'position_capture_calibration': {'enabled': True},
                    },
                    calibration_mode=True,
                )
            save.assert_not_called()

        self.assertEqual(len(injected), 1)
        retry = controller.lo_commander.calls[injected[0]]
        self.assertEqual(retry[0], 'position')
        self.assertEqual(retry[1][:3], (0.2, 0.0, 1.0))
        self.assertGreaterEqual(len(plan.samples), 2)
        starts = {sample['command_started_at'] for sample in plan.samples}
        self.assertEqual(len(starts), 1)
        self.assertTrue(all(sample['position_target'] == [0.2, 0.0, 1.0]
                            for sample in plan.samples))
        self.assertTrue(all(sample['timestamp'] >= sample['command_started_at']
                            for sample in plan.samples))
        names = [name for group, name, _ in logs.records if group == 'events']
        self.assertIn('Position Capture Calibration Evaluated', names)
        self.assertIn('Position Capture Calibration Rejected', names)
        self.assertNotIn('Wrench Model Calibration Saved', names)


if __name__ == '__main__':
    unittest.main()
