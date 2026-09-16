"""Offline checks for the opt-in bounded return between planar pulses."""

import unittest

import numpy as np

from Interaction.calibration_trial_readiness import CalibrationTrialReadinessGate
from Interaction.planar_calibration_recovery import BoundedPlanarCalibrationRecovery


class BoundedPlanarCalibrationRecoveryTests(unittest.TestCase):
    def config(self):
        return {
            'recovery_s': 5.0,
            'recovery_target_speed_m_s': .35,
            'recovery_max_target_lead_m': .12,
            'recovery_settle_speed_m_s': .08,
            'recovery_settle_tilt_deg': 5.0,
            'recovery_stable_dwell_s': .20,
            'recovery_max_tilt_deg': 25.0,
            'recovery_position_tolerance_m': .08,
        }

    def test_initial_target_is_measured_position_and_never_leads_far(self):
        recovery = BoundedPlanarCalibrationRecovery([0, 0, 1], self.config())
        position = np.array([-.44, -.93, 1.0])
        target, phase = recovery.update(5, position, [-.28, -.25], 1.6, 0.0)
        self.assertEqual(phase, 'settle')
        np.testing.assert_allclose(target, position)
        for index in range(1, 12):
            t = index * .05
            position = np.array([-.44-.02*index, -.93-.02*index, 1.0])
            target, phase = recovery.update(5, position, [-.28, -.25], 2.0, t)
            self.assertLessEqual(np.linalg.norm(target[:2]-position[:2]),
                                 .12+1e-9)
            self.assertEqual(phase, 'settle')

    def test_return_continues_past_nominal_window_then_requires_stability(self):
        recovery = BoundedPlanarCalibrationRecovery([0, 0, 1], self.config())
        position = np.array([-.44, -.93, 1.0])
        recovery.update(0, position, [-.28, -.25], 1.6, 0.0)
        for index in range(1, 6):
            recovery.update(0, position, [0.02, 0.01], 1.0, .05*index)
        self.assertEqual(recovery.phase, 'return')
        target, _ = recovery.update(0, position, [0.02, 0.01], 1.0, 5.0)
        self.assertFalse(recovery.complete)
        self.assertLessEqual(np.linalg.norm(target[:2]-position[:2]), .12+1e-9)
        with self.assertRaisesRegex(RuntimeError, 'did not settle'):
            recovery.require_complete()
        # The safety policy continues guiding during an extra wait. A
        # separate readiness gate then demands two seconds at the center.
        gate = CalibrationTrialReadinessGate({
            'trial_start_dwell_s': 2.0,
            'trial_start_timeout_s': 12.0,
            'trial_start_max_xy_speed_m_s': .05,
            'trial_start_max_tilt_deg': 4.0,
            'trial_start_max_position_error_m': .08,
        })
        gate.begin(1, 5.0)
        for index in range(1, 200):
            t = 5.0 + .05*index
            # Follow the bounded reference without violating its lead cap.
            position[:2] += (target[:2]-position[:2])*.6
            target, _ = recovery.update(0, position, [0.02, 0.01], 1.0, t)
            if recovery.complete:
                break
            gate.invalidate(t)
        self.assertTrue(recovery.complete)
        self.assertLessEqual(t, 17.0)
        self.assertFalse(gate.admitted(1))
        for index in range(41):
            t += .05
            self.assertEqual(
                gate.update(1, t, t, .02, 1.0, .01),
                index == 40,
            )
        self.assertTrue(gate.admitted(1))

    def test_recovery_rejects_tilt_and_unsafe_config(self):
        recovery = BoundedPlanarCalibrationRecovery([0, 0, 1], self.config())
        with self.assertRaisesRegex(RuntimeError, 'tilt limit'):
            recovery.update(0, [0, 0, 1], [0, 0], 25.1, 0.0)
        for change in ({'recovery_s': 2.0},
                       {'recovery_target_speed_m_s': .41},
                       {'recovery_max_target_lead_m': .16},
                       {'recovery_max_tilt_deg': 26.0}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                BoundedPlanarCalibrationRecovery(
                    [0, 0, 1], {**self.config(), **change},
                )


if __name__ == '__main__':
    unittest.main()
