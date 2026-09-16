import unittest

import numpy as np

from Interaction.calibration_attitude_response import (
    identify_attitude_acceleration_axis,
    identify_second_order_axis,
)
from Interaction.model_based_braking import _second_order_transition


class CalibrationAttitudeResponseTests(unittest.TestCase):
    def test_recovers_tilt_to_acceleration_gain(self):
        dt = 0.01
        times = np.arange(0.0, 12.0, dt)
        nominal = 2.5*np.sin(2*np.pi*0.31*times)
        actual_acceleration = 1.08*nominal+0.025
        velocity = np.cumsum(actual_acceleration)*dt

        fit = identify_attitude_acceleration_axis(times, velocity, nominal)

        self.assertTrue(fit["usable"])
        self.assertAlmostEqual(fit["gain"], 1.08, delta=0.01)
        self.assertAlmostEqual(fit["bias_m_s2"], 0.025, delta=0.01)
        self.assertGreater(fit["r_squared"], 0.99)

    def test_recovers_delayed_second_order_response(self):
        dt = 0.01
        times = np.arange(0.0, 12.0, dt)
        command = 9.0*np.sin(2*np.pi*(0.25*times + 0.035*times**2))
        delay, wn, zeta, gain, bias = 0.04, 13.0, 0.48, 0.96, 0.12
        delayed = np.interp(times-delay, times, command, left=command[0])
        transition = _second_order_transition(wn, zeta, dt)
        angle = bias
        rate = 0.0
        measured = []
        for input_deg in delayed:
            measured.append(angle)
            target = gain*input_deg+bias
            error, rate = transition @ np.array([angle-target, rate])
            angle = target+error

        fit = identify_second_order_axis(times, command, measured)

        self.assertTrue(fit["usable"])
        self.assertAlmostEqual(fit["delay_s"], delay, delta=0.006)
        self.assertAlmostEqual(fit["wn_rad_s"], wn, delta=0.7)
        self.assertAlmostEqual(fit["zeta"], zeta, delta=0.06)
        self.assertAlmostEqual(fit["gain"], gain, delta=0.04)
        self.assertGreater(fit["r_squared"], 0.995)

    def test_accepts_large_chirp_with_small_relative_fit_error(self):
        dt = 0.01
        times = np.arange(0.0, 12.0, dt)
        command = 20.0*np.sin(2*np.pi*(0.25*times + 0.035*times**2))
        transition = _second_order_transition(12.0, 0.45, dt)
        angle = 0.0
        rate = 0.0
        measured = []
        for input_deg in command:
            measured.append(angle)
            error, rate = transition @ np.array([angle-input_deg, rate])
            angle = input_deg+error
        # A deterministic high-frequency component represents unmodelled
        # vibration.  Its absolute RMSE can exceed 2 degrees while remaining a
        # small fraction of the calibration excitation.
        measured = np.asarray(measured)+2.8*np.sin(2*np.pi*19.0*times)

        fit = identify_second_order_axis(times, command, measured)

        self.assertTrue(fit["usable"])
        self.assertGreater(fit["rmse_deg"], 2.0)
        self.assertLess(fit["normalized_rmse"], 0.15)


if __name__ == "__main__":
    unittest.main()
