import unittest

from Interaction.calibrate_firmware_clock_rate import (
    fit_clock_rate,
    release_delay_in_firmware_clock,
)
from Interaction.evaluate_post_release_timing_shadow import (
    evaluate_timing_shadow,
)
from Interaction.tests.test_calibrate_firmware_clock_rate import session


class TimingShadowTests(unittest.TestCase):
    def test_assumed_few_ms_does_not_hide_stop_failure(self):
        clock = fit_clock_rate([session() for _ in range(5)])
        release = {
            "schema": "post_release_fixed_transport_delay_calibration_v1",
            "observed_lower_ms": 59.0,
            "observed_upper_ms": 62.0,
            "holdout_covered": True,
        }
        batch = {
            "schema": "bolt_clock_rate_prop_off_batch_v1",
            "clock_fit": clock,
            "release_fit": release,
            "firmware_delay_shadow": release_delay_in_firmware_clock(
                release, clock
            ),
        }
        result = evaluate_timing_shadow(batch, position_delay_ms=(0, 3, 5))
        self.assertTrue(result["position_delay_stress"][0]["harness_passed"])
        self.assertIn("stop_overshoot", result["position_delay_stress"][1]["failed_hard_gates"])
        self.assertIn("stop_overshoot", result["position_delay_stress"][2]["failed_hard_gates"])
        self.assertFalse(result["all_assumed_delays_passed"])
        corrected = result["fixed_position_delay_compensation_trial"]
        self.assertEqual(corrected["nominal_delay_ms"], 3.0)
        self.assertFalse(corrected["all_assumed_delays_passed"])
        self.assertIn("stop_overshoot", corrected["actual_delay_stress"][-1]["failed_hard_gates"])
        self.assertFalse(result["command_authority"])


if __name__ == "__main__":
    unittest.main()
