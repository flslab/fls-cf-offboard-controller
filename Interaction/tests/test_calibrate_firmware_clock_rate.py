import unittest

from Interaction.calibrate_firmware_clock_rate import (
    fit_clock_rate,
    release_delay_in_firmware_clock,
    session_rate_interval,
)


def session(ratio=0.99, *, start_firmware_us=1000000, rtt_us=1000):
    rows = []
    for index in range(32):
        host_send_us = index * 50000
        rows.append({
            "bolt_receive_us_mod32": (
                start_firmware_us + round(host_send_us * ratio)
            ) % (1 << 32),
            "host_send_start_monotonic_ns": host_send_us * 1000,
            "host_reply_end_monotonic_ns": (host_send_us + rtt_us) * 1000,
        })
    return {"schema": "bolt_radio_clock_brackets_v1", "probes": rows}


class FirmwareClockRateCalibrationTests(unittest.TestCase):
    def test_causal_rate_interval_requires_no_clock_epoch(self):
        result = session_rate_interval(session(start_firmware_us=4000000000))
        self.assertLess(result["ratio_lower"], 0.99)
        self.assertGreater(result["ratio_upper"], 0.99)
        self.assertEqual(result["warmup_probes_excluded"], 1)

    def test_wrap_and_stable_held_out_session(self):
        result = fit_clock_rate([
            session(start_firmware_us=(1 << 32) - 500000, ratio=0.99)
            for _ in range(5)
        ])
        self.assertTrue(result["holdout_compatible"])
        self.assertFalse(result["command_authority"])

    def test_drifting_holdout_is_not_called_stable(self):
        result = fit_clock_rate([session() for _ in range(4)]
                                + [session(ratio=0.97)])
        self.assertFalse(result["holdout_compatible"])
        self.assertLessEqual(result["ratio_lower"], 0.97)

    def test_invalid_order_and_short_baseline(self):
        bad = session()
        bad["probes"][-1]["host_send_start_monotonic_ns"] = 0
        with self.assertRaisesRegex(ValueError, "out of order"):
            session_rate_interval(bad)
        with self.assertRaisesRegex(ValueError, "at least 5"):
            fit_clock_rate([session() for _ in range(4)])

    def test_release_product_bounds_do_not_grant_authority(self):
        clock = fit_clock_rate([session() for _ in range(5)])
        release = {
            "schema": "post_release_fixed_transport_delay_calibration_v1",
            "observed_lower_ms": 59.0,
            "observed_upper_ms": 62.0,
            "holdout_covered": True,
        }
        result = release_delay_in_firmware_clock(release, clock)
        self.assertLess(result["observed_lower_firmware_ms"], 59 * 0.99)
        self.assertGreater(result["observed_upper_firmware_ms"], 62 * 0.99)
        self.assertFalse(result["position_transport_calibrated"])
        self.assertFalse(result["command_authority"])


if __name__ == "__main__":
    unittest.main()
