import unittest

from Interaction.calibrate_release_transport_delay import (
    fit_release_transport_delay,
    release_delay_interval_ms,
)


def record(index, elapsed_us=60000, roundtrip_ms=1.0):
    identity = dict(session_id=42, sequence=index,
                    release_event_arduino_time_ms=index * 100,
                    pi_receive_to_send_elapsed_us=elapsed_us)
    return dict(schema="post_release_receive_event_prop_off_v1",
                sent=dict(identity),
                ack=dict(identity, radio_delivery_confirmed=True),
                host_radio_roundtrip_ms=roundtrip_ms)


class ReleaseTransportDelayCalibrationTests(unittest.TestCase):
    def test_conservative_one_way_interval(self):
        self.assertEqual(release_delay_interval_ms(record(0)), (60.0, 61.0))

    def test_identity_and_delivery_required(self):
        bad = record(0)
        bad["ack"]["sequence"] = 1
        with self.assertRaisesRegex(ValueError, "do not match"):
            release_delay_interval_ms(bad)
        bad = record(0)
        bad["ack"]["radio_delivery_confirmed"] = False
        with self.assertRaisesRegex(ValueError, "not confirmed"):
            release_delay_interval_ms(bad)

    def test_held_out_envelope_and_no_authority(self):
        rows = [record(i, elapsed_us=60000 + (i % 3) * 1000,
                       roundtrip_ms=1) for i in range(20)]
        fit = fit_release_transport_delay(rows, maximum_uncertainty_ms=2)
        self.assertEqual(fit["fixed_delay_ms"], 61.5)
        self.assertEqual(fit["empirical_uncertainty_ms"], 1.5)
        self.assertTrue(fit["calibration_eligible"])
        self.assertFalse(fit["position_transport_calibrated"])
        self.assertFalse(fit["command_authority"])

    def test_holdout_outlier_is_not_hidden_by_refit(self):
        rows = [record(i) for i in range(19)] + [record(19, 90000)]
        fit = fit_release_transport_delay(rows)
        self.assertFalse(fit["holdout_covered"])
        self.assertFalse(fit["calibration_eligible"])

    def test_no_implicit_uncertainty_budget(self):
        fit = fit_release_transport_delay([record(i) for i in range(20)])
        self.assertTrue(fit["holdout_covered"])
        self.assertFalse(fit["within_uncertainty_limit"])
        self.assertFalse(fit["calibration_eligible"])

    def test_insufficient_or_invalid_samples_rejected(self):
        with self.assertRaisesRegex(ValueError, "20 matched"):
            fit_release_transport_delay([record(i) for i in range(19)])
        with self.assertRaisesRegex(ValueError, "nonnegative"):
            release_delay_interval_ms(record(0, roundtrip_ms=float("nan")))


if __name__ == "__main__":
    unittest.main()
