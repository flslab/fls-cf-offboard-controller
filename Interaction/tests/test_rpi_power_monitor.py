import subprocess
import time
import unittest

from Interaction.rpi_power_monitor import (
    RaspberryPiPowerMonitor,
    parse_get_throttled_output,
)


class RaspberryPiPowerMonitorTest(unittest.TestCase):
    def test_parses_current_and_historical_power_flags(self):
        sample = parse_get_throttled_output(
            "throttled=0x50005\n", host_time=12.5
        )

        self.assertEqual(sample.host_time, 12.5)
        self.assertEqual(sample.flags, 0x50005)
        self.assertTrue(sample.under_voltage_now)
        self.assertTrue(sample.throttled_now)
        self.assertTrue(sample.under_voltage_occurred)
        self.assertTrue(sample.throttled_occurred)
        self.assertFalse(sample.frequency_capped_now)

    def test_rejects_unexpected_output(self):
        with self.assertRaises(ValueError):
            parse_get_throttled_output("unknown")

    def test_background_monitor_refreshes_samples(self):
        outputs = iter(("throttled=0x0\n", "throttled=0x10001\n"))

        def fake_runner(*args, **kwargs):
            return subprocess.CompletedProcess(
                args=args[0], returncode=0, stdout=next(outputs), stderr=""
            )

        monitor = RaspberryPiPowerMonitor(
            poll_interval_s=0.01,
            command_runner=fake_runner,
        )
        monitor.start()
        deadline = time.monotonic() + 0.2
        while (
            monitor.latest() is not None
            and monitor.latest().flags == 0
            and time.monotonic() < deadline
        ):
            time.sleep(0.005)
        monitor.stop()

        self.assertEqual(monitor.latest().flags, 0x10001)
        self.assertTrue(monitor.latest().under_voltage_now)
        self.assertTrue(monitor.latest().under_voltage_occurred)


if __name__ == "__main__":
    unittest.main()
