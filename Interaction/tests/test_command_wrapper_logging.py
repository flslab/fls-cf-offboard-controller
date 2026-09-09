"""Storage failures must not prevent explicit safety-cleanup commands."""

from unittest.mock import Mock
import unittest

import numpy as np

from Interaction.command_wrapper import CommandWrapper
from Interaction.live_logger import LiveLoggerError


class Commander:
    def __init__(self):
        self.land = Mock(return_value="landing")
        self.stop = Mock(return_value="stopped")
        self.go_to = Mock(return_value="positioned")


class CommandWrapperLoggingTests(unittest.TestCase):
    def make_wrapper(self, **kwargs):
        target = Commander()
        log = Mock()
        wrapper = CommandWrapper(target, log, start_time=100.0, **kwargs)
        return wrapper, target, log

    def test_original_failure_prevents_command_and_clone_does_not_mutate_it(self):
        wrapper, target, log = self.make_wrapper()
        log.side_effect = LiveLoggerError("disk failed")
        safety = wrapper.for_safety_cleanup()
        with self.assertRaises(LiveLoggerError):
            wrapper.land(0.1, 2)
        target.land.assert_not_called()
        with self.assertLogs("Interaction.command_wrapper", level="ERROR") as messages:
            self.assertEqual(safety.land(0.1, 2), "landing")
        target.land.assert_called_once_with(0.1, 2)
        self.assertIn("without a flight-log record", messages.output[0])
        with self.assertRaises(LiveLoggerError):
            wrapper.stop()
        target.stop.assert_not_called()
        self.assertIs(wrapper.log_function, log)

    def test_healthy_log_and_actual_return_value_preserved(self):
        wrapper, target, log = self.make_wrapper()
        safety = wrapper.for_safety_cleanup()
        self.assertEqual(safety.land(0.1, 2), "landing")
        target.land.assert_called_once_with(0.1, 2)
        log.assert_called_once()
        kwargs = log.call_args.kwargs
        self.assertEqual(kwargs["group_name"], "commands")
        self.assertEqual(kwargs["name"], "Commander.land")
        self.assertEqual(kwargs["entry"]["args"], (0.1, 2))
        self.assertEqual(safety.start_time, 100.0)
        self.assertIs(safety._wrapped_instance, target)

    def test_execution_disabled_stays_disabled_even_with_failed_log(self):
        wrapper, target, log = self.make_wrapper(execute=False)
        log.side_effect = LiveLoggerError("queue overflow")
        with self.assertLogs("Interaction.command_wrapper", level="ERROR"):
            self.assertIsNone(wrapper.for_safety_cleanup().stop())
        target.stop.assert_not_called()

    def test_offset_preserved_and_cloned_independently(self):
        offset = np.array([1.0, 2.0, 3.0])
        wrapper, target, log = self.make_wrapper(offset=offset)
        safety = wrapper.for_safety_cleanup()
        offset[:] = 10
        self.assertEqual(safety.go_to(4, 5, 6, 90, 2, relative=False), "positioned")
        target.go_to.assert_called_once_with(5.0, 7.0, 9.0, 90, 2, relative=False)
        self.assertEqual(log.call_args.kwargs["entry"]["args"], (5.0, 7.0, 9.0, 90, 2))
        np.testing.assert_array_equal(wrapper.offset, [10, 10, 10])

    def test_logging_programming_errors_are_not_bypassed(self):
        wrapper, target, log = self.make_wrapper()
        log.side_effect = ValueError("incorrect log data")
        with self.assertRaisesRegex(ValueError, "incorrect log data"):
            wrapper.for_safety_cleanup().land(0.1, 2)
        target.land.assert_not_called()

    def test_actual_command_error_propagates_after_log_failure(self):
        wrapper, target, log = self.make_wrapper()
        log.side_effect = LiveLoggerError("disk failed")
        target.stop.side_effect = OSError("radio failed")
        with self.assertLogs("Interaction.command_wrapper", level="ERROR"):
            with self.assertRaisesRegex(OSError, "radio failed"):
                wrapper.for_safety_cleanup().stop()

    def test_absent_logging_callback_stays_absent(self):
        for callback in (None, False):
            with self.subTest(callback=callback):
                target = Commander()
                safety = CommandWrapper(target, callback).for_safety_cleanup()
                self.assertIsNone(safety.log_function)
                self.assertEqual(safety.stop(), "stopped")
                target.stop.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
