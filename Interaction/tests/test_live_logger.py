"""The disk writer must not stall telemetry producers or lose accepted data."""

import io
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from Interaction.live_logger import LiveLogger, LiveLoggerError, LiveLoggerQueueFull


class ControlledFile(io.StringIO):
    """Keep contents after close and optionally block/fail record writes."""

    def __init__(self):
        super().__init__()
        self.entered = threading.Event()
        self.release = threading.Event()
        self.fail = False
        self.was_closed = False
        self.block = True

    def write(self, data):
        if data not in ("[\n", "\n]"):
            self.entered.set()
            if self.block and not self.release.wait(3):
                raise TimeoutError("test failed to release disk writer")
            if self.fail:
                raise OSError("simulated disk failure")
        return super().write(data)

    def close(self):
        self.was_closed = True


class LiveLoggerTests(unittest.TestCase):
    def make_controlled(self, **kwargs):
        file = ControlledFile()
        with patch("Interaction.live_logger.open", return_value=file):
            logger = LiveLogger("unused.json", limit=1, **kwargs)
        self.addCleanup(file.release.set)
        return logger, file

    def wait_for(self, predicate):
        deadline = time.monotonic() + 2
        while not predicate():
            self.assertLess(time.monotonic(), deadline)
            time.sleep(0.001)

    def test_producer_and_stats_do_not_wait_for_blocked_disk(self):
        logger, file = self.make_controlled()
        try:
            logger.write({"id": 0})
            self.assertTrue(file.entered.wait(1))
            completed = threading.Event()

            def produce():
                for i in range(1, 101):
                    logger.write({"id": i})
                logger.stats_snapshot()
                completed.set()

            producer = threading.Thread(target=produce)
            producer.start()
            self.assertTrue(completed.wait(1), "producer waited for blocked disk")
            self.assertFalse(file.release.is_set())
            producer.join(1)
        finally:
            file.release.set()
            logger.close()
        self.assertEqual(json.loads(file.getvalue()), [{"id": i} for i in range(101)])
        self.assertEqual(logger.stats_snapshot()["written_records"], 101)

    def test_concurrent_producers_complete_json_and_preserve_each_order(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "flight.json"
            logger = LiveLogger(path, limit=7)
            errors = []

            def produce(producer):
                try:
                    for sequence in range(250):
                        logger.write({"producer": producer, "sequence": sequence})
                except Exception as error:
                    errors.append(error)

            threads = [threading.Thread(target=produce, args=(i,)) for i in range(6)]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join(3)
                self.assertFalse(thread.is_alive())
            logger.close()
            self.assertEqual(errors, [])
            records = json.loads(path.read_text())
            self.assertEqual(len(records), 1500)
            for producer in range(6):
                self.assertEqual([r["sequence"] for r in records if r["producer"] == producer],
                                 list(range(250)))

    def test_snapshot_prevents_later_mutation_and_callback_stays_synchronous(self):
        file = ControlledFile()
        callback_threads = []

        def callback(record):
            callback_threads.append(threading.get_ident())
            record["callback"] = True

        with patch("Interaction.live_logger.open", return_value=file):
            logger = LiveLogger("unused.json", logger_function=callback)
        record = {"nested": [1]}
        try:
            logger.write(record)
            record["nested"].append(2)
            self.assertEqual(callback_threads, [threading.get_ident()])
        finally:
            file.release.set()
            logger.close()
        self.assertEqual(json.loads(file.getvalue()), [{"nested": [1], "callback": True}])

    def test_close_flushes_small_batch_and_is_idempotent(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "flight.json"
            logger = LiveLogger(path, limit=1000)
            with patch("Interaction.live_logger.time.time", return_value=123.5):
                logger.mark_start()
            logger.write({"value": 2})
            logger.close()
            logger.close()
            logger.flush_to_disk()
            self.assertEqual(json.loads(path.read_text()), [
                {"type": "start", "data": 123.5}, {"value": 2},
            ])
            stats = logger.stats_snapshot()
            self.assertEqual(stats["accepted_records"], stats["written_records"])
            self.assertEqual(stats["queued_records"], 0)
            self.assertTrue(stats["closed"])

    def test_empty_close_writes_valid_empty_array(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "flight.json"
            logger = LiveLogger(path)
            logger.close()
            self.assertEqual(json.loads(path.read_text()), [])

    def test_flush_waits_for_disk_and_covers_all_prior_records(self):
        logger, file = self.make_controlled()
        completed = threading.Event()
        try:
            logger.write({"id": 1})
            self.assertTrue(file.entered.wait(1))

            def flush():
                logger.flush_to_disk()
                completed.set()

            flusher = threading.Thread(target=flush)
            flusher.start()
            self.assertFalse(completed.wait(0.03))
            file.release.set()
            self.assertTrue(completed.wait(1))
            flusher.join(1)
            self.assertEqual(logger.stats_snapshot()["written_records"], 1)
        finally:
            file.release.set()
            logger.close()

    def test_background_io_failure_is_latched_and_reported(self):
        logger, file = self.make_controlled()
        file.fail = True
        logger.write({"id": 1})
        file.release.set()
        self.wait_for(lambda: logger.stats_snapshot()["closed"])
        for operation in (lambda: logger.write({"id": 2}),
                          logger.flush_to_disk, logger.close, logger.close):
            with self.assertRaisesRegex(LiveLoggerError, "simulated disk failure"):
                operation()
        self.assertEqual(logger.stats_snapshot()["writer_errors"], 1)
        self.assertTrue(file.was_closed)

    def test_header_flush_and_close_failures_are_not_silent(self):
        for failing_operation in ("header", "flush", "close"):
            with self.subTest(failing_operation=failing_operation):
                file = ControlledFile()
                file.block = False
                original_write = file.write

                def write(data):
                    if failing_operation == "header" and data == "[\n":
                        raise OSError("header failure")
                    return original_write(data)

                def flush():
                    if failing_operation == "flush":
                        raise OSError("flush failure")

                def close():
                    file.was_closed = True
                    if failing_operation == "close":
                        raise OSError("close failure")

                file.write, file.flush, file.close = write, flush, close
                with patch("Interaction.live_logger.open", return_value=file):
                    logger = LiveLogger("unused.json")
                with self.assertRaisesRegex(LiveLoggerError, failing_operation + " failure"):
                    logger.close()
                self.assertEqual(logger.stats_snapshot()["writer_errors"], 1)
                self.assertTrue(file.was_closed)

    def test_close_racing_with_producers_keeps_every_accepted_record(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "flight.json"
            logger = LiveLogger(path, limit=3)
            accepted = []
            errors = []
            started = threading.Event()

            def produce():
                for sequence in range(5000):
                    try:
                        logger.write({"sequence": sequence})
                    except RuntimeError as error:
                        if "closing or closed" not in str(error):
                            errors.append(error)
                        break
                    accepted.append(sequence)
                    started.set()

            producer = threading.Thread(target=produce)
            producer.start()
            self.assertTrue(started.wait(1))
            logger.close()
            producer.join(2)
            self.assertFalse(producer.is_alive())
            self.assertEqual(errors, [])
            self.assertEqual([r["sequence"] for r in json.loads(path.read_text())], accepted)

    def test_overflow_fails_explicitly_but_drains_accepted_records(self):
        logger, file = self.make_controlled(queue_capacity=2)
        try:
            logger.write({"id": 0})
            self.assertTrue(file.entered.wait(1))
            logger.write({"id": 1})
            logger.write({"id": 2})
            with self.assertRaises(LiveLoggerQueueFull):
                logger.write({"id": 3})
            with self.assertRaises(LiveLoggerError):
                logger.write({"id": 4})
            self.assertEqual(logger.stats_snapshot()["accepted_records"], 3)
            self.assertEqual(logger.stats_snapshot()["overflow_errors"], 1)
        finally:
            file.release.set()
            with self.assertRaises(LiveLoggerError):
                logger.close()
        self.assertEqual(json.loads(file.getvalue()), [{"id": i} for i in range(3)])

    def test_timeouts_are_bounded_and_close_can_retry(self):
        logger, file = self.make_controlled(flush_timeout_s=0.03)
        try:
            logger.write({"id": 1})
            self.assertTrue(file.entered.wait(1))
            with self.assertRaises(TimeoutError):
                logger.flush_to_disk()
            with self.assertRaises(TimeoutError):
                logger.close()
            with self.assertRaisesRegex(RuntimeError, "closing or closed"):
                logger.write({"id": 2})
        finally:
            file.release.set()
            self.wait_for(lambda: logger.stats_snapshot()["closed"])
            logger.close()
        self.assertEqual(json.loads(file.getvalue()), [{"id": 1}])

    def test_write_after_close_rejected_before_callback(self):
        callbacks = []
        with tempfile.TemporaryDirectory() as directory:
            logger = LiveLogger(Path(directory) / "flight.json", logger_function=callbacks.append)
            logger.close()
            with self.assertRaisesRegex(RuntimeError, "closing or closed"):
                logger.write({"id": 1})
        self.assertEqual(callbacks, [])

    def test_invalid_json_is_rejected_without_poisoning_logger(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "flight.json"
            logger = LiveLogger(path)
            with self.assertRaises(TypeError):
                logger.write({"unsupported": object()})
            logger.write({"valid": True})
            logger.close()
            self.assertEqual(json.loads(path.read_text()), [{"valid": True}])

    def test_invalid_limits_rejected(self):
        for arguments in ({"limit": 0}, {"limit": True}, {"queue_capacity": -1},
                          {"queue_capacity": 1.5}, {"flush_timeout_s": 0},
                          {"flush_timeout_s": float("inf")}):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                LiveLogger("unused.json", **arguments)


if __name__ == "__main__":
    unittest.main()
