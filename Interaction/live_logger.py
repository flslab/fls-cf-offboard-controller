"""Ordered flight-log recording without disk I/O on telemetry callbacks.

Only the dedicated writer touches the file after construction. ``write`` still
runs ``logger_function`` synchronously and serializes its argument before return:
a slow callback or unusually large JSON record can therefore still take time.
Queue overflow and background I/O failures are explicit, sticky failures, never
silent record drops. Call ``close`` during normal shutdown to drain the queue and
finish the JSON array; an abrupt process exit can still leave an incomplete log.
"""

from collections import deque
import json
import math
import threading
import time


class LiveLoggerError(RuntimeError):
    """The log cannot be considered complete; inspect the chained cause."""


class LiveLoggerQueueFull(LiveLoggerError):
    """A record was rejected because the bounded producer queue was full."""


class LiveLogger:
    def __init__(self, file_dir, logger_function=None, limit=20, *,
                 queue_capacity=20000, flush_timeout_s=5.0):
        for name, value in (("limit", limit), ("queue_capacity", queue_capacity)):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if not math.isfinite(flush_timeout_s) or flush_timeout_s <= 0:
            raise ValueError("flush_timeout_s must be finite and positive")

        self.buffer_limit = limit
        self.logger_function = logger_function
        self.log_file = open(file_dir, "w")
        self.first_item = True
        self._queue = deque()
        self._queue_capacity = queue_capacity
        self._timeout_s = float(flush_timeout_s)
        self._condition = threading.Condition()
        self._accepted = 0
        self._written = 0
        self._in_flight = 0
        self._max_depth = 0
        self._flush_target = 0
        self._overflow_errors = 0
        self._writer_errors = 0
        self._error = None
        self._header_ready = False
        self._closing = False
        self._closed = False
        self._writer = threading.Thread(
            target=self._writer_main, name="LiveLogger-writer", daemon=True,
        )
        try:
            self._writer.start()
        except Exception:
            self.log_file.close()
            raise

    def _raise_error_locked(self):
        if self._error is not None:
            raise LiveLoggerError(f"flight logger failed: {self._error}") from self._error

    def _check_writable_locked(self):
        self._raise_error_locked()
        if self._closing or self._closed:
            raise RuntimeError("cannot write to a closing or closed flight logger")

    def write(self, data):
        """Snapshot and enqueue one record; never wait for disk I/O.

        Concurrent producers are ordered at enqueue time, with each producer's
        call order preserved. The optional callback runs before JSON snapshotting
        (as in the original logger), outside all internal locks.
        """
        with self._condition:
            self._check_writable_locked()
        if self.logger_function:
            self.logger_function(data)
        serialized = json.dumps(data)
        with self._condition:
            self._check_writable_locked()
            if len(self._queue) >= self._queue_capacity:
                self._overflow_errors += 1
                self._error = LiveLoggerQueueFull(
                    f"flight log queue capacity {self._queue_capacity} exceeded; "
                    "this record was not accepted"
                )
                self._condition.notify_all()
                raise self._error
            self._queue.append(serialized)
            self._accepted += 1
            self._max_depth = max(self._max_depth, len(self._queue))
            self._condition.notify_all()

    def mark_start(self):
        self.write({"type": "start", "data": time.time()})

    def flush_to_disk(self):
        """Wait at most ``flush_timeout_s`` for previously accepted records.

        Later concurrent writes need not be flushed before this call returns.
        This is an explicit blocking barrier, not part of ``write``.
        """
        deadline = time.monotonic() + self._timeout_s
        with self._condition:
            self._raise_error_locked()
            target = self._accepted
            self._flush_target = max(self._flush_target, target)
            self._condition.notify_all()
            while not self._header_ready or self._written < target:
                self._raise_error_locked()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("timed out flushing flight log; writer may still be running")
                self._condition.wait(remaining)
            self._raise_error_locked()

    def close(self):
        """Drain accepted records, finish the JSON array, and report failures.

        Successful calls are idempotent. A timed-out writer remains responsible
        for the file and may finish later; a later ``close`` can wait again. Even
        after queue overflow, accepted records are drained before raising.
        """
        deadline = time.monotonic() + self._timeout_s
        with self._condition:
            self._closing = True
            self._condition.notify_all()
            while not self._closed:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("timed out closing flight log; writer may still be running")
                self._condition.wait(remaining)
            self._raise_error_locked()

    def stats_snapshot(self):
        """Return counters without waiting for disk I/O or queue drainage."""
        with self._condition:
            return {
                "accepted_records": self._accepted,
                "written_records": self._written,
                "queued_records": len(self._queue),
                "in_flight_records": self._in_flight,
                "max_queue_depth": self._max_depth,
                "queue_capacity": self._queue_capacity,
                "overflow_errors": self._overflow_errors,
                "writer_errors": self._writer_errors,
                "closing": self._closing,
                "closed": self._closed,
                "error": str(self._error) if self._error is not None else None,
            }

    def _record_writer_error(self, error):
        with self._condition:
            self._writer_errors += 1
            if self._error is None:
                self._error = error
            self._closing = True
            self._condition.notify_all()

    def _writer_main(self):
        try:
            self.log_file.write("[\n")
            self.log_file.flush()
            with self._condition:
                self._header_ready = True
                self._condition.notify_all()
            while True:
                with self._condition:
                    while not self._queue and not self._closing:
                        self._condition.wait()
                    if not self._queue and self._closing:
                        break
                    # Batch normal traffic, but never delay an explicit flush
                    # or shutdown. Sparse traffic reaches disk within 0.1 s.
                    batch_deadline = time.monotonic() + 0.1
                    while (len(self._queue) < self.buffer_limit
                           and not self._closing
                           and self._flush_target <= self._written):
                        remaining = batch_deadline - time.monotonic()
                        if remaining <= 0:
                            break
                        self._condition.wait(remaining)
                    batch = [self._queue.popleft() for _ in range(
                        min(len(self._queue), self.buffer_limit)
                    )]
                    self._in_flight = len(batch)

                # No producer or stats lock is held across file operations.
                prefix = "" if self.first_item else ",\n"
                self.log_file.write(prefix + ",\n".join(batch))
                self.log_file.flush()
                self.first_item = False
                with self._condition:
                    self._written += len(batch)
                    self._in_flight = 0
                    self._condition.notify_all()

            self.log_file.write("\n]")
            self.log_file.flush()
        except Exception as error:
            self._record_writer_error(error)
        finally:
            try:
                self.log_file.close()
            except Exception as error:
                self._record_writer_error(error)
            with self._condition:
                self._closed = True
                self._condition.notify_all()
