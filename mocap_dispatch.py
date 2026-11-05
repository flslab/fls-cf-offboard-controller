"""One-slot dispatch for immutable mocap snapshots; acquisition never waits on callbacks."""
from __future__ import annotations

import threading


class LatestFrameDispatcher:
    def __init__(self, callback, on_error):
        self.callback = callback
        self.on_error = on_error
        self.condition = threading.Condition()
        self.pending = None
        self.closed = False
        self.overwritten = 0
        self.dispatched = 0
        self.thread = threading.Thread(target=self._run, name='mocap-callbacks', daemon=True)
        self.thread.start()

    def submit(self, value):
        with self.condition:
            if self.closed:
                return False
            if self.pending is not None:
                self.overwritten += 1
            self.pending = value
            self.condition.notify()
        return True

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.closed or self.pending is not None)
                if self.closed:
                    return
                value, self.pending = self.pending, None
                counts = {'callback_queue_capacity': 1,
                          'callback_queue_overwritten_total': self.overwritten,
                          'callback_frames_dispatched': self.dispatched + 1}
                self.dispatched += 1
            try:
                self.callback(value, counts)
            except Exception as error:
                with self.condition:
                    self.closed = True
                    self.pending = None
                    self.condition.notify_all()
                self.on_error(error)
                return

    def close(self):
        # Do not replay pending control poses during shutdown.
        with self.condition:
            self.closed = True
            self.pending = None
            self.condition.notify_all()
        if threading.current_thread() is not self.thread:
            self.thread.join()
