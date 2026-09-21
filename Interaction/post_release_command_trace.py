"""Best-effort, release-window-only controller target telemetry; no authority."""
from collections import deque
import threading
import time
from cflib.crazyflie.log import LogConfig


class ReleaseCommandTrace:
    def __init__(self, cf, *, period_scale=10):
        self.cf = cf
        self.block = None
        self.rows = deque(maxlen=512)
        self.lock = threading.Lock()
        self.closed = False
        self.dropped = 0
        self.period_scale = period_scale

    def _append(self, row):
        with self.lock:
            if self.closed:
                return
            if len(self.rows) == self.rows.maxlen:
                self.dropped += 1
            self.rows.append(row)

    def _sample(self, timestamp, data, _config):
        self._append(dict(schema='release_controller_targets_v1',
            event='sample', fc_timestamp_ms=int(timestamp),
            pi_receive_monotonic_ns=time.monotonic_ns(), pi_receive_wall_s=time.time(),
            data=dict(data), source_snapshot_atomic=False,
            rate_semantics='controller desired body rates including attitude feedback',
            angle_unit='deg', rate_unit='deg/s'))

    def start(self):
        try:
            # Existing hardware uses the documented legacy x10 period scaling.
            # Six FP16 targets + stage = 13 payload bytes, one packet per sample.
            self.block = LogConfig(name='ReleaseTargets', period_in_ms=20*self.period_scale)
            for name in ('roll', 'pitch', 'yaw', 'rollRate', 'pitchRate', 'yawRate'):
                self.block.add_variable('controller.'+name, 'FP16')
            self.block.add_variable('hlCommander.pRelAutoSt', 'uint8_t')
            self.cf.log.add_config(self.block)
            self.block.data_received_cb.add_callback(self._sample)
            self.block.error_cb.add_callback(lambda cfg, msg: self._append(
                dict(event='error', detail=str(msg))))
            self.block.start()
            self._append(dict(event='start_requested', pi_monotonic_ns=time.monotonic_ns(),
                              requested_period_ms=20))
        except Exception as exc:
            self._append(dict(event='unavailable', detail=str(exc)))

    def drain(self):
        with self.lock:
            rows = list(self.rows)
            self.rows.clear()
            if self.dropped:
                rows.insert(0, dict(event='buffer_overflow', dropped=self.dropped))
                self.dropped = 0
            return rows

    def close(self):
        # Delete this dedicated block, including a create still in flight.
        # It never joins the required flight-state log groups.
        with self.lock:
            self.closed = True
        if self.block is not None:
            try:
                self.block.delete()
            except Exception as exc:
                with self.lock:
                    self.rows.append(dict(event='cleanup_error', detail=str(exc)))
