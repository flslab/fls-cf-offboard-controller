"""Shared FC source-clock mapping for runtime attitude and offline logs."""

from collections import deque
import math


class CrazyflieLogClockMapper:
    """Map wrapping Crazyflie log ticks onto host ``CLOCK_MONOTONIC``.

    A Crazyflie log packet contains the low 24 bits of the millisecond clock
    since the flight controller booted.  A rolling affine fit estimates the FC
    clock rate from callback receipts, while an initial lower-envelope phase
    estimate keeps published samples causal.  Subsequent timestamps advance by
    FC elapsed time at the fitted rate, so transport jitter does not distort the
    intervals used for attitude interpolation.  If causality requires a
    material phase correction, ``history_revision`` changes so the caller can
    prevent prediction across that clock boundary.
    """

    TIMESTAMP_MODULUS_MS = 1 << 24
    DEFAULT_WARMUP_SAMPLES = 16
    DEFAULT_WARMUP_DURATION_S = 0.5
    DEFAULT_RATE_WINDOW_S = 1.0
    DEFAULT_MAXIMUM_CONTINUITY_ERROR_S = 0.5
    MAXIMUM_CLOCK_RATE_ERROR = 0.01
    HISTORY_RESET_ABSOLUTE_ERROR_S = 10e-6
    HISTORY_RESET_RELATIVE_ERROR = 0.01

    def __init__(self, warmup_samples=DEFAULT_WARMUP_SAMPLES,
                 warmup_duration_s=DEFAULT_WARMUP_DURATION_S,
                 rate_window_s=DEFAULT_RATE_WINDOW_S,
                 maximum_continuity_error_s=
                 DEFAULT_MAXIMUM_CONTINUITY_ERROR_S):
        if (isinstance(warmup_samples, bool)
                or not isinstance(warmup_samples, int)
                or warmup_samples < 1):
            raise ValueError("warmup_samples must be a positive integer")
        warmup_duration_s = float(warmup_duration_s)
        if not math.isfinite(warmup_duration_s) or warmup_duration_s < 0.0:
            raise ValueError(
                "warmup_duration_s must be finite and non-negative"
            )
        rate_window_s = float(rate_window_s)
        if (not math.isfinite(rate_window_s)
                or rate_window_s <= 0.0
                or rate_window_s < warmup_duration_s):
            raise ValueError(
                "rate_window_s must be finite, positive, and no shorter "
                "than warmup_duration_s"
            )
        maximum_continuity_error_s = float(maximum_continuity_error_s)
        if (not math.isfinite(maximum_continuity_error_s)
                or maximum_continuity_error_s <= 0.0):
            raise ValueError(
                "maximum_continuity_error_s must be finite and positive"
            )
        self._warmup_samples = warmup_samples
        self._warmup_duration_s = warmup_duration_s
        self._rate_window_s = rate_window_s
        self._maximum_continuity_error_s = maximum_continuity_error_s
        self._history_revision = 0
        self._last_mapped_time_s = None
        self._clear_epoch()

    def _clear_epoch(self):
        self._last_raw_timestamp_ms = None
        self._unwrapped_timestamp_ms = None
        self._last_receive_time_s = None
        self._epoch_unwrapped_timestamp_ms = None
        self._epoch_receive_time_s = None
        self._sample_count = 0
        self._last_published_device_time_s = None
        self._clock_rate = 1.0
        self._rate_samples = deque()
        self._sum_device = 0.0
        self._sum_receive = 0.0
        self._sum_device_squared = 0.0
        self._sum_device_receive = 0.0

    def _start_epoch(self, raw_timestamp_ms, receive_time_s):
        restarting = self._last_raw_timestamp_ms is not None
        self._clear_epoch()
        self._last_raw_timestamp_ms = raw_timestamp_ms
        self._unwrapped_timestamp_ms = raw_timestamp_ms
        self._last_receive_time_s = receive_time_s
        self._epoch_unwrapped_timestamp_ms = raw_timestamp_ms
        self._epoch_receive_time_s = receive_time_s
        self._sample_count = 1
        self._append_rate_sample(0.0, 0.0)
        if restarting:
            self._history_revision += 1

    @property
    def history_revision(self):
        """Generation of the timestamp history used for interpolation."""
        return self._history_revision

    def configuration(self):
        """Parameters required to reproduce the mapper from raw receipts."""
        return {'algorithm': 'fc_lower_envelope_affine_v1',
                'timestamp_modulus_ms': self.TIMESTAMP_MODULUS_MS,
                'warmup_samples': self._warmup_samples,
                'warmup_duration_s': self._warmup_duration_s,
                'rate_window_s': self._rate_window_s,
                'maximum_continuity_error_s': self._maximum_continuity_error_s,
                'maximum_clock_rate_error': self.MAXIMUM_CLOCK_RATE_ERROR,
                'history_reset_absolute_error_s': self.HISTORY_RESET_ABSOLUTE_ERROR_S,
                'history_reset_relative_error': self.HISTORY_RESET_RELATIVE_ERROR}

    def project(self, raw_timestamp_ms, receive_time_s):
        """Place another log group on the reference clock without updating it.

        Different groups may arrive out of tick order. Feeding them through
        map() would reset the reference clock or discard coincident ticks.
        The signed modular difference handles wrap for nearby packets.
        """
        raw = self._validate_raw_timestamp(raw_timestamp_ms)
        received = float(receive_time_s)
        if not math.isfinite(received):
            raise ValueError('receive_time_s must be finite')
        if self._last_published_device_time_s is None:
            return None
        if abs(received-self._last_receive_time_s) > self._maximum_continuity_error_s:
            return None
        modulus = self.TIMESTAMP_MODULUS_MS
        delta = ((raw-self._last_raw_timestamp_ms+modulus//2) % modulus-modulus//2)/1000.
        if abs(delta) > self._maximum_continuity_error_s:
            return None
        device_time = (self._unwrapped_timestamp_ms-self._epoch_unwrapped_timestamp_ms)/1000.+delta
        mapped = self._last_mapped_time_s+(device_time-self._last_published_device_time_s)*self._clock_rate
        # Preserve the common clock phase: do not clamp one group to its
        # receipt time and silently introduce a different inter-sensor offset.
        return mapped if mapped <= received else None

    def _append_rate_sample(self, device_time_s, receive_time_s):
        sample = (device_time_s, receive_time_s)
        self._rate_samples.append(sample)
        self._sum_device += device_time_s
        self._sum_receive += receive_time_s
        self._sum_device_squared += device_time_s * device_time_s
        self._sum_device_receive += device_time_s * receive_time_s

        cutoff = receive_time_s - self._rate_window_s
        while (len(self._rate_samples) > 2
               and self._rate_samples[0][1] < cutoff):
            old_device, old_receive = self._rate_samples.popleft()
            self._sum_device -= old_device
            self._sum_receive -= old_receive
            self._sum_device_squared -= old_device * old_device
            self._sum_device_receive -= old_device * old_receive

    def _update_clock_rate(self):
        count = len(self._rate_samples)
        if count < 2:
            return
        denominator = (
            count * self._sum_device_squared
            - self._sum_device * self._sum_device
        )
        if denominator <= 1e-15:
            return
        rate = (
            count * self._sum_device_receive
            - self._sum_device * self._sum_receive
        ) / denominator
        if (math.isfinite(rate)
                and abs(rate - 1.0) <= self.MAXIMUM_CLOCK_RATE_ERROR):
            self._clock_rate = rate

    def _initial_mapped_time(self, device_time_s):
        phase_s = min(
            receive_time_s - self._clock_rate * sample_time_s
            for sample_time_s, receive_time_s in self._rate_samples
        )
        return (
            self._epoch_receive_time_s
            + self._clock_rate * device_time_s
            + phase_s
        )

    @classmethod
    def _validate_raw_timestamp(cls, raw_timestamp_ms):
        if isinstance(raw_timestamp_ms, bool):
            raise ValueError("Crazyflie log timestamp must be an integer")
        try:
            numeric = float(raw_timestamp_ms)
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError(
                "Crazyflie log timestamp must be an integer"
            ) from error
        if not math.isfinite(numeric) or not numeric.is_integer():
            raise ValueError("Crazyflie log timestamp must be an integer")
        raw_timestamp_ms = int(numeric)
        if not 0 <= raw_timestamp_ms < cls.TIMESTAMP_MODULUS_MS:
            raise ValueError("Crazyflie log timestamp is outside 24-bit range")
        return raw_timestamp_ms

    def map(self, raw_timestamp_ms, receive_time_s):
        """Return source time in host-monotonic seconds, or ``None`` while unsafe."""
        raw_timestamp_ms = self._validate_raw_timestamp(raw_timestamp_ms)
        receive_time_s = float(receive_time_s)
        if not math.isfinite(receive_time_s):
            raise ValueError("receive_time_s must be finite")

        if self._last_raw_timestamp_ms is None:
            self._start_epoch(raw_timestamp_ms, receive_time_s)
        else:
            device_delta_ms = (
                raw_timestamp_ms - self._last_raw_timestamp_ms
            ) % self.TIMESTAMP_MODULUS_MS
            if device_delta_ms == 0:
                return None
            receive_delta_s = receive_time_s - self._last_receive_time_s
            device_delta_s = device_delta_ms / 1000.0
            discontinuity = (
                receive_delta_s < 0.0
                or device_delta_ms >= self.TIMESTAMP_MODULUS_MS // 2
                or abs(device_delta_s - receive_delta_s)
                > self._maximum_continuity_error_s
            )
            if discontinuity:
                self._start_epoch(raw_timestamp_ms, receive_time_s)
                return None

            self._last_raw_timestamp_ms = raw_timestamp_ms
            self._unwrapped_timestamp_ms += device_delta_ms
            self._last_receive_time_s = receive_time_s
            self._sample_count += 1

            device_time_s = (
                self._unwrapped_timestamp_ms
                - self._epoch_unwrapped_timestamp_ms
            ) / 1000.0
            receive_elapsed_s = receive_time_s - self._epoch_receive_time_s
            self._append_rate_sample(device_time_s, receive_elapsed_s)

        device_time_s = (
            self._unwrapped_timestamp_ms
            - self._epoch_unwrapped_timestamp_ms
        ) / 1000.0
        receive_elapsed_s = receive_time_s - self._epoch_receive_time_s
        self._update_clock_rate()
        if (self._sample_count < self._warmup_samples
                or receive_elapsed_s < self._warmup_duration_s):
            return None

        if self._last_published_device_time_s is None:
            mapped_time_s = min(
                self._initial_mapped_time(device_time_s), receive_time_s
            )
        else:
            device_delta_s = (
                device_time_s - self._last_published_device_time_s
            )
            expected_interval_s = device_delta_s * self._clock_rate
            predicted_time_s = (
                self._last_mapped_time_s + expected_interval_s
            )
            mapped_time_s = min(predicted_time_s, receive_time_s)
            correction_s = predicted_time_s - mapped_time_s
            reset_threshold_s = max(
                self.HISTORY_RESET_ABSOLUTE_ERROR_S,
                expected_interval_s * self.HISTORY_RESET_RELATIVE_ERROR,
            )
            if correction_s > reset_threshold_s:
                self._history_revision += 1

        # A callback clock that does not advance cannot safely place another
        # sample. Keep the previous published device epoch so the next usable
        # packet covers the complete FC interval.
        if (self._last_mapped_time_s is not None
                and mapped_time_s <= self._last_mapped_time_s):
            return None
        self._last_mapped_time_s = mapped_time_s
        self._last_published_device_time_s = device_time_s
        return mapped_time_s
