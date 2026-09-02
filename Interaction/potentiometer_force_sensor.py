"""Threaded Arduino potentiometer force-sensor reader.

The current Arduino sketch emits CSV rows with the following columns::

    time_ms,raw,filtered,voltage,compression_mm[,supply_voltage]

Firmware with the legacy ``distance_mm`` header remains positionally compatible;
its fifth value is also interpreted as compression.  The spring length is
``max_extension_mm - compression_mm`` and force follows Hooke's law.  The
reader owns the serial port and exposes the latest immutable sample so the
flight-control loop never blocks on UART I/O.
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
import math
import threading
import time
from typing import Callable

import serial


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class PotentiometerForceSample:
    host_time: float
    arduino_time_ms: int
    raw: int
    filtered_raw: float
    voltage_v: float
    supply_voltage_v: float | None
    compression_mm: float
    length_mm: float
    force_n: float


@dataclass(frozen=True)
class PotentiometerReleaseDecision:
    released: bool
    candidate_started: bool
    candidate_active: bool
    candidate_cancelled: bool
    current_force_n: float
    last_force_n: float | None
    peak_force_n: float
    force_rate_n_s: float
    force_drop_n: float
    unloaded_elapsed_s: float
    candidate_elapsed_s: float
    candidate_cancel_reason: str | None
    pre_release_force_n: float | None


@dataclass(frozen=True)
class PotentiometerContactDecision:
    started: bool
    active: bool
    ready: bool
    current_force_n: float
    peak_force_n: float
    onset_elapsed_s: float


class PotentiometerContactDetector:
    """Detect a new interaction from sustained spring compression.

    A reset detector must first observe a force below the onset threshold.
    This prevents a still-compressed spring from retriggering immediately after
    the post-coast grace period.
    """

    def __init__(self, force_threshold_n=0.08, onset_dwell_s=0.03):
        self.force_threshold_n = float(force_threshold_n)
        self.onset_dwell_s = float(onset_dwell_s)
        if (
            not math.isfinite(self.force_threshold_n)
            or not math.isfinite(self.onset_dwell_s)
            or self.force_threshold_n <= 0.0
            or self.onset_dwell_s < 0.0
        ):
            raise ValueError(
                'potentiometer contact threshold must be positive and dwell '
                'must be non-negative'
            )
        self.reset()

    def reset(self):
        self.ready = False
        self.active = False
        self._onset_started_at = None
        self._peak_force_n = 0.0
        self._last_timestamp = None

    def mark_released(self):
        """End the active contact without permitting an immediate retrigger."""
        self.ready = False
        self.active = False
        self._onset_started_at = None

    def update(self, force_n, timestamp, enabled=True):
        force_n = max(float(force_n), 0.0)
        timestamp = float(timestamp)
        if not math.isfinite(force_n) or not math.isfinite(timestamp):
            raise ValueError('potentiometer contact values must be finite')

        started = False
        if (
            self._last_timestamp is not None
            and timestamp < self._last_timestamp
        ):
            self.reset()
        self._last_timestamp = timestamp

        if not bool(enabled):
            self._onset_started_at = None
        elif not self.active:
            if force_n < self.force_threshold_n:
                self.ready = True
                self._onset_started_at = None
                self._peak_force_n = 0.0
            elif self.ready:
                self._peak_force_n = max(self._peak_force_n, force_n)
                if self._onset_started_at is None:
                    self._onset_started_at = timestamp
                if timestamp - self._onset_started_at >= self.onset_dwell_s:
                    self.active = True
                    started = True

        onset_elapsed_s = (
            0.0
            if self._onset_started_at is None
            else max(0.0, timestamp - self._onset_started_at)
        )
        return PotentiometerContactDecision(
            started=started,
            active=bool(self.active),
            ready=bool(self.ready),
            current_force_n=force_n,
            peak_force_n=float(self._peak_force_n),
            onset_elapsed_s=float(onset_elapsed_s),
        )


class PotentiometerReleaseDetector:
    """Confirm release only after unloading returns close to baseline.

    A sufficiently fast force decrease first creates a release candidate.  It
    does not end contact immediately: the spring must then remain below the
    unloaded-force threshold for a short dwell.  This separates "the user has
    started letting go" from "the force transfer has finished", so the
    coasting state is initialized from the post-unloading vehicle velocity.
    """

    def __init__(
            self,
            force_drop_n=0.01,
            decrease_rate_n_s=0.05,
            unloaded_force_n=0.05,
            unloaded_dwell_s=0.05,
            max_sample_gap_s=0.15,
            candidate_stall_timeout_s=0.50,
    ):
        self.force_drop_n = float(force_drop_n)
        self.decrease_rate_n_s = float(decrease_rate_n_s)
        self.unloaded_force_n = float(unloaded_force_n)
        self.unloaded_dwell_s = float(unloaded_dwell_s)
        self.max_sample_gap_s = float(max_sample_gap_s)
        self.candidate_stall_timeout_s = float(candidate_stall_timeout_s)
        if (
            not math.isfinite(self.force_drop_n)
            or not math.isfinite(self.decrease_rate_n_s)
            or not math.isfinite(self.unloaded_force_n)
            or not math.isfinite(self.unloaded_dwell_s)
            or not math.isfinite(self.max_sample_gap_s)
            or not math.isfinite(self.candidate_stall_timeout_s)
            or self.force_drop_n <= 0.0
            or self.decrease_rate_n_s <= 0.0
            or self.unloaded_force_n < 0.0
            or self.unloaded_dwell_s < 0.0
            or self.max_sample_gap_s <= 0.0
            or self.candidate_stall_timeout_s <= 0.0
        ):
            raise ValueError(
                'potentiometer release drop/rate thresholds must be positive; '
                'unloaded threshold/dwell must be non-negative and maximum '
                'sample gap/candidate stall timeout must be positive'
            )
        self.disarm()

    def disarm(self):
        self.armed = False
        self.released = False
        self._last_timestamp = None
        self._last_force_n = None
        self._peak_force_n = 0.0
        self._edge_peak_force_n = 0.0
        self._candidate_active = False
        self._candidate_reference_force_n = None
        self._pre_release_force_n = None
        self._unloaded_started_at = None
        self._candidate_started_at = None
        self._candidate_min_force_n = None
        self._candidate_last_progress_at = None

    def arm(self, force_n, timestamp, peak_force_n=None):
        force_n = max(float(force_n), 0.0)
        timestamp = float(timestamp)
        peak_force_n = (
            force_n if peak_force_n is None
            else max(float(peak_force_n), force_n)
        )
        if (
            not math.isfinite(force_n)
            or not math.isfinite(timestamp)
            or not math.isfinite(peak_force_n)
        ):
            raise ValueError('potentiometer release arm values must be finite')
        self.armed = True
        self.released = False
        self._last_timestamp = timestamp
        self._last_force_n = force_n
        self._peak_force_n = peak_force_n
        # The inherited contact peak is useful for diagnostics, but release
        # onset must use a local edge peak. Otherwise a historical large force
        # makes any later tiny negative slope look like a new release.
        self._edge_peak_force_n = force_n
        self._candidate_active = False
        self._candidate_reference_force_n = None
        self._pre_release_force_n = None
        self._unloaded_started_at = None
        self._candidate_started_at = None
        self._candidate_min_force_n = None
        self._candidate_last_progress_at = None

    @property
    def candidate_active(self):
        return bool(self._candidate_active)

    def _clear_candidate(self, rebase_edge_force_n=None):
        self._candidate_active = False
        self._candidate_reference_force_n = None
        self._pre_release_force_n = None
        self._unloaded_started_at = None
        self._candidate_started_at = None
        self._candidate_min_force_n = None
        self._candidate_last_progress_at = None
        if rebase_edge_force_n is not None:
            self._edge_peak_force_n = max(
                float(rebase_edge_force_n), 0.0
            )

    def _start_candidate(self, reference_force_n, force_n, timestamp):
        reference_force_n = max(float(reference_force_n), float(force_n))
        self._candidate_active = True
        self._candidate_reference_force_n = reference_force_n
        self._pre_release_force_n = reference_force_n
        self._candidate_started_at = float(timestamp)
        self._candidate_min_force_n = float(force_n)
        self._candidate_last_progress_at = float(timestamp)
        self._unloaded_started_at = None

    def cancel_candidate(self, preserve_loaded_evidence=False):
        """Cancel a pending release and keep detector/control state aligned.

        When a serial sample goes stale, retaining the last loaded evidence
        lets the first recovered unloaded sample start a fresh dwell.  The
        stale interval itself is never credited toward that dwell.
        """
        if not self._candidate_active:
            return False
        edge_force = (
            None if preserve_loaded_evidence else self._last_force_n
        )
        self._clear_candidate(rebase_edge_force_n=edge_force)
        return True

    def update(self, force_n, timestamp):
        force_n = max(float(force_n), 0.0)
        timestamp = float(timestamp)
        if not math.isfinite(force_n) or not math.isfinite(timestamp):
            raise ValueError('potentiometer release values must be finite')
        previous_force = self._last_force_n
        rate = 0.0
        candidate_started = False
        candidate_cancelled = False
        candidate_cancel_reason = None
        candidate_elapsed_s = 0.0
        reported_force_drop_n = None
        timestamp_advanced = bool(
            self._last_timestamp is not None
            and timestamp > self._last_timestamp
        )
        timestamp_rolled_back = bool(
            self._last_timestamp is not None
            and timestamp < self._last_timestamp
        )
        sample_gap_too_large = bool(
            timestamp_advanced
            and timestamp - self._last_timestamp > self.max_sample_gap_s
        )
        if self.armed and timestamp_rolled_back:
            candidate_cancelled = bool(self._candidate_active)
            if candidate_cancelled:
                candidate_cancel_reason = 'timestamp_rollback'
                if self._candidate_reference_force_n is not None:
                    reported_force_drop_n = max(
                        self._candidate_reference_force_n - force_n, 0.0
                    )
                if self._candidate_started_at is not None:
                    candidate_elapsed_s = max(
                        0.0, timestamp - self._candidate_started_at
                    )
            self._clear_candidate(rebase_edge_force_n=force_n)
            self._last_timestamp = timestamp
            self._last_force_n = force_n
        elif self.armed and sample_gap_too_large:
            # Do not count a missing-data interval as unloaded dwell.  If the
            # signal went from a known loaded contact to unloaded during the
            # gap, start (or restart) dwell at this recovered sample.
            gap_reference_force_n = max(
                self._edge_peak_force_n,
                0.0 if previous_force is None else float(previous_force),
            )
            gap_release_evidence_n = max(
                gap_reference_force_n, self._peak_force_n
            ) - force_n
            recovered_unloaded = bool(
                force_n <= self.unloaded_force_n
                and gap_release_evidence_n >= self.force_drop_n
            )
            if recovered_unloaded:
                if not self._candidate_active:
                    self._start_candidate(
                        gap_reference_force_n, force_n, timestamp
                    )
                    candidate_started = True
                else:
                    self._candidate_min_force_n = force_n
                    self._candidate_last_progress_at = timestamp
                self._unloaded_started_at = timestamp
            else:
                candidate_cancelled = bool(self._candidate_active)
                if candidate_cancelled:
                    candidate_cancel_reason = 'sample_gap'
                    if self._candidate_reference_force_n is not None:
                        reported_force_drop_n = max(
                            self._candidate_reference_force_n - force_n, 0.0
                        )
                    if self._candidate_started_at is not None:
                        candidate_elapsed_s = max(
                            0.0, timestamp - self._candidate_started_at
                        )
                self._clear_candidate(rebase_edge_force_n=force_n)
            self._peak_force_n = max(self._peak_force_n, force_n)
            self._last_timestamp = timestamp
            self._last_force_n = force_n
        if (
            self.armed
            and not self.released
            and not timestamp_rolled_back
            and not sample_gap_too_large
            and self._last_timestamp is not None
            and previous_force is not None
            and timestamp > self._last_timestamp
        ):
            rate = (force_n - previous_force) / (
                timestamp - self._last_timestamp
            )
            self._peak_force_n = max(self._peak_force_n, force_n)
            if not self._candidate_active:
                # Rebase after flat/rising/slowly falling motion.  Only a
                # contiguous fast falling edge may accumulate the configured
                # onset drop; an old contact peak cannot trigger a later tiny
                # dip.
                if rate > -self.decrease_rate_n_s:
                    self._edge_peak_force_n = force_n
                else:
                    self._edge_peak_force_n = max(
                        self._edge_peak_force_n, previous_force
                    )
            edge_drop = max(self._edge_peak_force_n - force_n, 0.0)
            if not self._candidate_active and bool(
                    edge_drop >= self.force_drop_n
                    and rate <= -self.decrease_rate_n_s):
                self._start_candidate(
                    self._edge_peak_force_n, force_n, timestamp
                )
                candidate_started = True

            if self._candidate_active:
                progress_epsilon_n = max(
                    0.001, min(0.005, 0.10 * self.force_drop_n)
                )
                if (
                    self._candidate_min_force_n is None
                    or force_n
                    <= self._candidate_min_force_n - progress_epsilon_n
                ):
                    self._candidate_min_force_n = force_n
                    self._candidate_last_progress_at = timestamp

                candidate_elapsed_s = max(
                    0.0, timestamp - self._candidate_started_at
                )
                rebounded = bool(
                    not candidate_started
                    and force_n > self.unloaded_force_n
                    and self._candidate_min_force_n is not None
                    and force_n >= (
                        self._candidate_min_force_n + self.force_drop_n
                    )
                )
                stalled = bool(
                    force_n > self.unloaded_force_n
                    and self._candidate_last_progress_at is not None
                    and timestamp - self._candidate_last_progress_at
                    >= self.candidate_stall_timeout_s
                )
                if rebounded or stalled:
                    candidate_cancelled = True
                    candidate_cancel_reason = (
                        'force_rebound' if rebounded
                        else 'no_downward_progress'
                    )
                    if self._candidate_reference_force_n is not None:
                        reported_force_drop_n = max(
                            self._candidate_reference_force_n - force_n, 0.0
                        )
                    self._clear_candidate(rebase_edge_force_n=force_n)

            if self._candidate_active and force_n <= self.unloaded_force_n:
                if self._unloaded_started_at is None:
                    self._unloaded_started_at = timestamp
                self.released = bool(
                    timestamp - self._unloaded_started_at
                    >= self.unloaded_dwell_s
                )
                if self.released:
                    self._candidate_active = False
            else:
                self._unloaded_started_at = None
            self._last_timestamp = timestamp
            self._last_force_n = force_n

        if reported_force_drop_n is not None:
            force_drop = reported_force_drop_n
        elif self._candidate_reference_force_n is not None:
            force_drop = max(
                self._candidate_reference_force_n - force_n, 0.0
            )
        else:
            force_drop = max(self._edge_peak_force_n - force_n, 0.0)

        unloaded_elapsed_s = (
            0.0
            if self._unloaded_started_at is None
            else max(0.0, timestamp - self._unloaded_started_at)
        )
        if (
            candidate_elapsed_s <= 0.0
            and self._candidate_started_at is not None
        ):
            candidate_elapsed_s = max(
                0.0, timestamp - self._candidate_started_at
            )

        return PotentiometerReleaseDecision(
            released=bool(self.released),
            candidate_started=bool(candidate_started),
            candidate_active=bool(self._candidate_active),
            candidate_cancelled=bool(candidate_cancelled),
            current_force_n=force_n,
            last_force_n=(
                None if previous_force is None else float(previous_force)
            ),
            peak_force_n=float(self._peak_force_n),
            force_rate_n_s=float(rate),
            force_drop_n=float(force_drop),
            unloaded_elapsed_s=float(unloaded_elapsed_s),
            candidate_elapsed_s=float(candidate_elapsed_s),
            candidate_cancel_reason=candidate_cancel_reason,
            pre_release_force_n=(
                None
                if self._pre_release_force_n is None
                else float(self._pre_release_force_n)
            ),
        )


def parse_potentiometer_line(
        line: bytes | str,
        spring_constant_n_per_mm: float = 0.16,
        host_time: float | None = None,
        max_extension_mm: float = 10.4,
) -> PotentiometerForceSample | None:
    """Parse one Arduino CSV row; headers and malformed rows return ``None``."""
    if isinstance(line, bytes):
        text = line.decode(errors="ignore").strip()
    else:
        text = str(line).strip()
    if not text:
        return None

    parts = [part.strip() for part in text.split(",")]
    if len(parts) not in (5, 6):
        return None
    try:
        arduino_time_ms = int(parts[0])
        raw = int(parts[1])
        filtered_raw = float(parts[2])
        voltage_v = float(parts[3])
        compression_mm = float(parts[4])
        supply_voltage_v = float(parts[5]) if len(parts) == 6 else None
        spring_constant = float(spring_constant_n_per_mm)
        max_extension = float(max_extension_mm)
    except ValueError:
        return None

    values = (
        filtered_raw, voltage_v, compression_mm, spring_constant, max_extension,
    )
    if (
        arduino_time_ms < 0
        or not 0 <= raw <= 1023
        or not all(math.isfinite(value) for value in values)
        or not 0.0 <= compression_mm <= max_extension
        or spring_constant <= 0.0
        or max_extension <= 0.0
        or (
            supply_voltage_v is not None
            and (
                not math.isfinite(supply_voltage_v)
                or not 0.0 <= supply_voltage_v <= 6.5
            )
        )
    ):
        return None

    length_mm = max(max_extension - compression_mm, 0.0)

    return PotentiometerForceSample(
        host_time=time.time() if host_time is None else float(host_time),
        arduino_time_ms=arduino_time_ms,
        raw=raw,
        filtered_raw=filtered_raw,
        voltage_v=voltage_v,
        supply_voltage_v=supply_voltage_v,
        compression_mm=compression_mm,
        length_mm=length_mm,
        force_n=compression_mm * spring_constant,
    )


class PotentiometerForceSensor:
    """Continuously read calibrated spring compression from an Arduino UART."""

    def __init__(
            self,
            port: str = "/dev/serial0",
            baud: int = 115200,
            spring_constant_n_per_mm: float = 0.16,
            max_extension_mm: float = 10.4,
            read_timeout_s: float = 0.1,
            serial_factory: Callable[..., serial.Serial] = serial.Serial,
            info_log_interval_s: float = 0.1,
    ):
        self.port = str(port)
        self.baud = int(baud)
        self.spring_constant_n_per_mm = float(spring_constant_n_per_mm)
        self.max_extension_mm = float(max_extension_mm)
        self.read_timeout_s = float(read_timeout_s)
        self.info_log_interval_s = float(info_log_interval_s)
        if self.baud <= 0:
            raise ValueError("baud must be positive")
        if self.spring_constant_n_per_mm <= 0.0:
            raise ValueError("spring constant must be positive")
        if self.max_extension_mm <= 0.0:
            raise ValueError("maximum extension must be positive")
        if self.read_timeout_s <= 0.0:
            raise ValueError("read timeout must be positive")
        if self.info_log_interval_s <= 0.0:
            raise ValueError("info log interval must be positive")

        self._serial_factory = serial_factory
        self._serial = None
        self._latest = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._sample_event = threading.Event()
        self._thread = None
        self._reader_error = None
        self._last_info_log_monotonic = None

    def start(self, startup_timeout_s: float = 3.0) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._serial = self._serial_factory(
            self.port,
            self.baud,
            timeout=self.read_timeout_s,
        )
        self._stop_event.clear()
        self._sample_event.clear()
        self._reader_error = None
        self._last_info_log_monotonic = None
        self._thread = threading.Thread(
            target=self._read_loop,
            name="potentiometer-force-sensor",
            daemon=True,
        )
        self._thread.start()
        received_event = self._sample_event.wait(float(startup_timeout_s))
        error = self._reader_error
        if not received_event or self.latest() is None:
            self.stop()
            if error is not None:
                raise RuntimeError(
                    f"force sensor reader failed on {self.port}: {error}"
                ) from error
            raise TimeoutError(
                f"no valid force-sensor CSV received from {self.port} "
                f"within {float(startup_timeout_s):.1f}s"
            )

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, 2.0 * self.read_timeout_s))
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                logger.exception("Failed to close force-sensor serial port")
        self._thread = None
        self._serial = None

    def latest(self) -> PotentiometerForceSample | None:
        with self._lock:
            return self._latest

    def _log_sample_info(
            self,
            sample: PotentiometerForceSample,
            monotonic_time: float | None = None,
    ) -> bool:
        """Log the live force at a bounded rate; return whether it was logged."""
        now = (
            time.monotonic()
            if monotonic_time is None else float(monotonic_time)
        )
        if (
            self._last_info_log_monotonic is not None
            and now - self._last_info_log_monotonic < self.info_log_interval_s
        ):
            return False
        self._last_info_log_monotonic = now
        logger.info(
            "Potentiometer force=%.3f N, compression=%.3f mm, "
            "length=%.3f mm, raw=%d, Arduino time=%d ms, Vcc=%s",
            sample.force_n,
            sample.compression_mm,
            sample.length_mm,
            sample.raw,
            sample.arduino_time_ms,
            (
                "unavailable"
                if sample.supply_voltage_v is None
                else f"{sample.supply_voltage_v:.3f} V"
            ),
        )
        return True

    def _read_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                line = self._serial.readline()
                sample = parse_potentiometer_line(
                    line,
                    spring_constant_n_per_mm=self.spring_constant_n_per_mm,
                    max_extension_mm=self.max_extension_mm,
                )
                if sample is None:
                    continue
                with self._lock:
                    self._latest = sample
                self._sample_event.set()
                self._log_sample_info(sample)
        except Exception as error:
            self._reader_error = error
            logger.exception("Force-sensor serial reader stopped unexpectedly")
            self._sample_event.set()
