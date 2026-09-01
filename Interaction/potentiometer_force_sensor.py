"""Threaded Arduino potentiometer force-sensor reader.

The Arduino sketch emits CSV rows with the following columns::

    time_ms,raw,filtered,voltage,distance_mm[,supply_voltage]

The reported distance is the spring's current extension.  It is converted to
compression with ``max_extension_mm - distance_mm`` and then to force with
Hooke's law.  The reader owns the serial port and exposes the latest immutable
sample so the flight-control loop never blocks on UART I/O.
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
    distance_mm: float
    supply_voltage_v: float | None
    compression_mm: float
    force_n: float


@dataclass(frozen=True)
class PotentiometerReleaseDecision:
    released: bool
    current_force_n: float
    last_force_n: float | None
    peak_force_n: float
    force_rate_n_s: float
    force_drop_n: float


class PotentiometerReleaseDetector:
    """One-shot release detector armed by an estimator contact onset."""

    def __init__(self, force_drop_n=0.01, decrease_rate_n_s=0.05):
        self.force_drop_n = float(force_drop_n)
        self.decrease_rate_n_s = float(decrease_rate_n_s)
        if (
            not math.isfinite(self.force_drop_n)
            or not math.isfinite(self.decrease_rate_n_s)
            or self.force_drop_n <= 0.0
            or self.decrease_rate_n_s <= 0.0
        ):
            raise ValueError('potentiometer release thresholds must be positive')
        self.disarm()

    def disarm(self):
        self.armed = False
        self.released = False
        self._last_timestamp = None
        self._last_force_n = None
        self._peak_force_n = 0.0

    def arm(self, force_n, timestamp):
        force_n = max(float(force_n), 0.0)
        timestamp = float(timestamp)
        if not math.isfinite(force_n) or not math.isfinite(timestamp):
            raise ValueError('potentiometer release arm values must be finite')
        self.armed = True
        self.released = False
        self._last_timestamp = timestamp
        self._last_force_n = force_n
        self._peak_force_n = force_n

    def update(self, force_n, timestamp):
        force_n = max(float(force_n), 0.0)
        timestamp = float(timestamp)
        if not math.isfinite(force_n) or not math.isfinite(timestamp):
            raise ValueError('potentiometer release values must be finite')
        previous_force = self._last_force_n
        rate = 0.0
        if (
            self.armed
            and not self.released
            and self._last_timestamp is not None
            and previous_force is not None
            and timestamp > self._last_timestamp
        ):
            rate = (force_n - previous_force) / (
                timestamp - self._last_timestamp
            )
            self._peak_force_n = max(self._peak_force_n, force_n)
            force_drop = self._peak_force_n - force_n
            self.released = bool(
                force_drop >= self.force_drop_n
                and rate <= -self.decrease_rate_n_s
            )
            self._last_timestamp = timestamp
            self._last_force_n = force_n
        else:
            force_drop = self._peak_force_n - force_n

        return PotentiometerReleaseDecision(
            released=bool(self.released),
            current_force_n=force_n,
            last_force_n=(
                None if previous_force is None else float(previous_force)
            ),
            peak_force_n=float(self._peak_force_n),
            force_rate_n_s=float(rate),
            force_drop_n=float(force_drop),
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
        distance_mm = float(parts[4])
        supply_voltage_v = float(parts[5]) if len(parts) == 6 else None
        spring_constant = float(spring_constant_n_per_mm)
        max_extension = float(max_extension_mm)
    except ValueError:
        return None

    values = (
        filtered_raw, voltage_v, distance_mm, spring_constant, max_extension,
    )
    if (
        arduino_time_ms < 0
        or not 0 <= raw <= 1023
        or not all(math.isfinite(value) for value in values)
        or distance_mm < 0.0
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

    compression_mm = max(max_extension - distance_mm, 0.0)

    return PotentiometerForceSample(
        host_time=time.time() if host_time is None else float(host_time),
        arduino_time_ms=arduino_time_ms,
        raw=raw,
        filtered_raw=filtered_raw,
        voltage_v=voltage_v,
        distance_mm=distance_mm,
        supply_voltage_v=supply_voltage_v,
        compression_mm=compression_mm,
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
            "distance=%.3f mm, raw=%d, Arduino time=%d ms, Vcc=%s",
            sample.force_n,
            sample.compression_mm,
            sample.distance_mm,
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
