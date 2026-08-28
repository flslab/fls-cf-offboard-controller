"""Threaded Arduino potentiometer force-sensor reader.

The Arduino sketch emits CSV rows with the following columns::

    time_ms,raw,filtered,voltage,distance_mm

Distance is converted to compression force with Hooke's law.  The reader owns
the serial port and exposes the latest immutable sample so the flight-control
loop never blocks on UART I/O.
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
    force_n: float


def parse_potentiometer_line(
        line: bytes | str,
        spring_constant_n_per_mm: float = 0.16,
        host_time: float | None = None,
) -> PotentiometerForceSample | None:
    """Parse one Arduino CSV row; headers and malformed rows return ``None``."""
    if isinstance(line, bytes):
        text = line.decode(errors="ignore").strip()
    else:
        text = str(line).strip()
    if not text:
        return None

    parts = [part.strip() for part in text.split(",")]
    if len(parts) != 5:
        return None
    try:
        arduino_time_ms = int(parts[0])
        raw = int(parts[1])
        filtered_raw = float(parts[2])
        voltage_v = float(parts[3])
        distance_mm = float(parts[4])
        spring_constant = float(spring_constant_n_per_mm)
    except ValueError:
        return None

    values = (filtered_raw, voltage_v, distance_mm, spring_constant)
    if (
        arduino_time_ms < 0
        or not 0 <= raw <= 1023
        or not all(math.isfinite(value) for value in values)
        or distance_mm < 0.0
        or spring_constant <= 0.0
    ):
        return None

    return PotentiometerForceSample(
        host_time=time.time() if host_time is None else float(host_time),
        arduino_time_ms=arduino_time_ms,
        raw=raw,
        filtered_raw=filtered_raw,
        voltage_v=voltage_v,
        distance_mm=distance_mm,
        force_n=distance_mm * spring_constant,
    )


class PotentiometerForceSensor:
    """Continuously read calibrated spring compression from an Arduino UART."""

    def __init__(
            self,
            port: str = "/dev/serial0",
            baud: int = 115200,
            spring_constant_n_per_mm: float = 0.16,
            read_timeout_s: float = 0.1,
            serial_factory: Callable[..., serial.Serial] = serial.Serial,
    ):
        self.port = str(port)
        self.baud = int(baud)
        self.spring_constant_n_per_mm = float(spring_constant_n_per_mm)
        self.read_timeout_s = float(read_timeout_s)
        if self.baud <= 0:
            raise ValueError("baud must be positive")
        if self.spring_constant_n_per_mm <= 0.0:
            raise ValueError("spring constant must be positive")
        if self.read_timeout_s <= 0.0:
            raise ValueError("read timeout must be positive")

        self._serial_factory = serial_factory
        self._serial = None
        self._latest = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._sample_event = threading.Event()
        self._thread = None
        self._reader_error = None

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

    def _read_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                line = self._serial.readline()
                sample = parse_potentiometer_line(
                    line,
                    self.spring_constant_n_per_mm,
                )
                if sample is None:
                    continue
                with self._lock:
                    self._latest = sample
                self._sample_event.set()
        except Exception as error:
            self._reader_error = error
            logger.exception("Force-sensor serial reader stopped unexpectedly")
            self._sample_event.set()
