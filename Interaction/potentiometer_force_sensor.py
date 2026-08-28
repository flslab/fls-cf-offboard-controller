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

import numpy as np
import serial

from Interaction.wrench_contact_detector import ContactDecision


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


class SpringForceTrendDetector:
    """Detect touch from rising spring force and release from falling force."""

    def __init__(
            self,
            contact_force_n: float = 0.02,
            rise_rate_n_s: float = 0.05,
            release_rate_n_s: float = 0.05,
            release_drop_n: float = 0.01,
            onset_time_s: float = 0.02,
            release_time_s: float = 0.04,
            cancel_rise_n: float = 0.01,
    ):
        self.contact_force_n = float(contact_force_n)
        self.rise_rate_n_s = float(rise_rate_n_s)
        self.release_rate_n_s = float(release_rate_n_s)
        self.release_drop_n = float(release_drop_n)
        self.onset_time_s = float(onset_time_s)
        self.release_time_s = float(release_time_s)
        self.cancel_rise_n = float(cancel_rise_n)
        values = (
            self.contact_force_n,
            self.rise_rate_n_s,
            self.release_rate_n_s,
            self.release_drop_n,
            self.onset_time_s,
            self.release_time_s,
            self.cancel_rise_n,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("spring-force detector settings must be positive")
        self.reset()

    def reset(self, timestamp: float | None = None) -> None:
        self.active = False
        self._last_timestamp = None if timestamp is None else float(timestamp)
        self._last_force_n = None
        self._onset_elapsed_s = 0.0
        self._release_elapsed_s = 0.0
        self._release_candidate = False
        self._peak_force_n = 0.0
        self._candidate_min_force_n = 0.0
        self._direction = np.zeros(3)
        self._rearm_blocked = False
        self.last_rate_n_s = 0.0

    def _decision(
            self,
            force_n,
            started=False,
            ended=False,
            candidate_started=False,
            candidate_cancelled=False,
    ) -> ContactDecision:
        return ContactDecision(
            active=self.active,
            started=bool(started),
            ended=bool(ended),
            magnitude=float(force_n),
            normalized_magnitude=float(force_n / self.contact_force_n),
            confidence_sigma=0.0,
            evidence=float(self._onset_elapsed_s),
            release_projected_value=float(force_n),
            release_projection_normalized=float(
                force_n / self.contact_force_n
            ),
            release_direction=tuple(float(value) for value in self._direction),
            release_direction_source="potentiometer_force_axis",
            release_candidate_active=self._release_candidate,
            release_candidate_started=bool(candidate_started),
            release_candidate_cancelled=bool(candidate_cancelled),
            release_elapsed_s=float(self._release_elapsed_s),
        )

    def update(
            self,
            force_n: float,
            timestamp: float,
            force_direction,
    ) -> ContactDecision:
        force_n = max(float(force_n), 0.0)
        timestamp = float(timestamp)
        direction = np.asarray(force_direction, dtype=float)
        if (
            not math.isfinite(force_n)
            or not math.isfinite(timestamp)
            or direction.shape != (3,)
            or not np.all(np.isfinite(direction))
        ):
            raise ValueError("spring force update must be finite")
        direction_norm = float(np.linalg.norm(direction))
        if direction_norm > 1e-9:
            direction = direction / direction_norm

        if self._rearm_blocked:
            return self._decision(force_n)

        if self._last_timestamp is None or self._last_force_n is None:
            self._last_timestamp = timestamp
            self._last_force_n = force_n
            self._peak_force_n = force_n
            self._direction = direction
            return self._decision(force_n)

        raw_dt = timestamp - self._last_timestamp
        if raw_dt <= 0.0:
            return self._decision(force_n)
        dt = min(raw_dt, 0.1)
        rate_n_s = (force_n - self._last_force_n) / raw_dt
        self.last_rate_n_s = rate_n_s
        self._last_timestamp = timestamp
        self._last_force_n = force_n

        started = False
        ended = False
        candidate_started = False
        candidate_cancelled = False

        if not self.active:
            rising = (
                force_n >= self.contact_force_n
                and rate_n_s >= self.rise_rate_n_s
            )
            self._onset_elapsed_s = (
                self._onset_elapsed_s + dt if rising else 0.0
            )
            if self._onset_elapsed_s >= self.onset_time_s:
                self.active = True
                started = True
                self._peak_force_n = force_n
                self._direction = direction
                self._onset_elapsed_s = 0.0
        else:
            self._peak_force_n = max(self._peak_force_n, force_n)
            force_drop_n = self._peak_force_n - force_n
            falling = rate_n_s <= -self.release_rate_n_s
            if not self._release_candidate:
                if falling and force_drop_n >= self.release_drop_n:
                    self._release_candidate = True
                    candidate_started = True
                    self._release_elapsed_s = 0.0
                    self._candidate_min_force_n = force_n
            else:
                self._candidate_min_force_n = min(
                    self._candidate_min_force_n, force_n
                )
                recovered = (
                    rate_n_s >= self.rise_rate_n_s
                    and force_n
                    >= self._candidate_min_force_n + self.cancel_rise_n
                )
                if recovered:
                    self._release_candidate = False
                    candidate_cancelled = True
                    self._release_elapsed_s = 0.0
                    self._peak_force_n = force_n
                else:
                    self._release_elapsed_s += dt
                    if self._release_elapsed_s >= self.release_time_s:
                        self.active = False
                        ended = True
                        self._rearm_blocked = True
                        self._release_candidate = False
                        self._release_elapsed_s = 0.0
                        self._onset_elapsed_s = 0.0

        return self._decision(
            force_n,
            started=started,
            ended=ended,
            candidate_started=candidate_started,
            candidate_cancelled=candidate_cancelled,
        )
