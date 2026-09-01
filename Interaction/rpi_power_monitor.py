"""Non-blocking Raspberry Pi undervoltage/throttling telemetry."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import subprocess
import threading
import time
from typing import Callable


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class RaspberryPiPowerSample:
    host_time: float
    flags: int

    @property
    def under_voltage_now(self) -> bool:
        return bool(self.flags & (1 << 0))

    @property
    def frequency_capped_now(self) -> bool:
        return bool(self.flags & (1 << 1))

    @property
    def throttled_now(self) -> bool:
        return bool(self.flags & (1 << 2))

    @property
    def soft_temperature_limit_now(self) -> bool:
        return bool(self.flags & (1 << 3))

    @property
    def under_voltage_occurred(self) -> bool:
        return bool(self.flags & (1 << 16))

    @property
    def frequency_capped_occurred(self) -> bool:
        return bool(self.flags & (1 << 17))

    @property
    def throttled_occurred(self) -> bool:
        return bool(self.flags & (1 << 18))

    @property
    def soft_temperature_limit_occurred(self) -> bool:
        return bool(self.flags & (1 << 19))


def parse_get_throttled_output(
        output: bytes | str,
        host_time: float | None = None,
) -> RaspberryPiPowerSample:
    """Parse ``vcgencmd get_throttled`` output such as ``throttled=0x0``."""
    if isinstance(output, bytes):
        text = output.decode(errors="ignore").strip()
    else:
        text = str(output).strip()
    prefix = "throttled="
    if not text.startswith(prefix):
        raise ValueError(f"unexpected get_throttled output: {text!r}")
    flags = int(text[len(prefix):], 0)
    if flags < 0:
        raise ValueError("get_throttled flags must be non-negative")
    return RaspberryPiPowerSample(
        host_time=time.time() if host_time is None else float(host_time),
        flags=flags,
    )


class RaspberryPiPowerMonitor:
    """Poll ``vcgencmd`` in a daemon thread without blocking flight control."""

    def __init__(
            self,
            poll_interval_s: float = 0.5,
            command_timeout_s: float = 0.5,
            command_runner: Callable[..., subprocess.CompletedProcess] = (
                subprocess.run
            ),
    ):
        self.poll_interval_s = float(poll_interval_s)
        self.command_timeout_s = float(command_timeout_s)
        if self.poll_interval_s <= 0.0 or self.command_timeout_s <= 0.0:
            raise ValueError("power-monitor timing values must be positive")
        self._command_runner = command_runner
        self._latest = None
        self._last_error = None
        self._last_warned_flags = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._poll_once()
        self._thread = threading.Thread(
            target=self._poll_loop,
            name="rpi-power-monitor",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, 2.0 * self.poll_interval_s))
        self._thread = None

    def latest(self) -> RaspberryPiPowerSample | None:
        with self._lock:
            return self._latest

    @property
    def last_error(self) -> Exception | None:
        with self._lock:
            return self._last_error

    def _poll_once(self) -> None:
        try:
            result = self._command_runner(
                ["vcgencmd", "get_throttled"],
                capture_output=True,
                text=True,
                check=True,
                timeout=self.command_timeout_s,
            )
            sample = parse_get_throttled_output(result.stdout)
            with self._lock:
                self._latest = sample
                self._last_error = None
            if sample.flags and sample.flags != self._last_warned_flags:
                logger.warning(
                    "Raspberry Pi power flags changed to 0x%x",
                    sample.flags,
                )
                self._last_warned_flags = sample.flags
        except Exception as error:
            with self._lock:
                previous_error = self._last_error
                self._last_error = error
            if previous_error is None:
                logger.warning(
                    "Unable to read Raspberry Pi power flags: %s", error
                )

    def _poll_loop(self) -> None:
        while not self._stop_event.wait(self.poll_interval_s):
            self._poll_once()
