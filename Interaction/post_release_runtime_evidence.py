"""Validated runtime evidence used by post-release estimator handoff.

The helpers in this module only translate already-calibrated clock evidence.
They do not estimate a mapping from callback arrival times and they never
grant command authority.
"""

from __future__ import annotations

from dataclasses import dataclass
from bisect import bisect_left
import math
import numbers
from typing import Mapping

from Interaction.contact_attitude_experiment import (
    ARDUINO_TO_CF_RELEASE_CLOCK_MAPPING_BASIS,
)
from Interaction.contact_attitude_observer import CF_TIMESTAMP_MODULUS_MS


ARDUINO_TIMESTAMP_MODULUS_MS = 1 << 32


def _nonnegative_integer(value, name):
    if (
        isinstance(value, bool)
        or not isinstance(value, numbers.Integral)
        or int(value) < 0
    ):
        raise ValueError(f"{name} must be a nonnegative integer")
    return int(value)


def _finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ValueError(f"{name} must be finite")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _nonempty_string(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")
    return value.strip()


def _signed_modular_delta(value, reference, modulus):
    delta = (int(value) - int(reference)) % int(modulus)
    if delta >= int(modulus) // 2:
        delta -= int(modulus)
    return int(delta)


@dataclass(frozen=True)
class MappedReleaseEpoch:
    """One physical sensor epoch expressed on the Crazyflie clock."""

    cf_timestamp_ms: int
    unwrapped_cf_timestamp_ms: int
    mapping_basis: str
    mapping_uncertainty_ms: float
    mapping_calibration_id: str
    arduino_delta_from_reference_ms: int
    drift_uncertainty_ms: float
    quantization_uncertainty_ms: float

    def interval_unwrapped_cf_ms(self):
        """Conservative source-time interval, never a fabricated exact epoch."""
        center = float(self.unwrapped_cf_timestamp_ms)
        radius = float(self.mapping_uncertainty_ms)
        if not math.isfinite(radius) or radius < 0.0:
            raise ValueError("release clock uncertainty is invalid")
        return center - radius, center + radius

    def possible_first_free_imu_epochs(
            self, imu_unwrapped_timestamps_ms, *, max_imu_gap_ms):
        """Bound which sampled IMU epoch could first follow release.

        This is diagnostic evidence, not a reconstructed ESKF state or a
        command-eligibility certificate. Missing coverage and timestamp gaps
        fail closed; callers must not select the midpoint candidate.
        """
        raw_timestamps = tuple(imu_unwrapped_timestamps_ms)
        if (
            not raw_timestamps
            or any(isinstance(value, bool)
                   or not isinstance(value, numbers.Integral)
                   or value < 0 for value in raw_timestamps)
        ):
            raise ValueError("IMU epochs must be nonnegative integers")
        timestamps = tuple(int(value) for value in raw_timestamps)
        if (
            not timestamps
            or any(later <= earlier for earlier, later in zip(
                timestamps, timestamps[1:]))
        ):
            raise ValueError("IMU epochs must be strictly increasing integers")
        gap_limit = _finite_number(max_imu_gap_ms, "max_imu_gap_ms")
        if gap_limit <= 0.0:
            raise ValueError("max_imu_gap_ms must be positive")
        lower, upper = self.interval_unwrapped_cf_ms()
        if lower < timestamps[0] or upper > timestamps[-1]:
            raise ValueError("release interval lacks IMU coverage")
        first_index = bisect_left(timestamps, lower)
        last_index = bisect_left(timestamps, upper)
        if last_index >= len(timestamps):
            raise ValueError("release interval lacks next IMU epoch")
        if any(timestamps[index + 1] - timestamps[index] > gap_limit
               for index in range(max(first_index - 1, 0), last_index)):
            raise ValueError("release interval crosses an IMU gap")
        return timestamps[first_index:last_index + 1]

    def release_kwargs(self):
        return {
            "release_event_cf_timestamp_ms": self.cf_timestamp_ms,
            "release_event_unwrapped_cf_timestamp_ms": (
                self.unwrapped_cf_timestamp_ms
            ),
            "release_clock_mapping_basis": self.mapping_basis,
            "release_clock_mapping_uncertainty_ms": (
                self.mapping_uncertainty_ms
            ),
            "release_clock_mapping_calibration_id": (
                self.mapping_calibration_id
            ),
        }


@dataclass(frozen=True)
class ArduinoToCfClockMapping:
    """Bounded affine Arduino-millis to unwrapped Crazyflie-millis map.

    A calibration supplies an anchor on both clocks, an estimated scale, a
    fixed anchor/residual uncertainty, a scale-error bound, and the maximum
    distance over which that calibration was validated.  Mapping outside that
    interval fails closed.  Scale extrapolation and rounding to a millisecond
    epoch are both added to the returned uncertainty.
    """

    calibration_id: str
    uncertainty_ms: float
    arduino_reference_timestamp_ms: int
    cf_reference_timestamp_ms: int
    cf_reference_unwrapped_timestamp_ms: int
    cf_ms_per_arduino_ms: float
    scale_error_ppm: float
    max_abs_delta_ms: int
    basis: str = ARDUINO_TO_CF_RELEASE_CLOCK_MAPPING_BASIS

    def __post_init__(self):
        calibration_id = _nonempty_string(
            self.calibration_id, "release clock calibration_id"
        )
        if self.basis != ARDUINO_TO_CF_RELEASE_CLOCK_MAPPING_BASIS:
            raise ValueError("release clock mapping basis is not trusted")
        uncertainty = _finite_number(
            self.uncertainty_ms, "release clock uncertainty_ms"
        )
        if uncertainty < 0.0:
            raise ValueError(
                "release clock uncertainty_ms must be nonnegative"
            )
        arduino_reference = _nonnegative_integer(
            self.arduino_reference_timestamp_ms,
            "release clock arduino_reference_timestamp_ms",
        )
        if arduino_reference >= ARDUINO_TIMESTAMP_MODULUS_MS:
            raise ValueError(
                "release clock Arduino reference is outside uint32"
            )
        cf_reference = _nonnegative_integer(
            self.cf_reference_timestamp_ms,
            "release clock cf_reference_timestamp_ms",
        )
        if cf_reference >= CF_TIMESTAMP_MODULUS_MS:
            raise ValueError(
                "release clock Crazyflie reference is outside uint24"
            )
        cf_reference_unwrapped = _nonnegative_integer(
            self.cf_reference_unwrapped_timestamp_ms,
            "release clock cf_reference_unwrapped_timestamp_ms",
        )
        if cf_reference_unwrapped % CF_TIMESTAMP_MODULUS_MS != cf_reference:
            raise ValueError(
                "release clock Crazyflie raw/unwrapped references disagree"
            )
        scale = _finite_number(
            self.cf_ms_per_arduino_ms,
            "release clock cf_ms_per_arduino_ms",
        )
        # Both sources are millisecond clocks.  A wider ratio is almost
        # certainly a unit/configuration error, not oscillator drift.
        if not 0.9 <= scale <= 1.1:
            raise ValueError(
                "release clock scale must be within 10 percent of unity"
            )
        scale_error_ppm = _finite_number(
            self.scale_error_ppm, "release clock scale_error_ppm"
        )
        if scale_error_ppm < 0.0 or scale_error_ppm > 100000.0:
            raise ValueError(
                "release clock scale_error_ppm must be in [0, 100000]"
            )
        max_delta = _nonnegative_integer(
            self.max_abs_delta_ms, "release clock max_abs_delta_ms"
        )
        if max_delta < 1 or max_delta >= ARDUINO_TIMESTAMP_MODULUS_MS // 2:
            raise ValueError(
                "release clock max_abs_delta_ms must be in [1, 2^31)"
            )
        object.__setattr__(self, "calibration_id", calibration_id)
        object.__setattr__(self, "uncertainty_ms", uncertainty)
        object.__setattr__(
            self, "arduino_reference_timestamp_ms", arduino_reference
        )
        object.__setattr__(self, "cf_reference_timestamp_ms", cf_reference)
        object.__setattr__(
            self,
            "cf_reference_unwrapped_timestamp_ms",
            cf_reference_unwrapped,
        )
        object.__setattr__(self, "cf_ms_per_arduino_ms", scale)
        object.__setattr__(self, "scale_error_ppm", scale_error_ppm)
        object.__setattr__(self, "max_abs_delta_ms", max_delta)

    @classmethod
    def from_mapping(cls, value):
        if value is None:
            return None
        if not isinstance(value, Mapping):
            raise ValueError(
                "post_release_estimator_control.release_clock_mapping must "
                "be a mapping"
            )
        allowed = {
            "basis",
            "calibration_id",
            "uncertainty_ms",
            "arduino_reference_timestamp_ms",
            "cf_reference_timestamp_ms",
            "cf_reference_unwrapped_timestamp_ms",
            "cf_ms_per_arduino_ms",
            "scale_error_ppm",
            "max_abs_delta_ms",
        }
        unknown = sorted(set(value) - allowed)
        if unknown:
            raise ValueError(
                "unknown release clock mapping fields: "
                + ", ".join(unknown)
            )
        missing = sorted(allowed - set(value))
        if missing:
            raise ValueError(
                "missing release clock mapping fields: "
                + ", ".join(missing)
            )
        return cls(**{key: value[key] for key in allowed})

    def map_arduino_timestamp(self, arduino_timestamp_ms):
        timestamp = _nonnegative_integer(
            arduino_timestamp_ms, "release Arduino timestamp"
        )
        if timestamp >= ARDUINO_TIMESTAMP_MODULUS_MS:
            raise ValueError("release Arduino timestamp is outside uint32")
        delta = _signed_modular_delta(
            timestamp,
            self.arduino_reference_timestamp_ms,
            ARDUINO_TIMESTAMP_MODULUS_MS,
        )
        if abs(delta) > self.max_abs_delta_ms:
            raise ValueError(
                "release Arduino timestamp is outside calibrated interval"
            )
        mapped_float = (
            self.cf_reference_unwrapped_timestamp_ms
            + self.cf_ms_per_arduino_ms * delta
        )
        mapped = int(round(mapped_float))
        if mapped < 0:
            raise ValueError(
                "mapped release Crazyflie timestamp would be negative"
            )
        quantization = abs(mapped_float - mapped)
        drift_bound = abs(delta) * self.scale_error_ppm * 1e-6
        return MappedReleaseEpoch(
            cf_timestamp_ms=mapped % CF_TIMESTAMP_MODULUS_MS,
            unwrapped_cf_timestamp_ms=mapped,
            mapping_basis=self.basis,
            mapping_uncertainty_ms=(
                self.uncertainty_ms + drift_bound + quantization
            ),
            mapping_calibration_id=self.calibration_id,
            arduino_delta_from_reference_ms=delta,
            drift_uncertainty_ms=drift_bound,
            quantization_uncertainty_ms=quantization,
        )

    def supports_exact_release_epoch(self):
        """Return false for every independent Arduino-to-CF clock fit.

        Even a YAML claim of unit scale and zero residual/drift uncertainty is
        not physical proof that two independent clocks identify the same sensor
        epoch.  These mappings remain useful for shadow diagnostics, but the
        production control preflight stays unconditionally blocked until a new
        firmware/shared-clock release-latch basis is implemented and verified.
        """
        return False
