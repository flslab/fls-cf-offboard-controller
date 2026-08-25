"""Uncertainty-aware contact decisions from estimated external wrench."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class ContactDecision:
    active: bool
    started: bool
    ended: bool
    magnitude: float
    normalized_magnitude: float
    confidence_sigma: float
    evidence: float


class ContactChannelDetector:
    """CUSUM-like contact detector with physical and covariance thresholds."""

    def __init__(
            self,
            component_thresholds: Sequence[float],
            covariance_floor: Sequence[float],
            confidence_sigma: float = 2.5,
            onset_evidence_s: float = 0.04,
            release_time_s: float = 0.12,
            release_ratio: float = 0.55,
            evidence_leak: float = 1.0,
            enabled: bool = True,
    ):
        self.thresholds = np.asarray(component_thresholds, dtype=float)
        self.covariance_floor = np.asarray(covariance_floor, dtype=float)
        if self.thresholds.ndim != 1 or self.thresholds.shape != self.covariance_floor.shape:
            raise ValueError("thresholds and covariance_floor must have equal one-dimensional shape")
        if np.any(self.thresholds <= 0) or np.any(self.covariance_floor <= 0):
            raise ValueError("thresholds and covariance_floor must be positive")
        self.required_sigma = float(confidence_sigma)
        self.onset_evidence_s = float(onset_evidence_s)
        self.release_time_s = float(release_time_s)
        self.release_ratio = float(release_ratio)
        self.evidence_leak = float(evidence_leak)
        self.enabled = bool(enabled)
        self.active = False
        self.evidence = 0.0
        self._release_elapsed = 0.0
        self._last_timestamp: float | None = None

    def reset(self, timestamp: float | None = None) -> None:
        """Clear contact evidence after a controller-mode handoff."""
        self.active = False
        self.evidence = 0.0
        self._release_elapsed = 0.0
        self._last_timestamp = None if timestamp is None else float(timestamp)

    def update(self, value: Sequence[float], covariance: np.ndarray, timestamp: float) -> ContactDecision:
        value = np.asarray(value, dtype=float)
        covariance = np.asarray(covariance, dtype=float)
        if value.shape != self.thresholds.shape or covariance.shape != (len(value), len(value)):
            raise ValueError("contact value/covariance dimensions do not match detector")
        timestamp = float(timestamp)
        dt = 0.0 if self._last_timestamp is None else min(max(timestamp - self._last_timestamp, 0.0), 0.1)
        self._last_timestamp = timestamp

        if not self.enabled:
            self.active = False
            self.evidence = 0.0
            self._release_elapsed = 0.0
            return ContactDecision(
                active=False,
                started=False,
                ended=False,
                magnitude=float(np.linalg.norm(value)),
                normalized_magnitude=0.0,
                confidence_sigma=0.0,
                evidence=0.0,
            )

        normalized = float(np.linalg.norm(value / self.thresholds))
        effective_covariance = covariance + np.diag(np.square(self.covariance_floor))
        confidence = math.sqrt(max(0.0, float(value.T @ np.linalg.pinv(effective_covariance) @ value)))
        significant = normalized >= 1.0 and confidence >= self.required_sigma
        started = False
        ended = False

        if not self.active:
            if significant:
                strength = min(normalized, confidence / max(self.required_sigma, 1e-6))
                self.evidence += dt * max(strength - self.evidence_leak, 0.25)
            else:
                self.evidence = max(0.0, self.evidence - dt * self.evidence_leak)
            if self.evidence >= self.onset_evidence_s:
                self.active = True
                started = True
                self._release_elapsed = 0.0
        else:
            if normalized <= self.release_ratio:
                self._release_elapsed += dt
                if self._release_elapsed >= self.release_time_s:
                    self.active = False
                    ended = True
                    self.evidence = 0.0
                    self._release_elapsed = 0.0
            else:
                self._release_elapsed = 0.0

        return ContactDecision(
            active=self.active,
            started=started,
            ended=ended,
            magnitude=float(np.linalg.norm(value)),
            normalized_magnitude=normalized,
            confidence_sigma=confidence,
            evidence=self.evidence,
        )


@dataclass(frozen=True)
class WrenchContactState:
    translation: ContactDecision
    yaw: ContactDecision


class WrenchContactDetector:
    """Separate translational, yaw, and diagnostic roll/pitch channels."""

    def __init__(self, translation: dict, yaw: dict):
        self.translation = ContactChannelDetector(**translation)
        self.yaw = ContactChannelDetector(**yaw)

    def update(self, estimate, timestamp: float | None = None) -> WrenchContactState:
        timestamp = estimate.timestamp if timestamp is None else timestamp
        return WrenchContactState(
            translation=self.translation.update(
                estimate.external_force, estimate.force_covariance, timestamp
            ),
            yaw=self.yaw.update(
                estimate.external_torque[2:3], estimate.torque_covariance[2:3, 2:3], timestamp
            ),
        )
