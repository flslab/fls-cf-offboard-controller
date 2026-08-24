"""Bounded 3D translation and yaw admittance reference generation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


def _vec3(value, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.ndim == 0:
        array = np.full(3, float(array))
    if array.shape != (3,):
        raise ValueError(f"{name} must contain three values")
    return array


def _clip_components(value: np.ndarray, limit: np.ndarray) -> np.ndarray:
    return np.clip(value, -np.abs(limit), np.abs(limit))


@dataclass(frozen=True)
class AdmittanceState:
    translation_offset: np.ndarray
    translation_velocity: np.ndarray
    translation_acceleration: np.ndarray
    yaw_offset: float
    yaw_rate: float
    yaw_acceleration: float


class AdmittanceController3DYaw:
    """Virtual mass/damper/spring with independent hard safety limits."""

    def __init__(
            self,
            translation_mass: Sequence[float],
            translation_damping: Sequence[float],
            translation_stiffness: Sequence[float],
            max_offset: Sequence[float],
            max_velocity: Sequence[float],
            max_acceleration: Sequence[float],
            yaw_inertia: float,
            yaw_damping: float,
            yaw_stiffness: float,
            max_yaw_offset: float,
            max_yaw_rate: float,
            max_yaw_acceleration: float,
    ):
        self.mass = _vec3(translation_mass, "translation_mass")
        self.damping = _vec3(translation_damping, "translation_damping")
        self.stiffness = _vec3(translation_stiffness, "translation_stiffness")
        self.max_offset = _vec3(max_offset, "max_offset")
        self.max_velocity = _vec3(max_velocity, "max_velocity")
        self.max_acceleration = _vec3(max_acceleration, "max_acceleration")
        if np.any(self.mass <= 0):
            raise ValueError("translation mass must be positive")
        self.yaw_inertia = float(yaw_inertia)
        self.yaw_damping = float(yaw_damping)
        self.yaw_stiffness = float(yaw_stiffness)
        self.max_yaw_offset = abs(float(max_yaw_offset))
        self.max_yaw_rate = abs(float(max_yaw_rate))
        self.max_yaw_acceleration = abs(float(max_yaw_acceleration))
        if self.yaw_inertia <= 0:
            raise ValueError("yaw inertia must be positive")
        self.offset = np.zeros(3)
        self.velocity = np.zeros(3)
        self.yaw_offset = 0.0
        self.yaw_rate = 0.0

    def step(
            self,
            external_force: Sequence[float],
            external_yaw_torque: float,
            dt: float,
    ) -> AdmittanceState:
        force = _vec3(external_force, "external_force")
        dt = min(max(float(dt), 1e-4), 0.05)
        acceleration = (force - self.damping * self.velocity - self.stiffness * self.offset) / self.mass
        acceleration = _clip_components(acceleration, self.max_acceleration)
        self.velocity = _clip_components(self.velocity + acceleration * dt, self.max_velocity)
        proposed_offset = self.offset + self.velocity * dt
        clipped_offset = _clip_components(proposed_offset, self.max_offset)
        saturated = proposed_offset != clipped_offset
        self.velocity[saturated & (np.sign(self.velocity) == np.sign(proposed_offset))] = 0.0
        self.offset = clipped_offset

        yaw_acceleration = (
            float(external_yaw_torque)
            - self.yaw_damping * self.yaw_rate
            - self.yaw_stiffness * self.yaw_offset
        ) / self.yaw_inertia
        yaw_acceleration = float(np.clip(
            yaw_acceleration, -self.max_yaw_acceleration, self.max_yaw_acceleration
        ))
        self.yaw_rate = float(np.clip(
            self.yaw_rate + yaw_acceleration * dt, -self.max_yaw_rate, self.max_yaw_rate
        ))
        proposed_yaw = self.yaw_offset + self.yaw_rate * dt
        self.yaw_offset = float(np.clip(proposed_yaw, -self.max_yaw_offset, self.max_yaw_offset))
        if proposed_yaw != self.yaw_offset and np.sign(self.yaw_rate) == np.sign(proposed_yaw):
            self.yaw_rate = 0.0

        return AdmittanceState(
            translation_offset=self.offset.copy(),
            translation_velocity=self.velocity.copy(),
            translation_acceleration=acceleration.copy(),
            yaw_offset=self.yaw_offset,
            yaw_rate=self.yaw_rate,
            yaw_acceleration=yaw_acceleration,
        )

    def reset(self) -> None:
        self.offset.fill(0.0)
        self.velocity.fill(0.0)
        self.yaw_offset = 0.0
        self.yaw_rate = 0.0
