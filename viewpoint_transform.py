"""Planar camera-viewpoint correction for authored illumination missions.

SFL positions are authored relative to a design camera.  The recording camera
localizer reports how the real camera differs from that design pose.  For the
lightweight correction used by the flight controller we keep Z as a
translation and use only the camera's world yaw around Z.

Angles passed to this module, including SFL mission target yaw, are radians.
Authored yaw is deliberately allowed to be unwrapped so multi-turn
trajectories retain their intended interpolation direction.
"""

from __future__ import annotations

import math
from typing import Iterable, Sequence


def wrap_radians(angle: float) -> float:
    """Wrap *angle* to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def transform_yaw_rad(authored_yaw_rad: float, camera_yaw_offset_rad: float) -> float:
    """Apply camera heading correction without wrapping authored trajectories."""
    return float(authored_yaw_rad) + float(camera_yaw_offset_rad)


def rotate_xy(vector: Sequence[float], yaw_rad: float) -> list[float]:
    """Rotate a three-vector about world +Z without changing its Z value."""
    if len(vector) < 3:
        raise ValueError("a 3-D vector is required")
    c = math.cos(float(yaw_rad))
    s = math.sin(float(yaw_rad))
    x, y, z = (float(vector[0]), float(vector[1]), float(vector[2]))
    return [c * x - s * y, s * x + c * y, z]


def transform_position(
    position: Sequence[float],
    design_camera_position: Sequence[float],
    camera_position_offset: Sequence[float],
    camera_yaw_offset_rad: float,
    light_module_offset: Sequence[float] = (0.0, 0.0, 0.0),
    *,
    light_module_yaw_rad: float | None = None,
) -> list[float]:
    """Map one authored point into the actual grid/world frame.

    ``camera_position_offset`` is ``actual_camera - design_camera``.  The
    point is rotated about the design camera, translated with the camera, and
    shifted from the light module to the drone marker.  A light-module offset
    is body-fixed, so callers with a target attitude should pass the drone's
    final world yaw in ``light_module_yaw_rad``.  Omitting it retains the
    zero-authored-yaw affine behavior for compatibility.
    """
    if any(len(value) < 3 for value in (
        position,
        design_camera_position,
        camera_position_offset,
        light_module_offset,
    )):
        raise ValueError("position and offsets must contain x, y, and z")

    relative = [
        float(position[index]) - float(design_camera_position[index])
        for index in range(3)
    ]
    rotated_relative = rotate_xy(relative, camera_yaw_offset_rad)
    module_yaw = (
        camera_yaw_offset_rad
        if light_module_yaw_rad is None
        else light_module_yaw_rad
    )
    rotated_module = rotate_xy(light_module_offset, module_yaw)
    return [
        float(design_camera_position[index])
        + float(camera_position_offset[index])
        + rotated_relative[index]
        - rotated_module[index]
        for index in range(3)
    ]


def affine_translation(
    design_camera_position: Sequence[float],
    camera_position_offset: Sequence[float],
    camera_yaw_offset_rad: float,
    light_module_offset: Sequence[float] = (0.0, 0.0, 0.0),
) -> list[float]:
    """Return the affine translation for a zero-authored-yaw reference point."""
    rotated_camera = rotate_xy(design_camera_position, camera_yaw_offset_rad)
    rotated_module = rotate_xy(light_module_offset, camera_yaw_offset_rad)
    return [
        float(design_camera_position[index])
        + float(camera_position_offset[index])
        - rotated_camera[index]
        - rotated_module[index]
        for index in range(3)
    ]


def add_xyz(position: Sequence[float], offset: Iterable[float]) -> list[float]:
    """Return a copy of *position* with an XYZ translation applied."""
    result = list(position)
    values = list(offset)
    if len(result) < 3 or len(values) < 3:
        raise ValueError("position and offset must contain x, y, and z")
    for index in range(3):
        result[index] = float(result[index]) + float(values[index])
    return result
