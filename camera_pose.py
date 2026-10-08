"""RGB pose estimation for a static camera viewing a PixyTile HyperGrid.

The HyperGrid LEDs do not carry IDs.  Their regular lattice nevertheless gives
enough correspondences for planar PnP once a rough camera pose selects the
correct lattice translation and orientation.  This module deliberately does
not decode MyGrid markers.

World coordinates follow the grid file (currently ``world_FLU``).  Camera
coordinates follow OpenCV: x right, y down, z forward.  Camera yaw is the
heading of the optical +z axis projected into the world xy plane.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import cv2
import numpy as np


class PoseEstimationError(RuntimeError):
    """Raised when an image does not provide a trustworthy HyperGrid pose."""


@dataclass(frozen=True)
class MarkerDetectionConfig:
    """Thresholds for saturated or near-saturated RGB HyperGrid LEDs."""

    min_brightness: int = 150
    min_local_contrast: int = 75
    max_saturation: int = 100
    background_sigma_px: float = 5.0
    # The recording camera is upright and framed above the swarm, leaving the
    # floor grid in the bottom of the image.  This is configurable for other
    # framing; values are normalized (left, top, right, bottom).
    roi_normalized: tuple[float, float, float, float] = (0.0, 0.65, 1.0, 1.0)
    min_component_area_px: int = 2
    max_component_area_px: int = 80
    max_component_width_px: int = 16
    max_component_height_px: int = 8
    rendered_marker_radius_px: int = 4


@dataclass(frozen=True)
class PoseSolverConfig:
    """Geometry and quality limits for correspondence search and PnP."""

    min_pattern_columns: int = 3
    min_pattern_rows: int = 3
    min_matched_markers: int = 12
    max_reprojection_rms_px: float = 2.5
    max_reprojection_error_px: float = 6.0
    max_rough_position_error_m: float = 0.30
    # An unlabeled lattice has metrically perfect solutions one marker spacing
    # apart.  The operator's rough XY measurement is therefore a correspondence
    # input, not merely an optimization hint, and must be much tighter than the
    # more permissive 3-D sanity bound above.
    max_rough_xy_error_m: float = 0.07
    max_rough_yaw_error_rad: float = 0.45
    min_correspondence_score_margin_m: float = 0.02
    # ``look_at`` defines the authored pitch because the controller corrects
    # only XYZ and yaw.  The broad look-at gate rejects backwards solutions;
    # the tighter residual-attitude gate below rejects a camera pitch that the
    # swarm transform cannot compensate.
    max_look_at_error_rad: float = 0.70
    max_unmodeled_attitude_error_rad: float = 0.05
    max_abs_camera_roll_rad: float = 0.05
    min_camera_height_m: float = 0.05
    row_cluster_tolerance_px: float = 2.5
    max_lattice_homography_rms_px: float = 2.5
    yaw_score_weight_m_per_rad: float = 0.04
    reprojection_score_weight_m_per_px: float = 0.002


@dataclass(frozen=True)
class CameraCalibration:
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    image_size: tuple[int, int] | None = None
    lens_position: float | None = None
    camera_model: str | None = None


@dataclass(frozen=True)
class HyperGrid:
    origin_xyz: tuple[float, float, float]
    marker_spacing_m: float
    grid_coordinates: tuple[tuple[int, int], ...]
    object_points: tuple[tuple[float, float, float], ...]
    tile_coordinates: tuple[tuple[int, int], ...]
    tile_size_m: float

    @property
    def coordinate_to_index(self) -> dict[tuple[int, int], int]:
        return {
            coordinate: index
            for index, coordinate in enumerate(self.grid_coordinates)
        }


@dataclass(frozen=True)
class PoseQuality:
    detected_markers: int
    matched_markers: int
    inliers: int
    reprojection_rms_px: float
    reprojection_max_px: float
    rough_position_error_m: float
    rough_xy_error_m: float = 0.0
    rough_yaw_error_rad: float = 0.0
    look_at_error_rad: float = 0.0
    unmodeled_attitude_error_rad: float = 0.0
    camera_roll_rad: float = 0.0
    correspondence_score_margin_m: float | None = None
    matching_method: str = "rectangle"


@dataclass(frozen=True)
class CameraPoseEstimate:
    position_xyz: tuple[float, float, float]
    yaw_rad: float
    position_offset_xyz: tuple[float, float, float]
    yaw_offset_rad: float
    world_from_camera_rotation: tuple[tuple[float, float, float], ...]
    rvec_world_to_camera: tuple[float, float, float]
    tvec_world_to_camera: tuple[float, float, float]
    image_points: tuple[tuple[float, float], ...]
    grid_coordinates: tuple[tuple[int, int], ...]
    tile_coordinates: tuple[tuple[int, int], ...]
    quality: PoseQuality

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-safe payload suitable for the camera ZMQ message."""

        return {
            "position_xyz": list(self.position_xyz),
            "yaw_rad": self.yaw_rad,
            "position_offset_xyz": list(self.position_offset_xyz),
            "yaw_offset_rad": self.yaw_offset_rad,
            "world_from_camera_rotation": [
                list(row) for row in self.world_from_camera_rotation
            ],
            "rvec_world_to_camera": list(self.rvec_world_to_camera),
            "tvec_world_to_camera": list(self.tvec_world_to_camera),
            "matches": [
                {
                    "image_point": list(image_point),
                    "grid_coordinate": list(grid_coordinate),
                    "tile_coordinate": list(tile_coordinate),
                }
                for image_point, grid_coordinate, tile_coordinate in zip(
                    self.image_points,
                    self.grid_coordinates,
                    self.tile_coordinates,
                )
            ],
            "quality": {
                "detected_markers": self.quality.detected_markers,
                "matched_markers": self.quality.matched_markers,
                "inliers": self.quality.inliers,
                "reprojection_rms_px": self.quality.reprojection_rms_px,
                "reprojection_max_px": self.quality.reprojection_max_px,
                "rough_position_error_m": (
                    self.quality.rough_position_error_m
                ),
                "rough_xy_error_m": self.quality.rough_xy_error_m,
                "rough_yaw_error_rad": self.quality.rough_yaw_error_rad,
                "look_at_error_rad": self.quality.look_at_error_rad,
                "unmodeled_attitude_error_rad": (
                    self.quality.unmodeled_attitude_error_rad
                ),
                "camera_roll_rad": self.quality.camera_roll_rad,
                "correspondence_score_margin_m": (
                    self.quality.correspondence_score_margin_m
                ),
                "matching_method": self.quality.matching_method,
            },
        }


@dataclass
class _PoseCandidate:
    score: float
    rvec: np.ndarray
    tvec: np.ndarray
    position: np.ndarray
    world_from_camera: np.ndarray
    yaw: float
    image_points: np.ndarray
    grid_coordinates: tuple[tuple[int, int], ...]
    object_points: np.ndarray
    reprojection_errors: np.ndarray
    rough_position_error: float
    rough_xy_error: float
    rough_yaw_error: float
    look_at_error: float
    unmodeled_attitude_error: float
    camera_roll: float


def _as_finite_vector(
    value: Sequence[float], length: int, name: str
) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64).reshape(-1)
    if result.size != length or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {length} finite numbers")
    return result


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _angle_between(first: np.ndarray, second: np.ndarray) -> float:
    """Return the unsigned angle between two non-zero 3-D vectors."""

    first = np.asarray(first, dtype=np.float64).reshape(3)
    second = np.asarray(second, dtype=np.float64).reshape(3)
    denominator = float(np.linalg.norm(first) * np.linalg.norm(second))
    if denominator < 1e-12:
        raise ValueError("cannot measure an angle to a zero-length vector")
    cosine = float(np.dot(first, second) / denominator)
    return math.acos(max(-1.0, min(1.0, cosine)))


def _heading_toward(position: np.ndarray, target: np.ndarray) -> float:
    delta = target[:2] - position[:2]
    if float(np.linalg.norm(delta)) < 1e-9:
        raise ValueError("camera position and look_at cannot share the same xy")
    return math.atan2(float(delta[1]), float(delta[0]))


def load_camera_calibration(path: str | Path) -> CameraCalibration:
    """Load either a direct calibration JSON or a deployment patch JSON."""

    calibration_path = Path(path)
    with calibration_path.open() as stream:
        root = json.load(stream)
    if not isinstance(root, Mapping):
        raise ValueError("camera calibration must be a JSON object")

    values = root.get("calibration", root)
    if not isinstance(values, Mapping):
        raise ValueError("calibration field must be a JSON object")
    try:
        camera_matrix = np.asarray(
            values["camera_matrix"], dtype=np.float64
        ).reshape(3, 3)
        distortion = np.asarray(
            values.get("distortion_coefficients", []), dtype=np.float64
        ).reshape(-1, 1)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"invalid camera calibration: {error}") from error
    if not np.all(np.isfinite(camera_matrix)) or not np.all(
        np.isfinite(distortion)
    ):
        raise ValueError("camera calibration contains non-finite values")
    if camera_matrix[0, 0] <= 0 or camera_matrix[1, 1] <= 0:
        raise ValueError("camera focal lengths must be positive")

    raw_size = values.get("image_size", root.get("image_size"))
    image_size = None
    if raw_size is not None:
        size = tuple(int(item) for item in raw_size)
        if len(size) != 2 or min(size) <= 0:
            raise ValueError("image_size must be [width, height]")
        image_size = size

    capture = values.get("capture", root.get("capture", {}))
    if capture is None:
        capture = {}
    if not isinstance(capture, Mapping):
        raise ValueError("camera calibration capture field must be an object")
    lens_position = capture.get("lens_position")
    if lens_position is not None:
        try:
            lens_position = float(lens_position)
        except (TypeError, ValueError) as error:
            raise ValueError("capture.lens_position must be numeric") from error
        if not math.isfinite(lens_position) or lens_position < 0:
            raise ValueError("capture.lens_position must be finite and non-negative")
    camera_model = capture.get("camera_model")
    if camera_model is not None and (
        not isinstance(camera_model, str) or not camera_model.strip()
    ):
        raise ValueError("capture.camera_model must be a non-empty string")
    return CameraCalibration(
        camera_matrix,
        distortion,
        image_size,
        lens_position,
        camera_model,
    )


def load_hypergrid(path: str | Path) -> HyperGrid:
    """Load the finite set of always-on HyperGrid points from a grid JSON.

    The JSON calls the HyperGrid infinite because its coordinate rule can be
    extended indefinitely.  The installed finite set is derived from the
    configured MyGrid tile centers plus the HyperGrid offsets on each tile.
    """

    grid_path = Path(path)
    with grid_path.open() as stream:
        root = json.load(stream)
    if not isinstance(root, Mapping):
        raise ValueError("grid file must contain a JSON object")
    if root.get("units") not in (None, "metres"):
        raise ValueError("grid units must be metres")
    if root.get("coordinate_frame") not in (None, "world_FLU"):
        raise ValueError("grid coordinate_frame must be world_FLU")

    hypergrid = root.get("hypergrid")
    mygrid = root.get("mygrid")
    if not isinstance(hypergrid, Mapping) or not isinstance(mygrid, Mapping):
        raise ValueError("grid file requires hypergrid and mygrid objects")
    if hypergrid.get("markers_always_on") is False:
        raise ValueError("HyperGrid markers must be configured always on")
    if hypergrid.get("markers_have_ids") is True:
        raise ValueError("this solver expects an unlabeled HyperGrid")

    origin = _as_finite_vector(root.get("grid_origin", [0, 0, 0]), 3, "grid_origin")
    try:
        spacing = float(hypergrid["marker_spacing"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("hypergrid.marker_spacing must be numeric") from error
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError("hypergrid.marker_spacing must be positive")

    offsets = np.asarray(hypergrid.get("tile_local_offsets"), dtype=np.float64)
    if offsets.ndim != 2 or offsets.shape[1] not in (2, 3) or len(offsets) < 1:
        raise ValueError("hypergrid.tile_local_offsets must be an Nx2/Nx3 array")
    if offsets.shape[1] == 2:
        offsets = np.column_stack([offsets, np.zeros(len(offsets))])
    if hypergrid.get("offset_units", "marker_spacing") == "marker_spacing":
        offsets = offsets * spacing
    elif hypergrid.get("offset_units") not in ("metres", "meters"):
        raise ValueError("unsupported hypergrid.offset_units")

    tiles = mygrid.get("tiles")
    if not isinstance(tiles, list) or not tiles:
        raise ValueError(
            "mygrid.tiles is required to bound the installed HyperGrid"
        )

    points_by_coordinate: dict[
        tuple[int, int], tuple[np.ndarray, tuple[int, int]]
    ] = {}
    tile_size = float(hypergrid.get("tile_size", 2.0 * spacing))
    for tile in tiles:
        if not isinstance(tile, Mapping):
            raise ValueError("each mygrid tile must be an object")
        try:
            tile_coordinate = (int(tile["i"]), int(tile["j"]))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("each mygrid tile requires integer i and j") from error
        if "center" in tile:
            center = _as_finite_vector(tile["center"], 3, "tile.center")
        else:
            center = origin + np.array(
                [tile_coordinate[0] * tile_size,
                 tile_coordinate[1] * tile_size, 0.0]
            )
        for offset in offsets:
            point = center + offset
            coordinate_float = (point[:2] - origin[:2]) / spacing - 0.5
            coordinate_array = np.rint(coordinate_float).astype(int)
            if float(np.max(np.abs(coordinate_float - coordinate_array))) > 1e-5:
                raise ValueError(
                    "HyperGrid point does not lie on the configured lattice"
                )
            coordinate = (int(coordinate_array[0]), int(coordinate_array[1]))
            previous = points_by_coordinate.get(coordinate)
            if previous is not None and not np.allclose(previous[0], point):
                raise ValueError(f"conflicting HyperGrid point {coordinate}")
            points_by_coordinate[coordinate] = (point, tile_coordinate)

    ordered = sorted(points_by_coordinate.items())
    return HyperGrid(
        origin_xyz=tuple(float(item) for item in origin),
        marker_spacing_m=spacing,
        grid_coordinates=tuple(item[0] for item in ordered),
        object_points=tuple(
            tuple(float(component) for component in item[1][0])
            for item in ordered
        ),
        tile_coordinates=tuple(item[1][1] for item in ordered),
        tile_size_m=tile_size,
    )


def render_camera_pose_overlay(
    image: np.ndarray,
    estimate: CameraPoseEstimate,
    grid: HyperGrid,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
) -> np.ndarray:
    """Draw every installed tile's projected outline on a pose capture."""
    overlay = np.asarray(image).copy()
    rvec = np.asarray(estimate.rvec_world_to_camera, dtype=np.float64)
    tvec = np.asarray(estimate.tvec_world_to_camera, dtype=np.float64)
    tile_points: dict[tuple[int, int], list[np.ndarray]] = {}
    for tile, point in zip(grid.tile_coordinates, grid.object_points):
        tile_points.setdefault(tile, []).append(np.asarray(point))
    matched = set(estimate.tile_coordinates)
    half_size = grid.tile_size_m / 2.0
    for tile, points in sorted(tile_points.items()):
        center = np.mean(points, axis=0)
        corners = np.asarray([
            center + [-half_size, -half_size, 0],
            center + [half_size, -half_size, 0],
            center + [half_size, half_size, 0],
            center + [-half_size, half_size, 0],
        ], dtype=np.float64)
        projected, _ = cv2.projectPoints(
            corners, rvec, tvec, camera_matrix, distortion_coefficients
        )
        projected = np.rint(projected.reshape(-1, 2)).astype(np.int32)
        color = (0, 220, 0) if tile in matched else (0, 190, 255)
        cv2.polylines(overlay, [projected], True, color, 2, cv2.LINE_AA)
        label = np.rint(np.mean(projected, axis=0)).astype(int)
        cv2.putText(
            overlay, f"{tile[0]},{tile[1]}", tuple(label),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA,
        )
    for point in estimate.image_points:
        cv2.circle(
            overlay, tuple(np.rint(point).astype(int)), 3,
            (255, 0, 255), -1, cv2.LINE_AA,
        )
    return overlay


def detect_hypergrid_markers(
    image: np.ndarray,
    config: MarkerDetectionConfig | None = None,
) -> np.ndarray:
    """Return candidate bright-marker centers as an ``Nx2`` float array."""

    config = config or MarkerDetectionConfig()
    frame = np.asarray(image)
    saturation = None
    if frame.ndim == 2:
        intensity = frame
    elif frame.ndim == 3 and frame.shape[2] in (3, 4):
        # Max-channel intensity detects white and single-color visible LEDs and
        # is independent of whether the caller labels its array RGB or BGR.
        color = frame[:, :, :3]
        intensity = np.max(color, axis=2)
        minimum = np.min(color, axis=2)
        saturation = np.zeros_like(intensity, dtype=np.float64)
        nonzero = intensity > 0
        saturation[nonzero] = (
            255.0
            * (intensity[nonzero].astype(np.float64)
               - minimum[nonzero].astype(np.float64))
            / intensity[nonzero]
        )
    else:
        raise ValueError("image must be grayscale, RGB, RGBA, BGR, or BGRA")
    if intensity.dtype != np.uint8:
        intensity = np.clip(intensity, 0, 255).astype(np.uint8)
    if min(intensity.shape) < 8:
        raise ValueError("image is too small for marker detection")

    background = cv2.GaussianBlur(
        intensity,
        (0, 0),
        sigmaX=config.background_sigma_px,
        sigmaY=config.background_sigma_px,
    )
    contrast = cv2.subtract(intensity, background)
    selected = (
        (intensity >= config.min_brightness)
        & (contrast >= config.min_local_contrast)
    )
    if saturation is not None:
        selected &= saturation <= config.max_saturation
    try:
        left, top, right, bottom = (
            float(item) for item in config.roi_normalized
        )
    except (TypeError, ValueError) as error:
        raise ValueError("roi_normalized must contain four numbers") from error
    if not (0.0 <= left < right <= 1.0 and 0.0 <= top < bottom <= 1.0):
        raise ValueError(
            "roi_normalized must satisfy 0 <= left < right <= 1 and "
            "0 <= top < bottom <= 1"
        )
    roi_mask = np.zeros_like(selected, dtype=bool)
    height, width = intensity.shape
    x0, x1 = int(left * width), int(math.ceil(right * width))
    y0, y1 = int(top * height), int(math.ceil(bottom * height))
    roi_mask[y0:y1, x0:x1] = True
    mask = np.where(selected & roi_mask, 255, 0).astype(np.uint8)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    centers: list[tuple[float, float]] = []
    for label in range(1, count):
        x, y, width, height, area = (
            int(item) for item in stats[label]
        )
        if not (
            config.min_component_area_px <= area <= config.max_component_area_px
            and width <= config.max_component_width_px
            and height <= config.max_component_height_px
        ):
            continue
        component = labels[y:y + height, x:x + width] == label
        weights = contrast[y:y + height, x:x + width].astype(np.float64)
        weights = np.where(component, weights + 1.0, 0.0)
        total = float(np.sum(weights))
        if total <= 0:
            continue
        yy, xx = np.indices(component.shape)
        centers.append((
            x + float(np.sum(xx * weights) / total),
            y + float(np.sum(yy * weights) / total),
        ))
    return np.asarray(centers, dtype=np.float64).reshape(-1, 2)


def _blob_detector(radius: int) -> cv2.SimpleBlobDetector:
    params = cv2.SimpleBlobDetector_Params()
    params.minThreshold = 1
    params.maxThreshold = 256
    params.thresholdStep = 16
    params.minRepeatability = 1
    params.minDistBetweenBlobs = max(2.0, float(radius))
    params.filterByColor = True
    params.blobColor = 255
    params.filterByArea = True
    params.minArea = max(3.0, math.pi * radius * radius * 0.45)
    params.maxArea = math.pi * radius * radius * 2.0
    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia = False
    return cv2.SimpleBlobDetector_create(params)


def _row_ordered_patterns(
    centers: np.ndarray,
    grid: HyperGrid,
    config: PoseSolverConfig,
) -> list[tuple[np.ndarray, int, int]]:
    """Extract near-horizontal projective lattice rows.

    ``findCirclesGrid`` is the general pattern finder below.  This small
    fallback is useful for the intended upright recording camera: compression
    can leave a handful of bright floor-edge outliers, while each HyperGrid row
    remains level to within a couple of pixels.
    """

    clusters: list[list[np.ndarray]] = []
    for point in sorted(centers, key=lambda item: float(item[1])):
        compatible = [
            row for row in clusters
            if abs(float(np.mean([item[1] for item in row])) - point[1])
            <= config.row_cluster_tolerance_px
        ]
        if compatible:
            min(
                compatible,
                key=lambda row: abs(
                    float(np.mean([item[1] for item in row])) - point[1]
                ),
            ).append(point)
        else:
            clusters.append([point])

    maximum_columns = len({item[0] for item in grid.grid_coordinates})
    maximum_rows = len({item[1] for item in grid.grid_coordinates})
    rows_by_length: dict[int, list[np.ndarray]] = {}
    for row in clusters:
        if not config.min_pattern_columns <= len(row) <= maximum_columns:
            continue
        ordered = np.asarray(sorted(row, key=lambda item: float(item[0])))
        # A true row has smoothly changing projective spacing.  This cheap
        # check rejects most accidental equal-y highlight clusters.
        gaps = np.diff(ordered[:, 0])
        if np.any(gaps <= 0):
            continue
        if float(np.std(gaps) / np.mean(gaps)) > 0.35:
            continue
        rows_by_length.setdefault(len(row), []).append(ordered)

    patterns: list[tuple[np.ndarray, int, int]] = []
    for columns, rows in rows_by_length.items():
        rows.sort(key=lambda row: float(np.mean(row[:, 1])))
        largest_row_count = min(maximum_rows, len(rows))
        found_for_length = False
        for row_count in range(
            largest_row_count, config.min_pattern_rows - 1, -1
        ):
            for start in range(0, len(rows) - row_count + 1):
                selected = rows[start:start + row_count]
                image_points = np.concatenate(selected, axis=0)
                lattice_points = np.asarray([
                    (column, row)
                    for row in range(row_count)
                    for column in range(columns)
                ], dtype=np.float64)
                homography, _ = cv2.findHomography(
                    lattice_points, image_points, method=0
                )
                if homography is None:
                    continue
                homogeneous = np.column_stack([
                    lattice_points, np.ones(len(lattice_points))
                ])
                projected = homogeneous @ homography.T
                projected = projected[:, :2] / projected[:, 2:3]
                rms = float(np.sqrt(np.mean(np.square(
                    np.linalg.norm(projected - image_points, axis=1)
                ))))
                if rms <= config.max_lattice_homography_rms_px:
                    patterns.append((image_points, columns, row_count))
                    found_for_length = True
            if found_for_length:
                break
    return patterns


def _largest_ordered_patterns(
    image_shape: tuple[int, int],
    centers: np.ndarray,
    grid: HyperGrid,
    detector_config: MarkerDetectionConfig,
    solver_config: PoseSolverConfig,
) -> list[tuple[np.ndarray, int, int]]:
    height, width = image_shape
    rendered = np.zeros((height, width), dtype=np.uint8)
    radius = detector_config.rendered_marker_radius_px
    for center in centers:
        pixel = tuple(int(round(item)) for item in center)
        if 0 <= pixel[0] < width and 0 <= pixel[1] < height:
            cv2.circle(rendered, pixel, radius, 255, -1)

    unique_x = len({item[0] for item in grid.grid_coordinates})
    unique_y = len({item[1] for item in grid.grid_coordinates})
    dimensions = [
        (columns, rows)
        for columns in range(solver_config.min_pattern_columns, unique_x + 1)
        for rows in range(solver_config.min_pattern_rows, unique_y + 1)
        if solver_config.min_matched_markers <= columns * rows <= len(centers)
    ]
    dimensions.sort(key=lambda value: value[0] * value[1], reverse=True)
    blob_detector = _blob_detector(radius)
    flags = cv2.CALIB_CB_SYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING
    results = _row_ordered_patterns(centers, grid, solver_config)
    row_area = max(
        (columns * rows for _, columns, rows in results), default=0
    )
    # A substantial level-row pattern is more selective than circles-grid
    # clustering in the presence of unrelated bright points.  Keep the latter
    # as the rotation-tolerant fallback when no such row pattern is available.
    if row_area >= max(20, 2 * solver_config.min_matched_markers):
        return [
            item for item in results if item[1] * item[2] == row_area
        ]
    found_area = max(
        (columns * rows for _, columns, rows in results), default=None
    )
    for columns, rows in dimensions:
        area = columns * rows
        if found_area is not None and area < found_area:
            break
        found, ordered = cv2.findCirclesGrid(
            rendered,
            (columns, rows),
            flags=flags,
            blobDetector=blob_detector,
        )
        if not found:
            continue
        ordered = ordered.reshape(-1, 2).astype(np.float64)
        # Replace the integer rendered centers with the original weighted
        # centroids.  Grid spacing is much larger than the sub-pixel snap.
        available = set(range(len(centers)))
        snapped: list[np.ndarray] = []
        for point in ordered:
            index = min(
                available,
                key=lambda candidate: float(
                    np.linalg.norm(centers[candidate] - point)
                ),
            )
            snapped.append(centers[index])
            available.remove(index)
        results.append((np.asarray(snapped), columns, rows))
        found_area = area
    largest_area = max(
        (columns * rows for _, columns, rows in results), default=0
    )
    return [
        item for item in results if item[1] * item[2] == largest_area
    ]


def _lattice_symmetries() -> tuple[np.ndarray, ...]:
    return tuple(
        np.asarray(((a, b), (c, d)), dtype=int)
        for a, b, c, d in (
            (1, 0, 0, 1),
            (1, 0, 0, -1),
            (-1, 0, 0, 1),
            (-1, 0, 0, -1),
            (0, 1, 1, 0),
            (0, 1, -1, 0),
            (0, -1, 1, 0),
            (0, -1, -1, 0),
        )
    )


def _coordinate_assignments(
    columns: int,
    rows: int,
    grid: HyperGrid,
):
    local = np.asarray(
        [(column, row) for row in range(rows) for column in range(columns)],
        dtype=int,
    )
    available = set(grid.grid_coordinates)
    emitted: set[tuple[tuple[int, int], ...]] = set()
    for symmetry in _lattice_symmetries():
        transformed = local @ symmetry.T
        for anchor in grid.grid_coordinates:
            coordinates = tuple(
                (int(item[0] + anchor[0]), int(item[1] + anchor[1]))
                for item in transformed
            )
            if coordinates in emitted or any(
                coordinate not in available for coordinate in coordinates
            ):
                continue
            emitted.add(coordinates)
            yield coordinates


def _candidate_from_prior(
    centers: np.ndarray,
    image_shape: tuple[int, int],
    grid: HyperGrid,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    prior_pose: Mapping[str, Any],
    rough_position: np.ndarray,
    rough_yaw: float,
    target: np.ndarray,
    desired_position: np.ndarray,
    config: PoseSolverConfig,
) -> _PoseCandidate | None:
    """Fit visible grid points when occlusions break the rectangle finder.

    The saved pose supplies correspondences only. RANSAC and the normal quality
    gates must validate a new pose from the current image.
    """
    try:
        prior_position = np.asarray(
            prior_pose["position_xyz"], dtype=np.float64
        ).reshape(3)
        prior_rvec = np.asarray(
            prior_pose["rvec_world_to_camera"], dtype=np.float64
        ).reshape(3, 1)
        prior_tvec = np.asarray(
            prior_pose["tvec_world_to_camera"], dtype=np.float64
        ).reshape(3, 1)
    except (KeyError, TypeError, ValueError):
        return None
    if not all(np.all(np.isfinite(value)) for value in (
        prior_position, prior_rvec, prior_tvec
    )):
        return None
    if (
        np.linalg.norm(prior_position - rough_position)
        > config.max_rough_position_error_m
        or np.linalg.norm(prior_position[:2] - rough_position[:2])
        > config.max_rough_xy_error_m
    ):
        return None
    rotation, _ = cv2.Rodrigues(prior_rvec)
    if np.linalg.norm(-(rotation.T @ prior_tvec).reshape(3) - prior_position) > 1e-3:
        return None

    object_points = np.asarray(grid.object_points, dtype=np.float64)
    projected, _ = cv2.projectPoints(
        object_points, prior_rvec, prior_tvec, camera_matrix, distortion
    )
    projected = projected.reshape(-1, 2)
    height, width = image_shape
    match_radius_px = 22.0
    possible_matches = sorted(
        (float(np.linalg.norm(predicted - observed)), grid_index, image_index)
        for grid_index, predicted in enumerate(projected)
        if 0 <= predicted[0] < width and 0 <= predicted[1] < height
        for image_index, observed in enumerate(centers)
        if np.linalg.norm(predicted - observed) <= match_radius_px
    )
    used_grid: set[int] = set()
    used_image: set[int] = set()
    matches = []
    for _, grid_index, image_index in possible_matches:
        if grid_index in used_grid or image_index in used_image:
            continue
        used_grid.add(grid_index)
        used_image.add(image_index)
        matches.append((grid_index, image_index))
    if len(matches) < config.min_matched_markers:
        return None

    matched_objects = object_points[[item[0] for item in matches]]
    matched_images = centers[[item[1] for item in matches]]
    solved, rvec, tvec, inliers = cv2.solvePnPRansac(
        matched_objects, matched_images, camera_matrix, distortion,
        rvec=prior_rvec.copy(), tvec=prior_tvec.copy(),
        useExtrinsicGuess=True, iterationsCount=1000,
        reprojectionError=3.0, confidence=0.999,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not solved or inliers is None or len(inliers) < config.min_matched_markers:
        return None
    indices = inliers.reshape(-1)
    return _candidate_from_solution(
        matched_objects[indices], matched_images[indices],
        tuple(grid.grid_coordinates[matches[index][0]] for index in indices),
        rvec, tvec, camera_matrix, distortion, rough_position, rough_yaw,
        target, desired_position, float(grid.origin_xyz[2]), config,
    )


def _candidate_from_solution(
    object_points: np.ndarray,
    image_points: np.ndarray,
    grid_coordinates: tuple[tuple[int, int], ...],
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    rough_position: np.ndarray,
    rough_yaw: float,
    look_at: np.ndarray,
    desired_position: np.ndarray,
    grid_plane_z: float,
    config: PoseSolverConfig,
) -> _PoseCandidate | None:
    rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
    tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1)
    if hasattr(cv2, "solvePnPRefineLM"):
        rvec, tvec = cv2.solvePnPRefineLM(
            object_points,
            image_points,
            camera_matrix,
            distortion,
            rvec,
            tvec,
        )
    world_to_camera, _ = cv2.Rodrigues(rvec)
    camera_points = (
        world_to_camera @ object_points.T + tvec
    ).T
    if np.any(camera_points[:, 2] <= 0):
        return None
    world_from_camera = world_to_camera.T
    position = -(world_from_camera @ tvec).reshape(3)
    if position[2] <= grid_plane_z + config.min_camera_height_m:
        return None
    forward = world_from_camera[:, 2]
    if float(np.linalg.norm(forward[:2])) < 1e-6:
        return None
    yaw = math.atan2(float(forward[1]), float(forward[0]))
    rough_yaw_error = abs(_wrap_angle(yaw - rough_yaw))

    expected_forward = look_at - position
    if float(np.linalg.norm(expected_forward)) < 1e-9:
        return None
    look_at_error = _angle_between(forward, expected_forward)

    # Preserve the pitch implied by the authored camera and look-at point,
    # while allowing the measured yaw correction.  Any remaining forward-axis
    # difference is pitch/attitude that an XYZ+yaw swarm transform cannot
    # reproduce and must therefore fail closed.
    authored_forward = look_at - desired_position
    if float(np.linalg.norm(authored_forward)) < 1e-9:
        return None
    authored_elevation = math.atan2(
        float(authored_forward[2]),
        float(np.linalg.norm(authored_forward[:2])),
    )
    expected_yaw_only_forward = np.asarray([
        math.cos(authored_elevation) * math.cos(yaw),
        math.cos(authored_elevation) * math.sin(yaw),
        math.sin(authored_elevation),
    ])
    unmodeled_attitude_error = _angle_between(
        forward, expected_yaw_only_forward
    )

    # With an upright recording camera its image-right axis is horizontal.
    # Compare the recovered right vector against that zero-roll direction so
    # roll cannot silently masquerade as a yaw-only swarm correction.
    world_up = np.asarray([0.0, 0.0, 1.0])
    ideal_right = np.cross(forward, world_up)
    if float(np.linalg.norm(ideal_right)) < 1e-9:
        return None
    ideal_right /= np.linalg.norm(ideal_right)
    actual_right = world_from_camera[:, 0]
    camera_roll = math.atan2(
        float(np.dot(np.cross(ideal_right, actual_right), forward)),
        float(np.dot(ideal_right, actual_right)),
    )

    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, distortion
    )
    errors = np.linalg.norm(
        projected.reshape(-1, 2) - image_points, axis=1
    )
    rms = float(np.sqrt(np.mean(np.square(errors))))
    rough_error = float(np.linalg.norm(position - rough_position))
    rough_xy_error = float(np.linalg.norm(position[:2] - rough_position[:2]))
    score = (
        rough_error
        + config.yaw_score_weight_m_per_rad
        * rough_yaw_error
        + config.reprojection_score_weight_m_per_px * rms
    )
    return _PoseCandidate(
        score=score,
        rvec=rvec,
        tvec=tvec,
        position=position,
        world_from_camera=world_from_camera,
        yaw=yaw,
        image_points=image_points,
        grid_coordinates=grid_coordinates,
        object_points=object_points,
        reprojection_errors=errors,
        rough_position_error=rough_error,
        rough_xy_error=rough_xy_error,
        rough_yaw_error=rough_yaw_error,
        look_at_error=look_at_error,
        unmodeled_attitude_error=unmodeled_attitude_error,
        camera_roll=camera_roll,
    )


def estimate_camera_pose_from_markers(
    image_points: np.ndarray,
    image_shape: Sequence[int],
    grid: HyperGrid,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray | Sequence[float] | None,
    rough_camera_position: Sequence[float],
    desired_camera_position: Sequence[float],
    *,
    rough_camera_yaw: float | None = None,
    desired_camera_yaw: float | None = None,
    look_at: Sequence[float] | None = None,
    detected_marker_count: int | None = None,
    detector_config: MarkerDetectionConfig | None = None,
    solver_config: PoseSolverConfig | None = None,
    prior_camera_pose: Mapping[str, Any] | None = None,
) -> CameraPoseEstimate:
    """Resolve unlabeled marker centers and estimate the camera world pose."""

    detector_config = detector_config or MarkerDetectionConfig()
    solver_config = solver_config or PoseSolverConfig()
    centers = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    if not np.all(np.isfinite(centers)):
        raise ValueError("image_points contains non-finite coordinates")
    if len(image_shape) < 2:
        raise ValueError("image_shape must contain height and width")
    height, width = (int(image_shape[0]), int(image_shape[1]))
    if height <= 0 or width <= 0:
        raise ValueError("image_shape must be positive")
    if len(centers) < solver_config.min_matched_markers:
        raise PoseEstimationError(
            f"only {len(centers)} marker candidates; "
            f"need at least {solver_config.min_matched_markers}"
        )

    camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    if distortion_coefficients is None:
        distortion = np.zeros((5, 1), dtype=np.float64)
    else:
        distortion = np.asarray(
            distortion_coefficients, dtype=np.float64
        ).reshape(-1, 1)
    rough_position = _as_finite_vector(
        rough_camera_position, 3, "rough_camera_position"
    )
    desired_position = _as_finite_vector(
        desired_camera_position, 3, "desired_camera_position"
    )
    target = _as_finite_vector(
        look_at if look_at is not None else grid.origin_xyz,
        3,
        "look_at",
    )
    if rough_camera_yaw is None:
        rough_yaw = _heading_toward(rough_position, target)
    else:
        rough_yaw = float(rough_camera_yaw)
    if desired_camera_yaw is None:
        desired_yaw = _heading_toward(desired_position, target)
    else:
        desired_yaw = float(desired_camera_yaw)
    if not math.isfinite(rough_yaw) or not math.isfinite(desired_yaw):
        raise ValueError("camera yaw must be finite")

    patterns = _largest_ordered_patterns(
        (height, width),
        centers,
        grid,
        detector_config,
        solver_config,
    )
    if not patterns and prior_camera_pose is None:
        raise PoseEstimationError(
            "bright points did not form a complete HyperGrid rectangle"
        )

    point_by_coordinate = {
        coordinate: np.asarray(point, dtype=np.float64)
        for coordinate, point in zip(
            grid.grid_coordinates, grid.object_points
        )
    }
    candidates: list[_PoseCandidate] = []
    prior_candidate = None
    for ordered_points, columns, rows in patterns:
        for coordinates in _coordinate_assignments(columns, rows, grid):
            object_points = np.asarray(
                [point_by_coordinate[item] for item in coordinates],
                dtype=np.float64,
            )
            try:
                solved, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                    object_points,
                    ordered_points,
                    camera_matrix,
                    distortion,
                    flags=cv2.SOLVEPNP_IPPE,
                )
            except cv2.error:
                continue
            if not solved:
                continue
            for rvec, tvec in zip(rvecs, tvecs):
                candidate = _candidate_from_solution(
                    object_points,
                    ordered_points,
                    coordinates,
                    rvec,
                    tvec,
                    camera_matrix,
                    distortion,
                    rough_position,
                    rough_yaw,
                    target,
                    desired_position,
                    float(grid.origin_xyz[2]),
                    solver_config,
                )
                if candidate is not None:
                    candidates.append(candidate)
    if prior_camera_pose is not None:
        prior_candidate = _candidate_from_prior(
            centers, (height, width), grid, camera_matrix, distortion,
            prior_camera_pose, rough_position, rough_yaw, target,
            desired_position, solver_config,
        )
        if prior_candidate is not None:
            candidates.append(prior_candidate)
    if not candidates:
        raise PoseEstimationError("planar PnP produced no physical pose")

    valid_candidates = []
    for candidate in candidates:
        rms = float(np.sqrt(np.mean(np.square(
            candidate.reprojection_errors
        ))))
        maximum = float(np.max(candidate.reprojection_errors))
        if (
            rms <= solver_config.max_reprojection_rms_px
            and maximum <= solver_config.max_reprojection_error_px
            and candidate.rough_position_error
            <= solver_config.max_rough_position_error_m
            and candidate.rough_xy_error
            <= solver_config.max_rough_xy_error_m
            and candidate.rough_yaw_error
            <= solver_config.max_rough_yaw_error_rad
            and candidate.look_at_error
            <= solver_config.max_look_at_error_rad
            and candidate.unmodeled_attitude_error
            <= solver_config.max_unmodeled_attitude_error_rad
            and abs(candidate.camera_roll)
            <= solver_config.max_abs_camera_roll_rad
        ):
            valid_candidates.append(candidate)
    if not valid_candidates:
        closest = min(candidates, key=lambda item: item.score)
        closest_rms = float(np.sqrt(np.mean(np.square(
            closest.reprojection_errors
        ))))
        closest_maximum = float(np.max(closest.reprojection_errors))
        raise PoseEstimationError(
            "no pose passed quality gates; best candidate had "
            f"RMS={closest_rms:.2f}px, max={closest_maximum:.2f}px, "
            f"rough_position_error={closest.rough_position_error:.3f}m, "
            f"rough_xy_error={closest.rough_xy_error:.3f}m, "
            f"rough_yaw_error={closest.rough_yaw_error:.3f}rad, "
            f"look_at_error={closest.look_at_error:.3f}rad, "
            "unmodeled_attitude_error="
            f"{closest.unmodeled_attitude_error:.3f}rad, "
            f"roll={closest.camera_roll:.3f}rad"
        )

    best = min(valid_candidates, key=lambda item: item.score)
    # Ignore numerically duplicate solutions for the same physical pose, then
    # require the rough prior to prefer the winning unlabeled correspondence by
    # a useful margin.  This catches measurements near a half-cell/D4 boundary
    # instead of moving the entire swarm on an arbitrary tie.
    distinct_alternatives = [
        candidate
        for candidate in valid_candidates
        if candidate is not best
        and (
            float(np.linalg.norm(candidate.position - best.position))
            > 0.25 * grid.marker_spacing_m
            or abs(_wrap_angle(candidate.yaw - best.yaw)) > 0.10
        )
    ]
    score_margin = None
    if distinct_alternatives:
        runner_up = min(distinct_alternatives, key=lambda item: item.score)
        score_margin = float(runner_up.score - best.score)
        if score_margin < solver_config.min_correspondence_score_margin_m:
            raise PoseEstimationError(
                "unlabeled HyperGrid correspondence is ambiguous; best-to-"
                f"runner-up score margin is {score_margin:.3f}m-equivalent "
                f"(need {solver_config.min_correspondence_score_margin_m:.3f})"
            )
    rms = float(np.sqrt(np.mean(np.square(best.reprojection_errors))))
    maximum = float(np.max(best.reprojection_errors))

    coordinate_to_index = grid.coordinate_to_index
    matched_tiles = tuple(
        grid.tile_coordinates[coordinate_to_index[item]]
        for item in best.grid_coordinates
    )
    # This matches the legacy viewpoint convention: actual camera pose minus
    # the authored/SFL camera pose.  The orchestrator decides how that camera
    # delta maps into its swarm-frame transform.
    position_offset = best.position - desired_position
    yaw_offset = _wrap_angle(best.yaw - desired_yaw)
    quality = PoseQuality(
        detected_markers=(
            int(detected_marker_count)
            if detected_marker_count is not None
            else len(centers)
        ),
        matched_markers=len(best.image_points),
        inliers=len(best.image_points),
        reprojection_rms_px=rms,
        reprojection_max_px=maximum,
        rough_position_error_m=best.rough_position_error,
        rough_xy_error_m=best.rough_xy_error,
        rough_yaw_error_rad=best.rough_yaw_error,
        look_at_error_rad=best.look_at_error,
        unmodeled_attitude_error_rad=best.unmodeled_attitude_error,
        camera_roll_rad=best.camera_roll,
        correspondence_score_margin_m=score_margin,
        matching_method=(
            "prior_seeded_ransac" if best is prior_candidate else "rectangle"
        ),
    )
    return CameraPoseEstimate(
        position_xyz=tuple(float(item) for item in best.position),
        yaw_rad=best.yaw,
        position_offset_xyz=tuple(float(item) for item in position_offset),
        yaw_offset_rad=yaw_offset,
        world_from_camera_rotation=tuple(
            tuple(float(item) for item in row)
            for row in best.world_from_camera
        ),
        rvec_world_to_camera=tuple(float(item) for item in best.rvec.reshape(3)),
        tvec_world_to_camera=tuple(float(item) for item in best.tvec.reshape(3)),
        image_points=tuple(
            tuple(float(item) for item in point)
            for point in best.image_points
        ),
        grid_coordinates=best.grid_coordinates,
        tile_coordinates=matched_tiles,
        quality=quality,
    )


def estimate_camera_pose(
    image: np.ndarray,
    grid_file: str | Path,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray | Sequence[float] | None,
    rough_camera_position: Sequence[float],
    desired_camera_position: Sequence[float],
    *,
    rough_camera_yaw: float | None = None,
    desired_camera_yaw: float | None = None,
    look_at: Sequence[float] | None = None,
    detector_config: MarkerDetectionConfig | None = None,
    solver_config: PoseSolverConfig | None = None,
    prior_camera_pose: Mapping[str, Any] | None = None,
) -> CameraPoseEstimate:
    """Detect RGB HyperGrid points and return pose plus SFL pose offsets."""

    grid = load_hypergrid(grid_file)
    detection_config = detector_config or MarkerDetectionConfig()
    centers = detect_hypergrid_markers(image, detection_config)
    return estimate_camera_pose_from_markers(
        centers,
        image.shape,
        grid,
        camera_matrix,
        distortion_coefficients,
        rough_camera_position,
        desired_camera_position,
        rough_camera_yaw=rough_camera_yaw,
        desired_camera_yaw=desired_camera_yaw,
        look_at=look_at,
        detected_marker_count=len(centers),
        detector_config=detection_config,
        solver_config=solver_config,
        prior_camera_pose=prior_camera_pose,
    )
