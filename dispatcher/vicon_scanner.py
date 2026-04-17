"""Stage 1: Vicon scanning and stable ground-truth coordinate computation.

Connects to Vicon, collects N frames of unlabeled marker data, clusters the
observations into drone-count centroids via k-means, and returns a sorted
initial assignment matched to the swarm manifest.
"""
import logging
import time

import numpy as np

logger = logging.getLogger(__name__)

VICON_PORT = 801


class ViconConnectionError(RuntimeError):
    pass


class InsufficientMarkersError(RuntimeError):
    pass


class ViconScanner:
    """Collects unlabeled Vicon markers over N frames and clusters them."""

    def __init__(self, host: str):
        self.host = host
        self.address = f"{host}:{VICON_PORT}"

    def scan(self, n_frames: int, n_drones: int) -> list:
        """Scan Vicon for n_frames and return n_drones averaged 3-D positions (meters).

        Returns:
            List of [x, y, z] numpy arrays in meters, length == n_drones.

        Raises:
            ViconConnectionError: Cannot reach the Vicon system.
            InsufficientMarkersError: Fewer markers than drones detected after retries.
        """
        try:
            from pyvicon_datastream import PyViconDatastream, StreamMode, Direction
        except ImportError as exc:
            raise ViconConnectionError(
                "pyvicon_datastream is not installed. "
                "Install it with: pip install pyvicon-datastream-sdk"
            ) from exc

        client = PyViconDatastream()
        logger.info(f"Connecting to Vicon at {self.address} ...")
        if not client.connect(self.address):
            raise ViconConnectionError(
                f"Could not connect to Vicon server at {self.address}. "
                "Check that the Vicon PC is on, Nexus/Tracker is running, "
                "and the host address in pipeline_config.yaml is correct."
            )

        try:
            client.enable_unlabeled_marker_data()
            client.set_stream_mode(StreamMode.ClientPull)
            client.set_axis_mapping(Direction.Forward, Direction.Left, Direction.Up)
            logger.info("Vicon connected. Collecting frames ...")

            all_observations, max_markers = self._collect_frames(client, n_frames, n_drones)
        finally:
            if client.is_connected():
                client.disconnect()
                logger.info("Disconnected from Vicon.")

        return self._cluster(all_observations, n_drones, max_markers)

    def _collect_frames(self, client, n_frames: int, n_drones: int) -> tuple[np.ndarray, int]:
        """Collect all unlabeled marker positions across n_frames.

        Retries up to 3 times if a frame has fewer markers than n_drones.
        Returns Nx3 array of raw observations in meters and the max markers seen in a frame.
        """
        all_points = []
        frames_collected = 0
        max_retries = 3

        max_markers = 0
        for attempt in range(1, max_retries + 1):
            all_points = []
            frames_collected = 0
            insufficient_count = 0
            max_markers = 0

            while frames_collected < n_frames:
                if not client.get_frame():
                    time.sleep(0.01)
                    continue

                marker_count = client.get_unlabeled_marker_count()
                if marker_count is None or marker_count < n_drones:
                    insufficient_count += 1
                    logger.warning(
                        f"Frame has {marker_count or 0} markers, expected >= {n_drones}. "
                        f"(insufficient count this attempt: {insufficient_count})"
                    )
                    continue

                max_markers = max(max_markers, marker_count)
                for i in range(marker_count):
                    pos = client.get_unlabeled_marker_global_translation(i)
                    if pos is not None:
                        x, y, z = pos
                        all_points.append([x / 1000.0, y / 1000.0, z / 1000.0])

                frames_collected += 1
                logger.debug(f"Frame {frames_collected}/{n_frames} collected ({marker_count} markers)")

            if all_points:
                break

            if attempt < max_retries:
                logger.warning(
                    f"Attempt {attempt}/{max_retries}: no valid frames collected. "
                    f"Retrying in 2 s ..."
                )
                time.sleep(2)

        if not all_points:
            raise InsufficientMarkersError(
                f"After {max_retries} attempts, could not collect {n_frames} frames "
                f"with at least {n_drones} markers. "
                f"Found 0 valid observations. "
                "Check that all drones are powered and visible to Vicon."
            )

        return np.array(all_points), max_markers

    def _cluster(self, observations: np.ndarray, n_drones: int, max_markers: int) -> list:
        """K-means cluster raw observations. Clusters into max_markers to isolate outliers."""
        try:
            from sklearn.cluster import KMeans
        except ImportError as exc:
            raise RuntimeError(
                "scikit-learn is required for Vicon scanning. "
                "Install it with: pip install scikit-learn"
            ) from exc

        n_clusters = max(n_drones, max_markers)
        km = KMeans(n_clusters=n_clusters, n_init=10, random_state=0)
        km.fit(observations)
        centroids = km.cluster_centers_  # shape (n_clusters, 3)
        logger.info(f"Clustered {len(observations)} observations into {n_clusters} centroids.")
        return [np.array(c) for c in centroids]


def mock_scan(manifest_drones: list, noise_std: float = 0.008) -> list:
    """Return manifest init_pos values with small Gaussian noise (no Vicon needed).

    Useful for hardware-free pipeline testing.  The noise mimics real-world
    marker jitter so the k-means cluster step (skipped here) is not needed.

    Args:
        manifest_drones: List of drone config dicts from swarm_manifest.yaml.
        noise_std:       Std-dev of position noise in meters (default 8 mm).

    Returns:
        List of [x, y, z] numpy arrays in meters, same order as manifest_drones.
    """
    import numpy as np
    rng = np.random.default_rng(seed=42)
    points = []
    for drone in manifest_drones:
        pos = np.array(drone['init_pos'], dtype=float)
        noisy = pos + rng.normal(0.0, noise_std, size=3)
        points.append(noisy)
    logger.info(
        f"[MockVicon] Returning {len(points)} positions from manifest init_pos "
        f"(noise_std={noise_std*1000:.0f} mm, no Vicon connection)."
    )
    return points

def sort_and_match_ordered(vicon_points: list, manifest_drones: list) -> list:
    """Produce initial assignment by sorting both lists by (y, x) and matching in order.

    Args:
        vicon_points: List of [x, y, z] arrays in meters.
        manifest_drones: List of drone config dicts from swarm_manifest.yaml.

    Returns:
        List of dicts: [{id, vicon_pos, manifest_pos}, ...] sorted by (y, x) of vicon_pos.
    """
    sorted_points = sorted(vicon_points, key=lambda p: (p[1], p[0]))
    sorted_drones = sorted(
        manifest_drones,
        key=lambda d: (d['init_pos'][1], d['init_pos'][0])
    )

    assignment = []
    for drone, point in zip(sorted_drones, sorted_points):
        assignment.append({
            "id": drone["id"],
            "vicon_pos": point.tolist(),
            "manifest_pos": list(drone["init_pos"]),
        })

    return assignment


def sort_and_match(vicon_points: list, manifest_drones: list) -> list:
    """Produce initial assignment by matching each vicon point to the closest drone's initial pos.

    Args:
        vicon_points: List of [x, y, z] arrays in meters.
        manifest_drones: List of drone config dicts from swarm_manifest.yaml.

    Returns:
        Tuple of (assignment, outliers) where assignment is a list of dicts: [{id, vicon_pos, manifest_pos}, ...] matched by minimum Euclidean distance.
    """
    import numpy as np
    
    v_points = [np.array(p) for p in vicon_points]
    d_points = [np.array(d['init_pos']) for d in manifest_drones]
    
    # Calculate all pairwise distances
    distances = []
    for i, vp in enumerate(v_points):
        for j, dp in enumerate(d_points):
            distances.append((i, j, np.linalg.norm(vp - dp)))
            
    # Sort distances ascending
    distances.sort(key=lambda x: x[2])
    
    used_v = set()
    used_d = set()
    matches = {} # v_idx -> d_idx
    
    for v_idx, d_idx, dist in distances:
        if v_idx not in used_v and d_idx not in used_d:
            matches[v_idx] = d_idx
            used_v.add(v_idx)
            used_d.add(d_idx)
            
    assignment = []
    outliers = []
    for v_idx in range(len(vicon_points)):
        vp = vicon_points[v_idx]
        vp_list = vp.tolist() if isinstance(vp, np.ndarray) else list(vp)
        if v_idx in matches:
            d_idx = matches[v_idx]
            drone = manifest_drones[d_idx]
            assignment.append({
                "id": drone["id"],
                "vicon_pos": vp_list,
                "manifest_pos": list(drone["init_pos"]),
            })
        else:
            outliers.append(vp_list)
            
    if outliers:
        logger.warning(f"Detected {len(outliers)} outlier markers (ignored from assignments):")
        for idx, o in enumerate(outliers):
            print(f"[Dispatcher] Outlier Marker {idx+1} Position: [{o[0]:.3f}, {o[1]:.3f}, {o[2]:.3f}]")

    return assignment, outliers
