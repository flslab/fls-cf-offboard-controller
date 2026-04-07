import argparse
import threading
import time
import logging
import motioncapture
import numpy as np

logger = logging.getLogger(__name__)


class Mocap(threading.Thread):
    def __init__(self, mocap_system_type="vicon", host_name="192.168.1.39", mode="mixed"):
        """
        Args:
            mocap_system_type (str): Type of system (vicon, optitrack, etc).
            host_name (str): Hostname or IP of the mocap server.
            mode (str): 'rigidbody' or 'pointcloud'.
        """
        threading.Thread.__init__(self)

        self.running = True
        self.mocap_system_type = mocap_system_type
        self.host_name = host_name
        self.mode = mode.lower()

        # Shared state
        self.objects_to_track = []  # For RigidBodies: list of (name, callback)
        self.points_to_track = []  # For PointCloud: list of mutable dicts
        self.shapes_to_track = []  # For MarkerShape: list of mutable dicts
        self.anchor_data = None

        # Lock is ONLY for writers (subscribe/unsubscribe), not the reader (run)
        self._write_lock = threading.Lock()

    def stop(self):
        self.running = False
        self.join()

    def _get_pointcloud_array(self, mc):
        """Fetches and formats the point cloud array if applicable."""
        if self.mode in ['pointcloud', 'mixed'] and hasattr(mc, 'pointCloud') and mc.pointCloud is not None:
            try:
                return np.array(mc.pointCloud, copy=False) * 1000.0
            except (ValueError, TypeError):
                return np.array(mc.pointCloud) * 1000.0
        return None

    def _calculate_noise_offset(self, mc, anchor, cloud_arr):
        """Calculates the global offset based on the anchor's drift."""
        current_noise_offset = np.array([0.0, 0.0, 0.0])

        if anchor is None:
            return current_noise_offset

        if anchor['type'] == 'rigidbody' and self.mode in ['rigidbody', 'mixed']:
            tracked = False
            for name, obj in mc.rigidBodies.items():
                if name == anchor['name']:
                    pos = np.array([float(obj.position[0]), float(obj.position[1]), float(obj.position[2])])
                    if anchor['initial_pos'] is None:
                        anchor['initial_pos'] = pos
                    anchor['offset'] = pos - anchor['initial_pos']
                    break
            if not tracked:
                logger.info("Anchor Not Tracked")

            current_noise_offset = anchor['offset']

        elif anchor['type'] == 'pointcloud' and cloud_arr is not None and len(cloud_arr) > 0:
            diff = cloud_arr - anchor['current_pos']
            dist_sq = np.einsum('ij,ij->i', diff, diff)
            min_idx = np.argmin(dist_sq)
            min_dist_sq = dist_sq[min_idx]

            max_dist_sq = anchor['max_dist_sq'] if anchor['captured'] else anchor['max_dist_sq'] * 4
            if min_dist_sq <= max_dist_sq:
                anchor['captured'] = True
                closest_point = cloud_arr[min_idx]
                anchor['current_pos'] = closest_point
                anchor['offset'] = (closest_point - anchor['initial_pos']) / 1000.0
            else:
                logger.info("Anchor Not Tracked")

            current_noise_offset = anchor['offset']

        return current_noise_offset

    def _process_rigid_bodies(self, mc, targets, offset, frame_count, now):
        """Applies offset to rigid bodies and triggers callbacks."""
        if self.mode not in ['rigidbody', 'mixed']:
            return

        for name, obj in mc.rigidBodies.items():
            for obj_name, callback in targets:
                if name == obj_name:
                    pos = np.array([float(obj.position[0]), float(obj.position[1]), float(obj.position[2])])
                    quat = obj.rotation
                    corrected_pos = pos - offset

                    if callback:
                        callback({
                            "frame_id": frame_count,
                            "tvec": [float(corrected_pos[0]), float(corrected_pos[1]), float(corrected_pos[2])],
                            "noise": offset.tolist(),
                            "quat": [float(quat.x), float(quat.y), float(quat.z), float(quat.w)],
                            "time": now
                        })

    @staticmethod
    def _kabsch_rotation(reference, current):
        """Compute rotation matrix from reference to current using Kabsch/SVD algorithm.
        Both arrays are Nx3 and must already be centroid-centered."""
        H = reference.T @ current
        U, S, Vt = np.linalg.svd(H)
        # Correct for reflection (det = -1 means improper rotation)
        d = np.linalg.det(Vt.T @ U.T)
        D = np.diag([1.0, 1.0, d])
        return Vt.T @ D @ U.T

    @staticmethod
    def _rotation_matrix_to_quat(R):
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return [float(x), float(y), float(z), float(w)]

    def _process_point_clouds(self, cloud_arr, targets, offset, frame_count, now):
        """Applies offset to point clouds and triggers callbacks."""
        if self.mode not in ['pointcloud', 'mixed'] or cloud_arr is None or len(cloud_arr) == 0:
            return

        for pt_data in targets:
            target_pos = pt_data['current_pos']
            diff = cloud_arr - target_pos
            dist_sq = np.einsum('ij,ij->i', diff, diff)
            min_idx = np.argmin(dist_sq)
            min_dist_sq = dist_sq[min_idx]

            max_dist_sq = pt_data['max_dist_sq'] if pt_data['captured'] else pt_data['max_dist_sq'] * 4
            if min_dist_sq <= max_dist_sq:
                pt_data['captured'] = True
                closest_point = cloud_arr[min_idx]
                pt_data['current_pos'] = closest_point

                if pt_data['callback']:
                    corrected_pos = (closest_point / 1000.0) - offset
                    pt_data['callback']({
                        "frame_id": frame_count,
                        "tvec": corrected_pos.tolist(),
                        "noise": offset.tolist(),
                        "dist_sq": float(min_dist_sq),
                        "time": now
                    })
    def _process_marker_shapes(self, cloud_arr, targets, offset, frame_count, now):
        """Track each marker individually, compute centroid (tvec) and rotation (quat) via Kabsch."""
        if self.mode not in ['pointcloud', 'mixed'] or cloud_arr is None or len(cloud_arr) == 0:
            return

        for shape in targets:
            new_positions = []
            all_found = True

            for i, cur_pos in enumerate(shape['current_positions']):
                diff = cloud_arr - cur_pos
                dist_sq = np.einsum('ij,ij->i', diff, diff)
                min_idx = np.argmin(dist_sq)
                min_dist_sq = dist_sq[min_idx]

                max_dist_sq = shape['max_dist_sq'] if shape['captured'][i] else shape['max_dist_sq'] * 4
                if min_dist_sq <= max_dist_sq:
                    shape['captured'][i] = True
                    new_positions.append(cloud_arr[min_idx].copy())
                else:
                    all_found = False
                    new_positions.append(cur_pos.copy())

            shape['current_positions'] = new_positions

            if all_found and shape['callback']:
                positions_mm = np.array(new_positions)          # Nx3, mm
                centroid_mm = positions_mm.mean(axis=0)

                current_centered = positions_mm - centroid_mm
                R = self._kabsch_rotation(shape['reference_shape'], current_centered)
                quat = self._rotation_matrix_to_quat(R)

                corrected_pos = (centroid_mm / 1000.0) - offset
                raw_m = (positions_mm / 1000.0).tolist()

                shape['callback']({
                    'frame_id': frame_count,
                    'tvec': corrected_pos.tolist(),
                    'quat': quat,
                    'raw': raw_m,
                    'noise': offset.tolist(),
                    'time': now
                })

    def run(self):
        logger.info(f"Connecting to {self.mocap_system_type} at {self.host_name} in {self.mode} mode...")
        try:
            mc = motioncapture.connect(self.mocap_system_type, {'hostname': self.host_name})
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            self.running = False
            return

        frame_count = 0
        while self.running:
            try:
                mc.waitForNextFrame()
            except Exception as e:
                logger.warning(f"Frame wait error: {e}")
                continue

            now = time.time()

            # Grab references once per frame
            with self._write_lock:
                anchor = self.anchor_data
                current_targets_rb = list(self.objects_to_track)
                current_targets_pc = list(self.points_to_track)
                current_targets_ms = list(self.shapes_to_track)

            # The loop logic is now clean and modular
            cloud_arr = self._get_pointcloud_array(mc)
            current_noise_offset = self._calculate_noise_offset(mc, anchor, cloud_arr)
            self._process_rigid_bodies(mc, current_targets_rb, current_noise_offset, frame_count, now)
            self._process_point_clouds(cloud_arr, current_targets_pc, current_noise_offset, frame_count, now)
            self._process_marker_shapes(cloud_arr, current_targets_ms, current_noise_offset, frame_count, now)

            frame_count += 1

    def set_anchor_rigidbody(self, obj_name):
        with self._write_lock:
            self.anchor_data = {
                'type': 'rigidbody',
                'name': obj_name,
                'initial_pos': None,
                'offset': np.array([0.0, 0.0, 0.0])
            }

    def set_anchor_point(self, initial_point, max_distance=0.05):
        with self._write_lock:
            self.anchor_data = {
                'type': 'pointcloud',
                'initial_pos': np.array(initial_point, dtype=float) * 1000,
                'current_pos': np.array(initial_point, dtype=float) * 1000,
                'max_dist_sq': (max_distance * 1000) ** 2,
                'captured': False,
                'offset': np.array([0.0, 0.0, 0.0])
            }

    def subscribe_object(self, obj_name, callback):
        """Subscribe to a named rigid body (RigidBody Mode)."""
        with self._write_lock:
            new_list = list(self.objects_to_track)
            new_list.append((obj_name, callback))
            self.objects_to_track = new_list

    def unsubscribe_object(self, obj_name):
        with self._write_lock:
            new_list = [obj for obj in self.objects_to_track if obj[0] != obj_name]
            self.objects_to_track = new_list

    def subscribe_point(self, initial_point, callback, name=None, max_distance=0.05):
        """
        Subscribe to a specific point in the pointcloud (Pointcloud Mode).

        Args:
            initial_point (list/array): [x, y, z] coordinates of the point to start tracking.
            callback (func): Function to call with frame data.
            name (str, optional): Unique identifier for this point to allow unsubscribing.
            max_distance (float): Maximum allowed distance (units, e.g. mm) the point can move
                                  between frames before being considered "lost".
        """
        with self._write_lock:
            # We store a mutable dictionary for each tracked point.
            # 'current_pos' is updated by the run loop to follow the marker.
            pt_data = {
                'name': name,
                'initial_pos': np.array(initial_point, dtype=float) * 1000,
                'current_pos': np.array(initial_point, dtype=float) * 1000,
                'max_dist_sq': (max_distance * 1000) ** 2,
                'captured': False,
                'callback': callback
            }
            self.points_to_track.append(pt_data)

    def unsubscribe_point(self, name):
        """
        Unsubscribe a point by its assigned name.
        """
        with self._write_lock:
            # Remove all points that match the given name
            self.points_to_track = [pt for pt in self.points_to_track if pt.get('name') != name]

    def subscribe_marker_shape(self, marker_points, callback, name=None, max_distance=0.05):
        """
        Track a rigid body defined by N markers (typically 4) in the point cloud.
        Computes centroid position (tvec) and rotation (quat) via the Kabsch algorithm,
        plus the raw positions of all markers.

        Args:
            marker_points (list): List of N [x, y, z] initial marker positions (meters).
            callback (func): Called each frame with:
                {
                  'frame_id': int,
                  'tvec':  [x, y, z],       # centroid, offset-corrected, meters
                  'quat':  [x, y, z, w],    # rotation relative to initial shape
                  'raw':   [[x,y,z], ...],  # raw positions of each marker, meters
                  'noise': [x, y, z],
                  'time':  float
                }
            name (str, optional): Identifier for unsubscribing.
            max_distance (float): Max per-marker jump between frames (meters).
        """
        points_mm = np.array(marker_points, dtype=float) * 1000.0
        centroid_mm = points_mm.mean(axis=0)
        reference_shape = points_mm - centroid_mm  # Nx3, centroid-centered

        shape_data = {
            'name': name,
            'current_positions': [points_mm[i].copy() for i in range(len(points_mm))],
            'reference_shape': reference_shape,
            'max_dist_sq': (max_distance * 1000.0) ** 2,
            'captured': [False] * len(points_mm),
            'callback': callback
        }
        with self._write_lock:
            self.shapes_to_track.append(shape_data)

    def unsubscribe_marker_shape(self, name):
        """Unsubscribe a marker shape by its assigned name."""
        with self._write_lock:
            self.shapes_to_track = [s for s in self.shapes_to_track if s.get('name') != name]


if __name__ == "__main__":
    # Example usage:
    # python mocap_extended.py --mode pointcloud --point 100 200 50 --max-dist 100

    ap = argparse.ArgumentParser()
    ap.add_argument("-t", default=140, type=int, help="duration")
    ap.add_argument("--mode", default="rigidbody", choices=["rigidbody", "pointcloud"], help="Tracking mode")

    # Rigidbody args
    ap.add_argument("--obj-name", type=str, help="Rigid body name")

    # Pointcloud args
    ap.add_argument("--point", type=float, nargs=3, help="Initial point x y z", default=[0.0, 0.0, 0.0])
    ap.add_argument("--max-dist", type=float, default=0.1, help="Max tracking jump distance")

    args = ap.parse_args()

    mw = Mocap(mode=args.mode)
    mw.start()

    if args.mode == "rigidbody":
        if args.obj_name:
            print(f"Subscribing to RigidBody: {args.obj_name}")
            mw.subscribe_object(args.obj_name, lambda frame: print(f"RB: {frame['tvec']}"))
        else:
            print("Warning: Mode is rigidbody but no --obj-name provided.")

    elif args.mode == "pointcloud":
        print(f"Subscribing to Point closest to: {args.point} (Max Jump: {args.max_dist})")
        mw.subscribe_point(
            args.point,
            lambda frame: print(f"PT: {[round(p, 3) for p in frame['tvec']]} (Error^2: {frame['dist_sq']:.2f})"),
            name="my_point",
            max_distance=args.max_dist
        )

    try:
        time.sleep(args.t)
    except KeyboardInterrupt:
        pass
    finally:
        mw.stop()
        print("Stopped.")