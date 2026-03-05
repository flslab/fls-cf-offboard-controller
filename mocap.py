import argparse
import threading
import time
import logging
import motioncapture
import numpy as np

logger = logging.getLogger(__name__)


class Mocap(threading.Thread):
    def __init__(self, mocap_system_type="vicon", host_name="vicon", mode="rigidbody"):
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

        # Lock is ONLY for writers (subscribe/unsubscribe), not the reader (run)
        self._write_lock = threading.Lock()

    def stop(self):
        self.running = False
        self.join()

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

            # --- Rigid Body Mode ---
            if self.mode == "rigidbody":
                # Grab reference to list (thread-safe copy logic in subscribe handles the writer side)
                current_targets = self.objects_to_track

                # Iterate over all rigid bodies provided by the SDK
                for name, obj in mc.rigidBodies.items():
                    for obj_name, callback in current_targets:
                        if name == obj_name:
                            pos = obj.position
                            quat = obj.rotation
                            if callback:
                                callback({
                                    "frame_id": frame_count,
                                    "tvec": [float(pos[0]), float(pos[1]), float(pos[2])],
                                    "quat": [float(quat.x), float(quat.y), float(quat.z), float(quat.w)],
                                    "time": now
                                })

            # --- Point Cloud Mode ---
            elif self.mode == "pointcloud":
                if hasattr(mc, 'pointCloud') and mc.pointCloud is not None:
                    # 1. Convert SDK PointCloud to Numpy Array (N x 3)
                    # C++ Type: Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>
                    # This maps efficiently to a numpy array, often allowing zero-copy access.
                    try:
                        # copy=False attempts to use the existing buffer without duplication
                        cloud_arr = np.array(mc.pointCloud, copy=False) * 1000
                    except (ValueError, TypeError):
                        # Fallback if the binding returns a list or requires a copy
                        cloud_arr = np.array(mc.pointCloud) * 1000

                    if len(cloud_arr) > 0:
                        # We only lock briefly to get the reference to our tracking list
                        with self._write_lock:
                            current_points = list(self.points_to_track)

                        for pt_data in current_points:
                            target_pos = pt_data['current_pos']

                            # --- Efficient Nearest Neighbor Search ---
                            # Vectorized calculation of Squared Euclidean Distance
                            # (x-x0)^2 + (y-y0)^2 + (z-z0)^2
                            diff = cloud_arr - target_pos

                            # Einstein summation is often faster/cleaner for dot products on rows
                            # Equivalent to: np.sum(diff**2, axis=1)
                            dist_sq = np.einsum('ij,ij->i', diff, diff)

                            # Find index of the minimum distance
                            min_idx = np.argmin(dist_sq)
                            min_dist_sq = dist_sq[min_idx]

                            # SAFETY GUARD: Only update if the point is within the allowed radius.
                            # If min_dist_sq is too large, it implies the point is occluded or
                            # not published, and the "closest" point is actually just some other random marker.
                            max_dist_sq = pt_data['max_dist_sq'] if pt_data['captured'] else pt_data['max_dist_sq'] * 4
                            if min_dist_sq <= max_dist_sq:
                                pt_data['captured'] = True
                                closest_point = cloud_arr[min_idx]

                                # Update 'current_pos' to the new found point.
                                # This allows us to "track" the point as it moves across the volume.
                                pt_data['current_pos'] = closest_point

                                if pt_data['callback']:
                                    pt_data['callback']({
                                        "frame_id": frame_count,
                                        "tvec": (closest_point / 1000).tolist(),
                                        "dist_sq": float(min_dist_sq),
                                        "time": now
                                    })
                            # else:
                            #   Point lost/occluded. We keep 'current_pos' at the last valid location
                            #   so we can re-acquire the point when it reappears nearby.

            frame_count += 1

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