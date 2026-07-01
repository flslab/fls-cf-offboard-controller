import numpy as np
from scipy.spatial.transform import Rotation as R

# ==========================================
# Static Constants (Calculated once)
# ==========================================
# Rotation from drone to camera (R_d_c)
# Camera: x=right, y=down, z=forward (Standard OpenCV camera frame)
# Drone: x=forward, y=left, z=up (FLU drone frame)
R_d_c = np.array([
    [0, -1, 0],
    [-1, 0, 0],
    [0, 0, -1]
])

# Global variable to store the most recently detected marker position [x, y, z]
# This should be updated asynchronously by your vision/tracking thread
latest_marker_position_c = None 

# ==========================================
# Real-time Callbacks
# ==========================================

def imu_callback_quat(quat_x, quat_y, quat_z, quat_w, latest_marker_position_c, marker_world_pos=None, camera_drone_pos=None):
    """
    Triggered when new Quaternion IMU data is available.
    Expects quaternion representing rotation from world to drone (R_w_d).
    Identical transformations to post_process_imu_pose.py
    """
    if marker_world_pos is None:
        marker_world_pos = [0.0, 0.0, 0.0]
    if camera_drone_pos is None:
        camera_drone_pos = [0.0, 0.0, 0.0]

    p_w_m = np.array(marker_world_pos)
    p_d_c = np.array(camera_drone_pos)
    p_c_m = np.array(latest_marker_position_c)

    q = np.array([quat_x, quat_y, quat_z, quat_w])
    q_norm = np.linalg.norm(q)
    if q_norm > 0:
        q = q / q_norm

    rot_w_d = R.from_quat(q)
    R_w_d_mat = rot_w_d.as_matrix()

    # Drone position in world frame
    p_w_d = p_w_m - R_w_d_mat @ (R_d_c @ p_c_m + p_d_c)

    return p_w_d, rot_w_d


def imu_callback_euler(roll, pitch, yaw, latest_marker_position_c, marker_world_pos=None, camera_drone_pos=None):
    """
    Triggered when new Euler IMU data is available (roll, pitch, yaw in degrees).
    Identical transformations to post_process_imu_pose.py (pitch is negated)
    """
    if marker_world_pos is None:
        marker_world_pos = [0.0, 0.0, 0.0]
    if camera_drone_pos is None:
        camera_drone_pos = [0.0, 0.0, 0.0]

    p_w_m = np.array(marker_world_pos)
    p_d_c = np.array(camera_drone_pos)
    p_c_m = np.array(latest_marker_position_c)

    roll_rad = np.radians(roll)
    pitch_rad = np.radians(pitch)
    yaw_rad = np.radians(yaw)

    # In post_process_imu_pose.py, pitch is negated
    rot_w_d = R.from_euler('ZYX', [yaw_rad, -pitch_rad, roll_rad], degrees=False)
    R_w_d_mat = rot_w_d.as_matrix()

    # Drone position in world frame
    p_w_d = p_w_m - R_w_d_mat @ (R_d_c @ p_c_m + p_d_c)

    return p_w_d, rot_w_d


def imu_callback(quat_x, quat_y, quat_z, quat_w, latest_marker_position_c, marker_world_pos=None, camera_drone_pos=None):
    """
    Legacy wrapper for backward compatibility. Returns camera position in world frame.
    """
    if camera_drone_pos is None:
        camera_drone_pos = [0.0, 0.0, 0.0]
    
    p_w_d, rot_w_d = imu_callback_quat(
        quat_x, quat_y, quat_z, quat_w,
        latest_marker_position_c,
        marker_world_pos=marker_world_pos,
        camera_drone_pos=camera_drone_pos
    )
    
    # Camera position in world: p_w_c = p_w_d + R_w_d @ p_d_c
    p_d_c = np.array(camera_drone_pos)
    p_w_c = p_w_d + rot_w_d.as_matrix() @ p_d_c
    return p_w_c

