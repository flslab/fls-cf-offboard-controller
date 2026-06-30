import numpy as np
from scipy.spatial.transform import Rotation as R

# ==========================================
# Static Constants (Calculated once)
# ==========================================
# Rotation from drone to camera (R_d_c)
R_d_c = np.array([
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, -1]
])

# Global variable to store the most recently detected marker position [x, y, z]
# This should be updated asynchronously by your vision/tracking thread
latest_marker_position_c = None 

# ==========================================
# Real-time Callback
# ==========================================
def imu_callback(quat_x, quat_y, quat_z, quat_w, latest_marker_position_c):
    """
    Triggered by the flight controller when new IMU data is available.
    Expects quaternion representing rotation from world to drone (R_w_d).
    """
    # 1. Create rotation object from quaternion (Assuming scalar-last [x,y,z,w])
    rot_w_d = R.from_quat([-quat_x, -quat_y, quat_z, quat_w])
    R_w_d_mat = rot_w_d.as_matrix()

    # 2. Compute Camera orientation in world frame (R_w_c)
    R_w_c_mat = R_w_d_mat @ R_d_c
    
    # 3. Calculate Camera position in world frame (p_w_c)
    p_c_m = np.array(latest_marker_position_c)
    
    # v_w is the vector from camera to marker in world frame
    v_w = R_w_c_mat @ p_c_m
    
    # The marker is at the origin (0,0,0) of the world, so camera position is just the inverse of the vector
    p_w_c = -v_w
    
    # 4. Extract Final World-Frame Pose
    rot_w_c = R.from_matrix(R_w_c_mat)
    
    camera_pos_world = p_w_c
    # camera_rot_world = rot_w_c.as_rotvec() # Use .as_quat() or .as_euler('ZYX') if you prefer
    
    return camera_pos_world
    # TODO: Pipe camera_pos_world and camera_rot_world to your control loop
    # print(f"World Pose | Pos: {camera_pos_world} | Rot: {camera_rot_world}")

