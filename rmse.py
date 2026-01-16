import matplotlib as mpl
mpl.use("macosx")

import json
import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d


# ==========================================
# 1. KINEMATICS & GEOMETRY
# ==========================================

def get_led_local_positions(rod_angle_1_deg, rod_angle_2_deg):
    """
    Computes the positions of all 50 LEDs in the DRONE BODY frame.

    Args:
        rod_angle_1_deg: Angle of rod 1 in degrees.
        rod_angle_2_deg: Angle of rod 2 in degrees.

    Returns:
        np.array of shape (50, 3) representing [x, y, z] of each LED.
    """
    leds = []

    # Constants
    SPACING = 0.006  # 6mm

    # --- GEOMETRY DEFINITION ---
    # Constraint 1: Rod rotation axis is X axis (Front).
    # Constraint 2: Rods are perpendicular to rotation axis (in Y-Z plane).
    # Constraint 3: 0 deg = 3 o'clock.
    # Constraint 4: 90 deg = 6 o'clock.
    #
    # Frame Assumption (Standard Drone): X-Forward, Y-Left, Z-Up.
    # View: From Front (+X) looking at the drone (towards origin).
    #
    # MAPPING:
    # Viewer's perspective looking at drone:
    # - Viewer's Right is Drone's Left (+Y).
    # - Viewer's Bottom is Drone's Down (-Z).
    #
    # "0 deg is 3 o'clock" -> 3 o'clock is to the Viewer's Right.
    # Therefore, 0 deg aligns with Drone +Y axis.
    #
    # "90 deg is 6 o'clock" -> 6 o'clock is Bottom.
    # Therefore, 90 deg aligns with Drone -Z axis.
    #
    # Math:
    # We need a rotation such that:
    # theta=0  => y=r, z=0
    # theta=90 => y=0, z=-r
    #
    # Formula:
    # y = r * cos(theta)
    # z = -r * sin(theta)

    # --- ROD 1 (LEDs 0 to 25) ---
    # LED 25 is at center (r=0). LED 0 is furthest out.
    theta1_rad = np.radians(rod_angle_1_deg)
    c1 = np.cos(theta1_rad)
    s1 = -np.sin(theta1_rad)  # Note the negative sign for CW rotation logic

    for i in range(26):
        r = (25 - i) * SPACING
        y = r * c1
        z = r * s1
        leds.append([0.0, y, z])

    # --- ROD 2 (LEDs 26 to 49) ---
    # LED 26 is 6mm away (1 unit). LED 49 is furthest.
    theta2_rad = np.radians(rod_angle_2_deg)
    c2 = np.cos(theta2_rad)
    s2 = -np.sin(theta2_rad)

    for i in range(26, 50):
        r = (i - 25) * SPACING
        y = r * c2
        z = r * s2
        leds.append([0.0, y, z])

    return np.array(leds)


def transform_points(points_body, drone_pos, drone_rpy_rad):
    """
    Transforms points from Body Frame to World Frame.
    """
    r = R.from_euler('xyz', drone_rpy_rad, degrees=False)
    points_world = r.apply(points_body) + drone_pos
    return points_world


def set_axes_equal(ax, points):
    """
    Sets the 3D axes to have equal aspect ratio based on the data bounds.
    """
    x_limits = [np.min(points[:, 0]), np.max(points[:, 0])]
    y_limits = [np.min(points[:, 1]), np.max(points[:, 1])]
    z_limits = [np.min(points[:, 2]), np.max(points[:, 2])]

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


# ==========================================
# 2. DATA LOADING & PARSING
# ==========================================

def load_ground_truth_trajectory(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    drone_data = data['drones']['lb2']
    waypoints = np.array(drone_data['waypoints'])  # [x, y, z, yaw, dt]
    servos = np.array(drone_data['servos'])  # [rod1, rod2]

    times = [0.0]
    for i in range(1, len(waypoints)):
        dt = waypoints[i, 4] if len(waypoints[i]) == 5 else drone_data['delta_t']
        times.append(times[-1] + dt)

    times = np.array(times)

    interp_pos = interp1d(times, waypoints[:, 0:3], axis=0, kind='linear', fill_value="extrapolate")
    unwrapped_yaw = np.unwrap(np.radians(waypoints[:, 3]))
    interp_yaw = interp1d(times, unwrapped_yaw, kind='linear', fill_value="extrapolate")

    # Use raw servo values in degrees as requested
    interp_servos = interp1d(times, servos, axis=0, kind='linear', fill_value="extrapolate")

    return interp_pos, interp_yaw, interp_servos, times[-1]


def load_actual_data(json_path):
    with open(json_path, 'r') as f:
        data = json.load(f)

    start_time = data['start_time']
    stop_time = data['stop_time']

    vicon_times = []
    vicon_pos = []

    for frame in data['frames']:
        t = frame['time']
        if start_time <= t <= stop_time:
            vicon_times.append(t)
            vicon_pos.append(frame['tvec'])

    vicon_times = np.array(vicon_times)
    vicon_pos = np.array(vicon_pos)

    ekf_time = np.array(data['cf']['time'])
    roll = np.array(data['cf']['params']['stateEstimate.roll']['data'])
    pitch = np.array(data['cf']['params']['stateEstimate.pitch']['data'])
    yaw = np.array(data['cf']['params']['stateEstimate.yaw']['data'])

    r_interp = interp1d(ekf_time, np.radians(roll), fill_value="extrapolate")
    p_interp = interp1d(ekf_time, np.radians(pitch), fill_value="extrapolate")
    y_interp = interp1d(ekf_time, np.radians(yaw), fill_value="extrapolate")

    return vicon_times, vicon_pos, r_interp, p_interp, y_interp, start_time


# ==========================================
# 3. MAIN ANALYSIS
# ==========================================

def calculate_rmse(json_file, yaml_file):
    print(f"Loading Actual Data from {json_file}...")
    act_times, act_pos, r_int, p_int, y_int, t_start_global = load_actual_data(json_file)

    print(f"Loading Ground Truth from {yaml_file}...")
    gt_pos_fn, gt_yaw_fn, gt_servo_fn, gt_duration = load_ground_truth_trajectory(yaml_file)

    errors_sq = []
    errors_per_frame = []
    timestamps = []

    all_gt_leds = []
    all_act_leds = []

    print("Computing Frame-by-Frame RMSE...")

    for i, t_global in enumerate(act_times):
        # 1. Relative Time for GT lookup
        t_rel = t_global - t_start_global

        # Guard: If Vicon data extends beyond GT definition
        if t_rel < 0 or t_rel > gt_duration:
            continue

        timestamps.append(t_rel)

        # --- PREPARE GROUND TRUTH STATE ---
        g_pos = gt_pos_fn(t_rel)
        g_yaw = gt_yaw_fn(t_rel)  # radians

        g_servos = gt_servo_fn(t_rel)  # Raw degrees directly from interpolator

        g_rpy = [0.0, 0.0, float(g_yaw)]

        # --- PREPARE ACTUAL STATE ---
        a_pos = act_pos[i]
        a_rpy = [r_int(t_global), p_int(t_global), y_int(t_global)]

        # Actual uses GT servos as per prompt
        a_servos = g_servos

        # --- COMPUTE LED POSITIONS ---
        leds_local = get_led_local_positions(g_servos[0], g_servos[1])

        leds_gt_world = transform_points(leds_local, g_pos, g_rpy)
        leds_act_world = transform_points(leds_local, a_pos, a_rpy)

        # --- ERROR CALCULATION ---
        diff = leds_gt_world - leds_act_world
        dist_sq = np.sum(diff ** 2, axis=1)

        errors_sq.append(dist_sq)
        frame_mse = np.mean(dist_sq)
        errors_per_frame.append(np.sqrt(frame_mse))

        all_gt_leds.append(leds_gt_world)
        all_act_leds.append(leds_act_world)

    # Convert to arrays
    errors_sq = np.array(errors_sq)
    errors_per_frame = np.array(errors_per_frame)
    timestamps = np.array(timestamps)

    total_mse = np.mean(errors_sq)
    total_rmse = np.sqrt(total_mse)

    max_error = np.max(errors_per_frame)
    max_error_idx = np.argmax(errors_per_frame)
    max_error_time = timestamps[max_error_idx]

    # Unit Conversions (Meters -> mm)
    total_rmse_mm = total_rmse * 1000
    max_error_mm = max_error * 1000
    errors_per_frame_mm = errors_per_frame * 1000

    print(f"\n=== RESULTS ===")
    print(f"Total Frames Processed: {len(timestamps)}")
    print(f"Overall LED RMSE: {total_rmse_mm:.4f} mm")
    print(f"Max Instantaneous RMSE: {max_error_mm:.4f} mm at t={max_error_time:.2f}s")

    # Calculate global bounds for consistent plotting
    all_pts_concat = np.vstack(all_gt_leds + all_act_leds)

    # ==========================================
    # 4. PLOTTING
    # ==========================================

    # --- STATIC PLOTS ---
    fig = plt.figure(figsize=(18, 6))

    # Plot 1: RMSE over Time (in mm)
    ax1 = fig.add_subplot(1, 3, 1)
    ax1.plot(timestamps, errors_per_frame_mm, label='RMSE')
    ax1.set_title('LED Position RMSE over Time')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Error (mm)')
    ax1.grid(True)
    ax1.axhline(total_rmse_mm, color='r', linestyle='--', label=f'Overall: {total_rmse_mm:.3f}mm')
    ax1.legend()

    # Plot 2: 3D Swath (Meters kept for World Context)
    ax2 = fig.add_subplot(1, 3, 2, projection='3d')
    subsample_step = 10

    for i in range(0, len(all_gt_leds), subsample_step):
        gt_frame = all_gt_leds[i]
        act_frame = all_act_leds[i]
        ax2.scatter(gt_frame[:, 0], gt_frame[:, 1], gt_frame[:, 2], s=1, c='b', alpha=0.05)
        ax2.scatter(act_frame[:, 0], act_frame[:, 1], act_frame[:, 2], s=1, c='r', alpha=0.05)

    ax2.set_title('LED Swath (Blue=GT, Red=Act) [m]')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_zlabel('Z (m)')
    set_axes_equal(ax2, all_pts_concat)

    # Plot 3: Error Histogram (Distance in mm)
    ax3 = fig.add_subplot(1, 3, 3)
    # Get vector difference for max error frame
    diff_vecs = all_gt_leds[max_error_idx] - all_act_leds[max_error_idx]  # (50, 3) in meters
    # Calculate Euclidean distance for each LED
    dists_m = np.linalg.norm(diff_vecs, axis=1)  # (50,) in meters
    dists_mm = dists_m * 1000  # Convert to mm

    ax3.hist(dists_mm, bins=15, edgecolor='black', alpha=0.7)
    ax3.set_title(f'LED Error Dist. @ Max Error (t={max_error_time:.2f}s)')
    ax3.set_xlabel('Distance Error (mm)')
    ax3.set_ylabel('Count (LEDs)')

    plt.tight_layout()
    plt.show(block=False)

    # ==========================================
    # 5. INTERACTIVE ANIMATION
    # ==========================================
    print("\nPreparing Interactive Animation...")
    fig_anim = plt.figure(figsize=(10, 8))
    ax_anim = fig_anim.add_subplot(111, projection='3d')
    plt.subplots_adjust(bottom=0.25)  # Make room for widgets

    # Set fixed axis limits for the animation using the helper
    set_axes_equal(ax_anim, all_pts_concat)
    ax_anim.set_xlabel('X (m)')
    ax_anim.set_ylabel('Y (m)')
    ax_anim.set_zlabel('Z (m)')
    ax_anim.set_title("LED Trajectory Replay")

    # Initialize scatters
    scatter_gt = ax_anim.scatter([], [], [], c='blue', marker='o', s=20, label='Ground Truth')
    scatter_act = ax_anim.scatter([], [], [], c='red', marker='^', s=20, label='Actual')
    time_text = ax_anim.text2D(0.05, 0.95, "", transform=ax_anim.transAxes)
    ax_anim.legend()

    # --- CONTROLS ---
    ax_slider = plt.axes([0.2, 0.1, 0.65, 0.03])
    slider = Slider(ax_slider, 'Time', 0, len(timestamps) - 1, valinit=0, valstep=1)

    ax_play = plt.axes([0.05, 0.1, 0.1, 0.04])
    btn_play = Button(ax_play, 'Pause')

    class AnimControl:
        def __init__(self):
            self.is_playing = True

        def toggle(self, event):
            if self.is_playing:
                ani.event_source.stop()
                btn_play.label.set_text('Play')
                self.is_playing = False
            else:
                ani.event_source.start()
                btn_play.label.set_text('Pause')
                self.is_playing = True

    ctrl = AnimControl()
    btn_play.on_clicked(ctrl.toggle)

    def update_plot(frame_idx):
        idx = int(frame_idx)
        # Update GT
        gt_pts = all_gt_leds[idx]
        scatter_gt._offsets3d = (gt_pts[:, 0], gt_pts[:, 1], gt_pts[:, 2])
        # Update Act
        act_pts = all_act_leds[idx]
        scatter_act._offsets3d = (act_pts[:, 0], act_pts[:, 1], act_pts[:, 2])
        # Update Text
        time_text.set_text(f"Time: {timestamps[idx]:.3f}s")
        # Sync slider without triggering callback loop (handled by FuncAnimation usually)
        # But here we want the slider to reflect the animation state
        # We can't update slider value inside here if slider calls this function
        # (circular dependency).
        # Strategy: The slider calls this. The animation ALSO calls this indirectly.
        pass

    def on_slider_change(val):
        update_plot(val)
        # If manual seek, redraw
        fig_anim.canvas.draw_idle()

    slider.on_changed(on_slider_change)

    def animate_step(i):
        # Update slider value, which triggers on_slider_change, which updates plot
        # We temporarily disconnect callback to avoid recursion if needed,
        # but simpler is just to update plot directly here and update slider visually.
        slider.set_val(i)
        return scatter_gt, scatter_act, time_text

    # Interval: 20ms
    ani = animation.FuncAnimation(fig_anim, animate_step, frames=len(timestamps), interval=20, blit=False)

    plt.show()


if __name__ == "__main__":
    # File paths (adjust if needed)
    json_path = 'logs/lb2_log_test_keyframe_linear_2026-01-15_14-54-28.json'
    yaml_path = 'mission/led_test_moving.yaml'

    calculate_rmse(json_path, yaml_path)