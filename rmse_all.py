import matplotlib
matplotlib.use('macosx') # Commented out to prevent errors in non-Mac environments, uncomment if needed.


import json
import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d
import glob
import os


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
    # Math:
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
    if plot_radius == 0: plot_radius = 1.0

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


# ==========================================
# 2. DATA PROCESSING CLASSES
# ==========================================

class DroneProcessor:
    def __init__(self, drone_id, yaml_config, json_path):
        self.drone_id = drone_id
        self.yaml_config = yaml_config
        self.json_path = json_path

        # Load Interpolators
        self._load_gt()
        self._load_act()

    def _load_gt(self):
        waypoints = self.yaml_config['waypoints']
        if not len(waypoints):
            waypoints.append(self.yaml_config['target'])
            waypoints.append(self.yaml_config['target'])
        waypoints = np.array(waypoints)  # [x, y, z, yaw, dt]
        servos = np.array(self.yaml_config['servos'])  # [rod1, rod2]

        times = [0.0]
        for i in range(1, len(waypoints)):
            dt = waypoints[i, 4] if len(waypoints[i]) == 5 else self.yaml_config['delta_t']
            times.append(times[-1] + dt)

        self.gt_times = np.array(times)
        self.gt_duration = times[-1]

        self.gt_pos_fn = interp1d(self.gt_times, waypoints[:, 0:3], axis=0, kind='linear', fill_value="extrapolate")
        unwrapped_yaw = np.unwrap(np.radians(waypoints[:, 3]))
        self.gt_yaw_fn = interp1d(self.gt_times, unwrapped_yaw, kind='linear', fill_value="extrapolate")

        # Use raw servo values in degrees
        self.gt_servo_fn = interp1d(self.gt_times, servos, axis=0, kind='linear', fill_value="extrapolate")

    def _load_act(self):
        with open(self.json_path, 'r') as f:
            data = json.load(f)

        self.start_time = data['start_time']
        self.stop_time = data['stop_time']

        vicon_times = []
        vicon_pos = []

        for frame in data['frames']:
            t = frame['time']
            if self.start_time <= t <= self.stop_time:
                vicon_times.append(t)
                vicon_pos.append(frame['tvec'])

        vicon_times = np.array(vicon_times)
        vicon_pos = np.array(vicon_pos)

        # Convert Vicon to interpolator for easy synchronization relative to start_time
        rel_times = vicon_times - self.start_time
        self.act_pos_fn = interp1d(rel_times, vicon_pos, axis=0, kind='linear', fill_value="extrapolate",
                                   bounds_error=False)

        ekf_time = np.array(data['cf']['time'])
        ekf_rel_times = ekf_time - self.start_time

        roll = np.array(data['cf']['params']['stateEstimate.roll']['data'])
        pitch = np.array(data['cf']['params']['stateEstimate.pitch']['data'])
        yaw = np.array(data['cf']['params']['stateEstimate.yaw']['data'])

        self.act_r_fn = interp1d(ekf_rel_times, np.radians(roll), fill_value="extrapolate")
        self.act_p_fn = interp1d(ekf_rel_times, np.radians(pitch), fill_value="extrapolate")
        self.act_y_fn = interp1d(ekf_rel_times, np.radians(yaw), fill_value="extrapolate")

        self.act_max_rel_time = rel_times[-1]

    def get_state_at_relative_time(self, t_rel):
        """
        Returns (GT_LEDs, Act_LEDs, Valid_Bool)
        t_rel: Time in seconds relative to the mission start (yaml t=0, json t=start_time)
        """
        # Validity check: Must be within GT definition and have actual data
        if t_rel < 0 or t_rel > self.gt_duration or t_rel > self.act_max_rel_time:
            return None, None, False

        # --- Ground Truth State ---
        g_pos = self.gt_pos_fn(t_rel)
        g_yaw = self.gt_yaw_fn(t_rel)  # radians
        g_servos = self.gt_servo_fn(t_rel)  # degrees
        g_rpy = [0.0, 0.0, float(g_yaw) * 180 / np.pi]

        # print(g_rpy)

        # --- Actual State ---
        a_pos = self.act_pos_fn(t_rel)
        if np.any(np.isnan(a_pos)): return None, None, False

        a_rpy = [self.act_r_fn(t_rel), self.act_p_fn(t_rel), self.act_y_fn(t_rel)]

        # --- LED Computation ---
        # Note: Actual uses GT servo angles as per prompt requirements
        leds_local = get_led_local_positions(g_servos[0], g_servos[1])

        leds_gt_world = transform_points(leds_local, g_pos, g_rpy)
        leds_act_world = transform_points(leds_local, a_pos, a_rpy)

        return leds_gt_world, leds_act_world, True


# ==========================================
# 3. MAIN ANALYSIS LOGIC
# ==========================================

def calculate_rmse(yaml_file, tag):
    print(f"Loading Configuration from {yaml_file}...")
    with open(yaml_file, 'r') as f:
        yaml_data = yaml.safe_load(f)

    drones_config = yaml_data.get('drones', {})
    processors = []

    # 1. Initialize Processors (Find logs and parse)
    for drone_id, config in drones_config.items():
        # Search for log file starting with drone_id
        search_pattern = f"logs/{drone_id}_{tag}*.json"
        files = glob.glob(search_pattern)

        if not files:
            # Fallback to current directory for user testing if logs/ doesn't exist or is empty
            search_pattern_local = f"{drone_id}_{tag}*.json"
            files = glob.glob(search_pattern_local)

        if not files:
            print(f"WARNING: No log file found for drone '{drone_id}' (Pattern: {search_pattern}). Skipping.")
            continue

        # Pick the first match (assuming one log per drone in directory)
        json_file = files[0]
        print(f"Found log for {drone_id}: {json_file}")

        try:
            p = DroneProcessor(drone_id, config, json_file)
            processors.append(p)
        except Exception as e:
            print(f"Error loading data for {drone_id}: {e}")

    if not processors:
        print("No valid drone data found.")
        return

    # 2. Define Master Timeline
    # We want to cover the extent of the longest GT plan
    max_duration = max([p.gt_duration for p in processors])

    # Sampling rate for analysis (100Hz)
    dt_analysis = 0.01
    timestamps = np.arange(0, max_duration, dt_analysis)

    results = {
        'timestamps': timestamps,
        'combined_rmse': [],
        'drones': {p.drone_id: {'rmse': [], 'leds_gt': [], 'leds_act': []} for p in processors}
    }

    print(f"Analyzing {len(processors)} drone(s) over {max_duration:.2f}s...")

    # 3. Time Loop
    for t in timestamps:
        frame_total_sse = 0
        frame_total_count = 0

        # Iterate over all drones for this specific time step
        for p in processors:
            gt_leds, act_leds, valid = p.get_state_at_relative_time(t)

            if valid:
                diff = gt_leds - act_leds
                sse = np.sum(diff ** 2)
                count = len(gt_leds)  # 50

                # Per drone metrics
                rmse_val = np.sqrt(sse / count) * 1000.0  # mm

                results['drones'][p.drone_id]['rmse'].append(rmse_val)

                # Store subsample for vis (every 10th step)
                if len(results['drones'][p.drone_id]['rmse']) % 10 == 0:
                    results['drones'][p.drone_id]['leds_gt'].append(gt_leds)
                    results['drones'][p.drone_id]['leds_act'].append(act_leds)

                # Accumulate for Combined Metric
                frame_total_sse += sse
                frame_total_count += count
            else:
                # Store NaN to keep array length consistent with timestamps
                results['drones'][p.drone_id]['rmse'].append(np.nan)

                if len(results['drones'][p.drone_id]['rmse']) % 10 == 0:
                    results['drones'][p.drone_id]['leds_gt'].append(None)
                    results['drones'][p.drone_id]['leds_act'].append(None)

        # Compute Combined RMSE for this frame (if any drone was active)
        if frame_total_count > 0:
            comb_rmse = np.sqrt(frame_total_sse / frame_total_count) * 1000.0  # mm
            results['combined_rmse'].append(comb_rmse)
        else:
            results['combined_rmse'].append(np.nan)

    # 4. Compute and Print Overall Stats
    print("\n=== RESULTS ===")

    comb_arr = np.array(results['combined_rmse'])
    valid_comb = ~np.isnan(comb_arr)

    if np.any(valid_comb):
        # Overall RMSE across entire trajectory
        overall_comb_rmse = np.sqrt(np.mean(comb_arr[valid_comb] ** 2))
        max_comb_rmse = np.nanmax(comb_arr)
        print(f"COMBINED (All Drones): Overall RMSE {overall_comb_rmse:.2f} mm")
        print(f"COMBINED (All Drones): Max RMSE {max_comb_rmse:.2f} mm")
    else:
        print("COMBINED: No valid data.")

    for p in processors:
        d_rmse = np.array(results['drones'][p.drone_id]['rmse'])
        valid = ~np.isnan(d_rmse)
        if np.any(valid):
            # This is 'Average of RMSEs' which is slightly different from 'RMSE of all points',
            # but standard for reporting time-series performance.
            max_d = np.nanmax(d_rmse)
            mean_d = np.nanmean(d_rmse)
            print(f"Drone {p.drone_id}: Max RMSE {max_d:.2f} mm, Mean RMSE {mean_d:.2f} mm")

        # ==========================================
        # 5. EXPORT DATA TO JSON
        # ==========================================
        output_filename = f"{tag}_rmse_analysis_output.json"
        print(f"\nExporting raw data to {output_filename}...")

        # Structure data for export (handle numpy types)
        export_data = {
            "timestamps": results['timestamps'].tolist(),
            "combined_rmse_mm": [None if np.isnan(x) else float(x) for x in results['combined_rmse']],
            "drones": {}
        }

        for p in processors:
            d_data = results['drones'][p.drone_id]

            # Helper to clean numpy arrays in LED lists
            def clean_leds(led_list):
                cleaned = []
                for item in led_list:
                    if item is None:
                        cleaned.append(None)
                    else:
                        # Rounding to save space, remove round() if max precision needed
                        cleaned.append(np.round(item, 5).tolist())
                return cleaned

            export_data["drones"][p.drone_id] = {
                "rmse_mm": [None if np.isnan(x) else float(x) for x in d_data['rmse']],
                # These are subsampled (every 10th frame relative to timestamp index)
                "subsampled_gt_leds": clean_leds(d_data['leds_gt']),
                "subsampled_act_leds": clean_leds(d_data['leds_act'])
            }

        with open(output_filename, 'w') as f:
            json.dump(export_data, f, indent=4)
        print("Export complete.")

    # ==========================================
    # 5. VISUALIZATION
    # ==========================================

    fig = plt.figure(figsize=(18, 8))

    # Plot 1: RMSE over Time
    ax1 = fig.add_subplot(1, 2, 1)

    # Plot Combined
    ax1.plot(timestamps, results['combined_rmse'], 'k-', linewidth=3, alpha=0.8, label='All Drones (Combined)')
    ax1.axhline(overall_comb_rmse, color='r', linestyle='--', label=f'Overall: {overall_comb_rmse:.3f}mm')

    # Plot Individuals
    colors = plt.cm.jet(np.linspace(0, 1, len(processors)))
    for i, p in enumerate(processors):
        rmse_data = results['drones'][p.drone_id]['rmse']
        ax1.plot(timestamps, rmse_data, color=colors[i], linewidth=1, label=f'{p.drone_id}')

    ax1.set_title('RMSE over Time (mm)')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Error (mm)')
    ax1.legend()
    ax1.grid(True)

    # Plot 2: 3D Animation
    ax_3d = fig.add_subplot(1, 2, 2, projection='3d')

    # Collect all valid points to set global bounds
    all_vis_pts = []
    for p in processors:
        gts = results['drones'][p.drone_id]['leds_gt']
        acts = results['drones'][p.drone_id]['leds_act']
        valid_pts = [x for x in gts if x is not None] + [x for x in acts if x is not None]
        if valid_pts:
            all_vis_pts.append(np.vstack(valid_pts))

    if all_vis_pts:
        set_axes_equal(ax_3d, np.vstack(all_vis_pts))

    ax_3d.set_xlabel('X (m)')
    ax_3d.set_ylabel('Y (m)')
    ax_3d.set_zlabel('Z (m)')
    ax_3d.set_title('Multi-Drone Replay')

    # Create Scatter Objects
    scatters = {}
    for i, p in enumerate(processors):
        # GT = Solid Circle, Act = Triangle
        sc_gt = ax_3d.scatter([], [], [], color=colors[i], marker='o', s=15, alpha=0.6, label=f'{p.drone_id} GT')
        sc_act = ax_3d.scatter([], [], [], color=colors[i], marker='^', s=15, label=f'{p.drone_id} Act')
        scatters[p.drone_id] = (sc_gt, sc_act)

    ax_3d.legend()

    # --- ANIMATION CONTROLS ---
    plt.subplots_adjust(bottom=0.25)

    # Determine number of frames in the visual subsample
    # Pick the first drone's list length (they should be identical due to uniform loop)
    num_frames = len(results['drones'][processors[0].drone_id]['leds_gt'])

    ax_slider = plt.axes([0.2, 0.1, 0.65, 0.03])
    slider = Slider(ax_slider, 'Frame', 0, num_frames - 1, valinit=0, valstep=1)

    ax_play = plt.axes([0.05, 0.1, 0.1, 0.04])
    btn_play = Button(ax_play, 'Pause')  # Start playing by default

    def update_anim(val):
        idx = int(val)

        # Approximate time for display (since we subsampled by 10)
        t_disp = timestamps[min(idx * 10, len(timestamps) - 1)]
        ax_3d.set_title(f"Multi-Drone Replay t={t_disp:.2f}s")

        for p in processors:
            gt_list = results['drones'][p.drone_id]['leds_gt']
            act_list = results['drones'][p.drone_id]['leds_act']

            s_gt, s_act = scatters[p.drone_id]

            if idx < len(gt_list) and gt_list[idx] is not None:
                gt_pts = gt_list[idx]
                act_pts = act_list[idx]
                s_gt._offsets3d = (gt_pts[:, 0], gt_pts[:, 1], gt_pts[:, 2])
                s_act._offsets3d = (act_pts[:, 0], act_pts[:, 1], act_pts[:, 2])
            else:
                # Hide if invalid for this frame
                s_gt._offsets3d = ([], [], [])
                s_act._offsets3d = ([], [], [])

        fig.canvas.draw_idle()

    slider.on_changed(update_anim)

    class Player:
        def __init__(self):
            self.playing = True
            self.anim = None

        def toggle(self, event):
            if self.playing:
                self.anim.event_source.stop()
                btn_play.label.set_text('Play')
                self.playing = False
            else:
                self.anim.event_source.start()
                btn_play.label.set_text('Pause')
                self.playing = True

    player = Player()
    btn_play.on_clicked(player.toggle)

    def animate_step(i):
        # Update slider which triggers update_anim
        slider.set_val(i)

    player.anim = animation.FuncAnimation(fig, animate_step, frames=num_frames, interval=50, blit=False)

    plt.show()


if __name__ == "__main__":
    # Adjust default file if needed
    yaml_path = 'mission/arrow.yaml'
    # Use glob to find the json file if the hardcoded one doesn't exist to prevent immediate crashes for user
    calculate_rmse(yaml_path, 'arrow_leader_follower_2026-01-16_18-41-27')

