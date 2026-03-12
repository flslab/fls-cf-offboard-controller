import json
import os
import shutil
import time
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.colors as mcolors
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.widgets import Slider, Button
import config
from matplotlib.animation import FuncAnimation
import cv2
import glob
from tqdm import tqdm
from plots.utils import load_log_data

matplotlib.use("macosx")


def load_cmd_log(file_path, freq=100):
    """
    Parses Crazyflie command logs by mapping 'args' to physical states.
    Separates Position (XYZ) and Orientation (RPY) streams.
    """
    with open(file_path, 'r') as f:
        data = json.load(f)

    # State variables
    curr_pos = np.array([0.0, 0.0, 0.0])
    curr_ori = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw]

    master_t, master_pos, master_ori = [], [], []
    dt = 1.0 / freq
    g = 9.81  # Gravity constant for acceleration

    vel_xy = np.zeros(3)
    for entry in data:
        t1 = entry.get('time', entry.get('t', 0.0))
        cmd = entry.get('command', '')
        args = entry.get('args', [])
        if not args:
            continue
        if cmd != 'Commander.send_zdistance_setpoint':
            vel_xy = np.zeros(3)

        if cmd == 'Commander.send_position_setpoint':
            curr_pos = np.array([args[0], args[1], args[2]])
            curr_ori[2] = args[3]

            master_t.append(t1)
            master_pos.append(curr_pos.copy())
            master_ori.append(curr_ori.copy())

        elif cmd == 'Commander.send_zdistance_setpoint':
            # Update targets
            curr_ori = np.array([args[1], args[0], args[2]])  # Roll, Pitch, Yaw
            curr_pos[2] = args[3]  # Z
            acc_x = g * np.sin(np.deg2rad(curr_ori[1]))
            acc_y = -g * np.sin(np.deg2rad(curr_ori[0]))
            acc = np.array([acc_x, acc_y, 0])
            t_step = t1 + dt

            disp = (vel_xy * dt) + (0.5 * acc * (dt ** 2))
            curr_pos += disp

            vel_xy += acc * dt

            master_t.append(t_step)
            master_pos.append(curr_pos + disp)
            master_ori.append(curr_ori.copy())

        # 3. HIGH LEVEL: Go To Trajectory
        # args: [x, y, z, yaw, duration]
        elif cmd == 'HighLevelCommander.go_to':
            duration = args[4]
            t2 = t1 + duration
            target_pos = np.array([args[0], args[1], args[2]])
            target_yaw = args[3]

            # Interpolate from t1 to t2 at 30Hz
            steps = np.arange(t1, t2, dt)
            for s_t in steps:
                frac = (s_t - t1) / duration if duration > 0 else 1.0
                interp_p = curr_pos + frac * (target_pos - curr_pos)
                interp_y = curr_ori[2] + frac * (target_yaw - curr_ori[2])

                master_t.append(s_t)
                master_pos.append(interp_p)
                master_ori.append(np.array([curr_ori[0], curr_ori[1], interp_y]))

            curr_pos = target_pos  # Update state to destination
            curr_ori[2] = target_yaw

    return np.array(master_t), np.array(master_pos), np.array(master_ori)


def load_interaction_log(file_path, target_name='drone'):
    """
    Separately extracts position and orientation streams.
    Handles partial orientation data (e.g., only Yaw) for sparse logs.
    """
    with open(file_path, 'r') as f:
        logs = json.load(f)

    # Data containers
    pos_t, pos_data = [], []
    ori_t, ori_data = [], []

    # Track last known orientation to handle partial updates
    last_ori = [0.0, 0.0, 0.0]  # [roll, pitch, yaw]

    for entry in logs:
        e_type = entry.get('type')
        # Filter for relevant entries (Mocap or State)
        if e_type in ['frames', 'state'] and entry.get('name', 'drone') == target_name:
            data = entry.get('data', None)
            t = data.get('time') or data.get('t')  # Handle both naming conventions

            if 'tvec' in data:
                pos_t.append(t)
                pos_data.append(data['tvec'])

            # 2. Independent Orientation Loading (Handles partial R/P/Y)
            # Check if at least one orientation key exists
            ori_keys = ['stateEstimate.roll', 'stateEstimate.pitch', 'stateEstimate.yaw']
            if any(k in data for k in ori_keys):
                ori_t.append(t)
                # Update only the provided axes, keep others as last known
                current_ori = [
                    data.get('roll', last_ori[0]),
                    data.get('pitch', last_ori[1]),
                    data.get('yaw', last_ori[2])
                ]
                ori_data.append(current_ori)
                last_ori = current_ori  # Update state for the next partial entry

    return (np.array(pos_t), np.array(pos_data),
            np.array(ori_t), np.array(ori_data))


def drone_body(pos, oriantation, scale=50):
    x, y, z = pos
    roll_deg, pitch_deg, yaw_deg = oriantation
    arm = np.array([
        [scale, scale, 0],
        [-scale, -scale, 0],
        [np.nan, np.nan, np.nan],  # Break the line for a 'X' shape
        [scale, -scale, 0],
        [-scale, scale, 0]
    ])

    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(yaw_deg)

    R_roll = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    R_pitch = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    R_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Combined Rotation: Yaw * Pitch * Roll
    R_total = R_yaw @ R_pitch @ R_roll

    # Apply rotation and translation
    rotated = arm @ R_total.T
    rotated += np.array([x, y, z])

    return rotated[:, 0], rotated[:, 1], rotated[:, 2]


def resample_to_fps(times, data, target_fps=30):
    # Normalize time to start at 0 and calculate duration
    t_start, t_end = times[0], times[-1]
    duration = t_end - t_start
    total_frames = int(duration * target_fps)

    # Create the new uniform timeline
    t_new = np.linspace(t_start, t_end, total_frames)

    # Initialize the resampled data array with the same number of columns
    data_new = np.zeros((total_frames, data.shape[1]))

    # Interpolate each column (X, Y, Z, etc.) individually
    for i in range(data.shape[1]):
        data_new[:, i] = np.interp(t_new, times, data[:, i])
    return t_new, data_new


def save_plot_frames(position, orientation, output_dir='plot_frames', draw_drone_func=drone_body, config=None, transparent=False, granularity='mm'):
    """Generates and saves a sequence of transparent 3D plots for video overlay."""
    scale = 0.03
    if granularity == 'm':
        position = np.array(position)
    elif granularity == 'cm':
        position = np.array(position) * 100
        scale *= 100
    else:
        position = np.array(position) * 1000
        scale *= 1000

    if config is None:
        config = {}
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    else:
        shutil.rmtree(output_dir)
        print(f"Cleared existing frames in {output_dir}.")

        os.makedirs(output_dir)

    fig = plt.figure(figsize=config.get('figsize', (3, 3)), dpi=config.get('dpi', 100))

    ax = fig.add_subplot(111, projection='3d')
    if transparent:
        fig.patch.set_alpha(0.0)
    ax.set_facecolor((0, 0, 0, 0))  # Fully transparent 3D background

    # Extract data for easier indexing
    x, y, z = position[:, 0], position[:, 1], position[:, 2]

    drone_body, = ax.plot([], [], [], color='C1', lw=2, label="LightBender")

    # text = ax.text2D(0.05, 0.85, '', transform=ax.transAxes, color='white',
    #                  fontsize=9, family='monospace', fontweight='bold',
    #                  bbox=dict(facecolor='black', alpha=0.5, edgecolor='none'))

    padding = 0.2
    ax.set_xlim(min(x) - padding, max(x) + padding)
    ax.set_ylim(min(y) - padding, max(y) + padding)
    ax.set_zlim(min(z) - padding, max(z) + padding)
    # ax.axis('off')  # Hide grid and axes for the overlay

    frame_num = min(len(position), len(orientation))
    print(f"Generating {frame_num} frames...")

    for i in tqdm(range(frame_num), desc="Rendering Frames", unit="frame"):
        if draw_drone_func:
            dx, dy, dz = draw_drone_func(position[i], orientation[i],scale=scale)
            drone_body.set_data(dx, dy)
            drone_body.set_3d_properties(dz)
        else:
            drone_body.set_data([x[i]], [y[i]])
            drone_body.set_3d_properties([z[i]])
            drone_body.set_marker('o')

        # text.set_text(f"POS: {x[i]:.2f}, {y[i]:.2f}, {z[i]:.2f}\n"
        #               f"YAW: {yaw[i]:.1f}°")

        plt.savefig(f"{output_dir}/frame_{i:04d}.png",
                    transparent=False,
                    bbox_inches='tight',
                    pad_inches=0)

    plt.close(fig)
    print(f"Frames saved to {output_dir}")


def make_video_from_images(image_dir, output_video='animation.mp4', fps=30):
    """Compiles a folder of images into a standalone video file."""
    images = sorted(glob.glob(f"{image_dir}/frame_*.png"))
    if not images:
        return

    # Read first image to get dimensions
    frame = cv2.imread(images[0])
    height, width, layers = frame.shape

    # Define codec and create VideoWriter
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

    for image in images:
        video.write(cv2.imread(image))

    video.release()
    print(f"Standalone video saved as {output_video}")


def overlay_plots_on_video(base_video_path, plots_dir, output_path='final_overlay.mp4'):
    """Overlays plot images onto the top-right corner of a background video."""
    cap = cv2.VideoCapture(base_video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (w, h))

    frame_idx = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Load corresponding plot image
        plot_path = f"{plots_dir}/frame_{frame_idx:04d}.png"
        if os.path.exists(plot_path):
            # Load overlay (with alpha channel)
            overlay = cv2.imread(plot_path, cv2.IMREAD_UNCHANGED)
            oh, ow, _ = overlay.shape

            # Position: Top Right Corner
            x_offset = w - ow
            y_offset = 0

            # Blend overlay onto frame using alpha channel
            alpha_s = overlay[:, :, 3] / 255.0
            alpha_l = 1.0 - alpha_s

            for c in range(0, 3):
                frame[y_offset:y_offset + oh, x_offset:x_offset + ow, c] = (
                        alpha_s * overlay[:, :, c] +
                        alpha_l * frame[y_offset:y_offset + oh, x_offset:x_offset + ow, c]
                )

        out.write(frame)
        frame_idx += 1

    cap.release()
    out.release()
    print(f"Final overlaid video saved as {output_path}")


if __name__ == '__main__':
    name = "video_"
    work_dir = "../logs/temp/"
    pos_t, pos_data, ori_t, ori_data = load_interaction_log('../logs/lb6_force_render_2026-02-26_17-14-43.json')
    # pos_t, pos_data, ori_data = load_cmd_log('./translation/translation_bh2_contact.json')
    # ori_t = pos_t

    pos_t, pos_data = resample_to_fps(pos_t, pos_data, target_fps=30)
    ori_t, ori_data = resample_to_fps(ori_t, ori_data, target_fps=30)

    config = {'figsize': [10, 6], 'dpi': 300}
    save_plot_frames(pos_data, ori_data, output_dir=work_dir+'plot_frames')

    make_video_from_images(work_dir+'plot_frames', work_dir+name+'log.mp4', fps=30)

    # overlay_plots_on_video('logs/force_render_2026-02-26_17-14-43.mp4', work_dir+'plot_frames', work_dir+name+'overlay.mp4')
