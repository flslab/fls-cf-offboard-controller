from utils import *
import numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation

def create_reveal_animation(x_data, y1_data, xlabel, ylabel, filename):
    fig, ax = plt.subplots(figsize=(10, 6))

    # Setup limits
    max_x = np.max(x_data)
    max_y = np.max(y1_data)
    ax.set_xlim(0, max_x)
    ax.set_ylim(0, max_y * 1.2)
    ax.set_xlabel(xlabel, fontsize=14)
    ax.set_title(ylabel, loc="left", fontsize=14)

    scat1 = ax.scatter(x_data, y1_data, s=5, color=mcolors.TABLEAU_COLORS['tab:gray'], alpha=0.2, label="Measured Force")

    # The scanning bar
    v_line = ax.axvline(x=0, color='gray', linestyle='--', linewidth=1.5)

    # ax.legend(loc='upper right')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.tick_params(axis='both', labelsize=16)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    plt.tight_layout()

    # Define the range for the vertical line to move (e.g., 200 steps)
    num_frames = 4320
    line_positions = np.linspace(0, max_x, num_frames)

    pbar = tqdm(total=num_frames, desc=f"Rendering {filename.split('/')[-1]}")
    def update(frame):
        current_x = line_positions[frame]

        # Move the bar
        v_line.set_xdata([current_x, current_x])

        # Logic for Scat 1 (Measured Force)
        colors1 = np.where(x_data <= current_x, mcolors.TABLEAU_COLORS['tab:blue'], mcolors.TABLEAU_COLORS['tab:gray'])
        alphas1 = np.where(x_data <= current_x, 0.8, 0.2)
        # Update colors/alphas (concatenating RGB + Alpha)
        scat1.set_color(colors1)
        scat1.set_alpha(alphas1)
        pbar.update(1)
        return v_line, scat1

    # interval=30ms for ~33 fps
    ani = FuncAnimation(fig, update, frames=num_frames, blit=False, interval=16.67)

    # Save
    writer = FFMpegWriter(fps=60, bitrate=2000)
    print(f"Generating reveal video: {filename}...")
    ani.save(filename, writer=writer)
    pbar.close()
    plt.close(fig)



if __name__ == "__main__":


    log_dir = "../../quantitative_evaluation/FLS_2.5N_50cm_5times_Shahram"
    lb_files = glob.glob(os.path.join(log_dir, "lb*.json"))
    lb_points = load_log_data(lb_files[0])

    displacement = np.array([item['displacement'] * 100 for item in lb_points])
    lb_timeline = np.array([item['time'] for item in lb_points])
    lb_timeline = np.array(lb_timeline) - lb_timeline[0]
    force_to_render = np.array([item['force'] for item in lb_points])

    mask = lb_timeline <= 72
    displacement = displacement[mask]
    lb_timeline = lb_timeline[mask]
    force_to_render = force_to_render[mask]

    # --- Usage with your filtered data ---

    # create_reveal_animation(
    #     displacement,
    #     force_to_render,
    #     "Position Offset [cm]", "Force Rendering [N]",
    #     log_dir + '/stiffness_displacement_reveal.mp4'
    # )

    create_reveal_animation(
        lb_timeline,
        force_to_render,
        "Time [s]", "Rendered Force [N]",
        log_dir + '/stiffness_time_reveal.mp4'
    )