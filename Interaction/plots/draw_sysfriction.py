from utils import *
import numpy as np


if __name__ == "__main__":

    log_dir = "../../logs/sys_friction"

    lb_file, lc_file = get_latest_paired_files(log_dir)
    print(f"Processing:\n LB: {lb_file}\n LC: {lc_file}")

    lb_points, lc_points = load_data(lb_file, lc_file)

    for i, item in enumerate(lb_points[1:]):
        item["delta_p"] = item["displacement"] - lb_points[i-1]["displacement"]

    matched_lb, matched_lc = sync_and_match(lb_points, lc_points)

    delta_p = [item['delta_p']*100 for item in matched_lb]

    sys_friction = [item['force'] for item in matched_lc]

    outliers = get_zscore_outlier_indices(sys_friction, threshold=2)
    delta_p, sys_friction = remove_by_indices([delta_p, sys_friction], outliers)
    delta_p, sys_friction = np.array(delta_p), np.array(sys_friction)

    mask = delta_p >= 0
    delta_p, sys_friction = delta_p[mask], sys_friction[mask]

    # Create a figure with 1 row and 2 columns
    # axes is an array containing [left_plot, right_plot]
    fig, axes = plt.subplots(1, 1, figsize=(10, 6))
    axes.scatter(delta_p, sys_friction, s=10, label="Measured Force")
    axes.set_ylim(0, max(sys_friction) * 1.2)
    axes.set_xlim([0, max(delta_p) * 1.2])
    axes.set_xlabel("Displacement [mm]")
    axes.set_ylabel("Measured Force [N]")
    axes.set_title("System Friction vs. Velocity")
    axes.grid(True, linestyle='--', alpha=0.7)
    axes.legend()

    # Automatically adjust spacing so labels don't overlap
    plt.tight_layout()
    plt.show()