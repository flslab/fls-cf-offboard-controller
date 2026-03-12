from utils import *
import numpy as np


if __name__ == "__main__":

    log_dir = "../../quantitative_evaluation/FLS_2.5N_50cm_5times_Shahram"

    lb_file, lc_file = get_latest_paired_files(log_dir)
    print(f"Processing:\n LB: {lb_file}\n LC: {lc_file}")

    lb_points, lc_points = load_data(lb_file, lc_file)

    matched_lb, matched_lc = sync_and_match(lb_points, lc_points)
    # matched_lc, matched_lb = sync_and_match(lc_points, lb_points)

    displacement = [item['displacement']*100 for item in matched_lb]
    lb_timeline = [item['time'] for item in matched_lb]
    force_to_render = [item['force'] for item in matched_lb]

    init_error = matched_lc[10]['force']
    measured_force = [item['force'] for item in matched_lc]
    lc_timeline = [item['time'] for item in matched_lc]


    start_time = min(lb_timeline[0], lc_timeline[0])
    lb_timeline = np.array(lb_timeline) - start_time
    lc_timeline = np.array(lc_timeline) - start_time


    # for i, item in enumerate(zip(measured_force, lc_timeline)):
    #     force, t = item
    #     measured_force[i] = (force - init_error)


    outliers = get_zscore_outlier_indices(measured_force, threshold=2)
    filtered_displacement, filtered_force_to_render, filtered_measured_force = remove_by_indices([displacement, force_to_render, measured_force], outliers)
    filtered_displacement, filtered_force_to_render, filtered_measured_force = np.array(filtered_displacement), np.array(filtered_force_to_render), np.array(filtered_measured_force)

    mask = filtered_displacement >= 0
    filtered_displacement, filtered_force_to_render, filtered_measured_force = filtered_displacement[mask], filtered_force_to_render[mask], filtered_measured_force[mask]

    fig, ax = plt.subplots(1, 1, figsize=(10, 6))

    ax.scatter(filtered_displacement, filtered_measured_force, alpha=0.5, s=10, label="Measured Force")
    ax.scatter(filtered_displacement, filtered_force_to_render, alpha=0.5, s=10, label="FLS Rendered Force")
    ax.set_ylim(0, max(max(filtered_force_to_render), max(filtered_measured_force)) * 1.2)
    ax.set_xlim([0, max(filtered_displacement) * 1.2])
    ax.set_xlabel("Position Offset [cm]", fontsize=16)
    ax.set_title("Measured Force [N]", loc="left", fontsize=16)
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.tick_params(axis='both', labelsize=16)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.legend(fontsize=16)
    plt.tight_layout()

    # fig.savefig(log_dir+'/stiffness_displacement.png', dpi=300)

    fig1, ax1 = plt.subplots(1, 1, figsize=(10, 6))
    ax1.scatter(lc_timeline, measured_force, alpha=0.5, s=10, label="Measured Force")
    ax1.scatter(lb_timeline, force_to_render, alpha=0.5, s=10, label="FLS Rendered Force")
    ax1.set_ylim(0, max(max(force_to_render), max(measured_force)) * 1.2)
    ax1.set_xlim([0, max(max(lc_timeline), max(lb_timeline)) * 1.2])
    ax1.set_xlabel("Time [s]", fontsize=16)
    ax1.set_title("Measured Force [N]", loc="left", fontsize=16)
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.tick_params(axis='both', labelsize=16)
    ax1.spines['top'].set_visible(False)
    ax1.spines['right'].set_visible(False)
    ax1.legend(fontsize=16)
    plt.tight_layout()
    plt.show()
    # fig1.savefig(log_dir + '/stiffness_time.png', dpi=300)