from utils import *
from Interaction.Kalman_Filter import *


def vel_baseline(positions, times):
    F = len(positions)
    if F < 2:
        return 0.0

    # Position at Frame 0 and Frame F-1
    p_start = positions[0]
    p_end = positions[-1]

    # Net distance traveled (Magnitude of displacement vector)
    net_distance = p_end - p_start

    # Velocity = Distance / Total Time
    velocity = net_distance / (times[-1] - times[0])
    return velocity, times[-1]


def calculate_velocity_interpretation_1(positions, times):
    duration = times[-1] - times[0]
    F = len(positions)
    if F < 2:
        return 0.0

    first_step = positions[1] - positions[0]

    if np.all(first_step == 0):
        return None, None

    direction = np.sign(first_step)

    for i in range(1, len(positions)):
        if np.sign(positions[i] - positions[i - 1]) != direction:
            positions[i] = positions[i - 1]

    step_distances = np.diff(np.array(positions), axis=0)

    total_path_distance = np.sum(step_distances)

    velocity = total_path_distance / duration

    return velocity, times[-1]


def calculate_velocity_interpretation_2(positions, times):
    """
    Interpretation 2: Path-Based Velocity (Average Speed).
    Assumes direction can change every frame; sums all segments.
    Matches Example 2 result: 4 cm/s.
    """
    F = len(positions)
    if F < 2:
        return None, None

    positions = np.array(positions)

    step_distances = np.diff(positions, axis=0)

    total_path_distance = np.sum(step_distances)

    velocity = total_path_distance / (times[-1] - times[0])
    return velocity, times[-1]

def apply_vel_calculation(positions, times, F, vel_calc):
    vels = []
    timelines = []
    i = F

    while i <= len(positions):
        # Extract the window of F frames
        window_pos = positions[i - F: i]
        window_times = times[i - F: i]

        smooth_vel, t = vel_calc(window_pos, window_times)
        if smooth_vel:
            vels.append(smooth_vel)
            timelines.append(t)

        i += F

    return np.array(vels), np.array(timelines)


def apply_kf(positions, times, F):
    kf = VelocityKalmanFilter(dt=0.01 * F, process_noise=10.0, measurement_noise=0.01)

    vels = []
    timelines = []
    counter = 0
    for pos, t in zip(positions, times):
        counter += 1
        if counter < F:
            continue
        else:
            counter = 0
        smooth_pos, smooth_vel = kf.update(pos)
        vels.append(smooth_vel)
        timelines.append(t)
    return vels, timelines


if __name__ == '__main__':
    log_dir = "../../quantitative_evaluation/FLS_2.5N_50cm_5times_Shahram"
    lb_files = glob.glob(os.path.join(log_dir, "lb*.json"))
    lb_points = load_log_data(lb_files[0])

    F = 4

    displacement = [item['displacement']*10 for item in lb_points]
    lb_timeline = [item['time'] for item in lb_points]
    lb_timeline = np.array(lb_timeline) - lb_timeline[0]


    vel_base_2, t_base_2 = apply_vel_calculation(displacement, lb_timeline, 2, vel_baseline)

    vel_base, t_base = apply_vel_calculation(displacement, lb_timeline, F, vel_baseline)

    vel_it_1, t_it_1 = apply_vel_calculation(displacement, lb_timeline, F, calculate_velocity_interpretation_1)

    vel_it_2, t_it_2 = apply_vel_calculation(displacement, lb_timeline, F, calculate_velocity_interpretation_2)

    vel_kf, t_kf = apply_kf(displacement, lb_timeline, F)

    vel_kf_1, t_kf_1 = apply_kf(displacement, lb_timeline, 1)

    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    ax.plot(t_base_2, vel_base_2, alpha=0.3, linewidth=1, color=mcolors.TABLEAU_COLORS['tab:gray'], label="Baseline: F=2")
    # ax.plot(t_base, vel_base, alpha=0.8, linewidth=1, color=mcolors.TABLEAU_COLORS['tab:pink'], label="Baseline: F=4")
    # ax.plot(t_it_1, vel_it_1, alpha=0.8, linewidth=1, color=mcolors.TABLEAU_COLORS['tab:orange'], label="Interpretation 1: F=4")
    # ax.plot(t_it_2, vel_it_2, alpha=0.8, linewidth=1, color=mcolors.TABLEAU_COLORS['tab:purple'],  label="Interpretation 2: F=4")
    ax.plot(t_kf, vel_kf, alpha=0.8, linewidth=1, color=mcolors.TABLEAU_COLORS['tab:blue'], label="Kalman Filter: F=4")
    ax.plot(t_kf, vel_kf, alpha=0.3, linewidth=3, color=mcolors.TABLEAU_COLORS['tab:red'], label="Kalman Filter: F=2")

    name = "vel_cmp"
    name = "vel_baseline"
    name = "vel_it1"
    name = "vel_it2"
    name = "vel_kf"

    ax.set_xlim(0, max(lb_timeline))
    ax.set_ylim(min(min(vel_base_2), min(vel_it_1), min(min(vel_it_2), min(vel_kf))) * 1.2, max(max(vel_base_2), max(vel_it_1), max(max(vel_it_2), max(vel_kf))) * 1.2)
    ax.set_xlabel("Time [s]", fontsize=16)
    ax.set_title("Measured Velocity [cm/s]", loc="left", fontsize=16)
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.tick_params(axis='both', labelsize=16)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.legend(fontsize=16)
    plt.tight_layout()

    # plt.show()

    fig.savefig('../logs/vels/' + name + '.png', dpi=300)
