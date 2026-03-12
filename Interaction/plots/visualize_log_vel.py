import json
from utils import *

def visualize_drone_velocity(file_path):
    timeline = []
    vel_xy = []
    onboard_vel = []
    onboard_timeline = []

    try:
        with open(file_path, 'r') as f:
            # Loading data (assuming the file is a list of JSON objects)
            data = json.load(f)

        for entry in data:
            # Filter by type and name
            if entry.get("type") == "frames" and entry.get("name") == "drone":
                # Extract position (tvec) for the origin of the arrow
                pos = entry["data"]["tvec"]
                # Extract velocity (vel) for the vector direction
                vel = entry["data"]["vel"]
                t = entry["data"]["time"]

                timeline.append(t)
                vel_xy.append(np.linalg.norm(vel[:2]))
            elif entry.get("type") == "state":
                vx = entry["data"]["stateEstimate.vx"]
                vy = entry["data"]["stateEstimate.vy"]
                onboard_timeline.append(entry["data"]['time'])
                onboard_vel.append(np.linalg.norm([vx, vy]))

        start_time = min([timeline[0], onboard_timeline[0]])
        timeline = np.array(timeline) - start_time
        onboard_timeline = np.array(onboard_timeline) - start_time

        fig, ax = plt.subplots(1, 1, figsize=(10, 6))

        ax.plot(timeline, vel_xy, label='kalman Filter Velocity')
        ax.plot(onboard_timeline, onboard_vel, label='On board Velocity')
        ax.set_xlabel("Time [s]", fontsize=16)
        ax.set_title("Velocity [m/s]", loc="left", fontsize=16)
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.tick_params(axis='both', labelsize=16)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.legend(fontsize=16)
        plt.tight_layout()
        plt.show()
    except FileNotFoundError:
        print(f"Error: The file '{file_path}' was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")



if __name__ == "__main__":
    # visualize_drone_velocity('../logs/translation_fail.json')
    visualize_drone_velocity('../../logs/vel_compare_pn1000.json')
