from matplotlib import pyplot as plt
import matplotlib as mpl
import numpy as np
import json
import argparse


def plot_logs(file=""):
    if file:
        with open(file) as f:
            json_data = json.load(f)
            _time = json_data["time"]
            log_vars = json_data["params"]

    fig, ax = plt.subplots()
    time_axis = (np.array(_time) - _time[0]) / 1000
    # ax.plot(time_axis, np.array(_thrust) / 0xffff * 100, label='thrust (%)')
    # ax.plot(time_axis, np.array(_roll), label='roll')
    # ax.plot(time_axis, np.array(_m1) / 0xffff * 100, label='m1 (%)')
    # ax.plot(time_axis, np.array(_m2) / 0xffff * 100, label='m2 (%)')
    # ax.plot(time_axis, np.array(_m3) / 0xffff * 100, label='m3 (%)')
    # ax.plot(time_axis, np.array(_m4) / 0xffff * 100, label='m4 (%)')
    # ax.plot(time_axis, np.array(_x) * 100, label='x (cm)')
    # ax.plot(time_axis, np.array(_y) * 100, label='y (cm)')
    # ax.plot(time_axis, np.array(_z) * 100, label='z (cm)')
    # ax.plot(time_axis, np.array(_delta_x), label='delta x (flow/fr)')
    # ax.plot(time_axis, np.array(_delta_y), label='delta y (flow/fr)')
    for par in log_vars.keys():
        ax.plot(time_axis, np.array(log_vars[par]["data"]), label=f"{par} ({log_vars[par]['unit']})")
    ax.set_xlabel('Time (s)')
    # ax.set_ylim([-15, 105])
    plt.legend()

    image_name = "".join(file.split('.')[:-1])
    plt.savefig(f'{image_name}.png', dpi=300)


def plot_tvec_from_log(log_file):
    with open(log_file, 'r') as f:
        data = json.load(f)

    frame_ids = []
    tvec_x = []
    tvec_y = []
    tvec_z = []

    for frame in data.get("frames", []):
        frame_ids.append(frame["frame_id"])
        tvec_x.append(frame["tvec"][0])
        tvec_y.append(frame["tvec"][1])
        tvec_z.append(frame["tvec"][2])

    plt.figure(figsize=(10, 5))
    plt.plot(frame_ids, tvec_x, label="tvec_x", marker='o')
    plt.plot(frame_ids, tvec_y, label="tvec_y", marker='o')
    plt.plot(frame_ids, tvec_z, label="tvec_z", marker='o')

    plt.xlabel("Frame ID")
    plt.ylabel("Translation Vector (tvec)")
    plt.title("tvec vs. Frame ID")
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == '__main__':
    mpl.use('macosx')
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--input", help="path to json log file")
    args = ap.parse_args()

    # plot_logs(args.input)
    plot_tvec_from_log("logs/pose_logs_2025-03-21_11-40-28.json")
