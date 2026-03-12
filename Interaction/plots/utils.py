import os
import json
import glob

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
import numpy as np
from bisect import bisect_left, bisect_right

matplotlib.use("macosx")
import matplotlib.colors as mcolors
import numpy as np

def get_zscore_outlier_indices(data, threshold=2):
    data = np.array(data)
    mean = np.mean(data)
    std = np.std(data)

    # Avoid division by zero if std is 0
    if std == 0:
        return []

    z_scores = np.abs((data - mean) / std)
    # Get indices where the Z-score is greater than the threshold
    outlier_indices = np.where(z_scores > threshold)[0]

    return outlier_indices.tolist()


def remove_by_indices(list_of_lists, indices_to_remove):
    # Convert to a set for O(1) lookup speed
    bad_indices = set(indices_to_remove)

    cleaned_result = []
    for original_list in list_of_lists:
        # Keep item if its index 'i' is NOT in our bad_indices set
        cleaned_list = [val for i, val in enumerate(original_list) if i not in bad_indices]
        cleaned_result.append(cleaned_list)

    return cleaned_result

def load_data(lb_path, lc_path):
    with open(lb_path, 'r') as f:
        lb_raw = json.load(f)
    with open(lc_path, 'r') as f:
        lc_raw = json.load(f)

    # Extract Rendering Force events from lb log
    lb_data = []
    for entry in lb_raw:
        if entry.get("type") == "event" and entry.get("data", {}).get("name") == "Rendering Force":
            event_data = entry["data"]
            # The vel is nested inside the 'value' key of the event
            lb_data.append(event_data["value"])

    # Extract loadcell measurements
    lc_data = []
    for entry in lc_raw:
        if entry.get("type") == "measurement" and entry.get("name") == "loadcell":
            lc_data.append({
                "time": entry["data"]["time"],
                "force": entry["data"]["force"]
            })

    # Sort both by time to ensure matching works correctly
    lb_data.sort(key=lambda x: x["time"])
    lc_data.sort(key=lambda x: x["time"])

    return lb_data, lc_data

def get_latest_paired_files(directory=".", index=1):
    """Finds the latest lb and loadcell files with matching timestamps in their names."""
    lb_files = glob.glob(os.path.join(directory, "lb*.json"))
    lc_files = glob.glob(os.path.join(directory, "loadcell*.json"))

    def extract_timestamp(filename):
        # Extracts 'YYYY-MM-DD_HH-MM-SS' from the end of the filename
        parts = os.path.basename(filename).replace(".json", "").split("_")
        return "_".join(parts[-2:])

    lb_map = {extract_timestamp(f): f for f in lb_files}
    lc_map = {extract_timestamp(f): f for f in lc_files}

    # Find intersection of timestamps and pick the latest one
    common_timestamps = sorted(set(lb_map.keys()) & set(lc_map.keys()))

    if not common_timestamps:
        raise FileNotFoundError("No matching pairs of lb and loadcell files found.")

    latest_ts = common_timestamps[-index]
    return lb_map[latest_ts], lc_map[latest_ts]


def sync_and_match(high_freq_data, low_freq_data):
    low_freq_t = [d["time"] for d in high_freq_data]

    matched_hi = []
    matched_lo = []

    for log_entry in low_freq_data:
        t = log_entry["time"]

        # Find the closest time in lb_data using binary search
        pos = bisect_right(low_freq_t, t)
        if pos == 0:
            closest_idx = 0
        elif pos == len(low_freq_t):
            closest_idx = pos - 1
        else:
            before = low_freq_t[pos - 1]
            after = low_freq_t[pos]
            if after - t < t - before:
                closest_idx = pos
            else:
                closest_idx = pos - 1

        # Only pair if the time difference is reasonably small (e.g., < 100ms)
        if abs(low_freq_t[closest_idx] - t) < 0.01:
            matched_hi.append(high_freq_data[closest_idx])
            matched_lo.append(log_entry)
    return matched_hi, matched_lo



def load_log_data(lb_path):
    with open(lb_path, 'r') as f:
        lb_raw = json.load(f)

    # Extract Rendering Force events from lb log
    lb_data = []
    for entry in lb_raw:
        if entry.get("type") == "event" and entry.get("data", {}).get("name") == "Rendering Force":
            event_data = entry["data"]
            # The vel is nested inside the 'value' key of the event
            lb_data.append(event_data["value"])

    lb_data.sort(key=lambda x: x["time"])

    return lb_data