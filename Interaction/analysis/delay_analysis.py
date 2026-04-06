"""
delay_analysis.py
=================
Read a .json log produced by LiveLogger and report:

  Graph 1 – Packet inter-arrival time (IAT) as a function of time for log
             types 'frames' and 'state'.

  Graph 2 – Number of packets received within each non-overlapping 1-second
             time window, for both types.

"""

import json
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


# ── helpers ───────────────────────────────────────────────────────────────────

def _load_log(file_path: str) -> list:
    """Load a (possibly truncated) LiveLogger JSON array."""
    with open(file_path, "r") as f:
        content = f.read()
    try:
        return json.loads(content)
    except json.JSONDecodeError:
        # File may be incomplete (logger interrupted mid-write).  Trim to the
        # last fully written entry.
        idx = content.rfind("},")
        if idx == -1:
            raise
        return json.loads(content[: idx + 1] + "\n]")


def _extract_timestamps(entries: list, log_type: str) -> np.ndarray:
    """Return sorted array of wall-clock timestamps for a given log type."""
    times = []
    for e in entries:
        if e.get("type") == log_type:
            t = e.get("data", {}).get("time")
            if t is not None:
                times.append(float(t))
    return np.array(sorted(times))


def _iat(timestamps: np.ndarray):
    """
    Inter-arrival times in milliseconds.

    Returns
    -------
    mid_times : array of shape (N-1,)  – midpoint between consecutive packets,
                expressed as elapsed seconds from the first timestamp in the
                full recording.
    iat_ms    : array of shape (N-1,)  – inter-arrival time in ms.
    t0        : float – absolute start timestamp (first packet of this type).
    """
    if len(timestamps) < 2:
        return np.array([]), np.array([]), float("nan")
    iat_ms = np.diff(timestamps) * 1000.0
    mid_abs = (timestamps[:-1] + timestamps[1:]) / 2.0
    return mid_abs, iat_ms, timestamps[0]


def _packet_counts_per_window(timestamps: np.ndarray, window_s: float,
                               t_start: float, t_end: float):
    """
    Count packets that fall within each non-overlapping window.

    Returns
    -------
    window_centers : array  – centre of each window in elapsed seconds.
    counts         : array  – number of packets in that window.
    """
    if len(timestamps) == 0:
        return np.array([]), np.array([])

    edges = np.arange(t_start, t_end + window_s, window_s)
    counts, _ = np.histogram(timestamps, bins=edges)
    centers = edges[:-1] + window_s / 2.0
    return centers, counts


# ── main analysis / plotting function ────────────────────────────────────────

def analyze_delay(file_path: str, window_s: float = 1.0,
                  save_path: str | None = None) -> None:
    """
    Read *file_path* and produce the two delay-characterisation graphs.

    Parameters
    ----------
    file_path : path to the LiveLogger JSON log.
    window_s  : width of the time-window for the packet-count graph (seconds).
    save_path : if given, save the figure to this path instead of showing it.
    """
    if not os.path.isfile(file_path):
        raise FileNotFoundError(f"Log file not found: {file_path}")

    entries = _load_log(file_path)

    # ── extract timestamps for each type ─────────────────────────────────────
    ts_frames = _extract_timestamps(entries, "frames")
    ts_states = _extract_timestamps(entries, "state")

    all_ts = np.concatenate([ts_frames, ts_states])
    if len(all_ts) == 0:
        print("No 'frames' or 'state' entries found in the log.")
        return
    t_end_global = all_ts.max()

    # ── determine t0 using same priority as compare_position.py ──────────────
    # 1) 'Waiting For User Interaction' event
    # 2) first frame/state timestamp after first HighLevelCommander.go_to command
    # 3) earliest timestamp
    wfi_time = None
    goto_time = None

    goto_cmd_idx = None
    for idx, item in enumerate(entries):
        if (item.get("type") == "commands"
                and item.get("name") == "HighLevelCommander.go_to"
                and goto_cmd_idx is None):
            goto_cmd_idx = idx

    if goto_cmd_idx is not None:
        for item in entries[goto_cmd_idx + 1:]:
            t = item.get("type")
            if t == "lb5":
                d = item.get("data", {})
                if "time" in d:
                    goto_time = d["time"]
                    break
            elif t == "state":
                d = item.get("data", {})
                if "time" in d:
                    goto_time = d["time"]
                    break

    for item in entries:
        if item.get("type") == "events":
            ev_name = item.get("name")
            ev_t = item.get("data", {}).get("time")
            if ev_name == "Waiting For User Interaction" and ev_t is not None:
                wfi_time = ev_t
                break

    if wfi_time is not None:
        t0_global = wfi_time
        print(f"t0: 'Waiting For User Interaction' at {wfi_time:.3f}")
    elif goto_time is not None:
        t0_global = goto_time
        print(f"t0: first HighLevelCommander.go_to at {goto_time:.3f}")
    else:
        t0_global = all_ts.min()
        print("t0: earliest timestamp (no WFI event or go_to command found)")

    # convert to elapsed seconds, trim before t0
    ts_frames_rel = ts_frames - t0_global
    ts_frames_rel = ts_frames_rel[ts_frames_rel >= 0.0]
    ts_states_rel = ts_states - t0_global
    ts_states_rel = ts_states_rel[ts_states_rel >= 0.0]

    # ── focus window ─────────────────────────────────────────────────────────
    T_START = 10.0   # seconds after t0
    T_END   = 20.0   # seconds after t0

    # ── IAT (computed on trimmed relative timestamps) ─────────────────────────
    mid_f_rel, iat_f, _ = _iat(ts_frames_rel)
    mid_s_rel, iat_s, _ = _iat(ts_states_rel)

    # restrict IAT to focus window
    if len(mid_f_rel):
        mask = (mid_f_rel >= T_START) & (mid_f_rel <= T_END)
        mid_f_rel, iat_f = mid_f_rel[mask], iat_f[mask]
    if len(mid_s_rel):
        mask = (mid_s_rel >= T_START) & (mid_s_rel <= T_END)
        mid_s_rel, iat_s = mid_s_rel[mask], iat_s[mask]

    # ── packet counts per window ──────────────────────────────────────────────
    t_end_rel = t_end_global - t0_global
    wc_f, cnt_f = _packet_counts_per_window(ts_frames_rel, window_s, T_START, T_END)
    wc_s, cnt_s = _packet_counts_per_window(ts_states_rel, window_s, T_START, T_END)

    # ── figure ────────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 9))
    # fig.suptitle(f"Delay analysis — {os.path.basename(file_path)}", fontsize=13)
    gs = gridspec.GridSpec(2, 1, hspace=0.45)

    # ── Graph 1: IAT vs elapsed time ─────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0])
    if len(iat_f):
        ax1.plot(mid_f_rel, iat_f, linewidth=0.8, alpha=0.8,
                 label=f"Vicon via Wifi  (mean {iat_f.mean():.2f} ms, "
                        f"median {np.median(iat_f):.2f} ms, "
                        f"max {iat_f.max():.2f} ms)")
    if len(iat_s):
        ax1.plot(mid_s_rel, iat_s, linewidth=0.8, alpha=0.8,
                 label=f"Radio   (mean {iat_s.mean():.2f} ms, "
                        f"median {np.median(iat_s):.2f} ms, "
                        f"max {iat_s.max():.2f} ms)")

    ax1.set_xlabel("Elapsed time (s)")
    ax1.set_ylabel("Inter-arrival time (ms)")
    ax1.set_title("Packet inter-arrival time over time")
    ax1.legend(fontsize=8)
    ax1.set_xlim(T_START, T_END)
    ax1.set_ylim(0, 70)
    ax1.grid(True, alpha=0.3)

    # ── Graph 2: packet count per 1-s window ──────────────────────────────────
    ax2 = fig.add_subplot(gs[1])
    bar_w = window_s * 0.38
    if len(wc_f):
        ax2.bar(wc_f - bar_w / 2, cnt_f, width=bar_w, alpha=0.8,
                label=f"Vicon via Wifi  (mean {cnt_f.mean():.1f}, "
                       f"max {int(cnt_f.max())}, min {int(cnt_f.min())})/window")
    if len(wc_s):
        ax2.bar(wc_s + bar_w / 2, cnt_s, width=bar_w, alpha=0.8,
                label=f"Radio   (mean {cnt_s.mean():.1f}, "
                       f"max {int(cnt_s.max())}, min {int(cnt_s.min())})/window")

    ax2.set_xlabel("Elapsed time (s)")
    ax2.set_ylabel(f"Packets per {window_s:.1f}-s window")
    ax2.set_title(f"Packet count in non-overlapping {window_s:.1f}-s time windows")
    ax2.legend(fontsize=8)
    ax2.set_xlim(T_START, T_END)
    ax2.set_ylim(0, 110)
    ax2.grid(True, alpha=0.3, axis="y")

    # ── summary to stdout ─────────────────────────────────────────────────────
    duration = t_end_rel
    print(f"\n{'─'*60}")
    print(f"  Log : {file_path}")
    print(f"  Duration : {duration:.2f} s  |  window : {window_s:.1f} s")
    print(f"{'─'*60}")
    for label, ts_rel, iat_ms in [("Vicon Via Wifi", ts_frames_rel, iat_f),
                                   ("Radio ", ts_states_rel, iat_s)]:
        n = len(ts_rel)
        if n == 0:
            print(f"  {label}: no entries")
            continue
        print(f"  {label}: {n} packets")
        if len(iat_ms):
            print(f"    IAT  mean={iat_ms.mean():.3f} ms  "
                  f"median={np.median(iat_ms):.3f} ms  "
                  f"max={iat_ms.max():.3f} ms  "
                  f"min={iat_ms.min():.3f} ms")
    print(f"{'─'*60}\n")

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Figure saved → {save_path}")
    else:
        plt.show()

    plt.close(fig)


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    window_s = 1.0
    
    # log_file = "./logs/lb5_arrow_interaction_follow_2026-03-29_22-49-16.json"
    # save_path = None
    # # save_path = "./logs/vicon_noise_reduce/1_marker_anchor_delay.png"
    # analyze_delay(log_file, window_s=window_s, save_path=save_path)

    log_file = "./logs/vicon_noise_reduce/1_marker_anchor_translation.json"
    save_path = "./logs/vicon_noise_reduce/1_marker_anchor_delay.png"
    analyze_delay(log_file, window_s=window_s, save_path=save_path)


    
    log_file = "./logs/vicon_noise_reduce/1_marker_translation.json"
    save_path = "./logs/vicon_noise_reduce/1_marker_delay.png"
    analyze_delay(log_file, window_s=window_s, save_path=save_path)


    
    log_file = "./logs/vicon_noise_reduce/4_markers_translation.json"
    save_path = "./logs/vicon_noise_reduce/4_markers_delay.png"
    analyze_delay(log_file, window_s=window_s, save_path=save_path)


if __name__ == "__main__":
    main()
