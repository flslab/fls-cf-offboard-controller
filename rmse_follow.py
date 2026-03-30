
import argparse
import json
import numpy as np
from scipy.interpolate import interp1d


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------

def load_partial_json_array(path):
    """Load a JSON array that may be truncated (e.g. crash mid-write)."""
    with open(path) as f:
        content = f.read()
    entries = []
    decoder = json.JSONDecoder()
    i = 0
    while i < len(content) and content[i] in ' \n\t[':
        i += 1
    while i < len(content):
        while i < len(content) and content[i] in ' \n\t,':
            i += 1
        if i >= len(content) or content[i] == ']':
            break
        try:
            obj, end = decoder.raw_decode(content, i)
            entries.append(obj)
            i += end - i
        except json.JSONDecodeError:
            nl = content.find('\n', i)
            if nl == -1:
                break
            i = nl + 1
    return entries


def get_first_event_time(entries, name):
    """Return timestamp of the first event with the given name, or None."""
    for e in entries:
        if e.get('type') == 'events' and e.get('name') == name:
            return e['data']['time']
    return None


def extract_frames(entries, start_time, stop_time=None):
    """Return (times, positions) numpy arrays for 'frames' entries in [start_time, stop_time]."""
    times, positions = [], []
    for e in entries:
        if e.get('type') == 'frames':
            d = e['data']
            t = d['time']
            if t < start_time:
                continue
            if stop_time is not None and t > stop_time:
                break
            times.append(t)
            positions.append(d['tvec'])
    return np.array(times), np.array(positions)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def compute_rmse(leader_path, follower_path, offset, grace_time=2.0):
    offset = np.array(offset, dtype=float)

    print(f"Loading leader:   {leader_path}")
    leader_entries = load_partial_json_array(leader_path)
    print(f"Loading follower: {follower_path}")
    follower_entries = load_partial_json_array(follower_path)

    t_push_leader   = get_first_event_time(leader_entries,   'User Pushing')
    t_push_follower = get_first_event_time(follower_entries, 'User Pushing')
    t_disengage_leader   = get_first_event_time(leader_entries,   'User Disengage')
    t_disengage_follower = get_first_event_time(follower_entries, 'User Disengage')

    if t_push_leader is None or t_push_follower is None:
        raise ValueError("Could not find 'User Pushing' event in one of the logs.")
    if t_disengage_leader is None or t_disengage_follower is None:
        raise ValueError("Could not find 'User Disengage' event in one of the logs.")

    # Start: later of the two first pushes (both are in the interaction window)
    start_time = max(t_push_leader, t_push_follower)

    # Stop: earlier disengage + grace_time (conservative — interaction window is over)
    t_disengage = min(t_disengage_leader, t_disengage_follower)
    stop_time = t_disengage + grace_time

    print(f"\nFirst User Pushing:")
    print(f"  leader   t = {t_push_leader:.6f}")
    print(f"  follower t = {t_push_follower:.6f}")
    print(f"  → analysis start: t = {start_time:.6f}")
    print(f"\nFirst User Disengage:")
    print(f"  leader   t = {t_disengage_leader:.6f}")
    print(f"  follower t = {t_disengage_follower:.6f}")
    print(f"  → analysis stop (earlier + grace {grace_time}s): t = {stop_time:.6f}")
    print(f"  → window duration: {stop_time - start_time:.3f} s")

    leader_t,   leader_pos   = extract_frames(leader_entries,   start_time, stop_time)
    follower_t, follower_pos = extract_frames(follower_entries, start_time, stop_time)

    print(f"\nFrames after start: leader={len(leader_t)}, follower={len(follower_t)}")

    if len(leader_t) < 2:
        raise ValueError("Not enough leader frames after start time.")
    if len(follower_t) == 0:
        raise ValueError("No follower frames after start time.")

    # Interpolate leader position onto follower timestamps
    t_min = max(leader_t[0],   follower_t[0])
    t_max = min(leader_t[-1],  follower_t[-1])

    mask = (follower_t >= t_min) & (follower_t <= t_max)
    follower_t   = follower_t[mask]
    follower_pos = follower_pos[mask]

    if len(follower_t) == 0:
        raise ValueError("No overlapping timestamps between leader and follower.")

    interp = interp1d(leader_t, leader_pos, axis=0, kind='linear', fill_value='extrapolate')
    leader_pos_at_follower_t = interp(follower_t)

    expected_pos = leader_pos_at_follower_t + offset

    error = follower_pos - expected_pos          # (N, 3)
    error_norm = np.linalg.norm(error, axis=1)   # (N,)

    rmse_3d = np.sqrt(np.mean(error_norm ** 2))
    rmse_x  = np.sqrt(np.mean(error[:, 0] ** 2))
    rmse_y  = np.sqrt(np.mean(error[:, 1] ** 2))
    rmse_z  = np.sqrt(np.mean(error[:, 2] ** 2))

    t_rel = follower_t - start_time

    print(f"\n=== RMSE Results (offset = {offset.tolist()}) ===")
    print(f"  Samples analysed : {len(follower_t)}")
    print(f"  Duration         : {t_rel[-1]:.2f} s")
    print(f"  3D RMSE          : {rmse_3d*100:.3f} cm  ({rmse_3d*1000:.2f} mm)")
    print(f"  X RMSE           : {rmse_x*100:.3f} cm  ({rmse_x*1000:.2f} mm)")
    print(f"  Y RMSE           : {rmse_y*100:.3f} cm  ({rmse_y*1000:.2f} mm)")
    print(f"  Z RMSE           : {rmse_z*100:.3f} cm  ({rmse_z*1000:.2f} mm)")

    worst_idx = np.argmax(error_norm)
    print(f"\n  Worst sample     : t={t_rel[worst_idx]:.3f}s  error={error_norm[worst_idx]*100:.3f} cm")
    print(f"    follower pos   : {follower_pos[worst_idx].round(4).tolist()}")
    print(f"    expected pos   : {expected_pos[worst_idx].round(4).tolist()}")

    return {
        'rmse_3d': rmse_3d,
        'rmse_x':  rmse_x,
        'rmse_y':  rmse_y,
        'rmse_z':  rmse_z,
        't_rel':   t_rel,
        'error_norm': error_norm,
        'follower_pos': follower_pos,
        'expected_pos': expected_pos,
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--leader',   required=True, help='Path to leader log JSON')
    parser.add_argument('--follower', required=True, help='Path to follower log JSON')
    parser.add_argument('--offset',   nargs=3, type=float, default=[0, 0, 0],
                        metavar=('X', 'Y', 'Z'), help='Fixed offset: follower = leader + offset')
    parser.add_argument('--grace-time', type=float, default=2.0,
                        help='Grace time (s) added to first disengage as stop time (default: 2.0)')
    args = parser.parse_args()

    compute_rmse(args.leader, args.follower, args.offset, args.grace_time)
