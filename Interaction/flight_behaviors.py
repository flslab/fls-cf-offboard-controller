import json
import threading
import time
from collections import deque
from typing import Callable
import numpy as np
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


def approach_target_vel(cf, cur_xy, goal_xy, z, move_speed=0.15, yawrate=0.0):
    err_xy = np.clip(goal_xy - cur_xy, -0.1, 0.1)
    dist = np.linalg.norm(err_xy)

    # Normalize to get direction
    if dist > 0.03:
        dir_xy = err_xy / dist
    else:
        dir_xy = np.zeros_like(err_xy)

    # Velocity commands
    vx, vy = dir_xy * move_speed
    # print(f"Move: from {cur_xy} to {goal_xy}, Vel: {vx}, {vy}, yawrate: {yawrate}")
    # Send velocity command
    cf.commander.send_hover_setpoint(vx, vy, yawrate, z)


def approach_target_pos(cf, cur_xy, goal_xy, z, move_speed=0.15, yawrate=0.0):
    err_xy = np.clip(goal_xy - cur_xy, -0.1, 0.1)
    dist = np.linalg.norm(err_xy)

    # Normalize to get direction
    if dist > 0.03:
        dir_xy = err_xy / dist
    else:
        dir_xy = np.zeros_like(err_xy)

    # Velocity commands
    vx, vy = dir_xy * move_speed

    set_point = [cur_xy[0] + vx, cur_xy[1] + vy, z]

    # Send velocity command
    cf.commander.send_position_setpoint(set_point[0], set_point[1], set_point[2], yawrate)



def contact_monitor_loop_std(get_state: Callable, get_pos: Callable, get_apparatus: Callable, log_event: Callable,
                             contact_event: threading.Event,
                             logger=None, sleep_function=None, contact_window_length=100, contact_outlier_sigma=4,
                             delay=0):

    if not sleep_function:
        sleep_function = time.sleep
    hist = deque()
    cf_pos = get_pos()
    goal_pos = get_apparatus()
    cf_xy = cf_pos[:2]
    goal_xy = goal_pos[:2]

    d = np.array(goal_xy, float) - np.array(cf_xy, float)
    if np.linalg.norm(d) < 1e-9:
        d = np.array([1.0, 0.0], float)
    else:
        d /= np.linalg.norm(d)

    while not contact_event.is_set():
        state = get_state()
        cf_xyz = get_pos()
        cur_xy = cf_xyz[:2]

        # Parallel displacement
        dist = float((goal_xy - cur_xy).dot(d))

        log_time = float(state.get("time", 0.0))

        # Acceleration in world/body (x,y)
        ax = float(state.get("acc.x", 0.0))
        ay = float(state.get("acc.y", 0.0))

        # Project onto goal direction
        a_par = ax * d[0] + ay * d[1]

        # Input u = sin(pitch)
        pitch_now = float(state.get("stateEstimate.pitch", 0.0))
        u_now = np.sin(np.radians(pitch_now))

        # Append to time history
        hist.append((log_time, a_par, u_now))
        # Keep only last cfg.window_s seconds
        while hist and len(hist) > contact_window_length:
            hist.popleft()

        impact = False
        n = len(hist)
        contact_score = 0
        if n >= 5:
            # t_arr = np.array([h[0] for h in hist])
            a_arr = np.array([h[1] for h in hist])
            u_arr = np.array([h[2] for h in hist])

            # Linear regression: a = k*u + b
            U = np.vstack([u_arr, np.ones_like(u_arr)]).T
            k, b = np.linalg.lstsq(U, a_arr, rcond=None)[0]

            y_pred = k * u_arr + b
            resids = a_arr - y_pred
            sigma = np.std(resids) if n > 1 else 1e-6

            resid_now = a_par - (k * u_now + b)
            contact_score = -resid_now / (sigma if sigma > 1e-9 else 1e-9)

            if logger is not None:
                logger.debug(f"Contact Score: {contact_score:.2f}, Dist: {dist:.3f}")
            log_event('Detecting Contact', contact_score)
            if contact_score > contact_outlier_sigma and dist < 0.05:
                impact = True

        if impact:
            logger.info(f"[CONTACT] Contact Score={contact_score:.2f} (> {contact_outlier_sigma}σ)  a_par={a_par:.3f}")
            log_event('Contact', contact_score)
            time.sleep(delay)
            contact_event.set()
            break

        sleep_function(0.01)


def hover_pwm(pitch=0, hover_pwm=30000):
    return int(round(hover_pwm / np.cos(np.deg2rad(pitch))))


def turning_pwm(pitch, default_pwm=10000, min_pwm=10000, hover_pwm=30000):
    safe_pitch = max(0.1, abs(pitch))

    raw_pwm = default_pwm / np.sin(np.deg2rad(safe_pitch))
    final_pwm = np.clip(raw_pwm, min_pwm, hover_pwm)

    return int(round(final_pwm))


def set_pwm_all(cf, pwm):
    cf.param.set_value('motorPowerSet.enable', '2')
    cf.param.set_value('motorPowerSet.m1', pwm)


def stop_pwm_override(cf):
    cf.param.set_value('motorPowerSet.enable', '0')

def load_commands(log_file_path, reset_start=False):
    """
    Reads a JSON log file (list of typed entries) and returns only the
    'commands' entries, remapped to the format expected by execute_commands:
        {"command": <name>, "time": <float>, "args": [...], "kwargs": {...}}
    """
    with open(log_file_path, 'r') as f:
        entries = json.load(f)

    start_time = 0.0

    if reset_start:
        # First pass: find the start time
        for entry in entries:
            if entry.get('type') == 'start':
                data = entry.get('data')
                # Handle if data is a dict ({"time": 123}) or a direct value (123)
                if isinstance(data, dict):
                    start_time = data.get('time', 0.0)
                else:
                    start_time = float(data)
                break

    cmds = []
    # Second pass: process the commands
    for entry in entries:
        if entry.get('type') != 'commands':
            continue

        data = entry.get('data', {})

        cmds.append({
            'command': entry['name'],
            'time': data['time'] - start_time,  # Relative to start
            'args': data.get('args', []),
            'kwargs': data.get('kwargs', {}),
        })
    return cmds



if __name__ == '__main__':
    # URI = 'radio://0/6/1M/E7E7E7E703'
    # with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    commands = load_commands('../recap_logs/3_poking.json')
    def execute_commands(cmds):
        """
        Reads a JSON command log and executes it, routing to either
        the standard Commander or the HighLevelCommander.
        """
        # logger.info(f"Executing log with {len(cmds)} entries...")
        start_real_time = time.time()

        for entry in cmds:
            target_log_time = entry['time']
            command_full_name = entry['command']
            args = entry['args']
            kwargs = entry['kwargs']

            current_elapsed = time.time() - start_real_time
            wait_duration = target_log_time - current_elapsed
            if wait_duration > 0:
                # self._safe_sleep(wait_duration)
                print(f"Wait: {wait_duration}")

            parts = command_full_name.split(".")
            if len(parts) != 2:
                # logger.info(f"Skipping malformed command: {command_full_name}")
                continue

            prefix, method_name = parts
            print(parts)

    execute_commands(commands)
