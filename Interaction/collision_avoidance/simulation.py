#!/usr/bin/env python3
"""
APF-based swarm collision avoidance simulation for FLS LightBenders.

Visualises the front view (XZ plane) as a matplotlib animation.
The UI-LB follows a user-supplied set of 3-D waypoints; all I-LBs
run Artificial Potential Field control (repulsion from UI-LB +
attraction to their assigned goal).

Usage examples
--------------
# Default waypoints, lb6 as UI-LB:
  python simulation.py

# Specify UI-LB and explicit waypoints (space-separated "x,y,z"):
  python simulation.py --ui-lb lb2 \
      --waypoints "-0.16,-0.01,1.32  0.3,0.0,1.0  0.0,0.0,0.8  -0.16,-0.01,1.32"

# Tune APF gains / detection threshold:
  python simulation.py --d-detect 0.4 --eta 0.8 --zeta 1.2 --v-max 0.4

# Save to a custom file:
  python simulation.py --output my_run.mp4

# Run the physics at 1000 Hz but render a 30 fps video:
  python simulation.py --dt 0.001 --render-fps 30

# Use a 50 Hz measured velocity profile with UI-LB dynamics limits:
  python simulation.py --ui-vel-hz 50 --ui-v-max 1.0 --ui-a-max 2.0
"""

import argparse
import csv
import os
import re
import sys
from datetime import datetime

import matplotlib

matplotlib.use("Agg")  # headless rendering
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

from pid_controller import CascadedPIDController

# ── Default APF / simulation parameters ───────────────────────────────────────
DT = 0.0025  # simulation time-step (s)
V_MAX = 2  # maximum APF-commanded speed (m/s)
UI_SPEED = 0.25  # speed at which UI-LB travels along waypoints (m/s)
ETA = 0.5  # repulsive gain η
ZETA = 0  # attractive gain ζ
D_DETECT = 0.47  # detection / repulsion radius d_detect (m)

# I-LB dynamics — cascaded PID
V_MAX_ILB = 2.0  # maximum I-LB velocity (m/s)
A_MAX_ILB = 5.0  # maximum I-LB acceleration (m/s²)
COLLISION_DIST = 0.25  # collision threshold (m) — 250 mm
SETTLE_TIME = 3.0  # seconds to hold UI-LB at final position after trajectory
INTERACTION_VEL_HZ = 50.0  # source sample rate of INTERACTION_VEL (Hz)
UI_V_MAX = 5.0  # maximum UI-LB velocity (m/s)
UI_A_MAX = 10.0  # maximum UI-LB acceleration (m/s²)

INTERACTION_VEL = [
    [-0.039, 0.241, 0.0],
    [-0.043, 0.283, 0.0],
    [-0.058, 0.337, 0.0],
    [-0.073, 0.394, 0.0],
    [-0.089, 0.452, 0.0],
    [-0.113, 0.507, 0.0],
    [-0.143, 0.6, 0.0],
    [-0.158, 0.655, 0.0],
    [-0.165, 0.7, 0.0],
    [-0.169, 0.73, 0.0],
    [-0.183, 0.757, 0.0],
    [-0.194, 0.772, 0.0],
    [-0.208, 0.771, 0.0],
    [-0.217, 0.766, 0.0],
    [-0.223, 0.758, 0.0],
    [-0.232, 0.75, 0.0],
    [-0.245, 0.736, 0.0],
    [-0.251, 0.726, 0.0],
    [-0.251, 0.723, 0.0],
    [-0.254, 0.718, 0.0],
    [-0.256, 0.713, 0.0],
    [-0.256, 0.706, 0.0],
    [-0.255, 0.693, 0.0],
    [-0.255, 0.685, 0.0],
    [-0.255, 0.675, 0.0],
    [-0.25, 0.657, 0.0],
    [-0.241, 0.624, 0.0],
    [-0.231, 0.596, 0.0],
    [-0.22, 0.569, 0.0],
    [-0.205, 0.532, 0.0],
    [-0.188, 0.498, 0.0],
    [-0.175, 0.454, 0.0],
    [-0.164, 0.409, 0.0],
    [-0.15, 0.359, 0.0],
    [-0.139, 0.31, 0.0],
    [-0.127, 0.26, 0.0],
    [-0.112, 0.212, 0.0],
    [-0.099, 0.138, 0.0]
]

INTERACTION_VEL_HZ = INTERACTION_VEL_HZ / 2  # halve sample rate → 2× Y distance at same velocity, 2× duration


# ── YAML parsing ──────────────────────────────────────────────────────────────

def _parse_target(target_str: str) -> np.ndarray:
    """Convert '[x, y, z, yaw]' string (with or without brackets) to xyz array."""
    nums = [float(v.strip()) for v in target_str.strip("[] \n").split(",")]
    return np.array(nums[:3])


def parse_mission(yaml_path: str) -> tuple:
    """
    Return (drones, servos, pointers) where
      drones   = {lb_id: np.array([x, y, z])}
      servos   = {lb_id: (angle1_deg, angle2_deg)}  — first [a,b] of servo list
      pointers = {lb_id: (p0, p1)}                  — first [a,b] of pointer list

    Servo angles (degrees) and pointer values (0-50 scale) are both taken from
    the first pair in their respective YAML lists.
    """
    with open(yaml_path) as fh:
        raw = fh.read()

    drones: dict = {}
    servos: dict = {}
    pointers: dict = {}

    _first_pair_pat = (
        r"^#?\s*(lb\w+):\s*\n"
        r"(?:#?[^\n]*\n){{0,15}}?"  # Doubled braces here
        r"#?\s*{key}:\s*\[\s*\[\s*([^\]]+)\]"
    )

    for key, target in (("target", drones), ("servos", servos), ("pointers", pointers)):
        if key == "target":
            pat = re.compile(
                r"^#?\s*(lb\w+):\s*\n"
                r"(?:#?[^\n]*\n){0,10}?"
                r"#?\s*target:\s*(\[[^\]]+\])",
                re.MULTILINE,
            )
            for m in pat.finditer(raw):
                lb_id = m.group(1)
                if lb_id not in target:
                    target[lb_id] = _parse_target(m.group(2))
        else:
            pat = re.compile(
                _first_pair_pat.format(key=key),
                re.MULTILINE,
            )
            for m in pat.finditer(raw):
                lb_id = m.group(1)
                if lb_id not in target:
                    vals = [float(v.strip()) for v in m.group(2).split(",")]
                    if len(vals) >= 2:
                        target[lb_id] = (vals[0], vals[1])

    if not drones:
        raise ValueError(f"No drone definitions found in {yaml_path}")

    return drones, servos, pointers


# ── APF math ──────────────────────────────────────────────────────────────────

def apf_velocity(
        lb_pos: np.ndarray,
        goal_pos: np.ndarray,
        ui_pos: np.ndarray,
        eta: float,
        zeta: float,
        d_detect: float,
        v_max: float,
) -> np.ndarray:
    """
    Compute the APF-commanded velocity for one I-LB.

    v_att = -ζ (P_LB - P_goal)
    v_rep = η (1/d - 1/d_detect) · (1/d²) · (P_LB - P_UI) / d   if d ≤ d_detect
    v_des = v_att + v_rep   (clipped to v_max)
    """
    # --- attractive component ---
    v_att = -zeta * (lb_pos - goal_pos)

    # --- repulsive component ---
    diff = lb_pos - ui_pos
    d = float(np.linalg.norm(diff))
    if d <= d_detect and d > 1e-9:
        scale = eta * (1.0 / d - 1.0 / d_detect) / d ** 2
        v_rep = scale * (diff / d)
    else:
        v_rep = np.zeros(3)

    v_des = v_att + v_rep

    # --- clip to v_max ---
    speed = float(np.linalg.norm(v_des))
    if speed > v_max:
        v_des = v_max * v_des / speed

    return v_des


# ── Waypoint interpolation ─────────────────────────────────────────────────────

def interpolate_waypoints(
        waypoints: np.ndarray, speed: float, dt: float
) -> np.ndarray:
    """
    Return an (N, 3) array of per-timestep positions that traverse
    *waypoints* at constant *speed*.
    """
    traj = [waypoints[0].copy()]
    for target in waypoints[1:]:
        origin = traj[-1]
        segment = target - origin
        dist = float(np.linalg.norm(segment))
        if dist < 1e-9:
            continue
        n_steps = max(1, int(round(dist / (speed * dt))))
        for k in range(1, n_steps + 1):
            traj.append(origin + segment * k / n_steps)
    return np.array(traj)


def velocities_to_trajectory(
        start: np.ndarray,
        vel_list: list,
        dt: float,
        source_hz: float = INTERACTION_VEL_HZ,
        v_max: float = UI_V_MAX,
        a_max: float = UI_A_MAX,
) -> np.ndarray:
    """
    Build a UI-LB trajectory from a velocity profile sampled at *source_hz*.

    The input profile is resampled to the simulation timestep *dt*, then
    tracked with acceleration and velocity limits so the resulting motion
    remains consistent for any value of *dt*.
    """
    vel_targets = resample_velocity_profile(vel_list, source_hz, dt)
    traj, _ = integrate_velocity_profile(start, vel_targets, dt, v_max, a_max)
    return traj


def resample_velocity_profile(
        vel_list: list,
        source_hz: float,
        dt: float,
) -> np.ndarray:
    """
    Resample a velocity profile from *source_hz* to the simulation timestep.

    Each input sample is treated as covering one source interval, so the total
    commanded duration is ``len(vel_list) / source_hz`` seconds.
    """
    vel_arr = np.asarray(vel_list, dtype=float)
    if vel_arr.ndim != 2 or vel_arr.shape[1] != 3:
        raise ValueError("Velocity profile must be an (N, 3) array-like")
    if len(vel_arr) == 0:
        return np.zeros((0, 3))
    if source_hz <= 0:
        raise ValueError("source_hz must be > 0")

    source_dt = 1.0 / source_hz
    source_times = np.arange(len(vel_arr)) * source_dt
    duration = len(vel_arr) * source_dt
    sim_times = np.arange(0.0, duration, dt)
    if len(sim_times) == 0 or sim_times[-1] < duration - 0.5 * dt:
        sim_times = np.append(sim_times, duration)

    resampled = np.column_stack([
        np.interp(sim_times, source_times, vel_arr[:, axis],
                  left=vel_arr[0, axis], right=vel_arr[-1, axis])
        for axis in range(3)
    ])
    return resampled


def integrate_velocity_profile(
        start: np.ndarray,
        vel_targets: np.ndarray,
        dt: float,
        v_max: float,
        a_max: float,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Track desired UI-LB velocity setpoints with acceleration and speed limits.

    Returns
    -------
    traj        : (N+1, 3) positions
    actual_vels : (N, 3) realised velocities after saturation
    """
    traj = [np.asarray(start, dtype=float).copy()]
    actual_vels = []
    velocity = np.zeros(3)

    for v_des in np.asarray(vel_targets, dtype=float):
        speed_des = float(np.linalg.norm(v_des))
        if speed_des > v_max:
            v_des = v_des * (v_max / speed_des)

        delta_v = v_des - velocity
        delta_speed = float(np.linalg.norm(delta_v))
        max_delta = a_max * dt
        if delta_speed > max_delta and delta_speed > 1e-12:
            delta_v = delta_v * (max_delta / delta_speed)

        velocity = velocity + delta_v

        speed = float(np.linalg.norm(velocity))
        if speed > v_max:
            velocity = velocity * (v_max / speed)

        actual_vels.append(velocity.copy())
        traj.append(traj[-1] + velocity * dt)

    return np.array(traj), np.array(actual_vels)


def trajectory_to_velocity_targets(traj: np.ndarray, dt: float) -> np.ndarray:
    """Convert a position trajectory into per-step velocity setpoints."""
    if len(traj) < 2:
        return np.zeros((0, 3))
    return np.diff(traj, axis=0) / dt
    return np.array(traj)


# ── Simulation loop ────────────────────────────────────────────────────────────

def run_simulation(
        drones: dict,
        ui_lb_id: str,
        ui_traj: np.ndarray,
        eta: float = ETA,
        zeta: float = ZETA,
        d_detect: float = D_DETECT,
        v_max: float = V_MAX,
        dt: float = DT,
        v_max_ilb: float = V_MAX_ILB,
        a_max_ilb: float = A_MAX_ILB,
        collision_dist: float = COLLISION_DIST,
        use_ui_vel: bool = False,
        stop_after_clear: bool = True,
        use_pos_cmd: bool = False,
        pos_kp: float = 2.0,
):
    """
    Simulate and return per-frame position histories.

    Parameters
    ----------
    ui_traj          : (N, 3) array of pre-computed UI-LB positions per timestep.
    use_ui_vel       : If True, I-LBs are commanded with the UI-LB’s velocity
                       instead of APF during active motion.  When UI-LB stops,
                       the last I-LB velocity command is held until all I-LBs
                       clear d_detect.
    stop_after_clear : If True (default), I-LBs stop moving once all have
                       cleared d_detect.  If False, the last command continues
                       until the safety cap is reached.
    use_pos_cmd      : If True, issue position setpoints to the controller
                       instead of velocity setpoints.  The position target each
                       step is  p_cmd = ilb_pos + v_cmd / pos_kp,  so the outer
                       P loop produces v_cmd at the current location.
    pos_kp           : Outer position P gain used when use_pos_cmd is True.

    Returns
    -------
    history              : dict  {lb_id: list of np.ndarray(3)}
    n_steps              : int
    cumulative_collisions: list[int]  — running collision count per frame
    """
    ilb_ids = [k for k in drones if k != ui_lb_id]
    ilb_pos = {k: drones[k].copy() for k in ilb_ids}
    ilb_goal = {k: drones[k].copy() for k in ilb_ids}
    all_ids = sorted(drones.keys())

    # Cascaded PID controller for I-LB swarm dynamics
    controller = CascadedPIDController(v_max=v_max_ilb, a_max=a_max_ilb)

    # Pre-compute UI-LB velocity at each step (finite difference)
    ui_vels = np.diff(ui_traj, axis=0) / dt  # (N-1, 3)

    # histories — store every frame
    history: dict = {k: [ilb_pos[k].copy()] for k in ilb_ids}
    history[ui_lb_id] = [ui_traj[0].copy()]

    # collision tracking (dynamic list; we’ll extend if needed)
    cumulative_collisions = [0]
    _count_frame_collisions_dyn(history, all_ids, 0, collision_dist,
                                cumulative_collisions)

    n_traj = len(ui_traj)

    # ── Phase 1: follow the pre-computed trajectory ───────────────────────
    last_vel_cmd = np.zeros(3)
    last_pos_cmd = {k: ilb_pos[k].copy() for k in ilb_ids}
    for step in range(1, n_traj):
        ui_pos = ui_traj[step]
        history[ui_lb_id].append(ui_pos.copy())

        closest_id = min(ilb_ids,
                         key=lambda k: np.linalg.norm(ilb_pos[k] - ui_pos))

        if use_ui_vel:
            v_cmd = ui_vels[step - 1]  # UI-LB’s own velocity
        else:
            v_cmd = apf_velocity(
                ilb_pos[closest_id], ilb_goal[closest_id], ui_pos,
                eta, zeta, d_detect, v_max,
            )
        
        if np.linalg.norm(last_vel_cmd) < 1e-9 and np.linalg.norm(v_cmd) < 1e-9:
            controller.reset()
            delta_p = np.zeros(3)
        elif use_pos_cmd:
            # Position target: one step ahead in the direction v_cmd points,
            # scaled so the outer P loop reproduces v_cmd at the current position.
            p_cmd = ilb_pos[closest_id] + v_cmd / pos_kp
            last_pos_cmd = {k: ilb_pos[k] + v_cmd / pos_kp for k in ilb_ids}
            delta_p = controller.step_position(p_cmd, ilb_pos[closest_id], dt, pos_kp)
        else:
            delta_p = controller.step_velocity(v_cmd, dt)

        last_vel_cmd = v_cmd
        for lb_id in ilb_ids:
            ilb_pos[lb_id] = ilb_pos[lb_id] + delta_p
            history[lb_id].append(ilb_pos[lb_id].copy())

        cumulative_collisions.append(0)
        _count_frame_collisions_dyn(history, all_ids, step, collision_dist,
                                    cumulative_collisions)

    # ── Phase 2 (use_ui_vel only): hold last command until all I-LBs clear ──
    if use_ui_vel:
        ui_pos = ui_traj[-1].copy()
        step = n_traj - 1
        max_extra = int(30.0 / dt)  # safety cap: 30 s
        for _ in range(max_extra):
            # Check if all I-LBs are outside d_detect
            all_clear = all(np.linalg.norm(ilb_pos[k] - ui_pos) > (d_detect - 1e-6)
                            for k in ilb_ids)
            if all_clear and stop_after_clear:
                break

            step += 1
            history[ui_lb_id].append(ui_pos.copy())

            if use_pos_cmd:
                closest_id = min(ilb_ids,
                                 key=lambda k: np.linalg.norm(ilb_pos[k] - ui_pos))
                delta_p = controller.step_position(
                    last_pos_cmd[closest_id], ilb_pos[closest_id], dt, pos_kp
                )
            else:
                delta_p = controller.step_velocity(last_vel_cmd, dt)

            for lb_id in ilb_ids:
                ilb_pos[lb_id] = ilb_pos[lb_id] + delta_p
                history[lb_id].append(ilb_pos[lb_id].copy())

            cumulative_collisions.append(0)
            _count_frame_collisions_dyn(history, all_ids, step, collision_dist,
                                        cumulative_collisions)

    n_steps = len(history[ui_lb_id])
    return history, n_steps, cumulative_collisions


def _count_frame_collisions_dyn(history, all_ids, frame, collision_dist, cum):
    """Count pairs closer than *collision_dist* at *frame*, update *cum*."""
    count = 0
    for i in range(len(all_ids)):
        for j in range(i + 1, len(all_ids)):
            d = float(np.linalg.norm(
                np.array(history[all_ids[i]][frame]) -
                np.array(history[all_ids[j]][frame])))
            if d < collision_dist:
                count += 1
    cum[frame] = (cum[frame - 1] if frame > 0 else 0) + count


def build_render_indices(n_steps: int, dt: float, render_fps: float) -> np.ndarray:
    """
    Return simulation-step indices to render at *render_fps*.

    The simulation always runs at *dt*; rendering samples the already-computed
    history at a lower or higher presentation rate as needed.
    """
    if n_steps <= 0:
        return np.array([], dtype=int)
    if render_fps <= 0:
        raise ValueError("render_fps must be > 0")

    duration = max(0.0, (n_steps - 1) * dt)
    render_dt = 1.0 / render_fps
    render_times = np.arange(0.0, duration + 0.5 * render_dt, render_dt)
    render_indices = np.rint(render_times / dt).astype(int)
    render_indices = np.clip(render_indices, 0, n_steps - 1)

    if render_indices[-1] != n_steps - 1:
        render_indices = np.append(render_indices, n_steps - 1)

    return np.unique(render_indices)


def compute_separation_metrics(history: dict, ui_lb_id: str) -> dict:
    """Return minimum separation metrics to help tune APF gains."""
    lb_ids = sorted(history.keys())
    ilb_ids = [lb_id for lb_id in lb_ids if lb_id != ui_lb_id]
    n_steps = len(history[ui_lb_id])

    min_pair_dist = float("inf")
    min_ui_dist = float("inf")

    for frame in range(n_steps):
        positions = {lb_id: np.asarray(history[lb_id][frame]) for lb_id in lb_ids}
        for ilb_id in ilb_ids:
            d_ui = float(np.linalg.norm(positions[ilb_id] - positions[ui_lb_id]))
            min_ui_dist = min(min_ui_dist, d_ui)
        for i in range(len(lb_ids)):
            for j in range(i + 1, len(lb_ids)):
                d_pair = float(np.linalg.norm(
                    positions[lb_ids[i]] - positions[lb_ids[j]]
                ))
                min_pair_dist = min(min_pair_dist, d_pair)

    return {
        "min_pair_dist": min_pair_dist,
        "min_ui_dist": min_ui_dist,
    }


def append_run_summary(csv_path: str, row: dict):
    """Append one simulation/run summary row to a CSV file."""
    fieldnames = [
        "timestamp",
        "mission",
        "ui_lb",
        "mode",
        "rendered",
        "output",
        "dt",
        "render_fps",
        "ui_vel_hz",
        "ui_v_max",
        "ui_a_max",
        "d_detect",
        "eta",
        "zeta",
        "v_max",
        "settle_time",
        "steps",
        "duration_s",
        "total_collisions",
        "min_pair_dist",
        "min_ui_dist",
    ]
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    file_exists = os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        if not file_exists:
            writer.writeheader()
        writer.writerow({key: row.get(key, "") for key in fieldnames})


def colliding_ids_at_frame(
        history: dict,
        lb_ids: list,
        frame: int,
        collision_dist: float,
) -> set[str]:
    """Return the set of LightBenders involved in a collision at *frame*."""
    colliding = set()
    for i in range(len(lb_ids)):
        for j in range(i + 1, len(lb_ids)):
            d = float(np.linalg.norm(
                np.asarray(history[lb_ids[i]][frame]) -
                np.asarray(history[lb_ids[j]][frame])
            ))
            if d < collision_dist:
                colliding.add(lb_ids[i])
                colliding.add(lb_ids[j])
    return colliding


# ── Colours (white-background theme) ──────────────────────────────────────────

_ILB_COLOR = "#1565C0"
_UI_COLOR = "#EF6C00"
_COLLISION_COLOR = "#D32F2F"
_CLOSEST_RING = "#FFD600"  # gold highlight ring for the closest I-LB

# Display scale: metres → millimetres
_MM = 1000.0

# 2-D view table: (horiz_world_idx, vert_world_idx, xlabel, ylabel, title)
_2D_VIEWS = [
    (1, 2, "Y (mm)", "Z (mm)", "Front  (looking −X)"),
    (0, 2, "X (mm)", "Z (mm)", "Side   (looking −Y)"),
    (1, 0, "Y (mm)", "X (mm)", "Top    (looking −Z)"),
]

# ── Animation ─────────────────────────────────────────────────────────────────

# ── LED / rod geometry ─────────────────────────────────────────────────────
#   50 LEDs total, spaced 6.24 mm apart.
#   Rod 1 (indices 0–25): 26 LEDs — index 0 at outer tip, 25 at centre (r=0).
#   Rod 2 (indices 26–49): 24 LEDs — index 26 near centre, 49 at outer tip.
_LED_SPACING = 0.00624  # metres between consecutive LEDs
_ROD1_LEN = 25 * _LED_SPACING  # 0.156 m  (26 LEDs)
_ROD2_LEN = 24 * _LED_SPACING  # 0.14976 m  (24 LEDs)
_V_TYPES = {'lb2', 'lb3', 'lb8'}  # V-type drones
_PTR_CENTER = 25  # pointer value at the drone centre (LED 25, r = 0)


def _pointer_to_offset(p: float, angles: tuple, is_v: bool) -> tuple:
    """
    Map pointer value p (0–50 scale) to a (dy, dz) offset from the drone centre.

    Pointer semantics (centre = 25, the base where the two rods meet):
      H-type: 0..25 → servo[0] (rod 1);  25..50 → servo[1] (rod 2)
      V-type: 0..25 → servo[1] (rod 1);  25..50 → servo[0] (rod 2)

    Distance from centre = |p − 25| × LED_SPACING.
    Angle convention: clockwise from +Y;  cos → ΔY,  -sin → ΔZ  (YZ plane, looking −X)
    """
    if p <= _PTR_CENTER:
        ang_deg = angles[1] if is_v else angles[0]
        dist = (_PTR_CENTER - p) * _LED_SPACING
    else:
        ang_deg = angles[0] if is_v else angles[1]
        dist = (p - _PTR_CENTER) * _LED_SPACING
    ang_rad = np.radians(ang_deg % 360)
    return dist * np.cos(ang_rad), -dist * np.sin(ang_rad)


def _ghost_rod_specs(angles: tuple, is_v: bool):
    """
    Return [(angle_deg, rod_length), ...] pairing each servo angle with its
    physical rod length.

      H-type: angles[0] → rod 1 (0.156 m), angles[1] → rod 2 (0.14976 m)
      V-type: angles[1] → rod 1,             angles[0] → rod 2  (reversed)
    """
    if is_v:
        return [(angles[1], _ROD1_LEN), (angles[0], _ROD2_LEN)]
    else:
        return [(angles[0], _ROD1_LEN), (angles[1], _ROD2_LEN)]


def _rod_segment_endpoints(ptrs: tuple, angles: tuple, is_v: bool):
    """
    Return ((dy0, dz0), (dy1, dz1)) pairs for line A and line B.

    If both pointers are on the same rod, line A spans between them and
    line B is zero-length (invisible).  If they span the centre, line A
    goes from p0 to centre and line B from centre to p1.
    """
    off0 = _pointer_to_offset(ptrs[0], angles, is_v)
    off1 = _pointer_to_offset(ptrs[1], angles, is_v)
    same_side = (ptrs[0] <= _PTR_CENTER) == (ptrs[1] <= _PTR_CENTER)
    if same_side:
        # Both on the same rod — single segment between p0 and p1
        return (off0, off1), ((0.0, 0.0), (0.0, 0.0))
    else:
        # Span centre — two segments through drone position
        return (off0, (0.0, 0.0)), ((0.0, 0.0), off1)


def make_animation(
        history: dict,
        ui_lb_id: str,
        drones: dict,
        d_detect: float,
        output: str,
        dt: float,
        render_fps: float,
        servos: dict = None,
        pointers: dict = None,
        cumulative_collisions: list = None,
        view_height: float = 1.0,
        snapshot_frame: int = None,
        render_steps: int = None,
):
    """
    2×2 quad-view animation (white background):
      top-left  → Front  (Y right, Z up)
      top-right → Side   (X right, Z up)
      bot-left  → Top    (Y right, X up)
      bot-right → Rod view (YZ plane, looking −X)

    Features
    --------
    - Gold highlight ring on the I-LB closest to UI-LB each frame
    - Collision counter display (pairs closer than 250 mm)
    - All four panels the same physical size
    - All labels in millimetres
    """
    if servos is None:
        servos = {}
    if pointers is None:
        pointers = {}

    ilb_ids = [k for k in sorted(drones.keys()) if k != ui_lb_id]
    lb_ids = sorted(drones.keys())
    n_steps = len(history[ui_lb_id])
    render_cap = min(n_steps, render_steps) if render_steps is not None else n_steps
    render_indices = build_render_indices(render_cap, dt, render_fps)
    if cumulative_collisions is None:
        cumulative_collisions = [0] * n_steps

    # ── colour map ────────────────────────────────────────────────────────────
    color_map: dict = {}
    for lb_id in lb_ids:
        if lb_id == ui_lb_id:
            color_map[lb_id] = _UI_COLOR
        else:
            color_map[lb_id] = _ILB_COLOR

    # ── world-space axis limits ───────────────────────────────────────────────
    margin = d_detect + 0.25
    x_all = np.array([p[0] for lb in lb_ids for p in history[lb]])
    y_all = np.array([p[1] for lb in lb_ids for p in history[lb]])
    z_all = np.array([p[2] for lb in lb_ids for p in history[lb]])

    x_min, x_max = x_all.min() - margin, x_all.max() + margin
    y_min, y_max = y_all.min() - margin, y_all.max() + margin

    z_ctr = float(np.mean([drones[k][2] for k in lb_ids]))
    z_win_min = min(z_ctr - view_height / 2.0, z_all.min() - 0.05)
    z_win_max = max(z_ctr + view_height / 2.0, z_all.max() + 0.05)

    # Make each 2-D view's two axes span the same range so panels fill equally
    def _square_lims(lo1, hi1, lo2, hi2):
        """Expand both ranges to the same span, keeping their centres."""
        span = max(hi1 - lo1, hi2 - lo2)
        c1, c2 = (lo1 + hi1) / 2, (lo2 + hi2) / 2
        return c1 - span / 2, c1 + span / 2, c2 - span / 2, c2 + span / 2

    # Compute limits in metres, then scale to mm for display
    fy0, fy1, fz0, fz1 = _square_lims(y_min, y_max, z_win_min, z_win_max)
    sx0, sx1, sz0, sz1 = _square_lims(x_min, x_max, z_win_min, z_win_max)
    ty0, ty1, tx0, tx1 = _square_lims(y_min, y_max, x_min, x_max)

    # Scale to mm
    fy0 *= _MM; fy1 *= _MM; fz0 *= _MM; fz1 *= _MM
    sx0 *= _MM; sx1 *= _MM; sz0 *= _MM; sz1 *= _MM
    ty0 *= _MM; ty1 *= _MM; tx0 *= _MM; tx1 *= _MM

    _view_lims = [
        (fy0, fy1, fz0, fz1),
        (sx0, sx1, sz0, sz1),
        (ty0, ty1, tx0, tx1),
    ]

    # ── figure — constrained_layout keeps all panels the same box size ────────
    fig = plt.figure(figsize=(16, 10), constrained_layout=True)
    fig.patch.set_facecolor("white")

    ax_front = fig.add_subplot(2, 2, 1)
    ax_side = fig.add_subplot(2, 2, 2)
    ax_top = fig.add_subplot(2, 2, 3)
    ax_rod = fig.add_subplot(2, 2, 4)  # rod view — replaces 3-D panel

    _2d_axes = [ax_front, ax_side, ax_top]

    for ax, (hi, vi, xl, yl, ttl), (hl, hh, vl, vh) in zip(
            _2d_axes, _2D_VIEWS, _view_lims
    ):
        ax.set_facecolor("white")
        ax.set_xlim(hl, hh)
        ax.set_ylim(vl, vh)
        ax.set_xlabel(xl, fontsize=9, color="#333333")
        ax.set_ylabel(yl, fontsize=9, color="#333333")
        ax.set_title(ttl, fontsize=10, color="#111111", pad=4, fontweight="bold")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.35, linestyle="--", color="#cccccc")
        ax.tick_params(colors="#555555", labelsize=7)
        for sp in ax.spines.values():
            sp.set_color("#aaaaaa")

    # Rod-view panel — YZ plane, looking along −X (front of drone)
    ax_rod.set_facecolor("white")
    ax_rod.set_xlim(fy0, fy1)
    ax_rod.set_ylim(fz0, fz1)
    ax_rod.set_xlabel("Y (mm)", fontsize=9, color="#333333")
    ax_rod.set_ylabel("Z (mm)", fontsize=9, color="#333333")
    ax_rod.set_title("Rod View  (YZ, looking −X)", fontsize=10,
                     color="#111111", pad=4, fontweight="bold")
    ax_rod.set_aspect("equal", adjustable="box")
    ax_rod.grid(True, alpha=0.35, linestyle="--", color="#cccccc")
    ax_rod.tick_params(colors="#555555", labelsize=7)
    for sp in ax_rod.spines.values():
        sp.set_color("#aaaaaa")

    fig.suptitle(
        f"APF Swarm  |  UI-LB = {ui_lb_id}  |  d_detect = {d_detect * _MM:.0f} mm",
        fontsize=13, color="#111111", fontweight="bold",
    )

    # ── static goal markers (2-D panels only) ────────────────────────────────
    for lb_id in ilb_ids:
        gp = drones[lb_id]
        for ax, (hi, vi, *_) in zip(_2d_axes, _2D_VIEWS):
            ax.plot(gp[hi] * _MM, gp[vi] * _MM, "x", color=color_map[lb_id],
                    markersize=9, markeredgewidth=1.8, alpha=0.4, zorder=2)

    # ── animated artists: 2-D dots / trails / circles / halos ─────────────────
    TRAIL_LEN = 80

    dots_2d = {ax: {} for ax in _2d_axes}
    trails_2d = {ax: {} for ax in _2d_axes}
    circles_2d = {ax: {} for ax in _2d_axes}
    halos_2d = {ax: {} for ax in _2d_axes}

    for lb_id in lb_ids:
        p0 = history[lb_id][0]
        is_ui = lb_id == ui_lb_id
        label = f"{lb_id}{'  ← UI-LB' if is_ui else ''}"
        circ_lw = 1.8 if is_ui else 1.0
        circ_alpha = 0.6 if is_ui else 0.35

        for ax, (hi, vi, *_) in zip(_2d_axes, _2D_VIEWS):
            dot, = ax.plot(p0[hi] * _MM, p0[vi] * _MM, "o",
                           color=color_map[lb_id],
                           markersize=11 if is_ui else 8,
                           zorder=7 if is_ui else 5,
                           label=label)
            dots_2d[ax][lb_id] = dot

            trail, = ax.plot([], [], "-", color=color_map[lb_id],
                             alpha=0.3, linewidth=1.2)
            trails_2d[ax][lb_id] = trail

            circ = plt.Circle(
                (p0[hi] * _MM, p0[vi] * _MM), d_detect * _MM,
                color=color_map[lb_id], fill=False,
                linestyle="--", linewidth=circ_lw, alpha=circ_alpha, zorder=4,
            )
            ax.add_patch(circ)
            circles_2d[ax][lb_id] = circ

            if not is_ui:
                halo = plt.Circle(
                    (p0[hi] * _MM, p0[vi] * _MM), d_detect * 0.18 * _MM,
                    color=_CLOSEST_RING, fill=False,
                    linewidth=2.5, alpha=0.0, zorder=8,
                )
                ax.add_patch(halo)
                halos_2d[ax][lb_id] = halo

    # ── animated artists: rod view ────────────────────────────────────────────
    # Full rods drawn in gray (ghost), then two pointer segments in colour on top.
    rod_ghosts = {}  # {lb_id: [(line_artist, ang_rad, rod_len_mm), ...]}
    rod_segments = {}  # {lb_id: (ln_a, ln_b, angles, ptrs, is_v)}
    rod_labels = {}  # {lb_id: Text}

    for lb_id in lb_ids:
        p0 = history[lb_id][0]
        y0, z0 = float(p0[1]) * _MM, float(p0[2]) * _MM
        angles = servos.get(lb_id, (0.0, 90.0))
        ptrs = pointers.get(lb_id, (0.0, 50.0))
        is_v = lb_id in _V_TYPES

        # Ghost rods — each rod has its own length
        ghosts = []
        for ang_deg, rod_len in _ghost_rod_specs(angles, is_v):
            ang_rad = np.radians(ang_deg % 360)
            rod_mm = rod_len * _MM
            gy = y0 + rod_mm * np.cos(ang_rad)
            gz = z0 - rod_mm * np.sin(ang_rad)  # clockwise
            gln, = ax_rod.plot(
                [y0, gy], [z0, gz], "-",
                color="#999999", linewidth=2.5, solid_capstyle="round",
                alpha=0.2, zorder=4,
            )
            ghosts.append((gln, ang_rad, rod_mm))
        rod_ghosts[lb_id] = ghosts

        # Coloured pointer segments on top (offsets are in metres → scale)
        (seg_a_start, seg_a_end), (seg_b_start, seg_b_end) = \
            _rod_segment_endpoints(ptrs, angles, is_v)
        ln_a, = ax_rod.plot(
            [y0 + seg_a_start[0] * _MM, y0 + seg_a_end[0] * _MM],
            [z0 + seg_a_start[1] * _MM, z0 + seg_a_end[1] * _MM], "-",
            color=color_map[lb_id],
            linewidth=2.5, solid_capstyle="round",
            alpha=0.85, zorder=5,
        )
        ln_b, = ax_rod.plot(
            [y0 + seg_b_start[0] * _MM, y0 + seg_b_end[0] * _MM],
            [z0 + seg_b_start[1] * _MM, z0 + seg_b_end[1] * _MM], "-",
            color=color_map[lb_id],
            linewidth=2.5, solid_capstyle="round",
            alpha=0.85, zorder=5,
        )
        rod_segments[lb_id] = (ln_a, ln_b, angles, ptrs, is_v)

        lbl = ax_rod.text(
            y0, z0, f" {lb_id}",
            fontsize=6.5, color=color_map[lb_id],
            ha="left", va="center", zorder=6,
        )
        rod_labels[lb_id] = lbl

    # ── legend & text artists ─────────────────────────────────────────────────
    ax_front.legend(loc="upper right", fontsize=7, framealpha=0.85,
                    facecolor="white", edgecolor="#aaaaaa")

    collision_text = fig.text(
        0.5, 0.003, "",
        ha="center", va="bottom", fontsize=10,
        color="#C62828", family="monospace", fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.3", fc="#fff3f3",
                  ec="#C62828", lw=0.8, alpha=0.9),
    )
    time_text = fig.text(
        0.01, 0.003, "",
        ha="left", va="bottom", fontsize=9,
        color="#555555", family="monospace",
    )

    # ── update ────────────────────────────────────────────────────────────────
    def update(render_frame):
        sim_frame = int(render_indices[render_frame])
        ui_pos = history[ui_lb_id][sim_frame]
        colliding_ids = colliding_ids_at_frame(
            history, lb_ids, sim_frame, COLLISION_DIST
        )

        closest_id = min(
            ilb_ids,
            key=lambda k: np.linalg.norm(
                np.array(history[k][sim_frame]) - np.array(ui_pos)
            ),
        )

        # 2-D panels (all coordinates in mm)
        for ax, (hi, vi, *_) in zip(_2d_axes, _2D_VIEWS):
            for lb_id in lb_ids:
                pos = history[lb_id][sim_frame]
                active_color = (
                    _COLLISION_COLOR if lb_id in colliding_ids else color_map[lb_id]
                )
                dots_2d[ax][lb_id].set_data([pos[hi] * _MM], [pos[vi] * _MM])
                dots_2d[ax][lb_id].set_color(active_color)

                trail_indices = render_indices[max(0, render_frame - TRAIL_LEN):render_frame + 1]
                th = [history[lb_id][i][hi] * _MM for i in trail_indices]
                tv = [history[lb_id][i][vi] * _MM for i in trail_indices]
                trails_2d[ax][lb_id].set_data(th, tv)
                trails_2d[ax][lb_id].set_color(active_color)

                circles_2d[ax][lb_id].set_center((pos[hi] * _MM, pos[vi] * _MM))
                circles_2d[ax][lb_id].set_edgecolor(active_color)

            for lb_id in ilb_ids:
                pos = history[lb_id][sim_frame]
                is_closest = lb_id == closest_id
                halos_2d[ax][lb_id].set_center((pos[hi] * _MM, pos[vi] * _MM))
                halos_2d[ax][lb_id].set_alpha(0.9 if is_closest else 0.0)

        # Rod view — move ghost rods and two pointer segments (in mm)
        for lb_id in lb_ids:
            pos = history[lb_id][sim_frame]
            y0, z0 = float(pos[1]) * _MM, float(pos[2]) * _MM
            active_color = (
                _COLLISION_COLOR if lb_id in colliding_ids else color_map[lb_id]
            )
            # Ghost rods (each has its own length)
            for gln, ang_rad, rod_mm in rod_ghosts[lb_id]:
                gy = y0 + rod_mm * np.cos(ang_rad)
                gz = z0 - rod_mm * np.sin(ang_rad)  # clockwise
                gln.set_data([y0, gy], [z0, gz])
            # Pointer segments (offsets in metres → scale)
            ln_a, ln_b, angles, ptrs, is_v = rod_segments[lb_id]
            (sa_s, sa_e), (sb_s, sb_e) = \
                _rod_segment_endpoints(ptrs, angles, is_v)
            ln_a.set_data([y0 + sa_s[0] * _MM, y0 + sa_e[0] * _MM],
                          [z0 + sa_s[1] * _MM, z0 + sa_e[1] * _MM])
            ln_b.set_data([y0 + sb_s[0] * _MM, y0 + sb_e[0] * _MM],
                          [z0 + sb_s[1] * _MM, z0 + sb_e[1] * _MM])
            ln_a.set_color(active_color)
            ln_b.set_color(active_color)
            rod_labels[lb_id].set_color(active_color)
            rod_labels[lb_id].set_position((y0, z0))

        # Collision counter
        n_col = cumulative_collisions[sim_frame]
        collision_text.set_text(f"Total Collisions (<250 mm): {n_col}")
        time_text.set_text(f"t = {sim_frame * dt:.2f} s")

    # ── single-frame PNG or full animation ─────────────────────────────────────
    if snapshot_frame is not None:
        frame_idx = max(0, min(snapshot_frame, len(render_indices) - 1))
        update(frame_idx)
        fig.savefig(output, dpi=200, facecolor="white", bbox_inches="tight")
        plt.close(fig)
        sim_frame = int(render_indices[frame_idx])
        print(f"[done] snapshot render frame {frame_idx} (simulation step {sim_frame}) saved → {output}")
        return

    import sys, time as _time
    _t0 = _time.monotonic()

    def _progress(frame_i, total):
        pct = (frame_i + 1) / total * 100
        elapsed = _time.monotonic() - _t0
        eta = elapsed / (frame_i + 1) * (total - frame_i - 1) if frame_i > 0 else 0
        bar_len = 30
        filled = int(bar_len * (frame_i + 1) / total)
        bar = "█" * filled + "░" * (bar_len - filled)
        sys.stdout.write(
            f"\rRendering {bar} {pct:5.1f}%  "
            f"{frame_i+1}/{total}  [{elapsed:.0f}s<{eta:.0f}s]"
        )
        sys.stdout.flush()
        if frame_i + 1 == total:
            sys.stdout.write("\n")

    ani = animation.FuncAnimation(
        fig, update,
        frames=len(render_indices),
        interval=1000.0 / render_fps,
        blit=False,
    )

    writer = animation.FFMpegWriter(fps=render_fps, bitrate=3600)
    ani.save(output, writer=writer, progress_callback=_progress)
    plt.close(fig)
    print(f"[done] saved → {output}")


# ── CLI entry-point ────────────────────────────────────────────────────────────

def build_default_waypoints(start: np.ndarray) -> np.ndarray:
    """Default: move UI-LB straight in the +Y direction."""
    return np.array([
        start,
        start + np.array([0.0, 1.0, 0.0]),
    ])


def main(**overrides):
    """
    Run one simulation + render.

    Keyword arguments override the corresponding CLI defaults, e.g.
    ``main(mission='path/to/m.yaml', ui_lb='lb3', output='out.mp4')``.
    Keys use underscores (matching argparse dest names).
    """
    ap = argparse.ArgumentParser(
        description="FLS LightBender APF collision-avoidance simulation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument(
        "--mission",
        default=os.path.join(
            os.path.dirname(__file__),
            "..", "..", "mission", "ACM.yaml",
        ),
        help="Path to mission YAML  (default: ../../mission/ACM.yaml)",
    )
    ap.add_argument(
        "--ui-lb", default=None,
        help="ID of the drone the user interacts with  (default: first drone in mission)",
    )
    ap.add_argument(
        "--velocities", default=INTERACTION_VEL,
        metavar="'vx,vy,vz  vx,vy,vz ...'",
        help=(
            "Space-separated 3-D velocity vectors for the UI-LB, e.g. "
            "'-0.04,-0.24,0  -0.04,-0.28,0'. "
            "Trajectory is built from a source-rate velocity profile. "
            "Mutually exclusive with --waypoints."
        ),
    )
    ap.add_argument(
        "--ui-vel-hz", type=float, default=INTERACTION_VEL_HZ,
        help=f"Source sample rate of --velocities / INTERACTION_VEL in Hz  (default: {INTERACTION_VEL_HZ})",
    )
    ap.add_argument(
        "--ui-v-max", type=float, default=UI_V_MAX,
        help=f"Max UI-LB speed in m/s  (default: {UI_V_MAX})",
    )
    ap.add_argument(
        "--ui-a-max", type=float, default=UI_A_MAX,
        help=f"Max UI-LB acceleration in m/s^2  (default: {UI_A_MAX})",
    )
    ap.add_argument("--d-detect", type=float, default=D_DETECT,
                    help=f"Detection radius in metres  (default: {D_DETECT})")
    ap.add_argument("--eta", type=float, default=ETA,
                    help=f"Repulsive APF gain η  (default: {ETA})")
    ap.add_argument("--zeta", type=float, default=ZETA,
                    help=f"Attractive APF gain ζ  (default: {ZETA})")
    ap.add_argument("--v-max", type=float, default=V_MAX,
                    help=f"Max I-LB speed in m/s  (default: {V_MAX})")
    ap.add_argument("--dt", type=float, default=DT,
                    help=f"Simulation time-step in s  (default: {DT})")
    ap.add_argument(
        "--render-fps", type=float, default=30.0,
        help="Video frame rate used for rendering/snapshots  (default: 30.0)",
    )
    ap.add_argument(
        "--view-height", type=float, default=1.0,
        metavar="METRES",
        help="Vertical window size in metres, centred on drone cluster  (default: 1.0)",
    )
    ap.add_argument(
        "--output", default=None,
        help="Output filename  (default: apf_simulation.mp4, or .png with --frame)",
    )
    ap.add_argument(
        "--frame", type=int, default=None, metavar="N",
        help="Render only frame N as a PNG snapshot instead of the full animation",
    )
    ap.add_argument(
        "--no-render", action="store_true", default=False,
        help="Run the simulation and write CSV results, but skip image/video rendering",
    )
    ap.add_argument(
        "--settle-time", type=float, default=SETTLE_TIME,
        help=f"Seconds to hold UI-LB at final position after trajectory  (default: {SETTLE_TIME})",
    )
    ap.add_argument(
        "--use-ui-vel", action="store_true", default=False,
        help=(
            "Drive I-LBs with the UI-LB's velocity instead of APF. "
            "After UI-LB stops, the last I-LB velocity command is held "
            "until all I-LBs clear d_detect."
        ),
    )
    ap.add_argument(
        "--no-stop-after-clear", action="store_false", dest="stop_after_clear",
        default=True,
        help=(
            "By default, I-LBs stop once all have cleared d_detect. "
            "Pass this flag to keep applying the last command "
            "until the safety cap is reached."
        ),
    )
    ap.add_argument(
        "--use-pos-cmd", action="store_true", default=True,
        help=(
            "Issue position setpoints to the controller instead of velocity "
            "setpoints.  The position target each step is derived from the "
            "APF/UI-vel command via the outer P gain (--pos-kp)."
        ),
    )
    ap.add_argument(
        "--pos-kp", type=float, default=2.0,
        help="Outer position P gain used with --use-pos-cmd  (default: 2.0)",
    )
    args = ap.parse_args()

    # Apply programmatic overrides
    for key, val in overrides.items():
        setattr(args, key, val)

    logs_dir = os.path.join(
        os.path.dirname(__file__), "..", "..", "logs", "collisions"
    )
    logs_dir = os.path.realpath(logs_dir)
    summary_csv_path = os.path.join(logs_dir, "simulation_summary.csv")

    # Default output extension depends on mode
    if args.output is None:
        filename = "apf_simulation.png" if args.frame is not None else "apf_simulation.mp4"
        args.output = os.path.join(logs_dir, filename)

    mission_path = os.path.realpath(args.mission)
    print(f"\n{'='*70}")
    print(f"Loading mission: {mission_path}")
    drones, servos, pointers = parse_mission(mission_path)
    print(f"Drones: {sorted(drones.keys())}")
    for lb_id, pos in sorted(drones.items()):
        srv = servos.get(lb_id, "—")
        ptr = pointers.get(lb_id, "—")
        print(f"  {lb_id}: pos = {pos}  servos = {srv}  pointers = {ptr}")

    if args.ui_lb is None:
        args.ui_lb = sorted(drones.keys())[0]
        print(f"No --ui-lb given; defaulting to '{args.ui_lb}'")

    if args.ui_lb not in drones:
        raise SystemExit(
            f"Error: --ui-lb '{args.ui_lb}' not found in mission. "
            f"Available: {sorted(drones.keys())}"
        )
    print(f"UI-LB: {args.ui_lb}")

    # ── UI-LB trajectory ──────────────────────────────────────────────────────
    assigned_start = drones[args.ui_lb].copy()
    start = assigned_start.copy()
    start[1] -= 0.3
    print(
        f"UI-LB simulated start offset: assigned={assigned_start}  "
        f"simulated_start={start}"
    )
    if args.velocities is not None:
        if isinstance(args.velocities, list):
            vel_list = args.velocities
        else:
            vel_list = []
            for token in args.velocities.split():
                coords = [float(v) for v in token.split(",")]
                if len(coords) != 3:
                    raise SystemExit(f"Each velocity must be 'vx,vy,vz', got: {token!r}")
                vel_list.append(coords)
        vel_targets = resample_velocity_profile(vel_list, args.ui_vel_hz, args.dt)
        if args.settle_time > 0:
            n_settle = int(round(args.settle_time / args.dt))
            if n_settle > 0:
                vel_targets = np.vstack([vel_targets, np.zeros((n_settle, 3))])
        ui_traj, ui_actual_vels = integrate_velocity_profile(
            start, vel_targets, args.dt, args.ui_v_max, args.ui_a_max
        )
        print(
            f"Built trajectory from {len(vel_list)} velocity samples"
            f" at {args.ui_vel_hz:.1f} Hz"
        )
    else:
        ui_waypoints = build_default_waypoints(start)
        ui_ref_traj = interpolate_waypoints(ui_waypoints, UI_SPEED, args.dt)
        vel_targets = trajectory_to_velocity_targets(ui_ref_traj, args.dt)
        if args.settle_time > 0:
            n_settle = int(round(args.settle_time / args.dt))
            if n_settle > 0:
                vel_targets = np.vstack([vel_targets, np.zeros((n_settle, 3))])
        ui_traj, ui_actual_vels = integrate_velocity_profile(
            start, vel_targets, args.dt, args.ui_v_max, args.ui_a_max
        )
        print("No --waypoints/--velocities given; using default sweep.")

    if args.settle_time > 0:
        print(f"Added {args.settle_time:.1f} s settle command")

    print(f"UI-LB trajectory: {len(ui_traj)} positions,"
          f" start=[{ui_traj[0][0]:.3f}, {ui_traj[0][1]:.3f}, {ui_traj[0][2]:.3f}]"
          f" end=[{ui_traj[-1][0]:.3f}, {ui_traj[-1][1]:.3f}, {ui_traj[-1][2]:.3f}]")
    if len(ui_actual_vels) > 0:
        print(
            f"UI-LB realised max speed: {np.linalg.norm(ui_actual_vels, axis=1).max():.3f} m/s"
            f"  |  limits: v_max={args.ui_v_max:.3f}, a_max={args.ui_a_max:.3f}"
        )

    # ── simulate ───────────────────────────────────────────────────────────────
    print("\nRunning simulation …")
    history, n_steps, cumulative_collisions = run_simulation(
        drones, args.ui_lb, ui_traj,
        eta=args.eta, zeta=args.zeta,
        d_detect=args.d_detect, v_max=args.v_max, dt=args.dt,
        use_ui_vel=args.use_ui_vel,
        stop_after_clear=args.stop_after_clear,
        use_pos_cmd=args.use_pos_cmd,
        pos_kp=args.pos_kp,
    )
    duration = n_steps * args.dt
    total_col = cumulative_collisions[-1] if cumulative_collisions else 0
    sep_metrics = compute_separation_metrics(history, args.ui_lb)
    print(
        f"Simulation: {n_steps} steps at {1.0 / args.dt:.1f} Hz"
        f"  ({duration:.1f} s of flight time)"
    )
    print(f"Total collisions (<250 mm): {total_col}")
    print(
        f"Minimum separation: any pair = {sep_metrics['min_pair_dist']:.3f} m"
        f"  |  UI-to-I-LB = {sep_metrics['min_ui_dist']:.3f} m"
    )

    # ── render / logging ───────────────────────────────────────────────────────
    output_path = args.output
    if not os.path.isabs(output_path):
        output_path = os.path.join(logs_dir, output_path)
    output_path = os.path.realpath(output_path)
    rendered = not args.no_render
    if rendered:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        if args.frame is not None:
            print(
                f"\nRendering frame {args.frame} at {args.render_fps:.1f} fps"
                f" → {output_path}  (height={args.view_height} m)"
            )
        else:
            print(
                f"\nRendering animation at {args.render_fps:.1f} fps"
                f" → {output_path}  (height={args.view_height} m)"
            )
        make_animation(
            history, args.ui_lb, drones,
            args.d_detect, output_path, args.dt, args.render_fps,
            servos=servos,
            pointers=pointers,
            cumulative_collisions=cumulative_collisions,
            view_height=args.view_height,
            snapshot_frame=args.frame,
            render_steps=len(ui_traj),
        )
    else:
        print("\nSkipping render (--no-render); simulation results only.")

    append_run_summary(
        summary_csv_path,
        {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "mission": mission_path,
            "ui_lb": args.ui_lb,
            "mode": "UI_vel" if args.use_ui_vel else "APF",
            "rendered": rendered,
            "output": output_path if rendered else "",
            "dt": args.dt,
            "render_fps": args.render_fps,
            "ui_vel_hz": args.ui_vel_hz,
            "ui_v_max": args.ui_v_max,
            "ui_a_max": args.ui_a_max,
            "d_detect": args.d_detect,
            "eta": args.eta,
            "zeta": args.zeta,
            "v_max": args.v_max,
            "settle_time": args.settle_time,
            "steps": n_steps,
            "duration_s": duration,
            "total_collisions": total_col,
            "min_pair_dist": sep_metrics["min_pair_dist"],
            "min_ui_dist": sep_metrics["min_ui_dist"],
        },
    )
    print(f"Run summary appended to {summary_csv_path}")


# ── Batch runner ──────────────────────────────────────────────────────────────

_MISSION_DIR = os.path.join(os.path.dirname(__file__), "..", "mission")

# Each entry: (mission_yaml, ui_lb_id, use_ui_vel)
BATCH_RUNS = [
    ("ACM.yaml", "lb1", True, 0.47),
    ("ACM.yaml", "lb1", False, 0.47),
    ("S.yaml", "lb6", False, 0.47),
    ("S.yaml", "lb6", True, 0.47),
    ("ACM.yaml", "lb1", False, 0.60),
    ("ACM.yaml", "lb1", True, 0.60),
    ("ACM.yaml", "lb1", False, 0.4),
    ("ACM.yaml", "lb1", True, 0.4),
    ("S.yaml", "lb6", False, 0.47),
    ("S.yaml", "lb6", True, 0.47),
    ("S.yaml", "lb6", False, 0.60),
    ("S.yaml", "lb6", True, 0.60),
    ("S.yaml", "lb6", False, 0.4),
    ("S.yaml", "lb6", True, 0.4),
]

BATCH_ZETA = [0.0, 0.1]

# ── Grid search ───────────────────────────────────────────────────────────────

# Missions and modes to evaluate in the grid search.
# Each entry: (mission_yaml, ui_lb_id, use_ui_vel)
GRID_MISSIONS = [
    ("S.yaml",   "lb6", False),
    ("ACM.yaml", "lb1", False),
]

# ── Grid search parameters ────────────────────────────────────────────────────
GRID_D_DETECT       = 0.47   # fixed detection radius
BALANCE_D           = 0.3   # distance at which |v_rep| should equal |v_att|
BALANCE_GOAL_DIST   = 0.5   # reference I-LB-to-goal distance for the formula (m)

# eta sweep: 0.10 → 1.00, step 0.05 (19 values)
GRID_ETA = np.linspace(0, 1, 200)
# For each eta, derive zeta so |v_rep| = |v_att| at d = BALANCE_D:
#   ζ = η · (1/d − 1/d_detect) / (d² · r_goal)
#     = η · (d_detect − d) / (d³ · d_detect · r_goal)
def _zeta_from_balance(eta: float) -> float:
    num = eta * (GRID_D_DETECT - BALANCE_D)
    den = (BALANCE_D ** 3) * GRID_D_DETECT * BALANCE_GOAL_DIST
    return num / den

# GRID_ZETA = [_zeta_from_balance(eta) for eta in GRID_ETA]
GRID_ZETA = [0 for eta in GRID_ETA]


def run_grid_search():
    """
    1-D sweep over eta (with zeta derived from the APF balance condition at
    BALANCE_D) for every (mission, mode) combination.

    For each eta value, zeta is set so that |v_rep| = |v_att| at
    d = BALANCE_D given a reference goal distance of BALANCE_GOAL_DIST.
    Rendering is disabled for speed.

    Results are written to grid_search_results.csv, then the summary reports
    the minimum eta (and its paired zeta) that achieves zero collisions for
    each mission/mode and overall.
    """
    logs_dir = os.path.join(
        os.path.dirname(__file__), "..", "..", "logs", "collisions"
    )
    logs_dir = os.path.realpath(logs_dir)
    grid_csv_path = os.path.join(logs_dir, "grid_search_results.csv")

    eta_zeta_pairs = list(zip(GRID_ETA, GRID_ZETA))
    total_runs = len(GRID_MISSIONS) * len(eta_zeta_pairs)

    print(f"\n{'='*70}")
    print(
        f"Grid search: {len(GRID_MISSIONS)} mission/mode combos × "
        f"{len(eta_zeta_pairs)} eta/zeta pairs = {total_runs} runs"
    )
    print(
        f"d_detect={GRID_D_DETECT}  balance_d={BALANCE_D}  "
        f"goal_ref={BALANCE_GOAL_DIST} m  |  rendering disabled"
    )
    print(f"eta range: {GRID_ETA[0]:.2f} → {GRID_ETA[-1]:.2f}  step 0.05")
    print(f"{'eta':>7}  {'zeta':>7}")
    for eta, zeta in eta_zeta_pairs:
        print(f"  {eta:>5.2f}  {zeta:>7.3f}")
    print(f"Results → {grid_csv_path}")
    print(f"{'='*70}")

    results = []
    run_idx = 0

    for mission_file, ui_lb, use_ui_vel in GRID_MISSIONS:
        mission_path = os.path.join(_MISSION_DIR, mission_file)
        mode_tag = "UI_vel" if use_ui_vel else "APF"

        for eta, zeta in eta_zeta_pairs:
            run_idx += 1
            print(
                f"[{run_idx:>4}/{total_runs}] "
                f"{mission_file}/{ui_lb}/{mode_tag}  "
                f"eta={eta:.2f}  zeta={zeta:.3f}",
                end="  … ",
                flush=True,
            )
            try:
                main(
                    mission=mission_path,
                    ui_lb=ui_lb,
                    use_ui_vel=use_ui_vel,
                    d_detect=GRID_D_DETECT,
                    eta=eta,
                    zeta=zeta,
                    no_render=True,
                    output="__grid_search_dummy__.mp4",
                )
                row = _read_last_csv_row(
                    os.path.join(logs_dir, "simulation_summary.csv")
                )
                collisions    = int(float(row.get("total_collisions", 0)))
                min_ui_dist   = float(row.get("min_ui_dist", 0.0))
                min_pair_dist = float(row.get("min_pair_dist", 0.0))
                print(
                    f"collisions={collisions}  "
                    f"min_ui={min_ui_dist:.3f}  "
                    f"min_pair={min_pair_dist:.3f}"
                )
            except Exception as exc:
                print(f"ERROR: {exc}")
                collisions, min_ui_dist, min_pair_dist = 9999, 0.0, 0.0

            results.append({
                "mission": mission_file,
                "ui_lb": ui_lb,
                "mode": mode_tag,
                "eta": eta,
                "zeta": zeta,
                "d_detect": GRID_D_DETECT,
                "total_collisions": collisions,
                "min_ui_dist": min_ui_dist,
                "min_pair_dist": min_pair_dist,
            })

    # ── Write grid CSV ────────────────────────────────────────────────────
    grid_fields = [
        "mission", "ui_lb", "mode", "eta", "zeta", "d_detect",
        "total_collisions", "min_ui_dist", "min_pair_dist",
    ]
    os.makedirs(logs_dir, exist_ok=True)
    with open(grid_csv_path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=grid_fields)
        writer.writeheader()
        writer.writerows(results)

    # ── Per-combo minimum eta with zero collisions ────────────────────────
    print(f"\n{'='*70}")
    print(
        f"Grid search complete — {total_runs} runs  |  "
        f"d_detect={GRID_D_DETECT}  balance_d={BALANCE_D}"
    )
    print(f"{'='*70}")
    print("\nMinimum eta (zero collisions) per mission/mode:")
    print(f"  {'Mission':<10} {'Mode':<8} {'min_eta':>8} {'zeta':>8} "
          f"{'min_ui':>8} {'min_pair':>10}")
    print("  " + "-" * 56)

    combo_keys = [(mf, ml, ("UI_vel" if uv else "APF"))
                  for mf, ml, uv in GRID_MISSIONS]
    min_eta_overall = None

    for mission_file, ui_lb, mode_tag in combo_keys:
        combo_rows = [
            r for r in results
            if r["mission"] == mission_file
            and r["ui_lb"] == ui_lb
            and r["mode"] == mode_tag
        ]
        # Rows are already in ascending eta order; find first with 0 collisions
        clean = [r for r in combo_rows if r["total_collisions"] == 0]
        if clean:
            best = min(clean, key=lambda r: r["eta"])
            print(
                f"  {mission_file:<10} {mode_tag:<8} "
                f"{best['eta']:>8.2f} {best['zeta']:>8.3f} "
                f"{best['min_ui_dist']:>8.3f} {best['min_pair_dist']:>10.3f}"
            )
            if min_eta_overall is None or best["eta"] < min_eta_overall["eta"]:
                min_eta_overall = best
        else:
            print(f"  {mission_file:<10} {mode_tag:<8}  — no collision-free result found")

    if min_eta_overall:
        print(
            f"\nOverall minimum eta with zero collisions: "
            f"eta={min_eta_overall['eta']:.2f}  "
            f"zeta={min_eta_overall['zeta']:.3f}  "
            f"({min_eta_overall['mission']} / {min_eta_overall['mode']})"
        )
    else:
        print("\nNo collision-free parameter combination found in this sweep.")

    print(f"\nFull results saved to {grid_csv_path}")


def _read_last_csv_row(csv_path: str) -> dict:
    """Return the last data row of a CSV file as a dict."""
    with open(csv_path, newline="") as fh:
        reader = csv.DictReader(fh)
        row = {}
        for row in reader:
            pass
    return row


if __name__ == "__main__":
    # run_grid_search()

    if len(sys.argv) > 1:
        main()
    else:
        # Batch mode — iterate over BATCH_RUNS × BATCH_ZETA
        total = len(BATCH_RUNS) * len(BATCH_ZETA)
        idx = 0
        for mission_file, ui_lb, use_ui_vel, d_detect in BATCH_RUNS:
            mission_path = os.path.join(_MISSION_DIR, mission_file)
            mode_tag = "UI_vel" if use_ui_vel else "APF"
            for zeta in BATCH_ZETA:
                idx += 1
                out_name = (
                    f"{os.path.splitext(mission_file)[0]}_{ui_lb}_{mode_tag}"
                    f"_ddetect_{d_detect:.2f}_zeta_{zeta:.2f}.mp4"
                )
                print(
                    f"\n[Batch {idx}/{total}] "
                    f"{mission_file} / {ui_lb} / {mode_tag} "
                    f"/ d_detect={d_detect:.2f} / zeta={zeta:.2f}"
                )
                main(
                    mission=mission_path,
                    ui_lb=ui_lb,
                    use_ui_vel=use_ui_vel,
                    d_detect=d_detect,
                    zeta=zeta,
                    output=out_name,
                )
        print(f"\n{'='*70}")
        print(f"Batch complete: {total} run(s)")
