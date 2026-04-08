"""
test_follow_velocity.py
A variant of controller_follow.py that sends velocity setpoints to the follower
instead of position setpoints.

Velocity is computed by extrapolating the leader's last two KF-estimated velocities
from log_manager.groups["block"], assuming the leader maintains its current acceleration:

    v_next = 2 * v_curr - v_prev   (constant-acceleration extrapolation)

The follower uses send_velocity_world_setpoint so the on-board position PID is bypassed
entirely and only the velocity PID loop runs.

Usage (follower):
    python Interaction/tests/test_follow_velocity.py \\
        --vicon --vicon-mode pointcloud --init-pos 0 0 0 \\
        --leader-id lb1 --leader-pos 1 0 0 --follow-offset 0.5 0 0 \\
        --takeoff-altitude 1.0 -t 30 -log

Usage (leader — just hovers):
    python Interaction/tests/test_follow_velocity.py \\
        --vicon --vicon-mode pointcloud --init-pos 1 0 0 \\
        --takeoff-altitude 1.0 -t 30
"""

import sys
from pathlib import Path

# Allow running directly from this subdirectory
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import logging

# Re-use everything from the parent controller
from controller_follow import FollowController

logger = logging.getLogger(__name__)


class VelocityFollowController(FollowController):
    """
    Same as FollowController but _follow_with_offset sends a world-frame
    velocity setpoint derived from the leader's recent KF-estimated velocities.
    """

    def _follow_with_offset(self, frame, offset):
        """
        Instead of a position setpoint, issue a velocity setpoint by
        extrapolating the leader's velocity under a constant-acceleration
        assumption.

        Steps
        -----
        1. Log the leader frame (adds KF-estimated 'vel' to the entry).
        2. Retrieve the last two 'block' log entries that contain velocity data.
        3. Extrapolate: v_next = 2 * v_curr - v_prev.
        4. Send the extrapolated velocity to the follower via
           send_velocity_world_setpoint.  Yaw rate is kept at 0.
        """
        dt = 1.0 / self.args.fps
        delay = 0.01

        recent_velocities = []
        if self.log_manager:
            self.log_manager.add_log_entry("block", frame)

            # Walk backward so we only inspect the newest entries we need.
            for entry in reversed(self.log_manager.groups.get("block", [])):
                vel = entry.get("vel") if entry else None
                if vel is None:
                    continue
                recent_velocities.append(vel)
                if len(recent_velocities) == 2:
                    break

        if len(recent_velocities) >= 2:
            v_curr = recent_velocities[0]   # [vx, vy, vz] most recent
            v_prev = recent_velocities[1]   # [vx, vy, vz] one step ago

            ax = (v_curr[0] - v_prev[0]) / dt
            ay = (v_curr[1] - v_prev[1]) / dt
            az = (v_curr[2] - v_prev[2]) / dt

            vx = v_curr[0] + ax * (dt + delay)
            vy = v_curr[1] + ay * (dt + delay)
            vz = v_curr[2] + az * (dt + delay)

            logger.debug(
                f"vel_prev={[f'{v:.3f}' for v in v_prev]}  "
                f"vel_curr={[f'{v:.3f}' for v in v_curr]}  "
                f"vel_cmd={[f'{v:.3f}' for v in (vx, vy, vz)]}"
            )

        elif len(recent_velocities) == 1:
            # Only one sample — use it directly, no extrapolation
            vx, vy, vz = recent_velocities[0]
        else:
            # No velocity data yet — hold in place
            vx, vy, vz = 0.0, 0.0, 0.0

        # send_velocity_world_setpoint bypasses the position PID and feeds the
        # velocity PID directly (low-level commander, world frame).
        self.cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0.0)


# ── CLI entry-point ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    import argparse
    import datetime

    tag = f"{datetime.datetime.now():%Y-%m-%d_%H-%M-%S}"

    ap = argparse.ArgumentParser(
        description="Leader-Follower controller — velocity setpoint variant"
    )

    # Drone / connection
    ap.add_argument("--radio", type=str, default=None)
    ap.add_argument("--takeoff-altitude", type=float, default=1.0)
    ap.add_argument("-t", type=float, default=None)
    ap.add_argument("--log-dir", type=str, default="./logs")
    ap.add_argument("--tag", default=tag, type=str)
    ap.add_argument("--cf-log-period", type=int, default=50)
    ap.add_argument("--fps", type=int, default=100)
    ap.add_argument("-v", "--verbose", action="store_true", default=False)
    ap.add_argument("-log", action="store_true", default=False)

    # Mocap
    ap.add_argument("--vicon", action="store_true")
    ap.add_argument("--vicon-full-pose", action="store_true")
    ap.add_argument("--vicon-mode", default="mixed",
                    choices=["rigidbody", "pointcloud", "mixed"])
    ap.add_argument("--obj-name", type=str, default=None)
    ap.add_argument("--init-pos", type=float, nargs=3, default=[0.0, 0.0, 0.0])

    # Leader-follower
    ap.add_argument("--leader-id", type=str, default=None)
    ap.add_argument("--leader-pos", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    ap.add_argument("--follow-offset", type=float, nargs=3, default=[0.0, 0.0, 0.0])

    args = ap.parse_args()

    with VelocityFollowController(args) as c:
        c.start()
