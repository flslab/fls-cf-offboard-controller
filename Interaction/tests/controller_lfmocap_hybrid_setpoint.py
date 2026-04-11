"""
controller_lfmocap_hybrid_setpoint.py

Hybrid leader-follower controller — identical logic to
controller_lfmocap_hybrid.py but uses send_position_setpoint loops instead of
go_to for every step move.  This avoids the high-level commander's internal
velocity profile and lets the low-level PID respond immediately.

ΔV mode  (velocity)  — existing behaviour
  The follower's Kalman Filter estimates the leader's Y-velocity.
  When |vel_y| > ΔV, the follower executes a fixed-distance step
  (``--follower-step`` mm) and logs T2 / T11.

ΔD mode  (position)  — existing behaviour
  A background-free drift loop continuously compares the leader's live Y
  position against a reference updated after every correction.
  When the accumulated displacement exceeds ΔD (``--delta-d`` mm), the
  follower mirrors that exact displacement with a position setpoint.

Fast-acceleration kick (optional)
  Before each step's setpoint loop, ``send_zdistance_setpoint`` is issued
  for ``--fast-accel-duration`` ms at the configured roll/pitch/yawrate to
  build initial momentum.  No send_notify_setpoint_stop is sent between the
  kick and the position setpoint loop (both are low-level).

Timing variables logged (ΔV steps only)
────────────────────────────────────────
  T1   — leader issues setpoint loop for step i
  T10  — leader first reaches target_y ± arrive_tol
  T2   — follower KF velocity exceeds ΔV
  T11  — follower first reaches its step target ± arrive_tol
"""

import argparse
import datetime
import json
import logging
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

import Interaction.config as cfg
from Interaction.Kalman_Filter import VelocityKalmanFilter
from Interaction.log_manager import InteractionLogger
from mocap import Mocap

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)

# ── Defaults ──────────────────────────────────────────────────────────────────
DEFAULT_URI      = "usb://0"
DEFAULT_HEIGHT   = 1.0

POSITION_STD_DEV    = 0.001
ORIENTATION_STD_DEV = 0.001

PID_VALUES = {
    'posCtlPid.xKp': '1.9',  'posCtlPid.xKi': '0.1',  'posCtlPid.xKd': '0.0',
    'posCtlPid.yKp': '2.1',  'posCtlPid.yKi': '0.1',  'posCtlPid.yKd': '0.0',
    'posCtlPid.zKp': '1.9',  'posCtlPid.zKi': '2.0',  'posCtlPid.zKd': '0.05',
    'posCtlPid.thrustMin': '12000',
    'posCtlPid.thrustBase': '28000',
    'velCtlPid.vxKp': '30.0', 'velCtlPid.vxKi': '4.0', 'velCtlPid.vxKd': '0.005',
    'velCtlPid.vyKp': '30.0', 'velCtlPid.vyKi': '4.0', 'velCtlPid.vyKd': '0.005',
    'velCtlPid.vzKp': '30.0', 'velCtlPid.vzKi': '5.0', 'velCtlPid.vzKd': '0.05',
    'pid_attitude.roll_kp': '6.0',  'pid_attitude.roll_ki': '1.0',  'pid_attitude.roll_kd': '0.005',
    'pid_attitude.pitch_kp': '7.1', 'pid_attitude.pitch_ki': '1.0', 'pid_attitude.pitch_kd': '0.005',
    'pid_rate.roll_kp': '90',  'pid_rate.roll_ki': '270.0', 'pid_rate.roll_kd': '2.5',
    'pid_rate.pitch_kp': '75', 'pid_rate.pitch_ki': '270.0', 'pid_rate.pitch_kd': '2.5',
    'pid_rate.rateFiltEn': '1',
    'pid_rate.omxFiltCut': '160',
    'pid_rate.omyFiltCut': '160',
    'pid_rate.omzFiltCut': '160',
}

# Keys whose values are proportional gains (Kp) — used for goto-boost feature
_KP_KEYS = [k for k in PID_VALUES if k.endswith('Kp') or k.endswith('kp')]


# ── Controller ────────────────────────────────────────────────────────────────
class LFMoCapHybridSetpointController:
    """
    Hybrid LF controller using send_position_setpoint loops for all step moves.

    Leader side: moves in fixed Y steps via setpoint loop.
    Follower side: two concurrent detection modes (ΔV step + ΔD drift),
                   both executed with position setpoints.
    """

    def __init__(self, args):
        self.args = args

        if self.args.takeoff_altitude is None:
            self.args.takeoff_altitude = DEFAULT_HEIGHT

        self.uri = uri_helper.uri_from_env(default=self.args.radio or DEFAULT_URI)

        self.scf         = None
        self.cf          = None
        self.commander   = None
        self.mocap       = None
        self.log_manager = None
        self.flying      = False
        self.init_coord  = None

        # Own Vicon frame
        self.latest_frame = None

        # Leader Vicon position + KF-estimated velocities (follower only)
        self.latest_leader_frame = None
        self.latest_leader_vel_x = 0.0
        self.latest_leader_vel_y = 0.0
        self.latest_leader_vel_z = 0.0
        self._leader_kf_x        = None
        self._leader_kf_y        = None
        self._leader_kf_z        = None

        # Hybrid follower state
        self._follower_target_y  = 0.0    # current commanded follower Y
        self._leader_drift_ref_y = None   # leader Y at last drift reset

    # ── context manager ──────────────────────────────────────────────────
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ── connection ───────────────────────────────────────────────────────
    def connect(self):
        logger.info(f"Connecting to {self.uri} ...")
        cflib.crtp.init_drivers(enable_serial_driver=True)
        self.scf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache="./cache"))
        self.scf.open_link()
        self.cf = self.scf.cf
        self.commander = self.cf.high_level_commander
        logger.info("Connected")

    def disconnect(self):
        if self.scf:
            self.scf.close_link()

    # ── lifecycle ────────────────────────────────────────────────────────
    def start(self):
        if self.args.log:
            self.setup_logging()
        self.setup_motion_capture()
        self.setup_params()
        self.save_init_coord()
        self.arm()
        self.takeoff()
        self.run_mission()

    def stop(self):
        self.land()
        if self.mocap:
            self.mocap.stop()
        if self.log_manager:
            self.log_manager.stop()
        self.disconnect()

    # ── motion capture ───────────────────────────────────────────────────
    def setup_motion_capture(self):
        if not self.args.vicon:
            return

        on_pose = (self._send_position_orientation
                   if self.args.vicon_full_pose
                   else self._send_position)

        self.mocap = Mocap(mode=self.args.vicon_mode)

        if self.args.vicon_mode == "rigidbody":
            logger.info(f"Subscribing to RigidBody: {self.args.obj_name}")
            self.mocap.subscribe_object(self.args.obj_name, on_pose)
        else:
            logger.info(f"Subscribing own point near: {self.args.init_pos}")
            self.mocap.subscribe_point(self.args.init_pos, on_pose, name="self")

            if self.args.role == "follower":
                kf_kwargs = dict(
                    dt=1.0 / self.args.fps,
                    process_noise=1.0,
                    measurement_noise=0.001 ** 2,
                )
                self._leader_kf_x = VelocityKalmanFilter(**kf_kwargs)
                self._leader_kf_y = VelocityKalmanFilter(**kf_kwargs)
                self._leader_kf_z = VelocityKalmanFilter(**kf_kwargs)
                logger.info(f"Subscribing leader point near: {self.args.leader_pos}")
                self.mocap.subscribe_point(
                    self.args.leader_pos, self._on_leader_frame, name="leader"
                )

        self.mocap.start()
        logger.info("Mocap activated")

    # ── logging ──────────────────────────────────────────────────────────
    def setup_logging(self):
        logger.info("Setting up logging ...")
        self.log_manager = InteractionLogger(controller_args=self.args)
        self.log_manager.init_cf_logger(self.cf, cfg.LOG_VARS, self.args.cf_log_period)
        self.log_manager.add_log_group("frames", kf=True)
        self.log_manager.add_log_group("leader_frames")
        self.log_manager.add_log_group("cmd_positions")
        self.log_manager.add_log_group("timing")
        self.log_manager.add_log_group("events")
        self.log_manager.start()
        logger.info("Logging activated")

    def _log_event(self, name: str, data: dict = None):
        entry = {
            "event":     name,
            "timestamp": datetime.datetime.now().isoformat(timespec="milliseconds"),
            "time":      time.time(),
        }
        if data:
            entry.update(data)
        if self.log_manager:
            self.log_manager.add_log_entry("events", entry)
        logger.info(f"[{self.args.role.upper()}] {name}: {data}")

    # ── params / estimator ───────────────────────────────────────────────
    def setup_params(self):
        logger.info("Setting up parameters ...")
        self.cf.param.set_value("stabilizer.estimator", "2")
        if self.args.vicon:
            self.cf.param.set_value("locSrv.extPosStdDev", POSITION_STD_DEV)
            self.cf.param.set_value("locSrv.extQuatStdDev", ORIENTATION_STD_DEV)
        self.cf.param.set_value("stabilizer.controller", "1")
        self.cf.param.set_value("commander.enHighLevel", "1")
        self.cf.param.set_value("hlCommander.vland", "0.1")
        for param, value in PID_VALUES.items():
            self.cf.param.set_value(param, value)

        if self.args.max_vel is not None:
            self.cf.param.set_value('posCtlPid.xVelMax', str(self.args.max_vel))
            self.cf.param.set_value('posCtlPid.yVelMax', str(self.args.max_vel))
            logger.info(f"Max XY velocity set: {self.args.max_vel} m/s")

        if self.args.vicon:
            reset_estimator(self.cf)

    # ── arm / takeoff / land ─────────────────────────────────────────────
    def arm(self):
        logger.info("Arming ...")
        self.cf.platform.send_arming_request(True)
        time.sleep(1.0)

    def save_init_coord(self):
        if self.mocap:
            logger.info("Waiting for first mocap frame ...")
            while self.latest_frame is None:
                time.sleep(0.05)
            self.init_coord = list(self.latest_frame["tvec"])
            logger.info(f"Home position: {self.init_coord}")

    def takeoff(self):
        alt = self.args.takeoff_altitude
        logger.info(f"Taking off to {alt} m ...")
        duration = alt * 2
        self.flying = True
        self.commander.takeoff(alt, duration)
        time.sleep(duration + 1.0)

    def land(self):
        # Stop any active low-level setpoints before switching to high-level land
        self.cf.commander.send_notify_setpoint_stop()
        if not self.flying:
            return
        logger.info("Landing ...")
        z = self.args.takeoff_altitude

        if self.init_coord and self.latest_frame:
            x, y, _ = self.latest_frame["tvec"]
            xi, yi, _ = self.init_coord
            dist = ((xi - x) ** 2 + (yi - y) ** 2) ** 0.5
            dt = max(dist * 2, 1.0)
            self._log_event("return_to_home", {"x": xi, "y": yi, "z": z})
            self._send_setpoint_loop(xi, yi, z, 0, dt + 0.5)

        self.cf.commander.send_notify_setpoint_stop()
        dt = z * 6
        self.commander.land(0.12, dt)
        time.sleep(dt + 1)
        self.commander.stop()
        self.flying = False

    # ── clock sync ───────────────────────────────────────────────────────
    def _sync_clocks(self):
        if self.args.no_tcp_sync:
            logger.info("Clock sync disabled (--no-tcp-sync)")
            return

        port = self.args.tcp_port

        if self.args.role == "leader":
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
                srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                srv.bind(("", port))
                srv.listen(1)
                srv.settimeout(30.0)
                logger.info(f"[LEADER] Awaiting follower clock-sync on port {port} ...")
                try:
                    conn, addr = srv.accept()
                except socket.timeout:
                    logger.warning("[LEADER] Clock-sync timed out — skipping")
                    return
                with conn:
                    t_send = time.time()
                    conn.sendall(json.dumps({"t_send": t_send}).encode())
            self._log_event("clock_sync", {"t_send": t_send})

        else:
            ip = self.args.leader_ip
            logger.info(f"[FOLLOWER] Connecting to {ip}:{port} for clock-sync ...")
            deadline = time.time() + 30.0
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                while time.time() < deadline:
                    try:
                        s.connect((ip, port))
                        break
                    except (ConnectionRefusedError, OSError):
                        time.sleep(0.3)
                data = b""
                while True:
                    chunk = s.recv(4096)
                    if not chunk:
                        break
                    data += chunk
            t_recv  = time.time()
            payload = json.loads(data.decode())
            t_send  = payload["t_send"]
            offset  = t_recv - t_send
            self._log_event("clock_sync", {
                "t_send": t_send, "t_recv": t_recv,
                "offset_s": round(offset, 6),
            })
            logger.info(f"[FOLLOWER] Clock offset: {offset:+.6f} s")

    # ── mission dispatcher ───────────────────────────────────────────────
    def run_mission(self):
        self._sync_clocks()
        if self.args.role == "leader":
            time.sleep(3)
            self._run_leader()
        else:
            self._run_follower()

    # ── shared helpers ───────────────────────────────────────────────────
    def _send_setpoint_loop(self, x: float, y: float, z: float,
                            yaw: float, duration: float):
        """Send position setpoints at setpoint_hz for ``duration`` seconds."""
        dt    = 1.0 / self.args.setpoint_hz
        t_end = time.time() + duration
        while time.time() < t_end:
            self.cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(dt)

    def _fast_accel_before_setpoint(self, z: float):
        """
        Send send_zdistance_setpoint for --fast-accel-duration ms to give the
        drone an initial acceleration kick before a position-setpoint step.

        No send_notify_setpoint_stop is issued afterwards — we remain in
        low-level mode and the caller continues with send_position_setpoint.

        No-op when --fast-accel-duration is 0 (default).
        """
        duration_s = self.args.fast_accel_duration / 1000.0
        if duration_s <= 0:
            return

        roll    = self.args.fast_accel_roll
        pitch   = self.args.fast_accel_pitch
        yawrate = self.args.fast_accel_yawrate
        dt      = 1.0 / self.args.setpoint_hz
        t_end   = time.time() + duration_s

        logger.info(
            f"[{self.args.role}] Fast-accel: roll={roll} pitch={pitch} "
            f"yawrate={yawrate} zdist={z:.3f} for {self.args.fast_accel_duration} ms"
        )
        while time.time() < t_end:
            self.cf.commander.send_zdistance_setpoint(roll, pitch, yawrate, z)
            time.sleep(dt)

    def _set_kp_multiplier(self, multiplier: float):
        """Scale all Kp PID gains by *multiplier* relative to their base values."""
        for key in _KP_KEYS:
            self.cf.param.set_value(key, str(float(PID_VALUES[key]) * multiplier))

    def _send_setpoint_loop_with_arrival(self, x: float, y: float, z: float,
                                         yaw: float, target_y: float,
                                         duration: float) -> float:
        """
        Optionally apply a fast-acceleration kick, then send position setpoints
        at setpoint_hz for ``duration`` seconds while polling for arrival at
        target_y ± arrive_tol.

        Returns the Unix timestamp of first arrival (or loop-end on timeout).
        """
        tol  = self.args.arrive_tol
        dt   = 1.0 / self.args.setpoint_hz
        t_arrival = None

        if self.log_manager:
            self.log_manager.add_log_entry("cmd_positions", {
                "timestamp": datetime.datetime.now().isoformat(timespec="milliseconds"),
                "cmd":       "position_setpoint",
                "x": x, "y": y, "z": z, "yaw": yaw,
                "target_y":  target_y,
                "duration":  duration,
            })

        m = self.args.kp_multiplier
        if m > 1:
            self._set_kp_multiplier(m)
            logger.info(f"[{self.args.role}] Kp boosted ×{m} for setpoint loop")

        self._fast_accel_before_setpoint(z)

        t_end = time.time() + duration
        while time.time() < t_end:
            self.cf.commander.send_position_setpoint(x, y, z, yaw)
            if t_arrival is None and self.latest_frame is not None:
                if abs(self.latest_frame["tvec"][1] - target_y) <= tol:
                    t_arrival = time.time()
            time.sleep(dt)

        if m > 1:
            self._set_kp_multiplier(1.0)
            logger.info(f"[{self.args.role}] Kp restored to base values")

        if t_arrival is None:
            logger.warning(
                f"[{self.args.role}] Arrival at Y≈{target_y:.4f} not detected "
                f"within {duration:.1f} s (tol={tol} m) — using loop-end time"
            )
            t_arrival = time.time()

        return t_arrival

    def _hover_and_detect_leader(self, hover_x: float, hover_y: float,
                                  hover_z: float, dv: float,
                                  timeout: float) -> float:
        """
        Hold position with send_position_setpoint while polling KF leader
        Y-velocity, with inline ΔD drift correction.

        Drift corrections also use send_position_setpoint — no commander
        transition required.  Returns Unix timestamp when |vel_y| > dv,
        or time.time() on timeout.
        """
        dd      = self.args.delta_d / 1000.0
        poll_dt = 1.0 / self.args.setpoint_hz
        t_start = time.time()

        leader_frame = self.latest_leader_frame
        if leader_frame is not None:
            self._leader_drift_ref_y = leader_frame["tvec"][1]

        while time.time() - t_start < timeout:
            # ── ΔD drift check ─────────────────────────────────────────────
            leader_frame = self.latest_leader_frame
            if leader_frame is not None:
                leader_y     = leader_frame["tvec"][1]
                ref_y        = self._leader_drift_ref_y
                follower_y   = self._follower_target_y
                displacement = leader_y - ref_y

                if abs(displacement) >= dd:
                    new_target_fy = follower_y + displacement
                    self.cf.commander.send_position_setpoint(
                        hover_x, new_target_fy, hover_z, 0
                    )
                    hover_y = new_target_fy
                    self._follower_target_y  = new_target_fy
                    self._leader_drift_ref_y = leader_y
                    self._log_event("drift_correction", {
                        "displacement_mm": round(displacement * 1000, 2),
                        "new_target_fy":   round(new_target_fy, 4),
                        "leader_y":        round(leader_y, 4),
                    })
                    time.sleep(poll_dt)
                    continue

            # ── ΔV detection ───────────────────────────────────────────────
            vel_y = self.latest_leader_vel_y
            if vel_y > dv:
                return time.time()

            # ── hold position ──────────────────────────────────────────────
            self.cf.commander.send_position_setpoint(hover_x, hover_y, hover_z, 0)
            time.sleep(poll_dt)

        logger.warning(
            f"[FOLLOWER] ΔV timeout (ΔV={dv} m/s, timeout={timeout:.1f} s)"
        )
        return time.time()

    # ── leader mission ───────────────────────────────────────────────────
    def _run_leader(self):
        """
        Move along Y axis ``steps`` times by ``alpha`` mm each step using
        send_position_setpoint loops.
        """
        alpha_m = self.args.alpha / 1000.0
        n       = self.args.steps
        dur     = self.args.step_duration
        settle  = self.args.settle_time
        timeout = dur + self.args.arrive_timeout_extra

        sx = self.args.init_pos[0]
        sy = self.args.init_pos[1]
        sz = self.args.takeoff_altitude

        self._log_event("mission_start", {
            "role": "leader", "steps": n, "alpha_mm": self.args.alpha,
        })

        for i in range(n):
            ty = sy + (i + 1) * alpha_m

            t1    = time.time()
            ts_t1 = datetime.datetime.fromtimestamp(t1).isoformat(timespec="milliseconds")
            self._log_event("step_start", {
                "step": i, "alpha_mm": self.args.alpha,
                "target_y": round(ty, 4),
                "T1": t1, "T1_iso": ts_t1,
            })

            t10 = self._send_setpoint_loop_with_arrival(
                sx, ty, sz, 0, target_y=ty, duration=timeout
            )
            ts_t10     = datetime.datetime.fromtimestamp(t10).isoformat(timespec="milliseconds")
            ttt_leader = t10 - t1

            if self.log_manager:
                self.log_manager.add_log_entry("timing", {
                    "role": "leader", "step": i, "alpha_mm": self.args.alpha,
                    "T1": t1, "T1_iso": ts_t1,
                    "T10": t10, "T10_iso": ts_t10,
                    "TTT_leader_s": round(ttt_leader, 4),
                })
            self._log_event("step_arrived", {
                "step": i,
                "T10": t10, "T10_iso": ts_t10,
                "TTT_leader_ms": round(ttt_leader * 1000, 1),
            })

            # Settle at target while continuing to send position setpoints
            remaining = (t1 + timeout + settle) - time.time()
            if remaining > 0:
                self._send_setpoint_loop(sx, ty, sz, 0, remaining)

        self.cf.commander.send_notify_setpoint_stop()
        self._log_event("mission_complete", {"role": "leader"})

    # ── follower mission ─────────────────────────────────────────────────
    def _run_follower(self):
        """
        Hybrid follower mission using send_position_setpoint throughout.

        ΔV + ΔD are handled inside _hover_and_detect_leader.

        Per step:
          ① Hover and wait for ΔV, applying ΔD drift corrections inline.
          ② Execute fixed step to (follower_y + follower_step) via setpoint loop.
          ③ Log T2, T11, TTT_follower.
          ④ Reset drift reference to current leader Y.
          ⑤ Hold target position for remainder of step period.
        """
        n          = self.args.steps
        dur        = self.args.step_duration
        dv         = self.args.delta_v
        step_m     = self.args.follower_step / 1000.0
        timeout    = dur + self.args.arrive_timeout_extra
        detect_tmo = dur + 100.0

        fx = self.args.init_pos[0]
        fy = self.args.init_pos[1]
        fz = self.args.takeoff_altitude

        self._follower_target_y = fy

        self._log_event("mission_start", {
            "role":             "follower",
            "steps":            n,
            "follower_step_mm": self.args.follower_step,
            "delta_v":          dv,
            "delta_d_mm":       self.args.delta_d,
        })

        for i in range(n):
            # ① Hover + ΔD drift corrections until ΔV detected
            hover_y = self._follower_target_y
            t2      = self._hover_and_detect_leader(fx, hover_y, fz, dv, detect_tmo)
            ts_t2      = datetime.datetime.fromtimestamp(t2).isoformat(timespec="milliseconds")
            _vx        = self.latest_leader_vel_x
            _vy        = self.latest_leader_vel_y
            _vz        = self.latest_leader_vel_z
            _total_vel = (_vx ** 2 + _vy ** 2 + _vz ** 2) ** 0.5
            self._log_event("leader_detected", {
                "step": i, "mode": "delta_v",
                "T2": t2, "T2_iso": ts_t2,
                "kf_vel_x_at_detection":     round(_vx, 4),
                "kf_vel_y_at_detection":     round(_vy, 4),
                "kf_vel_z_at_detection":     round(_vz, 4),
                "kf_total_vel_at_detection": round(_total_vel, 4),
            })

            # ② Move by fixed step distance via setpoint loop
            follower_y_now = self._follower_target_y
            target_fy      = follower_y_now + step_m
            t11 = self._send_setpoint_loop_with_arrival(
                fx, target_fy, fz, 0, target_y=target_fy, duration=timeout
            )
            ts_t11       = datetime.datetime.fromtimestamp(t11).isoformat(timespec="milliseconds")
            ttt_follower = t11 - t2

            # ③ Log timing
            if self.log_manager:
                self.log_manager.add_log_entry("timing", {
                    "role":             "follower",
                    "step":             i,
                    "follower_step_mm": self.args.follower_step,
                    "delta_v":          dv,
                    "delta_d_mm":       self.args.delta_d,
                    "T2":               t2,
                    "T2_iso":           ts_t2,
                    "T11":              t11,
                    "T11_iso":          ts_t11,
                    "TTT_follower_s":   round(ttt_follower, 4),
                })
            self._log_event("step_arrived", {
                "step": i, "mode": "delta_v",
                "T11": t11, "T11_iso": ts_t11,
                "TTT_follower_ms": round(ttt_follower * 1000, 1),
            })

            # ④ Reset drift reference
            leader_frame_now = self.latest_leader_frame
            new_ref          = leader_frame_now["tvec"][1] if leader_frame_now else None
            self._follower_target_y = target_fy
            if new_ref is not None:
                self._leader_drift_ref_y = new_ref

            # ⑤ Hold target position for remainder of step period
            remaining = (t2 + timeout) - time.time()
            if remaining > 0:
                self._send_setpoint_loop(fx, target_fy, fz, 0, remaining)

        self.cf.commander.send_notify_setpoint_stop()
        self._log_event("mission_complete", {"role": "follower"})

    # ── mocap callbacks ──────────────────────────────────────────────────
    def _send_position(self, frame):
        self.cf.extpos.send_extpos(*frame["tvec"])
        self.latest_frame = frame
        if self.log_manager:
            self.log_manager.add_log_entry("frames", frame)

    def _send_position_orientation(self, frame):
        self.cf.extpos.send_extpose(*frame["tvec"], *frame["quat"])
        self.latest_frame = frame
        if self.log_manager:
            self.log_manager.add_log_entry("frames", frame)

    def _on_leader_frame(self, frame):
        self.latest_leader_frame = frame
        pos_x, pos_y, pos_z     = frame["tvec"]
        vel_x = self._leader_kf_x.update(pos_x)
        vel_y = self._leader_kf_y.update(pos_y)
        vel_z = self._leader_kf_z.update(pos_z)
        self.latest_leader_vel_x = vel_x
        self.latest_leader_vel_y = vel_y
        self.latest_leader_vel_z = vel_z
        if self.log_manager:
            self.log_manager.add_log_entry("leader_frames", {
                **frame,
                "kf_vel_x": vel_x,
                "kf_vel_y": vel_y,
                "kf_vel_z": vel_z,
            })


# ── CLI ───────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    _ts = f"{datetime.datetime.now():%Y-%m-%d_%H-%M-%S}"

    ap = argparse.ArgumentParser(
        description="LFMoCapHybridSetpoint — ΔV step + ΔD drift controller "
                    "using send_position_setpoint loops (no go_to)"
    )

    # Role
    ap.add_argument("--role", required=True, choices=["leader", "follower"])

    # Drone / connection
    ap.add_argument("--radio", type=str, default=None)
    ap.add_argument("--takeoff-altitude", type=float, default=1.0)
    ap.add_argument("--log-dir", type=str, default="./logs")
    ap.add_argument("--tag", default=None, type=str)
    ap.add_argument("--cf-log-period", type=int, default=50)
    ap.add_argument("--fps", type=int, default=100,
                    help="Mocap frame rate — KF dt = 1/fps")
    ap.add_argument("-v", "--verbose", action="store_true", default=False)
    ap.add_argument("-log", action="store_true", default=True)

    # Mocap
    ap.add_argument("--vicon", action="store_true")
    ap.add_argument("--vicon-full-pose", action="store_true")
    ap.add_argument("--vicon-mode", default="mixed",
                    choices=["rigidbody", "pointcloud", "mixed"])
    ap.add_argument("--obj-name", type=str, default=None)
    ap.add_argument("--init-pos", type=float, nargs=3, default=[0.0, 0.0, 0.0])

    # Step mission
    ap.add_argument("--steps", type=int, default=5)
    ap.add_argument("--alpha", type=float, default=200.0,
                    help="[leader] Step distance in mm")
    ap.add_argument("--step-duration", type=float, default=2.0,
                    help="Setpoint loop duration per step in seconds")
    ap.add_argument("--settle-time", type=float, default=1.0,
                    help="[leader] Additional hold time after arrival before next step (s)")
    ap.add_argument("--setpoint-hz", type=float, default=100.0,
                    help="Rate at which send_position_setpoint is called (Hz)")
    ap.add_argument("--arrive-tol", type=float, default=0.03)
    ap.add_argument("--arrive-timeout-extra", type=float, default=0.0)

    # ΔV (velocity step) — follower
    ap.add_argument("--follower-step", type=float, default=500.0,
                    help="[follower] Fixed step distance for ΔV mode in mm (default 200)")
    ap.add_argument("--delta-v", type=float, default=0.1,
                    help="[follower] KF Y-velocity threshold ΔV in m/s (default 0.1)")
    ap.add_argument("--leader-pos", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    help="[follower] Initial position hint for leader Vicon marker")

    # ΔD (position drift) — follower
    ap.add_argument("--delta-d", type=float, default=10.0,
                    help="[follower] Position dead-band ΔD in mm for drift tracking "
                         "(default 5 mm)")

    ap.add_argument("--max-vel", type=float, default=2,
                    help="max speed along x and y axes in m/s "
                         "(sets posCtlPid.xVelMax and yVelMax)")
    ap.add_argument("--kp-multiplier", type=float, default=1.0,
                    help="Multiply all Kp PID gains by this factor during the setpoint loop, "
                         "then restore. Values <= 1 disable the boost (default 1.0).")

    # Fast acceleration kick before setpoint loop
    ap.add_argument("--fast-accel-duration", type=float, default=0.0,
                    help="Duration (ms) to send send_zdistance_setpoint before each "
                         "step's setpoint loop to give an initial acceleration kick.  "
                         "0 = disabled (default).")
    ap.add_argument("--fast-accel-pitch", type=float, default=0.0,
                    help="Pitch angle (degrees) for the fast-accel zdistance setpoint "
                         "(positive = forward/+Y). Default 0.")
    ap.add_argument("--fast-accel-roll", type=float, default=-50.0,
                    help="Roll angle (degrees) for the fast-accel zdistance setpoint. "
                         "Default 0.")
    ap.add_argument("--fast-accel-yawrate", type=float, default=0.0,
                    help="Yaw rate (deg/s) for the fast-accel zdistance setpoint. "
                         "Default 0.")

    # Clock sync
    ap.add_argument("--tcp-port", type=int, default=9000)
    ap.add_argument("--leader-ip", type=str, default="localhost")
    ap.add_argument("--no-tcp-sync", action="store_true", default=False)

    args = ap.parse_args()

    if args.tag is None:
        if args.role == "leader":
            step_cm = int(args.alpha / 10)
            args.tag = f"LFMoCapHybridSP_leader_{step_cm}_{_ts}"
        else:
            fstep_cm = int(args.follower_step / 10)
            args.tag = (
                f"LFMoCapHybridSP_follower_{fstep_cm}"
                f"_dv{args.delta_v}_dd{int(args.delta_d)}mm_{_ts}"
            )

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    with LFMoCapHybridSetpointController(args) as c:
        c.start()

# ── Example launch commands ───────────────────────────────────────────────────
# Leader (20 cm steps):
#   python Interaction/tests/controller_lfmocap_hybrid_setpoint.py --role leader \
#       --vicon --vicon-mode pointcloud --init-pos -1 0 0 \
#       --takeoff-altitude 1.0 --steps 5 --alpha 200 \
#       --step-duration 5.0 --settle-time 2.0
#
# Follower (ΔV=0.1 m/s, ΔD=3 mm dead-band):
#   python Interaction/tests/controller_lfmocap_hybrid_setpoint.py --role follower \
#       --vicon --vicon-mode pointcloud --init-pos 0 0 0 \
#       --leader-pos -1 0 0 \
#       --takeoff-altitude 1.0 --steps 5 \
#       --step-duration 5.0 \
#       --delta-v 0.1 --delta-d 3 \
#       --leader-ip 192.168.1.10
#
# With fast-accel kick (80 ms, 5° pitch):
#   add --fast-accel-duration 80 --fast-accel-pitch 5
