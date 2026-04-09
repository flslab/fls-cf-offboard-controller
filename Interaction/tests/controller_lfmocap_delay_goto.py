"""
controller_lfmocap_delay.py

LFMoCapDelay experiment — measures leader-follower detection and motion delay
using Vicon as the only communication channel between drones.

No user interaction.  No TCP between drones.
Both drones log independently; timing is compared offline.

Leader:
  Moves along the Y axis ``--steps`` times, each step by ``--alpha`` mm.
  Logs T1 (step issued) and T10 (own arrival) per step.

Follower:
  Monitors the leader's Vicon position via a Kalman Filter.
  When the estimated Y-velocity exceeds ΔV (``--delta-v``, default 0.1 m/s),
  it moves ``--follower-step`` mm (default 200 mm = 20 cm) along Y.
  Logs T2 (detection) and T11 (own arrival) per step.

Timing variables logged per step
─────────────────────────────────
  T1   — leader issues setpoint loop for step i
  T10  — leader's Y-position first reaches target_y ± arrive_tol
  T2   — follower KF-estimated leader Y-velocity exceeds ΔV
  T11  — follower's Y-position first reaches its step target ± arrive_tol

  Time To Detect   = T2  − T1   (computed offline, leader log vs follower log)
  Total Delay      = T11 − T10  (computed offline)
  TTT_leader       = T10 − T1   (logged by leader)
  TTT_follower     = T11 − T2   (logged by follower)

All important events are written to the log file via log_manager ("events" group).
The "timing" group stores the per-step timing variables.

Run leader (alpha = 20 cm):
    python Interaction/tests/controller_lfmocap_delay.py --role leader \\
        --vicon --vicon-mode pointcloud --init-pos -1 0 0 \\
        --takeoff-altitude 1.0 --steps 5 --alpha 200 \\
        --step-duration 5.0 --settle-time 2.0

Run follower:
    python Interaction/tests/controller_lfmocap_delay.py --role follower \\
        --vicon --vicon-mode pointcloud --init-pos 0 0 0 \\
        --leader-pos -1 0 0 \\
        --takeoff-altitude 1.0 --steps 5 \\
        --step-duration 5.0 --settle-time 2.0

Repeat with --alpha 500 for the 50 cm variant.
"""

import argparse
import datetime
import logging
import sys
import threading
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


# ── Controller ────────────────────────────────────────────────────────────────
class LFMoCapDelayController:
    """
    LFMoCapDelay experiment controller.

    Both roles (leader / follower) run this class.
    The leader moves autonomously; the follower reacts only to Vicon-detected
    motion, with no direct communication channel between the two drones.
    """

    def __init__(self, args):
        self.args = args

        if self.args.takeoff_altitude is None:
            self.args.takeoff_altitude = DEFAULT_HEIGHT

        self.uri = uri_helper.uri_from_env(default=self.args.radio or DEFAULT_URI)

        self.scf         = None
        self.cf          = None
        self.commander   = None   # high-level, used only for takeoff/land
        self.mocap       = None
        self.log_manager = None
        self.flying      = False
        self.init_coord  = None

        # Own Vicon frame (all roles)
        self.latest_frame = None

        # Leader Vicon position + KF-estimated Y velocity (follower only)
        self.latest_leader_frame = None
        self.latest_leader_vel_y = 0.0
        self._leader_kf_y        = None
        self._kf_lock            = threading.Lock()

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

            # Follower additionally subscribes to the leader's Vicon marker
            if self.args.role == "follower":
                self._leader_kf_y = VelocityKalmanFilter(
                    dt=1.0 / self.args.fps,
                    process_noise=1.0,
                    measurement_noise=0.001 ** 2,
                )
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
        self.log_manager.add_log_group("frames", kf=True)    # own position + KF vel
        self.log_manager.add_log_group("leader_frames")      # leader raw + KF vel_y (follower)
        self.log_manager.add_log_group("cmd_positions")      # setpoint commands issued
        self.log_manager.add_log_group("timing")             # per-step T1/T2/T10/T11 + delays
        self.log_manager.add_log_group("events")             # milestone events
        self.log_manager.start()
        logger.info("Logging activated")

    def _log_event(self, name: str, data: dict = None):
        """Write a named milestone event to the log file."""
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
            self.cf.high_level_commander.go_to(xi, yi, z, 0, dt + 0.5)

        dt = z * 6
        self.commander.land(0.12, dt)
        time.sleep(dt + 1)
        self.commander.stop()
        self.flying = False

    # ── mission dispatcher ───────────────────────────────────────────────
    def run_mission(self):
        if self.args.role == "leader":
            time.sleep(3)
            self._run_leader()
        else:
            self._run_follower()

    def _send_goto_check_arrival(self, x: float, y: float, z: float,
                                 yaw: float, target_y: float,
                                 duration: float) -> float:
        """
        Issue a go_to command (``--goto-duration`` ms) then poll Vicon for arrival.

        Polls own Vicon Y position until within ``arrive_tol`` of ``target_y``
        or ``duration`` seconds have elapsed.

        Returns the Unix timestamp of first arrival detection, or
        ``time.time()`` at timeout if arrival was never reached.
        """
        tol      = self.args.arrive_tol
        poll_dt  = 1.0 / self.args.setpoint_hz
        t_arrival = None

        if self.log_manager:
            self.log_manager.add_log_entry("cmd_positions", {
                "timestamp": datetime.datetime.now().isoformat(timespec="milliseconds"),
                "cmd":       "go_to",
                "x": x, "y": y, "z": z, "yaw": yaw,
                "target_y":  target_y,
                "duration":  duration,
            })

        self.commander.go_to(x, y, z, yaw, self.args.goto_duration / 1000.0)

        t_end = time.time() + duration
        while time.time() < t_end:
            if self.latest_frame is not None:
                if abs(self.latest_frame["tvec"][1] - target_y) <= tol:
                    t_arrival = time.time()
                    break
            time.sleep(poll_dt)

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
        Hold position with go_to while polling KF-estimated leader Y-velocity.

        Issues go_to(``--goto-duration`` ms) once to lock the hover position, then polls
        until ``|vel_y| > dv`` or ``timeout`` expires.

        Returns the Unix timestamp when ``|vel_y| > dv``, or
        ``time.time()`` on timeout.
        """
        poll_dt = 1.0 / self.args.setpoint_hz
        t_start = time.time()

        self.commander.go_to(hover_x, hover_y, hover_z, 0, self.args.goto_duration / 1000.0)

        while time.time() - t_start < timeout:
            with self._kf_lock:
                vel_y = self.latest_leader_vel_y
            if abs(vel_y) > dv:
                return time.time()
            time.sleep(poll_dt)

        logger.warning(
            f"[FOLLOWER] Timeout waiting for leader movement "
            f"(ΔV={dv} m/s, timeout={timeout:.1f} s)"
        )
        return time.time()

    # ── leader mission ───────────────────────────────────────────────────
    def _run_leader(self):
        """
        Move along Y axis ``steps`` times by ``alpha`` mm each step.

        Per step:
          ① Record T1, log step_start event.
          ② Run setpoint loop to target_y; detect own arrival → T10.
          ③ Log T1, T10, TTT_leader.
          ④ Settle at target_y before next step.
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

            # ① Record T1
            t1    = time.time()
            ts_t1 = datetime.datetime.fromtimestamp(t1).isoformat(timespec="milliseconds")
            self._log_event("step_start", {
                "step": i, "alpha_mm": self.args.alpha,
                "target_y": round(ty, 4),
                "T1": t1, "T1_iso": ts_t1,
            })

            # ② Setpoint loop with interleaved arrival detection → T10
            t10 = self._send_goto_check_arrival(
                sx, ty, sz, 0, target_y=ty, duration=timeout
            )
            ts_t10     = datetime.datetime.fromtimestamp(t10).isoformat(timespec="milliseconds")
            ttt_leader = t10 - t1

            # ③ Log timing
            if self.log_manager:
                self.log_manager.add_log_entry("timing", {
                    "role":         "leader",
                    "step":         i,
                    "alpha_mm":     self.args.alpha,
                    "T1":           t1,
                    "T1_iso":       ts_t1,
                    "T10":          t10,
                    "T10_iso":      ts_t10,
                    "TTT_leader_s": round(ttt_leader, 4),
                })
            self._log_event("step_arrived", {
                "step": i,
                "T10": t10, "T10_iso": ts_t10,
                "TTT_leader_ms": round(ttt_leader * 1000, 1),
            })

            # ④ Sleep until t1 + timeout so each step is fixed-length
            remaining = (t1 + timeout + settle) - time.time()
            if remaining > 0:
                time.sleep(remaining)

            # self.cf.high_level_commander.go_to(sx, ty, sz, 0, settle)

        # self.cf.commander.send_notify_setpoint_stop()
        self._log_event("mission_complete", {"role": "leader"})

    # ── follower mission ─────────────────────────────────────────────────
    def _run_follower(self):
        """
        Vicon-based reactive follower — no direct communication with leader.

        Per step:
          ① Hover and poll KF until leader Y-velocity > ΔV → T2.
          ② Compute target (current follower Y + follower_step).
          ③ Run setpoint loop to target; detect own arrival → T11.
          ④ Log T2, T11, TTT_follower.
          ⑤ Settle before next step.
        """
        n       = self.args.steps
        dur     = self.args.step_duration
        dv      = self.args.delta_v
        step_m  = self.args.follower_step / 1000.0
        timeout = dur + self.args.arrive_timeout_extra
        # Detection window: allow up to one full step + settle + extra
        detect_tmo = dur + settle + 5.0

        fx = self.args.init_pos[0]
        fy = self.args.init_pos[1]
        fz = self.args.takeoff_altitude

        self._log_event("mission_start", {
            "role":             "follower",
            "steps":            n,
            "follower_step_mm": self.args.follower_step,
            "delta_v":          dv,
        })

        for i in range(n):
            # ① Hover and wait for leader movement detection → T2
            t2    = self._hover_and_detect_leader(fx, fy, fz, dv, detect_tmo)
            ts_t2 = datetime.datetime.fromtimestamp(t2).isoformat(timespec="milliseconds")
            self._log_event("leader_detected", {
                "step": i,
                "T2": t2, "T2_iso": ts_t2,
                "kf_vel_y_at_detection": round(self.latest_leader_vel_y, 4),
            })

            # ② Compute follower target from current Vicon position
            if self.latest_frame is not None:
                fy = self.latest_frame["tvec"][1]
            target_fy = fy + step_m

            # ③ Setpoint loop to target with interleaved arrival detection → T11
            t11 = self._send_goto_check_arrival(
                fx, target_fy, fz, 0, target_y=target_fy, duration=timeout
            )
            ts_t11       = datetime.datetime.fromtimestamp(t11).isoformat(timespec="milliseconds")
            ttt_follower = t11 - t2

            # ④ Log timing
            if self.log_manager:
                self.log_manager.add_log_entry("timing", {
                    "role":             "follower",
                    "step":             i,
                    "follower_step_mm": self.args.follower_step,
                    "delta_v":          dv,
                    "T2":               t2,
                    "T2_iso":           ts_t2,
                    "T11":              t11,
                    "T11_iso":          ts_t11,
                    "TTT_follower_s":   round(ttt_follower, 4),
                })
            self._log_event("step_arrived", {
                "step": i,
                "T11": t11, "T11_iso": ts_t11,
                "TTT_follower_ms": round(ttt_follower * 1000, 1),
            })

            # ⑤ Sleep until t2 + timeout so each step is fixed-length, then advance
            remaining = (t2 + timeout) - time.time()
            if remaining > 0:
                time.sleep(remaining)
            fy = target_fy
            # self.cf.high_level_commander.go_to(fx, fy, fz, 0, settle)

        self.cf.commander.send_notify_setpoint_stop()
        self._log_event("mission_complete", {"role": "follower"})

    # ── mocap callbacks ──────────────────────────────────────────────────
    def _send_position(self, frame):
        """Own position callback — forwards extpos to CF and stores frame."""
        self.cf.extpos.send_extpos(*frame["tvec"])
        self.latest_frame = frame
        if self.log_manager:
            self.log_manager.add_log_entry("frames", frame)

    def _send_position_orientation(self, frame):
        """Own pose callback — forwards extpose to CF and stores frame."""
        self.cf.extpos.send_extpose(*frame["tvec"], *frame["quat"])
        self.latest_frame = frame
        if self.log_manager:
            self.log_manager.add_log_entry("frames", frame)

    def _on_leader_frame(self, frame):
        """
        Leader position callback (follower only).

        Runs the Kalman Filter on the leader's Y position every Vicon frame
        and stores the estimated Y-velocity in ``self.latest_leader_vel_y``.
        """
        self.latest_leader_frame = frame
        pos_y = frame["tvec"][1]
        with self._kf_lock:
            vel_y = self._leader_kf_y.update(pos_y)
            self.latest_leader_vel_y = vel_y
        if self.log_manager:
            self.log_manager.add_log_entry("leader_frames", {
                **frame,
                "kf_vel_y": vel_y,
            })


# ── CLI ───────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    _ts = f"{datetime.datetime.now():%Y-%m-%d_%H-%M-%S}"

    ap = argparse.ArgumentParser(
        description="LFMoCapDelay — Vicon-based leader-follower delay measurement"
    )

    # Role
    ap.add_argument("--role", required=True, choices=["leader", "follower"],
                    help="This drone's role")

    # Drone / connection
    ap.add_argument("--radio", type=str, default=None,
                    help="CrazyRadio URI (e.g. 'radio://0/6/1M/E7E7E7E704')")
    ap.add_argument("--takeoff-altitude", type=float, default=1.0,
                    help="Takeoff altitude in metres")
    ap.add_argument("--log-dir", type=str, default="./logs",
                    help="Directory to save log files")
    ap.add_argument("--tag", default=None, type=str,
                    help="Log filename tag (default: LFMoCapDelay_<role>_<timestamp>)")
    ap.add_argument("--cf-log-period", type=int, default=50,
                    help="CF logger period in ms")
    ap.add_argument("--fps", type=int, default=100,
                    help="Mocap frame rate — Kalman filter dt = 1/fps")
    ap.add_argument("-v", "--verbose", action="store_true", default=False)
    ap.add_argument("-log", action="store_true", default=True,
                    help="Enable flight logging (default: True)")

    # Mocap
    ap.add_argument("--vicon", action="store_true", help="Enable Vicon mocap")
    ap.add_argument("--vicon-full-pose", action="store_true",
                    help="Send position + orientation (otherwise position only)")
    ap.add_argument("--vicon-mode", default="mixed",
                    choices=["rigidbody", "pointcloud", "mixed"],
                    help="Mocap tracking mode")
    ap.add_argument("--obj-name", type=str, default=None,
                    help="Rigid-body name (rigidbody mode)")
    ap.add_argument("--init-pos", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    help="Initial position hint [x y z] for this drone's Vicon point")

    # Step mission
    ap.add_argument("--steps", type=int, default=5,
                    help="Number of Y-axis steps")
    ap.add_argument("--alpha", type=float, default=200.0,
                    help="[leader] Step distance in mm. Use 200 for 20 cm, 500 for 50 cm")
    ap.add_argument("--step-duration", type=float, default=5.0,
                    help="Setpoint loop duration per step in seconds")
    ap.add_argument("--settle-time", type=float, default=1.0,
                    help="Hover time between steps in seconds")
    ap.add_argument("--setpoint-hz", type=float, default=100.0,
                    help="Rate at which send_position_setpoint is called (Hz)")
    ap.add_argument("--arrive-tol", type=float, default=0.03,
                    help="Arrival detection tolerance in metres (default 3 cm)")
    ap.add_argument("--arrive-timeout-extra", type=float, default=5.0,
                    help="Seconds added to step-duration for the arrival timeout")
    ap.add_argument("--goto-duration", type=float, default=10.0,
                    help="Duration passed to go_to() in ms (default 10 ms = 0.01 s)")

    # Follower-specific
    ap.add_argument("--follower-step", type=float, default=200.0,
                    help="[follower] Distance to move upon detection in mm (default 200 mm = 20 cm)")
    ap.add_argument("--delta-v", type=float, default=0.1,
                    help="[follower] KF Y-velocity threshold ΔV in m/s to detect leader movement")
    ap.add_argument("--leader-pos", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    help="[follower] Initial position hint for the leader's Vicon marker [x y z]")

    args = ap.parse_args()

    # Build default tag if not overridden
    if args.tag is None:
        if args.role == "leader":
            step_cm = int(args.alpha / 10)
            args.tag = f"LFMoCapDelay_leader_alpha{step_cm}cm_goto{int(args.goto_duration)}ms_{_ts}"
        else:
            fstep_cm = int(args.follower_step / 10)
            args.tag = f"LFMoCapDelay_follower_fstep{fstep_cm}cm_goto{int(args.goto_duration)}ms_{_ts}"

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    with LFMoCapDelayController(args) as c:
        c.start()

# ── Example launch commands ───────────────────────────────────────────────────
# Alpha = 20 cm
# Leader:
#   python Interaction/tests/controller_lfmocap_delay.py --role leader \
#       --vicon --vicon-mode pointcloud --init-pos -1 0 0 \
#       --takeoff-altitude 1.0 --steps 5 --alpha 200 \
#       --step-duration 5.0 --settle-time 2.0
#
# Follower:
#   python Interaction/tests/controller_lfmocap_delay.py --role follower \
#       --vicon --vicon-mode pointcloud --init-pos 0 0 0 \
#       --leader-pos -1 0 0 \
#       --takeoff-altitude 1.0 --steps 5 \
#       --step-duration 5.0 --settle-time 2.0
#
# Alpha = 50 cm: add --alpha 500 to the leader command
