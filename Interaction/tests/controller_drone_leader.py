"""
controller_drone_leader.py

Two-drone synchronized step mission over TCP.

The LEADER drone moves forward along Y by `--step-size` mm, `--steps` times.
Before each step, it sends the absolute target to the FOLLOWER over TCP so both
drones issue their go_to commands simultaneously.

Usage — leader (runs TCP server, waits for follower before flying):
    python controller_drone_leader.py --role leader \\
        --vicon --vicon-mode pointcloud --init-pos 1 0 0 \\
        --takeoff-altitude 1.0 \\
        --steps 5 --step-size 200 --step-duration 3.0 \\
        --tcp-port 5005 --msg-order before

    --msg-order before (default): leader notifies follower then both move simultaneously.
    --msg-order after:            leader moves first, then notifies follower to move.

Usage — follower (connects to leader's TCP server):
    python controller_drone_leader.py --role follower \\
        --vicon --vicon-mode pointcloud --init-pos 0 0 0 \\
        --takeoff-altitude 1.0 \\
        --follow-offset 0.5 0 0 \\
        --tcp-host 192.168.1.XX --tcp-port 5005
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
from Interaction.log_manager import InteractionLogger
from mocap import Mocap

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
logger = logging.getLogger(__name__)

# ── Defaults ─────────────────────────────────────────────────────────────────
DEFAULT_URI = "usb://0"
DEFAULT_HEIGHT = 1.0
DEFAULT_TCP_PORT = 5005

POSITION_STD_DEV = 0.001
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


# ── TCP Communication ─────────────────────────────────────────────────────────
class TCPComm:
    """Handles synchronised TCP messaging between leader and follower.

    Both sides call setup() before the mission starts.  After that, use
    send() / recv() which exchange newline-delimited JSON objects.
    """

    _BUFSIZE = 4096

    def __init__(self, role: str, host: str, port: int):
        self.role = role
        self.host = host
        self.port = port
        self._server_sock = None   # leader only — kept open for clean close
        self._conn = None          # the active data socket on both sides
        self._recv_buf = b""

    def setup(self):
        if self.role == "leader":
            self._accept_follower()
        else:
            self._connect_to_leader()

    def _accept_follower(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(1)
        logger.info(f"[TCP] Listening on port {self.port}, waiting for follower ...")
        self._conn, addr = srv.accept()
        self._conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._server_sock = srv
        logger.info(f"[TCP] Follower connected from {addr}")

    def _connect_to_leader(self):
        logger.info(f"[TCP] Connecting to leader at {self.host}:{self.port} ...")
        while True:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((self.host, self.port))
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self._conn = s
                logger.info("[TCP] Connected to leader")
                return
            except (ConnectionRefusedError, OSError):
                logger.info("[TCP] Not ready, retrying in 1 s ...")
                time.sleep(1.0)

    def send(self, msg: dict):
        data = (json.dumps(msg) + "\n").encode()
        self._conn.sendall(data)

    def recv(self) -> dict:
        while b"\n" not in self._recv_buf:
            chunk = self._conn.recv(self._BUFSIZE)
            if not chunk:
                raise ConnectionError("[TCP] Connection closed by peer")
            self._recv_buf += chunk
        line, self._recv_buf = self._recv_buf.split(b"\n", 1)
        return json.loads(line.decode())

    def close(self):
        if self._conn:
            self._conn.close()
        if self._server_sock:
            self._server_sock.close()


# ── Controller ────────────────────────────────────────────────────────────────
class DroneleaderController:
    """
    Controller for the synchronized leader-follower step mission.

    Both drones run this class — their behaviour differs only by --role.
    The leader issues TCP commands; the follower reacts to them.
    """

    def __init__(self, args):
        self.args = args

        if self.args.takeoff_altitude is None:
            self.args.takeoff_altitude = DEFAULT_HEIGHT

        self.uri = uri_helper.uri_from_env(default=self.args.radio or DEFAULT_URI)

        self.scf = None
        self.cf = None
        self.commander = None
        self.mocap = None
        self.log_manager = None
        self.tcp = None

        self.flying = False
        self.init_coord = None   # XYZ at hover-start (absolute, metres)
        self.latest_frame = None

    # ── Context manager ──────────────────────────────────────────────────
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ── Connection ───────────────────────────────────────────────────────
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

    # ── Lifecycle ────────────────────────────────────────────────────────
    def start(self):
        if self.args.log:
            self.setup_logging()
        self.setup_motion_capture()
        self.setup_params()
        self.setup_tcp()        # establish TCP before takeoff
        self.save_init_coord()
        self.arm()
        self.takeoff()
        self.run_mission()

    def stop(self):
        self.land()
        if self.tcp:
            self.tcp.close()
        if self.mocap:
            self.mocap.stop()
        if self.log_manager:
            self.log_manager.stop()
        self.disconnect()

    # ── Motion capture ───────────────────────────────────────────────────
    def setup_motion_capture(self):
        if not self.args.vicon:
            return

        on_pose = self._send_position_orientation if self.args.vicon_full_pose else self._send_position

        self.mocap = Mocap(mode=self.args.vicon_mode)

        if self.args.vicon_mode == "rigidbody":
            logger.info(f"Subscribing to RigidBody: {self.args.obj_name}")
            self.mocap.subscribe_object(self.args.obj_name, on_pose)
        else:
            logger.info(f"Subscribing to pointcloud point near: {self.args.init_pos}")
            self.mocap.subscribe_point(self.args.init_pos, on_pose, name="self")

        self.mocap.start()
        logger.info("Mocap activated")

    # ── Logging ──────────────────────────────────────────────────────────
    def setup_logging(self):
        logger.info("Setting up logging ...")
        self.log_manager = InteractionLogger(controller_args=self.args)
        self.log_manager.init_cf_logger(self.cf, cfg.LOG_VARS, self.args.cf_log_period)
        self.log_manager.add_log_group("frames", kf=True)
        self.log_manager.add_log_group("cmd_positions")
        self.log_manager.start()
        logger.info("Logging activated")

    # ── TCP ──────────────────────────────────────────────────────────────
    def setup_tcp(self):
        role = self.args.role
        if role == "leader":
            host = "0.0.0.0"
        else:
            host = self.args.tcp_host

        self.tcp = TCPComm(role=role, host=host, port=self.args.tcp_port)
        self.tcp.setup()
        logger.info(f"[TCP] Setup complete as {role}")

    # ── Params / estimator ───────────────────────────────────────────────
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

    # ── Arm / Takeoff / Land ─────────────────────────────────────────────
    def arm(self):
        logger.info("Arming ...")
        self.cf.platform.send_arming_request(True)
        time.sleep(1.0)

    def save_init_coord(self):
        """Wait for at least one mocap frame and record it as the home position."""
        if self.mocap:
            logger.info("Waiting for first mocap frame ...")
            while self.latest_frame is None:
                time.sleep(0.05)
            self.init_coord = list(self.latest_frame["tvec"])
            logger.info(f"Home position set to {self.init_coord}")

    def takeoff(self):
        alt = self.args.takeoff_altitude
        logger.info(f"Taking off to {alt} m ...")
        self.flying = True
        duration = alt * 2
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
            if self.log_manager:
                self.log_manager.add_log_entry("cmd_positions", {
                    "timestamp": datetime.datetime.now().isoformat(timespec="milliseconds"),
                    "cmd": "go_to",
                    "x": xi, "y": yi, "z": z,
                    "note": "return to home before land",
                })
            self.commander.go_to(xi, yi, z, 0, dt, relative=False)
            time.sleep(dt + 0.5)

        dt = z * 6
        self.commander.land(0.12, dt)
        time.sleep(dt + 1)
        self.commander.stop()
        self.flying = False

    # ── Mission ──────────────────────────────────────────────────────────
    def run_mission(self):
        if self.args.role == "leader":
            self._run_leader()
        else:
            self._run_follower()

    def _send_setpoint_loop(self, x: float, y: float, z: float, yaw: float, duration: float):
        """Send position setpoints at `--setpoint-hz` for `duration` seconds."""
        dt = 1.0 / self.args.setpoint_hz
        t_end = time.time() + duration
        if self.log_manager:
            self.log_manager.add_log_entry("cmd_positions", {
                "timestamp": datetime.datetime.now().isoformat(timespec="milliseconds"),
                "cmd": "position_setpoint",
                "x": x, "y": y, "z": z, "yaw": yaw,
            })
        while time.time() < t_end:
            self.cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(dt)

    def _run_leader(self):
        """
        Move forward along Y by step_size mm, steps times.

        Protocol per step (--msg-order before, default):
          1. Send target to follower.
          2. Wait for ACK.
          3. Send GO.
          4. Start setpoint loop immediately (same moment as follower).
          5. After step_duration, hold position for settle_time, then next step.

        Protocol per step (--msg-order after):
          1. Start setpoint loop (leader moves first).
          2. Send target to follower.
          3. Wait for ACK.
          4. Send GO (follower starts moving after leader finishes this step).
          5. Next step.
        """
        step_m = self.args.step_size / 1000.0   # mm → m
        n = self.args.steps
        dur = self.args.step_duration
        settle = self.args.settle_time
        msg_order = self.args.msg_order

        # Absolute start position (XZ held constant throughout)
        if self.latest_frame:
            sx, sy, sz = self.latest_frame["tvec"]
        else:
            sx, sy, sz = self.init_coord

        logger.info(f"[LEADER] Starting step mission: {n} steps × {self.args.step_size} mm, "
                    f"duration {dur} s each, msg-order={msg_order}")

        for i in range(n):
            ty = sy + (i + 1) * step_m

            if msg_order == "before":
                logger.info(f"[LEADER] Step {i+1}/{n}: sending target y={ty:.4f} m (before move)")

                # ① Announce target
                self.tcp.send({"type": "target", "x": sx, "y": ty, "z": sz})
                self._send_setpoint_loop(sx, ty, sz, 0, dur + settle)

            else:  # msg_order == "after"
                logger.info(f"[LEADER] Step {i+1}/{n}: starting setpoint loop (before notifying follower)")

                # ① Leader moves first
                self._send_setpoint_loop(sx, ty, sz, 0, dur + settle)

                # ② Announce target to follower
                logger.info(f"[LEADER] Sending target y={ty:.4f} m (after move)")
                self.tcp.send({"type": "target", "x": sx, "y": ty, "z": sz})
        self.tcp.send({"type": "end"})

        self.cf.commander.send_notify_setpoint_stop()
        logger.info("[LEADER] Mission complete")

    def _run_follower(self):
        """
        React to leader's TCP commands.

        Protocol per step:
          1. Receive target from leader.
          2. Compute own absolute target (leader_target + offset).
          4. Wait for next step or "end".
        """
        offset = list(self.args.follow_offset)

        logger.info(f"[FOLLOWER] Ready with offset {offset}")

        i = 0
        tx, ty, tz = self.args.init_pos[0], self.args.init_pos[1], self.args.takeoff_altitude

        self.tcp.settimeout(0.01)
        while True:
            # ① Receive target or end

            try:
                msg = self.tcp.recv()
            except TimeoutError:
                # This block runs if 0.01s passes without data
                # print("No data received within the timeout period")
                msg = None
            if msg is not None and msg.get("type") == "end":
                break

            if msg is not None and msg.get("type") != "target":
                logger.warning(f"[FOLLOWER] Expected 'target', got: {msg}")
                tx = msg["x"] + offset[0]
                ty = msg["y"] + offset[1]
                tz = msg["z"] + offset[2]

            logger.info(f"[FOLLOWER]: target ({tx:.4f}, {ty:.4f}, {tz:.4f}) m")
            self.cf.commander.send_position_setpoint(tx, ty, tz, 0)

        self.cf.commander.send_notify_setpoint_stop()
        logger.info("[FOLLOWER] Mission complete")

    # ── Mocap callbacks ──────────────────────────────────────────────────
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


# ── CLI ───────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    tag = f"{datetime.datetime.now():%Y-%m-%d_%H-%M-%S}"

    ap = argparse.ArgumentParser(
        description="Synchronized drone leader-follower step mission over TCP"
    )

    # Role
    ap.add_argument("--role", required=True, choices=["leader", "follower"],
                    help="This drone's role: 'leader' (TCP server) or 'follower' (TCP client)")

    # Drone / connection
    ap.add_argument("--radio", type=str, default=None,
                    help="CrazyRadio URI (e.g. 'radio://0/6/1M/E7E7E7E704')")
    ap.add_argument("--takeoff-altitude", type=float, default=1.0,
                    help="Takeoff altitude in metres")
    ap.add_argument("--log-dir", type=str, default="./logs",
                    help="Directory to save log files")
    ap.add_argument("--tag", default=tag, type=str,
                    help="Tag included in log filenames")
    ap.add_argument("--cf-log-period", type=int, default=50,
                    help="CF logger period in ms")
    ap.add_argument("--fps", type=int, default=100,
                    help="Mocap frame rate (used for Kalman filter dt)")
    ap.add_argument("-v", "--verbose", action="store_true", default=False)
    ap.add_argument("-log", action="store_true", default=False,
                    help="Enable flight logging")

    # Mocap
    ap.add_argument("--vicon", action="store_true", help="Enable Vicon mocap")
    ap.add_argument("--vicon-full-pose", action="store_true",
                    help="Send position + orientation (otherwise position only)")
    ap.add_argument("--vicon-mode", default="mixed",
                    choices=["rigidbody", "pointcloud", "mixed"],
                    help="Mocap tracking mode")
    ap.add_argument("--obj-name", type=str, default=None,
                    help="Rigid-body object name in mocap (rigidbody mode)")
    ap.add_argument("--init-pos", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    help="Initial position hint [x y z] for pointcloud tracking")

    # Step mission
    ap.add_argument("--steps", type=int, default=5,
                    help="Number of Y-axis steps (n)")
    ap.add_argument("--step-size", type=float, default=200.0,
                    help="Distance per step in mm (k)")
    ap.add_argument("--step-duration", type=float, default=3.0,
                    help="Time allowed for each step in seconds")
    ap.add_argument("--settle-time", type=float, default=1.0,
                    help="Extra settle time after each step before the next")
    ap.add_argument("--setpoint-hz", type=float, default=50.0,
                    help="Rate at which send_position_setpoint is called during each step (Hz)")

    # Follower
    ap.add_argument("--follow-offset", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    help="[follower only] Offset [x y z] in metres from leader's target")

    # Synchronisation order
    ap.add_argument("--msg-order", choices=["before", "after"], default="before",
                    help="[leader only] 'before': notify follower then leader moves (default, simultaneous); "
                         "'after': leader moves first, then notifies follower")

    # TCP
    ap.add_argument("--tcp-host", type=str, default="0.0.0.0",
                    help="[follower] Leader's IP address; [leader] bind address")
    ap.add_argument("--tcp-port", type=int, default=DEFAULT_TCP_PORT,
                    help="TCP port for synchronisation socket")

    args = ap.parse_args()

    with DroneleaderController(args) as c:
        c.start()

# ── Example launch commands ───────────────────────────────────────────────────
# Leader (serves on port 5005, waits for follower, then flies 5 ×  mm steps):
#   python Interaction/tests/controller_drone_leader.py --role leader \
#       --vicon --vicon-mode pointcloud --init-pos -1 0 0.2 \
#       --takeoff-altitude 1.0 --steps 5 --step-size 50 --step-duration 3.0 \
#       --tcp-port 5005 --msg-order before
#
# Follower (connects to leader at 192.168.1.10, keeps +0.5 m X offset):
#   python Interaction/tests/controller_drone_leader.py --role follower \
#       --vicon --vicon-mode pointcloud --init-pos 0 0 0.2 \
#       --takeoff-altitude 1.0 --steps 5 \
#       --follow-offset 0.5 0 0 \
#       --tcp-host 192.168.1.177 --tcp-port 5005
