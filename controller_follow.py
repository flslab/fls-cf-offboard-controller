"""
controller_follow.py
Stripped-down controller that keeps ONLY the leader-follower logic with mocap.

Usage (follower):
    python controller_follow.py \
        --vicon --vicon-mode pointcloud --init-pos 0 0 0 \
        --leader-id lb1 --leader-pos 1 0 0 --follow-offset 0.5 0 0 \
        --takeoff-altitude 1.0 -t 30

Usage (leader — just hovers, no follow):
    python controller_follow.py \
        --vicon --vicon-mode pointcloud --init-pos 1 0 0 \
        --takeoff-altitude 1.0 -t 30
"""

import argparse
import datetime
import logging
import time

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
DEFAULT_DURATION = 10.0

POSITION_STD_DEV = 0.001
ORIENTATION_STD_DEV = 0.001

PID_VALUES = {
    # 'posCtlPid.xKp': '1.9',  'posCtlPid.xKi': '0.1',  'posCtlPid.xKd': '0.0',
    # 'posCtlPid.yKp': '2.1',  'posCtlPid.yKi': '0.1',  'posCtlPid.yKd': '0.0',

    'posCtlPid.xKp': '4', 'posCtlPid.xKi': '0.1', 'posCtlPid.xKd': '0.0',
    'posCtlPid.yKp': '4', 'posCtlPid.yKi': '0.1', 'posCtlPid.yKd': '0.0',
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


# ── Controller ───────────────────────────────────────────────────────────────
class FollowController:
    """Minimal controller: connect → mocap → takeoff → follow leader → land."""

    def __init__(self, args):
        self.args = args

        if self.args.takeoff_altitude is None:
            self.args.takeoff_altitude = DEFAULT_HEIGHT
        if self.args.t is None:
            self.args.t = DEFAULT_DURATION

        self.uri = uri_helper.uri_from_env(default=self.args.radio or DEFAULT_URI)

        self.scf = None
        self.cf = None
        self.commander = None
        self.mocap = None
        self.log_manager = None

        self.flying = False
        self.init_coord = None
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
        logger.info("Disconnecting ...")
        if self.scf:
            self.scf.close_link()

    # ── Lifecycle ────────────────────────────────────────────────────────
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
            logger.info(f"Subscribing to Point closest to: {self.args.init_pos}")
            self.mocap.subscribe_point(self.args.init_pos, on_pose, name="self")

        self.mocap.start()
        logger.info("Mocap activated")

    # ── Logging ───────────────────────────────────────────────────────────
    def setup_logging(self):
        logger.info("Setting up logging ...")
        self.log_manager = InteractionLogger(controller_args=self.args)
        self.log_manager.init_cf_logger(self.cf, cfg.LOG_VARS, self.args.cf_log_period)
        self.log_manager.add_log_group("frames", kf=True)
        self.log_manager.add_log_group("events")
        self.log_manager.add_log_group("commands")
        self.log_manager.add_log_group("configs")
        self.log_manager.start()
        logger.info("Logging activated")


    # ── Params / estimator ───────────────────────────────────────────────
    def setup_params(self):
        logger.info("Setting up parameters ...")
        self.cf.param.set_value("stabilizer.estimator", "2")        # Kalman
        if self.args.vicon:
            self.cf.param.set_value("locSrv.extPosStdDev", POSITION_STD_DEV)
            self.cf.param.set_value("locSrv.extQuatStdDev", ORIENTATION_STD_DEV)
        self.cf.param.set_value("stabilizer.controller", "1")       # PID
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
        if self.mocap and self.latest_frame:
            self.init_coord = self.latest_frame["tvec"]

    def takeoff(self):
        alt = self.args.takeoff_altitude
        logger.info(f"Taking off to {alt}m ...")
        self.flying = True
        t = alt * 2
        self.commander.takeoff(alt, t)
        time.sleep(t + 1)

    def land(self):
        logger.info("Landing ...")
        z = self.args.takeoff_altitude

        # Return to initial XY if we know it
        if self.init_coord and self.latest_frame:
            x, y, z = self.latest_frame["tvec"]
            xi, yi, _ = self.init_coord
            dist = ((xi - x) ** 2 + (yi - y) ** 2) ** 0.5
            dt = 2 * dist
            self.commander.go_to(xi, yi, z, 0, dt, relative=False)
            time.sleep(dt + 0.5)

        if self.flying:
            dt = z * 6
            self.commander.land(0.12, dt)
            time.sleep(dt + 1)
            self.commander.stop()
            self.flying = False

    # ── Mission ──────────────────────────────────────────────────────────
    def run_mission(self):
        if self.args.leader_id:
            self.follow_leader()
        else:
            logger.info(f"No leader specified — hovering for {self.args.t}s")
            time.sleep(self.args.t)

    def follow_leader(self):
        """Subscribe to the leader's mocap data and follow with offset."""
        leader_id = self.args.leader_id
        offset = list(self.args.follow_offset)
        duration = self.args.t

        logger.info(f"Following leader '{leader_id}' with offset {offset} for {duration}s")

        if self.args.vicon_mode == "rigidbody":
            self.mocap.subscribe_object(
                leader_id,
                lambda frame: self._follow_with_offset(frame, offset),
            )
            time.sleep(duration)
            self.mocap.unsubscribe_object(leader_id)

        elif self.args.vicon_mode in ("pointcloud", "mixed"):
            leader_pos = list(self.args.leader_pos)
            self.mocap.subscribe_point(
                leader_pos,
                lambda frame: self._follow_with_offset(frame, offset),
                name=leader_id,
            )
            time.sleep(duration)
            self.mocap.unsubscribe_point(leader_id)

        self.cf.commander.send_notify_setpoint_stop()
        logger.info("Follow complete")

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

    def _follow_with_offset(self, frame, offset):
        x, y, z = frame["tvec"]
        xo, yo, zo = offset
        self.cf.commander.send_position_setpoint(x + xo, y + yo, z + zo, 0)




# ── CLI ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    tag = f"{datetime.datetime.now():%Y-%m-%d_%H-%M-%S}"

    ap = argparse.ArgumentParser(description="Leader-Follower controller with mocap (minimal)")

    # Drone / connection
    ap.add_argument("--radio", type=str, default=None,
                    help="CrazyRadio URI (e.g. 'radio://0/6/1M/E7E7E7E704')")

    ap.add_argument("--takeoff-altitude", type=float, default=1.0, help="Takeoff altitude in metres")
    ap.add_argument("-t", type=float, default=None, help="Mission duration in seconds")
    ap.add_argument("--log-dir", type=str, default="./logs", help="Directory to save log files")
    ap.add_argument("--tag", default=tag, type=str, help="Tag included in log filenames")
    ap.add_argument("--cf-log-period", type=int, default=50, help="CF logger period in ms")
    ap.add_argument("--fps", type=int, default=100, help="Mocap frame rate (used for Kalman filter dt)")
    ap.add_argument("-v", "--verbose", action="store_true", default=False, help="Print logs if logging is enabled")
    ap.add_argument("-log", action="store_true", default=False, help="Log Movements")

    # Mocap
    ap.add_argument("--vicon", action="store_true", help="Enable Vicon mocap")
    ap.add_argument("--vicon-full-pose", action="store_true",
                    help="Send position + orientation (otherwise position only)")
    ap.add_argument("--vicon-mode", default="mixed", choices=["rigidbody", "pointcloud", "mixed"],
                    help="Mocap tracking mode")
    ap.add_argument("--obj-name", type=str, default=None,
                    help="Rigid-body object name in mocap (rigidbody mode)")
    ap.add_argument("--init-pos", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    help="Initial position hint [x y z] for pointcloud tracking")

    # Leader-follower
    ap.add_argument("--leader-id", type=str, default=None,
                    help="Leader object/point name to follow (omit to just hover)")
    ap.add_argument("--leader-pos", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    help="Leader initial position [x y z] (for pointcloud mode)")
    ap.add_argument("--follow-offset", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    help="Offset [x y z] to maintain from leader position")

    args = ap.parse_args()

    with FollowController(args) as c:
        c.start()

# # Leader (just hovers at 1m for 30s):
# python controller_follow.py --vicon --vicon-mode pointcloud --init-pos 1 0 0 -t 30
#
# # Follower (tracks leader with +0.5m X offset):
# python controller_follow.py --vicon --vicon-mode pointcloud --init-pos 0 0 0.2 \
#     --leader-id lb1 --leader-pos 1 0 0 --follow-offset 0.5 0 0 -t 30
