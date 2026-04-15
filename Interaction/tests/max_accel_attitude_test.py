"""
max_accel_attitude_test.py

Commands the drone to fly at maximum acceleration using the attitude controller
(send_zdistance_setpoint) and records full velocity telemetry for post-flight plotting.

Test sequence
─────────────
  1. Connect, set up logging and optional Vicon mocap.
  2. Arm and take off to --takeoff-altitude.
  3. Hover for --hover-time seconds (stabilise estimator).
  4. Command attitude burst: apply --pitch / --roll degrees for --accel-duration ms.
     The drone tilts and accelerates at maximum rate in the commanded direction.
  5. Return to hover, then go back to the home (takeoff) XY position.
  6. Land, save JSON log, and generate a velocity-vs-time PNG plot.

Velocity source
───────────────
  CF state estimator variables logged at --cf-log-period ms (default 10 ms → 100 Hz):
    stateEstimate.vx / vy / vz   — body-frame velocities (m/s)
    stateEstimate.x  / y  / z    — positions (m)
    stateEstimate.roll/pitch/yaw — Euler angles (deg)

Attitude convention
───────────────────
  send_zdistance_setpoint(roll, pitch, yawrate, zdistance)
    roll   > 0  → tilt right  → accelerate in  +X
    pitch  > 0  → tilt forward → accelerate in  +Y  (Crazyflie body frame)
  (opposite sign for negative direction)

Usage
─────
  python Interaction/tests/max_accel_attitude_test.py \\
      --vicon --vicon-mode pointcloud --init-pos 0 0 0 \\
      --takeoff-altitude 1.0 \\
      --pitch 20 --accel-duration 500 \\
      --log-dir ./logs

  # Without Vicon (onboard estimator only):
  python Interaction/tests/max_accel_attitude_test.py \\
      --takeoff-altitude 1.0 --pitch 20 --accel-duration 300
"""

import argparse
import datetime
import json
import logging
import os
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)

# ── Default parameters ────────────────────────────────────────────────────────
DEFAULT_URI          = "usb://0"
DEFAULT_HEIGHT       = 1.0
DEFAULT_LOG_PERIOD   = 10        # ms — 100 Hz velocity logging
DEFAULT_SETPOINT_HZ  = 100       # Hz — attitude setpoint rate during burst
DEFAULT_ACCEL_DUR_MS = 500       # ms — duration of the attitude kick
DEFAULT_HOVER_TIME   = 3.0       # s  — pre-burst hover stabilisation
DEFAULT_SETTLE_TIME  = 3.0       # s  — post-burst settling before return
DEFAULT_RETURN_TIME  = 3.0       # s  — time allowed to fly back to home XY

POSITION_STD_DEV    = 0.001
ORIENTATION_STD_DEV = 0.001

PID_VALUES = {
    'posCtlPid.xKp':         '1.9',  'posCtlPid.xKi':  '0.1',  'posCtlPid.xKd':  '0.0',
    'posCtlPid.yKp':         '2.1',  'posCtlPid.yKi':  '0.1',  'posCtlPid.yKd':  '0.0',
    'posCtlPid.zKp':         '1.9',  'posCtlPid.zKi':  '2.0',  'posCtlPid.zKd':  '0.05',
    'posCtlPid.thrustMin':   '12000',
    'posCtlPid.thrustBase':  '28000',
    'velCtlPid.vxKp':        '30.0', 'velCtlPid.vxKi': '4.0',  'velCtlPid.vxKd': '0.005',
    'velCtlPid.vyKp':        '30.0', 'velCtlPid.vyKi': '4.0',  'velCtlPid.vyKd': '0.005',
    'velCtlPid.vzKp':        '30.0', 'velCtlPid.vzKi': '5.0',  'velCtlPid.vzKd': '0.05',
    'pid_attitude.roll_kp':  '6.0',  'pid_attitude.roll_ki':  '1.0',  'pid_attitude.roll_kd':  '0.005',
    'pid_attitude.pitch_kp': '7.1',  'pid_attitude.pitch_ki': '1.0',  'pid_attitude.pitch_kd': '0.005',
    'pid_rate.roll_kp':      '90',   'pid_rate.roll_ki':  '270.0', 'pid_rate.roll_kd':  '2.5',
    'pid_rate.pitch_kp':     '75',   'pid_rate.pitch_ki': '270.0', 'pid_rate.pitch_kd': '2.5',
    'pid_rate.rateFiltEn':   '1',
    'pid_rate.omxFiltCut':   '160',
    'pid_rate.omyFiltCut':   '160',
    'pid_rate.omzFiltCut':   '160',
}


# ── Logging helper ────────────────────────────────────────────────────────────
class FlightLogger:
    """
    Collects CF telemetry via cflib LogConfig callbacks and writes everything
    to a JSON file on disk.

    Data schema
    ───────────
    {
      "meta": { "tag": ..., "args": {...} },
      "events": [
        { "name": "takeoff_complete", "t": 1234.567 },
        { "name": "accel_start",      "t": ..., "pitch_deg": 20, "roll_deg": 0 },
        { "name": "accel_end",        "t": ... },
        ...
      ],
      "cf_log": [
        { "t": 1234.567, "vx": 0.01, "vy": ..., "vz": ...,
          "x": ..., "y": ..., "z": ...,
          "roll": ..., "pitch": ..., "yaw": ... },
        ...
      ]
    }
    """

    def __init__(self, tag: str, log_dir: str):
        self.tag     = tag
        self.log_dir = log_dir
        self._lock   = threading.Lock()
        self.cf_log  = []      # list of dicts from CF telemetry callbacks
        self.events  = []      # milestone timestamps
        self._log_configs = []

    # ── CF telemetry ──────────────────────────────────────────────────────
    def start_cf_logging(self, cf, period_ms: int):
        """Attach LogConfig blocks to *cf* and start them."""
        # --- Velocity (vx, vy, vz) ---
        lc_vel = LogConfig("Vel", period_in_ms=period_ms)
        lc_vel.add_variable("stateEstimate.vx", "float")
        lc_vel.add_variable("stateEstimate.vy", "float")
        lc_vel.add_variable("stateEstimate.vz", "float")

        # --- Position (x, y, z) ---
        lc_pos = LogConfig("Pos", period_in_ms=period_ms)
        lc_pos.add_variable("stateEstimate.x", "float")
        lc_pos.add_variable("stateEstimate.y", "float")
        lc_pos.add_variable("stateEstimate.z", "float")

        # --- Attitude (roll, pitch, yaw) ---
        lc_att = LogConfig("Att", period_in_ms=period_ms)
        lc_att.add_variable("stateEstimate.roll",  "float")
        lc_att.add_variable("stateEstimate.pitch", "float")
        lc_att.add_variable("stateEstimate.yaw",   "float")

        # --- Accelerations (ax, ay, az) ---
        lc_acc = LogConfig("Acc", period_in_ms=period_ms)
        lc_acc.add_variable("acc.x", "float")
        lc_acc.add_variable("acc.y", "float")
        lc_acc.add_variable("acc.z", "float")

        for lc in (lc_vel, lc_pos, lc_att, lc_acc):
            try:
                cf.log.add_config(lc)
                lc.data_received_cb.add_callback(self._cf_data_cb)
                lc.start()
                self._log_configs.append(lc)
            except Exception as e:
                logger.warning(f"Could not start log block '{lc.name}': {e}")

    def _cf_data_cb(self, timestamp_ms, data, log_conf):
        t = time.time()
        entry = {"t": t}
        for k, v in data.items():
            # Strip namespace prefix: "stateEstimate.vx" → "vx"
            short = k.split(".")[-1]
            entry[short] = v
        with self._lock:
            self.cf_log.append(entry)

    # ── Events ────────────────────────────────────────────────────────────
    def record_event(self, name: str, **kwargs):
        entry = {"name": name, "t": time.time()}
        entry.update(kwargs)
        with self._lock:
            self.events.append(entry)
        logger.info(f"[EVENT] {name}  {kwargs if kwargs else ''}")

    # ── Teardown and save ─────────────────────────────────────────────────
    def stop(self):
        for lc in self._log_configs:
            try:
                lc.stop()
            except Exception:
                pass

    def save(self, args) -> str:
        os.makedirs(self.log_dir, exist_ok=True)
        path = os.path.join(self.log_dir, f"{self.tag}.json")
        payload = {
            "meta": {
                "tag":  self.tag,
                "args": vars(args),
            },
            "events":  self.events,
            "cf_log":  self.cf_log,
        }
        with open(path, "w") as fh:
            json.dump(payload, fh, indent=2)
        logger.info(f"Log saved → {path}")
        return path


# ── Main controller ───────────────────────────────────────────────────────────
class MaxAccelController:
    """
    Single-drone max-acceleration attitude test.

    Takes off, commands an attitude (pitch/roll) burst for a fixed duration
    to achieve maximum linear acceleration, then returns home and lands.
    """

    def __init__(self, args):
        self.args = args
        self.uri  = uri_helper.uri_from_env(default=args.radio or DEFAULT_URI)

        self.scf        = None
        self.cf         = None
        self.commander  = None   # high_level_commander
        self.mocap      = None
        self.logger     = FlightLogger(tag=args.tag, log_dir=args.log_dir)

        self.flying      = False
        self.init_coord  = None   # [x, y, z] at hover start
        self.latest_frame = None  # most recent Vicon frame

    # ── Context manager ───────────────────────────────────────────────────
    def __enter__(self):
        self._connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._stop()

    # ── Connection ────────────────────────────────────────────────────────
    def _connect(self):
        logger.info(f"Connecting to {self.uri} …")
        cflib.crtp.init_drivers(enable_serial_driver=True)
        self.scf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache="./cache"))
        self.scf.open_link()
        self.cf        = self.scf.cf
        self.commander = self.cf.high_level_commander
        logger.info("Connected")

    def _disconnect(self):
        if self.scf:
            self.scf.close_link()

    # ── Top-level lifecycle ───────────────────────────────────────────────
    def run(self):
        self._setup_params()
        self._setup_mocap()
        self._setup_logging()
        self._save_init_coord()
        self._arm()
        self._takeoff()
        self._mission()

    def _stop(self):
        self._land()
        if self.mocap:
            self.mocap.stop()
        self.logger.stop()
        self._disconnect()

    # ── Params / estimator ────────────────────────────────────────────────
    def _setup_params(self):
        logger.info("Configuring parameters …")
        self.cf.param.set_value("stabilizer.estimator", "2")    # Kalman
        self.cf.param.set_value("stabilizer.controller", "1")   # PID
        self.cf.param.set_value("commander.enHighLevel",  "1")
        self.cf.param.set_value("hlCommander.vland",      "0.1")
        if self.args.vicon:
            self.cf.param.set_value("locSrv.extPosStdDev", POSITION_STD_DEV)
            self.cf.param.set_value("locSrv.extQuatStdDev", ORIENTATION_STD_DEV)
        for param, value in PID_VALUES.items():
            self.cf.param.set_value(param, value)
        if self.args.vicon:
            reset_estimator(self.cf)

    # ── Mocap ─────────────────────────────────────────────────────────────
    def _setup_mocap(self):
        if not self.args.vicon:
            return
        from mocap import Mocap
        on_pose = (self._on_pose_full if self.args.vicon_full_pose
                   else self._on_pose)
        self.mocap = Mocap(mode=self.args.vicon_mode)
        if self.args.vicon_mode == "rigidbody":
            self.mocap.subscribe_object(self.args.obj_name, on_pose)
        else:
            self.mocap.subscribe_point(self.args.init_pos, on_pose, name="self")
        self.mocap.start()
        logger.info("Mocap running")

    def _on_pose(self, frame):
        self.cf.extpos.send_extpos(*frame["tvec"])
        self.latest_frame = frame

    def _on_pose_full(self, frame):
        self.cf.extpos.send_extpose(*frame["tvec"], *frame["quat"])
        self.latest_frame = frame

    # ── CF logging ────────────────────────────────────────────────────────
    def _setup_logging(self):
        logger.info("Starting CF telemetry logging …")
        self.logger.start_cf_logging(self.cf, self.args.cf_log_period)

    # ── Init coord ────────────────────────────────────────────────────────
    def _save_init_coord(self):
        if self.mocap:
            logger.info("Waiting for first Vicon frame …")
            while self.latest_frame is None:
                time.sleep(0.05)
            self.init_coord = list(self.latest_frame["tvec"])
            logger.info(f"Home position: {self.init_coord}")
        else:
            self.init_coord = list(self.args.init_pos)

    # ── Arm / Takeoff / Land ──────────────────────────────────────────────
    def _arm(self):
        logger.info("Arming …")
        self.cf.platform.send_arming_request(True)
        time.sleep(1.0)

    def _takeoff(self):
        alt = self.args.takeoff_altitude
        logger.info(f"Taking off to {alt:.2f} m …")
        self.flying = True
        duration = alt * 2
        self.commander.takeoff(alt, duration)
        time.sleep(duration + 1.0)
        self.logger.record_event("takeoff_complete", altitude_m=alt)

    def _land(self):
        self.cf.commander.send_notify_setpoint_stop()
        if not self.flying:
            return
        logger.info("Landing …")
        alt = self.args.takeoff_altitude
        # Return to home XY first (if we drifted far)
        if self.init_coord:
            xi, yi, _ = self.init_coord
            self.logger.record_event("return_to_home", x=xi, y=yi, z=alt)
            self.commander.go_to(xi, yi, alt, 0, self.args.return_time, relative=False)
            time.sleep(self.args.return_time + 0.5)
        dt = alt * 6
        self.commander.land(0.12, dt)
        time.sleep(dt + 1.0)
        self.commander.stop()
        self.flying = False
        self.logger.record_event("landed")

    # ── Mission ───────────────────────────────────────────────────────────
    def _mission(self):
        alt   = self.args.takeoff_altitude
        pitch = self.args.pitch    # degrees
        roll  = self.args.roll     # degrees
        yawrate = 0.0

        # 1. Hover and stabilise
        logger.info(f"Hovering for {self.args.hover_time:.1f} s …")
        self.logger.record_event("hover_start")
        time.sleep(self.args.hover_time)
        self.logger.record_event("hover_stable")

        # 2. Attitude burst — max acceleration
        duration_s = self.args.accel_duration / 1000.0
        dt         = 1.0 / self.args.setpoint_hz
        t_end      = time.time() + duration_s

        logger.info(
            f"Attitude burst: pitch={pitch}° roll={roll}° "
            f"for {self.args.accel_duration} ms at {self.args.setpoint_hz} Hz"
        )
        self.logger.record_event(
            "accel_start",
            pitch_deg=pitch,
            roll_deg=roll,
            duration_ms=self.args.accel_duration,
        )

        while time.time() < t_end:
            self.cf.commander.send_zdistance_setpoint(roll, pitch, yawrate, alt)
            time.sleep(dt)

        self.logger.record_event("accel_end")
        logger.info("Burst complete — stopping low-level commander")

        # Switch back to high-level commander
        self.cf.commander.send_notify_setpoint_stop()

        # 3. Settle in place
        logger.info(f"Settling for {self.args.settle_time:.1f} s …")
        self.logger.record_event("settle_start")
        time.sleep(self.args.settle_time)

        # 4. Return to home XY at cruise altitude
        if self.init_coord:
            xi, yi, _ = self.init_coord
            logger.info(f"Returning to home ({xi:.3f}, {yi:.3f}) …")
            self.logger.record_event("return_start", x=xi, y=yi, z=alt)
            self.commander.go_to(xi, yi, alt, 0, self.args.return_time, relative=False)
            time.sleep(self.args.return_time + 1.0)
            self.logger.record_event("return_complete")

        logger.info("Mission complete")


# ── Post-flight plotting ──────────────────────────────────────────────────────
def plot_velocity(log_path: str):
    """
    Read the JSON log and produce a velocity-vs-time PNG alongside it.

    Plots vx, vy, vz (m/s) on a shared time axis.
    Vertical dashed lines mark accel_start and accel_end.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        logger.warning("matplotlib not installed — skipping plot")
        return

    with open(log_path) as fh:
        data = json.load(fh)

    cf_log = data.get("cf_log", [])
    events = data.get("events", [])

    if not cf_log:
        logger.warning("No CF log data — skipping plot")
        return

    # ── Merge telemetry rows by timestamp ─────────────────────────────────
    # Each callback fires independently; align by rounding to 1 ms.
    rows: dict[float, dict] = {}
    for entry in cf_log:
        t = round(entry["t"], 3)
        if t not in rows:
            rows[t] = {"t": t}
        rows[t].update({k: v for k, v in entry.items() if k != "t"})

    sorted_rows = sorted(rows.values(), key=lambda r: r["t"])
    if not sorted_rows:
        return

    t0  = sorted_rows[0]["t"]
    ts  = [r["t"] - t0 for r in sorted_rows]
    vx  = [r.get("vx",    float("nan")) for r in sorted_rows]
    vy  = [r.get("vy",    float("nan")) for r in sorted_rows]
    vz  = [r.get("vz",    float("nan")) for r in sorted_rows]
    ax_ = [r.get("x",     float("nan")) for r in sorted_rows]
    ay_ = [r.get("y",     float("nan")) for r in sorted_rows]
    az_ = [r.get("z",     float("nan")) for r in sorted_rows]

    # ── Event markers ─────────────────────────────────────────────────────
    evt_times = {e["name"]: e["t"] - t0 for e in events}

    # ── Figure ────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # --- Velocity subplot ---
    ax_v = axes[0]
    ax_v.plot(ts, vx, label="vx (m/s)", color="tab:blue")
    ax_v.plot(ts, vy, label="vy (m/s)", color="tab:orange")
    ax_v.plot(ts, vz, label="vz (m/s)", color="tab:green")
    ax_v.set_ylabel("Velocity (m/s)")
    ax_v.set_title("Velocity vs. Time — Max Acceleration Attitude Test")
    ax_v.legend(loc="upper right")
    ax_v.grid(True, alpha=0.4)

    for marker, color, label in [
        ("accel_start", "red",    "Accel start"),
        ("accel_end",   "purple", "Accel end"),
        ("hover_stable", "gray",  "Hover stable"),
    ]:
        t_evt = evt_times.get(marker)
        if t_evt is not None:
            ax_v.axvline(t_evt, color=color, linestyle="--", linewidth=1.2, label=label)
    ax_v.legend(loc="upper right", fontsize=8)

    # --- Position subplot ---
    ax_p = axes[1]
    ax_p.plot(ts, ax_, label="x (m)", color="tab:blue")
    ax_p.plot(ts, ay_, label="y (m)", color="tab:orange")
    ax_p.plot(ts, az_, label="z (m)", color="tab:green")
    ax_p.set_xlabel("Time (s)")
    ax_p.set_ylabel("Position (m)")
    ax_p.set_title("Position vs. Time")
    ax_p.legend(loc="upper right")
    ax_p.grid(True, alpha=0.4)

    for marker, color in [("accel_start", "red"), ("accel_end", "purple")]:
        t_evt = evt_times.get(marker)
        if t_evt is not None:
            ax_p.axvline(t_evt, color=color, linestyle="--", linewidth=1.2)

    plt.tight_layout()
    plot_path = log_path.replace(".json", "_velocity.png")
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    logger.info(f"Plot saved → {plot_path}")


# ── CLI ───────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    _ts = f"{datetime.datetime.now():%Y-%m-%d_%H-%M-%S}"

    ap = argparse.ArgumentParser(
        description="Max-acceleration attitude test — logs velocity vs. time"
    )

    # Connection
    ap.add_argument("--radio", type=str, default=None,
                    help="CrazyRadio URI (default: usb://0 or CF_URI env var)")

    # Flight parameters
    ap.add_argument("--takeoff-altitude", type=float, default=DEFAULT_HEIGHT,
                    help="Takeoff altitude in metres (default %(default)s)")
    ap.add_argument("--hover-time", type=float, default=DEFAULT_HOVER_TIME,
                    help="Hover duration before burst in s (default %(default)s)")
    ap.add_argument("--settle-time", type=float, default=DEFAULT_SETTLE_TIME,
                    help="Settle time after burst in s (default %(default)s)")
    ap.add_argument("--return-time", type=float, default=DEFAULT_RETURN_TIME,
                    help="Time to fly back to home XY in s (default %(default)s)")

    # Attitude burst
    ap.add_argument("--pitch", type=float, default=0.0,
                    help="Pitch angle during burst in degrees (+Y acceleration). "
                         "Default %(default)s°")
    ap.add_argument("--roll", type=float, default=0.0,
                    help="Roll angle during burst in degrees (+X acceleration). "
                         "Default %(default)s°")
    ap.add_argument("--accel-duration", type=float, default=DEFAULT_ACCEL_DUR_MS,
                    help="Duration of attitude burst in ms (default %(default)s)")
    ap.add_argument("--setpoint-hz", type=float, default=DEFAULT_SETPOINT_HZ,
                    help="Setpoint rate during burst in Hz (default %(default)s)")

    # Mocap
    ap.add_argument("--vicon", action="store_true",
                    help="Enable Vicon mocap for external position updates")
    ap.add_argument("--vicon-full-pose", action="store_true",
                    help="Send position + orientation quaternion (instead of position only)")
    ap.add_argument("--vicon-mode", default="mixed",
                    choices=["rigidbody", "pointcloud", "mixed"],
                    help="Mocap tracking mode (default: %(default)s)")
    ap.add_argument("--obj-name", type=str, default=None,
                    help="Rigid-body object name (rigidbody mode only)")
    ap.add_argument("--init-pos", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    metavar=("X", "Y", "Z"),
                    help="Initial position hint [x y z] for pointcloud tracking")

    # Logging
    ap.add_argument("--log-dir", type=str, default="./logs",
                    help="Directory for log files (default: %(default)s)")
    ap.add_argument("--tag", type=str, default=None,
                    help="Log file name tag (default: auto-generated timestamp)")
    ap.add_argument("--cf-log-period", type=int, default=DEFAULT_LOG_PERIOD,
                    help="CF telemetry period in ms (default %(default)s = 100 Hz)")

    ap.add_argument("--fps", type=int, default=100,
                    help="Mocap frame rate used for KF dt (default %(default)s)")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="Enable DEBUG-level logging")

    args = ap.parse_args()

    if args.tag is None:
        p_str = f"p{int(args.pitch)}" if args.pitch != 0 else ""
        r_str = f"r{int(args.roll)}"  if args.roll  != 0 else ""
        dir_str = (p_str + r_str) or "hover"
        args.tag = f"max_accel_{dir_str}_{int(args.accel_duration)}ms_{_ts}"

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    if args.pitch == 0.0 and args.roll == 0.0:
        logger.warning(
            "Both --pitch and --roll are 0° — the drone will hover in place "
            "during the 'burst'. Pass a non-zero angle to test acceleration."
        )

    try:
        with MaxAccelController(args) as ctrl:
            ctrl.run()
    finally:
        log_path = ctrl.logger.save(args)
        plot_velocity(log_path)


# ── Example launch commands ───────────────────────────────────────────────────
# Maximum forward (+Y) acceleration, 20° pitch, 500 ms burst, with Vicon:
#   python Interaction/tests/max_accel_attitude_test.py \
#       --vicon --vicon-mode pointcloud --init-pos 0 0 0 \
#       --takeoff-altitude 1.0 \
#       --pitch 20 --accel-duration 500 \
#       --log-dir ./logs
#
# Maximum lateral (+X) acceleration, 15° roll, 400 ms burst, without Vicon:
#   python Interaction/tests/max_accel_attitude_test.py \
#       --takeoff-altitude 1.0 \
#       --roll 15 --accel-duration 400 \
#       --log-dir ./logs
#
# Combined pitch + roll (diagonal), short burst, verbose:
#   python Interaction/tests/max_accel_attitude_test.py \
#       --vicon --vicon-mode pointcloud --init-pos 0 0 0 \
#       --takeoff-altitude 1.0 \
#       --pitch 15 --roll 10 --accel-duration 300 -v \
#       --log-dir ./logs
