import argparse
import copy
import datetime
import json
from typing import Callable

import yaml
import logging
import numpy as np
import os
import subprocess
from threading import Event
import time
import urllib.request
import urllib.error
import zmq

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

from config import (
    DEFAULT_URI, LOG_VARS, DEFAULT_HEIGHT, DEFAULT_DURATION,
    PID_VALUES_PROP_2, POSITION_STD_DEV, ORIENTATION_STD_DEV,
    MIN_LIHV_VOLT
)

from mocap import Mocap
from tracker import Tracker
from logger import setup_logging

setup_logging()

logger = logging.getLogger(__name__)

# See https://github.com/whoenig/uav_trajectories for a tool to generate trajectories

pos_update_time_log = []
pos_update_profile_log = []


class LowBatteryException(Exception):
    pass


class EmergencyStopException(Exception):
    pass


def create_trajectory_from_file(file_path, takeoff_altitude):
    waypoints = []
    with open(file_path, "r") as f:
        trajectory = json.load(f)

    fps = trajectory["fps"]
    start_position = trajectory["start_position"]
    segments = trajectory["segments"]

    #  go to start position
    x0, y0, z0 = 0, 0, takeoff_altitude
    x, y, z = start_position
    z = takeoff_altitude + z
    dx, dy, dz = x - x0, y - y0, z - z0

    # move to start point in 3 seconds
    N = 3 * fps
    for i in range(1, N + 1):
        waypoints.append([x0 + i * dx / N, y0 + i * dy / N, z0 + i * dz / N])

    # stay at start point for 1 second
    for i in range(fps):
        waypoints.append([x, y, z])

    for i in range(2):
        for segment in segments:
            positions = segment["position"]
            velocities = segment["velocity"]
            state = segment["state"]
            if state == "RETURN" and i == 2:
                break

            for p, v in zip(positions, velocities):
                x, y, z = p
                # vx, vy, vz = v
                # self.send_position_target(x, y, -self.takeoff_altitude-z)
                # self.send_position_velocity_target(x, y, -self.takeoff_altitude - z, vx, vy, -vz)
                waypoints.append([x, y, takeoff_altitude + z])

    #  go to start position
    last_p = waypoints[-1]
    for i in range(fps):
        waypoints.append([last_p[0] - i * last_p[0] / fps, last_p[1] - i * last_p[1] / fps,
                          last_p[2] - i * (last_p[2] - takeoff_altitude) / fps])

    return np.array(waypoints), fps


class Controller:
    def __init__(self, args):
        self.args = args
        self.uri = uri_helper.uri_from_env(default=DEFAULT_URI)
        self.scf = None
        self.cf = None
        self.commander = None
        self.manifest = None
        self.mission = None
        self.min_voltage = MIN_LIHV_VOLT * 2

        self.mocap = None
        self.servo = None
        self.led = None
        self.tracker = None
        self.var_logger = None
        self.bat_logger = None
        self.sub_socket = None
        self.push_socket = None
        self.poller = None
        self.voltage = None
        self.deck_attached_event = Event()
        self.battery_critical = Event()
        self.start_time = 0
        self.flight_duration = 0

        self.log_data = copy.deepcopy(LOG_VARS)
        self.log_times = []
        self.flying = False
        self.failsafe = False
        self.init_coord = None

        self._safe_sleep: Callable[[float], None]
        if self.args.orchestrated:
            self._safe_sleep = self._safe_sleep_orchestrated
        else:
            self._safe_sleep = self._safe_sleep_standalone

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def connect(self):
        logger.info(f"Connecting to {self.uri}...")
        cflib.crtp.init_drivers(enable_serial_driver=True)
        self.scf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache'))
        self.scf.open_link()
        self.cf = self.scf.cf
        self.commander = self.cf.high_level_commander
        logger.info(f"Connected")

    def disconnect(self):
        logger.info("Disconnecting...")
        if self.scf:
            self.scf.close_link()

    def start(self):
        self.load_manifest()
        self.check_deck()
        self.setup_led()
        self.setup_servo()
        self.setup_sockets()
        self.setup_logging()
        self.setup_battery_watcher()
        self.setup_motion_capture()
        self.setup_tracker()
        self.setup_params()
        self.download_mission_config()
        self.handshake()
        if self.led:
            self.led.clear()
        self.save_init_coord()
        self.arm()
        self.takeoff()
        self.run_mission()

    def stop(self):
        self.flight_duration = time.time() - self.start_time

        if self.servo:
            self._set_safe_servo_angles()

        self.land()
        self._send_landing_confirmation()

        if self.var_logger:
            self.var_logger.stop()
            self.save_logs()

        if self.bat_logger:
            self.bat_logger.stop()

        if self.mocap:
            self.mocap.stop()

        if self.tracker:
            self.tracker.stop()

        if self.led:
            self.led.stop()

        if self.servo:
            del self.servo

        self.disconnect()

    def load_manifest(self):
        if not self.args.orchestrated:
            return

        with open('swarm_manifest.yaml', 'r') as f:
            self.manifest = yaml.safe_load(f)

        logger.debug("loaded manifest")

    def download_mission_config(self):
        if not self.args.orchestrated:
            return

        ip = self.manifest['controller']['ip']
        port = self.manifest['controller']['http_port']
        filename = self.manifest['controller']['mission_file']
        url = f"http://{ip}:{port}/{filename}"

        logger.info(f"  > Requesting: {url}")

        try:
            with urllib.request.urlopen(url) as response:
                data = response.read().decode('utf-8')
                self.mission = yaml.safe_load(data)

            logger.info(f"  > Download successful: {filename}")

        except urllib.error.URLError as e:
            logger.error(f"  > HTTP Error: {e}")
            raise

    def check_deck(self):
        if not self.args.check_deck:
            return

        self.cf.param.add_update_callback(group='deck', name=self.args.check_deck, cb=self._deck_callback)
        time.sleep(.5)

        if self.deck_attached_event.wait(timeout=5):
            logger.info("Deck detected")
        else:
            logger.error(f'No {self.args.check_deck} deck detected!')
            exit()

    def setup_servo(self):
        if self.args.servo:
            from servo_pwm import Servo
            offsets = [0, -180] if self.args.servo_type == 'a' else [-90, -270]
            self.servo = Servo(self.args.servo_count, offsets)
            time.sleep(0.1)
            self._set_safe_servo_angles()
            logger.info("servo activated")

    def setup_led(self):
        if self.args.led:
            from led import LED
            self.led = LED(brightness=self.args.led_brightness)
            self.led.show_single_color(color=(230, 180, 0))
            logger.debug("led activated")

    def setup_motion_capture(self):
        on_pose = None
        if self.args.vicon:
            if self.args.vicon_full_pose:
                on_pose = self._send_position_orientation
            else:
                on_pose = self._send_position
        if self.args.vicon:
            self.mocap = Mocap(
                object_name=self.args.obj_name,
                tag=self.args.tag,
                on_pose=on_pose
            )
            self.mocap.start()
            logger.debug("mocap activated")
        elif self.args.save_vicon:
            self.mocap = Mocap(
                object_name=self.args.obj_name,
                tag=self.args.tag,
                on_pose=on_pose
            )
            self.mocap.start()
            logger.debug("mocap activated")

    def setup_tracker(self):
        if self.args.tracker:
            self._start_tracker_process()
            time.sleep(2)
            self.tracker = Tracker(
                on_pose=self._send_position,
                on_failsafe=self._trigger_failsafe
            )
            self.tracker.start()
            logger.debug("tracker activated")

    def setup_sockets(self):
        if not self.args.orchestrated:
            return

        ctrl = self.manifest['controller']
        context = zmq.Context()
        self.push_socket = context.socket(zmq.PUSH)
        self.push_socket.connect(f"tcp://{ctrl['ip']}:{ctrl['zmq_ack_port']}")

        self.sub_socket = context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://{ctrl['ip']}:{ctrl['zmq_cmd_port']}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.poller = zmq.Poller()
        self.poller.register(self.sub_socket, zmq.POLLIN)

        logger.debug("sockets opened")

    def handshake(self):
        if not self.args.orchestrated:
            return

        logger.info(f"[{self.args.drone_id}] Sending READY...")
        self.push_socket.send_json({"id": self.args.drone_id, "status": "READY"})

        logger.info(f"[{self.args.drone_id}] Waiting for START...")
        while True:
            msg = self.sub_socket.recv_json()
            if msg.get('cmd') == 'START':
                break

        delay = int(self.args.drone_id.split('lb')[1]) * self.manifest['mission']['delta_t']
        logger.info(f"[{self.args.drone_id}] Launching in {delay}s...")
        time.sleep(delay)

    def save_init_coord(self):
        if self.mocap:
            self.init_coord = self.mocap.get_latest_pos()["tvec"]

    def takeoff(self):
        if self.args.ground_test:
            return

        logger.info("Taking off...")
        self.flying = True
        t = self.args.takeoff_altitude * 2
        self.cf.high_level_commander.takeoff(self.args.takeoff_altitude, t)
        self._safe_sleep(t + 1)

    def land(self):
        logger.info("Landing...")
        z = self.args.takeoff_altitude
        if self.init_coord:
            x, y, z = self.mocap.get_latest_pos()["tvec"]
            xi, yi, _ = self.init_coord
            dist = ((xi - x) ** 2 + (yi - y) ** 2) ** 0.5
            dt = 2 * dist
            self.commander.go_to(xi, yi, z, 0, dt, relative=False)
            time.sleep(dt + 0.5)

        if self.flying:
            dt = z * 6
            self.cf.high_level_commander.land(0.12, dt)
            time.sleep(dt + 1)
            self.cf.high_level_commander.stop()
            self.flying = False

    def setup_logging(self):
        if not self.args.log:
            return

        logger.info("Setting up logging...")
        self.var_logger = LogConfig(name='Controller', period_in_ms=10)

        for par, conf in LOG_VARS.items():
            self.var_logger.add_variable(par, conf["type"])

        self.cf.log.add_config(self.var_logger)
        self.var_logger.data_received_cb.add_callback(self._log_callback)
        self.var_logger.start()

        logger.debug("logging activated")

    def setup_battery_watcher(self):
        self.bat_logger = LogConfig(name='Battery', period_in_ms=1000)

        self.bat_logger.add_variable("pm.vbat", "float")

        self.cf.log.add_config(self.bat_logger)
        self.bat_logger.data_received_cb.add_callback(self._watch_battery)
        self.bat_logger.start()

        logger.info("Waiting for initial battery reading...")
        while self.voltage is None:
            time.sleep(0.1)

        logger.debug("logging activated")

    def setup_params(self):
        logger.info("Setting up parameters...")

        self._activate_kalman_estimator()
        self._set_position_sensitivity(POSITION_STD_DEV)
        self._set_orientation_sensitivity(ORIENTATION_STD_DEV)
        self._activate_pid_controller()
        self._activate_high_level_commander()
        self._set_pid_values(PID_VALUES_PROP_2)

        if not self.args.ground_test:
            reset_estimator(self.cf)

        if self.led:
            self.led.show_single_color(color=(80, 240, 30))

    def arm(self):
        if self.args.ground_test:
            return

        logger.info("Arming...")
        self.cf.platform.send_arming_request(True)
        time.sleep(1.0)

    def run_mission(self):
        self.start_time = time.time()

        if self.args.simple_takeoff:
            self.hover()
        elif self.args.xy_tune:
            self.xy_tune_pattern()
        elif self.args.z_tune:
            self.z_tune_pattern()
        elif self.args.trajectory:
            self.fly_trajectory(self.args.trajectory)
        elif self.args.orchestrated:
            self.orchestrated_mission()
        else:
            logger.info(f"Hovering for {self.args.t} seconds...")
            self._safe_sleep(self.args.t)

    def hover(self):
        self.commander.go_to(0.0, 0.0, self.args.takeoff_altitude, 0, 0.5, relative=False)
        self._safe_sleep(self.args.t)

    def xy_tune_pattern(self):
        logger.info("Executing XY Tune Pattern...")
        flight_time = 2
        self.commander.go_to(0, 0, 1, 0, flight_time, relative=False)
        self._safe_sleep(flight_time)

        for _ in range(3):
            self.commander.go_to(1.5, 0, 1, 0, flight_time, relative=False)
            self._safe_sleep(flight_time)

            self.commander.go_to(0, 0, 1, 0, flight_time, relative=False)
            self._safe_sleep(flight_time)

        for _ in range(3):
            self.commander.go_to(0, 1.5, 1, 0, flight_time, relative=False)
            self._safe_sleep(flight_time)

            self.commander.go_to(0, 0, 1, 0, flight_time, relative=False)
            self._safe_sleep(flight_time)

    def z_tune_pattern(self):
        logger.info("Executing Z Tune Pattern...")

        flight_time = 1.5
        self.commander.go_to(0, 0, 0.5, 0, flight_time, relative=False)
        self._safe_sleep(flight_time)

        for _ in range(3):
            self.commander.go_to(0, 0, 1.5, 0, flight_time, relative=False)
            self._safe_sleep(flight_time)
            self.commander.go_to(0, 0, 0.5, 0, flight_time, relative=False)
            self._safe_sleep(flight_time)

    def fly_trajectory(self, file_path):
        logger.info(f"Executing Trajectory from {file_path}...")
        trajectory, fps = create_trajectory_from_file(file_path, self.args.takeoff_altitude)

        for p in trajectory:
            self.commander.go_to(*p, 0, 1 / fps)
            self._safe_sleep(1 / fps)

    def orchestrated_mission(self):
        led_color = self.mission['drones'][self.args.drone_id]['color']
        self.led.show_single_color(led_color)

        mission_setting = self.mission['drones'][self.args.drone_id]
        x, y, z = mission_setting['target']
        waypoints = mission_setting.get('waypoints', [])
        params = mission_setting.get('params', {'linear': False, 'relative': False})
        angles = mission_setting.get('servos', [])
        delta_t = mission_setting['delta_t']
        iterations = mission_setting['iterations']

        total_flight_duration = delta_t * iterations * len(angles)

        dt = 1
        if self.init_coord:
            xi, yi, _ = self.init_coord
            dist = ((xi - x) ** 2 + (yi - y) ** 2) ** 0.5
            dt = 3 * dist

        if not self.args.ground_test:
            self.commander.go_to(x, y, z, 0, dt, relative=False)
            self._safe_sleep(dt + 1)

        if self.manifest['mission']['require_handshake']:
            self.handshake()

        if len(waypoints) and len(angles):
            self.sync_pos_servo(waypoints, angles, delta_t, iterations, params)
        elif len(angles):
            self.run_servo(angles, delta_t, iterations)
        self.led.clear()

    def run_servo(self, angles, delta_t, iterations):
        for _ in range(iterations):
            for a in angles:
                self.servo.set_all_smooth(a)
                self._safe_sleep(delta_t)

    def sync_pos_servo(self, waypoints, angles, delta_t, iterations, params):
        start_time = time.time()
        if len(angles) < len(waypoints):
            angles = angles + [angles[-1]] * (len(waypoints) - len(angles))
        n = len(waypoints)
        for i in range(iterations):
            for j, (w, a) in enumerate(zip(waypoints, angles)):
                self.commander.go_to(*w, delta_t, **params)
                logger.info(f"go to {w}")
                self.servo.set_all_smooth(a)

                target_time = start_time + ((i * n + j + 1) * delta_t)
                sleep_duration = target_time - time.time()
                # If we are ahead of schedule, sleep the difference
                if sleep_duration > 0:
                    self._safe_sleep(sleep_duration)
                else:
                    # If sleep_duration is negative, we are lagging behind!
                    logger.warning(f"Lagging behind by {abs(sleep_duration):.3f}s")

    def save_logs(self):
        log_dir = self.args.log_dir
        if not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)

        filename = os.path.join("logs", f"cf_{self.args.tag}.json")

        output_data = {
            "time": self.log_times,
            "params": self.log_data
        }
        with open(filename, 'w') as f:
            json.dump(output_data, f)
        logger.info(f"Logs saved to {filename}")

    def _safe_sleep_standalone(self, seconds):
        is_battery_critical = self.battery_critical.wait(timeout=seconds)

        if is_battery_critical:
            raise LowBatteryException(f"Battery Critical: {self.voltage:.2f}V")

    def _safe_sleep_orchestrated(self, duration):
        """
        Sleeps for 'duration' seconds, but interrupts IMMEDIATELY for:
        1. Low Battery
        2. ZMQ 'EMERGENCY' message
        """
        end_time = time.time() + duration

        # We wake up every 1000ms to check the Battery Event flag.
        # ZMQ messages will wake us up instantly.
        poll_interval_ms = 1000

        while True:
            remaining_seconds = end_time - time.time()
            if remaining_seconds <= 0:
                break

            # Check Battery
            if self.battery_critical.is_set():
                raise LowBatteryException(f"Battery Critical: {self.voltage:.2f}V")

            # Wait for ZMQ Message OR Timeout
            # take the smaller of: 100ms OR remaining time
            timeout_ms = int(min(poll_interval_ms, remaining_seconds * 1000))

            # This blocks until a message arrives OR timeout_ms passes
            socks = dict(self.poller.poll(timeout_ms))

            # Process ZMQ Messages (if any)
            if self.sub_socket in socks:
                try:
                    msg = self.sub_socket.recv_json(flags=zmq.NOBLOCK)

                    if msg.get('cmd') == 'EMERGENCY':
                        if self.led:
                            self.led.show_single_color((230, 20, 20))
                        raise EmergencyStopException("Orchestrator requested Emergency Stop")

                except zmq.Again:
                    pass

    def _watch_battery(self, timestamp, data, logconf):
        voltage = data['pm.vbat']
        self.voltage = voltage
        logger.debug(f"Voltage: {voltage:.2f}V")
        if voltage < self.min_voltage:
            self.battery_critical.set()
            if self.led:
                self.led.show_single_color((230, 20, 20))

    def _send_landing_confirmation(self):
        if self.args.orchestrated:
            self.push_socket.send_json({
                "id": args.drone_id,
                "status": "LANDED",
                "battery": self.voltage,
                "flight_duration": self.flight_duration
            })
            logger.info("Sent landing confirmation")

    def _log_callback(self, timestamp, data, log_conf):
        self.log_times.append(time.time())
        for par in self.log_data.keys():
            self.log_data[par]["data"].append(data[par])
            if self.args.verbose:
                logging.info(f"{par} = {data[par]}")

    def _set_pid_values(self, pid_dict):
        for param, value in pid_dict.items():
            self.cf.param.set_value(param, value)

    def _activate_high_level_commander(self):
        self.cf.param.set_value('commander.enHighLevel', '1')
        self.cf.param.set_value('hlCommander.vland', '0.1')

    def _activate_pid_controller(self):
        self.cf.param.set_value('stabilizer.controller', '1')

    def _activate_mellinger_controller(self):
        self.cf.param.set_value('stabilizer.controller', '2')

    def _activate_kalman_estimator(self):
        self.cf.param.set_value('stabilizer.estimator', '2')

    def _set_orientation_sensitivity(self, std_dev):
        self.cf.param.set_value('locSrv.extQuatStdDev', std_dev)

    def _set_position_sensitivity(self, std_dev):
        self.cf.param.set_value('locSrv.extPosStdDev', std_dev)

    def _set_safe_servo_angles(self):
        if self.args.servo_type == 'a':
            self.servo.set_all_smooth([0, 180])
        elif self.args.servo_type == 'b':
            self.servo.set_all_smooth([180, 360])

    def _start_tracker_process(self):
        """Starts the external C++ localization process."""
        params = [
            "/home/fls/fls-marker-localization/build/eye",
            "-t", str(20 + self.args.t),
            "--config", "/home/fls/fls-marker-localization/build/camera_config.json",
            "--brightness", "0.5",
            "--contrast", "2.5",
            "--exposure", "500",
            "--fps", str(self.args.fps),
        ]
        if self.args.save_camera:
            params.extend(["-s", "--save-rate", "10"])
        if self.args.stream_camera:
            params.extend(["--stream", "--stream-rate", "10"])

        subprocess.Popen(params)

    def _trigger_failsafe(self):
        self.failsafe = True

    def _deck_callback(self, _, value_str):
        value = int(value_str)
        if value:
            self.deck_attached_event.set()

    def _send_position(self, x, y, z, quat=None):
        self.scf.cf.extpos.send_extpos(x, y, z)

    def _send_position_orientation(self, x, y, z, quat):
        self.scf.cf.extpos.send_extpose(x, y, z, quat.x, quat.y, quat.z, quat.w)


if __name__ == '__main__':
    tag = f"{datetime.datetime.now():%Y-%m-%d_%H-%M-%S}"
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default=tag, type=str, help="tag included in filename of saved log files")
    ap.add_argument("--drone-id", type=str, help="drone id")
    ap.add_argument("--orchestrated", action="store_true", help="orchestrated by orchestrator")
    ap.add_argument("--takeoff-altitude", help="takeoff altitude", default=DEFAULT_HEIGHT, type=float)
    ap.add_argument("-t", help="flight duration", default=DEFAULT_DURATION, type=float)
    ap.add_argument("--fps", type=int, default=120, help="position estimation rate, works with --localize")
    ap.add_argument("--led", help="Turn LEDs on", action="store_true", default=False)
    ap.add_argument("--led-brightness", type=float, default=1.0, help="change led brightness between 0 and 1")
    ap.add_argument("--servo", help="Use servo", action="store_true", default=False)
    ap.add_argument("--servo-type", type=str, help="type of light bender servo setting")
    ap.add_argument("--servo-count", type=int, default=2, help="number of the servos")
    ap.add_argument("--check-deck", type=str, help="check if deck is attached, bcFlow2, bcZRanger2")
    ap.add_argument("--log", help="Enable logging", action="store_true", default=False)
    ap.add_argument("--tracker", help="Enable onboard marker localization", action="store_true", default=False)
    ap.add_argument("--vicon", action="store_true", help="localize using Vicon and save tracking data")
    ap.add_argument("--vicon-full-pose", action="store_true",
                    help="if passed send both position and orientation otherwise send only position")
    ap.add_argument("--obj-name", type=str, default="lightbender01",
                    help="object name in mocap system, works with --vicon.")
    ap.add_argument("--save-vicon", action="store_true", help="track with vicon and save the data")
    ap.add_argument("--log-dir", help="Log variables to the given directory", type=str, default="./logs")
    ap.add_argument("-v", "--verbose", help="Print logs if logging is enabled", action="store_true", default=False)
    ap.add_argument("--trajectory", type=str, help="path to trajectory file to follow")
    ap.add_argument("--ground-test", action="store_true", help="run mission without flying")
    ap.add_argument("--simple-takeoff", action="store_true", help="takeoff and land")
    ap.add_argument("--xy-tune", action="store_true", help="forward/back left/right flight pattern")
    ap.add_argument("--z-tune", action="store_true", help="up/down flight pattern")
    ap.add_argument("--save-camera", action="store_true",
                    help="save camera at 1/10 of original fps, works with --localize")
    ap.add_argument("--stream-camera", action="store_true",
                    help="stream camera at 1/10 of original fps, works with --localize")

    args = ap.parse_args()

    with Controller(args) as c:
        try:
            c.start()
        except LowBatteryException as e:
            logger.error(e)
        except EmergencyStopException as e:
            logger.error(e)

    # with open("pose_update_time.txt", "w") as f:
    #     for number in pos_update_time_log:
    #         f.write(f"{number}\n")
    #
    # with open("pose_update_profile.txt", "w") as f:
    #     for number in pos_update_profile_log:
    #         f.write(f"{number}\n")
