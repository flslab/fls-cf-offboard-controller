import argparse
import copy
import datetime
import json
import threading
from typing import Callable
import math
import random
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

from Interaction.interactions import InteractionsControl

from mocap import Mocap
from restart import reboot_crazyflie
from smooth_controller import SmoothController
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

        if getattr(self.args, 'illumination', False):
            import config as cfg
            self.cfg = cfg
        elif getattr(self.args, 'interaction', False):
            import Interaction.config as cfg
            self.cfg = cfg
        if self.args.takeoff_altitude is None:
            self.args.takeoff_altitude = self.cfg.DEFAULT_HEIGHT
        if self.args.t is None:
            self.args.t = self.cfg.DEFAULT_DURATION

        if self.args.radio:
            self.uri = uri_helper.uri_from_env(default=self.args.radio)
        else:
            self.uri = uri_helper.uri_from_env(default=self.cfg.DEFAULT_URI)
        self.scf = None
        self.cf = None
        self.commander = None
        self.manifest = None
        self.mission = None
        self.min_voltage = self.cfg.MIN_LIHV_VOLT * 2

        self.mocap = None
        self.servo = None
        self.led = None
        self.tracker = None
        self.log_manager = None
        self.bat_logger = None
        self.sub_socket = None
        self.push_socket = None
        self.poller = None
        self.voltage = None
        self.deck_attached_event = Event()
        self.battery_critical = Event()
        self.mission_start_time = 0
        self.mission_duration = 0
        self.animation_start_time = 0
        self.animation_stop_time = 0
        self.smooth_controller = None

        self.flying = False
        self.failsafe = False
        self.init_coord = None
        self.mocap_frames = []

        self._safe_sleep: Callable[[float], None]
        if self.args.orchestrated:
            self._safe_sleep = self._safe_sleep_orchestrated
        else:
            self._safe_sleep = self._safe_sleep_standalone

    def __enter__(self):
        if self.args.radio:
            reboot_crazyflie(self.uri)
            time.sleep(5)

        if self.args.droneless:
            logger.info("Testing Without Drone")
            self.cf = Crazyflie()
            return self

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
        self.setup_sockets()
        self.download_mission_config()
        self.setup_logging()
        self.setup_motion_capture()
        if not self.args.droneless:
            self.check_deck()
            self.setup_smooth_controller()
            self.setup_led()
            self.setup_servo()
            self.setup_battery_watcher()
            self.setup_tracker()
            self.setup_params()

        self.handshake()

        if not self.args.droneless:
            if self.led:
                self.led.clear()
            self.save_init_coord()
            self.arm()
            self.takeoff()
        self.run_mission()

    def stop(self):
        self.mission_duration = time.time() - self.mission_start_time

        if self.servo:
            self._set_safe_servo_angles()
            if self.args.ground_test:
                time.sleep(1)

        self.land()
        self._send_landing_confirmation()

        if self.bat_logger:
            self.bat_logger.stop()

        if self.mocap:
            self.mocap.stop()

        if self.log_manager:
            self.log_manager.stop(
                log_dir=self.args.log_dir,
                tag=self.args.tag,
                animation_start_time=self.animation_start_time,
                animation_stop_time=self.animation_stop_time
            )

        if self.tracker:
            self.tracker.stop()

        if self.smooth_controller:
            self.smooth_controller.stop()

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

    def setup_smooth_controller(self):
        if self.args.servo or self.args.led:
            self.smooth_controller = SmoothController(rate=self.args.smooth_controller_rate)

    def setup_servo(self):
        if not self.args.servo:
            return

        from servo_pwm import Servo
        offsets = [0, -180] if self.args.servo_type == "H" else [-90, -270]
        ranges = [(0, 180), (180, 360)] if args.servo_type == "H" else [(90, 270), (270, 450)]
        initial_values = (1, 181) if args.servo_type == "H" else (181, 361)

        self.servo = Servo(self.args.servo_count, offsets)
        time.sleep(0.1)
        self.smooth_controller.register_group(
            name="servos",
            initial_values=initial_values,
            callback=self.servo.set_all,
            ranges=ranges,
            offsets=self.args.servo_offsets
        )
        self._set_safe_servo_angles()
        logger.info("servos activated")

    def setup_led(self):
        if not self.args.led:
            return

        from led import LED
        self.led = LED(brightness=self.args.led_brightness, num_pixels=self.args.led_count)
        logger.debug("led activated")

    def setup_motion_capture(self):
        if not self.args.vicon:
            return

        if self.args.vicon_full_pose:
            on_pose = self._send_position_orientation
        else:
            on_pose = self._send_position

        self.mocap = Mocap(mode=self.args.vicon_mode)

        if self.args.vicon_mode == "rigidbody":
            logger.info(f"Subscribing to RigidBody: {self.args.obj_name}")
            self.mocap.subscribe_object(self.args.obj_name, on_pose)
        elif self.args.vicon_mode == "pointcloud":
            logger.info(f"Subscribing to Point closest to: {self.args.init_pos}")
            self.mocap.subscribe_point(self.args.init_pos, on_pose, name=self.args.drone_id)
        else:
            logger.info(f"Subscribing to Point closest to: {self.args.init_pos} in mix mode")
            self.mocap.subscribe_point(self.args.init_pos, on_pose, name=self.args.drone_id)

        self.mocap.start()

        anchor = self.mission.get("anchor", None)
        if anchor:
            logger.info(f"Tracking Anchor: {anchor}")
            self.mocap.set_anchor_point(anchor)

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
            if self._safe_sleep(1):
                break
            # msg = self.sub_socket.recv_json()
            # if msg.get('cmd') == 'START':
            #     break

        delay = (int(self.args.drone_id.split('lb')[1]) - 1) * self.manifest['mission']['delta_t']
        logger.info(f"[{self.args.drone_id}] Launching in {delay}s...")
        time.sleep(delay)

    def save_init_coord(self):
        if self.mocap and not self.args.ground_test and not (self.args.skip_landing and self.args.skip_takeoff):
            self.init_coord = self._get_latest_mocap_frame()["tvec"]

    def takeoff(self):
        if self.args.ground_test:
            return
        if self.args.skip_takeoff:
            self.flying = True
            return
        if self.args.interaction:
            self.log_manager.start()

        logger.info(f"Taking off to {args.takeoff_altitude}m ...")
        self.flying = True
        t = self.args.takeoff_altitude * 2
        self.cf.high_level_commander.takeoff(self.args.takeoff_altitude, t)
        self._safe_sleep(t + 1)

    def land(self):
        if self.args.skip_landing:
            return
        logger.info("Landing...")
        z = self.args.takeoff_altitude
        if self.init_coord:
            x, y, z = self._get_latest_mocap_frame()["tvec"]
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

    def _recap_takeoff(self):
        """Takeoff for a Recap iteration (flight-only, no log-start side-effects)."""
        if self.args.ground_test or self.args.skip_takeoff:
            self.flying = True
            return
        logger.info(f"[Recap] Taking off to {self.args.takeoff_altitude}m ...")
        self.flying = True
        t = self.args.takeoff_altitude * 2
        self.cf.high_level_commander.takeoff(self.args.takeoff_altitude, t)
        self._safe_sleep(t + 1)
        # Refresh init_coord so land() can return to the right spot.
        # if self.mocap:
        #     self.init_coord = self._get_latest_mocap_frame()["tvec"]

    def _recap_land(self):
        """Land after a Recap iteration and wait for the drone to settle."""
        if self.args.skip_landing:
            return
        logger.info("[Recap] Landing...")
        z = self.args.takeoff_altitude
        if self.init_coord:
            x, y, z = self._get_latest_mocap_frame()["tvec"]
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
        # Brief pause between iterations so the drone can fully settle.
        time.sleep(2)

    def setup_logging(self):
        if not self.args.log:
            return

        logger.info("Setting up logging...")
        if self.args.illumination:
            from log_manager import IlluminationLogger
            self.log_manager = IlluminationLogger(verbose=self.args.verbose)
            self.log_manager.start()
            if not self.args.droneless:
                self.log_manager.init_cf_logger(self.cf, self.cfg.LOG_VARS, self.args.cf_log_period)
            self.log_manager.add_log_group("frames")

        elif self.args.interaction:
            from Interaction.log_manager import InteractionLogger
            self.log_manager = InteractionLogger(controller_args=self.args)
            if not self.args.droneless:
                self.log_manager.init_cf_logger(self.cf, self.cfg.LOG_VARS, self.args.cf_log_period)
            self.log_manager.add_log_group("frames", kf=True)
            self.log_manager.add_log_group("events")
            self.log_manager.add_log_group("commands")
            self.log_manager.add_log_group("configs")
            self.log_manager.add_log_group("git")


        else:
            raise Exception("No mode is passed. Passing either --illumination or --interaction is required.")

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

        if self.led:
            self.led.show_single_color(color=(230, 180, 0))

        self._activate_kalman_estimator()
        if self.args.vicon:
            self._set_position_sensitivity(self.cfg.POSITION_STD_DEV)
            self._set_orientation_sensitivity(self.cfg.ORIENTATION_STD_DEV)
        self._activate_pid_controller()
        self._activate_high_level_commander()
        self._set_pid_values(self.cfg.PID_VALUES)

        if self.args.vicon and (not self.args.ground_test) and not (self.args.skip_landing and self.args.skip_takeoff):
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
        self.mission_start_time = time.time()

        if self.args.simple_takeoff:
            self.hover()
        elif self.args.rotation_test:
            self.test_rotation_limit()
        elif self.args.xy_tune:
            self.xy_tune_pattern()
        elif self.args.z_tune:
            self.z_tune_pattern()
        elif self.args.trajectory:
            self.fly_trajectory(self.args.trajectory)
        elif self.args.orchestrated:
            if self.args.illumination:
                self.orchestrated_mission()
            elif self.args.interaction:
                self.interation_switch()

        else:
            logger.info(f"Hovering for {self.args.t} seconds...")
            self._safe_sleep(self.args.t)

    def hover(self, hover_time=0.5):
        if hover_time is None:
            hover_time = self.args.t
        self.commander.go_to(0.0, 0.0, self.args.takeoff_altitude, 0, hover_time, relative=False)
        self._safe_sleep(hover_time)

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

    def orchestrated_mission_interaction(self):
        mission_setting = self.mission['drones'][self.args.drone_id]
        target = mission_setting['target']
        waypoints = mission_setting.get('waypoints', [])
        position_offset = mission_setting.get('position_offset', None)
        low_level_setpoint = mission_setting.get('low_level_setpoint', [])
        rotation_test = mission_setting.get('rotation_test', [])
        follow = mission_setting.get('follow', False)
        params = mission_setting.get('params', {'linear': False, 'relative': False})
        angles = mission_setting.get('servos', [])
        pointers = mission_setting.get('pointers', [])
        delta_t = mission_setting['delta_t']
        led_color = mission_setting.get('color')
        led_setting = mission_setting.get('led', {})
        interaction_mode = mission_setting.get('interaction', None)
        execution = False if self.args.droneless else True

        if position_offset:
            for i in range(3):
                target[i] += position_offset[i]
            for j in range(len(waypoints)):
                for i in range(3):
                    waypoints[j][i] += position_offset[i]

        if len(target) == 3:
            target.append(0.0)
        if len(target) == 4:
            target.append(0.0)
        x, y, z, yaw, _ = target

        dt = 3
        if self.init_coord:
            xi, yi, _ = self.init_coord
            dist = ((xi - x) ** 2 + (yi - y) ** 2) ** 0.5
            dt = 6 * dist

        if len(angles):
            self.smooth_controller.set_group_values("servos", angles[0], duration=1.0)

        if not self.args.ground_test:
            self.commander.go_to(x, y, z, yaw, dt, relative=False)
            self._safe_sleep(dt + 1)

        if self.manifest['mission']['require_handshake']:
            self.handshake()

        self.animation_start_time = time.time()

        if len(pointers):
            self.smooth_controller.register_group(
                name="pointers",
                initial_values=pointers[0],
                callback=lambda vals: self.update_led(vals, led_setting)
            )
        elif led_setting.get('mode') == 'expression':
            def update_led_cb():
                self.update_led(pointers, led_setting)
            self.smooth_controller.add_update_callback(update_led_cb)
        elif led_color is not None:
            self.led.show_single_color(led_color)

        if interaction_mode == 'peer':
            port = self.manifest['controller'].get('zmq_interact_port', 5560)
            peer_ips = [d['ip'] for d in self.manifest['drones'] if d['id'] != self.args.drone_id]
            peer_transport = self.mission.get('Interaction', {}).get('peer_transport', 'udp').lower()
            if peer_transport == 'tcp':
                from Interaction.ConnectionHelper import TCPPeerPublisher, TCPPeerSubscriber
                latency_test = self.mission.get('Interaction', {}).get('action') == 'peer_latency_test'
                interact_pub = TCPPeerPublisher(port, unlimited_hwm=latency_test)
                interact_sub = TCPPeerSubscriber(peer_ips, port, unlimited_hwm=latency_test)
                logger.info(f"Peer mode: TCP/ZMQ bound on :{port}, peers={peer_ips}")
            else:
                from Interaction.ConnectionHelper import UDPPublisher, UDPSubscriber
                interact_pub = UDPPublisher(peer_ips, port)
                interact_sub = UDPSubscriber(port)
                logger.info(f"Peer mode: UDP bound on :{port}, peers={peer_ips}")
            time.sleep(0.2)
            IC = InteractionsControl(self.cf, self._safe_sleep, self.log_manager, self.mission,
                                     self.args.smooth_controller_rate, drone_id=self.args.drone_id,
                                     pub_socket=interact_pub, sub_socket=interact_sub, execute=execution, set_color=self.led.show_single_color)
            IC.run()
            interact_pub.close()
            interact_sub.close()

        elif follow:
            leader_id = follow['id']
            leader_drone = self._get_drone_by_id(leader_id)
            if leader_drone:
                logger.info(f"following {leader_id}")
            
                if self.args.vicon_mode == "rigidbody":
                    leader_obj_name = leader_drone['obj_name']
                    self.mocap.subscribe_object(leader_obj_name,
                                                lambda frame: self._follow_with_offset(frame, follow['offset']))
                    self._safe_sleep(delta_t)
                    self.mocap.unsubscribe_object(leader_obj_name)
                elif self.args.vicon_mode == "pointcloud":
                    leader_target_pos = self.mission['drones'][leader_id]['target'][:3]
                    self.mocap.subscribe_point(leader_target_pos,
                                               lambda frame: self._follow_with_offset(frame, follow['offset']),
                                               name=leader_id)
                    self._safe_sleep(delta_t)
                    self.mocap.unsubscribe_point(leader_id)
                self.cf.commander.send_notify_setpoint_stop()
            leader_id = follow['id']
            leader_drone = self._get_drone_by_id(leader_id)

            if leader_drone:
                logger.info(f"following {leader_id}")
                self.log_manager.add_log_group(follow['id'], kf=True)
            if self.args.vicon_mode == "rigidbody":
                self.mocap.subscribe_object(follow['id'],
                                            lambda frame: self._log_mocap(frame, follow['id']))
            elif self.args.vicon_mode == "pointcloud":
                leader_target_pos = self.mission['drones'][follow['id']]['target'][:3]
                self.mocap.subscribe_point(leader_target_pos,
                                               lambda frame: self._log_mocap(frame, follow['id']),
                                               name=follow['id'])

            IC = InteractionsControl(self.cf, self._safe_sleep, self.log_manager, self.mission,
                                     self.args.smooth_controller_rate, leader_info=follow, execute=execution)
            IC.run()
            self.mocap.unsubscribe_point(leader_id)
            self.cf.commander.send_notify_setpoint_stop()

        elif interaction_mode == 'single':
            # Find drones that are passive (no 'interaction' and no 'follow' in their
            # mission config) — these become the I-LBs that receive commands from this drone.
            drone_mission = self.mission.get('drones', {})
            passive_ips = [
                d['ip'] for d in self.manifest['drones']
                if d['id'] != self.args.drone_id
                and drone_mission.get(d['id'], {}).get('interaction', None) == 'avoid'
            ]

            interact_pub = None
            interact_sub = None
            if passive_ips:
                port = self.manifest['controller'].get('zmq_interact_port', 5560)
                peer_transport = self.mission.get('Interaction', {}).get('peer_transport', 'tcp').lower()
                if peer_transport == 'tcp':
                    from Interaction.ConnectionHelper import TCPPeerPublisher, TCPPeerSubscriber
                    interact_pub = TCPPeerPublisher(port)
                    interact_sub = TCPPeerSubscriber(passive_ips, port)
                    logger.info(f"Interaction single: TCP on :{port}, I-LBs={passive_ips}")
                else:
                    from Interaction.ConnectionHelper import UDPPublisher, UDPSubscriber
                    interact_pub = UDPPublisher(passive_ips, port)
                    interact_sub = UDPSubscriber(port)
                    logger.info(f"Interaction single: UDP on :{port}, I-LBs={passive_ips}")
                time.sleep(0.5)  # allow I-LB subscribers to connect before first publish

            IC = InteractionsControl(self.cf, self._safe_sleep, self.log_manager, self.mission,
                                     self.args.smooth_controller_rate,
                                     pub_socket=interact_pub, sub_socket=interact_sub,
                                     execute=execution)
            IC.run()
            if interact_pub is not None:
                interact_pub.close()
            if interact_sub is not None:
                interact_sub.close()

        elif interaction_mode == 'avoid':
            # Passive I-LB: connect back to the UI-LB (interaction: single drone),
            # publish own position, and execute APF commands sent by the UI-LB.
            drone_mission = self.mission.get('drones', {})
            ui_lb_drone = next(
                (d for d in self.manifest['drones']
                 if drone_mission.get(d['id'], {}).get('interaction') == 'single'),
                None
            )
            interact_pub = None
            interact_sub = None
            if ui_lb_drone is not None:
                ui_lb_ip = ui_lb_drone['ip']
                port = self.manifest['controller'].get('zmq_interact_port', 5560)
                peer_transport = self.mission.get('Interaction', {}).get('peer_transport', 'tcp').lower()
                if peer_transport == 'tcp':
                    from Interaction.ConnectionHelper import TCPPeerPublisher, TCPPeerSubscriber
                    interact_pub = TCPPeerPublisher(port)
                    interact_sub = TCPPeerSubscriber([ui_lb_ip], port)
                    logger.info(f"Passive avoid: TCP on :{port}, UI-LB={ui_lb_ip}")
                else:
                    from Interaction.ConnectionHelper import UDPPublisher, UDPSubscriber
                    interact_pub = UDPPublisher([ui_lb_ip], port)
                    interact_sub = UDPSubscriber(port)
                    logger.info(f"Passive avoid: UDP on :{port}, UI-LB={ui_lb_ip}")
                time.sleep(0.5)

            IC = InteractionsControl(self.cf, self._safe_sleep, self.log_manager, self.mission,
                                     self.args.smooth_controller_rate,
                                     drone_id=self.args.drone_id,
                                     pub_socket=interact_pub, sub_socket=interact_sub,
                                     execute=execution)
            IC.run_passive_avoidance()
            if interact_pub is not None:
                interact_pub.close()
            if interact_sub is not None:
                interact_sub.close()

        self.animation_stop_time = time.time()

        if not len(pointers) and led_setting.get('mode') == 'expression':
            self.smooth_controller.remove_update_callback(update_led_cb)

        self.led.clear()

    def interation_switch(self):
        try:
            if self.args.ground_test:
                self._safe_sleep(30)
                return

            if self.args.intractable_illumination:
                self.orchestrated_mission_interaction()
            elif self.mission.get("Recap", None):
                recap_cfg = self.mission["Recap"]
                n = recap_cfg.get("iterations", 1)

                # Support both single file (legacy) and list of files.
                raw_files = recap_cfg.get("files", recap_cfg.get("file"))
                if isinstance(raw_files, str):
                    files = [raw_files]
                else:
                    files = list(raw_files)

                logger.info(f"Recap mode: {n} iteration(s) x {len(files)} file(s)")
                first_run = True
                for i in range(n):
                    for j, file_path in enumerate(files):
                        logger.info(f"--- Recap iteration {i + 1}/{n}, file {j + 1}/{len(files)}: {file_path} ---")

                        # Skip takeoff only on the very first run (already airborne).
                        if not first_run:
                            self._recap_land()
                            self._recap_takeoff()
                        first_run = False

                        # Build a per-file mission dict so IC sees the right file + alpha_vel.
                        IC = InteractionsControl(
                            self.cf, self._safe_sleep,
                            self.log_manager, self.mission,
                            self.args.smooth_controller_rate
                        )
                        IC.run_recap(file_path)
            elif self.mission.get('Interaction', {}).get('action') == 'peer_latency_test':
                port = self.manifest['controller'].get('zmq_interact_port', 5560)
                peer_ips = [d['ip'] for d in self.manifest['drones'] if d['id'] != self.args.drone_id]
                from Interaction.ConnectionHelper import TCPPeerPublisher, TCPPeerSubscriber
                interact_pub = TCPPeerPublisher(port, unlimited_hwm=True)
                interact_sub = TCPPeerSubscriber(peer_ips, port, unlimited_hwm=True)
                logger.info(f"Peer latency test: TCP bound on :{port}, peers={peer_ips}")
                time.sleep(0.2)
                IC = InteractionsControl(self.cf, self._safe_sleep, self.log_manager, self.mission,
                                         self.args.smooth_controller_rate, drone_id=self.args.drone_id,
                                         pub_socket=interact_pub, sub_socket=interact_sub)
                IC.run()
                interact_pub.close()
                interact_sub.close()

            else:
                n = self.mission["Interaction"].get("iteration", 1)
                logger.info(f"Iteration {n}")
                first_run = True
                for i in range(n):

                    if not first_run:
                        self._recap_land()
                        self._recap_takeoff()
                    first_run = False
                    mission_setting = self.mission['drones'][self.args.drone_id]
                    follow = mission_setting.get('follow', None)
                    if follow:
                        self.log_manager.add_log_group(follow['id'], kf=True)
                        if self.args.vicon_mode == "rigidbody":
                            self.mocap.subscribe_object(follow['id'],
                                                        lambda frame: self._log_mocap(frame, follow['id']))
                        elif self.args.vicon_mode == "pointcloud":
                            leader_target_pos = self.mission['drones'][follow['id']]['target'][:3]
                            self.mocap.subscribe_point(leader_target_pos,
                                                       lambda frame: self._log_mocap(frame, follow['id']),
                                                       name=follow['id'])

                    IC = InteractionsControl(self.cf, self._safe_sleep, self.log_manager, self.mission,
                                             self.args.smooth_controller_rate, leader_info=follow)
                    IC.run()

        except Exception as e:
            logging.error(f"Interaction Error: {e}\n")
        finally:
            self.cf.commander.send_notify_setpoint_stop()

    def orchestrated_mission(self):
        mission_setting = self.mission['drones'][self.args.drone_id]
        target = mission_setting['target']
        waypoints = mission_setting.get('waypoints', [])
        position_offset = mission_setting.get('position_offset', None)
        low_level_setpoint = mission_setting.get('low_level_setpoint', [])
        rotation_test = mission_setting.get('rotation_test', [])
        follow = mission_setting.get('follow', False)
        params = mission_setting.get('params', {'linear': False, 'relative': False})
        angles = mission_setting.get('servos', [])
        pointers = mission_setting.get('pointers', [])
        delta_t = mission_setting['delta_t']
        iterations = mission_setting['iterations']
        led_color = mission_setting.get('color')
        led_setting = mission_setting.get('led', {})

        if position_offset:
            for i in range(3):
                target[i] += position_offset[i]
            for j in range(len(waypoints)):
                for i in range(3):
                    waypoints[j][i] += position_offset[i]

        if len(target) == 3:
            target.append(0.0)
        if len(target) == 4:
            target.append(0.0)
        x, y, z, yaw, _ = target

        dt = 3
        if self.init_coord:
            xi, yi, _ = self.init_coord
            dist = ((xi - x) ** 2 + (yi - y) ** 2) ** 0.5
            dt = 6 * dist

        if len(angles):
            if self.args.morphing:
                self.smooth_controller.add_update_callback(lambda: self.update_servos(base_angle=angles[0]))
            else:
                self.smooth_controller.set_group_values("servos", angles[0], duration=1.0)

        if not self.args.ground_test:
            self.commander.go_to(x, y, z, yaw, dt, relative=False)
            self._safe_sleep(dt + 1)

        if self.manifest['mission']['require_handshake']:
            self.handshake()

        self.animation_start_time = time.time()

        if len(pointers):
            self.smooth_controller.register_group(
                name="pointers",
                initial_values=pointers[0],
                callback=lambda vals: self.update_led(vals, led_setting)
            )
            logger.info(f"Registered pointers with initial value: {pointers[0]}")
            self.update_led(pointers[0], led_setting)
        elif led_setting.get('mode') == 'expression':
            def update_led_cb():
                self.update_led(pointers, led_setting)
            self.smooth_controller.add_update_callback(update_led_cb)
        elif led_color is not None:
            self.led.show_single_color(led_color)

        if follow:
            leader_id = follow['id']
            leader_drone = self._get_drone_by_id(leader_id)
            if leader_drone:
                logger.info(f"following {leader_id}")
                if self.args.vicon_mode == "rigidbody":
                    leader_obj_name = leader_drone['obj_name']
                    self.mocap.subscribe_object(leader_obj_name,
                                                lambda frame: self._follow_with_offset(frame, follow['offset']))
                    self._safe_sleep(delta_t)
                    self.mocap.unsubscribe_object(leader_obj_name)
                elif self.args.vicon_mode == "pointcloud":
                    leader_target_pos = self.mission['drones'][leader_id]['target'][:3]
                    self.mocap.subscribe_point(leader_target_pos,
                                               lambda frame: self._follow_with_offset(frame, follow['offset']),
                                               name=leader_id)
                    self._safe_sleep(delta_t)
                    self.mocap.unsubscribe_point(leader_id)
                self.cf.commander.send_notify_setpoint_stop()
        elif len(low_level_setpoint):
            def position_setpoint_cb():
                self.cf.commander.send_position_setpoint(*low_level_setpoint)
            self.smooth_controller.add_update_callback(position_setpoint_cb)
            logger.info(f"send position setpoint {low_level_setpoint}")
            self._safe_sleep(delta_t)
            self.smooth_controller.remove_update_callback(position_setpoint_cb)
            self.cf.commander.send_notify_setpoint_stop()
        elif len(rotation_test):
            self.test_rotation_limit(*rotation_test)
        else:
            if not len(waypoints):
                waypoints.append([target[0], target[1], target[2], target[3], delta_t])
            if len(waypoints[0]) == 4:
                for w in waypoints:
                    w.append(delta_t)
            self.run_control_loop(waypoints, angles, pointers, params, delta_t, iterations)

        self.animation_stop_time = time.time()

        if not len(pointers) and led_setting.get('mode') == 'expression':
            self.smooth_controller.remove_update_callback(update_led_cb)

        self.led.clear()

    def run_control_loop(self, waypoints, angles, pointers, params, delta_t, iterations=1):
        elapsed_time = 0.0
        num_steps = max(len(waypoints), len(angles), len(pointers))
        if num_steps == 1:
            self._safe_sleep(waypoints[0][4])
            return

        for _ in range(iterations):
            if iterations > 1:
                self.smooth_controller.set_group_values("pointers", pointers[0], duration=0)
                sleep_duration = self.animation_start_time + elapsed_time + 0.1 - time.time()
                elapsed_time += 0.1
                self._safe_sleep(sleep_duration)

            for i in range(1, num_steps):
                duration = delta_t

                if i < len(waypoints):
                    duration = waypoints[i][4]
                    self.commander.go_to(*waypoints[i], **params)
                    logger.info(f"go to {waypoints[i]}")
                if i < len(angles):
                    self.smooth_controller.set_group_values("servos", angles[i], duration=duration)
                if i < len(pointers):
                    self.smooth_controller.set_group_values("pointers", pointers[i], duration=duration)

                target_time = self.animation_start_time + elapsed_time + duration
                sleep_duration = target_time - time.time()
                elapsed_time += duration
                # If we are ahead of schedule, sleep the difference
                if sleep_duration > 0:
                    self._safe_sleep(sleep_duration)
                else:
                    # If sleep_duration is negative, we are lagging behind!
                    logger.warning(f"Lagging behind by {abs(sleep_duration):.3f}s")

    def update_led(self, pointers, led_setting):
        formula_str = led_setting["formula"]
        led_buffer = []
        current_time = time.time() - self.animation_start_time
        if self.mocap:
            x, y, z = self._get_latest_mocap_frame()["tvec"]
        else:
            x, y, z = 0, 0, 0
        context = {"t": current_time, "i": 0, "N": self.args.led_count, "math": math, "x": x, "y": y, "z": z, "random": random}

        for j, p in enumerate(pointers):
            context[f"p{j}"] = p

        for i in range(self.args.led_count):
            context["i"] = i
            # Evaluate the string
            rgb = eval(formula_str, {}, context)
            led_buffer.append(rgb)

        self.led.set_colors(led_buffer)

    def update_servos(self, base_angle):
        latest_angles = self._get_latest_angles()

        if latest_angles is None:
            return

        logger.info(f"angles: {base_angle}")

        roll_deg = latest_angles[0]

        if self.args.servo_type == "H":
            target = np.array([base_angle[0] + roll_deg, base_angle[1] + roll_deg], dtype=float)
            limits = [(0.0, 180.0), (180.0, 360.0)]
        elif self.args.servo_type == "V":
            target = np.array([base_angle[0] + roll_deg, base_angle[1] + roll_deg], dtype=float)
            limits = [(90.0, 270.0), (270.0, 450.0)]
        else:
            return

        # Keep values inside the configured servo range.
        for i, (lo, hi) in enumerate(limits):
            target[i] = float(np.clip(target[i], lo, hi))

        self.smooth_controller.set_group_values(
            "servos",
            target.tolist(),
            duration=0.0,
        )

    def run_servo(self, angles, delta_t, iterations):
        for _ in range(iterations):
            for a in angles:
                self.smooth_controller.set_group_values("servos", a, duration=1.0)
                self._safe_sleep(delta_t)

    def sync_pos_servo(self, waypoints, angles, iterations, params):
        start_time = time.time()
        elapsed_time = 0.0
        if len(angles) < len(waypoints):
            angles = angles + [angles[-1]] * (len(waypoints) - len(angles))
        for i in range(iterations):
            for j, (w, a) in enumerate(zip(waypoints, angles)):
                duration = w[4]
                self.commander.go_to(*w, **params)
                logger.info(f"go to {w}")
                self.smooth_controller.set_group_values("servos", a, duration=duration)

                target_time = start_time + elapsed_time + duration
                sleep_duration = target_time - time.time()
                elapsed_time += duration
                # If we are ahead of schedule, sleep the difference
                if sleep_duration > 0:
                    self._safe_sleep(sleep_duration)
                else:
                    # If sleep_duration is negative, we are lagging behind!
                    logger.warning(f"Lagging behind by {abs(sleep_duration):.3f}s")

    def test_rotation_limit(self, low_limit=400, high_limit=600, num_steps=3, duration=5):
        dt = 1.0 / self.args.smooth_controller_rate
        start_t = time.time()

        while time.time() - start_t < 2:
            self.cf.commander.send_position_setpoint(0.0, 0.0, self.args.takeoff_altitude, 0)
            self._safe_sleep(dt)

        for yawrate in np.linspace(low_limit, high_limit, num=num_steps):

            start_t = time.time()
            while time.time() - start_t < duration:
                self.cf.commander.send_hover_setpoint(0.0, 0.0, yawrate, self.args.takeoff_altitude)
                self._safe_sleep(dt)

            while time.time() - start_t < duration + 2:
                self.cf.commander.send_position_setpoint(0.0, 0.0, self.args.takeoff_altitude, 0)
                self._safe_sleep(dt)

        self.cf.commander.send_notify_setpoint_stop()

    def _safe_sleep_standalone(self, seconds):
        is_battery_critical = self.battery_critical.wait(timeout=seconds)

        if is_battery_critical:
            self._prepare_for_emergency_landing()
            raise LowBatteryException(f"Battery Critical: {self.voltage:.2f}V")

    def _prepare_for_emergency_landing(self):
        self._set_safe_servo_angles()
        time.sleep(0.6)
        if self.smooth_controller:
            self.smooth_controller.stop()
        if self.led:
            self.led.show_single_color((230, 20, 20))
        self.cf.commander.send_notify_setpoint_stop()
        time.sleep(0.01)

    def _safe_sleep_orchestrated(self, duration):
        end_time = time.time() + duration
        poll_interval_ms = 1000

        while True:
            if self.battery_critical.is_set():
                self._prepare_for_emergency_landing()
                raise LowBatteryException(f"Battery Critical: {self.voltage:.2f}V")

            # If duration is 0, remaining_seconds will be 0 or slightly negative.
            # use max(0, ...) to ensure poll(0), which is a non-blocking check.
            remaining_seconds = max(0, end_time - time.time())
            timeout_ms = int(min(poll_interval_ms, remaining_seconds * 1000))

            # This blocks until a message arrives OR timeout_ms passes.
            # If duration=0, this returns immediately with any pending messages.
            socks = dict(self.poller.poll(timeout_ms))

            # Process ZMQ Messages (if any)
            if self.sub_socket in socks:
                try:
                    msg = self.sub_socket.recv_json(flags=zmq.NOBLOCK)

                    if msg.get('cmd') == 'EMERGENCY':
                        self._prepare_for_emergency_landing()
                        raise EmergencyStopException("Orchestrator requested Emergency Stop")
                    elif msg.get('cmd') == 'START':
                        return True
                except zmq.Again:
                    pass

            if time.time() >= end_time:
                break

    def _watch_battery(self, timestamp, data, logconf):
        voltage = data['pm.vbat']
        self.voltage = voltage
        logger.debug(f"Voltage: {voltage:.2f}V")
        if voltage < self.min_voltage:
            self.battery_critical.set()

    def _send_landing_confirmation(self):
        if self.args.orchestrated:
            self.push_socket.send_json({
                "id": self.args.drone_id,
                "status": "LANDED",
                "battery": self.voltage,
                "flight_duration": self.mission_duration
            })
            logger.info("Sent landing confirmation")

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
        if not self.servo or not self.smooth_controller:
            return
        if self.args.servo_type == "H":
            self.smooth_controller.set_group_values("servos", [0, 180], 0.5)
        elif self.args.servo_type == "V":
            self.smooth_controller.set_group_values("servos", [180, 360], 0.5)

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

    def _send_position(self, frame):
        self.cf.extpos.send_extpos(*frame['tvec'])
        self._log_mocap(frame)

    def _send_position_orientation(self, frame):
        self.cf.extpos.send_extpose(*frame['tvec'], *frame['quat'])
        self._log_mocap(frame)

    def _log_mocap(self, frame, group_name='frames'):
        self.log_manager.add_log_entry(group_name, frame)

    def _get_latest_mocap_frame(self):
        return self.log_manager.groups['frames'][-1]

    def _get_latest_angles(self, window_size=5):

        latest_roll = self.log_manager.cf_log_data["stateEstimate.roll"]["data"][-1]
        latest_pitch = self.log_manager.cf_log_data["stateEstimate.pitch"]["data"][-1]
        latest_yaw = self.log_manager.cf_log_data["stateEstimate.yaw"]["data"][-1]

        if latest_roll is None or latest_pitch is None or latest_yaw is None:
            return None

        # [roll, pitch, yaw] in degrees
        latest_angles = [latest_roll, latest_pitch, latest_yaw]

        return latest_angles

    def _get_drone_by_id(self, drone_id):
        for drone in self.manifest['drones']:
            if drone['id'] == drone_id:
                return drone

    def _get_interaction_drone_ip(self):
        for drone in self.manifest['drones']:
            if self.mission.get('drones', {}).get(drone['id'], {}).get('interaction', False):
                return drone['ip']
        raise RuntimeError("No drone with 'interaction: true' found in mission config")

    def _follow_with_offset(self, frame, offset):
        x, y, z = frame['tvec']
        xo, yo, zo = offset
        self.cf.commander.send_position_setpoint(x + xo, y + yo, z + zo, 0)


if __name__ == '__main__':
    tag = f"{datetime.datetime.now():%Y-%m-%d_%H-%M-%S}"
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default=tag, type=str, help="tag included in filename of saved log files")
    ap.add_argument("--drone-id", type=str, help="drone id")
    ap.add_argument("--orchestrated", action="store_true", help="orchestrated by orchestrator")
    ap.add_argument("--illumination", action="store_true", help="illumination application")
    ap.add_argument("--interaction", action="store_true", help="interaction application")
    ap.add_argument("--intractable-illumination", action="store_true", help="interaction application with illumination")
    ap.add_argument("--morphing", action="store_true", help="illumination application with morphing emulator")
    ap.add_argument("--takeoff-altitude", help="takeoff altitude", default=None, type=float)
    ap.add_argument("-t", help="flight duration", default=None, type=float)
    ap.add_argument("--fps", type=int, default=120, help="position estimation rate, works with --localize")
    ap.add_argument("--led", help="Turn LEDs on", action="store_true", default=False)
    ap.add_argument("--led-brightness", type=float, default=1.0, help="change led brightness between 0 and 1")
    ap.add_argument("--led-count", type=int, default=50, help="Number of LEDs")
    ap.add_argument("--servo", help="Use servo", action="store_true", default=False)
    ap.add_argument("--servo-type", type=str, choices=["H", "V"], default="H", help="type of light bender servo setting")
    ap.add_argument("--servo-count", type=int, default=2, help="number of the servos")
    ap.add_argument("--servo-offsets", type=float, nargs="*", default=[0, 0], help="calibration offsets of the servos")
    ap.add_argument("--smooth-controller-rate", type=int, default=30, help="rate of smooth controller update loop")
    ap.add_argument("--check-deck", type=str, help="check if deck is attached, bcFlow2, bcZRanger2")
    ap.add_argument("--log", help="Enable logging", action="store_true", default=False)
    ap.add_argument("--cf-log-period", type=int, default=50, help="log period of cf logger in millisecond")
    ap.add_argument("--log-dir", help="Log variables to the given directory", type=str, default="./logs")
    ap.add_argument("--tracker", help="Enable onboard marker localization", action="store_true", default=False)
    ap.add_argument("--vicon", action="store_true", help="localize using Vicon and save tracking data")
    ap.add_argument("--vicon-full-pose", action="store_true",
                    help="if passed send both position and orientation otherwise send only position")
    ap.add_argument("--obj-name", type=str,
                    help="object name in mocap system, works with --vicon.")
    ap.add_argument("--vicon-mode", default="mixed", choices=["rigidbody", "pointcloud", "mixed"], help="Tracking mode")
    ap.add_argument("--init-pos", type=float, nargs=3, help="Initial point x y z", default=[0.0, 0.0, 0.0])
    ap.add_argument("--save-vicon", action="store_true", help="track with vicon and save the data")
    ap.add_argument("-v", "--verbose", help="Print logs if logging is enabled", action="store_true", default=False)
    ap.add_argument("--trajectory", type=str, help="path to trajectory file to follow")
    ap.add_argument("--ground-test", action="store_true", help="run mission without flying")
    ap.add_argument("--simple-takeoff", action="store_true", help="takeoff and land")
    ap.add_argument("--rotation-test", action="store_true", help="test rotation rate")
    ap.add_argument("--xy-tune", action="store_true", help="forward/back left/right flight pattern")
    ap.add_argument("--z-tune", action="store_true", help="up/down flight pattern")
    ap.add_argument("--save-camera", action="store_true",
                    help="save camera at 1/10 of original fps, works with --localize")
    ap.add_argument("--stream-camera", action="store_true",
                    help="stream camera at 1/10 of original fps, works with --localize")
    ap.add_argument("--skip-takeoff", action="store_true", help="run mission without taking off")
    ap.add_argument("--skip-landing", action="store_true", help="run mission without landing")
    ap.add_argument("--radio", type=str, help="specify the CrazyRadio URI (e.g., 'radio://0/6/1M/E7E7E7E704')")
    ap.add_argument("--droneless", action="store_true", help="run mission without connecting to fc")


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
