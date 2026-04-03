import argparse
import io
import os
import yaml
import json
import time
import zmq
import threading
import http.server
import socketserver
import sys
import signal
import logging
from datetime import datetime
from functools import partial
from fabric import Connection
from invoke.exceptions import CommandTimedOut

# Assumed local modules based on your import list
from logger import setup_logging
from restart import reboot_crazyflie

# from Interaction.vicon_noise_tracker import run_tracker

MANIFEST_FILE = 'swarm_manifest.yaml'
DRONE_SCRIPT = 'controller.py'
CAMERA_SCRIPT = 'camera_node.py'


class SwarmOrchestrator:
    def __init__(self, args):
        setup_logging()
        self.logger = logging.getLogger(__name__)
        self.args = args

        # Configuration State
        self.manifest = self._load_manifest()
        self.ctrl_cfg = self.manifest['controller']
        if args.http_port:
            self.ctrl_cfg['http_port'] = args.http_port
        if args.zmq_cmd_port:
            self.ctrl_cfg['zmq_cmd_port'] = args.zmq_cmd_port
        if args.zmq_ack_port:
            self.ctrl_cfg['zmq_ack_port'] = args.zmq_ack_port
        self.drones = self.manifest.get('drones', [])
        self.camera_cfg = self.manifest.get('camera_node')
        self.radio_node = self.manifest.get('radio_node')
        self.common_cfg = self.manifest['common']
        self.mission = self._load_mission()

        # Runtime State
        self.tag = None
        self.running = threading.Event()
        self.emergency = False
        self.pending_downloads = self._load_previous_downloads()
        self.landed_drones = set()
        self.ready_ids = set()

        self.loadcell_thread = None
        self.vicon_tracker_thread = None

        # Network Resources
        self.zmq_context = None
        self.pub_socket = None
        self.pull_socket = None
        self.http_server = None

        # Bind signal handlers for graceful exit
        # signal.signal(signal.SIGINT, self._signal_handler)
        # signal.signal(signal.SIGTERM, self._signal_handler)

    def _load_manifest(self):
        with open(MANIFEST_FILE) as f:
            return yaml.safe_load(f)

    def _load_mission(self):
        with open(os.path.join(self.ctrl_cfg['mission_path'], self.ctrl_cfg['mission_file'])) as f:
            return yaml.safe_load(f)

    def _load_previous_downloads(self):
        try:
            with open(self.ctrl_cfg['downloads_file']) as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return []

    def _signal_handler(self, sig, frame):
        self.logger.warning(f"Signal {sig} received. Initiating Emergency Shutdown...")
        self.emergency_stop()

    def setup_network(self):
        """Initializes ZMQ sockets and HTTP server."""
        # HTTP Server
        port = self.ctrl_cfg['http_port']
        directory = self.ctrl_cfg['mission_path']

        # Reuse address to prevent 'Address already in use' on quick restarts
        socketserver.TCPServer.allow_reuse_address = True

        handler = partial(http.server.SimpleHTTPRequestHandler, directory=directory)
        self.http_server = socketserver.TCPServer(("", port), handler)

        http_thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
        http_thread.start()
        self.logger.info(f"HTTP Config Server running on port {port} serving {directory}")

        # ZMQ
        self.zmq_context = zmq.Context()
        self.pub_socket = self.zmq_context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://*:{self.ctrl_cfg['zmq_cmd_port']}")

        self.pull_socket = self.zmq_context.socket(zmq.PULL)
        self.pull_socket.bind(f"tcp://*:{self.ctrl_cfg['zmq_ack_port']}")

        # Set a timeout on receive so we can check for shutdown flags periodically
        self.pull_socket.setsockopt(zmq.RCVTIMEO, 1000)

        self.logger.info(f"ZMQ Server Online at {self.ctrl_cfg['ip']}")

    def _get_drone_cmd(self, drone):
        if self.args.interaction:
            return self._get_drone_cmd_interaction(drone)
        elif self.args.illumination:
            return self._get_drone_cmd_illumination(drone)
        elif self.args.morphing:
            return self._get_drone_cmd_morphing(drone)
        else:
            raise Exception("mode not supported")

    def _get_drone_cmd_interaction(self, drone):
        alt = self.mission['drones'][drone['id']]['target'][2]
        servo_count = drone.get('servo_count', 2)
        led_count = drone.get('led_count', 50)
        radio_arg = f"--radio {drone['uri']}" if self.args.radio else ""
        p = drone.get('init_pos', None)
        obj_name = drone.get('label', None)
        if p:
            mocap_args = f"--init-pos {p[0]} {p[1]} {p[2]} --vicon-mode pointcloud "
        elif obj_name:
            mocap_args = f"--obj-name {obj_name} --vicon-mode rigidbody "

        extra_markers = self.manifest.get('apparatus', None)
        extra_marker_args = ""
        if extra_markers:
            for m in extra_markers:
                extra_marker_args += f"--extra-marker {m['id']} "
                init_pos = m.get("init_pos")
                if init_pos is not None:
                    extra_marker_args += f"{' '.join(str(c) for c in m['init_pos'])} "
        cmd = [
            f"cd {self.common_cfg['work_dir']} && "
            f"source {self.common_cfg['venv_path']}/bin/activate && "
            "git pull && "
            f"nohup python3 {DRONE_SCRIPT} "
            f"--orchestrated --interaction --tag {self.tag} ",
            f"--intractable-illumination" if getattr(self.args, 'intractable_illumination', False) else "",
            f"{radio_arg} "
            f"{extra_marker_args} "
            f"--vicon {mocap_args} "
            f"--drone-id {drone['id']} ",
            f"--led --led-count {led_count} " if led_count > 0 and not self.args.radio else " ",
            f"--servo --servo-type {drone['type']} --servo-count {servo_count} " if servo_count > 0 and not self.args.radio else " ",
            f"--takeoff-altitude {alt} "
            "--smooth-controller-rate 50 "
            "--log "
            # "--cf-log-period 10 "
            # "--skip-takeoff --skip-landing "
            f"> drone_{drone['id']}.log 2>&1 < /dev/null &",
        ]

        # If a reference object is configured, also launch the noise tracker on the drone.
        ref_obj = self.common_cfg.get('reference_object')
        if ref_obj:
            vicon_log = f"{self.common_cfg['work_dir']}/logs/vicon_{self.tag}.json"
            cmd.append(
                f"nohup python3 Interaction/vicon_noise_tracker.py "
                f"--subject {ref_obj} --duration 10 "
                f"--out {vicon_log} "
                f"> vicon_noise.log 2>&1 < /dev/null &"
            )

        return f" ".join(cmd)

    def _get_drone_cmd_illumination(self, drone):
        alt = self.mission['drones'][drone['id']]['target'][2]
        servo_count = drone.get('servo_count', 2)
        led_count = drone.get('led_count', 50)
        if hasattr(drone, "obj_name"):
            mocap_args = f"--obj-name {drone['obj_name']} --vicon-mode rigidbody --vicon-full-pose "
        else:
            p = drone['init_pos']
            mocap_args = f"--init-pos {p[0]} {p[1]} {p[2]} --vicon-mode pointcloud "
        cmd = [
            f"cd {self.common_cfg['work_dir']} && ",
            f"source {self.common_cfg['venv_path']}/bin/activate && ",
            "git pull && ",
            f"nohup python3 {DRONE_SCRIPT} ",
            f"--illumination --orchestrated --tag {self.tag} ",
            "--ground-test " if self.args.ground else f"--vicon {mocap_args} ",
            f"--drone-id {drone['id']} ",
            f"--led --led-count {led_count} " if led_count > 0 else " ",
            f"--servo --servo-type {drone['type']} --servo-count {servo_count} " if servo_count > 0 else " ",
            f"--takeoff-altitude {alt} ",
            "--smooth-controller-rate 50 ",
            "--log ",
            f"> drone_{drone['id']}.log 2>&1 < /dev/null &",
        ]
        return " ".join(cmd)

    def _get_drone_cmd_morphing(self, drone):
        alt = self.mission['drones'][drone['id']]['target'][2]
        servo_count = drone.get('servo_count', 2)
        led_count = drone.get('led_count', 50)
        if hasattr(drone, "obj_name"):
            mocap_args = f"--obj-name {drone['obj_name']} --vicon-mode rigidbody --vicon-full-pose "
        else:
            p = drone['init_pos']
            mocap_args = f"--init-pos {p[0]} {p[1]} {p[2]} --vicon-mode pointcloud "
        cmd = [
            f"cd {self.common_cfg['work_dir']} && ",
            f"source {self.common_cfg['venv_path']}/bin/activate && ",
            "git pull && ",
            f"nohup python3 {DRONE_SCRIPT} ",
            f"--illumination --morphing --orchestrated --tag {self.tag} ",
            "--ground-test " if self.args.ground else f"--vicon {mocap_args} ",
            f"--drone-id {drone['id']} ",
            f"--led --led-count {led_count} " if led_count > 0 else " ",
            f"--servo --servo-type {drone['type']} --servo-count {servo_count} " if servo_count > 0 else " ",
            f"--takeoff-altitude {alt} ",
            "--smooth-controller-rate 50 ",
            "--log ",
            f"> drone_{drone['id']}.log 2>&1 < /dev/null &",
        ]
        return " ".join(cmd)

    def _get_camera_cmd(self):
        camera_params = [
            "--autofocus-mode", "manual",
            "--lens-position", "0.5",
            "--shutter", "35000",
            "--awb", "indoor",
        ]
        if self.args.dark:
            camera_params += ["--gain", "2.0"]
        else:
            camera_params += ["--gain", "0.8"]

        return (
            f"bash -c 'cd {self.common_cfg['work_dir']} && "
            f"source {self.common_cfg['venv_path']}/bin/activate && "
            f"nohup python3 {CAMERA_SCRIPT} {' '.join(camera_params)} > CAM.log 2>&1 < /dev/null &'"
        )

    def _boot_remote_node(self, device_cfg, cmd, node_type="Drone"):
        node_id = device_cfg.get('id', 'CAM')
        self.logger.info(f"Booting {node_type} {node_id} at {device_cfg['ip']}...")

        try:
            conn = Connection(host=device_cfg['ip'], user=device_cfg['user'], connect_timeout=5)
            # Push manifest from in-memory state (reflects any arg overrides) without touching local file
            manifest_buf = io.BytesIO(yaml.dump(self.manifest).encode('utf-8'))
            conn.put(manifest_buf, remote=f"{self.common_cfg['work_dir']}/swarm_manifest.yaml")
            # Run command (detach)
            conn.run(cmd, timeout=2, pty=False)
            return True
        except CommandTimedOut:
            self.logger.info(f"  > {node_type} {node_id} started successfully (timed out as expected).")
            return True
        except Exception as e:
            self.logger.error(f"  > Error booting {node_type} {node_id}: {e}")
            return False

    def _download_file(self, device_cfg, remote_path, local_path, description):
        self.logger.info(f"Downloading {description} from {device_cfg.get('id', 'CAM')}...")
        try:
            conn = Connection(host=device_cfg['ip'], user=device_cfg['user'], connect_timeout=10)
            conn.get(remote_path, local_path)
            self.logger.info(f"Saved: {local_path}")
            return True
        except FileNotFoundError:
            if not self.emergency:
                self.logger.info(f"File not found, it will be removed from list.")
                return True
            else:
                self.logger.info(f"File not found, retry next time.")
                return False
        except Exception as e:
            self.logger.error(f"Failed to download {description}: {e}")
            return False

    def shutdown_nodes(self):
        """Sends sudo shutdown command to all drones."""
        for drone in self.drones:
            self.logger.info(f"Shutting down Drone {drone['id']}...")
            try:
                conn = Connection(host=drone['ip'], user=drone['user'])
                conn.run("sudo shutdown now", pty=False, timeout=2, warn=True)
            except Exception:
                pass

    def kill_processes(self):
        """Kills python processes on all drones."""
        for drone in self.drones:
            self.logger.info(f"Killing processes on Drone {drone['id']}...")
            try:
                conn = Connection(host=drone['ip'], user=drone['user'])
                conn.run("pkill python3", pty=False, timeout=2, warn=True)
            except Exception:
                pass

    def reboot_flight_controllers(self, remote=False):
        """
        Reboots flight controllers.
        If remote=True, sends a command to the Radio Node to perform the reboot.
        If remote=False, attempts to reboot locally using cflib.
        """
        uris = [d['uri'] for d in self.drones]

        if remote:
            if not self.radio_node:
                print("[Orchestrator] Cannot perform remote reboot: No 'radio_node' in manifest.")
                return

            print(f"[Orchestrator] Sending REMOTE REBOOT request to Radio Node ({len(uris)} drones)...")
            
            # Connect existing sockets to radio node since it now binds to receive/send messages
            self.pub_socket.connect(f"tcp://{self.radio_node['ip']}:{self.ctrl_cfg['zmq_cmd_port']}")
            self.pull_socket.connect(f"tcp://{self.radio_node['ip']}:{self.ctrl_cfg['zmq_ack_port']}")

            msg = {
                "cmd": "REBOOT",
                "uris": uris
            }

            # --- Retry Logic ---
            retries = 3
            timeout_ms = 3000
            ack_received = False

            # Use a poller to listen for the specific ACK
            poller = zmq.Poller()
            poller.register(self.pull_socket, zmq.POLLIN)

            for attempt in range(1, retries + 1):
                self.logger.info(f"Attempt {attempt}/{retries}: Sending REBOOT command...")
                self.pub_socket.send_json(msg)

                # Wait for ACK
                events = dict(poller.poll(timeout_ms))
                if self.pull_socket in events:
                    try:
                        reply = self.pull_socket.recv_json(flags=zmq.NOBLOCK)
                        # Check if this is the ACK we want
                        if reply.get('id') == 'RADIO' and reply.get('status') == 'REBOOT_STARTED':
                            self.logger.info("Radio Node confirmed receipt. Reboot sequence initiated.")
                            ack_received = True
                            break
                        else:
                            self.logger.info(f"(Ignored message while waiting for ACK: {reply})")
                    except zmq.Again:
                        pass

                if not ack_received:
                    self.logger.info("Timeout waiting for Radio Node confirmation.")

            if not ack_received:
                self.logger.info("Radio Node did not respond after multiple attempts. Reboot failed.")
                return
        else:
            if not self.args.radio:
                for drone in self.drones:
                    reboot_crazyflie(drone['uri'])
        time.sleep(5)  # Wait for reboot

    def run(self):
        if self.args.off:
            self.shutdown_nodes()
            return
        if self.args.kill:
            self.kill_processes()
            return

        date_tag = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.tag = f"{self.mission['name']}_{date_tag}"
        self.logger.info(f"Mission Tag: {self.tag}")

        self.setup_network()
        self.running.set()

        try:
            # Process Pending Downloads from previous runs
            self._process_pending_downloads()

            self.pub_socket.send_json({"cmd": "_"})
            time.sleep(2)
            if not self.args.record:
                self.reboot_flight_controllers(remote=bool(self.radio_node))

                for drone in self.drones:
                    cmd = self._get_drone_cmd(drone)
                    if self._boot_remote_node(drone, cmd, "Drone"):
                        entry = {'drone': drone, 'tag': self.tag}
                        if self.args.interaction and self.common_cfg.get('reference_object'):
                            entry['vicon_log'] = True  # also download vicon_{tag}.json
                        self.pending_downloads.append(entry)

            if self.args.loadcell:
                from Interaction.loadcell_worker import loadcell_worker
                self.loadcell_thread = threading.Thread(
                    target=loadcell_worker,
                    args=(self.logger, self.running, self.tag)  #
                )
            if self.camera_cfg:
                if not self.args.ground and not self.args.skip_record:
                    self._boot_remote_node(self.camera_cfg, self._get_camera_cmd(), "Camera")
            else:
                self.logger.warning("No camera_node found. Skipping.")

            if self.loadcell_thread:
                self.loadcell_thread.start()

            # if self.args.interaction:
            #     ref_obj = self.common_cfg.get('reference_object')
            #     if ref_obj:
            #         log_path = os.path.join('./logs', f'vicon_{self.tag}.json')
            #         self.logger.info(f"Starting Vicon noise tracker for '{ref_obj}' → {log_path}")
            #         self.vicon_tracker_thread = threading.Thread(
            #             target=run_tracker,
            #             args=(ref_obj, log_path, self.running),
            #             daemon=True,
            #             name="vicon_noise_tracker",
            #         )
            #         self.vicon_tracker_thread.start()
            #     else:
            #         self.logger.warning("No 'reference_object' in swarm_manifest common — skipping Vicon noise tracker.")

            self._wait_for_ready()

            if not self.args.skip_confirm:
                input(">>> All Green. Press ENTER to Launch Swarm (Ctrl+C to Abort)...")
            self.logger.info("Broadcasting START...")
            self.pub_socket.send_json({"cmd": "START"})

            if self.manifest['mission']['require_handshake']:
                self.ready_ids = set()
                self._wait_for_ready()
                input(">>> All Green. Press ENTER to proceed (Ctrl+C to Abort)...")
                self.logger.info("Broadcasting START...")
                self.pub_socket.send_json({"cmd": "START"})

            self._monitor_flight()

        except KeyboardInterrupt:
            self.logger.info("Keyboard Interrupt detected in main loop.")
            self.emergency_stop()
        except Exception as e:
            self.logger.exception(f"Unexpected error: {e}")
            self.emergency_stop()
        finally:
            self.cleanup()

    def _process_pending_downloads(self):
        if not self.pending_downloads:
            return

        self.logger.info(f"Processing {len(self.pending_downloads)} pending downloads...")
        remaining = []
        for item in self.pending_downloads:
            drone = item['drone']
            tag = item['tag']
            work = self.common_cfg['work_dir']

            # Main controller log
            remote = f"{work}/logs/{tag}.json"
            local = f"./logs/{drone['id']}_{tag}.json"
            success = self._download_file(drone, remote, local, "Log")

            # Vicon noise log (only if tracker was launched)
            if item.get('vicon_log'):
                vr = f"{work}/logs/vicon_{tag}.json"
                vl = f"./logs/vicon_{tag}.json"
                v_ok = self._download_file(drone, vr, vl, "Vicon Noise Log")
                success = success and v_ok

            if not success:
                remaining.append(item)

        self.pending_downloads = remaining

    def _wait_for_ready(self):
        self.logger.info("Waiting for swarm readiness...")
        total_drones = len(self.drones)

        while len(self.ready_ids) < total_drones and self.running.is_set():
            try:
                msg = self.pull_socket.recv_json()
                sender_id = msg.get('id')
                status = msg.get('status')

                if sender_id == 'CAM' and status == 'READY':
                    self.logger.info("Camera Node Ready.")
                    if self.args.record:
                        return
                elif "lb" in sender_id:
                    if status == 'READY' and sender_id not in self.ready_ids:
                        self.ready_ids.add(sender_id)
                        self.logger.info(f"Drone {sender_id} Ready. ({len(self.ready_ids)}/{total_drones})")
            except zmq.Again:
                continue  # Timeout, loop back to check self.running

    def _monitor_flight(self):
        launched_drones = len(self.ready_ids)

        if self.args.record:
            while self.running.is_set():
                time.sleep(1)
        while len(self.landed_drones) < launched_drones and self.running.is_set():
            try:
                msg = self.pull_socket.recv_json()
                if msg.get('status') == 'LANDED':
                    self._handle_landed_msg(msg)
            except zmq.Again:
                continue

    def _handle_landed_msg(self, msg):
        d_id = msg['id']
        if d_id not in self.landed_drones:
            self.landed_drones.add(d_id)
            batt = msg.get('battery', 'N/A')
            elapsed = msg.get('flight_duration', 'N/A')
            self.logger.info(f"Drone {d_id} LANDED. Batt: {batt} Time: {elapsed}s")

    def emergency_stop(self):
        """Broadcasts emergency command and waits for landing confirmations."""
        self.running.clear()  # Stop inner loops
        self.emergency = True
        self.logger.info("!!! TRIGGERING EMERGENCY LANDING !!!")

        if self.pub_socket:
            self.pub_socket.send_json({"cmd": "EMERGENCY"})

        time.sleep(1)
        self._monitor_flight()

    def cleanup(self):
        self.logger.info("Running cleanup sequence...")
        self.running.clear()

        # Stop Camera
        if self.camera_cfg and self.pub_socket and not self.args.ground and not self.args.skip_record:
            self.logger.info("Stopping Camera...")
            self.pub_socket.send_json({"cmd": "STOP_CAMERA"})
            time.sleep(2)

            # Download Video
            remote = f"{self.common_cfg['work_dir']}/mission_footage.mp4"
            local = f"./logs/{self.tag}.mp4"
            self._download_file(self.camera_cfg, remote, local, "Mission Video")

        # Retry Pending Downloads (Logs)
        self._process_pending_downloads()

        # Save unfinished state
        if self.pending_downloads:
            self.logger.warning(f"Saving {len(self.pending_downloads)} unfinished downloads to file.")
            with open(self.ctrl_cfg['downloads_file'], 'w') as f:
                json.dump(self.pending_downloads, f)

        # Close Sockets
        if self.http_server:
            self.http_server.shutdown()
            self.http_server.server_close()
        #
        # if self.zmq_context:
        #     self.zmq_context.term()

        self.logger.info("Orchestrator cleanup complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--illumination", action="store_true", help="illumination application")
    parser.add_argument("--interaction", action="store_true", help="interaction application")
    parser.add_argument("--intractable-illumination", action="store_true",
                        help="interaction application with illumination")
    parser.add_argument("--morphing", action="store_true", help="illumination application with morphing emulator")
    parser.add_argument("--off", action="store_true", help="shutdown the raspberry pis")
    parser.add_argument("--kill", action="store_true", help="stop the controller")
    parser.add_argument("--ground", action="store_true", help="ground test")
    parser.add_argument("--dark", action="store_true", help="recording in darkness")
    parser.add_argument("--record", action="store_true", help="run the camera only to record")
    parser.add_argument("--skip-record", action="store_true", help="run without the camera")
    parser.add_argument("--radio", action="store_true", help="run mission with CrazyRadio")
    parser.add_argument("--loadcell", action="store_true", help="run with loadcell")
    parser.add_argument("--skip-confirm", action="store_true", help="run without pressing enter")
    parser.add_argument("--http-port", type=int, default=None, help="override manifest http_port")
    parser.add_argument("--zmq-cmd-port", type=int, default=None, help="override manifest zmq_cmd_port")
    parser.add_argument("--zmq-ack-port", type=int, default=None, help="override manifest zmq_ack_port")
    args = parser.parse_args()

    orchestrator = SwarmOrchestrator(args)
    orchestrator.run()
