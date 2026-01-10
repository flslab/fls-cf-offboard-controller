import argparse
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
        self.drones = self.manifest['drones']
        self.camera_cfg = self.manifest.get('camera_node')
        self.common_cfg = self.manifest['common']
        self.mission = self._load_mission()

        # Runtime State
        self.tag = None
        self.running = False
        self.pending_downloads = self._load_previous_downloads()
        self.landed_drones = set()
        self.ready_ids = set()

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
        with open(self.ctrl_cfg['mission_file']) as f:
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
        alt = self.mission['drones'][drone['id']]['target'][2]
        return (
            f"cd {self.common_cfg['work_dir']} && "
            f"source {self.common_cfg['venv_path']}/bin/activate && "
            "git pull && "
            f"nohup python3 {DRONE_SCRIPT} "
            f"--vicon --orchestrated --vicon-full-pose "
            f"--drone-id {drone['id']} "
            f"--obj-name {drone['obj_name']} "
            f"--led --servo --servo-type {drone['type']} "
            f"--takeoff-altitude {alt} "
            f"--tag {self.tag} "
            f"> drone_{drone['id']}.log 2>&1 < /dev/null &"
        )

    def _get_camera_cmd(self):
        return (
            f"bash -c 'cd {self.common_cfg['work_dir']} && "
            f"source {self.common_cfg['venv_path']}/bin/activate && "
            f"nohup python3 {CAMERA_SCRIPT} > CAM.log 2>&1 < /dev/null &'"
        )

    def _boot_remote_node(self, device_cfg, cmd, node_type="Drone"):
        node_id = device_cfg.get('id', 'CAM')
        self.logger.info(f"Booting {node_type} {node_id} at {device_cfg['ip']}...")

        try:
            conn = Connection(host=device_cfg['ip'], user=device_cfg['user'], connect_timeout=5)
            # Push manifest
            conn.put(MANIFEST_FILE, remote=f"{self.common_cfg['work_dir']}/swarm_manifest.yaml")
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
            self.logger.info(f"File not found, it will be removed from list.")
            return True
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

    def reboot_flight_controllers(self):
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
        self.running = True

        try:
            # Process Pending Downloads from previous runs
            self._process_pending_downloads()

            self.reboot_flight_controllers()

            for drone in self.drones:
                cmd = self._get_drone_cmd(drone)
                if self._boot_remote_node(drone, cmd, "Drone"):
                    self.pending_downloads.append({'drone': drone, 'tag': self.tag})

            if self.camera_cfg:
                self._boot_remote_node(self.camera_cfg, self._get_camera_cmd(), "Camera")
            else:
                self.logger.warning("No camera_node found. Skipping.")

            self._wait_for_ready()

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
            remote = f"{self.common_cfg['work_dir']}/logs/vicon_{item['tag']}.json"
            local = f"./logs/{item['drone']['id']}_vicon_{item['tag']}.json"

            success = self._download_file(item['drone'], remote, local, "Vicon Log")
            if not success:
                remaining.append(item)

        self.pending_downloads = remaining

    def _wait_for_ready(self):
        self.logger.info("Waiting for swarm readiness...")
        total_drones = len(self.drones)

        while len(self.ready_ids) < total_drones and self.running:
            try:
                msg = self.pull_socket.recv_json()
                sender_id = msg.get('id')
                status = msg.get('status')

                if sender_id == 'CAM' and status == 'READY':
                    self.logger.info("Camera Node Ready.")
                elif status == 'READY' and sender_id not in self.ready_ids:
                    self.ready_ids.add(sender_id)
                    self.logger.info(f"Drone {sender_id} Ready. ({len(self.ready_ids)}/{total_drones})")
            except zmq.Again:
                continue  # Timeout, loop back to check self.running

    def _monitor_flight(self):
        launched_drones = len(self.ready_ids)

        while len(self.landed_drones) < launched_drones and self.running:
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
        self.running = False  # Stop inner loops
        self.logger.info("!!! TRIGGERING EMERGENCY LANDING !!!")

        if self.pub_socket:
            self.pub_socket.send_json({"cmd": "EMERGENCY"})

        time.sleep(1)
        self._monitor_flight()

    def cleanup(self):
        self.logger.info("Running cleanup sequence...")
        self.running = False

        # Stop Camera
        if self.camera_cfg and self.pub_socket:
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
    parser.add_argument("--off", action="store_true", help="shutdown the raspberry pis")
    parser.add_argument("--kill", action="store_true", help="stop the controller")
    args = parser.parse_args()

    orchestrator = SwarmOrchestrator(args)
    orchestrator.run()
