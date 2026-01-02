import yaml
import zmq
import subprocess
import signal
import os
from log import LoggerFactory


MANIFEST_FILE = 'swarm_manifest.yaml'
OUTPUT_FILENAME = 'mission_footage.mp4'


def load_manifest():
    with open(MANIFEST_FILE, 'r') as f:
        return yaml.safe_load(f)


class CameraNode:
    def __init__(self, manifest):
        self.manifest = manifest
        self.ctrl = manifest['controller']
        self.context = zmq.Context()
        self.recording_process = None
        self.logger = LoggerFactory("Camera").get_logger()

        # Subscribe to Commands (START, STOP_CAMERA, EMERGENCY)
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://{self.ctrl['ip']}:{self.ctrl['zmq_cmd_port']}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # Push Status (READY)
        self.push_socket = self.context.socket(zmq.PUSH)
        self.push_socket.connect(f"tcp://{self.ctrl['ip']}:{self.ctrl['zmq_ack_port']}")

    def start_recording(self):
        if self.recording_process is None:
            self.logger.info("Starting Recording...")
            # Using libcamera-vid.
            # -t 0: Record indefinitely until signal.
            # --inline: Improves compatibility for streaming/concatenation
            cmd = [
                "libcamera-vid",
                "-t", "0",
                "-o", OUTPUT_FILENAME,
                "--width", "1920",
                "--height", "1080",
                "--nopreview"
            ]
            try:
                # Start process in new process group for clean termination
                self.recording_process = subprocess.Popen(
                    cmd,
                    preexec_fn=os.setsid
                )
            except FileNotFoundError:
                self.logger.error("Error: libcamera-vid not found. Is this a RPi with Camera enabled?")
        else:
            self.logger.info("Already recording.")

    def stop_recording(self):
        if self.recording_process:
            self.logger.info("Stopping Recording and Saving...")
            # Send SIGINT (Ctrl+C) to the process group to close file gracefully
            os.killpg(os.getpgid(self.recording_process.pid), signal.SIGINT)
            try:
                self.recording_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.recording_process.kill()

            self.recording_process = None
            self.logger.info(f"[Camera] Saved to {OUTPUT_FILENAME}")
        else:
            self.logger.error("[Camera] No active recording to stop.")

    def run(self):
        self.logger.info("Node Online. Waiting for commands.")

        # Notify Laptop we are ready
        self.push_socket.send_json({"id": "CAM", "status": "READY"})

        try:
            while True:
                msg = self.sub_socket.recv_json()
                cmd = msg.get('cmd')

                if cmd == 'START':
                    self.start_recording()

                elif cmd == 'STOP_CAMERA':
                    self.stop_recording()
                    break  # Exit loop to allow script to finish (and download to happen)

                elif cmd == 'EMERGENCY':
                    self.logger.warning("Emergency received. Stopping recording.")
                    self.stop_recording()
                    break

                elif cmd == 'SHUTDOWN':
                    self.stop_recording()
                    break

        except KeyboardInterrupt:
            self.stop_recording()
        except Exception as e:
            self.logger.error(f"Error: {e}")
            self.stop_recording()


if __name__ == "__main__":
    manifest = load_manifest()
    node = CameraNode(manifest)
    node.run()
