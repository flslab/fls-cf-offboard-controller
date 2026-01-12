import yaml
import zmq

try:
    from restart import reboot_crazyflie

    CFLIB_AVAILABLE = True
except ImportError:
    print("[RadioNode] cflib not found! Radio functions will fail.")
    CFLIB_AVAILABLE = False

MANIFEST_FILE = 'swarm_manifest_sample.yaml'


def load_manifest():
    with open(MANIFEST_FILE, 'r') as f:
        return yaml.safe_load(f)


class RadioGatewayNode:
    def __init__(self, manifest):
        self.manifest = manifest
        self.ctrl = manifest['controller']
        self.context = zmq.Context()

        # Subscribe to Commands (REBOOT, SHUTDOWN)
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://{self.ctrl['ip']}:{self.ctrl['zmq_cmd_port']}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # Push Status
        self.push_socket = self.context.socket(zmq.PUSH)
        self.push_socket.connect(f"tcp://{self.ctrl['ip']}:{self.ctrl['zmq_ack_port']}")

    def handle_reboot(self, uris):
        if not CFLIB_AVAILABLE:
            print("[RadioNode] Cannot reboot: cflib missing.")
            return

        print(f"[RadioNode] Received Reboot Request for {len(uris)} drones.")
        for uri in uris:
            print(f"[RadioNode] Rebooting {uri}...")
            try:
                reboot_crazyflie(uri)
            except Exception as e:
                print(f"[RadioNode] Failed to reboot {uri}: {e}")

        print("[RadioNode] Reboot sequence complete.")

    def run(self):
        print("[RadioNode] Gateway Online. Listening for radio commands...")
        # self.push_socket.send_json({"id": "RADIO", "status": "READY"})

        try:
            while True:
                msg = self.sub_socket.recv_json()
                cmd = msg.get('cmd')

                if cmd == 'REBOOT':
                    uris = msg.get('uris', [])
                    self.handle_reboot(uris)

                elif cmd == 'SHUTDOWN':
                    print("[RadioNode] Shutdown received.")
                    break  # Exit to allow system shutdown via SSH or orchestrator

        except KeyboardInterrupt:
            print("[RadioNode] Stopping.")
        except Exception as e:
            print(f"[RadioNode] Error: {e}")


if __name__ == "__main__":
    manifest = load_manifest()
    node = RadioGatewayNode(manifest)
    node.run()
