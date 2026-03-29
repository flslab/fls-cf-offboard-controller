import json
import socket


class UDPPublisher:
    """Sends JSON datagrams to a fixed list of (ip, port) peers."""

    def __init__(self, peer_ips, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.targets = [(ip, port) for ip in peer_ips]

    def send_json(self, data):
        payload = json.dumps(data).encode()
        for addr in self.targets:
            try:
                self.sock.sendto(payload, addr)
            except OSError:
                pass

    def close(self):
        self.sock.close()


class UDPSubscriber:
    """Non-blocking UDP receiver. recv_latest() drains the buffer and returns the newest datagram."""

    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', port))
        self.sock.setblocking(False)

    def recv_latest(self):
        latest = None
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
                latest = json.loads(data.decode())
            except (BlockingIOError, json.JSONDecodeError, OSError):
                break
        return latest

    def close(self):
        self.sock.close()
