import json
import socket
import zmq


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


class TCPPeerPublisher:
    """ZMQ PUB socket — broadcasts JSON to all peer subscribers."""

    def __init__(self, port):
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.bind(f"tcp://*:{port}")

    def send_json(self, data):
        self._sock.send_json(data)

    def close(self):
        self._sock.close()
        self._ctx.term()


class TCPPeerSubscriber:
    """ZMQ SUB socket — drains all pending messages and returns the latest."""

    def __init__(self, peer_ips, port):
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt_string(zmq.SUBSCRIBE, "")
        for ip in peer_ips:
            self._sock.connect(f"tcp://{ip}:{port}")

    def recv_latest(self):
        latest = None
        while True:
            try:
                latest = self._sock.recv_json(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
        return latest

    def recv_blocking(self):
        """Block until one message arrives and return it."""
        return self._sock.recv_json()

    def recv_one_timeout(self, timeout_s: float):
        """Block up to timeout_s for one message. Returns None on timeout."""
        if self._sock.poll(timeout=int(timeout_s * 1000)):
            return self._sock.recv_json()
        return None

    def close(self):
        self._sock.close()
        self._ctx.term()
