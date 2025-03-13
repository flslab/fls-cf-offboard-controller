import socket
import pickle
import select
import struct
import numpy as np


class Config:
    MULTICAST = False


class Constants:
    BROADCAST_ADDRESS = ("<broadcast>", 5000)
    # BROADCAST_ADDRESS = ("127.0.0.1", 5000)
    # BROADCAST_ADDRESS = ("192.168.1.220", 8080)
    BROADCAST_PORT = 8080
    WORKER_ADDRESS = ("", 8080)

    MULTICAST_GROUP_ADDRESS = ('224.3.29.25', 5000)
    MULTICAST_GROUP = '224.3.29.25'


class WorkerSocket:
    def __init__(self):
        self.sock = None
        if Config.MULTICAST:
            self.create_multicast_socket()
        else:
            self.create_udp_socket()

    def create_udp_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(Constants.WORKER_ADDRESS)
        self.sock = sock

    def create_multicast_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        sock.settimeout(0.05)
        # ttl = struct.pack('b', 1)
        # sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)
        sock.bind(Constants.WORKER_ADDRESS)
        # add the socket to the multicast group on all interfaces.
        group = socket.inet_aton(Constants.MULTICAST_GROUP)
        mreq = struct.pack('4sL', group, socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        self.sock = sock

    def close(self):
        self.sock.close()

    def receive(self):
        data, _ = self.sock.recvfrom(1024)
        try:
            msg = pickle.loads(data)
            return msg, len(data)
        except (pickle.UnpicklingError, EOFError):
            return None, 0

    def broadcast(self, msg, retry=2):
        # data = pickle.dumps(msg)
        data = msg.encode()
        address = Constants.MULTICAST_GROUP_ADDRESS if Config.MULTICAST else Constants.BROADCAST_ADDRESS
        try:
            self.sock.sendto(data, address)
        except OSError:
            if retry:
                self.broadcast(msg, retry - 1)
        return len(data)

    def is_ready(self):
        ready = select.select([self.sock], [], [], 1)
        return ready[0]

    def send_test_msgs(self):
        id = 1
        n = 5
        for i in range(n):
            self.broadcast((id, i))

        for i in range(2 * n):
            print(ws.receive())


if __name__ == '__main__':
    ws = WorkerSocket()
    ws.send_test_msgs()
