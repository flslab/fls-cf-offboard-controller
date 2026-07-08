import mmap
import struct
import time
import logging
import numpy as np


logger = logging.getLogger(__name__)


class Tracker():
    def __init__(self, controller, on_pose=None, on_failsafe=None):
        self.controller = controller
        self.on_pose = on_pose
        self.on_failsafe = on_failsafe
        self.running = False
        self.failsafe = False
        self.position_size = 40  # sizeof(Position) in C++ (40 bytes due to double alignment)
        self.shm_name = "/pos_shared_mem"
        self.shm_fd = open(f"/dev/shm{self.shm_name}", "r+b")  # Open shared memory
        self.shm_map = mmap.mmap(self.shm_fd.fileno(), self.position_size, access=mmap.ACCESS_READ)

    def get_latest_pose(self):
        data = self.shm_map[:self.position_size]  # Read 40 bytes (bool + padding + 6 floats + padding + double)
        valid = struct.unpack("<4?", data[:4])[0]  # Extract the validity flag (1 byte)
        if valid:
            x, y, z, roll, pitch, yaw = struct.unpack("<6f", data[4:28])
            timestamp = struct.unpack("<d", data[32:40])[0]
            return x, y, z, roll, pitch, yaw, timestamp
        return None
        
    def run(self):
        last_valid = time.time()
        while self.running:
            data = self.shm_map[:self.position_size]  # Read 40 bytes (bool + padding + 6 floats + padding + double)
            # print("raw data:", data)
            valid = struct.unpack("<4?", data[:4])[0]  # Extract the validity flag (1 byte)

            if time.time() - last_valid > 3 and self.failsafe is False:
                self.failsafe = True
                if self.on_failsafe:
                    self.on_failsafe()
                logger.warning("Failsafe triggered due to lack of position estimation")
            if valid:
                last_valid = time.time()
                left, forward, up, roll, pitch, yaw = struct.unpack("<6f", data[4:28])
                if self.on_pose:
                    self.on_pose(forward, left, up)
            else:
                pass

            time.sleep(1 / 120)
