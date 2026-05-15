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
        self.position_size = 1 + 3 + 6 * 4  # 1 byte + 3 byte padding + 6 floats (4 bytes each)
        self.shm_name = "/pos_shared_mem"
        self.shm_fd = open(f"/dev/shm{self.shm_name}", "r+b")  # Open shared memory
        self.shm_map = mmap.mmap(self.shm_fd.fileno(), self.position_size, access=mmap.ACCESS_READ)

    def get_latest_pose(self):
        data = self.shm_map[:self.position_size]  # Read 8 bytes (bool + 7 floats = 1 byte + 28 bytes)
        valid = struct.unpack("<4?", data[:4])[0]  # Extract the validity flag (1 byte)
        if valid:
            right, down, forward, roll, pitch, yaw = struct.unpack("<6f", data[4:28])
            return right, down, forward, roll, pitch, yaw
        return None

    def localize(self, relative_position):
        latest_pose = self.get_latest_pose()
        if not latest_pose:
            return

        right, down, forward, _, _, _ = latest_pose
        act = np.array([-right, -forward])
        gt = np.array([relative_position[0], relative_position[1]])
        v = act - gt
        p = 1.0
        v *= p
        z = relative_position[2]
        # self.controller.ll_commander.send_hover_setpoint(v[0], v[1], 0, z)
        logger.info(f"gt: {gt}")
        logger.info(f"act: {act}")
        logger.info(f"hover command: {v[0]}, {v[1]}, {z}")
        
        
    def run(self):
        last_valid = time.time()
        while self.running:
            data = self.shm_map[:self.position_size]  # Read 8 bytes (bool + 7 floats = 1 byte + 28 bytes)
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
