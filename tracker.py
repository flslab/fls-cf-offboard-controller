import mmap
import struct
import time
import threading
import logging


logger = logging.getLogger(__name__)


class Tracker(threading.Thread):
    def __init__(self, on_pose=None, on_failsafe=None):
        super().__init__()
        self.on_pose = on_pose
        self.on_failsafe = on_failsafe
        self.running = False
        self.failsafe = False
        self.position_size = 1 + 3 + 6 * 4  # 1 byte + 3 byte padding + 6 floats (4 bytes each)
        self.shm_name = "/pos_shared_mem"
        self.shm_fd = open(f"/dev/shm{self.shm_name}", "r+b")  # Open shared memory
        self.shm_map = mmap.mmap(self.shm_fd.fileno(), self.position_size, access=mmap.ACCESS_READ)

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

    def stop(self):
        self.running = False
        self.join()
