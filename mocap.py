import argparse
import threading
import os
import json
import time
import logging
import motioncapture


logger = logging.getLogger(__name__)


class Mocap(threading.Thread):
    def __init__(self, object_name, tag="", on_pose=None, mocap_system_type="vicon", host_name="vicon"):
        threading.Thread.__init__(self)

        self.object_name = object_name
        self.on_pose = on_pose
        self.running = True
        self.all_frames = []
        self.mocap_system_type = mocap_system_type
        self.host_name = host_name
        self.log_filename = ''
        self.set_log_filename(tag)

    def set_log_filename(self, tag):
        if not tag:
            import datetime

            now = datetime.datetime.now()
            tag = now.strftime("%Y-%m-%d_%H-%M-%S")

        self.log_filename = os.path.join("logs", f"{self.mocap_system_type}_{tag}.json")

    def get_latest_pos(self):
        return self.all_frames[-1]

    def stop(self):
        self.running = False

        with open(self.log_filename, "w") as f:
            json.dump({"frames": self.all_frames}, f)
        logger.info(f"Mocap log saved in {self.log_filename}")
        self.join()

    def run(self):
        mc = motioncapture.connect(self.mocap_system_type, {'hostname': self.host_name})
        i = 0
        while self.running:
            mc.waitForNextFrame()
            now = time.time()
            for name, obj in mc.rigidBodies.items():
                if name == self.object_name:
                    pos = obj.position
                    quat = obj.rotation
                    if self.on_pose:
                        self.on_pose(pos[0], pos[1], pos[2], quat)
                    self.all_frames.append({
                        "frame_id": i,
                        "tvec": [float(pos[0]), float(pos[1]), float(pos[2])],
                        "quat": [float(quat.x), float(quat.y), float(quat.z), float(quat.w)],
                        "time": now
                    })
            i += 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", default=140, type=int, help="duration")
    ap.add_argument("--object-name", type=str, help="object name in motion capture system")
    args = ap.parse_args()

    mw = Mocap(object_name=args.object_name)
    time.sleep(args.t)
    mw.stop()
