import argparse
import threading
import time
import logging
import motioncapture

logger = logging.getLogger(__name__)


class Mocap(threading.Thread):
    def __init__(self, mocap_system_type="vicon", host_name="vicon"):
        threading.Thread.__init__(self)

        self.running = True
        self.mocap_system_type = mocap_system_type
        self.host_name = host_name

        # Shared state
        self.objects_to_track = []

        # Lock is ONLY for writers (subscribe/unsubscribe), not the reader (run)
        self._write_lock = threading.Lock()

    def stop(self):
        self.running = False
        self.join()

    def run(self):
        mc = motioncapture.connect(self.mocap_system_type, {'hostname': self.host_name})
        i = 0
        while self.running:
            mc.waitForNextFrame()
            now = time.time()

            # Grab a reference to the current list
            # If unsubscribe happens during this line, we just keep using the "old" list
            current_targets = self.objects_to_track

            for name, obj in mc.rigidBodies.items():
                for obj_name, callback in current_targets:
                    if name == obj_name:
                        pos = obj.position
                        quat = obj.rotation
                        if callback:
                            callback({
                                "frame_id": i,
                                "tvec": [float(pos[0]), float(pos[1]), float(pos[2])],
                                "quat": [float(quat.x), float(quat.y), float(quat.z), float(quat.w)],
                                "time": now
                            })
            i += 1

    def subscribe_object(self, obj_name, callback):
        with self._write_lock:
            new_list = list(self.objects_to_track)
            new_list.append((obj_name, callback))
            self.objects_to_track = new_list

    def unsubscribe_object(self, obj_name):
        with self._write_lock:
            new_list = [obj for obj in self.objects_to_track if obj[0] != obj_name]
            self.objects_to_track = new_list


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", default=140, type=int, help="duration")
    ap.add_argument("--obj-name", type=str, help="object name in motion capture system")
    args = ap.parse_args()

    mw = Mocap()
    mw.subscribe_object(args.obj_name, lambda frame: print(frame))
    time.sleep(args.t)
    mw.stop()
