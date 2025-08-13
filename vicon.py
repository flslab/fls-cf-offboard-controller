import argparse
import json
import logging
import os.path
import threading
import time
from datetime import datetime

from pyvicon_datastream import PyViconDatastream, StreamMode, Direction

from log import LoggerFactory

VICON_PC_IP = '192.168.1.39'
VICON_ADDRESS = f"{VICON_PC_IP}:801"


class ViconWrapper(threading.Thread):
    def __init__(self, callback=None, log_level=logging.INFO, labeled_object=False):
        super().__init__()
        self.running = False
        self.logger = LoggerFactory("Vicon", level=log_level).get_logger()
        self.callback = callback
        self.position_log = []
        self.labeled_object = labeled_object

    def stop(self):
        self.running = False
        self.join()

    def run(self):
        self.running = True

        client = PyViconDatastream()
        self.logger.info("Client object created.")

        try:
            self.logger.info(f"Attempting to connect to Vicon server at {VICON_ADDRESS}...")
            client.connect(VICON_ADDRESS)
            self.logger.info("Connection successful!")

            if self.labeled_object:
                client.enable_marker_data()
                client.enable_segment_data()
                self.logger.info("Marker and Segment data enabled.")
            else:
                client.enable_unlabeled_marker_data()
                self.logger.info("Unlabeled marker data enabled.")

            client.set_stream_mode(StreamMode.ClientPull)
            self.logger.info("Stream mode set to ClientPull.")

            # Set axis mapping for standard coordinate systems
            client.set_axis_mapping(Direction.Forward, Direction.Left, Direction.Up)

            last_time = time.time()
            while self.running:
                if client.get_frame():
                    frame_num = client.get_frame_number()
                    self.logger.debug(f"\n--- Frame {frame_num} ---")

                    if self.labeled_object:
                        object_count = client.get_subject_count()
                        self.logger.debug(f"\tSubject count: {object_count}")
                    else:
                        object_count = client.get_unlabeled_marker_count()
                        self.logger.debug(f"\tUnlabeled marker count: {object_count}")

                    if object_count is not None and object_count >= 1:
                        translation = None

                        if self.labeled_object:
                            subject_name = client.get_subject_name(0)
                            if subject_name:
                                self.logger.debug(f"\tSubject: {subject_name}")
                                root_segment = client.get_subject_root_segment_name(subject_name)
                                translation = client.get_segment_global_translation(subject_name, root_segment)
                        else:
                            translation = client.get_unlabeled_marker_global_translation(0)

                        if translation is not None:
                            now = time.time()
                            pos_x, pos_y, pos_z = translation
                            self.position_log.append({
                                "frame_id": frame_num,
                                "tvec": [pos_x, pos_y, pos_z],
                                "time": now * 1000
                            })
                            if callable(self.callback):
                                self.callback(pos_x, pos_y, pos_z, timestamp=now)

                            time_interval = now - last_time
                            last_time = now
                            self.logger.debug(f"Position (mm): X={pos_x:.2f}, Y={pos_y:.2f}, Z={pos_z:.2f}")
                            self.logger.debug(f"Time Interval (ms): {time_interval * 1000}")

                            # slow down loop
                            # time.sleep(0.05)
                        else:
                            self.logger.warning(f"\tPosition (mm): Occluded or no data")

                else:
                    time.sleep(0.01)

        except KeyboardInterrupt:
            self.logger.error("\nScript interrupted by user.")
        except Exception as e:
            self.logger.error(f"An unexpected error occurred: {e}")

        finally:
            now = datetime.now()
            formatted = now.strftime("%H_%M_%S_%m_%d_%Y")
            file_path = os.path.join("logs", f"vicon_{formatted}.json")
            with open(file_path, "w") as f:
                json.dump({"frames": self.position_log}, f)
            self.logger.info(f"Vicon log saved in {file_path}")

            if client.is_connected():
                client.disconnect()
                self.logger.info("Disconnected from Vicon server.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", default=10, type=int, help="time to run")
    args = ap.parse_args()

    vw = ViconWrapper(labeled_object=False, log_level=logging.DEBUG)
    vw.start()
    time.sleep(args.t)
    vw.stop()
