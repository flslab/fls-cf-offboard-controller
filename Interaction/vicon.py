import argparse
import json
import logging
import os.path
import threading
import time
import numpy as np
from datetime import datetime

from pyvicon_datastream import PyViconDatastream, StreamMode, Direction

from log import LoggerFactory

VICON_PC_IP = '192.168.1.39'
VICON_ADDRESS = f"{VICON_PC_IP}:801"


class ViconWrapper(threading.Thread):
    def __init__(self, log_level=logging.INFO, marker_type='mixed', stop_event=None, vicon_rotation=False, marker_order=None):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.stop_event = threading.Event() if stop_event is None else stop_event
        self.logger = LoggerFactory("Vicon", level=log_level).get_logger()
        self.all_frames = []
        self.latest_frame = None
        self.frame_version = 0
        label_types = {
            'mixed': 0,
            'unlabeled': 1,
            'labeled': 2
        }
        self.marker_type = label_types[marker_type]
        self.vicon_rotation = vicon_rotation
        self.vicon_write_q = None

        self.marker_order = marker_order if marker_order is not None else {}

    def stop(self):
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()

    def run(self):
        client = PyViconDatastream()
        self.logger.info("Client object created.")

        try:
            self.logger.info(f"Attempting to connect to Vicon server at {VICON_ADDRESS}...")
            client.connect(VICON_ADDRESS)
            self.logger.info("Connection successful!")

            if self.marker_type == 2:
                client.enable_marker_data()
                client.enable_segment_data()
                self.logger.info("Marker and Segment data enabled.")
            elif self.marker_type == 1:
                client.enable_unlabeled_marker_data()
                self.logger.info("Unlabeled marker data enabled.")
            else:
                client.enable_marker_data()
                client.enable_segment_data()
                client.enable_unlabeled_marker_data()
                self.logger.info("All Marker, Unlabeled Marker, and Segment data enabled.")

            client.set_stream_mode(StreamMode.ServerPush)
            self.logger.info("Stream mode set to ServerPush.")

            # Set axis mapping for standard coordinate systems
            client.set_axis_mapping(Direction.Forward, Direction.Left, Direction.Up)

            while not self.stop_event.is_set():
                if client.get_frame():
                    frame_num = client.get_frame_number()
                    self.logger.debug(f"\n--- Frame {frame_num} ---")

                    labeled_object_count = None
                    unlabeled_object_count = None
                    if self.marker_type == 2:
                        labeled_object_count = client.get_subject_count()
                        self.logger.debug(f"\tSubject count: {labeled_object_count}")
                    elif self.marker_type == 1:
                        unlabeled_object_count = client.get_unlabeled_marker_count()
                        self.logger.debug(f"\tUnlabeled marker count: {unlabeled_object_count}")
                    else:
                        labeled_object_count = client.get_subject_count()
                        self.logger.debug(f"\tSubject count: {labeled_object_count}")
                        for i in range(labeled_object_count):
                            self.logger.debug(f"    Subject [{i}]: {client.get_subject_name(i)}")
                        unlabeled_object_count = client.get_unlabeled_marker_count()
                        self.logger.debug(f"\tUnlabeled marker count: {unlabeled_object_count}")

                    if labeled_object_count is not None and unlabeled_object_count is not None:
                        object_count = labeled_object_count + unlabeled_object_count
                    elif labeled_object_count is not None:
                        object_count = labeled_object_count
                    else:
                        object_count = unlabeled_object_count

                    if object_count is None:
                        continue

                    translation = [None for _ in range(object_count)]
                    rotation = [None for _ in range(object_count)]
                    if object_count is not None and object_count >= 1:
                        if labeled_object_count is not None:
                            for i in range(labeled_object_count):
                                subject_name = client.get_subject_name(i)
                                if subject_name:
                                    self.logger.debug(f"\tSubject: {subject_name}")
                                    root_segment = client.get_subject_root_segment_name(subject_name)
                                    index = self.marker_order.get('subject_name', i)
                                    tvec = client.get_segment_global_translation(subject_name, root_segment)

                                    translation[index] = client.get_segment_global_translation(subject_name, root_segment)

                                    rotation[index] = (
                                        client.get_segment_global_rotation_euler_xyz(subject_name, root_segment)) \
                                        if self.vicon_rotation else []


                        if unlabeled_object_count is not None:
                            for i in range(unlabeled_object_count):
                                translation[object_count - unlabeled_object_count + i] = (
                                    list(client.get_unlabeled_marker_global_translation(i)))
                                rotation[object_count - unlabeled_object_count + i] = []

                        if translation is not None:
                            now = time.time()
                            full_report = ""

                            for x, y, z in translation:
                                # Appending a new f-string to the existing string
                                full_report += f"   * marker: [{x/1000:.4f}, {y/1000:.4f}, {z/1000:.4f}]\n"

                            self.logger.info(f"\tPosition (m): \n{full_report}")
                            # pos_x, pos_y, pos_z = translation[:, 0], translation[:, 1], translation[:, 2]
                            #
                            # with self.condition:
                            #     # self.latest_frame = [pos_x, pos_y, pos_z, now]
                            #     self.latest_frame = [np.array(translation)/1000, rotation, now]
                            #     self.logger.warning(f"\tPosition (mm): Occluded or no data")
                            #     self.frame_version += 1
                            #     self.condition.notify_all()

                        else:
                            self.logger.warning(f"\tPosition (mm): Occluded or no data")

                # else:
                #     time.sleep(0.001)

        except KeyboardInterrupt:
            self.logger.error("\nScript interrupted by user.")
        except Exception as e:
            self.logger.error(f"An unexpected error occurred: {e}", exc_info=True)

        finally:
            # now = datetime.now()
            # formatted = now.strftime("%H_%M_%S_%m_%d_%Y")
            # file_path = os.path.join("logs", f"vicon_{formatted}.json")
            # with open(file_path, "w") as f:
            #     json.dump({"frames": self.all_frames}, f)
            # self.logger.info(f"Vicon log saved in {file_path}")

            if client.is_connected():
                client.disconnect()
                self.logger.info("Disconnected from Vicon server.")

    def wait_for_new(self, last_version):
        with self.condition:
            self.condition.wait_for(
                lambda: self.frame_version != last_version or self.stop_event.is_set()
            )
            return self.latest_frame, self.frame_version



if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", default=20, type=int, help="time to run")
    args = ap.parse_args()
    LOG_DIR = './test_logs'
    # name = "ClientPull"

    name = "ServerPush"
    vw = ViconWrapper(marker_type='unlabeled', log_level=logging.DEBUG)
    vw.start()
    time.sleep(args.t)
    vw.stop()
