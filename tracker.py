"""Bridge between the Crazyflie and the high-rate localizer shared memory."""

from dataclasses import dataclass
from enum import IntEnum
import math
import mmap
import os
import struct
from threading import Condition, Lock
import time

from cflib.crazyflie.log import LogConfig


class LocalizerState(IntEnum):
    STARTING = 0
    MYGRID_DECODING = 1
    INITIAL_POSE_READY = 2
    TAKEOFF_TRACKING = 3
    HYPERGRID_ACQUIRE = 4
    HYPERGRID_TRACKING = 5
    LANDING_ACQUIRE = 6
    LANDING_TRACKING = 7
    LOST = 8
    FAULT = 9


class MyGridRequest(IntEnum):
    BLINK = 0
    STATIC = 1
    OFF = 2


@dataclass(frozen=True)
class LocalizerOutput:
    pose_sequence: int
    frame_id: int
    timestamp: float
    position: tuple
    quaternion: tuple
    initial_yaw: float
    reprojection_rms: float
    processing_ms: float
    acquisition_height: float
    initial_pose_generation: int
    feature_count: int
    state: LocalizerState
    pose_source: int
    mygrid_request: MyGridRequest
    pose_valid: bool
    tile: tuple


class Tracker:
    """Own the controller side of the version-1 localizer ABI."""

    MAGIC = 0x324C5346
    ABI_VERSION = 1
    LAYOUT_SIZE = 256
    CONTROLLER_OFFSET = 64
    LOCALIZER_OFFSET = 128
    CONTROLLER = struct.Struct("<IId4fiiBB6xII8x")
    LOCALIZER = struct.Struct("<IIQd3f4f4fIH4B2xiiII32x")

    def __init__(self, controller, shm_name="/fls_localizer_v2", timeout=5.0):
        self.controller = controller
        self._lock = Lock()
        self._changed = Condition(self._lock)
        self._ack_generation = 0
        self._landing_requested = False
        self._landing_tile = (0, 0)
        self._latest = None
        self._sent_pose_sequence = 0
        self._mapping, self._file = self._open(shm_name, timeout)

        self._attitude_log = LogConfig(name="LocalizerAttitude", period_in_ms=10)
        for name in ("qx", "qy", "qz", "qw"):
            self._attitude_log.add_variable(f"stateEstimate.{name}", "float")
        self.controller.cf.log.add_config(self._attitude_log)
        self._attitude_log.data_received_cb.add_callback(self._on_attitude)
        self._attitude_log.start()

    @staticmethod
    def _checksum(data):
        value = 2166136261
        for byte in data:
            value = ((value ^ byte) * 16777619) & 0xFFFFFFFF
        return value

    def _open(self, name, timeout):
        if not name.startswith("/") or "/" in name[1:]:
            raise ValueError("shared-memory name must have the form /name")
        path = f"/dev/shm/{name.lstrip('/')}"
        deadline = time.monotonic() + timeout
        last_error = None
        while True:
            try:
                file = open(path, "r+b", buffering=0)
                if os.fstat(file.fileno()).st_size != self.LAYOUT_SIZE:
                    file.close()
                    raise RuntimeError("shared memory has the wrong size")
                mapping = mmap.mmap(file.fileno(), self.LAYOUT_SIZE,
                                    access=mmap.ACCESS_WRITE)
                magic, version, size = struct.unpack_from("<III", mapping, 0)
                if (magic, version, size) != (
                        self.MAGIC, self.ABI_VERSION, self.LAYOUT_SIZE):
                    mapping.close()
                    file.close()
                    raise RuntimeError("shared memory ABI mismatch")
                return mapping, file
            except (FileNotFoundError, RuntimeError) as error:
                last_error = error
                if time.monotonic() >= deadline:
                    raise TimeoutError(
                        f"localizer did not initialize {path}: {last_error}"
                    ) from error
                time.sleep(0.05)

    def _on_attitude(self, _timestamp, data, _log_config):
        quaternion = tuple(float(data[f"stateEstimate.{name}"])
                           for name in ("qx", "qy", "qz", "qw"))
        norm = math.sqrt(sum(value * value for value in quaternion))
        if not math.isfinite(norm) or norm < 1e-6:
            return
        quaternion = tuple(value / norm for value in quaternion)
        self._write_controller(time.monotonic(), quaternion)
        output = self._read_localizer()
        if output is None or not self._is_fresh(output):
            return

        with self._changed:
            self._latest = output
            acknowledged = (
                output.initial_pose_generation != 0
                and output.initial_pose_generation == self._ack_generation
            )
            send_position = (
                acknowledged and output.pose_valid
                and output.pose_sequence != self._sent_pose_sequence
                and all(math.isfinite(value) for value in output.position)
            )
            if send_position:
                self._sent_pose_sequence = output.pose_sequence
            self._changed.notify_all()
        if send_position:
            self.controller._send_position_no_log({"tvec": output.position})

    def _write_controller(self, timestamp, quaternion):
        with self._lock:
            generation = self._ack_generation
            landing_requested = self._landing_requested
            tile_i, tile_j = self._landing_tile

        current, = struct.unpack_from("<I", self._mapping, self.CONTROLLER_OFFSET)
        even = 2 if current == 0 else ((current + 2) & 0xFFFFFFFE)
        if even == 0:
            even = 2
        block = bytearray(self.CONTROLLER.pack(
            even - 1, generation, timestamp, *quaternion, tile_i, tile_j,
            True, landing_requested, even, 0,
        ))
        checksum = self._checksum(block[4:48])
        struct.pack_into("<I", block, 52, checksum)
        struct.pack_into("<I", self._mapping, self.CONTROLLER_OFFSET, even - 1)
        self._mapping[self.CONTROLLER_OFFSET + 4:self.CONTROLLER_OFFSET + 64] = block[4:]
        struct.pack_into("<I", self._mapping, self.CONTROLLER_OFFSET, even)

    def _read_localizer(self):
        for _ in range(32):
            begin, = struct.unpack_from("<I", self._mapping, self.LOCALIZER_OFFSET)
            if begin == 0 or begin & 1:
                continue
            data = self._mapping[self.LOCALIZER_OFFSET:self.LOCALIZER_OFFSET + 128]
            after, = struct.unpack_from("<I", self._mapping, self.LOCALIZER_OFFSET)
            values = self.LOCALIZER.unpack(data)
            if (begin != after or begin != values[-2]
                    or values[-1] != self._checksum(data[4:88])):
                continue
            try:
                return LocalizerOutput(
                    pose_sequence=values[1], frame_id=values[2], timestamp=values[3],
                    position=values[4:7], quaternion=values[7:11],
                    initial_yaw=values[11], reprojection_rms=values[12],
                    processing_ms=values[13], acquisition_height=values[14],
                    initial_pose_generation=values[15], feature_count=values[16],
                    state=LocalizerState(values[17]), pose_source=values[18],
                    mygrid_request=MyGridRequest(values[19]), pose_valid=bool(values[20]),
                    tile=(values[21], values[22]),
                )
            except ValueError:
                return None
        return None

    @staticmethod
    def _is_fresh(output, maximum_age=0.5):
        return abs(time.monotonic() - output.timestamp) <= maximum_age

    def latest(self):
        with self._lock:
            return self._latest

    def wait_for(self, states, timeout=15.0):
        states = set(states)
        deadline = time.monotonic() + timeout
        with self._changed:
            while (self._latest is None or self._latest.state not in states
                   or not self._is_fresh(self._latest)):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    state = None if self._latest is None else self._latest.state.name
                    expected = sorted(item.name for item in states)
                    raise TimeoutError(
                        f"localizer did not reach {expected}; last state was {state}"
                    )
                self._changed.wait(min(remaining, 0.1))
            return self._latest

    def acknowledge_initial_pose(self, generation):
        with self._lock:
            self._ack_generation = generation

    def request_landing(self, tile):
        with self._lock:
            self._landing_tile = tuple(tile)
            self._landing_requested = True

    def close(self):
        self._attitude_log.stop()
        self._mapping.close()
        self._file.close()
