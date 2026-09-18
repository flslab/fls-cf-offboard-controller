"""Bridge between the Crazyflie and the high-rate localizer shared memory."""

from collections import deque
from dataclasses import dataclass
from enum import IntEnum
import math
import mmap
import os
import struct
from threading import Condition, Event, Lock
import time

from cflib.crazyflie.log import LogConfig

from yaw_error import YawCorrectionConfig, YawCorrectionGate


def reset_estimator_and_acknowledge(cf, generation, acknowledge):
    """Reset the EKF, then allow localizer positions to reach it."""
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    acknowledge(generation)


def wait_for_position_estimator(cf, timeout, threshold=0.001,
                                history_size=10, period_ms=500):
    """Wait for bounded Kalman variance convergence."""
    converged = Event()
    history_lock = Lock()
    variables = ('kalman.varPX', 'kalman.varPY', 'kalman.varPZ')
    histories = {
        variable: deque(maxlen=history_size) for variable in variables
    }
    sample_count = 0

    def on_data(_timestamp, data, _log_config):
        nonlocal sample_count
        with history_lock:
            sample_count += 1
            for variable in variables:
                histories[variable].append(float(data[variable]))
            if all(
                len(history) == history_size
                and max(history) - min(history) < threshold
                for history in histories.values()
            ):
                converged.set()

    variance_log = LogConfig(
        name='LocalizerEstimatorVariance', period_in_ms=period_ms
    )
    for variable in variables:
        variance_log.add_variable(variable, 'float')
    cf.log.add_config(variance_log)
    variance_log.data_received_cb.add_callback(on_data)
    timed_out = False
    try:
        variance_log.start()
        timed_out = not converged.wait(timeout)
    finally:
        variance_log.stop()
        variance_log.delete()
        variance_log.data_received_cb.remove_callback(on_data)
    if timed_out:
        with history_lock:
            latest = {
                variable: history[-1] if history else None
                for variable, history in histories.items()
            }
            ranges = {
                variable: max(history) - min(history) if history else None
                for variable, history in histories.items()
            }
        raise TimeoutError(
            f'position estimator did not converge within {timeout:.1f}s; '
            f'samples={sample_count}, latest={latest}, ranges={ranges}'
        )


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
    yaw_error: float
    pnp_reprojection_rms: float
    pnp_image_span_px: float
    yaw_error_valid: bool


class Tracker:
    """Own the controller side of the version-3 localizer ABI."""

    MAGIC = 0x334C5346
    ABI_VERSION = 3
    LAYOUT_SIZE = 1280
    CONTROLLER_OFFSET = 64
    ATTITUDE_OFFSET = 128
    ATTITUDE_COUNT = 16
    ATTITUDE_SIZE = 64
    LOCALIZER_OFFSET = 1152
    CONTROLLER = struct.Struct("<IIiiB7xII32x")
    ATTITUDE = struct.Struct("<IIIB3xd4fII16x")
    LOCALIZER = struct.Struct("<IIQd3f4f4fIH4B2xii3fB3xII16x")

    def __init__(self, controller, shm_name="/fls_localizer_v3", timeout=5.0,
                 yaw_correction=None):
        self.controller = controller
        self._lock = Lock()
        self._callback_lock = Lock()
        self._changed = Condition(self._lock)
        self._closed = False
        self._ack_generation = 0
        self._landing_requested = False
        self._landing_tile = (0, 0)
        self._latest = None
        self._sent_pose_sequence = 0
        self._yaw_correction_gate = YawCorrectionGate(
            YawCorrectionConfig.from_mapping(yaw_correction)
        )
        self._mapping, self._file = self._open(shm_name, timeout)
        self._attitude_sequence, = struct.unpack_from(
            "<I", self._mapping, self.CONTROLLER_OFFSET + 4
        )

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
        received_at = time.monotonic()
        with self._callback_lock:
            if self._closed:
                return
            quaternion = tuple(float(data[f"stateEstimate.{name}"])
                               for name in ("qx", "qy", "qz", "qw"))
            norm = math.sqrt(sum(value * value for value in quaternion))
            if not math.isfinite(norm) or norm < 1e-6:
                return
            quaternion = tuple(value / norm for value in quaternion)
            self._write_controller(received_at, quaternion)
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
                yaw_correction = self._yaw_correction_gate.consider(
                    output=output,
                    acknowledged=acknowledged,
                    tracking=(output.state == LocalizerState.HYPERGRID_TRACKING),
                    now=received_at,
                )
                self._changed.notify_all()
            if send_position:
                self.controller._send_position_no_log({"tvec": output.position})
            if yaw_correction is not None:
                self.controller._send_yaw_error(yaw_correction)

    def _write_controller(self, timestamp, quaternion):
        with self._lock:
            generation = self._ack_generation
            landing_requested = self._landing_requested
            tile_i, tile_j = self._landing_tile

        sequence = (self._attitude_sequence + 1) & 0xFFFFFFFF
        if sequence == 0:
            sequence = 1
        index = (sequence - 1) % self.ATTITUDE_COUNT
        offset = self.ATTITUDE_OFFSET + index * self.ATTITUDE_SIZE
        current, = struct.unpack_from("<I", self._mapping, offset)
        even = 2 if current == 0 else ((current + 2) & 0xFFFFFFFE)
        if even == 0:
            even = 2
        sample = bytearray(self.ATTITUDE.pack(
            even - 1, sequence, generation, True, timestamp, *quaternion,
            even, 0,
        ))
        checksum = self._checksum(sample[4:40])
        struct.pack_into("<I", sample, 44, checksum)
        struct.pack_into("<I", self._mapping, offset, even - 1)
        self._mapping[offset + 4:offset + self.ATTITUDE_SIZE] = sample[4:]
        struct.pack_into("<I", self._mapping, offset, even)
        self._attitude_sequence = sequence

        current, = struct.unpack_from("<I", self._mapping, self.CONTROLLER_OFFSET)
        even = 2 if current == 0 else ((current + 2) & 0xFFFFFFFE)
        if even == 0:
            even = 2
        block = bytearray(self.CONTROLLER.pack(
            even - 1, sequence, tile_i, tile_j, landing_requested, even, 0,
        ))
        checksum = self._checksum(block[4:24])
        struct.pack_into("<I", block, 28, checksum)
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
                    or values[-1] != self._checksum(data[4:104])):
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
                    yaw_error=values[23],
                    pnp_reprojection_rms=values[24],
                    pnp_image_span_px=values[25],
                    yaw_error_valid=bool(values[26]),
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
        with self._callback_lock:
            if self._closed:
                return
            self._closed = True
        self._attitude_log.data_received_cb.remove_callback(self._on_attitude)
        self._attitude_log.stop()
        self._mapping.close()
        self._file.close()
