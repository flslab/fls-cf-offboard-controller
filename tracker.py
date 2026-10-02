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


class CrazyflieLogClockMapper:
    """Map wrapping Crazyflie log ticks onto host ``CLOCK_MONOTONIC``.

    A Crazyflie log packet contains the low 24 bits of the millisecond clock
    since the flight controller booted.  A rolling affine fit estimates the FC
    clock rate from callback receipts, while an initial lower-envelope phase
    estimate keeps published samples causal.  Subsequent timestamps advance by
    FC elapsed time at the fitted rate, so transport jitter does not distort the
    intervals used for attitude interpolation.  If causality requires a
    material phase correction, ``history_revision`` changes so the caller can
    prevent prediction across that clock boundary.
    """

    TIMESTAMP_MODULUS_MS = 1 << 24
    DEFAULT_WARMUP_SAMPLES = 16
    DEFAULT_WARMUP_DURATION_S = 0.5
    DEFAULT_RATE_WINDOW_S = 1.0
    DEFAULT_MAXIMUM_CONTINUITY_ERROR_S = 0.5
    MAXIMUM_CLOCK_RATE_ERROR = 0.01
    HISTORY_RESET_ABSOLUTE_ERROR_S = 10e-6
    HISTORY_RESET_RELATIVE_ERROR = 0.01

    def __init__(self, warmup_samples=DEFAULT_WARMUP_SAMPLES,
                 warmup_duration_s=DEFAULT_WARMUP_DURATION_S,
                 rate_window_s=DEFAULT_RATE_WINDOW_S,
                 maximum_continuity_error_s=
                 DEFAULT_MAXIMUM_CONTINUITY_ERROR_S):
        if (isinstance(warmup_samples, bool)
                or not isinstance(warmup_samples, int)
                or warmup_samples < 1):
            raise ValueError("warmup_samples must be a positive integer")
        warmup_duration_s = float(warmup_duration_s)
        if not math.isfinite(warmup_duration_s) or warmup_duration_s < 0.0:
            raise ValueError(
                "warmup_duration_s must be finite and non-negative"
            )
        rate_window_s = float(rate_window_s)
        if (not math.isfinite(rate_window_s)
                or rate_window_s <= 0.0
                or rate_window_s < warmup_duration_s):
            raise ValueError(
                "rate_window_s must be finite, positive, and no shorter "
                "than warmup_duration_s"
            )
        maximum_continuity_error_s = float(maximum_continuity_error_s)
        if (not math.isfinite(maximum_continuity_error_s)
                or maximum_continuity_error_s <= 0.0):
            raise ValueError(
                "maximum_continuity_error_s must be finite and positive"
            )
        self._warmup_samples = warmup_samples
        self._warmup_duration_s = warmup_duration_s
        self._rate_window_s = rate_window_s
        self._maximum_continuity_error_s = maximum_continuity_error_s
        self._history_revision = 0
        self._last_mapped_time_s = None
        self._clear_epoch()

    def _clear_epoch(self):
        self._last_raw_timestamp_ms = None
        self._unwrapped_timestamp_ms = None
        self._last_receive_time_s = None
        self._epoch_unwrapped_timestamp_ms = None
        self._epoch_receive_time_s = None
        self._sample_count = 0
        self._last_published_device_time_s = None
        self._clock_rate = 1.0
        self._rate_samples = deque()
        self._sum_device = 0.0
        self._sum_receive = 0.0
        self._sum_device_squared = 0.0
        self._sum_device_receive = 0.0

    def _start_epoch(self, raw_timestamp_ms, receive_time_s):
        restarting = self._last_raw_timestamp_ms is not None
        self._clear_epoch()
        self._last_raw_timestamp_ms = raw_timestamp_ms
        self._unwrapped_timestamp_ms = raw_timestamp_ms
        self._last_receive_time_s = receive_time_s
        self._epoch_unwrapped_timestamp_ms = raw_timestamp_ms
        self._epoch_receive_time_s = receive_time_s
        self._sample_count = 1
        self._append_rate_sample(0.0, 0.0)
        if restarting:
            self._history_revision += 1

    @property
    def history_revision(self):
        """Generation of the timestamp history used for interpolation."""
        return self._history_revision

    def _append_rate_sample(self, device_time_s, receive_time_s):
        sample = (device_time_s, receive_time_s)
        self._rate_samples.append(sample)
        self._sum_device += device_time_s
        self._sum_receive += receive_time_s
        self._sum_device_squared += device_time_s * device_time_s
        self._sum_device_receive += device_time_s * receive_time_s

        cutoff = receive_time_s - self._rate_window_s
        while (len(self._rate_samples) > 2
               and self._rate_samples[0][1] < cutoff):
            old_device, old_receive = self._rate_samples.popleft()
            self._sum_device -= old_device
            self._sum_receive -= old_receive
            self._sum_device_squared -= old_device * old_device
            self._sum_device_receive -= old_device * old_receive

    def _update_clock_rate(self):
        count = len(self._rate_samples)
        if count < 2:
            return
        denominator = (
            count * self._sum_device_squared
            - self._sum_device * self._sum_device
        )
        if denominator <= 1e-15:
            return
        rate = (
            count * self._sum_device_receive
            - self._sum_device * self._sum_receive
        ) / denominator
        if (math.isfinite(rate)
                and abs(rate - 1.0) <= self.MAXIMUM_CLOCK_RATE_ERROR):
            self._clock_rate = rate

    def _initial_mapped_time(self, device_time_s):
        phase_s = min(
            receive_time_s - self._clock_rate * sample_time_s
            for sample_time_s, receive_time_s in self._rate_samples
        )
        return (
            self._epoch_receive_time_s
            + self._clock_rate * device_time_s
            + phase_s
        )

    @classmethod
    def _validate_raw_timestamp(cls, raw_timestamp_ms):
        if isinstance(raw_timestamp_ms, bool):
            raise ValueError("Crazyflie log timestamp must be an integer")
        try:
            numeric = float(raw_timestamp_ms)
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError(
                "Crazyflie log timestamp must be an integer"
            ) from error
        if not math.isfinite(numeric) or not numeric.is_integer():
            raise ValueError("Crazyflie log timestamp must be an integer")
        raw_timestamp_ms = int(numeric)
        if not 0 <= raw_timestamp_ms < cls.TIMESTAMP_MODULUS_MS:
            raise ValueError("Crazyflie log timestamp is outside 24-bit range")
        return raw_timestamp_ms

    def map(self, raw_timestamp_ms, receive_time_s):
        """Return source time in host-monotonic seconds, or ``None`` while unsafe."""
        raw_timestamp_ms = self._validate_raw_timestamp(raw_timestamp_ms)
        receive_time_s = float(receive_time_s)
        if not math.isfinite(receive_time_s):
            raise ValueError("receive_time_s must be finite")

        if self._last_raw_timestamp_ms is None:
            self._start_epoch(raw_timestamp_ms, receive_time_s)
        else:
            device_delta_ms = (
                raw_timestamp_ms - self._last_raw_timestamp_ms
            ) % self.TIMESTAMP_MODULUS_MS
            if device_delta_ms == 0:
                return None
            receive_delta_s = receive_time_s - self._last_receive_time_s
            device_delta_s = device_delta_ms / 1000.0
            discontinuity = (
                receive_delta_s < 0.0
                or device_delta_ms >= self.TIMESTAMP_MODULUS_MS // 2
                or abs(device_delta_s - receive_delta_s)
                > self._maximum_continuity_error_s
            )
            if discontinuity:
                self._start_epoch(raw_timestamp_ms, receive_time_s)
                return None

            self._last_raw_timestamp_ms = raw_timestamp_ms
            self._unwrapped_timestamp_ms += device_delta_ms
            self._last_receive_time_s = receive_time_s
            self._sample_count += 1

            device_time_s = (
                self._unwrapped_timestamp_ms
                - self._epoch_unwrapped_timestamp_ms
            ) / 1000.0
            receive_elapsed_s = receive_time_s - self._epoch_receive_time_s
            self._append_rate_sample(device_time_s, receive_elapsed_s)

        device_time_s = (
            self._unwrapped_timestamp_ms
            - self._epoch_unwrapped_timestamp_ms
        ) / 1000.0
        receive_elapsed_s = receive_time_s - self._epoch_receive_time_s
        self._update_clock_rate()
        if (self._sample_count < self._warmup_samples
                or receive_elapsed_s < self._warmup_duration_s):
            return None

        if self._last_published_device_time_s is None:
            mapped_time_s = min(
                self._initial_mapped_time(device_time_s), receive_time_s
            )
        else:
            device_delta_s = (
                device_time_s - self._last_published_device_time_s
            )
            expected_interval_s = device_delta_s * self._clock_rate
            predicted_time_s = (
                self._last_mapped_time_s + expected_interval_s
            )
            mapped_time_s = min(predicted_time_s, receive_time_s)
            correction_s = predicted_time_s - mapped_time_s
            reset_threshold_s = max(
                self.HISTORY_RESET_ABSOLUTE_ERROR_S,
                expected_interval_s * self.HISTORY_RESET_RELATIVE_ERROR,
            )
            if correction_s > reset_threshold_s:
                self._history_revision += 1

        # A callback clock that does not advance cannot safely place another
        # sample. Keep the previous published device epoch so the next usable
        # packet covers the complete FC interval.
        if (self._last_mapped_time_s is not None
                and mapped_time_s <= self._last_mapped_time_s):
            return None
        self._last_mapped_time_s = mapped_time_s
        self._last_published_device_time_s = device_time_s
        return mapped_time_s


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

    ATTITUDE_LOG_GROUP = "QUAT"
    # IlluminationLogger applies the deployed-firmware x10 compensation, so
    # this produces the same LogConfig(period_in_ms=10) used by the old stream.
    ATTITUDE_LOG_PERIOD_MS = 1
    ATTITUDE_VARIABLES = tuple(
        f"stateEstimate.{name}" for name in ("qx", "qy", "qz", "qw")
    )

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
                 yaw_correction=None, log_manager=None):
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
        self._unsubscribe_attitude = None
        self._attitude_log = None
        self._attitude_clock = CrazyflieLogClockMapper()
        self._attitude_clock_revision = self._attitude_clock.history_revision
        if log_manager is None:
            log_manager = getattr(controller, "log_manager", None)
        try:
            self._start_attitude_stream(log_manager)
        except Exception:
            self._mapping.close()
            self._file.close()
            raise

    def _start_attitude_stream(self, log_manager):
        """Use the managed QUAT packet stream, with a no-logging fallback."""
        register = getattr(log_manager, "register_cf_log_callback", None)
        if callable(register):
            unsubscribe = register(
                self.ATTITUDE_LOG_GROUP, self._on_attitude
            )
            if not callable(unsubscribe):
                raise TypeError(
                    "log manager callback registration must return an "
                    "unsubscribe function"
                )
            self._unsubscribe_attitude = unsubscribe
            return

        # Preserve tracker-only and interaction runs whose logger does not
        # expose the shared QUAT group yet.  Normal illumination/hover runs use
        # the manager path above and therefore allocate no duplicate log block.
        attitude_log = LogConfig(name="LocalizerAttitude", period_in_ms=10)
        for name in self.ATTITUDE_VARIABLES:
            attitude_log.add_variable(name, "float")
        self.controller.cf.log.add_config(attitude_log)
        attitude_log.data_received_cb.add_callback(self._on_attitude)
        self._attitude_log = attitude_log
        try:
            attitude_log.start()
        except Exception:
            attitude_log.data_received_cb.remove_callback(self._on_attitude)
            attitude_log.stop()
            self._attitude_log = None
            raise

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

    def _on_attitude(self, timestamp, data, _log_config):
        received_at = time.monotonic()
        with self._callback_lock:
            if self._closed:
                return
            sample_time = None
            try:
                sample_time = self._attitude_clock.map(
                    timestamp, received_at
                )
            except (TypeError, ValueError, OverflowError):
                pass
            clock_revision = getattr(
                self._attitude_clock, "history_revision", 0
            )
            previous_revision = getattr(
                self, "_attitude_clock_revision", clock_revision
            )
            if clock_revision != previous_revision:
                self._clear_attitude_history()
                self._attitude_clock_revision = clock_revision
            if sample_time is not None:
                try:
                    quaternion = tuple(
                        float(data[name]) for name in self.ATTITUDE_VARIABLES
                    )
                    norm = math.sqrt(
                        sum(value * value for value in quaternion)
                    )
                except (KeyError, TypeError, ValueError, OverflowError):
                    norm = math.nan
                if math.isfinite(norm) and norm >= 1e-6:
                    quaternion = tuple(value / norm for value in quaternion)
                    self._write_controller(sample_time, quaternion)

            # Localizer output is independent of whether this particular FC
            # tick was publishable (for example during mapper warmup or on a
            # duplicate tick).  Keep acknowledgements and pose forwarding live.
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

    def _clear_attitude_history(self):
        """Invalidate interpolation samples from an older clock alignment."""
        for index in range(self.ATTITUDE_COUNT):
            offset = self.ATTITUDE_OFFSET + index * self.ATTITUDE_SIZE
            struct.pack_into("<I", self._mapping, offset, 0)

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
        try:
            unsubscribe_attitude = getattr(
                self, "_unsubscribe_attitude", None
            )
            if unsubscribe_attitude is not None:
                unsubscribe_attitude()
                self._unsubscribe_attitude = None
            attitude_log = getattr(self, "_attitude_log", None)
            if attitude_log is not None:
                attitude_log.data_received_cb.remove_callback(
                    self._on_attitude
                )
                attitude_log.stop()
        finally:
            self._mapping.close()
            self._file.close()
